#include "qemu/osdep.h"
#include "qemu/iov.h"
#include "qemu/module.h"
#include "qemu/queue.h"
#include "qemu/timer.h"
#include "qemu/madvise.h"
#include "qemu/error-report.h"
#include "hw/virtio/virtio.h"
#include "hw/virtio/virtio-bus.h"
#include "hw/qdev-properties.h"
#include "hw/boards.h"
#include "hw/virtio/virtio-llfree-balloon.h"
#include "llfree_platform.h"
#include "subprojects/llfree-c/llc/src/utils.h"
#include "sysemu/llfree-balloon.h"
#include "sysemu/kvm.h"
#include "exec/address-spaces.h"
#include "qapi/error.h"
#include "tests/unit/iothread.h"
#include "trace.h"
#include "standard-headers/linux/virtio_llfree_balloon.h"
#include "standard-headers/linux/virtio_ids.h"
#include "llfree.h"
#include "llfree_zone.h"
#include "llfree_states.h"
#include <stdbool.h>
#include <stdint.h>
#include <linux/kvm.h>

// For Testing
// #define CONFIG_SIMULATE_BROKEN_VFIO 1
// #define CONFIG_DEBUG 1

#define LL_PAGE_SHIFT 12
#define LL_PAGE_SIZE (1 << LL_PAGE_SHIFT)
#define LL_HUGE_ORDER 9
#define LL_HUGE_COUNT (1 << LL_HUGE_ORDER)
#define LL_HUGE_SHIFT (LL_PAGE_SHIFT + LL_HUGE_ORDER)
#define LL_HUGE_SIZE (1 << LL_HUGE_SHIFT)
#define LL_RETRY_INTERVAL (5 * NANOSECONDS_PER_SECOND)
#define LL_PAGECACHE_RETRIES 1
#define LL_NUM_STATIC_QUEUES 2

/*-----------------------------------------------------------------------------------------------
| Shared Structs and Enums
-------------------------------------------------------------------------------------------------*/
typedef enum ll_notification {
    PAGECACHE_DROPPED,
} ll_notification_t;

typedef struct ll_install_info {
    uint32_t node;
    uint32_t zone;
    uint64_t frame;
} ll_install_info_t;

/// Static Memory Zone Preference Definitions.
/// We exclude ZONE_DMA as it is already quite small.
static const ll_zone_type_t zone_priority[] = {
    LL_ZONE_NORMAL,
    LL_ZONE_MOVABLE,
    LL_ZONE_DMA32,
};
#define ZONE_PRIORITY_LEN (sizeof(zone_priority) / sizeof(*zone_priority))

/*-----------------------------------------------------------------------------------------------
| Helper Functions
-------------------------------------------------------------------------------------------------*/
static ram_addr_t get_current_ram_size(void)
{
    MachineState *machine = MACHINE(qdev_get_machine());
    if (machine->device_memory) {
        return machine->ram_size + machine->device_memory->dimm_size;
    } else {
        return machine->ram_size;
    }
}

static uint32_t get_num_vcpus(void)
{
    MachineState *machine = MACHINE(qdev_get_machine());
    return machine->smp.cpus;
}

typedef int (*aggregate_fn)(void *ctx, uint64_t frame, size_t len);
typedef struct aggerage {
    aggregate_fn fn;
    void *ctx;
    uint64_t frame;
    size_t len;
} aggregate_t;

static aggregate_t aggregate_start(aggregate_fn fn, void *ctx)
{
    return (aggregate_t){
        .fn = fn,
        .ctx = ctx,
        .frame = 0,
        .len = 0,
    };
}

static int aggregate_next(aggregate_t *self, uint64_t frame, size_t len)
{
    g_assert(len > 0);
    // first range
    if (self->len == 0) {
        self->frame = frame;
        self->len = len;
        return 0;
    }
    // extend right
    if (self->frame + self->len == frame) {
        self->len += len;
        return 0;
    }
    // extend left
    if (frame + len == self->frame) {
        self->frame = frame;
        self->len += len;
        return 0;
    }
    // not extendable
    int ret = (self->fn)(self->ctx, self->frame, self->len);
    self->frame = frame;
    self->len = len;
    return ret;
}

static int aggregate_finish(aggregate_t *self)
{
    if (self->len > 0)
        return (self->fn)(self->ctx, self->frame, self->len);
    return 0;
}

/*-----------------------------------------------------------------------------------------------
| RamDiscardManager Functions,  Part 1
| Bitmap Manipulation and explicit VFIO Notifications
| Implementation is largely taken directly from virtio-mem
-------------------------------------------------------------------------------------------------*/

/*
 * Adjust the memory section to cover the intersection with the given range.
 *
 * Returns false if the intersection is empty, otherwise returns true.
 */
static bool intersect_memory_section(MemoryRegionSection *s, uint64_t offset,
                                     uint64_t size)
{
    uint64_t start = MAX(s->offset_within_region, offset);
    uint64_t end =
        MIN(s->offset_within_region + int128_get64(s->size), offset + size);

    if (end <= start) {
        return false;
    }

    s->offset_within_address_space += start - s->offset_within_region;
    s->offset_within_region = start;
    s->size = int128_make64(end - start);
    return true;
}

static void notify_discard(VirtIOLLFreeBalloon *dev, uint64_t offset,
                           uint64_t size)
{
    RamDiscardListener *rdl;

#ifdef CONFIG_SIMULATE_BROKEN_VFIO
    return;
#endif

    if (QLIST_EMPTY(&dev->rdl_list)) {
        return;
    }

    bool found_intersection = false;
    QLIST_FOREACH(rdl, &dev->rdl_list, next)
    {
        MemoryRegionSection tmp = *rdl->section;

        if (!intersect_memory_section(&tmp, offset, size)) {
            continue;
        }
        found_intersection = true;
        rdl->notify_discard(rdl, &tmp);
    }

    // we always expect to find at least one match, as we are inflating
    // RAM memory
    g_assert(found_intersection == true);
}

static int notify_populate(VirtIOLLFreeBalloon *dev, uint64_t offset,
                           uint64_t size)
{
    RamDiscardListener *rdl, *rdl2;
    int ret = 0;

#ifdef CONFIG_SIMULATE_BROKEN_VFIO
    return 0;
#endif

    if (QLIST_EMPTY(&dev->rdl_list)) {
        return 0;
    }

    bool found_intersection = false;
    QLIST_FOREACH(rdl, &dev->rdl_list, next)
    {
        MemoryRegionSection tmp = *rdl->section;

        if (!intersect_memory_section(&tmp, offset, size)) {
            continue;
        }
        found_intersection = true;
        ret = rdl->notify_populate(rdl, &tmp);
        if (ret) {
            break;
        }
    }

    // we always expect to find at least one match, as we are inflating
    // RAM memory
    g_assert(found_intersection == true);

    if (ret) {
        /* Notify all already-notified listeners. */
        QLIST_FOREACH(rdl2, &dev->rdl_list, next)
        {
            MemoryRegionSection tmp = *rdl2->section;

            if (rdl2 == rdl) {
                break;
            }
            if (!intersect_memory_section(&tmp, offset, size)) {
                continue;
            }
            rdl2->notify_discard(rdl2, &tmp);
        }
    }
    return ret;
}

/*-----------------------------------------------------------------------------------------------
| LLFree-Balloon Inflation Operation Functions
-------------------------------------------------------------------------------------------------*/
static void unmap_range(VirtIOLLFreeBalloon *dev, hwaddr gpa, size_t size)
{
    g_assert(gpa > 0 && gpa > 0 && size % LL_HUGE_SIZE == 0);

    MemoryRegionSection section =
        memory_region_find(get_system_memory(), gpa, size);

    // we registered as RamDiscardManager for the RAM memory region
    // every section here should be part of this region and therefore we should
    // see that the RamDiscardManager field has been set by us
    g_assert(section.mr != NULL);

    if (!memory_region_is_ram(section.mr) || memory_region_is_rom(section.mr) ||
        memory_region_is_romd(section.mr)) {
        g_assert(!"unmap: invalid section");
    }

    void *addr =
        memory_region_get_ram_ptr(section.mr) + section.offset_within_region;
    ram_addr_t rb_offset;
    RAMBlock *rb = qemu_ram_block_from_host(addr, false, &rb_offset);
    int ret = ram_block_discard_range(rb, rb_offset, size);
    if (ret < 0)
        g_assert(!"unmap: discard failed");

    notify_discard(dev, section.offset_within_region, size);
    memory_region_unref(section.mr);
}

struct ag_dev_zone {
    VirtIOLLFreeBalloon *dev;
    ll_zone_t *zone;
};

static int reclaim_mapped_range_cb(void *ctx, uint64_t frame, size_t len)
{
    struct ag_dev_zone *data = ctx;
    ll_reclaim_states_t *states = ll_zone_states(data->zone);
    // Check if range is valid
    for (size_t i = 0; i < len; i += LL_HUGE_COUNT) {
        if (ll_reclaim_states_is_hard(states, frame + i)) {
            error_report("range already reclaimed %lx-%lx", frame, frame + len);
            return -1;
        }
        if (ll_reclaim_states_is_soft(states, frame + i)) {
            error_report("range already unmapped %lx-%lx", frame, frame + len);
            return -1;
        }
    }

    hwaddr addr = (ll_zone_start_frame(data->zone) + frame) * LL_PAGE_SIZE;
    unmap_range(data->dev, addr, len * LL_PAGE_SIZE);

    for (size_t i = 0; i < len; i += LL_HUGE_COUNT) {
        if (!ll_frame_states_hard_reclaim(states, frame + i, false)) {
            error_report("invalid frame state");
            return -1;
        }
    }
    return 0;
}

static int32_t hard_reclaim_node(VirtIOLLFreeBalloon *dev, size_t node,
                                 size_t target)
{
    g_assert(target > 0);

    size_t reclaimed = 0;

    for (size_t i = 0; i < ZONE_PRIORITY_LEN; i++) {
        ll_zone_type_t zone_type = zone_priority[i];
        ll_zone_t *zone = ll_nodes_get_zone(dev->nodes, node, zone_type);
        if (zone == NULL)
            continue;

        ll_zone_lock(zone);
        ll_reclaim_states_t *states = ll_zone_states(zone);

        struct ag_dev_zone data = {.dev = dev, .zone = zone};
        aggregate_t ag = aggregate_start(reclaim_mapped_range_cb, &data);
        for (; reclaimed < target; reclaimed++) {
            llfree_result_t res = ll_zone_reclaim(zone, true);
            if (llfree_is_ok(res)) {
                if (res.reclaimed) {
                    // already unmapped
                    if (!ll_frame_states_hard_reclaim(states, res.frame, true))
                        error_report("invalid frame state");
                } else {
                    // we have to unmap it
                    aggregate_next(&ag, res.frame, LL_HUGE_COUNT);
                }
            } else {
                if (res.error != LLFREE_ERR_MEMORY)
                    error_report("failed llfree reclaim");
                break;
            }
        }
        aggregate_finish(&ag);

        ll_zone_unlock(zone);

        if (reclaimed >= target) {
            break;
        }
    }

    info_report("reclaimed %zu", reclaimed);

    return reclaimed;
}

static int soft_reclaim_cb(void *ctx, uint64_t frame, size_t len)
{
    struct ag_dev_zone *data = ctx;
    ll_reclaim_states_t *states = ll_zone_states(data->zone);
    // Check if range is valid
    for (size_t i = 0; i < len; i += LL_HUGE_COUNT) {
        if (ll_reclaim_states_is_soft(states, frame + i)) {
            error_report("range already unmapped %lx-%lx", frame, frame + len);
            return -1;
        }
    }

    hwaddr addr = (ll_zone_start_frame(data->zone) + frame) * LL_PAGE_SIZE;
    unmap_range(data->dev, addr, len * LL_PAGE_SIZE);

    for (size_t i = 0; i < len; i += LL_HUGE_COUNT) {
        if (!ll_frame_states_soft_reclaim(states, frame + i)) {
            error_report("invalid frame state");
            return -1;
        }
    }
    return 0;
}

static int32_t ll_auto_soft_reclaim(VirtIOLLFreeBalloon *dev)
{
    size_t unmapped_huge = 0;

    for (size_t node = 0; node < ll_nodes_len(dev->nodes); node++) {
        for (size_t i = 0; i < ZONE_PRIORITY_LEN; i++) {
            ll_zone_type_t zid = zone_priority[i];
            ll_zone_t *zone = ll_nodes_get_zone(dev->nodes, node, zid);
            if (zone == NULL)
                continue;

            // allocate a chunk of frames, inflate them, and repeat until nomem
            while (true) {
                ll_zone_lock(zone);

                // aggregate syscalls
                struct ag_dev_zone data = {.dev = dev, .zone = zone};
                aggregate_t ag = aggregate_start(soft_reclaim_cb, &data);

                // find inflatable frames
                bool nomem = false;
                for (size_t i = 0; i < LLFREE_TREE_CHILDREN; i++) {
                    llfree_result_t res = ll_zone_reclaim(zone, false);
                    if (!llfree_is_ok(res)) {
                        if (res.error != LLFREE_ERR_MEMORY)
                            error_report("llfree unmap failed");
                        nomem = true;
                        break;
                    }
                    aggregate_next(&ag, res.frame, LL_HUGE_COUNT);
                    unmapped_huge += 1;
                }
                aggregate_finish(&ag);

                ll_zone_unlock(zone);

                if (nomem) {
                    break; // we were unable to allocate enough frames
                }

                // let others run
                sched_yield();
            }
        }
    }
    if (unmapped_huge > 0) {
        info_report("Auto unmap: %zu huge pages", unmapped_huge);
    }

    return unmapped_huge;
}

/*-----------------------------------------------------------------------------------------------
| LLFree-Balloon Deflation Operation Functions
-------------------------------------------------------------------------------------------------*/
static void map_range(VirtIOLLFreeBalloon *dev, hwaddr gpa, uint64_t size)
{
    // find QEMUs matching memory section
    MemoryRegionSection section =
        memory_region_find(get_system_memory(), gpa, size);

    // we registered as RamDiscardManager for the RAM memory region
    // every section here should be part of this region and therefore we should
    // see that the RamDiscardManager field has been set by us
    if (section.mr == NULL)
        g_assert(!"map: invalid section");

    if (!memory_region_is_ram(section.mr) || memory_region_is_rom(section.mr) ||
        memory_region_is_romd(section.mr))
        g_assert(!"map: invalid section");

    // Allocating host physical memory, in case of VFIO
    // additionally create IOMMU mapping
    void *addr =
        memory_region_get_ram_ptr(section.mr) + section.offset_within_region;
    void *host_addr = (void *)((uintptr_t)addr & ~(size - 1));

    if (virtio_has_feature(dev->host_features, LL_BALLOON_F_VFIO)) {
        notify_populate(dev, section.offset_within_region, size);
    }

    int ret = qemu_madvise(host_addr, size, MADV_POPULATE_WRITE);
    if (ret != 0)
        error_report("madvise failed with %d", ret);
    memory_region_unref(section.mr);
}

static int32_t return_node(VirtIOLLFreeBalloon *dev, size_t node, size_t target)
{
    g_assert(target > 0);

    size_t returned = 0;

    for (size_t i = ZONE_PRIORITY_LEN; i > 0; i--) {
        ll_zone_t *zone =
            ll_nodes_get_zone(dev->nodes, node, zone_priority[i - 1]);
        if (zone == NULL)
            continue;

        ll_reclaim_states_t *states = ll_zone_states(zone);
        g_assert(states != NULL);

        ll_zone_lock(zone);
        for (; returned < target; returned++) {
            llfree_result_t res = ll_frame_states_return_next(states);
            if (!llfree_is_ok(res))
                break;

            res = ll_zone_return(zone, res.frame);
            if (!llfree_is_ok(res)) {
                error_report("failed llfree return %u", res.error);
                break;
            }
        }
        ll_zone_unlock(zone);

        if (returned >= target) {
            break;
        }
    }

    info_report("returned %zu", returned);
    g_assert(returned == target);

    return returned;
}

/*-----------------------------------------------------------------------------------------------
| LLFree-Balloon Classic Memory Ballooning Functions + Guest Pagecache Shrinking
-------------------------------------------------------------------------------------------------*/

static bool ll_try_shrink_pagecache(VirtIOLLFreeBalloon *dev,
                                    uint32_t diff_huge_pages)
{
    VirtIODevice *vdev = VIRTIO_DEVICE(dev);

    uint32_t file_pages = 0;
    for (size_t node = 0; node < ll_nodes_len(dev->nodes); node++) {
        for (ll_zone_type_t zid = 0; zid < LL_ZONE_LEN; zid++) {
            ll_zone_t *zone = ll_nodes_get_zone(dev->nodes, node, zid);
            if (zone != NULL)
                file_pages += ll_zone_file_pages(zone);
        }
    }
    if (dev->shrink_pagecache == 0 &&
        dev->pagecache_retries < LL_PAGECACHE_RETRIES && file_pages > 0) {
        size_t target = MIN(diff_huge_pages * LL_HUGE_COUNT, file_pages);
        dev->shrink_pagecache = target;
        dev->pagecache_retries += 1;
        virtio_notify_config(vdev);
        return true;
    }

    return false;
}

static void ll_balloon_update_size(VirtIOLLFreeBalloon *dev,
                                   bool shrink_pagecache)
{
    // how much memory should be reclaimed, or given back if negative
    int64_t diff_huge_pages =
        qatomic_read(&dev->target_huge) - qatomic_read(&dev->actual_huge);

    if (diff_huge_pages == 0)
        return;

    info_report("update balloon size %ld->%ld: %ld",
                qatomic_read(&dev->actual_huge),
                qatomic_read(&dev->target_huge), diff_huge_pages);

    g_assert(dev->nodes != NULL);

    if (diff_huge_pages > 0) {
        // shrink VM
        for (size_t node = 0;
             node < ll_nodes_len(dev->nodes) && diff_huge_pages > 0; node++) {
            size_t reclaimed = hard_reclaim_node(dev, node, diff_huge_pages);
            qatomic_add(&dev->actual_huge, reclaimed);
            diff_huge_pages -= reclaimed;
        }

    } else {
        // grow VM
        for (size_t node = 0;
             node < ll_nodes_len(dev->nodes) && diff_huge_pages < 0; node++) {
            size_t returned = return_node(dev, node, -diff_huge_pages);
            qatomic_sub(&dev->actual_huge, returned);
            diff_huge_pages += returned;
        }
    }

    info_report("llfree_balloon_end %lu ns",
                qemu_clock_get_ns(QEMU_CLOCK_REALTIME));

    // decide whether to shrink the guests page cache
    diff_huge_pages =
        qatomic_read(&dev->target_huge) - qatomic_read(&dev->actual_huge);
    if (diff_huge_pages > 0 && shrink_pagecache &&
        virtio_has_feature(dev->host_features, LL_BALLOON_F_SHRINK_PAGECACHE)) {
        if (ll_try_shrink_pagecache(dev, diff_huge_pages)) {
            // Retry inflation later
            timer_mod(dev->retry_llfree_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_HOST) + LL_RETRY_INTERVAL);
        }
    }
}

static void ll_balloon_update_size_cb(void *opaque)
{
    VirtIOLLFreeBalloon *dev = opaque;
    ll_balloon_update_size(dev, true);
}

// target is rounded down to next aligned huge page
static void ll_balloon_resize(void *opaque, ram_addr_t target)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(opaque);
    ram_addr_t vm_ram_size = get_current_ram_size();

    if (target & (LL_HUGE_SIZE - 1)) {
        warn_report("Target is not huge page (2MiB) aligned, will be rounded "
                    "down to next alignment\n");
    }

    if (target > vm_ram_size)
        target = vm_ram_size;

    qatomic_set_u64(&dev->target_huge, (vm_ram_size - target) >> LL_HUGE_SHIFT);

    // we reset the retry counter on a new target value
    dev->pagecache_retries = 0;

    info_report("llfree_balloon_start %lu ns",
                qemu_clock_get_ns(QEMU_CLOCK_REALTIME));
    ll_balloon_update_size(dev, true);
}

/*-----------------------------------------------------------------------------------------------
| (Automatic) Scheduling Functions
-------------------------------------------------------------------------------------------------*/

static void ll_schedule_auto_reclaim(VirtIOLLFreeBalloon *dev)
{
    timer_mod(dev->auto_mode_timer,
              qemu_clock_get_ns(QEMU_CLOCK_HOST) + LL_RETRY_INTERVAL);
}

static void ll_balloon_auto_reclaim(void *opaque)
{
    VirtIOLLFreeBalloon *dev = opaque;

    ll_auto_soft_reclaim(dev);

    // and repeat with configured time interval
    ll_schedule_auto_reclaim(dev);
}

/*-----------------------------------------------------------------------------------------------
| VirtQueue Handler Functions
-------------------------------------------------------------------------------------------------*/

// scheduled in main event loop
static void handle_llfree_zone_info(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(vdev);

    VirtQueueElement *elem = virtqueue_pop(vq, sizeof(VirtQueueElement));
    if (!elem) {
        warn_report("%s: No element in queue (null pointer)", __func__);
        return;
    }

    void *buffer = g_malloc(ll_zone_buffer_size());
    iov_to_buf(elem->out_sg, elem->out_num, 0, buffer, ll_zone_buffer_size());
    ll_nodes_add_zone(dev->nodes, buffer);

    virtqueue_push(vq, elem, 0);
    virtio_notify(vdev, vq);
    g_free(elem);
    return;
}

struct install_range_ctx {
    VirtIOLLFreeBalloon *dev;
    ll_zone_t *zone;
};

static int install_range_cb(void *ctx, uint64_t frame, size_t len)
{
    struct install_range_ctx *data = ctx;
    g_assert(frame % LL_HUGE_COUNT == 0);
    g_assert(frame < ll_zone_frames(data->zone));

    hwaddr gpa_addr = (ll_zone_start_frame(data->zone) + frame) * LL_PAGE_SIZE;
    map_range(data->dev, gpa_addr, len * LL_PAGE_SIZE);

    // Update llfree state
    for (size_t i = 0; i < len; i += LL_HUGE_COUNT) {
        llfree_result_t res = ll_zone_install(data->zone, frame + i);
        if (!llfree_is_ok(res)) {
            error_report("Failed install %ld", frame + i);
            return -1;
        }
    }
    return 0;
}

// Handle request from guest to populate the given memory range
static void handle_install_request(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(vdev);

    VirtQueueElement *elem = virtqueue_pop(vq, sizeof(VirtQueueElement));
    if (!elem)
        return;

    ll_install_info_t info;
    iov_to_buf(elem->out_sg, elem->out_num, 0, &info,
               sizeof(ll_install_info_t));

    // calculate corresponding huge frame
    uint64_t huge_frame = align_down(info.frame, LL_HUGE_COUNT);
    ll_zone_t *zone = ll_nodes_get_zone(dev->nodes, info.node, info.zone);

    ll_zone_lock(zone);

    // already populated
    if (!ll_zone_is_evicted(zone, huge_frame)) {
        // check host state
        g_assert(
            ll_reclaim_states_is_installed(ll_zone_states(zone), huge_frame));
        goto finish;
    }

    // check host state
    g_assert(ll_reclaim_states_is_soft(ll_zone_states(zone), huge_frame));

    struct install_range_ctx ctx = {.zone = zone, .dev = dev};

#if 0
    // deflate whole tree
    aggregate_t ag = aggregate_start(install_range_cb, &ctx);
    uint64_t tree_offset = align_down(huge_frame, LLFREE_TREE_SIZE);
    for (size_t child = 0; child < LLFREE_TREE_CHILDREN; child++) {
        uint64_t frame = tree_offset + child * LLFREE_CHILD_SIZE;
        if (ll_frame_states_install(ll_zone_states(zone), frame)) {
            aggregate_next(&ag, frame, LL_HUGE_COUNT);
        }
    }
    aggregate_finish(&ag);
#else
    g_assert(ll_frame_states_install(ll_zone_states(zone), huge_frame));
    install_range_cb(&ctx, huge_frame, LL_HUGE_COUNT);
#endif

finish:
    ll_zone_unlock(zone);

    virtqueue_push(vq, elem, 0);
    virtio_notify(vdev, vq);
    g_free(elem);
}

// scheduled in main event loop
static void handle_update_request(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(vdev);

    VirtQueueElement *elem = virtqueue_pop(vq, sizeof(VirtQueueElement));
    if (!elem) {
        warn_report("No element in llfree_request_vq (null pointer) \n");
        return;
    }

    ll_notification_t guest_req;
    iov_to_buf(elem->out_sg, elem->out_num, 0, &guest_req,
               sizeof(ll_notification_t));
    virtqueue_push(vq, elem, 0);
    virtio_notify(vdev, vq);
    g_free(elem);

    switch (guest_req) {
    case PAGECACHE_DROPPED:
        ll_balloon_update_size(dev, false);
        break;
    default:
        break;
    }
}

/*-----------------------------------------------------------------------------------------------
| RamDiscardManager Functions,  Part 2
| Implementation is largely taken directly from virtio-mem
-------------------------------------------------------------------------------------------------*/

typedef int (*ll_wrapper_cb)(MemoryRegionSection *s, void *opaque);

struct ll_wrapper_data {
    size_t start_frame;
    MemoryRegionSection *section;
    ll_wrapper_cb cb;
    void *opaque;
};
static int ll_section_wrapper(void *ctx, uint64_t frame, size_t len)
{
    struct ll_wrapper_data *data = ctx;
    MemoryRegionSection tmp = *data->section;
    hwaddr offset = (data->start_frame + frame) * LL_PAGE_SIZE;
    size_t size = len * LL_PAGE_SIZE;

    if (intersect_memory_section(&tmp, offset, size)) {
        int ret = (data->cb)(&tmp, data->opaque);
        if (ret < 0)
            error_report("rdl notify failed %lx-%lx", offset, offset + size);
        return ret;
    }
    return 0;
}

static int ll_for_each_section(const VirtIOLLFreeBalloon *dev,
                               MemoryRegionSection *s, ll_wrapper_cb cb,
                               void *opaque, bool installed)
{
    if (ll_nodes_len(dev->nodes) == 0) {
        // Not initialized: just map all!
        struct ll_wrapper_data ctx = {
            .start_frame = s->offset_within_address_space / LL_PAGE_SIZE,
            .section = s,
            .cb = cb,
            .opaque = opaque,
        };
        return ll_section_wrapper(&ctx, 0, int128_get64(s->size));
    }

    for (size_t node = 0; node < ll_nodes_len(dev->nodes); node++) {
        for (ll_zone_type_t zid = 0; zid < LL_ZONE_LEN; zid++) {
            ll_zone_t *zone = ll_nodes_get_zone(dev->nodes, node, zid);
            if (zone == NULL)
                continue;
            ll_zone_lock(zone);

            struct ll_wrapper_data ctx = {
                .start_frame = ll_zone_start_frame(zone),
                .section = s,
                .cb = cb,
                .opaque = opaque,
            };
            aggregate_t ag = aggregate_start(ll_section_wrapper, &ctx);
            for (size_t f = 0; f < ll_zone_frames(zone); f += LL_HUGE_COUNT) {
                if (ll_reclaim_states_is_installed(ll_zone_states(zone), f) ==
                    installed) {

                    int ret = aggregate_next(&ag, f, LL_HUGE_COUNT);
                    if (ret < 0) {
                        ll_zone_unlock(zone);
                        return ret;
                    }
                }
            }
            int ret = aggregate_finish(&ag);

            ll_zone_unlock(zone);
            return ret;
        }
    }
    return 0;
}

static uint64_t ll_rdm_get_min_granularity(const RamDiscardManager *rdm,
                                           const MemoryRegion *mr)
{
    return LL_HUGE_SIZE;
}

static bool ll_rdm_is_populated(const RamDiscardManager *rdm,
                                const MemoryRegionSection *s)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(rdm);
    uint64_t start = s->offset_within_region / LL_PAGE_SIZE;
    uint64_t end = DIV_ROUND_UP(s->offset_within_region + int128_get64(s->size),
                                LL_PAGE_SIZE);
    uint64_t len = end - start;

    for (size_t node = 0; node < ll_nodes_len(dev->nodes); node++) {
        for (ll_zone_type_t zid = 0; zid < LL_ZONE_LEN; zid++) {
            ll_zone_t *zone = ll_nodes_get_zone(dev->nodes, node, zid);
            if (zone == NULL)
                continue;
            if (ll_zone_start_frame(zone) <= start &&
                len <= ll_zone_frames(zone)) {
                ll_reclaim_states_t *states = ll_zone_states(zone);
                for (size_t i = 0; i < len; i++)
                    if (!ll_reclaim_states_is_installed(states, start + i))
                        return false;
                return true;
            }
        }
    }
    error_report("Zone not found for region %lu", start);
    return false;
}

static int ll_rdm_replay_populated(const RamDiscardManager *rdm,
                                   MemoryRegionSection *section,
                                   ReplayRamPopulate replay_fn, void *opaque)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(rdm);
    return ll_for_each_section(dev, section, replay_fn, opaque, true);
}

struct replay_discard_data {
    ReplayRamDiscard replay_fn;
    void *opaque;
};
static int ll_replay_discard_cb(MemoryRegionSection *section, void *opaque)
{
    struct replay_discard_data *data = opaque;
    (data->replay_fn)(section, data->opaque);
    return 0;
}
static void ll_rdm_replay_discarded(const RamDiscardManager *rdm,
                                    MemoryRegionSection *section,
                                    ReplayRamDiscard replay_fn, void *opaque)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(rdm);
    struct replay_discard_data data = {.replay_fn = replay_fn,
                                       .opaque = opaque};
    ll_for_each_section(dev, section, ll_replay_discard_cb, &data, false);
}

struct ll_notify_populate_data {
    NotifyRamPopulate cb;
    RamDiscardListener *rdl;
};
static int ll_notify_populate_cb(MemoryRegionSection *s, void *opaque)
{
    struct ll_notify_populate_data *data = opaque;
    int ret = (data->cb)(data->rdl, s);
    if (ret < 0)
        error_report("rdl notify failed %lx-%lx",
                     s->offset_within_address_space,
                     s->offset_within_address_space + int128_get64(s->size));
    return ret;
}
static void ll_rdm_register_listener(RamDiscardManager *rdm,
                                     RamDiscardListener *rdl,
                                     MemoryRegionSection *s)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(rdm);

    rdl->section = memory_region_section_new_copy(s);

    QLIST_INSERT_HEAD(&dev->rdl_list, rdl, next);
    struct ll_notify_populate_data data = {
        .cb = rdl->notify_populate,
        .rdl = rdl,
    };
    int ret = ll_for_each_section(dev, rdl->section, ll_notify_populate_cb,
                                  &data, true);
    if (ret < 0) {
        error_report("%s: Replaying non inflated ranges failed: %s", __func__,
                     strerror(-ret));
    }

    virtio_add_feature((uint64_t *)&dev->host_features, LL_BALLOON_F_VFIO);
}

struct ll_notify_discard_data {
    NotifyRamDiscard cb;
    RamDiscardListener *rdl;
};
static int ll_notify_discard_cb(MemoryRegionSection *s, void *opaque)
{
    struct ll_notify_discard_data *data = opaque;
    (data->cb)(data->rdl, s);
    return 0;
}
static void ll_rdm_unregister_listener(RamDiscardManager *rdm,
                                       RamDiscardListener *rdl)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(rdm);

    if (rdl->double_discard_supported) {
        rdl->notify_discard(rdl, rdl->section);
    } else {
        struct ll_notify_discard_data data = {
            .cb = rdl->notify_discard,
            .rdl = rdl,
        };
        ll_for_each_section(dev, rdl->section, ll_notify_discard_cb, &data,
                            true);
    }

    memory_region_section_free_copy(rdl->section);
    rdl->section = NULL;
    QLIST_REMOVE(rdl, next);

    if (QLIST_EMPTY(&dev->rdl_list))
        virtio_clear_feature((uint64_t *)&dev->host_features,
                             LL_BALLOON_F_VFIO);
}

/*-----------------------------------------------------------------------------------------------
| Functions for managing iothread pool backend for multi-threaded auto-deflate
| taken from virtio-blk.c, slightly modified
-------------------------------------------------------------------------------------------------*/
static void balloon_ioeventfd_attach(VirtIOLLFreeBalloon *dev)
{
    VirtIODevice *vdev = VIRTIO_DEVICE(dev);

    for (uint16_t i = 0; i < dev->num_queues; i++) {
        VirtQueue *vq = virtio_get_queue(vdev, i);
        virtio_queue_aio_attach_host_notifier(vq, dev->vq_aio_context[i]);
    }
}

/* Context: BQL held */
static void balloon_stop_ioeventfd(VirtIODevice *vdev)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(vdev);
    BusState *qbus = qdev_get_parent_bus(DEVICE(dev));
    unsigned i;
    unsigned nvqs = dev->num_queues;

    /*
     * Batch all the host notifiers in a single transaction to avoid
     * quadratic time complexity in address_space_update_ioeventfds().
     */
    memory_region_transaction_begin();

    for (i = 0; i < nvqs; i++) {
        virtio_bus_set_host_notifier(VIRTIO_BUS(qbus), i, false);
    }

    /*
     * The transaction expects the ioeventfds to be open when it
     * commits. Do it now, before the cleanup loop.
     */
    memory_region_transaction_commit();

    for (i = 0; i < nvqs; i++) {
        virtio_bus_cleanup_host_notifier(VIRTIO_BUS(qbus), i);
    }
}

/* Context: BQL held */
static int balloon_start_ioeventfd(VirtIODevice *vdev)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(vdev);
    BusState *qbus = BUS(qdev_get_parent_bus(DEVICE(dev)));
    unsigned i;
    unsigned nvqs = dev->num_queues;
    int r;

    /*
     * Batch all the host notifiers in a single transaction to avoid
     * quadratic time complexity in address_space_update_ioeventfds().
     */
    memory_region_transaction_begin();

    /* Set up virtqueue notify */
    for (i = 0; i < nvqs; i++) {
        r = virtio_bus_set_host_notifier(VIRTIO_BUS(qbus), i, true);
        if (r != 0) {
            int j = i;

            fprintf(stderr, "virtio-blk failed to set host notifier (%d)\n", r);
            while (i--) {
                virtio_bus_set_host_notifier(VIRTIO_BUS(qbus), i, false);
            }

            /*
             * The transaction expects the ioeventfds to be open when it
             * commits. Do it now, before the cleanup loop.
             */
            memory_region_transaction_commit();

            while (j--) {
                virtio_bus_cleanup_host_notifier(VIRTIO_BUS(qbus), j);
            }
            goto fail_host_notifiers;
        }
    }

    memory_region_transaction_commit();

    /*
     * Attach our iothreads
     */
    balloon_ioeventfd_attach(dev);
    return 0;

fail_host_notifiers:
    return -ENOSYS;
}

static bool
validate_iothread_vq_mapping_list(IOThreadVirtQueueMappingList *list,
                                  uint16_t num_queues, Error **errp)
{
    g_autofree unsigned long *vqs = bitmap_new(num_queues);
    g_autoptr(GHashTable) iothreads = g_hash_table_new(g_str_hash, g_str_equal);

    for (IOThreadVirtQueueMappingList *node = list; node; node = node->next) {
        const char *name = node->value->iothread;
        uint16List *vq;

        if (!iothread_by_id(name)) {
            error_setg(errp, "IOThread \"%s\" object does not exist", name);
            return false;
        }

        if (!g_hash_table_add(iothreads, (gpointer)name)) {
            error_setg(errp,
                       "duplicate IOThread name \"%s\" in iothread-vq-mapping",
                       name);
            return false;
        }

        if (node != list) {
            if (!!node->value->vqs != !!list->value->vqs) {
                error_setg(errp, "either all items in iothread-vq-mapping "
                                 "must have vqs or none of them must have it");
                return false;
            }
        }

        for (vq = node->value->vqs; vq; vq = vq->next) {
            if (vq->value >= num_queues) {
                error_setg(errp,
                           "vq index %u for IOThread \"%s\" must be "
                           "less than num_queues %u in iothread-vq-mapping",
                           vq->value, name, num_queues);
                return false;
            }

            if (test_and_set_bit(vq->value, vqs)) {
                error_setg(errp,
                           "cannot assign vq %u to IOThread \"%s\" "
                           "because it is already assigned",
                           vq->value, name);
                return false;
            }
        }
    }

    if (list->value->vqs) {
        for (uint16_t i = 0; i < num_queues; i++) {
            if (!test_bit(i, vqs)) {
                error_setg(
                    errp,
                    "missing vq %u IOThread assignment in iothread-vq-mapping",
                    i);
                return false;
            }
        }
    }

    return true;
}

/**
 * apply_iothread_vq_mapping:
 * @iothread_vq_mapping_list: The mapping of virtqueues to IOThreads.
 * @vq_aio_context: The array of AioContext pointers to fill in.
 * @num_queues: The length of @vq_aio_context.
 * @errp: If an error occurs, a pointer to the area to store the error.
 *
 * Fill in the AioContext for each virtqueue in the @vq_aio_context array given
 * the iothread-vq-mapping parameter in @iothread_vq_mapping_list.
 *
 * Returns: %true on success, %false on failure.
 **/
static bool
apply_iothread_vq_mapping(IOThreadVirtQueueMappingList *iothread_vq_mapping,
                          AioContext **vq_aio_context, uint16_t num_queues,
                          Error **errp)
{
    IOThreadVirtQueueMappingList *node;
    size_t num_iothreads = 0;

    if (!validate_iothread_vq_mapping_list(iothread_vq_mapping, num_queues,
                                           errp)) {
        return false;
    }

    for (node = iothread_vq_mapping; node; node = node->next) {
        num_iothreads++;
    }

    AioContext *main_loop_ctx = qemu_get_aio_context();
    for (uint32_t i = 0; i < LL_NUM_STATIC_QUEUES; i++) {
        vq_aio_context[i] = main_loop_ctx;
    }

    size_t cur_iothread = LL_NUM_STATIC_QUEUES;
    for (node = iothread_vq_mapping; node; node = node->next) {
        IOThread *iothread = iothread_by_id(node->value->iothread);
        AioContext *ctx = iothread_get_aio_context(iothread);

        /* Released in virtio_blk_vq_aio_context_cleanup() */
        object_ref(OBJECT(iothread));

        if (node->value->vqs) {
            /* Explicit vq:IOThread assignment */
            for (uint16List *vq = node->value->vqs; vq; vq = vq->next) {
                g_assert(vq->value < num_queues);
                vq_aio_context[vq->value] = ctx;
            }
        } else {
            /* Round-robin vq:IOThread assignment */
            for (size_t i = cur_iothread; i < num_queues; i += num_iothreads) {
                vq_aio_context[i] = ctx;
            }
        }

        cur_iothread++;
    }

    return true;
}

/*-----------------------------------------------------------------------------------------------
| Virtio Device Management Functions
-------------------------------------------------------------------------------------------------*/
static void ll_balloon_get_config(VirtIODevice *vdev, uint8_t *config_data)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(vdev);
    struct ll_balloon_config config = {};
    config.shrink_pagecache = cpu_to_le32(dev->shrink_pagecache);
    memcpy(config_data, &config, sizeof(struct ll_balloon_config));
    return;
}

static void ll_balloon_set_config(VirtIODevice *vdev,
                                  const uint8_t *config_data)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(vdev);
    struct ll_balloon_config config = {};
    memcpy(&config, config_data, sizeof(struct ll_balloon_config));
    dev->shrink_pagecache = le32_to_cpu(config.shrink_pagecache);
    return;
}

static void ll_balloon_stat(void *opaque, LLfreeBalloonInfo *info)
{
    VirtIOLLFreeBalloon *dev = opaque;
    info->actual = get_current_ram_size() -
                   (qatomic_read(&dev->actual_huge) << LL_HUGE_SHIFT);
    return;
}

static void ll_init_vqs(VirtIODevice *vdev, VirtIOLLFreeBalloon *dev)
{
    dev->llfree_info_vq = virtio_add_queue(vdev, 128, handle_llfree_zone_info);
    dev->llfree_request_vq = virtio_add_queue(vdev, 128, handle_update_request);

    // add one auto-deflate virtqueue per vcpu
    dev->llfree_auto_deflate_vqs = g_new(VirtQueue *, get_num_vcpus());
    for (uint32_t i = 0; i < get_num_vcpus(); i++) {
        dev->llfree_auto_deflate_vqs[i] =
            virtio_add_queue(vdev, 1, handle_install_request);
    }

    dev->num_queues = LL_NUM_STATIC_QUEUES + get_num_vcpus();
}

static bool ll_init_iothread_mapping(VirtIOLLFreeBalloon *dev, Error **errp)
{
    if (dev->num_queues == 0) {
        error_setg(errp, "number of queues can't be zero");
        return false;
    }

    dev->vq_aio_context = g_new(AioContext *, dev->num_queues);

    // start with default and map every virtqueue
    // to the main event loop
    AioContext *ctx = qemu_get_aio_context();
    for (unsigned i = 0; i < dev->num_queues; i++) {
        dev->vq_aio_context[i] = ctx;
    }

    // and if we have iothreads, then remap virtqueues
    // round robin onto the iothreads
    if (!dev->iothread_vq_mapping_list) {
        return false;
    }

    bool success =
        apply_iothread_vq_mapping(dev->iothread_vq_mapping_list,
                                  dev->vq_aio_context, dev->num_queues, errp);
    if (!success) {
        g_free(dev->vq_aio_context);
        dev->vq_aio_context = NULL;
    }
    return success;
}

// initialization and destruction
static void ll_balloon_device_realize(DeviceState *dev_state, Error **errp)
{
    int ret;
    VirtIODevice *vdev = VIRTIO_DEVICE(dev_state);
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(dev_state);

    // config size is zero for now, create a config?
    virtio_init(vdev, VIRTIO_ID_BALLOON, sizeof(struct ll_balloon_config));

    ret = qemu_add_llfree_balloon_handler(ll_balloon_resize, ll_balloon_stat,
                                          dev);

    if (ret < 0) {
        error_setg(errp, "Only one llfree balloon device is supported");
        return;
    }

    if (!virtio_device_ioeventfd_enabled(vdev)) {
        error_setg(errp, "virtio-llfree-balloon needs ioeventfd!");
        virtio_cleanup(vdev);
    }

    dev->nodes = ll_nodes_create();

    ll_init_vqs(vdev, dev);
    bool success = ll_init_iothread_mapping(dev, errp);
    g_assert(success && "Failed to initialize iothreads");

    qatomic_set_u64(&dev->actual_huge, 0);
    qatomic_set_u64(&dev->target_huge, 0);

    dev->retry_llfree_timer =
        aio_timer_new(qemu_get_aio_context(), QEMU_CLOCK_HOST, SCALE_NS,
                      ll_balloon_update_size_cb, dev);

    if (virtio_has_feature(dev->host_features, LL_BALLOON_F_AUTO_MODE)) {
        if (dev->auto_mode_iothread) {
            dev->auto_mode_timer = aio_timer_new(
                iothread_get_aio_context(dev->auto_mode_iothread),
                QEMU_CLOCK_HOST, SCALE_NS, ll_balloon_auto_reclaim, dev);
        } else {
            dev->auto_mode_timer =
                aio_timer_new(qemu_get_aio_context(), QEMU_CLOCK_HOST, SCALE_NS,
                              ll_balloon_auto_reclaim, dev);
        }

        ll_schedule_auto_reclaim(dev);
    }

    // register as a ramdiscard manager to get vfio notifications
#ifndef CONFIG_SIMULATE_BROKEN_VFIO
    MachineState *machine = MACHINE(qdev_get_machine());
    memory_region_set_ram_discard_manager(machine->ram,
                                          RAM_DISCARD_MANAGER(dev));
#endif
}

static void ll_balloon_device_unrealize(DeviceState *dev_state)
{
    VirtIODevice *vdev = VIRTIO_DEVICE(dev_state);
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(dev_state);

    ll_nodes_destroy(dev->nodes);

    qemu_remove_llfree_balloon_handler(dev);

    virtio_delete_queue(dev->llfree_info_vq);
    virtio_delete_queue(dev->llfree_request_vq);

    for (uint32_t i = 0; i < get_num_vcpus(); i++) {
        virtio_delete_queue(dev->llfree_auto_deflate_vqs[i]);
    }

    g_free(dev->vq_aio_context);
    dev->vq_aio_context = NULL;

    virtio_cleanup(vdev);
}

static uint64_t ll_balloon_get_features(VirtIODevice *vdev, uint64_t f,
                                        Error **errp)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(vdev);
    return f | dev->host_features;
}

// for now unused functions, but must exist
static int ll_balloon_post_load_device(void *opaque, int version_id)
{
    return 0;
}

// called on reboot, multiple times apparently...
static void ll_balloon_device_reset(VirtIODevice *vdev)
{
    VirtIOLLFreeBalloon *dev = VIRTIO_LLFREE_BALLOON(vdev);

    // here we need to deflate completely
    qatomic_set_u64(&dev->target_huge, 0);
    ll_balloon_update_size(dev, false);

    // delete all zones, they are no longer valid
    ll_nodes_destroy(dev->nodes);
    dev->nodes = ll_nodes_create();

    // ideally we could inflate after a reboot
    // but for that we would need a different, more appropriate callback
    return;
}

static void ll_balloon_set_status(VirtIODevice *vdev, uint8_t status)
{
    return;
}

static void ll_balloon_instance_init(Object *obj) { return; }

static const VMStateDescription vmstate_virtio_llfree_balloon_device = {
    .name = "virtio-llfree-balloon-device",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ll_balloon_post_load_device,
    .fields =
        (VMStateField[]){VMSTATE_UINT32(shrink_pagecache, VirtIOLLFreeBalloon),
                         VMSTATE_END_OF_LIST()},
};

static const VMStateDescription vmstate_virtio_llfree_balloon = {
    .name = "virtio-llfree-balloon",
    .minimum_version_id = 1,
    .version_id = 1,
    .fields = (VMStateField[]){VMSTATE_VIRTIO_DEVICE, VMSTATE_END_OF_LIST()},
};

static Property virtio_llfree_balloon_properties[] = {
    DEFINE_PROP_BIT("shrink-pagecache", VirtIOLLFreeBalloon, host_features,
                    LL_BALLOON_F_SHRINK_PAGECACHE, false),
    DEFINE_PROP_BIT("auto-mode", VirtIOLLFreeBalloon, host_features,
                    LL_BALLOON_F_AUTO_MODE, true),
    DEFINE_PROP_BIT("vfio", VirtIOLLFreeBalloon, host_features,
                    LL_BALLOON_F_VFIO, false),
    DEFINE_PROP_LINK("auto-mode-iothread", VirtIOLLFreeBalloon,
                     auto_mode_iothread, TYPE_IOTHREAD, IOThread *),
    DEFINE_PROP_IOTHREAD_VQ_MAPPING_LIST(
        "iothread-vq-mapping", VirtIOLLFreeBalloon, iothread_vq_mapping_list),
    DEFINE_PROP_END_OF_LIST(),
};

static void ll_balloon_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    VirtioDeviceClass *vdc = VIRTIO_DEVICE_CLASS(klass);
    RamDiscardManagerClass *rdmc = RAM_DISCARD_MANAGER_CLASS(klass);

    device_class_set_props(dc, virtio_llfree_balloon_properties);
    dc->vmsd = &vmstate_virtio_llfree_balloon;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    vdc->realize = ll_balloon_device_realize;
    vdc->unrealize = ll_balloon_device_unrealize;
    vdc->reset = ll_balloon_device_reset;
    vdc->get_config = ll_balloon_get_config;
    vdc->set_config = ll_balloon_set_config;
    vdc->get_features = ll_balloon_get_features;
    vdc->set_status = ll_balloon_set_status;
    vdc->start_ioeventfd = balloon_start_ioeventfd;
    vdc->stop_ioeventfd = balloon_stop_ioeventfd;
    vdc->vmsd = &vmstate_virtio_llfree_balloon_device;

    rdmc->get_min_granularity = ll_rdm_get_min_granularity;
    rdmc->is_populated = ll_rdm_is_populated;
    rdmc->replay_populated = ll_rdm_replay_populated;
    rdmc->replay_discarded = ll_rdm_replay_discarded;
    rdmc->register_listener = ll_rdm_register_listener;
    rdmc->unregister_listener = ll_rdm_unregister_listener;
}

static const TypeInfo virtio_llfree_balloon_info = {
    .name = TYPE_VIRTIO_LLFREE_BALLOON,
    .parent = TYPE_VIRTIO_DEVICE,
    .instance_size = sizeof(VirtIOLLFreeBalloon),
    .instance_init = ll_balloon_instance_init,
    .class_init = ll_balloon_class_init,
    .interfaces = (InterfaceInfo[]){{TYPE_RAM_DISCARD_MANAGER}, {}},
};

static void virtio_register_types(void)
{
    type_register_static(&virtio_llfree_balloon_info);
}

type_init(virtio_register_types)
