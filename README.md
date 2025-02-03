# HyperAlloc: QEMU

This repository contains a QEMU 8.2.1 with a modified virtio-balloon device that supports huge pages as comparison to HyperAlloc.

- See: https://github.com/luhsra/hyperalloc-bench

## Publication

HyperAlloc: Efficient VM Memory De/Inflation via Hypervisor-Shared Page-Frame Allocators Lars Wrenger, Kenny Albes, Marco Wurps, Christian Dietrich, Daniel Lohmann In: Proceedings of the 20th European Conference on Computer Systems (EuroSys 2025); ACM

## Building

To build QEMU with Huge Paging:

```sh
mkdir build-huge
cd build-huge
export CC=clang
../configure --enable-debug --target-list=x86_64-softmmu --enable-slirp --enable-trace-backends=simple
ninja
```
