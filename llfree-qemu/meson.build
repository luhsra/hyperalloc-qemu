llfree_qemu_inc = include_directories('include')
llfree_qemu_inc_dep = declare_dependency(include_directories : llfree_qemu_inc)
system_llfree_c_ss = ss.source_set()
system_llfree_c_ss.add(files('llfree_zone.c'))
system_llfree_c_ss.add(files('llfree_states.c'))
system_llfree_c_ss.add(llfree_qemu_inc_dep)
system_llfree_c_ss.add(llfree_c_private_dep)
system_ss.add_all(system_llfree_c_ss)
