# s16-kernel (MMNeko)

XinTaiQi S16B Plus (Unisoc UWS6152 / ums312 / sharkl5), Linux 4.14.98 ARM64 non-GKI vendor tree.

- Base: pristine vendor snapshot (`w527 upstream snapshot`)
- Hardware fixes: touch IRQ edge, FGU probe-defer, eta6947 charging gate, sprd_mcu camera steal,
  MIR3DA rotation remap, bd1662 health sensors (HR/SpO2/BP), BT tty HCI, och1970 hall input name
- Kernel extras: `nekoboot_bootguard` (boot watchdog), `nekoboot_keyrescue` (boot-window key combos)
- KernelSU: BakaSU main (ReSukiSU 7b65bde0, UAPI 6) vendored; printk flood
  silenced + SUSFS in-tree (`fs/susfs.c` v2.2.0) + `ksu_handle_vfs_fstat` hook
  wired into `fs/stat.c` (vfs_statx_fd), fixmap cast fix for gcc 4.9
- Build: edit `.config` in place. NEVER `make olddefconfig` / `make defconfig` — it drops
  `CONFIG_ARCH_SPRD` and `CONFIG_COMPAT` and the link explodes with hundreds of undefined SPRD symbols.
- CI: `.github/workflows/build.yml` builds `Image` with the GCC 4.9 LineageOS 18.1 toolchain.
