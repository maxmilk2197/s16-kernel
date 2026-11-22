#!/usr/bin/env bash
# MMNeko kernel build — runs on GitHub Actions (ubuntu-22.04).
#
# Lives in the repo on purpose: the workflow file cannot be pushed with the
# token available here (`workflow` scope), so the workflow stays a thin wrapper
# and every real change happens in this script, which a normal push carries.
#
# Toolchain: GCC 4.9.x 20150123 from LineageOS 18.1 — the only one that builds
# this 4.14 vendor tree without choking on SPRD's inline asm and old-style
# declarations.
set -o pipefail
set +e

: "${GITHUB_WORKSPACE:=$(pwd)}"
TC_DIR="${HOME}/toolchain"
LOG="${GITHUB_WORKSPACE}/build.log"

say() { printf '\n=== %s ===\n' "$*"; }

say "1. toolchain"
if [ ! -x "${TC_DIR}/bin/aarch64-linux-android-gcc" ] && \
   [ ! -x "${TC_DIR}/binok/aarch64-linux-android-gcc" ]; then
    rm -rf "${TC_DIR}"
    git clone --depth=1 --branch lineage-18.1 \
        https://github.com/LineageOS/android_prebuilts_gcc_linux-x86_aarch64_aarch64-linux-android-4.9 \
        "${TC_DIR}" || { echo "TOOLCHAIN_CLONE_FAIL"; exit 1; }
fi

if [ -d "${TC_DIR}/bin" ] && [ -x "${TC_DIR}/bin/aarch64-linux-android-gcc" ]; then
    TC="${TC_DIR}/bin"
elif [ -d "${TC_DIR}/binok" ]; then
    TC="${TC_DIR}/binok"
else
    TC="${TC_DIR}/bin"
fi
export CROSS_COMPILE="${TC}/aarch64-linux-android-"
echo "TC=${TC}"
"${CROSS_COMPILE}gcc" --version | head -1 || { echo "NO_GCC"; exit 1; }

say "2. host deps"
if ! command -v bc >/dev/null 2>&1; then
    sudo apt-get update -qq
    sudo apt-get install -y -qq bc bison flex libssl-dev libelf-dev ccache
fi

say "3. config guard"
BAD=0
need() { grep -q "^CONFIG_$1=y" .config || { echo "FAIL: CONFIG_$1 lost"; BAD=1; }; }
off()  { grep -q "^CONFIG_$1=y" .config && { echo "FAIL: CONFIG_$1 must be off"; BAD=1; }; }
need ARCH_SPRD
need COMPAT
need SENSORS_BD1662
need SENSORS_MIR3DA
need NEKOBOOT_KEYRESCUE
need KSU_SUSFS
off KSU_DEBUG
off KSU_SUSFS_ENABLE_LOG
off DA280
off NEKOBOOT_BOOTGUARD
[ "$BAD" = 0 ] || { echo "CONFIG_GUARD_FAIL"; exit 1; }
echo "config guard passed"

say "4. build"
export ARCH=arm64
export CCACHE_COMPILERCHECK=content
export CC="ccache ${CROSS_COMPILE}gcc"
export KBUILD_BUILD_USER=MMNeko
export KBUILD_BUILD_HOST=MMNeko
rm -f arch/arm64/boot/Image
START=$(date +%s)
make -j"$(nproc)" Image > "$LOG" 2>&1
RC=$?
END=$(date +%s)
echo "make rc=${RC} elapsed=$((END-START))s"

if [ "$RC" -ne 0 ]; then
    echo "---- first errors ----"
    grep -nE "error:|\*\*\* \[|Error [0-9]+|undefined|No rule|not found" "$LOG" | head -40
    echo "---- tail 40 ----"
    tail -40 "$LOG"
    exit "$RC"
fi

say "5. result"
ls -l arch/arm64/boot/Image
sha256sum arch/arm64/boot/Image
strings -n 5 arch/arm64/boot/Image | grep '^Linux version' | head -1
echo "BUILD_OK"
