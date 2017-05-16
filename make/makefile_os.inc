#  Basic Makefile Environment Variables.
#
#   Only modified when Kernel or Buildroot Version Changes.
#
BUILDROOT_BASE := ${SROS_ROOT}/buildroot-2017.02.2
KERNEL_BASE := ${SROS_ROOT}/linux-4.9.13
UBOOT_BASE := ${SROS_ROOT}/u-boot-2017.05
TOOLCHAIN_DIR := ${BUILDROOT_BASE}/output/host/usr/bin
TOOLCHAIN_PREFIX := powerpc-linux-
export PATH := ${PATH}:${TOOLCHAIN_DIR}
