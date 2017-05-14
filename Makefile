SROS_ROOT := $(shell pwd)

BUILDROOT_BASE := ${SROS_ROOT}/buildroot-2017.02.2
KERNEL_BASE := ${SROS_ROOT}/linux-4.9.13
TOOLCHAIN_DIR := ${BUILDROOT_BASE}/output/host/usr/bin
TOOLCHAIN_PREFIX := powerpc-linux-

export PATH := ${PATH}:${TOOLCHAIN_DIR}

.PHONY: buildroot
buildroot:
	make -C ${BUILDROOT_BASE}
	cp -p ${BUILDROOT_BASE}/output/images/rootfs.tar ${SROS_ROOT}/output/rootfs.tar

.PHONY: buildroot_defconfig
buildroot_defconfig:
	cp -p ${SROS_ROOT}/config/sros_config ${BUILDROOT_BASE}/.config

.PHONY: buildroot_saveconfig
buildroot_saveconfig:
	cp -p ${BUILDROOT_BASE}/.config ${SROS_ROOT}/config/sros_config 

.PHONY: buildroot_xconfig
buildroot_xconfig:
	make -C ${BUILDROOT_BASE} xconfig

.PHONY: buildroot_clean
buildroot_clean:
	make -C ${BUILDROOT_BASE} clean
	rm -f ${SROS_ROOT}/output/rootfs.tar


.PHONY: kernel
kernel:
	make -C ${KERNEL_BASE} V=1 ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} uImage
	make -C ${KERNEL_BASE} V=1 ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} yosemite.dtb
	cp -p ${KERNEL_BASE}/arch/powerpc/boot/uImage ${SROS_ROOT}/output/uImage
	cp -p ${KERNEL_BASE}/arch/powerpc/boot/yosemite.dtb ${SROS_ROOT}/output/yosemite.dtb

.PHONY: kernel_defconfig
kernel_defconfig:
	cp -p ${SROS_ROOT}/config/sros_kernel_config ${KERNEL_BASE}/.config
	make -C ${KERNEL_BASE} ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} oldconfig

.PHONY: kernel_xconfig
kernel_xconfig:
	make -C ${KERNEL_BASE} ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} xconfig

.PHONY: kernel_saveconfig
kernel_saveconfig:
	cp -p ${KERNEL_BASE}/.config ${SROS_ROOT}/config/sros_kernel_config 

.PHONY: kernel_mrproper
kernel_mrproper:
	make -C ${KERNEL_BASE} ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} mrproper
	rm -f ${SROS_ROOT}/output/uImage
	rm -f ${SROS_ROOT}/output/yosemite.dtb

