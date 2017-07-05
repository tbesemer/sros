#  Setup the Environment.
#
SROS_ROOT := $(shell pwd)
include ${SROS_ROOT}/make/makefile_os.inc

#  Master Build Targets
#
.PHONY: all
all:  buildroot_defconfig buildroot kernel_defconfig kernel

.PHONY: clean
clean:	kernel_mrproper buildroot_clean


#  Buildroot Build Targets.
#
.PHONY: buildroot
buildroot:
	make -C ${BUILDROOT_BASE}
	cp -p ${BUILDROOT_BASE}/output/images/rootfs.tar ${SROS_ROOT}/output/rootfs.tar

.PHONY: buildroot_defconfig
buildroot_defconfig:
	@ if [ -f ${BUILDROOT_BASE}/.config ]; then \
	      echo "Config Exists, Not Overwriting"; \
	  else \
	      cp -p ${SROS_ROOT}/config/sros_config ${BUILDROOT_BASE}/.config; \
          fi;

.PHONY: buildroot_saveconfig
buildroot_saveconfig:
	cp -p ${BUILDROOT_BASE}/.config ${SROS_ROOT}/config/sros_config 

.PHONY: buildroot_xconfig
buildroot_xconfig:
	make -C ${BUILDROOT_BASE} xconfig

.PHONY: buildroot_clean
buildroot_clean:
	make -C ${BUILDROOT_BASE} clean
	rm -f ${BUILDROOT_BASE}/.config
	rm -f ${SROS_ROOT}/output/rootfs.tar

#  Kernel Build Targets.
#
.PHONY: kernel
kernel:
	make -C ${KERNEL_BASE} V=1 ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} cuImage.yosemite modules
	# make -C ${KERNEL_BASE} V=1 ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} yosemite.dtb
	cp -p ${KERNEL_BASE}/arch/powerpc/boot/cuImage.yosemite ${SROS_ROOT}/output/cuImage.yosemite
	# cp -p ${KERNEL_BASE}/arch/powerpc/boot/yosemite.dtb ${SROS_ROOT}/output/yosemite.dtb

.PHONY: kernel_defconfig
kernel_defconfig:
	@ if [ -f ${KERNEL_BASE}/.config ]; then \
	      echo "Config Exists, Not Overwriting"; \
	  else \
	      cp -p ${SROS_ROOT}/config/sros_kernel_config ${KERNEL_BASE}/.config; \
	      make -C ${KERNEL_BASE} ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} oldconfig; \
          fi;

.PHONY: kernel_xconfig
kernel_xconfig:
	make -C ${KERNEL_BASE} ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} xconfig

.PHONY: kernel_saveconfig
kernel_saveconfig:
	cp -p ${KERNEL_BASE}/.config ${SROS_ROOT}/config/sros_kernel_config 

.PHONY: kernel_mrproper
kernel_mrproper:
	make -C ${KERNEL_BASE} V=1 ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} mrproper
	rm -f ${SROS_ROOT}/output/uImage
	rm -f ${SROS_ROOT}/output/yosemite.dtb

.PHONY: uboot
uboot:
	make -C ${UBOOT_BASE} CROSS_COMPILE=${TOOLCHAIN_PREFIX} V=1 flyer3d_defconfig
	make -C ${UBOOT_BASE} CROSS_COMPILE=${TOOLCHAIN_PREFIX} V=1 
	cp -p ${UBOOT_BASE}/u-boot.bin ${SROS_ROOT}/output/u-boot.bin

.PHONY: uboot_saveconfig
uboot_saveconfig:
	cp -p ${UBOOT_BASE}/.config ${SROS_ROOT}/config/sros_uboot_config 

.PHONY: help
help:
	@ less README_help.txt
