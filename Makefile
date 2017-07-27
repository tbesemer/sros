#  Setup the Environment.
#
SROS_ROOT := $(shell pwd)
include ${SROS_ROOT}/make/makefile_os.inc

export STAGING_DIR := ${SROS_ROOT}/staging
export OUTPUT_DIR := ${SROS_ROOT}/output
export CPIO_INPUT := ${SROS_ROOT}/output/rootfs_initramfs.cpio
export CPIO_OUTPUT := ${SROS_ROOT}/output/rootfs_initramfs_kernel.cpio
export INITRAMFS_OVERLAY :=${SROS_ROOT}/initramfs_overlay
export ROOTFS_INPUT := ${SROS_ROOT}/output/new_rootfs.tar
export ROOTFS_OUTPUT := ${SROS_ROOT}/output/rootfs_production.tar

#  Master Build Targets
#
.PHONY: all
all:  buildroot_initramfs_defconfig buildroot_initramfs kernel_initramfs_defconfig kernel_initramfs

.PHONY: buildroot_production
buildroot_production: buildroot_clean buildroot_defconfig buildroot

.PHONY: clean
clean:	kernel_mrproper buildroot_clean


#  Buildroot Build Targets.
#
.PHONY: buildroot
buildroot:
	make -C ${BUILDROOT_BASE}
	cp -p ${BUILDROOT_BASE}/output/images/rootfs.tar ${SROS_ROOT}/output/new_rootfs.tar
	fakeroot bin/do_build_rootfs.sh

.PHONY: buildroot_initramfs
buildroot_initramfs:
	make -C ${BUILDROOT_BASE}
	cp -p ${BUILDROOT_BASE}/output/images/rootfs.cpio ${SROS_ROOT}/output/rootfs_initramfs.cpio

.PHONY: buildroot_defconfig
buildroot_defconfig:
	@ if [ -f ${BUILDROOT_BASE}/.config ]; then \
	      echo "Config Exists, Not Overwriting"; \
	  else \
	      cp -p ${SROS_ROOT}/config/sros_buildroot_config ${BUILDROOT_BASE}/.config; \
          fi;

.PHONY: buildroot_initramfs_defconfig
buildroot_initramfs_defconfig:
	@ if [ -f ${BUILDROOT_BASE}/.config ]; then \
	      echo "Config Exists, Not Overwriting"; \
	  else \
	      cp -p ${SROS_ROOT}/config/sros_buildroot_initramfs_config ${BUILDROOT_BASE}/.config; \
          fi;

.PHONY: buildroot_saveconfig
buildroot_saveconfig:
	cp -p ${BUILDROOT_BASE}/.config ${SROS_ROOT}/config/sros_buildroot_config 

.PHONY: buildroot_initramfs_saveconfig
buildroot_initramfs_saveconfig:
	cp -p ${BUILDROOT_BASE}/.config ${SROS_ROOT}/config/sros_buildroot_initramfs_config 

.PHONY: buildroot_xconfig
buildroot_xconfig:
	make -C ${BUILDROOT_BASE} xconfig

.PHONY: buildroot_clean
buildroot_clean:
	make -C ${BUILDROOT_BASE} clean
	rm -f ${BUILDROOT_BASE}/.config

#  Kernel Build Targets.
#
.PHONY: kernel
kernel:
	make -C ${KERNEL_BASE} V=1 ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} cuImage.yosemite
	cp -p ${KERNEL_BASE}/arch/powerpc/boot/cuImage.yosemite ${SROS_ROOT}/output/cuImage.yosemite
	cp output/cuImage.yosemite /tftpboot/

.PHONY: kernel_initramfs
kernel_initramfs: initramfs_final
	make -C ${KERNEL_BASE} V=1 ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} cuImage.yosemite
	cp -p ${KERNEL_BASE}/arch/powerpc/boot/cuImage.yosemite ${SROS_ROOT}/output/cuImage.yosemite.initramfs
	cp output/cuImage.yosemite.initramfs /tftpboot/


.PHONY: kernel_defconfig
kernel_defconfig:
	@ if [ -f ${KERNEL_BASE}/.config ]; then \
	      echo "Config Exists, Not Overwriting"; \
	  else \
	      cp -p ${SROS_ROOT}/config/sros_kernel_config ${KERNEL_BASE}/.config; \
	      make -C ${KERNEL_BASE} ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} oldconfig; \
          fi;

.PHONY: kernel_initramfs_defconfig
kernel_initramfs_defconfig:
	@ if [ -f ${KERNEL_BASE}/.config ]; then \
	      echo "Config Exists, Not Overwriting"; \
	  else \
	      cp -p ${SROS_ROOT}/config/sros_initramfs_config ${KERNEL_BASE}/.config; \
	      make -C ${KERNEL_BASE} ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} oldconfig; \
          fi;

.PHONY: kernel_xconfig
kernel_xconfig:
	make -C ${KERNEL_BASE} ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} xconfig

.PHONY: kernel_saveconfig
kernel_saveconfig:
	cp -p ${KERNEL_BASE}/.config ${SROS_ROOT}/config/sros_kernel_config 

.PHONY: kernel_initramfs_saveconfig
kernel_initramfs_saveconfig:
	cp -p ${KERNEL_BASE}/.config ${SROS_ROOT}/config/sros_initramfs_config

.PHONY: kernel_mrproper
kernel_mrproper:
	make -C ${KERNEL_BASE} V=1 ARCH=powerpc CROSS_COMPILE=${TOOLCHAIN_PREFIX} mrproper
	rm -f ${SROS_ROOT}/output/uImage
	rm -f ${SROS_ROOT}/output/yosemite.dtb

.PHONY: uboot
uboot:
	@ if [ -f ${OUTPUT_DIR}/fw_printenv ]; then \
		echo "U-Boot built"; \
	  else \
		make -C ${UBOOT_BASE} CROSS_COMPILE=${TOOLCHAIN_PREFIX} V=1 flyer3d_defconfig ;\
		make -C ${UBOOT_BASE} CROSS_COMPILE=${TOOLCHAIN_PREFIX} V=1  ;\
		cp -p ${UBOOT_BASE}/u-boot.bin ${SROS_ROOT}/output/u-boot.bin ;\
	  fi

.PHONY: uboot_env
uboot_env: uboot
	@ if [ -f ${OUTPUT_DIR}/fw_printenv ]; then \
		echo "U-Boot fw_printenv built"; \
	  else \
		make -C ${UBOOT_BASE} CROSS_COMPILE=${TOOLCHAIN_PREFIX} V=1 env;\
		cp -p ${UBOOT_BASE}/tools/env/fw_printenv output/;\
		cp -p ${UBOOT_BASE}/tools/env/fw_env.config output/;\
	  fi

.PHONY: uboot_saveconfig
uboot_saveconfig:
	cp -p ${UBOOT_BASE}/.config ${SROS_ROOT}/config/sros_uboot_config 

.PHONY: initramfs_final
initramfs_final: initramfs_final_clean uboot_env
	fakeroot bin/do_build_initramfs.sh

.PHONY: initramfs_final_clean
initramfs_final_clean:
	rm -rf ${STAGING_DIR}/*

.PHONY: help
help:
	@ less README_help.txt
