SROS_ROOT := $(shell pwd)

UBOOT_BASE := ${SROS_ROOT}/u-boot3D

.PHONY: default
default: uboot_env

uboot:
	make -C ${UBOOT_BASE}  V=1 flyer3d_config
	make -C ${UBOOT_BASE}  V=1 

uboot_no_config:
	make -C ${UBOOT_BASE}  V=1 


.PHONY: uboot_env
uboot_env: uboot
	make -C ${UBOOT_BASE}  V=1 env
	cp -p ${UBOOT_BASE}/tools/env/fw_printenv output/

uboot_env_clean:
	make -C ${UBOOT_BASE} V=1 env clean

clean:
	make -C ${UBOOT_BASE} V=1 clean
	rm -f output/*

mrproper:
	make -C ${UBOOT_BASE} V=1 mrproper
	rm -f output/*
	


