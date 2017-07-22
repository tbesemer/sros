/*
 * (C) Copyright 2005-2007
 * Stefan Roese, DENX Software Engineering, sr@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/************************************************************************
 * flyer.h - configuration for Flyer/Mosaic boards
 ***********************************************************************/
#ifndef __CONFIG_H
#define __CONFIG_H

/*-----------------------------------------------------------------------
 * High Level Configuration Options
 *----------------------------------------------------------------------*/
/* This config file is used for Flyer (440EP)*/
#undef DEBUG
/*#define DEBUG*/
#define CONFIG_440EP		1	/* Specific PPC440EP support	*/
#define CONFIG_HOSTNAME		flyer3d
#define CONFIG_440		1	/* ... PPC440 family		*/
#define CONFIG_4xx		1	/* ... PPC4xx family		*/
#define CONFIG_SYS_CLK_FREQ	66666666    /* external freq to pll	*/
#define CONFIG_SYS_PLL_RECONFIG 667         /* reconfigure pll to run at 533 MHz */

/*
 * Include common defines/options for all AMCC eval boards
 */
#include "amcc-common.h"
#define CONFIG_BOOT_DELAY 1
#define CONFIG_BOARD_EARLY_INIT_F 1     /* Call board_early_init_f	*/
#define CONFIG_MISC_INIT_R	1	/* call misc_init_r()		*/
#define CONFIG_BOARD_RESET	1	/* call board_reset()		*/
#define CONFIG_BOOT_SWITCH	1	/* Check external switches for boot type*/

/*-----------------------------------------------------------------------
 * Base addresses -- Note these are effective addresses where the
 * actual resources get mapped (not physical addresses)
 *----------------------------------------------------------------------*/
#define CONFIG_SYS_FLASH_BASE	        0xfe000000	    /* start of FLASH	*/
#define CONFIG_SYS_PCI_MEMBASE	        0xa0000000	    /* mapped pci memory*/
#define CONFIG_SYS_PCI_MEMBASE1        CONFIG_SYS_PCI_MEMBASE  + 0x10000000
#define CONFIG_SYS_PCI_MEMBASE2        CONFIG_SYS_PCI_MEMBASE1 + 0x10000000
#define CONFIG_SYS_PCI_MEMBASE3        CONFIG_SYS_PCI_MEMBASE2 + 0x10000000

/*Don't change either of these*/
#define CONFIG_SYS_PERIPHERAL_BASE     0xef600000	    /* internal peripherals*/
#define CONFIG_SYS_PCI_BASE	        0xe0000000	    /* internal PCI regs*/
/*Don't change either of these*/

#define CONFIG_SYS_USB_DEVICE          0x50000000
#define CONFIG_SYS_NVRAM_BASE_ADDR     0x80000000
#define CONFIG_SYS_BCSR_BASE	        (CONFIG_SYS_NVRAM_BASE_ADDR | 0x2000)
#define CONFIG_SYS_BOOT_BASE_ADDR      0xf0000000

/*-----------------------------------------------------------------------
 * Initial RAM & stack pointer (placed in SDRAM)
 *----------------------------------------------------------------------*/
#define CONFIG_SYS_INIT_RAM_DCACHE	1		/* d-cache as init ram	*/
#define CONFIG_SYS_INIT_RAM_ADDR	0x70000000		/* DCache       */
#define CONFIG_SYS_INIT_RAM_END		(4 << 10)
#define CONFIG_SYS_GBL_DATA_SIZE	256			/* num bytes initial data*/
#define CONFIG_SYS_GBL_DATA_OFFSET	(CONFIG_SYS_INIT_RAM_END - CONFIG_SYS_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_OFFSET	CONFIG_SYS_GBL_DATA_OFFSET

/*-----------------------------------------------------------------------
 * Serial Port
 *----------------------------------------------------------------------*/
#define CONFIG_SYS_EXT_SERIAL_CLOCK	11059200 /* use external 11.059MHz clk	*/
/*define this if you want console on UART1*/
#undef CONFIG_UART1_CONSOLE

/*-----------------------------------------------------------------------
 * Environment
 *----------------------------------------------------------------------*/
/*
 * Define here the location of the environment variables (FLASH or EEPROM).
 * Note: DENX encourages to use redundant environment in FLASH.
 */
#if 1
#define CONFIG_ENV_IS_IN_FLASH     1	/* use FLASH for environment vars	*/
#else
#define CONFIG_ENV_IS_IN_EEPROM	1	/* use EEPROM for environment vars	*/
#endif

/*-----------------------------------------------------------------------
 * FLASH related
 *----------------------------------------------------------------------*/
#define CONFIG_SYS_FLASH_CFI				/* The flash is CFI compatible	*/
#define CONFIG_FLASH_CFI_DRIVER			/* Use common CFI driver	*/
#define CONFIG_SYS_FLASH_CFI_AMD_RESET 1		/* AMD RESET for STM 29W320DB!	*/
#define CONFIG_SYS_FLASH		CONFIG_SYS_FLASH_BASE
#define CONFIG_SYS_FLASH2		0xfc000000
#define CONFIG_SYS_FLASH_BANKS_LIST {CONFIG_SYS_FLASH, CONFIG_SYS_FLASH2}


#define CONFIG_SYS_MAX_FLASH_BANKS	2	/* max number of memory banks		*/
#define CONFIG_SYS_MAX_FLASH_SECT	256	/* max number of sectors on one chip	*/

//#define CONFIG_SYS_FLASH_ERASE_TOUT	4096	/* Timeout for Flash Erase (in ms)	*/
//#define CONFIG_SYS_FLASH_WRITE_TOUT	500	/* Timeout for Flash Write (in ms)	*/
//#define CONFIG_SYS_FLASH_ERASE_TOUT	1000	/* Timeout for Flash Erase (in ms)	*/
//#define CONFIG_SYS_FLASH_WRITE_TOUT	100	/* Timeout for Flash Write (in ms)	*/

#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE 1	/* use buffered writes (20x faster)	*/
#define CONFIG_SYS_FLASH_PROTECTION	1	/* use hardware flash protection	*/

#define CONFIG_SYS_FLASH_EMPTY_INFO		/* print 'E' for empty sector on flinfo */

#ifdef CONFIG_ENV_IS_IN_FLASH
#define CONFIG_ENV_SECT_SIZE	0x20000	/* size of one complete sector		*/
#define CONFIG_ENV_ADDR		(CONFIG_SYS_MONITOR_BASE-CONFIG_ENV_SECT_SIZE)
//#define CONFIG_ENV_ADDR		0xfff40000
#define	CONFIG_ENV_SIZE		0x2000	/* Total Size of Environment Sector	*/

/* Address and size of Redundant Environment Sector	*/
//#define CONFIG_ENV_ADDR_REDUND	(CONFIG_ENV_ADDR-CONFIG_ENV_SECT_SIZE)
//#define CONFIG_ENV_ADDR_REDUND	0xfff40000
//#define CONFIG_ENV_SIZE_REDUND	(CONFIG_ENV_SIZE)
#endif /* CONFIG_ENV_IS_IN_FLASH */

/*-----------------------------------------------------------------------
 * DDR SDRAM
 *----------------------------------------------------------------------*/
#undef CONFIG_SPD_EEPROM	       /* Don't use SPD EEPROM for setup    */
#define CONFIG_SYS_KBYTES_SDRAM        (64 * 1024)    /* 128MB		    */
#define CONFIG_SYS_SDRAM_BANKS	        (1)

/*-----------------------------------------------------------------------
 * I2C
 *----------------------------------------------------------------------*/
#define CONFIG_SYS_I2C_SPEED		400000	/* I2C speed and slave address	*/

#define CONFIG_SYS_I2C_MULTI_EEPROMS
#define CONFIG_SYS_I2C_EEPROM_ADDR	(0xa8>>1)
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN 1
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS 3
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS 10

#ifdef CONFIG_ENV_IS_IN_EEPROM
#define CONFIG_ENV_SIZE		0x200	    /* Size of Environment vars */
#define CONFIG_ENV_OFFSET		0x0
#endif /* CONFIG_ENV_IS_IN_EEPROM */

/* I2C SYSMON (LM75, AD7414 is almost compatible)			*/

//#define CONFIG_DTT_LM75		1		/* ON Semi's LM75	*/
//#define CONFIG_DTT_AD7414	1		/* use AD7414		*/
//#define CONFIG_DTT_SENSORS	{0}		/* Sensor addresses	*/
//#define CONFIG_SYS_DTT_MAX_TEMP	70
//#define CONFIG_SYS_DTT_LOW_TEMP	-30
//#define CONFIG_SYS_DTT_HYSTERESIS	3

/*
 * Default environment variables
 */
#define	CONFIG_EXTRA_ENV_SETTINGS					\
	CONFIG_AMCC_DEF_ENV						\
	CONFIG_AMCC_DEF_ENV_PPC						\
	CONFIG_AMCC_DEF_ENV_NOR_UPD					\
	"kernel_addr=fe000000\0"					\
	"ipaddr=192.168.90.44\0"					\
	"serverip=172.28.168.50\0"					\
	"ethaddr=00:10:ec:00:e2:ac\0"					\
"fileaddr=200000\0"                   \
"baudrate=115200\0"                     \
"loads_echo=\0"                     \
"hostname=flyer3D\0"                     \
"netdev=eth0\0"                     \
"nfsargs=setenv bootargs root=/dev/nfs rw nfsroot=${serverip}:${rootpath}\0"                     \
"ramargs=setenv bootargs root=/dev/ram rw\0"                     \
"addip=setenv bootargs ${bootargs} ip=${ipaddr}:${serverip}:${gatewayip}:${netmask}:${hostname}:${netdev}:off panic=1\0"                     \
"addtty=setenv bootargs ${bootargs} console=ttyS0,${baudrate}\0"                  \
"addmem=mem=64M\0"                     \
"rootpath=/home/mh1/tftpboot/rootfs\0"                     \
"flash_self=run ramargs addip addtty;bootm ${kernel_addr} ${ramdisk_addr}\0"                     \
"flash_nfs=run nfsargs addip addtty;bootm ${kernel_addr}\0"                     \
"net_nfs=tftp 200000 $\{bootfile\};run nfsargs addip addtty addmem;bootm $\{kernel_addr\}\0"                     \
"update=protect off 0xFFF80000 FFFFFFFF;era 0xFFF80000 FFFFFFFF;cp.b ${fileaddr} 0xFFF80000 ${filesize};setenv filesize;saveenv\0"                     \
"uboot_upd=run load update\0"                     \
"kernel_addr=fe000000\0"                     \
"flashargs=setenv bootargs root=/dev/mtdblock1 rw rootfstype=jffs2\0"		\
"ramdiskargs=setenv bootargs root=/dev/ram rw initrd=0x400000,6000000 ramdisk_size=15360\0"		\
"flyer=run flashargs addip addtty;bootm ${kernel_addr}\0"		\
"net_flash=tftp 200000 ${bootfile};run flashargs addip addtty;bootm\0"		\
"load_kernel=tftp 200000 /flyer3d/uImage\0"		\
"load_rootfs=tftp 200000 /flyer3d/flyer3d_flash.img\0"		\
"load_filestore=tftp 200000 /flyer3d/flyer3d_filestore.img\0"		\
"load_ramdisk=tftp 400000 /flyer3d/ramdisk\0"		\
"erase_kernel=era fe000000 fe17ffff\0"		\
"erase_rootfs=era fe180000 fff3ffff\0"		\
"erase_fs=era bank 2\0"		\
"save_kernel=cp.b ${fileaddr} fe000000 ${filesize}\0"		\
"save_rootfs=cp.b ${fileaddr} fe180000 ${filesize}\0"		\
"save_fs=cp.b ${fileaddr} fc000000 ${filesize}\0"		\
"flash_kernel=run load_kernel;run erase_kernel;run save_kernel\0"		\
"flash_rootfs=run load_rootfs;run erase_rootfs;run save_rootfs\0"		\
"flash_fs=run load_filestore;run erase_fs;run save_fs\0"		\
"prog_flyer_full=saveenv;run flash_kernel;run flash_rootfs;run flash_fs;run flyer\0"          \
"prog_flyer_field=run flash_kernel;run flash_rootfs;run flyer\0"          \
"prog_flyer_nofs=run flash_ubootenv;run flash_kernel;run flash_rootfs;run flyer\0"          \
"prog_uboot=run upd;run flash_ubootenv;run flyer\0"          \
"eth_boot=run load_ramdisk;run load_kernel;run ramdiskargs addip addtty;bootm 200000 400000\0"          \
"scalingX=512\0"          \
"scalingY=512\0"          \
"scalingZ=512\0"          \
"0zzB21x=-0.97023\0"          \
"1zzB11x=1.95741\0"          \
"2zzA2A01x=1.0000\0"          \
"3zzA1A01x=-1.98722\0"          \
"4zzA01x=1.00261\0"          \
"5zzB22x=-0.96285\0"          \
"6zzB12x=1.91849\0"          \
"7zzA2A02x=1.0000\0"          \
"8zzA1A02x=-1.95515\0"          \
"9zzA02x=0.98909\0"          \
"10zzB21y=-0.97023\0"          \
"11zzB11y=1.95706\0"          \
"12zzA2A01y=1.0000\0"          \
"13zzA1A01y=-1.98687\0"          \
"14zzA01y=1.00213\0"          \
"15zzB22y=-0.95991\0"          \
"16zzB12y=1.90536\0"          \
"17zzA2A02y=1.0000\0"          \
"18zzA1A02y=-1.94474\0"          \
"19zzA02y=0.98717\0"          \
"20zzOffx=0.0\0"          \
"21zzErrx=0.20000\0"          \
"22zzLFDGx=0.03700\0"          \
"23zzHFDGx=0.35300\0"          \
"24zzOffy=0.0\0"          \
"25zzErry=0.30000\0"         \
"26zzLFDGy=0.04800\0"          \
"27zzHFDGy=0.39600\0"          \
"28zzB21z=-0.96875\0"          \
"29zzB11z=1.95610\0"          \
"30zzA2A01z=1.00000\0"          \
"31zzA1A01z=-1.98740\0"          \
"32zzA01z=1.00393\0"          \
"33zzB22z=-0.96212\0"          \
"34zzB12z=1.91803\0"          \
"35zzA2A02z=1.00000\0"          \
"36zzA1A02z=-1.95542\0"          \
"37zzA02z=0.98908\0"          \
"38zzOffz=0.00000\0"          \
"39zzErrz=0.30000\0"          \
"40zzLFDGz=0.53000\0"          \
"41zzHFDGz=0.13000\0"          \
"42zzRBMMz=1.771654\0"          \
"43zzWDSTz=14.7187874\0"          \
"44zzDMMMz=1.484252\0"          \
"45zzLMMMz=3.543307\0"          \
"46zzFEMMz=-53.4980\0"          \
"47zzFFMMz=144.1700\0"          \
"48zzDZEROz=137.05701\0"          \
"49zzWAVEz=10.6\0" \
"filesize=2000\0" \
"serialnum=0000\0"          \
	""


#define CONFIG_HAS_ETH0		1	/* add support for "ethaddr"	*/
#define CONFIG_RESET_PHY_R	1	/* use reset_phy() this enables LXT971 no sleep and sets up LXT971 LEDs*/
#define CONFIG_PHY_ADDR		0	/* PHY address config for port 0			*/
#define CONFIG_LXT971_NO_SLEEP  1       /* disable sleep mode in LXT971 */
#define CONFIG_PHY_MII			// sbs setup PPC Eth bridge for MII rather than hard coded RMII

/*-----------------------------------------------------------------------
 * Definitions for boot switch
 */
#define CONFIG_SWITCH_BIT		3
#define CONFIG_SWITCH_BIT1              4
#define CONFIG_SWITCH_BIT2		38
#define CONFIG_SWITCH_BIT3              37
/*-----------------------------------------------------------------------
 * Definitions for flash check
 */
#define CONFIG_FLASH_BIT		9
#define CONFIG_FLASH_BIT1               10

/*-----------------------------------------------------------------------
 * Definitions for status LED
 */
#define CONFIG_SYS_HZ		        1000	/* decrementer freq: 1 ms ticks */
#define CONFIG_STATUS_LED	1	/* Status LED enabled		*/
#define CONFIG_BOARD_SPECIFIC_LED	1


#define STATUS_LED_RED			        0
#define STATUS_LED_GREEN			1
#define STATUS_USB_LED_RED			2
#define STATUS_USB_LED_GREEN			3

/*
#define STATUS_LED_RED			        	1
#define STATUS_LED_GREEN					0
#define STATUS_USB_LED_RED					3
#define STATUS_USB_LED_GREEN				2
*/


#define STATUS_LED_BIT		47			
#define STATUS_LED_PERIOD	((CONFIG_SYS_HZ / 2) / 5)	/* blink at 5 Hz */
#define STATUS_LED_STATE	STATUS_LED_OFF
#define STATUS_LED_BIT1		48		
#define STATUS_LED_PERIOD1	((CONFIG_SYS_HZ / 2) / 2)	/* blink at 5 Hz */
#define STATUS_LED_STATE1	STATUS_LED_OFF
#define STATUS_LED_BIT2		26			
#define STATUS_LED_PERIOD2	((CONFIG_SYS_HZ / 2) / 5)	/* blink at 5 Hz */
#define STATUS_LED_STATE2	STATUS_LED_OFF
#define STATUS_LED_BIT3		28		
#define STATUS_LED_PERIOD3	((CONFIG_SYS_HZ / 2) / 5)	/* blink at 5 Hz */
#define STATUS_LED_STATE3	STATUS_LED_OFF

#define STATUS_LED_BOOT			STATUS_LED_GREEN

/* Partitions */
#define CONFIG_MAC_PARTITION
#define CONFIG_DOS_PARTITION
#define CONFIG_ISO_PARTITION

#ifdef CONFIG_440EP
/* USB */
#define CONFIG_USB_OHCI_NEW
#define CONFIG_USB_STORAGE
#define CONFIG_SYS_OHCI_BE_CONTROLLER

#undef CONFIG_SYS_USB_OHCI_BOARD_INIT
#define CONFIG_SYS_USB_OHCI_CPU_INIT	1
#define CONFIG_SYS_USB_OHCI_REGS_BASE	(CONFIG_SYS_PERIPHERAL_BASE | 0x1000)
#define CONFIG_SYS_USB_OHCI_SLOT_NAME	"ppc440"
#define CONFIG_SYS_USB_OHCI_MAX_ROOT_PORTS	15

/* Comment this out to enable USB 1.1 device */
#define USB_2_0_DEVICE

#define CONFIG_SUPPORT_VFAT
#endif /* CONFIG_440EP */

#ifdef DEBUG
#define CONFIG_PANIC_HANG
#else
#define CONFIG_HW_WATCHDOG			/* watchdog */
#endif

/*
 * Commands additional to the ones defined in amcc-common.h
 */
//#define CONFIG_CMD_DTT
//#define CONFIG_CMD_PCI

#ifdef CONFIG_440EP
    #define CONFIG_CMD_MII
    #define CONFIG_CMD_USB
    #define CONFIG_CMD_FAT
    #define CONFIG_CMD_EXT2
#endif

/*-----------------------------------------------------------------------
 * PCI stuff
 *-----------------------------------------------------------------------
 */
/* General PCI */
#undef CONFIG_PCI
//#define CONFIG_PCI			/* include pci support	        */
#undef  CONFIG_PCI_PNP			/* do (not) pci plug-and-play   */
#define CONFIG_PCI_SCAN_SHOW            /* show pci devices on startup  */
#define CONFIG_SYS_PCI_TARGBASE        0x80000000 /* PCIaddr mapped to CONFIG_SYS_PCI_MEMBASE*/

/* Board-specific PCI */
#define CONFIG_SYS_PCI_TARGET_INIT
#define CONFIG_SYS_PCI_MASTER_INIT

#define CONFIG_SYS_PCI_SUBSYS_VENDORID 0x10e8	/* AMCC */
#define CONFIG_SYS_PCI_SUBSYS_ID       0xcafe	/* Whatever */

/*-----------------------------------------------------------------------
 * External Bus Controller (EBC) Setup
 *----------------------------------------------------------------------*/
//#define CONFIG_SYS_FLASH		CONFIG_SYS_FLASH_BASE
#define CONFIG_SYS_CPLD		0xfb000000
#define CONFIG_SYS_RTC		0xfb100000

/* Memory Bank 0 (NOR-FLASH) initialization					*/
//#define CONFIG_SYS_EBC_PB0AP		0x03017200
#define CONFIG_SYS_EBC_PB0AP		0x03017200
#define CONFIG_SYS_EBC_PB0CR		(CONFIG_SYS_FLASH | 0xba000)

/* Memory Bank 1 (NOR-FLASH) initialization					*/
#define CONFIG_SYS_EBC_PB1AP		0x03017200
#define CONFIG_SYS_EBC_PB1CR		(CONFIG_SYS_FLASH2 | 0xba000)

/* Memory Bank 2 (RTC/NVRAM) initialization						*/
#define CONFIG_SYS_EBC_PB2AP		0x02815480
#define CONFIG_SYS_EBC_PB2CR		(CONFIG_SYS_RTC | 0x18000)

/* Memory Bank 3 (XILINX) initialization						*/
#define CONFIG_SYS_EBC_PB3AP		0x800200
#define CONFIG_SYS_EBC_PB3CR		(CONFIG_SYS_CPLD | 0x1A000)

#define CONFIG_SYS_BCSR5_PCI66EN	0x80

#endif	/* __CONFIG_H */
