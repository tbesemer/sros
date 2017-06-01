/*
 * (C) Copyright 2005-2007
 * Stefan Roese, DENX Software Engineering, sr@denx.de.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/************************************************************************
 * flyer3d.h - flyer 3D configuration
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
#define CONFIG_SYS_CLK_FREQ	66666666    /* external freq to pll	*/
#define CONFIG_SYS_PLL_RECONFIG 667         /* reconfigure pll to run at 533 MHz */

#define	CONFIG_SYS_TEXT_BASE	0xFFF80000

/*
 * Include common defines/options for all AMCC eval boards
 */
#include "amcc-common.h"
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
#define CONFIG_SYS_INIT_RAM_SIZE	(4 << 10)
#define CONFIG_SYS_GBL_DATA_OFFSET	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_OFFSET	CONFIG_SYS_GBL_DATA_OFFSET

/*-----------------------------------------------------------------------
 * Serial Port
 *----------------------------------------------------------------------*/
#define CONFIG_CONS_INDEX	1	/* Use UART0			*/
#define CONFIG_SYS_EXT_SERIAL_CLOCK	11059200 /* use external 11.059MHz clk	*/

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

/*#define CONFIG_SYS_FLASH_ERASE_TOUT	120000	* Timeout for Flash Erase (in ms)	*/
/*#define CONFIG_SYS_FLASH_WRITE_TOUT	500	* Timeout for Flash Write (in ms)	*/

#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE 1	/* use buffered writes (20x faster)	*/
#define CONFIG_SYS_FLASH_PROTECTION	1	/* use hardware flash protection	*/

#define CONFIG_SYS_FLASH_EMPTY_INFO		/* print 'E' for empty sector on flinfo */

#ifdef CONFIG_ENV_IS_IN_FLASH
#define CONFIG_ENV_SECT_SIZE	0x20000	/* size of one complete sector		*/
#define CONFIG_ENV_ADDR		(CONFIG_SYS_MONITOR_BASE-CONFIG_ENV_SECT_SIZE)
#define	CONFIG_ENV_SIZE		0x2000	/* Total Size of Environment Sector	*/

/* Address and size of Redundant Environment Sector	*/
/*
#define CONFIG_ENV_ADDR_REDUND	(CONFIG_ENV_ADDR-CONFIG_ENV_SECT_SIZE)
#define CONFIG_ENV_SIZE_REDUND	(CONFIG_ENV_SIZE)
*/
#endif /* CONFIG_ENV_IS_IN_FLASH */

/*-----------------------------------------------------------------------
 * DDR SDRAM
 *----------------------------------------------------------------------*/
#undef CONFIG_SPD_EEPROM	       /* Don't use SPD EEPROM for setup    */
#define CONFIG_SYS_KBYTES_SDRAM        (64 * 1024)    /* 64MB		    */
#define CONFIG_SYS_SDRAM_BANKS	        (1)

/*-----------------------------------------------------------------------
 * I2C
 *----------------------------------------------------------------------*/
#define CONFIG_SYS_I2C_PPC4XX_SPEED_0		400000

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
/*
#define CONFIG_DTT_LM75		1		* ON Semi's LM75	
#define CONFIG_DTT_AD7414	1		* use AD7414		
#define CONFIG_DTT_SENSORS	{0}		* Sensor addresses	
#define CONFIG_SYS_DTT_MAX_TEMP	70
#define CONFIG_SYS_DTT_LOW_TEMP	-30
#define CONFIG_SYS_DTT_HYSTERESIS	3
*/
/*
 * Default environment variables
 */
#define	CONFIG_EXTRA_ENV_SETTINGS					\
	CONFIG_AMCC_DEF_ENV						\
	CONFIG_AMCC_DEF_ENV_POWERPC					\
	CONFIG_AMCC_DEF_ENV_PPC_OLD					\
	CONFIG_AMCC_DEF_ENV_NOR_UPD					\
	"kernel_addr=fc000000\0"					\
	"ramdisk_addr=fc180000\0"					\
	""

#define CONFIG_HAS_ETH0		1	/* add support for "ethaddr"	*/
#define CONFIG_RESET_PHY_R	1	/* use reset_phy() this enables LXT971 no sleep and sets up LXT971 LEDs*/
#define CONFIG_PHY_ADDR		0	/* PHY address config for port 0			*/
#define CONFIG_LXT971_NO_SLEEP  1       /* disable sleep mode in LXT971 */

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

#ifdef CONFIG_440EP
/* USB */
#define CONFIG_USB_OHCI_NEW
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
/*
#define CONFIG_CMD_DTT
#define CONFIG_CMD_PCI
*/

/*-----------------------------------------------------------------------
 * PCI stuff
 *-----------------------------------------------------------------------
 */
/* General PCI */
#undef  CONFIG_PCI_INDIRECT_BRIDGE	/* indirect PCI bridge support */
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
#define CONFIG_SYS_CPLD		0xfb000000
#define CONFIG_SYS_RTC		0xfb100000

/* Memory Bank 0 (NOR-FLASH) initialization					*/
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
