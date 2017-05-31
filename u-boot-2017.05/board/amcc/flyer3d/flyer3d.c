/*
 * (C) Copyright 2006-2007
 * Stefan Roese, DENX Software Engineering, sr@denx.de.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/ppc4xx.h>
#include <asm/processor.h>
#include <asm/io.h>
#include <spd_sdram.h>
#include <libfdt.h>
#include <status_led.h>
#include <asm/gpio.h>
#include <fdt_support.h>

#define BIT32(x)	   (1 << (31-x))
#define TWOBIT32(x)        ((x>16)?(3 << 2*(31-x)):(3 << 2*(15-x)))

DECLARE_GLOBAL_DATA_PTR;

extern flash_info_t flash_info[CONFIG_SYS_MAX_FLASH_BANKS]; /* info for FLASH chips	*/
extern void lxt971_no_sleep(void);

static inline u32 get_async_pci_freq(void)
{
	if (in_8((void *)(CONFIG_SYS_BCSR_BASE + 5)) &
		CONFIG_SYS_BCSR5_PCI66EN)
		return 66666666;
	else
		return 33333333;
}

#ifdef CONFIG_BOOT_SWITCH 
void initBootSwitch(void)
{
    //gpio_config(CONFIG_SWITCH_BIT, GPIO_IN, 0,0);
    //gpio_config(CONFIG_SWITCH_BIT1, GPIO_IN, 0,0);
    gpio_config(CONFIG_SWITCH_BIT2, GPIO_IN, 0,0);
    gpio_config(CONFIG_SWITCH_BIT3, GPIO_IN, 0,0);
}

int readBootSwitch(void)
{
    int ret = 0;
    ret = gpio_read_in_bit(CONFIG_SWITCH_BIT2);
    ret |= (gpio_read_in_bit(CONFIG_SWITCH_BIT3) << 1);
    /*
    ret = gpio_read_in_bit(CONFIG_SWITCH_BIT);
    ret |= (gpio_read_in_bit(CONFIG_SWITCH_BIT1) << 1);
    ret |= (gpio_read_in_bit(CONFIG_SWITCH_BIT2) << 3);
    ret |= (gpio_read_in_bit(CONFIG_SWITCH_BIT3) << 2);
    */
    
    return ret;
}

#endif
void __led_init (led_id_t mask, int state)
{
    
    int out_val;
    out_val = (state == STATUS_LED_ON)?GPIO_OUT_1:GPIO_OUT_0;
    gpio_config(mask, GPIO_OUT, 0,out_val);
}

void __led_set (led_id_t mask, int state)
{
    int out_val;
    out_val = (state == STATUS_LED_ON)?GPIO_OUT_1:GPIO_OUT_0;
    gpio_write_bit(mask, out_val);
}

void __led_toggle (led_id_t mask)
{
    if(gpio_read_out_bit(mask))
       gpio_write_bit(mask, GPIO_OUT_0);
    else
	gpio_write_bit(mask, GPIO_OUT_1);
    
    
}

void CheckFlashReady(void)
{
    int fl1 = 0;
    int fl2 = 0;
    gpio_config(CONFIG_FLASH_BIT, GPIO_IN, 0,0);
    gpio_config(CONFIG_FLASH_BIT1, GPIO_IN, 0,0);
    //printf("Checking Flash pins...\n");
    while (fl1==0 && fl2==0)
    {
	fl1 = gpio_read_in_bit(CONFIG_FLASH_BIT);
	fl2 = (gpio_read_in_bit(CONFIG_FLASH_BIT1) << 1);
    }
}


int board_early_init_f(void)
{
	register uint reg;

	/*--------------------------------------------------------------------
	 * Setup the external bus controller/chip selects
	 *-------------------------------------------------------------------*/
	mtdcr(EBC0_CFGADDR, EBC0_CFG);
	reg = mfdcr(EBC0_CFGDATA);
	mtdcr(EBC0_CFGDATA, reg | 0x04000000);	/* Set ATC */

	/*--------------------------------------------------------------------
	 * Setup the GPIO pins
	 *-------------------------------------------------------------------*/
	/*CPLD cs */
	/*setup Address lines for flash size 64Meg. */
	out32(GPIO0_OSRL, in32(GPIO0_OSRL) | 0x50010000);
	out32(GPIO0_TSRL, in32(GPIO0_TSRL) | 0x50010000);
	
	/* EBC (external bus controller) CS (chip select) Setup
	 * Set up GPIO6-8 for CS1-3 respectively (page 699 PPC 440EP embedded manual) 
	 * CS1:2x32MB memory, CS2:RTC, CS3:Xilinx */
	out32(GPIO0_OSRL, in32(GPIO0_OSRL) | 0x54000);
	out32(GPIO0_TSRL, in32(GPIO0_TSRL) | 0x54000);

	out32(GPIO0_ISR1L, in32(GPIO0_ISR1L) | 0x50000000);

	/*setup emac */
	out32(GPIO0_TCR, in32(GPIO0_TCR) | 0xC080);
	out32(GPIO0_TSRL, in32(GPIO0_TSRL) | 0x40);
	out32(GPIO0_ISR1L, in32(GPIO0_ISR1L) | 0x55);
	out32(GPIO0_OSRH, in32(GPIO0_OSRH) | 0x50004000);
	out32(GPIO0_ISR1H, in32(GPIO0_ISR1H) | 0x00440000);

	/*UART1 */
	out32(GPIO1_TCR, in32(GPIO1_TCR) | 0x02000000);
	out32(GPIO1_OSRL, in32(GPIO1_OSRL) | 0x00080000);
	out32(GPIO1_ISR2L, in32(GPIO1_ISR2L) | 0x00010000);

	/* external interrupts IRQ0...4 on GPIO 41-44
	 * (page 701 PPC 440EP embedded manual) */
	out32(GPIO1_TCR, in32(GPIO1_TCR) & ~0x00f80000);
	out32(GPIO1_TSRL, in32(GPIO1_TSRL) & ~0x0000ffc0);
	out32(GPIO1_ISR1L, in32(GPIO1_ISR1L) | 0x00005540);

#ifdef CONFIG_440EP
	/*setup USB 2.0 */
	out32(GPIO1_TCR, in32(GPIO1_TCR) | 0xc0000000);
	out32(GPIO1_OSRL, in32(GPIO1_OSRL) | 0x50000000);
	out32(GPIO0_TCR, in32(GPIO0_TCR) | 0xf);
	out32(GPIO0_OSRH, in32(GPIO0_OSRH) | 0xaa);
	out32(GPIO0_ISR2H, in32(GPIO0_ISR2H) | 0x00000500);
#endif

	/*--------------------------------------------------------------------
	 * Setup the interrupt controller polarities, triggers, etc.
	 *-------------------------------------------------------------------*/
	mtdcr(UIC0SR, 0xffffffff);	/* clear all */
	mtdcr(UIC0ER, 0x00000000);	/* disable all */
	/* IRQ5 is ETHIRQ interrupt, *level sensitive*, non-critical */
	mtdcr(UIC0CR, 0x00000001);	/* ATI & UIC1 crit are critical, non-critical pg 239 */
	mtdcr(UIC0PR, 0xfffffe13);	/* per ref-board manual */
	mtdcr(UIC0TR, 0x01c00000);	/* per ref-board manual, level sensitive pg 246 */
	mtdcr(UIC0VR, 0x00000001);	/* int31 highest, base=0x000 */
	mtdcr(UIC0SR, 0xffffffff);	/* clear all */

	mtdcr(UIC1SR, 0xffffffff);	/* clear all */
	mtdcr(UIC1ER, 0x00000000);	/* disable all */
	mtdcr(UIC1CR, 0x00000000);	/* all non-critical */
	mtdcr(UIC1PR, 0xffffe0ff);	/* per ref-board manual */
	mtdcr(UIC1TR, 0x00ffc000);	/* per ref-board manual */
	mtdcr(UIC1VR, 0x00000001);	/* int31 highest, base=0x000 */
	mtdcr(UIC1SR, 0xffffffff);	/* clear all */

	/*--------------------------------------------------------------------
	 * Setup other serial configuration
	 *-------------------------------------------------------------------*/
	mfsdr(SDR0_PCI0, reg);
	mtsdr(SDR0_PCI0, 0x80000000 | reg);	/* PCI arbiter enabled */
	mtsdr(SDR0_PFC0, 0x00003e00);	/* Pin function */
	mtsdr(SDR0_PFC1, 0x00048000);	/* Pin function: UART0 has 4 pins */

	/* Check and reconfigure the PCI sync clock if necessary */
	ppc4xx_pci_sync_clock_config(get_async_pci_freq());

	/* Why are these commented out? */
	/*clear tmrclk divisor */
	//*(unsigned char *)(CONFIG_SYS_BCSR_BASE | 0x04) = 0x00;
	/*enable ethernet */
	//*(unsigned char *)(CONFIG_SYS_BCSR_BASE | 0x08) = 0xf0;

#ifdef CONFIG_440EP
	/*enable usb 1.1 fs device and remove usb 2.0 reset */
	*(unsigned char *)(CONFIG_SYS_BCSR_BASE | 0x09) = 0x00;
#endif

	/*get rid of flash write protect */
	*(unsigned char *)(CONFIG_SYS_BCSR_BASE | 0x07) = 0x00;
	
	// Immediately drive the pullup line low so that USB device does not enumerate on powerup
	out32(GPIO1_OSRL, in32(GPIO1_OSRL) & ~ TWOBIT32(1));
	out32(GPIO1_TSRL, in32(GPIO1_TSRL) & ~ TWOBIT32(1));
	out32(GPIO1_ODR, in32(GPIO1_ODR) | BIT32(1));
	out32(GPIO1_TCR, in32(GPIO1_TCR) | BIT32(1));
	out32(GPIO1_OR, in32(GPIO1_OR) & ~ BIT32(1));
	

	return 0;
}

int misc_init_r (void)
{
	/* Why is this commented out?
	uint pbcr;
	int size_val = 0;

	// Re-do sizing to get full correct info
	mtdcr(EBC0_CFGADDR, PB0CR);
	pbcr = mfdcr(EBC0_CFGDATA);
	switch (gd->bd->bi_flashsize) {
	case 1 << 20:
		size_val = 0;
		break;
	case 2 << 20:
		size_val = 1;
		break;
	case 4 << 20:
		size_val = 2;
		break;
	case 8 << 20:
		size_val = 3;
		break;
	case 16 << 20:
		size_val = 4;
		break;
	case 32 << 20:
		size_val = 5;
		break;
	case 64 << 20:
		size_val = 6;
		break;
	case 128 << 20:
		size_val = 7;
		break;
	}
	pbcr = (pbcr & 0x0001ffff) | gd->bd->bi_flashstart | (size_val << 17);
	mtdcr(EBC0_CFGADDR, PB0CR);
	mtdcr(EBC0_CFGDATA, pbcr);
	*/
	/* adjust flash start and offset */
	gd->bd->bi_flashstart = 0 - gd->bd->bi_flashsize;
	gd->bd->bi_flashoffset = 0;

	/* Monitor protection OFF? by default
	 * Covers u-boot environment and u-boot executable? */
	(void)flash_protect(FLAG_PROTECT_CLEAR,
			   0xfff60000,
			    0xffffffff,
			    &flash_info[0]);

	return 0;
}

int checkboard(void)
{
	char buf[64];
	int i = getenv_f("serial#", buf, sizeof(buf));
	u8 rev;
	u32 clock = get_async_pci_freq();

#ifdef CONFIG_440EP
	printf("Board: Flyer3D - AMCC PPC440EP Marking Head");
#else
	printf("Board: Yellowstone - AMCC PPC440GR Evaluation Board");
#endif

	rev = in_8((void *)(CONFIG_SYS_BCSR_BASE + 0));
	printf(", Rev. %X, PCI-Async=%d MHz", rev, clock / 1000000);

	if (i > 0) {
		puts(", serial# ");
		puts(buf);
	}
	putc('\n');

	/*
	 * Reconfiguration of the PCI sync clock is already done,
	 * now check again if everything is in range:
	 */
	if (ppc4xx_pci_sync_clock_config(clock)) {
		printf("ERROR: PCI clocking incorrect (async=%d "
		       "sync=%ld)!\n", clock, get_PCI_freq());
	}

	return (0);
}

/*************************************************************************
 *  dram_init -- doesn't use serial presence detect.
 *
 *  Assumes:    256 MB, ECC, non-registered
 *              PLB @ 133 MHz
 *
 ************************************************************************/
#define NUM_TRIES 64
#define NUM_READS 10

void sdram_tr1_set(int ram_address, int* tr1_value)
{
	int i;
	int j, k;
	volatile unsigned int* ram_pointer =  (unsigned int*)ram_address;
	int first_good = -1, last_bad = 0x1ff;

	unsigned long test[NUM_TRIES] = {
		0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF,
		0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF,
		0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0x00000000,
		0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0x00000000,
		0xAAAAAAAA, 0xAAAAAAAA, 0x55555555, 0x55555555,
		0xAAAAAAAA, 0xAAAAAAAA, 0x55555555, 0x55555555,
		0x55555555, 0x55555555, 0xAAAAAAAA, 0xAAAAAAAA,
		0x55555555, 0x55555555, 0xAAAAAAAA, 0xAAAAAAAA,
		0xA5A5A5A5, 0xA5A5A5A5, 0x5A5A5A5A, 0x5A5A5A5A,
		0xA5A5A5A5, 0xA5A5A5A5, 0x5A5A5A5A, 0x5A5A5A5A,
		0x5A5A5A5A, 0x5A5A5A5A, 0xA5A5A5A5, 0xA5A5A5A5,
		0x5A5A5A5A, 0x5A5A5A5A, 0xA5A5A5A5, 0xA5A5A5A5,
		0xAA55AA55, 0xAA55AA55, 0x55AA55AA, 0x55AA55AA,
		0xAA55AA55, 0xAA55AA55, 0x55AA55AA, 0x55AA55AA,
		0x55AA55AA, 0x55AA55AA, 0xAA55AA55, 0xAA55AA55,
		0x55AA55AA, 0x55AA55AA, 0xAA55AA55, 0xAA55AA55 };

	/* go through all possible SDRAM0_TR1[RDCT] values */
	for (i=0; i<=0x1ff; i++) {
		/* set the current value for TR1 */
		mtsdram(SDRAM0_TR1, (0x80800800 | i));

		/* write values */
		for (j=0; j<NUM_TRIES; j++) {
			ram_pointer[j] = test[j];

			/* clear any cache at ram location */
			__asm__("dcbf 0,%0": :"r" (&ram_pointer[j]));
		}

		/* read values back */
		for (j=0; j<NUM_TRIES; j++) {
			for (k=0; k<NUM_READS; k++) {
				/* clear any cache at ram location */
				__asm__("dcbf 0,%0": :"r" (&ram_pointer[j]));

				if (ram_pointer[j] != test[j])
					break;
			}

			/* read error */
			if (k != NUM_READS) {
				break;
			}
		}

		/* we have a SDRAM0_TR1[RDCT] that is part of the window */
		if (j == NUM_TRIES) {
			if (first_good == -1)
				first_good = i;		/* found beginning of window */
		} else { /* bad read */
			/* if we have not had a good read then don't care */
			if(first_good != -1) {
				/* first failure after a good read */
				last_bad = i-1;
				break;
			}
		}
	}

	/* return the current value for TR1 */
	*tr1_value = (first_good + last_bad) / 2;
}

int dram_init(void)
{
	register uint reg;
	int tr1_bank1, tr1_bank2;

	/*--------------------------------------------------------------------
	 * Setup some default
	 *------------------------------------------------------------------*/
	mtsdram(SDRAM0_UABBA, 0x00000000);	/* ubba=0 (default)             */
	mtsdram(SDRAM0_SLIO, 0x00000000);	/* rdre=0 wrre=0 rarw=0         */
	mtsdram(SDRAM0_DEVOPT, 0x00000000);	/* dll=0 ds=0 (normal)          */
	mtsdram(SDRAM0_CLKTR, 0x40000000);	/* ?? */
	mtsdram(SDRAM0_WDDCTR, 0x40000000);	/* ?? */

	/*clear this first, if the DDR is enabled by a debugger
	  then you can not make changes. */
	mtsdram(SDRAM0_CFG0, 0x00000000);	/* Disable EEC */

	/*--------------------------------------------------------------------
	 * Setup for board-specific specific mem
	 *------------------------------------------------------------------*/
	/*
	 * Following for CAS Latency = 2.5 @ 133 MHz PLB
	 */
	mtsdram(SDRAM0_B0CR, 0x00084001);	/* SDBA=0x000 64MB, Mode 3, enabled, pg 325 */

	mtsdram(SDRAM0_TR0, 0x410a4012);	/* ?? */
	mtsdram(SDRAM0_RTR, 0x04080000);	/* ?? */
	mtsdram(SDRAM0_CFG1, 0x00000000);	/* Self-refresh exit, disable PM    */
	mtsdram(SDRAM0_CFG0, 0x30000000);	/* Disable EEC */
	udelay(800);		/* Delay 200 usecs (min)            */

	/*--------------------------------------------------------------------
	 * Enable the controller, then wait for DCEN to complete
	 *------------------------------------------------------------------*/
	mtsdram(SDRAM0_CFG0, 0x80000000);	/* Enable */

	for (;;) {
		mfsdram(SDRAM0_MCSTS, reg);
		if (reg & 0x80000000)
			break;
	}

	sdram_tr1_set(0x00000000, &tr1_bank1);
	mtsdram(SDRAM0_TR1, tr1_bank1 | 0x80800800);

	gd->ram_size = CONFIG_SYS_SDRAM_BANKS *
		(CONFIG_SYS_KBYTES_SDRAM * 1024);	/* set bytes */

	return 0;
}

/*************************************************************************
 *  hw_watchdog_reset
 *
 *	This routine is called to reset (keep alive) the watchdog timer
 *
 ************************************************************************/
#if defined(CONFIG_HW_WATCHDOG)
void hw_watchdog_reset(void)
{

}
#endif

void reset_phy(void)
{
#ifdef CONFIG_LXT971_NO_SLEEP
	/*
	 * Disable sleep mode in LXT971
	 */
	lxt971_no_sleep();
#endif
}

void board_reset(void)
{
	/* give reset to BCSR */
	// Why is this commented out?
	//*(unsigned char *)(CONFIG_SYS_BCSR_BASE | 0x06) = 0x09;
}