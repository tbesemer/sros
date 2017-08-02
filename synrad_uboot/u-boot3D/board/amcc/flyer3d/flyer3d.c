/*
 * (C) Copyright 2006-2007
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

#include <common.h>
#include <ppc4xx.h>
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
	mtdcr(ebccfga, xbcfg);
	reg = mfdcr(ebccfgd);
	mtdcr(ebccfgd, reg | 0x04000000);	/* Set ATC */

	

	/*--------------------------------------------------------------------
	 * Setup the GPIO pins
	 *-------------------------------------------------------------------*/
	/*CPLD cs */
	/*setup Address lines for flash size 64Meg. */
	out32(GPIO0_OSRL, in32(GPIO0_OSRL) | 0x50010000);
	out32(GPIO0_TSRL, in32(GPIO0_TSRL) | 0x50010000);
	
	// EBC CS 1 Setup
	out32(GPIO0_OSRL, in32(GPIO0_OSRL) | 0x54000);
	out32(GPIO0_TSRL, in32(GPIO0_TSRL) | 0x54000);
	//out32(GPIO0_OSRL, in32(GPIO0_OSRL) | 0x4000);
	//out32(GPIO0_TSRL, in32(GPIO0_TSRL) | 0x4000);

	out32(GPIO0_ISR1L, in32(GPIO0_ISR1L) | 0x50000000);
	//gpio_config(6, GPIO_OUT, GPIO_ALT1_SEL,GPIO_OUT_1);

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

	/* external interrupts IRQ0...3 */
	//out32(GPIO1_TCR, in32(GPIO1_TCR) & ~0x00f00000);
	//out32(GPIO1_TSRL, in32(GPIO1_TSRL) & ~0x0000ff00);
	//out32(GPIO1_ISR1L, in32(GPIO1_ISR1L) | 0x00005500);

/* external interrupts IRQ0...3 */
	/*
	// These are the old settings (04-Apr-2011) I need to activate IRQ4 for Xilinx interrupts 
	out32(GPIO1_TCR, in32(GPIO1_TCR) & ~0x00f00000);
	out32(GPIO1_TSRL, in32(GPIO1_TSRL) & ~0x0000f000);
	out32(GPIO1_ISR1L, in32(GPIO1_ISR1L) | 0x00005000);
	*/
	
	out32(GPIO1_TCR, in32(GPIO1_TCR) & ~0x00f80000);
	out32(GPIO1_TSRL, in32(GPIO1_TSRL) & ~0x0000ffc0);
	out32(GPIO1_ISR1L, in32(GPIO1_ISR1L) | 0x00005540);
	
	// Immediately drive the pullup line low so that USB device does not enumerate on powerup
	/*
	out32(GPIO1_OSRL, in32(GPIO1_OSRL) & ~ TWOBIT32(1));
	out32(GPIO1_TSRL, in32(GPIO1_TSRL) & ~ TWOBIT32(1));
	out32(GPIO1_ODR, in32(GPIO1_ODR) & ~ BIT32(1));
	out32(GPIO1_TCR, in32(GPIO1_TCR) | BIT32(1));
	out32(GPIO1_OR, in32(GPIO1_OR) & ~ BIT32(1));
	*/
	

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
	mtdcr(uic0sr, 0xffffffff);	/* clear all */
	mtdcr(uic0er, 0x00000000);	/* disable all */
	mtdcr(uic0cr, 0x00000001);	/* ATI & UIC1 crit are critical */	// sbs IRQ 5 is not critical
	mtdcr(uic0pr, 0xfffffe13);	/* per ref-board manual */
	mtdcr(uic0tr, 0x01c00000);	/* per ref-board manual */		// sbs IRQ 5 is level sensitive
	mtdcr(uic0vr, 0x00000001);	/* int31 highest, base=0x000 */
	mtdcr(uic0sr, 0xffffffff);	/* clear all */

	mtdcr(uic1sr, 0xffffffff);	/* clear all */
	mtdcr(uic1er, 0x00000000);	/* disable all */
	mtdcr(uic1cr, 0x00000000);	/* all non-critical */
	mtdcr(uic1pr, 0xffffe0ff);	/* per ref-board manual */
	mtdcr(uic1tr, 0x00ffc000);	/* per ref-board manual */
	mtdcr(uic1vr, 0x00000001);	/* int31 highest, base=0x000 */
	mtdcr(uic1sr, 0xffffffff);	/* clear all */

	/*--------------------------------------------------------------------
	 * Setup other serial configuration
	 *-------------------------------------------------------------------*/
	mfsdr(sdr_pci0, reg);
	mtsdr(sdr_pci0, 0x80000000 | reg);	/* PCI arbiter enabled */
	mtsdr(sdr_pfc0, 0x00003e00);	/* Pin function */
	mtsdr(sdr_pfc1, 0x00048000);	/* Pin function: UART0 has 4 pins */
	

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
	/*
	uint pbcr;
	int size_val = 0;
	//gpio_config(6, GPIO_OUT, GPIO_ALT1_SEL,GPIO_OUT_1);
	// Re-do sizing to get full correct info 
	mtdcr(ebccfga, pb0cr);
	pbcr = mfdcr(ebccfgd);
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
	pbcr = (pbcr & 0x0001ffff) | CONFIG_SYS_FLASH | (size_val << 17);
	mtdcr(ebccfga, pb0cr);
	mtdcr(ebccfgd, pbcr);
	//pbcr = (pbcr & 0x0001ffff) | CONFIG_SYS_FLASH2 | (size_val << 17);
	//mtdcr(ebccfga, pb1cr);
	//mtdcr(ebccfgd, pbcr);

    	*/
	/* adjust flash start and offset */
	gd->bd->bi_flashstart = 0 - gd->bd->bi_flashsize;
	gd->bd->bi_flashoffset = 0;

	/* Monitor protection ON by default */
	/*
	(void)flash_protect(FLAG_PROTECT_SET,
			    -CONFIG_SYS_MONITOR_LEN,
			    0xffffffff,
			    &flash_info[1]);
	*/
(void)flash_protect(FLAG_PROTECT_CLEAR,
			   0xfff60000,
			    0xffffffff,
			    &flash_info[0]);
	
	return 0;
}

int checkboard(void)
{
        
	char *s = getenv("serial#");
	u8 rev;
	u8 val;
	//delay(1000);
#ifdef CONFIG_440EP
	/*
	int	reset_needed = 0;
	u32	reg, temp;
	u32	prbdv0, target_prbdv0,			// CLK_PRIMBD 
	fwdva, target_fwdva, fwdvb, target_fwdvb,	// CLK_PLLD 
	fbdv, target_fbdv, lfbdv, target_lfbdv,
	perdv0,	target_perdv0,				// CLK_PERD 
	//spcid0,	target_spcid0,			// CLK_SPCID 
	opbdv0,     target_opbdv0,                      // CLK_OPB 
	    //pllc0,      target_pllc0,
	    //rst,        target_rst,
	pradv0, target_pradv0,
	eng,        target_eng,
	src,        target_src,
	sel,        target_sel,
	tune,       target_tune;
	*/
	printf("Board: Flyer3D - AMCC PPC440EP Marking Head");
	//printf("Env Address: 0x%8x\n",CONFIG_ENV_ADDR);
#else
	printf("Board: Yellowstone - AMCC PPC440GR Evaluation Board");
	
#endif
	
	int n = 0;
	int i ;
	rev = in_8((void *)(CONFIG_SYS_BCSR_BASE + 0));
	val = in_8((void *)(CONFIG_SYS_BCSR_BASE + 5)) & CONFIG_SYS_BCSR5_PCI66EN;
	printf(", Rev. %X, PCI=%d MHz", rev, val ? 66 : 33);

	if (s != NULL) {
		puts(", serial# ");
		puts(s);
	}
	putc('\n');
	/*
	printf("************** PLL Settings ****************\n");	
	mfcpr(clk_pllc, reg);
	printf(" --- PLLC Reg: 0x%8x ---\n",reg);
	tune = (reg & PLLC_TUNE_MASK);
	sel =  (reg & PLLC_SEL_MASK) >> 24;
	src =  (reg & PLLC_SRC_MASK) >> 29;
	eng =  (reg & PLLC_ENG_MASK) >> 30;
	printf("tune = %u  sel = %u  src = %u  eng = %u\n\n",tune,sel,src,eng);
	
	mfcpr(clk_plld, reg);
	printf(" --- PLLD Reg: 0x%8x ---\n",reg);
	fwdva = (reg & PLLD_FWDVA_MASK) >> 16;
	fwdvb = (reg & PLLD_FWDVB_MASK) >> 8;	
	fbdv = (reg & PLLD_FBDV_MASK) >> 24;	
	lfbdv = (reg & PLLD_LFBDV_MASK);
	printf("fwdva = %u  fwdvb = %u  fbdv = %u  lfbdv = %u\n\n",fwdva,fwdvb,fbdv,lfbdv);
	
	mfcpr(clk_opbd, reg);
	printf(" --- OPBD Reg: 0x%8x ---\n",reg);
	opbdv0 = (reg & OPBDDV_MASK) >> 24;
	printf("opbdv0 = %u\n\n",opbdv0);
	
	mfcpr(clk_primad, reg);
	printf(" --- PRIMA Reg: 0x%8x ---\n",reg);
	pradv0 = (reg & PRADV_MASK) >> 24;
	printf("pradv0 = %u\n\n",pradv0);
	
	mfcpr(clk_primbd, reg);
	printf(" --- PRIMB Reg: 0x%8x ---\n",reg);
	prbdv0 = (reg & PRBDV_MASK) >> 24;
	printf("prbdv0 = %u\n\n",prbdv0);
	
	mfcpr(clk_perd, reg);
	printf(" --- PERD Reg: 0x%8x ---\n",reg);
	perdv0 = (reg & PERDV_MASK) >> 24;
	printf("perdv0 = %u\n\n",perdv0);
	
	printf("CPU:PLB %u:%u\n", (fwdvb*prbdv0),(fwdva*pradv0));
	
	target_prbdv0 = 2;
	target_pradv0 = 1;
	target_fwdva = 2;
	target_fwdvb = 5;
	target_fbdv = 5;
	target_lfbdv = 2;
	target_perdv0 = 4;
	//target_spcid0 = 4;
	target_opbdv0 = 2;
	target_eng = 1;
	target_src = 0;
	target_sel = 0;
	target_tune = 0x2B8;
	//target_tune = 310;
	
	mfcpr(clk_pllc, reg);
	tune = (reg & PLLC_TUNE_MASK);
	sel =  (reg & PLLC_SEL_MASK) >> 24;
	src =  (reg & PLLC_SRC_MASK) >> 29;
	eng =  (reg & PLLC_ENG_MASK) >> 30;
	
	if (tune != target_tune || sel != target_sel || src != target_src || eng != target_eng)
	{
	    reg &= ~(PLLC_TUNE_MASK | PLLC_SEL_MASK |
		    PLLC_SRC_MASK | PLLC_ENG_MASK);
	    reg |= (target_eng << 30) |
		    (target_src << 29) |
		    (target_sel << 24) |
		    target_tune;
	    mtcpr(clk_pllc, reg);
	    printf("clk_pllc reset needed\n");
	    reset_needed = 1;
	}
	
	mfcpr(clk_primad, reg);
	pradv0 = (reg & PRADV_MASK) >> 24;
	if (pradv0 != target_pradv0) {
	    reg &= ~PRADV_MASK;
	    reg |= (target_pradv0 << 24);
	    mtcpr(clk_primad, reg);
	    printf("cclk_primad reset needed\n");
	    reset_needed = 1;
	}
	    
	mfcpr(clk_primbd, reg);
	prbdv0 = (reg & PRBDV_MASK) >> 24;
	if (prbdv0 != target_prbdv0) {
	    reg &= ~PRBDV_MASK;
	    reg |= (target_prbdv0 << 24);
	    mtcpr(clk_primbd, reg);
	    printf("cclk_primbd reset needed\n");
	    reset_needed = 1;
	}

	mfcpr(clk_plld, reg);

	fwdva = (reg & PLLD_FWDVA_MASK) >> 16;
	fwdvb = (reg & PLLD_FWDVB_MASK) >> 8;
	fbdv = (reg & PLLD_FBDV_MASK) >> 24;
	lfbdv = (reg & PLLD_LFBDV_MASK);
	
	if (fwdva != target_fwdva || fbdv != target_fbdv || lfbdv != target_lfbdv) {
	    reg &= ~(PLLD_FWDVA_MASK | PLLD_FWDVB_MASK |
		    PLLD_FBDV_MASK | PLLD_LFBDV_MASK);
	    reg |= (target_fwdva << 16) |
		    (target_fwdvb << 8) |
		    (target_fbdv << 24) |
		    target_lfbdv;
	    mtcpr(clk_plld, reg);
	    printf("clk_plld reset needed\n");
	    reset_needed = 1;
	}
	
	mfcpr(clk_opbd, reg);	
	opbdv0 = (reg & OPBDDV_MASK) >> 24;
	if (opbdv0 != target_opbdv0) {
	    reg &= ~OPBDDV_MASK;
	    reg |= (target_opbdv0 << 24);
	    mtcpr(clk_opbd, reg);
	    printf("clk_opbd reset needed\n");
	    reset_needed = 1;
	}
	
	mfcpr(clk_perd, reg);
	perdv0 = (reg & PERDV_MASK) >> 24;
	if (perdv0 != target_perdv0) {
	    reg &= ~PERDV_MASK;
	    reg |= (target_perdv0 << 24);
	    mtcpr(clk_perd, reg);
	    printf("clk_perd reset needed\n");
	    reset_needed = 1;
	}
	
	if (reset_needed) 
	{
	// Set reload inhibit so configuration will persist across
	// processor resets 
	    mfcpr(clk_icfg, reg);
	    reg &= ~CPR0_ICFG_RLI_MASK;
	    reg |= 1 << 31;
	    mtcpr(clk_icfg, reg);
	
	    __asm__ __volatile__ ("sync; isync");
	    mtspr (SPRN_DBCR0, 0x20000000);
	}
	*/
	
	
	
	
	
	//mtspr (SPRN_DBCR0, 0x20000000);
	return (0);
}

/*************************************************************************
 *  initdram -- doesn't use serial presence detect.
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
		mtsdram(mem_tr1, (0x80800800 | i));

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

phys_size_t initdram(int board)
{
	register uint reg;
	int tr1_bank1, tr1_bank2;
	//udelay(1000);
	/*--------------------------------------------------------------------
	 * Setup some default
	 *------------------------------------------------------------------*/
	
	mtsdram(mem_uabba, 0x00000000);	/* ubba=0 (default)             */
	mtsdram(mem_slio, 0x00000000);	/* rdre=0 wrre=0 rarw=0         */
	mtsdram(mem_devopt, 0x00000000);	/* dll=0 ds=0 (normal)          */
	mtsdram(mem_clktr, 0x40000000);	/* ?? */
	mtsdram(mem_wddctr, 0x40000000);	/* ?? */

	/*clear this first, if the DDR is enabled by a debugger
	  then you can not make changes. */
	mtsdram(mem_cfg0, 0x00000000);	/* Disable EEC */

	/*--------------------------------------------------------------------
	 * Setup for board-specific specific mem
	 *------------------------------------------------------------------*/
	/*
	 * Following for CAS Latency = 2.5 @ 133 MHz PLB
	 */
	mtsdram(mem_b0cr, 0x00084001);	/* SDBA=0x000 64MB, Mode 3, enabled */
	//mtsdram(mem_b0cr, 0x000a4001);	/* SDBA=0x000 128MB, Mode 3, enabled */
	//mtsdram(mem_b1cr, 0x00000000));	/* SDBA=0x080 128MB, Mode 3, enabled */

	mtsdram(mem_tr0, 0x410a4012);	/* ?? */
	mtsdram(mem_rtr, 0x04080000);	/* ?? */
	mtsdram(mem_cfg1, 0x00000000);	/* Self-refresh exit, disable PM    */
	mtsdram(mem_cfg0, 0x30000000);	/* Disable EEC */
	udelay(800);		/* Delay 200 usecs (min)            */

	/*--------------------------------------------------------------------
	 * Enable the controller, then wait for DCEN to complete
	 *------------------------------------------------------------------*/
	mtsdram(mem_cfg0, 0x80000000);	/* Enable */

	for (;;) {
		mfsdram(mem_mcsts, reg);
		if (reg & 0x80000000)
			break;
	}

	sdram_tr1_set(0x00000000, &tr1_bank1);
	//sdram_tr1_set(0x08000000, &tr1_bank2);
	//mtsdram(mem_tr1, (((tr1_bank1+tr1_bank2)/2) | 0x80800800));
	mtsdram(mem_tr1, (tr1_bank1|0x80800800) );
	//mtsdram(mem_tr1, 0x80800819);
	
	return CONFIG_SYS_SDRAM_BANKS * (CONFIG_SYS_KBYTES_SDRAM * 1024);	/* return bytes */
}

/*************************************************************************
 *  pci_pre_init
 *
 *  This routine is called just prior to registering the hose and gives
 *  the board the opportunity to check things. Returning a value of zero
 *  indicates that things are bad & PCI initialization should be aborted.
 *
 *	Different boards may wish to customize the pci controller structure
 *	(add regions, override default access routines, etc) or perform
 *	certain pre-initialization actions.
 *
 ************************************************************************/
#if defined(CONFIG_PCI)
int pci_pre_init(struct pci_controller *hose)
{
	unsigned long addr;

	/*-------------------------------------------------------------------------+
	  | Set priority for all PLB3 devices to 0.
	  | Set PLB3 arbiter to fair mode.
	  +-------------------------------------------------------------------------*/
	mfsdr(sdr_amp1, addr);
	mtsdr(sdr_amp1, (addr & 0x000000FF) | 0x0000FF00);
	addr = mfdcr(plb3_acr);
	mtdcr(plb3_acr, addr | 0x80000000);

	/*-------------------------------------------------------------------------+
	  | Set priority for all PLB4 devices to 0.
	  +-------------------------------------------------------------------------*/
	mfsdr(sdr_amp0, addr);
	mtsdr(sdr_amp0, (addr & 0x000000FF) | 0x0000FF00);
	addr = mfdcr(plb4_acr) | 0xa0000000;	/* Was 0x8---- */
	mtdcr(plb4_acr, addr);

	/*-------------------------------------------------------------------------+
	  | Set Nebula PLB4 arbiter to fair mode.
	  +-------------------------------------------------------------------------*/
	/* Segment0 */
	addr = (mfdcr(plb0_acr) & ~plb0_acr_ppm_mask) | plb0_acr_ppm_fair;
	addr = (addr & ~plb0_acr_hbu_mask) | plb0_acr_hbu_enabled;
	addr = (addr & ~plb0_acr_rdp_mask) | plb0_acr_rdp_4deep;
	addr = (addr & ~plb0_acr_wrp_mask) | plb0_acr_wrp_2deep;
	mtdcr(plb0_acr, addr);

	/* Segment1 */
	addr = (mfdcr(plb1_acr) & ~plb1_acr_ppm_mask) | plb1_acr_ppm_fair;
	addr = (addr & ~plb1_acr_hbu_mask) | plb1_acr_hbu_enabled;
	addr = (addr & ~plb1_acr_rdp_mask) | plb1_acr_rdp_4deep;
	addr = (addr & ~plb1_acr_wrp_mask) | plb1_acr_wrp_2deep;
	mtdcr(plb1_acr, addr);

	return 1;
}
#endif	/* defined(CONFIG_PCI) */

/*************************************************************************
 *  pci_target_init
 *
 *	The bootstrap configuration provides default settings for the pci
 *	inbound map (PIM). But the bootstrap config choices are limited and
 *	may not be sufficient for a given board.
 *
 ************************************************************************/
#if defined(CONFIG_PCI) && defined(CONFIG_SYS_PCI_TARGET_INIT)
void pci_target_init(struct pci_controller *hose)
{
	/*--------------------------------------------------------------------------+
	 * Set up Direct MMIO registers
	 *--------------------------------------------------------------------------*/
	/*--------------------------------------------------------------------------+
	  | PowerPC440 EP PCI Master configuration.
	  | Map one 1Gig range of PLB/processor addresses to PCI memory space.
	  |   PLB address 0xA0000000-0xDFFFFFFF ==> PCI address 0xA0000000-0xDFFFFFFF
	  |   Use byte reversed out routines to handle endianess.
	  | Make this region non-prefetchable.
	  +--------------------------------------------------------------------------*/
	out32r(PCIX0_PMM0MA, 0x00000000);	/* PMM0 Mask/Attribute - disabled b4 setting */
	out32r(PCIX0_PMM0LA, CONFIG_SYS_PCI_MEMBASE);	/* PMM0 Local Address */
	out32r(PCIX0_PMM0PCILA, CONFIG_SYS_PCI_MEMBASE);	/* PMM0 PCI Low Address */
	out32r(PCIX0_PMM0PCIHA, 0x00000000);	/* PMM0 PCI High Address */
	out32r(PCIX0_PMM0MA, 0xE0000001);	/* 512M + No prefetching, and enable region */

	out32r(PCIX0_PMM1MA, 0x00000000);	/* PMM0 Mask/Attribute - disabled b4 setting */
	out32r(PCIX0_PMM1LA, CONFIG_SYS_PCI_MEMBASE2);	/* PMM0 Local Address */
	out32r(PCIX0_PMM1PCILA, CONFIG_SYS_PCI_MEMBASE2);	/* PMM0 PCI Low Address */
	out32r(PCIX0_PMM1PCIHA, 0x00000000);	/* PMM0 PCI High Address */
	out32r(PCIX0_PMM1MA, 0xE0000001);	/* 512M + No prefetching, and enable region */

	out32r(PCIX0_PTM1MS, 0x00000001);	/* Memory Size/Attribute */
	out32r(PCIX0_PTM1LA, 0);	/* Local Addr. Reg */
	out32r(PCIX0_PTM2MS, 0);	/* Memory Size/Attribute */
	out32r(PCIX0_PTM2LA, 0);	/* Local Addr. Reg */

	/*--------------------------------------------------------------------------+
	 * Set up Configuration registers
	 *--------------------------------------------------------------------------*/

	/* Program the board's subsystem id/vendor id */
	pci_write_config_word(0, PCI_SUBSYSTEM_VENDOR_ID,
			      CONFIG_SYS_PCI_SUBSYS_VENDORID);
	pci_write_config_word(0, PCI_SUBSYSTEM_ID, CONFIG_SYS_PCI_SUBSYS_ID);

	/* Configure command register as bus master */
	pci_write_config_word(0, PCI_COMMAND, PCI_COMMAND_MASTER);

	/* 240nS PCI clock */
	pci_write_config_word(0, PCI_LATENCY_TIMER, 1);

	/* No error reporting */
	pci_write_config_word(0, PCI_ERREN, 0);

	pci_write_config_dword(0, PCI_BRDGOPT2, 0x00000101);

}
#endif				/* defined(CONFIG_PCI) && defined(CONFIG_SYS_PCI_TARGET_INIT) */

/*************************************************************************
 *  pci_master_init
 *
 ************************************************************************/
#if defined(CONFIG_PCI) && defined(CONFIG_SYS_PCI_MASTER_INIT)
void pci_master_init(struct pci_controller *hose)
{
	unsigned short temp_short;

	/*--------------------------------------------------------------------------+
	  | Write the PowerPC440 EP PCI Configuration regs.
	  |   Enable PowerPC440 EP to be a master on the PCI bus (PMM).
	  |   Enable PowerPC440 EP to act as a PCI memory target (PTM).
	  +--------------------------------------------------------------------------*/
	pci_read_config_word(0, PCI_COMMAND, &temp_short);
	pci_write_config_word(0, PCI_COMMAND,
			      temp_short | PCI_COMMAND_MASTER |
			      PCI_COMMAND_MEMORY);
}
#endif				/* defined(CONFIG_PCI) && defined(CONFIG_SYS_PCI_MASTER_INIT) */

/*************************************************************************
 *  is_pci_host
 *
 *	This routine is called to determine if a pci scan should be
 *	performed. With various hardware environments (especially cPCI and
 *	PPMC) it's insufficient to depend on the state of the arbiter enable
 *	bit in the strap register, or generic host/adapter assumptions.
 *
 *	Rather than hard-code a bad assumption in the general 440 code, the
 *	440 pci code requires the board to decide at runtime.
 *
 *	Return 0 for adapter mode, non-zero for host (monarch) mode.
 *
 *
 ************************************************************************/
#if defined(CONFIG_PCI)
int is_pci_host(struct pci_controller *hose)
{
	/* Bamboo is always configured as host. */
	return (1);
}
#endif				/* defined(CONFIG_PCI) */

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
	//*(unsigned char *)(CONFIG_SYS_BCSR_BASE | 0x06) = 0x09;
    
	
}
