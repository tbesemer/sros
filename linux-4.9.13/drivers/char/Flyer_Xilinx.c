/***************************************************************************
 *   Copyright (C) 2005 by J. Warren                                       *
 *   jeffw@synrad.com                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/Flyer_Xilinx.h>
#include <asm/FlyerII.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/fs.h>

#define XILINX_VERSION "1.1"

#define BUFSIZE 4000
#define XIL_DONE_DELAY 30

#ifdef DEBUG
#define MSG(string, args...) printk(KERN_DEBUG "flyer_xil:" string, ##args)
#else
#define MSG(string, args...)
#endif

#define KEY_INTERRUPT		0x1
#define IO_INTERRUPT		0x2
#define TRACK_INTERRUPT		0x4
#define TEMP_INTERRUPTS  	0x78
#define TIMEOUT_INTERRUPT	0x80

#define TESTMARK_KEY 1

#define XILINX_ALIVE 1
#define XILINX_DEAD 0

#define XILINX_INIT_PIN            30
#define XILINX_DATA_IN_PIN         29
#define XILINX_DONE_PIN            11
#define XILINX_PROGRAM_PIN         31
#define XILINX_CLOCK_PIN           27
#define SWITCH_3_PIN               3
#define SWITCH_4_PIN               4
#define SWITCH_5_PIN               6
#define SWITCH_6_PIN               5
#define DSP_IRQ_LINE_PIN           5
#define ENCODER_PIN                8

#define XILINX_IRQ_PIN             12
#define XILINX_IRQ_LINE            BIT32(12)

#define XILINX_INIT            BIT32(30)
#define XILINX_DATA_IN         BIT32(29)
#define XILINX_DONE            BIT32(11)
#define XILINX_PROGRAM         BIT32(31)
#define XILINX_CLOCK           BIT32(27)
#define SWITCH_3               BIT32(3)
#define SWITCH_4               BIT32(4)
#define SWITCH_5               BIT32(6)
#define SWITCH_6               BIT32(5)
#define ENCODER                BIT32(8)
#define GPT_DECREMENT          BIT32(25)
#define UIC_CRITICAL           BIT32(30)


#define DSP_IRQ_LINE           BIT32(5)

#define XILINX_SLOW_CLOCK AT91C_PA24_PCK1
#define XILINX_FAST_CLOCK AT91C_PB27_PCK0

#define XILINX_ALL (XILINX_INIT | XILINX_DATA_IN | XILINX_DONE | XILINX_PROGRAM | XILINX_CLOCK)

#define FLYER_XILINX_BASE 0xfb000000

/*LED defines*/
/*
#define USB_RED_LINE           BIT32(28)
#define USB_GREEN_LINE         BIT32(26)
#define STAT_RED_LINE          BIT32(16)
#define STAT_GREEN_LINE        BIT32(15)
*/

#define USB_RED_LINE           BIT32(26)
#define USB_GREEN_LINE         BIT32(28)
#define STAT_RED_LINE          BIT32(15)
#define STAT_GREEN_LINE        BIT32(16)
#define MAX_FREQ 10
/*END LED defines*/

static char* readBuf;
static char* writeBuf;
static char bitmask[8] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
static void* xil_addr_base = NULL;
static int xilinx_irq = -1;
DECLARE_WAIT_QUEUE_HEAD(io_queue);
DECLARE_WAIT_QUEUE_HEAD(track_queue);

static u32 opb_freq;
static irqreturn_t flyer_xil_interrupt(int irq, void *dev_id);
static irqreturn_t flyer_xil_status_led_interrupt(int irq, void *dev_id);
static irqreturn_t flyer_xil_encoder_interrupt(int irq, void *dev_id);

static pAMCC440EP_GPIO __iomem pGPIO0 = NULL;
static pAMCC440EP_GPIO __iomem pGPIO1 = NULL;
static AMCC_REG* __iomem pGPT_TBC = NULL;
static pAMCC440EP_GPT_INT __iomem pGPT_INT = NULL;
static pAMCC440EP_GPT_COMP __iomem pGPT_COMP = NULL;
static pAMCC440EP_GPT_MASK __iomem pGPT_MASK= NULL;
static AMCC_REG* __iomem pGPT_DCT0= NULL;
static AMCC_REG* __iomem pGPT_DCIS= NULL;

enum LEDState
{
    Off = 0,
    On = 1,
    Blink = 2
};
enum LEDColor
{
    None = 0,
    Green = 1,
    Red = 2
};

int curstate = Off;
int newstate = Off;
int encstate = Off;

int curcolor = None;
int newcolor = None;
u8 toggle = 0xFF;
u8 enctoggle = 0x0;

int bLEDChanged = 0;
int curfreq = 0,newfreq = 0;

const int clock = 32768;//slow clock

void init_registers(void);
void init_gpio(void);
void set_usb_led(int state);
void setup_status_timer(u32 freq);
void setup_encoder_timer(u32 freq);
void enable_status_timer(void);
void disable_status_timer(void);
//void init_status_time(void);
void set_status_led(int state);
void set_status_color(int color);
void setLED(int freq,int red,int green);
int checkSwitch(int switchNum);
//static const int Xilinx_Size = 78756;
static const int Xilinx_Size = 54664;
static const int Xilinx_Size_3d = 149516;
int tracking=0;
int enable_part_interrupt = 0;
int marking_testmark = 0;
int waitIOSuccess = 0;
struct pid* main_pid = NULL;
int temp_status = 0;
unsigned short enc_cfg = 0;
//unsigned short enable_main_interrupts = 0;
// 05-Aug-2009 sbs 2.57: Allow Mark Aborts from a predetermined user-enabled input (Input 7).
// Ran out of signals so Abort and IO Change share a signal. The interrupt type determines
// What the signal handler should do.
unsigned long interrupt_type = 0;
u32 encodercount = 0;

__inline unsigned short u16_le_to_be(unsigned short a)
{
    return(((a>>8)&0xff)+((a<<8)&0xff00));
}

__inline unsigned long u32_le_to_be(unsigned long a)
{
    
    a = (a>>24) |
	    ((a<<8)&0x00ff0000) |
	    ((a>>8)&0x0000ff00) |
	    (a<<24);
    
    return a;
}

#define INVERTBITS(b)   (~(b))
#define REVERSEBITS(b)  (BitReverseTable[b])

static unsigned char BitReverseTable[256] =
{
    0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
    0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
    0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
    0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
    0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
    0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
    0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
    0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
    0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
    0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
    0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
    0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
    0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
    0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
    0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
    0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
    0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
    0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
    0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
    0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
    0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
    0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
    0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
    0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
    0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
    0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
    0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
    0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
    0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
    0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
    0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
    0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff
};

inline unsigned short ConvertEndian(unsigned short xival)
{
    unsigned char* ptst;
    xival = (((xival>>8)&0xff)+((xival<<8)&0xff00));
    ptst = (unsigned char*)&xival;
    ptst[0] = REVERSEBITS(ptst[0]);
    ptst[1] = REVERSEBITS(ptst[1]);
    return xival;  
}


void init_gpio(void)
{
    
    // Xilinx GPIO
    pGPIO0->osrh = pGPIO0->osrh & ~TWOBIT32(XILINX_INIT_PIN);
    pGPIO0->tsrh = pGPIO0->tsrh & ~TWOBIT32(XILINX_INIT_PIN);
    pGPIO0->odr = pGPIO0->odr & ~XILINX_INIT;
    pGPIO0->tcr = pGPIO0->tcr & ~XILINX_INIT;
    
    
    pGPIO0->osrh = pGPIO0->osrh & ~TWOBIT32(XILINX_DATA_IN_PIN);
    pGPIO0->tsrh = pGPIO0->tsrh & ~TWOBIT32(XILINX_DATA_IN_PIN);
    pGPIO0->odr = pGPIO0->odr & ~XILINX_DATA_IN;
    pGPIO0->tcr = pGPIO0->tcr | XILINX_DATA_IN;
    
    pGPIO0->osrh = pGPIO0->osrh & ~TWOBIT32(XILINX_DONE_PIN);
    pGPIO0->tsrh = pGPIO0->tsrh & ~TWOBIT32(XILINX_DONE_PIN);
    pGPIO0->odr = pGPIO0->odr & ~XILINX_DONE;
    pGPIO0->tcr = pGPIO0->tcr & ~XILINX_DONE;
    
    pGPIO0->osrh = pGPIO0->osrh & ~TWOBIT32(XILINX_PROGRAM_PIN);
    pGPIO0->tsrh = pGPIO0->tsrh & ~TWOBIT32(XILINX_PROGRAM_PIN);
    pGPIO0->odr = pGPIO0->odr & ~XILINX_PROGRAM;
    pGPIO0->tcr = pGPIO0->tcr | XILINX_PROGRAM;
    
    pGPIO0->osrh = pGPIO0->osrh & ~TWOBIT32(XILINX_CLOCK_PIN);
    pGPIO0->tsrh = pGPIO0->tsrh & ~TWOBIT32(XILINX_CLOCK_PIN);
    pGPIO0->odr = pGPIO0->odr & ~XILINX_CLOCK;
    pGPIO0->tcr = pGPIO0->tcr | XILINX_CLOCK;
    
    pGPIO0->osrh = pGPIO0->osrh & ~TWOBIT32(DSP_IRQ_LINE_PIN);
    pGPIO0->tsrh = pGPIO0->tsrh & ~TWOBIT32(DSP_IRQ_LINE_PIN);
    pGPIO0->odr = pGPIO0->odr & ~DSP_IRQ_LINE;
    pGPIO0->tcr = pGPIO0->tcr | DSP_IRQ_LINE;
    
    pGPIO0->osrh = pGPIO0->osrh & ~TWOBIT32(SWITCH_3_PIN);
    pGPIO0->tsrh = pGPIO0->tsrh & ~TWOBIT32(SWITCH_3_PIN);
    pGPIO0->odr = pGPIO0->odr & ~SWITCH_3;
    pGPIO0->tcr = pGPIO0->tcr & ~SWITCH_3;
    
    pGPIO0->osrh = pGPIO0->osrh & ~TWOBIT32(SWITCH_4_PIN);
    pGPIO0->tsrh = pGPIO0->tsrh & ~TWOBIT32(SWITCH_4_PIN);
    pGPIO0->odr = pGPIO0->odr & ~SWITCH_4;
    pGPIO0->tcr = pGPIO0->tcr & ~SWITCH_4;
    
    pGPIO1->osrh = pGPIO1->osrh & ~TWOBIT32(SWITCH_5_PIN);
    pGPIO1->tsrh = pGPIO1->tsrh & ~TWOBIT32(SWITCH_5_PIN);
    pGPIO1->odr = pGPIO1->odr & ~SWITCH_5;
    pGPIO1->tcr = pGPIO1->tcr & ~SWITCH_5;
    
    pGPIO1->osrh = pGPIO1->osrh & ~TWOBIT32(SWITCH_6_PIN);
    pGPIO1->tsrh = pGPIO1->tsrh & ~TWOBIT32(SWITCH_6_PIN);
    pGPIO1->odr = pGPIO1->odr & ~SWITCH_6;
    pGPIO1->tcr = pGPIO1->tcr & ~SWITCH_6;
    /*
    printk("UIC Registers - Init\n");
    printk("UIC0_ER: 0x%x\n",mfdcr(DCRN_UIC_ER(UIC0)));
    printk("UIC1_ER: 0x%x\n",mfdcr(DCRN_UIC_ER(UIC1)));
    printk("UIC0_CR: 0x%x\n",mfdcr(DCRN_UIC_CR(UIC0)));
    printk("UIC1_CR: 0x%x\n",mfdcr(DCRN_UIC_CR(UIC1)));
    printk("UIC0_PR: 0x%x\n",mfdcr(DCRN_UIC_PR(UIC0)));
    printk("UIC1_PR: 0x%x\n",mfdcr(DCRN_UIC_PR(UIC1)));
    printk("UIC0_TR: 0x%x\n",mfdcr(DCRN_UIC_TR(UIC0)));
    printk("UIC1_TR: 0x%x\n",mfdcr(DCRN_UIC_TR(UIC1)));
    */
   /*
    pGPIO0->osrh = pGPIO0->osrh & ~TWOBIT32(ENCODER_PIN);
    pGPIO0->tsrh = pGPIO0->tsrh & ~TWOBIT32(ENCODER_PIN);
    pGPIO0->odr = pGPIO0->odr & ~ENCODER;
    pGPIO0->tcr = pGPIO0->tcr | ENCODER;
    pGPIO0->orr = pGPIO0->orr & ~ ENCODER;
	*/
     // Setup the DSP Interrupt Line
   // pGPIO0->orr = pGPIO0->orr & ~DSP_IRQ_LINE;
 
}

void setup_status_timer(u32 freq)
{
    u32 timercounts;
    u32 comparemask;
    u32 interruptmask;
    u32 interruptenable;
  
    timercounts = opb_freq/freq;
    //comparemask = 0xFF << 7;
    comparemask = 0xFE000000;
    // Load the compare register
    //pGPT_COMP->comp0 = timercounts;
    pGPT_COMP->comp0 = timercounts;
    // Load up the compare mask
    pGPT_MASK->mask0 = comparemask;
    //Unmask the interrupt
    interruptmask = pGPT_INT->im;
    
    interruptmask &= ~BIT32(16);
    pGPT_INT->im = interruptmask;
    interruptenable = pGPT_INT->ie;
    interruptenable |= BIT32(16);
    
   
    pGPT_INT->ie = interruptenable;
    *pGPT_TBC = 0;
    /*
    printk("UIC Registers - After\n");
    printk("UIC0_ER: 0x%x\n",mfdcr(DCRN_UIC_ER(UIC0)));
    printk("UIC1_ER: 0x%x\n",mfdcr(DCRN_UIC_ER(UIC1)));
    printk("UIC0_CR: 0x%x\n",mfdcr(DCRN_UIC_CR(UIC0)));
    printk("UIC1_CR: 0x%x\n",mfdcr(DCRN_UIC_CR(UIC1)));
    printk("UIC0_PR: 0x%x\n",mfdcr(DCRN_UIC_PR(UIC0)));
    printk("UIC1_PR: 0x%x\n",mfdcr(DCRN_UIC_PR(UIC1)));
    printk("UIC0_TR: 0x%x\n",mfdcr(DCRN_UIC_TR(UIC0)));
    printk("UIC1_TR: 0x%x\n",mfdcr(DCRN_UIC_TR(UIC1)));
    */
    //printk("Done with timer init\n");    
}

void setup_encoder_timer(u32 timercounts)
{
    //printk("\ntimercounts = %d\n",timercounts); 
    u32 interruptmask;
    u32 interruptenable;
  
    
    if(timercounts == 0)
    {
	*pGPT_DCT0 = 0xFFFFFFFF;
	interruptenable = pGPT_INT->ie;
	interruptenable &= ~BIT32(17);
	interruptmask = pGPT_INT->im;   
	interruptmask |= BIT32(17);
	pGPT_INT->im = interruptmask;
	
    }
    else
    {
	timercounts = timercounts*2;
    // Load the compare register
	pGPT_COMP->comp1 = timercounts;
    // Load up the compare mask
	pGPT_MASK->mask1 = ~timercounts;
    //Unmask the interrupt
	
	*pGPT_DCIS = 1;
	encodercount = timercounts;
	*pGPT_DCT0 = timercounts;
	interruptmask = pGPT_INT->im;   
	interruptmask &= ~BIT32(17);
	pGPT_INT->im = interruptmask;
	//printk("*pGPT_DCT0: %d   0x%x\n",*pGPT_DCT0, *pGPT_DCIS = 1);
	interruptenable = pGPT_INT->ie;
	interruptenable |= BIT32(17);
	
    }
 
      
}
int flyer_xil_alive(void)
{
    if (pGPIO0->ir & XILINX_DONE)
	return XILINX_ALIVE; //it has been programmed
    
    return XILINX_DEAD;//return the status of the Xilinx
}

int flyer_xil_program_init(void)
{
    
    pGPIO0->orr = pGPIO0->orr | XILINX_PROGRAM;
    udelay(10);
    pGPIO0->orr = pGPIO0->orr & ~XILINX_PROGRAM;
    udelay(25); //wait 100 usecs;
    pGPIO0->orr = pGPIO0->orr | XILINX_PROGRAM;
    
    while(!(pGPIO0->ir & XILINX_INIT));//wait for init to go high
    
    udelay(10); //delay 10 usecs
    
    return 1;
}

int flyer_xil_program_done(void)
{
    int start = jiffies;
    while ( (!(pGPIO0->ir & XILINX_DONE)) && ((jiffies - start) < XIL_DONE_DELAY) );
    
    if ( (jiffies - start) >= XIL_DONE_DELAY)
	return 0;//timed out
    
    return 1;
}

// Function that actually does the clocking out of the data through the GPIO to the Xilinx
void flyer_xil_send(int size, char* buf)
{
    int i=0, j=0;
    //printk("size = %d\n",size);
    for (i=0;i<size;i++)
    {
	for (j=0;j<8;j++)
	{
	    //set clock low
	    //udelay(1);
	    pGPIO0->orr = pGPIO0->orr & ~XILINX_CLOCK;
	    //udelay(1);	    
	    if (buf[i] & bitmask[j])
		pGPIO0->orr = pGPIO0->orr | XILINX_DATA_IN;//clock in a 1
	    else
		pGPIO0->orr = pGPIO0->orr & ~XILINX_DATA_IN;//clock in a 0
	    //udelay(1);
	    pGPIO0->orr = pGPIO0->orr | XILINX_CLOCK;
	}	
    }
    
}

void set_usb_led(int state)
{        
    if (state & SOLID_GREEN)
	pGPIO0->orr = pGPIO0->orr | USB_GREEN_LINE;
    else
	pGPIO0->orr = pGPIO0->orr & ~USB_GREEN_LINE;
    if (state & SOLID_RED)
	pGPIO0->orr = pGPIO0->orr | USB_RED_LINE;
    else
	pGPIO0->orr = pGPIO0->orr & ~USB_RED_LINE;
}

void set_status_color(int color)
{
    if(color==Red)
    {
	pGPIO1->orr = pGPIO1->orr | STAT_RED_LINE;
	pGPIO1->orr = pGPIO1->orr & ~STAT_GREEN_LINE;
    }
    else if(color==Green)
    {
	pGPIO1->orr = pGPIO1->orr | STAT_GREEN_LINE;
	pGPIO1->orr = pGPIO1->orr & ~STAT_RED_LINE;
    }
    else
    {
	pGPIO1->orr = pGPIO1->orr & ~STAT_GREEN_LINE;
	pGPIO1->orr = pGPIO1->orr & ~STAT_RED_LINE;
    }  
}

void set_status_led(int state)
{
    setLED((state & FREQ_MASK) >> 2,state & SOLID_RED,state & SOLID_GREEN);
}

//if freq is <= 0, light is solid.
//if both colors are 0 and freq is <=0 turns off LED
void setLED(int freq,int red,int green)
{
    // Turn LED off
    //printk("led color is %s  freq = %d\n",red?"Red":"Green", freq);
    if( !red && !green && freq <= 0)
    {
	
	if(curstate == On)
	{
	    set_status_color(Off);
	    curstate = Off;
	    curcolor = None;
	}
	else if(curstate == Blink)
	{
	    newstate = Off;
	    newcolor = None;
	}
	curfreq = 0;
	return;
    }
    // Turn Led solid on
    if(freq <=0)
    {
	
	if(curstate == Off)
	{
	    if(red)
	    {
		set_status_color(Red);
		curcolor = Red;
	    }
	    else
	    {
		set_status_color(Green);
		curcolor = Green;		
	    }
	    curstate = On;
	}
	else if(curstate == Blink)
	{
	    newstate = On;
	    newcolor = red?Red:Green;
	}
	curfreq = 0;
	return;
    }

    // Set LED to Blink
    newstate = Blink;
    newcolor = red?Red:Green;
    newfreq = freq;
    
    if(curstate != Blink)
    {
	// changing frequency color or both
	if(red)
	{
	    set_status_color(Red);
	}
	else
	{
	    set_status_color(Green);
	}
		
	curstate = newstate;
	curcolor = newcolor;
	curfreq = newfreq;
	toggle = 0xFF;
	
	setup_status_timer(freq);
	
    }
 
}

static irqreturn_t flyer_xil_status_led_interrupt(int irq, void *dev_id)
{    
    u32 encstatus = 0;
    u32 ledstatus = 0;

    int ledcolor;
    u32 newcount = 0;
    ledstatus = pGPT_INT->isc;
    ledstatus &= BIT32(16);
    encstatus = *pGPT_DCIS;
    encstatus &= BIT32(0);   

    
    if(ledstatus)
    {
	
	pGPT_INT->isc = ledstatus;
	//pGPT_INT->isc = pGPT_INT->isc | BIT32(16);
	toggle = ~toggle;
	
	if(newstate == curstate && newcolor == curcolor && newfreq == curfreq)
	{
	    // Toggle the LED and return
	    toggle?(ledcolor=curcolor):(ledcolor=None);
	    set_status_color(ledcolor);
	   //*pGPT_TBC = 0;
	    //return IRQ_HANDLED;
	}
	    
	if(newstate != curstate)
	{
	    // No longer blinking disable the interrupt and set the color
	    pGPT_INT->ie &= ~BIT32(16);
	    set_status_color(newcolor);
	    curcolor = newcolor;
	    curstate = newstate;
	}
	else	     
	{
	    // check for a change in frequency
	   
	    if(newfreq != curfreq)
	    {
		pGPT_INT->ie &= ~BIT32(16);		
		// Toggle the LED
		curfreq = newfreq;
		newcount = opb_freq/newfreq;
		pGPT_COMP->comp0 = newcount;
		if(newcolor != curcolor)
		    toggle?(ledcolor=newcolor):(ledcolor=None);
		else
		    toggle?(ledcolor=curcolor):(ledcolor=None);
		set_status_color(ledcolor);
		pGPT_INT->ie |= BIT32(16);
	    }
	    else
	    {
		//color change
		toggle?(ledcolor=curcolor):(ledcolor=None);
		set_status_color(ledcolor);
	    }	    
	
	}
    }
    
   	

    return IRQ_HANDLED;
}
static irqreturn_t flyer_xil_encoder_interrupt(int irq, void *dev_id)
{
    
    u32 encstatus = 0;
    u32 ledstatus = 0;
    encstatus = *pGPT_DCIS;
    encstatus &= BIT32(0);   
    ledstatus = pGPT_INT->isc;
    ledstatus &= BIT32(17);
    pGPT_INT->isc = ledstatus;
    
    

    //printk("0x%x  %d\n", encstatus, *pGPT_DCT0);
    if(encstatus)
    {
	//printk("\nvalid encoder interrupt  %d     %d\n", *pGPT_DCIS, *pGPT_DCT0);
	*pGPT_DCIS = encstatus;
	*pGPT_DCT0 = encodercount;
	//printk("after regs  %d     %d\n",*pGPT_DCIS, *pGPT_DCT0);
	//*pGPT_TBC = 0;
	enctoggle = ~enctoggle;
	
	if(enctoggle)
	    pGPIO0->orr = pGPIO0->orr | XILINX_DATA_IN;
	else
	    pGPIO0->orr = pGPIO0->orr & ~XILINX_DATA_IN;
	
    }
    return IRQ_HANDLED;
}

int checkSwitch(int switchNum)
{
    int ret;
    ulong result, result56;
    
    result = pGPIO0->ir;
    result56 = pGPIO1->ir;
    
    switch (switchNum)
    {
	case 3:
	    ret = (result & SWITCH_3) ? 0:1 ;
	break;
	case 4:
	    ret = (result & SWITCH_4) ? 0:1 ;
	break;
	case 5:
	    ret = (result56 & SWITCH_5) ? 0:1 ;
	    break;
	case 6:
	    ret = (result56 & SWITCH_6) ? 0:1 ;
	    break;   
	default:
	ret = -1;
	break;
    }
    //printk("\nswitch num = %d  CheckSwitch = %d\n",switchNum, ret);
    return ret;
}



static ssize_t flyer_xil_read(struct file* file, char* buf, size_t count, loff_t *offset)
{
    int readCount = 1;//(count > BUFSIZE ? BUFSIZE : count);
   
    /* Actually read something*/
    //MSG("Read cmd, %s, count: %d\n",readBuf,readCount);
    readBuf[0] = flyer_xil_alive();
    copy_to_user(buf,readBuf,readCount);
    main_pid = find_vpid(current->pid);
    return readCount;
}

static ssize_t flyer_xil_write(struct file* file, const char* buf, size_t count, loff_t *offset)
{
    int writeCount = 0;
    int MasterCount = count;
    int BytesTxferd = 0;
    char* bufPtr = (char*)buf;
    
    if (count != Xilinx_Size && count != Xilinx_Size_3d)
    {
	//MSG("Not the right size, got %d, should have %d or %d\n",count,Xilinx_Size,Xilinx_Size_3d);
	return 0;
    }
    main_pid = find_vpid(current->pid);
    /* Actually write something*/
    //MSG("Write cmd, %s, count: %d\n",writeBuf,MasterCount);
    flyer_xil_program_init();
    while (MasterCount)
    {
	writeCount = (MasterCount > BUFSIZE ? BUFSIZE : MasterCount);
	//copy to buffer
	copy_from_user(writeBuf,bufPtr,writeCount);
	// write it out...
	flyer_xil_send(writeCount,writeBuf);
	//increment the vars
	BytesTxferd += writeCount;
	MasterCount -= writeCount;
	bufPtr = (char*)((unsigned int)bufPtr + writeCount);
			
    }   
    //set clock low
    pGPIO0->orr = pGPIO0->orr & ~XILINX_CLOCK;
    if (flyer_xil_program_done()) //check to see it was programmed
	return BytesTxferd;
    return 0;    
}

static int flyer_xil_open(struct inode* inode, struct file* file)
{
    //int iDevCurrent = iminor(inode);
    MSG("Module flyer_xil open, iMinor = %d\n",iDevCurrent );
    //printk("SBS Module at91_flyer_xil open\n");  
    
    return 0;
}

static int flyer_xil_release(struct inode* inode, struct file* file)
{
    MSG("Module flyer_xil release\n" );
    
    return 0;
}

static long flyer_xil_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    /*
    AT91PS_PIO pPIOA = (AT91PS_PIO)AT91_IO_P2V(AT91C_BASE_PIOA);
    AT91PS_PIO pPIOB = (AT91PS_PIO)AT91_IO_P2V(AT91C_BASE_PIOB);
    */
    /* Make sure the command belongs to us*/
    int ret_val = 0;
    //int n;
    //unsigned short tst, tst2;
    //unsigned char* ptst;
    unsigned char io_stat;
    int iTimeout;
    waitStruct Wait;
    unsigned short usTimeLow,usTimeHigh; //low has 16, high has four...
    wait_queue_t wait;
    
    if (_IOC_TYPE(cmd) != XILINX_CONFIG_IOCTL_BASE)
	return -ENOTTY;
    
    switch(cmd )
    {
    case XIL_CHECK_STATUS:
	//MSG("at91_flyer_xil Check Status\n");
	return flyer_xil_alive();
	/*
	case XIL_START_CLOCKS:
	if (arg)
	{
	    //enable the PIO for the clocks
	    pPIOA->PIO_PDR = XILINX_SLOW_CLOCK;
	    pPIOA->PIO_BSR = XILINX_SLOW_CLOCK;
	    pPIOB->PIO_PDR = XILINX_FAST_CLOCK;
	    pPIOB->PIO_ASR = XILINX_FAST_CLOCK;
	    
	    AT91_SYS->PMC_SCER = 0x300;//enable PCKO and PCK1
	    AT91_SYS->PMC_IDR = 0xf00;//disable interrupts, probably not needed
	    AT91_SYS->PMC_PCKR[0] = 0x07; //PLLB/2 for 48 MHz
	    AT91_SYS->PMC_PCKR[1] = 0x11; //MainClock/16 for 1 MHz 
	    
	}
	else
	{
	    //disable the PIO for the clocks
	    pPIOA->PIO_PER = XILINX_SLOW_CLOCK;
	    pPIOA->PIO_CODR = XILINX_SLOW_CLOCK;
	    pPIOA->PIO_OER = XILINX_SLOW_CLOCK;
	    
	    pPIOB->PIO_PER = XILINX_FAST_CLOCK;
	    pPIOB->PIO_CODR = XILINX_FAST_CLOCK;
	    pPIOB->PIO_OER = XILINX_FAST_CLOCK;
	}
	return arg;
	*/	
    case XIL_ENC_PULSE_SET:
	{
	    setup_encoder_timer(arg);
	    
	}
	return arg;
	 
    case XIL_WAKE_IO_SLEEPERS: //called by the thread that accepts data from WinMark on an abort.
	wake_up_interruptible(&io_queue);
	wake_up_interruptible(&track_queue);
	
	break;
	
    case XIL_TESTMARK_DONE: //called so that the driver knows the testmark is complete..
	marking_testmark = 0;
	break;
    case XIL_CHECK_SWITCH:
	return checkSwitch(arg);
	break;
	
    
    //Here are the Read commands for the Xilinx...
    case XIL_GET_IO:
	if (xil_addr_base)
	{
	    	    
	    arg = (unsigned long)ConvertEndian(*((unsigned short*)xil_addr_base + XIL_IO_OFFSET));		
	}
	else
	    arg = -1;
	return arg;
    
    case XIL_SYSTEM_STATUS:
	if (xil_addr_base)
	{
	    arg = (unsigned long)ConvertEndian(*((unsigned short*)xil_addr_base + XIL_STATUS_OFFSET));
	    //arg = (unsigned long)*((unsigned short*)xil_addr_base + XIL_STATUS_OFFSET);  
	}
	else
	    arg = -1;
	return arg;	
	
    case XIL_GET_VERSION:
	if (xil_addr_base)
	{
	    arg = (unsigned long)ConvertEndian(*((unsigned short*)xil_addr_base + XIL_VERSION_OFFSET));
	    //arg = (unsigned long)*((unsigned short*)xil_addr_base + XIL_VERSION_OFFSET);   
	}
	else
	    arg = -1;
	return arg;
	
    case XIL_GET_KEYPAD:
	if (xil_addr_base)
	{
	    arg = (unsigned long)ConvertEndian(*((unsigned short*)xil_addr_base + XIL_KEYPAD_OFFSET));
	    //arg = (unsigned long)*((unsigned short*)xil_addr_base + XIL_KEYPAD_OFFSET);   
	}
	else
	    arg = -1;
	return arg;
	
    case XIL_GET_TESTMARK:
	return marking_testmark;
    
    case XIL_GET_WAIT_DIGITAL:
	if (xil_addr_base)
	{
	    arg = (unsigned long)ConvertEndian(*((unsigned short*)xil_addr_base + XIL_WAIT_DIGITAL_OFFSET));
	    //arg = (unsigned long)*((unsigned short*)xil_addr_base + XIL_WAIT_DIGITAL_OFFSET); 
	}
	else
	    arg = -1;
	return arg;
    case XIL_GET_TEMP_STATUS://Get the temp status of the last interrupt
	return temp_status;
    case XIL_GET_SWITCHES:
	if (xil_addr_base)
	{
	    arg = (unsigned long)ConvertEndian(*((unsigned short*)xil_addr_base + XIL_SWITCHES));
	    //arg = (unsigned long)*((unsigned short*)xil_addr_base + XIL_SWITCHES); 
	}
	else
	    arg = -1;
	return arg;
    //Here are the Write commands for the Xilinx...
    case XIL_SET_IO:
	if (xil_addr_base)
	{
	    *((unsigned short*)xil_addr_base + XIL_IO_OFFSET) = ConvertEndian((unsigned short)arg);
	   
	}
	else
	    arg = -1;
	return arg;
	
    case XIL_DEBUG_SET:
	if (xil_addr_base)
	{
	 //*((unsigned short*)xil_addr_base + XIL_DEBUG_SET_OFFSET) = (unsigned short)arg;
	    *((unsigned short*)xil_addr_base + XIL_DEBUG_SET_OFFSET) = ConvertEndian((unsigned short)arg);  
	}
	else
	    arg = -1;
	return arg;
    case XIL_DEBUG_CLEAR:
	if (xil_addr_base)
	{
	 //*((unsigned short*)xil_addr_base + XIL_DEBUG_CLR_OFFSET) = (unsigned short)arg;
	    *((unsigned short*)xil_addr_base + XIL_DEBUG_CLR_OFFSET) = ConvertEndian((unsigned short)arg);     
	}
	else
	    arg = -1;
	return arg;
    case XIL_ENCODER_CFG:
	if (xil_addr_base)
	{
	    //printk("XIL_ENCODER_CFG %x\n",arg);
	 //*((unsigned short*)xil_addr_base + XIL_ENC_CFG_OFFSET) = (unsigned short)arg;
	 *((unsigned short*)xil_addr_base + XIL_ENC_CFG_OFFSET) = ConvertEndian((unsigned short)arg);     
	}
	else
	    arg = -1;
	return arg;
    case XIL_ENCODER_CFG_STR:
	if (xil_addr_base)
	{
	// 07-Nov-2011 sbs 3.11 Issue: Make internal part triggering work in Banner marking 
	// Store the enc_cfg so that it can be sent with wait digital below
	    //printk("XIL_ENCODER_CFG_STR %x\n",arg);
	    enc_cfg = (unsigned short)arg;   
	}
	else
	    arg = -1;
	return arg;
    case XIL_SET_PART_PITCH:
	if (xil_addr_base)
	{
	    //printk("XIL_SET_PART_PITCH %x\n",arg);
	 //*((unsigned short*)xil_addr_base + XIL_PPC_OFFSET) = (unsigned short)arg;
	 *((unsigned short*)xil_addr_base + XIL_PPC_OFFSET) = ConvertEndian((unsigned short)arg);   
	}
	else
	    arg = -1;
	return arg;
    // 01-Nov-2011 sbs 3.11 Issue: Banner marking with user part pitch enabled 
    case XIL_SET_BANNER_PITCH:
	if (xil_addr_base)
	{
	    *((unsigned short*)xil_addr_base + XIL_BANNER_PITCH) = ConvertEndian((unsigned short)arg);   
	}
	else
	    arg = -1;
	return arg;
    case XIL_SET_BANNER_COUNT:
	if (xil_addr_base)
	{
	    *((unsigned short*)xil_addr_base + XIL_BANNER_COUNT) = ConvertEndian((unsigned short)arg);   
	}
	else
	    arg = -1;
	return arg;
    case XIL_SET_WAIT_DIGITAL://This will cause the calling process to be placed on the IO wait queue...
	if (xil_addr_base)
	{
	    waitIOSuccess = 0;
	    Wait = * (waitStruct*)(arg);
	    if (Wait.iTimeout == 0)//shouldn't happen, but handle it just as well...
	    {
		ret_val = (unsigned long)ConvertEndian(*((unsigned short*)xil_addr_base + XIL_IO_OFFSET));
		//ret_val = (unsigned long)*((unsigned short*)xil_addr_base + XIL_IO_OFFSET);
	    }
	    else if (Wait.iTimeout > 0)
	    {
		iTimeout = Wait.iTimeout -1;//for Xilinx, goes from 0-999999 for 1-1000000
		usTimeLow = (unsigned short)(iTimeout & 0xffff);
		usTimeHigh = (unsigned short)( (iTimeout & 0xf0000) >> 16);
		//*((unsigned short*)xil_addr_base + XIL_TIMEOUT_LSB) = usTimeLow; 
		//*((unsigned short*)xil_addr_base + XIL_TIMEOUT_MSB) = usTimeHigh;
		*((unsigned short*)xil_addr_base + XIL_TIMEOUT_LSB) = ConvertEndian(usTimeLow);
		*((unsigned short*)xil_addr_base + XIL_TIMEOUT_MSB) = ConvertEndian(usTimeHigh);
	    }
	    
	    
	    init_waitqueue_entry(&wait,current);
	    add_wait_queue(&io_queue,&wait);
	    set_current_state(TASK_INTERRUPTIBLE);
	    if (enable_part_interrupt)//should only occur once at the start of the mark session.
	    {
		enable_part_interrupt = 0;
		
		//AT91_SYS->PIOB_CODR = AT91C_PIO_PB24;//generate interrupt.pull this down so that the DSP is on the same part sense
		//AT91_SYS->PIOB_SODR = AT91C_PIO_PB24;//set it back.
		pGPIO0->orr = pGPIO0->orr & ~BIT32(DSP_IRQ_LINE_PIN);//generate interrupt.pull this down so that the DSP is on the same part sense
		pGPIO0->orr = pGPIO0->orr | BIT32(DSP_IRQ_LINE_PIN);//set it back.
		
		//*((unsigned short*)xil_addr_base + XIL_PARTSENSE_ENABLE) = (unsigned short)1;
		*((unsigned short*)xil_addr_base + XIL_PARTSENSE_ENABLE) = ConvertEndian(1); 
		*((unsigned short*)xil_addr_base + XIL_WAIT_DIGITAL_OFFSET) = ConvertEndian((unsigned short)Wait.dwState); 
		//udelay(200);
		// 07-Nov-2011 sbs 3.11 Issue: Make internal part triggering work in Banner marking 
	        // Send the encoder cfg now that Flyer is ready to receive interrupts from the Xilinx 
		*((unsigned short*)xil_addr_base + XIL_ENC_CFG_OFFSET) = ConvertEndian((unsigned short)enc_cfg);    
	    }
	    else		
		*((unsigned short*)xil_addr_base + XIL_WAIT_DIGITAL_OFFSET) = (unsigned short)Wait.dwState; 
		
	    //*((unsigned short*)xil_addr_base + XIL_WAIT_DIGITAL_OFFSET) = (unsigned short)Wait.dwState;
	    *((unsigned short*)xil_addr_base + XIL_WAIT_DIGITAL_OFFSET) = ConvertEndian((unsigned short)Wait.dwState); 
	    if (Wait.dwState & 0xff00)//there is a bit in the mask set, or tracking bit set
	    {
		
		schedule();//put the calling process to sleep
	    }
	    set_current_state(TASK_RUNNING);
	    remove_wait_queue(&io_queue,&wait);
	    if (waitIOSuccess)
		return 0;
	    else
		return -1;
	}
	else
	    ret_val = -1;
	return ret_val;
    case XIL_TRACK_WAIT://This will cause the calling process to be placed on the track wait queue...
	if (xil_addr_base)
	{
	    //printk("XIL_TRACK_WAIT \n");
	    wait_queue_t wait;
	    //printk("current pid = %d\n", current->pid);
	    init_waitqueue_entry(&wait,current);
	    add_wait_queue(&track_queue,&wait);
	    set_current_state(TASK_INTERRUPTIBLE);
	    //send the interrupt to the DSP telling it we are ready to handle the tracking interrupt.
	    //AT91_SYS->PIOB_CODR = AT91C_PIO_PB24;//generate interrupt
	    //AT91_SYS->PIOB_SODR = AT91C_PIO_PB24;//set it back.
	    pGPIO0->orr = pGPIO0->orr & ~BIT32(DSP_IRQ_LINE_PIN);//generate interrupt.pull this down so that the DSP is on the same part sense
	    pGPIO0->orr = pGPIO0->orr | BIT32(DSP_IRQ_LINE_PIN);//set it back.
	    schedule();//put the calling process to sleep
	    set_current_state(TASK_RUNNING);
	    remove_wait_queue(&track_queue,&wait);
	}
	else
	    arg = -1;
	return arg;
    case XIL_LED_SET_USB:
	set_usb_led(arg);
	return arg;
    case XIL_LED_SET_STATUS:
        set_status_led(arg);
	return arg;
    case XIL_SET_SWITCHES:
	//If this is disabled, the very next XIL_SET_WAIT_DIGITAL will then call it again to enable.
	//That way the Xilinx part sense is enable right when we look for the first part.
	if (xil_addr_base)
	{
	  // 05-Aug-2009 sbs 2.57: Allow Mark Aborts from a predetermined user-enabled input (Input 7).
	 // Read XIL switches (which includes enabling/disabling ABORT IO in order to clear the last
	 // interrupt prior to enabling/disabling Abort IO
	    
	    io_stat = *((unsigned char*)xil_addr_base + XIL_SWITCHES);
	    //io_stat = (unsigned long)ConvertEndian(*((unsigned short*)xil_addr_base + XIL_SWITCHES));
	 //*((unsigned short*)xil_addr_base + XIL_SWITCHES) = (unsigned short)arg;
	 *((unsigned short*)xil_addr_base + XIL_SWITCHES) = ConvertEndian((unsigned short)arg); 
	
	}
	else
	    arg = -1;
	return arg;
    case XIL_SET_PART_INTERRUPT:
	if (xil_addr_base)
	{
	 //*((unsigned short*)xil_addr_base + XIL_PARTSENSE_ENABLE) = (unsigned short)arg;
	    //printk("XIL_SET_PART_INTERRUPT %x\n",arg);
	 *((unsigned short*)xil_addr_base + XIL_PARTSENSE_ENABLE) = ConvertEndian((unsigned short)arg); 
	 if (!arg)	    
	     enable_part_interrupt = 1;
        }
	return arg;
// sbs 2.21 11-Jul-2008 MOSAIC: Z-axis servo status is read indirectly through the Xilinx
    case XIL_Z_SERVO_STATUS:
	if (xil_addr_base)
	{
	    //arg = (unsigned long)*((unsigned short*)xil_addr_base + XIL_Z_STATUS_OFFSET);
	    arg = (unsigned long)ConvertEndian(*((unsigned short*)xil_addr_base + XIL_Z_STATUS_OFFSET));   
	}
	else
	    arg = -1;
	//printk("XIL_Z_SERVO_STATUS %x\n",arg);
	return arg;
//sbs 2.21 07-Jul-2008 Add an IO Change Event for PANNIER
    case XIL_SET_IO_CHANGE:
	//printk("Set IO change %x\n",(unsigned short)arg);
	if (xil_addr_base)
	{
	 //*((unsigned short*)xil_addr_base + XIL_IO_CHANGE_OFFSET) = (unsigned short)arg;
	   *((unsigned short*)xil_addr_base + XIL_IO_CHANGE_OFFSET) = ConvertEndian((unsigned short)arg);    
	}
	else
	    arg = -1;
	return arg;
    
	// 05-Aug-2009 sbs 2.57: Allow Mark Aborts from a predetermined user-enabled input (Input 7).
	// Ran out of signals so Abort and IO Change share a signal. The interrupt type determines
	// What the signal handler should do.
    case XIL_GET_INTERRUPT_TYPE:	
	arg = interrupt_type;
	return arg;
	
    case XIL_SET_DIODE_PTR:
	if (xil_addr_base)
	{
	 //*((unsigned short*)xil_addr_base + XIL_IO_CHANGE_OFFSET) = (unsigned short)arg;
	    *((unsigned short*)xil_addr_base + XIL_DIODE_PTR_OFFSET) = ConvertEndian((unsigned short)arg);    
	}
	else
	    arg = -1;
	return arg;
    case XIL_GET_DIODE_PTR:
	if (xil_addr_base)
	{
	    	    
	    arg = (unsigned long)ConvertEndian(*((unsigned short*)xil_addr_base + XIL_DIODE_PTR_OFFSET));		
	}
	else
	    arg = -1;
	return arg;
/*
    case XIL_ENABLE_INTERRUPTS:
	//printk("Interrupts %x\n",(unsigned short)arg);
	if((unsigned short)arg > 0)
	    enable_main_interrupts = 1;
	else
	    enable_main_interrupts = 0;
	return arg;
*/
    default:
	return -ENOTTY;    	
    }
    return 0;
}
/***************************************
  * Function so that other modules can communicate with the Xilinx, namely the SPI module...
  **************************************/
void* xil_get_mapped_address(void)
{
   return xil_addr_base; 
}

/*
void init_registers(void)
{
    
    
}
*/
/********************************************************************/
static struct file_operations flyer_xil_fops = {
    owner:		THIS_MODULE,
    read:		flyer_xil_read,
    write:		flyer_xil_write,
    unlocked_ioctl:	flyer_xil_ioctl,
    open:		flyer_xil_open,
    release:	        flyer_xil_release,
    };

static int __init flyer_xil_init_module(void)
{
    int res = 0;
    int status;
    unsigned long phys_addr;
    unsigned long end_addr;
    unsigned long base_len;
    struct device_node* node = NULL;
    MSG("Module flyer_xil init\n" );

    /* get the clock (Hz) for the OPB.*/
    node = of_find_node_by_path("/plb/opb");
    if (!node) {
    	printk(KERN_ERR "(E) Flyer_Xilinx failed to find device tree node /plb/opb\n");
	return -1;
    }
    opb_freq = *(u32*)of_get_property(node, "clock-frequency", NULL);

    readBuf = kmalloc(BUFSIZE, GFP_KERNEL);
    writeBuf = kmalloc(BUFSIZE,GFP_KERNEL);
    if (!readBuf || !writeBuf)
	return -ENOMEM;
    
    /*register the device with the kernel*/	
    res = register_chrdev(XILINX_CONFIG_MAJOR,"flyer_xil",&flyer_xil_fops);
    if (res<0)
    {
	MSG("Can't register device flyer_xil with kernel.\n");
	return res;
    }
    
    if (!xil_addr_base)
    {
	xil_addr_base = (void*) ioremap(FLYER_XILINX_BASE, SZ_16K);
	printk(KERN_DEBUG "xil_addr_base=0x%x\n",(unsigned int)xil_addr_base);
    }
    
    //write a mask of zero and a input input wait of zero prior to getting the interrupt
    *((unsigned short*)xil_addr_base + XIL_WAIT_DIGITAL_OFFSET) = 0x0000;
    
    // Setup GPIO 0 Registers 
    
    phys_addr = GPIO_0_PHYS_START;
    end_addr = GPIO_0_PHYS_END;
    base_len = end_addr - phys_addr + 1;
	
    pGPIO0 = ioremap(phys_addr, base_len);
    if (!pGPIO0)
    {
	printk(KERN_ERR "AMCC440EP GPIO 0 ioremap FAILED\n");
	return -1;
    }   
    
    // Setup GPIO 1 Registers -- Status Leds
    
    phys_addr = GPIO_1_PHYS_START;
    end_addr = GPIO_1_PHYS_END;
    base_len = end_addr - phys_addr + 1;
	
    pGPIO1 = ioremap(phys_addr, base_len);
    if (!pGPIO1)
    {
	printk(KERN_ERR "AMCC440EP GPIO 1 ioremap FAILED\n");
	return -1;
    }
    // Setup the General Purpose Timer registers
     
    phys_addr = GPT_TBC_PHYS_START;
    end_addr = GPT_TBC_PHYS_END;
    base_len = end_addr - phys_addr + 1;
	
    pGPT_TBC = ioremap(phys_addr, base_len);
    if (!pGPT_TBC)
    {
	printk(KERN_ERR "AMCC440EP General Purpose Timers pGPT_TBC ioremap FAILED\n");
	return -1;
    }
    
    phys_addr = GPT_INT_PHYS_START;
    end_addr = GPT_INT_PHYS_END;
    base_len = end_addr - phys_addr + 1;
	
    pGPT_INT = ioremap(phys_addr, base_len);
    if (!pGPT_INT)
    {
	printk(KERN_ERR "AMCC440EP General Purpose Timers pGPT_INT ioremap FAILED\n");
	return -1;
    }
    
    phys_addr = GPT_COMP_PHYS_START;
    end_addr = GPT_COMP_PHYS_END;
    base_len = end_addr - phys_addr + 1;
	
    pGPT_COMP = ioremap(phys_addr, base_len);
    if (!pGPT_COMP)
    {
	printk(KERN_ERR "AMCC440EP General Purpose Timers pGPT_COMP ioremap FAILED\n");
	return -1;
    }
    
    phys_addr = GPT_MASK_PHYS_START;
    end_addr = GPT_MASK_PHYS_END;
    base_len = end_addr - phys_addr + 1;
	
    pGPT_MASK = ioremap(phys_addr, base_len);
    if (!pGPT_MASK)
    {
	printk(KERN_ERR "AMCC440EP General Purpose Timers pGPT_MASK ioremap FAILED\n");
	return -1;
    }
    
    phys_addr = GPT_DCT0_PHYS_START;
    end_addr = GPT_DCT0_PHYS_END;
    base_len = end_addr - phys_addr + 1;
	
    pGPT_DCT0 = ioremap(phys_addr, base_len);
    if (!pGPT_DCT0)
    {
	printk(KERN_ERR "AMCC440EP General Purpose Timers pGPT_DCT0 ioremap FAILED\n");
	return -1;
    }
    
    phys_addr = GPT_DCIS_PHYS_START;
    end_addr = GPT_DCIS_PHYS_END;
    base_len = end_addr - phys_addr + 1;
	
    pGPT_DCIS = ioremap(phys_addr, base_len);
    if (!pGPT_DCIS)
    {
	printk(KERN_ERR "AMCC440EP General Purpose Timers pGPT_DCIS ioremap FAILED\n");
	return -1;
    }
    
    init_gpio();
    
    
    //set_usb_led(SOLID_GREEN);
    
    
    node = of_find_node_by_path("/xilinx");

    // External IRQ 4 on UIC0 is interrupt 27 (PPC 440EP manual page 224).  See DTS file.
    xilinx_irq = irq_of_parse_and_map(node, 0);
    status = request_irq(xilinx_irq, flyer_xil_interrupt, 0, "flyer_xil", xil_addr_base);
    if (status) 
    {
	printk(KERN_ERR "flyer_xil: IRQ %d xil interrupt request failed - status %d!\n", res, status);
	return -EBUSY;
    } 
    
    node = of_find_node_by_path("/timer0");
    // GPT0 on UIC0 is interrupt 18 (PPC 440EP manual page 224).  See DTS file.
    res = irq_of_parse_and_map(node, 0);
    status = request_irq(res, flyer_xil_status_led_interrupt, 0, "flyer_xil",0);
    if (status) 
    {
	printk(KERN_ERR "flyer_xil: IRQ %d status_led_interrupt request failed - status %d!\n", res, status);
	return -EBUSY;
    } 
    
    // GPT1 on UIC0 is interrupt 19 (PPC 440EP manual page 224).  See DTS file.
    node = of_find_node_by_path("/timer1");
    res = irq_of_parse_and_map(node, 0);
    status = request_irq(res, flyer_xil_encoder_interrupt, 0, "flyer_xil",0);
    if (status) 
    {
	printk(KERN_ERR "flyer_xil: IRQ %d xil_encoder_interrupt request failed - status %d!\n", res, status);
	return -EBUSY;
    } 
   
    
    *((unsigned short*)xil_addr_base + XIL_IO_CHANGE_OFFSET) = 0;
    printk(KERN_INFO "FlyerII Xilinx driver v%s\n",XILINX_VERSION);   
    return 0;
}

static void __exit flyer_xil_exit_module(void)
{
    printk( KERN_DEBUG "Module flyer_xil exit\n" );
    if (readBuf)
	kfree(readBuf);
    if (writeBuf)
	kfree(writeBuf);
    if (xilinx_irq != -1)
    {
	free_irq(xilinx_irq,0);
	xilinx_irq = -1;
    }
    
    unregister_chrdev(XILINX_CONFIG_MAJOR,"flyer_xil");
    if (xil_addr_base)
    {
	iounmap(xil_addr_base);
        printk(KERN_DEBUG "Releasing xil_addr_base @ 0x%x\n",(unsigned int)xil_addr_base);
	xil_addr_base = NULL;
    }   
}

static irqreturn_t flyer_xil_interrupt(int irq, void *dev_id)
{
    
	// Must declare the interrupt table address as volatile so compiler
	// doesn't accidentally optimize away reads, which have the side effect
	// of re-setting XILINX interrupts on the FPGA.
	unsigned short volatile * int_table_addr = \
		(unsigned short*)xil_addr_base + XIL_INT_TABLE;
    unsigned short int_table = 0;
    unsigned char io_stat;
    //if(!enable_main_interrupts)
	//return IRQ_HANDLED;
    
    if (!xil_addr_base)
    {
	printk (KERN_ERR "In xilinx interrupt and xil_addr_base not set!\n");
	return IRQ_HANDLED;
    }
    int_table = ConvertEndian(*int_table_addr); 
    //printk("int_table: %x\n",int_table);
    if ( (int_table & IO_INTERRUPT) )
    {
	//printk("IO_INTERRUPT\n");
	interrupt_type = IO_INTERRUPT;
	int_table &= ~TIMEOUT_INTERRUPT;
	waitIOSuccess = 1;
	*((unsigned short*)xil_addr_base + XIL_WAIT_DIGITAL_OFFSET) = 0;//clear the mask!
	wake_up_interruptible(&io_queue);//wake up the waiting process(es)
    }
    if ( (int_table & TIMEOUT_INTERRUPT) )
    {
	//printk("TIMEOUT_INTERRUPT\n");
	waitIOSuccess = 0;
	interrupt_type = TIMEOUT_INTERRUPT;
	*((unsigned short*)xil_addr_base + XIL_WAIT_DIGITAL_OFFSET) = 0;//clear the mask!
	wake_up_interruptible(&io_queue);//wake up the waiting process(es)
    }
    if ( (int_table & TRACK_INTERRUPT) )
    {
	//printk("TRACK_INTERRUPT\n");
	interrupt_type = TRACK_INTERRUPT;
	wake_up_interruptible(&track_queue);//wake up the waiting process(es)
    }
    
    if (int_table & KEY_INTERRUPT)
    {
	//printk("KEY_INTERRUPT\n");
	//read the key_stat to clear the interrupt with the Xilinx
	unsigned char key_stat = *((unsigned char*)xil_addr_base + XIL_KEYPAD_OFFSET);
	interrupt_type = KEY_INTERRUPT;
	if (marking_testmark)
	{
	    //printk(KERN_ERR "Already Marking testmark, ignoring...\n");
	    return IRQ_HANDLED;
	}
	if (main_pid && key_stat & TESTMARK_KEY)
	{
	    printk(KERN_ERR "Running TestMark, key register: 0x%02x\n",key_stat);
	    marking_testmark = 1;
	    kill_pid(main_pid,SIG_TESTMARK,1);
	}
    }
    
    if (int_table & TEMP_INTERRUPTS)
    {
	//printk("Got Overtemp\n");
	temp_status = (int_table & TEMP_INTERRUPTS) >> 3;
	interrupt_type = TEMP_INTERRUPTS;
	if (main_pid)
	    kill_pid(main_pid,SIG_OVERTEMP,1);
	
    }
    
    //sbs 2.21 07-Jul-2008 Add an IO Change Event for PANNIER
    if (int_table & IOCHANGE_INTERRUPT)
    {
	io_stat = *((unsigned char*)xil_addr_base + XIL_IO_CHANGE_OFFSET);
	//printk("IOCHANGE_INTERRUPT\n");
	//printk("Got IO Change %x  : %d  :%x\n",int_table, SIG_IOCHANGE, io_stat);
	interrupt_type = IOCHANGE_INTERRUPT;
	if (main_pid)
	    kill_pid(main_pid,SIG_IOCHANGE,1);
    }
    
    if (int_table & ABORT_INTERRUPT)
    {
	io_stat = *((unsigned char*)xil_addr_base + XIL_SWITCHES);
	//printk("ABORT_INTERRUPT\n");
	interrupt_type = ABORT_INTERRUPT;
	//printk("Got Abort %x  : %d  :%x\n",int_table, SIG_IOCHANGE, io_stat);
	if (main_pid)
	    kill_pid(main_pid,SIG_IOCHANGE,1);
    }    
    // As a last step, clear the XILINX interrupt again just in case another
    // interrupt occurred after the previous read but before this handler
    // completed (all such interrupts will be ignored by the Linux kernel
    // anyway).  This fixes a June 2016 bug encountered by mondragon in which
    // the head would randomly no longer mark using the input 0 trigger.
    int_table = *int_table_addr;
    return IRQ_HANDLED;
}




EXPORT_SYMBOL(xil_get_mapped_address);

module_init(flyer_xil_init_module);
module_exit(flyer_xil_exit_module);


MODULE_DESCRIPTION("Module to initialize the Xilinx XC2S50E FPGA");
MODULE_AUTHOR("S. Saban (ssaban@synrad.com)");
MODULE_LICENSE("GPL");
