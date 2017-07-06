/***************************************************************************
 *   Copyright (C) 2010 by Synrad, Inc                                     *
 *   ssaban@synrad.com                                                     *
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
#include <asm/uaccess.h>
#include <linux/delay.h>

#include <linux/spi/synrad_spi.h>
#include <asm/Flyer_Xilinx.h>
#include <asm/FlyerII.h>
#include <linux/pci.h>
#include <asm/byteorder.h>

#define SPI_LINK_MAJOR 241
#define SPI_IOCTL_BASE 0xAF
#define SPI_GET_SERVO_STATUS _IOWR(SPI_IOCTL_BASE,1,int)
#define SPI_SET_SPI_MODE _IOR(SPI_IOCTL_BASE,2,int)
#define SPI_GET_BUFFER_SIZE _IOWR(SPI_IOCTL_BASE,5,int)
#define SPI_SERVO_RESET _IOR(SPI_IOCTL_BASE,6,int)
#define SPI_SERVO_BUS _IOR(SPI_IOCTL_BASE,7,int)
#define SPI_SERVO_READY _IOR(SPI_IOCTL_BASE,8,int)
#define SPI_SERVO_STATUS _IOR(SPI_IOCTL_BASE,9,int)
#define SPI_Z_SERVO_STATUS _IOR(SPI_IOCTL_BASE,10,int)
#define SPI_DISABLE _IOR(SPI_IOCTL_BASE,11,int)
#define SPI_SET_SPEED _IOR(SPI_IOCTL_BASE,12,int)
#define SPI_SERVO_BUS_CLEAR _IOR(SPI_IOCTL_BASE,13,int)
//#define DEBUG_SPI

#define CS_0_PIN                   8     // GPIO Bank 1
#define CS_1_PIN                   9     // GPIO Bank 1
#define SERVO_RESET_PIN           10     // GPIO Bank 1

#define CS_0                      BIT32(8)     // GPIO Bank 1
#define CS_1                      BIT32(9)     // GPIO Bank 1
#define SERVO_RESET               BIT32(10)     // GPIO Bank 1

//make it big enough to accept the boot code currently 11000 bytes
#define BUFSIZE PAGE_SIZE*8

#ifdef DEBUG
#define MSG(string, args...) printk(KERN_DEBUG "spi_link:" string, ##args)
#else
#define MSG(string, args...)
#endif

static u32 opb_quarter_freq;
static int iDev2 = 2;
static int iDev3 = 3;
static int bAcquiring = 0;
//static int bAborted = 0;
static int spi_enabled = 0;
static struct semaphore spi_lock;			/* protect access to SPI bus */
//static int current_device = -1;				/* currently selected SPI device */
//DECLARE_WAIT_QUEUE_HEAD(wait_queue);
static int m_iStatus = 0;
static int m_iZStatus = 0;
static int current_speed = 0;
static unsigned short m_lastTxfer = 0;
char* readBuf;
char* writeBuf;
void* xil_addr;

/* Allocate a single SPI transfer descriptor.  We're assuming that if multiple
   SPI transfers occur at the same time, spi_access_bus() will serialize them.
   If this is not valid, then either (i) each dataflash 'priv' structure
   needs it's own transfer descriptor, (ii) we lock this one, or (iii) use
   another mechanism.   */
//static struct spi_transfer_list* spi_transfer_desc;
static struct spi_local spi_dev[NR_SPI_DEVICES];

static pAMCC440EP_SPI __iomem controller = NULL;
static pAMCC440EP_GPIO __iomem chipselect = NULL;

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

void spi_access_bus(short device)
{
   device = device-2;
   /* Ensure that requested device is valid */
   if ((device < 0) || (device >= NR_SPI_DEVICES))
       panic("synrad_spi: spi_access_bus called with invalid device");
    
   spi_enabled++;
   //Lock the bus
   down(&spi_lock);
   //current_device = device;
   chipselect->orr &= ~spi_dev[device].pcs;
   

#ifdef DEBUG_SPI
		printk("SPI CS%i enabled\n", device);
#endif
}

void spi_release_bus(short device)
{
        device = device-2;
//    	if (device != current_device)
		//panic("Synrad_spi: spi_release called with invalid device");

	/* Release the SPI bus */
	//current_device = -1;
	up(&spi_lock);
	spi_enabled--;
	chipselect->orr |= spi_dev[device].pcs;

#ifdef DEBUG_SPI
		printk("SPI CS%i disabled\n", device);
#endif
	
}

//read is used for the asynchronous write
static ssize_t spi_link_read(struct file* file, char* buf, size_t count, loff_t *offset)
{
    ssize_t ret;
    u32 bcount = 0;
    int temp;   
    unsigned short xil;
       
    int writeCount =(count > BUFSIZE ? BUFSIZE : count);
    int minor = *((int*)file->private_data);
    //printk("Module spi_link_read, iMinor = %d\n",minor );
    
    copy_from_user(writeBuf,buf,writeCount);
   
    xil_addr = xil_get_mapped_address();
    if(minor == iDev2)
	xil = ConvertEndian(*((unsigned short*)xil_addr + XIL_STATUS_OFFSET));
    else // if (minor == iDev3)
	xil = ConvertEndian(*((unsigned short*)xil_addr + XIL_Z_STATUS_OFFSET));
    //xil = *((unsigned short*)xil_addr + XIL_STATUS_OFFSET);
    
    
    spi_access_bus(minor);
    while(bcount < writeCount)
    {
	    //while(!(controller->status & SPSTATUS_RXREADY));
	    controller->TxD = writeBuf[bcount];
	    controller->control = SPCTRL_STR_ENABLE;
	    //ndelay(700*(current_speed+1)+current_speed*200);
	    while(!(controller->status & SPSTATUS_RXREADY));
	    readBuf[0] = controller->RxD;
	    bcount++;
	    controller->TxD = writeBuf[bcount];
	    controller->control = SPCTRL_STR_ENABLE;
	    //ndelay(700*(current_speed+1)+current_speed*200);
	    while(!(controller->status & SPSTATUS_RXREADY));
	    readBuf[1] = controller->RxD;
	    bcount++;
	    
	    /*
	    controller->TxD = writeBuf[bcount];
	    controller->control = SPCTRL_STR_ENABLE;
	    //ndelay(700*(current_speed+1)+current_speed*200);
	    while(!(controller->status & SPSTATUS_RXREADY));
	    readBuf[0] = controller->RxD;
	    bcount++;
	    controller->TxD = writeBuf[bcount];
	    controller->control = SPCTRL_STR_ENABLE;
	    //ndelay(700*(current_speed+1)+current_speed*200);
	    while(!(controller->status & SPSTATUS_RXREADY));
	    readBuf[1] = controller->RxD;
	    bcount++;
	    */
    }
    spi_release_bus(minor);

    temp = ((unsigned short*)readBuf)[0];
    //printk("dev = %d retbuf = %x\n",minor,temp);
   // m_iStatus = ((int)xil << 16) + (temp & 0xffff);
    if(minor == iDev2)
	m_iStatus = ((int)xil << 16) + (temp & 0xffff);
    else if (minor == iDev3)
	m_iZStatus = ((int)xil << 16) + (temp & 0xffff);

     ret = (ssize_t)( (int)xil << 16);

    return ret;
}

//write is ALWAYS synchronous
static ssize_t spi_link_write(struct file* file, const char* buf, size_t count, loff_t *offset)
{
    ssize_t ret;
    int bcount = 0;
    int writeCount =(count > BUFSIZE ? BUFSIZE : count);    
    int minor = *((int*)file->private_data);
    //printk("Module spi_link_write, iMinor = %d\n",minor );
    
    unsigned short xil = 0;

    if ((minor < iDev2) || (minor > iDev3))
    {
	printk("Module spi_link_write, return early\n");
	return 0;
    }
    
	
    copy_from_user(writeBuf,buf,writeCount);
    if(!xil_addr)
	xil_addr = xil_get_mapped_address();

    spi_access_bus(minor);
    	while(bcount < writeCount)
	{
	    //printk("TX[%d]: %d\n",bcount,writeBuf[bcount]);
	    //while(!(controller->status & SPSTATUS_RXREADY));
	    controller->TxD = writeBuf[bcount];
	    controller->control = SPCTRL_STR_ENABLE;	    
	    
	    while(!(controller->status & SPSTATUS_RXREADY));
	    readBuf[bcount] = controller->RxD;
	    bcount++;
	    //printk("TX[%d]: %d\n",bcount,writeBuf[bcount]);
	    controller->TxD = writeBuf[bcount];
	    controller->control = SPCTRL_STR_ENABLE;	    
	    
	    while(!(controller->status & SPSTATUS_RXREADY));
	    readBuf[bcount] = controller->RxD;
	    bcount++;
	    /*
	    //printk("TX[%d]: %d\n",bcount,writeBuf[bcount]);
	    controller->TxD = writeBuf[bcount];
	    controller->control = SPCTRL_STR_ENABLE;	    
	    
	    while(!(controller->status & SPSTATUS_RXREADY));
	    readBuf[bcount] = controller->RxD;
	    bcount++;
	    //printk("TX[%d]: %d\n",bcount,writeBuf[bcount]);
	    controller->TxD = writeBuf[bcount];
	    controller->control = SPCTRL_STR_ENABLE;	    
	    //ndelay(700*(current_speed+1)+current_speed*200);
	    //while(controller->status & SPSTATUS_BUSY);
	    while(!(controller->status & SPSTATUS_RXREADY));
	    readBuf[bcount] = controller->RxD;
	    bcount++;
	    */
	}
    
	spi_release_bus(minor);	
    

    if(minor == iDev2)
	xil = ConvertEndian(*((unsigned short*)xil_addr + XIL_STATUS_OFFSET));
    else if (minor == iDev3)
	xil = ConvertEndian(*((unsigned short*)xil_addr + XIL_Z_STATUS_OFFSET));
	//xil = *((unsigned short*)xil_addr + XIL_Z_STATUS_OFFSET);
    //printk("dev = %d  xil = %d\n",minor,xil);
    m_lastTxfer = ((unsigned short*)readBuf)[writeCount/2 -1];
    ret = (ssize_t)( ((int)xil << 16) + m_lastTxfer);
    //printk("\nwrite ret = %x\n",ret);
    
    if(minor == iDev2)
	m_iStatus = ret;
    else if (minor == iDev3)
	m_iZStatus = ret;
    return ret;
}

static int spi_link_open(struct inode* inode, struct file* file)
{
	int iDevCurrent = iminor(inode);
	MSG("Module spi_link open, iMinor = %d\n",iDevCurrent );
	//printk("Module spi_link open, iMinor = %d\n",iDevCurrent );
	if (iDevCurrent < 2 || iDevCurrent > 3)
		return -ENODEV;
	if (iDevCurrent == 2)
		file->private_data = &iDev2;
	else if (iDevCurrent ==3)
		file->private_data = &iDev3;
	//current_device = iDevCurrent;
	
	return 0;
}

static int spi_link_release(struct inode* inode, struct file* file)
{
    int minor = *((int*)file->private_data);
    //printk("Module spi_link_release, iMinor = %d\n",minor );
    //if(current_device !=-1)
    spi_release_bus(minor);
	
    return 0;
}

static long spi_link_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    
	struct inode* inode = file->f_path.dentry->d_inode;
    short* pBuf = (short*)0;
    int iMinor = iminor(inode);
    short* pReadBuf = (short*)readBuf;
    int bufSize = 0;
    u8 mode;
    unsigned char cdm;
    int scr;
    int i=0;
	/* Make sure the command belongs to us*/
    if (_IOC_TYPE(cmd) != SPI_IOCTL_BASE)
    	return -ENOTTY;
    //printk("Module spi_link_ioctl, iMinor = %d\n",minor );
    switch(cmd )
    {
    case SPI_GET_SERVO_STATUS:
	//the buffer size is passed in first, then the buffer location
	//the buffer size gives us the count of bytes sent which we can
	//then deduce the word that we want to send back.  We want to only
	//send back the last status.
	bufSize = *(int*)arg;
	pBuf = (short*) *(int*)(arg + sizeof(int));
	MSG("iMinor:%d, pReadBuf:0x%x, bufSize:%d\n",iMinor,(int)pReadBuf,bufSize);
	if (bufSize > BUFSIZE)
	    bufSize = BUFSIZE;
	i = copy_to_user(pBuf,&pReadBuf[(bufSize/2) -1],sizeof(short));
    	return sizeof(short) - i;
	break;
    case SPI_SET_SPI_MODE:
	//Set the bit mode to 8 or 16 bit, polarity,phase and baud rate
	//The calling application will have to know how to fill out this register.
	
	if (iMinor == 2)
	{
	    mode = arg;
	    controller->mode &= ~SPMODE_ENABLE;
	    controller->mode = mode;
	    controller->mode |= SPMODE_ENABLE;
	}
	else if (iMinor == 3)
	{
	    mode = arg;
	    controller->mode &= ~SPMODE_ENABLE;
	    controller->mode = mode;
	    controller->mode |= SPMODE_ENABLE;
	}
	return arg;
	break;
    case SPI_SET_SPEED:
	
	controller->mode &= ~SPMODE_ENABLE;
	
	cdm = 0;
	scr = opb_quarter_freq/((arg>opb_quarter_freq)?opb_quarter_freq:arg) - 1;
	cdm = scr & 0xff;
	current_speed = cdm;
	controller->clock = cdm;
	controller->mode |= SPMODE_ENABLE;
	//spi_access_bus(current_device);
	
	return arg;
	break;
	
    case SPI_GET_BUFFER_SIZE:
	return BUFSIZE;
	break;
    case SPI_SERVO_RESET:
	//printk("Servo Reset\n");
	chipselect->orr |= SERVO_RESET; //set it high initially;
	udelay(20);
	//here is the actual reset
	chipselect->orr &= ~SERVO_RESET;
	//give it enough time so that it will hit the reset.
	udelay(20);
	chipselect->orr |= SERVO_RESET;
	return 1;
	break;
    case SPI_SERVO_BUS:
	if (arg)//grabbing the bus
	{	
		bAcquiring = 1;
		spi_access_bus(arg);		
		bAcquiring = 0;		
	}
	break;
    case SPI_SERVO_BUS_CLEAR:
	if (arg)//grabbing the bus
	{
	    spi_release_bus(arg);
	}    
    case SPI_SERVO_READY:
	//return spi_ready(iMinor);
	break;
    case SPI_SERVO_STATUS:
	return m_iStatus;
	break;
    case SPI_Z_SERVO_STATUS:
	return m_iZStatus;
	break;
    case SPI_DISABLE:
	// Disable the peripheral pins make them IO
	printk("Disabling SPI\n");
	
	return 1;
	break;
	
	
    default:
    	return -ENOTTY;    	
    }
    return 0;
}

#ifdef ENABLE_INT
static irqreturn_t spi_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
    
    
    
}
#endif
/********************************************************************/
static struct file_operations spi_link_fops = {
owner:		THIS_MODULE,
read:		spi_link_read,
write:		spi_link_write,
unlocked_ioctl:	spi_link_ioctl,
open:		spi_link_open,
release:	spi_link_release,
};

static int __init synrad_spi_link_init_module(void)
{
	int res = 0;
#ifdef ENABLE_INT
	int irq = 8;
#endif
	unsigned long phys_addr;
	unsigned long end_addr;
	unsigned long base_len;
	unsigned char cdm;
	int scr;
	struct device_node* opb_node = NULL;
	u8 regval = SPMODE_ENABLE;
	xil_addr = NULL;
	MSG("Module synrad_spi_link init\n" );
	/* get the clock (Hz) for the OPB.*/
	opb_node = of_find_node_by_path("/plb/opb");
	if (!opb_node) {
		printk(KERN_ERR "(E) Flyer_Xilinx failed to find device tree node /plb/opb\n");
		return -1;
	}
	opb_quarter_freq = *(u32*)of_get_property(opb_node, "clock-frequency", NULL) >> 2;

	sema_init(&spi_lock,1);
	phys_addr = SPI_PHYS_START;
	end_addr = SPI_PHYS_END;
	base_len = end_addr - phys_addr + 1;
	
	controller = ioremap(phys_addr, base_len);
	if (!controller)
	{
	    printk(KERN_ERR "Module Synrad SPI ioremap FAILED\n");
	    return -ENXIO;
	}
	
	    // Get VM address for GPIO Chipselect
	phys_addr = GPIO_1_PHYS_START;
	end_addr = GPIO_1_PHYS_END;
	base_len = end_addr - phys_addr + 1;
	
	chipselect = ioremap(phys_addr, base_len);
	if (!chipselect)
	{
	printk(KERN_ERR "Synrad GPIO ioremap FAILED\n");
	goto out_unmap_controller;
        }
	
	readBuf = kmalloc(BUFSIZE, GFP_KERNEL);
	writeBuf = kmalloc(BUFSIZE, GFP_KERNEL);
	//spi_transfer_desc = kmalloc(sizeof(struct spi_transfer_list), GFP_KERNEL);
	//spi_dev = kmalloc(sizeof(struct spi_local), GFP_KERNEL);
	if (!readBuf || !writeBuf)
		return -ENOMEM;
	memset(readBuf,0,BUFSIZE);//clear the read buffer
	printk(KERN_DEBUG "readBuf:0x%x, writeBuf:0x%x\n",(int)readBuf,(int)writeBuf);
	/*register the device with the kernel*/	
	
	spi_dev[0].pcs = CS_0;
	spi_dev[1].pcs = CS_1;
    
	/* get the clock (Hz) for the OPB. Set in sequoia_setup_arch() */
	spi_dev[0].opb_freq = opb_quarter_freq;
	spi_dev[1].opb_freq = opb_quarter_freq;
    
	// Set up GPIO40 and GPIO41 for chip select active low
	
	chipselect->osrh = chipselect->osrh & ~TWOBIT32(CS_0_PIN);
	chipselect->tsrh = chipselect->tsrh & ~TWOBIT32(CS_0_PIN);
	chipselect->odr = chipselect->odr & ~CS_0;
	chipselect->tcr = chipselect->tcr | CS_0;
	
	chipselect->osrh = chipselect->osrh & ~TWOBIT32(CS_1_PIN);
	chipselect->tsrh = chipselect->tsrh & ~TWOBIT32(CS_1_PIN);
	chipselect->odr = chipselect->odr & ~CS_1;
	chipselect->tcr = chipselect->tcr | CS_1;
	
	// Set up GPIO42 for servo reset active low
	
	chipselect->osrh = chipselect->osrh & ~TWOBIT32(SERVO_RESET_PIN);
	chipselect->tsrh = chipselect->tsrh & ~TWOBIT32(SERVO_RESET_PIN);
	chipselect->odr = chipselect->odr & ~SERVO_RESET;
	chipselect->tcr = chipselect->tcr | SERVO_RESET;
	
	// Set them high
	chipselect->orr = chipselect->orr | CS_0;
	chipselect->orr = chipselect->orr | CS_1;
	chipselect->orr = chipselect->orr | SERVO_RESET;
    
	// Set up SPI default values
	// set the clock 
	cdm = 0;
	scr = (spi_dev[0].opb_freq/DEFAULT_CLK_HZ1) - 1;
	cdm = scr & 0xff;
	spi_dev[0].clock = cdm;
	current_speed = cdm;
	controller->clock = cdm;
	cdm = 0;
	scr = ((spi_dev[0].opb_freq/DEFAULT_CLK_HZ2) - 1);
	cdm = scr & 0xff;
	spi_dev[1].clock = cdm;
	
 	#ifdef ENABLE_INT
	res = request_irq(irq, spi_interrupt, 0,
			"amcc440ep_spi", NULL);	
	if (res)
	{
		MSG("Can't register device spi_link with kernel.\n");
		
		iounmap(chipselect);
		iounmap(controller);
		
		return res;
	}
	#endif
	
	// Set the mode and Enable
	controller->mode = regval;
	
	res = register_chrdev(SPI_LINK_MAJOR,"spi_link",&spi_link_fops);
	if (res)
	{
		MSG("Can't register device spi_link with kernel.\n");
		
		#ifdef ENABLE_INT
		free_irq(irq, 0);
		#endif
		iounmap(chipselect);
		iounmap(controller);
		
		return res;
	}
	 printk("Synrad SPI Registered 16-Sep-2010 -A\n");
	return 0;

out_unmap_controller:
    iounmap(controller);
	
    return 0;

}

static void __exit synrad_spi_link_exit_module(void)
{
	printk( KERN_DEBUG "Module synrad_spi_link exit\n" );
	printk("Module synrad_spi_link exit\n");
	//if (spi_transfer_desc)
		//kfree(spi_transfer_desc);
	if (readBuf)
	{
		kfree(readBuf);
	}
	if (writeBuf)
	{
		kfree(writeBuf);
	}
	
	iounmap(chipselect);
	iounmap(controller);
	
	/*unregister the device with the kernel*/	
	unregister_chrdev(SPI_LINK_MAJOR,"spi_link");
	
}

module_init(synrad_spi_link_init_module);
module_exit(synrad_spi_link_exit_module);


MODULE_DESCRIPTION("Module to communicate with the spi peripherals");
MODULE_AUTHOR("S. Saban (ssaban@synrad.com)");
MODULE_LICENSE("GPL");
