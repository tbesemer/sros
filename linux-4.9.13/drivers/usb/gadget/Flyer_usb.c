/*
 * zero.c -- Gadget Zero, for USB development
 *
 * Copyright (C) 2003-2004 David Brownell
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Copyright (C) 2006 Jeff Warren
 * All rights reserved.
 * Modified zero.c file for our own purposes and used under the GPL license
 */

/*
 * Flyer_usb is the driver that grabs the communications from WinMark and sends them to a specific file through 
 * the char driver interface.
 */

//#define DEBUG 1
//#define VERBOSE

//#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/wait.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/unaligned.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

//#include <asm/arch/AT91RM9200_EMAC.h>
//#include <asm/arch/at91_Flyer_Xilinx.h> //for the reads/writes to Xilinx
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include "gadget_chips.h"

#define URGENT_QUEUE_LEN 4

#define FLYER_MAJOR 243
#define FLYER_IOCTL_BASE 0xAE
#define FLYER_CONNECTED _IO(FLYER_IOCTL_BASE,0x20)
#define FLYER_INITIATE_CONNECT _IO(FLYER_IOCTL_BASE,0x21)

#define FLYER_PCIL0_BASE            0x00000000ef400000ULL
#define FLYER_PCIL0_SIZE            0x40

#define FLYER_PCIL0_PMM0PCIHA       0x00C

#define PCI_READL(offset)				\
        (readl((void *)((u32)pci_reg_base+offset)))


static int iDevMain = 0;
static int iDevUrgent = 1;
int g_bUSBConnected = 0;
static int g_Connected = 0; //this is the var for connecting the D+ pullup pin.  It needs to be off on bootup
extern int g_bEthConnected;//from ibm_emac_core.h

struct proc_dir_entry   *g_flyer_pde = NULL;

static ssize_t at91_flyer_read(struct file* file, char* buf, size_t count, loff_t *offset);
static ssize_t at91_flyer_write(struct file* file, const char* buf, size_t count, loff_t *offset);
static int     at91_flyer_open(struct inode* inode, struct file* file);
static int     at91_flyer_release(struct inode* inode, struct file* file);
static long     at91_flyer_ioctl(struct file* file, unsigned int cmd, unsigned long arg);
static unsigned int at91_flyer_poll(struct file* file, poll_table* wait);
/* circular buffer */
struct gs_buf {
	unsigned int		buf_size;
	char			*buf_buf;
	char			*buf_get;
	char			*buf_put;
	int 			count;
	struct usb_request      **pUrb;
	struct usb_ep		*ep;
};

/* struct for urgent buffer received */
struct g_urg_queue
{
    int 		count;
    int			listening;
    struct usb_request *pBuf[URGENT_QUEUE_LEN];
};
    
static struct gs_buf *gs_buf_alloc(unsigned int size, int kmalloc_flags);
void gs_buf_free(struct gs_buf *gb);
void gs_buf_clear(struct gs_buf *gb);
unsigned int gs_buf_requeue(struct gs_buf *gb,struct usb_ep *ep, struct usb_request *req);
unsigned int gs_buf_clear_hold(struct gs_buf *gb);
unsigned int gs_buf_data_avail(struct gs_buf *gb);
unsigned int gs_buf_space_avail(struct gs_buf *gb);
unsigned int gs_buf_put(struct gs_buf *gb, const char *buf, unsigned int count);
unsigned int gs_buf_get(struct gs_buf *gb, char *buf, unsigned int count);

/*-------------------------------------------------------------------------*/

#define DRIVER_VERSION		"April 2013"

static const char shortname[] = "flyer_usb";
static const char longname[] = "Flyer USB Driver";

static const char flyer_standard[] = "Flyer USB device";
static const char flyer_main[] = "Flyer Main Data";
static const char flyer_urgent[] = "Flyer Urgent Data";


/*-------------------------------------------------------------------------*/

/*
 * driver assumes self-powered hardware, and
 * has no way for users to trigger remote wakeup.
 *
 * this version autoconfigures as much as possible,
 * which is reasonable for most "bulk-only" drivers.
 */

static const char *EP_IN_MAIN_NAME;		/* source */
static const char *EP_OUT_MAIN_NAME;		/* sink */
static const char *EP_IN_URGENT_NAME;		/* source */
static const char *EP_OUT_URGENT_NAME;		/* sink */

/*-------------------------------------------------------------------------*/

/* big enough to hold our biggest descriptor */
#define USB_BUFSIZ	256
/*16 packets * 1536 = 24Kb*/
//#define PACKET_SIZE     4096
#define PACKET_SIZE     1536 
//#define PACKET_SIZE     1024

struct flyer_dev {
	spinlock_t		lock;
	struct usb_gadget	*gadget;
	struct usb_request	*req;		/* for control responses */

	/* when configured, we have one of two configs:
	 * - source data (in to host) and sink it (out from host)
	 * - or loop it back (out from host back in to host)
	 */
	u8			config;
	struct usb_ep		*in_main_ep, *out_main_ep, *in_urgent_ep, *out_urgent_ep;
	struct gs_buf		*main_buf;//this is for incoming, outgoing will be queued immediately
	struct g_urg_queue      urg_queue;
	/* autoresume timer */	
	wait_queue_head_t	wait;
	struct timer_list	resume;
};

#define xprintk(d,level,fmt,args...) \
	dev_printk(level , &(d)->gadget->dev , fmt , ## args)

#ifdef DEBUG
#define DBG(dev,fmt,args...) \
	xprintk(dev , KERN_INFO , fmt , ## args)
#else
#define DBG(dev,fmt,args...) \
	do { } while (0)
#endif /* DEBUG */

#ifdef VERBOSE
#define VDBG	DBG
#else
#define VDBG(dev,fmt,args...) \
	do { } while (0)
#endif /* VERBOSE */

#define ERROR(dev,fmt,args...) \
	xprintk(dev , KERN_ERR , fmt , ## args)
#define INFO(dev,fmt,args...) \
	xprintk(dev , KERN_INFO , fmt , ## args)

/*-------------------------------------------------------------------------*/
	    
/*The dev global pointer is used by the char driver side to (push/get) data (on to /off of) the queues/buffers*/
static struct flyer_dev* pFlyerDev = NULL;
	
//static unsigned buflen = PAGE_SIZE*16; //64k
static unsigned buflen = PAGE_SIZE*128; //64k
static unsigned qlen = 16;
module_param (buflen, uint, S_IRUGO|S_IWUSR);
module_param (qlen, uint, S_IRUGO|S_IWUSR);

/*
 * if it's nonzero, autoresume says how many seconds to wait
 * before trying to wake up the host after suspend.
 */
static unsigned autoresume = 0;
module_param (autoresume, uint, 0);

/*-------------------------------------------------------------------------*/

/* These Driver ID's have not been registered with the governing USB body.
 * I think this is fine as we never registered our PCI card VENDOR and ID either.
 */

#define DRIVER_VENDOR_NUM	0x7967		/* "Synr"ad  on the phone */
//#define DRIVER_PRODUCT_NUM	0x3593		/* "Flye"r on the phone */
#define DRIVER_PRODUCT_NUM	0x3473		/* "Dire"on the phone for Flyer 3D and FlyerII*/

/*-------------------------------------------------------------------------*/

/*
 * DESCRIPTORS ... most are static, but strings and (full)
 * configuration descriptors are built on demand.
 */

#define STRING_MANUFACTURER		25
#define STRING_PRODUCT			42
#define STRING_SERIAL			101
#define MAIN_STRING_CONFIG		102
#define URGENT_STRING_CONFIG		103
#define STRING_CONFIG   		250


/*
 * This device advertises one configurations; 
 */
#define	CONFIG_FLYER	1
#define MAIN_INTERFACE_ID	0
#define URGENT_INTERFACE_ID	1

static struct usb_device_descriptor
device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	//.bcdUSB =		__constant_cpu_to_le16 (0x0200),
	.bcdUSB =		__constant_cpu_to_le16 (0x0110),
	.bDeviceClass =		USB_CLASS_VENDOR_SPEC,

	.idVendor =		__constant_cpu_to_le16 (DRIVER_VENDOR_NUM),
	.idProduct =		__constant_cpu_to_le16 (DRIVER_PRODUCT_NUM),
	.iManufacturer =	STRING_MANUFACTURER,
	.iProduct =		STRING_PRODUCT,
	.iSerialNumber =	STRING_SERIAL,
	.bNumConfigurations =	1,
};

static struct usb_config_descriptor
full_speed_config = {
	.bLength =		sizeof full_speed_config,
	.bDescriptorType =	USB_DT_CONFIG,

	/* compute wTotalLength on the fly */
	.bNumInterfaces =	2,
	.bConfigurationValue =	CONFIG_FLYER,
	.iConfiguration =	STRING_CONFIG,
	.bmAttributes =		USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower =		1,	/* self-powered */
};

static struct usb_otg_descriptor
otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	.bmAttributes =		USB_OTG_SRP,
};

static const struct usb_interface_descriptor
main_intf = {
	.bLength =		sizeof main_intf,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	MAIN_INTERFACE_ID,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.iInterface =		MAIN_STRING_CONFIG,
};

static const struct usb_interface_descriptor
urgent_intf = {
	.bLength =		sizeof urgent_intf,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	URGENT_INTERFACE_ID,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.iInterface =		URGENT_STRING_CONFIG,
};

/* four full speed bulk endpoints; their use is config-dependent */

static struct usb_endpoint_descriptor
fs_source_main_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,	
};

static struct usb_endpoint_descriptor
fs_sink_main_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,	
};

static struct usb_endpoint_descriptor
fs_source_urgent_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,	
};

static struct usb_endpoint_descriptor
fs_sink_urgent_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,	
};

static const struct usb_descriptor_header *fs_standard_function [] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	(struct usb_descriptor_header *) &main_intf,
	(struct usb_descriptor_header *) &fs_source_main_desc,
	(struct usb_descriptor_header *) &fs_sink_main_desc,
	(struct usb_descriptor_header *) &urgent_intf,
	(struct usb_descriptor_header *) &fs_source_urgent_desc,
	(struct usb_descriptor_header *) &fs_sink_urgent_desc,
	NULL,
};

#ifdef	CONFIG_USB_GADGET_DUALSPEED

/*
 * usb 2.0 devices need to expose both high speed and full speed
 * descriptors, unless they only run at full speed.
 *
 * that means alternate endpoint descriptors (bigger packets)
 * and a "device qualifier" ... plus more construction options
 * for the config descriptor.
 */

static struct usb_endpoint_descriptor
hs_source_main_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16 (512),
};

static struct usb_endpoint_descriptor
hs_sink_main_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16 (512),
};

static struct usb_endpoint_descriptor
hs_source_urgent_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16 (512),
};

static struct usb_endpoint_descriptor
hs_sink_urgent_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16 (512),
};

static struct usb_qualifier_descriptor
dev_qualifier = {
	.bLength =		sizeof dev_qualifier,
	.bDescriptorType =	USB_DT_DEVICE_QUALIFIER,

	//.bcdUSB =		__constant_cpu_to_le16 (0x0200),
	.bcdUSB =		__constant_cpu_to_le16 (0x0110),
	.bDeviceClass =		USB_CLASS_VENDOR_SPEC,

	.bNumConfigurations =	1,
};

static const struct usb_descriptor_header *hs_standard_function [] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	(struct usb_descriptor_header *) &main_intf,
	(struct usb_descriptor_header *) &hs_source_main_desc,
	(struct usb_descriptor_header *) &hs_sink_main_desc,
	(struct usb_descriptor_header *) &urgent_intf,
	(struct usb_descriptor_header *) &hs_source_urgent_desc,
	(struct usb_descriptor_header *) &hs_sink_urgent_desc,
	NULL,
};

// maxpacket and other transfer characteristics vary by speed. 
#define ep_desc(g,hs,fs) (((g)->speed==USB_SPEED_HIGH)?(hs):(fs))

#else

/* if there's no high speed support, maxpacket doesn't change. */
#define ep_desc(g,hs,fs) fs

#endif	/* !CONFIG_USB_GADGET_DUALSPEED */

static char				manufacturer [50];
static char				serial [40];

/* static strings, in UTF-8 */
static struct usb_string		strings [] = {
	{ STRING_MANUFACTURER, manufacturer, },
	{ STRING_PRODUCT, longname, },
	{ STRING_SERIAL, serial, },
	{ STRING_CONFIG, flyer_standard, },
	{ MAIN_STRING_CONFIG, flyer_main, },
	{ URGENT_STRING_CONFIG, flyer_urgent, },
	{  }			/* end of list */
};

static struct usb_gadget_strings	stringtab = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings,
};

/*
 * config descriptors are also handcrafted.  these must agree with code
 * that sets configurations, and with code managing interfaces and their
 * altsettings.  other complexity may come from:
 *
 *  - high speed support, including "other speed config" rules
 *  - multiple configurations
 *  - interfaces with alternate settings
 *  - embedded class or vendor-specific descriptors
 *
 * this handles high speed, and has a second config that could as easily
 * have been an alternate interface setting (on most hardware).
 *
 * NOTE:  to demonstrate (and test) more USB capabilities, this driver
 * should include an altsetting to test interrupt transfers, including
 * high bandwidth modes at high speed.  (Maybe work like Intel's test
 * device?)
 */
static int
config_buf (struct usb_gadget *gadget, u8 *buf, u8 type, unsigned index)
{
	int				len;
	const struct usb_descriptor_header **function;
	
	//printk("Config_buf\n");

#ifdef CONFIG_USB_GADGET_DUALSPEED
	int				hs = (gadget->speed == USB_SPEED_HIGH);
#endif

	/* two configurations will always be index 0 and index 1 */
	if (index > 1)
		return -EINVAL;
	
#ifdef CONFIG_USB_GADGET_DUALSPEED
	if (type == USB_DT_OTHER_SPEED_CONFIG)
		hs = !hs;
	if (hs)
		function = hs_standard_function;
	else
#endif
	
		function = fs_standard_function;

	/* for now, don't advertise srp-only devices */
	if (!gadget->is_otg)
		function++;

	len = usb_gadget_config_buf (&full_speed_config,buf, USB_BUFSIZ, function);
	if (len < 0)
		return len;
	((struct usb_config_descriptor *) buf)->bDescriptorType = type;
	return len;
}

/*-------------------------------------------------------------------------*/

static struct usb_request *alloc_ep_req (struct usb_ep *ep, unsigned length)
{
	struct usb_request	*req;

	req = usb_ep_alloc_request (ep, GFP_ATOMIC);
	if (req) {
		req->length = length;
		req->buf = kmalloc(length, GFP_ATOMIC);
		if (!req->buf) {
			usb_ep_free_request (ep, req);
			req = NULL;
		}
	}
	return req;
}

static void free_ep_req (struct usb_ep *ep, struct usb_request *req)
{
	if (req->buf)
		kfree(req->buf);
	usb_ep_free_request (ep, req);
}

/*-------------------------------------------------------------------------*/

/* if there is only one request in the queue, there'll always be an
 * irq delay between end of one request and start of the next.
 * that prevents using hardware dma queues.
 */
static void flyer_main_complete (struct usb_ep *ep, struct usb_request *req)
{
    struct flyer_dev	*dev = ep->driver_data;
    int	status = req->status;
    int ret = 0;
    switch (status) 
    {
	
	case 0: 			/* normal completion? */
	if (ep == dev->out_main_ep)
	{
	    //received some vector data, place it in the right place...
	    //printk (KERN_ERR "Got main data size:%d\n",req->actual);
	    do
	    {
		ret = gs_buf_put(dev->main_buf, req->buf,req->actual);
		if (!ret)
		    INFO (pFlyerDev, "gs_buf_put failed\n");
		wake_up_interruptible(&dev->wait);    
	    }
	    while(!ret);
	    
	    status = gs_buf_requeue(dev->main_buf,ep, req);
	    if (status == 0)
		return;
	}
	else if (ep == dev->in_main_ep)
	{
	    //sent something over the main data line...	    
	    free_ep_req (ep, req); //free the request
	    return;
	}
	break;
	
	/* this endpoint is normally active while we're configured */
	case -ECONNABORTED: 		/* hardware forced ep reset */
	case -ECONNRESET:		/* request dequeued */
	case -ESHUTDOWN:		/* disconnect from host */
	VDBG (dev, "%s gone (%d), %d/%d\n", ep->name, status,
	      req->actual, req->length);
	
	free_ep_req (ep, req);
	return;
	
	case -EOVERFLOW:		/* buffer overrun on read means that
					 * we didn't provide a big enough
					 * buffer.
					 */
	default:
#if 1
	DBG (dev, "%s complete --> %d, %d/%d\n", ep->name,
	     status, req->actual, req->length);
#endif
	case -EREMOTEIO:		/* short read */
	break;
    }
    
    status = usb_ep_queue (ep, req, GFP_ATOMIC);
    if (status) {
	ERROR (dev, "kill %s:  resubmit %d bytes --> %d\n",
	       ep->name, req->length, status);
	usb_ep_set_halt (ep);
	/* FIXME recover later ... somehow */
    }
}

/* if there is only one request in the queue, there'll always be an
 * irq delay between end of one request and start of the next.
 * that prevents using hardware dma queues.
 */
static void flyer_urgent_complete (struct usb_ep *ep, struct usb_request *req)
{
    struct flyer_dev	*dev = ep->driver_data;
    int		status = req->status;
    
    switch (status) 
    {
	
	case 0: 			/* normal completion? */
	if (ep == dev->out_urgent_ep)
	{
	    //received some urgent data, probably a response to something we sent
	    //this leaves the urb out of the queue until it is read instead of using a circular buffer.
	    //printk (KERN_ERR "Got urgent data  size:%d\n",req->actual);
	    if (dev->urg_queue.listening)
	    {
		dev->urg_queue.pBuf[dev->urg_queue.count] = req;
		dev->urg_queue.count ++;
		if (dev->urg_queue.count > URGENT_QUEUE_LEN)
		{
		    printk(KERN_ERR "count is :%d in flyer_urgent_complete!!!\n",dev->urg_queue.count);
		    dev->urg_queue.count = URGENT_QUEUE_LEN;
		}
		wake_up_interruptible(&dev->wait);
	    }
	    else
	    {
		status = usb_ep_queue (ep, req, GFP_ATOMIC);
	    }
	    return;
	}
	else if (ep == dev->in_urgent_ep)
	{
	    //send something over the urgent data line...
	    free_ep_req (ep, req); //free the request
	    return;
	}
	break;
	
	/* this endpoint is normally active while we're configured */
	case -ECONNABORTED: 		/* hardware forced ep reset */
	case -ECONNRESET:		/* request dequeued */
	case -ESHUTDOWN:		/* disconnect from host */
	VDBG (dev, "%s gone (%d), %d/%d\n", ep->name, status,
	      req->actual, req->length);
	
	free_ep_req (ep, req);
	return;
	
	case -EOVERFLOW:		/* buffer overrun on read means that
					 * we didn't provide a big enough
					 * buffer.
					 */
	default:
#if 1
	DBG (dev, "%s complete --> %d, %d/%d\n", ep->name,
	     status, req->actual, req->length);
#endif
	case -EREMOTEIO:		/* short read */
	break;
    }
    
    status = usb_ep_queue (ep, req, GFP_ATOMIC);
    if (status) {
	ERROR (dev, "kill %s:  resubmit %d bytes --> %d\n",
	       ep->name, req->length, status);
	usb_ep_set_halt (ep);
	/* FIXME recover later ... somehow */
    }
}

static int set_flyer_config (struct flyer_dev *dev, int gfp_flags)
{
	int			result = 0;
	struct usb_ep		*ep;
	struct usb_gadget	*gadget = dev->gadget;

	gadget_for_each_ep (ep, gadget) 
	{
		/* two endpoints write to the host */
		if (strcmp (ep->name, EP_IN_MAIN_NAME) == 0) 
		{
		    result = usb_ep_enable (ep);
		    if (result == 0) 
		    {
			ep->driver_data = dev;
			dev->in_main_ep = ep;
		        continue;
		    }
		}
		else if (strcmp (ep->name, EP_IN_URGENT_NAME) == 0) 
		{
		    result = usb_ep_enable (ep);
		    if (result == 0) 
		    {
			ep->driver_data = dev;
			dev->in_urgent_ep = ep;
			continue;
		    }
		}	

		/* two endpoints read from the host */
		
		else if (strcmp (ep->name, EP_OUT_MAIN_NAME) == 0) 
		{
		    result = usb_ep_enable (ep);
		    if (result == 0) 
		    {
			ep->driver_data = dev;
			dev->out_main_ep = ep;
			//make sure the buffer for out_main_ep has the ep it is attached too...
			dev->main_buf->ep = dev->out_main_ep;
			continue;
		    }
		    
		    /* ignore any other endpoints */
		} 
		else if (strcmp (ep->name, EP_OUT_URGENT_NAME) == 0) 
		{
		    result = usb_ep_enable (ep);
		    if (result == 0) 
		    {
			ep->driver_data = dev;
			dev->out_urgent_ep = ep;
			continue;
		    }
		    
		    /* ignore any other endpoints */
		} 
		else
		    continue;

		/* stop on error */
		ERROR (dev, "can't start %s, result %d\n", ep->name, result);
		break;
	}
	if (result == 0)
		DBG (dev, "buflen %d\n", buflen);

	/* allocate a bunch of read buffers and queue them all at once for the incoming vector data.
	 * and qlen/8 for the incoming urgent data.
	 * we buffer at most 'qlen' transfers; fewer if any need more
	 * than 'buflen' bytes each.
	 */
	if (result == 0) 
	{
	    struct usb_request	*req;
	    unsigned		i;
		    
	    ep = dev->out_main_ep;
	    for (i = 0; i < qlen && result == 0; i++) 
	    {
		req = alloc_ep_req (ep, PACKET_SIZE);
		if (req) 
		{
		    req->complete = flyer_main_complete;
		    result = usb_ep_queue (ep, req, GFP_ATOMIC);
		    if (result)
			DBG (dev, "%s queue req --> %d\n",
			     ep->name, result);
		} else
		    result = -ENOMEM;
	    }
	    //for the urgent queue, just do qlen/8 or default to 4 in the queue, this will be enough
	    ep = dev->out_urgent_ep;
	    for (i = 0; i < URGENT_QUEUE_LEN && result == 0; i++) 
	    {
		req = alloc_ep_req (ep, ep->maxpacket);
		if (req) 
		{
		    req->complete = flyer_urgent_complete;
		    result = usb_ep_queue (ep, req, GFP_ATOMIC);
		    if (result)
			DBG (dev, "%s queue req --> %d\n",
			     ep->name, result);
		} else
		    result = -ENOMEM;
	    }
	}
	if (result == 0)
		DBG (dev, "qlen %d, buflen %d\n", qlen, buflen);
	/* caller is responsible for cleanup on error */
	return result;
}

/*-------------------------------------------------------------------------*/

static void flyer_reset_config (struct flyer_dev *dev)
{
    if (dev->config == 0)
	return;
    
    DBG (dev, "reset config\n");
    
    /* just disable endpoints, forcing completion of pending i/o.
     * all our completion handlers free their requests in this case.
     */    
    if (dev->in_main_ep) 
    {
	usb_ep_disable (dev->in_main_ep);
	dev->in_main_ep = NULL;
    }
    if (dev->out_main_ep) 
    {
	usb_ep_disable (dev->out_main_ep);
	dev->out_main_ep = NULL;
    }
    if (dev->in_urgent_ep) 
    {
	usb_ep_disable (dev->in_urgent_ep);
	dev->in_urgent_ep = NULL;
    }
    if (dev->out_urgent_ep) 
    {
	usb_ep_disable (dev->out_urgent_ep);
	dev->out_urgent_ep = NULL;
    }
    dev->config = 0;
    del_timer (&dev->resume);
}

/* change our operational config.  this code must agree with the code
 * that returns config descriptors, and altsetting code.
 *
 * it's also responsible for power management interactions. some
 * configurations might not work with our current power sources.
 *
 * note that some device controller hardware will constrain what this
 * code can do, perhaps by disallowing more than one configuration or
 * by limiting configuration choices (like the pxa2xx).
 */
static int flyer_set_config (struct flyer_dev *dev, unsigned number, int gfp_flags)
{
	int			result = 0;
	struct usb_gadget	*gadget = dev->gadget;
	
	//printk("Flyer_Set_Config--Enter\n");

	if (number == dev->config)
	{
	        printk("flyer_set_config err: number %d != dev->config %d\n",number, dev->config);
		return 0;
	}

	if (gadget_is_sa1100 (gadget) && dev->config) 
	{
		/* tx fifo is full, but we can't clear it...*/
		INFO (dev, "can't change configurations\n");
		return -ESPIPE;
	}
	flyer_reset_config (dev);

	switch (number) 
	{
	case CONFIG_FLYER:
		result = set_flyer_config (dev, gfp_flags);
		break;
	default:
		result = -EINVAL;
		/* FALL THROUGH */
	case 0:
		return result;
	}
	 //printk("flyer_set_config: result %d\n",result);
	if (!result && (!dev->in_main_ep || !dev->out_main_ep || !dev->in_urgent_ep || !dev->out_urgent_ep))
        {
	        //printk("flyer_set_config err: result %d\n",result);
		result = -ENODEV;
	}
	if (result)
		flyer_reset_config (dev);
	else {
		char *speed;

		switch (gadget->speed) {
		case USB_SPEED_LOW:	speed = "low"; break;
		case USB_SPEED_FULL:	speed = "full"; break;
		case USB_SPEED_HIGH:	speed = "high"; break;
		default: 		speed = "?"; break;
		}

		dev->config = number;
		INFO (dev, "%s speed config #%d: %s\n", speed, number,flyer_standard);
		g_bUSBConnected = 1;
		
	}
	return result;
}

/*-------------------------------------------------------------------------*/

static void flyer_setup_complete (struct usb_ep *ep, struct usb_request *req)
{
	if (req->status || req->actual != req->length)
		DBG ((struct flyer_dev *) ep->driver_data,
				"setup complete --> %d, %d/%d\n",
				req->status, req->actual, req->length);
}

/*
 * The setup() callback implements all the ep0 functionality that's
 * not handled lower down, in hardware or the hardware driver (like
 * device and endpoint feature flags, and their status).  It's all
 * housekeeping for the gadget function we're implementing.  Most of
 * the work is in config-specific setup.
 */
static int flyer_setup (struct usb_gadget *gadget, const struct usb_ctrlrequest *ctrl)
{
	struct flyer_dev	*dev = get_gadget_data (gadget);
	struct usb_request	*req = dev->req;
	int			value = -EOPNOTSUPP;
	u16 w_index = le16_to_cpu(ctrl->wIndex);
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 w_length = le16_to_cpu(ctrl->wLength);

	/* usually this stores reply data in the pre-allocated ep0 buffer,
	 * but config change events will reconfigure hardware.
	 */
	//printk("Flyer_Setup -Entered   ctrl->bRequest = %d w_value = %d  w_length = %d\n", ctrl->bRequest, w_value, w_length);
	req->zero = 0;
	switch (ctrl->bRequest) {
	    
	case USB_REQ_GET_DESCRIPTOR:
	        //printk("Flyer_Setup -USB_REQ_GET_DESCRIPTOR\n");
		if (ctrl->bRequestType != USB_DIR_IN)
			goto unknown;
		
		switch (w_value >> 8) {

		case USB_DT_DEVICE:
		        //printk("Flyer_Setup -USB_DT_DEVICE\n");
			value = min (w_length, (u16) sizeof device_desc);
			memcpy (req->buf, &device_desc, value);
			break;

#ifdef CONFIG_USB_GADGET_DUALSPEED
		case USB_DT_DEVICE_QUALIFIER:
			if (!gadget->is_dualspeed)
				break;
			value = min (w_length, (u16) sizeof dev_qualifier);
			memcpy (req->buf, &dev_qualifier, value);
			break;

		case USB_DT_OTHER_SPEED_CONFIG:
			if (!gadget->is_dualspeed)
				break;
			// FALLTHROUGH
#endif

		case USB_DT_CONFIG:
			//printk("Flyer_Setup -USB_DT_CONFIG\n");
			value = config_buf (gadget, req->buf,
					w_value >> 8,
					w_value & 0xff);
			if (value >= 0)
				value = min (w_length, (u16) value);
			break;

		case USB_DT_STRING:
			//printk("Flyer_Setup -USB_DT_STRING\n");
			/* wIndex == language code.
			 * this driver only handles one language, you can
			 * add string tables for other languages, using
			 * any UTF-8 characters
			 */
			value = usb_gadget_get_string (&stringtab,
					w_value & 0xff, req->buf);
			if (value >= 0)
				value = min (w_length, (u16) value);
			break;
		default:
			printk("Flyer_Setup -Unknown DT: %x\n", w_value);
		    }
		break;

	/* currently two configs, two speeds */
	case USB_REQ_SET_CONFIGURATION:
		//printk("Flyer_Setup -USB_REQ_SET_CONFIGURATION\n");
		if (ctrl->bRequestType != 0)
		{
			
			goto unknown;
		}
		if (gadget->a_hnp_support)
			DBG (dev, "HNP available\n");
		else if (gadget->a_alt_hnp_support)
			DBG (dev, "HNP needs a different root port\n");
		else
			VDBG (dev, "HNP inactive\n");
		spin_lock (&dev->lock);
		value = flyer_set_config (dev, w_value, GFP_ATOMIC);
		spin_unlock (&dev->lock);
		break;
	case USB_REQ_GET_CONFIGURATION:
		//printk("Flyer_Setup -USB_REQ_GET_CONFIGURATION\n");
		if (ctrl->bRequestType != USB_DIR_IN)
			goto unknown;
		*(u8 *)req->buf = dev->config;
		value = min (w_length, (u16) 1);
		break;

	/* until we add altsetting support, or other interfaces,
	 * only 0/0 are possible.  pxa2xx only supports 0/0 (poorly)
	 * and already killed pending endpoint I/O.
	 */
	case USB_REQ_SET_INTERFACE:
		//printk("Flyer_Setup -USB_REQ_SET_INTERFACE\n");
		if (ctrl->bRequestType != USB_RECIP_INTERFACE)
			goto unknown;
		spin_lock (&dev->lock);
		if (dev->config && w_index == 0 && w_value == 0) {
			u8		config = dev->config;

			/* resets interface configuration, forgets about
			 * previous transaction state (queued bufs, etc)
			 * and re-inits endpoint state (toggle etc)
			 * no response queued, just zero status == success.
			 * if we had more than one interface we couldn't
			 * use this "reset the config" shortcut.
			 */
			flyer_reset_config (dev);
			flyer_set_config (dev, config, GFP_ATOMIC);
			value = 0;
		}
		spin_unlock (&dev->lock);
		break;
	case USB_REQ_GET_INTERFACE:
		//printk("Flyer_Setup -USB_REQ_GET_INTERFACE\n");
		if (ctrl->bRequestType != (USB_DIR_IN|USB_RECIP_INTERFACE))
			goto unknown;
		if (!dev->config)
			break;
		if (w_index != 0) {
			value = -EDOM;
			break;
		}
		*(u8 *)req->buf = 0;
		value = min (w_length, (u16) 1);
		break;

	/*
	 * These are the same vendor-specific requests supported by
	 * Intel's USB 2.0 compliance test devices.  We exceed that
	 * device spec by allowing multiple-packet requests.
	 */
	case 0x5b:	/* control WRITE test -- fill the buffer */
		if (ctrl->bRequestType != (USB_DIR_OUT|USB_TYPE_VENDOR))
			goto unknown;
		if (w_value || w_index)
			break;
		/* just read that many bytes into the buffer */
		if (w_length > USB_BUFSIZ)
			break;
		value = w_length;
		break;
	case 0x5c:	/* control READ test -- return the buffer */
		if (ctrl->bRequestType != (USB_DIR_IN|USB_TYPE_VENDOR))
			goto unknown;
		if (w_value || w_index)
			break;
		/* expect those bytes are still in the buffer; send back */
		if (w_length > USB_BUFSIZ
				|| w_length != req->length)
			break;
		value = w_length;
		break;

	default:
unknown:
		//printk("Flyer_Setup -unknown\n");
		VDBG (dev,
			"unknown control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	/* respond with data transfer before status phase? */
	if (value >= 0) {
		req->length = value;
		req->zero = value < w_length;
		value = usb_ep_queue (gadget->ep0, req, GFP_ATOMIC);
		if (value < 0) {
			DBG (dev, "ep_queue --> %d\n", value);
			req->status = 0;
			flyer_setup_complete (gadget->ep0, req);
		}
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

static void
flyer_disconnect (struct usb_gadget *gadget)
{
	struct flyer_dev	*dev = get_gadget_data (gadget);
	unsigned long		flags;

	spin_lock_irqsave (&dev->lock, flags);
	flyer_reset_config (dev);

	/* a more significant application might have some non-usb
	 * activities to quiesce here, saving resources like power
	 * or pushing the notification up a network stack.
	 */
	spin_unlock_irqrestore (&dev->lock, flags);

	/* next we may get setup() calls to enumerate new connections;
	 * or an unbind() during shutdown (including removing module).
	 */
	INFO(dev,"Flyer disconnected sbs\n");
	g_bUSBConnected = 0;
	
}

static void
flyer_autoresume (unsigned long _dev)
{
	struct flyer_dev*dev = (struct flyer_dev *) _dev;
	int		status;

	/* normally the host would be woken up for something
	 * more significant than just a timer firing...
	 */
	if (dev->gadget->speed != USB_SPEED_UNKNOWN) {
		status = usb_gadget_wakeup (dev->gadget);
		DBG (dev, "wakeup --> %d\n", status);
	}
}

/*-------------------------------------------------------------------------*/

static void
flyer_unbind (struct usb_gadget *gadget)
{
    struct flyer_dev	*dev = get_gadget_data (gadget);
    
    DBG (dev, "unbind\n");
    
    /* we've already been disconnected ... no i/o is active */
    if (dev->req)
	free_ep_req (gadget->ep0, dev->req);
    del_timer_sync (&dev->resume);
    gs_buf_clear(dev->main_buf);
    gs_buf_free(dev->main_buf);
    dev->urg_queue.count = 0;
    pFlyerDev = NULL;
    wake_up_interruptible(&dev->wait);
    kfree (dev);
    
    set_gadget_data (gadget, NULL);
}

static int
flyer_bind (struct usb_gadget *gadget, struct usb_gadget_driver* driver)
{
    struct usb_endpoint_descriptor* ued[4] = {&fs_source_main_desc,&fs_sink_main_desc,&fs_source_urgent_desc,
					      &fs_sink_urgent_desc};
    struct flyer_dev	*dev;
    struct usb_ep		*ep;
    int i=0;
    const char** names[4] = {&EP_IN_MAIN_NAME,&EP_OUT_MAIN_NAME,&EP_IN_URGENT_NAME,&EP_OUT_URGENT_NAME};
    /* Bulk-only drivers like this one SHOULD be able to
	 * autoconfigure on any sane usb controller driver,
	 * but there may also be important quirks to address.
	 */
    usb_ep_autoconfig_reset (gadget);
    for (i=0;i<4;i++)
    {
	ep = usb_ep_autoconfig (gadget, ued[i]);
	if (!ep) 
	{
	    printk (KERN_ERR "%s: can't autoconfigure on %s\n",
		    shortname, gadget->name);
	    return -ENODEV;
	}

	*names[i] = ep->name; 
	ep->driver_data = ep;	/* claim */
    }
	/*
	 * DRIVER POLICY CHOICE:  you may want to do this differently.
	 * One thing to avoid is reusing a bcdDevice revision code
	 * with different host-visible configurations or behavior
	 * restrictions -- using ep1in/ep2out vs ep1out/ep3in, etc
	 */
    if (gadget_is_fsl_usb2(gadget)) 
    {
	device_desc.bcdDevice = __constant_cpu_to_le16 (0x0218);
    } 
    else 
    {
	/* gadget zero is so simple (for now, no altsettings) that
	 * it SHOULD NOT have problems with bulk-capable hardware.
	 * so warn about unrcognized controllers, don't panic.
	 *
	 * things like configuration and altsetting numbering
	 * can need hardware-specific attention though.
	 */
	printk (KERN_WARNING "%s: controller '%s' not recognized\n",
		shortname, gadget->name);
	device_desc.bcdDevice = __constant_cpu_to_le16 (0x9999);
    }
    
    
    /* ok, we made sense of the hardware ... */
    dev = kmalloc (sizeof *dev, GFP_KERNEL);
    if (!dev)
	return -ENOMEM;
    memset (dev, 0, sizeof *dev);
    spin_lock_init (&dev->lock);
    //initialize the buffers...
    dev->main_buf = gs_buf_alloc(buflen, GFP_KERNEL);
    gs_buf_clear(dev->main_buf);
    dev->urg_queue.count = 0;
    if (!dev->main_buf )
	goto enomem;
    init_waitqueue_head(&dev->wait);
    dev->gadget = gadget;
    set_gadget_data (gadget, dev);
    
    /* preallocate control response and buffer */
    dev->req = usb_ep_alloc_request (gadget->ep0, GFP_KERNEL);
    if (!dev->req)
	goto enomem;
    dev->req->buf = kmalloc(USB_BUFSIZ, GFP_KERNEL);
    if (!dev->req->buf)
	goto enomem;
    
    dev->req->complete = flyer_setup_complete;
    
    device_desc.bMaxPacketSize0 = gadget->ep0->maxpacket;

#ifdef CONFIG_USB_GADGET_DUALSPEED
    // assume ep0 uses the same value for both speeds ... 
    dev_qualifier.bMaxPacketSize0 = device_desc.bMaxPacketSize0;
    
    // and that all endpoints are dual-speed 
   hs_source_main_desc.bEndpointAddress = fs_source_main_desc.bEndpointAddress;
   hs_sink_main_desc.bEndpointAddress = fs_sink_main_desc.bEndpointAddress;
   hs_source_urgent_desc.bEndpointAddress = fs_source_urgent_desc.bEndpointAddress;
   hs_sink_urgent_desc.bEndpointAddress = fs_sink_urgent_desc.bEndpointAddress;
#endif
   
    if (gadget->is_otg) 
    {
	otg_descriptor.bmAttributes |= USB_OTG_HNP,
	    full_speed_config.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
    }
    
    usb_gadget_set_selfpowered (gadget);
    
    init_timer (&dev->resume);
    dev->resume.function = flyer_autoresume;
    dev->resume.data = (unsigned long) dev;
    if (autoresume) 
    {
	full_speed_config.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
    }
    
    gadget->ep0->driver_data = dev;
    
    INFO (dev, "%s, version: " DRIVER_VERSION "\n", longname);
    INFO (dev, "using %s, OUT_MAIN %s IN_MAIN %s\n", gadget->name,
	  EP_OUT_MAIN_NAME, EP_IN_MAIN_NAME);
    INFO (dev, "using %s, OUT_URGENT %s IN_URGENT %s\n", gadget->name,
	  EP_OUT_URGENT_NAME, EP_IN_URGENT_NAME);
    
    snprintf (manufacturer, sizeof manufacturer, "%s %s with %s",
	      init_utsname()->sysname, init_utsname()->release,
	      gadget->name);
    pFlyerDev = dev; //set the global pointer to the dev
    if (!g_Connected)
	usb_gadget_disconnect(gadget);
    return 0;

enomem:
    flyer_unbind (gadget);
    return -ENOMEM;
}

/*-------------------------------------------------------------------------*/

static void
flyer_suspend (struct usb_gadget *gadget)
{
	struct flyer_dev	*dev = get_gadget_data (gadget);
	if (gadget->speed == USB_SPEED_UNKNOWN)
		return;

	if (autoresume) {
		mod_timer (&dev->resume, jiffies + (HZ * autoresume));
		DBG (dev, "suspend, wakeup in %d seconds\n", autoresume);
	} else
		DBG (dev, "suspend\n");
}

static void
flyer_resume (struct usb_gadget *gadget)
{
	struct flyer_dev		*dev = get_gadget_data (gadget);
	DBG (dev, "resume\n");
	del_timer (&dev->resume);
	flyer_bind(gadget, NULL);
}


/*-------------------------------------------------------------------------*/

static struct usb_gadget_driver flyer_driver = {
#ifdef CONFIG_USB_GADGET_DUALSPEED
	.max_speed	= USB_SPEED_HIGH,
#else
	.max_speed	= USB_SPEED_FULL,
#endif
	.function	= (char *) longname,
	.bind		= flyer_bind,
	.unbind		= flyer_unbind,

	.setup		= flyer_setup,
	.disconnect	= flyer_disconnect,

	.suspend	= flyer_suspend,
	.resume		= flyer_resume,

	.driver 	= {
		.name		= (char *) shortname,
	},
};

/*----------------------End USB, Start Char Driver-------------------------------------*/
static struct file_operations flyer_fops = {
owner:		THIS_MODULE,
read:		at91_flyer_read,
write:		at91_flyer_write,
unlocked_ioctl:	at91_flyer_ioctl,
open:		at91_flyer_open,
release:	at91_flyer_release,
poll:		at91_flyer_poll
};

static ssize_t at91_flyer_read(struct file* file, char* buf, size_t count, loff_t *offset)
{
    int status = 0;
    int rxamount, finalCount = 0;
    char* pCurBuf = buf;
    int index = 0;
    int i = 0;
    unsigned long flags;
    int *pMinor = (int*)file->private_data;
    if (*pMinor == iDevMain)
    {
	// get as much data as possible from the circular buffer...	
	/*rxamount = gs_buf_data_avail(pFlyerDev->main_buf);
	if (!rxamount)	
	    interruptible_sleep_on(&pFlyerDev->wait);*/
	if (!pFlyerDev )
	    return -ENODEV;
	rxamount = gs_buf_get(pFlyerDev->main_buf, buf, count);
	return rxamount;	   
    }
    else if (*pMinor == iDevUrgent)
    {
	if (!pFlyerDev || !pFlyerDev->out_urgent_ep)
	    return -ENODEV;
	//we are expecting that the program is asking to read more than the maximum amount of 64 * 4 bytes
	if (count < (pFlyerDev->out_urgent_ep->maxpacket * URGENT_QUEUE_LEN))
	    return 0;
	//check for data to read from the urgent queue...
	
	local_irq_save(flags);
	while (index < pFlyerDev->urg_queue.count )
	{
	    rxamount = pFlyerDev->urg_queue.pBuf[index]->actual;
	    memcpy(pCurBuf,pFlyerDev->urg_queue.pBuf[index]->buf,rxamount);
	    pCurBuf = (char*)((int)pCurBuf + rxamount);
	    index++;
	    finalCount += rxamount;
	}
	index = pFlyerDev->urg_queue.count;
	pFlyerDev->urg_queue.count = 0;
	for (i=0;i<index;i++)
	{
	    status = usb_ep_queue (pFlyerDev->out_urgent_ep, pFlyerDev->urg_queue.pBuf[i], GFP_ATOMIC);
	    if (status)
		INFO(pFlyerDev,"Problem re-queueing urgent data request block 0x%x\n",
		     (int)pFlyerDev->urg_queue.pBuf[i]);
	}
	
	local_irq_restore(flags);
	return finalCount;
    }
    else
    {
	INFO(pFlyerDev,"Read from unexpected minor number! minor:%d\n",*pMinor);
    }
    return 0;
}

#define PACKET_WRITE 768
static ssize_t at91_flyer_write(struct file* file, const char* buf, size_t count, loff_t *offset)
{     
    char* pBuf = (char*)buf;
    int countLeft = count;
    int *pMinor = (int*)file->private_data;
    struct usb_request* req;
    struct usb_ep *ep;
    int i=0, result=0;
    int numPackets;
    
    if ( (count <= 0) || !buf)
	return 0;
    if (!pFlyerDev)
	return -ENODEV;
    
    if (*pMinor == iDevMain)
    {	
	ep = pFlyerDev->in_main_ep;
    }
    else if (*pMinor == iDevUrgent)
    {
	ep = pFlyerDev->in_urgent_ep;
    }
    else
    {
	INFO(pFlyerDev,"Read from unexpected minor number! minor:%d\n",*pMinor);
	return 0;
    }
    if (!ep)
	return -ENODEV;
    numPackets = count/PACKET_WRITE; //full packets only 
    
    //INFO (pFlyerDev, "Sending %d bytes over %s, numPackets:%d \n",count,ep->name,numPackets);
    for (i = 0; i < numPackets && result == 0; i++) 
    {
	req = alloc_ep_req (ep, PACKET_WRITE);
	if (req) 
	{
	    req->complete = (*pMinor == iDevMain) ? flyer_main_complete : flyer_urgent_complete;
	    memcpy(req->buf,pBuf,PACKET_WRITE);
	    pBuf = (char*)((int)pBuf + PACKET_WRITE);
	    countLeft -= PACKET_WRITE;
	    result = usb_ep_queue (ep, req, GFP_ATOMIC);
	    if (result)
		DBG (pFlyerDev, "%s queue req --> %d\n",ep->name, result);
	} 
	else
	    result = -ENOMEM;
    }
    
    if (countLeft)
    {
	if (countLeft > PACKET_WRITE)
	    INFO (pFlyerDev, "Count error %d left which is > than %d\n",countLeft,PACKET_WRITE);
	req = alloc_ep_req (ep, PACKET_WRITE);
	if (req) 
	{
	    req->complete = (*pMinor == iDevMain) ? flyer_main_complete : flyer_urgent_complete;
	    memcpy(req->buf,pBuf,countLeft);
	    pBuf = (char*)((int)pBuf + countLeft);
	    req->length = countLeft;
	    countLeft = 0;
	    result = usb_ep_queue (ep, req, GFP_ATOMIC);
	    if (result)
		DBG (pFlyerDev, "%s queue req --> %d\n",ep->name, result);
	} 
	else
	    result = -ENOMEM;
    }
    
    
    
    return count - countLeft;
}

static unsigned int at91_flyer_poll(struct file* file, poll_table* wait)
{
    /*This only works for reads, not writes*/
    unsigned int mask = (POLLOUT | POLLWRNORM);
    int iDevCurrent;
    if (!pFlyerDev)
	return 0;
    iDevCurrent = *(int*)file->private_data;
    if (iDevCurrent == iDevMain)
    {
	if (gs_buf_data_avail(pFlyerDev->main_buf))
	    mask |= (POLLIN | POLLRDNORM);	
    }
    else if (iDevCurrent == iDevUrgent)
    {
	if (pFlyerDev->urg_queue.count)
	    mask |= (POLLIN | POLLRDNORM);
	
    }
    
    if (!(mask & POLLRDNORM))
    {
	poll_wait(file, &pFlyerDev->wait, wait);
	if (!pFlyerDev)
	    return 0;
	if (iDevCurrent == iDevMain)
	{
	    if (gs_buf_data_avail(pFlyerDev->main_buf))
		mask |= (POLLIN | POLLRDNORM);	
	}
	else if (iDevCurrent == iDevUrgent)
	{
	    if (pFlyerDev->urg_queue.count)
		mask |= (POLLIN | POLLRDNORM);
	    
	}
    }
    return mask;
}

static int at91_flyer_open(struct inode* inode, struct file* file)
{
    int iDevCurrent = iminor(inode);
    if (iDevCurrent < iDevMain || iDevCurrent > iDevUrgent || !pFlyerDev)
	return -ENODEV;
    if (iDevCurrent == iDevMain)
	file->private_data = &iDevMain;
    else if (iDevCurrent == iDevUrgent)
    {
	pFlyerDev->urg_queue.listening = 1;
	file->private_data = &iDevUrgent;
    }
    return 0;
}

static int at91_flyer_release(struct inode* inode, struct file* file)
{
    int i = 0,status = 0;
    if (!pFlyerDev || !pFlyerDev->out_urgent_ep)
	return -ENODEV;
    //we are expecting that the program is asking to read more than the maximum amount of 64 * 4 bytes
    
    if (iminor(inode) == iDevUrgent)
    {
	//check for data to read from the urgent queue...
	unsigned long flags;
	local_irq_save(flags);
	pFlyerDev->urg_queue.listening = 0;
	for (i=0;i<pFlyerDev->urg_queue.count;i++)
	{
	    status = usb_ep_queue (pFlyerDev->out_urgent_ep, pFlyerDev->urg_queue.pBuf[i], GFP_ATOMIC);
	    if (status)
		INFO(pFlyerDev,"Problem re-queueing urgent data request block 0x%x\n",
		     (int)pFlyerDev->urg_queue.pBuf[i]);
	}
	pFlyerDev->urg_queue.count = 0;
	local_irq_restore(flags);
    }
    return 0;
}

static long at91_flyer_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    int iArg;
    /* Make sure the command belongs to us*/
    if (_IOC_TYPE(cmd) != FLYER_IOCTL_BASE)
    	return -ENOTTY;
    iArg = (int)arg;
    switch(cmd )
    {
    case FLYER_CONNECTED:
    	return g_bUSBConnected;
    case FLYER_INITIATE_CONNECT://-1 to disconnect on a FAULT, 0 to check and 1 to connect...
	if (!iArg)//passes 0 to just check the state...
	    return g_Connected;
	if (!pFlyerDev)
	    return 0;
	if (iArg > 0)
	{
	    if (g_Connected)
		return 1;
	    g_Connected = 1;
	    return usb_gadget_connect(pFlyerDev->gadget) ? 0 : 1;
	}
	else if (iArg < 0)
	{
	    return usb_gadget_unregister_driver (&flyer_driver) ? 0 : -1;
	}
    default:
    	return -ENOTTY;    	
    }
    return 0;
}

/*-----------------------------End Char Driver-------------------------------------*/

/*-----------------------------Circular Buffer-------------------------------------*/

/*
 * gs_buf_alloc
 *
 * Allocate a circular buffer and all associated memory.
 */
static struct gs_buf *gs_buf_alloc(unsigned int size, int kmalloc_flags)
{
	struct gs_buf *gb;
	if (size == 0)
		return NULL;

	gb = (struct gs_buf *)kmalloc(sizeof(struct gs_buf), kmalloc_flags);
	if (gb == NULL)
		return NULL;

	gb->buf_buf = kmalloc(size, kmalloc_flags);
	if (gb->buf_buf == NULL) {
		kfree(gb);
		return NULL;
	}
	gb->count = 0;
	gb->pUrb = kmalloc(sizeof (void*) * (qlen+1), kmalloc_flags);
	if (gb->pUrb == NULL)
	{
	    kfree(gb->buf_buf);
	    kfree(gb);
	    return NULL;
	}
	else
	{
	    memset(gb->pUrb,0,sizeof (void*) * qlen);
	}
	gb->buf_size = size;
	gb->buf_get = gb->buf_put = gb->buf_buf;
	

	return gb;
}

/*
 * gs_buf_free
 *
 * Free the buffer and all associated memory.
 */
void gs_buf_free(struct gs_buf *gb)
{
	if (gb) {
		kfree(gb->buf_buf);
		kfree(gb);
	}
}

/*
 * gs_buf_clear
 *
 * Clear out all data in the circular buffer.
 */
void gs_buf_clear(struct gs_buf *gb)
{
    if (gb != NULL)
    {
	/*disable interrupts as this affects both the reader and the writer...*/
	unsigned long	flags;
	local_irq_save(flags);	
	gb->buf_get = gb->buf_put;
	local_irq_restore(flags);
	
	/* equivalent to a get of all data available */
    }
}

/*
 * gs_buf_data_avail
 *
 * Return the number of bytes of data available in the circular
 * buffer.
 */
unsigned int gs_buf_data_avail(struct gs_buf *gb)
{
	if (gb != NULL)
		return (gb->buf_size + gb->buf_put - gb->buf_get) % gb->buf_size;
	else
		return 0;
}

/*
 * gs_buf_space_avail
 *
 * Return the number of bytes of space available in the circular
 * buffer.
 */
unsigned int gs_buf_space_avail(struct gs_buf *gb)
{
	if (gb != NULL)
		return (gb->buf_size + gb->buf_get - gb->buf_put - 1) % gb->buf_size;
	else
		return 0;
}

/*
 * gs_buf_put
 *
 * Copy data data from a user buffer and put it into the circular buffer.
 * Restrict to the amount of space available.
 *
 * Changed to copy all or error out...
 * Return the number of bytes copied.
 */
unsigned int gs_buf_put(struct gs_buf *gb, const char *buf, unsigned int count)
{
	unsigned int len;
	unsigned long flags;
	if (gb == NULL || count == 0)
		return 0;
	len = gs_buf_space_avail(gb);
	//printk(KERN_ERR "space:%d, count to add:%d\n",len,count);
	if ( len < count )
	{
	    printk(KERN_ERR "Buffer Overrun!\n");
	    return 0;
	}
	len = gb->buf_buf + gb->buf_size - gb->buf_put;
	local_irq_save(flags);
	if (count > len) {
		memcpy(gb->buf_put, buf, len);
		memcpy(gb->buf_buf, buf+len, count - len);
		gb->buf_put = gb->buf_buf + count - len;
	} else {
		memcpy(gb->buf_put, buf, count);
		if (count < len)
			gb->buf_put += count;
		else /* count == len */
			gb->buf_put = gb->buf_buf;
	}
	local_irq_restore(flags);
	return count;
}

/*
 * gs_buf_requeue
 *
 * Decide whether to requeue the URB or to set it aside until there is more room in the buffer
 * 
 * return the status of which ever operation is valid
 *
 */
unsigned int gs_buf_requeue(struct gs_buf *gb,struct usb_ep *ep, struct usb_request *req)
{
    unsigned int status;
    uint len;
    if (gb->ep != ep)
    {
	printk(KERN_ERR "gb->ep:0x%x, ep:0x%x\n",(uint)gb->ep,(uint)ep);
	return (uint)-1;
    }
    req->length = PACKET_SIZE; //always setup the length to be correct...
    //decide what we want to do with this one...
    len = gs_buf_space_avail(gb);
    if ( len > ((qlen+2)*PACKET_SIZE)  )
    {
	status = usb_ep_queue (ep, req, GFP_ATOMIC);
	if (status == 0)
	    return status;
    }
    else
    {
	unsigned long	flags;
	local_irq_save(flags);	
	//printk(KERN_ERR "Holding onto a req, count:%d\n",gb->count);
	gb->pUrb[gb->count] = req;
	gb->count++;
	status = 0;
	local_irq_restore(flags);
    }
    return status;
}

/*
 * gs_buf_requeue
 *
 * Decide whether to requeue the URB or to set it aside until there is more room in the buffer
 * 
 * return the status of which ever operation is valid
 *
 */
unsigned int gs_buf_clear_hold(struct gs_buf *gb)
{
    uint i = 0,status = 0;
    if ((gb->count) && (gs_buf_space_avail(gb) > ( (qlen+2)*PACKET_SIZE)))
    {
	unsigned long	flags;
	local_irq_save(flags);	
	//printk(KERN_ERR"Re-adding %d reqs into the spinlock queue\n",gb->count);

	
	for(i=0;i<gb->count;i++)
	{
	    status = usb_ep_queue (gb->ep, gb->pUrb[i], GFP_ATOMIC);
	    if (status != 0)
		printk(KERN_ERR "Failed to requeue req:0x%x at i:%d\n",(uint)gb->pUrb[i],i);
	    gb->pUrb[i] = NULL;
	}
	//printk(KERN_ERR"Setting gb->count to 0 from %d\n",gb->count);
	gb->count = 0;
	local_irq_restore(flags);	
    }
    return i;
}

/*
 * gs_buf_get
 *
 * Get data from the circular buffer and copy to the given buffer.
 * Restrict to the amount of data available.
 *
 * Return the number of bytes copied.
 */
unsigned int gs_buf_get(struct gs_buf *gb, char *buf, unsigned int count)
{
	unsigned int len;
	unsigned long	flags;
	if (gb == NULL || count == 0)
		return 0;
	
	len = gs_buf_data_avail(gb);
	if (count > len)
	    count = len;
	
	len = gb->buf_buf + gb->buf_size - gb->buf_get;
	local_irq_save(flags);	
	if (count > len) {
		memcpy(buf, gb->buf_get, len);
		memcpy(buf+len, gb->buf_buf, count - len);
		gb->buf_get = gb->buf_buf + count - len;
	} else {
		memcpy(buf, gb->buf_get, count);
		if (count < len)
			gb->buf_get += count;
		else /* count == len */
			gb->buf_get = gb->buf_buf;
	}
	local_irq_restore(flags);
	if (count)
	{
	    gs_buf_clear_hold(gb);
	}
	return count;
}

/*-----------------------------End Circular Buffer---------------------------------*/

/*-----------------------------Proc stuff------------------------------------------*/

static const char debug_filename[] = "driver/flyer";

static const char ethernet[] = "ethernet";
static const char usb[] = "usb";
static const char none[] = "none";
static const char both[] = "both";

static int proc_flyer_show(struct seq_file *s, void *unused)
{
    //usb is paramount over ethernet...
    char* result;
    if (g_bUSBConnected && g_bEthConnected)
    {
	result = (char*)both;
    }
    else if (g_bUSBConnected)
    {
	result = (char*)usb;
    }
    else if (g_bEthConnected)
    {
	result = (char*)ethernet;
    }
    else
    {
	result = (char*)none;
    }	
    
    seq_printf(s, "%s\n", result);
    return 0;
}

static int proc_flyer_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_flyer_show, inode);
}

static struct file_operations proc_ops = {
        .open		= proc_flyer_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debug_file(void)
{    
    g_flyer_pde = proc_create (debug_filename, 0, NULL, &proc_ops);
}

static void remove_debug_file(void)
{
    if (g_flyer_pde)
	remove_proc_entry(debug_filename, NULL);
}
/*-----------------------------End proc stuff---------------------------------------*/

MODULE_AUTHOR ("Jeff Warren");
MODULE_LICENSE ("GPL");

/*
 *Dont register the USB driver unless and until the flyer program is running.  Otherwise WinMark will
 *not successfully detect the head.
 */
static int __init init (void)
{
    int char_ret;
    
    // All this for the bloody serial number!
    void *pci_reg_base;
    int serialnum;
    pci_reg_base = ioremap(FLYER_PCIL0_BASE, FLYER_PCIL0_SIZE);   
    serialnum = (int)PCI_READL(FLYER_PCIL0_PMM0PCIHA);
    sprintf (serial, "%09d", serialnum);
    iounmap(pci_reg_base);
    // Serial number done
    
    char_ret = register_chrdev(FLYER_MAJOR,shortname,&flyer_fops);
    if (char_ret)
    {
	printk(KERN_ERR "flyer_usb: Can't register char device with kernel.\n");
	return char_ret;
    }
    else
    {
	char_ret = usb_gadget_probe_driver(&flyer_driver);
	if (char_ret)
	{
	    unregister_chrdev(FLYER_MAJOR,shortname);
	    printk(KERN_ERR "flyer_usb: Can't register USB Gadget device with kernel.\n");
	    return char_ret;
	}
    }
    
    create_debug_file();
    
    return char_ret;
}
module_init (init);

static void __exit cleanup (void)
{
    unregister_chrdev(FLYER_MAJOR,shortname);
    //int res = unregister_chrdev(FLYER_MAJOR,shortname);
    //if (res)
    //{
	//printk(KERN_ERR "flyer_usb: Can't unregister device at91_ssc_temp with kernel.\n");
    //}
    usb_gadget_unregister_driver (&flyer_driver);
    remove_debug_file();
}
module_exit (cleanup);

