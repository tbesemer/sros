/*********************at91_Flyer_Xilinx_Config.h**********************/
#ifndef _XIL_CONF__H
#define _XIL_CONF__H

#ifdef MODULE
extern void* xil_get_mapped_address(void);
#endif

typedef struct
{
    uint dwState; //only the first 16 are used...
    int iTimeout;
} waitStruct;

	

#define SIG_TESTMARK SIGUSR1
#define SIG_ABORT SIGURG //this won't be used for the servo thread.
#define SIG_OVERTEMP SIGURG
//sbs 2.21 07-Jul-2008 Add an IO Change Event for PANNIER
#define SIG_IOCHANGE SIGWINCH
//#define SIG_IOCHANGE SIGPWR

#define RC_PULSE_MIN 50
#define XILINX_CONFIG_MAJOR 242
#define XILINX_CONFIG_IOCTL_BASE 0xAF

#define XIL_CHECK_STATUS _IO(XILINX_CONFIG_IOCTL_BASE,0x20)
#define XIL_START_CLOCKS _IOR(XILINX_CONFIG_IOCTL_BASE,0x21,int)
#define XIL_ENC_PULSE_SET _IO(XILINX_CONFIG_IOCTL_BASE,0x22)
#define XIL_WAKE_IO_SLEEPERS _IO(XILINX_CONFIG_IOCTL_BASE,0x23)
#define XIL_TRACK_WAIT _IO(XILINX_CONFIG_IOCTL_BASE,0x24)
#define XIL_TESTMARK_DONE _IO(XILINX_CONFIG_IOCTL_BASE,0x25)
#define XIL_CHECK_SWITCH _IO(XILINX_CONFIG_IOCTL_BASE,0x26)

//Reads from the Xilinx...
#define XIL_GET_IO _IO(XILINX_CONFIG_IOCTL_BASE,0x30)
/*| Output(8) | Input(8) | (16)*/

#define XIL_SYSTEM_STATUS _IO(XILINX_CONFIG_IOCTL_BASE,0x31)
//see MarkDefines.h of flyermarker upper word of SERVO_STATUS defines

#define XIL_GET_VERSION _IO(XILINX_CONFIG_IOCTL_BASE,0x34)
/*| Version | (16)*/

#define XIL_GET_KEYPAD _IO(XILINX_CONFIG_IOCTL_BASE,0x35)
/*| Unused(8bits) | keypad pressed(bit7) | bit(6-1) Reserved | TestMark key(bit0) | (16)*/
#define KEY_TEST_MARK 1
#define KEY_PRESSED 0x80

#define XIL_GET_WAIT_DIGITAL _IO(XILINX_CONFIG_IOCTL_BASE,0x36)
/*| Mask(8) | Value(8) | (16)*/

#define XIL_GET_TEMP_STATUS _IO(XILINX_CONFIG_IOCTL_BASE,0x37)
/*| Unused(12bits) | Temp2 Hyst(3) | Temp2 OT(2) | Temp1 Hyst(1) | Temp1 OT(0) | */

#define XIL_GET_SWITCHES _IO(XILINX_CONFIG_IOCTL_BASE,0x38)
/*| Unused(10bits) | Bit 7 Temp Out(5) | Auto Index TestMark(4) | KeyLock(3) | Tickle Disable(2) | PWM Gate(1) | FASI(0) | */

#define XIL_GET_TESTMARK _IO(XILINX_CONFIG_IOCTL_BASE,0x39)

#define FASI_ON		0x1
#define PWM_OUTPUT 	0x2
#define TICKLE_DISABLE	0x4
#define KEYBOARD_LOCKED	0x8
#define TM_AUTO_INDEX	0x10
#define TEMP_OUTPUT	0x20
#define DIODE_PTR       0x40

//Writes to the Xilinx
#define XIL_SET_IO _IO(XILINX_CONFIG_IOCTL_BASE,0x50)
/*| Output(8) | Input(8) | (16)*/

#define XIL_DEBUG_SET _IO(XILINX_CONFIG_IOCTL_BASE,0x51)
/*| Reserved (bits 15-10) | DebugTest(bits 9-0) | (16)*/

#define XIL_ENCODER_CFG _IO(XILINX_CONFIG_IOCTL_BASE,0x52)
/*| Reserved(15-5)  | Use Quadrature(bit 4) | Use Fixed Part Pitch (bit3) | Invert Part Sense (bit 2) 
 *| Invert Encoder Direction (bit 1) | Encoderless Tracking (bit 0) | (16)*/

#define XIL_SET_PART_PITCH _IO(XILINX_CONFIG_IOCTL_BASE,0x53)
/*| Part Pitch Count in encoder pulses | (16)*/

#define XIL_SET_WAIT_DIGITAL _IO(XILINX_CONFIG_IOCTL_BASE,0x54)
/*| Mask(8) | Value(8) | (16)*/

#define XIL_LED_SET_USB _IO(XILINX_CONFIG_IOCTL_BASE,0x55)
#define XIL_LED_SET_STATUS _IO(XILINX_CONFIG_IOCTL_BASE,0x56)
/*| Value(16) See Below | (16)*/
#define SOLID_OFF 0x0
#define SOLID_RED 0x1
#define SOLID_GREEN 0x2
#define SOLID_YELLOW 0x3
//the other 6 bits
#define FREQ_MASK 0xfc
#define FAST_BLINK (5 << 2)
#define SLOW_BLINK (2 << 2)

#define XIL_DEBUG_CLEAR _IO(XILINX_CONFIG_IOCTL_BASE,0x57)
/*| Reserved (bits 15-10) | DebugTest(bits 9-0) | (16)*/

#define XIL_SET_SWITCHES _IO(XILINX_CONFIG_IOCTL_BASE,0x58)
/*| Unused(10bits) | Bit 7 Temp Out(5) | Auto Index TestMark(4) | KeyLock(3) | Reserved(2-0) | */

#define XIL_SET_PART_INTERRUPT _IO(XILINX_CONFIG_IOCTL_BASE,0x59)
/*| Unused(15bits) | Bit 0(1-enable,0-disable) | */

//sbs 2.21 07-Jul-2008 Add an IO Change Event for PANNIER
#define XIL_SET_IO_CHANGE _IO(XILINX_CONFIG_IOCTL_BASE,0x5A)
//#define XIL_ENABLE_INTERRUPTS _IO(XILINX_CONFIG_IOCTL_BASE,0x5B)

// sbs 2.21 11-Jul-2008 MOSAIC: Z-axis servo status is read indirectly through the Xilinx
#define XIL_Z_SERVO_STATUS _IO(XILINX_CONFIG_IOCTL_BASE,0x5B)

// 05-Aug-2009 sbs 2.57: Allow Mark Aborts from a predetermined user-enabled input (Input 7).
// Ran out of signals so Abort and IO Change share a signal. The interrupt type determines
// What the signal handler should do.
#define XIL_GET_INTERRUPT_TYPE _IO(XILINX_CONFIG_IOCTL_BASE,0x5C)

// 01-Nov-2011 sbs 3.11 Issue: Banner marking with user part pitch enabled
#define XIL_SET_BANNER_PITCH _IO(XILINX_CONFIG_IOCTL_BASE,0x5D)
/*| Banner Pitch Count in encoder pulses | (16)*/
#define XIL_SET_BANNER_COUNT _IO(XILINX_CONFIG_IOCTL_BASE,0x5E)
/*| Banner Window Count | (16)*/
#define XIL_ENCODER_CFG_STR _IO(XILINX_CONFIG_IOCTL_BASE,0x5F)

#define XIL_SET_DIODE_PTR _IO(XILINX_CONFIG_IOCTL_BASE,0x60)
/*| Unused(15bits) | Bit 0(1-enable,0-disable) | */

#define XIL_GET_DIODE_PTR _IO(XILINX_CONFIG_IOCTL_BASE,0x61)
/*| Unused(15bits) | Bit 0(1-enabled,0-disabled) | */

//Read/Write offsets from the Xilinx base address...
#define XIL_KEYPAD_OFFSET       0x0   /* Read Only */
#define XIL_ENC_CFG_OFFSET	0x0   /* Write Only */
#define XIL_IO_OFFSET		0x1   /* Read/Write */
#define XIL_STATUS_OFFSET 	0x2   /* Read Only */
#define XIL_RESERVED_OFFSET	0x2   /* Write Only */
#define XIL_INT_TABLE		0x3   /* Read Only */
#define XIL_PPC_OFFSET		0x3   /* Write Only , Fixed Part Pitch Count*/
#define XIL_WAIT_DIGITAL_OFFSET	0x4   /* Read/Write */
#define XIL_SWITCHES		0x5   /* Read/Write , FASI(RO),PWM(RO),KeyLock(RW), and Tickle Disable(RO)*/
//#define XIL_LED_OFFSET		0x6   /* Write Only */
// sbs 2.21 11-Jul-2008 MOSAIC: Z-axis servo status is read indirectly through the Xilinx
#define XIL_Z_STATUS_OFFSET		0x6   /* Write Only */
#define XIL_TIMEOUT_LSB         0x7   /* Write Only */
#define XIL_TIMEOUT_MSB         0x8   /* Write Only */
#define XIL_PARTSENSE_ENABLE    0x9   /* Write Only */
//sbs 2.21 07-Jul-2008 Add an IO Change Event for PANNIER
#define XIL_IO_CHANGE_OFFSET           0xA   /* Write Only */
// 01-Nov-2011 sbs 3.11 Issue: Banner marking with user part pitch enabled 
#define XIL_BANNER_PITCH        0xB   /* Write Only  - Banner Window interval in encoder pulses*/ 
#define XIL_BANNER_COUNT        0xC   /* Write Only  - Number of Banner windows*/
#define XIL_DIODE_PTR_OFFSET    0xD   /* Read/Write  - Diode Pointer status and control */

/*NOTE: for debug output SW1 Switches 3&4 have to be ON*/
#define XIL_DEBUG_CLR_OFFSET	0x1E  /* Write Only */
#define XIL_DEBUG_SET_OFFSET    0x1F  /* Write Only */
#define XIL_VERSION_OFFSET	0x1F  /* Read Only */

//sbs 2.21 07-Jul-2008 Add an IO Change Event for PANNIER
#define IOCHANGE_INTERRUPT	0x100
// 05-Aug-2009 sbs 2.57: Allow Mark Aborts from a predetermined user-enabled input (Input 7).
#define ABORT_INTERRUPT	        0x200


#endif /*_XIL_CONF__H */
