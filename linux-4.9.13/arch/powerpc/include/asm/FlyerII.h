/*********************FlyerII.h**********************/
#ifndef _FLYII_CONF__H
#define _FLYII_CONF__H

typedef volatile __be32 AMCC_REG;

extern void* xil_get_mapped_address(void);
	
#define XILINX_IRQ   27

/* handy sizes */
#define SZ_16				0x00000010
#define SZ_256				0x00000100
#define SZ_512				0x00000200

#define SZ_1K                           0x00000400
#define SZ_4K                           0x00001000
#define SZ_8K                           0x00002000
#define SZ_16K                          0x00004000
#define SZ_64K                          0x00010000
#define SZ_128K                         0x00020000
#define SZ_256K                         0x00040000
#define SZ_512K                         0x00080000

#define SZ_1M                           0x00100000
#define SZ_2M                           0x00200000
#define SZ_4M                           0x00400000
#define SZ_8M                           0x00800000
#define SZ_16M                          0x01000000
#define SZ_32M                          0x02000000
#define SZ_64M                          0x04000000
#define SZ_128M                         0x08000000
#define SZ_256M                         0x10000000
#define SZ_512M                         0x20000000

#define SZ_1G                           0x40000000
#define SZ_2G                           0x80000000


/* General Purpose Timer Registers */
#define GPT0_IRQ     18
#define GPT1_IRQ     19
#define GPT_DOWN_COUNT_IRQ    30

#define GPT_TBC_PHYS_START                      0x0EF600000
#define GPT_TBC_PHYS_END		        0x0EF600004

 //AMCC_REG tbc;			// Time Base Counter		                        RW
#define GPT_INT_PHYS_START		        0x0EF600018
#define GPT_INT_PHYS_END		        0x0EF600024

typedef struct _AMCC440EP_GPT_INT
{
   
    AMCC_REG im;			// GPT Interrupt Mask		                        RW
    AMCC_REG iss;			// GPT Interrupt Status (Set bits if write 1)		RW
    AMCC_REG isc;			// GPT Interrupt Status (Clear bits if write 1)		RW
    AMCC_REG ie;			// GPT Interrupt Enable	                                RW
    
} AMCC440EP_GPT_INT, *pAMCC440EP_GPT_INT;

#define GPT_COMP_PHYS_START		        0x0EF600080
#define GPT_COMP_PHYS_END		        0x0EF600098

typedef struct _AMCC440EP_GPT_COMP
{
    AMCC_REG comp0;			// Compare Timer 0		                        RW
    AMCC_REG comp1;			// Compare Timer 1			                RW
    AMCC_REG comp2;			// Compare Timer 2		                        RW
    AMCC_REG comp3;			// Compare Timer 3		                        RW
    AMCC_REG comp4;			// Compare Timer 4		                        RW
    AMCC_REG comp5;			// Compare Timer 5		                        RW
    AMCC_REG comp6;			// Compare Timer 6		                        RW
} AMCC440EP_GPT_COMP, *pAMCC440EP_GPT_COMP;

#define GPT_MASK_PHYS_START		        0x0EF6000C0
#define GPT_MASK_PHYS_END		        0x0EF6000D8

typedef struct _AMCC440EP_GPT_MASK
{
    AMCC_REG mask0;			// Compare Mask  1		                        RW
    AMCC_REG mask1;			// Compare Mask  1		                        RW
    AMCC_REG mask2;			// Compare Mask  2		                        RW
    AMCC_REG mask3;			// Compare Mask  3		                        RW
    AMCC_REG mask4;			// Compare Mask  4		                        RW
    AMCC_REG mask5;			// Compare Mask  5		                        RW
    AMCC_REG mask6;			// Compare Mask  6		                        RW
} AMCC440EP_GPT_MASK, *pAMCC440EP_GPT_MASK;


#define GPT_DCT0_PHYS_START		        0x0EF600110
#define GPT_DCT0_PHYS_END		        0x0EF600114

#define GPT_DCIS_PHYS_START		        0x0EF60011C
#define GPT_DCIS_PHYS_END		        0x0EF600120
/*
typedef struct _AMCC440EP_GPT_DCT
{    
    AMCC_REG dct0;			// Down Count Timer		                        RW
    AMCC_REG dcis;			// Down Count Timer Interrupt Status		        RW
} AMCC440EP_GPT_DCT, *pAMCC440EP_GPT_DCT;
*/

/* GPIO Registers */

#define GPIO_0_PHYS_START		0x0EF600B00
#define GPIO_0_PHYS_END		        0x0EF600B44
	
#define GPIO_1_PHYS_START		0x0EF600C00
#define GPIO_1_PHYS_END		        0x0EF600C44

typedef struct _AMCC440EP_GPIO
{
    AMCC_REG orr;				// Output Register		        RW
    AMCC_REG tcr;				// Tri-State Control Register		RW
    AMCC_REG osrl;			// Output Select Register Low		RW
    AMCC_REG osrh;			// Output Select Register Hi		RW
    AMCC_REG tsrl;			// Tri-State Select Register Low	RW
    AMCC_REG tsrh;			// Tri-State Select Register HI		RW
    AMCC_REG odr;				// Open Drain Register			RW
    AMCC_REG ir;				// Input register			R
    AMCC_REG rr1;				// Receive Register 1			RW
    AMCC_REG rr2;				// Receive Register 2			RW
    AMCC_REG rr3;				// Receive Register 3			RW
    AMCC_REG isr1l;			// Input Select Register 1 Low		RW
    AMCC_REG isr1h;			// Input Select Register 1 Hi		RW
    AMCC_REG isr2l;			// Input Select Register 2 Low		RW
    AMCC_REG isr2h;			// Input Select Register 2 Hi		RW
    AMCC_REG isr3l;			// Input Select Register 3 Low		RW
    AMCC_REG isr3h;			// Input Select Register 3 Hi		RW
} AMCC440EP_GPIO, *pAMCC440EP_GPIO;

/* GPIO Register definitions */

#define SIGNAL_CTRL(x)	   ((x) << 30)
#define SET_BIT32(x)	   (1 << (31-x))
#define BIT32(x)	   (1 << (31-x))
#define TWOBIT32(x)        ((x>16)?(3 << 2*(31-x)):(3 << 2*(15-x)))
/*
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
*/    

#endif /*_XIL_CONF__H */
