/*
 * Serial Peripheral Interface (SPI) driver for the Atmel AT91RM9200
 *
 * (c) SAN People (Pty) Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifndef SYNRAD_SPI_H
#define SYNRAD_SPI_H

/* Maximum number of buffers in a single SPI transfer.
 *  DataFlash uses maximum of 2
 *  spidev interface supports up to 8.
 */
#define MAX_SPI_TRANSFERS	8

#define NR_SPI_DEVICES  	2	/* number of devices on SPI bus */

#define SPI_MAJOR		241	/* registered device number */

#define SPI_SYNC	0
#define SPI_ASYNC	1

/* SPI Registers */
typedef volatile u8 SPI_REG;

/* SPI Controller registers */
typedef struct amcc440ep_spi_reg {
    SPI_REG mode;
    SPI_REG RxD;
    SPI_REG TxD;
    SPI_REG control;
    SPI_REG status;
    SPI_REG pad;
    SPI_REG clock;
}AMCC440EP_SPI, *pAMCC440EP_SPI;;

/* SPI Controller mode register definitions */
#define	SPMODE_LOOP		0x1
#define	SPMODE_CI_ACTIVELOW	0x2
#define	SPMODE_MSB		0x4
#define	SPMODE_ENABLE		0x8
#define	SPMODE_CP_LEADING_EDGE	0x10

/* SPI Status Register definitions */
#define SPSTATUS_RXREADY	0x1
#define SPSTATUS_BUSY		0x2

/*SPI Control Register Definitions */
#define SPCTRL_STR_ENABLE	0x1

/* clock settings (SCP and CI) for various SPI modes */
#define SPI_CLK_MODE0      SPMODE_CP_LEADING_EDGE
#define SPI_CLK_MODE1      0
#define SPI_CLK_MODE2      SPMODE_CI_ACTIVELOW
#define SPI_CLK_MODE3      (SPMODE_CP_LEADING_EDGE|SPMODE_CI_ACTIVELOW)

/*SPI Registers */

#define	SPI_PHYS_START          0x0EF600900
#define	SPI_PHYS_END            0x0EF600906

// Default Controller Values
#define DEFAULT_CLK_HZ1		16666666
#define DEFAULT_CLK_HZ2		16666666/7
#define DEFAULT_MODE		SPI_CLK_MODE2 | SPMODE_ENABLE

/*
 * Describes the buffers for a SPI transfer.
 * A transmit & receive buffer must be specified for each transfer
 */
struct spi_transfer_list {
	void* tx[MAX_SPI_TRANSFERS];	/* transmit */
	int txlen[MAX_SPI_TRANSFERS];
	void* rx[MAX_SPI_TRANSFERS];	/* receive */
	int rxlen[MAX_SPI_TRANSFERS];
	int nr_transfers;		/* number of transfers */
	int curr;			/* current transfer */
	int txtype;
	int servo_rxval;
};

struct spi_local {
	u32 pcs;		/* Peripheral Chip Select value */
	u8  clock;
	//short pio_enabled;		/* has PIO been enabled? */
	unsigned int opb_freq;
	//struct spi_transfer_list *xfers;	/* current transfer list */
	char tx[PAGE_SIZE*8];
	char rx[PAGE_SIZE*8];

};


/* Exported functions */
//extern void spi_access_bus(short device);
//extern void spi_release_bus(short device);
//extern int spi_transfer(struct spi_transfer_list* list);
//extern int spi_ready(short device);

#endif
