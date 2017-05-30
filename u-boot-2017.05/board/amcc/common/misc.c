/*
 * (C) Copyright 2004
 * Stefan Roese, esd gmbh germany, stefan.roese@esd-electronics.com
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>

#ifdef CONFIG_LXT971_NO_SLEEP
#include <miiphy.h>
#endif


#ifdef CONFIG_LXT971_NO_SLEEP
void lxt971_no_sleep(void)
{
	unsigned short reg;

	miiphy_read("ppc_4xx_eth0", CONFIG_PHY_ADDR, 0x10, &reg);
	reg &= ~0x0040;                  /* disable sleep mode */
	miiphy_write("ppc_4xx_eth0", CONFIG_PHY_ADDR, 0x10, reg);
	/* write correct LED configuration */
	//left  off: no link, green 100MBit, yellow 10MBit
	//right off: no activity, green full-duplex, yellow half-duplex
	if (miiphy_write("ppc_4xx_eth0", CONFIG_PHY_ADDR, 0x14, 0x2402) != 0) {
	//if (miiphy_write("ppc_4xx_eth0", CONFIG_PHY_ADDR, 0x14, 0x0452) != 0) {
		printf ("Error writing to the PHY\n");
	}
	//mii_phy_write(20, 0x0452);

}
#endif /* CONFIG_LXT971_NO_SLEEP */
