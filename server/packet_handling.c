/**
 ******************************************************************************
 *
 * @brief      Processes packets from Revo and pases them to the
 *             UAVObjectManager as well as logging them optionally.
 *
 * @file       packet_handler.c
 * @author     Tau Labs, http://github.com/TauLabs Copyright (C) 2012-2013.
 * @brief      Include files of the uavobjectlist library
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "packet_handling.h"

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>

//! The global uavTalk handle which talks to the uavobject manager
extern UAVTalkConnection uavTalk;

extern FILE *file_fd_err;
extern FILE *file_fd;

#define MAX_UNKNOWN_IDS 100
static struct statistics {
	uint32_t unknown_ids[MAX_UNKNOWN_IDS];
	uint32_t unknown_id_count[MAX_UNKNOWN_IDS];
	uint32_t unknown_id_idx;
	uint32_t badsize_count;
} ph_statistics;

//! Private methods
//! Write data to disk
static void packet_to_disk(unsigned char *buf, int len);

//! Parse a packet into the UAVTalk parts
static int parse_packet(unsigned char *buf, int len, bool logging);

/**
 * Print a summary of the statistics from the packet handler
 */
void ph_print_statistics()
{
	fprintf(stdout, "Total nonsense packets: %d\n", ph_statistics.badsize_count);
}

/**
 * Write a UAVTalk log entry (one UAVTalk message) with a timestamp and CRC
 * @param [in] buf The buffer pointer
 * @param [in] len The number of bytes in this packet
 * @param [in] timestamp The timestamp parsed for this packet
 */
static void packet_to_disk(unsigned char *buf, int len)
{
       fwrite(buf, 1, len, file_fd);
}

/**
 * Parse the packet received and afterwards if required log to disk
 * @param [in] buf The data buffer
 * @param [in] len The number of bytes in the buffer
 * @param [in] logging Whether to log to disk
 * @return The number of bytes received
 */
static int parse_packet(unsigned char *buf, int len, bool logging)
{
	unsigned int i = 0;

	// Make sure there is at least room for the timestamp and uavtalk
	// header (timestamp = 4 sync = 1 type = 1 packet size = 2 object id = 4)
	while(i < len)
		UAVTalkProcessInputStream(uavTalk, buf[i++]);

	if(logging)
		packet_to_disk(buf, len);

	return i;
}

/**
 * Get a packet from the SPIDEV and pass it to the UAVObjectManager.  If
 * logging is true write it to a file.
 * @param [in] dev_fd The SPIDEV handle
 * @param [in] file_fd The file to write to
 * @param [in] logging Whether to log to file
 */
int process_packet(int dev_fd, bool logging)
{
	const int len = PACKET_SIZE;
	unsigned char	buf[len], tx_buf[len], *bp;
	int		status;

	struct spi_ioc_transfer	xfer[1];
	memset(buf, 0, sizeof buf);
	memset(&xfer[0], 0, sizeof(xfer[0]));
	
	xfer[0].rx_buf = (unsigned long) buf;
	xfer[0].len = len;
	xfer[0].tx_buf = (unsigned long) tx_buf;
	xfer[0].len = len;

	status = ioctl(dev_fd, SPI_IOC_MESSAGE(1), xfer);

	if (status < 0) {
		perror("read");
		return;
	}

	return parse_packet(buf, len, logging);
}
