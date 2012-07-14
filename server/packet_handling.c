/**
 ******************************************************************************
 *
 * @brief      Processes packets from Revo and pases them to the
 *             UAVObjectManager as well as logging them optionally.
 *
 * @file       packet_handler.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
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

//! The global uavTalk handle which talks to the uavobject manager
extern UAVTalkConnection uavTalk;

extern FILE *file_fd_err;

/**
 * Write a UAVTalk log entry (one UAVTalk message) with a timestamp and CRC
 * @param [in] buf The buffer pointer
 * @param [in] len The number of bytes in this packet
 * @param [in] timestamp The timestamp parsed for this packet
 */
static void packet_to_disk(unsigned char *buf, int len, __u32 timestamp)
{
       struct timeval now;
       __u64 packet_size;

       packet_size = len;
       fwrite(&timestamp, 1, sizeof(timestamp), file_fd);
       fwrite(&packet_size, 1, sizeof(packet_size), file_fd);
       // Must add 1 because CRC not included in this number
       fwrite(buf, 1, len + 1, file_fd);
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
	unsigned char *cp = buf;
	unsigned int i = 0, j;
	__u32 timestamp;
	int packet_size;
	int object_id;
	int received_bytes = 0;

	// Make sure there is at least room for the timestamp and uavtalk
	// header (timestamp = 4 sync = 1 type = 1 packet size = 2 object id = 4)
	while(i < (len - 11)) {
		timestamp = cp[i] + (cp[i+1] << 8) + (cp[i+2] << 16) + (cp[i+3] << 24);
		if(cp[i+4] == 0x3c) {
			// Get the packet size and object id
			packet_size = cp[i+6] + (cp[i+7] << 8);
			object_id = cp[i+8] + (cp[i+9] << 8) + (cp[i+10] << 16) + (cp[i+11] << 24);

			// Add 4 for timestamp and 1 for crc
			if ((i + packet_size + 5) >= len) {
				fprintf(stderr,"Nonsense packet size\n");
				fwrite(buf, 1, len, file_fd_err);
				return;
			}
			//fprintf(stdout, "Got object %x %u\n", object_id, packet_size);
			received_bytes += packet_size + 1;

			if (logging)
				packet_to_disk(&cp[i+4], packet_size, timestamp);

			// Send packets after removing the timestamp to the
			// event system.  Plus one for the crc.
			for(j = i+4; j < i+4+packet_size+1; j++)
				UAVTalkProcessInputStream(uavTalk, buf[j]);

			if(packet_size == 0) 
				i++;
			i += packet_size + 1;
		} else {
			i++;
		}	
	}	
	return received_bytes;
}

/**
 * Get a packet from the SPIDEV and pass it to the UAVObjectManager.  If
 * logging is true write it to a file.
 * @param [in] dev_fd The SPIDEV handle
 * @param [in] file_fd The file to write to
 * @param [in] logging Whether to log to file
 */
static void process_packet(int dev_fd, FILE  *file_fd, bool logging)
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

	parse_packet(buf, len, logging);
}
