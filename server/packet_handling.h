/**
 ******************************************************************************
 *
 * @brief      Processes packets from Freedom and pases them to the
 *             UAVObjectManager as well as logging them optionally.
 *
 * @file       packet_handler.h
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

#ifndef PACKET_HANDLER_H
#define PACKET_HANDLER_H

#define PACKET_SIZE 1024

#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <linux/types.h>

#include "uavtalk.h"
#include "uavobjectmanager.h"

//! Print the statistics from the packet handler
void ph_print_statistics();

//! Get a packet from the SPI port and passes it to the UAVObjectManager
int process_packet(int dev_fd, bool logging);

#endif
