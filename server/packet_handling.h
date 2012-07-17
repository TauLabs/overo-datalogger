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
