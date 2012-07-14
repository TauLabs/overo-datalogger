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

//! Write data to disk
static void packet_to_disk(unsigned char *buf, int len, __u32 timestamp);

//! Parse a packet into the UAVTalk parts
static void parse_packet(unsigned char *buf, int len, bool logging);

//! Get a packet from the SPI port and passes it to the UAVObjectManager
static void process_packet(int dev_fd, FILE  *file_fd, bool logging);


static void do_read(int fd, int len);

#endif
