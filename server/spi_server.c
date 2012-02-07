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
#include <linux/spi/spidev.h>

#include "openpilot.h"
#include "uavtalk.h"
#include "uavobjectmanager.h"

static int verbose;
int received_bytes;
FILE *file_fd;
FILE *file_fd_err;
UAVTalkConnection uavTalk;

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

static void parse_packet(unsigned char *buf, int len)
{
	unsigned char *cp = buf;
	unsigned int i = 0;
	__u32 timestamp;
	int packet_size;
	int object_id;

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
			received_bytes += packet_size;
			packet_to_disk(&cp[i], packet_size, timestamp);
			
			if(packet_size == 0) 
				i++;
			i += packet_size + 1;
		} else {
			i++;
		}	
	}	

	for(i = 0; i < len; i++)
		UAVTalkProcessInputStream(uavTalk, buf[i]);


}

static void grab_log_packet(int dev_fd, FILE  *file_fd)
{
	const int len = 256;
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

	parse_packet(buf, len);
}

static void to_disk(int dev_fd, FILE  *file_fd)
{
	const int len = 256;
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

	fwrite(buf, 1, len, file_fd);
}

static void do_read(int fd, int len)
{
	unsigned char	buf[256], *bp;
	int		status;

	/* read at least 2 bytes, no more than 32 */
	if (len < 2)
		len = 2;
	else if (len > sizeof(buf))
		len = sizeof(buf);
	memset(buf, 0, sizeof buf);

	status = read(fd, buf, len);
	if (status < 0) {
		perror("read");
		return;
	}
	if (status != len) {
		fprintf(stderr, "short read\n");
		return;
	}

	printf("read(%2d, %2d): %02x %02x,", len, status,
		buf[0], buf[1]);
	status -= 2;
	bp = buf + 2;
	while (status-- > 0)
		printf(" %02x", *bp++);
	printf("\n");
}

static void do_msg(int fd, int len)
{
	struct spi_ioc_transfer	xfer[2];
	unsigned char		buf[256], *bp;
	int			status;

	memset(xfer, 0, sizeof xfer);
	memset(buf, 0, sizeof buf);

	if (len > sizeof buf)
		len = sizeof buf;

	buf[0] = 0xaa;
	xfer[0].tx_buf = (unsigned long)buf;
	xfer[0].len = 1;

	xfer[1].rx_buf = (unsigned long) buf;
	xfer[1].len = len;

	status = ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSAGE");
		return;
	}

	printf("response(%2d, %2d): ", len, status);
	for (bp = buf; len; len--)
		printf(" %02x", *bp++);
	printf("\n");
}

static void dumpstat(const char *name, int fd)
{
	__u8	mode, lsb, bits;
	__u32	speed;

	if (ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0) {
		perror("SPI rd_mode");
		return;
	}
	if (ioctl(fd, SPI_IOC_RD_LSB_FIRST, &lsb) < 0) {
		perror("SPI rd_lsb_fist");
		return;
	}
	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
		perror("SPI bits_per_word");
		return;
	}
	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
		perror("SPI max_speed_hz");
		return;
	}

	printf("%s: spi mode %d, %d bits %sper word, %d Hz max\n",
		name, mode, bits, lsb ? "(lsb first) " : "", speed);
}

int main(int argc, char **argv)
{
	int		c;
	int		readcount = 0;
	int		msglen = 0;
	int		dumpcount = 0;
	int		logcount = 0;
	int		fd;
	int             i;
	const char	*name;

	while ((c = getopt(argc, argv, "hm:r:l:d:v")) != EOF) {
		switch (c) {
		case 'm':
			msglen = atoi(optarg);
			if (msglen < 0)
				goto usage;
			continue;
		case 'r':
			readcount = atoi(optarg);
			if (readcount < 0)
				goto usage;
			continue;
		case 'l':
			logcount = atoi(optarg);
			if (logcount < 0)
				goto usage;
			continue;
		case 'd':
			dumpcount = atoi(optarg);
			if (dumpcount < 0)
				goto usage;
			continue;
		case 'v':
			verbose++;
			continue;
		case 'h':
		case '?':
usage:
			fprintf(stderr,
				"usage: %s [-h] [-d N] [-m N] [-r N] /dev/spidevB.D\n",
				argv[0]);
			return 1;
		}
	}

	fprintf(stdout, "Starting the OpenPilot SPI server\n");

	if ((optind + 1) != argc)
		goto usage;
	name = argv[optind];

	fd = open(name, O_RDWR);
	if (fd < 0) {
		perror("open");
		return 1;
	}

	__u32 speed = 10000000;
	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		perror("cannot set SPI max_speed_hz");
		return;
	}

	__u8 mode = 3;
	if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
		perror("cannot set SPI wr_mode");
		return;
	}
	dumpstat(name, fd);

	if (msglen)
		do_msg(fd, msglen);

	if (readcount)
		do_read(fd, readcount);

	if (logcount) {
		received_bytes = 0;
		file_fd = fopen("/home/root/log.dat", "w");
		file_fd_err = fopen("/home/root/raw_err.dat", "w");

		// Initialize the uavTalk object
		UAVObjInitialize();
		UAVObjectsInitializeAll();
		uavTalk = UAVTalkInitialize(NULL);

		UAVTalkStats stats;

		for (i = 0; i < logcount; i++) {
			if ((i % 500) == 0) {
				UAVTalkGetStats(uavTalk, &stats);
				fprintf(stdout, "Grabbing %d packet.  Received %d bytes.  Received %d objects.\n", i, received_bytes, stats.rxObjects);
			}
			grab_log_packet(fd, file_fd);	
			usleep(500);
		}
		fclose(file_fd);
		fclose(file_fd_err);
	}
	if (dumpcount) {
		received_bytes = 0;
		file_fd = fopen("/home/root/raw.dat", "w");
		for (i = 0; i < dumpcount; i++) {
			if ((i % 500) == 0)
				fprintf(stdout, "Grabbing %d packet.  Received %d bytes\n", i, received_bytes);
			to_disk(fd, file_fd);	
			usleep(500);
		}
		fclose(file_fd);
	}
	close(fd);
	return 0;
}
