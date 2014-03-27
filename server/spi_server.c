/**
 ******************************************************************************
 *
 * @brief      A server that connects to Freedom and passes updates to the
 *             UAVObjectManager.  Based on the @ref OveroSyncSettings it
 *             will also log to a file.
 *
 * @file       spi_server.c
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

#include <signal.h>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/rtc.h>

#include "packet_handling.h"

#include "openpilot.h"
#include "uavtalk.h"
#include "uavobjectmanager.h"

#include "systemstats.h"
#include "gpstime.h"
#include "gpsposition.h"
#include "overosyncsettings.h"
#include "flightstatus.h"

//! Private methods
void sig_handler(int signum);

//! Private variables
int fd;                // The spidev device
FILE *file_fd;         // The log file
FILE *file_fd_err;     // The error log file
UAVTalkConnection uavTalk;

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
	int		delay_time = 2000;
	int		logcount = 0;
	int             verbose = 0;

	int             i;
	const char	*name;

	bool gps_time_set = false;

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
			delay_time = atoi(optarg);
			if (delay_time < 0)
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

	// Install the signal handler
	signal(SIGINT, sig_handler);

	fprintf(stdout, "Starting the Tau Labs SPI server\n");

	if ((optind + 1) != argc) {
		name = "/dev/spidev1.1";
	} else {
		name = argv[optind];
	}

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

	__u8 mode = 0;
	if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
		perror("cannot set SPI wr_mode");
		return;
	}
	dumpstat(name, fd);

	file_fd = NULL;
	file_fd_err = fopen("/home/root/raw_err.dat", "w");
	bool logging = false;
	bool new_logging;
	int received_bytes = 0;

	// Initialize the uavTalk object
	UAVObjInitialize();
	UAVObjectsInitializeAll();
	uavTalk = UAVTalkInitialize(NULL);

	UAVTalkStats stats;

	struct timeval this_time, last_time;
	struct timezone tz;
	tz.tz_minuteswest = 0;
	tz.tz_dsttime = 0;
	gettimeofday(&last_time, &tz);

	while (1) {
		received_bytes += process_packet(fd, logging);

		OveroSyncSettingsData settings;
		OveroSyncSettingsGet(&settings);

		switch(settings.LogOn) {
			case OVEROSYNCSETTINGS_LOGON_NEVER:
				new_logging = false;
				break;
			case OVEROSYNCSETTINGS_LOGON_ALWAYS:
				new_logging = true;
				break;
			case OVEROSYNCSETTINGS_LOGON_ARMED:
			{
				FlightStatusData flightStatus;
				FlightStatusGet(&flightStatus);
				new_logging = flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED;
			}
				break;
		}

		if (!logging && new_logging) {
			// Open a new log file
			time_t t;
			struct tm tm;
			char file_name[50];

			time(&t);
			gmtime_r(&t, &tm);
			strftime(file_name, sizeof(file_name), "/home/root/log_%Y%m%d_%H%M%S.dat", &tm);
			file_fd = fopen(file_name, "w");

			fprintf(stdout, "Starting logging to: %s\n", file_name);
			logging = new_logging;

		} else if (logging && !new_logging) {
			// Close the log file
			fclose(file_fd);
			file_fd = NULL;
			fprintf(stdout, "Stopping logging\n");
			logging = new_logging;
		} else {
			// No change
		}

		// Look for first GPS lock to set the system time
		if (!gps_time_set) {
			GPSTimeData gpsTime;
			GPSTimeGet(&gpsTime);
			if (gpsTime.Year > 2000) {
				
				struct timeval   current_time_seconds;
				struct timezone  timezone;
				struct tm       *current_time;

				gettimeofday(&current_time_seconds, &timezone);
				current_time = localtime((time_t *) &current_time_seconds.tv_sec);
				current_time->tm_year  = gpsTime.Year - 1900;
				current_time->tm_mon   = gpsTime.Month - 1;
				current_time->tm_mday  = gpsTime.Day;
				current_time->tm_hour  = gpsTime.Hour;
				current_time->tm_min   = gpsTime.Minute;
				current_time->tm_sec   = gpsTime.Second;
				current_time_seconds.tv_sec = mktime(current_time);;

				if (settimeofday(&current_time_seconds, &timezone) == 0) {
					fprintf(stdout, "Set the system time to some time in %u\n", gpsTime.Year);
				} else {
					fprintf(stdout, "Error setting the time\n");
				}

				gps_time_set = true;
			}
		}

		i++;
		if (i % 1000 == 0) {
			UAVTalkGetStats(uavTalk, &stats);
			SystemStatsData sysStats;
			SystemStatsGet(&sysStats);

			if (verbose) {
				fprintf(stdout, "Grabbing %d packet.  Received %d bytes.  Received %d objects.  Received %d errors.\n", i, received_bytes, stats.rxObjects, stats.rxErrors);
				fprintf(stdout, "Uptime: %d ms\n", sysStats.FlightTime);
			}
		}

		if (delay_time) {
			unsigned int d = 0;
			while(d < delay_time) {
				gettimeofday(&this_time, &tz);
				d = this_time.tv_usec - last_time.tv_usec;
			}
			last_time = this_time;
		}
	}
	
	return 0;
}

/**
 * Signal handler that closes out all the devices
 * @param signum the signal that was received
 */
void sig_handler(int signum) 
{
	switch(signum) {
		case SIGQUIT:
		case SIGINT:
			fprintf(stdout, "Shutting down.");
			if(file_fd) {
				fclose(file_fd);
				file_fd = NULL;
			}
			fclose(file_fd_err);
			ph_print_statistics();
			close(fd);
			exit(0);
			break;
		default:
			break;
	}
}
