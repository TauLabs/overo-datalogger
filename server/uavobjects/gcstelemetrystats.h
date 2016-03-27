/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup GCSTelemetryStats GCSTelemetryStats
 * @brief The telemetry statistics from the ground computer
 *
 *
 * @file       gcstelemetrystats.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the GCSTelemetryStats object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: gcstelemetrystats.xml. 
 *             This is an automatically generated file.
 *             DO NOT modify manually.
 *
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

#ifndef GCSTELEMETRYSTATS_H
#define GCSTELEMETRYSTATS_H

#include "pios_queue.h"
#include "uavoversion.h"



// Object constants
#define GCSTELEMETRYSTATS_OBJID 0xABC72744
#define GCSTELEMETRYSTATS_ISSINGLEINST 1
#define GCSTELEMETRYSTATS_ISSETTINGS 0
#define GCSTELEMETRYSTATS_NUMBYTES 21

// Generic interface functions
int32_t GCSTelemetryStatsInitialize();
UAVObjHandle GCSTelemetryStatsHandle();
void GCSTelemetryStatsSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    float TxDataRate;
    float RxDataRate;
    uint32_t TxFailures;
    uint32_t RxFailures;
    uint32_t TxRetries;
    uint8_t Status;

} __attribute__((packed)) __attribute__((aligned(4))) GCSTelemetryStatsData;

// Typesafe Object access functions
/**
 * @function GCSTelemetryStatsGet(dataOut)
 * @brief Populate a GCSTelemetryStatsData object
 * @param[out] dataOut 
 */
static inline int32_t GCSTelemetryStatsGet(GCSTelemetryStatsData *dataOut) { return UAVObjGetData(GCSTelemetryStatsHandle(), dataOut); }

static inline int32_t GCSTelemetryStatsSet(const GCSTelemetryStatsData *dataIn) { return UAVObjSetData(GCSTelemetryStatsHandle(), dataIn); }

static inline int32_t GCSTelemetryStatsInstGet(uint16_t instId, GCSTelemetryStatsData *dataOut) { return UAVObjGetInstanceData(GCSTelemetryStatsHandle(), instId, dataOut); }

static inline int32_t GCSTelemetryStatsInstSet(uint16_t instId, const GCSTelemetryStatsData *dataIn) { return UAVObjSetInstanceData(GCSTelemetryStatsHandle(), instId, dataIn); }

static inline int32_t GCSTelemetryStatsConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(GCSTelemetryStatsHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t GCSTelemetryStatsConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(GCSTelemetryStatsHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t GCSTelemetryStatsCreateInstance() { return UAVObjCreateInstance(GCSTelemetryStatsHandle(), &GCSTelemetryStatsSetDefaults); }

static inline void GCSTelemetryStatsRequestUpdate() { UAVObjRequestUpdate(GCSTelemetryStatsHandle()); }

static inline void GCSTelemetryStatsRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(GCSTelemetryStatsHandle(), instId); }

static inline void GCSTelemetryStatsUpdated() { UAVObjUpdated(GCSTelemetryStatsHandle()); }

static inline void GCSTelemetryStatsInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(GCSTelemetryStatsHandle(), instId); }

static inline int32_t GCSTelemetryStatsGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(GCSTelemetryStatsHandle(), dataOut); }

static inline int32_t GCSTelemetryStatsSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(GCSTelemetryStatsHandle(), dataIn); }

static inline int8_t GCSTelemetryStatsReadOnly() { return UAVObjReadOnly(GCSTelemetryStatsHandle()); }

static inline uint16_t GCSTelemetryStatsGetNumInstances(){ return UAVObjGetNumInstances(GCSTelemetryStatsHandle()); }

static inline uint32_t GCSTelemetryStatsGetNumBytes(){ return UAVObjGetNumBytes(GCSTelemetryStatsHandle()); }

// Field information
// Field TxDataRate information
// Field RxDataRate information
// Field TxFailures information
// Field RxFailures information
// Field TxRetries information
// Field Status information
/* Enumeration options for field Status */
typedef enum { GCSTELEMETRYSTATS_STATUS_DISCONNECTED=0, GCSTELEMETRYSTATS_STATUS_HANDSHAKEREQ=1, GCSTELEMETRYSTATS_STATUS_HANDSHAKEACK=2, GCSTELEMETRYSTATS_STATUS_CONNECTED=3 }  __attribute__((packed)) GCSTelemetryStatsStatusOptions;
/* Max value of any option in topmost parent Status of field Status */
#define GCSTELEMETRYSTATS_STATUS_GLOBAL_MAXOPTVAL 3
/* Max value of any option in field Status */
#define GCSTELEMETRYSTATS_STATUS_MAXOPTVAL 3
/* Ensure field Status contains valid data */
static inline bool GCSTelemetryStatsStatusIsValid( uint8_t CurrentStatus ) { return CurrentStatus < GCSTELEMETRYSTATS_STATUS_MAXOPTVAL; }


// set/Get functions
extern void GCSTelemetryStatsTxDataRateSet( float *NewTxDataRate );
extern void GCSTelemetryStatsTxDataRateGet( float *NewTxDataRate );
extern void GCSTelemetryStatsRxDataRateSet( float *NewRxDataRate );
extern void GCSTelemetryStatsRxDataRateGet( float *NewRxDataRate );
extern void GCSTelemetryStatsTxFailuresSet( uint32_t *NewTxFailures );
extern void GCSTelemetryStatsTxFailuresGet( uint32_t *NewTxFailures );
extern void GCSTelemetryStatsRxFailuresSet( uint32_t *NewRxFailures );
extern void GCSTelemetryStatsRxFailuresGet( uint32_t *NewRxFailures );
extern void GCSTelemetryStatsTxRetriesSet( uint32_t *NewTxRetries );
extern void GCSTelemetryStatsTxRetriesGet( uint32_t *NewTxRetries );
extern void GCSTelemetryStatsStatusSet( uint8_t *NewStatus );
extern void GCSTelemetryStatsStatusGet( uint8_t *NewStatus );


#endif // GCSTELEMETRYSTATS_H

/**
 * @}
 * @}
 */
