/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup RFM22BStatus RFM22BStatus
 * @brief RFM22B link status.
 *
 *
 * @file       rfm22bstatus.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the RFM22BStatus object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: rfm22bstatus.xml. 
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

#ifndef RFM22BSTATUS_H
#define RFM22BSTATUS_H

#include "pios_queue.h"
#include "uavoversion.h"



// Object constants
#define RFM22BSTATUS_OBJID 0x2026FD3C
#define RFM22BSTATUS_ISSINGLEINST 0
#define RFM22BSTATUS_ISSETTINGS 0
#define RFM22BSTATUS_NUMBYTES 24

// Generic interface functions
int32_t RFM22BStatusInitialize();
UAVObjHandle RFM22BStatusHandle();
void RFM22BStatusSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    uint32_t DeviceID;
    uint16_t BoardRevision;
    uint16_t HeapRemaining;
    uint16_t TXRate;
    uint16_t RXRate;
    uint8_t BoardType;
    uint8_t RxGood;
    uint8_t RxCorrected;
    uint8_t RxErrors;
    uint8_t RxSyncMissed;
    uint8_t TxMissed;
    uint8_t RxFailure;
    uint8_t Resets;
    uint8_t Timeouts;
    int8_t RSSI;
    uint8_t LinkQuality;
    uint8_t LinkState;

} __attribute__((packed)) __attribute__((aligned(4))) RFM22BStatusData;

// Typesafe Object access functions
/**
 * @function RFM22BStatusGet(dataOut)
 * @brief Populate a RFM22BStatusData object
 * @param[out] dataOut 
 */
static inline int32_t RFM22BStatusGet(RFM22BStatusData *dataOut) { return UAVObjGetData(RFM22BStatusHandle(), dataOut); }

static inline int32_t RFM22BStatusSet(const RFM22BStatusData *dataIn) { return UAVObjSetData(RFM22BStatusHandle(), dataIn); }

static inline int32_t RFM22BStatusInstGet(uint16_t instId, RFM22BStatusData *dataOut) { return UAVObjGetInstanceData(RFM22BStatusHandle(), instId, dataOut); }

static inline int32_t RFM22BStatusInstSet(uint16_t instId, const RFM22BStatusData *dataIn) { return UAVObjSetInstanceData(RFM22BStatusHandle(), instId, dataIn); }

static inline int32_t RFM22BStatusConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(RFM22BStatusHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t RFM22BStatusConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(RFM22BStatusHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t RFM22BStatusCreateInstance() { return UAVObjCreateInstance(RFM22BStatusHandle(), &RFM22BStatusSetDefaults); }

static inline void RFM22BStatusRequestUpdate() { UAVObjRequestUpdate(RFM22BStatusHandle()); }

static inline void RFM22BStatusRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(RFM22BStatusHandle(), instId); }

static inline void RFM22BStatusUpdated() { UAVObjUpdated(RFM22BStatusHandle()); }

static inline void RFM22BStatusInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(RFM22BStatusHandle(), instId); }

static inline int32_t RFM22BStatusGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(RFM22BStatusHandle(), dataOut); }

static inline int32_t RFM22BStatusSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(RFM22BStatusHandle(), dataIn); }

static inline int8_t RFM22BStatusReadOnly() { return UAVObjReadOnly(RFM22BStatusHandle()); }

static inline uint16_t RFM22BStatusGetNumInstances(){ return UAVObjGetNumInstances(RFM22BStatusHandle()); }

static inline uint32_t RFM22BStatusGetNumBytes(){ return UAVObjGetNumBytes(RFM22BStatusHandle()); }

// Field information
// Field DeviceID information
// Field BoardRevision information
// Field HeapRemaining information
// Field TXRate information
// Field RXRate information
// Field BoardType information
// Field RxGood information
// Field RxCorrected information
// Field RxErrors information
// Field RxSyncMissed information
// Field TxMissed information
// Field RxFailure information
// Field Resets information
// Field Timeouts information
// Field RSSI information
// Field LinkQuality information
// Field LinkState information
/* Enumeration options for field LinkState */
typedef enum { RFM22BSTATUS_LINKSTATE_DISABLED=0, RFM22BSTATUS_LINKSTATE_ENABLED=1, RFM22BSTATUS_LINKSTATE_DISCONNECTED=2, RFM22BSTATUS_LINKSTATE_CONNECTED=3 }  __attribute__((packed)) RFM22BStatusLinkStateOptions;
/* Max value of any option in topmost parent LinkState of field LinkState */
#define RFM22BSTATUS_LINKSTATE_GLOBAL_MAXOPTVAL 3
/* Max value of any option in field LinkState */
#define RFM22BSTATUS_LINKSTATE_MAXOPTVAL 3
/* Ensure field LinkState contains valid data */
static inline bool RFM22BStatusLinkStateIsValid( uint8_t CurrentLinkState ) { return CurrentLinkState < RFM22BSTATUS_LINKSTATE_MAXOPTVAL; }


// set/Get functions
extern void RFM22BStatusDeviceIDSet( uint32_t *NewDeviceID );
extern void RFM22BStatusDeviceIDGet( uint32_t *NewDeviceID );
extern void RFM22BStatusBoardRevisionSet( uint16_t *NewBoardRevision );
extern void RFM22BStatusBoardRevisionGet( uint16_t *NewBoardRevision );
extern void RFM22BStatusHeapRemainingSet( uint16_t *NewHeapRemaining );
extern void RFM22BStatusHeapRemainingGet( uint16_t *NewHeapRemaining );
extern void RFM22BStatusTXRateSet( uint16_t *NewTXRate );
extern void RFM22BStatusTXRateGet( uint16_t *NewTXRate );
extern void RFM22BStatusRXRateSet( uint16_t *NewRXRate );
extern void RFM22BStatusRXRateGet( uint16_t *NewRXRate );
extern void RFM22BStatusBoardTypeSet( uint8_t *NewBoardType );
extern void RFM22BStatusBoardTypeGet( uint8_t *NewBoardType );
extern void RFM22BStatusRxGoodSet( uint8_t *NewRxGood );
extern void RFM22BStatusRxGoodGet( uint8_t *NewRxGood );
extern void RFM22BStatusRxCorrectedSet( uint8_t *NewRxCorrected );
extern void RFM22BStatusRxCorrectedGet( uint8_t *NewRxCorrected );
extern void RFM22BStatusRxErrorsSet( uint8_t *NewRxErrors );
extern void RFM22BStatusRxErrorsGet( uint8_t *NewRxErrors );
extern void RFM22BStatusRxSyncMissedSet( uint8_t *NewRxSyncMissed );
extern void RFM22BStatusRxSyncMissedGet( uint8_t *NewRxSyncMissed );
extern void RFM22BStatusTxMissedSet( uint8_t *NewTxMissed );
extern void RFM22BStatusTxMissedGet( uint8_t *NewTxMissed );
extern void RFM22BStatusRxFailureSet( uint8_t *NewRxFailure );
extern void RFM22BStatusRxFailureGet( uint8_t *NewRxFailure );
extern void RFM22BStatusResetsSet( uint8_t *NewResets );
extern void RFM22BStatusResetsGet( uint8_t *NewResets );
extern void RFM22BStatusTimeoutsSet( uint8_t *NewTimeouts );
extern void RFM22BStatusTimeoutsGet( uint8_t *NewTimeouts );
extern void RFM22BStatusRSSISet( int8_t *NewRSSI );
extern void RFM22BStatusRSSIGet( int8_t *NewRSSI );
extern void RFM22BStatusLinkQualitySet( uint8_t *NewLinkQuality );
extern void RFM22BStatusLinkQualityGet( uint8_t *NewLinkQuality );
extern void RFM22BStatusLinkStateSet( uint8_t *NewLinkState );
extern void RFM22BStatusLinkStateGet( uint8_t *NewLinkState );


#endif // RFM22BSTATUS_H

/**
 * @}
 * @}
 */
