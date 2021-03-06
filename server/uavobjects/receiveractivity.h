/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup ReceiverActivity ReceiverActivity
 * @brief Monitors which receiver channels have been active within the last second.
 *
 *
 * @file       receiveractivity.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the ReceiverActivity object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: receiveractivity.xml. 
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

#ifndef RECEIVERACTIVITY_H
#define RECEIVERACTIVITY_H

#include "pios_queue.h"
#include "uavoversion.h"



// Object constants
#define RECEIVERACTIVITY_OBJID 0x1ABCFB7E
#define RECEIVERACTIVITY_ISSINGLEINST 1
#define RECEIVERACTIVITY_ISSETTINGS 0
#define RECEIVERACTIVITY_NUMBYTES 2

// Generic interface functions
int32_t ReceiverActivityInitialize();
UAVObjHandle ReceiverActivityHandle();
void ReceiverActivitySetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    uint8_t ActiveGroup;
    uint8_t ActiveChannel;

} __attribute__((packed)) __attribute__((aligned(4))) ReceiverActivityData;

// Typesafe Object access functions
/**
 * @function ReceiverActivityGet(dataOut)
 * @brief Populate a ReceiverActivityData object
 * @param[out] dataOut 
 */
static inline int32_t ReceiverActivityGet(ReceiverActivityData *dataOut) { return UAVObjGetData(ReceiverActivityHandle(), dataOut); }

static inline int32_t ReceiverActivitySet(const ReceiverActivityData *dataIn) { return UAVObjSetData(ReceiverActivityHandle(), dataIn); }

static inline int32_t ReceiverActivityInstGet(uint16_t instId, ReceiverActivityData *dataOut) { return UAVObjGetInstanceData(ReceiverActivityHandle(), instId, dataOut); }

static inline int32_t ReceiverActivityInstSet(uint16_t instId, const ReceiverActivityData *dataIn) { return UAVObjSetInstanceData(ReceiverActivityHandle(), instId, dataIn); }

static inline int32_t ReceiverActivityConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(ReceiverActivityHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t ReceiverActivityConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(ReceiverActivityHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t ReceiverActivityCreateInstance() { return UAVObjCreateInstance(ReceiverActivityHandle(), &ReceiverActivitySetDefaults); }

static inline void ReceiverActivityRequestUpdate() { UAVObjRequestUpdate(ReceiverActivityHandle()); }

static inline void ReceiverActivityRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(ReceiverActivityHandle(), instId); }

static inline void ReceiverActivityUpdated() { UAVObjUpdated(ReceiverActivityHandle()); }

static inline void ReceiverActivityInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(ReceiverActivityHandle(), instId); }

static inline int32_t ReceiverActivityGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(ReceiverActivityHandle(), dataOut); }

static inline int32_t ReceiverActivitySetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(ReceiverActivityHandle(), dataIn); }

static inline int8_t ReceiverActivityReadOnly() { return UAVObjReadOnly(ReceiverActivityHandle()); }

static inline uint16_t ReceiverActivityGetNumInstances(){ return UAVObjGetNumInstances(ReceiverActivityHandle()); }

static inline uint32_t ReceiverActivityGetNumBytes(){ return UAVObjGetNumBytes(ReceiverActivityHandle()); }

// Field information
// Field ActiveGroup information
/* Enumeration options for field ActiveGroup */
typedef enum { RECEIVERACTIVITY_ACTIVEGROUP_PWM=0, RECEIVERACTIVITY_ACTIVEGROUP_PPM=1, RECEIVERACTIVITY_ACTIVEGROUP_DSM=2, RECEIVERACTIVITY_ACTIVEGROUP_SBUS=3, RECEIVERACTIVITY_ACTIVEGROUP_RFM22B=4, RECEIVERACTIVITY_ACTIVEGROUP_OPENLRS=5, RECEIVERACTIVITY_ACTIVEGROUP_GCS=6, RECEIVERACTIVITY_ACTIVEGROUP_HOTTSUM=7, RECEIVERACTIVITY_ACTIVEGROUP_NONE=8 }  __attribute__((packed)) ReceiverActivityActiveGroupOptions;
/* Max value of any option in topmost parent ActiveGroup of field ActiveGroup */
#define RECEIVERACTIVITY_ACTIVEGROUP_GLOBAL_MAXOPTVAL 8
/* Max value of any option in field ActiveGroup */
#define RECEIVERACTIVITY_ACTIVEGROUP_MAXOPTVAL 8
/* Ensure field ActiveGroup contains valid data */
static inline bool ReceiverActivityActiveGroupIsValid( uint8_t CurrentActiveGroup ) { return CurrentActiveGroup < RECEIVERACTIVITY_ACTIVEGROUP_MAXOPTVAL; }
// Field ActiveChannel information


// set/Get functions
extern void ReceiverActivityActiveGroupSet( uint8_t *NewActiveGroup );
extern void ReceiverActivityActiveGroupGet( uint8_t *NewActiveGroup );
extern void ReceiverActivityActiveChannelSet( uint8_t *NewActiveChannel );
extern void ReceiverActivityActiveChannelGet( uint8_t *NewActiveChannel );


#endif // RECEIVERACTIVITY_H

/**
 * @}
 * @}
 */
