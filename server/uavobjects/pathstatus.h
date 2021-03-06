/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup PathStatus PathStatus
 * @brief Status of the current path mode  Can come from any @ref PathFollower module
 *
 *
 * @file       pathstatus.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the PathStatus object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: pathstatus.xml. 
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

#ifndef PATHSTATUS_H
#define PATHSTATUS_H

#include "pios_queue.h"
#include "uavoversion.h"



// Object constants
#define PATHSTATUS_OBJID 0xF8A5279A
#define PATHSTATUS_ISSINGLEINST 1
#define PATHSTATUS_ISSETTINGS 0
#define PATHSTATUS_NUMBYTES 11

// Generic interface functions
int32_t PathStatusInitialize();
UAVObjHandle PathStatusHandle();
void PathStatusSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    float fractional_progress;
    float error;
    int16_t Waypoint;
    uint8_t Status;

} __attribute__((packed)) __attribute__((aligned(4))) PathStatusData;

// Typesafe Object access functions
/**
 * @function PathStatusGet(dataOut)
 * @brief Populate a PathStatusData object
 * @param[out] dataOut 
 */
static inline int32_t PathStatusGet(PathStatusData *dataOut) { return UAVObjGetData(PathStatusHandle(), dataOut); }

static inline int32_t PathStatusSet(const PathStatusData *dataIn) { return UAVObjSetData(PathStatusHandle(), dataIn); }

static inline int32_t PathStatusInstGet(uint16_t instId, PathStatusData *dataOut) { return UAVObjGetInstanceData(PathStatusHandle(), instId, dataOut); }

static inline int32_t PathStatusInstSet(uint16_t instId, const PathStatusData *dataIn) { return UAVObjSetInstanceData(PathStatusHandle(), instId, dataIn); }

static inline int32_t PathStatusConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(PathStatusHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t PathStatusConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(PathStatusHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t PathStatusCreateInstance() { return UAVObjCreateInstance(PathStatusHandle(), &PathStatusSetDefaults); }

static inline void PathStatusRequestUpdate() { UAVObjRequestUpdate(PathStatusHandle()); }

static inline void PathStatusRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(PathStatusHandle(), instId); }

static inline void PathStatusUpdated() { UAVObjUpdated(PathStatusHandle()); }

static inline void PathStatusInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(PathStatusHandle(), instId); }

static inline int32_t PathStatusGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(PathStatusHandle(), dataOut); }

static inline int32_t PathStatusSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(PathStatusHandle(), dataIn); }

static inline int8_t PathStatusReadOnly() { return UAVObjReadOnly(PathStatusHandle()); }

static inline uint16_t PathStatusGetNumInstances(){ return UAVObjGetNumInstances(PathStatusHandle()); }

static inline uint32_t PathStatusGetNumBytes(){ return UAVObjGetNumBytes(PathStatusHandle()); }

// Field information
// Field fractional_progress information
// Field error information
// Field Waypoint information
// Field Status information
/* Enumeration options for field Status */
typedef enum { PATHSTATUS_STATUS_INPROGRESS=0, PATHSTATUS_STATUS_COMPLETED=1, PATHSTATUS_STATUS_WARNING=2, PATHSTATUS_STATUS_CRITICAL=3 }  __attribute__((packed)) PathStatusStatusOptions;
/* Max value of any option in topmost parent Status of field Status */
#define PATHSTATUS_STATUS_GLOBAL_MAXOPTVAL 3
/* Max value of any option in field Status */
#define PATHSTATUS_STATUS_MAXOPTVAL 3
/* Ensure field Status contains valid data */
static inline bool PathStatusStatusIsValid( uint8_t CurrentStatus ) { return CurrentStatus < PATHSTATUS_STATUS_MAXOPTVAL; }


// set/Get functions
extern void PathStatusfractional_progressSet( float *Newfractional_progress );
extern void PathStatusfractional_progressGet( float *Newfractional_progress );
extern void PathStatuserrorSet( float *Newerror );
extern void PathStatuserrorGet( float *Newerror );
extern void PathStatusWaypointSet( int16_t *NewWaypoint );
extern void PathStatusWaypointGet( int16_t *NewWaypoint );
extern void PathStatusStatusSet( uint8_t *NewStatus );
extern void PathStatusStatusGet( uint8_t *NewStatus );


#endif // PATHSTATUS_H

/**
 * @}
 * @}
 */
