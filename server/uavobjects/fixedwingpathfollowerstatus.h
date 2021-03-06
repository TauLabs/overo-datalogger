/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup FixedWingPathFollowerStatus FixedWingPathFollowerStatus
 * @brief Object Storing Debugging Information on PID internals
 *
 *
 * @file       fixedwingpathfollowerstatus.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the FixedWingPathFollowerStatus object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: fixedwingpathfollowerstatus.xml. 
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

#ifndef FIXEDWINGPATHFOLLOWERSTATUS_H
#define FIXEDWINGPATHFOLLOWERSTATUS_H

#include "pios_queue.h"
#include "uavoversion.h"



// Object constants
#define FIXEDWINGPATHFOLLOWERSTATUS_OBJID 0xACA80808
#define FIXEDWINGPATHFOLLOWERSTATUS_ISSINGLEINST 1
#define FIXEDWINGPATHFOLLOWERSTATUS_ISSETTINGS 0
#define FIXEDWINGPATHFOLLOWERSTATUS_NUMBYTES 56

// Generic interface functions
int32_t FixedWingPathFollowerStatusInitialize();
UAVObjHandle FixedWingPathFollowerStatusHandle();
void FixedWingPathFollowerStatusSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    float Error[4];
    float ErrorInt[4];
    float Command[4];
    uint8_t Errors[8];

} __attribute__((packed)) __attribute__((aligned(4))) FixedWingPathFollowerStatusData;

// Typesafe Object access functions
/**
 * @function FixedWingPathFollowerStatusGet(dataOut)
 * @brief Populate a FixedWingPathFollowerStatusData object
 * @param[out] dataOut 
 */
static inline int32_t FixedWingPathFollowerStatusGet(FixedWingPathFollowerStatusData *dataOut) { return UAVObjGetData(FixedWingPathFollowerStatusHandle(), dataOut); }

static inline int32_t FixedWingPathFollowerStatusSet(const FixedWingPathFollowerStatusData *dataIn) { return UAVObjSetData(FixedWingPathFollowerStatusHandle(), dataIn); }

static inline int32_t FixedWingPathFollowerStatusInstGet(uint16_t instId, FixedWingPathFollowerStatusData *dataOut) { return UAVObjGetInstanceData(FixedWingPathFollowerStatusHandle(), instId, dataOut); }

static inline int32_t FixedWingPathFollowerStatusInstSet(uint16_t instId, const FixedWingPathFollowerStatusData *dataIn) { return UAVObjSetInstanceData(FixedWingPathFollowerStatusHandle(), instId, dataIn); }

static inline int32_t FixedWingPathFollowerStatusConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(FixedWingPathFollowerStatusHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t FixedWingPathFollowerStatusConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(FixedWingPathFollowerStatusHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t FixedWingPathFollowerStatusCreateInstance() { return UAVObjCreateInstance(FixedWingPathFollowerStatusHandle(), &FixedWingPathFollowerStatusSetDefaults); }

static inline void FixedWingPathFollowerStatusRequestUpdate() { UAVObjRequestUpdate(FixedWingPathFollowerStatusHandle()); }

static inline void FixedWingPathFollowerStatusRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(FixedWingPathFollowerStatusHandle(), instId); }

static inline void FixedWingPathFollowerStatusUpdated() { UAVObjUpdated(FixedWingPathFollowerStatusHandle()); }

static inline void FixedWingPathFollowerStatusInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(FixedWingPathFollowerStatusHandle(), instId); }

static inline int32_t FixedWingPathFollowerStatusGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(FixedWingPathFollowerStatusHandle(), dataOut); }

static inline int32_t FixedWingPathFollowerStatusSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(FixedWingPathFollowerStatusHandle(), dataIn); }

static inline int8_t FixedWingPathFollowerStatusReadOnly() { return UAVObjReadOnly(FixedWingPathFollowerStatusHandle()); }

static inline uint16_t FixedWingPathFollowerStatusGetNumInstances(){ return UAVObjGetNumInstances(FixedWingPathFollowerStatusHandle()); }

static inline uint32_t FixedWingPathFollowerStatusGetNumBytes(){ return UAVObjGetNumBytes(FixedWingPathFollowerStatusHandle()); }

// Field information
// Field Error information
/* Array element names for field Error */
typedef enum { FIXEDWINGPATHFOLLOWERSTATUS_ERROR_BEARING=0, FIXEDWINGPATHFOLLOWERSTATUS_ERROR_SPEED=1, FIXEDWINGPATHFOLLOWERSTATUS_ERROR_ACCEL=2, FIXEDWINGPATHFOLLOWERSTATUS_ERROR_POWER=3 } __attribute__((packed)) FixedWingPathFollowerStatusErrorElem;
/* Number of elements for field Error */
#define FIXEDWINGPATHFOLLOWERSTATUS_ERROR_NUMELEM 4
// Field ErrorInt information
/* Array element names for field ErrorInt */
typedef enum { FIXEDWINGPATHFOLLOWERSTATUS_ERRORINT_BEARING=0, FIXEDWINGPATHFOLLOWERSTATUS_ERRORINT_SPEED=1, FIXEDWINGPATHFOLLOWERSTATUS_ERRORINT_ACCEL=2, FIXEDWINGPATHFOLLOWERSTATUS_ERRORINT_POWER=3 } __attribute__((packed)) FixedWingPathFollowerStatusErrorIntElem;
/* Number of elements for field ErrorInt */
#define FIXEDWINGPATHFOLLOWERSTATUS_ERRORINT_NUMELEM 4
// Field Command information
/* Array element names for field Command */
typedef enum { FIXEDWINGPATHFOLLOWERSTATUS_COMMAND_BEARING=0, FIXEDWINGPATHFOLLOWERSTATUS_COMMAND_SPEED=1, FIXEDWINGPATHFOLLOWERSTATUS_COMMAND_ACCEL=2, FIXEDWINGPATHFOLLOWERSTATUS_COMMAND_POWER=3 } __attribute__((packed)) FixedWingPathFollowerStatusCommandElem;
/* Number of elements for field Command */
#define FIXEDWINGPATHFOLLOWERSTATUS_COMMAND_NUMELEM 4
// Field Errors information
/* Array element names for field Errors */
typedef enum { FIXEDWINGPATHFOLLOWERSTATUS_ERRORS_WIND=0, FIXEDWINGPATHFOLLOWERSTATUS_ERRORS_STALLSPEED=1, FIXEDWINGPATHFOLLOWERSTATUS_ERRORS_LOWSPEED=2, FIXEDWINGPATHFOLLOWERSTATUS_ERRORS_HIGHSPEED=3, FIXEDWINGPATHFOLLOWERSTATUS_ERRORS_OVERSPEED=4, FIXEDWINGPATHFOLLOWERSTATUS_ERRORS_LOWPOWER=5, FIXEDWINGPATHFOLLOWERSTATUS_ERRORS_HIGHPOWER=6, FIXEDWINGPATHFOLLOWERSTATUS_ERRORS_PITCHCONTROL=7 } __attribute__((packed)) FixedWingPathFollowerStatusErrorsElem;
/* Number of elements for field Errors */
#define FIXEDWINGPATHFOLLOWERSTATUS_ERRORS_NUMELEM 8


// set/Get functions
extern void FixedWingPathFollowerStatusErrorSet( float *NewError );
extern void FixedWingPathFollowerStatusErrorGet( float *NewError );
extern void FixedWingPathFollowerStatusErrorIntSet( float *NewErrorInt );
extern void FixedWingPathFollowerStatusErrorIntGet( float *NewErrorInt );
extern void FixedWingPathFollowerStatusCommandSet( float *NewCommand );
extern void FixedWingPathFollowerStatusCommandGet( float *NewCommand );
extern void FixedWingPathFollowerStatusErrorsSet( uint8_t *NewErrors );
extern void FixedWingPathFollowerStatusErrorsGet( uint8_t *NewErrors );


#endif // FIXEDWINGPATHFOLLOWERSTATUS_H

/**
 * @}
 * @}
 */
