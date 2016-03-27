/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup FlightStatsSettings FlightStatsSettings
 * @brief Settings for the FlightStats module
 *
 *
 * @file       flightstatssettings.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the FlightStatsSettings object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: flightstatssettings.xml. 
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

#ifndef FLIGHTSTATSSETTINGS_H
#define FLIGHTSTATSSETTINGS_H

#include "pios_queue.h"
#include "uavoversion.h"



// Object constants
#define FLIGHTSTATSSETTINGS_OBJID 0x88BAE028
#define FLIGHTSTATSSETTINGS_ISSINGLEINST 1
#define FLIGHTSTATSSETTINGS_ISSETTINGS 1
#define FLIGHTSTATSSETTINGS_NUMBYTES 1

// Generic interface functions
int32_t FlightStatsSettingsInitialize();
UAVObjHandle FlightStatsSettingsHandle();
void FlightStatsSettingsSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    uint8_t StatsBehavior;

} __attribute__((packed)) __attribute__((aligned(4))) FlightStatsSettingsData;

// Typesafe Object access functions
/**
 * @function FlightStatsSettingsGet(dataOut)
 * @brief Populate a FlightStatsSettingsData object
 * @param[out] dataOut 
 */
static inline int32_t FlightStatsSettingsGet(FlightStatsSettingsData *dataOut) { return UAVObjGetData(FlightStatsSettingsHandle(), dataOut); }

static inline int32_t FlightStatsSettingsSet(const FlightStatsSettingsData *dataIn) { return UAVObjSetData(FlightStatsSettingsHandle(), dataIn); }

static inline int32_t FlightStatsSettingsInstGet(uint16_t instId, FlightStatsSettingsData *dataOut) { return UAVObjGetInstanceData(FlightStatsSettingsHandle(), instId, dataOut); }

static inline int32_t FlightStatsSettingsInstSet(uint16_t instId, const FlightStatsSettingsData *dataIn) { return UAVObjSetInstanceData(FlightStatsSettingsHandle(), instId, dataIn); }

static inline int32_t FlightStatsSettingsConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(FlightStatsSettingsHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t FlightStatsSettingsConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(FlightStatsSettingsHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t FlightStatsSettingsCreateInstance() { return UAVObjCreateInstance(FlightStatsSettingsHandle(), &FlightStatsSettingsSetDefaults); }

static inline void FlightStatsSettingsRequestUpdate() { UAVObjRequestUpdate(FlightStatsSettingsHandle()); }

static inline void FlightStatsSettingsRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(FlightStatsSettingsHandle(), instId); }

static inline void FlightStatsSettingsUpdated() { UAVObjUpdated(FlightStatsSettingsHandle()); }

static inline void FlightStatsSettingsInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(FlightStatsSettingsHandle(), instId); }

static inline int32_t FlightStatsSettingsGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(FlightStatsSettingsHandle(), dataOut); }

static inline int32_t FlightStatsSettingsSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(FlightStatsSettingsHandle(), dataIn); }

static inline int8_t FlightStatsSettingsReadOnly() { return UAVObjReadOnly(FlightStatsSettingsHandle()); }

static inline uint16_t FlightStatsSettingsGetNumInstances(){ return UAVObjGetNumInstances(FlightStatsSettingsHandle()); }

static inline uint32_t FlightStatsSettingsGetNumBytes(){ return UAVObjGetNumBytes(FlightStatsSettingsHandle()); }

// Field information
// Field StatsBehavior information
/* Enumeration options for field StatsBehavior */
typedef enum { FLIGHTSTATSSETTINGS_STATSBEHAVIOR_RESETONARM=0, FLIGHTSTATSSETTINGS_STATSBEHAVIOR_RESETONBOOT=1 }  __attribute__((packed)) FlightStatsSettingsStatsBehaviorOptions;
/* Max value of any option in topmost parent StatsBehavior of field StatsBehavior */
#define FLIGHTSTATSSETTINGS_STATSBEHAVIOR_GLOBAL_MAXOPTVAL 1
/* Max value of any option in field StatsBehavior */
#define FLIGHTSTATSSETTINGS_STATSBEHAVIOR_MAXOPTVAL 1
/* Ensure field StatsBehavior contains valid data */
static inline bool FlightStatsSettingsStatsBehaviorIsValid( uint8_t CurrentStatsBehavior ) { return CurrentStatsBehavior < FLIGHTSTATSSETTINGS_STATSBEHAVIOR_MAXOPTVAL; }


// set/Get functions
extern void FlightStatsSettingsStatsBehaviorSet( uint8_t *NewStatsBehavior );
extern void FlightStatsSettingsStatsBehaviorGet( uint8_t *NewStatsBehavior );


#endif // FLIGHTSTATSSETTINGS_H

/**
 * @}
 * @}
 */
