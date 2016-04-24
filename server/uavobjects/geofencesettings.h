/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup GeoFenceSettings GeoFenceSettings
 * @brief Radius for simple geofence boundaries
 *
 *
 * @file       geofencesettings.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the GeoFenceSettings object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: geofencesettings.xml. 
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

#ifndef GEOFENCESETTINGS_H
#define GEOFENCESETTINGS_H

#include "pios_queue.h"
#include "uavoversion.h"



// Object constants
#define GEOFENCESETTINGS_OBJID 0xDF5EA7FE
#define GEOFENCESETTINGS_ISSINGLEINST 1
#define GEOFENCESETTINGS_ISSETTINGS 1
#define GEOFENCESETTINGS_NUMBYTES 4

// Generic interface functions
int32_t GeoFenceSettingsInitialize();
UAVObjHandle GeoFenceSettingsHandle();
void GeoFenceSettingsSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    uint16_t WarningRadius;
    uint16_t ErrorRadius;

} __attribute__((packed)) __attribute__((aligned(4))) GeoFenceSettingsData;

// Typesafe Object access functions
/**
 * @function GeoFenceSettingsGet(dataOut)
 * @brief Populate a GeoFenceSettingsData object
 * @param[out] dataOut 
 */
static inline int32_t GeoFenceSettingsGet(GeoFenceSettingsData *dataOut) { return UAVObjGetData(GeoFenceSettingsHandle(), dataOut); }

static inline int32_t GeoFenceSettingsSet(const GeoFenceSettingsData *dataIn) { return UAVObjSetData(GeoFenceSettingsHandle(), dataIn); }

static inline int32_t GeoFenceSettingsInstGet(uint16_t instId, GeoFenceSettingsData *dataOut) { return UAVObjGetInstanceData(GeoFenceSettingsHandle(), instId, dataOut); }

static inline int32_t GeoFenceSettingsInstSet(uint16_t instId, const GeoFenceSettingsData *dataIn) { return UAVObjSetInstanceData(GeoFenceSettingsHandle(), instId, dataIn); }

static inline int32_t GeoFenceSettingsConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(GeoFenceSettingsHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t GeoFenceSettingsConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(GeoFenceSettingsHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t GeoFenceSettingsCreateInstance() { return UAVObjCreateInstance(GeoFenceSettingsHandle(), &GeoFenceSettingsSetDefaults); }

static inline void GeoFenceSettingsRequestUpdate() { UAVObjRequestUpdate(GeoFenceSettingsHandle()); }

static inline void GeoFenceSettingsRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(GeoFenceSettingsHandle(), instId); }

static inline void GeoFenceSettingsUpdated() { UAVObjUpdated(GeoFenceSettingsHandle()); }

static inline void GeoFenceSettingsInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(GeoFenceSettingsHandle(), instId); }

static inline int32_t GeoFenceSettingsGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(GeoFenceSettingsHandle(), dataOut); }

static inline int32_t GeoFenceSettingsSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(GeoFenceSettingsHandle(), dataIn); }

static inline int8_t GeoFenceSettingsReadOnly() { return UAVObjReadOnly(GeoFenceSettingsHandle()); }

static inline uint16_t GeoFenceSettingsGetNumInstances(){ return UAVObjGetNumInstances(GeoFenceSettingsHandle()); }

static inline uint32_t GeoFenceSettingsGetNumBytes(){ return UAVObjGetNumBytes(GeoFenceSettingsHandle()); }

// Field information
// Field WarningRadius information
// Field ErrorRadius information


// set/Get functions
extern void GeoFenceSettingsWarningRadiusSet( uint16_t *NewWarningRadius );
extern void GeoFenceSettingsWarningRadiusGet( uint16_t *NewWarningRadius );
extern void GeoFenceSettingsErrorRadiusSet( uint16_t *NewErrorRadius );
extern void GeoFenceSettingsErrorRadiusGet( uint16_t *NewErrorRadius );


#endif // GEOFENCESETTINGS_H

/**
 * @}
 * @}
 */