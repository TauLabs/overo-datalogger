/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup HomeLocation HomeLocation
 * @brief HomeLocation setting which contains the constants to tranlate from longitutde and latitude to NED reference frame.  Automatically set by @ref GPSModule after acquiring a 3D lock.  Used by @ref AHRSCommsModule.
 *
 *
 * @file       homelocation.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the HomeLocation object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: homelocation.xml. 
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

#ifndef HOMELOCATION_H
#define HOMELOCATION_H

#include "pios_queue.h"
#include "uavoversion.h"



// Object constants
#define HOMELOCATION_OBJID 0xCA32B032
#define HOMELOCATION_ISSINGLEINST 1
#define HOMELOCATION_ISSETTINGS 1
#define HOMELOCATION_NUMBYTES 29

// Generic interface functions
int32_t HomeLocationInitialize();
UAVObjHandle HomeLocationHandle();
void HomeLocationSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    int32_t Latitude;
    int32_t Longitude;
    float Altitude;
    float Be[3];
    int16_t GroundTemperature;
    uint16_t SeaLevelPressure;
    uint8_t Set;

} __attribute__((packed)) __attribute__((aligned(4))) HomeLocationData;

// Typesafe Object access functions
/**
 * @function HomeLocationGet(dataOut)
 * @brief Populate a HomeLocationData object
 * @param[out] dataOut 
 */
static inline int32_t HomeLocationGet(HomeLocationData *dataOut) { return UAVObjGetData(HomeLocationHandle(), dataOut); }

static inline int32_t HomeLocationSet(const HomeLocationData *dataIn) { return UAVObjSetData(HomeLocationHandle(), dataIn); }

static inline int32_t HomeLocationInstGet(uint16_t instId, HomeLocationData *dataOut) { return UAVObjGetInstanceData(HomeLocationHandle(), instId, dataOut); }

static inline int32_t HomeLocationInstSet(uint16_t instId, const HomeLocationData *dataIn) { return UAVObjSetInstanceData(HomeLocationHandle(), instId, dataIn); }

static inline int32_t HomeLocationConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(HomeLocationHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t HomeLocationConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(HomeLocationHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t HomeLocationCreateInstance() { return UAVObjCreateInstance(HomeLocationHandle(), &HomeLocationSetDefaults); }

static inline void HomeLocationRequestUpdate() { UAVObjRequestUpdate(HomeLocationHandle()); }

static inline void HomeLocationRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(HomeLocationHandle(), instId); }

static inline void HomeLocationUpdated() { UAVObjUpdated(HomeLocationHandle()); }

static inline void HomeLocationInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(HomeLocationHandle(), instId); }

static inline int32_t HomeLocationGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(HomeLocationHandle(), dataOut); }

static inline int32_t HomeLocationSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(HomeLocationHandle(), dataIn); }

static inline int8_t HomeLocationReadOnly() { return UAVObjReadOnly(HomeLocationHandle()); }

static inline uint16_t HomeLocationGetNumInstances(){ return UAVObjGetNumInstances(HomeLocationHandle()); }

static inline uint32_t HomeLocationGetNumBytes(){ return UAVObjGetNumBytes(HomeLocationHandle()); }

// Field information
// Field Latitude information
// Field Longitude information
// Field Altitude information
// Field Be information
/* Number of elements for field Be */
#define HOMELOCATION_BE_NUMELEM 3
// Field GroundTemperature information
// Field SeaLevelPressure information
// Field Set information
/* Enumeration options for field Set */
typedef enum { HOMELOCATION_SET_FALSE=0, HOMELOCATION_SET_TRUE=1 }  __attribute__((packed)) HomeLocationSetOptions;
/* Max value of any option in topmost parent Set of field Set */
#define HOMELOCATION_SET_GLOBAL_MAXOPTVAL 1
/* Max value of any option in field Set */
#define HOMELOCATION_SET_MAXOPTVAL 1
/* Ensure field Set contains valid data */
static inline bool HomeLocationSetIsValid( uint8_t CurrentSet ) { return CurrentSet < HOMELOCATION_SET_MAXOPTVAL; }


// set/Get functions
extern void HomeLocationLatitudeSet( int32_t *NewLatitude );
extern void HomeLocationLatitudeGet( int32_t *NewLatitude );
extern void HomeLocationLongitudeSet( int32_t *NewLongitude );
extern void HomeLocationLongitudeGet( int32_t *NewLongitude );
extern void HomeLocationAltitudeSet( float *NewAltitude );
extern void HomeLocationAltitudeGet( float *NewAltitude );
extern void HomeLocationBeSet( float *NewBe );
extern void HomeLocationBeGet( float *NewBe );
extern void HomeLocationGroundTemperatureSet( int16_t *NewGroundTemperature );
extern void HomeLocationGroundTemperatureGet( int16_t *NewGroundTemperature );
extern void HomeLocationSeaLevelPressureSet( uint16_t *NewSeaLevelPressure );
extern void HomeLocationSeaLevelPressureGet( uint16_t *NewSeaLevelPressure );
extern void HomeLocationSetSet( uint8_t *NewSet );
extern void HomeLocationSetGet( uint8_t *NewSet );


#endif // HOMELOCATION_H

/**
 * @}
 * @}
 */
