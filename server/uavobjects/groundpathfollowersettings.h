/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup GroundPathFollowerSettings GroundPathFollowerSettings
 * @brief Settings for the @ref GroundPathFollowerModule
 *
 *
 * @file       groundpathfollowersettings.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the GroundPathFollowerSettings object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: groundpathfollowersettings.xml. 
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

#ifndef GROUNDPATHFOLLOWERSETTINGS_H
#define GROUNDPATHFOLLOWERSETTINGS_H

#include "pios_queue.h"
#include "uavoversion.h"



// Object constants
#define GROUNDPATHFOLLOWERSETTINGS_OBJID 0x9090C16
#define GROUNDPATHFOLLOWERSETTINGS_ISSINGLEINST 1
#define GROUNDPATHFOLLOWERSETTINGS_ISSETTINGS 1
#define GROUNDPATHFOLLOWERSETTINGS_NUMBYTES 47

// Generic interface functions
int32_t GroundPathFollowerSettingsInitialize();
UAVObjHandle GroundPathFollowerSettingsHandle();
void GroundPathFollowerSettingsSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    float HorizontalPosPI[3];
    float HorizontalVelPID[4];
    float VelocityFeedforward;
    float MaxThrottle;
    int32_t UpdatePeriod;
    uint16_t HorizontalVelMax;
    uint8_t ManualOverride;
    uint8_t ThrottleControl;
    uint8_t VelocitySource;
    uint8_t PositionSource;
    uint8_t EndpointRadius;

} __attribute__((packed)) __attribute__((aligned(4))) GroundPathFollowerSettingsData;

// Typesafe Object access functions
/**
 * @function GroundPathFollowerSettingsGet(dataOut)
 * @brief Populate a GroundPathFollowerSettingsData object
 * @param[out] dataOut 
 */
static inline int32_t GroundPathFollowerSettingsGet(GroundPathFollowerSettingsData *dataOut) { return UAVObjGetData(GroundPathFollowerSettingsHandle(), dataOut); }

static inline int32_t GroundPathFollowerSettingsSet(const GroundPathFollowerSettingsData *dataIn) { return UAVObjSetData(GroundPathFollowerSettingsHandle(), dataIn); }

static inline int32_t GroundPathFollowerSettingsInstGet(uint16_t instId, GroundPathFollowerSettingsData *dataOut) { return UAVObjGetInstanceData(GroundPathFollowerSettingsHandle(), instId, dataOut); }

static inline int32_t GroundPathFollowerSettingsInstSet(uint16_t instId, const GroundPathFollowerSettingsData *dataIn) { return UAVObjSetInstanceData(GroundPathFollowerSettingsHandle(), instId, dataIn); }

static inline int32_t GroundPathFollowerSettingsConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(GroundPathFollowerSettingsHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t GroundPathFollowerSettingsConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(GroundPathFollowerSettingsHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t GroundPathFollowerSettingsCreateInstance() { return UAVObjCreateInstance(GroundPathFollowerSettingsHandle(), &GroundPathFollowerSettingsSetDefaults); }

static inline void GroundPathFollowerSettingsRequestUpdate() { UAVObjRequestUpdate(GroundPathFollowerSettingsHandle()); }

static inline void GroundPathFollowerSettingsRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(GroundPathFollowerSettingsHandle(), instId); }

static inline void GroundPathFollowerSettingsUpdated() { UAVObjUpdated(GroundPathFollowerSettingsHandle()); }

static inline void GroundPathFollowerSettingsInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(GroundPathFollowerSettingsHandle(), instId); }

static inline int32_t GroundPathFollowerSettingsGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(GroundPathFollowerSettingsHandle(), dataOut); }

static inline int32_t GroundPathFollowerSettingsSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(GroundPathFollowerSettingsHandle(), dataIn); }

static inline int8_t GroundPathFollowerSettingsReadOnly() { return UAVObjReadOnly(GroundPathFollowerSettingsHandle()); }

static inline uint16_t GroundPathFollowerSettingsGetNumInstances(){ return UAVObjGetNumInstances(GroundPathFollowerSettingsHandle()); }

static inline uint32_t GroundPathFollowerSettingsGetNumBytes(){ return UAVObjGetNumBytes(GroundPathFollowerSettingsHandle()); }

// Field information
// Field HorizontalPosPI information
/* Array element names for field HorizontalPosPI */
typedef enum { GROUNDPATHFOLLOWERSETTINGS_HORIZONTALPOSPI_KP=0, GROUNDPATHFOLLOWERSETTINGS_HORIZONTALPOSPI_KI=1, GROUNDPATHFOLLOWERSETTINGS_HORIZONTALPOSPI_ILIMIT=2 } __attribute__((packed)) GroundPathFollowerSettingsHorizontalPosPIElem;
/* Number of elements for field HorizontalPosPI */
#define GROUNDPATHFOLLOWERSETTINGS_HORIZONTALPOSPI_NUMELEM 3
// Field HorizontalVelPID information
/* Array element names for field HorizontalVelPID */
typedef enum { GROUNDPATHFOLLOWERSETTINGS_HORIZONTALVELPID_KP=0, GROUNDPATHFOLLOWERSETTINGS_HORIZONTALVELPID_KI=1, GROUNDPATHFOLLOWERSETTINGS_HORIZONTALVELPID_KD=2, GROUNDPATHFOLLOWERSETTINGS_HORIZONTALVELPID_ILIMIT=3 } __attribute__((packed)) GroundPathFollowerSettingsHorizontalVelPIDElem;
/* Number of elements for field HorizontalVelPID */
#define GROUNDPATHFOLLOWERSETTINGS_HORIZONTALVELPID_NUMELEM 4
// Field VelocityFeedforward information
// Field MaxThrottle information
// Field UpdatePeriod information
// Field HorizontalVelMax information
// Field ManualOverride information
/* Enumeration options for field ManualOverride */
typedef enum { GROUNDPATHFOLLOWERSETTINGS_MANUALOVERRIDE_FALSE=0, GROUNDPATHFOLLOWERSETTINGS_MANUALOVERRIDE_TRUE=1 }  __attribute__((packed)) GroundPathFollowerSettingsManualOverrideOptions;
/* Max value of any option in topmost parent ManualOverride of field ManualOverride */
#define GROUNDPATHFOLLOWERSETTINGS_MANUALOVERRIDE_GLOBAL_MAXOPTVAL 1
/* Max value of any option in field ManualOverride */
#define GROUNDPATHFOLLOWERSETTINGS_MANUALOVERRIDE_MAXOPTVAL 1
/* Ensure field ManualOverride contains valid data */
static inline bool GroundPathFollowerSettingsManualOverrideIsValid( uint8_t CurrentManualOverride ) { return CurrentManualOverride < GROUNDPATHFOLLOWERSETTINGS_MANUALOVERRIDE_MAXOPTVAL; }
// Field ThrottleControl information
/* Enumeration options for field ThrottleControl */
typedef enum { GROUNDPATHFOLLOWERSETTINGS_THROTTLECONTROL_MANUAL=0, GROUNDPATHFOLLOWERSETTINGS_THROTTLECONTROL_PROPORTIONAL=1, GROUNDPATHFOLLOWERSETTINGS_THROTTLECONTROL_AUTO=2 }  __attribute__((packed)) GroundPathFollowerSettingsThrottleControlOptions;
/* Max value of any option in topmost parent ThrottleControl of field ThrottleControl */
#define GROUNDPATHFOLLOWERSETTINGS_THROTTLECONTROL_GLOBAL_MAXOPTVAL 2
/* Max value of any option in field ThrottleControl */
#define GROUNDPATHFOLLOWERSETTINGS_THROTTLECONTROL_MAXOPTVAL 2
/* Ensure field ThrottleControl contains valid data */
static inline bool GroundPathFollowerSettingsThrottleControlIsValid( uint8_t CurrentThrottleControl ) { return CurrentThrottleControl < GROUNDPATHFOLLOWERSETTINGS_THROTTLECONTROL_MAXOPTVAL; }
// Field VelocitySource information
/* Enumeration options for field VelocitySource */
typedef enum { GROUNDPATHFOLLOWERSETTINGS_VELOCITYSOURCE_EKF=0, GROUNDPATHFOLLOWERSETTINGS_VELOCITYSOURCE_NEDVEL=1, GROUNDPATHFOLLOWERSETTINGS_VELOCITYSOURCE_GPSPOS=2 }  __attribute__((packed)) GroundPathFollowerSettingsVelocitySourceOptions;
/* Max value of any option in topmost parent VelocitySource of field VelocitySource */
#define GROUNDPATHFOLLOWERSETTINGS_VELOCITYSOURCE_GLOBAL_MAXOPTVAL 2
/* Max value of any option in field VelocitySource */
#define GROUNDPATHFOLLOWERSETTINGS_VELOCITYSOURCE_MAXOPTVAL 2
/* Ensure field VelocitySource contains valid data */
static inline bool GroundPathFollowerSettingsVelocitySourceIsValid( uint8_t CurrentVelocitySource ) { return CurrentVelocitySource < GROUNDPATHFOLLOWERSETTINGS_VELOCITYSOURCE_MAXOPTVAL; }
// Field PositionSource information
/* Enumeration options for field PositionSource */
typedef enum { GROUNDPATHFOLLOWERSETTINGS_POSITIONSOURCE_EKF=0, GROUNDPATHFOLLOWERSETTINGS_POSITIONSOURCE_GPSPOS=1 }  __attribute__((packed)) GroundPathFollowerSettingsPositionSourceOptions;
/* Max value of any option in topmost parent PositionSource of field PositionSource */
#define GROUNDPATHFOLLOWERSETTINGS_POSITIONSOURCE_GLOBAL_MAXOPTVAL 1
/* Max value of any option in field PositionSource */
#define GROUNDPATHFOLLOWERSETTINGS_POSITIONSOURCE_MAXOPTVAL 1
/* Ensure field PositionSource contains valid data */
static inline bool GroundPathFollowerSettingsPositionSourceIsValid( uint8_t CurrentPositionSource ) { return CurrentPositionSource < GROUNDPATHFOLLOWERSETTINGS_POSITIONSOURCE_MAXOPTVAL; }
// Field EndpointRadius information


// set/Get functions
extern void GroundPathFollowerSettingsHorizontalPosPISet( float *NewHorizontalPosPI );
extern void GroundPathFollowerSettingsHorizontalPosPIGet( float *NewHorizontalPosPI );
extern void GroundPathFollowerSettingsHorizontalVelPIDSet( float *NewHorizontalVelPID );
extern void GroundPathFollowerSettingsHorizontalVelPIDGet( float *NewHorizontalVelPID );
extern void GroundPathFollowerSettingsVelocityFeedforwardSet( float *NewVelocityFeedforward );
extern void GroundPathFollowerSettingsVelocityFeedforwardGet( float *NewVelocityFeedforward );
extern void GroundPathFollowerSettingsMaxThrottleSet( float *NewMaxThrottle );
extern void GroundPathFollowerSettingsMaxThrottleGet( float *NewMaxThrottle );
extern void GroundPathFollowerSettingsUpdatePeriodSet( int32_t *NewUpdatePeriod );
extern void GroundPathFollowerSettingsUpdatePeriodGet( int32_t *NewUpdatePeriod );
extern void GroundPathFollowerSettingsHorizontalVelMaxSet( uint16_t *NewHorizontalVelMax );
extern void GroundPathFollowerSettingsHorizontalVelMaxGet( uint16_t *NewHorizontalVelMax );
extern void GroundPathFollowerSettingsManualOverrideSet( uint8_t *NewManualOverride );
extern void GroundPathFollowerSettingsManualOverrideGet( uint8_t *NewManualOverride );
extern void GroundPathFollowerSettingsThrottleControlSet( uint8_t *NewThrottleControl );
extern void GroundPathFollowerSettingsThrottleControlGet( uint8_t *NewThrottleControl );
extern void GroundPathFollowerSettingsVelocitySourceSet( uint8_t *NewVelocitySource );
extern void GroundPathFollowerSettingsVelocitySourceGet( uint8_t *NewVelocitySource );
extern void GroundPathFollowerSettingsPositionSourceSet( uint8_t *NewPositionSource );
extern void GroundPathFollowerSettingsPositionSourceGet( uint8_t *NewPositionSource );
extern void GroundPathFollowerSettingsEndpointRadiusSet( uint8_t *NewEndpointRadius );
extern void GroundPathFollowerSettingsEndpointRadiusGet( uint8_t *NewEndpointRadius );


#endif // GROUNDPATHFOLLOWERSETTINGS_H

/**
 * @}
 * @}
 */