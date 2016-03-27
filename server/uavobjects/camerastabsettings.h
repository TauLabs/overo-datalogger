/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup CameraStabSettings CameraStabSettings
 * @brief Settings for the @ref CameraStab mmodule
 *
 *
 * @file       camerastabsettings.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the CameraStabSettings object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: camerastabsettings.xml. 
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

#ifndef CAMERASTABSETTINGS_H
#define CAMERASTABSETTINGS_H

#include "pios_queue.h"
#include "uavoversion.h"



// Object constants
#define CAMERASTABSETTINGS_OBJID 0xFA09A51A
#define CAMERASTABSETTINGS_ISSINGLEINST 1
#define CAMERASTABSETTINGS_ISSETTINGS 1
#define CAMERASTABSETTINGS_NUMBYTES 29

// Generic interface functions
int32_t CameraStabSettingsInitialize();
UAVObjHandle CameraStabSettingsHandle();
void CameraStabSettingsSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    float MaxAxisLockRate;
    float MaxAccel;
    uint8_t Input[3];
    uint8_t InputRange[3];
    uint8_t InputRate[3];
    uint8_t OutputRange[3];
    uint8_t FeedForward[3];
    uint8_t StabilizationMode[3];
    uint8_t AttitudeFilter;
    uint8_t InputFilter;
    uint8_t FeedForwardTime;

} __attribute__((packed)) __attribute__((aligned(4))) CameraStabSettingsData;

// Typesafe Object access functions
/**
 * @function CameraStabSettingsGet(dataOut)
 * @brief Populate a CameraStabSettingsData object
 * @param[out] dataOut 
 */
static inline int32_t CameraStabSettingsGet(CameraStabSettingsData *dataOut) { return UAVObjGetData(CameraStabSettingsHandle(), dataOut); }

static inline int32_t CameraStabSettingsSet(const CameraStabSettingsData *dataIn) { return UAVObjSetData(CameraStabSettingsHandle(), dataIn); }

static inline int32_t CameraStabSettingsInstGet(uint16_t instId, CameraStabSettingsData *dataOut) { return UAVObjGetInstanceData(CameraStabSettingsHandle(), instId, dataOut); }

static inline int32_t CameraStabSettingsInstSet(uint16_t instId, const CameraStabSettingsData *dataIn) { return UAVObjSetInstanceData(CameraStabSettingsHandle(), instId, dataIn); }

static inline int32_t CameraStabSettingsConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(CameraStabSettingsHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t CameraStabSettingsConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(CameraStabSettingsHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t CameraStabSettingsCreateInstance() { return UAVObjCreateInstance(CameraStabSettingsHandle(), &CameraStabSettingsSetDefaults); }

static inline void CameraStabSettingsRequestUpdate() { UAVObjRequestUpdate(CameraStabSettingsHandle()); }

static inline void CameraStabSettingsRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(CameraStabSettingsHandle(), instId); }

static inline void CameraStabSettingsUpdated() { UAVObjUpdated(CameraStabSettingsHandle()); }

static inline void CameraStabSettingsInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(CameraStabSettingsHandle(), instId); }

static inline int32_t CameraStabSettingsGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(CameraStabSettingsHandle(), dataOut); }

static inline int32_t CameraStabSettingsSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(CameraStabSettingsHandle(), dataIn); }

static inline int8_t CameraStabSettingsReadOnly() { return UAVObjReadOnly(CameraStabSettingsHandle()); }

static inline uint16_t CameraStabSettingsGetNumInstances(){ return UAVObjGetNumInstances(CameraStabSettingsHandle()); }

static inline uint32_t CameraStabSettingsGetNumBytes(){ return UAVObjGetNumBytes(CameraStabSettingsHandle()); }

// Field information
// Field MaxAxisLockRate information
// Field MaxAccel information
// Field Input information
/* Enumeration options for field Input */
typedef enum { CAMERASTABSETTINGS_INPUT_ACCESSORY0=0, CAMERASTABSETTINGS_INPUT_ACCESSORY1=1, CAMERASTABSETTINGS_INPUT_ACCESSORY2=2, CAMERASTABSETTINGS_INPUT_ACCESSORY3=3, CAMERASTABSETTINGS_INPUT_ACCESSORY4=4, CAMERASTABSETTINGS_INPUT_ACCESSORY5=5, CAMERASTABSETTINGS_INPUT_POI=6, CAMERASTABSETTINGS_INPUT_NONE=7 }  __attribute__((packed)) CameraStabSettingsInputOptions;
/* Max value of any option in topmost parent Input of field Input */
#define CAMERASTABSETTINGS_INPUT_GLOBAL_MAXOPTVAL 7
/* Max value of any option in field Input */
#define CAMERASTABSETTINGS_INPUT_MAXOPTVAL 7
/* Ensure field Input contains valid data */
static inline bool CameraStabSettingsInputIsValid( uint8_t CurrentInput ) { return CurrentInput < CAMERASTABSETTINGS_INPUT_MAXOPTVAL; }
/* Array element names for field Input */
typedef enum { CAMERASTABSETTINGS_INPUT_ROLL=0, CAMERASTABSETTINGS_INPUT_PITCH=1, CAMERASTABSETTINGS_INPUT_YAW=2 } __attribute__((packed)) CameraStabSettingsInputElem;
/* Number of elements for field Input */
#define CAMERASTABSETTINGS_INPUT_NUMELEM 3
// Field InputRange information
/* Array element names for field InputRange */
typedef enum { CAMERASTABSETTINGS_INPUTRANGE_ROLL=0, CAMERASTABSETTINGS_INPUTRANGE_PITCH=1, CAMERASTABSETTINGS_INPUTRANGE_YAW=2 } __attribute__((packed)) CameraStabSettingsInputRangeElem;
/* Number of elements for field InputRange */
#define CAMERASTABSETTINGS_INPUTRANGE_NUMELEM 3
// Field InputRate information
/* Array element names for field InputRate */
typedef enum { CAMERASTABSETTINGS_INPUTRATE_ROLL=0, CAMERASTABSETTINGS_INPUTRATE_PITCH=1, CAMERASTABSETTINGS_INPUTRATE_YAW=2 } __attribute__((packed)) CameraStabSettingsInputRateElem;
/* Number of elements for field InputRate */
#define CAMERASTABSETTINGS_INPUTRATE_NUMELEM 3
// Field OutputRange information
/* Array element names for field OutputRange */
typedef enum { CAMERASTABSETTINGS_OUTPUTRANGE_ROLL=0, CAMERASTABSETTINGS_OUTPUTRANGE_PITCH=1, CAMERASTABSETTINGS_OUTPUTRANGE_YAW=2 } __attribute__((packed)) CameraStabSettingsOutputRangeElem;
/* Number of elements for field OutputRange */
#define CAMERASTABSETTINGS_OUTPUTRANGE_NUMELEM 3
// Field FeedForward information
/* Array element names for field FeedForward */
typedef enum { CAMERASTABSETTINGS_FEEDFORWARD_ROLL=0, CAMERASTABSETTINGS_FEEDFORWARD_PITCH=1, CAMERASTABSETTINGS_FEEDFORWARD_YAW=2 } __attribute__((packed)) CameraStabSettingsFeedForwardElem;
/* Number of elements for field FeedForward */
#define CAMERASTABSETTINGS_FEEDFORWARD_NUMELEM 3
// Field StabilizationMode information
/* Enumeration options for field StabilizationMode */
typedef enum { CAMERASTABSETTINGS_STABILIZATIONMODE_ATTITUDE=0, CAMERASTABSETTINGS_STABILIZATIONMODE_AXISLOCK=1 }  __attribute__((packed)) CameraStabSettingsStabilizationModeOptions;
/* Max value of any option in topmost parent StabilizationMode of field StabilizationMode */
#define CAMERASTABSETTINGS_STABILIZATIONMODE_GLOBAL_MAXOPTVAL 1
/* Max value of any option in field StabilizationMode */
#define CAMERASTABSETTINGS_STABILIZATIONMODE_MAXOPTVAL 1
/* Ensure field StabilizationMode contains valid data */
static inline bool CameraStabSettingsStabilizationModeIsValid( uint8_t CurrentStabilizationMode ) { return CurrentStabilizationMode < CAMERASTABSETTINGS_STABILIZATIONMODE_MAXOPTVAL; }
/* Array element names for field StabilizationMode */
typedef enum { CAMERASTABSETTINGS_STABILIZATIONMODE_ROLL=0, CAMERASTABSETTINGS_STABILIZATIONMODE_PITCH=1, CAMERASTABSETTINGS_STABILIZATIONMODE_YAW=2 } __attribute__((packed)) CameraStabSettingsStabilizationModeElem;
/* Number of elements for field StabilizationMode */
#define CAMERASTABSETTINGS_STABILIZATIONMODE_NUMELEM 3
// Field AttitudeFilter information
// Field InputFilter information
// Field FeedForwardTime information


// set/Get functions
extern void CameraStabSettingsMaxAxisLockRateSet( float *NewMaxAxisLockRate );
extern void CameraStabSettingsMaxAxisLockRateGet( float *NewMaxAxisLockRate );
extern void CameraStabSettingsMaxAccelSet( float *NewMaxAccel );
extern void CameraStabSettingsMaxAccelGet( float *NewMaxAccel );
extern void CameraStabSettingsInputSet( uint8_t *NewInput );
extern void CameraStabSettingsInputGet( uint8_t *NewInput );
extern void CameraStabSettingsInputRangeSet( uint8_t *NewInputRange );
extern void CameraStabSettingsInputRangeGet( uint8_t *NewInputRange );
extern void CameraStabSettingsInputRateSet( uint8_t *NewInputRate );
extern void CameraStabSettingsInputRateGet( uint8_t *NewInputRate );
extern void CameraStabSettingsOutputRangeSet( uint8_t *NewOutputRange );
extern void CameraStabSettingsOutputRangeGet( uint8_t *NewOutputRange );
extern void CameraStabSettingsFeedForwardSet( uint8_t *NewFeedForward );
extern void CameraStabSettingsFeedForwardGet( uint8_t *NewFeedForward );
extern void CameraStabSettingsStabilizationModeSet( uint8_t *NewStabilizationMode );
extern void CameraStabSettingsStabilizationModeGet( uint8_t *NewStabilizationMode );
extern void CameraStabSettingsAttitudeFilterSet( uint8_t *NewAttitudeFilter );
extern void CameraStabSettingsAttitudeFilterGet( uint8_t *NewAttitudeFilter );
extern void CameraStabSettingsInputFilterSet( uint8_t *NewInputFilter );
extern void CameraStabSettingsInputFilterGet( uint8_t *NewInputFilter );
extern void CameraStabSettingsFeedForwardTimeSet( uint8_t *NewFeedForwardTime );
extern void CameraStabSettingsFeedForwardTimeGet( uint8_t *NewFeedForwardTime );


#endif // CAMERASTABSETTINGS_H

/**
 * @}
 * @}
 */
