/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup PathPlannerSettings PathPlannerSettings
 * @brief Settings for the @ref PathPlanner Module
 *
 *
 * @file       pathplannersettings.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the PathPlannerSettings object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: pathplannersettings.xml. 
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

#ifndef PATHPLANNERSETTINGS_H
#define PATHPLANNERSETTINGS_H

#include "pios_queue.h"
#include "uavoversion.h"



// Object constants
#define PATHPLANNERSETTINGS_OBJID 0x77A9E8F0
#define PATHPLANNERSETTINGS_ISSINGLEINST 1
#define PATHPLANNERSETTINGS_ISSETTINGS 1
#define PATHPLANNERSETTINGS_NUMBYTES 2

// Generic interface functions
int32_t PathPlannerSettingsInitialize();
UAVObjHandle PathPlannerSettingsHandle();
void PathPlannerSettingsSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    uint8_t PreprogrammedPath;
    uint8_t FlashOperation;

} __attribute__((packed)) __attribute__((aligned(4))) PathPlannerSettingsData;

// Typesafe Object access functions
/**
 * @function PathPlannerSettingsGet(dataOut)
 * @brief Populate a PathPlannerSettingsData object
 * @param[out] dataOut 
 */
static inline int32_t PathPlannerSettingsGet(PathPlannerSettingsData *dataOut) { return UAVObjGetData(PathPlannerSettingsHandle(), dataOut); }

static inline int32_t PathPlannerSettingsSet(const PathPlannerSettingsData *dataIn) { return UAVObjSetData(PathPlannerSettingsHandle(), dataIn); }

static inline int32_t PathPlannerSettingsInstGet(uint16_t instId, PathPlannerSettingsData *dataOut) { return UAVObjGetInstanceData(PathPlannerSettingsHandle(), instId, dataOut); }

static inline int32_t PathPlannerSettingsInstSet(uint16_t instId, const PathPlannerSettingsData *dataIn) { return UAVObjSetInstanceData(PathPlannerSettingsHandle(), instId, dataIn); }

static inline int32_t PathPlannerSettingsConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(PathPlannerSettingsHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t PathPlannerSettingsConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(PathPlannerSettingsHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t PathPlannerSettingsCreateInstance() { return UAVObjCreateInstance(PathPlannerSettingsHandle(), &PathPlannerSettingsSetDefaults); }

static inline void PathPlannerSettingsRequestUpdate() { UAVObjRequestUpdate(PathPlannerSettingsHandle()); }

static inline void PathPlannerSettingsRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(PathPlannerSettingsHandle(), instId); }

static inline void PathPlannerSettingsUpdated() { UAVObjUpdated(PathPlannerSettingsHandle()); }

static inline void PathPlannerSettingsInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(PathPlannerSettingsHandle(), instId); }

static inline int32_t PathPlannerSettingsGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(PathPlannerSettingsHandle(), dataOut); }

static inline int32_t PathPlannerSettingsSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(PathPlannerSettingsHandle(), dataIn); }

static inline int8_t PathPlannerSettingsReadOnly() { return UAVObjReadOnly(PathPlannerSettingsHandle()); }

static inline uint16_t PathPlannerSettingsGetNumInstances(){ return UAVObjGetNumInstances(PathPlannerSettingsHandle()); }

static inline uint32_t PathPlannerSettingsGetNumBytes(){ return UAVObjGetNumBytes(PathPlannerSettingsHandle()); }

// Field information
// Field PreprogrammedPath information
/* Enumeration options for field PreprogrammedPath */
typedef enum { PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_NONE=0, PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_10M_BOX=1, PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_LOGO=2 }  __attribute__((packed)) PathPlannerSettingsPreprogrammedPathOptions;
/* Max value of any option in topmost parent PreprogrammedPath of field PreprogrammedPath */
#define PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_GLOBAL_MAXOPTVAL 2
/* Max value of any option in field PreprogrammedPath */
#define PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_MAXOPTVAL 2
/* Ensure field PreprogrammedPath contains valid data */
static inline bool PathPlannerSettingsPreprogrammedPathIsValid( uint8_t CurrentPreprogrammedPath ) { return CurrentPreprogrammedPath < PATHPLANNERSETTINGS_PREPROGRAMMEDPATH_MAXOPTVAL; }
// Field FlashOperation information
/* Enumeration options for field FlashOperation */
typedef enum { PATHPLANNERSETTINGS_FLASHOPERATION_NONE=0, PATHPLANNERSETTINGS_FLASHOPERATION_FAILED=1, PATHPLANNERSETTINGS_FLASHOPERATION_COMPLETED=2, PATHPLANNERSETTINGS_FLASHOPERATION_SAVE1=3, PATHPLANNERSETTINGS_FLASHOPERATION_SAVE2=4, PATHPLANNERSETTINGS_FLASHOPERATION_SAVE3=5, PATHPLANNERSETTINGS_FLASHOPERATION_SAVE4=6, PATHPLANNERSETTINGS_FLASHOPERATION_SAVE5=7, PATHPLANNERSETTINGS_FLASHOPERATION_LOAD1=8, PATHPLANNERSETTINGS_FLASHOPERATION_LOAD2=9, PATHPLANNERSETTINGS_FLASHOPERATION_LOAD3=10, PATHPLANNERSETTINGS_FLASHOPERATION_LOAD4=11, PATHPLANNERSETTINGS_FLASHOPERATION_LOAD5=12 }  __attribute__((packed)) PathPlannerSettingsFlashOperationOptions;
/* Max value of any option in topmost parent FlashOperation of field FlashOperation */
#define PATHPLANNERSETTINGS_FLASHOPERATION_GLOBAL_MAXOPTVAL 12
/* Max value of any option in field FlashOperation */
#define PATHPLANNERSETTINGS_FLASHOPERATION_MAXOPTVAL 12
/* Ensure field FlashOperation contains valid data */
static inline bool PathPlannerSettingsFlashOperationIsValid( uint8_t CurrentFlashOperation ) { return CurrentFlashOperation < PATHPLANNERSETTINGS_FLASHOPERATION_MAXOPTVAL; }


// set/Get functions
extern void PathPlannerSettingsPreprogrammedPathSet( uint8_t *NewPreprogrammedPath );
extern void PathPlannerSettingsPreprogrammedPathGet( uint8_t *NewPreprogrammedPath );
extern void PathPlannerSettingsFlashOperationSet( uint8_t *NewFlashOperation );
extern void PathPlannerSettingsFlashOperationGet( uint8_t *NewFlashOperation );


#endif // PATHPLANNERSETTINGS_H

/**
 * @}
 * @}
 */
