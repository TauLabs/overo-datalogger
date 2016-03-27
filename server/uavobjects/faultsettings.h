/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup FaultSettings FaultSettings
 * @brief Allows testers to simulate various fault scenarios.
 *
 *
 * @file       faultsettings.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the FaultSettings object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: faultsettings.xml. 
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

#ifndef FAULTSETTINGS_H
#define FAULTSETTINGS_H

#include "pios_queue.h"
#include "uavoversion.h"



// Object constants
#define FAULTSETTINGS_OBJID 0x2778BA3C
#define FAULTSETTINGS_ISSINGLEINST 1
#define FAULTSETTINGS_ISSETTINGS 1
#define FAULTSETTINGS_NUMBYTES 1

// Generic interface functions
int32_t FaultSettingsInitialize();
UAVObjHandle FaultSettingsHandle();
void FaultSettingsSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    uint8_t ActivateFault;

} __attribute__((packed)) __attribute__((aligned(4))) FaultSettingsData;

// Typesafe Object access functions
/**
 * @function FaultSettingsGet(dataOut)
 * @brief Populate a FaultSettingsData object
 * @param[out] dataOut 
 */
static inline int32_t FaultSettingsGet(FaultSettingsData *dataOut) { return UAVObjGetData(FaultSettingsHandle(), dataOut); }

static inline int32_t FaultSettingsSet(const FaultSettingsData *dataIn) { return UAVObjSetData(FaultSettingsHandle(), dataIn); }

static inline int32_t FaultSettingsInstGet(uint16_t instId, FaultSettingsData *dataOut) { return UAVObjGetInstanceData(FaultSettingsHandle(), instId, dataOut); }

static inline int32_t FaultSettingsInstSet(uint16_t instId, const FaultSettingsData *dataIn) { return UAVObjSetInstanceData(FaultSettingsHandle(), instId, dataIn); }

static inline int32_t FaultSettingsConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(FaultSettingsHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t FaultSettingsConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(FaultSettingsHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t FaultSettingsCreateInstance() { return UAVObjCreateInstance(FaultSettingsHandle(), &FaultSettingsSetDefaults); }

static inline void FaultSettingsRequestUpdate() { UAVObjRequestUpdate(FaultSettingsHandle()); }

static inline void FaultSettingsRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(FaultSettingsHandle(), instId); }

static inline void FaultSettingsUpdated() { UAVObjUpdated(FaultSettingsHandle()); }

static inline void FaultSettingsInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(FaultSettingsHandle(), instId); }

static inline int32_t FaultSettingsGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(FaultSettingsHandle(), dataOut); }

static inline int32_t FaultSettingsSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(FaultSettingsHandle(), dataIn); }

static inline int8_t FaultSettingsReadOnly() { return UAVObjReadOnly(FaultSettingsHandle()); }

static inline uint16_t FaultSettingsGetNumInstances(){ return UAVObjGetNumInstances(FaultSettingsHandle()); }

static inline uint32_t FaultSettingsGetNumBytes(){ return UAVObjGetNumBytes(FaultSettingsHandle()); }

// Field information
// Field ActivateFault information
/* Enumeration options for field ActivateFault */
typedef enum { FAULTSETTINGS_ACTIVATEFAULT_NOFAULT=0, FAULTSETTINGS_ACTIVATEFAULT_MODULEINITASSERT=1, FAULTSETTINGS_ACTIVATEFAULT_INITOUTOFMEMORY=2, FAULTSETTINGS_ACTIVATEFAULT_INITBUSERROR=3, FAULTSETTINGS_ACTIVATEFAULT_RUNAWAYTASK=4, FAULTSETTINGS_ACTIVATEFAULT_TASKOUTOFMEMORY=5 }  __attribute__((packed)) FaultSettingsActivateFaultOptions;
/* Max value of any option in topmost parent ActivateFault of field ActivateFault */
#define FAULTSETTINGS_ACTIVATEFAULT_GLOBAL_MAXOPTVAL 5
/* Max value of any option in field ActivateFault */
#define FAULTSETTINGS_ACTIVATEFAULT_MAXOPTVAL 5
/* Ensure field ActivateFault contains valid data */
static inline bool FaultSettingsActivateFaultIsValid( uint8_t CurrentActivateFault ) { return CurrentActivateFault < FAULTSETTINGS_ACTIVATEFAULT_MAXOPTVAL; }


// set/Get functions
extern void FaultSettingsActivateFaultSet( uint8_t *NewActivateFault );
extern void FaultSettingsActivateFaultGet( uint8_t *NewActivateFault );


#endif // FAULTSETTINGS_H

/**
 * @}
 * @}
 */
