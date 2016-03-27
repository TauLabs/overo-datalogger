/**
 ******************************************************************************
 * @addtogroup TauLabsCore Tau Labs Core components
 * @{
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 *
 * @file       firmwareiapobj.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Implementation of the FirmwareIAPObj object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: firmwareiapobj.xml. 
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

#include <string.h>
#include "uavobjectmanager.h"
#include "firmwareiapobj.h"

// Private variables
static UAVObjHandle handle = NULL;

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t FirmwareIAPObjInitialize(void)
{
	// Don't set the handle to null if already registered
	if(UAVObjGetByID(FIRMWAREIAPOBJ_OBJID) != NULL)
		return -2;
	
	// Register object with the object manager
	handle = UAVObjRegister(FIRMWAREIAPOBJ_OBJID,
			FIRMWAREIAPOBJ_ISSINGLEINST, FIRMWAREIAPOBJ_ISSETTINGS, FIRMWAREIAPOBJ_NUMBYTES, &FirmwareIAPObjSetDefaults);

	// Done
	if (handle != 0)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

/**
 * Initialize object fields and metadata with the default values.
 * If a default value is not specified the object fields
 * will be initialized to zero.
 */
void FirmwareIAPObjSetDefaults(UAVObjHandle obj, uint16_t instId)
{
	FirmwareIAPObjData data;

	// Initialize object fields to their default values
	UAVObjGetInstanceData(obj, instId, &data);
	memset(&data, 0, sizeof(FirmwareIAPObjData));

	UAVObjSetInstanceData(obj, instId, &data);

	// Initialize object metadata to their default values
	if (instId == 0) {
		UAVObjMetadata metadata;
		metadata.flags =
			ACCESS_READWRITE << UAVOBJ_ACCESS_SHIFT |
			ACCESS_READWRITE << UAVOBJ_GCS_ACCESS_SHIFT |
			1 << UAVOBJ_TELEMETRY_ACKED_SHIFT |
			1 << UAVOBJ_GCS_TELEMETRY_ACKED_SHIFT |
			UPDATEMODE_ONCHANGE << UAVOBJ_TELEMETRY_UPDATE_MODE_SHIFT |
			UPDATEMODE_MANUAL << UAVOBJ_GCS_TELEMETRY_UPDATE_MODE_SHIFT;
		metadata.telemetryUpdatePeriod = 0;
		metadata.gcsTelemetryUpdatePeriod = 0;
		metadata.loggingUpdatePeriod = 0;
		UAVObjSetMetadata(obj, &metadata);
	}
}

/**
 * Get object handle
 */
UAVObjHandle FirmwareIAPObjHandle()
{
	return handle;
}

/**
 * Get/Set object Functions
 */
void FirmwareIAPObjcrcSet( uint32_t *Newcrc )
{
	UAVObjSetDataField(FirmwareIAPObjHandle(), (void*)Newcrc, offsetof( FirmwareIAPObjData, crc), sizeof(uint32_t));
}
void FirmwareIAPObjcrcGet( uint32_t *Newcrc )
{
	UAVObjGetDataField(FirmwareIAPObjHandle(), (void*)Newcrc, offsetof( FirmwareIAPObjData, crc), sizeof(uint32_t));
}
void FirmwareIAPObjCommandSet( uint16_t *NewCommand )
{
	UAVObjSetDataField(FirmwareIAPObjHandle(), (void*)NewCommand, offsetof( FirmwareIAPObjData, Command), sizeof(uint16_t));
}
void FirmwareIAPObjCommandGet( uint16_t *NewCommand )
{
	UAVObjGetDataField(FirmwareIAPObjHandle(), (void*)NewCommand, offsetof( FirmwareIAPObjData, Command), sizeof(uint16_t));
}
void FirmwareIAPObjBoardRevisionSet( uint16_t *NewBoardRevision )
{
	UAVObjSetDataField(FirmwareIAPObjHandle(), (void*)NewBoardRevision, offsetof( FirmwareIAPObjData, BoardRevision), sizeof(uint16_t));
}
void FirmwareIAPObjBoardRevisionGet( uint16_t *NewBoardRevision )
{
	UAVObjGetDataField(FirmwareIAPObjHandle(), (void*)NewBoardRevision, offsetof( FirmwareIAPObjData, BoardRevision), sizeof(uint16_t));
}
void FirmwareIAPObjDescriptionSet( uint8_t *NewDescription )
{
	UAVObjSetDataField(FirmwareIAPObjHandle(), (void*)NewDescription, offsetof( FirmwareIAPObjData, Description), 100*sizeof(uint8_t));
}
void FirmwareIAPObjDescriptionGet( uint8_t *NewDescription )
{
	UAVObjGetDataField(FirmwareIAPObjHandle(), (void*)NewDescription, offsetof( FirmwareIAPObjData, Description), 100*sizeof(uint8_t));
}
void FirmwareIAPObjCPUSerialSet( uint8_t *NewCPUSerial )
{
	UAVObjSetDataField(FirmwareIAPObjHandle(), (void*)NewCPUSerial, offsetof( FirmwareIAPObjData, CPUSerial), 12*sizeof(uint8_t));
}
void FirmwareIAPObjCPUSerialGet( uint8_t *NewCPUSerial )
{
	UAVObjGetDataField(FirmwareIAPObjHandle(), (void*)NewCPUSerial, offsetof( FirmwareIAPObjData, CPUSerial), 12*sizeof(uint8_t));
}
void FirmwareIAPObjBoardTypeSet( uint8_t *NewBoardType )
{
	UAVObjSetDataField(FirmwareIAPObjHandle(), (void*)NewBoardType, offsetof( FirmwareIAPObjData, BoardType), sizeof(uint8_t));
}
void FirmwareIAPObjBoardTypeGet( uint8_t *NewBoardType )
{
	UAVObjGetDataField(FirmwareIAPObjHandle(), (void*)NewBoardType, offsetof( FirmwareIAPObjData, BoardType), sizeof(uint8_t));
}
void FirmwareIAPObjArmResetSet( uint8_t *NewArmReset )
{
	UAVObjSetDataField(FirmwareIAPObjHandle(), (void*)NewArmReset, offsetof( FirmwareIAPObjData, ArmReset), sizeof(uint8_t));
}
void FirmwareIAPObjArmResetGet( uint8_t *NewArmReset )
{
	UAVObjGetDataField(FirmwareIAPObjHandle(), (void*)NewArmReset, offsetof( FirmwareIAPObjData, ArmReset), sizeof(uint8_t));
}


/**
 * @}
 * @}
 */

