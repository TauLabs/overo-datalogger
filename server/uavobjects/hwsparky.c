/**
 ******************************************************************************
 * @addtogroup TauLabsCore Tau Labs Core components
 * @{
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 *
 * @file       hwsparky.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Implementation of the HwSparky object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: hwsparky.xml. 
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
#include "hwsparky.h"

// Private variables
static UAVObjHandle handle = NULL;

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t HwSparkyInitialize(void)
{
	// Don't set the handle to null if already registered
	if(UAVObjGetByID(HWSPARKY_OBJID) != NULL)
		return -2;
	
	// Register object with the object manager
	handle = UAVObjRegister(HWSPARKY_OBJID,
			HWSPARKY_ISSINGLEINST, HWSPARKY_ISSETTINGS, HWSPARKY_NUMBYTES, &HwSparkySetDefaults);

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
void HwSparkySetDefaults(UAVObjHandle obj, uint16_t instId)
{
	HwSparkyData data;

	// Initialize object fields to their default values
	UAVObjGetInstanceData(obj, instId, &data);
	memset(&data, 0, sizeof(HwSparkyData));
	data.RcvrPort = 0;
	data.FlexiPort = 0;
	data.MainPort = 0;
	data.OutPort = 0;
	data.USB_HIDPort = 0;
	data.USB_VCPPort = 4;
	data.DSMxMode = 0;
	data.GyroRange = 1;
	data.AccelRange = 2;
	data.MPU9150DLPF = 1;
	data.MPU9150Rate = 2;
	data.Magnetometer = 0;
	data.ExtMagOrientation = 0;

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
			UPDATEMODE_ONCHANGE << UAVOBJ_GCS_TELEMETRY_UPDATE_MODE_SHIFT;
		metadata.telemetryUpdatePeriod = 0;
		metadata.gcsTelemetryUpdatePeriod = 0;
		metadata.loggingUpdatePeriod = 0;
		UAVObjSetMetadata(obj, &metadata);
	}
}

/**
 * Get object handle
 */
UAVObjHandle HwSparkyHandle()
{
	return handle;
}

/**
 * Get/Set object Functions
 */
void HwSparkyRcvrPortSet( uint8_t *NewRcvrPort )
{
	UAVObjSetDataField(HwSparkyHandle(), (void*)NewRcvrPort, offsetof( HwSparkyData, RcvrPort), sizeof(uint8_t));
}
void HwSparkyRcvrPortGet( uint8_t *NewRcvrPort )
{
	UAVObjGetDataField(HwSparkyHandle(), (void*)NewRcvrPort, offsetof( HwSparkyData, RcvrPort), sizeof(uint8_t));
}
void HwSparkyFlexiPortSet( uint8_t *NewFlexiPort )
{
	UAVObjSetDataField(HwSparkyHandle(), (void*)NewFlexiPort, offsetof( HwSparkyData, FlexiPort), sizeof(uint8_t));
}
void HwSparkyFlexiPortGet( uint8_t *NewFlexiPort )
{
	UAVObjGetDataField(HwSparkyHandle(), (void*)NewFlexiPort, offsetof( HwSparkyData, FlexiPort), sizeof(uint8_t));
}
void HwSparkyMainPortSet( uint8_t *NewMainPort )
{
	UAVObjSetDataField(HwSparkyHandle(), (void*)NewMainPort, offsetof( HwSparkyData, MainPort), sizeof(uint8_t));
}
void HwSparkyMainPortGet( uint8_t *NewMainPort )
{
	UAVObjGetDataField(HwSparkyHandle(), (void*)NewMainPort, offsetof( HwSparkyData, MainPort), sizeof(uint8_t));
}
void HwSparkyOutPortSet( uint8_t *NewOutPort )
{
	UAVObjSetDataField(HwSparkyHandle(), (void*)NewOutPort, offsetof( HwSparkyData, OutPort), sizeof(uint8_t));
}
void HwSparkyOutPortGet( uint8_t *NewOutPort )
{
	UAVObjGetDataField(HwSparkyHandle(), (void*)NewOutPort, offsetof( HwSparkyData, OutPort), sizeof(uint8_t));
}
void HwSparkyUSB_HIDPortSet( uint8_t *NewUSB_HIDPort )
{
	UAVObjSetDataField(HwSparkyHandle(), (void*)NewUSB_HIDPort, offsetof( HwSparkyData, USB_HIDPort), sizeof(uint8_t));
}
void HwSparkyUSB_HIDPortGet( uint8_t *NewUSB_HIDPort )
{
	UAVObjGetDataField(HwSparkyHandle(), (void*)NewUSB_HIDPort, offsetof( HwSparkyData, USB_HIDPort), sizeof(uint8_t));
}
void HwSparkyUSB_VCPPortSet( uint8_t *NewUSB_VCPPort )
{
	UAVObjSetDataField(HwSparkyHandle(), (void*)NewUSB_VCPPort, offsetof( HwSparkyData, USB_VCPPort), sizeof(uint8_t));
}
void HwSparkyUSB_VCPPortGet( uint8_t *NewUSB_VCPPort )
{
	UAVObjGetDataField(HwSparkyHandle(), (void*)NewUSB_VCPPort, offsetof( HwSparkyData, USB_VCPPort), sizeof(uint8_t));
}
void HwSparkyDSMxModeSet( uint8_t *NewDSMxMode )
{
	UAVObjSetDataField(HwSparkyHandle(), (void*)NewDSMxMode, offsetof( HwSparkyData, DSMxMode), sizeof(uint8_t));
}
void HwSparkyDSMxModeGet( uint8_t *NewDSMxMode )
{
	UAVObjGetDataField(HwSparkyHandle(), (void*)NewDSMxMode, offsetof( HwSparkyData, DSMxMode), sizeof(uint8_t));
}
void HwSparkyGyroRangeSet( uint8_t *NewGyroRange )
{
	UAVObjSetDataField(HwSparkyHandle(), (void*)NewGyroRange, offsetof( HwSparkyData, GyroRange), sizeof(uint8_t));
}
void HwSparkyGyroRangeGet( uint8_t *NewGyroRange )
{
	UAVObjGetDataField(HwSparkyHandle(), (void*)NewGyroRange, offsetof( HwSparkyData, GyroRange), sizeof(uint8_t));
}
void HwSparkyAccelRangeSet( uint8_t *NewAccelRange )
{
	UAVObjSetDataField(HwSparkyHandle(), (void*)NewAccelRange, offsetof( HwSparkyData, AccelRange), sizeof(uint8_t));
}
void HwSparkyAccelRangeGet( uint8_t *NewAccelRange )
{
	UAVObjGetDataField(HwSparkyHandle(), (void*)NewAccelRange, offsetof( HwSparkyData, AccelRange), sizeof(uint8_t));
}
void HwSparkyMPU9150DLPFSet( uint8_t *NewMPU9150DLPF )
{
	UAVObjSetDataField(HwSparkyHandle(), (void*)NewMPU9150DLPF, offsetof( HwSparkyData, MPU9150DLPF), sizeof(uint8_t));
}
void HwSparkyMPU9150DLPFGet( uint8_t *NewMPU9150DLPF )
{
	UAVObjGetDataField(HwSparkyHandle(), (void*)NewMPU9150DLPF, offsetof( HwSparkyData, MPU9150DLPF), sizeof(uint8_t));
}
void HwSparkyMPU9150RateSet( uint8_t *NewMPU9150Rate )
{
	UAVObjSetDataField(HwSparkyHandle(), (void*)NewMPU9150Rate, offsetof( HwSparkyData, MPU9150Rate), sizeof(uint8_t));
}
void HwSparkyMPU9150RateGet( uint8_t *NewMPU9150Rate )
{
	UAVObjGetDataField(HwSparkyHandle(), (void*)NewMPU9150Rate, offsetof( HwSparkyData, MPU9150Rate), sizeof(uint8_t));
}
void HwSparkyMagnetometerSet( uint8_t *NewMagnetometer )
{
	UAVObjSetDataField(HwSparkyHandle(), (void*)NewMagnetometer, offsetof( HwSparkyData, Magnetometer), sizeof(uint8_t));
}
void HwSparkyMagnetometerGet( uint8_t *NewMagnetometer )
{
	UAVObjGetDataField(HwSparkyHandle(), (void*)NewMagnetometer, offsetof( HwSparkyData, Magnetometer), sizeof(uint8_t));
}
void HwSparkyExtMagOrientationSet( uint8_t *NewExtMagOrientation )
{
	UAVObjSetDataField(HwSparkyHandle(), (void*)NewExtMagOrientation, offsetof( HwSparkyData, ExtMagOrientation), sizeof(uint8_t));
}
void HwSparkyExtMagOrientationGet( uint8_t *NewExtMagOrientation )
{
	UAVObjGetDataField(HwSparkyHandle(), (void*)NewExtMagOrientation, offsetof( HwSparkyData, ExtMagOrientation), sizeof(uint8_t));
}


/**
 * @}
 * @}
 */

