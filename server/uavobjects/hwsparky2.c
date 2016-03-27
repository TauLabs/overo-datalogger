/**
 ******************************************************************************
 * @addtogroup TauLabsCore Tau Labs Core components
 * @{
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 *
 * @file       hwsparky2.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Implementation of the HwSparky2 object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: hwsparky2.xml. 
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
#include "hwsparky2.h"

// Private variables
static UAVObjHandle handle = NULL;

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t HwSparky2Initialize(void)
{
	// Don't set the handle to null if already registered
	if(UAVObjGetByID(HWSPARKY2_OBJID) != NULL)
		return -2;
	
	// Register object with the object manager
	handle = UAVObjRegister(HWSPARKY2_OBJID,
			HWSPARKY2_ISSINGLEINST, HWSPARKY2_ISSETTINGS, HWSPARKY2_NUMBYTES, &HwSparky2SetDefaults);

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
void HwSparky2SetDefaults(UAVObjHandle obj, uint16_t instId)
{
	HwSparky2Data data;

	// Initialize object fields to their default values
	UAVObjGetInstanceData(obj, instId, &data);
	memset(&data, 0, sizeof(HwSparky2Data));
	data.CoordID = 0;
	data.RcvrPort = 0;
	data.MainPort = 0;
	data.FlexiPort = 0;
	data.USB_HIDPort = 0;
	data.USB_VCPPort = 4;
	data.DSMxMode = 0;
	data.Radio = 0;
	data.MaxRfSpeed = 3;
	data.MaxRfPower = 1;
	data.RfBand = 0;
	data.MinChannel = 0;
	data.MaxChannel = 250;
	data.GyroRange = 3;
	data.AccelRange = 2;
	data.MPU9250Rate = 3;
	data.MPU9250GyroLPF = 0;
	data.MPU9250AccelLPF = 1;
	data.VTX_Ch = 0;
	data.Magnetometer = 0;
	data.ExtMagOrientation = 0;
	data.AdcDac = 0;

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
UAVObjHandle HwSparky2Handle()
{
	return handle;
}

/**
 * Get/Set object Functions
 */
void HwSparky2CoordIDSet( uint32_t *NewCoordID )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewCoordID, offsetof( HwSparky2Data, CoordID), sizeof(uint32_t));
}
void HwSparky2CoordIDGet( uint32_t *NewCoordID )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewCoordID, offsetof( HwSparky2Data, CoordID), sizeof(uint32_t));
}
void HwSparky2RcvrPortSet( uint8_t *NewRcvrPort )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewRcvrPort, offsetof( HwSparky2Data, RcvrPort), sizeof(uint8_t));
}
void HwSparky2RcvrPortGet( uint8_t *NewRcvrPort )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewRcvrPort, offsetof( HwSparky2Data, RcvrPort), sizeof(uint8_t));
}
void HwSparky2MainPortSet( uint8_t *NewMainPort )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewMainPort, offsetof( HwSparky2Data, MainPort), sizeof(uint8_t));
}
void HwSparky2MainPortGet( uint8_t *NewMainPort )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewMainPort, offsetof( HwSparky2Data, MainPort), sizeof(uint8_t));
}
void HwSparky2FlexiPortSet( uint8_t *NewFlexiPort )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewFlexiPort, offsetof( HwSparky2Data, FlexiPort), sizeof(uint8_t));
}
void HwSparky2FlexiPortGet( uint8_t *NewFlexiPort )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewFlexiPort, offsetof( HwSparky2Data, FlexiPort), sizeof(uint8_t));
}
void HwSparky2USB_HIDPortSet( uint8_t *NewUSB_HIDPort )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewUSB_HIDPort, offsetof( HwSparky2Data, USB_HIDPort), sizeof(uint8_t));
}
void HwSparky2USB_HIDPortGet( uint8_t *NewUSB_HIDPort )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewUSB_HIDPort, offsetof( HwSparky2Data, USB_HIDPort), sizeof(uint8_t));
}
void HwSparky2USB_VCPPortSet( uint8_t *NewUSB_VCPPort )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewUSB_VCPPort, offsetof( HwSparky2Data, USB_VCPPort), sizeof(uint8_t));
}
void HwSparky2USB_VCPPortGet( uint8_t *NewUSB_VCPPort )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewUSB_VCPPort, offsetof( HwSparky2Data, USB_VCPPort), sizeof(uint8_t));
}
void HwSparky2DSMxModeSet( uint8_t *NewDSMxMode )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewDSMxMode, offsetof( HwSparky2Data, DSMxMode), sizeof(uint8_t));
}
void HwSparky2DSMxModeGet( uint8_t *NewDSMxMode )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewDSMxMode, offsetof( HwSparky2Data, DSMxMode), sizeof(uint8_t));
}
void HwSparky2RadioSet( uint8_t *NewRadio )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewRadio, offsetof( HwSparky2Data, Radio), sizeof(uint8_t));
}
void HwSparky2RadioGet( uint8_t *NewRadio )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewRadio, offsetof( HwSparky2Data, Radio), sizeof(uint8_t));
}
void HwSparky2MaxRfSpeedSet( uint8_t *NewMaxRfSpeed )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewMaxRfSpeed, offsetof( HwSparky2Data, MaxRfSpeed), sizeof(uint8_t));
}
void HwSparky2MaxRfSpeedGet( uint8_t *NewMaxRfSpeed )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewMaxRfSpeed, offsetof( HwSparky2Data, MaxRfSpeed), sizeof(uint8_t));
}
void HwSparky2MaxRfPowerSet( uint8_t *NewMaxRfPower )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewMaxRfPower, offsetof( HwSparky2Data, MaxRfPower), sizeof(uint8_t));
}
void HwSparky2MaxRfPowerGet( uint8_t *NewMaxRfPower )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewMaxRfPower, offsetof( HwSparky2Data, MaxRfPower), sizeof(uint8_t));
}
void HwSparky2RfBandSet( uint8_t *NewRfBand )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewRfBand, offsetof( HwSparky2Data, RfBand), sizeof(uint8_t));
}
void HwSparky2RfBandGet( uint8_t *NewRfBand )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewRfBand, offsetof( HwSparky2Data, RfBand), sizeof(uint8_t));
}
void HwSparky2MinChannelSet( uint8_t *NewMinChannel )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewMinChannel, offsetof( HwSparky2Data, MinChannel), sizeof(uint8_t));
}
void HwSparky2MinChannelGet( uint8_t *NewMinChannel )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewMinChannel, offsetof( HwSparky2Data, MinChannel), sizeof(uint8_t));
}
void HwSparky2MaxChannelSet( uint8_t *NewMaxChannel )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewMaxChannel, offsetof( HwSparky2Data, MaxChannel), sizeof(uint8_t));
}
void HwSparky2MaxChannelGet( uint8_t *NewMaxChannel )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewMaxChannel, offsetof( HwSparky2Data, MaxChannel), sizeof(uint8_t));
}
void HwSparky2GyroRangeSet( uint8_t *NewGyroRange )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewGyroRange, offsetof( HwSparky2Data, GyroRange), sizeof(uint8_t));
}
void HwSparky2GyroRangeGet( uint8_t *NewGyroRange )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewGyroRange, offsetof( HwSparky2Data, GyroRange), sizeof(uint8_t));
}
void HwSparky2AccelRangeSet( uint8_t *NewAccelRange )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewAccelRange, offsetof( HwSparky2Data, AccelRange), sizeof(uint8_t));
}
void HwSparky2AccelRangeGet( uint8_t *NewAccelRange )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewAccelRange, offsetof( HwSparky2Data, AccelRange), sizeof(uint8_t));
}
void HwSparky2MPU9250RateSet( uint8_t *NewMPU9250Rate )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewMPU9250Rate, offsetof( HwSparky2Data, MPU9250Rate), sizeof(uint8_t));
}
void HwSparky2MPU9250RateGet( uint8_t *NewMPU9250Rate )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewMPU9250Rate, offsetof( HwSparky2Data, MPU9250Rate), sizeof(uint8_t));
}
void HwSparky2MPU9250GyroLPFSet( uint8_t *NewMPU9250GyroLPF )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewMPU9250GyroLPF, offsetof( HwSparky2Data, MPU9250GyroLPF), sizeof(uint8_t));
}
void HwSparky2MPU9250GyroLPFGet( uint8_t *NewMPU9250GyroLPF )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewMPU9250GyroLPF, offsetof( HwSparky2Data, MPU9250GyroLPF), sizeof(uint8_t));
}
void HwSparky2MPU9250AccelLPFSet( uint8_t *NewMPU9250AccelLPF )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewMPU9250AccelLPF, offsetof( HwSparky2Data, MPU9250AccelLPF), sizeof(uint8_t));
}
void HwSparky2MPU9250AccelLPFGet( uint8_t *NewMPU9250AccelLPF )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewMPU9250AccelLPF, offsetof( HwSparky2Data, MPU9250AccelLPF), sizeof(uint8_t));
}
void HwSparky2VTX_ChSet( uint8_t *NewVTX_Ch )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewVTX_Ch, offsetof( HwSparky2Data, VTX_Ch), sizeof(uint8_t));
}
void HwSparky2VTX_ChGet( uint8_t *NewVTX_Ch )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewVTX_Ch, offsetof( HwSparky2Data, VTX_Ch), sizeof(uint8_t));
}
void HwSparky2MagnetometerSet( uint8_t *NewMagnetometer )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewMagnetometer, offsetof( HwSparky2Data, Magnetometer), sizeof(uint8_t));
}
void HwSparky2MagnetometerGet( uint8_t *NewMagnetometer )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewMagnetometer, offsetof( HwSparky2Data, Magnetometer), sizeof(uint8_t));
}
void HwSparky2ExtMagOrientationSet( uint8_t *NewExtMagOrientation )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewExtMagOrientation, offsetof( HwSparky2Data, ExtMagOrientation), sizeof(uint8_t));
}
void HwSparky2ExtMagOrientationGet( uint8_t *NewExtMagOrientation )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewExtMagOrientation, offsetof( HwSparky2Data, ExtMagOrientation), sizeof(uint8_t));
}
void HwSparky2AdcDacSet( uint8_t *NewAdcDac )
{
	UAVObjSetDataField(HwSparky2Handle(), (void*)NewAdcDac, offsetof( HwSparky2Data, AdcDac), sizeof(uint8_t));
}
void HwSparky2AdcDacGet( uint8_t *NewAdcDac )
{
	UAVObjGetDataField(HwSparky2Handle(), (void*)NewAdcDac, offsetof( HwSparky2Data, AdcDac), sizeof(uint8_t));
}


/**
 * @}
 * @}
 */

