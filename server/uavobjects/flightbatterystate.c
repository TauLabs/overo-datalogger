/**
 ******************************************************************************
 * @addtogroup TauLabsCore Tau Labs Core components
 * @{
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 *
 * @file       flightbatterystate.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Implementation of the FlightBatteryState object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: flightbatterystate.xml. 
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
#include "flightbatterystate.h"

// Private variables
static UAVObjHandle handle = NULL;

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t FlightBatteryStateInitialize(void)
{
	// Don't set the handle to null if already registered
	if(UAVObjGetByID(FLIGHTBATTERYSTATE_OBJID) != NULL)
		return -2;
	
	// Register object with the object manager
	handle = UAVObjRegister(FLIGHTBATTERYSTATE_OBJID,
			FLIGHTBATTERYSTATE_ISSINGLEINST, FLIGHTBATTERYSTATE_ISSETTINGS, FLIGHTBATTERYSTATE_NUMBYTES, &FlightBatteryStateSetDefaults);

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
void FlightBatteryStateSetDefaults(UAVObjHandle obj, uint16_t instId)
{
	FlightBatteryStateData data;

	// Initialize object fields to their default values
	UAVObjGetInstanceData(obj, instId, &data);
	memset(&data, 0, sizeof(FlightBatteryStateData));
	data.Voltage = 0;
	data.Current = 0;
	data.BoardSupplyVoltage = 0;
	data.PeakCurrent = 0;
	data.AvgCurrent = 0;
	data.ConsumedEnergy = 0;
	data.EstimatedFlightTime = 0;

	UAVObjSetInstanceData(obj, instId, &data);

	// Initialize object metadata to their default values
	if (instId == 0) {
		UAVObjMetadata metadata;
		metadata.flags =
			ACCESS_READWRITE << UAVOBJ_ACCESS_SHIFT |
			ACCESS_READONLY << UAVOBJ_GCS_ACCESS_SHIFT |
			0 << UAVOBJ_TELEMETRY_ACKED_SHIFT |
			0 << UAVOBJ_GCS_TELEMETRY_ACKED_SHIFT |
			UPDATEMODE_PERIODIC << UAVOBJ_TELEMETRY_UPDATE_MODE_SHIFT |
			UPDATEMODE_MANUAL << UAVOBJ_GCS_TELEMETRY_UPDATE_MODE_SHIFT;
		metadata.telemetryUpdatePeriod = 1000;
		metadata.gcsTelemetryUpdatePeriod = 0;
		metadata.loggingUpdatePeriod = 0;
		UAVObjSetMetadata(obj, &metadata);
	}
}

/**
 * Get object handle
 */
UAVObjHandle FlightBatteryStateHandle()
{
	return handle;
}

/**
 * Get/Set object Functions
 */
void FlightBatteryStateVoltageSet( float *NewVoltage )
{
	UAVObjSetDataField(FlightBatteryStateHandle(), (void*)NewVoltage, offsetof( FlightBatteryStateData, Voltage), sizeof(float));
}
void FlightBatteryStateVoltageGet( float *NewVoltage )
{
	UAVObjGetDataField(FlightBatteryStateHandle(), (void*)NewVoltage, offsetof( FlightBatteryStateData, Voltage), sizeof(float));
}
void FlightBatteryStateCurrentSet( float *NewCurrent )
{
	UAVObjSetDataField(FlightBatteryStateHandle(), (void*)NewCurrent, offsetof( FlightBatteryStateData, Current), sizeof(float));
}
void FlightBatteryStateCurrentGet( float *NewCurrent )
{
	UAVObjGetDataField(FlightBatteryStateHandle(), (void*)NewCurrent, offsetof( FlightBatteryStateData, Current), sizeof(float));
}
void FlightBatteryStateBoardSupplyVoltageSet( float *NewBoardSupplyVoltage )
{
	UAVObjSetDataField(FlightBatteryStateHandle(), (void*)NewBoardSupplyVoltage, offsetof( FlightBatteryStateData, BoardSupplyVoltage), sizeof(float));
}
void FlightBatteryStateBoardSupplyVoltageGet( float *NewBoardSupplyVoltage )
{
	UAVObjGetDataField(FlightBatteryStateHandle(), (void*)NewBoardSupplyVoltage, offsetof( FlightBatteryStateData, BoardSupplyVoltage), sizeof(float));
}
void FlightBatteryStatePeakCurrentSet( float *NewPeakCurrent )
{
	UAVObjSetDataField(FlightBatteryStateHandle(), (void*)NewPeakCurrent, offsetof( FlightBatteryStateData, PeakCurrent), sizeof(float));
}
void FlightBatteryStatePeakCurrentGet( float *NewPeakCurrent )
{
	UAVObjGetDataField(FlightBatteryStateHandle(), (void*)NewPeakCurrent, offsetof( FlightBatteryStateData, PeakCurrent), sizeof(float));
}
void FlightBatteryStateAvgCurrentSet( float *NewAvgCurrent )
{
	UAVObjSetDataField(FlightBatteryStateHandle(), (void*)NewAvgCurrent, offsetof( FlightBatteryStateData, AvgCurrent), sizeof(float));
}
void FlightBatteryStateAvgCurrentGet( float *NewAvgCurrent )
{
	UAVObjGetDataField(FlightBatteryStateHandle(), (void*)NewAvgCurrent, offsetof( FlightBatteryStateData, AvgCurrent), sizeof(float));
}
void FlightBatteryStateConsumedEnergySet( float *NewConsumedEnergy )
{
	UAVObjSetDataField(FlightBatteryStateHandle(), (void*)NewConsumedEnergy, offsetof( FlightBatteryStateData, ConsumedEnergy), sizeof(float));
}
void FlightBatteryStateConsumedEnergyGet( float *NewConsumedEnergy )
{
	UAVObjGetDataField(FlightBatteryStateHandle(), (void*)NewConsumedEnergy, offsetof( FlightBatteryStateData, ConsumedEnergy), sizeof(float));
}
void FlightBatteryStateEstimatedFlightTimeSet( float *NewEstimatedFlightTime )
{
	UAVObjSetDataField(FlightBatteryStateHandle(), (void*)NewEstimatedFlightTime, offsetof( FlightBatteryStateData, EstimatedFlightTime), sizeof(float));
}
void FlightBatteryStateEstimatedFlightTimeGet( float *NewEstimatedFlightTime )
{
	UAVObjGetDataField(FlightBatteryStateHandle(), (void*)NewEstimatedFlightTime, offsetof( FlightBatteryStateData, EstimatedFlightTime), sizeof(float));
}


/**
 * @}
 * @}
 */

