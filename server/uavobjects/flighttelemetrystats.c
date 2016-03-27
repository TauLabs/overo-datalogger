/**
 ******************************************************************************
 * @addtogroup TauLabsCore Tau Labs Core components
 * @{
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 *
 * @file       flighttelemetrystats.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Implementation of the FlightTelemetryStats object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: flighttelemetrystats.xml. 
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
#include "flighttelemetrystats.h"

// Private variables
static UAVObjHandle handle = NULL;

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t FlightTelemetryStatsInitialize(void)
{
	// Don't set the handle to null if already registered
	if(UAVObjGetByID(FLIGHTTELEMETRYSTATS_OBJID) != NULL)
		return -2;
	
	// Register object with the object manager
	handle = UAVObjRegister(FLIGHTTELEMETRYSTATS_OBJID,
			FLIGHTTELEMETRYSTATS_ISSINGLEINST, FLIGHTTELEMETRYSTATS_ISSETTINGS, FLIGHTTELEMETRYSTATS_NUMBYTES, &FlightTelemetryStatsSetDefaults);

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
void FlightTelemetryStatsSetDefaults(UAVObjHandle obj, uint16_t instId)
{
	FlightTelemetryStatsData data;

	// Initialize object fields to their default values
	UAVObjGetInstanceData(obj, instId, &data);
	memset(&data, 0, sizeof(FlightTelemetryStatsData));

	UAVObjSetInstanceData(obj, instId, &data);

	// Initialize object metadata to their default values
	if (instId == 0) {
		UAVObjMetadata metadata;
		metadata.flags =
			ACCESS_READWRITE << UAVOBJ_ACCESS_SHIFT |
			ACCESS_READWRITE << UAVOBJ_GCS_ACCESS_SHIFT |
			0 << UAVOBJ_TELEMETRY_ACKED_SHIFT |
			0 << UAVOBJ_GCS_TELEMETRY_ACKED_SHIFT |
			UPDATEMODE_PERIODIC << UAVOBJ_TELEMETRY_UPDATE_MODE_SHIFT |
			UPDATEMODE_MANUAL << UAVOBJ_GCS_TELEMETRY_UPDATE_MODE_SHIFT;
		metadata.telemetryUpdatePeriod = 5000;
		metadata.gcsTelemetryUpdatePeriod = 0;
		metadata.loggingUpdatePeriod = 5000;
		UAVObjSetMetadata(obj, &metadata);
	}
}

/**
 * Get object handle
 */
UAVObjHandle FlightTelemetryStatsHandle()
{
	return handle;
}

/**
 * Get/Set object Functions
 */
void FlightTelemetryStatsTxDataRateSet( float *NewTxDataRate )
{
	UAVObjSetDataField(FlightTelemetryStatsHandle(), (void*)NewTxDataRate, offsetof( FlightTelemetryStatsData, TxDataRate), sizeof(float));
}
void FlightTelemetryStatsTxDataRateGet( float *NewTxDataRate )
{
	UAVObjGetDataField(FlightTelemetryStatsHandle(), (void*)NewTxDataRate, offsetof( FlightTelemetryStatsData, TxDataRate), sizeof(float));
}
void FlightTelemetryStatsRxDataRateSet( float *NewRxDataRate )
{
	UAVObjSetDataField(FlightTelemetryStatsHandle(), (void*)NewRxDataRate, offsetof( FlightTelemetryStatsData, RxDataRate), sizeof(float));
}
void FlightTelemetryStatsRxDataRateGet( float *NewRxDataRate )
{
	UAVObjGetDataField(FlightTelemetryStatsHandle(), (void*)NewRxDataRate, offsetof( FlightTelemetryStatsData, RxDataRate), sizeof(float));
}
void FlightTelemetryStatsTxFailuresSet( uint32_t *NewTxFailures )
{
	UAVObjSetDataField(FlightTelemetryStatsHandle(), (void*)NewTxFailures, offsetof( FlightTelemetryStatsData, TxFailures), sizeof(uint32_t));
}
void FlightTelemetryStatsTxFailuresGet( uint32_t *NewTxFailures )
{
	UAVObjGetDataField(FlightTelemetryStatsHandle(), (void*)NewTxFailures, offsetof( FlightTelemetryStatsData, TxFailures), sizeof(uint32_t));
}
void FlightTelemetryStatsRxFailuresSet( uint32_t *NewRxFailures )
{
	UAVObjSetDataField(FlightTelemetryStatsHandle(), (void*)NewRxFailures, offsetof( FlightTelemetryStatsData, RxFailures), sizeof(uint32_t));
}
void FlightTelemetryStatsRxFailuresGet( uint32_t *NewRxFailures )
{
	UAVObjGetDataField(FlightTelemetryStatsHandle(), (void*)NewRxFailures, offsetof( FlightTelemetryStatsData, RxFailures), sizeof(uint32_t));
}
void FlightTelemetryStatsTxRetriesSet( uint32_t *NewTxRetries )
{
	UAVObjSetDataField(FlightTelemetryStatsHandle(), (void*)NewTxRetries, offsetof( FlightTelemetryStatsData, TxRetries), sizeof(uint32_t));
}
void FlightTelemetryStatsTxRetriesGet( uint32_t *NewTxRetries )
{
	UAVObjGetDataField(FlightTelemetryStatsHandle(), (void*)NewTxRetries, offsetof( FlightTelemetryStatsData, TxRetries), sizeof(uint32_t));
}
void FlightTelemetryStatsStatusSet( uint8_t *NewStatus )
{
	UAVObjSetDataField(FlightTelemetryStatsHandle(), (void*)NewStatus, offsetof( FlightTelemetryStatsData, Status), sizeof(uint8_t));
}
void FlightTelemetryStatsStatusGet( uint8_t *NewStatus )
{
	UAVObjGetDataField(FlightTelemetryStatsHandle(), (void*)NewStatus, offsetof( FlightTelemetryStatsData, Status), sizeof(uint8_t));
}


/**
 * @}
 * @}
 */

