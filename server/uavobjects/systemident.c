/**
 ******************************************************************************
 * @addtogroup TauLabsCore Tau Labs Core components
 * @{
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 *
 * @file       systemident.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Implementation of the SystemIdent object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: systemident.xml. 
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
#include "systemident.h"

// Private variables
static UAVObjHandle handle = NULL;

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t SystemIdentInitialize(void)
{
	// Don't set the handle to null if already registered
	if(UAVObjGetByID(SYSTEMIDENT_OBJID) != NULL)
		return -2;
	
	// Register object with the object manager
	handle = UAVObjRegister(SYSTEMIDENT_OBJID,
			SYSTEMIDENT_ISSINGLEINST, SYSTEMIDENT_ISSETTINGS, SYSTEMIDENT_NUMBYTES, &SystemIdentSetDefaults);

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
void SystemIdentSetDefaults(UAVObjHandle obj, uint16_t instId)
{
	SystemIdentData data;

	// Initialize object fields to their default values
	UAVObjGetInstanceData(obj, instId, &data);
	memset(&data, 0, sizeof(SystemIdentData));
	data.Tau = 0;
	data.Beta[0] = 0;
	data.Beta[1] = 0;
	data.Beta[2] = 0;
	data.Bias[0] = 0;
	data.Bias[1] = 0;
	data.Bias[2] = 0;
	data.Noise[0] = 0;
	data.Noise[1] = 0;
	data.Noise[2] = 0;
	data.Period = 0;
	data.NumAfPredicts = 0;

	UAVObjSetInstanceData(obj, instId, &data);

	// Initialize object metadata to their default values
	if (instId == 0) {
		UAVObjMetadata metadata;
		metadata.flags =
			ACCESS_READWRITE << UAVOBJ_ACCESS_SHIFT |
			ACCESS_READONLY << UAVOBJ_GCS_ACCESS_SHIFT |
			0 << UAVOBJ_TELEMETRY_ACKED_SHIFT |
			0 << UAVOBJ_GCS_TELEMETRY_ACKED_SHIFT |
			UPDATEMODE_ONCHANGE << UAVOBJ_TELEMETRY_UPDATE_MODE_SHIFT |
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
UAVObjHandle SystemIdentHandle()
{
	return handle;
}

/**
 * Get/Set object Functions
 */
void SystemIdentTauSet( float *NewTau )
{
	UAVObjSetDataField(SystemIdentHandle(), (void*)NewTau, offsetof( SystemIdentData, Tau), sizeof(float));
}
void SystemIdentTauGet( float *NewTau )
{
	UAVObjGetDataField(SystemIdentHandle(), (void*)NewTau, offsetof( SystemIdentData, Tau), sizeof(float));
}
void SystemIdentBetaSet( float *NewBeta )
{
	UAVObjSetDataField(SystemIdentHandle(), (void*)NewBeta, offsetof( SystemIdentData, Beta), 3*sizeof(float));
}
void SystemIdentBetaGet( float *NewBeta )
{
	UAVObjGetDataField(SystemIdentHandle(), (void*)NewBeta, offsetof( SystemIdentData, Beta), 3*sizeof(float));
}
void SystemIdentBiasSet( float *NewBias )
{
	UAVObjSetDataField(SystemIdentHandle(), (void*)NewBias, offsetof( SystemIdentData, Bias), 3*sizeof(float));
}
void SystemIdentBiasGet( float *NewBias )
{
	UAVObjGetDataField(SystemIdentHandle(), (void*)NewBias, offsetof( SystemIdentData, Bias), 3*sizeof(float));
}
void SystemIdentNoiseSet( float *NewNoise )
{
	UAVObjSetDataField(SystemIdentHandle(), (void*)NewNoise, offsetof( SystemIdentData, Noise), 3*sizeof(float));
}
void SystemIdentNoiseGet( float *NewNoise )
{
	UAVObjGetDataField(SystemIdentHandle(), (void*)NewNoise, offsetof( SystemIdentData, Noise), 3*sizeof(float));
}
void SystemIdentPeriodSet( float *NewPeriod )
{
	UAVObjSetDataField(SystemIdentHandle(), (void*)NewPeriod, offsetof( SystemIdentData, Period), sizeof(float));
}
void SystemIdentPeriodGet( float *NewPeriod )
{
	UAVObjGetDataField(SystemIdentHandle(), (void*)NewPeriod, offsetof( SystemIdentData, Period), sizeof(float));
}
void SystemIdentNumAfPredictsSet( uint32_t *NewNumAfPredicts )
{
	UAVObjSetDataField(SystemIdentHandle(), (void*)NewNumAfPredicts, offsetof( SystemIdentData, NumAfPredicts), sizeof(uint32_t));
}
void SystemIdentNumAfPredictsGet( uint32_t *NewNumAfPredicts )
{
	UAVObjGetDataField(SystemIdentHandle(), (void*)NewNumAfPredicts, offsetof( SystemIdentData, NumAfPredicts), sizeof(uint32_t));
}


/**
 * @}
 * @}
 */

