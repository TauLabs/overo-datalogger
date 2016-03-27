/**
 ******************************************************************************
 * @addtogroup TauLabsCore Tau Labs Core components
 * @{
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 *
 * @file       groundtruth.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Implementation of the GroundTruth object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: groundtruth.xml. 
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
#include "groundtruth.h"

// Private variables
static UAVObjHandle handle = NULL;

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t GroundTruthInitialize(void)
{
	// Don't set the handle to null if already registered
	if(UAVObjGetByID(GROUNDTRUTH_OBJID) != NULL)
		return -2;
	
	// Register object with the object manager
	handle = UAVObjRegister(GROUNDTRUTH_OBJID,
			GROUNDTRUTH_ISSINGLEINST, GROUNDTRUTH_ISSETTINGS, GROUNDTRUTH_NUMBYTES, &GroundTruthSetDefaults);

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
void GroundTruthSetDefaults(UAVObjHandle obj, uint16_t instId)
{
	GroundTruthData data;

	// Initialize object fields to their default values
	UAVObjGetInstanceData(obj, instId, &data);
	memset(&data, 0, sizeof(GroundTruthData));

	UAVObjSetInstanceData(obj, instId, &data);

	// Initialize object metadata to their default values
	if (instId == 0) {
		UAVObjMetadata metadata;
		metadata.flags =
			ACCESS_READONLY << UAVOBJ_ACCESS_SHIFT |
			ACCESS_READWRITE << UAVOBJ_GCS_ACCESS_SHIFT |
			0 << UAVOBJ_TELEMETRY_ACKED_SHIFT |
			0 << UAVOBJ_GCS_TELEMETRY_ACKED_SHIFT |
			UPDATEMODE_MANUAL << UAVOBJ_TELEMETRY_UPDATE_MODE_SHIFT |
			UPDATEMODE_PERIODIC << UAVOBJ_GCS_TELEMETRY_UPDATE_MODE_SHIFT;
		metadata.telemetryUpdatePeriod = 50000;
		metadata.gcsTelemetryUpdatePeriod = 50000;
		metadata.loggingUpdatePeriod = 0;
		UAVObjSetMetadata(obj, &metadata);
	}
}

/**
 * Get object handle
 */
UAVObjHandle GroundTruthHandle()
{
	return handle;
}

/**
 * Get/Set object Functions
 */
void GroundTruthAccelerationXYZSet( float *NewAccelerationXYZ )
{
	UAVObjSetDataField(GroundTruthHandle(), (void*)NewAccelerationXYZ, offsetof( GroundTruthData, AccelerationXYZ), 3*sizeof(float));
}
void GroundTruthAccelerationXYZGet( float *NewAccelerationXYZ )
{
	UAVObjGetDataField(GroundTruthHandle(), (void*)NewAccelerationXYZ, offsetof( GroundTruthData, AccelerationXYZ), 3*sizeof(float));
}
void GroundTruthPositionNEDSet( float *NewPositionNED )
{
	UAVObjSetDataField(GroundTruthHandle(), (void*)NewPositionNED, offsetof( GroundTruthData, PositionNED), 3*sizeof(float));
}
void GroundTruthPositionNEDGet( float *NewPositionNED )
{
	UAVObjGetDataField(GroundTruthHandle(), (void*)NewPositionNED, offsetof( GroundTruthData, PositionNED), 3*sizeof(float));
}
void GroundTruthVelocityNEDSet( float *NewVelocityNED )
{
	UAVObjSetDataField(GroundTruthHandle(), (void*)NewVelocityNED, offsetof( GroundTruthData, VelocityNED), 3*sizeof(float));
}
void GroundTruthVelocityNEDGet( float *NewVelocityNED )
{
	UAVObjGetDataField(GroundTruthHandle(), (void*)NewVelocityNED, offsetof( GroundTruthData, VelocityNED), 3*sizeof(float));
}
void GroundTruthRPYSet( float *NewRPY )
{
	UAVObjSetDataField(GroundTruthHandle(), (void*)NewRPY, offsetof( GroundTruthData, RPY), 3*sizeof(float));
}
void GroundTruthRPYGet( float *NewRPY )
{
	UAVObjGetDataField(GroundTruthHandle(), (void*)NewRPY, offsetof( GroundTruthData, RPY), 3*sizeof(float));
}
void GroundTruthAngularRatesSet( float *NewAngularRates )
{
	UAVObjSetDataField(GroundTruthHandle(), (void*)NewAngularRates, offsetof( GroundTruthData, AngularRates), 3*sizeof(float));
}
void GroundTruthAngularRatesGet( float *NewAngularRates )
{
	UAVObjGetDataField(GroundTruthHandle(), (void*)NewAngularRates, offsetof( GroundTruthData, AngularRates), 3*sizeof(float));
}
void GroundTruthTrueAirspeedSet( float *NewTrueAirspeed )
{
	UAVObjSetDataField(GroundTruthHandle(), (void*)NewTrueAirspeed, offsetof( GroundTruthData, TrueAirspeed), sizeof(float));
}
void GroundTruthTrueAirspeedGet( float *NewTrueAirspeed )
{
	UAVObjGetDataField(GroundTruthHandle(), (void*)NewTrueAirspeed, offsetof( GroundTruthData, TrueAirspeed), sizeof(float));
}
void GroundTruthCalibratedAirspeedSet( float *NewCalibratedAirspeed )
{
	UAVObjSetDataField(GroundTruthHandle(), (void*)NewCalibratedAirspeed, offsetof( GroundTruthData, CalibratedAirspeed), sizeof(float));
}
void GroundTruthCalibratedAirspeedGet( float *NewCalibratedAirspeed )
{
	UAVObjGetDataField(GroundTruthHandle(), (void*)NewCalibratedAirspeed, offsetof( GroundTruthData, CalibratedAirspeed), sizeof(float));
}
void GroundTruthAngleOfAttackSet( float *NewAngleOfAttack )
{
	UAVObjSetDataField(GroundTruthHandle(), (void*)NewAngleOfAttack, offsetof( GroundTruthData, AngleOfAttack), sizeof(float));
}
void GroundTruthAngleOfAttackGet( float *NewAngleOfAttack )
{
	UAVObjGetDataField(GroundTruthHandle(), (void*)NewAngleOfAttack, offsetof( GroundTruthData, AngleOfAttack), sizeof(float));
}
void GroundTruthAngleOfSlipSet( float *NewAngleOfSlip )
{
	UAVObjSetDataField(GroundTruthHandle(), (void*)NewAngleOfSlip, offsetof( GroundTruthData, AngleOfSlip), sizeof(float));
}
void GroundTruthAngleOfSlipGet( float *NewAngleOfSlip )
{
	UAVObjGetDataField(GroundTruthHandle(), (void*)NewAngleOfSlip, offsetof( GroundTruthData, AngleOfSlip), sizeof(float));
}


/**
 * @}
 * @}
 */

