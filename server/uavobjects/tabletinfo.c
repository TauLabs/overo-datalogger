/**
 ******************************************************************************
 * @addtogroup TauLabsCore Tau Labs Core components
 * @{
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 *
 * @file       tabletinfo.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Implementation of the TabletInfo object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: tabletinfo.xml. 
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
#include "tabletinfo.h"

// Private variables
static UAVObjHandle handle = NULL;

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t TabletInfoInitialize(void)
{
	// Don't set the handle to null if already registered
	if(UAVObjGetByID(TABLETINFO_OBJID) != NULL)
		return -2;
	
	// Register object with the object manager
	handle = UAVObjRegister(TABLETINFO_OBJID,
			TABLETINFO_ISSINGLEINST, TABLETINFO_ISSETTINGS, TABLETINFO_NUMBYTES, &TabletInfoSetDefaults);

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
void TabletInfoSetDefaults(UAVObjHandle obj, uint16_t instId)
{
	TabletInfoData data;

	// Initialize object fields to their default values
	UAVObjGetInstanceData(obj, instId, &data);
	memset(&data, 0, sizeof(TabletInfoData));

	UAVObjSetInstanceData(obj, instId, &data);

	// Initialize object metadata to their default values
	if (instId == 0) {
		UAVObjMetadata metadata;
		metadata.flags =
			ACCESS_READWRITE << UAVOBJ_ACCESS_SHIFT |
			ACCESS_READWRITE << UAVOBJ_GCS_ACCESS_SHIFT |
			0 << UAVOBJ_TELEMETRY_ACKED_SHIFT |
			0 << UAVOBJ_GCS_TELEMETRY_ACKED_SHIFT |
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
UAVObjHandle TabletInfoHandle()
{
	return handle;
}

/**
 * Get/Set object Functions
 */
void TabletInfoLatitudeSet( int32_t *NewLatitude )
{
	UAVObjSetDataField(TabletInfoHandle(), (void*)NewLatitude, offsetof( TabletInfoData, Latitude), sizeof(int32_t));
}
void TabletInfoLatitudeGet( int32_t *NewLatitude )
{
	UAVObjGetDataField(TabletInfoHandle(), (void*)NewLatitude, offsetof( TabletInfoData, Latitude), sizeof(int32_t));
}
void TabletInfoLongitudeSet( int32_t *NewLongitude )
{
	UAVObjSetDataField(TabletInfoHandle(), (void*)NewLongitude, offsetof( TabletInfoData, Longitude), sizeof(int32_t));
}
void TabletInfoLongitudeGet( int32_t *NewLongitude )
{
	UAVObjGetDataField(TabletInfoHandle(), (void*)NewLongitude, offsetof( TabletInfoData, Longitude), sizeof(int32_t));
}
void TabletInfoAltitudeSet( float *NewAltitude )
{
	UAVObjSetDataField(TabletInfoHandle(), (void*)NewAltitude, offsetof( TabletInfoData, Altitude), sizeof(float));
}
void TabletInfoAltitudeGet( float *NewAltitude )
{
	UAVObjGetDataField(TabletInfoHandle(), (void*)NewAltitude, offsetof( TabletInfoData, Altitude), sizeof(float));
}
void TabletInfoConnectedSet( uint8_t *NewConnected )
{
	UAVObjSetDataField(TabletInfoHandle(), (void*)NewConnected, offsetof( TabletInfoData, Connected), sizeof(uint8_t));
}
void TabletInfoConnectedGet( uint8_t *NewConnected )
{
	UAVObjGetDataField(TabletInfoHandle(), (void*)NewConnected, offsetof( TabletInfoData, Connected), sizeof(uint8_t));
}
void TabletInfoTabletModeDesiredSet( uint8_t *NewTabletModeDesired )
{
	UAVObjSetDataField(TabletInfoHandle(), (void*)NewTabletModeDesired, offsetof( TabletInfoData, TabletModeDesired), sizeof(uint8_t));
}
void TabletInfoTabletModeDesiredGet( uint8_t *NewTabletModeDesired )
{
	UAVObjGetDataField(TabletInfoHandle(), (void*)NewTabletModeDesired, offsetof( TabletInfoData, TabletModeDesired), sizeof(uint8_t));
}
void TabletInfoPOISet( uint8_t *NewPOI )
{
	UAVObjSetDataField(TabletInfoHandle(), (void*)NewPOI, offsetof( TabletInfoData, POI), sizeof(uint8_t));
}
void TabletInfoPOIGet( uint8_t *NewPOI )
{
	UAVObjGetDataField(TabletInfoHandle(), (void*)NewPOI, offsetof( TabletInfoData, POI), sizeof(uint8_t));
}


/**
 * @}
 * @}
 */

