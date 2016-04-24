/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup HwShared HwShared
 * @brief Templates for common enums.
 *
 *
 * @file       hwshared.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the HwShared object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: hwshared.xml. 
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

#ifndef HWSHARED_H
#define HWSHARED_H

#include "pios_queue.h"
#include "uavoversion.h"



// Object constants
#define HWSHARED_OBJID 0x1431B48E
#define HWSHARED_ISSINGLEINST 1
#define HWSHARED_ISSETTINGS 1
#define HWSHARED_NUMBYTES 9

// Generic interface functions
int32_t HwSharedInitialize();
UAVObjHandle HwSharedHandle();
void HwSharedSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    uint8_t PortTypes;
    uint8_t MagOrientation;
    uint8_t USB_HIDPort;
    uint8_t USB_VCPPort;
    uint8_t RadioPort;
    uint8_t MaxRfSpeed;
    uint8_t MaxRfPower;
    uint8_t DSMxMode;
    uint8_t RfBand;

} __attribute__((packed)) __attribute__((aligned(4))) HwSharedData;

// Typesafe Object access functions
/**
 * @function HwSharedGet(dataOut)
 * @brief Populate a HwSharedData object
 * @param[out] dataOut 
 */
static inline int32_t HwSharedGet(HwSharedData *dataOut) { return UAVObjGetData(HwSharedHandle(), dataOut); }

static inline int32_t HwSharedSet(const HwSharedData *dataIn) { return UAVObjSetData(HwSharedHandle(), dataIn); }

static inline int32_t HwSharedInstGet(uint16_t instId, HwSharedData *dataOut) { return UAVObjGetInstanceData(HwSharedHandle(), instId, dataOut); }

static inline int32_t HwSharedInstSet(uint16_t instId, const HwSharedData *dataIn) { return UAVObjSetInstanceData(HwSharedHandle(), instId, dataIn); }

static inline int32_t HwSharedConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(HwSharedHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t HwSharedConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(HwSharedHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t HwSharedCreateInstance() { return UAVObjCreateInstance(HwSharedHandle(), &HwSharedSetDefaults); }

static inline void HwSharedRequestUpdate() { UAVObjRequestUpdate(HwSharedHandle()); }

static inline void HwSharedRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(HwSharedHandle(), instId); }

static inline void HwSharedUpdated() { UAVObjUpdated(HwSharedHandle()); }

static inline void HwSharedInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(HwSharedHandle(), instId); }

static inline int32_t HwSharedGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(HwSharedHandle(), dataOut); }

static inline int32_t HwSharedSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(HwSharedHandle(), dataIn); }

static inline int8_t HwSharedReadOnly() { return UAVObjReadOnly(HwSharedHandle()); }

static inline uint16_t HwSharedGetNumInstances(){ return UAVObjGetNumInstances(HwSharedHandle()); }

static inline uint32_t HwSharedGetNumBytes(){ return UAVObjGetNumBytes(HwSharedHandle()); }

// Field information
// Field PortTypes information
/* Enumeration options for field PortTypes */
typedef enum { HWSHARED_PORTTYPES_DISABLED=0, HWSHARED_PORTTYPES_TELEMETRY=1, HWSHARED_PORTTYPES_GPS=2, HWSHARED_PORTTYPES_DSM=3, HWSHARED_PORTTYPES_SBUS=4, HWSHARED_PORTTYPES_DEBUGCONSOLE=5, HWSHARED_PORTTYPES_COMBRIDGE=6, HWSHARED_PORTTYPES_MAVLINKTX=7, HWSHARED_PORTTYPES_MAVLINKTX_GPS_RX=8, HWSHARED_PORTTYPES_MSP=9, HWSHARED_PORTTYPES_HOTTSUMD=10, HWSHARED_PORTTYPES_HOTTSUMH=11, HWSHARED_PORTTYPES_HOTTTELEMETRY=12, HWSHARED_PORTTYPES_FRSKYSENSORHUB=13, HWSHARED_PORTTYPES_FRSKYSPORTTELEMETRY=14, HWSHARED_PORTTYPES_LIGHTTELEMETRYTX=15, HWSHARED_PORTTYPES_PICOC=16, HWSHARED_PORTTYPES_OPENLOG=17, HWSHARED_PORTTYPES_I2C=18, HWSHARED_PORTTYPES_PPM=19, HWSHARED_PORTTYPES_PWM=20 }  __attribute__((packed)) HwSharedPortTypesOptions;
/* Max value of any option in topmost parent PortTypes of field PortTypes */
#define HWSHARED_PORTTYPES_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field PortTypes */
#define HWSHARED_PORTTYPES_MAXOPTVAL 20
/* Ensure field PortTypes contains valid data */
static inline bool HwSharedPortTypesIsValid( uint8_t CurrentPortTypes ) { return CurrentPortTypes < HWSHARED_PORTTYPES_MAXOPTVAL; }
// Field MagOrientation information
/* Enumeration options for field MagOrientation */
typedef enum { HWSHARED_MAGORIENTATION_TOP0DEGCW=0, HWSHARED_MAGORIENTATION_TOP90DEGCW=1, HWSHARED_MAGORIENTATION_TOP180DEGCW=2, HWSHARED_MAGORIENTATION_TOP270DEGCW=3, HWSHARED_MAGORIENTATION_BOTTOM0DEGCW=4, HWSHARED_MAGORIENTATION_BOTTOM90DEGCW=5, HWSHARED_MAGORIENTATION_BOTTOM180DEGCW=6, HWSHARED_MAGORIENTATION_BOTTOM270DEGCW=7 }  __attribute__((packed)) HwSharedMagOrientationOptions;
/* Max value of any option in topmost parent MagOrientation of field MagOrientation */
#define HWSHARED_MAGORIENTATION_GLOBAL_MAXOPTVAL 7
/* Max value of any option in field MagOrientation */
#define HWSHARED_MAGORIENTATION_MAXOPTVAL 7
/* Ensure field MagOrientation contains valid data */
static inline bool HwSharedMagOrientationIsValid( uint8_t CurrentMagOrientation ) { return CurrentMagOrientation < HWSHARED_MAGORIENTATION_MAXOPTVAL; }
// Field USB_HIDPort information
/* Enumeration options for field USB_HIDPort */
typedef enum { HWSHARED_USB_HIDPORT_USBTELEMETRY=0, HWSHARED_USB_HIDPORT_DISABLED=1 }  __attribute__((packed)) HwSharedUSB_HIDPortOptions;
/* Max value of any option in topmost parent USB_HIDPort of field USB_HIDPort */
#define HWSHARED_USB_HIDPORT_GLOBAL_MAXOPTVAL 1
/* Max value of any option in field USB_HIDPort */
#define HWSHARED_USB_HIDPORT_MAXOPTVAL 1
/* Ensure field USB_HIDPort contains valid data */
static inline bool HwSharedUSB_HIDPortIsValid( uint8_t CurrentUSB_HIDPort ) { return CurrentUSB_HIDPort < HWSHARED_USB_HIDPORT_MAXOPTVAL; }
// Field USB_VCPPort information
/* Enumeration options for field USB_VCPPort */
typedef enum { HWSHARED_USB_VCPPORT_USBTELEMETRY=0, HWSHARED_USB_VCPPORT_COMBRIDGE=1, HWSHARED_USB_VCPPORT_DEBUGCONSOLE=2, HWSHARED_USB_VCPPORT_PICOC=3, HWSHARED_USB_VCPPORT_DISABLED=4 }  __attribute__((packed)) HwSharedUSB_VCPPortOptions;
/* Max value of any option in topmost parent USB_VCPPort of field USB_VCPPort */
#define HWSHARED_USB_VCPPORT_GLOBAL_MAXOPTVAL 4
/* Max value of any option in field USB_VCPPort */
#define HWSHARED_USB_VCPPORT_MAXOPTVAL 4
/* Ensure field USB_VCPPort contains valid data */
static inline bool HwSharedUSB_VCPPortIsValid( uint8_t CurrentUSB_VCPPort ) { return CurrentUSB_VCPPort < HWSHARED_USB_VCPPORT_MAXOPTVAL; }
// Field RadioPort information
/* Enumeration options for field RadioPort */
typedef enum { HWSHARED_RADIOPORT_DISABLED=0, HWSHARED_RADIOPORT_TELEM=1, HWSHARED_RADIOPORT_TELEMPPM=2, HWSHARED_RADIOPORT_PPM=3, HWSHARED_RADIOPORT_OPENLRS=4 }  __attribute__((packed)) HwSharedRadioPortOptions;
/* Max value of any option in topmost parent RadioPort of field RadioPort */
#define HWSHARED_RADIOPORT_GLOBAL_MAXOPTVAL 4
/* Max value of any option in field RadioPort */
#define HWSHARED_RADIOPORT_MAXOPTVAL 4
/* Ensure field RadioPort contains valid data */
static inline bool HwSharedRadioPortIsValid( uint8_t CurrentRadioPort ) { return CurrentRadioPort < HWSHARED_RADIOPORT_MAXOPTVAL; }
// Field MaxRfSpeed information
/* Enumeration options for field MaxRfSpeed */
typedef enum { HWSHARED_MAXRFSPEED_9600=0, HWSHARED_MAXRFSPEED_19200=1, HWSHARED_MAXRFSPEED_32000=2, HWSHARED_MAXRFSPEED_64000=3, HWSHARED_MAXRFSPEED_100000=4, HWSHARED_MAXRFSPEED_192000=5 }  __attribute__((packed)) HwSharedMaxRfSpeedOptions;
/* Max value of any option in topmost parent MaxRfSpeed of field MaxRfSpeed */
#define HWSHARED_MAXRFSPEED_GLOBAL_MAXOPTVAL 5
/* Max value of any option in field MaxRfSpeed */
#define HWSHARED_MAXRFSPEED_MAXOPTVAL 5
/* Ensure field MaxRfSpeed contains valid data */
static inline bool HwSharedMaxRfSpeedIsValid( uint8_t CurrentMaxRfSpeed ) { return CurrentMaxRfSpeed < HWSHARED_MAXRFSPEED_MAXOPTVAL; }
// Field MaxRfPower information
/* Enumeration options for field MaxRfPower */
typedef enum { HWSHARED_MAXRFPOWER_0=0, HWSHARED_MAXRFPOWER_125=1, HWSHARED_MAXRFPOWER_16=2, HWSHARED_MAXRFPOWER_316=3, HWSHARED_MAXRFPOWER_63=4, HWSHARED_MAXRFPOWER_126=5, HWSHARED_MAXRFPOWER_25=6, HWSHARED_MAXRFPOWER_50=7, HWSHARED_MAXRFPOWER_100=8 }  __attribute__((packed)) HwSharedMaxRfPowerOptions;
/* Max value of any option in topmost parent MaxRfPower of field MaxRfPower */
#define HWSHARED_MAXRFPOWER_GLOBAL_MAXOPTVAL 8
/* Max value of any option in field MaxRfPower */
#define HWSHARED_MAXRFPOWER_MAXOPTVAL 8
/* Ensure field MaxRfPower contains valid data */
static inline bool HwSharedMaxRfPowerIsValid( uint8_t CurrentMaxRfPower ) { return CurrentMaxRfPower < HWSHARED_MAXRFPOWER_MAXOPTVAL; }
// Field DSMxMode information
/* Enumeration options for field DSMxMode */
typedef enum { HWSHARED_DSMXMODE_AUTODETECT=0, HWSHARED_DSMXMODE_FORCE10BIT=1, HWSHARED_DSMXMODE_FORCE11BIT=2, HWSHARED_DSMXMODE_BIND3PULSES=3, HWSHARED_DSMXMODE_BIND4PULSES=4, HWSHARED_DSMXMODE_BIND5PULSES=5, HWSHARED_DSMXMODE_BIND6PULSES=6, HWSHARED_DSMXMODE_BIND7PULSES=7, HWSHARED_DSMXMODE_BIND8PULSES=8, HWSHARED_DSMXMODE_BIND9PULSES=9, HWSHARED_DSMXMODE_BIND10PULSES=10 }  __attribute__((packed)) HwSharedDSMxModeOptions;
/* Max value of any option in topmost parent DSMxMode of field DSMxMode */
#define HWSHARED_DSMXMODE_GLOBAL_MAXOPTVAL 10
/* Max value of any option in field DSMxMode */
#define HWSHARED_DSMXMODE_MAXOPTVAL 10
/* Ensure field DSMxMode contains valid data */
static inline bool HwSharedDSMxModeIsValid( uint8_t CurrentDSMxMode ) { return CurrentDSMxMode < HWSHARED_DSMXMODE_MAXOPTVAL; }
// Field RfBand information
/* Enumeration options for field RfBand */
typedef enum { HWSHARED_RFBAND_BOARDDEFAULT=0, HWSHARED_RFBAND_433=1, HWSHARED_RFBAND_868=2, HWSHARED_RFBAND_915=3 }  __attribute__((packed)) HwSharedRfBandOptions;
/* Max value of any option in topmost parent RfBand of field RfBand */
#define HWSHARED_RFBAND_GLOBAL_MAXOPTVAL 3
/* Max value of any option in field RfBand */
#define HWSHARED_RFBAND_MAXOPTVAL 3
/* Ensure field RfBand contains valid data */
static inline bool HwSharedRfBandIsValid( uint8_t CurrentRfBand ) { return CurrentRfBand < HWSHARED_RFBAND_MAXOPTVAL; }


// set/Get functions
extern void HwSharedPortTypesSet( uint8_t *NewPortTypes );
extern void HwSharedPortTypesGet( uint8_t *NewPortTypes );
extern void HwSharedMagOrientationSet( uint8_t *NewMagOrientation );
extern void HwSharedMagOrientationGet( uint8_t *NewMagOrientation );
extern void HwSharedUSB_HIDPortSet( uint8_t *NewUSB_HIDPort );
extern void HwSharedUSB_HIDPortGet( uint8_t *NewUSB_HIDPort );
extern void HwSharedUSB_VCPPortSet( uint8_t *NewUSB_VCPPort );
extern void HwSharedUSB_VCPPortGet( uint8_t *NewUSB_VCPPort );
extern void HwSharedRadioPortSet( uint8_t *NewRadioPort );
extern void HwSharedRadioPortGet( uint8_t *NewRadioPort );
extern void HwSharedMaxRfSpeedSet( uint8_t *NewMaxRfSpeed );
extern void HwSharedMaxRfSpeedGet( uint8_t *NewMaxRfSpeed );
extern void HwSharedMaxRfPowerSet( uint8_t *NewMaxRfPower );
extern void HwSharedMaxRfPowerGet( uint8_t *NewMaxRfPower );
extern void HwSharedDSMxModeSet( uint8_t *NewDSMxMode );
extern void HwSharedDSMxModeGet( uint8_t *NewDSMxMode );
extern void HwSharedRfBandSet( uint8_t *NewRfBand );
extern void HwSharedRfBandGet( uint8_t *NewRfBand );


#endif // HWSHARED_H

/**
 * @}
 * @}
 */