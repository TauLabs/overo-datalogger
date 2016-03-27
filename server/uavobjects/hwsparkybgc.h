/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup HwSparkyBGC HwSparkyBGC
 * @brief Selection of optional hardware configurations.
 *
 *
 * @file       hwsparkybgc.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the HwSparkyBGC object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: hwsparkybgc.xml. 
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

#ifndef HWSPARKYBGC_H
#define HWSPARKYBGC_H

#include "pios_queue.h"
#include "uavoversion.h"

#include <hwshared.h>


// Object constants
#define HWSPARKYBGC_OBJID 0x37240138
#define HWSPARKYBGC_ISSINGLEINST 1
#define HWSPARKYBGC_ISSETTINGS 1
#define HWSPARKYBGC_NUMBYTES 9

// Generic interface functions
int32_t HwSparkyBGCInitialize();
UAVObjHandle HwSparkyBGCHandle();
void HwSparkyBGCSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    uint8_t RcvrPort;
    uint8_t FlexiPort;
    uint8_t USB_HIDPort;
    uint8_t USB_VCPPort;
    uint8_t DSMxMode;
    uint8_t GyroRange;
    uint8_t AccelRange;
    uint8_t MPU9150DLPF;
    uint8_t MPU9150Rate;

} __attribute__((packed)) __attribute__((aligned(4))) HwSparkyBGCData;

// Typesafe Object access functions
/**
 * @function HwSparkyBGCGet(dataOut)
 * @brief Populate a HwSparkyBGCData object
 * @param[out] dataOut 
 */
static inline int32_t HwSparkyBGCGet(HwSparkyBGCData *dataOut) { return UAVObjGetData(HwSparkyBGCHandle(), dataOut); }

static inline int32_t HwSparkyBGCSet(const HwSparkyBGCData *dataIn) { return UAVObjSetData(HwSparkyBGCHandle(), dataIn); }

static inline int32_t HwSparkyBGCInstGet(uint16_t instId, HwSparkyBGCData *dataOut) { return UAVObjGetInstanceData(HwSparkyBGCHandle(), instId, dataOut); }

static inline int32_t HwSparkyBGCInstSet(uint16_t instId, const HwSparkyBGCData *dataIn) { return UAVObjSetInstanceData(HwSparkyBGCHandle(), instId, dataIn); }

static inline int32_t HwSparkyBGCConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(HwSparkyBGCHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t HwSparkyBGCConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(HwSparkyBGCHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t HwSparkyBGCCreateInstance() { return UAVObjCreateInstance(HwSparkyBGCHandle(), &HwSparkyBGCSetDefaults); }

static inline void HwSparkyBGCRequestUpdate() { UAVObjRequestUpdate(HwSparkyBGCHandle()); }

static inline void HwSparkyBGCRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(HwSparkyBGCHandle(), instId); }

static inline void HwSparkyBGCUpdated() { UAVObjUpdated(HwSparkyBGCHandle()); }

static inline void HwSparkyBGCInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(HwSparkyBGCHandle(), instId); }

static inline int32_t HwSparkyBGCGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(HwSparkyBGCHandle(), dataOut); }

static inline int32_t HwSparkyBGCSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(HwSparkyBGCHandle(), dataIn); }

static inline int8_t HwSparkyBGCReadOnly() { return UAVObjReadOnly(HwSparkyBGCHandle()); }

static inline uint16_t HwSparkyBGCGetNumInstances(){ return UAVObjGetNumInstances(HwSparkyBGCHandle()); }

static inline uint32_t HwSparkyBGCGetNumBytes(){ return UAVObjGetNumBytes(HwSparkyBGCHandle()); }

// Field information
// Field RcvrPort information
/* Enumeration options for field RcvrPort */
typedef enum { HWSPARKYBGC_RCVRPORT_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWSPARKYBGC_RCVRPORT_PPM=HWSHARED_PORTTYPES_PPM, HWSPARKYBGC_RCVRPORT_SBUS=HWSHARED_PORTTYPES_SBUS, HWSPARKYBGC_RCVRPORT_DSM=HWSHARED_PORTTYPES_DSM, HWSPARKYBGC_RCVRPORT_HOTTSUMD=HWSHARED_PORTTYPES_HOTTSUMD, HWSPARKYBGC_RCVRPORT_HOTTSUMH=HWSHARED_PORTTYPES_HOTTSUMH, HWSPARKYBGC_RCVRPORT_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwSparkyBGCRcvrPortOptions;
/* Max value of any option in topmost parent PortTypes of field RcvrPort */
#define HWSPARKYBGC_RCVRPORT_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field RcvrPort */
#define HWSPARKYBGC_RCVRPORT_MAXOPTVAL 19
/* Ensure field RcvrPort contains valid data */
static inline bool HwSparkyBGCRcvrPortIsValid( uint8_t CurrentRcvrPort ) { return CurrentRcvrPort < HWSPARKYBGC_RCVRPORT_MAXOPTVAL; }
// Field FlexiPort information
/* Enumeration options for field FlexiPort */
typedef enum { HWSPARKYBGC_FLEXIPORT_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWSPARKYBGC_FLEXIPORT_TELEMETRY=HWSHARED_PORTTYPES_TELEMETRY, HWSPARKYBGC_FLEXIPORT_DEBUGCONSOLE=HWSHARED_PORTTYPES_DEBUGCONSOLE, HWSPARKYBGC_FLEXIPORT_COMBRIDGE=HWSHARED_PORTTYPES_COMBRIDGE, HWSPARKYBGC_FLEXIPORT_GPS=HWSHARED_PORTTYPES_GPS, HWSPARKYBGC_FLEXIPORT_SBUS=HWSHARED_PORTTYPES_SBUS, HWSPARKYBGC_FLEXIPORT_DSM=HWSHARED_PORTTYPES_DSM, HWSPARKYBGC_FLEXIPORT_MAVLINKTX=HWSHARED_PORTTYPES_MAVLINKTX, HWSPARKYBGC_FLEXIPORT_MAVLINKTX_GPS_RX=HWSHARED_PORTTYPES_MAVLINKTX_GPS_RX, HWSPARKYBGC_FLEXIPORT_HOTTTELEMETRY=HWSHARED_PORTTYPES_HOTTTELEMETRY, HWSPARKYBGC_FLEXIPORT_FRSKYSENSORHUB=HWSHARED_PORTTYPES_FRSKYSENSORHUB, HWSPARKYBGC_FLEXIPORT_LIGHTTELEMETRYTX=HWSHARED_PORTTYPES_LIGHTTELEMETRYTX, HWSPARKYBGC_FLEXIPORT_FRSKYSPORTTELEMETRY=HWSHARED_PORTTYPES_FRSKYSPORTTELEMETRY, HWSPARKYBGC_FLEXIPORT_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwSparkyBGCFlexiPortOptions;
/* Max value of any option in topmost parent PortTypes of field FlexiPort */
#define HWSPARKYBGC_FLEXIPORT_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field FlexiPort */
#define HWSPARKYBGC_FLEXIPORT_MAXOPTVAL 15
/* Ensure field FlexiPort contains valid data */
static inline bool HwSparkyBGCFlexiPortIsValid( uint8_t CurrentFlexiPort ) { return CurrentFlexiPort < HWSPARKYBGC_FLEXIPORT_MAXOPTVAL; }
// Field USB_HIDPort information
/* Enumeration options for field USB_HIDPort */
typedef enum { HWSPARKYBGC_USB_HIDPORT_USBTELEMETRY=HWSHARED_USB_HIDPORT_USBTELEMETRY, HWSPARKYBGC_USB_HIDPORT_DISABLED=HWSHARED_USB_HIDPORT_DISABLED, HWSPARKYBGC_USB_HIDPORT_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwSparkyBGCUSB_HIDPortOptions;
/* Max value of any option in topmost parent USB_HIDPort of field USB_HIDPort */
#define HWSPARKYBGC_USB_HIDPORT_GLOBAL_MAXOPTVAL 1
/* Max value of any option in field USB_HIDPort */
#define HWSPARKYBGC_USB_HIDPORT_MAXOPTVAL 1
/* Ensure field USB_HIDPort contains valid data */
static inline bool HwSparkyBGCUSB_HIDPortIsValid( uint8_t CurrentUSB_HIDPort ) { return CurrentUSB_HIDPort < HWSPARKYBGC_USB_HIDPORT_MAXOPTVAL; }
// Field USB_VCPPort information
/* Enumeration options for field USB_VCPPort */
typedef enum { HWSPARKYBGC_USB_VCPPORT_USBTELEMETRY=HWSHARED_USB_VCPPORT_USBTELEMETRY, HWSPARKYBGC_USB_VCPPORT_COMBRIDGE=HWSHARED_USB_VCPPORT_COMBRIDGE, HWSPARKYBGC_USB_VCPPORT_DEBUGCONSOLE=HWSHARED_USB_VCPPORT_DEBUGCONSOLE, HWSPARKYBGC_USB_VCPPORT_PICOC=HWSHARED_USB_VCPPORT_PICOC, HWSPARKYBGC_USB_VCPPORT_DISABLED=HWSHARED_USB_VCPPORT_DISABLED, HWSPARKYBGC_USB_VCPPORT_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwSparkyBGCUSB_VCPPortOptions;
/* Max value of any option in topmost parent USB_VCPPort of field USB_VCPPort */
#define HWSPARKYBGC_USB_VCPPORT_GLOBAL_MAXOPTVAL 4
/* Max value of any option in field USB_VCPPort */
#define HWSPARKYBGC_USB_VCPPORT_MAXOPTVAL 4
/* Ensure field USB_VCPPort contains valid data */
static inline bool HwSparkyBGCUSB_VCPPortIsValid( uint8_t CurrentUSB_VCPPort ) { return CurrentUSB_VCPPort < HWSPARKYBGC_USB_VCPPORT_MAXOPTVAL; }
// Field DSMxMode information
/* Enumeration options for field DSMxMode */
typedef enum { HWSPARKYBGC_DSMXMODE_AUTODETECT=HWSHARED_DSMXMODE_AUTODETECT, HWSPARKYBGC_DSMXMODE_FORCE10BIT=HWSHARED_DSMXMODE_FORCE10BIT, HWSPARKYBGC_DSMXMODE_FORCE11BIT=HWSHARED_DSMXMODE_FORCE11BIT, HWSPARKYBGC_DSMXMODE_BIND3PULSES=HWSHARED_DSMXMODE_BIND3PULSES, HWSPARKYBGC_DSMXMODE_BIND4PULSES=HWSHARED_DSMXMODE_BIND4PULSES, HWSPARKYBGC_DSMXMODE_BIND5PULSES=HWSHARED_DSMXMODE_BIND5PULSES, HWSPARKYBGC_DSMXMODE_BIND6PULSES=HWSHARED_DSMXMODE_BIND6PULSES, HWSPARKYBGC_DSMXMODE_BIND7PULSES=HWSHARED_DSMXMODE_BIND7PULSES, HWSPARKYBGC_DSMXMODE_BIND8PULSES=HWSHARED_DSMXMODE_BIND8PULSES, HWSPARKYBGC_DSMXMODE_BIND9PULSES=HWSHARED_DSMXMODE_BIND9PULSES, HWSPARKYBGC_DSMXMODE_BIND10PULSES=HWSHARED_DSMXMODE_BIND10PULSES, HWSPARKYBGC_DSMXMODE_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwSparkyBGCDSMxModeOptions;
/* Max value of any option in topmost parent DSMxMode of field DSMxMode */
#define HWSPARKYBGC_DSMXMODE_GLOBAL_MAXOPTVAL 10
/* Max value of any option in field DSMxMode */
#define HWSPARKYBGC_DSMXMODE_MAXOPTVAL 10
/* Ensure field DSMxMode contains valid data */
static inline bool HwSparkyBGCDSMxModeIsValid( uint8_t CurrentDSMxMode ) { return CurrentDSMxMode < HWSPARKYBGC_DSMXMODE_MAXOPTVAL; }
// Field GyroRange information
/* Enumeration options for field GyroRange */
typedef enum { HWSPARKYBGC_GYRORANGE_250=0, HWSPARKYBGC_GYRORANGE_500=1, HWSPARKYBGC_GYRORANGE_1000=2, HWSPARKYBGC_GYRORANGE_2000=3 }  __attribute__((packed)) HwSparkyBGCGyroRangeOptions;
/* Max value of any option in topmost parent GyroRange of field GyroRange */
#define HWSPARKYBGC_GYRORANGE_GLOBAL_MAXOPTVAL 3
/* Max value of any option in field GyroRange */
#define HWSPARKYBGC_GYRORANGE_MAXOPTVAL 3
/* Ensure field GyroRange contains valid data */
static inline bool HwSparkyBGCGyroRangeIsValid( uint8_t CurrentGyroRange ) { return CurrentGyroRange < HWSPARKYBGC_GYRORANGE_MAXOPTVAL; }
// Field AccelRange information
/* Enumeration options for field AccelRange */
typedef enum { HWSPARKYBGC_ACCELRANGE_2G=0, HWSPARKYBGC_ACCELRANGE_4G=1, HWSPARKYBGC_ACCELRANGE_8G=2, HWSPARKYBGC_ACCELRANGE_16G=3 }  __attribute__((packed)) HwSparkyBGCAccelRangeOptions;
/* Max value of any option in topmost parent AccelRange of field AccelRange */
#define HWSPARKYBGC_ACCELRANGE_GLOBAL_MAXOPTVAL 3
/* Max value of any option in field AccelRange */
#define HWSPARKYBGC_ACCELRANGE_MAXOPTVAL 3
/* Ensure field AccelRange contains valid data */
static inline bool HwSparkyBGCAccelRangeIsValid( uint8_t CurrentAccelRange ) { return CurrentAccelRange < HWSPARKYBGC_ACCELRANGE_MAXOPTVAL; }
// Field MPU9150DLPF information
/* Enumeration options for field MPU9150DLPF */
typedef enum { HWSPARKYBGC_MPU9150DLPF_256=0, HWSPARKYBGC_MPU9150DLPF_188=1, HWSPARKYBGC_MPU9150DLPF_98=2, HWSPARKYBGC_MPU9150DLPF_42=3, HWSPARKYBGC_MPU9150DLPF_20=4, HWSPARKYBGC_MPU9150DLPF_10=5, HWSPARKYBGC_MPU9150DLPF_5=6 }  __attribute__((packed)) HwSparkyBGCMPU9150DLPFOptions;
/* Max value of any option in topmost parent MPU9150DLPF of field MPU9150DLPF */
#define HWSPARKYBGC_MPU9150DLPF_GLOBAL_MAXOPTVAL 6
/* Max value of any option in field MPU9150DLPF */
#define HWSPARKYBGC_MPU9150DLPF_MAXOPTVAL 6
/* Ensure field MPU9150DLPF contains valid data */
static inline bool HwSparkyBGCMPU9150DLPFIsValid( uint8_t CurrentMPU9150DLPF ) { return CurrentMPU9150DLPF < HWSPARKYBGC_MPU9150DLPF_MAXOPTVAL; }
// Field MPU9150Rate information
/* Enumeration options for field MPU9150Rate */
typedef enum { HWSPARKYBGC_MPU9150RATE_200=0, HWSPARKYBGC_MPU9150RATE_333=1, HWSPARKYBGC_MPU9150RATE_444=2, HWSPARKYBGC_MPU9150RATE_500=3, HWSPARKYBGC_MPU9150RATE_666=4, HWSPARKYBGC_MPU9150RATE_1000=5, HWSPARKYBGC_MPU9150RATE_2000=6, HWSPARKYBGC_MPU9150RATE_4000=7, HWSPARKYBGC_MPU9150RATE_8000=8 }  __attribute__((packed)) HwSparkyBGCMPU9150RateOptions;
/* Max value of any option in topmost parent MPU9150Rate of field MPU9150Rate */
#define HWSPARKYBGC_MPU9150RATE_GLOBAL_MAXOPTVAL 8
/* Max value of any option in field MPU9150Rate */
#define HWSPARKYBGC_MPU9150RATE_MAXOPTVAL 8
/* Ensure field MPU9150Rate contains valid data */
static inline bool HwSparkyBGCMPU9150RateIsValid( uint8_t CurrentMPU9150Rate ) { return CurrentMPU9150Rate < HWSPARKYBGC_MPU9150RATE_MAXOPTVAL; }


// set/Get functions
extern void HwSparkyBGCRcvrPortSet( uint8_t *NewRcvrPort );
extern void HwSparkyBGCRcvrPortGet( uint8_t *NewRcvrPort );
extern void HwSparkyBGCFlexiPortSet( uint8_t *NewFlexiPort );
extern void HwSparkyBGCFlexiPortGet( uint8_t *NewFlexiPort );
extern void HwSparkyBGCUSB_HIDPortSet( uint8_t *NewUSB_HIDPort );
extern void HwSparkyBGCUSB_HIDPortGet( uint8_t *NewUSB_HIDPort );
extern void HwSparkyBGCUSB_VCPPortSet( uint8_t *NewUSB_VCPPort );
extern void HwSparkyBGCUSB_VCPPortGet( uint8_t *NewUSB_VCPPort );
extern void HwSparkyBGCDSMxModeSet( uint8_t *NewDSMxMode );
extern void HwSparkyBGCDSMxModeGet( uint8_t *NewDSMxMode );
extern void HwSparkyBGCGyroRangeSet( uint8_t *NewGyroRange );
extern void HwSparkyBGCGyroRangeGet( uint8_t *NewGyroRange );
extern void HwSparkyBGCAccelRangeSet( uint8_t *NewAccelRange );
extern void HwSparkyBGCAccelRangeGet( uint8_t *NewAccelRange );
extern void HwSparkyBGCMPU9150DLPFSet( uint8_t *NewMPU9150DLPF );
extern void HwSparkyBGCMPU9150DLPFGet( uint8_t *NewMPU9150DLPF );
extern void HwSparkyBGCMPU9150RateSet( uint8_t *NewMPU9150Rate );
extern void HwSparkyBGCMPU9150RateGet( uint8_t *NewMPU9150Rate );


#endif // HWSPARKYBGC_H

/**
 * @}
 * @}
 */
