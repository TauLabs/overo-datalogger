/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup HwSparky HwSparky
 * @brief Selection of optional hardware configurations.
 *
 *
 * @file       hwsparky.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the HwSparky object. This file has been 
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

#ifndef HWSPARKY_H
#define HWSPARKY_H

#include "pios_queue.h"
#include "uavoversion.h"

#include <hwshared.h>


// Object constants
#define HWSPARKY_OBJID 0xCBFD19FC
#define HWSPARKY_ISSINGLEINST 1
#define HWSPARKY_ISSETTINGS 1
#define HWSPARKY_NUMBYTES 13

// Generic interface functions
int32_t HwSparkyInitialize();
UAVObjHandle HwSparkyHandle();
void HwSparkySetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    uint8_t RcvrPort;
    uint8_t FlexiPort;
    uint8_t MainPort;
    uint8_t OutPort;
    uint8_t USB_HIDPort;
    uint8_t USB_VCPPort;
    uint8_t DSMxMode;
    uint8_t GyroRange;
    uint8_t AccelRange;
    uint8_t MPU9150DLPF;
    uint8_t MPU9150Rate;
    uint8_t Magnetometer;
    uint8_t ExtMagOrientation;

} __attribute__((packed)) __attribute__((aligned(4))) HwSparkyData;

// Typesafe Object access functions
/**
 * @function HwSparkyGet(dataOut)
 * @brief Populate a HwSparkyData object
 * @param[out] dataOut 
 */
static inline int32_t HwSparkyGet(HwSparkyData *dataOut) { return UAVObjGetData(HwSparkyHandle(), dataOut); }

static inline int32_t HwSparkySet(const HwSparkyData *dataIn) { return UAVObjSetData(HwSparkyHandle(), dataIn); }

static inline int32_t HwSparkyInstGet(uint16_t instId, HwSparkyData *dataOut) { return UAVObjGetInstanceData(HwSparkyHandle(), instId, dataOut); }

static inline int32_t HwSparkyInstSet(uint16_t instId, const HwSparkyData *dataIn) { return UAVObjSetInstanceData(HwSparkyHandle(), instId, dataIn); }

static inline int32_t HwSparkyConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(HwSparkyHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t HwSparkyConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(HwSparkyHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t HwSparkyCreateInstance() { return UAVObjCreateInstance(HwSparkyHandle(), &HwSparkySetDefaults); }

static inline void HwSparkyRequestUpdate() { UAVObjRequestUpdate(HwSparkyHandle()); }

static inline void HwSparkyRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(HwSparkyHandle(), instId); }

static inline void HwSparkyUpdated() { UAVObjUpdated(HwSparkyHandle()); }

static inline void HwSparkyInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(HwSparkyHandle(), instId); }

static inline int32_t HwSparkyGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(HwSparkyHandle(), dataOut); }

static inline int32_t HwSparkySetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(HwSparkyHandle(), dataIn); }

static inline int8_t HwSparkyReadOnly() { return UAVObjReadOnly(HwSparkyHandle()); }

static inline uint16_t HwSparkyGetNumInstances(){ return UAVObjGetNumInstances(HwSparkyHandle()); }

static inline uint32_t HwSparkyGetNumBytes(){ return UAVObjGetNumBytes(HwSparkyHandle()); }

// Field information
// Field RcvrPort information
/* Enumeration options for field RcvrPort */
typedef enum { HWSPARKY_RCVRPORT_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWSPARKY_RCVRPORT_PPM=HWSHARED_PORTTYPES_PPM, HWSPARKY_RCVRPORT_SBUS=HWSHARED_PORTTYPES_SBUS, HWSPARKY_RCVRPORT_DSM=HWSHARED_PORTTYPES_DSM, HWSPARKY_RCVRPORT_HOTTSUMD=HWSHARED_PORTTYPES_HOTTSUMD, HWSPARKY_RCVRPORT_HOTTSUMH=HWSHARED_PORTTYPES_HOTTSUMH, HWSPARKY_RCVRPORT_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwSparkyRcvrPortOptions;
/* Max value of any option in topmost parent PortTypes of field RcvrPort */
#define HWSPARKY_RCVRPORT_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field RcvrPort */
#define HWSPARKY_RCVRPORT_MAXOPTVAL 19
/* Ensure field RcvrPort contains valid data */
static inline bool HwSparkyRcvrPortIsValid( uint8_t CurrentRcvrPort ) { return CurrentRcvrPort < HWSPARKY_RCVRPORT_MAXOPTVAL; }
// Field FlexiPort information
/* Enumeration options for field FlexiPort */
typedef enum { HWSPARKY_FLEXIPORT_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWSPARKY_FLEXIPORT_TELEMETRY=HWSHARED_PORTTYPES_TELEMETRY, HWSPARKY_FLEXIPORT_DEBUGCONSOLE=HWSHARED_PORTTYPES_DEBUGCONSOLE, HWSPARKY_FLEXIPORT_COMBRIDGE=HWSHARED_PORTTYPES_COMBRIDGE, HWSPARKY_FLEXIPORT_GPS=HWSHARED_PORTTYPES_GPS, HWSPARKY_FLEXIPORT_I2C=HWSHARED_PORTTYPES_I2C, HWSPARKY_FLEXIPORT_SBUS=HWSHARED_PORTTYPES_SBUS, HWSPARKY_FLEXIPORT_DSM=HWSHARED_PORTTYPES_DSM, HWSPARKY_FLEXIPORT_MAVLINKTX=HWSHARED_PORTTYPES_MAVLINKTX, HWSPARKY_FLEXIPORT_MAVLINKTX_GPS_RX=HWSHARED_PORTTYPES_MAVLINKTX_GPS_RX, HWSPARKY_FLEXIPORT_MSP=HWSHARED_PORTTYPES_MSP, HWSPARKY_FLEXIPORT_HOTTTELEMETRY=HWSHARED_PORTTYPES_HOTTTELEMETRY, HWSPARKY_FLEXIPORT_FRSKYSENSORHUB=HWSHARED_PORTTYPES_FRSKYSENSORHUB, HWSPARKY_FLEXIPORT_LIGHTTELEMETRYTX=HWSHARED_PORTTYPES_LIGHTTELEMETRYTX, HWSPARKY_FLEXIPORT_FRSKYSPORTTELEMETRY=HWSHARED_PORTTYPES_FRSKYSPORTTELEMETRY, HWSPARKY_FLEXIPORT_OPENLOG=HWSHARED_PORTTYPES_OPENLOG, HWSPARKY_FLEXIPORT_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwSparkyFlexiPortOptions;
/* Max value of any option in topmost parent PortTypes of field FlexiPort */
#define HWSPARKY_FLEXIPORT_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field FlexiPort */
#define HWSPARKY_FLEXIPORT_MAXOPTVAL 18
/* Ensure field FlexiPort contains valid data */
static inline bool HwSparkyFlexiPortIsValid( uint8_t CurrentFlexiPort ) { return CurrentFlexiPort < HWSPARKY_FLEXIPORT_MAXOPTVAL; }
// Field MainPort information
/* Enumeration options for field MainPort */
typedef enum { HWSPARKY_MAINPORT_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWSPARKY_MAINPORT_TELEMETRY=HWSHARED_PORTTYPES_TELEMETRY, HWSPARKY_MAINPORT_DEBUGCONSOLE=HWSHARED_PORTTYPES_DEBUGCONSOLE, HWSPARKY_MAINPORT_COMBRIDGE=HWSHARED_PORTTYPES_COMBRIDGE, HWSPARKY_MAINPORT_GPS=HWSHARED_PORTTYPES_GPS, HWSPARKY_MAINPORT_SBUS=HWSHARED_PORTTYPES_SBUS, HWSPARKY_MAINPORT_DSM=HWSHARED_PORTTYPES_DSM, HWSPARKY_MAINPORT_MAVLINKTX=HWSHARED_PORTTYPES_MAVLINKTX, HWSPARKY_MAINPORT_MAVLINKTX_GPS_RX=HWSHARED_PORTTYPES_MAVLINKTX_GPS_RX, HWSPARKY_MAINPORT_MSP=HWSHARED_PORTTYPES_MSP, HWSPARKY_MAINPORT_HOTTTELEMETRY=HWSHARED_PORTTYPES_HOTTTELEMETRY, HWSPARKY_MAINPORT_FRSKYSENSORHUB=HWSHARED_PORTTYPES_FRSKYSENSORHUB, HWSPARKY_MAINPORT_LIGHTTELEMETRYTX=HWSHARED_PORTTYPES_LIGHTTELEMETRYTX, HWSPARKY_MAINPORT_FRSKYSPORTTELEMETRY=HWSHARED_PORTTYPES_FRSKYSPORTTELEMETRY, HWSPARKY_MAINPORT_OPENLOG=HWSHARED_PORTTYPES_OPENLOG, HWSPARKY_MAINPORT_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwSparkyMainPortOptions;
/* Max value of any option in topmost parent PortTypes of field MainPort */
#define HWSPARKY_MAINPORT_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field MainPort */
#define HWSPARKY_MAINPORT_MAXOPTVAL 17
/* Ensure field MainPort contains valid data */
static inline bool HwSparkyMainPortIsValid( uint8_t CurrentMainPort ) { return CurrentMainPort < HWSPARKY_MAINPORT_MAXOPTVAL; }
// Field OutPort information
/* Enumeration options for field OutPort */
typedef enum { HWSPARKY_OUTPORT_PWM10=0, HWSPARKY_OUTPORT_PWM73ADC=1, HWSPARKY_OUTPORT_PWM82ADC=2, HWSPARKY_OUTPORT_PWM9PWM_IN=3, HWSPARKY_OUTPORT_PWM7PWM_IN2ADC=4 }  __attribute__((packed)) HwSparkyOutPortOptions;
/* Max value of any option in topmost parent OutPort of field OutPort */
#define HWSPARKY_OUTPORT_GLOBAL_MAXOPTVAL 4
/* Max value of any option in field OutPort */
#define HWSPARKY_OUTPORT_MAXOPTVAL 4
/* Ensure field OutPort contains valid data */
static inline bool HwSparkyOutPortIsValid( uint8_t CurrentOutPort ) { return CurrentOutPort < HWSPARKY_OUTPORT_MAXOPTVAL; }
// Field USB_HIDPort information
/* Enumeration options for field USB_HIDPort */
typedef enum { HWSPARKY_USB_HIDPORT_USBTELEMETRY=HWSHARED_USB_HIDPORT_USBTELEMETRY, HWSPARKY_USB_HIDPORT_DISABLED=HWSHARED_USB_HIDPORT_DISABLED, HWSPARKY_USB_HIDPORT_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwSparkyUSB_HIDPortOptions;
/* Max value of any option in topmost parent USB_HIDPort of field USB_HIDPort */
#define HWSPARKY_USB_HIDPORT_GLOBAL_MAXOPTVAL 1
/* Max value of any option in field USB_HIDPort */
#define HWSPARKY_USB_HIDPORT_MAXOPTVAL 1
/* Ensure field USB_HIDPort contains valid data */
static inline bool HwSparkyUSB_HIDPortIsValid( uint8_t CurrentUSB_HIDPort ) { return CurrentUSB_HIDPort < HWSPARKY_USB_HIDPORT_MAXOPTVAL; }
// Field USB_VCPPort information
/* Enumeration options for field USB_VCPPort */
typedef enum { HWSPARKY_USB_VCPPORT_USBTELEMETRY=HWSHARED_USB_VCPPORT_USBTELEMETRY, HWSPARKY_USB_VCPPORT_COMBRIDGE=HWSHARED_USB_VCPPORT_COMBRIDGE, HWSPARKY_USB_VCPPORT_DEBUGCONSOLE=HWSHARED_USB_VCPPORT_DEBUGCONSOLE, HWSPARKY_USB_VCPPORT_PICOC=HWSHARED_USB_VCPPORT_PICOC, HWSPARKY_USB_VCPPORT_DISABLED=HWSHARED_USB_VCPPORT_DISABLED, HWSPARKY_USB_VCPPORT_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwSparkyUSB_VCPPortOptions;
/* Max value of any option in topmost parent USB_VCPPort of field USB_VCPPort */
#define HWSPARKY_USB_VCPPORT_GLOBAL_MAXOPTVAL 4
/* Max value of any option in field USB_VCPPort */
#define HWSPARKY_USB_VCPPORT_MAXOPTVAL 4
/* Ensure field USB_VCPPort contains valid data */
static inline bool HwSparkyUSB_VCPPortIsValid( uint8_t CurrentUSB_VCPPort ) { return CurrentUSB_VCPPort < HWSPARKY_USB_VCPPORT_MAXOPTVAL; }
// Field DSMxMode information
/* Enumeration options for field DSMxMode */
typedef enum { HWSPARKY_DSMXMODE_AUTODETECT=HWSHARED_DSMXMODE_AUTODETECT, HWSPARKY_DSMXMODE_FORCE10BIT=HWSHARED_DSMXMODE_FORCE10BIT, HWSPARKY_DSMXMODE_FORCE11BIT=HWSHARED_DSMXMODE_FORCE11BIT, HWSPARKY_DSMXMODE_BIND3PULSES=HWSHARED_DSMXMODE_BIND3PULSES, HWSPARKY_DSMXMODE_BIND4PULSES=HWSHARED_DSMXMODE_BIND4PULSES, HWSPARKY_DSMXMODE_BIND5PULSES=HWSHARED_DSMXMODE_BIND5PULSES, HWSPARKY_DSMXMODE_BIND6PULSES=HWSHARED_DSMXMODE_BIND6PULSES, HWSPARKY_DSMXMODE_BIND7PULSES=HWSHARED_DSMXMODE_BIND7PULSES, HWSPARKY_DSMXMODE_BIND8PULSES=HWSHARED_DSMXMODE_BIND8PULSES, HWSPARKY_DSMXMODE_BIND9PULSES=HWSHARED_DSMXMODE_BIND9PULSES, HWSPARKY_DSMXMODE_BIND10PULSES=HWSHARED_DSMXMODE_BIND10PULSES, HWSPARKY_DSMXMODE_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwSparkyDSMxModeOptions;
/* Max value of any option in topmost parent DSMxMode of field DSMxMode */
#define HWSPARKY_DSMXMODE_GLOBAL_MAXOPTVAL 10
/* Max value of any option in field DSMxMode */
#define HWSPARKY_DSMXMODE_MAXOPTVAL 10
/* Ensure field DSMxMode contains valid data */
static inline bool HwSparkyDSMxModeIsValid( uint8_t CurrentDSMxMode ) { return CurrentDSMxMode < HWSPARKY_DSMXMODE_MAXOPTVAL; }
// Field GyroRange information
/* Enumeration options for field GyroRange */
typedef enum { HWSPARKY_GYRORANGE_250=0, HWSPARKY_GYRORANGE_500=1, HWSPARKY_GYRORANGE_1000=2, HWSPARKY_GYRORANGE_2000=3 }  __attribute__((packed)) HwSparkyGyroRangeOptions;
/* Max value of any option in topmost parent GyroRange of field GyroRange */
#define HWSPARKY_GYRORANGE_GLOBAL_MAXOPTVAL 3
/* Max value of any option in field GyroRange */
#define HWSPARKY_GYRORANGE_MAXOPTVAL 3
/* Ensure field GyroRange contains valid data */
static inline bool HwSparkyGyroRangeIsValid( uint8_t CurrentGyroRange ) { return CurrentGyroRange < HWSPARKY_GYRORANGE_MAXOPTVAL; }
// Field AccelRange information
/* Enumeration options for field AccelRange */
typedef enum { HWSPARKY_ACCELRANGE_2G=0, HWSPARKY_ACCELRANGE_4G=1, HWSPARKY_ACCELRANGE_8G=2, HWSPARKY_ACCELRANGE_16G=3 }  __attribute__((packed)) HwSparkyAccelRangeOptions;
/* Max value of any option in topmost parent AccelRange of field AccelRange */
#define HWSPARKY_ACCELRANGE_GLOBAL_MAXOPTVAL 3
/* Max value of any option in field AccelRange */
#define HWSPARKY_ACCELRANGE_MAXOPTVAL 3
/* Ensure field AccelRange contains valid data */
static inline bool HwSparkyAccelRangeIsValid( uint8_t CurrentAccelRange ) { return CurrentAccelRange < HWSPARKY_ACCELRANGE_MAXOPTVAL; }
// Field MPU9150DLPF information
/* Enumeration options for field MPU9150DLPF */
typedef enum { HWSPARKY_MPU9150DLPF_256=0, HWSPARKY_MPU9150DLPF_188=1, HWSPARKY_MPU9150DLPF_98=2, HWSPARKY_MPU9150DLPF_42=3, HWSPARKY_MPU9150DLPF_20=4, HWSPARKY_MPU9150DLPF_10=5, HWSPARKY_MPU9150DLPF_5=6 }  __attribute__((packed)) HwSparkyMPU9150DLPFOptions;
/* Max value of any option in topmost parent MPU9150DLPF of field MPU9150DLPF */
#define HWSPARKY_MPU9150DLPF_GLOBAL_MAXOPTVAL 6
/* Max value of any option in field MPU9150DLPF */
#define HWSPARKY_MPU9150DLPF_MAXOPTVAL 6
/* Ensure field MPU9150DLPF contains valid data */
static inline bool HwSparkyMPU9150DLPFIsValid( uint8_t CurrentMPU9150DLPF ) { return CurrentMPU9150DLPF < HWSPARKY_MPU9150DLPF_MAXOPTVAL; }
// Field MPU9150Rate information
/* Enumeration options for field MPU9150Rate */
typedef enum { HWSPARKY_MPU9150RATE_200=0, HWSPARKY_MPU9150RATE_333=1, HWSPARKY_MPU9150RATE_444=2, HWSPARKY_MPU9150RATE_500=3, HWSPARKY_MPU9150RATE_666=4, HWSPARKY_MPU9150RATE_1000=5, HWSPARKY_MPU9150RATE_2000=6, HWSPARKY_MPU9150RATE_4000=7, HWSPARKY_MPU9150RATE_8000=8 }  __attribute__((packed)) HwSparkyMPU9150RateOptions;
/* Max value of any option in topmost parent MPU9150Rate of field MPU9150Rate */
#define HWSPARKY_MPU9150RATE_GLOBAL_MAXOPTVAL 8
/* Max value of any option in field MPU9150Rate */
#define HWSPARKY_MPU9150RATE_MAXOPTVAL 8
/* Ensure field MPU9150Rate contains valid data */
static inline bool HwSparkyMPU9150RateIsValid( uint8_t CurrentMPU9150Rate ) { return CurrentMPU9150Rate < HWSPARKY_MPU9150RATE_MAXOPTVAL; }
// Field Magnetometer information
/* Enumeration options for field Magnetometer */
typedef enum { HWSPARKY_MAGNETOMETER_INTERNAL=0, HWSPARKY_MAGNETOMETER_EXTERNALI2CFLEXIPORT=1 }  __attribute__((packed)) HwSparkyMagnetometerOptions;
/* Max value of any option in topmost parent Magnetometer of field Magnetometer */
#define HWSPARKY_MAGNETOMETER_GLOBAL_MAXOPTVAL 1
/* Max value of any option in field Magnetometer */
#define HWSPARKY_MAGNETOMETER_MAXOPTVAL 1
/* Ensure field Magnetometer contains valid data */
static inline bool HwSparkyMagnetometerIsValid( uint8_t CurrentMagnetometer ) { return CurrentMagnetometer < HWSPARKY_MAGNETOMETER_MAXOPTVAL; }
// Field ExtMagOrientation information
/* Enumeration options for field ExtMagOrientation */
typedef enum { HWSPARKY_EXTMAGORIENTATION_TOP0DEGCW=HWSHARED_MAGORIENTATION_TOP0DEGCW, HWSPARKY_EXTMAGORIENTATION_TOP90DEGCW=HWSHARED_MAGORIENTATION_TOP90DEGCW, HWSPARKY_EXTMAGORIENTATION_TOP180DEGCW=HWSHARED_MAGORIENTATION_TOP180DEGCW, HWSPARKY_EXTMAGORIENTATION_TOP270DEGCW=HWSHARED_MAGORIENTATION_TOP270DEGCW, HWSPARKY_EXTMAGORIENTATION_BOTTOM0DEGCW=HWSHARED_MAGORIENTATION_BOTTOM0DEGCW, HWSPARKY_EXTMAGORIENTATION_BOTTOM90DEGCW=HWSHARED_MAGORIENTATION_BOTTOM90DEGCW, HWSPARKY_EXTMAGORIENTATION_BOTTOM180DEGCW=HWSHARED_MAGORIENTATION_BOTTOM180DEGCW, HWSPARKY_EXTMAGORIENTATION_BOTTOM270DEGCW=HWSHARED_MAGORIENTATION_BOTTOM270DEGCW, HWSPARKY_EXTMAGORIENTATION_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwSparkyExtMagOrientationOptions;
/* Max value of any option in topmost parent MagOrientation of field ExtMagOrientation */
#define HWSPARKY_EXTMAGORIENTATION_GLOBAL_MAXOPTVAL 7
/* Max value of any option in field ExtMagOrientation */
#define HWSPARKY_EXTMAGORIENTATION_MAXOPTVAL 7
/* Ensure field ExtMagOrientation contains valid data */
static inline bool HwSparkyExtMagOrientationIsValid( uint8_t CurrentExtMagOrientation ) { return CurrentExtMagOrientation < HWSPARKY_EXTMAGORIENTATION_MAXOPTVAL; }


// set/Get functions
extern void HwSparkyRcvrPortSet( uint8_t *NewRcvrPort );
extern void HwSparkyRcvrPortGet( uint8_t *NewRcvrPort );
extern void HwSparkyFlexiPortSet( uint8_t *NewFlexiPort );
extern void HwSparkyFlexiPortGet( uint8_t *NewFlexiPort );
extern void HwSparkyMainPortSet( uint8_t *NewMainPort );
extern void HwSparkyMainPortGet( uint8_t *NewMainPort );
extern void HwSparkyOutPortSet( uint8_t *NewOutPort );
extern void HwSparkyOutPortGet( uint8_t *NewOutPort );
extern void HwSparkyUSB_HIDPortSet( uint8_t *NewUSB_HIDPort );
extern void HwSparkyUSB_HIDPortGet( uint8_t *NewUSB_HIDPort );
extern void HwSparkyUSB_VCPPortSet( uint8_t *NewUSB_VCPPort );
extern void HwSparkyUSB_VCPPortGet( uint8_t *NewUSB_VCPPort );
extern void HwSparkyDSMxModeSet( uint8_t *NewDSMxMode );
extern void HwSparkyDSMxModeGet( uint8_t *NewDSMxMode );
extern void HwSparkyGyroRangeSet( uint8_t *NewGyroRange );
extern void HwSparkyGyroRangeGet( uint8_t *NewGyroRange );
extern void HwSparkyAccelRangeSet( uint8_t *NewAccelRange );
extern void HwSparkyAccelRangeGet( uint8_t *NewAccelRange );
extern void HwSparkyMPU9150DLPFSet( uint8_t *NewMPU9150DLPF );
extern void HwSparkyMPU9150DLPFGet( uint8_t *NewMPU9150DLPF );
extern void HwSparkyMPU9150RateSet( uint8_t *NewMPU9150Rate );
extern void HwSparkyMPU9150RateGet( uint8_t *NewMPU9150Rate );
extern void HwSparkyMagnetometerSet( uint8_t *NewMagnetometer );
extern void HwSparkyMagnetometerGet( uint8_t *NewMagnetometer );
extern void HwSparkyExtMagOrientationSet( uint8_t *NewExtMagOrientation );
extern void HwSparkyExtMagOrientationGet( uint8_t *NewExtMagOrientation );


#endif // HWSPARKY_H

/**
 * @}
 * @}
 */