/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup HwColibri HwColibri
 * @brief Selection of optional hardware configurations.
 *
 *
 * @file       hwcolibri.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the HwColibri object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: hwcolibri.xml. 
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

#ifndef HWCOLIBRI_H
#define HWCOLIBRI_H

#include "pios_queue.h"
#include "uavoversion.h"

#include <hwshared.h>


// Object constants
#define HWCOLIBRI_OBJID 0x5D51A6D4
#define HWCOLIBRI_ISSINGLEINST 1
#define HWCOLIBRI_ISSETTINGS 1
#define HWCOLIBRI_NUMBYTES 15

// Generic interface functions
int32_t HwColibriInitialize();
UAVObjHandle HwColibriHandle();
void HwColibriSetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    uint8_t Frame;
    uint8_t RcvrPort;
    uint8_t Uart1;
    uint8_t Uart2;
    uint8_t Uart3;
    uint8_t Uart4;
    uint8_t USB_HIDPort;
    uint8_t USB_VCPPort;
    uint8_t DSMxMode;
    uint8_t GyroRange;
    uint8_t AccelRange;
    uint8_t MPU6000Rate;
    uint8_t MPU6000DLPF;
    uint8_t Magnetometer;
    uint8_t ExtMagOrientation;

} __attribute__((packed)) __attribute__((aligned(4))) HwColibriData;

// Typesafe Object access functions
/**
 * @function HwColibriGet(dataOut)
 * @brief Populate a HwColibriData object
 * @param[out] dataOut 
 */
static inline int32_t HwColibriGet(HwColibriData *dataOut) { return UAVObjGetData(HwColibriHandle(), dataOut); }

static inline int32_t HwColibriSet(const HwColibriData *dataIn) { return UAVObjSetData(HwColibriHandle(), dataIn); }

static inline int32_t HwColibriInstGet(uint16_t instId, HwColibriData *dataOut) { return UAVObjGetInstanceData(HwColibriHandle(), instId, dataOut); }

static inline int32_t HwColibriInstSet(uint16_t instId, const HwColibriData *dataIn) { return UAVObjSetInstanceData(HwColibriHandle(), instId, dataIn); }

static inline int32_t HwColibriConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(HwColibriHandle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t HwColibriConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(HwColibriHandle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t HwColibriCreateInstance() { return UAVObjCreateInstance(HwColibriHandle(), &HwColibriSetDefaults); }

static inline void HwColibriRequestUpdate() { UAVObjRequestUpdate(HwColibriHandle()); }

static inline void HwColibriRequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(HwColibriHandle(), instId); }

static inline void HwColibriUpdated() { UAVObjUpdated(HwColibriHandle()); }

static inline void HwColibriInstUpdated(uint16_t instId) { UAVObjInstanceUpdated(HwColibriHandle(), instId); }

static inline int32_t HwColibriGetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(HwColibriHandle(), dataOut); }

static inline int32_t HwColibriSetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(HwColibriHandle(), dataIn); }

static inline int8_t HwColibriReadOnly() { return UAVObjReadOnly(HwColibriHandle()); }

static inline uint16_t HwColibriGetNumInstances(){ return UAVObjGetNumInstances(HwColibriHandle()); }

static inline uint32_t HwColibriGetNumBytes(){ return UAVObjGetNumBytes(HwColibriHandle()); }

// Field information
// Field Frame information
/* Enumeration options for field Frame */
typedef enum { HWCOLIBRI_FRAME_GEMINI=0 }  __attribute__((packed)) HwColibriFrameOptions;
/* Max value of any option in topmost parent Frame of field Frame */
#define HWCOLIBRI_FRAME_GLOBAL_MAXOPTVAL 0
/* Max value of any option in field Frame */
#define HWCOLIBRI_FRAME_MAXOPTVAL 0
/* Ensure field Frame contains valid data */
static inline bool HwColibriFrameIsValid( uint8_t CurrentFrame ) { return CurrentFrame < HWCOLIBRI_FRAME_MAXOPTVAL; }
// Field RcvrPort information
/* Enumeration options for field RcvrPort */
typedef enum { HWCOLIBRI_RCVRPORT_DISABLED=0, HWCOLIBRI_RCVRPORT_OUTPUTS=1, HWCOLIBRI_RCVRPORT_OUTPUTSADC=2, HWCOLIBRI_RCVRPORT_PPM=3, HWCOLIBRI_RCVRPORT_PPMADC=4, HWCOLIBRI_RCVRPORT_PPMOUTPUTS=5, HWCOLIBRI_RCVRPORT_PPMOUTPUTSADC=6, HWCOLIBRI_RCVRPORT_PPMPWM=7, HWCOLIBRI_RCVRPORT_PPMPWMADC=8, HWCOLIBRI_RCVRPORT_PWM=9, HWCOLIBRI_RCVRPORT_PWMADC=10 }  __attribute__((packed)) HwColibriRcvrPortOptions;
/* Max value of any option in topmost parent RcvrPort of field RcvrPort */
#define HWCOLIBRI_RCVRPORT_GLOBAL_MAXOPTVAL 10
/* Max value of any option in field RcvrPort */
#define HWCOLIBRI_RCVRPORT_MAXOPTVAL 10
/* Ensure field RcvrPort contains valid data */
static inline bool HwColibriRcvrPortIsValid( uint8_t CurrentRcvrPort ) { return CurrentRcvrPort < HWCOLIBRI_RCVRPORT_MAXOPTVAL; }
// Field Uart1 information
/* Enumeration options for field Uart1 */
typedef enum { HWCOLIBRI_UART1_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWCOLIBRI_UART1_DEBUGCONSOLE=HWSHARED_PORTTYPES_DEBUGCONSOLE, HWCOLIBRI_UART1_COMBRIDGE=HWSHARED_PORTTYPES_COMBRIDGE, HWCOLIBRI_UART1_DSM=HWSHARED_PORTTYPES_DSM, HWCOLIBRI_UART1_FRSKYSENSORHUB=HWSHARED_PORTTYPES_FRSKYSENSORHUB, HWCOLIBRI_UART1_GPS=HWSHARED_PORTTYPES_GPS, HWCOLIBRI_UART1_HOTTSUMD=HWSHARED_PORTTYPES_HOTTSUMD, HWCOLIBRI_UART1_HOTTSUMH=HWSHARED_PORTTYPES_HOTTSUMH, HWCOLIBRI_UART1_HOTTTELEMETRY=HWSHARED_PORTTYPES_HOTTTELEMETRY, HWCOLIBRI_UART1_I2C=HWSHARED_PORTTYPES_I2C, HWCOLIBRI_UART1_LIGHTTELEMETRYTX=HWSHARED_PORTTYPES_LIGHTTELEMETRYTX, HWCOLIBRI_UART1_MAVLINKTX=HWSHARED_PORTTYPES_MAVLINKTX, HWCOLIBRI_UART1_MAVLINKTX_GPS_RX=HWSHARED_PORTTYPES_MAVLINKTX_GPS_RX, HWCOLIBRI_UART1_PICOC=HWSHARED_PORTTYPES_PICOC, HWCOLIBRI_UART1_TELEMETRY=HWSHARED_PORTTYPES_TELEMETRY, HWCOLIBRI_UART1_OPENLOG=HWSHARED_PORTTYPES_OPENLOG, HWCOLIBRI_UART1_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwColibriUart1Options;
/* Max value of any option in topmost parent PortTypes of field Uart1 */
#define HWCOLIBRI_UART1_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field Uart1 */
#define HWCOLIBRI_UART1_MAXOPTVAL 18
/* Ensure field Uart1 contains valid data */
static inline bool HwColibriUart1IsValid( uint8_t CurrentUart1 ) { return CurrentUart1 < HWCOLIBRI_UART1_MAXOPTVAL; }
// Field Uart2 information
/* Enumeration options for field Uart2 */
typedef enum { HWCOLIBRI_UART2_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWCOLIBRI_UART2_COMBRIDGE=HWSHARED_PORTTYPES_COMBRIDGE, HWCOLIBRI_UART2_DEBUGCONSOLE=HWSHARED_PORTTYPES_DEBUGCONSOLE, HWCOLIBRI_UART2_DSM=HWSHARED_PORTTYPES_DSM, HWCOLIBRI_UART2_FRSKYSENSORHUB=HWSHARED_PORTTYPES_FRSKYSENSORHUB, HWCOLIBRI_UART2_GPS=HWSHARED_PORTTYPES_GPS, HWCOLIBRI_UART2_HOTTSUMD=HWSHARED_PORTTYPES_HOTTSUMD, HWCOLIBRI_UART2_HOTTSUMH=HWSHARED_PORTTYPES_HOTTSUMH, HWCOLIBRI_UART2_HOTTTELEMETRY=HWSHARED_PORTTYPES_HOTTTELEMETRY, HWCOLIBRI_UART2_LIGHTTELEMETRYTX=HWSHARED_PORTTYPES_LIGHTTELEMETRYTX, HWCOLIBRI_UART2_MAVLINKTX=HWSHARED_PORTTYPES_MAVLINKTX, HWCOLIBRI_UART2_MAVLINKTX_GPS_RX=HWSHARED_PORTTYPES_MAVLINKTX_GPS_RX, HWCOLIBRI_UART2_PICOC=HWSHARED_PORTTYPES_PICOC, HWCOLIBRI_UART2_SBUS=HWSHARED_PORTTYPES_SBUS, HWCOLIBRI_UART2_TELEMETRY=HWSHARED_PORTTYPES_TELEMETRY, HWCOLIBRI_UART2_OPENLOG=HWSHARED_PORTTYPES_OPENLOG, HWCOLIBRI_UART2_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwColibriUart2Options;
/* Max value of any option in topmost parent PortTypes of field Uart2 */
#define HWCOLIBRI_UART2_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field Uart2 */
#define HWCOLIBRI_UART2_MAXOPTVAL 17
/* Ensure field Uart2 contains valid data */
static inline bool HwColibriUart2IsValid( uint8_t CurrentUart2 ) { return CurrentUart2 < HWCOLIBRI_UART2_MAXOPTVAL; }
// Field Uart3 information
/* Enumeration options for field Uart3 */
typedef enum { HWCOLIBRI_UART3_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWCOLIBRI_UART3_COMBRIDGE=HWSHARED_PORTTYPES_COMBRIDGE, HWCOLIBRI_UART3_DEBUGCONSOLE=HWSHARED_PORTTYPES_DEBUGCONSOLE, HWCOLIBRI_UART3_DSM=HWSHARED_PORTTYPES_DSM, HWCOLIBRI_UART3_FRSKYSENSORHUB=HWSHARED_PORTTYPES_FRSKYSENSORHUB, HWCOLIBRI_UART3_GPS=HWSHARED_PORTTYPES_GPS, HWCOLIBRI_UART3_HOTTSUMD=HWSHARED_PORTTYPES_HOTTSUMD, HWCOLIBRI_UART3_HOTTSUMH=HWSHARED_PORTTYPES_HOTTSUMH, HWCOLIBRI_UART3_HOTTTELEMETRY=HWSHARED_PORTTYPES_HOTTTELEMETRY, HWCOLIBRI_UART3_I2C=HWSHARED_PORTTYPES_I2C, HWCOLIBRI_UART3_LIGHTTELEMETRYTX=HWSHARED_PORTTYPES_LIGHTTELEMETRYTX, HWCOLIBRI_UART3_MAVLINKTX=HWSHARED_PORTTYPES_MAVLINKTX, HWCOLIBRI_UART3_MAVLINKTX_GPS_RX=HWSHARED_PORTTYPES_MAVLINKTX_GPS_RX, HWCOLIBRI_UART3_PICOC=HWSHARED_PORTTYPES_PICOC, HWCOLIBRI_UART3_TELEMETRY=HWSHARED_PORTTYPES_TELEMETRY, HWCOLIBRI_UART3_OPENLOG=HWSHARED_PORTTYPES_OPENLOG, HWCOLIBRI_UART3_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwColibriUart3Options;
/* Max value of any option in topmost parent PortTypes of field Uart3 */
#define HWCOLIBRI_UART3_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field Uart3 */
#define HWCOLIBRI_UART3_MAXOPTVAL 18
/* Ensure field Uart3 contains valid data */
static inline bool HwColibriUart3IsValid( uint8_t CurrentUart3 ) { return CurrentUart3 < HWCOLIBRI_UART3_MAXOPTVAL; }
// Field Uart4 information
/* Enumeration options for field Uart4 */
typedef enum { HWCOLIBRI_UART4_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWCOLIBRI_UART4_COMBRIDGE=HWSHARED_PORTTYPES_COMBRIDGE, HWCOLIBRI_UART4_DEBUGCONSOLE=HWSHARED_PORTTYPES_DEBUGCONSOLE, HWCOLIBRI_UART4_DSM=HWSHARED_PORTTYPES_DSM, HWCOLIBRI_UART4_FRSKYSENSORHUB=HWSHARED_PORTTYPES_FRSKYSENSORHUB, HWCOLIBRI_UART4_GPS=HWSHARED_PORTTYPES_GPS, HWCOLIBRI_UART4_LIGHTTELEMETRYTX=HWSHARED_PORTTYPES_LIGHTTELEMETRYTX, HWCOLIBRI_UART4_HOTTSUMD=HWSHARED_PORTTYPES_HOTTSUMD, HWCOLIBRI_UART4_HOTTSUMH=HWSHARED_PORTTYPES_HOTTSUMH, HWCOLIBRI_UART4_HOTTTELEMETRY=HWSHARED_PORTTYPES_HOTTTELEMETRY, HWCOLIBRI_UART4_MAVLINKTX=HWSHARED_PORTTYPES_MAVLINKTX, HWCOLIBRI_UART4_MAVLINKTX_GPS_RX=HWSHARED_PORTTYPES_MAVLINKTX_GPS_RX, HWCOLIBRI_UART4_PICOC=HWSHARED_PORTTYPES_PICOC, HWCOLIBRI_UART4_TELEMETRY=HWSHARED_PORTTYPES_TELEMETRY, HWCOLIBRI_UART4_OPENLOG=HWSHARED_PORTTYPES_OPENLOG, HWCOLIBRI_UART4_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwColibriUart4Options;
/* Max value of any option in topmost parent PortTypes of field Uart4 */
#define HWCOLIBRI_UART4_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field Uart4 */
#define HWCOLIBRI_UART4_MAXOPTVAL 17
/* Ensure field Uart4 contains valid data */
static inline bool HwColibriUart4IsValid( uint8_t CurrentUart4 ) { return CurrentUart4 < HWCOLIBRI_UART4_MAXOPTVAL; }
// Field USB_HIDPort information
/* Enumeration options for field USB_HIDPort */
typedef enum { HWCOLIBRI_USB_HIDPORT_USBTELEMETRY=HWSHARED_USB_HIDPORT_USBTELEMETRY, HWCOLIBRI_USB_HIDPORT_DISABLED=HWSHARED_USB_HIDPORT_DISABLED, HWCOLIBRI_USB_HIDPORT_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwColibriUSB_HIDPortOptions;
/* Max value of any option in topmost parent USB_HIDPort of field USB_HIDPort */
#define HWCOLIBRI_USB_HIDPORT_GLOBAL_MAXOPTVAL 1
/* Max value of any option in field USB_HIDPort */
#define HWCOLIBRI_USB_HIDPORT_MAXOPTVAL 1
/* Ensure field USB_HIDPort contains valid data */
static inline bool HwColibriUSB_HIDPortIsValid( uint8_t CurrentUSB_HIDPort ) { return CurrentUSB_HIDPort < HWCOLIBRI_USB_HIDPORT_MAXOPTVAL; }
// Field USB_VCPPort information
/* Enumeration options for field USB_VCPPort */
typedef enum { HWCOLIBRI_USB_VCPPORT_USBTELEMETRY=HWSHARED_USB_VCPPORT_USBTELEMETRY, HWCOLIBRI_USB_VCPPORT_COMBRIDGE=HWSHARED_USB_VCPPORT_COMBRIDGE, HWCOLIBRI_USB_VCPPORT_DEBUGCONSOLE=HWSHARED_USB_VCPPORT_DEBUGCONSOLE, HWCOLIBRI_USB_VCPPORT_PICOC=HWSHARED_USB_VCPPORT_PICOC, HWCOLIBRI_USB_VCPPORT_DISABLED=HWSHARED_USB_VCPPORT_DISABLED, HWCOLIBRI_USB_VCPPORT_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwColibriUSB_VCPPortOptions;
/* Max value of any option in topmost parent USB_VCPPort of field USB_VCPPort */
#define HWCOLIBRI_USB_VCPPORT_GLOBAL_MAXOPTVAL 4
/* Max value of any option in field USB_VCPPort */
#define HWCOLIBRI_USB_VCPPORT_MAXOPTVAL 4
/* Ensure field USB_VCPPort contains valid data */
static inline bool HwColibriUSB_VCPPortIsValid( uint8_t CurrentUSB_VCPPort ) { return CurrentUSB_VCPPort < HWCOLIBRI_USB_VCPPORT_MAXOPTVAL; }
// Field DSMxMode information
/* Enumeration options for field DSMxMode */
typedef enum { HWCOLIBRI_DSMXMODE_AUTODETECT=HWSHARED_DSMXMODE_AUTODETECT, HWCOLIBRI_DSMXMODE_FORCE10BIT=HWSHARED_DSMXMODE_FORCE10BIT, HWCOLIBRI_DSMXMODE_FORCE11BIT=HWSHARED_DSMXMODE_FORCE11BIT, HWCOLIBRI_DSMXMODE_BIND3PULSES=HWSHARED_DSMXMODE_BIND3PULSES, HWCOLIBRI_DSMXMODE_BIND4PULSES=HWSHARED_DSMXMODE_BIND4PULSES, HWCOLIBRI_DSMXMODE_BIND5PULSES=HWSHARED_DSMXMODE_BIND5PULSES, HWCOLIBRI_DSMXMODE_BIND6PULSES=HWSHARED_DSMXMODE_BIND6PULSES, HWCOLIBRI_DSMXMODE_BIND7PULSES=HWSHARED_DSMXMODE_BIND7PULSES, HWCOLIBRI_DSMXMODE_BIND8PULSES=HWSHARED_DSMXMODE_BIND8PULSES, HWCOLIBRI_DSMXMODE_BIND9PULSES=HWSHARED_DSMXMODE_BIND9PULSES, HWCOLIBRI_DSMXMODE_BIND10PULSES=HWSHARED_DSMXMODE_BIND10PULSES, HWCOLIBRI_DSMXMODE_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwColibriDSMxModeOptions;
/* Max value of any option in topmost parent DSMxMode of field DSMxMode */
#define HWCOLIBRI_DSMXMODE_GLOBAL_MAXOPTVAL 10
/* Max value of any option in field DSMxMode */
#define HWCOLIBRI_DSMXMODE_MAXOPTVAL 10
/* Ensure field DSMxMode contains valid data */
static inline bool HwColibriDSMxModeIsValid( uint8_t CurrentDSMxMode ) { return CurrentDSMxMode < HWCOLIBRI_DSMXMODE_MAXOPTVAL; }
// Field GyroRange information
/* Enumeration options for field GyroRange */
typedef enum { HWCOLIBRI_GYRORANGE_250=0, HWCOLIBRI_GYRORANGE_500=1, HWCOLIBRI_GYRORANGE_1000=2, HWCOLIBRI_GYRORANGE_2000=3 }  __attribute__((packed)) HwColibriGyroRangeOptions;
/* Max value of any option in topmost parent GyroRange of field GyroRange */
#define HWCOLIBRI_GYRORANGE_GLOBAL_MAXOPTVAL 3
/* Max value of any option in field GyroRange */
#define HWCOLIBRI_GYRORANGE_MAXOPTVAL 3
/* Ensure field GyroRange contains valid data */
static inline bool HwColibriGyroRangeIsValid( uint8_t CurrentGyroRange ) { return CurrentGyroRange < HWCOLIBRI_GYRORANGE_MAXOPTVAL; }
// Field AccelRange information
/* Enumeration options for field AccelRange */
typedef enum { HWCOLIBRI_ACCELRANGE_2G=0, HWCOLIBRI_ACCELRANGE_4G=1, HWCOLIBRI_ACCELRANGE_8G=2, HWCOLIBRI_ACCELRANGE_16G=3 }  __attribute__((packed)) HwColibriAccelRangeOptions;
/* Max value of any option in topmost parent AccelRange of field AccelRange */
#define HWCOLIBRI_ACCELRANGE_GLOBAL_MAXOPTVAL 3
/* Max value of any option in field AccelRange */
#define HWCOLIBRI_ACCELRANGE_MAXOPTVAL 3
/* Ensure field AccelRange contains valid data */
static inline bool HwColibriAccelRangeIsValid( uint8_t CurrentAccelRange ) { return CurrentAccelRange < HWCOLIBRI_ACCELRANGE_MAXOPTVAL; }
// Field MPU6000Rate information
/* Enumeration options for field MPU6000Rate */
typedef enum { HWCOLIBRI_MPU6000RATE_200=0, HWCOLIBRI_MPU6000RATE_333=1, HWCOLIBRI_MPU6000RATE_500=2, HWCOLIBRI_MPU6000RATE_666=3, HWCOLIBRI_MPU6000RATE_1000=4, HWCOLIBRI_MPU6000RATE_2000=5, HWCOLIBRI_MPU6000RATE_4000=6, HWCOLIBRI_MPU6000RATE_8000=7 }  __attribute__((packed)) HwColibriMPU6000RateOptions;
/* Max value of any option in topmost parent MPU6000Rate of field MPU6000Rate */
#define HWCOLIBRI_MPU6000RATE_GLOBAL_MAXOPTVAL 7
/* Max value of any option in field MPU6000Rate */
#define HWCOLIBRI_MPU6000RATE_MAXOPTVAL 7
/* Ensure field MPU6000Rate contains valid data */
static inline bool HwColibriMPU6000RateIsValid( uint8_t CurrentMPU6000Rate ) { return CurrentMPU6000Rate < HWCOLIBRI_MPU6000RATE_MAXOPTVAL; }
// Field MPU6000DLPF information
/* Enumeration options for field MPU6000DLPF */
typedef enum { HWCOLIBRI_MPU6000DLPF_256=0, HWCOLIBRI_MPU6000DLPF_188=1, HWCOLIBRI_MPU6000DLPF_98=2, HWCOLIBRI_MPU6000DLPF_42=3, HWCOLIBRI_MPU6000DLPF_20=4, HWCOLIBRI_MPU6000DLPF_10=5, HWCOLIBRI_MPU6000DLPF_5=6 }  __attribute__((packed)) HwColibriMPU6000DLPFOptions;
/* Max value of any option in topmost parent MPU6000DLPF of field MPU6000DLPF */
#define HWCOLIBRI_MPU6000DLPF_GLOBAL_MAXOPTVAL 6
/* Max value of any option in field MPU6000DLPF */
#define HWCOLIBRI_MPU6000DLPF_MAXOPTVAL 6
/* Ensure field MPU6000DLPF contains valid data */
static inline bool HwColibriMPU6000DLPFIsValid( uint8_t CurrentMPU6000DLPF ) { return CurrentMPU6000DLPF < HWCOLIBRI_MPU6000DLPF_MAXOPTVAL; }
// Field Magnetometer information
/* Enumeration options for field Magnetometer */
typedef enum { HWCOLIBRI_MAGNETOMETER_DISABLED=0, HWCOLIBRI_MAGNETOMETER_INTERNAL=1, HWCOLIBRI_MAGNETOMETER_EXTERNALI2CUART1=2, HWCOLIBRI_MAGNETOMETER_EXTERNALI2CUART3=3 }  __attribute__((packed)) HwColibriMagnetometerOptions;
/* Max value of any option in topmost parent Magnetometer of field Magnetometer */
#define HWCOLIBRI_MAGNETOMETER_GLOBAL_MAXOPTVAL 3
/* Max value of any option in field Magnetometer */
#define HWCOLIBRI_MAGNETOMETER_MAXOPTVAL 3
/* Ensure field Magnetometer contains valid data */
static inline bool HwColibriMagnetometerIsValid( uint8_t CurrentMagnetometer ) { return CurrentMagnetometer < HWCOLIBRI_MAGNETOMETER_MAXOPTVAL; }
// Field ExtMagOrientation information
/* Enumeration options for field ExtMagOrientation */
typedef enum { HWCOLIBRI_EXTMAGORIENTATION_TOP0DEGCW=HWSHARED_MAGORIENTATION_TOP0DEGCW, HWCOLIBRI_EXTMAGORIENTATION_TOP90DEGCW=HWSHARED_MAGORIENTATION_TOP90DEGCW, HWCOLIBRI_EXTMAGORIENTATION_TOP180DEGCW=HWSHARED_MAGORIENTATION_TOP180DEGCW, HWCOLIBRI_EXTMAGORIENTATION_TOP270DEGCW=HWSHARED_MAGORIENTATION_TOP270DEGCW, HWCOLIBRI_EXTMAGORIENTATION_BOTTOM0DEGCW=HWSHARED_MAGORIENTATION_BOTTOM0DEGCW, HWCOLIBRI_EXTMAGORIENTATION_BOTTOM90DEGCW=HWSHARED_MAGORIENTATION_BOTTOM90DEGCW, HWCOLIBRI_EXTMAGORIENTATION_BOTTOM180DEGCW=HWSHARED_MAGORIENTATION_BOTTOM180DEGCW, HWCOLIBRI_EXTMAGORIENTATION_BOTTOM270DEGCW=HWSHARED_MAGORIENTATION_BOTTOM270DEGCW, HWCOLIBRI_EXTMAGORIENTATION_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwColibriExtMagOrientationOptions;
/* Max value of any option in topmost parent MagOrientation of field ExtMagOrientation */
#define HWCOLIBRI_EXTMAGORIENTATION_GLOBAL_MAXOPTVAL 7
/* Max value of any option in field ExtMagOrientation */
#define HWCOLIBRI_EXTMAGORIENTATION_MAXOPTVAL 7
/* Ensure field ExtMagOrientation contains valid data */
static inline bool HwColibriExtMagOrientationIsValid( uint8_t CurrentExtMagOrientation ) { return CurrentExtMagOrientation < HWCOLIBRI_EXTMAGORIENTATION_MAXOPTVAL; }


// set/Get functions
extern void HwColibriFrameSet( uint8_t *NewFrame );
extern void HwColibriFrameGet( uint8_t *NewFrame );
extern void HwColibriRcvrPortSet( uint8_t *NewRcvrPort );
extern void HwColibriRcvrPortGet( uint8_t *NewRcvrPort );
extern void HwColibriUart1Set( uint8_t *NewUart1 );
extern void HwColibriUart1Get( uint8_t *NewUart1 );
extern void HwColibriUart2Set( uint8_t *NewUart2 );
extern void HwColibriUart2Get( uint8_t *NewUart2 );
extern void HwColibriUart3Set( uint8_t *NewUart3 );
extern void HwColibriUart3Get( uint8_t *NewUart3 );
extern void HwColibriUart4Set( uint8_t *NewUart4 );
extern void HwColibriUart4Get( uint8_t *NewUart4 );
extern void HwColibriUSB_HIDPortSet( uint8_t *NewUSB_HIDPort );
extern void HwColibriUSB_HIDPortGet( uint8_t *NewUSB_HIDPort );
extern void HwColibriUSB_VCPPortSet( uint8_t *NewUSB_VCPPort );
extern void HwColibriUSB_VCPPortGet( uint8_t *NewUSB_VCPPort );
extern void HwColibriDSMxModeSet( uint8_t *NewDSMxMode );
extern void HwColibriDSMxModeGet( uint8_t *NewDSMxMode );
extern void HwColibriGyroRangeSet( uint8_t *NewGyroRange );
extern void HwColibriGyroRangeGet( uint8_t *NewGyroRange );
extern void HwColibriAccelRangeSet( uint8_t *NewAccelRange );
extern void HwColibriAccelRangeGet( uint8_t *NewAccelRange );
extern void HwColibriMPU6000RateSet( uint8_t *NewMPU6000Rate );
extern void HwColibriMPU6000RateGet( uint8_t *NewMPU6000Rate );
extern void HwColibriMPU6000DLPFSet( uint8_t *NewMPU6000DLPF );
extern void HwColibriMPU6000DLPFGet( uint8_t *NewMPU6000DLPF );
extern void HwColibriMagnetometerSet( uint8_t *NewMagnetometer );
extern void HwColibriMagnetometerGet( uint8_t *NewMagnetometer );
extern void HwColibriExtMagOrientationSet( uint8_t *NewExtMagOrientation );
extern void HwColibriExtMagOrientationGet( uint8_t *NewExtMagOrientation );


#endif // HWCOLIBRI_H

/**
 * @}
 * @}
 */
