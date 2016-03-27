/**
 ******************************************************************************
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 * @addtogroup HwAQ32 HwAQ32
 * @brief Selection of optional hardware configurations.
 *
 *
 * @file       hwaq32.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @brief      Include files for the HwAQ32 object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: hwaq32.xml. 
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

#ifndef HWAQ32_H
#define HWAQ32_H

#include "pios_queue.h"
#include "uavoversion.h"

#include <hwshared.h>


// Object constants
#define HWAQ32_OBJID 0xB749A992
#define HWAQ32_ISSINGLEINST 1
#define HWAQ32_ISSETTINGS 1
#define HWAQ32_NUMBYTES 16

// Generic interface functions
int32_t HwAQ32Initialize();
UAVObjHandle HwAQ32Handle();
void HwAQ32SetDefaults(UAVObjHandle obj, uint16_t instId);

// Object data
typedef struct {
    uint8_t RcvrPort;
    uint8_t Uart1;
    uint8_t Uart2;
    uint8_t Uart3;
    uint8_t Uart4;
    uint8_t Uart6;
    uint8_t ADCInputs;
    uint8_t USB_HIDPort;
    uint8_t USB_VCPPort;
    uint8_t DSMxMode;
    uint8_t GyroRange;
    uint8_t AccelRange;
    uint8_t MPU6000Rate;
    uint8_t MPU6000DLPF;
    uint8_t Magnetometer;
    uint8_t ExtMagOrientation;

} __attribute__((packed)) __attribute__((aligned(4))) HwAQ32Data;

// Typesafe Object access functions
/**
 * @function HwAQ32Get(dataOut)
 * @brief Populate a HwAQ32Data object
 * @param[out] dataOut 
 */
static inline int32_t HwAQ32Get(HwAQ32Data *dataOut) { return UAVObjGetData(HwAQ32Handle(), dataOut); }

static inline int32_t HwAQ32Set(const HwAQ32Data *dataIn) { return UAVObjSetData(HwAQ32Handle(), dataIn); }

static inline int32_t HwAQ32InstGet(uint16_t instId, HwAQ32Data *dataOut) { return UAVObjGetInstanceData(HwAQ32Handle(), instId, dataOut); }

static inline int32_t HwAQ32InstSet(uint16_t instId, const HwAQ32Data *dataIn) { return UAVObjSetInstanceData(HwAQ32Handle(), instId, dataIn); }

static inline int32_t HwAQ32ConnectQueue(struct pios_queue *queue) { return UAVObjConnectQueue(HwAQ32Handle(), queue, EV_MASK_ALL_UPDATES); }

static inline int32_t HwAQ32ConnectCallback(UAVObjEventCallback cb) { return UAVObjConnectCallback(HwAQ32Handle(), cb, EV_MASK_ALL_UPDATES); }

static inline uint16_t HwAQ32CreateInstance() { return UAVObjCreateInstance(HwAQ32Handle(), &HwAQ32SetDefaults); }

static inline void HwAQ32RequestUpdate() { UAVObjRequestUpdate(HwAQ32Handle()); }

static inline void HwAQ32RequestInstUpdate(uint16_t instId) { UAVObjRequestInstanceUpdate(HwAQ32Handle(), instId); }

static inline void HwAQ32Updated() { UAVObjUpdated(HwAQ32Handle()); }

static inline void HwAQ32InstUpdated(uint16_t instId) { UAVObjInstanceUpdated(HwAQ32Handle(), instId); }

static inline int32_t HwAQ32GetMetadata(UAVObjMetadata *dataOut) { return UAVObjGetMetadata(HwAQ32Handle(), dataOut); }

static inline int32_t HwAQ32SetMetadata(const UAVObjMetadata *dataIn) { return UAVObjSetMetadata(HwAQ32Handle(), dataIn); }

static inline int8_t HwAQ32ReadOnly() { return UAVObjReadOnly(HwAQ32Handle()); }

static inline uint16_t HwAQ32GetNumInstances(){ return UAVObjGetNumInstances(HwAQ32Handle()); }

static inline uint32_t HwAQ32GetNumBytes(){ return UAVObjGetNumBytes(HwAQ32Handle()); }

// Field information
// Field RcvrPort information
/* Enumeration options for field RcvrPort */
typedef enum { HWAQ32_RCVRPORT_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWAQ32_RCVRPORT_PPM=HWSHARED_PORTTYPES_PPM, HWAQ32_RCVRPORT_PWM=HWSHARED_PORTTYPES_PWM, HWAQ32_RCVRPORT_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwAQ32RcvrPortOptions;
/* Max value of any option in topmost parent PortTypes of field RcvrPort */
#define HWAQ32_RCVRPORT_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field RcvrPort */
#define HWAQ32_RCVRPORT_MAXOPTVAL 20
/* Ensure field RcvrPort contains valid data */
static inline bool HwAQ32RcvrPortIsValid( uint8_t CurrentRcvrPort ) { return CurrentRcvrPort < HWAQ32_RCVRPORT_MAXOPTVAL; }
// Field Uart1 information
/* Enumeration options for field Uart1 */
typedef enum { HWAQ32_UART1_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWAQ32_UART1_TELEMETRY=HWSHARED_PORTTYPES_TELEMETRY, HWAQ32_UART1_DEBUGCONSOLE=HWSHARED_PORTTYPES_DEBUGCONSOLE, HWAQ32_UART1_MAVLINKTX=HWSHARED_PORTTYPES_MAVLINKTX, HWAQ32_UART1_MSP=HWSHARED_PORTTYPES_MSP, HWAQ32_UART1_HOTTTELEMETRY=HWSHARED_PORTTYPES_HOTTTELEMETRY, HWAQ32_UART1_FRSKYSENSORHUB=HWSHARED_PORTTYPES_FRSKYSENSORHUB, HWAQ32_UART1_LIGHTTELEMETRYTX=HWSHARED_PORTTYPES_LIGHTTELEMETRYTX, HWAQ32_UART1_PICOC=HWSHARED_PORTTYPES_PICOC, HWAQ32_UART1_FRSKYSPORTTELEMETRY=HWSHARED_PORTTYPES_FRSKYSPORTTELEMETRY, HWAQ32_UART1_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwAQ32Uart1Options;
/* Max value of any option in topmost parent PortTypes of field Uart1 */
#define HWAQ32_UART1_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field Uart1 */
#define HWAQ32_UART1_MAXOPTVAL 16
/* Ensure field Uart1 contains valid data */
static inline bool HwAQ32Uart1IsValid( uint8_t CurrentUart1 ) { return CurrentUart1 < HWAQ32_UART1_MAXOPTVAL; }
// Field Uart2 information
/* Enumeration options for field Uart2 */
typedef enum { HWAQ32_UART2_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWAQ32_UART2_GPS=HWSHARED_PORTTYPES_GPS, HWAQ32_UART2_MAVLINKTX_GPS_RX=HWSHARED_PORTTYPES_MAVLINKTX_GPS_RX, HWAQ32_UART2_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwAQ32Uart2Options;
/* Max value of any option in topmost parent PortTypes of field Uart2 */
#define HWAQ32_UART2_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field Uart2 */
#define HWAQ32_UART2_MAXOPTVAL 8
/* Ensure field Uart2 contains valid data */
static inline bool HwAQ32Uart2IsValid( uint8_t CurrentUart2 ) { return CurrentUart2 < HWAQ32_UART2_MAXOPTVAL; }
// Field Uart3 information
/* Enumeration options for field Uart3 */
typedef enum { HWAQ32_UART3_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWAQ32_UART3_TELEMETRY=HWSHARED_PORTTYPES_TELEMETRY, HWAQ32_UART3_GPS=HWSHARED_PORTTYPES_GPS, HWAQ32_UART3_SBUS=HWSHARED_PORTTYPES_SBUS, HWAQ32_UART3_DEBUGCONSOLE=HWSHARED_PORTTYPES_DEBUGCONSOLE, HWAQ32_UART3_COMBRIDGE=HWSHARED_PORTTYPES_COMBRIDGE, HWAQ32_UART3_MAVLINKTX=HWSHARED_PORTTYPES_MAVLINKTX, HWAQ32_UART3_MAVLINKTX_GPS_RX=HWSHARED_PORTTYPES_MAVLINKTX_GPS_RX, HWAQ32_UART3_MSP=HWSHARED_PORTTYPES_MSP, HWAQ32_UART3_HOTTSUMD=HWSHARED_PORTTYPES_HOTTSUMD, HWAQ32_UART3_HOTTSUMH=HWSHARED_PORTTYPES_HOTTSUMH, HWAQ32_UART3_HOTTTELEMETRY=HWSHARED_PORTTYPES_HOTTTELEMETRY, HWAQ32_UART3_FRSKYSENSORHUB=HWSHARED_PORTTYPES_FRSKYSENSORHUB, HWAQ32_UART3_LIGHTTELEMETRYTX=HWSHARED_PORTTYPES_LIGHTTELEMETRYTX, HWAQ32_UART3_PICOC=HWSHARED_PORTTYPES_PICOC, HWAQ32_UART3_FRSKYSPORTTELEMETRY=HWSHARED_PORTTYPES_FRSKYSPORTTELEMETRY, HWAQ32_UART3_OPENLOG=HWSHARED_PORTTYPES_OPENLOG, HWAQ32_UART3_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwAQ32Uart3Options;
/* Max value of any option in topmost parent PortTypes of field Uart3 */
#define HWAQ32_UART3_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field Uart3 */
#define HWAQ32_UART3_MAXOPTVAL 17
/* Ensure field Uart3 contains valid data */
static inline bool HwAQ32Uart3IsValid( uint8_t CurrentUart3 ) { return CurrentUart3 < HWAQ32_UART3_MAXOPTVAL; }
// Field Uart4 information
/* Enumeration options for field Uart4 */
typedef enum { HWAQ32_UART4_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWAQ32_UART4_TELEMETRY=HWSHARED_PORTTYPES_TELEMETRY, HWAQ32_UART4_GPS=HWSHARED_PORTTYPES_GPS, HWAQ32_UART4_DSM=HWSHARED_PORTTYPES_DSM, HWAQ32_UART4_DEBUGCONSOLE=HWSHARED_PORTTYPES_DEBUGCONSOLE, HWAQ32_UART4_COMBRIDGE=HWSHARED_PORTTYPES_COMBRIDGE, HWAQ32_UART4_MAVLINKTX=HWSHARED_PORTTYPES_MAVLINKTX, HWAQ32_UART4_MAVLINKTX_GPS_RX=HWSHARED_PORTTYPES_MAVLINKTX_GPS_RX, HWAQ32_UART4_MSP=HWSHARED_PORTTYPES_MSP, HWAQ32_UART4_HOTTSUMD=HWSHARED_PORTTYPES_HOTTSUMD, HWAQ32_UART4_HOTTSUMH=HWSHARED_PORTTYPES_HOTTSUMH, HWAQ32_UART4_HOTTTELEMETRY=HWSHARED_PORTTYPES_HOTTTELEMETRY, HWAQ32_UART4_FRSKYSENSORHUB=HWSHARED_PORTTYPES_FRSKYSENSORHUB, HWAQ32_UART4_LIGHTTELEMETRYTX=HWSHARED_PORTTYPES_LIGHTTELEMETRYTX, HWAQ32_UART4_PICOC=HWSHARED_PORTTYPES_PICOC, HWAQ32_UART4_FRSKYSPORTTELEMETRY=HWSHARED_PORTTYPES_FRSKYSPORTTELEMETRY, HWAQ32_UART4_OPENLOG=HWSHARED_PORTTYPES_OPENLOG, HWAQ32_UART4_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwAQ32Uart4Options;
/* Max value of any option in topmost parent PortTypes of field Uart4 */
#define HWAQ32_UART4_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field Uart4 */
#define HWAQ32_UART4_MAXOPTVAL 17
/* Ensure field Uart4 contains valid data */
static inline bool HwAQ32Uart4IsValid( uint8_t CurrentUart4 ) { return CurrentUart4 < HWAQ32_UART4_MAXOPTVAL; }
// Field Uart6 information
/* Enumeration options for field Uart6 */
typedef enum { HWAQ32_UART6_DISABLED=HWSHARED_PORTTYPES_DISABLED, HWAQ32_UART6_TELEMETRY=HWSHARED_PORTTYPES_TELEMETRY, HWAQ32_UART6_GPS=HWSHARED_PORTTYPES_GPS, HWAQ32_UART6_DSM=HWSHARED_PORTTYPES_DSM, HWAQ32_UART6_DEBUGCONSOLE=HWSHARED_PORTTYPES_DEBUGCONSOLE, HWAQ32_UART6_COMBRIDGE=HWSHARED_PORTTYPES_COMBRIDGE, HWAQ32_UART6_MAVLINKTX=HWSHARED_PORTTYPES_MAVLINKTX, HWAQ32_UART6_MAVLINKTX_GPS_RX=HWSHARED_PORTTYPES_MAVLINKTX_GPS_RX, HWAQ32_UART6_MSP=HWSHARED_PORTTYPES_MSP, HWAQ32_UART6_HOTTSUMD=HWSHARED_PORTTYPES_HOTTSUMD, HWAQ32_UART6_HOTTSUMH=HWSHARED_PORTTYPES_HOTTSUMH, HWAQ32_UART6_HOTTTELEMETRY=HWSHARED_PORTTYPES_HOTTTELEMETRY, HWAQ32_UART6_FRSKYSENSORHUB=HWSHARED_PORTTYPES_FRSKYSENSORHUB, HWAQ32_UART6_LIGHTTELEMETRYTX=HWSHARED_PORTTYPES_LIGHTTELEMETRYTX, HWAQ32_UART6_PICOC=HWSHARED_PORTTYPES_PICOC, HWAQ32_UART6_FRSKYSPORTTELEMETRY=HWSHARED_PORTTYPES_FRSKYSPORTTELEMETRY, HWAQ32_UART6_OPENLOG=HWSHARED_PORTTYPES_OPENLOG, HWAQ32_UART6_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwAQ32Uart6Options;
/* Max value of any option in topmost parent PortTypes of field Uart6 */
#define HWAQ32_UART6_GLOBAL_MAXOPTVAL 20
/* Max value of any option in field Uart6 */
#define HWAQ32_UART6_MAXOPTVAL 17
/* Ensure field Uart6 contains valid data */
static inline bool HwAQ32Uart6IsValid( uint8_t CurrentUart6 ) { return CurrentUart6 < HWAQ32_UART6_MAXOPTVAL; }
// Field ADCInputs information
/* Enumeration options for field ADCInputs */
typedef enum { HWAQ32_ADCINPUTS_DISABLED=0, HWAQ32_ADCINPUTS_ENABLED=1 }  __attribute__((packed)) HwAQ32ADCInputsOptions;
/* Max value of any option in topmost parent ADCInputs of field ADCInputs */
#define HWAQ32_ADCINPUTS_GLOBAL_MAXOPTVAL 1
/* Max value of any option in field ADCInputs */
#define HWAQ32_ADCINPUTS_MAXOPTVAL 1
/* Ensure field ADCInputs contains valid data */
static inline bool HwAQ32ADCInputsIsValid( uint8_t CurrentADCInputs ) { return CurrentADCInputs < HWAQ32_ADCINPUTS_MAXOPTVAL; }
// Field USB_HIDPort information
/* Enumeration options for field USB_HIDPort */
typedef enum { HWAQ32_USB_HIDPORT_USBTELEMETRY=0, HWAQ32_USB_HIDPORT_DISABLED=1 }  __attribute__((packed)) HwAQ32USB_HIDPortOptions;
/* Max value of any option in topmost parent USB_HIDPort of field USB_HIDPort */
#define HWAQ32_USB_HIDPORT_GLOBAL_MAXOPTVAL 1
/* Max value of any option in field USB_HIDPort */
#define HWAQ32_USB_HIDPORT_MAXOPTVAL 1
/* Ensure field USB_HIDPort contains valid data */
static inline bool HwAQ32USB_HIDPortIsValid( uint8_t CurrentUSB_HIDPort ) { return CurrentUSB_HIDPort < HWAQ32_USB_HIDPORT_MAXOPTVAL; }
// Field USB_VCPPort information
/* Enumeration options for field USB_VCPPort */
typedef enum { HWAQ32_USB_VCPPORT_USBTELEMETRY=0, HWAQ32_USB_VCPPORT_COMBRIDGE=1, HWAQ32_USB_VCPPORT_DEBUGCONSOLE=2, HWAQ32_USB_VCPPORT_PICOC=3, HWAQ32_USB_VCPPORT_DISABLED=4 }  __attribute__((packed)) HwAQ32USB_VCPPortOptions;
/* Max value of any option in topmost parent USB_VCPPort of field USB_VCPPort */
#define HWAQ32_USB_VCPPORT_GLOBAL_MAXOPTVAL 4
/* Max value of any option in field USB_VCPPort */
#define HWAQ32_USB_VCPPORT_MAXOPTVAL 4
/* Ensure field USB_VCPPort contains valid data */
static inline bool HwAQ32USB_VCPPortIsValid( uint8_t CurrentUSB_VCPPort ) { return CurrentUSB_VCPPort < HWAQ32_USB_VCPPORT_MAXOPTVAL; }
// Field DSMxMode information
/* Enumeration options for field DSMxMode */
typedef enum { HWAQ32_DSMXMODE_AUTODETECT=HWSHARED_DSMXMODE_AUTODETECT, HWAQ32_DSMXMODE_FORCE10BIT=HWSHARED_DSMXMODE_FORCE10BIT, HWAQ32_DSMXMODE_FORCE11BIT=HWSHARED_DSMXMODE_FORCE11BIT, HWAQ32_DSMXMODE_BIND3PULSES=HWSHARED_DSMXMODE_BIND3PULSES, HWAQ32_DSMXMODE_BIND4PULSES=HWSHARED_DSMXMODE_BIND4PULSES, HWAQ32_DSMXMODE_BIND5PULSES=HWSHARED_DSMXMODE_BIND5PULSES, HWAQ32_DSMXMODE_BIND6PULSES=HWSHARED_DSMXMODE_BIND6PULSES, HWAQ32_DSMXMODE_BIND7PULSES=HWSHARED_DSMXMODE_BIND7PULSES, HWAQ32_DSMXMODE_BIND8PULSES=HWSHARED_DSMXMODE_BIND8PULSES, HWAQ32_DSMXMODE_BIND9PULSES=HWSHARED_DSMXMODE_BIND9PULSES, HWAQ32_DSMXMODE_BIND10PULSES=HWSHARED_DSMXMODE_BIND10PULSES, HWAQ32_DSMXMODE_DONTSWITCHONCHILDENUMS=255 }  __attribute__((packed)) HwAQ32DSMxModeOptions;
/* Max value of any option in topmost parent DSMxMode of field DSMxMode */
#define HWAQ32_DSMXMODE_GLOBAL_MAXOPTVAL 10
/* Max value of any option in field DSMxMode */
#define HWAQ32_DSMXMODE_MAXOPTVAL 10
/* Ensure field DSMxMode contains valid data */
static inline bool HwAQ32DSMxModeIsValid( uint8_t CurrentDSMxMode ) { return CurrentDSMxMode < HWAQ32_DSMXMODE_MAXOPTVAL; }
// Field GyroRange information
/* Enumeration options for field GyroRange */
typedef enum { HWAQ32_GYRORANGE_250=0, HWAQ32_GYRORANGE_500=1, HWAQ32_GYRORANGE_1000=2, HWAQ32_GYRORANGE_2000=3 }  __attribute__((packed)) HwAQ32GyroRangeOptions;
/* Max value of any option in topmost parent GyroRange of field GyroRange */
#define HWAQ32_GYRORANGE_GLOBAL_MAXOPTVAL 3
/* Max value of any option in field GyroRange */
#define HWAQ32_GYRORANGE_MAXOPTVAL 3
/* Ensure field GyroRange contains valid data */
static inline bool HwAQ32GyroRangeIsValid( uint8_t CurrentGyroRange ) { return CurrentGyroRange < HWAQ32_GYRORANGE_MAXOPTVAL; }
// Field AccelRange information
/* Enumeration options for field AccelRange */
typedef enum { HWAQ32_ACCELRANGE_2G=0, HWAQ32_ACCELRANGE_4G=1, HWAQ32_ACCELRANGE_8G=2, HWAQ32_ACCELRANGE_16G=3 }  __attribute__((packed)) HwAQ32AccelRangeOptions;
/* Max value of any option in topmost parent AccelRange of field AccelRange */
#define HWAQ32_ACCELRANGE_GLOBAL_MAXOPTVAL 3
/* Max value of any option in field AccelRange */
#define HWAQ32_ACCELRANGE_MAXOPTVAL 3
/* Ensure field AccelRange contains valid data */
static inline bool HwAQ32AccelRangeIsValid( uint8_t CurrentAccelRange ) { return CurrentAccelRange < HWAQ32_ACCELRANGE_MAXOPTVAL; }
// Field MPU6000Rate information
/* Enumeration options for field MPU6000Rate */
typedef enum { HWAQ32_MPU6000RATE_200=0, HWAQ32_MPU6000RATE_333=1, HWAQ32_MPU6000RATE_500=2, HWAQ32_MPU6000RATE_666=3, HWAQ32_MPU6000RATE_1000=4, HWAQ32_MPU6000RATE_2000=5, HWAQ32_MPU6000RATE_4000=6, HWAQ32_MPU6000RATE_8000=7 }  __attribute__((packed)) HwAQ32MPU6000RateOptions;
/* Max value of any option in topmost parent MPU6000Rate of field MPU6000Rate */
#define HWAQ32_MPU6000RATE_GLOBAL_MAXOPTVAL 7
/* Max value of any option in field MPU6000Rate */
#define HWAQ32_MPU6000RATE_MAXOPTVAL 7
/* Ensure field MPU6000Rate contains valid data */
static inline bool HwAQ32MPU6000RateIsValid( uint8_t CurrentMPU6000Rate ) { return CurrentMPU6000Rate < HWAQ32_MPU6000RATE_MAXOPTVAL; }
// Field MPU6000DLPF information
/* Enumeration options for field MPU6000DLPF */
typedef enum { HWAQ32_MPU6000DLPF_256=0, HWAQ32_MPU6000DLPF_188=1, HWAQ32_MPU6000DLPF_98=2, HWAQ32_MPU6000DLPF_42=3, HWAQ32_MPU6000DLPF_20=4, HWAQ32_MPU6000DLPF_10=5, HWAQ32_MPU6000DLPF_5=6 }  __attribute__((packed)) HwAQ32MPU6000DLPFOptions;
/* Max value of any option in topmost parent MPU6000DLPF of field MPU6000DLPF */
#define HWAQ32_MPU6000DLPF_GLOBAL_MAXOPTVAL 6
/* Max value of any option in field MPU6000DLPF */
#define HWAQ32_MPU6000DLPF_MAXOPTVAL 6
/* Ensure field MPU6000DLPF contains valid data */
static inline bool HwAQ32MPU6000DLPFIsValid( uint8_t CurrentMPU6000DLPF ) { return CurrentMPU6000DLPF < HWAQ32_MPU6000DLPF_MAXOPTVAL; }
// Field Magnetometer information
/* Enumeration options for field Magnetometer */
typedef enum { HWAQ32_MAGNETOMETER_INTERNAL=0, HWAQ32_MAGNETOMETER_EXTERNAL=1 }  __attribute__((packed)) HwAQ32MagnetometerOptions;
/* Max value of any option in topmost parent Magnetometer of field Magnetometer */
#define HWAQ32_MAGNETOMETER_GLOBAL_MAXOPTVAL 1
/* Max value of any option in field Magnetometer */
#define HWAQ32_MAGNETOMETER_MAXOPTVAL 1
/* Ensure field Magnetometer contains valid data */
static inline bool HwAQ32MagnetometerIsValid( uint8_t CurrentMagnetometer ) { return CurrentMagnetometer < HWAQ32_MAGNETOMETER_MAXOPTVAL; }
// Field ExtMagOrientation information
/* Enumeration options for field ExtMagOrientation */
typedef enum { HWAQ32_EXTMAGORIENTATION_TOP0DEGCW=0, HWAQ32_EXTMAGORIENTATION_TOP90DEGCW=1, HWAQ32_EXTMAGORIENTATION_TOP180DEGCW=2, HWAQ32_EXTMAGORIENTATION_TOP270DEGCW=3, HWAQ32_EXTMAGORIENTATION_BOTTOM0DEGCW=4, HWAQ32_EXTMAGORIENTATION_BOTTOM90DEGCW=5, HWAQ32_EXTMAGORIENTATION_BOTTOM180DEGCW=6, HWAQ32_EXTMAGORIENTATION_BOTTOM270DEGCW=7 }  __attribute__((packed)) HwAQ32ExtMagOrientationOptions;
/* Max value of any option in topmost parent ExtMagOrientation of field ExtMagOrientation */
#define HWAQ32_EXTMAGORIENTATION_GLOBAL_MAXOPTVAL 7
/* Max value of any option in field ExtMagOrientation */
#define HWAQ32_EXTMAGORIENTATION_MAXOPTVAL 7
/* Ensure field ExtMagOrientation contains valid data */
static inline bool HwAQ32ExtMagOrientationIsValid( uint8_t CurrentExtMagOrientation ) { return CurrentExtMagOrientation < HWAQ32_EXTMAGORIENTATION_MAXOPTVAL; }


// set/Get functions
extern void HwAQ32RcvrPortSet( uint8_t *NewRcvrPort );
extern void HwAQ32RcvrPortGet( uint8_t *NewRcvrPort );
extern void HwAQ32Uart1Set( uint8_t *NewUart1 );
extern void HwAQ32Uart1Get( uint8_t *NewUart1 );
extern void HwAQ32Uart2Set( uint8_t *NewUart2 );
extern void HwAQ32Uart2Get( uint8_t *NewUart2 );
extern void HwAQ32Uart3Set( uint8_t *NewUart3 );
extern void HwAQ32Uart3Get( uint8_t *NewUart3 );
extern void HwAQ32Uart4Set( uint8_t *NewUart4 );
extern void HwAQ32Uart4Get( uint8_t *NewUart4 );
extern void HwAQ32Uart6Set( uint8_t *NewUart6 );
extern void HwAQ32Uart6Get( uint8_t *NewUart6 );
extern void HwAQ32ADCInputsSet( uint8_t *NewADCInputs );
extern void HwAQ32ADCInputsGet( uint8_t *NewADCInputs );
extern void HwAQ32USB_HIDPortSet( uint8_t *NewUSB_HIDPort );
extern void HwAQ32USB_HIDPortGet( uint8_t *NewUSB_HIDPort );
extern void HwAQ32USB_VCPPortSet( uint8_t *NewUSB_VCPPort );
extern void HwAQ32USB_VCPPortGet( uint8_t *NewUSB_VCPPort );
extern void HwAQ32DSMxModeSet( uint8_t *NewDSMxMode );
extern void HwAQ32DSMxModeGet( uint8_t *NewDSMxMode );
extern void HwAQ32GyroRangeSet( uint8_t *NewGyroRange );
extern void HwAQ32GyroRangeGet( uint8_t *NewGyroRange );
extern void HwAQ32AccelRangeSet( uint8_t *NewAccelRange );
extern void HwAQ32AccelRangeGet( uint8_t *NewAccelRange );
extern void HwAQ32MPU6000RateSet( uint8_t *NewMPU6000Rate );
extern void HwAQ32MPU6000RateGet( uint8_t *NewMPU6000Rate );
extern void HwAQ32MPU6000DLPFSet( uint8_t *NewMPU6000DLPF );
extern void HwAQ32MPU6000DLPFGet( uint8_t *NewMPU6000DLPF );
extern void HwAQ32MagnetometerSet( uint8_t *NewMagnetometer );
extern void HwAQ32MagnetometerGet( uint8_t *NewMagnetometer );
extern void HwAQ32ExtMagOrientationSet( uint8_t *NewExtMagOrientation );
extern void HwAQ32ExtMagOrientationGet( uint8_t *NewExtMagOrientation );


#endif // HWAQ32_H

/**
 * @}
 * @}
 */
