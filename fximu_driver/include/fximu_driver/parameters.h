#ifndef __PARAMETERS_H_
#define	__PARAMETERS_H_

#define IMU_DATA_SIZE 64
#define PARAM_PACKET_SIZE 12
#define SYNC_PACKET_SIZE 6

#define PACKET_TYPE_CTL 0x03
#define PACKET_TYPE_PARAM 0x04

#define PARAM_TYPE_UINT8 0                        // unsigned int8 parameter
#define PARAM_TYPE_UINT16 1                       // unsigned int16 parameter
#define PARAM_TYPE_INT16 2                        // signed int16 parameter
#define PARAM_TYPE_FLOAT 3                        // float parameter
#define PARAM_TYPE_BOOL 4                         // boolean parameter

#define PACKET_PREFIX '$'                         // imu packet identifier
#define DIAG_PREFIX 'D'                           // diag packet identifier
#define CALIBRATION_PREFIX 'C'                    // calibration packet identifier
#define CALIBRATION_RAW_PREFIX 'R'                // calibration packet identifier
#define PACKET_POSTFIX '\n'                       // last byte of any packet

#define SYSCTL_RESET 0x01
#define SYSCTL_SOFTRESET 0x02
#define SYSCTL_RECALIB 0x03

#define FAIL_SENSOR_PARAM 0                       // error code: wrong parameters
#define FAIL_SENSOR_SELFTEST 1                    // error code: self test fail
#define FAIL_SENSOR_UNSTABLE 2                    // error code: initial calibration failed

#ifdef __cplusplus
extern "C" {
#endif

// TODO: order

#define SIZE_PARAMS_UINT8 13
const char *names_uint8[SIZE_PARAMS_UINT8] = {
                              "gyroODR",
                              "gyroFSR",
                              "accelODR",
                              "accelFSR",
                              "notchFilterBW",
                              "notchFilterDIR",
                              "antiAliasFilterBW",
                              "outputDiv",
                              "gyroUIFilterOrder",
                              "gyroUIFilterIndex",
                              "accelUIFilterOrder",
                              "accelUIFilterIndex",
                              "steadyLimit"
                            };                                

#define SIZE_PARAMS_UINT16 2
const char *names_uint16[SIZE_PARAMS_UINT16] = {
                               "timer0Frq",
                               "timer1Sec"
                             };

#define SIZE_PARAMS_INT16 6
const char *names_int16[SIZE_PARAMS_INT16] = {
                              "offsetGyroX",
                              "offsetGyroY",
                              "offsetGyroZ",
                              "offsetAccelX",
                              "offsetAccelY",
                              "offsetAccelZ"
                            };

#define SIZE_PARAMS_FLOAT 9
const char *names_float[] = {
                              "kDeltaAccelerationThreshold",
                              "kDeltaAngularThreshold",
                              "kAngularThreshold",
                              "notchFilterFHZ",
                              "gainACC",
                              "gainMAG",
                              "biasAlpha",
                              "gainAlpha",
                              "stableAlpha"
                            };

#define SIZE_PARAMS_BOOL 10
const char *names_bool[SIZE_PARAMS_BOOL] = {
                             "enableNotchFilter",
                             "enableAAFilter",
                             "enableUIFilter",
                             "enableSerial",
                             "enableInitialCalibration",
                             "enableAdaptiveBias",
                             "enableAdaptiveGain",
                             "enableMagnetometer",
                             "useMagnetometerData",
                             "pubMagnetometerData"
                            };

#define SIZE_PARAMS_STRING 2
const char *names_string[SIZE_PARAMS_STRING] = {
                             "imu_frame_id",
                             "mag_frame_id"
                            };

#ifdef __cplusplus
}
#endif 

#endif