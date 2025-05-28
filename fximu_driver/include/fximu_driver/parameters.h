#ifndef PARAMETERS_H_
#define	PARAMETERS_H_

#define PARAM_TYPE_UINT8 0                        // unsigned int8 parameter
#define PARAM_TYPE_UINT16 1                       // unsigned int16 parameter
#define PARAM_TYPE_INT16 2                        // signed int16 parameter
#define PARAM_TYPE_FLOAT 3                        // float parameter
#define PARAM_TYPE_BOOL 4                         // boolean parameter

#define PARAMETER_PREFIX 'P'                      // parameter packet identifier
#define DATA_PREFIX '$'                           // imu packet identifier
#define DIAG_PREFIX 'D'                           // diag packet identifier
#define CALIBRATION_PREFIX 'C'                    // calibration packet identifier
#define RAW_PREFIX 'R'                            // raw packet identifier
#define SYNC_PREFIX '+'                           // sync packet
#define INIT_PREFIX '>'                           // initial sync packet
#define PACKET_POSTFIX '\n'                       // last byte of any packet

#define SYNC_PACKET_SIZE 6                        // packet size for initial sync packet
#define USB_PACKET_SIZE 64                        // all the rest of the packets are 64 bytes

#define PACKET_TYPE_SYSCTL 0x03                   // sysctl packet
#define PACKET_TYPE_PARAM 0x04                    // parameter packet

#define SYSCTL_RESET 0x01
#define SYSCTL_SOFTRESET 0x02
#define SYSCTL_RECALIB 0x03

#define FAIL_SENSOR_PARAM 0                       // error code: wrong parameters
#define FAIL_SENSOR_SELFTEST 1                    // error code: self test fail
#define FAIL_SENSOR_UNSTABLE 2                    // error code: initial calibration failed

#ifdef __cplusplus
extern "C" {
#endif

#define SIZE_PARAMS_UINT8 15
inline const char *names_uint8[SIZE_PARAMS_UINT8] = {
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
                              "steadyLimit",
                              "magOdr",
                              "magAvg"
                            };                                

#define SIZE_PARAMS_UINT16 2
inline const char *names_uint16[SIZE_PARAMS_UINT16] = {
                               "timer0Sec",
                               "timer1Sec"
                             };

#define SIZE_PARAMS_INT16 6
inline const char *names_int16[SIZE_PARAMS_INT16] = {
                              "offsetGyroX",
                              "offsetGyroY",
                              "offsetGyroZ",
                              "offsetAccelX",
                              "offsetAccelY",
                              "offsetAccelZ"
                            };

#define SIZE_PARAMS_FLOAT 22
inline const char *names_float[] = {
                              "kDeltaAccelerationThreshold",
                              "kDeltaAngularThreshold",
                              "kAngularThreshold",
                              "notchFilterFHZ",
                              "filterGainAccel",
                              "filterGainMag",
                              "biasAlpha",
                              "gainAlpha",
                              "stableAlpha",
                              "magAlpha",
                              "magBiasX",
                              "magBiasY",
                              "magBiasZ",
                              "magSoftA1",
                              "magSoftA2",
                              "magSoftA3",
                              "magSoftB1",
                              "magSoftB2",
                              "magSoftB3",
                              "magSoftC1",
                              "magSoftC2",
                              "magSoftC3"
                            };

#define SIZE_PARAMS_BOOL 10
inline const char *names_bool[SIZE_PARAMS_BOOL] = {
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
inline const char *names_string[SIZE_PARAMS_STRING] = {
                             "imu_frame_id",
                             "mag_frame_id"
                            };

#ifdef __cplusplus
}
#endif 

#endif