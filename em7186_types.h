#ifndef EM7186_TYPES_H
#define EM7186_TYPES_H

#include "main.h"

// Global data
typedef signed char     s8;
typedef char            u8;    
typedef short           s16;   
typedef unsigned short  u16;   
typedef int             s32;   
typedef unsigned int    u32;   
typedef unsigned long   u64;

typedef struct
{
    u8 paramNo;
    u8 size;
} ParamInfo;



#define M_PI (3.14159274101F)
#define TRUE  1
#define FALSE 0
//=========================================================================
// em7186 Defines
//=========================================================================
#define SENtral_ADDRESS                                 0x50 // Sentral 8-bit Address (MBED uses 8-bit version)

#define MAX_I2C_WRITE                                   32
#define I2C_MAX_READ                                    8
#define FIRMWARE_HEADER_SIZE                            16
#define IMAGE_SIGNATURE_LSB                             0x2A
#define IMAGE_SIGNATURE_MSG                             0x65

// REGISTER MAP
#define FIFO_FLUSH_REG                                  0x32 // flush sensor data from fifo
#define CHIP_CONTROL_REG                                0x34 // enable cpu
#define HOST_STATUS_REG                                 0x35 // host status
#define INT_STATUS_REG                                  0x36 // interrupt status
#define CHIP_STATUS_REG                                 0x37 // chip status
#define BYTES_REMANING_REG                              0x38 // LSB of remaing bytes in FIFO
#define PARAM_ACK_REG                                   0x3A
#define PARAM_SAVE_REG                                  0x3B
#define ERR_REG                                         0x50 // error register
#define INT_STATE_REG                                   0x51 // interrupt state
#define DEBUG_VAL_REG                                   0x52 // debug value
#define DEBUG_STATE_REG                                 0x53 // debug state
#define PARAM_PAGE_SELECT_REG                           0x54 // parameter page select (and transfer size)
#define HOST_INTERFACE_CTRL_REG                         0x55
#define PARAM_LOAD_REG                                  0x5C
#define PARAM_REQUEST_REG                               0x64
#define HOST_IRQ_TIME_REG                               0x6C
#define ROM_VERSION_REG                                 0x70
#define PRODUCT_ID_REG                                  0x90 // Product ID
#define REVISION_ID_REG                                 0x91 // Product ID
#define SR_UPLOAD_DATA_REG                              0x96 // Firmware upload address
#define HOST_CRC_REG                                    0x97 // Firmware upload CRC
#define RESET_REQ_REG                                   0x9B // Request system reset
#define PASS_THROUGH_RDY_REG                            0x9E
#define SCL_LOW_CYCLES_REG                              0x9F
#define PASS_THROUGH_CFG_REG                            0xA0

// FIFO FLUSH
#define FIFO_FLUSH_DISCARD_NON_WAKE                     0xFA
#define FIFO_FLUSH_DISCARD_WAKE                         0xFB
#define FIFO_FLUSH_TRANSFER_NON_WAKE                    0xFC
#define FIFO_FLUSH_TRANSFER_WAKE                        0xFD
#define FIFO_FLUSH_DISCARD_ALL                          0xFE
#define FIFO_FLUSH_TRANSFER_ALL                         0xFF

// CHIP CONTROL
#define CHIP_CONTROL_CPU_STOP                           0x00
#define CHIP_CONTROL_CPU_RUN                            0x01
#define CHIP_CONTROL_HOST_UPLOAD                        0x02

// HOST STATUS
#define HOST_STATUS_RESET                               0X01
#define HOST_STATUS_ALGORITHM_STANDBY                   0x02
#define HOST_STATUS_INTERFACE_BITS                      0x1C
#define HOST_STATUS_INTERFACE_K                         0x00
#define HOST_STATUS_INTERFACE_L                         0x04
#define HOST_STATUS_INTERFACE_L_EXTENDED                0x08
#define HOST_STATUS_ALGORITHM_BITS                      0xE0
#define HOST_STATUS_ALGORITHM_SPACEPOINT                0x20

// INT STATUS
#define INT_STATUS_HOST_INT                             0x01
#define INT_STATUS_WAKE_WATERMARK                       0x02
#define INT_STATUS_WAKE_LATENCY                         0x04
#define INT_STATUS_WAKE_IMMEDIATE                       0x08
#define INT_STATUS_NON_WAKE_WATERMARK                   0x10
#define INT_STATUS_NON_WAKE_LATENCY                     0x20
#define INT_STATUS_NON_WAKE_IMMEDIATE                   0x40

// CHIP STATUS
#define CHIP_STATUS_EEPROM_DETECTED                     0x01
#define CHIP_STATUS_EEPROM_UPLOAD_DONE                  0x02
#define CHIP_STATUS_EEPROM_UPLOAD_ERROR                 0x04
#define CHIP_STATUS_FIRMWARE_IDLE                       0x08
#define CHIP_STATUS_NO_EEPROM                           0x10

// HOST INTERFACE CONTROL
#define HOST_INTERFACE_CTRL_ALGORITHM_STANDBY           0x01
#define HOST_INTERFACE_CTRL_ABORT_TRANSFER              0x02
#define HOST_INTERFACE_CTRL_UPDATE_TRANSFER_CNT         0x04
#define HOST_INTERFACE_CTRL_WAKE_FIFO_INT_DISABLE       0x08
#define HOST_INTERFACE_CTRL_NED_COORDINATES             0x10
#define HOST_INTERFACE_CTRL_AP_SUSPEND                  0x20
#define HOST_INTERFACE_CTRL_REQ_SENSOR_SELF_TEST        0x40
#define HOST_INTERFACE_CTRL_NON_WAKE_FIFO_INT_DISABLE   0x80

// ROM VERSION
#define ROM_VERSION_7180_DIO1                           0x07A8
#define ROM_VERSION_7180_DIO2                           0x09E6
#define ROM_VERSION_7184_DIO1                           0x1F9D
#define ROM_VERSION_7186_DIO1                           0xFFFF // TODO:update when available

// PRODUCT ID
#define PRODUCT_ID_7180                                 0x80
#define PRODUCT_ID_7184                                 0x84
#define PRODUCT_ID_7186                                 0x86

// REVISION ID
#define REVISION_ID_718X                                1

// PARAM IO PAGES
#define PARAM_PAGE_SYSTEM                               1
#define PARAM_PAGE_WARM_START                           2 
#define PARAM_PAGE_SENSOR_INFO                          3
#define PARAM_PAGE_SENSOR_CONF                          5
#define PARAM_PAGE_KNOBS                                13

// PAGE 1 PARAMETERS
#define PARAM_META_EVENT_CONTROL                        1
#define PARAM_FIFO_CONTROL                              2
#define PARAM_SENSOR_STATUS_BANK_0                      3
#define SENSORS_PER_STATUS_BANK                         16
#define PARAM_WAKE_META_EVENT_CONTRAL                   29
#define PARAM_HOST_IRQ_TIMESTAMP                        30
#define PARAM_PHYSICAL_SENSOR_STATUS                    31

#define SENSOR_TYPE_ACCELEROMETER                       1
#define SENSOR_TYPE_MAGNETIC_FIELD                      2
#define SENSOR_TYPE_ORIENTATION                         3
#define SENSOR_TYPE_GYROSCOPE                           4
#define SENSOR_TYPE_LIGHT                               5
#define SENSOR_TYPE_PRESSURE                            6
#define SENSOR_TYPE_TEMPERATURE                         7
#define SENSOR_TYPE_PROXIMITY                           8
#define SENSOR_TYPE_GRAVITY                             9
#define SENSOR_TYPE_LINEAR_ACCELERATION                 10
#define SENSOR_TYPE_ROTATION_VECTOR                     11
#define SENSOR_TYPE_RELATIVE_HUMIDITY                   12
#define SENSOR_TYPE_AMBIENT_TEMPERATURE                 13
#define SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED         14
#define SENSOR_TYPE_GAME_ROTATION_VECTOR                15
#define SENSOR_TYPE_GYROSCOPE_UNCALIBRATED              16
#define SENSOR_TYPE_SIGNIFICANT_MOTION                  17
#define SENSOR_TYPE_STEP_DETECTOR                       18
#define SENSOR_TYPE_STEP_COUNTER                        19
#define SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR         20
#define SENSOR_TYPE_HEART_RATE                          21
#define SENSOR_TYPE_TILT_DETECTOR                       22
#define SENSOR_TYPE_WAKE_GESTURE                        23
#define SENSOR_TYPE_GLANCE_GESTURE                      24
#define SENSOR_TYPE_PICK_UP_GESTURE                     25
#define SENSOR_TYPE_PDR                                 26
#define SENSOR_TYPE_RAW_ACCEL                           28  //jm 
#define SENSOR_TYPE_RAW_MAG                             29  //jm
#define SENSOR_TYPE_RAW_GYRO                            30  //jm
#define SENSOR_TYPE_ACTIVITY                            31
#define SENSOR_TYPE_CAR_MAG_DATA                        32
#define SENSOR_TYPE_VISIBLE_END                         63


#define SENSOR_TYPE_ACCELEROMETER_WAKE                  65
#define SENSOR_TYPE_MAGNETIC_FIELD_WAKE                 66
#define SENSOR_TYPE_ORIENTATION_WAKE                    67
#define SENSOR_TYPE_GYROSCOPE_WAKE                      68
#define SENSOR_TYPE_LIGHT_WAKE                          69
#define SENSOR_TYPE_PRESSURE_WAKE                       70
#define SENSOR_TYPE_TEMPERATURE_WAKE                    71
#define SENSOR_TYPE_PROXIMITY_WAKE                      72
#define SENSOR_TYPE_GRAVITY_WAKE                        73
#define SENSOR_TYPE_LINEAR_ACCELERATION_WAKE            74
#define SENSOR_TYPE_ROTATION_VECTOR_WAKE                75
#define SENSOR_TYPE_RELATIVE_HUMIDITY_WAKE              76
#define SENSOR_TYPE_AMBIENT_TEMPERATURE_WAKE            77
#define SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED_WAKE    78
#define SENSOR_TYPE_GAME_ROTATION_VECTOR_WAKE           79
#define SENSOR_TYPE_GYROSCOPE_UNCALIBRATED_WAKE         80
#define SENSOR_TYPE_SIGNIFICANT_MOTION_WAKE             81
#define SENSOR_TYPE_STEP_DETECTOR_WAKE                  82
#define SENSOR_TYPE_STEP_COUNTER_WAKE                   83
#define SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR_WAKE    84
#define SENSOR_TYPE_HEART_RATE_WAKE                     85
#define SENSOR_TYPE_TILT_DETECTOR_WAKE                  86
#define SENSOR_TYPE_WAKE_GESTURE_WAKE                   87
#define SENSOR_TYPE_GLANCE_GESTURE_WAKE                 88
#define SENSOR_TYPE_PICK_UP_GESTURE_WAKE                89
#define SENSOR_TYPE_PDR_WAKE                            90
#define SENSOR_TYPE_ACTIVITY_WAKE                       95
#define SENSOR_TYPE_CAR_MAG_DATA_WAKE                   96

/** System events - SENSOR_TYPE >= 0x40 */
#define SENSOR_TYPE_DEBUG                               245  // 0xF5
#define SENSOR_TYPE_TIMESTAMP_WAKE                      246  // 0xF6
#define SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE             247  // 0xF7
#define SENSOR_TYPE_META_WAKE                           248  // 0xF8
//#define SENSOR_TYPE_RAW_GYRO                            249  // 0xF9
//#define SENSOR_TYPE_RAW_MAG                             250  // 0xFA
//#define SENSOR_TYPE_RAW_ACCEL                           251  // 0xFB
#define SENSOR_TYPE_TIMESTAMP                           252  // 0xFC
#define SENSOR_TYPE_TIMESTAMP_OVERFLOW                  253  // 0xFD
#define SENSOR_TYPE_META                                254  // 0xFE

#define META_EVENT_FLUSH_COMPLETE                       1
#define META_EVENT_SAMPLE_RATE_CHANGED                  2
#define META_EVENT_POWER_MODE_CHANGED                   3
#define META_EVENT_ERROR                                4
#define META_EVENT_ALGORITHM_EVENT                      5
#define META_EVENT_CAL_STATUS_CHANGED                   6
#define META_EVENT_SENSOR_EVENT                         11
#define META_EVENT_FIFO_OVERFLOW                        12
#define META_EVENT_DYNAMIC_RANGE_CHANGED                13
#define META_EVENT_FIFO_WATERMARK                       14
#define META_EVENT_SELF_TEST_RESULT                     15
#define META_EVENT_INITIALIZED                          16
#define META_EVENT_TRANSFER_CAUSE                       17

//=========================================================================
// Parameter structures
//=========================================================================

typedef struct 
{
    u16 sampleRate;
    u16 maxReportLatency;
    u16 changeSensitivity;
    u16 dynamicRange;
} SensorConfiguration;


typedef struct
{
    u8 dataAvailable : 1;
    u8 i2cNack : 1;
    u8 deviceIdError : 1;
    u8 transientError : 1;
    u8 dataLost : 1;
    u8 powerMode : 3;
} SensorStatus;

typedef struct
{
    u16 sampleRate;
    u16 dynamicRange;
    SensorStatus status;
} PhysicalSensorStatus;

static struct 
{
    PhysicalSensorStatus mag;
    PhysicalSensorStatus accel;
    PhysicalSensorStatus gyro;
} physicalSensorStatus;

typedef struct
{
    u8 sensorId;
    u8 driverId;
    u8 driverVersion;
    u8 power;
    u16 maxRange;
    u16 resolution;
    u16 maxRate;
    u16 fifoReserved;
    u16 fifoMax;
    u8 eventSize;
    u8 minRate;
} SensorDescriptor;


typedef struct
{
    float dipAngle;
    float magRadius;
    u8 gBiasMode;
    float gyroThresh[6];
    float noiseLevls[9];
    float mcalSIHI[6];
    float accelScale[3];
    float accelOffsets[3];
    float gyroScale[3];
    float gyroOffsets[3];
    u8 mcalStatus;
    u8 bgCalMode;
    float calScore;
    float amatrix[9];
    float aoffsets[3];
} WarmStartParams;

static ParamInfo warmStartList[] =
{
    { 1, 4 },
    { 2, 4 },
    { 3, 1 },
    { 4, 8 },
    { 5, 8 },
    { 6, 8 },
    { 7, 8 },
    { 8, 8 },
    { 9, 8 },
    { 10, 8 },
    { 11, 4 },
    { 12, 8 },
    { 13, 8 },
    { 14, 8 },
    { 15, 8 },
    { 16, 8 },
    { 17, 8 },
    { 18, 8 },
    { 19, 8 },
    { 20, 8 },
    { 27, 1 },
    { 28, 1 },
    { 29, 4 },
    { 37, 8 },
    { 38, 8 },
    { 39, 8 },
    { 40, 8 },
    { 41, 4 },
    { 42, 8 },
    { 43, 4 }
};



typedef struct
{
    s16 x;
    s16 y;
    s16 z;
    u8 status;
} SensorData3AxisRaw;


typedef struct
{
    float x;
    float y;
    float z;
    float extra;
} SensorData3Axis;


typedef struct
{
    s16 x;
    s16 y;
    s16 z;
    s16 x_bias;
    s16 y_bias;
    s16 z_bias;
    u8 status;
} SensorData3AxisUncalRaw;


typedef struct
{
    float x;
    float y;
    float z;
    float x_bias;
    float y_bias;
    float z_bias;
    float extra;
} SensorData6Axis;


typedef struct
{
    s16 x;
    s16 y;
    s16 z;
    s16 w;
    s16 accuracy;
} RotationVectorRaw;


typedef struct
{
    float x;
    float y;
    float z;
    float w;
    float extra;
} SensorData4Axis;


typedef struct
{
    s16 dataum;
} SensorDataS16Raw;

typedef struct
{
    float datum;
} SensorDataS16;

typedef struct
{
    u16 datum;
} SensorDataU16Raw;

typedef struct
{
    float datum;
} SensorDataU16;

typedef struct
{
    u16 datum;
} SensorDataCount;


typedef struct
{
    u8 datum[3];
} SensorDataU24Raw;

typedef struct
{
    float datum;
} SensorDataU24;

// Data Queue Structs

typedef struct
{
    u8 ID;
    u32 timestamp;
    u8 size;
} DataPacket0Axis;


typedef struct
{
    u8 ID;
    u32 timestamp;
    u8 size;
    float x;
} DataPacket1Axis;


typedef struct
{
    u8 ID;
    u32 timestamp;
    u8 size;
    float x;
    float y;
    float z;
    float extra;
} DataPacket3Axis;


typedef struct
{
    u8 ID;
    u32 timestamp;
    u8 size;
    float x;
    float y;
    float z;
    float w;
    float extra;
} DataPacket4Axis;


typedef struct
{
    u8 ID;
    u32 timestamp;
    u8 size;
    float x;
    float y;
    float z;
    float x_bias;
    float y_bias;
    float z_bias;
    float extra;
} DataPacket6Axis;



const static char *em7186_meta_event_name[18] =
{
    "META EVENT NOOP", //0
    "META_EVENT_FLUSH_COMPLETE",                      // 1
    "META_EVENT_SAMPLE_RATE_CHANGED",                 // 2
    "META_EVENT_POWER_MODE_CHANGED",                  // 3
    "META_EVENT_ERROR",                               // 4
    "META_EVENT_ALGORITHM_EVENT",                     // 5
    "META_EVENT_CAL_STATUS_CHANGED",                  // 6
    "Undefined Meta Event",                           // 7
    "Undefined Meta Event",                           // 8
    "Undefined Meta Event",                           // 9
    "Undefined Meta Event",                           // 10
    "META_EVENT_SENSOR_EVENT",                        // 11
    "META_EVENT_FIFO_OVERFLOW",                       // 12
    "META_EVENT_DYNAMIC_RANGE_CHANGED",               // 13
    "META_EVENT_FIFO_WATERMARK",                      // 14
    "META_EVENT_SELF_TEST_RESULT",                    // 15
    "META_EVENT_INITIALIZED",                         // 16
    "META_EVENT_TRANSFER_CAUSE",                      // 17
};



const static char *em7186_sensor_name[128] =
{
    "na",                                       // 0
    "accelerometer",                            // 1
    "magnetic field",                           // 2
    "Orientation",                              // 3
    "gyroscope",                                // 4
    "light",                                    // 5
    "pressure",                                 // 6
    "temperature",                              // 7
    "proximity",                                // 8
    "gravity",                                  // 9
    "linear acceleration",                      // 10
    "rotation vector",                          // 11
    "relative humidity",                        // 12
    "ambient temperature",                      // 13
    "magnetic field uncalibrated",              // 14
    "game rotation vector",                     // 15
    "gyroscope uncalibrated",                   // 16
    "significant motion",                       // 17
    "step detector",                            // 18
    "step counter",                             // 19
    "geomagnetic rotation vector",              // 20
    "Heart rate -OR- Car Detector",             // 21
    "tilt detector",                            // 22
    "wake gesture",                             // 23
    "glance gesture",                           // 24
    "pick up gesture",                          // 25
    "custom_26",                                // 26
    "custom_27",                                // 27
    "Raw Accel",                                // 28
    "Raw Mag",                                  // 29
    "Raw Gyro",                                 // 30
    "activity",                                 // 31
    "Car Detect Mag Data(uT)",                  // 32
    "custom_33",                                // 33
    "custom_34",                                // 34
    "custom_35",                                // 35
    "custom_36",                                // 36
    "custom_37",                                // 37
    "custom_38",                                // 38
    "custom_39",                                // 39
    "custom_40",                                // 40
    "custom_41",                                // 41
    "custom_42",                                // 42
    "custom_43",                                // 43
    "custom_44",                                // 44
    "custom_45",                                // 45
    "custom_46",                                // 46
    "custom_47",                                // 47
    "custom_48",                                // 48
    "custom_49",                                // 49
    "custom_50",                                // 50
    "custom_51",                                // 51
    "custom_52",                                // 52
    "custom_53",                                // 53
    "custom_54",                                // 54
    "custom_55",                                // 55
    "custom_56",                                // 56
    "custom_57",                                // 57
    "custom_58",                                // 58
    "custom_59",                                // 59
    "custom_60",                                // 60
    "custom_61",                                // 61
    "custom_62",                                // 62
    "custom_63",                                // 63
    "reserved",                                 // 64
    "accelerometer wake",                       // 65
    "magnetic field wake",                      // 66
    "orientation wake",                         // 67
    "gyroscope wake",                           // 68
    "light wake",                               // 69
    "pressure wake",                            // 70
    "temperature wake",                         // 71
    "proximity wake",                           // 72
    "gravity wake",                             // 73
    "linear acceleration wake",                 // 74
    "rotation vector wake",                     // 75
    "relative humidity wake",                   // 76
    "ambient temperature wake",                 // 77
    "magnetic field uncalibrated wake",         // 78
    "game rotation vector wake",                // 79
    "gyroscope uncalibrated wake",              // 80
    "significant motion wake",                  // 81
    "step detector wake.",                      // 82
    "step counter wake",                        // 83
    "geomagnetic rotation vector wake",         // 84
    "Car Detect wake",                          // 85
    "tilt detector wake",                       // 86
    "wake gesture wake",                        // 87
    "glance gesture wake",                      // 88
    "pick up gesture wake",                     // 89
    "custom_26 wake",                           // 90
    "custom_27 wake",                           // 91
    "custom_28 wake",                           // 92
    "Raw Mag wake",                             // 93
    "custom_30 wake",                           // 94
    "activity wake",                            // 95
    "Car Detect Mag Data(uT) wake",             // 96
    "custom_33 wake",                           // 97
    "custom_34 wake",                           // 98
    "custom_35 wake",                           // 99
    "custom_36 wake",                           // 100
    "custom_37 wake",                           // 101
    "custom_38 wake",                           // 102
    "custom_39 wake",                           // 103
    "custom_40 wake",                           // 104
    "custom_41 wake",                           // 105
    "custom_42 wake",                           // 106
    "custom_43 wake",                           // 107
    "custom_44 wake",                           // 108
    "custom_45 wake",                           // 109
    "custom_46 wake",                           // 110
    "custom_47 wake",                           // 111
    "custom_48 wake",                           // 112
    "custom_49 wake",                           // 113
    "custom_50 wake",                           // 114
    "custom_51 wake",                           // 115
    "custom_52 wake",                           // 116
    "custom_53 wake",                           // 117
    "custom_54 wake",                           // 118
    "custom_55 wake",                           // 119
    "custom_56 wake",                           // 120
    "custom_57 wake",                           // 121
    "custom_58 wake",                           // 122
    "custom_59 wake",                           // 123
    "custom_60 wake",                           // 124
    "custom_61 wake",                           // 125
    "custom_62 wake",                           // 126
    "custom_63 wake"                            // 127
};



#endif