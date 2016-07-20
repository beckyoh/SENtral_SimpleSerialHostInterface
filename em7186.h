/**
* @file         em7186.h
*
* @brief        Sample interface for the em7186.
*
* @authors      David Vincent
* @date         04/01/2015
* @copyright    (C) 2015 PNI Corp
*
* @copyright    Disclosure to third parties or reproduction in any form
*               whatsoever, without prior written consent, is strictly forbidden
*
*/

#ifndef EM7186_H
#define EM7186_H

#include "main.h"

extern float em7186_sensor_scale[128];

extern u32 timestampNonWake;
extern u32 timestampWake;
extern u8 printData;
extern u8 logData;
extern u8 sensorEnabled[64];


//=============================================================================
// Hardware specific functions
//=============================================================================


/**
* Initialize the i2c interface. Implementation is hardware specific.
*
* @return 1 if initialization succeeded, 0 otherwise.
*/
u32 em7186_i2c_init();
/**
* Initialize the i2c interface indicated by the device id. Implementation is hardware specific.
*
* @param deviceId the unique device id for the I2C interface hardware.
* @return 1 if initialization succeeded, 0 otherwise.
*/
u32 em7186_i2c_init_by_id(s32 deviceId);
/**
* Read a block of data from the i2c bus. Implementation is hardware specific.
* 
* @param registerAddress The address of the first register in the block.
* @param[out] buffer The data.
* @param length The number of bytes to read.
* @return The number of bytes read.
*/
u32 em7186_i2c_read(u8 registerAddress, char* buffer, u16 length);
/**
* Write a block of data to the i2c bus. Implementation is hardware specific.
*
* @param registerAddress The address of the first register in the block.
* @param buffer The data to write.
* @param length The number of bytes to write.
* @return The number of bytes written.
*/
u32 em7186_i2c_write(u8 registerAddress, char* buffer, u16 length);
/**
* Write a single byte to the i2c bus. Implementation is hardware specific.
*
* @param registerAddress The register to write.
* @param value The data to write.
* @return 1 if the write succeeded, 0 otherwise.
*/
u32 em7186_i2c_write_value(u8 registerAddress, char value);
/**
* Check for an interrupt from the em7186. Implementation is hardware specific.
* 
* @return 1 if the interrupt line is high, 0 otherwise.
*/
u8 em7186_interrupt();
/**
* Set the sensor scale factors. Some of these values could change depending on 
* the hardware being used.
*
* @return 1 on success, 0 on failure.
*/
u8 em7186_set_scale_factors();
/**
* Save a set of warm-start coefficients to file.
*
* @see em7186_warm_start_load
* @param filename The name of the file to create.
* @return 1 on success, 0 on failure.
*/
//u32 em7186_warm_start_save(const char *filename);
/**
* Load a set of warm-start coefficients from file.
*
* @see em7186_warm_start_save
* @param filename The name of the file to load.
* @return 1 on success, 0 on failure.
*/
//u32 em7186_warm_start_load(const char *filename);

//=============================================================================
// Core functions
//=============================================================================
/**

* Upload firmware. Resets the CPU and uploads firmware.
*
* @param filename The name of the firmware file to upload.
* @return 1 on success, 0 otherwise.
*/
u32 em7186_firmware_Transfer2RAM(const char *filename);
/**
* Read a set of parameters. Initiate the transfer of a set of 
* parameters on the same parameter page.
*
* @param[out] values, The values of the parameters that have been read.
* @param page, The page to read values from.
* @param params, The list of parameters to read.
* @param numParams, The number of parameters to read.
* @return, The number of parameters that were read.
*/
u32 em7186_param_read(char *values, u8 page, ParamInfo *params, u8 numParams);
/**
* Write a set of parameters. Initiate the transfer of a set of
* parameters on the same parameter page.
*
* @param values, The values to write to the specified parameters.
* @param page, The page to write values to.
* @param params, The list of parameters to write.
* @param numParams, The number of parameters to write.
* @return, The number of parameters that were written.
*/
u32 em7186_param_write(char *values, u8 page, ParamInfo *params, u8 numParams);
/**
* Read the contents of the fifo. Note that the size of the buffer must be at
* least as large as the space available for the fifo.
*
* @param[out] buffer The contents of the fifo.
* @return The number of bytes read from the fifo.
*/
u32 em7186_read_fifo(char *buffer);
/**
* Parse the fifo buffer.
*
* @param buffer The contents of the fifo.
* @param size The number of bytes in the fifo buffer.
* @return The number of bytes parsed.
*/
u32 em7186_parse_fifo(char* buffer, u32 size);
/**
* Set a sensor output rate.
*
* @param sensorId The id of the sensor.
* @param rate The desired rate in Hz.
* @return 1 on success, 0 on failure.
*/
u32 em7186_set_sensor_rate(u8 sensorId, u16 rate);
/**
* Set a sensor maximum report latency.
*
* @param sensorId The id of the sensor.
* @param rate The desired latency in milliseconds.
* @return 1 on success, 0 on failure.
*/
u32 em7186_set_sensor_delay(u8 sensorId, u16 delay);

//=============================================================================
// Additional Control Functions
//=============================================================================
/**
* Set fifo watermark values. The fifo watermark can be used to wake the host
* before data is lost from the fifo. Default behavior is to allow the non-wake
* fifo to overflow while the host is in suspend mode and for the wake fifo to
* wake the host before data is lost. This function sets the minimum number of
* bytes remaining in the fifo before a host interrupt is issued.
*
* Note that the watermark values used in this function differ from those in
* the documentation.
* 
* @param wakeFifoWatermark The minimum number of bytes remaining in the wake 
*    fifo before interrupting the host.
* @param nonWakeFifoWatermark The minimum number of bytes remaining in the 
*    non-wake fifo before interrupting the host.
* @return 1 on success, 0 on failure.
*/
u32 em7186_set_fifo_watermarks(u16 wakeFifoWatermark, u16 nonWakeFifoWatermark);
/**
* Change meta-event settings. Enable or disable individual meta-events and
* their interrupts.
*
* @param eventId The id of the meta-event.
* @param enable True to enable the meta-event.
* @param enableInt True to cause meta-event to interrupt the host.
* @return 1 on success, 0 on failure.
*/
u32 em7186_set_meta_event(u8 eventId, u8 enable, u8 enableInt);
/**
* Enter or exit ap suspend mode. AP suspend mode changes the interrupt behavior
* of the em7186. While in AP suspend mode only wake events will interrupt the
* host.
*
* @param suspend True to enter suspend mode, False to exit suspend mode.
* @return 1 on success, 0 on failure.
*/
u32 em7186_ap_suspend(u8 suspend);
/**
* Flush data from a fifo. Flush can be for an individual sensor or an entire
* fifo.
*
* @see FIFO_FLUSH_DISCARD_NON_WAKE
* @see FIFO_FLUSH_DISCARD_WAKE
* @see FIFO_FLUSH_TRANSFER_NON_WAKE
* @see FIFO_FLUSH_TRANSFER_WAKE
* @see FIFO_FLUSH_DISCARD_ALL
* @see FIFO_FLUSH_TRANSFER_ALL
*
* @param sensorId The id of the sensor to flush, or a special flush command.
* @return 1 on success, 0 on failure.
*/
u32 em7186_flush_sensor(u8 sensorId);
/**
* Enable raw sensor data for calibration or debugging.
*
* @param emableMag True to enable the magnetometer debug output.
* @param emableAccel True to enable the accelerometer debug output.
* @param emableGyro True to enable the gyroscope debug output.
* @return 1 on success, 0 on failure.
*/
u32 em7186_enable_raw_sensors(u8 enableMag, u8 enableAccel, u8 enableGyro);

void displayParams(u8 paramPage, ParamInfo *paramList, u8 numParams);
//void warmStart();
void displayStatusRegisters();
void displaySensorStatus();
void displayPhysicalSensorInformation();
void displayPhysicalSensorStatus();
void getSensorInformation();
void getSensorConfiguration();
void displaySensorConfiguration();
void displaySensorInformation();
void firmwareTransfer(char srcDestCode);
u32 em7186_firmware_Transfer2EE(const u8 *firmwareName);
u32 EE_Write(u8 I2C_Addr, u16 EE_MemAddr, u8*buffer, u16 length);
void executeSerialFunction(unsigned char key);



#endif //EM7186_H
