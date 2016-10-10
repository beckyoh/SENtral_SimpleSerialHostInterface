/**
* @file         em7186.c
*
* @brief        Sample interface for the em7186.
*
* @authors      David Vincent, Joe Miller
* @date         05/12/2016
* @copyright    (C) 2015, 2016 PNI Corp
*
* @copyright    Disclosure to third parties or reproduction in any form
*               whatsoever, without prior written consent, is strictly forbidden
*
*/
#include "em7186.h"


float   em7186_sensor_scale[128];
u32     timestampNonWake;
u32     timestampWake;
u8      printData = 1; // a switch to enable/disable display data (vs logData)
u8      logData = 0;   // a switch to enable/disable log data to SD Card 
u8      sensorEnabled[64];
u8      haveSensorInfo = 0;
u16     magMaxRate = 0;
u16     accelMaxRate = 0;
u16     gyroMaxRate = 0;
u32     timestamp;
u16     timestampPtr[2];

SensorDescriptor sensorInformation[128];
SensorConfiguration sensorConfiguration[127];


ParamInfo sensorInfoParamList[128] =
{
    { 0, 16 },
    { 1, 16 },
    { 2, 16 },
    { 3, 16 },
    { 4, 16 },
    { 5, 16 },
    { 6, 16 },
    { 7, 16 },
    { 8, 16 },
    { 9, 16 },
    { 10, 16 },
    { 11, 16 },
    { 12, 16 },
    { 13, 16 },
    { 14, 16 },
    { 15, 16 },
    { 16, 16 },
    { 17, 16 },
    { 18, 16 },
    { 19, 16 },
    { 20, 16 },
    { 21, 16 },
    { 22, 16 },
    { 23, 16 },
    { 24, 16 },
    { 25, 16 },
    { 26, 16 },
    { 27, 16 },
    { 28, 16 },
    { 29, 16 },
    { 30, 16 },
    { 31, 16 },
    { 32, 16 },
    { 33, 16 },
    { 34, 16 },
    { 35, 16 },
    { 36, 16 },
    { 37, 16 },
    { 38, 16 },
    { 39, 16 },
    { 40, 16 },
    { 41, 16 },
    { 42, 16 },
    { 43, 16 },
    { 44, 16 },
    { 45, 16 },
    { 46, 16 },
    { 47, 16 },
    { 48, 16 },
    { 49, 16 },
    { 50, 16 },
    { 51, 16 },
    { 52, 16 },
    { 53, 16 },
    { 54, 16 },
    { 55, 16 },
    { 56, 16 },
    { 57, 16 },
    { 58, 16 },
    { 59, 16 },
    { 60, 16 },
    { 61, 16 },
    { 62, 16 },
    { 63, 16 },
    { 64, 16 },
    { 65, 16 },
    { 66, 16 },
    { 67, 16 },
    { 68, 16 },
    { 69, 16 },
    { 70, 16 },
    { 71, 16 },
    { 72, 16 },
    { 73, 16 },
    { 74, 16 },
    { 75, 16 },
    { 76, 16 },
    { 77, 16 },
    { 78, 16 },
    { 79, 16 },
    { 80, 16 },
    { 81, 16 },
    { 82, 16 },
    { 83, 16 },
    { 84, 16 },
    { 85, 16 },
    { 86, 16 },
    { 87, 16 },
    { 88, 16 },
    { 89, 16 },
    { 90, 16 },
    { 91, 16 },
    { 92, 16 },
    { 93, 16 },
    { 94, 16 },
    { 95, 16 },
    { 96, 16 },
    { 97, 16 },
    { 98, 16 },
    { 99, 16 },
    { 100, 16 },
    { 101, 16 },
    { 102, 16 },
    { 103, 16 },
    { 104, 16 },
    { 105, 16 },
    { 106, 16 },
    { 107, 16 },
    { 108, 16 },
    { 109, 16 },
    { 110, 16 },
    { 111, 16 },
    { 112, 16 },
    { 113, 16 },
    { 114, 16 },
    { 115, 16 },
    { 116, 16 },
    { 117, 16 },
    { 118, 16 },
    { 119, 16 },
    { 120, 16 },
    { 121, 16 },
    { 122, 16 },
    { 123, 16 },
    { 124, 16 },
    { 125, 16 },
    { 126, 16 },
    { 127, 16 },
};


u32 em7186_i2c_init()
{
    u8 buffer[1];
    em7186_i2c_read(PRODUCT_ID_REG, buffer, 1);
    
    switch(buffer[0])
    {
        case PRODUCT_ID_7180:
            printf("SENtral found\n\r");
            break;   
        case PRODUCT_ID_7184:
            printf("SENtral-A found\n\r");        
            break;   
        case PRODUCT_ID_7186:
            printf("SENtral-A2 found\n\r");        
            break;   
        default:
            printf("SENtral NOT FOUND, ID returned = %u\n\r",buffer[0]);
            return 0;
  
     }  
    return 1;
}

void firmwareTransfer(char srcDestCode)
{
    switch (srcDestCode) {
//        case 'r':
//            if (displayText) printf("begin Upload of firmware to  SENtral RAM\n\r");
//            SENtral_InterruptPin.disable_irq();
//            if (!) {
//                if (displayText) printf("Error uploading SENtral firmware to RAM\n\r");
//            }
//            SENtral_InterruptPin.enable_irq();
//            break;
//            
//        case 'e':
//            if (displayText) printf("begin Upload of firmware to PNI Module EEPROM\n\r");
//            SENtral_InterruptPin.disable_irq();
//            if (!) {
//                if (displayText) printf("Error uploading SENtral firmware to EEPROM\n\r");
//            }
//            SENtral_InterruptPin.enable_irq();
//            break;
//            
//        case 's':
//            if (displayText) printf("begin Upload of firmware to SDCard\n\r");
//            SENtral_InterruptPin.disable_irq();
//            if (!) {
//                if (displayText) printf("Error uploading SENtral firmware to SDCard\n\r");
//            }
//            SENtral_InterruptPin.enable_irq();
//            break;
            
        case 'R':
            if (displayText) printf("Transfering firmware from SDCard to SENtral RAM\n\r");
            SENtral_InterruptPin.disable_irq();
            if (!em7186_firmware_Transfer2RAM(fw)) {
                if (displayText) printf("Error transfering SENtral firmware to RAM\n\r");
                break;
            }
            // Enable CPU
            em7186_i2c_write_value(CHIP_CONTROL_REG, CHIP_CONTROL_CPU_RUN);        
            em7186_set_scale_factors();
            SENtral_InterruptPin.enable_irq();
            break;
            
        case 'E':
            if (displayText) printf("Transfering firmware from SDCard to PNI Module EEPROM\n\r");
            SENtral_InterruptPin.disable_irq();
            if (!em7186_firmware_Transfer2EE(fw)) {
                if (displayText) printf("Error transfering SENtral firmware to EEPROM\n\r");
            }
            break;

        default:
            serialCommandMode = 0;

    }


}




u32 em7186_i2c_write_value(u8 registerAddress, u8 value)
{
    u32 status = em7186_i2c_write(registerAddress, &value, 1);
    return status;
}


//=========================================================================
// Helper functions
//=========================================================================
u8  get_3_axis_sensor_data(SensorData3Axis *data, float scale, u8* buffer)
{
    SensorData3AxisRaw rawData;
    memcpy(&rawData, &buffer[1], sizeof(rawData));
    data->x = (float)rawData.x * scale;
    data->y = (float)rawData.y * scale;
    data->z = (float)rawData.z * scale;
    data->extra = rawData.status;

    return 1;
}
u8  get_3_axis_uncal_sensor_data(SensorData6Axis *data, float scale, u8* buffer)
{
    SensorData3AxisUncalRaw rawData;
    memcpy(&rawData, &buffer[1], sizeof(rawData));
    data->x = (float)rawData.x * scale;
    data->y = (float)rawData.y * scale;
    data->z = (float)rawData.z * scale;
    data->x_bias = (float)rawData.x_bias * scale;
    data->y_bias = (float)rawData.y_bias * scale;
    data->z_bias = (float)rawData.z_bias * scale;
    data->extra = rawData.status;

    return 1;
}
u8  get_rotation_vector(SensorData4Axis *rv, float quaternionScale, u8* buffer)
{
    RotationVectorRaw rawData;
    memcpy(&rawData, &buffer[1], sizeof(rawData));
    rv->x = (float)rawData.x * quaternionScale;
    rv->y = (float)rawData.y * quaternionScale;
    rv->z = (float)rawData.z * quaternionScale;
    rv->w = (float)rawData.w * quaternionScale;
    rv->extra = (float)rawData.accuracy * ((float)M_PI / powf(2.0f, 14.0f));

    return 1;
}

//=========================================================================
// Core functions
//=========================================================================
u32 em7186_firmware_Transfer2RAM(const u8 *firmwareName)
{
   
    // reset Sentral
    em7186_i2c_write_value(RESET_REQ_REG, 1);
    em7186_i2c_write_value(CHIP_CONTROL_REG, CHIP_CONTROL_HOST_UPLOAD);

    ///////////////////////////////////////////////////////////////////////////
    // Load firmware from file
    ///////////////////////////////////////////////////////////////////////////
    // Open firmware file
    FILE *fw_h = fopen(firmwareName, "rb");
    if (!fw_h)
    {
        if (displayText) printf("ERROR: Unable to open file\n\r");
        return 0;
    }
    
    if (displayText) printf("Firmware file found\n\r");

    // Read the firmware header
    struct fwHeader
    {
        u8 imageSignatureLsb;
        u8 imageSignatureMsb;
        u16 flags;
        u32 crc;
        u32 reserved;
        u16 imageLength;
        u16 reserved2;
    } fwHeader;
    int hsize = fread(&fwHeader, 1, FIRMWARE_HEADER_SIZE, fw_h);
    if ( hsize != FIRMWARE_HEADER_SIZE)
    {
        if (displayText) printf("ERROR: File smaller than expected header size %d\n\r", hsize);
        fclose(fw_h);
        return 0;
    }

    // Validate firmware
    if (fwHeader.imageSignatureLsb != IMAGE_SIGNATURE_LSB || fwHeader.imageSignatureMsb != IMAGE_SIGNATURE_MSG)
    {
        if (displayText) printf("ERROR: Firmware version doesn't match\n\r");
        fclose(fw_h);
        return 0;
    }

    // TODO: ensure that firmware version matches rom version

    // Read the firmware image
    u8 *fw = (u8 *)malloc(fwHeader.imageLength);
    u32 firmwareSize = fread(fw, 1, fwHeader.imageLength, fw_h);
    fclose(fw_h);
    if (firmwareSize != fwHeader.imageLength || firmwareSize % 4)
    {
        if (displayText) printf("ERROR: Firmware size must break on 4 byte boundary\n\r");
        free(fw);
        return 0;
    }

    ///////////////////////////////////////////////////////////////////////////
    // Upload firmware to RAM
    ///////////////////////////////////////////////////////////////////////////
    // TODO: set upload address if needed. (zero by default)

    if (displayText) printf("Uploading Firmware to SENtral RAM...\n\r");

    s32 bytesWritten = 0,
        bytesRemaining = firmwareSize;
    u8 bytesToWrite, i,
        buf[MAX_I2C_WRITE],
        maxI2cWrite = MAX_I2C_WRITE;

    while (bytesRemaining > 0)
    {
        if (bytesRemaining < MAX_I2C_WRITE)      
            bytesToWrite = bytesRemaining;
        else
            bytesToWrite = maxI2cWrite;

        // Reverse byte order per word
        for (i = 0; i < bytesToWrite; i += 4)
        {
            buf[i + 0] = fw[bytesWritten + i + 3];
            buf[i + 1] = fw[bytesWritten + i + 2];
            buf[i + 2] = fw[bytesWritten + i + 1];
            buf[i + 3] = fw[bytesWritten + i + 0];
        }

        if (!em7186_i2c_write(SR_UPLOAD_DATA_REG, buf, bytesToWrite))
        {
            if (displayText) printf("ERROR: Problem writing to sentral\n\r");
            free(fw);
            return 0;
        }

        bytesRemaining -= bytesToWrite;
        bytesWritten += bytesToWrite;
    }

    free(fw);

    if (displayText) printf("Firmware Uploaded......");

    // Read and verify CRC
    u32 hostCRC = 0;
    em7186_i2c_read(HOST_CRC_REG, (u8*)&hostCRC, 4);
    u32 fwHeaderCRC = fwHeader.crc;//((u32*)fwHeader)[1];
    if (hostCRC != fwHeaderCRC)
    {
        if (displayText) printf("ERROR: CRC doesn't match\n\r");
        return 0;
    }
    
    if (displayText) printf(" CRC Match!!\n\r");
    return 1;
}




u32 em7186_firmware_Transfer2EE(const u8 *firmwareName)
{
    u8  buffer[16];
    u8  trys = 1;
    u16 count;
    // Open firmware file
    FILE *fw_h = fopen(firmwareName, "rb");
    if (!fw_h) {
        if (displayText) printf("ERROR: Unable to open file\n\r");
        return 0;
    }

    if (displayText) printf("Firmware file found\n\r");

    // Read the firmware header
    struct fwHeader {
        u8 imageSignatureLsb;
        u8 imageSignatureMsb;
        u16 flags;
        u32 crc;
        u32 reserved;
        u16 imageLength;
        u16 reserved2;
    } fwHeader;

    int hsize = fread(&fwHeader, 1, FIRMWARE_HEADER_SIZE, fw_h);
    if ( hsize != FIRMWARE_HEADER_SIZE) {
        if (displayText) printf("ERROR: File smaller than expected header size %d\n\r", hsize);
        fclose(fw_h);
        return 0;
    }

    // Validate firmware
    if (fwHeader.imageSignatureLsb != IMAGE_SIGNATURE_LSB || fwHeader.imageSignatureMsb != IMAGE_SIGNATURE_MSG) {
        if (displayText) printf("ERROR: Firmware version doesn't match\n\r");
        fclose(fw_h);
        return 0;
    }

    // Read the firmware image From the SD Card
    u8 *fw = (u8 *)malloc(fwHeader.imageLength);
    u32 firmwareSize = fread(fw, 1, fwHeader.imageLength, fw_h);
    fclose(fw_h);
    if (firmwareSize != fwHeader.imageLength || firmwareSize % 4) {
        if (displayText) printf("ERROR: Firmware size must break on 4 byte boundary\n\r");
        free(fw);
        return 0;
    }
    if (displayText) printf("Firmware size = %u\n\r",firmwareSize);


   u8 headerBytes[FIRMWARE_HEADER_SIZE];
   memcpy(headerBytes,(u8*)&fwHeader,FIRMWARE_HEADER_SIZE); // create a bytewise copy of fwHeader


    ///////////////////////////////////////////////////////////////////////////
    // Upload firmware to EEPROM
    ///////////////////////////////////////////////////////////////////////////

    do {
        // request SENtral passthrough mode
        em7186_i2c_write_value(PASS_THROUGH_CFG_REG, 1);
        em7186_i2c_read(PASS_THROUGH_RDY_REG, buffer, 1);
    } while((buffer[0] != 1) && (++trys < 20));


    if (trys <20) {

        // Reverse byte order per word


        if (displayText) printf("SENtral confirmed passthrough mode after %u try(s)\n\r",trys);


        // write header portion to EEPROM from SDCard
        //key: EE_Write(u8 I2C_Addr, u16 EE_MemAddr, u8*buffer, u16 length)
        u32 status = EE_Write(0xA0, 0, (u8*)&fwHeader, FIRMWARE_HEADER_SIZE);
        if (displayText) printf("Header Sent to EEPROM......\n\r",firmwareSize);
        wait_ms(5);
        EE_Read(0xA0, 0, buffer, FIRMWARE_HEADER_SIZE);
        status = 0;

        // Readback EEPROM to verify header write
        for (count = 0; count<FIRMWARE_HEADER_SIZE; count++) {
            if (headerBytes[count] != buffer[count]) {
                status = 1;  // 1 = fail
                if (displayText) printf("Failed Header readback from EEPROM at %u, value=%u\n\r",count,headerBytes[count]);
                break;
            }
        }

        if (!status) {
           u32 bytesWritten = 0;
            u32 bytesRemaining = firmwareSize;
            u8 bytesToWrite,
               maxI2cWrite = 16; // kind of small but ensures page alignment

            while (bytesRemaining > 0) {
                if (bytesRemaining < maxI2cWrite)
                    bytesToWrite = bytesRemaining;
                else
                    bytesToWrite = maxI2cWrite;

                if (!EE_Write(0xA0, (bytesWritten+FIRMWARE_HEADER_SIZE), &fw[bytesWritten], bytesToWrite)) {
                    if (displayText) printf("\n\rCould not write to EEPROM\n\r");
                    free(fw);
                    wait_ms(5);
                    em7186_i2c_write_value(PASS_THROUGH_CFG_REG, 0);
                    return 0;
                }
                bytesWritten += bytesToWrite;
                bytesRemaining -= bytesToWrite;
                if (displayText) printf("\r%u",bytesWritten);
                wait_ms(5);
            }

            if (displayText) printf("\n\rFirmware Transfered from SDCard to EEPROM.  Now Verifying.....\n\r");

            u32 bytesRead = 0;
            bytesRemaining = firmwareSize;
            u8 bytesToRead,
               maxI2cRead = 16;
            status = 0; 
            
            while ((bytesRemaining > 0) && (status == 0)) {
                if (bytesRemaining < maxI2cRead)
                    bytesToRead = bytesRemaining;
                else
                    bytesToRead = maxI2cRead;
                    
                if (!EE_Read(0xA0, (bytesRead+FIRMWARE_HEADER_SIZE), buffer, bytesToRead)) {
                    if (displayText) printf("\n\rCould not Read EEPROM\n\r");
                    free(fw);
                    wait_ms(5);
                    em7186_i2c_write_value(PASS_THROUGH_CFG_REG, 0);
                    return 0;
                }

                for (count = 0; count<bytesToRead; count++) {
                    if (fw[bytesRead+count] != buffer[count]) {
                        status = 1;  // 1 = fail
                        if (displayText) printf("Failed firmware readback from EEPROM at %u, value=%u\n\r",bytesRead+FIRMWARE_HEADER_SIZE+count,buffer[count]);
                        wait_ms(5);
                        em7186_i2c_write_value(PASS_THROUGH_CFG_REG, 0);
                        return 0;
                    }
                }
                bytesRead += bytesToRead;
                bytesRemaining -= bytesToRead;
                if (displayText) printf("\r%u",bytesRead);
                wait_ms(1);
            }
            if(status == 0)
            {
                if (displayText) printf("\n\rVerify EEPROM **** SUCCESSFUL *****\n\r");
            }
        } else {
            if (displayText) printf("\r\nCould not write to EEPROM (Header)\n\r");
            wait_ms(5);
            em7186_i2c_write_value(PASS_THROUGH_CFG_REG, 0);
            return 0;
        }

        free(fw);

    } else {
        if (displayText) printf("Could not confirm SENtral Passthrough mode\n\r");
        return 0;
    }

    em7186_i2c_write_value(PASS_THROUGH_CFG_REG, 0); // turn off passthrough mode

    return 1;
}





u32 em7186_param_read(u8 *values, u8 page, ParamInfo *paramList, u8 numParams)
{
    
    u8 i, paramAck, pageSelectValue;
    u16 valIndex = 0;
    for (i = 0; i < numParams; i++)
    {
        pageSelectValue = page | (paramList[i].size << 4);
        em7186_i2c_write_value(PARAM_PAGE_SELECT_REG, pageSelectValue);
        em7186_i2c_write_value(PARAM_REQUEST_REG, paramList[i].paramNo);
        do
        {
            em7186_i2c_read(PARAM_ACK_REG, &paramAck, 1);
            if (paramAck == 0x80)
            {
                em7186_i2c_write_value(PARAM_REQUEST_REG, 0);
                em7186_i2c_write_value(PARAM_PAGE_SELECT_REG, 0);
                

                return 0;
            }
        } while (paramAck != paramList[i].paramNo);
        em7186_i2c_read(PARAM_SAVE_REG, &values[valIndex], paramList[i].size);
//      printf("%u ", values[valIndex]);
        valIndex += paramList[i].size;
    }
    em7186_i2c_write_value(PARAM_REQUEST_REG, 0);
    em7186_i2c_write_value(PARAM_PAGE_SELECT_REG, 0);
    

    return 1;
}
u32 em7186_param_write(u8 *values, u8 page, ParamInfo *paramList, u8 numParams)
{
    

    u8 i, paramAck, paramNum, pageSelectValue;
    u16 valIndex = 0;
    for (i = 0; i < numParams; i++)
    {
        pageSelectValue = page | (paramList[i].size << 4);
        em7186_i2c_write_value(PARAM_PAGE_SELECT_REG, pageSelectValue);

        em7186_i2c_write(PARAM_LOAD_REG, &values[valIndex], (u16)paramList[i].size);

        paramNum = paramList[i].paramNo | 0x80;
        em7186_i2c_write_value(PARAM_REQUEST_REG, paramNum);
        do
        {
            em7186_i2c_read(PARAM_ACK_REG, &paramAck, 1);
            if (paramAck == 0x80)
            {
                em7186_i2c_write_value(PARAM_REQUEST_REG, 0);
                em7186_i2c_write_value(PARAM_PAGE_SELECT_REG, 0);
                

                return 0;
            }
        } while (paramAck != paramNum);

        valIndex += paramList[i].size;
    }

    em7186_i2c_write_value(PARAM_REQUEST_REG, 0);
    em7186_i2c_write_value(PARAM_PAGE_SELECT_REG, 0);
    

    return 1;
}
u32 em7186_read_fifo(u8 *buffer)
{
    // Check number of bytes available
    u16 bytesAvailable, bytesRead = 0;

    em7186_i2c_read(BYTES_REMANING_REG, (u8*)&bytesAvailable, 2);


    //printf("FIFO bytesAvailable:%u\n\r",bytesAvailable);

    

#define CONTINUOUS_FIFO_READ
#ifdef CONTINUOUS_FIFO_READ
    bytesRead = em7186_i2c_read(0, buffer, bytesAvailable);
#else
    u16 i, bytesToRead;
    while (bytesAvailable > 0)
    {
        // Break on 50 byte fifo register block
        bytesToRead = bytesRead % 50 + I2C_MAX_READ > 50 ? 50 - bytesRead % 50 : I2C_MAX_READ;
        // Make sure we don't read more than is available in the fifo
        bytesToRead = min(bytesAvailable, bytesToRead);
        if (!em7186_i2c_read(bytesRead % 50, &buffer[bytesRead], bytesToRead))
        {
            
            return 0;
        }
        bytesAvailable -= bytesToRead;
        bytesRead += bytesToRead;
    }
#endif
    
    //printf("FIFO bytesRead:%u\n\r",bytesRead);

    return bytesRead;
}
u32 em7186_parse_next_fifo_block(u8* buffer, u32 size)
{
    u8 sensorId = buffer[0];
    
//    if (sensorId < SENSOR_TYPE_ACCELEROMETER_WAKE ||
//        sensorId == SENSOR_TYPE_DEBUG ||
//        sensorId == SENSOR_TYPE_TIMESTAMP ||
//        sensorId == SENSOR_TYPE_TIMESTAMP_OVERFLOW ||
//        sensorId == SENSOR_TYPE_META ||
//        sensorId == SENSOR_TYPE_RAW_GYRO ||
//        sensorId == SENSOR_TYPE_RAW_MAG ||
//        sensorId == SENSOR_TYPE_RAW_ACCEL
//        )
//    {
//     }
//    else
//    {
//        timestamp = timestampWake;
//        timestampPtr = (u16*)&timestampWake;
//    }

    switch(sensorId)
    {
        case 0:
        {
            //printf("Padding: %d\n\r", size);
            return size;
        }
        case SENSOR_TYPE_ACCELEROMETER:
        case SENSOR_TYPE_ACCELEROMETER_WAKE:
        case SENSOR_TYPE_MAGNETIC_FIELD:
        case SENSOR_TYPE_MAGNETIC_FIELD_WAKE:
        case SENSOR_TYPE_GYROSCOPE:
        case SENSOR_TYPE_GYROSCOPE_WAKE:
        case SENSOR_TYPE_GRAVITY:
        case SENSOR_TYPE_GRAVITY_WAKE:
        case SENSOR_TYPE_LINEAR_ACCELERATION:
        case SENSOR_TYPE_LINEAR_ACCELERATION_WAKE:
        case SENSOR_TYPE_ORIENTATION:
        case SENSOR_TYPE_ORIENTATION_WAKE:
        {
            SensorData3Axis sensorData;
            get_3_axis_sensor_data(&sensorData, em7186_sensor_scale[sensorId], buffer);
            if (printData) printf("%u %s: %f, %f, %f, %f\n\r", timestamp,em7186_sensor_name[sensorId], sensorData.x, sensorData.y, sensorData.z, sensorData.extra);
            if (logData) fprintf(flog,"%u,%u,%f,%f,%f,%f\n", timestamp,sensorId, sensorData.x, sensorData.y, sensorData.z, sensorData.extra);
            
            return 8;
        }
        case SENSOR_TYPE_LIGHT:
        case SENSOR_TYPE_LIGHT_WAKE:
        case SENSOR_TYPE_PROXIMITY:
        case SENSOR_TYPE_PROXIMITY_WAKE:
        case SENSOR_TYPE_RELATIVE_HUMIDITY:
        case SENSOR_TYPE_RELATIVE_HUMIDITY_WAKE:
        case SENSOR_TYPE_ACTIVITY:
        case SENSOR_TYPE_ACTIVITY_WAKE:
        {
            float sensorData = (float)((buffer[2] << 8) + buffer[1]) * em7186_sensor_scale[sensorId];
            if (printData) printf("%u %s: %fLux\n\r", timestamp,em7186_sensor_name[sensorId], sensorData);
            if (logData) fprintf(flog,"%u,%u,%u,%f\n",timestamp, sensorId, sensorData);
            return 3;
        }
        case SENSOR_TYPE_PRESSURE:
        case SENSOR_TYPE_PRESSURE_WAKE:
        {
            float pressure = (float)((buffer[3] << 16) + (buffer[2] << 8) + buffer[1]) * em7186_sensor_scale[sensorId];
            if (printData) printf("%u %s: %fPa\n\r", timestamp, em7186_sensor_name[sensorId], pressure);
            if (logData) fprintf(flog,"%u,%u,%f\n",timestamp,sensorId,pressure);
            return 4;
        }
        case SENSOR_TYPE_TEMPERATURE:
        case SENSOR_TYPE_TEMPERATURE_WAKE:
        case SENSOR_TYPE_AMBIENT_TEMPERATURE:
        case SENSOR_TYPE_AMBIENT_TEMPERATURE_WAKE:
        {
            s16 *sensorData = (s16*)&buffer[1];
            float temp = (float)(sensorData[0]) * em7186_sensor_scale[sensorId];
            if (printData) printf("%u %s: %fC\n\r", timestamp, em7186_sensor_name[sensorId], temp);
            if (logData) fprintf(flog,"%u,%u,%f\n",timestamp,sensorId, temp);
            return 3;
        }
        case SENSOR_TYPE_ROTATION_VECTOR:
        case SENSOR_TYPE_ROTATION_VECTOR_WAKE:
        case SENSOR_TYPE_GAME_ROTATION_VECTOR:
        case SENSOR_TYPE_GAME_ROTATION_VECTOR_WAKE:
        case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
        case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR_WAKE:
        case SENSOR_TYPE_PDR:
        case SENSOR_TYPE_PDR_WAKE:
        case 60:
        {
            SensorData4Axis rotationVector;
            get_rotation_vector(&rotationVector, em7186_sensor_scale[sensorId], buffer);
            if (printData) printf("%u %s: %f, %f, %f, %f, %f\n\r", timestamp, em7186_sensor_name[sensorId], rotationVector.x, rotationVector.y, rotationVector.z, rotationVector.w, rotationVector.extra);
            if (logData) fprintf(flog,"%u,%u,%f,%f,%f,%f,%f\n", timestamp,sensorId, rotationVector.x, rotationVector.y, rotationVector.z, rotationVector.w, rotationVector.extra);
            return 11;
        }
        case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
        case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED_WAKE:
        case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
        case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED_WAKE:
        {
            SensorData6Axis sensorDataUncal;
            get_3_axis_uncal_sensor_data(&sensorDataUncal, em7186_sensor_scale[sensorId], buffer);
            if (printData) printf("%u %s: %f, %f, %f, %f, %f, %f, %f\n\r", timestamp, em7186_sensor_name[sensorId], sensorDataUncal.x, sensorDataUncal.y, sensorDataUncal.z, sensorDataUncal.x_bias, sensorDataUncal.y_bias, sensorDataUncal.z_bias, sensorDataUncal.extra);
            if (logData) fprintf(flog,"%u,%u,%f,%f,%f,%f,%f,%f,%f\n", timestamp,sensorId, sensorDataUncal.x, sensorDataUncal.y, sensorDataUncal.z, sensorDataUncal.x_bias, sensorDataUncal.y_bias, sensorDataUncal.z_bias, sensorDataUncal.extra);
            return 14;
        }
        case SENSOR_TYPE_STEP_COUNTER:
        case SENSOR_TYPE_STEP_COUNTER_WAKE:
        {
            u16 sensorData = (buffer[2] << 8) + buffer[1];
            if (printData) printf("%u %s: %u\n\r", timestamp, em7186_sensor_name[sensorId], sensorData);
            if (logData) fprintf(flog,"%u,%u,%u\n", timestamp,sensorId, sensorData);
           return 3;
        }
        case SENSOR_TYPE_HEART_RATE:       
        case SENSOR_TYPE_HEART_RATE_WAKE:
        {
            u8 sensorData = buffer[1];
    
    // Heart Rate
            if (printData) printf("%u %s: %u\n\r", timestamp, em7186_sensor_name[sensorId], sensorData);
            if (logData) fprintf(flog,"%u,%u,%u\n",timestamp,sensorId, sensorData);
            return 3;
        }
        case SENSOR_TYPE_CAR_MAG_DATA:
        case SENSOR_TYPE_CAR_MAG_DATA_WAKE:
        {
            // u8 SensorId, float[3] RawMagData, u8 Status, u8 expansion data 
            float RawMagData[3];
            memcpy(RawMagData, &buffer[1], sizeof(RawMagData));
    
            if (printData) printf("%u Car Detect Mag Data = %3.3f,%3.3f,%3.3f\n\r", timestamp, RawMagData[0], RawMagData[1], RawMagData[2]);
            if (logData) fprintf(flog,"%u,%u,%f,%f,%f\n", timestamp,sensorId, RawMagData[0], RawMagData[1], RawMagData[2]);

            return 15;
        }
    
        case SENSOR_TYPE_SIGNIFICANT_MOTION:
        case SENSOR_TYPE_SIGNIFICANT_MOTION_WAKE:
        case SENSOR_TYPE_STEP_DETECTOR:
        case SENSOR_TYPE_STEP_DETECTOR_WAKE:
        case SENSOR_TYPE_TILT_DETECTOR:
        case SENSOR_TYPE_TILT_DETECTOR_WAKE:
        case SENSOR_TYPE_PICK_UP_GESTURE:
        case SENSOR_TYPE_PICK_UP_GESTURE_WAKE:
        case SENSOR_TYPE_WAKE_GESTURE:
        case SENSOR_TYPE_WAKE_GESTURE_WAKE:
        {
           if (printData) printf("%u %s\n\r", timestamp, em7186_sensor_name[sensorId]);
           if (logData) fprintf(flog,"%u,%u\n", timestamp,sensorId);
             return 1;
        }
        case SENSOR_TYPE_TIMESTAMP:
        case SENSOR_TYPE_TIMESTAMP_WAKE:
        {
            u16* uPacket = (u16*)&buffer[1];
            timestampPtr[0] = uPacket[0];
            timestamp = *(u32*)timestampPtr;
            return 3;
        }
        case SENSOR_TYPE_TIMESTAMP_OVERFLOW:
        case SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE:
        {
            u16* uPacket = (u16*)&buffer[1];
            timestampPtr[1] = uPacket[0];
            timestampPtr[0] = 0;
            timestamp = *(u32*)timestampPtr;
            return 3;
        }
        case SENSOR_TYPE_META:
        case SENSOR_TYPE_META_WAKE: 
        {
            if (reportMetaData) 
            {
                if (printData) 
                {
                    if (sensorId == SENSOR_TYPE_META_WAKE)
                    {
                        printf("%u WAKEUP %s:, %s/%u, %u\n\r", timestamp, em7186_meta_event_name[buffer[1]], em7186_sensor_name[buffer[2]],buffer[2], buffer[3]);
                    }
                    else 
                    {
                        printf("%u %s: %s/%u, %u\n\r", timestamp, em7186_meta_event_name[buffer[1]], em7186_sensor_name[buffer[2]], buffer[2], buffer[3]);
                        // special hook for sample script
                        if (buffer[2] == 14)
                        {
                            if (buffer[3] == 0 ) {
                                green_LED = 1;
                                printf("PASS **** The RM3100 Magnetometer ASIC and sensors are operating properly\n\r");
                            } else {
                                green_LED = 1;
                                wait(0.1);
                                green_LED = 0;
                                printf("FAIL **** The RM3100 Magnetometer ASIC and sensors fail code = %u\n\r",buffer[3]);
                            }
                        }                
                    }
                }
                if (logData) fprintf(flog,"%u,%u,%u,%u,%u\n",timestamp,sensorId, buffer[1],buffer[2],buffer[3]);
            }
            return 4;
        }
        case SENSOR_TYPE_RAW_ACCEL:
        case SENSOR_TYPE_RAW_GYRO:
            SensorData3Axis sensorData;
            float* fPacket = (float*)&buffer[7];
            sensorData.x = 256 * (s8)buffer[2] + buffer[1]; //s16 will convert to float here
            sensorData.y = 256 * (s8)buffer[4] + buffer[3];
            sensorData.z = 256 * (s8)buffer[6] + buffer[5];
            sensorData.extra = fPacket[0];
            if (printData) printf("%u %s: %3.0f, %3.0f, %3.0f, %3.1f\n\r", timestamp, em7186_sensor_name[sensorId], sensorData.x, sensorData.y, sensorData.z, sensorData.extra);
            if (logData) fprintf(flog,"%u,%u,%f,%f,%f,%f\n", timestamp,sensorId, sensorData.x, sensorData.y, sensorData.z, sensorData.extra);
            return 11;
        case SENSOR_TYPE_RAW_MAG:  //jm modified to s32 for RM3100
        {
            SensorData3Axis sensorData;
            //s32* sPacket = (s32*)&buffer[1];
            float* fPacket = (float*)&buffer[1];
            //sensorData.x = sPacket[0];    //s32 will convert to float here
            //sensorData.y = sPacket[1];
            //sensorData.z = sPacket[2];
            sensorData.x = (float)(buffer[1] + 256 * (buffer[2] + 256 * (buffer[3] + 256 * (buffer[4]))));  //s32 will convert to float here
            sensorData.y = buffer[5] + 256 * (buffer[6] + 256 * (buffer[7] + 256 * (buffer[8])));
            sensorData.z = buffer[9] + 256 * (buffer[10] + 256 * (buffer[11] + 256 * (buffer[12])));
            sensorData.extra = fPacket[3];
            if (printData) printf("%u %s: %f, %f, %f\n\r", timestamp, em7186_sensor_name[sensorId], sensorData.x, sensorData.y, sensorData.z);
            if (logData) fprintf(flog,"%u,%u,%f,%f,%f\n", timestamp,sensorId, sensorData.x, sensorData.y, sensorData.z);
            return 17;
        }
        case SENSOR_TYPE_DEBUG:
        {
            u8 packetSize = buffer[1] & 0x3F;
            u8 i;
            for (i = 0; i < packetSize; i++)
            {
                if (printData) printf("%c", (u8)buffer[2 + i]);
            }
            return 14;
        }
        default:
        {
            // Parsing error or unkown sensor type. Clear out the rest of the 
            // buffer and start clean on the next read.
            if (printData) printf("%u Other: 0x%x: %d bytes skipped\n\r", timestamp, buffer[0], size);
            break;
        }
    
    } // end switch(sensorId) 
    
    return 0;
}
u32 em7186_parse_fifo(u8* buffer, u32 size)
{
    u32 index = 0;
    u32 bytesUsed;
    u32 bytesRemaining = size;

    //if (displayText) printf("FIFO #Bytes to parse:%u\n\r",bytesRemaining);


    if (size == 0) return size;

    do {
        bytesUsed = em7186_parse_next_fifo_block(&buffer[index], bytesRemaining);
        index += bytesUsed;
        bytesRemaining -= bytesUsed;
    } while (bytesUsed > 0 && bytesRemaining > 0);

    return size - bytesRemaining;
}
u32 em7186_set_sensor_rate(u8 sensorId, u16 rate)
{
#ifdef BHI160
    u8 paramPage = PARAM_PAGE_SENSOR_INFO;
    ParamInfo param[] = { sensorId + 64, 2 };
#else
    u8 paramPage = PARAM_PAGE_SENSOR_CONF;
    ParamInfo param[] = { sensorId, 2 };
#endif
    return em7186_param_write((u8*)&rate, paramPage, param, 1);
}


u32 em7186_set_sensor_delay(u8 sensorId, u16 delay)
{
#ifdef BHI160
    u8 paramPage = PARAM_PAGE_SENSOR_INFO;
    ParamInfo param[] = { sensorId + 64, 2 };
#else
    u8 paramPage = PARAM_PAGE_SENSOR_CONF;
    ParamInfo param[] = { sensorId, 2 };
#endif
    u16 config[2];

    em7186_param_read((u8*)config, paramPage, param, 1);
    config[1] = delay;
    return em7186_param_write((u8*)config, paramPage, param, 1);
}
u32 em7186_set_meta_event(u8 eventId, u8 enable, u8 enableInt)
{
    unsigned long metaEventBits;
    ParamInfo param[] = { 1, 8 };

    em7186_param_read((u8*)&metaEventBits, PARAM_PAGE_SYSTEM, param, 1);
    unsigned long bitMask = !(0x03 << ((eventId - 1) * 2));
    unsigned long setting = ((enable ? 0x02 : 0x00) | (enableInt ? 0x01 : 0x00)) << ((eventId - 1) * 2);
    metaEventBits = (metaEventBits & bitMask) | setting;
    em7186_param_write((u8*)&metaEventBits, PARAM_PAGE_SYSTEM, param, 1);

    return 1;
}
u8  em7186_set_scale_factors()
{
    // Fixed range
    em7186_sensor_scale[SENSOR_TYPE_ORIENTATION_WAKE] =
        em7186_sensor_scale[SENSOR_TYPE_ORIENTATION] =
        360.0f / powf(2.0f, 15.0f); // Fixed

    em7186_sensor_scale[SENSOR_TYPE_ROTATION_VECTOR] =
        em7186_sensor_scale[SENSOR_TYPE_ROTATION_VECTOR_WAKE] =
        em7186_sensor_scale[SENSOR_TYPE_GAME_ROTATION_VECTOR] =
        em7186_sensor_scale[SENSOR_TYPE_GAME_ROTATION_VECTOR_WAKE] =
        em7186_sensor_scale[SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR] =
        em7186_sensor_scale[SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR_WAKE] =
        em7186_sensor_scale[SENSOR_TYPE_PDR] =
        em7186_sensor_scale[SENSOR_TYPE_PDR_WAKE] =
        em7186_sensor_scale[60] =
        1.0f / powf(2.0f, 14.0f);  // Fixed
    


    // Can change during system operation (if desired)
    float accelDynamicRange = 4.0f; // g
    float magDynamicRange = 2000.0f; // uT
    float gyroDynamicRange = 2000.0f; // d/s

    // Change depending on dynamic range
    em7186_sensor_scale[SENSOR_TYPE_ACCELEROMETER] =
        em7186_sensor_scale[SENSOR_TYPE_ACCELEROMETER_WAKE] =
        em7186_sensor_scale[SENSOR_TYPE_GRAVITY] =
        em7186_sensor_scale[SENSOR_TYPE_GRAVITY_WAKE] =
        em7186_sensor_scale[SENSOR_TYPE_LINEAR_ACCELERATION] =
        em7186_sensor_scale[SENSOR_TYPE_LINEAR_ACCELERATION_WAKE] =
        9.81f * accelDynamicRange / powf(2.0f, 15.0f);

    em7186_sensor_scale[SENSOR_TYPE_MAGNETIC_FIELD] =
        em7186_sensor_scale[SENSOR_TYPE_MAGNETIC_FIELD_WAKE] =
        em7186_sensor_scale[SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED] =
        em7186_sensor_scale[SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED_WAKE] =
        magDynamicRange / powf(2.0f, 15.0f);

    em7186_sensor_scale[SENSOR_TYPE_GYROSCOPE] =
        em7186_sensor_scale[SENSOR_TYPE_GYROSCOPE_WAKE] =
        em7186_sensor_scale[SENSOR_TYPE_GYROSCOPE_UNCALIBRATED] =
        em7186_sensor_scale[SENSOR_TYPE_GYROSCOPE_UNCALIBRATED_WAKE] =
        (3.1415927f / 180.0f) * gyroDynamicRange / powf(2.0f, 15.0f);

    // Could change depending on dynamic range of physical sensor
    em7186_sensor_scale[SENSOR_TYPE_PRESSURE] =
        em7186_sensor_scale[SENSOR_TYPE_PRESSURE_WAKE] =
        1.0f / 128.0f;              // Subject to change

    em7186_sensor_scale[SENSOR_TYPE_LIGHT] =
        em7186_sensor_scale[SENSOR_TYPE_LIGHT_WAKE] =
        10000.0f / powf(2.0f, 16.0f);    // Subject to change

    em7186_sensor_scale[SENSOR_TYPE_PROXIMITY] =
        em7186_sensor_scale[SENSOR_TYPE_PROXIMITY_WAKE] =
        100.0f / powf(2.0f, 16.0f); // Subject to change

    em7186_sensor_scale[SENSOR_TYPE_RELATIVE_HUMIDITY] =
        em7186_sensor_scale[SENSOR_TYPE_RELATIVE_HUMIDITY_WAKE] =
        100.0f / powf(2.0f, 16.0f);  // Subject to change

    em7186_sensor_scale[SENSOR_TYPE_TEMPERATURE] =
        em7186_sensor_scale[SENSOR_TYPE_TEMPERATURE_WAKE] =
        em7186_sensor_scale[SENSOR_TYPE_AMBIENT_TEMPERATURE] =
        em7186_sensor_scale[SENSOR_TYPE_AMBIENT_TEMPERATURE_WAKE] =
        150.0f / powf(2.0f, 15.0f); // Subject to change

    em7186_sensor_scale[SENSOR_TYPE_ACTIVITY] =
        em7186_sensor_scale[SENSOR_TYPE_ACTIVITY_WAKE] =
        1.0f;

    return 1;
}

/*
// Warm start parameter transfer. The parameter list and flags are presently going
//   through a major revision therefore, these functions are now on hold

u32 em7186_warm_start_load(const char *filename)
{
    FILE *fin;
    fin = fopen(filename, "rb");
    if (!fin) return 0;
    WarmStartParams warmStartParams;
    fread(&warmStartParams, 1, sizeof(warmStartParams), fin);
    fclose(fin);

    em7186_param_write((u8*)&warmStartParams, PARAM_PAGE_WARM_START, warmStartList, sizeof(warmStartList) / sizeof(warmStartList[0]));

    return 1;
}
u32 em7186_warm_start_save(const char *filename)
{
    WarmStartParams warmStartParams;
    em7186_param_read((u8*)&warmStartParams, PARAM_PAGE_WARM_START, warmStartList, sizeof(warmStartList) / sizeof(warmStartList[0]));

    FILE *fout;
    fout = fopen(filename, "wb");
    if (!fout) return 0;
    fwrite(&warmStartParams, 1, sizeof(warmStartParams), fout);
    fclose(fout);
    if (displayText) printf("Warm start parmeters Saved to warmstart.dat\n\r");
    return 1;
}*/



//=========================================================================
// Additional Control Functions
//=========================================================================
u32 em7186_ap_suspend(u8 suspend)
{
    u32 status = 0;

    if (suspend) status = em7186_i2c_write_value(HOST_INTERFACE_CTRL_REG, HOST_INTERFACE_CTRL_AP_SUSPEND);
    else status = em7186_i2c_write_value(HOST_INTERFACE_CTRL_REG, 0);

    return status;
}

u32 em7186_flush_sensor(u8 sensorId)
{
    return em7186_i2c_write_value(FIFO_FLUSH_REG, sensorId);
}

void disableSensors()
{
   u8 i;
    for ( i = 0; i<64 ;i++)
    {
        if (sensorEnabled[i] == TRUE)
        {
            em7186_set_sensor_rate(sensorEnabled[i+1], 0);
            sensorEnabled[i] = FALSE;
        }
    }        
}

u32 em7186_set_fifo_watermarks(u16 minRemainingWakeFifo, u16 minRemainingNonWakeFifo)
{
    // if min remaining values are non-zero the watermark will be set that many bytes from the end of the fifo.
    // if a zero value is used the watermark is disabled.
    u16 config[4];
    ParamInfo param[1] = { { PARAM_FIFO_CONTROL, 8 } };
    
    em7186_param_read((u8*)config, PARAM_PAGE_SYSTEM, param, 1);
    config[0] = minRemainingWakeFifo ? config[1] - minRemainingWakeFifo : 0;
    config[2] = minRemainingNonWakeFifo ? config[3] - minRemainingNonWakeFifo : 0;
    return em7186_param_write((u8*)config, PARAM_PAGE_SYSTEM, param, 1);
}

u32 em7186_enable_raw_sensors(u8 enableMag, u8 enableAccel, u8 enableGyro)
{
    u8 value = 0x00;
    if (enableMag) value |= 0x04;
    if (enableAccel) value |= 0x01;
    if (enableGyro) value |= 0x02;

    ParamInfo param[1] = { { 127, 1 } };
    
    em7186_param_write(&value, PARAM_PAGE_WARM_START, param, 1);

    return 1;
}

void resetCpu()
{
    disableSensors();
    em7186_i2c_write_value(0xB6, 63);
}

u32 paramListSize(ParamInfo *paramList, u8 numParams)
{
    u8 i;
    u32 size = 0;
    for (i = 0; i < numParams; i++)
    {
        size += paramList[i].size;
    }
    return size;
}

void displaySavedParams(u8 *values, ParamInfo *paramList, u8 numParams)
{
    u8 i;
    u16 valueIndex = 0;
    for (i = 0; i < numParams; i++)
    {
        float* floatVals = (float*)&values[valueIndex];
        u32* hexVals = (u32*)&values[valueIndex];

        if (paramList[i].size >= 4)
        {
            printf("%d 1: %f, %08x\n\r", paramList[i].paramNo, floatVals[0], hexVals[0]);
        }
        if (paramList[i].size >= 8)
        {
            printf("%d 2: %f, %08x\n\r", paramList[i].paramNo, floatVals[1], hexVals[1]);
        }
        if (paramList[i].size == 1)
        {
            u8* hexVals = (u8*)&values[valueIndex];
            printf("%d 1: %d\n\r", paramList[i].paramNo, hexVals[0]);
        }
        valueIndex += paramList[i].size;
    }
}

void displayParams(u8 paramPage, ParamInfo *paramList, u8 numParams)
{
    u32 size = paramListSize(paramList, numParams);
    u8 *values = (u8 *)malloc(size);
    em7186_param_read(values, paramPage, paramList, numParams);
    displaySavedParams(values, paramList, numParams);
    free(values);
}

/*
// Warm start parameter transfer. The parameter list and flags are presently going
//   through a major revision therefore, these functions are now on hold

void warmStart()
{
    // Save warm start params
    em7186_warm_start_save(warmStartFile);

    if (displayText) printf("\n\r\n\r-------------------- CPU Reset -------------------------------------\n\r");
    // Reset the cpu
    resetCpu();

    if (displayText) printf("\n\r\n\r---------------- Parameters after reset ----------------------------\n\r");
    // Read the warmstart params after reset
    displayParams(2, warmStartList, sizeof(warmStartList) / sizeof(warmStartList[0]));

    if (displayText) printf("\n\r\n\r---------------- Parameters after upload----------------------------\n\r");
    // Load warmstart parameters
    em7186_warm_start_load(warmStartFile);

    // Read the warmstart params after warmstart
    displayParams(2, warmStartList, sizeof(warmStartList) / sizeof(warmStartList[0]));
}*/

char* strBits(void const * const ptr, u8 numBytes, char* str)
{
    u8 *bytes = (u8*)ptr;
    u8 i, j;
    for (i = 0; i < numBytes; i++)
    {
        for (j = 0; j < 8; j++)
        {
            str[i * 8 + (7 - j)] = bytes[(numBytes - 1) - i] & (1 << j) ? '1' : '0';
        }
    }
    str[numBytes * 8] = '\0';
    return str;
}

void displayStatusRegisters()
{
    u8 buf[4];
    char str[17];
    printf("\n------------ Displaying Status Registers -----------\n\r");
    em7186_i2c_read(HOST_STATUS_REG, buf, 3);
    printf("Host Status:       % 5u, %s\n\r", buf[0], strBits(&buf[0], sizeof(buf[0]), str));
    printf("Interrupt Status:  % 5u, %s\n\r", buf[1], strBits(&buf[1], sizeof(buf[0]), str));
    printf("Chip Status:       % 5u, %s\n\r", buf[2], strBits(&buf[2], sizeof(buf[0]), str));
    em7186_i2c_read(ERR_REG, buf, 4);
    printf("Error Register:    % 5u, %s\n\r", buf[0], strBits(&buf[0], sizeof(buf[0]), str));
    printf("Interrupt State:   % 5u, %s\n\r", buf[1], strBits(&buf[1], sizeof(buf[0]), str));
    printf("Debug Value:       % 5u, %s\n\r", buf[2], strBits(&buf[2], sizeof(buf[0]), str));
    printf("Debug State:       % 5u, %s\n\r", buf[3], strBits(&buf[3], sizeof(buf[0]), str));
    em7186_i2c_read(BYTES_REMANING_REG, buf, 2);
    u16* v = (u16*)&buf;
    printf("Bytes Remaining:   % 5u, %s\n\n\r", v[0], strBits(&v[0], sizeof(v[0]), str));
}

void displaySensorStatusBits(u8 id, const SensorStatus *status)
{
    printf("|% 4u |      % 4u | % 4u |   % 4u |      % 4u | % 4u |  % 4u |\n\r", 
        id, status->dataAvailable, status->i2cNack, status->deviceIdError, 
        status->transientError, status->dataLost, status->powerMode);
}

void displaySensorStatus()
{
    SensorStatus sensorStatus[32];
    ParamInfo param[] = { PARAM_SENSOR_STATUS_BANK_0, 32 };
    em7186_param_read((u8*)sensorStatus, PARAM_PAGE_SYSTEM, param, 1);
    printf("+------------------------------------------------------------+\n\r");
    printf("|                       SENSOR STATUS                        |\n\r");
    printf("+-----+-----------+------+--------+-----------+------+-------+\n\r");
    printf("| ID  | Data      | I2C  | DEVICE | Transient | Data | Power |\n\r");
    printf("|     | Available | NACK | ID ERR | Error     | Lost | Mode  |\n\r");
    printf("+-----+-----------+------+--------+-----------+------+-------+\n\r");
    u8 i;
    for (i = 0; i < 32; i++)
    {
        displaySensorStatusBits(i + 1, &sensorStatus[i]);
    }
    param[0].paramNo = PARAM_SENSOR_STATUS_BANK_0 + 4;
    em7186_param_read((u8*)sensorStatus, PARAM_PAGE_SYSTEM, param, 1);
    for (i = 0; i < 16; i++)
    {
        displaySensorStatusBits(i + 65, &sensorStatus[i]);
    }
    printf("+-----+-----------+------+--------+-----------+------+-------+\n\r");

}

void getPhysicalSensorStatus()
{
    ParamInfo param[] = { PARAM_PHYSICAL_SENSOR_STATUS, 15 };
    em7186_param_read((u8*)&physicalSensorStatus, PARAM_PAGE_SYSTEM, param, 1);
}

void displayPhysicalSensorStatus()
{
    getPhysicalSensorStatus();
    printf("+----------------------------------------------------------------------------------------+\n\r");
    printf("|                                  PHYSICAL SENSOR STATUS                                |\n\r");
    printf("+--------+--------+---------+-----+-----------+------+--------+-----------+------+-------+\n\r");
    printf("| Sensor | Sample | Dynamic | ID  | Data      | I2C  | DEVICE | Transient | Data | Power |\n\r");
    printf("|        | Rate   | Range   |     | Available | NACK | ID ERR | Error     | Lost | Mode  |\n\r");
    printf("+--------+--------+---------+-----+-----------+------+--------+-----------+------+-------+\n\r");

    printf("| Accel  |   % 4u |    % 4u ", physicalSensorStatus.accel.sampleRate, physicalSensorStatus.accel.dynamicRange);
    displaySensorStatusBits(1, &physicalSensorStatus.accel.status);

    printf("| Gyro   |   % 4u |    % 4u ", physicalSensorStatus.gyro.sampleRate, physicalSensorStatus.gyro.dynamicRange);
    displaySensorStatusBits(2, &physicalSensorStatus.gyro.status);

    printf("| Mag    |   % 4u |    % 4u ", physicalSensorStatus.mag.sampleRate, physicalSensorStatus.mag.dynamicRange);
    displaySensorStatusBits(3, &physicalSensorStatus.mag.status);
    printf("+--------+--------+---------+-----+-----------+------+--------+-----------+------+-------+\n\r");
}
void displayPhysicalSensorInformation()
{
    typedef struct
    {
        u8 sensorType;
        u8 driverId;
        u8 driverVersion;
        u8 current;
        u16 currentDynamicRange;
        u8 flags;
        u8 reserved;
        u16 currentRate;
        u8 numAxes;
        u8 orientationMatrix[5];
    } PhysicalSensorInformation;
 
    PhysicalSensorInformation s;

    u64 physicalSensorPresent = 0;
    ParamInfo param = { 32, 4 };
    em7186_param_read((u8*)&physicalSensorPresent, PARAM_PAGE_SYSTEM, &param, 1);
    
    u32 i;
    param.size = 16;

    printf("\n+----------------------------------+--------+---------+-------+---------+------------+------+\n\r");
    printf("| Sensor                           | Driver | Driver  | Power | Current | Current    | Num  |\n\r");
    printf("|                                  | ID     | Version |       | Range   | Rate       | Axes |\n\r");
    printf("+----------------------------------+--------+---------+-------+---------+------------+------+\n\r");
    for (i = 0; i < 64; i++)
    {
        if (physicalSensorPresent & (1LL << i))
        {
            param.paramNo = i+32;
            em7186_param_read((u8*)&s, PARAM_PAGE_SYSTEM, &param, 1);
            printf("| %-32s |   % 4u |    % 4u |  % 4u |    % 4u |       % 4u | % 4u |\n\r",
                em7186_sensor_name[s.sensorType], s.driverId, s.driverVersion, s.current, s.currentDynamicRange, s.currentRate, s.numAxes);
        }
    }
    printf("+----------------------------------+--------+---------+-------+---------+------------+------+\n\r");
}

void getSensorInformation()
{
    em7186_param_read((u8*)sensorInformation, PARAM_PAGE_SENSOR_INFO, sensorInfoParamList, sizeof(sensorInfoParamList) / sizeof(sensorInfoParamList[0]));
    haveSensorInfo = 1;

    magMaxRate = sensorInformation[SENSOR_TYPE_MAGNETIC_FIELD].maxRate;
    accelMaxRate = sensorInformation[SENSOR_TYPE_ACCELEROMETER].maxRate;
    gyroMaxRate = sensorInformation[SENSOR_TYPE_GYROSCOPE].maxRate;
}

void getSensorConfiguration()
{
    u8 i;
    ParamInfo param[1] = { 0, 8 };
    for (i = 1; i<sizeof(sensorInformation) / sizeof(sensorInformation[0]); i++)
    { 
        if (sensorInformation[i].sensorId > 0)
        {
            param[0].paramNo = sensorInformation[i].sensorId;
            em7186_param_read((u8*)&sensorConfiguration[i], PARAM_PAGE_SENSOR_CONF, param, 1);
        }
    }
}

void displaySensorConfiguration()
{
    if (!haveSensorInfo) { getSensorInformation(); };
    getSensorConfiguration();
    printf("+-------------------------------------------------------------------------+\n\r");
    printf("|                          Sensor Configuration                           |\n\r");
    printf("+----------------------------------+-------+-------+-------------+--------+\n\r");
    printf("| Sensor                           | Rate  | Delay | Sensitivity | Range  |\n\r");
    printf("+----------------------------------+-------+-------+-------------+--------+\n\r");
    u8 i;
    for (i = 0; i < sizeof(sensorInformation) / sizeof(sensorInformation[0]); i++)
    {
        if (sensorInformation[i].sensorId > 0)
        {
            printf("| %-32s | % 5u | % 5u | % 11u | % 6u |\n\r",
                em7186_sensor_name[sensorInformation[i].sensorId],
                sensorConfiguration[i].sampleRate,
                sensorConfiguration[i].maxReportLatency, 
                sensorConfiguration[i].changeSensitivity, 
                sensorConfiguration[i].dynamicRange);
        }
    }
    printf("+----------------------------------+-------+-------+-------------+--------+\n\r");
}

void displaySensorInformation()
{
    if (!haveSensorInfo) { getSensorInformation(); };
    printf("+------------------------------------------------------------------------------------------+\n\r");
    printf("|                                  Sensor Information                                      |\n\r");
    printf("+----------------------------------------+---------+-------+-------+-----+----------+------+\n\r");
    printf("| ID  |Sensor                            | Driver  | Power | Range | Res | Rate     | Size |\n\r");
    printf("+-----|----------------------------------+---------+-------+-------+-----+----------+------+\n\r");
    u8 i;
    for (i = 0; i < sizeof(sensorInfoParamList) / sizeof(sensorInfoParamList[0]); i++)
    {
        if (sensorInformation[i].sensorId > 0)
        {
            printf("|%3u  | %-32s | % 3u.%-3u |  % 4u | % 5u | % 3u | %-3u-% 4u |  % 3u |\n\r",
                i,
                em7186_sensor_name[sensorInformation[i].sensorId], sensorInformation[i].driverId, 
                sensorInformation[i].driverVersion, sensorInformation[i].power,
                sensorInformation[i].maxRange, sensorInformation[i].resolution, 
                sensorInformation[i].minRate, sensorInformation[i].maxRate,
                sensorInformation[i].eventSize);
        }
    }
    printf("+-----|----------------------------------+---------+-------+-------+-----+----------+------+\n\r");
}


void read_BuildVersion(void) {

    ParamInfo param; // elements:   {u8 paramNo; u8 size;}

    printf("\n\r\n\rSENtral Firmware build version information from PramIO Page 9\n\r");

    struct pni_version {
        u8 major;
        u8 minor;
        u8 patch;
        u8 other;
        u32 build;
    };

    struct pni_version pni_ver;


    param.paramNo = 1;
    param.size = 8;
    em7186_param_read((u8 *)&pni_ver, 9, &param, 1);

    printf("  major = %u\n\r", pni_ver.major);
    printf("  minor = %u\n\r", pni_ver.minor);
    printf("  patch = %u\n\r", pni_ver.patch);
    printf("  other = %u\n\r", pni_ver.other);
    printf("  build = %u\n\r", pni_ver.build);


}
