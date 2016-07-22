// The purpose of this file and associated header file is to 
//   enable globalization of mbed platform specific objects
//   for all source files to utilize

#include "mbed_objects.h"

//******************************
// MBED Library instatiations 
//******************************
                //mosi miso sck cs
SDFileSystem sd(D11, D12, D13, D10, "sd");
I2C i2c(I2C_SDA, I2C_SCL);
Serial pc(SERIAL_TX, SERIAL_RX);
InterruptIn SENtral_InterruptPin(D2);

// These LEDs and PBSwitch are Not part of the RM3100RTI Arduino Shield.    
DigitalOut green_LED(D4);

DigitalIn pushButton(D5);

//=========================================================================
// I/O functions customized for MBED platform
//=========================================================================

// MBED native READ/ WRITE functions
u32 em7186_i2c_write(u8 registerAddress, u8* buffer, u16 length)
{
    u8 writeBuffer[MAX_I2C_WRITE + 1];
    writeBuffer[0] = registerAddress;
    memcpy(&writeBuffer[1], buffer, length);
   //i2c.write(int address, const u8 *data, int length, bool repeated=false)
   // returns 0 on success (ack), non-0 on failure (nack)
    int status = i2c.write(SENtral_ADDRESS, writeBuffer, length+1, 0);
    return !status; // return True if successfull. mbed:0=Success
}

u32 em7186_i2c_read(u8 registerAddress, u8* buffer, u16 length)
{
    u8 writeBuffer[1] = {registerAddress};
    i2c.write(SENtral_ADDRESS, writeBuffer, 1, 1);
    int status = i2c.read(SENtral_ADDRESS, buffer, length, 0);
    if (!status) //mbed:0=Success
        return length;
    else
        return 0;
}

u32 EE_Write(u8 I2C_Addr, u16 EE_MemAddr, u8*buffer, u16 length)
{
    u8 outbuffer[MAX_I2C_WRITE + 2];
    
    memcpy((u8 *)&outbuffer[0],&EE_MemAddr, 2);
    u8 temp = outbuffer[0]; outbuffer[0] = outbuffer[1];outbuffer[1]=temp; // swap endian
    memcpy((u8 *)&outbuffer[2],buffer,length );
    int status = i2c.write(I2C_Addr, outbuffer, length+2, 0);
    return !status; // return True if successfull. mbed:0=Success   
}  

u32 EE_Read(u8 I2C_Addr, u16 EE_MemAddr, u8*buffer, u16 length)
{
    u8 writeBuffer[2];
    memcpy((u8 *)&writeBuffer[0],&EE_MemAddr, 2);
    u8 temp = writeBuffer[0]; writeBuffer[0] = writeBuffer[1];writeBuffer[1]=temp; // swap endian
    i2c.write(I2C_Addr, writeBuffer, 2, 1);
    int status = i2c.read(I2C_Addr, buffer, length, 0);
    return !status; // return True if successfull. mbed:0=Success   
}

