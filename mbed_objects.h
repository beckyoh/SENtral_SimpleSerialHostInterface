#ifndef OBJECTS_H
#define OBJECTS_H
// The purpose of this file and associated header file is to 
//   enable globalization of mbed platform specific objects
//   for all source files to utilize
#include "main.h"
#include "SDBlockDevice.h"
   
    extern SDBlockDevice sd;
    extern I2C i2c;
    extern Serial pc;
    extern InterruptIn SENtral_InterruptPin;
    
// These LEDs and PBSwitch are Not part of the RM3100RTI Arduino Shield.    
    extern DigitalOut green_LED;
    extern DigitalIn pushButton;

    u32 em7186_i2c_write(u8 registerAddress, u8* buffer, u16 length);
    u32 em7186_i2c_read(u8 registerAddress, u8* buffer, u16 length);
    u32 EE_Write(u8 I2C_Addr, u16 EE_MemAddr, u8*buffer, u16 length);
    u32 EE_Read(u8 I2C_Addr, u16 EE_MemAddr, u8*buffer, u16 length);


#endif