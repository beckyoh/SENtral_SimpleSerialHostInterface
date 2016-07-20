#include "main.h"

#define REVISION "1.0.0"
#define CR 13
#define LF 10

// Note: all warmstart related functions are commented-out. The parameter list and 
//   flags are presently going through a major revision.



//******************************
//  Function prototypes
//******************************
void read_BuildVersion(void);
void executeSerialFunction(char key);

//*******************************
// Global Variables
//*******************************
u8      serialCommandMode;
u8      serialSensorID;
u16     serialSensorRate; 
u16     serialCommandValue;

u8      fifoBuffer[24 * 1024];
u32     bytesRead;

u8      apSuspendMode = 0;
u8      displayText = 1;       // Also see simular "printData" in em7186.cpp
u8      reportMetaData = 0;

u8      fw[] = { "/sd/sentral.fw" };
u8      logfilename[] = { "/sd/log.csv" };
//u8      warmStartFile[] = { "/sd/warmstart.dat" }; 

FILE *flog;

char    serial_inchar;

//******************************
//  Interrupt callback functions
//******************************
 
void SENtral_Interrupt(void)
{

    bytesRead = em7186_read_fifo(fifoBuffer);
    em7186_parse_fifo(fifoBuffer, bytesRead);
    
}

void OnSerial(void)
{
    serial_inchar = pc.getc();
}

//=============================================================================
//  Serial input Character (Key) Commands
//=============================================================================

// processSerialInchar function is called upon every incomming serial character 
//   as defined in mbed_objects.cpp
void processSerialInchar(char key)
{
    // The Simple Serial protocal mostly consists of single character commands
    //   the exceptions are commands that require additional information like 's' and 'X'.
    //   In these cases serialCommandMode states are used and operands are then preprocessed 
    //   inside this function.

    // If the protocal gets larger and more commands require additional data a more complex
    //    command processor with incoming serial buffer will have to be implimented
    //    presently this simple processor does the job

    // serialCommandMode  ** STATES **
    // 0: Not in Sensor command building mode
    // 1: 's' Sensor Rate chane. State 1 = Constructing Sensor ID number
    // 2:                        State 2 = Constructing Sensor Rate Value
    // 3: 'X' Firmware Image Tranfer Mode (waiting for source/destination character)
    // CR: [carrage return] sends constructed sensor rate request
    
    // send '?' to mbed for menu of commands


    serial_inchar = NULL;

    if ((serialCommandMode > 0) && (serialCommandMode < 3)) { //'s' virutual sensor rate change request
        if ((key >= '0') && (key <= '9')) {
            serialCommandValue = (serialCommandValue * 10) + (key - '0');
        }
        else if (key == ',') {
            serialSensorID = (unsigned char)serialCommandValue;
            serialCommandValue = 0;
            serialCommandMode = 2;
        } else if (key == CR || key == LF) {
            if (serialCommandMode == 1) {
                serialSensorID = serialCommandValue;
            } else {
                serialSensorRate = serialCommandValue;
            }
            if (serialSensorID > 0) {
                if (displayText) printf("\n\rChanging rate of sensor %u to %d\n\r", serialSensorID, serialSensorRate);
                em7186_set_sensor_rate(serialSensorID,serialSensorRate);
                if (serialSensorRate > 0) {
                    sensorEnabled[serialSensorID-1] = TRUE;
                } else {
                    sensorEnabled[serialSensorID-1] = FALSE;
                }
            }
            serialCommandMode = 0;
            serialSensorID = 0;
            serialSensorRate = 0;
        }
    } else if (serialCommandMode == 3) { // 'X' Transfer firmware command
        firmwareTransfer(key);
        serialCommandMode = 0;
    } else
        executeSerialFunction(key);
}

void executeSerialFunction(char key)
{
    u8 paramValues[2];
    ParamInfo param = { 1,1 }; // parameter# 1, size=1
    switch (key)
    {
    case 'B':
        if (displayText) printf("\n\r\n\r ********** PREPARING FOR SENSOR SELF TEST REQUEST; STANDBY REQUEST MADE ********** \n\r\n\r");
        paramValues[0] = 0x01; // Request Sensor Self Test bit and NOT standby bit
        em7186_i2c_write(0x55, paramValues, 1); // 0x55 = Host Interface Control Reg
        if (displayText) printf("\n\r\n\r ********** SENSOR SELF TEST REQUEST + RUN MADE ********** \n\r\n\r");
        paramValues[0] = 0x40; // Request Sensor Self Test bit and NOT standby bit
        em7186_i2c_write(0x55, paramValues, 1); // 0x55 = Host Interface Control Reg
        break;
    case 'c':
        displayStatusRegisters();
        break;
//    case 'd':
//        displayParams(2, warmStartList, sizeof(warmStartList) / sizeof(warmStartList[0]));
//        break;
    case 'D':  // this togggles display of SENtral(FIFO) Data 
               // Only useful when intending to log data through SD Card and want to suspend terminal display of data
        printData = !printData;
        if (displayText) printf("Display SENtral Data  - %s\n\r", (printData ? "Enabled" : "Disabled"));
        break;
    case 'e':
        displayPhysicalSensorStatus();
        break;
    case 'f':
        displaySensorStatus();
        break;

    case 'g':
        logData = !logData;
        if(logData) {
            flog = fopen(logfilename, "a");
            if (!flog) {
                printf("Error Opening log file\n\r");
                logData = 0;
            }
            else
                printf("%s Log file open\n\r",logfilename);            
        }
        else {
            if(flog) {
                fclose(flog);
                printf("Log file closed\n\r");
            }
        }
        break;    
              
    case 'h':
        apSuspendMode = !apSuspendMode;
        if (apSuspendMode) em7186_ap_suspend(1);
        else em7186_ap_suspend(0);
        if (displayText) printf("AP Suspend Mode %s\n\r", (apSuspendMode ? "Enabled" : "Disabled"));
        break;
    case 'H':
        em7186_i2c_write_value(CHIP_CONTROL_REG, CHIP_CONTROL_CPU_STOP);        
        //SENtral_InterruptPin.enable_irq();
        if (displayText) printf("Exit Run Mode Request sent\n\r");
        break;
    case 'i':
    {
        displayPhysicalSensorInformation();
        break;
    }
    case 'j':
        em7186_set_fifo_watermarks(1000, 1000);
        break;
//    case 'l':
//        em7186_warm_start_load(warmStartFile);
//        break;
    case 'm':
        reportMetaData = !reportMetaData;
        if (reportMetaData) printf("Meta Data reporting  - %s\n\r", (printData ? "Enabled" : "Disabled"));
        break;
    case 'n':
        displaySensorInformation();
        break;
    case 'r':
        em7186_i2c_write_value(CHIP_CONTROL_REG, CHIP_CONTROL_CPU_RUN);        
        em7186_set_scale_factors();
        SENtral_InterruptPin.enable_irq();
        if (displayText) printf("Run Mode Request sent\n\r");
        break;
    case 'R':
        em7186_i2c_write_value(RESET_REQ_REG, 1);        
        if (displayText) printf("Reset Request Sent\n\r"); 
        break;
    case 's':
        serialCommandMode = 1; // user beginning to Enable/Disable or change SensorRate
        serialSensorID = 0; serialSensorRate = 0; serialCommandValue = 0;
        if (displayText) printf("Enter Sensor ID,Rate--> ");
        // reminder: SENtral must be "run"-ing before the virtual sensor actually starts
        // run mode ('r' command) can be executed before or after this command
        break;
//    case 'S':
//        em7186_warm_start_save(warmStartFile);
//        break;
   case 't':
        displayText = !displayText;
        if (displayText) printf("Text Display %s\n\r", (displayText ? "Enabled" : "Disabled"));
        break;
    case 'X':
            if (displayText) printf("Firmware Transfer. Enter source/destination code: %s\n\r",fw);
            serialCommandMode = 3;
            break;
    case 'v':
    case 'V':
        read_BuildVersion();
        break;
//    case 'w':
//        warmStart(); // this is to test the warmstart save/load process
//        break;
    case 'y':
        break;
    case 'z':
        displaySensorConfiguration();
        break;
     case '?':
        {
        u8 bar[45];
        memset(bar, 205, sizeof(bar));
        bar[sizeof(bar)-1] = 0;
        printf("\n\r");
        printf("  RM3100RTI-SEntral Simple Serial Interface\n\r");
        printf("                  Revision: %s\n\r",REVISION);
        printf("  %c%s%c\n\r", 201, bar, 187);
        printf("  %c        Commands  (case sensitive)          %c\n\r", 186, 186);
        // Status and configuration
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c         Configuration and Status           %c\n\r", 186, 186);
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c c : Display status registers               %c\n\r", 186, 186);
        printf("  %c e : Display physical sensor status         %c\n\r", 186, 186);
        printf("  %c f : Display sensor status                  %c\n\r", 186, 186);
        printf("  %c i : Display Physical Sensor Information    %c\n\r", 186, 186);
        printf("  %c n : Display sensor information             %c\n\r", 186, 186);
        printf("  %c z : Display sensor configuration           %c\n\r", 186, 186);
        printf("  %c v : Display Firmware Version information   %c\n\r", 186, 186);

        // Sensor control
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c               Firmware Transfers           %c\n\r", 186, 186);
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c XR : From SD Card to SENtral RAM           %c\n\r", 186, 186);
        printf("  %c XE : From SD Card to EEPROM                %c\n\r", 186, 186);
 
        // Sensor control
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c               Sensor Rates                 %c\n\r", 186, 186);
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c s @@@,###[CR]                              %c\n\r", 186, 186);
        printf("  %c   where:                                   %c\n\r", 186, 186);
        printf("  %c    @@@ = Sensor ID                         %c\n\r", 186, 186);
        printf("  %c    ### = Data rate                         %c\n\r", 186, 186); 
        printf("  %c    [CR]= carriage return (0x0D)            %c\n\r", 186, 186); 
 
        // Display controls
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c             Display Controls               %c\n\r", 186, 186);
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c t : Toggle command feedback text           %c\n\r", 186, 186);
        printf("  %c m : Meta event reporting (on/off)          %c\n\r", 186, 186);

        // Data logging
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c               Data Logging                 %c\n\r", 186, 186);
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c g : Toggle sensor Data log (on/off)        %c\n\r", 186, 186);
        printf("  %c D : Toggle sensor Data display (on/off)    %c\n\r", 186, 186);

        // Additional controls
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c           Additional Controls              %c\n\r", 186, 186);
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c j : Set fifo watermarks to 1000            %c\n\r", 186, 186);
        printf("  %c h : Toggle AP suspend mode (on/off)        %c\n\r", 186, 186);
        printf("  %c R : Send Reset Request to SENtral          %c\n\r", 186, 186);
        printf("  %c r : Request SENtral Run Mode ON            %c\n\r", 186, 186);
        printf("  %c H : Request SENtral Run Mode OFF (Halt)    %c\n\r", 186, 186);
 
        // Warm-start
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c                Warm-Start                  %c\n\r", 186, 186);
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c S : Save warm-start parameters             %c\n\r", 186, 186);
        printf("  %c l : Load warm-start parameters             %c\n\r", 186, 186);
        printf("  %c d : Display warm-start parameters          %c\n\r", 186, 186);
        printf("  %c w : Perform warm-start test                %c\n\r", 186, 186);
        
        // Tests
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c                  Tests                     %c\n\r", 186, 186);
        printf("  %c%s%c\n\r", 204, bar, 185);
        printf("  %c B : Run sensor Self tests                  %c\n\r", 186, 186);

        printf("  %c%s%c\n\r", 200, bar, 188);

        }
    }
}


void runScript(void)
{
    
    // This script(function) runs when the Pushbutton on the RM3100 Arduino
    // shield is pressed, which is connected to pin D5.
    
    green_LED = 1; // flash to denote start
    wait(0.1);
    green_LED = 0;
    
    if (displayText) printf("Running Special script\n\r");  
    
/*
    Your custom code goes here
*/


}


//******************************
//  MAIN
//******************************

int main()
{

    // Init user serial interface
    pc.baud(115200);
    printf("SENtral Simple Serial Host Interface %s\n\r",REVISION);

    // Initialize Stuff
    if (!em7186_i2c_init()) {
        printf("Failed to see SENtral device.\n\r Check connections\n\r");
    }


    // Setup interrupt callback functions
    SENtral_InterruptPin.rise(&SENtral_Interrupt); // SENtral host interrupt
    pc.attach(&OnSerial);                          // user input from serial term 
    
    
    // flash ready signal on expansion board Assumes LED is jumpered to D4
    for (char i=4;i;i--)
    {
        green_LED       = 1;     // This LED is optional RM3100RTI shield board specific
        NucleoRedLED    = 1; // This LED is Nucleo board specific used here only to denote program upload success
        wait(0.1);
        green_LED       = 0;
        NucleoRedLED    = 0;
        wait(.1);
    }
    green_LED       = 1;
    NucleoRedLED    = 1;
    wait(.25);
   
    // This LED is Nucleo board specific
    NucleoRedLED  = 1; // used here only to denote program upload success

    while (1) 
    {

        if (serial_inchar) 
        {
            processSerialInchar(serial_inchar); // process user key commands
            serial_inchar = NULL;
        }

        if (pushButton == 1) // PBSwitch is wired as active high
            runScript();    // execute special script(function)
 
    }

}

