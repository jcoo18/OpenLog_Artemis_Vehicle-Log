#include <Arduino.h>
#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\OpenLog_Artemis.ino"
/*
  OpenLog Artemis
  By: Nathan Seidle and Paul Clark
  SparkFun Electronics
  Date: November 26th, 2019
  License: MIT. Please see LICENSE.md for more details.
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/16832
  https://www.sparkfun.com/products/19426

  This firmware runs the OpenLog Artemis. A large variety of system settings can be
  adjusted by connecting at 115200bps.

  The Board should be set to SparkFun Apollo3 \ RedBoard Artemis ATP.

  v1.0 Power Consumption:
   Sleep between reads, RTC fully charged, no Qwiic, SD, no USB, no Power LED: 260uA
   10Hz logging IMU, no Qwiic, SD, no USB, no Power LED: 9-27mA

  TODO:
  (done) Create settings file for sensor. Load after qwiic bus is scanned.
  (done on larger Strings) Remove String dependencies.
  (done) Bubble sort list of devices.
  (done) Remove listing for muxes.
  (done) Verify the printing of all sensors is %f, %d correct
  (done) Add begin function seperate from everything, call after wakeup instead of detect
  (done) Add counter to output to look for memory leaks on long runs
  (done) Add AHT20 support
  (done) Add SHTC3 support
  (done) Change settings extension to txt
  (done) Fix max I2C speed to use linked list
  Currently device settings are not recorded to EEPROM, only deviceSettings.txt
  Is there a better way to dynamically create size of sdOutputData array so we don't ever get larger than X sensors outputting?
  Find way to store device configs into EEPROM
  Log four pressure sensors and graph them on plotter
  (checked) Test GPS - not sure about %d with int32s. Does lat, long, and alt look correct?
  (done) Test NAU7802s
  (done) Test SCD30s (Add an extended delay for the SCD30. (Issue #5))
  (won't do?) Add a 'does not like to be powered cycled' setting for each device type. I think this has been superceded by "Add individual power-on delays for each sensor type?.
  (done) Add support for logging VIN
  (done) Investigate error in time between logs (https://github.com/sparkfun/OpenLog_Artemis/issues/13)
  (done) Invesigate RTC reset issue (https://github.com/sparkfun/OpenLog_Artemis/issues/13 + https://forum.sparkfun.com/viewtopic.php?f=123&t=53157)
    The solution is to make sure that the OLA goes into deep sleep as soon as the voltage monitor detects that the power has failed.
    The user will need to press the reset button once power has been restored. Using the WDT to check the monitor and do a POR wasn't reliable.
  (done) Investigate requires-reset issue on battery power (") (X04 + CCS811/BME280 enviro combo)
  (done) Add a fix so that the MS8607 does not also appear as an MS5637
  (done) Add "set RTC from GPS" functionality
  (done) Add UTCoffset functionality (including support for negative numbers)
  (done) Figure out how to give the u-blox time to establish a fix if it has been powered down between log intervals. The user can specify up to 60s for the Qwiic power-on delay.
  Add support for VREG_ENABLE
  (done) Add support for PWR_LED
  (done) Use the WDT to reset the Artemis when power is reconnected (previously the Artemis would have stayed in deep sleep)
  Add a callback function to the u-blox library so we can abort waiting for UBX data if the power goes low
  (done) Add support for the ADS122C04 ADC (Qwiic PT100)
  (done) Investigate why usBetweenReadings appears to be longer than expected. We needed to read millis _before_ enabling the lower power clock!
  (done) Correct u-blox pull-ups
  (done) Add an olaIdentifier to prevent problems when using two code variants that have the same sizeOfSettings
  (done) Add a fix for the IMU wake-up issue identified in https://github.com/sparkfun/OpenLog_Artemis/issues/18
  (done) Add a "stop logging" feature on GPIO 32: allow the pin to be used to read a stop logging button instead of being an analog input
  (done) Allow the user to set the default qwiic bus pull-up resistance (u-blox will still use 'none')
  (done) Add support for low battery monitoring using VIN
  (done) Output sensor data via the serial TX pin (Issue #32)
  (done) Add support for SD card file transfer (ZMODEM) and delete. (Issue #33) With thanks to: ecm-bitflipper (https://github.com/ecm-bitflipper/Arduino_ZModem)
  (done) Add file creation and access timestamps
  (done) Add the ability to trigger data collection via Pin 11 (Issue #36)
  (done) Correct the measurement count misbehaviour (Issue #31)
  (done) Use the corrected IMU temperature calculation (Issue #28)
  (done) Add individual power-on delays for each sensor type. Add an extended delay for the SCD30. (Issue #5)
  (done) v1.7: Fix readVin after sleep bug: https://github.com/sparkfun/OpenLog_Artemis/issues/39
  (done) Change detectQwiicDevices so that the MCP9600 (Qwiic Thermocouple) is detected correctly
  (done) Add support for the MPRLS0025PA micro pressure sensor
  (done) Add support for the SN-GCJA5 particle sensor
  (done) Add IMU accelerometer and gyro full scale and digital low pass filter settings to menuIMU
  (done) Add a fix to make sure the MS8607 is detected correctly: https://github.com/sparkfun/OpenLog_Artemis/issues/54
  (done) Add logMicroseconds: https://github.com/sparkfun/OpenLog_Artemis/issues/49
  (done) Add an option to use autoPVT when logging GNSS data: https://github.com/sparkfun/OpenLog_Artemis/issues/50
  (done) Corrected an issue when using multiple MS8607's: https://github.com/sparkfun/OpenLog_Artemis/issues/62
  (done) Add a feature to use the TX and RX pins as a duplicate Terminal
  (done) Add serial log timestamps with a token (as suggested by @DennisMelamed in PR https://github.com/sparkfun/OpenLog_Artemis/pull/70 and Issue https://github.com/sparkfun/OpenLog_Artemis/issues/63)
  (done) Add "sleep on pin" functionality based @ryanneve's PR https://github.com/sparkfun/OpenLog_Artemis/pull/64 and Issue https://github.com/sparkfun/OpenLog_Artemis/issues/46
  (done) Add "wake at specified times" functionality based on Issue https://github.com/sparkfun/OpenLog_Artemis/issues/46
  (done) Add corrections for the SCD30 based on Forum post by paulvha: https://forum.sparkfun.com/viewtopic.php?p=222455#p222455
  (done) Add support for the SGP40 VOC Index sensor
  (done) Add support for the SDP3X Differential Pressure sensor
  (done) Add support for the MS5837 - as used in the BlueRobotics BAR02 and BAR30 water pressure sensors
  (done) Correct an issue which was causing the OLA to crash when waking from sleep and outputting serial data https://github.com/sparkfun/OpenLog_Artemis/issues/79
  (done) Correct low-power code as per https://github.com/sparkfun/OpenLog_Artemis/issues/78
  (done) Correct a bug in menuAttachedDevices when useTxRxPinsForTerminal is enabled https://github.com/sparkfun/OpenLog_Artemis/issues/82
  (done) Add ICM-20948 DMP support. Requires v1.2.6 of the ICM-20948 library. DMP logging is limited to: Quat6 or Quat9, plus raw accel, gyro and compass. https://github.com/sparkfun/OpenLog_Artemis/issues/47
  (done) Add support for exFAT. Requires v2.0.6 of Bill Greiman's SdFat library. https://github.com/sparkfun/OpenLog_Artemis/issues/34
  (done) Add minimum awake time: https://github.com/sparkfun/OpenLog_Artemis/issues/83
  (done) Add support for the Pulse Oximeter: https://github.com/sparkfun/OpenLog_Artemis/issues/81
  (done - but does not work) Add support for the Qwiic Button. The QB uses clock-stretching and the Artemis really doesn't enjoy that...
  (done) Increase DMP data resolution to five decimal places https://github.com/sparkfun/OpenLog_Artemis/issues/90

  (in progress) Update to Apollo3 v2.1.0 - FIRMWARE_VERSION_MAJOR = 2.
  (done) Implement printf float (OLA uses printf float in _so_ many places...): https://github.com/sparkfun/Arduino_Apollo3/issues/278
  (worked around) attachInterrupt(PIN_POWER_LOSS, powerDownOLA, FALLING); triggers an immediate interrupt - https://github.com/sparkfun/Arduino_Apollo3/issues/416
  (done) Add a setQwiicPullups function
  (done) Check if we need ap3_set_pin_to_analog when coming out of sleep
  (done) Investigate why code does not wake from deep sleep correctly
  (worked around) Correct SerialLog RX: https://github.com/sparkfun/Arduino_Apollo3/issues/401
    The work-around is to use Serial1 in place of serialLog and then to manually force UART1 to use pins 12 and 13
    We need a work-around anyway because if pins 12 or 13 have been used as analog inputs, Serial1.begin does not re-configure them for UART TX and RX
  (in progress) Reduce sleep current as much as possible. v1.2.1 achieved ~110uA. With v2.1.0 the draw is more like 260uA...

  (in progress) Update to Apollo3 v2.2.0 - FIRMWARE_VERSION_MAJOR = 2; FIRMWARE_VERSION_MINOR = 1.
  (done) Add a fix for issue #109 - check if a BME280 is connected before calling multiplexerBegin: https://github.com/sparkfun/OpenLog_Artemis/issues/109
  (done) Correct issue #104. enableSD was redundant. The microSD power always needs to be on if there is a card inserted, otherwise the card pulls
         the SPI lines low, preventing communication with the IMU:  https://github.com/sparkfun/OpenLog_Artemis/issues/104

  v2.2:
    Use Apollo3 v2.2.1 with changes by paulvha to fix Issue 117 (Thank you Paul!)
      https://github.com/sparkfun/OpenLog_Artemis/issues/117#issuecomment-1085881142
    Also includes Paul's SPI.end fix
      https://github.com/sparkfun/Arduino_Apollo3/issues/442
      In libraries/SPI/src/SPI.cpp change end() to:
        void arduino::MbedSPI::end() {
            if (dev) {
                delete dev;
                dev = NULL;
            }
        }      
    Use SdFat v2.1.2
    Compensate for missing / not-populated IMU
    Add support for yyyy/mm/dd and ISO 8601 date style (Issue 118)
    Add support for fractional time zone offsets

  v2.3 BETA:
    Change GNSS maximum rate to 25Hz as per:
      https://github.com/sparkfun/OpenLog_Artemis/issues/121
      https://forum.sparkfun.com/viewtopic.php?f=172&t=57512
    Decided not to include this in the next full release.
    I suspect it will cause major badness on (e.g.) M8 modules that cannot support 25Hz.
    
  v2.3:
    Resolve https://forum.sparkfun.com/viewtopic.php?f=171&t=58109

  v2.4:
    Add noPowerLossProtection to the main branch
    Add changes by KDB: If we are streaming to Serial, start the stream with a Mime Type marker, followed by CR
    Add debug option to only open the menu using a printable character: based on https://github.com/sparkfun/OpenLog_Artemis/pull/125

  v2.5:
    Add Tony Whipple's PR #146 - thank you @whipple63
    Add support for the ISM330DHCX, MMC5983MA, KX134 and ADS1015
    Resolve issue #87

  v2.6:
    Add support for the LPS28DFW - thank you @gauteh #179
    Only disable I2C SDA and SCL during sleep when I2C bus is being powered down - thank you @whipple63 #167
    Add calibrationConcentration support for the SCD30 - thank you @hotstick #181
    Add limited support for the VEML7700 light sensor

  v2.7:
    Resolve serial logging issue - crash on startup - #182

  v2.8:
    Corrects the serial token timestamp printing - resolves #192
    The charsReceived debug print ("Total chars received: ") now excludes the length of the timestamps
  TODO:
  (in progress) Add PCF8575 read ports capability
  (in progress) Add custom slave microcontroller input   

*/

const int FIRMWARE_VERSION_MAJOR = 2;
const int FIRMWARE_VERSION_MINOR = 8;

//Define the OLA board identifier:
//  This is an int which is unique to this variant of the OLA and which allows us
//  to make sure that the settings in EEPROM are correct for this version of the OLA
//  (sizeOfSettings is not necessarily unique and we want to avoid problems when swapping from one variant to another)
//  It is the sum of:
//    the variant * 0x100 (OLA = 1; GNSS_LOGGER = 2; GEOPHONE_LOGGER = 3)
//    the major firmware version * 0x10
//    the minor firmware version
#define OLA_IDENTIFIER 0x128 // Stored as 296 decimal in OLA_settings.txt

//#define noPowerLossProtection // Uncomment this line to disable the sleep-on-power-loss functionality

#include "Sensors.h"

#include "settings.h"

//Define the pin functions
//Depends on hardware version. This can be found as a marking on the PCB.
//x04 was the SparkX 'black' version.
//v10 was the first red version.
#define HARDWARE_VERSION_MAJOR 1
#define HARDWARE_VERSION_MINOR 0

#if(HARDWARE_VERSION_MAJOR == 0 && HARDWARE_VERSION_MINOR == 4)
const byte PIN_MICROSD_CHIP_SELECT = 10;
const byte PIN_IMU_POWER = 22;
#elif(HARDWARE_VERSION_MAJOR == 1 && HARDWARE_VERSION_MINOR == 0)
const byte PIN_MICROSD_CHIP_SELECT = 23;
const byte PIN_IMU_POWER = 27;
const byte PIN_PWR_LED = 29;
const byte PIN_VREG_ENABLE = 25;
const byte PIN_VIN_MONITOR = 34; // VIN/3 (1M/2M - will require a correction factor)
#endif

const byte PIN_POWER_LOSS = 3;
//const byte PIN_LOGIC_DEBUG = 11; // Useful for debugging issues like the slippery mux bug
const byte PIN_MICROSD_POWER = 15;
const byte PIN_QWIIC_POWER = 18;
const byte PIN_STAT_LED = 19;
const byte PIN_IMU_INT = 37;
const byte PIN_IMU_CHIP_SELECT = 44;
const byte PIN_STOP_LOGGING = 32;
const byte BREAKOUT_PIN_32 = 32;
const byte BREAKOUT_PIN_TX = 12;
const byte BREAKOUT_PIN_RX = 13;
const byte BREAKOUT_PIN_11 = 11;
const byte PIN_TRIGGER = 11;
const byte PIN_QWIIC_SCL = 8;
const byte PIN_QWIIC_SDA = 9;

const byte PIN_SPI_SCK = 5;
const byte PIN_SPI_CIPO = 6;
const byte PIN_SPI_COPI = 7;

// Include this many extra bytes when starting a mux - to try and avoid the slippery mux bug
// This should be 0 but 3 or 7 seem to work better depending on which way the wind is blowing.
const byte EXTRA_MUX_STARTUP_BYTES = 3;

enum returnStatus {
  STATUS_GETBYTE_TIMEOUT = 255,
  STATUS_GETNUMBER_TIMEOUT = -123455555,
  STATUS_PRESSED_X,
};

//Setup Qwiic Port
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <Wire.h>
TwoWire qwiic(PIN_QWIIC_SDA,PIN_QWIIC_SCL); //Will use pads 8/9
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//EEPROM for storing settings
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <EEPROM.h>
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//microSD Interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <SPI.h>

#include <SdFat.h> //SdFat by Bill Greiman: http://librarymanager/All#SdFat_exFAT

#define SD_FAT_TYPE 3 // SD_FAT_TYPE = 0 for SdFat/File, 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_CONFIG SdSpiConfig(PIN_MICROSD_CHIP_SELECT, SHARED_SPI, SD_SCK_MHZ(24)) // 24MHz

#if SD_FAT_TYPE == 1
SdFat32 sd;
File32 sensorDataFile; //File that all sensor data is written to
File32 serialDataFile; //File that all incoming serial data is written to
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile sensorDataFile; //File that all sensor data is written to
ExFile serialDataFile; //File that all incoming serial data is written to
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile sensorDataFile; //File that all sensor data is written to
FsFile serialDataFile; //File that all incoming serial data is written to
#else // SD_FAT_TYPE == 0
SdFat sd;
File sensorDataFile; //File that all sensor data is written to
File serialDataFile; //File that all incoming serial data is written to
#endif  // SD_FAT_TYPE

//#define PRINT_LAST_WRITE_TIME // Uncomment this line to enable the 'measure the time between writes' diagnostic

char sensorDataFileName[30] = ""; //We keep a record of this file name so that we can re-open it upon wakeup from sleep
char serialDataFileName[30] = ""; //We keep a record of this file name so that we can re-open it upon wakeup from sleep
const int sdPowerDownDelay = 100; //Delay for this many ms before turning off the SD card power
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Add RTC interface for Artemis
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include "RTC.h" //Include RTC library included with the Aruino_Apollo3 core
Apollo3RTC myRTC; //Create instance of RTC class
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Create UART instance for OpenLog style serial logging
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//UART SerialLog(BREAKOUT_PIN_TX, BREAKOUT_PIN_RX);  // Declares a Uart object called SerialLog with TX on pin 12 and RX on pin 13

uint64_t lastSeriaLogSyncTime = 0;
uint64_t lastAwakeTimeMillis;
const int MAX_IDLE_TIME_MSEC = 500;
bool newSerialData = false;
char incomingBuffer[256 * 2]; //This size of this buffer is sensitive. Do not change without analysis using OpenLog_Serial.
int incomingBufferSpot = 0;
int charsReceived = 0; //Used for verifying/debugging serial reception
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Add ICM IMU interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
ICM_20948_SPI myICM;
icm_20948_DMP_data_t dmpData; // Global storage for the DMP data - extracted from the FIFO
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Header files for all compatible Qwiic sensors
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#include "SparkFun_I2C_Mux_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
#include "SparkFunCCS811.h" //Click here to get the library: http://librarymanager/All#SparkFun_CCS811
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include "SparkFunBME280.h" //Click here to get the library: http://librarymanager/All#SparkFun_BME280
#include "SparkFun_LPS25HB_Arduino_Library.h"  //Click here to get the library: http://librarymanager/All#SparkFun_LPS25HB
#include "SparkFun_VEML6075_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_VEML6075
#include "SparkFun_PHT_MS8607_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_PHT_MS8607
#include "SparkFun_MCP9600.h" //Click here to get the library: http://librarymanager/All#SparkFun_MCP9600
#include "SparkFun_SGP30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SGP30
#include "SparkFun_VCNL4040_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_VCNL4040
#include "SparkFun_MS5637_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_MS5637
#include "SparkFun_TMP117.h" //Click here to get the library: http://librarymanager/All#SparkFun_TMP117
#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_NAU7802
#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include "SparkFun_Qwiic_Humidity_AHT20.h" //Click here to get the library: http://librarymanager/All#Qwiic_Humidity_AHT20 by SparkFun
#include "SparkFun_SHTC3.h" // Click here to get the library: http://librarymanager/All#SparkFun_SHTC3
#include "SparkFun_ADS122C04_ADC_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_ADS122C04
#include "SparkFun_MicroPressure.h" // Click here to get the library: http://librarymanager/All#SparkFun_MicroPressure
#include "SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Particle_Sensor_SN-GCJA5
#include "SparkFun_SGP40_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SGP40
#include "SparkFun_SDP3x_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SDP3x
#include "MS5837.h" // Click here to download the library: https://github.com/sparkfunX/BlueRobotics_MS5837_Library
#include "SparkFun_Qwiic_Button.h" // Click here to get the library: http://librarymanager/All#SparkFun_Qwiic_Button_Switch
#include "SparkFun_Bio_Sensor_Hub_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Bio_Sensor
#include "SparkFun_ISM330DHCX.h" // Click here to get the library: http://librarymanager/All#SparkFun_6DoF_ISM330DHCX
#include "SparkFun_MMC5983MA_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA
#include "SparkFun_ADS1015_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_ADS1015
#include "SparkFun_KX13X.h" //Click here to get the library: http://librarymanager/All#SparkFun_KX13X
#include "SparkFun_LPS28DFW_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_LPS28DFW_Arduino_Library
#include "SparkFun_VEML7700_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_VEML7700
#include "PCF8575.h"  // Click here to download the library: https://github.com/jcoo18/PCF8575-lib/tree/wip
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
uint64_t measurementStartTime; //Used to calc the actual update rate. Max is ~80,000,000ms in a 24 hour period.
uint64_t lastSDFileNameChangeTime; //Used to calculate the interval since the last SD filename change
unsigned long measurementCount = 0; //Used to calc the actual update rate.
unsigned long measurementTotal = 0; //The total number of recorded measurements. (Doesn't get reset when the menu is opened)
char sdOutputData[512 * 2]; //Factor of 512 for easier recording to SD in 512 chunks
unsigned long lastReadTime = 0; //Used to delay until user wants to record a new reading
unsigned long lastDataLogSyncTime = 0; //Used to record to SD every half second
unsigned int totalCharactersPrinted = 0; //Limit output rate based on baud rate and number of characters to print
bool takeReading = true; //Goes true when enough time has passed between readings or we've woken from sleep
bool sleepAfterRead = false; //Used to keep the code awake for at least minimumAwakeTimeMillis
const uint64_t maxUsBeforeSleep = 2000000ULL; //Number of us between readings before sleep is activated.
const byte menuTimeout = 15; //Menus will exit/timeout after this number of seconds
const int sdCardMenuTimeout = 60; // sdCard menu will exit/timeout after this number of seconds
volatile static bool stopLoggingSeen = false; //Flag to indicate if we should stop logging
uint64_t qwiicPowerOnTime = 0; //Used to delay after Qwiic power on to allow sensors to power on, then answer autodetect
unsigned long qwiicPowerOnDelayMillis; //Wait for this many milliseconds after turning on the Qwiic power before attempting to communicate with Qwiic devices
int lowBatteryReadings = 0; // Count how many times the battery voltage has read low
const int lowBatteryReadingsLimit = 10; // Don't declare the battery voltage low until we have had this many consecutive low readings (to reject sampling noise)
volatile static bool triggerEdgeSeen = false; //Flag to indicate if a trigger interrupt has been seen
char serialTimestamp[50]; //Buffer to store serial timestamp, if needed
volatile static bool powerLossSeen = false; //Flag to indicate if a power loss event has been seen
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// gfvalvo's flash string helper code: https://forum.arduino.cc/index.php?topic=533118.msg3634809#msg3634809
void SerialPrint(const char *);
void SerialPrint(const __FlashStringHelper *);
void SerialPrintln(const char *);
void SerialPrintln(const __FlashStringHelper *);
void DoSerialPrint(char (*)(const char *), const char *, bool newLine = false);

#define DUMP( varname ) {Serial.printf("%s: %d\r\n", #varname, varname); if (settings.useTxRxPinsForTerminal == true) Serial1.printf("%s: %d\r\n", #varname, varname);}
#define SerialPrintf1( var ) {Serial.printf( var ); if (settings.useTxRxPinsForTerminal == true) Serial1.printf( var );}
#define SerialPrintf2( var1, var2 ) {Serial.printf( var1, var2 ); if (settings.useTxRxPinsForTerminal == true) Serial1.printf( var1, var2 );}
#define SerialPrintf3( var1, var2, var3 ) {Serial.printf( var1, var2, var3 ); if (settings.useTxRxPinsForTerminal == true) Serial1.printf( var1, var2, var3 );}
#define SerialPrintf4( var1, var2, var3, var4 ) {Serial.printf( var1, var2, var3, var4 ); if (settings.useTxRxPinsForTerminal == true) Serial1.printf( var1, var2, var3, var4 );}
#define SerialPrintf5( var1, var2, var3, var4, var5 ) {Serial.printf( var1, var2, var3, var4, var5 ); if (settings.useTxRxPinsForTerminal == true) Serial1.printf( var1, var2, var3, var4, var5 );}

// The Serial port for the Zmodem connection
// must not be the same as DSERIAL unless all
// debugging output to DSERIAL is removed
Stream *ZSERIAL;

// Serial output for debugging info for Zmodem
Stream *DSERIAL;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include "WDT.h" // WDT support

volatile static bool petTheDog = true; // Flag to control whether the WDT ISR pets (resets) the timer.

// Interrupt handler for the watchdog.
extern "C" void am_watchdog_isr(void)
{
  // Clear the watchdog interrupt.
  wdt.clear();

  // Restart the watchdog if petTheDog is true
  if (petTheDog)
    wdt.restart(); // "Pet" the dog.
}

void startWatchdog()
{
  // Set watchdog timer clock to 16 Hz
  // Set watchdog interrupt to 1 seconds (16 ticks / 16 Hz = 1 second)
  // Set watchdog reset to 1.25 seconds (20 ticks / 16 Hz = 1.25 seconds)
  // Note: Ticks are limited to 255 (8-bit)
  wdt.configure(WDT_16HZ, 16, 20);
  wdt.start(); // Start the watchdog
}

void stopWatchdog()
{
  wdt.stop();
}
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void setup() {
  //If 3.3V rail drops below 3V, system will power down and maintain RTC
  pinMode(PIN_POWER_LOSS, INPUT); // BD49K30G-TL has CMOS output and does not need a pull-up

  delay(1); // Let PIN_POWER_LOSS stabilize

#ifndef noPowerLossProtection
  if (digitalRead(PIN_POWER_LOSS) == LOW) powerDownOLA(); //Check PIN_POWER_LOSS just in case we missed the falling edge
  //attachInterrupt(PIN_POWER_LOSS, powerDownOLA, FALLING); // We can't do this with v2.1.0 as attachInterrupt causes a spontaneous interrupt
  attachInterrupt(PIN_POWER_LOSS, powerLossISR, FALLING);
#else
  // No Power Loss Protection
  // Set up the WDT to generate a reset just in case the code crashes during a brown-out
  startWatchdog();
#endif
  powerLossSeen = false; // Make sure the flag is clear

  powerLEDOn(); // Turn the power LED on - if the hardware supports it

  pinMode(PIN_STAT_LED, OUTPUT);
  digitalWrite(PIN_STAT_LED, HIGH); // Turn the STAT LED on while we configure everything

  SPI.begin(); //Needed if SD is disabled

  //Do not start Serial1 before productionTest() otherwise the pin configuration gets overwritten
  //and subsequent Serial1.begin's don't restore the pins to UART mode...

  productionTest(); //Check if we need to go into production test mode

  //We need to manually restore the Serial1 TX and RX pins after they were changed by productionTest()
  configureSerial1TxRx();

  Serial.begin(115200); //Default for initial debug messages if necessary
  Serial1.begin(115200); //Default for initial debug messages if necessary

  //pinMode(PIN_LOGIC_DEBUG, OUTPUT); // Debug pin to assist tracking down slippery mux bugs
  //digitalWrite(PIN_LOGIC_DEBUG, HIGH);

  // Use the worst case power on delay for the Qwiic bus for now as we don't yet know what sensors are connected
  // (worstCaseQwiicPowerOnDelay is defined in settings.h)
  qwiicPowerOnDelayMillis = worstCaseQwiicPowerOnDelay;

  EEPROM.init();

  beginQwiic(); // Turn the qwiic power on as early as possible

  beginSD(); //285 - 293ms

  enableCIPOpullUp(); // Enable CIPO pull-up _after_ beginSD

  loadSettings(); //50 - 250ms

  if (settings.useTxRxPinsForTerminal == true)
  {
    Serial1.flush(); //Complete any previous prints at the previous baud rate
    Serial1.begin(settings.serialTerminalBaudRate); // Restart the serial port
  }
  else
  {
    Serial1.flush(); //Complete any previous prints
    if (settings.logSerial == false)
      Serial1.end(); // Stop the SerialLog port - but only if not logging serial, otherwise incoming data can cause a crash!
  }

  Serial.flush(); //Complete any previous prints
  Serial.begin(settings.serialTerminalBaudRate);

  SerialPrintf3("Artemis OpenLog v%d.%d\r\n", FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);

#ifdef noPowerLossProtection
  SerialPrintln(F("** No Power Loss Protection **"));
#endif

  if (settings.useGPIO32ForStopLogging == true)
  {
    SerialPrintln(F("Stop Logging is enabled. Pull GPIO pin 32 to GND to stop logging."));
    pinMode(PIN_STOP_LOGGING, INPUT_PULLUP);
    delay(1); // Let the pin stabilize
    attachInterrupt(PIN_STOP_LOGGING, stopLoggingISR, FALLING); // Enable the interrupt
    am_hal_gpio_pincfg_t intPinConfig = g_AM_HAL_GPIO_INPUT_PULLUP;
    intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
    pin_config(PinName(PIN_STOP_LOGGING), intPinConfig); // Make sure the pull-up does actually stay enabled
    stopLoggingSeen = false; // Make sure the flag is clear
  }

  if (settings.useGPIO11ForTrigger == true)
  {
    pinMode(PIN_TRIGGER, INPUT_PULLUP);
    delay(1); // Let the pin stabilize
    am_hal_gpio_pincfg_t intPinConfig = g_AM_HAL_GPIO_INPUT_PULLUP;
    if (settings.fallingEdgeTrigger == true)
    {
      SerialPrintln(F("Falling-edge triggering is enabled. Sensor data will be logged on a falling edge on GPIO pin 11."));
      attachInterrupt(PIN_TRIGGER, triggerPinISR, FALLING); // Enable the interrupt
      intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
    }
    else
    {
      SerialPrintln(F("Rising-edge triggering is enabled. Sensor data will be logged on a rising edge on GPIO pin 11."));
      attachInterrupt(PIN_TRIGGER, triggerPinISR, RISING); // Enable the interrupt
      intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
    }
    pin_config(PinName(PIN_TRIGGER), intPinConfig); // Make sure the pull-up does actually stay enabled
    triggerEdgeSeen = false; // Make sure the flag is clear
  }

  analogReadResolution(14); //Increase from default of 10

  beginDataLogging(); //180ms
  lastSDFileNameChangeTime = rtcMillis(); // Record the time of the file name change

  serialTimestamp[0] = '\0'; // Empty the serial timestamp buffer

  if (settings.useTxRxPinsForTerminal == false)
  {
    beginSerialLogging(); //20 - 99ms
    beginSerialOutput(); // Begin serial data output on the TX pin
  }

  beginIMU(); //61ms

  if (online.microSD == true) SerialPrintln(F("SD card online"));
  else SerialPrintln(F("SD card offline"));

  if (online.dataLogging == true) SerialPrintln(F("Data logging online"));
  else SerialPrintln(F("Datalogging offline"));

  if (online.serialLogging == true) SerialPrintln(F("Serial logging online"));
  else SerialPrintln(F("Serial logging offline"));

  if (online.IMU == true) SerialPrintln(F("IMU online"));
  else SerialPrintln(F("IMU offline - or not present"));

  if (settings.logMaxRate == true) SerialPrintln(F("Logging analog pins at max data rate"));

  if (settings.enableTerminalOutput == false && settings.logData == true) SerialPrintln(F("Logging to microSD card with no terminal output"));

  if (detectQwiicDevices() == true) //159 - 865ms but varies based on number of devices attached
  {
    beginQwiicDevices(); //Begin() each device in the node list
    loadDeviceSettingsFromFile(); //Load config settings into node list
    configureQwiicDevices(); //Apply config settings to each device in the node list
    int deviceCount = printOnlineDevice(); // Pretty-print the online devices

    if ((deviceCount == 0) && (settings.resetOnZeroDeviceCount == true)) // Check for resetOnZeroDeviceCount
    {
      if ((Serial.available()) || ((settings.useTxRxPinsForTerminal == true) && (Serial1.available())))
        menuMain(); //Present user menu - in case the user wants to disable resetOnZeroDeviceCount
      else
      {
        SerialPrintln(F("*** Zero Qwiic Devices Found! Resetting... ***"));
        SerialFlush();
        resetArtemis(); //Thank you and goodnight...
      }
    }
  }
  else
    SerialPrintln(F("No Qwiic devices detected"));

  // KDB add
  // If we are streaming to Serial, start the stream with a Mime Type marker, followed by CR
  SerialPrintln(F("Content-Type: text/csv"));
  SerialPrintln("");
  
  if (settings.showHelperText == true) 
    printHelperText(OL_OUTPUT_SERIAL | OL_OUTPUT_SDCARD); //printHelperText to terminal and sensor file

  //If we are sleeping between readings then we cannot rely on millis() as it is powered down
  //Use RTC instead
  measurementStartTime = rtcMillis();

  digitalWrite(PIN_STAT_LED, LOW); // Turn the STAT LED off now that everything is configured

  lastAwakeTimeMillis = rtcMillis();

  //If we are immediately going to go to sleep after the first reading then
  //first present the user with the config menu in case they need to change something
  if (checkIfItIsTimeToSleep())
    menuMain(true); // Always open the menu - even if there is nothing in the serial buffers
}

void loop() {

  checkBattery(); // Check for low battery

  if ((Serial.available()) || ((settings.useTxRxPinsForTerminal == true) && (Serial1.available())))
    menuMain(); //Present user menu

  if (settings.logSerial == true && online.serialLogging == true && settings.useTxRxPinsForTerminal == false)
  {
    size_t timestampCharsLeftToWrite = strlen(serialTimestamp);
    //SerialPrintf2("timestampCharsLeftToWrite is %d\r\n", timestampCharsLeftToWrite);
    //SerialFlush();

    if (Serial1.available() || (timestampCharsLeftToWrite > 0))
    {
      while (Serial1.available() || (timestampCharsLeftToWrite > 0))
      {
        if (timestampCharsLeftToWrite > 0) // Based on code written by @DennisMelamed in PR #70
        {
          incomingBuffer[incomingBufferSpot++] = serialTimestamp[0]; // Add a timestamp character to incomingBuffer

          for (size_t i = 0; i < timestampCharsLeftToWrite; i++)
          {
            serialTimestamp[i] = serialTimestamp[i+1]; // Shuffle the remaining chars along by one, including the NULL terminator
          }

          timestampCharsLeftToWrite -= 1; // Now decrement timestampCharsLeftToWrite
        }
        else
        {
          incomingBuffer[incomingBufferSpot++] = Serial1.read();
          charsReceived++;

          //Get the RTC timestamp if we just received the timestamp token
          if (settings.timestampSerial && (incomingBuffer[incomingBufferSpot-1] == settings.timeStampToken))
          {
            getTimeString(&serialTimestamp[2]);
            serialTimestamp[0] = 0x0A; // Add Line Feed at the start of the timestamp
            serialTimestamp[1] = '^'; // Add an up-arrow to indicate the timestamp relates to the preceeding data
            serialTimestamp[strlen(serialTimestamp) - 1] = 0x0A; // Change the final comma of the timestamp to a Line Feed
            timestampCharsLeftToWrite = strlen(serialTimestamp); // Update timestampCharsLeftToWrite now, so timestamp is printed immediately (#192)
          }
        }

        if (incomingBufferSpot == sizeof(incomingBuffer))
        {
          digitalWrite(PIN_STAT_LED, HIGH); //Toggle stat LED to indicating log recording
          serialDataFile.write(incomingBuffer, sizeof(incomingBuffer)); //Record the buffer to the card
          digitalWrite(PIN_STAT_LED, LOW);
          incomingBufferSpot = 0;
        }
        checkBattery();
      }

      //If we are sleeping between readings then we cannot rely on millis() as it is powered down
      //Use RTC instead
      lastSeriaLogSyncTime = rtcMillis(); //Reset the last sync time to now
      newSerialData = true;
    }
    else if (newSerialData == true)
    {
      if ((rtcMillis() - lastSeriaLogSyncTime) > MAX_IDLE_TIME_MSEC) //If we haven't received any characters recently then sync log file
      {
        if (incomingBufferSpot > 0)
        {
          //Write the remainder of the buffer
          digitalWrite(PIN_STAT_LED, HIGH); //Toggle stat LED to indicating log recording
          serialDataFile.write(incomingBuffer, incomingBufferSpot); //Record the buffer to the card
          serialDataFile.sync();
          if (settings.frequentFileAccessTimestamps == true)
            updateDataFileAccess(&serialDataFile); // Update the file access time & date
          digitalWrite(PIN_STAT_LED, LOW);

          incomingBufferSpot = 0;
        }

        newSerialData = false;
        lastSeriaLogSyncTime = rtcMillis(); //Reset the last sync time to now
        printDebug("Total chars received: " + (String)charsReceived + "\r\n");
      }
    }
  }

  //In v2.1 of the core micros() becomes corrupted during deep sleep so only test if we are not sleeping
  if (settings.usBetweenReadings < maxUsBeforeSleep)
  {
    if ((micros() - lastReadTime) >= settings.usBetweenReadings)
      takeReading = true;
  }

  //Check for a trigger event
  if (settings.useGPIO11ForTrigger == true)
  {
    if (triggerEdgeSeen == true)
    {
      takeReading = true; // If triggering is enabled and a trigger event has been seen, then take a reading.
    }
    else
    {
      takeReading = false; // If triggering is enabled and a trigger even has not been seen, then make sure we don't take a reading based on settings.usBetweenReadings.
    }
  }

  //Is it time to get new data?
  if ((settings.logMaxRate == true) || (takeReading == true))
  {
    takeReading = false;
    lastReadTime = micros();

#ifdef PRINT_LAST_WRITE_TIME
    if (settings.printDebugMessages)
    {
      // Print how long it has been since the last write
      char tempTimeRev[20]; // Char array to hold to usBR (reversed order)
      char tempTime[20]; // Char array to hold to usBR (correct order)
      static uint64_t lastWriteTime; //Used to calculate the time since the last SD write (sleep-proof)
      unsigned long usBR = rtcMillis() - lastWriteTime;
      unsigned int i = 0;
      if (usBR == 0ULL) // if usBetweenReadings is zero, set tempTime to "0"
      {
        tempTime[0] = '0';
        tempTime[1] = 0;
      }
      else
      {
        while (usBR > 0)
        {
          tempTimeRev[i++] = (usBR % 10) + '0'; // divide by 10, convert the remainder to char
          usBR /= 10; // divide by 10
        }
        unsigned int j = 0;
        while (i > 0)
        {
          tempTime[j++] = tempTimeRev[--i]; // reverse the order
          tempTime[j] = 0; // mark the end with a NULL
        }
      }

      //printDebug("ms since last write: " + (String)tempTime + "\r\n");
      printDebug((String)tempTime + "\r\n");

      lastWriteTime = rtcMillis();
    }
#endif

    getData(sdOutputData, sizeof(sdOutputData)); //Query all enabled sensors for data

    //Print to terminal
    if (settings.enableTerminalOutput == true)
      SerialPrint(sdOutputData); //Print to terminal

    //Output to TX pin
    if ((settings.outputSerial == true) && (online.serialOutput == true))
      Serial1.print(sdOutputData); //Print to TX pin

    //Record to SD
    if ((settings.logData == true) && (online.microSD))
    {
      digitalWrite(PIN_STAT_LED, HIGH);
      uint32_t recordLength = sensorDataFile.write(sdOutputData, strlen(sdOutputData));
      if (recordLength != strlen(sdOutputData)) //Record the buffer to the card
      {
        if (settings.printDebugMessages == true)
        {
          SerialPrintf3("*** sensorDataFile.write data length mismatch! *** recordLength: %d, outputDataLength: %d\r\n", recordLength, strlen(sdOutputData));
        }
      }

      //Force sync every 500ms
      if (rtcMillis() - lastDataLogSyncTime > 500)
      {
        lastDataLogSyncTime = rtcMillis();
        sensorDataFile.sync();
        if (settings.frequentFileAccessTimestamps == true)
          updateDataFileAccess(&sensorDataFile); // Update the file access time & date
      }

      //Check if it is time to open a new log file
      uint64_t secsSinceLastFileNameChange = rtcMillis() - lastSDFileNameChangeTime; // Calculate how long we have been logging for
      secsSinceLastFileNameChange /= 1000ULL; // Convert to secs
      if ((settings.openNewLogFilesAfter > 0) && (((unsigned long)secsSinceLastFileNameChange) >= settings.openNewLogFilesAfter))
      {
        //Close existings files
        if (online.dataLogging == true)
        {
          sensorDataFile.sync();
          updateDataFileAccess(&sensorDataFile); // Update the file access time & date
          sensorDataFile.close();
          strcpy(sensorDataFileName, findNextAvailableLog(settings.nextDataLogNumber, "dataLog"));
          beginDataLogging(); //180ms
          if (settings.showHelperText == true) 
            printHelperText(OL_OUTPUT_SDCARD); //printHelperText to the sensor file
        }
        if (online.serialLogging == true)
        {
          serialDataFile.sync();
          updateDataFileAccess(&serialDataFile); // Update the file access time & date
          serialDataFile.close();
          strcpy(serialDataFileName, findNextAvailableLog(settings.nextSerialLogNumber, "serialLog"));
          beginSerialLogging();
        }

        lastSDFileNameChangeTime = rtcMillis(); // Record the time of the file name change
      }

      digitalWrite(PIN_STAT_LED, LOW);
    }

    if ((settings.useGPIO32ForStopLogging == true) && (stopLoggingSeen == true)) // Has the user pressed the stop logging button?
    {
      stopLogging();
    }

    triggerEdgeSeen = false; // Clear the trigger seen flag here - just in case another trigger was received while we were logging data to SD card

    // Code changes here are based on suggestions by @ryanneve in Issue #46, PR #64 and Issue #83
    if (checkIfItIsTimeToSleep())
    {
      sleepAfterRead = true;
    }
  }

  if (sleepAfterRead == true)
  {
    // Check if we should stay awake because settings.minimumAwakeTimeMillis is non-zero
    if ((settings.usBetweenReadings >= maxUsBeforeSleep) && (settings.minimumAwakeTimeMillis > 0))
    {
      // Check if we have been awake long enough (millis is reset to zero when waking from sleep)
      // goToSleep will automatically compensate for how long we have been awake
      if ((rtcMillis() - lastAwakeTimeMillis) < settings.minimumAwakeTimeMillis)
        return; // Too early to sleep - leave sleepAfterRead set true
    }

    sleepAfterRead = false;
    goToSleep(howLongToSleepFor());
  }
}

uint32_t howLongToSleepFor(void)
{
  //Counter/Timer 6 will use the 32kHz clock
  //Calculate how many 32768Hz system ticks we need to sleep for:
  //sysTicksToSleep = msToSleep * 32768L / 1000
  //We need to be careful with the multiply as we will overflow uint32_t if msToSleep is > 131072

  //goToSleep will automatically compensate for how long we have been awake

  uint32_t msToSleep;

  if (checkSleepOnFastSlowPin())
    msToSleep = (uint32_t)(settings.slowLoggingIntervalSeconds * 1000UL);
  else if (checkSleepOnRTCTime())
  {
    // checkSleepOnRTCTime has returned true, so we know that we are between slowLoggingStartMOD and slowLoggingStopMOD
    // We need to check how long it is until slowLoggingStopMOD (accounting for midnight!) and adjust the sleep duration
    // if slowLoggingStopMOD occurs before slowLoggingIntervalSeconds

    msToSleep = (uint32_t)(settings.slowLoggingIntervalSeconds * 1000UL); // Default to this

    myRTC.getTime(); // Get the RTC time
    long secondsOfDay = (myRTC.hour * 60 * 60) + (myRTC.minute * 60) + myRTC.seconds;

    long slowLoggingStopSOD = settings.slowLoggingStopMOD * 60; // Convert slowLoggingStop to seconds-of-day

    long secondsUntilStop = slowLoggingStopSOD - secondsOfDay; // Calculate how long it is until slowLoggingStop

    // If secondsUntilStop is negative then we know that now is before midnight and slowLoggingStop is after midnight
    if (secondsUntilStop < 0) secondsUntilStop += 24 * 60 * 60; // Add a day's worth of seconds if required to make secondsUntilStop positive

    if (secondsUntilStop < settings.slowLoggingIntervalSeconds) // If we need to sleep for less than slowLoggingIntervalSeconds
      msToSleep = (secondsUntilStop + 1) * 1000UL; // Adjust msToSleep, adding one extra second to make sure the next wake is > slowLoggingStop
  }
  else // checkSleepOnUsBetweenReadings
  {
    msToSleep = (uint32_t)(settings.usBetweenReadings / 1000ULL); // Sleep for usBetweenReadings
  }

  uint32_t sysTicksToSleep;
  if (msToSleep < 131000)
  {
    sysTicksToSleep = msToSleep * 32768L; // Do the multiply first for short intervals
    sysTicksToSleep = sysTicksToSleep / 1000L; // Now do the divide
  }
  else
  {
    sysTicksToSleep = msToSleep / 1000L; // Do the division first for long intervals (to avoid an overflow)
    sysTicksToSleep = sysTicksToSleep * 32768L; // Now do the multiply
  }

  return (sysTicksToSleep);
}

bool checkIfItIsTimeToSleep(void)
{

  if (checkSleepOnUsBetweenReadings()
  || checkSleepOnRTCTime()
  || checkSleepOnFastSlowPin())
    return(true);
  else
    return(false);
}

//Go to sleep if the time between readings is greater than maxUsBeforeSleep (2 seconds) and triggering is not enabled
bool checkSleepOnUsBetweenReadings(void)
{
  if ((settings.useGPIO11ForTrigger == false) && (settings.usBetweenReadings >= maxUsBeforeSleep))
    return (true);
  else
    return (false);
}

//Go to sleep if Fast/Slow logging on Pin 11 is enabled and Pin 11 is in the correct state
bool checkSleepOnFastSlowPin(void)
{
  if ((settings.useGPIO11ForFastSlowLogging == true) && (digitalRead(PIN_TRIGGER) == settings.slowLoggingWhenPin11Is))
    return (true);
  else
    return (false);
}

// Go to sleep if useRTCForFastSlowLogging is enabled and RTC time is between the start and stop times
bool checkSleepOnRTCTime(void)
{
  // Check if we should be sleeping based on useGPIO11ForFastSlowLogging and slowLoggingStartMOD + slowLoggingStopMOD
  bool sleepOnRTCTime = false;
  if (settings.useRTCForFastSlowLogging == true)
  {
    if (settings.slowLoggingStartMOD != settings.slowLoggingStopMOD) // Only perform the check if the start and stop times are not equal
    {
      myRTC.getTime(); // Get the RTC time
      int minutesOfDay = (myRTC.hour * 60) + myRTC.minute;

      if (settings.slowLoggingStartMOD > settings.slowLoggingStopMOD) // If slow logging starts later than the stop time (i.e. slow over midnight)
      {
        if ((minutesOfDay >= settings.slowLoggingStartMOD) || (minutesOfDay < settings.slowLoggingStopMOD))
          sleepOnRTCTime = true;
      }
      else // Slow logging starts earlier than the stop time
      {
        if ((minutesOfDay >= settings.slowLoggingStartMOD) && (minutesOfDay < settings.slowLoggingStopMOD))
          sleepOnRTCTime = true;
      }
    }
  }
  return(sleepOnRTCTime);
}

void beginQwiic()
{
  pinMode(PIN_QWIIC_POWER, OUTPUT);
  pin_config(PinName(PIN_QWIIC_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  qwiicPowerOn();
  qwiic.begin();
  setQwiicPullups(settings.qwiicBusPullUps); //Just to make it really clear what pull-ups are being used, set pullups here.
}

void setQwiicPullups(uint32_t qwiicBusPullUps)
{
  //Change SCL and SDA pull-ups manually using pin_config
  am_hal_gpio_pincfg_t sclPinCfg = g_AM_BSP_GPIO_IOM1_SCL;
  am_hal_gpio_pincfg_t sdaPinCfg = g_AM_BSP_GPIO_IOM1_SDA;

  if (qwiicBusPullUps == 0)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE; // No pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE;
  }
  else if (qwiicBusPullUps == 1)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K; // Use 1K5 pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
  }
  else if (qwiicBusPullUps == 6)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K; // Use 6K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K;
  }
  else if (qwiicBusPullUps == 12)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_12K; // Use 12K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_12K;
  }
  else
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_24K; // Use 24K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_24K;
  }

  pin_config(PinName(PIN_QWIIC_SCL), sclPinCfg);
  pin_config(PinName(PIN_QWIIC_SDA), sdaPinCfg);
}

void beginSD(bool silent)
{
  pinMode(PIN_MICROSD_POWER, OUTPUT);
  pin_config(PinName(PIN_MICROSD_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  pinMode(PIN_MICROSD_CHIP_SELECT, OUTPUT);
  pin_config(PinName(PIN_MICROSD_CHIP_SELECT), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_MICROSD_CHIP_SELECT, HIGH); //Be sure SD is deselected

  // If the microSD card is present, it needs to be powered on otherwise the IMU will fail to start
  // (The microSD card will pull the SPI pins low, preventing communication with the IMU)

  // For reasons I don't understand, we seem to have to wait for at least 1ms after SPI.begin before we call microSDPowerOn.
  // If you comment the next line, the Artemis resets at microSDPowerOn when beginSD is called from wakeFromSleep...
  // But only on one of my V10 red boards. The second one I have doesn't seem to need the delay!?
  delay(5);

  microSDPowerOn();

  //Max power up time is 250ms: https://www.kingston.com/datasheets/SDCIT-specsheet-64gb_en.pdf
  //Max current is 200mA average across 1s, peak 300mA
  for (int i = 0; i < 10; i++) //Wait
  {
    checkBattery();
    delay(1);
  }

  if (sd.begin(SD_CONFIG) == false) // Try to begin the SD card using the correct chip select
  {
    printDebug(F("SD init failed (first attempt). Trying again...\r\n"));
    for (int i = 0; i < 250; i++) //Give SD more time to power up, then try again
    {
      checkBattery();
      delay(1);
    }
    if (sd.begin(SD_CONFIG) == false) // Try to begin the SD card using the correct chip select
    {
      if (!silent)
      {
        SerialPrintln(F("SD init failed (second attempt). Is card present? Formatted?"));
        SerialPrintln(F("Please ensure the SD card is formatted correctly using https://www.sdcard.org/downloads/formatter/"));
      }
      digitalWrite(PIN_MICROSD_CHIP_SELECT, HIGH); //Be sure SD is deselected
      online.microSD = false;
      return;
    }
  }

  //Change to root directory. All new file creation will be in root.
  if (sd.chdir() == false)
  {
    if (!silent)
    {
      SerialPrintln(F("SD change directory failed"));
    }
    online.microSD = false;
    return;
  }

  online.microSD = true;
}

void enableCIPOpullUp() // updated for v2.1.0 of the Apollo3 core
{
  //Add 1K5 pull-up on CIPO
  am_hal_gpio_pincfg_t cipoPinCfg = g_AM_BSP_GPIO_IOM0_MISO;
  cipoPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
  pin_config(PinName(PIN_SPI_CIPO), cipoPinCfg);
}

void disableCIPOpullUp() // updated for v2.1.0 of the Apollo3 core
{
  am_hal_gpio_pincfg_t cipoPinCfg = g_AM_BSP_GPIO_IOM0_MISO;
  pin_config(PinName(PIN_SPI_CIPO), cipoPinCfg);
}

void configureSerial1TxRx(void) // Configure pins 12 and 13 for UART1 TX and RX
{
  am_hal_gpio_pincfg_t pinConfigTx = g_AM_BSP_GPIO_COM_UART_TX;
  pinConfigTx.uFuncSel = AM_HAL_PIN_12_UART1TX;
  pin_config(PinName(BREAKOUT_PIN_TX), pinConfigTx);
  am_hal_gpio_pincfg_t pinConfigRx = g_AM_BSP_GPIO_COM_UART_RX;
  pinConfigRx.uFuncSel = AM_HAL_PIN_13_UART1RX;
  pinConfigRx.ePullup = AM_HAL_GPIO_PIN_PULLUP_WEAK; // Put a weak pull-up on the Rx pin
  pin_config(PinName(BREAKOUT_PIN_RX), pinConfigRx);
}

void beginIMU(bool silent)
{
  pinMode(PIN_IMU_POWER, OUTPUT);
  pin_config(PinName(PIN_IMU_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  pinMode(PIN_IMU_CHIP_SELECT, OUTPUT);
  pin_config(PinName(PIN_IMU_CHIP_SELECT), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_IMU_CHIP_SELECT, HIGH); //Be sure IMU is deselected

  if (settings.enableIMU == true && settings.logMaxRate == false)
  {
    //Reset ICM by power cycling it
    imuPowerOff();
    for (int i = 0; i < 10; i++) //10 is fine
    {
      checkBattery();
      delay(1);
    }
    imuPowerOn();
    for (int i = 0; i < 25; i++) //Allow ICM to come online. Typical is 11ms. Max is 100ms. https://cdn.sparkfun.com/assets/7/f/e/c/d/DS-000189-ICM-20948-v1.3.pdf
    {
      checkBattery();
      delay(1);
    }

    if (settings.printDebugMessages) myICM.enableDebugging();
    myICM.begin(PIN_IMU_CHIP_SELECT, SPI, 4000000); //Set IMU SPI rate to 4MHz
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      printDebug("beginIMU: first attempt at myICM.begin failed. myICM.status = " + (String)myICM.status + "\r\n");
      //Try one more time with longer wait

      //Reset ICM by power cycling it
      imuPowerOff();
      for (int i = 0; i < 10; i++) //10 is fine
      {
        checkBattery();
        delay(1);
      }
      imuPowerOn();
      for (int i = 0; i < 100; i++) //Allow ICM to come online. Typical is 11ms. Max is 100ms.
      {
        checkBattery();
        delay(1);
      }

      myICM.begin(PIN_IMU_CHIP_SELECT, SPI, 4000000); //Set IMU SPI rate to 4MHz
      if (myICM.status != ICM_20948_Stat_Ok)
      {
        printDebug("beginIMU: second attempt at myICM.begin failed. myICM.status = " + (String)myICM.status + "\r\n");
        digitalWrite(PIN_IMU_CHIP_SELECT, HIGH); //Be sure IMU is deselected
        if (!silent)
          SerialPrintln(F("ICM-20948 failed to init."));
        imuPowerOff();
        online.IMU = false;
        return;
      }
    }

    //Give the IMU extra time to get its act together. This seems to fix the IMU-not-starting-up-cleanly-after-sleep problem...
    //Seems to need a full 25ms. 10ms is not enough.
    for (int i = 0; i < 25; i++) //Allow ICM to come online.
    {
      checkBattery();
      delay(1);
    }

    bool success = true;

    //Check if we are using the DMP
    if (settings.imuUseDMP == false)
    {
      //Perform a full startup (not minimal) for non-DMP mode
      ICM_20948_Status_e retval = myICM.startupDefault(false);
      if (retval != ICM_20948_Stat_Ok)
      {
        SerialPrintln(F("Error: Could not startup the IMU in non-DMP mode!"));
        success = false;
      }
      //Update the full scale and DLPF settings
      retval = myICM.enableDLPF(ICM_20948_Internal_Acc, settings.imuAccDLPF);
      if (retval != ICM_20948_Stat_Ok)
      {
        SerialPrintln(F("Error: Could not configure the IMU Accelerometer DLPF!"));
        success = false;
      }
      retval = myICM.enableDLPF(ICM_20948_Internal_Gyr, settings.imuGyroDLPF);
      if (retval != ICM_20948_Stat_Ok)
      {
        SerialPrintln(F("Error: Could not configure the IMU Gyro DLPF!"));
        success = false;
      }
      ICM_20948_dlpcfg_t dlpcfg;
      dlpcfg.a = settings.imuAccDLPFBW;
      dlpcfg.g = settings.imuGyroDLPFBW;
      retval = myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
      if (retval != ICM_20948_Stat_Ok)
      {
        SerialPrintln(F("Error: Could not configure the IMU DLPF BW!"));
        success = false;
      }
      ICM_20948_fss_t FSS;
      FSS.a = settings.imuAccFSS;
      FSS.g = settings.imuGyroFSS;
      retval = myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
      if (retval != ICM_20948_Stat_Ok)
      {
        SerialPrintln(F("Error: Could not configure the IMU Full Scale!"));
        success = false;
      }
    }
    else
    {
      // Initialize the DMP
      ICM_20948_Status_e retval = myICM.initializeDMP();
      if (retval != ICM_20948_Stat_Ok)
      {
        SerialPrintln(F("Error: Could not startup the IMU in DMP mode!"));
        success = false;
      }

      int ODR = 0; // Set ODR to 55Hz
      if (settings.usBetweenReadings >= 500000ULL)
        ODR = 3; // 17Hz ODR rate when DMP is running at 55Hz
      if (settings.usBetweenReadings >= 1000000ULL)
        ODR = 10; // 5Hz ODR rate when DMP is running at 55Hz

      if (settings.imuLogDMPQuat6)
      {
        retval = myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
        if (retval != ICM_20948_Stat_Ok)
        {
          SerialPrintln(F("Error: Could not enable the Game Rotation Vector (Quat6)!"));
          success = false;
        }
        retval = myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, ODR);
        if (retval != ICM_20948_Stat_Ok)
        {
          SerialPrintln(F("Error: Could not set the Quat6 ODR!"));
          success = false;
        }
      }
      if (settings.imuLogDMPQuat9)
      {
        retval = myICM.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR);
        if (retval != ICM_20948_Stat_Ok)
        {
          SerialPrintln(F("Error: Could not enable the Rotation Vector (Quat9)!"));
          success = false;
        }
        retval = myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, ODR);
        if (retval != ICM_20948_Stat_Ok)
        {
          SerialPrintln(F("Error: Could not set the Quat9 ODR!"));
          success = false;
        }
      }
      if (settings.imuLogDMPAccel)
      {
        retval = myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER);
        if (retval != ICM_20948_Stat_Ok)
        {
          SerialPrintln(F("Error: Could not enable the DMP Accelerometer!"));
          success = false;
        }
        retval = myICM.setDMPODRrate(DMP_ODR_Reg_Accel, ODR);
        if (retval != ICM_20948_Stat_Ok)
        {
          SerialPrintln(F("Error: Could not set the Accel ODR!"));
          success = false;
        }
      }
      if (settings.imuLogDMPGyro)
      {
        retval = myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE);
        if (retval != ICM_20948_Stat_Ok)
        {
          SerialPrintln(F("Error: Could not enable the DMP Gyroscope!"));
          success = false;
        }
        retval = myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, ODR);
        if (retval != ICM_20948_Stat_Ok)
        {
          SerialPrintln(F("Error: Could not set the Gyro ODR!"));
          success = false;
        }
        retval = myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, ODR);
        if (retval != ICM_20948_Stat_Ok)
        {
          SerialPrintln(F("Error: Could not set the Gyro Calibr ODR!"));
          success = false;
        }
      }
      if (settings.imuLogDMPCpass)
      {
        retval = myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED);
        if (retval != ICM_20948_Stat_Ok)
        {
          SerialPrintln(F("Error: Could not enable the DMP Compass!"));
          success = false;
        }
        retval = myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, ODR);
        if (retval != ICM_20948_Stat_Ok)
        {
          SerialPrintln(F("Error: Could not set the Compass ODR!"));
          success = false;
        }
        retval = myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, ODR);
        if (retval != ICM_20948_Stat_Ok)
        {
          SerialPrintln(F("Error: Could not set the Compass Calibr ODR!"));
          success = false;
        }
      }
      retval = myICM.enableFIFO(); // Enable the FIFO
      if (retval != ICM_20948_Stat_Ok)
      {
        SerialPrintln(F("Error: Could not enable the FIFO!"));
        success = false;
      }
      retval = myICM.enableDMP(); // Enable the DMP
      if (retval != ICM_20948_Stat_Ok)
      {
        SerialPrintln(F("Error: Could not enable the DMP!"));
        success = false;
      }
      retval = myICM.resetDMP(); // Reset the DMP
      if (retval != ICM_20948_Stat_Ok)
      {
        SerialPrintln(F("Error: Could not reset the DMP!"));
        success = false;
      }
      retval = myICM.resetFIFO(); // Reset the FIFO
      if (retval != ICM_20948_Stat_Ok)
      {
        SerialPrintln(F("Error: Could not reset the FIFO!"));
        success = false;
      }
    }

    if (success)
    {
      online.IMU = true;
      delay(50); // Give the IMU time to get its first measurement ready
    }
    else
    {
      //Power down IMU
      imuPowerOff();
      online.IMU = false;
    }
  }
  else
  {
    //Power down IMU
    imuPowerOff();
    online.IMU = false;
  }
}

void beginDataLogging()
{
  if (online.microSD == true && settings.logData == true)
  {
    //If we don't have a file yet, create one. Otherwise, re-open the last used file
    if (strlen(sensorDataFileName) == 0)
      strcpy(sensorDataFileName, findNextAvailableLog(settings.nextDataLogNumber, "dataLog"));

    // O_CREAT - create the file if it does not exist
    // O_APPEND - seek to the end of the file prior to each write
    // O_WRITE - open for write
    if (sensorDataFile.open(sensorDataFileName, O_CREAT | O_APPEND | O_WRITE) == false)
    {
      SerialPrintln(F("Failed to create sensor data file"));
      online.dataLogging = false;
      return;
    }

    updateDataFileCreate(&sensorDataFile); // Update the file create time & date
    sensorDataFile.sync();

    online.dataLogging = true;
  }
  else
    online.dataLogging = false;
}

void beginSerialLogging()
{
  if (online.microSD == true && settings.logSerial == true)
  {
    //If we don't have a file yet, create one. Otherwise, re-open the last used file
    if (strlen(serialDataFileName) == 0)
      strcpy(serialDataFileName, findNextAvailableLog(settings.nextSerialLogNumber, "serialLog"));

    if (serialDataFile.open(serialDataFileName, O_CREAT | O_APPEND | O_WRITE) == false)
    {
      SerialPrintln(F("Failed to create serial log file"));
      //systemError(ERROR_FILE_OPEN);
      online.serialLogging = false;
      return;
    }

    updateDataFileCreate(&serialDataFile); // Update the file create time & date
    serialDataFile.sync();

    //We need to manually restore the Serial1 TX and RX pins
    configureSerial1TxRx();

    Serial1.begin(settings.serialLogBaudRate);

    online.serialLogging = true;
  }
  else
    online.serialLogging = false;
}

void beginSerialOutput()
{
  if (settings.outputSerial == true)
  {
    //We need to manually restore the Serial1 TX and RX pins
    configureSerial1TxRx();

    Serial1.begin(settings.serialLogBaudRate); // (Re)start the serial port
    online.serialOutput = true;
  }
  else
    online.serialOutput = false;
}

#if SD_FAT_TYPE == 1
void updateDataFileCreate(File32 *dataFile)
#elif SD_FAT_TYPE == 2
void updateDataFileCreate(ExFile *dataFile)
#elif SD_FAT_TYPE == 3
void updateDataFileCreate(FsFile *dataFile)
#else // SD_FAT_TYPE == 0
void updateDataFileCreate(File *dataFile)
#endif  // SD_FAT_TYPE
{
  myRTC.getTime(); //Get the RTC time so we can use it to update the create time
  //Update the file create time
  dataFile->timestamp(T_CREATE, (myRTC.year + 2000), myRTC.month, myRTC.dayOfMonth, myRTC.hour, myRTC.minute, myRTC.seconds);
}

#if SD_FAT_TYPE == 1
void updateDataFileAccess(File32 *dataFile)
#elif SD_FAT_TYPE == 2
void updateDataFileAccess(ExFile *dataFile)
#elif SD_FAT_TYPE == 3
void updateDataFileAccess(FsFile *dataFile)
#else // SD_FAT_TYPE == 0
void updateDataFileAccess(File *dataFile)
#endif  // SD_FAT_TYPE
{
  myRTC.getTime(); //Get the RTC time so we can use it to update the last modified time
  //Update the file access time
  dataFile->timestamp(T_ACCESS, (myRTC.year + 2000), myRTC.month, myRTC.dayOfMonth, myRTC.hour, myRTC.minute, myRTC.seconds);
  //Update the file write time
  dataFile->timestamp(T_WRITE, (myRTC.year + 2000), myRTC.month, myRTC.dayOfMonth, myRTC.hour, myRTC.minute, myRTC.seconds);
}

//Called once number of milliseconds has passed
extern "C" void am_stimer_cmpr6_isr(void)
{
  uint32_t ui32Status = am_hal_stimer_int_status_get(false);
  if (ui32Status & AM_HAL_STIMER_INT_COMPAREG)
  {
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREG);
  }
}

//Power Loss ISR
void powerLossISR(void)
{
  powerLossSeen = true;
}

//Stop Logging ISR
void stopLoggingISR(void)
{
  stopLoggingSeen = true;
}

//Trigger Pin ISR
void triggerPinISR(void)
{
  triggerEdgeSeen = true;
}

void SerialFlush(void)
{
  Serial.flush();
  if (settings.useTxRxPinsForTerminal == true)
  {
    Serial1.flush();
  }
}

// gfvalvo's flash string helper code: https://forum.arduino.cc/index.php?topic=533118.msg3634809#msg3634809

void SerialPrint(const char *line)
{
  DoSerialPrint([](const char *ptr) {return *ptr;}, line);
}

void SerialPrint(const __FlashStringHelper *line)
{
  DoSerialPrint([](const char *ptr) {return (char) pgm_read_byte_near(ptr);}, (const char*) line);
}

void SerialPrintln(const char *line)
{
  DoSerialPrint([](const char *ptr) {return *ptr;}, line, true);
}

void SerialPrintln(const __FlashStringHelper *line)
{
  DoSerialPrint([](const char *ptr) {return (char) pgm_read_byte_near(ptr);}, (const char*) line, true);
}

void DoSerialPrint(char (*funct)(const char *), const char *string, bool newLine)
{
  char ch;

  while ((ch = funct(string++)))
  {
    Serial.print(ch);
    if (settings.useTxRxPinsForTerminal == true)
      Serial1.print(ch);
  }

  if (newLine)
  {
    Serial.println();
    if (settings.useTxRxPinsForTerminal == true)
      Serial1.println();
  }
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\Sensors.ino"



#include "Sensors.h"

#define HELPER_BUFFER_SIZE 1024

//Query each enabled sensor for its most recent data
void getData(char* sdOutputData, size_t lenData)
{
  measurementCount++;
  measurementTotal++;

  char tempData[50];
  char tempData1[16];
  char tempData2[16];
  char tempData3[16];
  sdOutputData[0] = '\0'; //Clear string contents

  if (settings.logRTC)
  {
    //Code written by @DennisMelamed in PR #70
    char timeString[37];
    getTimeString(timeString); // getTimeString is in timeStamp.ino
    strlcat(sdOutputData, timeString, lenData);
  }

  if (settings.logA11)
  {
    unsigned int analog11 = analogRead(11);

    if (settings.logAnalogVoltages == true)
    {
      float voltage = analog11 * 2 / 16384.0;
      olaftoa(voltage, tempData1, 2, sizeof(tempData1) / sizeof(char));
      sprintf(tempData, "%s,", tempData1);
    }
    else
      sprintf(tempData, "%d,", analog11);

    strlcat(sdOutputData, tempData, lenData);
  }

  if (settings.logA12)
  {
    unsigned int analog12 = analogRead(12);

    if (settings.logAnalogVoltages == true)
    {
      float voltage = analog12 * 2 / 16384.0;
      olaftoa(voltage, tempData1, 2, sizeof(tempData1) / sizeof(char));
      sprintf(tempData, "%s,", tempData1);
    }
    else
      sprintf(tempData, "%d,", analog12);

    strlcat(sdOutputData, tempData, lenData);
  }

  if (settings.logA13)
  {
    unsigned int analog13 = analogRead(13);

    if (settings.logAnalogVoltages == true)
    {
      float voltage = analog13 * 2 / 16384.0;
      olaftoa(voltage, tempData1, 2, sizeof(tempData1) / sizeof(char));
      sprintf(tempData, "%s,", tempData1);
    }
    else
      sprintf(tempData, "%d,", analog13);

    strlcat(sdOutputData, tempData, lenData);
  }

  if (settings.logA32)
  {
    unsigned int analog32 = analogRead(32);

    if (settings.logAnalogVoltages == true)
    {
      float voltage = analog32 * 2 / 16384.0;
      olaftoa(voltage, tempData1, 2, sizeof(tempData1) / sizeof(char));
      sprintf(tempData, "%s,", tempData1);
    }
    else
      sprintf(tempData, "%d,", analog32);

    strlcat(sdOutputData, tempData, lenData);
  }

  if (settings.logVIN)
  {
    float voltage = readVIN();
    olaftoa(voltage, tempData1, 2, sizeof(tempData1) / sizeof(char));
    sprintf(tempData, "%s,", tempData1);
    strlcat(sdOutputData, tempData, lenData);
  }

  if (online.IMU)
  {
    //printDebug("getData: online.IMU = " + (String)online.IMU + "\r\n");

    if (settings.imuUseDMP == false)
    {
      if (myICM.dataReady())
      {
        //printDebug("getData: myICM.dataReady = " + (String)myICM.dataReady() + "\r\n");

        myICM.getAGMT(); //Update values

        if (settings.logIMUAccel)
        {
          olaftoa(myICM.accX(), tempData1, 2, sizeof(tempData1) / sizeof(char));
          olaftoa(myICM.accY(), tempData2, 2, sizeof(tempData2) / sizeof(char));
          olaftoa(myICM.accZ(), tempData3, 2, sizeof(tempData3) / sizeof(char));
          sprintf(tempData, "%s,%s,%s,", tempData1, tempData2, tempData3);
          strlcat(sdOutputData, tempData, lenData);
        }
        if (settings.logIMUGyro)
        {
          olaftoa(myICM.gyrX(), tempData1, 2, sizeof(tempData1) / sizeof(char));
          olaftoa(myICM.gyrY(), tempData2, 2, sizeof(tempData2) / sizeof(char));
          olaftoa(myICM.gyrZ(), tempData3, 2, sizeof(tempData3) / sizeof(char));
          sprintf(tempData, "%s,%s,%s,", tempData1, tempData2, tempData3);
          strlcat(sdOutputData, tempData, lenData);
        }
        if (settings.logIMUMag)
        {
          olaftoa(myICM.magX(), tempData1, 2, sizeof(tempData1) / sizeof(char));
          olaftoa(myICM.magY(), tempData2, 2, sizeof(tempData2) / sizeof(char));
          olaftoa(myICM.magZ(), tempData3, 2, sizeof(tempData3) / sizeof(char));
          sprintf(tempData, "%s,%s,%s,", tempData1, tempData2, tempData3);
          strlcat(sdOutputData, tempData, lenData);
        }
        if (settings.logIMUTemp)
        {
          olaftoa(myICM.temp(), tempData1, 2, sizeof(tempData1) / sizeof(char));
          sprintf(tempData, "%s,", tempData1);
          strlcat(sdOutputData, tempData, lenData);
        }
      }
      //else
      //{
      //  printDebug("getData: myICM.dataReady = " + (String)myICM.dataReady() + "\r\n");
      //}
    }
    else
    {
      myICM.readDMPdataFromFIFO(&dmpData);
      while (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)
      {
        myICM.readDMPdataFromFIFO(&dmpData); // Empty the FIFO - make sure data contains the most recent data
      }
      if (settings.imuLogDMPQuat6)
      {
        olaftoa(((double)dmpData.Quat6.Data.Q1) / 1073741824.0, tempData1, 5, sizeof(tempData1) / sizeof(char));
        olaftoa(((double)dmpData.Quat6.Data.Q2) / 1073741824.0, tempData2, 5, sizeof(tempData2) / sizeof(char));
        olaftoa(((double)dmpData.Quat6.Data.Q3) / 1073741824.0, tempData3, 5, sizeof(tempData3) / sizeof(char));
        sprintf(tempData, "%s,%s,%s,", tempData1, tempData2, tempData3);
        strlcat(sdOutputData, tempData, lenData);
      }
      if (settings.imuLogDMPQuat9)
      {
        olaftoa(((double)dmpData.Quat9.Data.Q1) / 1073741824.0, tempData1, 5, sizeof(tempData1) / sizeof(char));
        olaftoa(((double)dmpData.Quat9.Data.Q2) / 1073741824.0, tempData2, 5, sizeof(tempData2) / sizeof(char));
        olaftoa(((double)dmpData.Quat9.Data.Q3) / 1073741824.0, tempData3, 5, sizeof(tempData3) / sizeof(char));
        sprintf(tempData, "%s,%s,%s,%d,", tempData1, tempData2, tempData3, dmpData.Quat9.Data.Accuracy);
        strlcat(sdOutputData, tempData, lenData);
      }
      if (settings.imuLogDMPAccel)
      {
        sprintf(tempData, "%d,%d,%d,", dmpData.Raw_Accel.Data.X, dmpData.Raw_Accel.Data.Y, dmpData.Raw_Accel.Data.Z);
        strlcat(sdOutputData, tempData, lenData);
      }
      if (settings.imuLogDMPGyro)
      {
        sprintf(tempData, "%d,%d,%d,", dmpData.Raw_Gyro.Data.X, dmpData.Raw_Gyro.Data.Y, dmpData.Raw_Gyro.Data.Z);
        strlcat(sdOutputData, tempData, lenData);
      }
      if (settings.imuLogDMPCpass)
      {
        sprintf(tempData, "%d,%d,%d,", dmpData.Compass.Data.X, dmpData.Compass.Data.Y, dmpData.Compass.Data.Z);
        strlcat(sdOutputData, tempData, lenData);
      }
    }
  }

  //Append all external sensor data on linked list to sdOutputData
  gatherDeviceValues(sdOutputData, lenData);

  if (settings.logHertz)
  {
    uint64_t currentMillis;

    //If we are sleeping between readings then we cannot rely on millis() as it is powered down
    //Use RTC instead
    currentMillis = rtcMillis();
    float actualRate;
    if ((currentMillis - measurementStartTime) < 1) // Avoid divide by zero
      actualRate = 0.0;
    else
      actualRate = measurementCount * 1000.0 / (currentMillis - measurementStartTime);
    olaftoa(actualRate, tempData1, 3, sizeof(tempData) / sizeof(char));
    sprintf(tempData, "%s,", tempData1);
    strlcat(sdOutputData, tempData, lenData);
  }

  if (settings.printMeasurementCount)
  {
    sprintf(tempData, "%d,", measurementTotal);
    strlcat(sdOutputData, tempData, lenData);
  }

  strlcat(sdOutputData, "\r\n", lenData);

  totalCharactersPrinted += strlen(sdOutputData);
}

//Read values from the devices on the node list
//Append values to sdOutputData
void gatherDeviceValues(char * sdOutputData, size_t lenData)
{
  char tempData[100];
  char tempData1[20];
  char tempData2[20];
  char tempData3[20];

  //Step through list, printing values as we go
  node *temp = head;
  while (temp != NULL)
  {
    //If this node successfully begin()'d
    if (temp->online == true)
    {
      openConnection(temp->muxAddress, temp->portNumber); //Connect to this device through muxes as needed

      //Switch on device type to set proper class and setting struct
      switch (temp->deviceType)
      {
        case DEVICE_MULTIPLEXER:
          {
            //No data to print for a mux
          }
          break;
        case DEVICE_LOADCELL_NAU7802:
          {
            NAU7802 *nodeDevice = (NAU7802 *)temp->classPtr;
            struct_NAU7802 *nodeSetting = (struct_NAU7802 *)temp->configPtr; //Create a local pointer that points to same spot as node does

            if (nodeSetting->log == true)
            {
              float currentWeight = nodeDevice->getWeight(true, nodeSetting->averageAmount); //Allow negative weights, take average of X readings
              olaftoa(currentWeight, tempData1, nodeSetting->decimalPlaces, sizeof(tempData1) / sizeof(char));
              sprintf(tempData, "%s,", tempData1);
              strlcat(sdOutputData, tempData, lenData);
            }
          }
          break;
        case DEVICE_DISTANCE_VL53L1X:
          {
            SFEVL53L1X *nodeDevice = (SFEVL53L1X *)temp->classPtr;
            struct_VL53L1X *nodeSetting = (struct_VL53L1X *)temp->configPtr;

            if (nodeSetting->log == true)
            {
              if (nodeSetting->logDistance)
              {
                sprintf(tempData, "%d,", nodeDevice->getDistance());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logRangeStatus)
              {
                sprintf(tempData, "%d,", nodeDevice->getRangeStatus());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logSignalRate)
              {
                sprintf(tempData, "%d,", nodeDevice->getSignalRate());
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_GPS_UBLOX:
          {
            setQwiicPullups(0); //Disable pullups to minimize CRC issues

            SFE_UBLOX_GNSS *nodeDevice = (SFE_UBLOX_GNSS *)temp->classPtr;
            struct_ublox *nodeSetting = (struct_ublox *)temp->configPtr;

            if (nodeSetting->log == true)
            {
              if (nodeSetting->logDate)
              {
                char gnssDayStr[3];
                char gnssMonthStr[3];
                char gnssYearStr[5];
                int gnssDay = nodeDevice->getDay();
                int gnssMonth = nodeDevice->getMonth();
                int gnssYear = nodeDevice->getYear();
                if (gnssDay < 10)
                  sprintf(gnssDayStr, "0%d", gnssDay);
                else
                  sprintf(gnssDayStr, "%d", gnssDay);
                if (gnssMonth < 10)
                  sprintf(gnssMonthStr, "0%d", gnssMonth);
                else
                  sprintf(gnssMonthStr, "%d", gnssMonth);
                sprintf(gnssYearStr, "%d", gnssYear);
                if (settings.dateStyle == 0)
                {
                  sprintf(tempData, "%s/%s/%s,", gnssMonthStr, gnssDayStr, gnssYearStr);
                }
                else if (settings.dateStyle == 1)
                {
                  sprintf(tempData, "%s/%s/%s,", gnssDayStr, gnssMonthStr, gnssYearStr);
                }
                else // if (settings.dateStyle == 2)
                {
                  sprintf(tempData, "%s/%s/%s,", gnssYearStr, gnssMonthStr, gnssDayStr);
                }
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logTime)
              {
                int adjustedHour = nodeDevice->getHour();
                if (settings.hour24Style == false)
                  if (adjustedHour > 12) adjustedHour -= 12;

                char gnssHourStr[3];
                char gnssMinStr[3];
                char gnssSecStr[3];
                char gnssMillisStr[4];
                int gnssMin = nodeDevice->getMinute();
                int gnssSec = nodeDevice->getSecond();
                int gnssMillis = nodeDevice->getMillisecond();

                if (adjustedHour < 10)
                  sprintf(gnssHourStr, "0%d", adjustedHour);
                else
                  sprintf(gnssHourStr, "%d", adjustedHour);
                if (gnssMin < 10)
                  sprintf(gnssMinStr, "0%d", gnssMin);
                else
                  sprintf(gnssMinStr, "%d", gnssMin);
                if (gnssSec < 10)
                  sprintf(gnssSecStr, "0%d", gnssSec);
                else
                  sprintf(gnssSecStr, "%d", gnssSec);
                if (gnssMillis < 10)
                  sprintf(gnssMillisStr, "00%d", gnssMillis);
                else if (gnssMillis < 100)
                  sprintf(gnssMillisStr, "0%d", gnssMillis);
                else
                  sprintf(gnssMillisStr, "%d", gnssMillis);

                sprintf(tempData, "%s:%s:%s.%s,", gnssHourStr, gnssMinStr, gnssSecStr, gnssMillisStr);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logPosition)
              {
                sprintf(tempData, "%d,%d,", nodeDevice->getLatitude(), nodeDevice->getLongitude());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logAltitude)
              {
                sprintf(tempData, "%d,", nodeDevice->getAltitude());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logAltitudeMSL)
              {
                sprintf(tempData, "%d,", nodeDevice->getAltitudeMSL());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logSIV)
              {
                sprintf(tempData, "%d,", nodeDevice->getSIV());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logFixType)
              {
                sprintf(tempData, "%d,", nodeDevice->getFixType());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logCarrierSolution)
              {
                sprintf(tempData, "%d,", nodeDevice->getCarrierSolutionType());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logGroundSpeed)
              {
                sprintf(tempData, "%d,", nodeDevice->getGroundSpeed());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logHeadingOfMotion)
              {
                sprintf(tempData, "%d,", nodeDevice->getHeading());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logpDOP)
              {
                sprintf(tempData, "%d,", nodeDevice->getPDOP());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logiTOW)
              {
                sprintf(tempData, "%d,", nodeDevice->getTimeOfWeek());
                strlcat(sdOutputData, tempData, lenData);
              }
            }

            setQwiicPullups(settings.qwiicBusPullUps); //Re-enable pullups
          }
          break;
        case DEVICE_PROXIMITY_VCNL4040:
          {
            VCNL4040 *nodeDevice = (VCNL4040 *)temp->classPtr;
            struct_VCNL4040 *nodeSetting = (struct_VCNL4040 *)temp->configPtr;

            //Get ambient takes 80ms minimum and may not play properly with power cycling
            if (nodeSetting->log == true)
            {
              if (nodeSetting->logProximity)
              {
                sprintf(tempData, "%d,", nodeDevice->getProximity());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logAmbientLight)
              {
                sprintf(tempData, "%d,", nodeDevice->getAmbient());
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_TEMPERATURE_TMP117:
          {
            TMP117 *nodeDevice = (TMP117 *)temp->classPtr;
            struct_TMP117 *nodeSetting = (struct_TMP117 *)temp->configPtr;

            if (nodeSetting->log == true)
            {
              if (nodeSetting->logTemperature)
              {
                olaftoa(nodeDevice->readTempC(), tempData1, 4, sizeof(tempData1) / sizeof(char)); //Resolution to 0.0078°C, accuracy of 0.1°C
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_PRESSURE_MS5637:
          {
            MS5637 *nodeDevice = (MS5637 *)temp->classPtr;
            struct_MS5637 *nodeSetting = (struct_MS5637 *)temp->configPtr;

            if (nodeSetting->log == true)
            {
              if (nodeSetting->logPressure)
              {
                olaftoa(nodeDevice->getPressure(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logTemperature)
              {
                olaftoa(nodeDevice->getTemperature(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_PRESSURE_LPS25HB:
          {
            LPS25HB *nodeDevice = (LPS25HB *)temp->classPtr;
            struct_LPS25HB *nodeSetting = (struct_LPS25HB *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              if (nodeSetting->logPressure)
              {
                olaftoa(nodeDevice->getPressure_hPa(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logTemperature)
              {
                olaftoa(nodeDevice->getTemperature_degC(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_PRESSURE_LPS28DFW:
          {
            LPS28DFW *nodeDevice = (LPS28DFW *)temp->classPtr;
            struct_LPS28DFW *nodeSetting = (struct_LPS28DFW *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              nodeDevice->getSensorData();

              if (nodeSetting->logPressure)
              {
                olaftoa(nodeDevice->data.pressure.hpa, tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logTemperature)
              {
                olaftoa(nodeDevice->data.heat.deg_c, tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_PHT_BME280:
          {
            BME280 *nodeDevice = (BME280 *)temp->classPtr;
            struct_BME280 *nodeSetting = (struct_BME280 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              if (nodeSetting->logPressure)
              {
                olaftoa(nodeDevice->readFloatPressure(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logHumidity)
              {
                olaftoa(nodeDevice->readFloatHumidity(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logAltitude)
              {
                olaftoa(nodeDevice->readFloatAltitudeMeters(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logTemperature)
              {
                olaftoa(nodeDevice->readTempC(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_UV_VEML6075:
          {
            VEML6075 *nodeDevice = (VEML6075 *)temp->classPtr;
            struct_VEML6075 *nodeSetting = (struct_VEML6075 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              if (nodeSetting->logUVA)
              {
                olaftoa(nodeDevice->uva(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logUVB)
              {
                olaftoa(nodeDevice->uvb(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logUVIndex)
              {
                olaftoa(nodeDevice->index(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_LIGHT_VEML7700:
          {
            VEML7700 *nodeDevice = (VEML7700 *)temp->classPtr;
            struct_VEML7700 *nodeSetting = (struct_VEML7700 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              olaftoa(nodeDevice->getLux(), tempData1, 2, sizeof(tempData1) / sizeof(char));
              sprintf(tempData, "%s,", tempData1);
              strlcat(sdOutputData, tempData, lenData);
            }
          }
          break;

        case DEVICE_VOC_CCS811:
          {
            CCS811 *nodeDevice = (CCS811 *)temp->classPtr;
            struct_CCS811 *nodeSetting = (struct_CCS811 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              nodeDevice->readAlgorithmResults();
              if (nodeSetting->logTVOC)
              {
                sprintf(tempData, "%d,", nodeDevice->getTVOC());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logCO2)
              {
                sprintf(tempData, "%d,", nodeDevice->getCO2());
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_VOC_SGP30:
          {
            SGP30 *nodeDevice = (SGP30 *)temp->classPtr;
            struct_SGP30 *nodeSetting = (struct_SGP30 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              nodeDevice->measureAirQuality();
              nodeDevice->measureRawSignals(); //To get H2 and Ethanol

              if (nodeSetting->logTVOC)
              {
                sprintf(tempData, "%d,", nodeDevice->TVOC);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logCO2)
              {
                sprintf(tempData, "%d,", nodeDevice->CO2);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logH2)
              {
                sprintf(tempData, "%d,", nodeDevice->H2);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logEthanol)
              {
                sprintf(tempData, "%d,", nodeDevice->ethanol);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_CO2_SCD30:
          {
            SCD30 *nodeDevice = (SCD30 *)temp->classPtr;
            struct_SCD30 *nodeSetting = (struct_SCD30 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              if (nodeSetting->logCO2)
              {
                sprintf(tempData, "%d,", nodeDevice->getCO2());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logHumidity)
              {
                olaftoa(nodeDevice->getHumidity(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logTemperature)
              {
                olaftoa(nodeDevice->getTemperature(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_PHT_MS8607:
          {
            MS8607 *nodeDevice = (MS8607 *)temp->classPtr;
            struct_MS8607 *nodeSetting = (struct_MS8607 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              if (nodeSetting->logHumidity)
              {
                olaftoa(nodeDevice->getHumidity(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logPressure)
              {
                olaftoa(nodeDevice->getPressure(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logTemperature)
              {
                olaftoa(nodeDevice->getTemperature(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_TEMPERATURE_MCP9600:
          {
            MCP9600 *nodeDevice = (MCP9600 *)temp->classPtr;
            struct_MCP9600 *nodeSetting = (struct_MCP9600 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              if (nodeSetting->logTemperature)
              {
                olaftoa(nodeDevice->getThermocoupleTemp(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logAmbientTemperature)
              {
                olaftoa(nodeDevice->getAmbientTemp(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_HUMIDITY_AHT20:
          {
            AHT20 *nodeDevice = (AHT20 *)temp->classPtr;
            struct_AHT20 *nodeSetting = (struct_AHT20 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              if (nodeSetting->logHumidity)
              {
                olaftoa(nodeDevice->getHumidity(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logTemperature)
              {
                olaftoa(nodeDevice->getTemperature(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_HUMIDITY_SHTC3:
          {
            SHTC3 *nodeDevice = (SHTC3 *)temp->classPtr;
            struct_SHTC3 *nodeSetting = (struct_SHTC3 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              nodeDevice->update();
              if (nodeSetting->logHumidity)
              {
                olaftoa(nodeDevice->toPercent(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logTemperature)
              {
                olaftoa(nodeDevice->toDegC(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_ADC_ADS122C04:
          {
            SFE_ADS122C04 *nodeDevice = (SFE_ADS122C04 *)temp->classPtr;
            struct_ADS122C04 *nodeSetting = (struct_ADS122C04 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              // The ADS122C04 supports sampling up to 2kHz but the library functions default to 20Hz.
              // To be able to log faster than 20Hz we need to use setDataRate to change the data rate (sample speed).
              // Note: readInternalTemperature and readRawVoltage are hard wired to 20Hz in the library and
              //       - at the moment - there's nothing we can do about that! If you want to log faster than
              //       20Hz, you'll need to disable readInternalTemperature and readRawVoltage.
              // At the time of writing, the maximum achieveable sample rate is ~156Hz.

              //It looks like configureDevice will take care of this. No need to do it here.
              //if (nodeSetting->useFourWireMode)
              //  nodeDevice->configureADCmode(ADS122C04_4WIRE_MODE);
              //else if (nodeSetting->useThreeWireMode)
              //  nodeDevice->configureADCmode(ADS122C04_3WIRE_MODE);
              //else if (nodeSetting->useTwoWireMode)
              //  nodeDevice->configureADCmode(ADS122C04_2WIRE_MODE);
              //else if (nodeSetting->useFourWireHighTemperatureMode)
              //  nodeDevice->configureADCmode(ADS122C04_4WIRE_HI_TEMP);
              //else if (nodeSetting->useThreeWireHighTemperatureMode)
              //  nodeDevice->configureADCmode(ADS122C04_3WIRE_HI_TEMP);
              //else if (nodeSetting->useTwoWireHighTemperatureMode)
              //  nodeDevice->configureADCmode(ADS122C04_2WIRE_HI_TEMP);

              if (settings.usBetweenReadings < 50000ULL) // Check if we are trying to sample quicker than 20Hz
              {
                if (settings.usBetweenReadings <= 1000ULL) // Check if we are trying to sample at 1kHz
                  nodeDevice->setDataRate(ADS122C04_DATA_RATE_1000SPS);
                else if (settings.usBetweenReadings <= 1667ULL) // Check if we are trying to sample at 600Hz
                  nodeDevice->setDataRate(ADS122C04_DATA_RATE_600SPS);
                else if (settings.usBetweenReadings <= 3031ULL) // Check if we are trying to sample at 330Hz
                  nodeDevice->setDataRate(ADS122C04_DATA_RATE_330SPS);
                else if (settings.usBetweenReadings <= 5715ULL) // Check if we are trying to sample at 175Hz
                  nodeDevice->setDataRate(ADS122C04_DATA_RATE_175SPS);
                else if (settings.usBetweenReadings <= 11112ULL) // Check if we are trying to sample at 90Hz
                  nodeDevice->setDataRate(ADS122C04_DATA_RATE_90SPS);
                else if (settings.usBetweenReadings <= 22223ULL) // Check if we are trying to sample at 45Hz
                  nodeDevice->setDataRate(ADS122C04_DATA_RATE_45SPS);
              }
              else
                nodeDevice->setDataRate(ADS122C04_DATA_RATE_20SPS); // Default to 20Hz

              if (nodeSetting->logCentigrade)
              {
                olaftoa(nodeDevice->readPT100Centigrade(), tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logFahrenheit)
              {
                olaftoa(nodeDevice->readPT100Fahrenheit(), tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logInternalTemperature)
              {
                olaftoa(nodeDevice->readInternalTemperature(), tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logRawVoltage)
              {
                sprintf(tempData, "%d,", nodeDevice->readRawVoltage());
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_PRESSURE_MPR0025PA1:
          {
            SparkFun_MicroPressure *nodeDevice = (SparkFun_MicroPressure *)temp->classPtr;
            struct_MPR0025PA1 *nodeSetting = (struct_MPR0025PA1 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              if (nodeSetting->usePSI)
              {
                olaftoa(nodeDevice->readPressure(), tempData1, 4, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->usePA)
              {
                olaftoa(nodeDevice->readPressure(PA), tempData1, 1, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->useKPA)
              {
                olaftoa(nodeDevice->readPressure(KPA), tempData1, 4, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->useTORR)
              {
                olaftoa(nodeDevice->readPressure(TORR), tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->useINHG)
              {
                olaftoa(nodeDevice->readPressure(INHG), tempData1, 4, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->useATM)
              {
                olaftoa(nodeDevice->readPressure(ATM), tempData1, 6, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->useBAR)
              {
                olaftoa(nodeDevice->readPressure(BAR), tempData1, 6, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_PARTICLE_SNGCJA5:
          {
            SFE_PARTICLE_SENSOR *nodeDevice = (SFE_PARTICLE_SENSOR *)temp->classPtr;
            struct_SNGCJA5 *nodeSetting = (struct_SNGCJA5 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              if (nodeSetting->logPM1)
              {
                olaftoa(nodeDevice->getPM1_0(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logPM25)
              {
                olaftoa(nodeDevice->getPM2_5(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logPM10)
              {
                olaftoa(nodeDevice->getPM10(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logPC05)
              {
                sprintf(tempData, "%d,", nodeDevice->getPC0_5());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logPC1)
              {
                sprintf(tempData, "%d,", nodeDevice->getPC1_0());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logPC25)
              {
                sprintf(tempData, "%d,", nodeDevice->getPC2_5());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logPC50)
              {
                sprintf(tempData, "%d,", nodeDevice->getPC5_0());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logPC75)
              {
                sprintf(tempData, "%d,", nodeDevice->getPC7_5());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logPC10)
              {
                sprintf(tempData, "%d,", nodeDevice->getPC10());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logSensorStatus)
              {
                sprintf(tempData, "%d,", nodeDevice->getStatusSensors());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logPDStatus)
              {
                sprintf(tempData, "%d,", nodeDevice->getStatusPD());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logLDStatus)
              {
                sprintf(tempData, "%d,", nodeDevice->getStatusLD());
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logFanStatus)
              {
                sprintf(tempData, "%d,", nodeDevice->getStatusFan());
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_VOC_SGP40:
          {
            SGP40 *nodeDevice = (SGP40 *)temp->classPtr;
            struct_SGP40 *nodeSetting = (struct_SGP40 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              if (nodeSetting->logVOC)
              {
                sprintf(tempData, "%d,", nodeDevice->getVOCindex(nodeSetting->RH, nodeSetting->T));
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_PRESSURE_SDP3X:
          {
            SDP3X *nodeDevice = (SDP3X *)temp->classPtr;
            struct_SDP3X *nodeSetting = (struct_SDP3X *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              float pressure;
              float temperature;
              if ((nodeSetting->logPressure) || (nodeSetting->logTemperature))
              {
                // Each triggered measurement takes 45ms to complete so we need to use continuous measurements
                nodeDevice->readMeasurement(&pressure, &temperature); // Read the latest measurement
              }
              if (nodeSetting->logPressure)
              {
                olaftoa(pressure, tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logTemperature)
              {
                olaftoa(temperature, tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_PRESSURE_MS5837:
          {
            MS5837 *nodeDevice = (MS5837 *)temp->classPtr;
            struct_MS5837 *nodeSetting = (struct_MS5837 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              if ((nodeSetting->logPressure) || (nodeSetting->logTemperature) || (nodeSetting->logDepth) || (nodeSetting->logAltitude))
              {
                nodeDevice->read();
              }
              if (nodeSetting->logPressure)
              {
                olaftoa(nodeDevice->pressure(nodeSetting->conversion), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logTemperature)
              {
                olaftoa(nodeDevice->temperature(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logDepth)
              {
                olaftoa(nodeDevice->depth(), tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logAltitude)
              {
                olaftoa(nodeDevice->altitude(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_QWIIC_BUTTON:
          {
            QwiicButton *nodeDevice = (QwiicButton *)temp->classPtr;
            struct_QWIIC_BUTTON *nodeSetting = (struct_QWIIC_BUTTON *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              long pressedPopped = 0;
              while (nodeDevice->isPressedQueueEmpty() == false)
              {
                pressedPopped = nodeDevice->popPressedQueue();
              }
              if (nodeSetting->logPressed)
              {
                olaftoa(((float)pressedPopped) / 1000.0, tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }

              long clickedPopped = 0;
              while (nodeDevice->isClickedQueueEmpty() == false)
              {
                clickedPopped = nodeDevice->popClickedQueue();
                nodeSetting->ledState ^= 1; // Toggle nodeSetting->ledState on _every_ click (not just the most recent)
              }
              if (nodeSetting->logClicked)
              {
                olaftoa(((float)clickedPopped) / 1000.0, tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }

              if (nodeSetting->toggleLEDOnClick)
              {
                if (nodeSetting->ledState)
                  nodeDevice->LEDon(nodeSetting->ledBrightness);
                else
                  nodeDevice->LEDoff();
                sprintf(tempData, "%d,", nodeSetting->ledState);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_BIO_SENSOR_HUB:
          {
            SparkFun_Bio_Sensor_Hub *nodeDevice = (SparkFun_Bio_Sensor_Hub *)temp->classPtr;
            struct_BIO_SENSOR_HUB *nodeSetting = (struct_BIO_SENSOR_HUB *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              bioData body;
              if ((nodeSetting->logHeartrate) || (nodeSetting->logConfidence) || (nodeSetting->logOxygen) || (nodeSetting->logStatus) || (nodeSetting->logExtendedStatus) || (nodeSetting->logRValue))
              {
                body = nodeDevice->readBpm();
              }
              if (nodeSetting->logHeartrate)
              {
                sprintf(tempData, "%d,", body.heartRate);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logConfidence)
              {
                sprintf(tempData, "%d,", body.confidence);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logOxygen)
              {
                sprintf(tempData, "%d,", body.oxygen);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logStatus)
              {
                sprintf(tempData, "%d,", body.status);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logExtendedStatus)
              {
                sprintf(tempData, "%d,", body.extStatus);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logRValue)
              {
                olaftoa(body.rValue, tempData1, 1, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_ISM330DHCX:
          {
            SparkFun_ISM330DHCX *nodeDevice = (SparkFun_ISM330DHCX *)temp->classPtr;
            struct_ISM330DHCX *nodeSetting = (struct_ISM330DHCX *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              // Structs for X,Y,Z data
              static sfe_ism_data_t accelData;
              static sfe_ism_data_t gyroData;
              static bool dataReady;
              if ((nodeSetting->logAccel) || (nodeSetting->logGyro))
              {
                // Check if both gyroscope and accelerometer data is available.
                dataReady = nodeDevice->checkStatus();
                if( dataReady )
                {
                  nodeDevice->getAccel(&accelData);
                  nodeDevice->getGyro(&gyroData);
                }
              }
              if (nodeSetting->logAccel)
              {
                olaftoa(accelData.xData, tempData1, 2, sizeof(tempData1) / sizeof(char));
                olaftoa(accelData.yData, tempData2, 2, sizeof(tempData2) / sizeof(char));
                olaftoa(accelData.zData, tempData3, 2, sizeof(tempData3) / sizeof(char));
                sprintf(tempData, "%s,%s,%s,", tempData1, tempData2, tempData3);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logGyro)
              {
                olaftoa(gyroData.xData, tempData1, 2, sizeof(tempData1) / sizeof(char));
                olaftoa(gyroData.yData, tempData2, 2, sizeof(tempData2) / sizeof(char));
                olaftoa(gyroData.zData, tempData3, 2, sizeof(tempData3) / sizeof(char));
                sprintf(tempData, "%s,%s,%s,", tempData1, tempData2, tempData3);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logDataReady)
              {
                sprintf(tempData, "%d,", dataReady);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_MMC5983MA:
          {
            SFE_MMC5983MA *nodeDevice = (SFE_MMC5983MA *)temp->classPtr;
            struct_MMC5983MA *nodeSetting = (struct_MMC5983MA *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              // X,Y,Z data
              uint32_t rawValueX;
              uint32_t rawValueY;
              uint32_t rawValueZ;
              double normalizedX;
              double normalizedY;
              double normalizedZ;
              if (nodeSetting->logMag)
              {

                nodeDevice->getMeasurementXYZ(&rawValueX, &rawValueY, &rawValueZ);

                // The magnetic field values are 18-bit unsigned. The zero (mid) point is 2^17 (131072).
                // Normalize each field to +/- 1.0
                normalizedX = (double)rawValueX - 131072.0;
                normalizedX /= 131072.0;
                normalizedY = (double)rawValueY - 131072.0;
                normalizedY /= 131072.0;
                normalizedZ = (double)rawValueZ - 131072.0;
                normalizedZ /= 131072.0;
                // Convert to Gauss
                normalizedX *= 8.0;
                normalizedY *= 8.0;
                normalizedZ *= 8.0;

                olaftoa(normalizedX, tempData1, 6, sizeof(tempData1) / sizeof(char));
                olaftoa(normalizedY, tempData2, 6, sizeof(tempData2) / sizeof(char));
                olaftoa(normalizedZ, tempData3, 6, sizeof(tempData3) / sizeof(char));
                sprintf(tempData, "%s,%s,%s,", tempData1, tempData2, tempData3);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logTemperature)
              {
                int temperature = nodeDevice->getTemperature();
                sprintf(tempData, "%d,", temperature);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_ADS1015:
          {
            ADS1015 *nodeDevice = (ADS1015 *)temp->classPtr;
            struct_ADS1015 *nodeSetting = (struct_ADS1015 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              if (nodeSetting->logA0)
              {
                float channel_mV = nodeDevice->getSingleEndedMillivolts(0);
                olaftoa(channel_mV, tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logA1)
              {
                float channel_mV = nodeDevice->getSingleEndedMillivolts(1);
                olaftoa(channel_mV, tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logA2)
              {
                float channel_mV = nodeDevice->getSingleEndedMillivolts(2);
                olaftoa(channel_mV, tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logA3)
              {
                float channel_mV = nodeDevice->getSingleEndedMillivolts(3);
                olaftoa(channel_mV, tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logA0A1)
              {
                float channel_mV = nodeDevice->getDifferentialMillivolts(ADS1015_CONFIG_MUX_DIFF_P0_N1);
                olaftoa(channel_mV, tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logA0A3)
              {
                float channel_mV = nodeDevice->getDifferentialMillivolts(ADS1015_CONFIG_MUX_DIFF_P0_N3);
                olaftoa(channel_mV, tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logA1A3)
              {
                float channel_mV = nodeDevice->getDifferentialMillivolts(ADS1015_CONFIG_MUX_DIFF_P1_N3);
                olaftoa(channel_mV, tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logA2A3)
              {
                float channel_mV = nodeDevice->getDifferentialMillivolts(ADS1015_CONFIG_MUX_DIFF_P2_N3);
                olaftoa(channel_mV, tempData1, 3, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_KX134:
          {
            SparkFun_KX134 *nodeDevice = (SparkFun_KX134 *)temp->classPtr;
            struct_KX134 *nodeSetting = (struct_KX134 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
              // X,Y,Z data
              static outputData xyzData;
              static bool dataReady = false;

              if ((nodeSetting->logAccel) || (nodeSetting->logDataReady))
              {
                // Check if accel data is available.
                dataReady = nodeDevice->dataReady();
                if (dataReady)
                  nodeDevice->getAccelData(&xyzData);
              }

              if (nodeSetting->logAccel)
              {
                olaftoa(xyzData.xData, tempData1, 4, sizeof(tempData1) / sizeof(char));
                olaftoa(xyzData.yData, tempData2, 4, sizeof(tempData2) / sizeof(char));
                olaftoa(xyzData.zData, tempData3, 4, sizeof(tempData3) / sizeof(char));
                sprintf(tempData, "%s,%s,%s,", tempData1, tempData2, tempData3);
                strlcat(sdOutputData, tempData, lenData);
              }
              if (nodeSetting->logDataReady)
              {
                sprintf(tempData, "%d,", dataReady);
                strlcat(sdOutputData, tempData, lenData);
              }
            }
          }
          break;
        case DEVICE_PCF8575:
          {
            PCF8575 *nodeDevice = (PCF8575 *)temp->classPtr;
            struct_PCF8575 *nodeSetting = (struct_PCF8575 *)temp->configPtr;
            if (nodeSetting->log == true)
            {
                olaftoa(nodeDevice->read(), tempData1, 2, sizeof(tempData1) / sizeof(char));
                sprintf(tempData, "%s,", tempData1);
                strlcat(sdOutputData, tempData, lenData);
            }
          }
          break;
        default:
          SerialPrintf2("printDeviceValue unknown device type: %s\r\n", getDeviceName(temp->deviceType));
          break;
      }

    }
    temp = temp->next;
  }
}

//Step through the node list and print helper text for the enabled readings


static void getHelperText(char* helperText, size_t lenText)
{

  helperText[0]='\0';

  if (settings.logRTC)
  {
    if (settings.logDate)
      strlcat(helperText, "rtcDate,", lenText);
    if (settings.logTime)
      strlcat(helperText, "rtcTime,", lenText);
    if (settings.logMicroseconds)
      strlcat(helperText, "micros,", lenText);
  }

  if (settings.logA11)
    strlcat(helperText, "analog_11,", lenText);

  if (settings.logA12)
    strlcat(helperText, "analog_12,", lenText);

  if (settings.logA13)
    strlcat(helperText, "analog_13,", lenText);

  if (settings.logA32)
    strlcat(helperText, "analog_32,", lenText);

  if (settings.logVIN)
    strlcat(helperText, "VIN,", lenText);

  if (online.IMU)
  {
    if (settings.imuUseDMP == false)
    {
      if (settings.logIMUAccel)
        strlcat(helperText, "aX,aY,aZ,", lenText);
      if (settings.logIMUGyro)
        strlcat(helperText, "gX,gY,gZ,", lenText);
      if (settings.logIMUMag)
        strlcat(helperText, "mX,mY,mZ,", lenText);
      if (settings.logIMUTemp)
        strlcat(helperText, "imu_degC,", lenText);
    }
    else
    {
      if (settings.imuLogDMPQuat6)
        strlcat(helperText, "Q6_1,Q6_2,Q6_3,", lenText);
      if (settings.imuLogDMPQuat9)
        strlcat(helperText, "Q9_1,Q9_2,Q9_3,HeadAcc,", lenText);
      if (settings.imuLogDMPAccel)
        strlcat(helperText, "RawAX,RawAY,RawAZ,", lenText);
      if (settings.imuLogDMPGyro)
        strlcat(helperText, "RawGX,RawGY,RawGZ,", lenText);
      if (settings.imuLogDMPCpass)
        strlcat(helperText, "RawMX,RawMY,RawMZ,", lenText);
    }
  }

  //Step through list, printing values as we go
  node *temp = head;
  while (temp != NULL)
  {

    //If this node successfully begin()'d
    if (temp->online == true)
    {
      //Switch on device type to set proper class and setting struct
      switch (temp->deviceType)
      {
        case DEVICE_MULTIPLEXER:
          {
            //No data to print for a mux
          }
          break;
        case DEVICE_LOADCELL_NAU7802:
          {
            struct_NAU7802 *nodeSetting = (struct_NAU7802 *)temp->configPtr;
            if (nodeSetting->log)
              strlcat(helperText, "weight(no unit),", lenText);
          }
          break;
        case DEVICE_DISTANCE_VL53L1X:
          {
            struct_VL53L1X *nodeSetting = (struct_VL53L1X *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logDistance)
                strlcat(helperText, "distance_mm,", lenText);
              if (nodeSetting->logRangeStatus)
                strlcat(helperText, "distance_rangeStatus(0=good),", lenText);
              if (nodeSetting->logSignalRate)
                strlcat(helperText, "distance_signalRate,", lenText);
            }
          }
          break;
        case DEVICE_GPS_UBLOX:
          {
            struct_ublox *nodeSetting = (struct_ublox *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logDate)
                strlcat(helperText, "gps_Date,", lenText);
              if (nodeSetting->logTime)
                strlcat(helperText, "gps_Time,", lenText);
              if (nodeSetting->logPosition)
                strlcat(helperText, "gps_Lat,gps_Long,", lenText);
              if (nodeSetting->logAltitude)
                strlcat(helperText, "gps_Alt,", lenText);
              if (nodeSetting->logAltitudeMSL)
                strlcat(helperText, "gps_AltMSL,", lenText);
              if (nodeSetting->logSIV)
                strlcat(helperText, "gps_SIV,", lenText);
              if (nodeSetting->logFixType)
                strlcat(helperText, "gps_FixType,", lenText);
              if (nodeSetting->logCarrierSolution)
                strlcat(helperText, "gps_CarrierSolution,", lenText);
              if (nodeSetting->logGroundSpeed)
                strlcat(helperText, "gps_GroundSpeed,", lenText);
              if (nodeSetting->logHeadingOfMotion)
                strlcat(helperText, "gps_Heading,", lenText);
              if (nodeSetting->logpDOP)
                strlcat(helperText, "gps_pDOP,", lenText);
              if (nodeSetting->logiTOW)
                strlcat(helperText, "gps_iTOW,", lenText);
            }
          }
          break;
        case DEVICE_PROXIMITY_VCNL4040:
          {
            struct_VCNL4040 *nodeSetting = (struct_VCNL4040 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logProximity)
                strlcat(helperText, "prox(no unit),", lenText);
              if (nodeSetting->logAmbientLight)
                strlcat(helperText, "ambient_lux,", lenText);
            }
          }
          break;
        case DEVICE_TEMPERATURE_TMP117:
          {
            struct_TMP117 *nodeSetting = (struct_TMP117 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logTemperature)
                strlcat(helperText, "degC,", lenText);
            }
          }
          break;
        case DEVICE_PRESSURE_MS5637:
          {
            struct_MS5637 *nodeSetting = (struct_MS5637 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logPressure)
                strlcat(helperText, "pressure_hPa,", lenText);
              if (nodeSetting->logTemperature)
                strlcat(helperText, "temperature_degC,", lenText);
            }
          }
          break;
        case DEVICE_PRESSURE_LPS25HB:
          {
            struct_LPS25HB *nodeSetting = (struct_LPS25HB *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logPressure)
                strlcat(helperText, "pressure_hPa,", lenText);
              if (nodeSetting->logTemperature)
                strlcat(helperText, "temperature_degC,", lenText);
            }
          }
          break;
        case DEVICE_PRESSURE_LPS28DFW:
          {
            struct_LPS28DFW *nodeSetting = (struct_LPS28DFW *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logPressure)
                strlcat(helperText, "pressure_hPa,", lenText);
              if (nodeSetting->logTemperature)
                strlcat(helperText, "temperature_degC,", lenText);
            }
          }
          break;
        case DEVICE_PHT_BME280:
          {
            struct_BME280 *nodeSetting = (struct_BME280 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logPressure)
                strlcat(helperText, "pressure_Pa,", lenText);
              if (nodeSetting->logHumidity)
                strlcat(helperText, "humidity_%,", lenText);
              if (nodeSetting->logAltitude)
                strlcat(helperText, "altitude_m,", lenText);
              if (nodeSetting->logTemperature)
                strlcat(helperText, "temp_degC,", lenText);
            }
          }
          break;
        case DEVICE_UV_VEML6075:
          {
            struct_VEML6075 *nodeSetting = (struct_VEML6075 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logUVA)
                strlcat(helperText, "uva,", lenText);
              if (nodeSetting->logUVB)
                strlcat(helperText, "uvb,", lenText);
              if (nodeSetting->logUVIndex)
                strlcat(helperText, "uvIndex,", lenText);
            }
          }
          break;
        case DEVICE_LIGHT_VEML7700:
          {
            struct_VEML7700 *nodeSetting = (struct_VEML7700 *)temp->configPtr;
            if (nodeSetting->log)
            {
              strlcat(helperText, "lux,", lenText);
            }
          }
          break;
        case DEVICE_VOC_CCS811:
          {
            struct_CCS811 *nodeSetting = (struct_CCS811 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logTVOC)
                strlcat(helperText, "tvoc_ppb,", lenText);
              if (nodeSetting->logCO2)
                strlcat(helperText, "co2_ppm,", lenText);
            }
          }
          break;
        case DEVICE_VOC_SGP30:
          {
            struct_SGP30 *nodeSetting = (struct_SGP30 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logTVOC)
                strlcat(helperText, "tvoc_ppb,", lenText);
              if (nodeSetting->logCO2)
                strlcat(helperText, "co2_ppm,", lenText);
              if (nodeSetting->logH2)
                strlcat(helperText, "H2,", lenText);
              if (nodeSetting->logEthanol)
                strlcat(helperText, "ethanol,", lenText);
            }
          }
          break;
        case DEVICE_CO2_SCD30:
          {
            struct_SCD30 *nodeSetting = (struct_SCD30 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logCO2)
                strlcat(helperText, "co2_ppm,", lenText);
              if (nodeSetting->logHumidity)
                strlcat(helperText, "humidity_%,", lenText);
              if (nodeSetting->logTemperature)
                strlcat(helperText, "degC,", lenText);
            }
          }
          break;
        case DEVICE_PHT_MS8607:
          {
            struct_MS8607 *nodeSetting = (struct_MS8607 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logHumidity)
                strlcat(helperText, "humidity_%,", lenText);
              if (nodeSetting->logPressure)
                strlcat(helperText, "hPa,", lenText);
              if (nodeSetting->logTemperature)
                strlcat(helperText, "degC,", lenText);
            }
          }
          break;
        case DEVICE_TEMPERATURE_MCP9600:
          {
            struct_MCP9600 *nodeSetting = (struct_MCP9600 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logTemperature)
                strlcat(helperText, "thermo_degC,", lenText);
              if (nodeSetting->logAmbientTemperature)
                strlcat(helperText, "thermo_ambientDegC,", lenText);
            }
          }
          break;
        case DEVICE_HUMIDITY_AHT20:
          {
            struct_AHT20 *nodeSetting = (struct_AHT20 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logHumidity)
                strlcat(helperText, "humidity_%,", lenText);
              if (nodeSetting->logTemperature)
                strlcat(helperText, "degC,", lenText);
            }
          }
          break;
        case DEVICE_HUMIDITY_SHTC3:
          {
            struct_SHTC3 *nodeSetting = (struct_SHTC3 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logHumidity)
                strlcat(helperText, "humidity_%,", lenText);
              if (nodeSetting->logTemperature)
                strlcat(helperText, "degC,", lenText);
            }
          }
          break;
        case DEVICE_ADC_ADS122C04:
          {
            struct_ADS122C04 *nodeSetting = (struct_ADS122C04 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logCentigrade)
                strlcat(helperText, "degC,", lenText);
              if (nodeSetting->logFahrenheit)
                strlcat(helperText, "degF,", lenText);
              if (nodeSetting->logInternalTemperature)
                strlcat(helperText, "degC,", lenText);
              if (nodeSetting->logRawVoltage)
                strlcat(helperText, "V*2.048/2^23,", lenText);
            }
          }
          break;
        case DEVICE_PRESSURE_MPR0025PA1:
          {
            struct_MPR0025PA1 *nodeSetting = (struct_MPR0025PA1 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->usePSI)
                strlcat(helperText, "PSI,", lenText);
              if (nodeSetting->usePA)
                strlcat(helperText, "Pa,", lenText);
              if (nodeSetting->useKPA)
                strlcat(helperText, "kPa,", lenText);
              if (nodeSetting->useTORR)
                strlcat(helperText, "torr,", lenText);
              if (nodeSetting->useINHG)
                strlcat(helperText, "inHg,", lenText);
              if (nodeSetting->useATM)
                strlcat(helperText, "atm,", lenText);
              if (nodeSetting->useBAR)
                strlcat(helperText, "bar,", lenText);
            }
          }
          break;
        case DEVICE_PARTICLE_SNGCJA5:
          {
            struct_SNGCJA5 *nodeSetting = (struct_SNGCJA5 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logPM1)
                strlcat(helperText, "PM1_0,", lenText);
              if (nodeSetting->logPM25)
                strlcat(helperText, "PM2_5,", lenText);
              if (nodeSetting->logPM10)
                strlcat(helperText, "PM10,", lenText);
              if (nodeSetting->logPC05)
                strlcat(helperText, "PC0_5,", lenText);
              if (nodeSetting->logPC1)
                strlcat(helperText, "PC1_0,", lenText);
              if (nodeSetting->logPC25)
                strlcat(helperText, "PC2_5,", lenText);
              if (nodeSetting->logPC50)
                strlcat(helperText, "PC5_0,", lenText);
              if (nodeSetting->logPC75)
                strlcat(helperText, "PC7_5,", lenText);
              if (nodeSetting->logPC10)
                strlcat(helperText, "PC10,", lenText);
              if (nodeSetting->logSensorStatus)
                strlcat(helperText, "Sensors,", lenText);
              if (nodeSetting->logPDStatus)
                strlcat(helperText, "PD,", lenText);
              if (nodeSetting->logLDStatus)
                strlcat(helperText, "LD,", lenText);
              if (nodeSetting->logFanStatus)
                strlcat(helperText, "Fan,", lenText);
            }
          }
          break;
        case DEVICE_VOC_SGP40:
          {
            struct_SGP40 *nodeSetting = (struct_SGP40 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logVOC)
                strlcat(helperText, "VOCindex,", lenText);
            }
          }
          break;
        case DEVICE_PRESSURE_SDP3X:
          {
            struct_SDP3X *nodeSetting = (struct_SDP3X *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logPressure)
                strlcat(helperText, "Pa,", lenText);
              if (nodeSetting->logTemperature)
                strlcat(helperText, "degC,", lenText);
            }
          }
          break;
        case DEVICE_PRESSURE_MS5837:
          {
            struct_MS5837 *nodeSetting = (struct_MS5837 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logPressure)
                strlcat(helperText, "mbar,", lenText);
              if (nodeSetting->logTemperature)
                strlcat(helperText, "degC,", lenText);
              if (nodeSetting->logDepth)
                strlcat(helperText, "depth_m,", lenText);
              if (nodeSetting->logAltitude)
                strlcat(helperText, "alt_m,", lenText);
            }
          }
          break;
        case DEVICE_QWIIC_BUTTON:
          {
            struct_QWIIC_BUTTON *nodeSetting = (struct_QWIIC_BUTTON *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logPressed)
                strlcat(helperText, "pressS,", lenText);
              if (nodeSetting->logClicked)
                strlcat(helperText, "clickS,", lenText);
              if (nodeSetting->toggleLEDOnClick)
                strlcat(helperText, "LED,", lenText);
            }
          }
          break;
        case DEVICE_BIO_SENSOR_HUB:
          {
            struct_BIO_SENSOR_HUB *nodeSetting = (struct_BIO_SENSOR_HUB *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logHeartrate)
                strlcat(helperText, "bpm,", lenText);
              if (nodeSetting->logConfidence)
                strlcat(helperText, "conf%,", lenText);
              if (nodeSetting->logOxygen)
                strlcat(helperText, "O2%,", lenText);
              if (nodeSetting->logStatus)
                strlcat(helperText, "stat,", lenText);
              if (nodeSetting->logExtendedStatus)
                strlcat(helperText, "eStat,", lenText);
              if (nodeSetting->logRValue)
                strlcat(helperText, "O2R,", lenText);
            }
          }
          break;
        case DEVICE_ISM330DHCX:
          {
            struct_ISM330DHCX *nodeSetting = (struct_ISM330DHCX *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logAccel)
                strlcat(helperText, "aX,aY,aZ,", lenText);
              if (nodeSetting->logGyro)
                strlcat(helperText, "gX,gY,gZ,", lenText);
              if (nodeSetting->logDataReady)
                strlcat(helperText, "dataRdy,", lenText);
            }
          }
          break;
        case DEVICE_MMC5983MA:
          {
            struct_MMC5983MA *nodeSetting = (struct_MMC5983MA *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logMag)
                strlcat(helperText, "mX,mY,mZ,", lenText);
              if (nodeSetting->logTemperature)
                strlcat(helperText, "degC,", lenText);
            }
          }
          break;
        case DEVICE_KX134:
          {
            struct_KX134 *nodeSetting = (struct_KX134 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logAccel)
                strlcat(helperText, "aX,aY,aZ,", lenText);
              if (nodeSetting->logDataReady)
                strlcat(helperText, "dataRdy,", lenText);
            }
          }
          break;
        case DEVICE_ADS1015:
          {
            struct_ADS1015 *nodeSetting = (struct_ADS1015 *)temp->configPtr;
            if (nodeSetting->log)
            {
              if (nodeSetting->logA0)
                strlcat(helperText, "A0mV,", lenText);
              if (nodeSetting->logA1)
                strlcat(helperText, "A1mV,", lenText);
              if (nodeSetting->logA2)
                strlcat(helperText, "A2mV,", lenText);
              if (nodeSetting->logA3)
                strlcat(helperText, "A3mV,", lenText);
              if (nodeSetting->logA0A1)
                strlcat(helperText, "A0A1mV,", lenText);
              if (nodeSetting->logA0A3)
                strlcat(helperText, "A0A3mV,", lenText);
              if (nodeSetting->logA1A3)
                strlcat(helperText, "A1A3mV,", lenText);
              if (nodeSetting->logA2A3)
                strlcat(helperText, "A2A3mV,", lenText);
            }
          }
          break;
        default:
          SerialPrintf2("\nprinterHelperText device not found: %d\r\n", temp->deviceType);
          break;
      }
    }
    temp = temp->next;
  }

  if (settings.logHertz)
    strlcat(helperText, "output_Hz,", lenText);

  if (settings.printMeasurementCount)
    strlcat(helperText, "count,", lenText);

  strlcat(helperText, "\r\n", lenText);
}


void printHelperText(uint8_t outputDest)
{
  char helperText[HELPER_BUFFER_SIZE];
  helperText[0] = '\0';

  getHelperText(helperText, sizeof(helperText));

  if(outputDest & OL_OUTPUT_SERIAL)
    SerialPrint(helperText);

  if ((outputDest & OL_OUTPUT_SDCARD) && (settings.logData == true) && (online.microSD))
    sensorDataFile.print(helperText);
}

//If certain devices are attached, we need to reduce the I2C max speed
void setMaxI2CSpeed()
{
  uint32_t maxSpeed = 400000; //Assume 400kHz - but beware! 400kHz with no pull-ups can cause u-blox issues.

  //Search nodes for MCP9600s and Ublox modules
  node *temp = head;
  while (temp != NULL)
  {
    if (temp->deviceType == DEVICE_TEMPERATURE_MCP9600)
    {
      //TODO Are we sure the MCP9600, begin'd() on the bus, but not logged will behave when bus is 400kHz?
      //Check if logging is enabled
      struct_MCP9600 *sensor = (struct_MCP9600*)temp->configPtr;
      if (sensor->log == true)
        maxSpeed = 100000;
    }

    if (temp->deviceType == DEVICE_GPS_UBLOX)
    {
      //Check if i2cSpeed is lowered
      struct_ublox *sensor = (struct_ublox*)temp->configPtr;
      if (sensor->i2cSpeed == 100000)
      {
        //printDebug("setMaxI2CSpeed: sensor->i2cSpeed is 100000. Reducing maxSpeed to 100kHz\r\n");
        maxSpeed = 100000;
      }
    }

    temp = temp->next;
  }

  //If user wants to limit the I2C bus speed, do it here
  if (maxSpeed > settings.qwiicBusMaxSpeed)
  {
    //printDebug("setMaxI2CSpeed: maxSpeed is > settings.qwiicBusMaxSpeed. Reducing maxSpeed to " + (String)settings.qwiicBusMaxSpeed + "Hz\r\n");
    maxSpeed = settings.qwiicBusMaxSpeed;
  }

  if (maxSpeed > 200000)
  {
    printDebug("setMaxI2CSpeed: setting qwiic clock speed to " + (String)AM_HAL_IOM_400KHZ + "Hz\r\n");
    qwiic.setClock(AM_HAL_IOM_400KHZ);
  }
  else
  {
    printDebug("setMaxI2CSpeed: setting qwiic clock speed to " + (String)AM_HAL_IOM_100KHZ + "Hz\r\n");
    qwiic.setClock(AM_HAL_IOM_100KHZ);
  }
  for (int i = 0; i < 100; i++) //Allow time for the speed to change
  {
    checkBattery();
    delay(1);
  }
}

//Read the VIN voltage
float readVIN()
{
  // Only supported on >= V10 hardware
#if(HARDWARE_VERSION_MAJOR == 0)
  return(0.0); // Return 0.0V on old hardware
#else
  int div3 = analogRead(PIN_VIN_MONITOR); //Read VIN across a 1/3 resistor divider
  float vin = (float)div3 * 3.0 * 2.0 / 16384.0; //Convert 1/3 VIN to VIN (14-bit resolution)
  vin = vin * settings.vinCorrectionFactor; //Correct for divider impedance (determined experimentally)
  return (vin);
#endif
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\autoDetect.ino"
/*
  Autodetect theory of operation:

  The TCA9548A I2C mux introduces a can of worms but enables multiple (up to 64) of
  a single I2C device to be connected to a given I2C bus. You just have to turn on/off
  a given port while you communicate with said device.

  This is how the autodection algorithm works:
   Scan bus for muxes (0x70 to 0x77)
   Begin() any muxes. This causes them to turn off all their ports.
   With any possible muxes turned off, finish scanning main branch
   Any detected device is stored as a node in a linked list containing their address and device type,
   If muxes are found, begin scanning mux0/port0. Any new device is stored with their address and mux address/port.
   Begin() all devices in our linked list. Connections through muxes are performed as needed.

  All of this works and has the side benefit of enabling regular devices, that support multiple address, to
  auto-detect, begin(), and behave as before, but now in multiples.

  In the case where a device has two I2C address that are used in one library (ex: MS8607) the first address is stored in
  the node list, the 2nd address is ignored.

  Future work:

  Theoretically you could attach 8 muxes configured to 0x71 off the 8 ports of an 0x70 mux. We could
  do this for other muxes as well to create a mux monster:
   - 0x70 - (port 0) 0x71 - 8 ports - device * 8
                     0x72 - 8 ports - device * 8
                     0x73 - 8 ports - device * 8
                     0x74 - 8 ports - device * 8
                     0x75 - 8 ports - device * 8
                     0x76 - 8 ports - device * 8
                     0x77 - 8 ports - device * 8
  This would allow a maximum of 8 * 7 * 8 = 448 of the *same* I2C device address to be
  connected at once. We don't support this sub-muxing right now. So the max we support
  is 64 identical address devices. That should be enough.
*/

//Given node number, get a pointer to the node
node *getNodePointer(uint8_t nodeNumber)
{
  //Search the list of nodes
  node *temp = head;

  int counter = 0;
  while (temp != NULL)
  {
    if (nodeNumber == counter)
      return (temp);
    counter++;
    temp = temp->next;
  }

  return (NULL);
}

node *getNodePointer(deviceType_e deviceType, uint8_t address, uint8_t muxAddress, uint8_t portNumber)
{
  //Search the list of nodes
  node *temp = head;
  while (temp != NULL)
  {
    if (temp->address == address)
      if (temp->muxAddress == muxAddress)
        if (temp->portNumber == portNumber)
          if (temp->deviceType == deviceType)
            return (temp);

    temp = temp->next;
  }
  return (NULL);
}

//Given nodenumber, pull out the device type
deviceType_e getDeviceType(uint8_t nodeNumber)
{
  node *temp = getNodePointer(nodeNumber);
  if (temp == NULL) return (DEVICE_UNKNOWN_DEVICE);
  return (temp->deviceType);
}

//Given nodeNumber, return the config pointer
void *getConfigPointer(uint8_t nodeNumber)
{
  //Search the list of nodes
  node *temp = getNodePointer(nodeNumber);
  if (temp == NULL) return (NULL);
  return (temp->configPtr);
}

//Given a bunch of ID'ing info, return the config pointer to a node
void *getConfigPointer(deviceType_e deviceType, uint8_t address, uint8_t muxAddress, uint8_t portNumber)
{
  //Search the list of nodes
  node *temp = getNodePointer(deviceType, address, muxAddress, portNumber);
  if (temp == NULL) return (NULL);
  return (temp->configPtr);
}

//Add a device to the linked list
//Creates a class but does not begin or configure the device
bool addDevice(deviceType_e deviceType, uint8_t address, uint8_t muxAddress, uint8_t portNumber)
{
  //Ignore devices we've already logged. This was causing the mux to get tested, a begin() would happen, and the mux would be reset.
  if (deviceExists(deviceType, address, muxAddress, portNumber) == true) return false;

  //Create class instantiation for this device
  //Create logging details for this device

  node *temp = new node; //Create the node in memory

  //Setup this node
  temp->deviceType = deviceType;
  temp->address = address;
  temp->muxAddress = muxAddress;
  temp->portNumber = portNumber;

  //Instantiate a class and settings struct for this device
  switch (deviceType)
  {
    case DEVICE_MULTIPLEXER:
      {
        temp->classPtr = new QWIICMUX; //This allocates the memory needed for this class
        temp->configPtr = new struct_multiplexer;
      }
      break;
    case DEVICE_LOADCELL_NAU7802:
      {
        temp->classPtr = new NAU7802;
        temp->configPtr = new struct_NAU7802;
      }
      break;
    case DEVICE_DISTANCE_VL53L1X:
      {
        temp->classPtr = new SFEVL53L1X(qwiic);
        temp->configPtr = new struct_VL53L1X;
      }
      break;
    case DEVICE_GPS_UBLOX:
      {
        temp->classPtr = new SFE_UBLOX_GNSS;
        temp->configPtr = new struct_ublox;
      }
      break;
    case DEVICE_PROXIMITY_VCNL4040:
      {
        temp->classPtr = new VCNL4040;
        temp->configPtr = new struct_VCNL4040;
      }
      break;
    case DEVICE_TEMPERATURE_TMP117:
      {
        temp->classPtr = new TMP117;
        temp->configPtr = new struct_TMP117;
      }
      break;
    case DEVICE_PRESSURE_LPS25HB:
      {
        temp->classPtr = new LPS25HB;
        temp->configPtr = new struct_LPS25HB;
      }
      break;
    case DEVICE_PRESSURE_LPS28DFW:
      {
        temp->classPtr = new LPS28DFW;
        temp->configPtr = new struct_LPS28DFW;
      }
      break;
    case DEVICE_PRESSURE_MS5637:
      {
        temp->classPtr = new MS5637;
        temp->configPtr = new struct_MS5637;
      }
      break;
    case DEVICE_PHT_BME280:
      {
        temp->classPtr = new BME280;
        temp->configPtr = new struct_BME280;
      }
      break;
    case DEVICE_UV_VEML6075:
      {
        temp->classPtr = new VEML6075;
        temp->configPtr = new struct_VEML6075;
      }
      break;
    case DEVICE_LIGHT_VEML7700:
      {
        temp->classPtr = new VEML7700;
        temp->configPtr = new struct_VEML7700;
      }
      break;
    case DEVICE_VOC_CCS811:
      {
        temp->classPtr = new CCS811(address);
        temp->configPtr = new struct_CCS811;
      }
      break;
    case DEVICE_VOC_SGP30:
      {
        temp->classPtr = new SGP30;
        temp->configPtr = new struct_SGP30;
      }
      break;
    case DEVICE_CO2_SCD30:
      {
        temp->classPtr = new SCD30;
        temp->configPtr = new struct_SCD30;
      }
      break;
    case DEVICE_PHT_MS8607:
      {
        temp->classPtr = new MS8607;
        temp->configPtr = new struct_MS8607;
      }
      break;
    case DEVICE_TEMPERATURE_MCP9600:
      {
        temp->classPtr = new MCP9600;
        temp->configPtr = new struct_MCP9600;
      }
      break;
    case DEVICE_HUMIDITY_AHT20:
      {
        temp->classPtr = new AHT20;
        temp->configPtr = new struct_AHT20;
      }
      break;
    case DEVICE_HUMIDITY_SHTC3:
      {
        temp->classPtr = new SHTC3;
        temp->configPtr = new struct_SHTC3;
      }
      break;
    case DEVICE_ADC_ADS122C04:
      {
        temp->classPtr = new SFE_ADS122C04;
        temp->configPtr = new struct_ADS122C04;
      }
      break;
    case DEVICE_PRESSURE_MPR0025PA1:
      {
        temp->classPtr = new SparkFun_MicroPressure;
        temp->configPtr = new struct_MPR0025PA1;
      }
      break;
    case DEVICE_PARTICLE_SNGCJA5:
      {
        temp->classPtr = new SFE_PARTICLE_SENSOR;
        temp->configPtr = new struct_SNGCJA5;
      }
      break;
    case DEVICE_VOC_SGP40:
      {
        temp->classPtr = new SGP40;
        temp->configPtr = new struct_SGP40;
      }
      break;
    case DEVICE_PRESSURE_SDP3X:
      {
        temp->classPtr = new SDP3X;
        temp->configPtr = new struct_SDP3X;
      }
      break;
    case DEVICE_PRESSURE_MS5837:
      {
        temp->classPtr = new MS5837;
        temp->configPtr = new struct_MS5837;
      }
      break;
    case DEVICE_QWIIC_BUTTON:
      {
        temp->classPtr = new QwiicButton;
        temp->configPtr = new struct_QWIIC_BUTTON;
      }
      break;
    case DEVICE_BIO_SENSOR_HUB:
      {
        temp->classPtr = new SparkFun_Bio_Sensor_Hub(32, 11, address); // Reset pin is 32, MFIO pin is 11
        temp->configPtr = new struct_BIO_SENSOR_HUB;
      }
      break;
    case DEVICE_ISM330DHCX:
      {
        temp->classPtr = new SparkFun_ISM330DHCX;
        temp->configPtr = new struct_ISM330DHCX;
      }
      break;
    case DEVICE_MMC5983MA:
      {
        temp->classPtr = new SFE_MMC5983MA;
        temp->configPtr = new struct_MMC5983MA;
      }
      break;
    case DEVICE_KX134:
      {
        temp->classPtr = new SparkFun_KX134;
        temp->configPtr = new struct_KX134;
      }
      break;
    case DEVICE_ADS1015:
      {
        temp->classPtr = new ADS1015;
        temp->configPtr = new struct_ADS1015;
      }
      break;
    case DEVICE_PCF8575:
      {
        temp->classPtr = new PCF8575;
        temp->configPtr = new struct_PCF8575;
      }
      break;
    default:
      SerialPrintf2("addDevice Device type not found: %d\r\n", deviceType);
      break;
  }

  //Link to next node
  temp->next = NULL;
  if (head == NULL)
  {
    head = temp;
    tail = temp;
    temp = NULL;
  }
  else
  {
    tail->next = temp;
    tail = temp;
  }

  return true;
}

//Begin()'s all devices in the node list
bool beginQwiicDevices()
{
  bool everythingStarted = true;

  waitForQwiicBusPowerDelay(); // Wait while the qwiic devices power up - if required

  qwiicPowerOnDelayMillis = settings.qwiicBusPowerUpDelayMs; // Set qwiicPowerOnDelayMillis to the _minimum_ defined by settings.qwiicBusPowerUpDelayMs. It will be increased if required.

  int numberOfSCD30s = 0; // Keep track of how many SCD30s we begin so we can delay before starting the second and subsequent ones

  //Step through the list
  node *temp = head;

  if (temp == NULL)
  {
    printDebug(F("beginQwiicDevices: No devices detected\r\n"));
    return (true);
  }

  while (temp != NULL)
  {
    openConnection(temp->muxAddress, temp->portNumber); //Connect to this device through muxes as needed

    if (settings.printDebugMessages == true)
    {
      SerialPrintf2("beginQwiicDevices: attempting to begin deviceType %s", getDeviceName(temp->deviceType));
      SerialPrintf4(" at address 0x%02X using mux address 0x%02X and port number %d\r\n", temp->address, temp->muxAddress, temp->portNumber);
    }

    //Attempt to begin the device
    switch (temp->deviceType)
    {
      case DEVICE_MULTIPLEXER:
        {
          QWIICMUX *tempDevice = (QWIICMUX *)temp->classPtr;
          struct_multiplexer *nodeSetting = (struct_multiplexer *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(temp->address, qwiic); //Address, Wire port
        }
        break;
      case DEVICE_LOADCELL_NAU7802:
        {
          NAU7802 *tempDevice = (NAU7802 *)temp->classPtr;
          struct_NAU7802 *nodeSetting = (struct_NAU7802 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(qwiic, false); //Wire port. Skip the initialization. Let configureDevice do it
        }
        break;
      case DEVICE_DISTANCE_VL53L1X:
        {
          SFEVL53L1X *tempDevice = (SFEVL53L1X *)temp->classPtr;
          struct_VL53L1X *nodeSetting = (struct_VL53L1X *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin() == 0) //Returns 0 if init was successful. Wire port passed in constructor.
            temp->online = true;
        }
        break;
      case DEVICE_GPS_UBLOX:
        {
          setQwiicPullups(0); //Disable pullups for u-blox comms.
          SFE_UBLOX_GNSS *tempDevice = (SFE_UBLOX_GNSS *)temp->classPtr;
          struct_ublox *nodeSetting = (struct_ublox *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if(settings.printGNSSDebugMessages == true) tempDevice->enableDebugging(); // Enable debug messages if required
          temp->online = tempDevice->begin(qwiic, temp->address); //Wire port, Address
          setQwiicPullups(settings.qwiicBusPullUps); //Re-enable pullups.
        }
        break;
      case DEVICE_PROXIMITY_VCNL4040:
        {
          VCNL4040 *tempDevice = (VCNL4040 *)temp->classPtr;
          struct_VCNL4040 *nodeSetting = (struct_VCNL4040 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(qwiic); //Wire port
        }
        break;
      case DEVICE_TEMPERATURE_TMP117:
        {
          TMP117 *tempDevice = (TMP117 *)temp->classPtr;
          struct_TMP117 *nodeSetting = (struct_TMP117 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(temp->address, qwiic); //Address, Wire port
        }
        break;
      case DEVICE_PRESSURE_LPS25HB:
        {
          LPS25HB *tempDevice = (LPS25HB *)temp->classPtr;
          struct_LPS25HB *nodeSetting = (struct_LPS25HB *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(qwiic, temp->address); //Wire port, Address
        }
        break;
      case DEVICE_PRESSURE_LPS28DFW:
        {
          LPS28DFW *tempDevice = (LPS28DFW *)temp->classPtr;
          struct_LPS28DFW *nodeSetting = (struct_LPS28DFW *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(temp->address, qwiic) == LPS28DFW_OK;
          lps28dfw_md_t modeConfig =
          {
              .fs  = LPS28DFW_1260hPa,    // Full scale range
              .odr = LPS28DFW_ONE_SHOT,        // Output data rate
              .avg = LPS28DFW_4_AVG,      // Average filter
              .lpf = LPS28DFW_LPF_DISABLE // Low-pass filter
          };
          tempDevice->setModeConfig(&modeConfig);
        }
        break;
      case DEVICE_PRESSURE_MS5637:
        {
          MS5637 *tempDevice = (MS5637 *)temp->classPtr;
          struct_MS5637 *nodeSetting = (struct_MS5637 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(qwiic); //Wire port
        }
        break;
      case DEVICE_PHT_BME280:
        {
          BME280 *tempDevice = (BME280 *)temp->classPtr;
          struct_BME280 *nodeSetting = (struct_BME280 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          tempDevice->setI2CAddress(temp->address);
          temp->online = tempDevice->beginI2C(qwiic); //Wire port
        }
        break;
      case DEVICE_UV_VEML6075:
        {
          VEML6075 *tempDevice = (VEML6075 *)temp->classPtr;
          struct_VEML6075 *nodeSetting = (struct_VEML6075 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(qwiic); //Wire port
        }
        break;
      case DEVICE_LIGHT_VEML7700:
        {
          VEML7700 *tempDevice = (VEML7700 *)temp->classPtr;
          struct_VEML7700 *nodeSetting = (struct_VEML7700 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(qwiic); //Wire port
        }
        break;
      case DEVICE_VOC_CCS811:
        {
          CCS811 *tempDevice = (CCS811 *)temp->classPtr;
          struct_CCS811 *nodeSetting = (struct_CCS811 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(qwiic); //Wire port
        }
        break;
      case DEVICE_VOC_SGP30:
        {
          SGP30 *tempDevice = (SGP30 *)temp->classPtr;
          struct_SGP30 *nodeSetting = (struct_SGP30 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(qwiic); //Wire port
        }
        break;
      case DEVICE_CO2_SCD30:
        {
          SCD30 *tempDevice = (SCD30 *)temp->classPtr;
          struct_SCD30 *nodeSetting = (struct_SCD30 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          numberOfSCD30s++; // Keep track of how many SCD30s we begin
          // Delay before starting the second and subsequent SCD30s to try and stagger the measurements and the peak current draw
          if (numberOfSCD30s >= 2)
          {
            printDebug(F("beginQwiicDevices: found more than one SCD30. Delaying for 375ms to stagger the peak current draw...\r\n"));
            delay(375);
          }
          if(settings.printDebugMessages == true) tempDevice->enableDebugging(); // Enable debug messages if required
          temp->online = tempDevice->begin(qwiic); //Wire port
        }
        break;
      case DEVICE_PHT_MS8607:
        {
          MS8607 *tempDevice = (MS8607 *)temp->classPtr;
          struct_MS8607 *nodeSetting = (struct_MS8607 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(qwiic); //Wire port
        }
        break;
      case DEVICE_TEMPERATURE_MCP9600:
        {
          MCP9600 *tempDevice = (MCP9600 *)temp->classPtr;
          struct_MCP9600 *nodeSetting = (struct_MCP9600 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(temp->address, qwiic); //Address, Wire port
        }
        break;
      case DEVICE_HUMIDITY_AHT20:
        {
          AHT20 *tempDevice = (AHT20 *)temp->classPtr;
          struct_AHT20 *nodeSetting = (struct_AHT20 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          temp->online = tempDevice->begin(qwiic); //Wire port
        }
        break;
      case DEVICE_HUMIDITY_SHTC3:
        {
          SHTC3 *tempDevice = (SHTC3 *)temp->classPtr;
          struct_SHTC3 *nodeSetting = (struct_SHTC3 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin(qwiic) == 0) //Wire port. Returns 0 on success.
            temp->online = true;
        }
        break;
      case DEVICE_ADC_ADS122C04:
        {
          SFE_ADS122C04 *tempDevice = (SFE_ADS122C04 *)temp->classPtr;
          struct_ADS122C04 *nodeSetting = (struct_ADS122C04 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin(temp->address, qwiic) == true) //Address, Wire port. Returns true on success.
            temp->online = true;
        }
        break;
      case DEVICE_PRESSURE_MPR0025PA1:
        {
          // TO DO: Figure out how to pass minimumPSI and maximumPSI when instantiating the sensor. Maybe add an update-_minPsi-and-_maxPsi function to the library?
          SparkFun_MicroPressure *tempDevice = (SparkFun_MicroPressure *)temp->classPtr;
          struct_MPR0025PA1 *nodeSetting = (struct_MPR0025PA1 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin(temp->address, qwiic) == true) //Address, Wire port. Returns true on success.
            temp->online = true;
        }
        break;
      case DEVICE_PARTICLE_SNGCJA5:
        {
          SFE_PARTICLE_SENSOR *tempDevice = (SFE_PARTICLE_SENSOR *)temp->classPtr;
          struct_SNGCJA5 *nodeSetting = (struct_SNGCJA5 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin(qwiic) == true) //Wire port. Returns true on success.
            temp->online = true;
        }
        break;
      case DEVICE_VOC_SGP40:
        {
          SGP40 *tempDevice = (SGP40 *)temp->classPtr;
          struct_SGP40 *nodeSetting = (struct_SGP40 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin(qwiic) == true) //Wire port. Returns true on success.
            temp->online = true;
        }
        break;
      case DEVICE_PRESSURE_SDP3X:
        {
          SDP3X *tempDevice = (SDP3X *)temp->classPtr;
          struct_SDP3X *nodeSetting = (struct_SDP3X *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          tempDevice->stopContinuousMeasurement(temp->address, qwiic); //Make sure continuous measurements are stopped or .begin will fail
          if (tempDevice->begin(temp->address, qwiic) == true) //Address, Wire port. Returns true on success.
            temp->online = true;
        }
        break;
      case DEVICE_PRESSURE_MS5837:
        {
          MS5837 *tempDevice = (MS5837 *)temp->classPtr;
          struct_MS5837 *nodeSetting = (struct_MS5837 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin(qwiic) == true) //Wire port. Returns true on success.
            temp->online = true;
        }
        break;
      case DEVICE_QWIIC_BUTTON:
        {
          QwiicButton *tempDevice = (QwiicButton *)temp->classPtr;
          struct_QWIIC_BUTTON *nodeSetting = (struct_QWIIC_BUTTON *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin(temp->address, qwiic) == true) //Address, Wire port. Returns true on success.
            temp->online = true;
        }
        break;
      case DEVICE_BIO_SENSOR_HUB:
        {
          SparkFun_Bio_Sensor_Hub *tempDevice = (SparkFun_Bio_Sensor_Hub *)temp->classPtr;
          struct_BIO_SENSOR_HUB *nodeSetting = (struct_BIO_SENSOR_HUB *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin(qwiic) == 0x00) //Wire port. Returns 0x00 on success.
            temp->online = true;
        }
        break;
      case DEVICE_ISM330DHCX:
        {
          SparkFun_ISM330DHCX *tempDevice = (SparkFun_ISM330DHCX *)temp->classPtr;
          struct_ISM330DHCX *nodeSetting = (struct_ISM330DHCX *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin(qwiic, temp->address)) //Wire port, address
            temp->online = true;
        }
        break;
      case DEVICE_MMC5983MA:
        {
          SFE_MMC5983MA *tempDevice = (SFE_MMC5983MA *)temp->classPtr;
          struct_MMC5983MA *nodeSetting = (struct_MMC5983MA *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin(qwiic)) //Wire port
            temp->online = true;
        }
        break;
      case DEVICE_KX134:
        {
          SparkFun_KX134 *tempDevice = (SparkFun_KX134 *)temp->classPtr;
          struct_KX134 *nodeSetting = (struct_KX134 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin(qwiic, temp->address)) //Wire port, address
            temp->online = true;
        }
        break;
      case DEVICE_ADS1015:
        {
          ADS1015 *tempDevice = (ADS1015 *)temp->classPtr;
          struct_ADS1015 *nodeSetting = (struct_ADS1015 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin(temp->address, qwiic)) //address, Wire port
            temp->online = true;
        }
        break;
      case DEVICE_PCF8575:
        {
          PCF8575 *tempDevice = (PCF8575 *)temp->classPtr;
          struct_PCF8575 *nodeSetting = (struct_PCF8575 *)temp->configPtr; //Create a local pointer that points to same spot as node does
          if (nodeSetting->powerOnDelayMillis > qwiicPowerOnDelayMillis) qwiicPowerOnDelayMillis = nodeSetting->powerOnDelayMillis; // Increase qwiicPowerOnDelayMillis if required
          if (tempDevice->begin(temp->address)) // begin and set address. Returns true if connected
            temp->online = true;
        }
        break;
      default:
        SerialPrintf2("beginQwiicDevices: device type not found: %d\r\n", temp->deviceType);
        break;
    }

    if (temp->online == true)
    {
      printDebug(F("beginQwiicDevices: device is online\r\n"));
    }
    else
    {
      printDebug(F("beginQwiicDevices: device is **NOT** online\r\n"));
      everythingStarted = false;
    }

    temp = temp->next;
  }

  return everythingStarted;
}

//Pretty print all the online devices
int printOnlineDevice()
{
  int deviceCount = 0;

  //Step through the list
  node *temp = head;

  if (temp == NULL)
  {
    printDebug(F("printOnlineDevice: No devices detected\r\n"));
    return (0);
  }

  while (temp != NULL)
  {
    char sensorOnlineText[75];
    if (temp->online)
    {
      if (temp->muxAddress == 0)
        sprintf(sensorOnlineText, "%s online at address 0x%02X\r\n", getDeviceName(temp->deviceType), temp->address);
      else
        sprintf(sensorOnlineText, "%s online at address 0x%02X.0x%02X.%d\r\n", getDeviceName(temp->deviceType), temp->address, temp->muxAddress, temp->portNumber);

      deviceCount++;
    }
    else
    {
      sprintf(sensorOnlineText, "%s failed to respond\r\n", getDeviceName(temp->deviceType));
    }
    SerialPrint(sensorOnlineText);

    temp = temp->next;
  }

  if (settings.printDebugMessages == true)
  {
    SerialPrintf2("Device count: %d\r\n", deviceCount);
  }

  return (deviceCount);
}

//Given the node number, apply the node's configuration settings to the device
void configureDevice(uint8_t nodeNumber)
{
  node *temp = getNodePointer(nodeNumber);
  configureDevice(temp);
}

//Given the node pointer, apply the node's configuration settings to the device
void configureDevice(node * temp)
{
  uint8_t deviceType = (uint8_t)temp->deviceType;

  openConnection(temp->muxAddress, temp->portNumber); //Connect to this device through muxes as needed

  switch (deviceType)
  {
    case DEVICE_MULTIPLEXER:
      //Nothing to configure
      break;
    case DEVICE_LOADCELL_NAU7802:
      {
        NAU7802 *sensor = (NAU7802 *)temp->classPtr;
        struct_NAU7802 *sensorSetting = (struct_NAU7802 *)temp->configPtr;

        // Here we should reset the chip again - to clear the offset calibration - and only call calibrateAFE if desired

        sensor->reset();
        sensor->powerUp();
        sensor->setLDO(sensorSetting->LDO);
        sensor->setGain(sensorSetting->gain);
        sensor->setSampleRate(sensorSetting->sampleRate);
        //Turn off CLK_CHP. From 9.1 power on sequencing.
        uint8_t adc = sensor->getRegister(NAU7802_ADC);
        adc |= 0x30;
        sensor->setRegister(NAU7802_ADC, adc);
        sensor->setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR); //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
        sensor->clearBit(NAU7802_PGA_LDOMODE, NAU7802_PGA); //Ensure LDOMODE bit is clear - improved accuracy and higher DC gain, with ESR < 1 ohm
        sensor->setCalibrationFactor(sensorSetting->calibrationFactor);
        sensor->setZeroOffset(sensorSetting->zeroOffset);

        delay(sensor->getLDORampDelay()); // Wait for LDO to ramp

        if (sensorSetting->calibrationMode == 1) //Internal calibration
        {
          sensor->getWeight(true, 10); //Flush

          sensor->calibrateAFE(NAU7802_CALMOD_INTERNAL); //Recalibrate after changing gain / sample rate          
        }
        else if (sensorSetting->calibrationMode == 2) //Use saved or default external calibration
        {
          sensor->set24BitRegister(NAU7802_OCAL1_B2, sensorSetting->offsetReg);
          sensor->set32BitRegister(NAU7802_GCAL1_B3, sensorSetting->gainReg);
        }

        sensor->getWeight(true, 10); //Flush
      }
      break;
    case DEVICE_DISTANCE_VL53L1X:
      {
        SFEVL53L1X *sensor = (SFEVL53L1X *)temp->classPtr;
        struct_VL53L1X *sensorSetting = (struct_VL53L1X *)temp->configPtr;

        if (sensorSetting->distanceMode == VL53L1X_DISTANCE_MODE_SHORT)
          sensor->setDistanceModeShort();
        else
          sensor->setDistanceModeLong();

        sensor->setIntermeasurementPeriod(sensorSetting->intermeasurementPeriod - 1);
        sensor->setXTalk(sensorSetting->crosstalk);
        sensor->setOffset(sensorSetting->offset);

        sensor->startRanging(); //Write configuration bytes to initiate measurement
      }
      break;
    case DEVICE_GPS_UBLOX:
      {
        setQwiicPullups(0); //Disable pullups for u-blox comms.

        SFE_UBLOX_GNSS *sensor = (SFE_UBLOX_GNSS *)temp->classPtr;
        struct_ublox *nodeSetting = (struct_ublox *)temp->configPtr;

        sensor->setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

        sensor->saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the current ioPortsettings to flash and BBR

        sensor->setAutoPVT(nodeSetting->useAutoPVT); // Use autoPVT as required

        if (1000000ULL / settings.usBetweenReadings <= 1) //If we are slower than 1Hz logging rate
          // setNavigationFrequency expects a uint8_t to define the number of updates per second
          // So the slowest rate we can set with setNavigationFrequency is 1Hz
          // (Whereas UBX_CFG_RATE can actually support intervals as slow as 65535ms)
          sensor->setNavigationFrequency(1); //Set output rate to 1Hz
        else if (1000000ULL / settings.usBetweenReadings <= 10) //If we are slower than 10Hz logging rate
          sensor->setNavigationFrequency((uint8_t)(1000000ULL / settings.usBetweenReadings)); //Set output rate equal to our query rate
        else
          sensor->setNavigationFrequency(10); //Set nav freq to 10Hz. Max output depends on the module used.

        setQwiicPullups(settings.qwiicBusPullUps); //Re-enable pullups.
      }
      break;
    case DEVICE_PROXIMITY_VCNL4040:
      {
        VCNL4040 *sensor = (VCNL4040 *)temp->classPtr;
        struct_VCNL4040 *sensorSetting = (struct_VCNL4040 *)temp->configPtr;

        sensor->powerOnAmbient(); //Turn on ambient sensing
        sensor->setLEDCurrent(sensorSetting->LEDCurrent);
        sensor->setIRDutyCycle(sensorSetting->IRDutyCycle);
        sensor->setProxIntegrationTime(sensorSetting->proximityIntegrationTime);
        sensor->setProxResolution(sensorSetting->resolution);
        sensor->setAmbientIntegrationTime(sensorSetting->ambientIntegrationTime);
      }
      break;
    case DEVICE_TEMPERATURE_TMP117:
      {
        TMP117 *sensor = (TMP117 *)temp->classPtr;
        struct_TMP117 *sensorSetting = (struct_TMP117 *)temp->configPtr;

        sensor->setConversionAverageMode(sensorSetting->conversionAverageMode);
        sensor->setConversionCycleBit(sensorSetting->conversionCycle);
        sensor->setContinuousConversionMode();
      }
      break;
    case DEVICE_PRESSURE_MS5637:
      //Nothing to configure
      break;
    case DEVICE_PRESSURE_LPS25HB:
      //Nothing to configure
      break;
    case DEVICE_PRESSURE_LPS28DFW:
      //Nothing to configure
      break;
    case DEVICE_PHT_BME280:
      //Nothing to configure
      break;
    case DEVICE_UV_VEML6075:
      //Nothing to configure
      break;
    case DEVICE_LIGHT_VEML7700:
      //Nothing to configure
      break;
    case DEVICE_VOC_CCS811:
      //Nothing to configure
      break;
    case DEVICE_VOC_SGP30:
      {
        SGP30 *sensor = (SGP30 *)temp->classPtr;
        sensor->initAirQuality(); //Initializes sensor for air quality readings
      }
      break;
    case DEVICE_CO2_SCD30:
      {
        SCD30 *sensor = (SCD30 *)temp->classPtr;
        struct_SCD30 *sensorSetting = (struct_SCD30 *)temp->configPtr;

        //Apply one-off calibrations
        if(sensorSetting->applyCalibrationConcentration)
        {
          sensor->setForcedRecalibrationFactor(sensorSetting->calibrationConcentration);
          sensorSetting->applyCalibrationConcentration = false;
        }

        sensor->setMeasurementInterval(sensorSetting->measurementInterval);
        sensor->setAltitudeCompensation(sensorSetting->altitudeCompensation);
        sensor->setAmbientPressure(sensorSetting->ambientPressure);
        //sensor->setTemperatureOffset(sensorSetting->temperatureOffset);
      }
      break;
    case DEVICE_PHT_MS8607:
      {
        MS8607 *sensor = (MS8607 *)temp->classPtr;
        struct_MS8607 *sensorSetting = (struct_MS8607 *)temp->configPtr;

        if (sensorSetting->enableHeater == true)
          sensor->enable_heater();
        else
          sensor->disable_heater();

        sensor->set_pressure_resolution(sensorSetting->pressureResolution);
        sensor->set_humidity_resolution(sensorSetting->humidityResolution);
      }
      break;
    case DEVICE_TEMPERATURE_MCP9600:
      {
        MCP9600 *sensor = (MCP9600 *)temp->classPtr;

        //set the resolution on the ambient (cold) junction
        Ambient_Resolution ambientRes = RES_ZERO_POINT_0625; //_25 and _0625
        sensor->setAmbientResolution(ambientRes);

        Thermocouple_Resolution thermocoupleRes = RES_14_BIT; //12, 14, 16, and 18
        sensor->setThermocoupleResolution(thermocoupleRes);
      }
      break;
    case DEVICE_HUMIDITY_AHT20:
      //Nothing to configure
      break;
    case DEVICE_HUMIDITY_SHTC3:
      //Nothing to configure
      break;
    case DEVICE_ADC_ADS122C04:
      {
        SFE_ADS122C04 *sensor = (SFE_ADS122C04 *)temp->classPtr;
        struct_ADS122C04 *sensorSetting = (struct_ADS122C04 *)temp->configPtr;

        //Configure the wite mode for readPT100Centigrade and readPT100Fahrenheit
        //(readInternalTemperature and readRawVoltage change and restore the mode automatically)
        if (sensorSetting->useFourWireMode)
          sensor->configureADCmode(ADS122C04_4WIRE_MODE);
        else if (sensorSetting->useThreeWireMode)
          sensor->configureADCmode(ADS122C04_3WIRE_MODE);
        else if (sensorSetting->useTwoWireMode)
          sensor->configureADCmode(ADS122C04_2WIRE_MODE);
        else if (sensorSetting->useFourWireHighTemperatureMode)
          sensor->configureADCmode(ADS122C04_4WIRE_HI_TEMP);
        else if (sensorSetting->useThreeWireHighTemperatureMode)
          sensor->configureADCmode(ADS122C04_3WIRE_HI_TEMP);
        else if (sensorSetting->useTwoWireHighTemperatureMode)
          sensor->configureADCmode(ADS122C04_2WIRE_HI_TEMP);
      }
      break;
    case DEVICE_PRESSURE_MPR0025PA1:
      //Nothing to configure
      break;
    case DEVICE_PARTICLE_SNGCJA5:
      //Nothing to configure
      break;
    case DEVICE_VOC_SGP40:
      //Nothing to configure
      break;
    case DEVICE_PRESSURE_SDP3X:
      {
        SDP3X *sensor = (SDP3X *)temp->classPtr;
        struct_SDP3X *sensorSetting = (struct_SDP3X *)temp->configPtr;

        // Each triggered measurement takes 45ms to complete so we need to use continuous measurements
        sensor->stopContinuousMeasurement(); //Make sure continuous measurements are stopped or startContinuousMeasurement will fail
        sensor->startContinuousMeasurement(sensorSetting->massFlow, sensorSetting->averaging); //Request continuous measurements
      }
      break;
    case DEVICE_PRESSURE_MS5837:
      {
        MS5837 *sensor = (MS5837 *)temp->classPtr;
        struct_MS5837 *sensorSetting = (struct_MS5837 *)temp->configPtr;

        //sensor->setModel(sensorSetting->model); // We could override the sensor model, but let's not...
        sensorSetting->model = sensor->getModel();
        sensor->setFluidDensity(sensorSetting->fluidDensity);
      }
      break;
    case DEVICE_QWIIC_BUTTON:
      {
        QwiicButton *sensor = (QwiicButton *)temp->classPtr;
        struct_QWIIC_BUTTON *sensorSetting = (struct_QWIIC_BUTTON *)temp->configPtr;

        if (sensorSetting->ledState)
          sensor->LEDon(sensorSetting->ledBrightness);
        else
          sensor->LEDoff();
      }
      break;
    case DEVICE_BIO_SENSOR_HUB:
      {
        SparkFun_Bio_Sensor_Hub *sensor = (SparkFun_Bio_Sensor_Hub *)temp->classPtr;
        struct_BIO_SENSOR_HUB *sensorSetting = (struct_BIO_SENSOR_HUB *)temp->configPtr;

        sensor->configBpm(MODE_TWO); // MODE_TWO provides the oxygen R value
      }
      break;
    case DEVICE_ISM330DHCX:
      {
        SparkFun_ISM330DHCX *sensor = (SparkFun_ISM330DHCX *)temp->classPtr;
        struct_ISM330DHCX *sensorSetting = (struct_ISM330DHCX *)temp->configPtr;

        sensor->deviceReset();

        // Wait for it to finish reseting
        while( !sensor->getDeviceReset() ){
          delay(1);
        }

        sensor->setDeviceConfig();
        sensor->setBlockDataUpdate();

        // Set the output data rate and precision of the accelerometer
        sensor->setAccelDataRate(sensorSetting->accelRate);
        sensor->setAccelFullScale(sensorSetting->accelScale);

        // Turn on the accelerometer's filter and apply settings.
        sensor->setAccelFilterLP2(sensorSetting->accelFilterLP2);
        sensor->setAccelSlopeFilter(sensorSetting->accelSlopeFilter);

        // Set the output data rate and precision of the gyroscope
        sensor->setGyroDataRate(sensorSetting->gyroRate);
        sensor->setGyroFullScale(sensorSetting->gyroScale);

        // Turn on the gyroscope's filter and apply settings.
        sensor->setGyroFilterLP1(sensorSetting->gyroFilterLP1);
        sensor->setGyroLP1Bandwidth(sensorSetting->gyroLP1BW);
      }
      break;
    case DEVICE_MMC5983MA:
      {
        SFE_MMC5983MA *sensor = (SFE_MMC5983MA *)temp->classPtr;
        struct_MMC5983MA *sensorSetting = (struct_MMC5983MA *)temp->configPtr;

        sensor->softReset();

        sensor->enableAutomaticSetReset();
      }
      break;
    case DEVICE_KX134:
      {
        SparkFun_KX134 *sensor = (SparkFun_KX134 *)temp->classPtr;
        struct_KX134 *sensorSetting = (struct_KX134 *)temp->configPtr;

        sensor->softwareReset();
        delay(5);

        sensor->enableAccel(false);

        if (sensorSetting->range8G) sensor->setRange(SFE_KX134_RANGE8G);
        else if (sensorSetting->range16G) sensor->setRange(SFE_KX134_RANGE16G);
        else if (sensorSetting->range32G) sensor->setRange(SFE_KX134_RANGE32G);
        else sensor->setRange(SFE_KX134_RANGE64G);

        sensor->enableDataEngine();     // Enables the bit that indicates data is ready.

        if (sensorSetting->highSpeed) sensor->setOutputDataRate(9); // 400Hz
        else sensor->setOutputDataRate(6); // Default is 50Hz

        sensor->enableAccel();
      }
      break;
    case DEVICE_ADS1015:
      {
        ADS1015 *sensor = (ADS1015 *)temp->classPtr;
        struct_ADS1015 *sensorSetting = (struct_ADS1015 *)temp->configPtr;

        if (sensorSetting->gain23) sensor->setGain(ADS1015_CONFIG_PGA_TWOTHIRDS);
        else if (sensorSetting->gain1) sensor->setGain(ADS1015_CONFIG_PGA_1);
        else if (sensorSetting->gain2) sensor->setGain(ADS1015_CONFIG_PGA_2);
        else if (sensorSetting->gain4) sensor->setGain(ADS1015_CONFIG_PGA_4);
        else if (sensorSetting->gain8) sensor->setGain(ADS1015_CONFIG_PGA_8);
        else sensor->setGain(ADS1015_CONFIG_PGA_16);

        //sensor->setSampleRate(ADS1015_CONFIG_RATE_490HZ); // Default is 1600Hz

        sensor->useConversionReady(true);
      }
      break;
    case DEVICE_PCF8575:
        //Nothing to configure
      break;
    default:
      SerialPrintf3("configureDevice: Unknown device type %d: %s\r\n", deviceType, getDeviceName((deviceType_e)deviceType));
      break;
  }
}

//Apply device settings to each node
void configureQwiicDevices()
{
  //Step through the list
  node *temp = head;

  while (temp != NULL)
  {
    configureDevice(temp);
    temp = temp->next;
  }

  //Now that the settings are loaded and the devices are configured,
  //try for 400kHz but reduce to 100kHz if certain devices are attached
  setMaxI2CSpeed();
}

//Returns a pointer to the menu function that configures this particular device type
FunctionPointer getConfigFunctionPtr(uint8_t nodeNumber)
{
  FunctionPointer ptr = NULL;

  node *temp = getNodePointer(nodeNumber);
  if (temp == NULL) return (NULL);
  deviceType_e deviceType = temp->deviceType;

  switch (deviceType)
  {
    case DEVICE_MULTIPLEXER:
      ptr = (FunctionPointer)menuConfigure_Multiplexer;
      break;
    case DEVICE_LOADCELL_NAU7802:
      ptr = (FunctionPointer)menuConfigure_NAU7802;
      break;
    case DEVICE_DISTANCE_VL53L1X:
      ptr = (FunctionPointer)menuConfigure_VL53L1X;
      break;
    case DEVICE_GPS_UBLOX:
      ptr = (FunctionPointer)menuConfigure_ublox;
      break;
    case DEVICE_PROXIMITY_VCNL4040:
      ptr = (FunctionPointer)menuConfigure_VCNL4040;
      break;
    case DEVICE_TEMPERATURE_TMP117:
      ptr = (FunctionPointer)menuConfigure_TMP117;
      break;
    case DEVICE_PRESSURE_MS5637:
      ptr = (FunctionPointer)menuConfigure_MS5637;
      break;
    case DEVICE_PRESSURE_LPS25HB:
      ptr = (FunctionPointer)menuConfigure_LPS25HB;
      break;
    case DEVICE_PRESSURE_LPS28DFW:
      ptr = (FunctionPointer)menuConfigure_LPS28DFW;
      break;
    case DEVICE_PHT_BME280:
      ptr = (FunctionPointer)menuConfigure_BME280;
      break;
    case DEVICE_UV_VEML6075:
      ptr = (FunctionPointer)menuConfigure_VEML6075;
      break;
    case DEVICE_LIGHT_VEML7700:
      ptr = (FunctionPointer)menuConfigure_VEML7700;
      break;
    case DEVICE_VOC_CCS811:
      ptr = (FunctionPointer)menuConfigure_CCS811;
      break;
    case DEVICE_VOC_SGP30:
      ptr = (FunctionPointer)menuConfigure_SGP30;
      break;
    case DEVICE_CO2_SCD30:
      ptr = (FunctionPointer)menuConfigure_SCD30;
      break;
    case DEVICE_PHT_MS8607:
      ptr = (FunctionPointer)menuConfigure_MS8607;
      break;
    case DEVICE_TEMPERATURE_MCP9600:
      ptr = (FunctionPointer)menuConfigure_MCP9600;
      break;
    case DEVICE_HUMIDITY_AHT20:
      ptr = (FunctionPointer)menuConfigure_AHT20;
      break;
    case DEVICE_HUMIDITY_SHTC3:
      ptr = (FunctionPointer)menuConfigure_SHTC3;
      break;
    case DEVICE_ADC_ADS122C04:
      ptr = (FunctionPointer)menuConfigure_ADS122C04;
      break;
    case DEVICE_PRESSURE_MPR0025PA1:
      ptr = (FunctionPointer)menuConfigure_MPR0025PA1;
      break;
    case DEVICE_PARTICLE_SNGCJA5:
      ptr = (FunctionPointer)menuConfigure_SNGCJA5;
      break;
    case DEVICE_VOC_SGP40:
      ptr = (FunctionPointer)menuConfigure_SGP40;
      break;
    case DEVICE_PRESSURE_SDP3X:
      ptr = (FunctionPointer)menuConfigure_SDP3X;
      break;
    case DEVICE_PRESSURE_MS5837:
      ptr = (FunctionPointer)menuConfigure_MS5837;
      break;
    case DEVICE_QWIIC_BUTTON:
      ptr = (FunctionPointer)menuConfigure_QWIIC_BUTTON;
      break;
    case DEVICE_BIO_SENSOR_HUB:
      ptr = (FunctionPointer)menuConfigure_BIO_SENSOR_HUB;
      break;
    case DEVICE_ISM330DHCX:
      ptr = (FunctionPointer)menuConfigure_ISM330DHCX;
      break;
    case DEVICE_MMC5983MA:
      ptr = (FunctionPointer)menuConfigure_MMC5983MA;
      break;
    case DEVICE_KX134:
      ptr = (FunctionPointer)menuConfigure_KX134;
      break;
    case DEVICE_ADS1015:
      ptr = (FunctionPointer)menuConfigure_ADS1015;
      break;
    case DEVICE_PCF8575:
      ptr = (FunctionPointer)menuConfigure_PCF8575;
      break;
    default:
      SerialPrintln(F("getConfigFunctionPtr: Unknown device type"));
      SerialFlush();
      break;
  }

  return (ptr);
}

//Search the linked list for a given address
//Returns true if this device address already exists in our system
bool deviceExists(deviceType_e deviceType, uint8_t address, uint8_t muxAddress, uint8_t portNumber)
{
  node *temp = head;
  while (temp != NULL)
  {
    if (temp->address == address)
      if (temp->muxAddress == muxAddress)
        if (temp->portNumber == portNumber)
          if (temp->deviceType == deviceType) return (true);

    //Devices that were discovered on the main branch will be discovered over and over again
    //If a device has a 0/0 mux/port address, it's on the main branch and exists on all
    //sub branches.
    if (temp->address == address)
      if (temp->muxAddress == 0)
        if (temp->portNumber == 0)
        {
          if (temp->deviceType == deviceType) return (true);
          // Use DEVICE_TOTAL_DEVICES as a special case.
          // Return true if the device address exists on the main branch so we can avoid looking for it on mux branches.
          if (deviceType == DEVICE_TOTAL_DEVICES) return (true);
        }

    temp = temp->next;
  }
  return (false);
}

//Given the address of a device, enable muxes appropriately to open connection access device
//Return true if connection was opened
bool openConnection(uint8_t muxAddress, uint8_t portNumber)
{
  if (head == NULL)
  {
    SerialPrintln(F("OpenConnection Error: No devices in list"));
    return false;
  }

  if (muxAddress == 0) //This device is on main branch, nothing needed
    return true;

  //Get the pointer to the node that contains this mux address
  node *muxNode = getNodePointer(DEVICE_MULTIPLEXER, muxAddress, 0, 0);
  QWIICMUX *myMux = (QWIICMUX *)muxNode->classPtr;

  //Connect to this mux and port
  myMux->setPort(portNumber);

  return (true);
}

//Bubble sort the given linked list by the device address
//https://www.geeksforgeeks.org/c-program-bubble-sort-linked-list/
void bubbleSortDevices(struct node * start)
{
  int swapped, i;
  struct node *ptr1;
  struct node *lptr = NULL;

  //Checking for empty list
  if (start == NULL) return;

  do
  {
    swapped = 0;
    ptr1 = start;

    while (ptr1->next != lptr)
    {
      if (ptr1->address > ptr1->next->address)
      {
        swap(ptr1, ptr1->next);
        swapped = 1;
      }
      ptr1 = ptr1->next;
    }
    lptr = ptr1;
  }
  while (swapped);
}

//Swap data of two nodes a and b
void swap(struct node * a, struct node * b)
{
  node temp;

  temp.deviceType = a->deviceType;
  temp.address = a->address;
  temp.portNumber = a->portNumber;
  temp.muxAddress = a->muxAddress;
  temp.online = a->online;
  temp.classPtr = a->classPtr;
  temp.configPtr = a->configPtr;

  a->deviceType = b->deviceType;
  a->address = b->address;
  a->portNumber = b->portNumber;
  a->muxAddress = b->muxAddress;
  a->online = b->online;
  a->classPtr = b->classPtr;
  a->configPtr = b->configPtr;

  b->deviceType = temp.deviceType;
  b->address = temp.address;
  b->portNumber = temp.portNumber;
  b->muxAddress = temp.muxAddress;
  b->online = temp.online;
  b->classPtr = temp.classPtr;
  b->configPtr = temp.configPtr;
}

//The functions below are specific to the steps of auto-detection rather than node manipulation
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Available Qwiic devices
//We no longer use defines in the search table. These are just here for reference.
#define ADR_VEML6075 0x10
#define ADR_VEML7700 0x10
#define ADR_MPR0025PA1 0x18
#define ADR_KX134 0x1E //Alternate: 0x1F
#define ADR_SDP3X 0x21 //Alternates: 0x22, 0x23
#define ADR_NAU7802 0x2A
#define ADR_VL53L1X 0x29
#define ADR_MMC5983MA 0x30
#define ADR_SNGCJA5 0x33
#define ADR_AHT20 0x38
#define ADR_MS8607 0x40 //Humidity portion of the MS8607 sensor
#define ADR_UBLOX 0x42 //But can be set to any address
#define ADR_ADS122C04 0x45 //Alternates: 0x44, 0x41 and 0x40
#define ADR_TMP117 0x48 //Alternates: 0x49, 0x4A, and 0x4B
#define ADR_ADS1015 0x48 //Alternates: 0x49, 0x4A, and 0x4B
#define ADR_BIO_SENSOR_HUB 0x55
#define ADR_SGP30 0x58
#define ADR_SGP40 0x59
#define ADR_CCS811 0x5B //Alternates: 0x5A
#define ADR_LPS25HB 0x5D //Alternates: 0x5C
#define ADR_LPS28DFW 0x5C //Alternates: 0x5D
#define ADR_VCNL4040 0x60
#define ADR_SCD30 0x61
#define ADR_MCP9600 0x60 //0x60 to 0x67
#define ADR_ISM330DHCX 0x6A //Alternate: 0x6B
#define ADR_QWIIC_BUTTON 0x6F //But can be any address... Limit the range to 0x68-0x6F
#define ADR_MULTIPLEXER 0x70 //0x70 to 0x77
#define ADR_SHTC3 0x70
#define ADR_MS5637 0x76
#define ADR_MS5837 0x76
//#define ADR_MS8607 0x76 //Pressure portion of the MS8607 sensor. We'll catch the 0x40 first
#define ADR_BME280 0x77 //Alternates: 0x76
#define ADR_PCF8575 0x27  // All address lines pulled high

//Given an address, returns the device type if it responds as we would expect
//Does not test for multiplexers. See testMuxDevice for dedicated mux testing.
deviceType_e testDevice(uint8_t i2cAddress, uint8_t muxAddress, uint8_t portNumber)
{
  switch (i2cAddress)
  {
    case 0x10:
      {
        //Confidence: High - Checks ID
        VEML6075 sensor;
        if (sensor.begin(qwiic) == true) //Wire port
          return (DEVICE_UV_VEML6075);

        //Confidence: Low - just checks registers can be written to
        VEML7700 sensor1;
        if (sensor1.begin(qwiic) == true) //Wire port
          return (DEVICE_LIGHT_VEML7700);
      }
      break;
    case 0x18:
      {
        //Confidence: Medium - Checks the status byte power indication bit and three "always 0" bits
        SparkFun_MicroPressure sensor;
        if (sensor.begin(i2cAddress, qwiic) == true) //Address, Wire port
          if ((sensor.readStatus() & 0x5A) == 0x40) // Mask the power indication bit and three "always 0" bits
            return (DEVICE_PRESSURE_MPR0025PA1);
      }
      break;
    case 0x1E:
    case 0x1F:
      {
        //Confidence: High - reads ID register
        SparkFun_KX134 sensor;
        if (sensor.begin(qwiic, i2cAddress) == true) //Wire port, Address
          return (DEVICE_KX134);
      }
      break;
    case 0x21:
    case 0x22:
    case 0x23:
      {
        //Confidence: High - .begin reads the product ID
        SDP3X sensor;
        sensor.stopContinuousMeasurement(i2cAddress, qwiic); //Make sure continuous measurements are stopped or .begin will fail
        if (sensor.begin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_PRESSURE_SDP3X);
      }
      break;
    case 0x2A:
      {
        //Confidence: High - Checks 8 bit revision code (0x0F)
        NAU7802 sensor;
        if (sensor.begin(qwiic) == true) //Wire port. Note: this will reset the NAU7802 and call calibrateAFE but getRevisionCode fails otherwise
          if (sensor.getRevisionCode() == 0x0F)
            return (DEVICE_LOADCELL_NAU7802);
      }
      break;
    case 0x27:
      {
        PCF8575 sensor;
        bool retVal = sensor.begin();
        SerialPrintf2("Return Value of device 0x27: %d" \r\n, retVal);
        //if(sensor.begin(i2cAddress) == true)
          return (DEVICE_PCF8575);
      }
    case 0x29:
      {
        //Confidence: High - Checks 16 bit ID
        SFEVL53L1X sensor(qwiic); //Start with given wire port
        if (sensor.begin() == 0) //Returns 0 if init was successful. Wire port passed in constructor.
          return (DEVICE_DISTANCE_VL53L1X);
      }
      break;
    case 0x30:
      {
        //Confidence: high - reads PROD_ID_REG
        SFE_MMC5983MA sensor;
        if (sensor.begin(qwiic) == true) //Wire port
          return (DEVICE_MMC5983MA);
      }
      break;
    case 0x33:
      {
        //Confidence: low - basic isConnected test only...
        SFE_PARTICLE_SENSOR sensor;
        if (sensor.begin(qwiic) == true) //Wire port
          return (DEVICE_PARTICLE_SNGCJA5);
      }
      break;
    case 0x38:
      {
        //Confidence: Medium - begin() does a variety of inits and checks
        AHT20 sensor;
        if (sensor.begin(qwiic) == true) //Wire port
          return (DEVICE_HUMIDITY_AHT20);
      }
      break;
    case 0x40:
      {
        //Humidity portion of the MS8607 sensor
        //Confidence: High - does CRC on internal EEPROM read
        MS8607 sensor;
        if (sensor.begin(qwiic) == true) //Wire port
          return (DEVICE_PHT_MS8607);

        //Confidence: High - Configures ADC mode
        SFE_ADS122C04 sensor1;
        if (sensor1.begin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_ADC_ADS122C04);
      }
      break;
    case 0x41:
      {
        //Confidence: High - Configures ADC mode
        SFE_ADS122C04 sensor;
        if (sensor.begin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_ADC_ADS122C04);
      }
      break;
    case 0x42:
      {
        //Confidence: High - Sends/receives CRC checked data response
        setQwiicPullups(0); //Disable pullups to minimize CRC issues
        SFE_UBLOX_GNSS sensor;
        if(settings.printGNSSDebugMessages == true) sensor.enableDebugging(); // Enable debug messages if required
        if (sensor.begin(qwiic, i2cAddress) == true) //Wire port, address
        {
          setQwiicPullups(settings.qwiicBusPullUps); //Re-enable pullups to prevent ghosts at 0x43 onwards
          return (DEVICE_GPS_UBLOX);
        }
        setQwiicPullups(settings.qwiicBusPullUps); //Re-enable pullups for normal discovery
      }
      break;
    case 0x44:
    case 0x45:
      {
        //Confidence: High - Configures ADC mode
        SFE_ADS122C04 sensor;
        if (sensor.begin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_ADC_ADS122C04);
      }
      break;
    case 0x48:
    case 0x49:
    case 0x4A:
    case 0x4B:
      {
        //Confidence: High - Checks 16 bit ID
        TMP117 sensor;
        if (sensor.begin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_TEMPERATURE_TMP117);

        //Confidence: Low - only does a simple isConnected
        ADS1015 sensor1;
        if (sensor1.begin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_ADS1015);
      }
      break;
    case 0x55:
      {
        if (settings.identifyBioSensorHubs == true)
        {
          SparkFun_Bio_Sensor_Hub sensor(32, 11, i2cAddress); // Reset pin is 32, MFIO pin is 11
          if (sensor.begin(qwiic) == 0x00) //Wire port
            return (DEVICE_BIO_SENSOR_HUB);
        }
      }
      break;
    case 0x58:
      {
        SGP30 sensor;
        if (sensor.begin(qwiic) == true) //Wire port
          return (DEVICE_VOC_SGP30);
      }
      break;
    case 0x59:
      {
        SGP40 sensor;
        if (sensor.begin(qwiic) == true) //Wire port
          return (DEVICE_VOC_SGP40);
      }
      break;
    case 0x5A:
    case 0x5B:
      {
        CCS811 sensor(i2cAddress); //Start with given I2C address
        if (sensor.begin(qwiic) == true) //Wire port
          return (DEVICE_VOC_CCS811);
      }
      break;
    case 0x5C:
    case 0x5D:
      {
        // Same address, but different WHO_AM_I value.
        LPS28DFW sensor1;
        if (sensor1.begin(i2cAddress, qwiic) == LPS28DFW_OK) //Wire port, address
          return (DEVICE_PRESSURE_LPS28DFW);

        LPS25HB sensor;
        if (sensor.begin(qwiic, i2cAddress) == true) //Wire port, address
          return (DEVICE_PRESSURE_LPS25HB);

      }
      break;
    case 0x60:
      {
        //Always do the MCP9600 first. It's fussy...
        //Confidence: High - Checks 8bit ID
        MCP9600 sensor;
        if (sensor.begin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_TEMPERATURE_MCP9600);

        //Confidence: High - Checks ID
        VCNL4040 sensor1;
        if (sensor1.begin(qwiic) == true) //Wire port
          return (DEVICE_PROXIMITY_VCNL4040);
      }
      break;
    case 0x61:
      {
        //Always do the MCP9600 first. It's fussy...
        //Confidence: High - Checks 8bit ID
        MCP9600 sensor;
        if (sensor.begin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_TEMPERATURE_MCP9600);

        //Confidence: High - begin now checks FW Ver CRC
        SCD30 sensor1;
        if(settings.printDebugMessages == true) sensor1.enableDebugging(); // Enable debug messages if required
        // Set measBegin to false. beginQwiicDevices will call begin with measBegin set true.
        if (sensor1.begin(qwiic, true, false) == true) //Wire port, autoCalibrate, measBegin
          return (DEVICE_CO2_SCD30);
      }
      break;
    case 0x62:
    case 0x63:
    case 0x64:
    case 0x65:
    case 0x66:
    case 0x67:
      {
        //Always do the MCP9600 first. It's fussy...
        //Confidence: High - Checks 8bit ID
        MCP9600 sensor;
        if (sensor.begin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_TEMPERATURE_MCP9600);
      }
      break;
    case 0x68:
    case 0x69:
      {
        QwiicButton sensor;
        if (sensor.begin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_QWIIC_BUTTON);
      }
      break;
    case 0x6A:
    case 0x6B:
      {
        SparkFun_ISM330DHCX sensor;
        if (sensor.begin(qwiic, i2cAddress))
          return(DEVICE_ISM330DHCX);

        QwiicButton sensor1;
        if (sensor1.begin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_QWIIC_BUTTON);
      }
      break;
    case 0x6C:
    case 0x6D:
    case 0x6E:
    case 0x6F:
      {
        QwiicButton sensor;
        if (sensor.begin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_QWIIC_BUTTON);
      }
      break;
    case 0x70:
      {
        //Ignore devices we've already recorded. This was causing the mux to get tested, a begin() would happen, and the mux would be reset.
        if (deviceExists(DEVICE_MULTIPLEXER, i2cAddress, muxAddress, portNumber) == true) return (DEVICE_MULTIPLEXER);

        //Confidence: High - 16 bit ID check with CRC
        SHTC3 sensor;
        if (sensor.begin(qwiic) == 0) //Wire port. Device returns 0 upon success.
          return (DEVICE_HUMIDITY_SHTC3);
      }
      break;
    case 0x71:
    case 0x72:
    case 0x73:
    case 0x74:
    case 0x75:
      {
        //Ignore devices we've already recorded. This was causing the mux to get tested, a begin() would happen, and the mux would be reset.
        if (deviceExists(DEVICE_MULTIPLEXER, i2cAddress, muxAddress, portNumber) == true) return (DEVICE_MULTIPLEXER);
      }
      break;
    case 0x76:
      {
        //Ignore devices we've already recorded. This was causing the mux to get tested, a begin() would happen, and the mux would be reset.
        if (deviceExists(DEVICE_MULTIPLEXER, i2cAddress, muxAddress, portNumber) == true) return (DEVICE_MULTIPLEXER);

        //Confidence: High - does CRC on internal EEPROM read and checks sensor version
        MS5837 sensor2;
        if (sensor2.begin(qwiic) == true) //Wire port
        {
          if (sensor2.getModel() <= 1) // Check that getModel returns 0 or 1. It will (hopefully) return 255 if an MS5637 is attached.
            return (DEVICE_PRESSURE_MS5837);
        }

        //Confidence: High - does CRC on internal EEPROM read - but do this second as a MS5837 will appear as a MS5637
        MS5637 sensor;
        if (sensor.begin(qwiic) == true) //Wire port
          return (DEVICE_PRESSURE_MS5637);

        //Confidence: High - ID check but may pass with BMP280
        BME280 sensor1;
        sensor1.setI2CAddress(i2cAddress);
        if (sensor1.beginI2C(qwiic) == true) //Wire port
          return (DEVICE_PHT_BME280);

        //Pressure portion of the MS8607 combo sensor. We'll catch the 0x40 first
        //By the time we hit this address, MS8607 should have already been started by its first address
        //Since we don't need to harvest this address, this will cause this extra I2C address to be ignored/not added to node list, and not printed.
        return (DEVICE_UNKNOWN_DEVICE);
      }
      break;
    case 0x77:
      {
        //Ignore devices we've already recorded. This was causing the mux to get tested, a begin() would happen, and the mux would be reset.
        if (deviceExists(DEVICE_MULTIPLEXER, i2cAddress, muxAddress, portNumber) == true) return (DEVICE_MULTIPLEXER);

        BME280 sensor;
        sensor.setI2CAddress(i2cAddress);
        if (sensor.beginI2C(qwiic) == true) //Wire port
          return (DEVICE_PHT_BME280);
      }
      break;
    default:
      {
        if (muxAddress == 0)
        {
          SerialPrintf2("Unknown device at address (0x%02X)\r\n", i2cAddress);
        }
        else
        {
          SerialPrintf4("Unknown device at address (0x%02X)(Mux:0x%02X Port:%d)\r\n", i2cAddress, muxAddress, portNumber);
        }
        return DEVICE_UNKNOWN_DEVICE;
      }
      break;
  }
  SerialPrintf2("Known I2C address but device failed identification at address 0x%02X\r\n", i2cAddress);
  return DEVICE_UNKNOWN_DEVICE;
}

//Given an address, returns the device type if it responds as we would expect
//This version is dedicated to testing muxes and uses a custom .begin to avoid the slippery mux problem
//However, we also need to check if an MS8607 is attached (address 0x76) as it can cause the I2C bus to lock up if not detected correctly
//Also check for a BME280 - to prevent multiplexerBegin from confusing it
deviceType_e testMuxDevice(uint8_t i2cAddress, uint8_t muxAddress, uint8_t portNumber)
{
  switch (i2cAddress)
  {
    case 0x70:
      {
        //Ignore devices we've already recorded. This was causing the mux to get tested, a begin() would happen, and the mux would be reset.
        if (deviceExists(DEVICE_MULTIPLEXER, i2cAddress, muxAddress, portNumber) == true) return (DEVICE_MULTIPLEXER);

        //I don't think multiplexerBegin will cause any badness for the SHTC3 as its commands are all 16-bit
        //But, just in case, let's see if one is connected
        SHTC3 sensor;
        if (sensor.begin(qwiic) == 0) //Wire port. Device returns 0 upon success.
          return (DEVICE_HUMIDITY_SHTC3);

        //Confidence: Medium - Write/Read/Clear to 0x00
        if (multiplexerBegin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_MULTIPLEXER);
      }
      break;
    case 0x71:
    case 0x72:
    case 0x73:
    case 0x74:
    case 0x75:
      {
        //Ignore devices we've already recorded. This was causing the mux to get tested, a begin() would happen, and the mux would be reset.
        if (deviceExists(DEVICE_MULTIPLEXER, i2cAddress, muxAddress, portNumber) == true) return (DEVICE_MULTIPLEXER);

        //Confidence: Medium - Write/Read/Clear to 0x00
        if (multiplexerBegin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_MULTIPLEXER);
      }
      break;
    case 0x76:
      {
        //Ignore devices we've already recorded. This was causing the mux to get tested, a begin() would happen, and the mux would be reset.
        if (deviceExists(DEVICE_MULTIPLEXER, i2cAddress, muxAddress, portNumber) == true) return (DEVICE_MULTIPLEXER);

        // If an MS8607 is connected, multiplexerBegin causes the MS8607 to 'crash' and lock up the I2C bus... So we need to check if an MS8607 is connected first.
        // We will use the MS5637 as this will test for itself, the MS5837 and the pressure sensor of the MS8607
        // Just to make life even more complicated, a mux with address 0x76 will also appear as an MS5637 due to the way the MS5637 eeprom crc check is calculated.
        // So, we can't use .begin as the test for a MS5637 / MS5837 / MS8607. We need to be more creative!
        // If we write 0xA0 to i2cAddress and then read two bytes:
        //  A mux will return 0xA0A0
        //  An MS5637 / MS5837 / MS8607 / BME280 will return an eeprom or register value which _hopefully_ is not 0xA0A0!

        qwiic.beginTransmission((uint8_t)i2cAddress);
        qwiic.write((uint8_t)0xA0);
        uint8_t i2c_status = qwiic.endTransmission();

        if (i2c_status == 0) // If the I2C write was successful
        {
          qwiic.requestFrom((uint8_t)i2cAddress, 2U); // Read two bytes
          uint8_t buffer[2] = { 0, 0 };
          for (uint8_t i = 0; i < 2; i++)
          {
            buffer[i] = qwiic.read();
          }
          if ((buffer[0] != 0xA0) || (buffer[1] != 0xA0)) // If we read back something other than 0xA0A0 then we are probably talking to an MS5637 / MS5837 / MS8607 / BME280, not a mux
          {
            return (DEVICE_PRESSURE_MS5637);
          }
        }

        //Confidence: Medium - Write/Read/Clear to 0x00
        if (multiplexerBegin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_MULTIPLEXER);
      }
      break;
    case 0x77:
      {
        //Ignore devices we've already recorded. This was causing the mux to get tested, a begin() would happen, and the mux would be reset.
        if (deviceExists(DEVICE_MULTIPLEXER, i2cAddress, muxAddress, portNumber) == true) return (DEVICE_MULTIPLEXER);

        //multiplexerBegin confuses the BME280, so let's check if one is connected first
        // If we write 0xA0 to i2cAddress and then read two bytes:
        //  A mux will return 0xA0A0
        //  A BME280 will return a register value which _hopefully_ is not 0xA0A0!

        qwiic.beginTransmission((uint8_t)i2cAddress);
        qwiic.write((uint8_t)0xA0);
        uint8_t i2c_status = qwiic.endTransmission();

        if (i2c_status == 0) // If the I2C write was successful
        {
          qwiic.requestFrom((uint8_t)i2cAddress, 2U); // Read two bytes
          uint8_t buffer[2] = { 0, 0 };
          for (uint8_t i = 0; i < 2; i++)
          {
            buffer[i] = qwiic.read();
          }
          if ((buffer[0] != 0xA0) || (buffer[1] != 0xA0)) // If we read back something other than 0xA0A0 then we are probably talking to a BME280, not a mux
          {
            return (DEVICE_PHT_BME280);
          }
        }

        //Confidence: Medium - Write/Read/Clear to 0x00
        if (multiplexerBegin(i2cAddress, qwiic) == true) //Address, Wire port
          return (DEVICE_MULTIPLEXER);
      }
      break;
    default:
      {
        if (muxAddress == 0)
        {
          SerialPrintf2("Unknown device at address (0x%02X)\r\n", i2cAddress);
        }
        else
        {
          SerialPrintf4("Unknown device at address (0x%02X)(Mux:0x%02X Port:%d)\r\n", i2cAddress, muxAddress, portNumber);
        }
        return DEVICE_UNKNOWN_DEVICE;
      }
      break;
  }
  return DEVICE_UNKNOWN_DEVICE;
}

//Returns true if mux is present
//Tests for device ack to I2C address
//Then tests if device behaves as we expect
//Leaves with all ports disabled
bool multiplexerBegin(uint8_t deviceAddress, TwoWire &wirePort)
{
  wirePort.beginTransmission(deviceAddress);
  if (wirePort.endTransmission() != 0)
    return (false); //Device did not ACK

  //Write to device, expect a return
  setMuxPortState(0xA4, deviceAddress, wirePort, EXTRA_MUX_STARTUP_BYTES); //Set port register to a known value - using extra bytes to avoid the mux problem
  uint8_t response = getMuxPortState(deviceAddress, wirePort);
  setMuxPortState(0x00, deviceAddress, wirePort, 0); //Disable all ports - seems to work just fine without extra bytes (not sure why...)
  if (response == 0xA4) //Make sure we got back what we expected
  {
    response = getMuxPortState(deviceAddress, wirePort); //Make doubly sure we got what we expected
    if (response == 0x00)
    {
      return (true); //All good
    }
  }
  return (false);
}

//Writes a 8-bit value to mux
//Overwrites any other bits
//This allows us to enable/disable multiple ports at same time
bool setMuxPortState(uint8_t portBits, uint8_t deviceAddress, TwoWire &wirePort, int extraBytes)
{
  wirePort.beginTransmission(deviceAddress);
  for (int i = 0; i < extraBytes; i++)
  {
    wirePort.write((uint8_t)0x00); // Writing these extra bytes seems key to avoiding the slippery mux problem
  }
  wirePort.write(portBits);
  if (wirePort.endTransmission() != 0)
    return (false); //Device did not ACK
  return (true);
}

//Gets the current port state
//Returns byte that may have multiple bits set
uint8_t getMuxPortState(uint8_t deviceAddress, TwoWire &wirePort)
{
  //Read the current mux settings
  wirePort.beginTransmission(deviceAddress);
  wirePort.requestFrom(deviceAddress, 1);
  return (wirePort.read());
}

//Given a device number return the string associated with that entry
const char* getDeviceName(deviceType_e deviceNumber)
{
  switch (deviceNumber)
  {
    case DEVICE_MULTIPLEXER:
      return "Multiplexer";
      break;
    case DEVICE_LOADCELL_NAU7802:
      return "LoadCell-NAU7802";
      break;
    case DEVICE_DISTANCE_VL53L1X:
      return "Distance-VL53L1X";
      break;
    case DEVICE_GPS_UBLOX:
      return "GPS-ublox";
      break;
    case DEVICE_PROXIMITY_VCNL4040:
      return "Proximity-VCNL4040";
      break;
    case DEVICE_TEMPERATURE_TMP117:
      return "Temperature-TMP117";
      break;
    case DEVICE_PRESSURE_MS5637:
      return "Pressure-MS5637";
      break;
    case DEVICE_PRESSURE_LPS25HB:
      return "Pressure-LPS25HB";
      break;
    case DEVICE_PRESSURE_LPS28DFW:
      return "Pressure-LPS28DFW";
      break;
    case DEVICE_PHT_BME280:
      return "PHT-BME280";
      break;
    case DEVICE_UV_VEML6075:
      return "UV-VEML6075";
      break;
    case DEVICE_LIGHT_VEML7700:
      return "LIGHT-VEML7700";
      break;
    case DEVICE_VOC_CCS811:
      return "VOC-CCS811";
      break;
    case DEVICE_VOC_SGP30:
      return "VOC-SGP30";
      break;
    case DEVICE_CO2_SCD30:
      return "CO2-SCD30";
      break;
    case DEVICE_PHT_MS8607:
      return "PHT-MS8607";
      break;
    case DEVICE_TEMPERATURE_MCP9600:
      return "Temperature-MCP9600";
      break;
    case DEVICE_HUMIDITY_AHT20:
      return "Humidity-AHT20";
      break;
    case DEVICE_HUMIDITY_SHTC3:
      return "Humidity-SHTC3";
      break;
    case DEVICE_ADC_ADS122C04:
      return "ADC-ADS122C04";
      break;
    case DEVICE_PRESSURE_MPR0025PA1:
      return "Pressure-MPR";
      break;
    case DEVICE_PARTICLE_SNGCJA5:
      return "Particle-SNGCJA5";
      break;
    case DEVICE_VOC_SGP40:
      return "VOC-SGP40";
      break;
    case DEVICE_PRESSURE_SDP3X:
      return "Pressure-SDP3X";
      break;
    case DEVICE_PRESSURE_MS5837:
      return "Pressure-MS5837";
      break;
    case DEVICE_QWIIC_BUTTON:
      return "Qwiic_Button";
      break;
    case DEVICE_BIO_SENSOR_HUB:
      return "Bio-Sensor-Oximeter";
      break;
    case DEVICE_ISM330DHCX:
      return "IMU-ISM330DHCX";
      break;
    case DEVICE_MMC5983MA:
      return "Mag-MMC5983MA";
      break;
    case DEVICE_KX134:
      return "Accel-KX134";
      break;
    case DEVICE_ADS1015:
      return "ADC-ADS1015";
      break;
    case DEVICE_PCF8575:
      return "GPIO-PCF8575";
      break;

    case DEVICE_UNKNOWN_DEVICE:
      return "Unknown device";
      break;
    default:
      return "Unknown Status";
      break;
  }
  return "None";
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\logging.ino"
//Print a message both to terminal and to log
void msg(const char * message)
{
  SerialPrintln(message);
  if (online.microSD)
    sensorDataFile.println(message);
}

//Returns next available log file name
//Checks the spots in EEPROM for the next available LOG# file name
//Updates EEPROM and then appends to the new log file.
char* findNextAvailableLog(int &newFileNumber, const char *fileLeader)
{
  SdFile newFile; //This will contain the file for SD writing

  if (newFileNumber < 2) //If the settings have been reset, let's warn the user that this could take a while!
  {
    SerialPrintln(F("Finding the next available log file."));
    SerialPrintln(F("This could take a long time if the SD card contains many existing log files."));
  }

  if (newFileNumber > 0)
    newFileNumber--; //Check if last log file was empty. Reuse it if it is.

  //Search for next available log spot
  static char newFileName[40];
  while (1)
  {
    char newFileNumberStr[6];
    if (newFileNumber < 10)
      sprintf(newFileNumberStr, "0000%d", newFileNumber);
    else if (newFileNumber < 100)
      sprintf(newFileNumberStr, "000%d", newFileNumber);
    else if (newFileNumber < 1000)
      sprintf(newFileNumberStr, "00%d", newFileNumber);
    else if (newFileNumber < 10000)
      sprintf(newFileNumberStr, "0%d", newFileNumber);
    else
      sprintf(newFileNumberStr, "%d", newFileNumber);
    sprintf(newFileName, "%s%s.TXT", fileLeader, newFileNumberStr); //Splice the new file number into this file name. Max no. is 99999.

    if (sd.exists(newFileName) == false) break; //File name not found so we will use it.

    //File exists so open and see if it is empty. If so, use it.
    newFile.open(newFileName, O_READ);
    if (newFile.fileSize() == 0) break; // File is empty so we will use it. Note: we need to make the user aware that this can happen!

    newFile.close(); // Close this existing file we just opened.

    newFileNumber++; //Try the next number
    if (newFileNumber >= 100000) break; // Have we hit the maximum number of files?
  }
  
  newFile.close(); //Close this new file we just opened

  newFileNumber++; //Increment so the next power up uses the next file #

  if (newFileNumber >= 100000) // Have we hit the maximum number of files?
  {
    SerialPrint(F("***** WARNING! File number limit reached! (Overwriting "));
    SerialPrint(newFileName);
    SerialPrintln(F(") *****"));
    newFileNumber = 100000; // This will overwrite Log99999.TXT next time thanks to the newFileNumber-- above
  }
  else
  {
    SerialPrint(F("Logging to: "));
    SerialPrintln(newFileName);    
  }

  //Record new file number to EEPROM and to config file
  //This works because newFileNumber is a pointer to settings.newFileNumber
  recordSystemSettings();

  return (newFileName);
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\lowerPower.ino"
// Read the battery voltage
// If it is low, increment lowBatteryReadings
// If lowBatteryReadings exceeds lowBatteryReadingsLimit then powerDownOLA
void checkBattery(void)
{
#if(HARDWARE_VERSION_MAJOR >= 1)
  if (settings.enableLowBatteryDetection == true)
  {
    float voltage = readVIN(); // Read the battery voltage
    if (voltage < settings.lowBatteryThreshold) // Is the voltage low?
    {
      lowBatteryReadings++; // Increment the low battery count
      if (lowBatteryReadings > lowBatteryReadingsLimit) // Have we exceeded the low battery count limit?
      {
        // Gracefully powerDownOLA

        //Save files before powerDownOLA
        if (online.dataLogging == true)
        {
          sensorDataFile.sync();
          updateDataFileAccess(&sensorDataFile); // Update the file access time & date
          sensorDataFile.close(); //No need to close files. https://forum.arduino.cc/index.php?topic=149504.msg1125098#msg1125098
        }
        if (online.serialLogging == true)
        {
          serialDataFile.sync();
          updateDataFileAccess(&serialDataFile); // Update the file access time & date
          serialDataFile.close();
        }
      
        delay(sdPowerDownDelay); // Give the SD card time to finish writing ***** THIS IS CRITICAL *****

#ifdef noPowerLossProtection
        SerialPrintln(F("*** LOW BATTERY VOLTAGE DETECTED! RESETTING... ***"));
        SerialPrintln(F("*** PLEASE CHANGE THE POWER SOURCE TO CONTINUE ***"));
      
        SerialFlush(); //Finish any prints

        resetArtemis(); // Reset the Artemis so we don't get stuck in a low voltage infinite loop
#else
        SerialPrintln(F("***      LOW BATTERY VOLTAGE DETECTED! GOING INTO POWERDOWN      ***"));
        SerialPrintln(F("*** PLEASE CHANGE THE POWER SOURCE AND RESET THE OLA TO CONTINUE ***"));
      
        SerialFlush(); //Finish any prints

        powerDownOLA(); // Power down and wait for reset
#endif
      }
    }
    else
    {
      lowBatteryReadings = 0; // Reset the low battery count (to reject noise)
    }    
  }
#endif

#ifndef noPowerLossProtection // Redundant - since the interrupt is not attached if noPowerLossProtection is defined... But you never know...
  if (powerLossSeen)
    powerDownOLA(); // power down and wait for reset
#endif
}

//Power down the entire system but maintain running of RTC
//This function takes 100us to run including GPIO setting
//This puts the Apollo3 into 2.36uA to 2.6uA consumption mode
//With leakage across the 3.3V protection diode, it's approx 3.00uA.
void powerDownOLA(void)
{
#ifndef noPowerLossProtection // Probably redundant - included just in case detachInterrupt causes badness when it has not been attached
  //Prevent voltage supervisor from waking us from sleep
  detachInterrupt(PIN_POWER_LOSS);
#endif

  //Prevent stop logging button from waking us from sleep
  if (settings.useGPIO32ForStopLogging == true)
  {
    detachInterrupt(PIN_STOP_LOGGING); // Disable the interrupt
    pinMode(PIN_STOP_LOGGING, INPUT); // Remove the pull-up
  }

  //Prevent trigger from waking us from sleep
  if (settings.useGPIO11ForTrigger == true)
  {
    detachInterrupt(PIN_TRIGGER); // Disable the interrupt
    pinMode(PIN_TRIGGER, INPUT); // Remove the pull-up
  }

  //WE NEED TO POWER DOWN ASAP - we don't have time to close the SD files
  //Save files before going to sleep
  //  if (online.dataLogging == true)
  //  {
  //    sensorDataFile.sync();
  //    sensorDataFile.close();
  //  }
  //  if (online.serialLogging == true)
  //  {
  //    serialDataFile.sync();
  //    serialDataFile.close();
  //  }

  //SerialFlush(); //Don't waste time waiting for prints to finish

  //  Wire.end(); //Power down I2C
  qwiic.end(); //Power down I2C

  SPI.end(); //Power down SPI

  powerControlADC(false); // power_adc_disable(); //Power down ADC. It it started by default before setup().

  Serial.end(); //Power down UART
  if ((settings.useTxRxPinsForTerminal == true) || (online.serialLogging == true))
    Serial1.end();

  //Force the peripherals off
  //This will cause badness with v2.1 of the core but we don't care as we are waiting for a reset
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1);

  //Disable pads (this disables the LEDs too)
  for (int x = 0; x < 50; x++)
  {
    if ((x != PIN_POWER_LOSS) &&
        //(x != PIN_LOGIC_DEBUG) &&
        (x != PIN_MICROSD_POWER) &&
        (x != PIN_QWIIC_POWER) &&
        (x != PIN_IMU_POWER))
    {
      am_hal_gpio_pinconfig(x, g_AM_HAL_GPIO_DISABLE);
    }
  }

  //Make sure PIN_POWER_LOSS is configured as an input for the WDT
  pinMode(PIN_POWER_LOSS, INPUT); // BD49K30G-TL has CMOS output and does not need a pull-up

  //We can't leave these power control pins floating
  imuPowerOff();
  microSDPowerOff();

  //Keep Qwiic bus powered on if user desires it - but only for X04 to avoid a brown-out
#if(HARDWARE_VERSION_MAJOR == 0)
  if (settings.powerDownQwiicBusBetweenReads == true)
    qwiicPowerOff();
  else
    qwiicPowerOn(); //Make sure pins stays as output
#else
  qwiicPowerOff();
#endif

#ifdef noPowerLossProtection // If noPowerLossProtection is defined, then the WDT will already be running
  stopWatchdog();
#endif

  //Power down cache, flash, SRAM
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_ALL); // Power down all flash and cache
  am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_384K); // Retain all SRAM

  //Keep the 32kHz clock running for RTC
  am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
  am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ);

  while (1) // Stay in deep sleep until we get reset
  {
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP); //Sleep
  }
}

//Reset the Artemis
void resetArtemis(void)
{
  //Save files before resetting
  if (online.dataLogging == true)
  {
    sensorDataFile.sync();
    updateDataFileAccess(&sensorDataFile); // Update the file access time & date
    sensorDataFile.close(); //No need to close files. https://forum.arduino.cc/index.php?topic=149504.msg1125098#msg1125098
  }
  if (online.serialLogging == true)
  {
    serialDataFile.sync();
    updateDataFileAccess(&serialDataFile); // Update the file access time & date
    serialDataFile.close();
  }

  delay(sdPowerDownDelay); // Give the SD card time to finish writing ***** THIS IS CRITICAL *****

  SerialFlush(); //Finish any prints

  //  Wire.end(); //Power down I2C
  qwiic.end(); //Power down I2C

  SPI.end(); //Power down SPI

  powerControlADC(false); // power_adc_disable(); //Power down ADC. It it started by default before setup().

  Serial.end(); //Power down UART
  if ((settings.useTxRxPinsForTerminal == true) || (online.serialLogging == true))
    Serial1.end();

  //Force the peripherals off
  //This will cause badness with v2.1 of the core but we don't care as we are going to force a WDT reset
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1);

  //Disable pads
  for (int x = 0; x < 50; x++)
  {
    if ((x != PIN_POWER_LOSS) &&
        //(x != PIN_LOGIC_DEBUG) &&
        (x != PIN_MICROSD_POWER) &&
        (x != PIN_QWIIC_POWER) &&
        (x != PIN_IMU_POWER))
    {
      am_hal_gpio_pinconfig(x, g_AM_HAL_GPIO_DISABLE);
    }
  }

  //We can't leave these power control pins floating
  imuPowerOff();
  microSDPowerOff();

  //Disable power for the Qwiic bus
  qwiicPowerOff();

  //Disable the power LED
  powerLEDOff();

  //Enable the Watchdog so it can reset the Artemis
  petTheDog = false; // Make sure the WDT will not restart
#ifndef noPowerLossProtection // If noPowerLossProtection is defined, then the WDT will already be running
  startWatchdog(); // Start the WDT to generate a reset
#endif
  while (1) // That's all folks! Artemis will reset in 1.25 seconds
    ;
}

//Power everything down and wait for interrupt wakeup
void goToSleep(uint32_t sysTicksToSleep)
{
  printDebug("goToSleep: sysTicksToSleep = " + (String)sysTicksToSleep + "\r\n");
  
#ifndef noPowerLossProtection // Probably redundant - included just in case detachInterrupt causes badness when it has not been attached
  //Prevent voltage supervisor from waking us from sleep
  detachInterrupt(PIN_POWER_LOSS);
#endif

  //Prevent stop logging button from waking us from sleep
  if (settings.useGPIO32ForStopLogging == true)
  {
    detachInterrupt(PIN_STOP_LOGGING); // Disable the interrupt
    pinMode(PIN_STOP_LOGGING, INPUT); // Remove the pull-up
  }

  //Prevent trigger from waking us from sleep
  //(This should be redundant. We should not be going to sleep if triggering is enabled?)
  if (settings.useGPIO11ForTrigger == true)
  {
    detachInterrupt(PIN_TRIGGER); // Disable the interrupt
    pinMode(PIN_TRIGGER, INPUT); // Remove the pull-up
  }

  //Save files before going to sleep
  if (online.dataLogging == true)
  {
    sensorDataFile.sync();
    updateDataFileAccess(&sensorDataFile); // Update the file access time & date
    sensorDataFile.close(); //No need to close files. https://forum.arduino.cc/index.php?topic=149504.msg1125098#msg1125098
  }
  if (online.serialLogging == true)
  {
    serialDataFile.sync();
    updateDataFileAccess(&serialDataFile); // Update the file access time & date
    serialDataFile.close();
  }

  delay(sdPowerDownDelay); // Give the SD card time to finish writing ***** THIS IS CRITICAL *****

  //qwiic.end(); //DO NOT Power down I2C - causes badness with v2.1 of the core: https://github.com/sparkfun/Arduino_Apollo3/issues/412

  SPI.end(); //Power down SPI

  powerControlADC(false); //Power down ADC

  //Adjust sysTicks down by the amount we've be at 48MHz
  //Read millis _before_ we switch to the lower power clock!
  uint64_t msBeenAwake = rtcMillis() - lastAwakeTimeMillis;
  uint64_t sysTicksAwake = msBeenAwake * 32768L / 1000L; //Convert to 32kHz systicks
  if (sysTicksAwake < sysTicksToSleep)
    sysTicksToSleep -= sysTicksAwake;
  else
    sysTicksToSleep = 0;
  printDebug("goToSleep: sysTicksToSleep (adjusted) = " + (String)sysTicksToSleep + "\r\n\r\n");
  
  SerialFlush(); //Finish any prints
  Serial.end();
  if ((settings.useTxRxPinsForTerminal == true) || (online.serialLogging == true))
    Serial1.end();

  //Force the peripherals off

  //With v2.1 of the core, very bad things happen if the IOMs are disabled.
  //We must leave them enabled: https://github.com/sparkfun/Arduino_Apollo3/issues/412
  //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0); // SPI
  //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1); // qwiic I2C
  //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
  //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
  //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4);
  //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
  //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0);
  //if (settings.useTxRxPinsForTerminal == true)
  //  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1);

  //Disable as many pins as we can
  const int pinsToDisable[] = {0,1,2,10,14,17,12,24,25,28,36,38,39,40,41,42,43,45,21,22,16,31,35,-1};
  for (int x = 0; pinsToDisable[x] >= 0; x++)
  {
    am_hal_gpio_pinconfig(pinsToDisable[x], g_AM_HAL_GPIO_DISABLE);
  }

  //Do disable CIPO, COPI, SCLK and chip selects to minimise the current draw during deep sleep
  am_hal_gpio_pinconfig(PIN_SPI_CIPO , g_AM_HAL_GPIO_DISABLE); //ICM / microSD CIPO
  am_hal_gpio_pinconfig(PIN_SPI_COPI , g_AM_HAL_GPIO_DISABLE); //ICM / microSD COPI
  am_hal_gpio_pinconfig(PIN_SPI_SCK , g_AM_HAL_GPIO_DISABLE); //ICM / microSD SCK
  am_hal_gpio_pinconfig(PIN_IMU_CHIP_SELECT , g_AM_HAL_GPIO_DISABLE); //ICM CS
  am_hal_gpio_pinconfig(PIN_IMU_INT , g_AM_HAL_GPIO_DISABLE); //ICM INT
  am_hal_gpio_pinconfig(PIN_MICROSD_CHIP_SELECT , g_AM_HAL_GPIO_DISABLE); //microSD CS

  //If requested, disable pins 48 and 49 (UART0) to stop them back-feeding the CH340
  if (settings.serialTxRxDuringSleep == false)
  {
    am_hal_gpio_pinconfig(48 , g_AM_HAL_GPIO_DISABLE); //TX0
    am_hal_gpio_pinconfig(49 , g_AM_HAL_GPIO_DISABLE); //RX0
    if (settings.useTxRxPinsForTerminal == true)
    {
      am_hal_gpio_pinconfig(12 , g_AM_HAL_GPIO_DISABLE); //TX1
      am_hal_gpio_pinconfig(13 , g_AM_HAL_GPIO_DISABLE); //RX1
    }
  }

  //Make sure PIN_POWER_LOSS is configured as an input for the WDT
  pinMode(PIN_POWER_LOSS, INPUT); // BD49K30G-TL has CMOS output and does not need a pull-up

  //We can't leave these power control pins floating
  imuPowerOff();
  microSDPowerOff();

  //Keep Qwiic bus powered on if user desires it
  if (settings.powerDownQwiicBusBetweenReads == true)
  {
    //Do disable qwiic SDA and SCL to minimise the current draw during deep sleep
    am_hal_gpio_pinconfig(PIN_QWIIC_SDA , g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(PIN_QWIIC_SCL , g_AM_HAL_GPIO_DISABLE);
    qwiicPowerOff();
  }
  else
    qwiicPowerOn(); //Make sure pins stays as output

  //Leave the power LED on if the user desires it
  if (settings.enablePwrLedDuringSleep == true)
    powerLEDOn();
  else
    powerLEDOff();


#ifdef noPowerLossProtection
  // If noPowerLossProtection is defined, then the WDT will be running
  // We need to stop it otherwise it will wake the Artemis
  stopWatchdog();
#endif

  //Power down cache, flash, SRAM
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_ALL); // Power down all flash and cache
  am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_384K); // Retain all SRAM

  //Use the lower power 32kHz clock. Use it to run CT6 as well.
  am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
  am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_G_ENABLE);

  //Check that sysTicksToSleep is >> sysTicksAwake
  if (sysTicksToSleep > 3277) // Abort if we are trying to sleep for < 100ms
  {
    //Setup interrupt to trigger when the number of ms have elapsed
    am_hal_stimer_compare_delta_set(6, sysTicksToSleep);

    //We use counter/timer 6 to cause us to wake up from sleep but 0 to 7 are available
    //CT 7 is used for Software Serial. All CTs are used for Servo.
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREG);  //Clear CT6
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREG); //Enable C/T G=6

    //Enable the timer interrupt in the NVIC.
    NVIC_EnableIRQ(STIMER_CMPR6_IRQn);

    //Deep Sleep
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

    //Turn off interrupt
    NVIC_DisableIRQ(STIMER_CMPR6_IRQn);
    am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREG); //Disable C/T G=6
  }

  //We're BACK!
  wakeFromSleep();
}

//Power everything up gracefully
void wakeFromSleep()
{
  //Go back to using the main clock
  //am_hal_stimer_int_enable(AM_HAL_STIMER_INT_OVERFLOW);
  //NVIC_EnableIRQ(STIMER_IRQn);
  am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
  am_hal_stimer_config(AM_HAL_STIMER_HFRC_3MHZ);

  // Power up SRAM, turn on entire Flash
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_MAX);

  // Update lastAwakeTimeMillis
  lastAwakeTimeMillis = rtcMillis();

  //Turn on ADC
  powerControlADC(true);

  //Re-enable analog inputs
  //if (settings.logA11 == true) adcError += (uint32_t)ap3_set_pin_to_analog(11); // Set _pad_ 11 to analog if enabled for logging
  //if (settings.logA12 == true) adcError += (uint32_t)ap3_set_pin_to_analog(12); // Set _pad_ 12 to analog if enabled for logging
  //if (settings.logA13 == true) adcError += (uint32_t)ap3_set_pin_to_analog(13); // Set _pad_ 13 to analog if enabled for logging
  //if (settings.logA32 == true) adcError += (uint32_t)ap3_set_pin_to_analog(32); // Set _pad_ 32 to analog if enabled for logging
#if(HARDWARE_VERSION_MAJOR >= 1)
  //adcError += (uint32_t)ap3_set_pin_to_analog(PIN_VIN_MONITOR); // Set _pad_ PIN_VIN_MONITOR to analog
#endif

  //Run setup again

  //If 3.3V rail drops below 3V, system will enter low power mode and maintain RTC
  pinMode(PIN_POWER_LOSS, INPUT); // BD49K30G-TL has CMOS output and does not need a pull-up
  pin_config(PinName(PIN_POWER_LOSS), g_AM_HAL_GPIO_INPUT); // Make sure the pin does actually get re-configured

  delay(1); // Let PIN_POWER_LOSS stabilize

#ifndef noPowerLossProtection
  if (digitalRead(PIN_POWER_LOSS) == LOW) powerDownOLA(); //Check PIN_POWER_LOSS just in case we missed the falling edge
  //attachInterrupt(PIN_POWER_LOSS, powerDownOLA, FALLING); // We can't do this with v2.1.0 as attachInterrupt causes a spontaneous interrupt
  attachInterrupt(PIN_POWER_LOSS, powerLossISR, FALLING);
#else
  // No Power Loss Protection
  // Set up the WDT to generate a reset just in case the code crashes during a brown-out
  startWatchdog();
#endif
  powerLossSeen = false; // Make sure the flag is clear

  if (settings.useGPIO32ForStopLogging == true)
  {
    pinMode(PIN_STOP_LOGGING, INPUT_PULLUP);
    pin_config(PinName(PIN_STOP_LOGGING), g_AM_HAL_GPIO_INPUT_PULLUP); // Make sure the pin does actually get re-configured
    delay(1); // Let the pin stabilize
    attachInterrupt(PIN_STOP_LOGGING, stopLoggingISR, FALLING); // Enable the interrupt
    am_hal_gpio_pincfg_t intPinConfig = g_AM_HAL_GPIO_INPUT_PULLUP;
    intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
    pin_config(PinName(PIN_STOP_LOGGING), intPinConfig); // Make sure the pull-up does actually stay enabled
    stopLoggingSeen = false; // Make sure the flag is clear
  }

  if (settings.useGPIO11ForTrigger == true) //(This should be redundant. We should not be going to sleep if triggering is enabled?)
  {
    pinMode(PIN_TRIGGER, INPUT_PULLUP);
    pin_config(PinName(PIN_TRIGGER), g_AM_HAL_GPIO_INPUT_PULLUP); // Make sure the pin does actually get re-configured
    delay(1); // Let the pin stabilize
    am_hal_gpio_pincfg_t intPinConfig = g_AM_HAL_GPIO_INPUT_PULLUP;
    if (settings.fallingEdgeTrigger == true)
    {
      attachInterrupt(PIN_TRIGGER, triggerPinISR, FALLING); // Enable the interrupt
      intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
    }
    else
    {
      attachInterrupt(PIN_TRIGGER, triggerPinISR, RISING); // Enable the interrupt
      intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
    }
    pin_config(PinName(PIN_TRIGGER), intPinConfig); // Make sure the pull-up does actually stay enabled
    triggerEdgeSeen = false; // Make sure the flag is clear
  }

  if (settings.useGPIO11ForFastSlowLogging == true)
  {
    pinMode(PIN_TRIGGER, INPUT_PULLUP);
    pin_config(PinName(PIN_TRIGGER), g_AM_HAL_GPIO_INPUT_PULLUP); // Make sure the pin does actually get re-configured
  }

  pinMode(PIN_STAT_LED, OUTPUT);
  pin_config(PinName(PIN_STAT_LED), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_STAT_LED, LOW);

  powerLEDOn();

  //Re-enable pins 48 and 49 (UART0) if requested
  if (settings.serialTxRxDuringSleep == false)
  {
    pin_config(PinName(48), g_AM_BSP_GPIO_COM_UART_TX);    
    pin_config(PinName(49), g_AM_BSP_GPIO_COM_UART_RX);
    if (settings.useTxRxPinsForTerminal == true)
    {
      configureSerial1TxRx();
    }
  }

  //Re-enable CIPO, COPI, SCK and the chip selects but may as well leave ICM_INT disabled
  pin_config(PinName(PIN_SPI_CIPO), g_AM_BSP_GPIO_IOM0_MISO);
  pin_config(PinName(PIN_SPI_COPI), g_AM_BSP_GPIO_IOM0_MOSI);
  pin_config(PinName(PIN_SPI_SCK), g_AM_BSP_GPIO_IOM0_SCK);
  pin_config(PinName(PIN_IMU_CHIP_SELECT), g_AM_HAL_GPIO_OUTPUT);
  pin_config(PinName(PIN_MICROSD_CHIP_SELECT) , g_AM_HAL_GPIO_OUTPUT);

  //Re-enable the SDA and SCL pins
  pin_config(PinName(PIN_QWIIC_SCL), g_AM_BSP_GPIO_IOM1_SCL);
  pin_config(PinName(PIN_QWIIC_SDA), g_AM_BSP_GPIO_IOM1_SDA);

  Serial.begin(settings.serialTerminalBaudRate);

  if (settings.useTxRxPinsForTerminal == true)
  {
    Serial1.begin(settings.serialTerminalBaudRate); // Start the serial port
  }

  printDebug(F("wakeFromSleep: I'm awake!\r\n")); SerialFlush();
  
  beginQwiic(); //Power up Qwiic bus as early as possible

  SPI.begin(); //Needed if SD is disabled

  beginSD(true); //285 - 293ms

  enableCIPOpullUp(); // Enable CIPO pull-up _after_ beginSD

  beginDataLogging(); //180ms

  if (settings.useTxRxPinsForTerminal == false)
  {
    beginSerialLogging(); //20 - 99ms
    beginSerialOutput();
  }

  beginIMU(true); //61ms
  printDebug("wakeFromSleep: online.IMU = " + (String)online.IMU + "\r\n");

  //If we powered down the Qwiic bus, then re-begin and re-configure everything
  if (settings.powerDownQwiicBusBetweenReads == true)
  {
    beginQwiicDevices(); // beginQwiicDevices will wait for the qwiic devices to power up
    //loadDeviceSettingsFromFile(); //Apply device settings after the Qwiic bus devices have been detected and begin()'d
    configureQwiicDevices(); //Apply config settings to each device in the node list
  }
  
  // Late in the process to allow time for external device to generate unwanted signals
  while(Serial.available())  // Flush the input buffer
    Serial.read();
  if (settings.useTxRxPinsForTerminal == true)
  {
    while(Serial1.available())  // Flush the input buffer
      Serial1.read();
  }
  
  //When we wake up micros has been reset to zero so we need to let the main loop know to take a reading
  takeReading = true;
}

void stopLogging(void)
{
  detachInterrupt(PIN_STOP_LOGGING); // Disable the interrupt

  //Save files before going to sleep
  if (online.dataLogging == true)
  {
    sensorDataFile.sync();
    updateDataFileAccess(&sensorDataFile); // Update the file access time & date
    sensorDataFile.close(); //No need to close files. https://forum.arduino.cc/index.php?topic=149504.msg1125098#msg1125098
  }
  if (online.serialLogging == true)
  {
    serialDataFile.sync();
    updateDataFileAccess(&serialDataFile); // Update the file access time & date
    serialDataFile.close();
  }

  SerialPrint(F("Logging is stopped. Please reset OpenLog Artemis and open a terminal at "));
  Serial.print((String)settings.serialTerminalBaudRate);
  if (settings.useTxRxPinsForTerminal == true)
      Serial1.print((String)settings.serialTerminalBaudRate);
  SerialPrintln(F("bps..."));
  delay(sdPowerDownDelay); // Give the SD card time to shut down
  powerDownOLA();
}

void waitForQwiicBusPowerDelay() // Wait while the qwiic devices power up
{
  //Depending on what hardware is configured, the Qwiic bus may have only been turned on a few ms ago
  //Give sensors, specifically those with a low I2C address, time to turn on
  // If we're not using the SD card, everything will have happened much quicker than usual.
  uint64_t qwiicPowerHasBeenOnFor = rtcMillis() - qwiicPowerOnTime;
  printDebug("waitForQwiicBusPowerDelay: qwiicPowerHasBeenOnFor " + (String)((unsigned long)qwiicPowerHasBeenOnFor) + "ms\r\n");
  if (qwiicPowerHasBeenOnFor < (uint64_t)qwiicPowerOnDelayMillis)
  {
    uint64_t delayFor = (uint64_t)qwiicPowerOnDelayMillis - qwiicPowerHasBeenOnFor;
    printDebug("waitForQwiicBusPowerDelay: delaying for " + (String)((unsigned long)delayFor) + "\r\n");
    for (uint64_t i = 0; i < delayFor; i++)
    {
      checkBattery();
      delay(1);
    }
  }
}

void qwiicPowerOn()
{
  pinMode(PIN_QWIIC_POWER, OUTPUT);
  pin_config(PinName(PIN_QWIIC_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
#if(HARDWARE_VERSION_MAJOR == 0)
  digitalWrite(PIN_QWIIC_POWER, LOW);
#else
  digitalWrite(PIN_QWIIC_POWER, HIGH);
#endif

  qwiicPowerOnTime = rtcMillis(); //Record this time so we wait enough time before detecting certain sensors
}
void qwiicPowerOff()
{
  pinMode(PIN_QWIIC_POWER, OUTPUT);
  pin_config(PinName(PIN_QWIIC_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
#if(HARDWARE_VERSION_MAJOR == 0)
  digitalWrite(PIN_QWIIC_POWER, HIGH);
#else
  digitalWrite(PIN_QWIIC_POWER, LOW);
#endif
}

void microSDPowerOn()
{
  pinMode(PIN_MICROSD_POWER, OUTPUT);
  pin_config(PinName(PIN_MICROSD_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_MICROSD_POWER, LOW);
}
void microSDPowerOff()
{
  pinMode(PIN_MICROSD_POWER, OUTPUT);
  pin_config(PinName(PIN_MICROSD_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_MICROSD_POWER, HIGH);
}

void imuPowerOn()
{
  pinMode(PIN_IMU_POWER, OUTPUT);
  pin_config(PinName(PIN_IMU_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_IMU_POWER, HIGH);
}
void imuPowerOff()
{
  pinMode(PIN_IMU_POWER, OUTPUT);
  pin_config(PinName(PIN_IMU_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_IMU_POWER, LOW);
}

void powerLEDOn()
{
#if(HARDWARE_VERSION_MAJOR >= 1)
  pinMode(PIN_PWR_LED, OUTPUT);
  pin_config(PinName(PIN_PWR_LED), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_PWR_LED, HIGH); // Turn the Power LED on
#endif
}
void powerLEDOff()
{
#if(HARDWARE_VERSION_MAJOR >= 1)
  pinMode(PIN_PWR_LED, OUTPUT);
  pin_config(PinName(PIN_PWR_LED), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_PWR_LED, LOW); // Turn the Power LED off
#endif
}

//Returns the number of milliseconds according to the RTC
//(In increments of 10ms)
//Watch out for the year roll-over!
uint64_t rtcMillis()
{
  myRTC.getTime();
  uint64_t millisToday = 0;
  int dayOfYear = calculateDayOfYear(myRTC.dayOfMonth, myRTC.month, myRTC.year + 2000);
  millisToday += ((uint64_t)dayOfYear * 86400000ULL);
  millisToday += ((uint64_t)myRTC.hour * 3600000ULL);
  millisToday += ((uint64_t)myRTC.minute * 60000ULL);
  millisToday += ((uint64_t)myRTC.seconds * 1000ULL);
  millisToday += ((uint64_t)myRTC.hundredths * 10ULL);

  return (millisToday);
}

//Returns the day of year
//https://gist.github.com/jrleeman/3b7c10712112e49d8607
int calculateDayOfYear(int day, int month, int year)
{
  // Given a day, month, and year (4 digit), returns
  // the day of year. Errors return 999.

  int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  // Verify we got a 4-digit year
  if (year < 1000) {
    return 999;
  }

  // Check if it is a leap year, this is confusing business
  // See: https://support.microsoft.com/en-us/kb/214019
  if (year % 4  == 0) {
    if (year % 100 != 0) {
      daysInMonth[1] = 29;
    }
    else {
      if (year % 400 == 0) {
        daysInMonth[1] = 29;
      }
    }
  }

  // Make sure we are on a valid day of the month
  if (day < 1)
  {
    return 999;
  } else if (day > daysInMonth[month - 1]) {
    return 999;
  }

  int doy = 0;
  for (int i = 0; i < month - 1; i++) {
    doy += daysInMonth[i];
  }

  doy += day;
  return doy;
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\menuAnalogLogging.ino"
void menuAnalogLogging()
{
  while (1)
  {
#if(HARDWARE_VERSION_MAJOR == 0)
    SerialPrintln(F(""));
    SerialPrintln(F("Note: VIN logging is only supported on V10+ hardware. X04 will show 0.0V."));
#endif
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Analog Logging"));

    if (settings.identifyBioSensorHubs == false)
    {
      SerialPrint(F("1) Log analog pin 11 (2V Max): "));
      if (settings.logA11 == true) SerialPrintln(F("Enabled. (Triggering is disabled)"));
      else SerialPrintln(F("Disabled"));
    }

    SerialPrint(F("2) Log analog pin 12 (TX) (2V Max): "));
    if (settings.useTxRxPinsForTerminal == true) SerialPrintln(F("Disabled. (TX and RX pins are being used for the Terminal)"));
    else if (settings.logA12 == true) SerialPrintln(F("Enabled. (Serial output is disabled)"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("3) Log analog pin 13 (RX) (2V Max): "));
    if (settings.useTxRxPinsForTerminal == true) SerialPrintln(F("Disabled. (TX and RX pins are being used for the Terminal)"));
    else if (settings.logA13 == true) SerialPrintln(F("Enabled. (Serial logging is disabled)"));
    else SerialPrintln(F("Disabled"));

    if (settings.identifyBioSensorHubs == false)
    {
      SerialPrint(F("4) Log analog pin 32 (2V Max): "));
      if (settings.logA32 == true) SerialPrintln(F("Enabled. (Stop logging is disabled)"));
      else SerialPrintln(F("Disabled"));
    }

    SerialPrint(F("5) Log output type: "));
    if (settings.logAnalogVoltages == true) SerialPrintln(F("Calculated Voltage"));
    else SerialPrintln(F("Raw ADC reading"));

    SerialPrint(F("6) Log VIN (battery) voltage (6V Max): "));
    if (settings.logVIN == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
    {
      if (settings.identifyBioSensorHubs == false)
      {
        if(settings.logA11 == false)
        {
          settings.logA11 = true;
          // Disable triggering
          settings.useGPIO11ForTrigger = false;
          detachInterrupt(PIN_TRIGGER); // Disable the interrupt
          pinMode(PIN_TRIGGER, INPUT); // Remove the pull-up
          pin_config(PinName(PIN_TRIGGER), g_AM_HAL_GPIO_INPUT); // Make sure the pin does actually get re-configured
          triggerEdgeSeen = false; // Make sure the flag is clear
        }
        else
          settings.logA11 = false;
      }
      else
      {
        SerialPrintln(F(""));
        SerialPrintln(F("Analog logging on pin 11 is not possible. Bio Sensor Hubs (Pulse Oximeters) are enabled."));
        SerialPrintln(F(""));
      }      
    }
    else if (incoming == '2')
    {
      if(settings.logA12 == false)
      {
        if (settings.useTxRxPinsForTerminal == true)
        {
          SerialPrintln(F(""));
          SerialPrintln(F("Analog logging on pin 12 is not possible. TX and RX pins are being used for the Terminal."));
          SerialPrintln(F(""));
        }
        else
        {
          online.serialOutput = false; // Disable serial output
          settings.outputSerial = false;
          settings.logA12 = true;
        }
      }
      else
        settings.logA12 = false;
    }
    else if (incoming == '3')
    {
      if(settings.logA13 == false)
      {
        if (settings.useTxRxPinsForTerminal == true)
        {
          SerialPrintln(F(""));
          SerialPrintln(F("Analog logging on pin 13 is not possible. TX and RX pins are being used for the Terminal."));
          SerialPrintln(F(""));
        }
        else
        {
          online.serialLogging = false; //Disable serial logging
          settings.logSerial = false;
          settings.logA13 = true;
        }
      }
      else
        settings.logA13 = false;
    }
    else if (incoming == '4')
    {
      if (settings.identifyBioSensorHubs == false)
      {
        if(settings.logA32 == false)
        {
          settings.logA32 = true;
          // Disable stop logging
          settings.useGPIO32ForStopLogging = false;
          detachInterrupt(PIN_STOP_LOGGING); // Disable the interrupt
          pinMode(PIN_STOP_LOGGING, INPUT); // Remove the pull-up
          pin_config(PinName(PIN_STOP_LOGGING), g_AM_HAL_GPIO_INPUT); // Make sure the pin does actually get re-configured
        }
        else
          settings.logA32 = false;
      }
      else
      {
        SerialPrintln(F(""));
        SerialPrintln(F("Analog logging on pin 32 is not possible. Bio Sensor Hubs (Pulse Oximeters) are enabled."));
        SerialPrintln(F(""));
      }      
    }
    else if (incoming == '5')
      settings.logAnalogVoltages ^= 1;
    else if (incoming == '6')
    {
      settings.logVIN ^= 1;
    }
    else if (incoming == 'x')
      return;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      return;
    else
      printUnknown(incoming);
  }
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\menuAttachedDevices.ino"
/*
  To add a new sensor to the system:

  Add the library in OpenLog_Artemis
  Add DEVICE_ name to settings.h
  Add struct_MCP9600 to settings.h - This will define what settings for the sensor we will control

  Add gathering of data to gatherDeviceValues() in Sensors
  Add helper text to printHelperText() in Sensors

  Add class creation to addDevice() in autoDetect
  Add begin fucntions to beginQwiicDevices() in autoDetect
  Add configuration functions to configureDevice() in autoDetect
  Add pointer to configuration menu name to getConfigFunctionPtr() in autodetect
  Add test case to testDevice() in autoDetect
  Add pretty print device name to getDeviceName() in autoDetect

  Add menu title to menuAttachedDevices() list in menuAttachedDevices
  Create a menuConfigure_LPS25HB() function in menuAttachedDevices

  Add settings to the save/load device file settings in nvm
*/


//Let's see what's on the I2C bus
//Scan I2C bus including sub-branches of multiplexers
//Creates a linked list of devices
//Creates appropriate classes for each device
//Begin()s each device in list
//Returns true if devices detected > 0
bool detectQwiicDevices()
{
  printDebug(F("detectQwiicDevices started\r\n"));
  bool somethingDetected = false;

  qwiic.setClock(AM_HAL_IOM_100KHZ); //During detection, go slow

  setQwiicPullups(settings.qwiicBusPullUps); //Set pullups. (Redundant. beginQwiic has done this too.) If we don't have pullups, detectQwiicDevices() takes ~900ms to complete. We'll disable pullups if something is detected.

  //24k causes a bunch of unknown devices to be falsely detected.
  //setQwiicPullups(24); //Set pullups to 24k. If we don't have pullups, detectQwiicDevices() takes ~900ms to complete. We'll disable pullups if something is detected.

  waitForQwiicBusPowerDelay(); // Wait while the qwiic devices power up

  // Note: The MCP9600 (Qwiic Thermocouple) is a fussy device. If we use beginTransmission + endTransmission more than once
  // the second and subsequent times will fail. The MCP9600 only ACKs the first time. The MCP9600 also appears to be able to
  // lock up the I2C bus if you don't discover it and then begin it in one go...
  // The following code has been restructured to try and keep the MCP9600 happy.

  SerialPrintln(F("Identifying Qwiic Muxes..."));

  //First scan for Muxes. Valid addresses are 0x70 to 0x77 (112 to 119).
  //If any are found, they will be begin()'d causing their ports to turn off
  //testMuxDevice will check if an MS8607 is attached (address 0x76) as it can cause the I2C bus to lock up if we try to detect it as a mux
  uint8_t muxCount = 0;
  for (uint8_t address = 0x70 ; address < 0x78 ; address++)
  {
    qwiic.beginTransmission(address);
    if (qwiic.endTransmission() == 0)
    {
      //if (address == 0x74) // Debugging the slippery mux bug - trigger the scope when we test mux 0x74
      //{
      //  digitalWrite(PIN_LOGIC_DEBUG, LOW);
      //  digitalWrite(PIN_LOGIC_DEBUG, HIGH);
      //}
      somethingDetected = true;
      if (settings.printDebugMessages == true)
      {
        SerialPrintf2("detectQwiicDevices: something detected at address 0x%02X\r\n", address);
      }
      deviceType_e foundType = testMuxDevice(address, 0, 0); //No mux or port numbers for this test
      if (foundType == DEVICE_MULTIPLEXER)
      {
        addDevice(foundType, address, 0, 0); //Add this device to our map
        if (settings.printDebugMessages == true)
        {
          SerialPrintf2("detectQwiicDevices: multiplexer found at address 0x%02X\r\n", address);
        }
        muxCount++;
      }
      else if (foundType == DEVICE_PRESSURE_MS5637)
      {
        if (settings.printDebugMessages == true)
        {
          SerialPrintf2("detectQwiicDevices: MS8607/MS5637/MS5837/BME280 found at address 0x%02X. Ignoring it for now...\r\n", address);
        }
      }
      else if (foundType == DEVICE_PHT_BME280)
      {
        if (settings.printDebugMessages == true)
        {
          SerialPrintf2("detectQwiicDevices: BME280 found at address 0x%02X. Ignoring it for now...\r\n", address);
        }
      }
      else if (foundType == DEVICE_HUMIDITY_SHTC3)
      {
        if (settings.printDebugMessages == true)
        {
          SerialPrintf2("detectQwiicDevices: SHTC3 found at address 0x%02X. Ignoring it for now...\r\n", address);
        }
      }
    }
  }

  if (muxCount > 0)
  {
    if (settings.printDebugMessages == true)
    {
      SerialPrintf2("detectQwiicDevices: found %d", muxCount);
      if (muxCount == 1)
        SerialPrintln(F(" multiplexer"));
      else
        SerialPrintln(F(" multiplexers"));
    }
    beginQwiicDevices(); //begin() the muxes to disable their ports
  }

  //Before going into mux sub branches, scan the main branch for all remaining devices
  SerialPrintln(F("Identifying Qwiic Devices..."));
  bool foundMS8607 = false; // The MS8607 appears as two devices (MS8607 and MS5637). We need to skip the MS5637/MS5837 if we have found a MS8607.
  for (uint8_t address = 1 ; address < 127 ; address++)
  {
    qwiic.beginTransmission(address);
    if (qwiic.endTransmission() == 0)
    {
      somethingDetected = true;
      if (settings.printDebugMessages == true)
      {
        SerialPrintf2("detectQwiicDevices: something detected at address 0x%02X\r\n", address);
      }
      deviceType_e foundType = testDevice(address, 0, 0); //No mux or port numbers for this test
      if (foundType != DEVICE_UNKNOWN_DEVICE)
      {
        if ((foundMS8607 == true) && ((foundType == DEVICE_PRESSURE_MS5637) || (foundType == DEVICE_PRESSURE_MS5837)))
        {
          ; // Skip MS5637/MS5837 as we have already found an MS8607
        }
        else
        {
          if (addDevice(foundType, address, 0, 0) == true) //Records this device. //Returns false if mux/device was already recorded.
          {
            if (settings.printDebugMessages == true)
            {
              SerialPrintf3("detectQwiicDevices: added %s at address 0x%02X\r\n", getDeviceName(foundType), address);
            }
          }
        }
        if (foundType == DEVICE_PHT_MS8607)
        {
          foundMS8607 = true; // Flag that we have found an MS8607
        }
      }
    }
  }

  if (somethingDetected == false) return (false);

  //If we have muxes, begin scanning their sub nets
  if (muxCount > 0)
  {
    SerialPrintln(F("Multiplexers found. Scanning sub nets..."));

    //Step into first mux and begin stepping through ports
    for (int muxNumber = 0 ; muxNumber < muxCount ; muxNumber++)
    {
      //The node tree starts with muxes so we can align node numbers
      node *muxNode = getNodePointer(muxNumber);
      QWIICMUX *myMux = (QWIICMUX *)muxNode->classPtr;

      printDebug("detectQwiicDevices: scanning the ports of multiplexer " + (String)muxNumber);
      printDebug(F("\r\n"));

      for (int portNumber = 0 ; portNumber < 8 ; portNumber++) //Assumes we are using a mux with 8 ports max
      {
        myMux->setPort(portNumber);
        foundMS8607 = false; // The MS8607 appears as two devices (MS8607 and MS5637). We need to skip the MS5637 if we have found a MS8607.

        printDebug("detectQwiicDevices: scanning port number " + (String)portNumber);
        printDebug(" on multiplexer " + (String)muxNumber);
        printDebug(F("\r\n"));

        //Scan this new bus for new addresses
        for (uint8_t address = 1 ; address < 127 ; address++)
        {
          // If we found a device on the main branch, we cannot/should not attempt to scan for it on mux branches or bad things may happen
          if (deviceExists(DEVICE_TOTAL_DEVICES, address, 0, 0)) // Check if we found any type of device with this address on the main branch
          {
            if (settings.printDebugMessages == true)
            {
              SerialPrintf2("detectQwiicDevices: skipping device address 0x%02X because we found one on the main branch\r\n", address);
            }
          }
          else
          {
            qwiic.beginTransmission(address);
            if (qwiic.endTransmission() == 0)
            {
              // We don't need to do anything special for the MCP9600 here, because we can guarantee that beginTransmission + endTransmission
              // have only been used once for each MCP9600 address

              somethingDetected = true;

              deviceType_e foundType = testDevice(address, muxNode->address, portNumber);
              if (foundType != DEVICE_UNKNOWN_DEVICE)
              {
                if ((foundType == DEVICE_PRESSURE_MS5637) && (foundMS8607 == true))
                {
                  ; // Skip MS5637 as we have already found an MS8607
                }
                else
                {
                  if (foundType == DEVICE_MULTIPLEXER) // Let's ignore multiplexers hanging off multiplexer ports. (Multiple muxes on the main branch is OK.)
                  {
                    if (settings.printDebugMessages == true)
                      SerialPrintf5("detectQwiicDevices: ignoring %s at address 0x%02X.0x%02X.%d\r\n", getDeviceName(foundType), address, muxNode->address, portNumber);
                  }
                  else
                  {
                    if (addDevice(foundType, address, muxNode->address, portNumber) == true) //Record this device, with mux port specifics.
                    {
                      if (settings.printDebugMessages == true)
                        SerialPrintf5("detectQwiicDevices: added %s at address 0x%02X.0x%02X.%d\r\n", getDeviceName(foundType), address, muxNode->address, portNumber);
                    }
                  }
                }
                if (foundType == DEVICE_PHT_MS8607)
                {
                  foundMS8607 = true; // Flag that we have found an MS8607
                }
              }
            } //End I2c check
          } //End not on main branch check
        } //End I2C scanning
      } //End mux port stepping

      myMux->setPortState(0); // Disable all ports on this mux now that we have finished scanning them.

    } //End mux stepping
  } //End mux > 0

  bubbleSortDevices(head); //This may destroy mux alignment to node 0.

  //*** Let's leave pull-ups set to 1k and only disable them when taking to a u-blox device ***
  //setQwiicPullups(0); //We've detected something on the bus so disable pullups.

  //We need to call setMaxI2CSpeed in configureQwiicDevices
  //We cannot do it here as the device settings have not been loaded

  SerialPrintln(F("Autodetect complete"));

  return (true);
} // /detectQwiicDevices

void menuAttachedDevices()
{
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Attached Devices"));

    int availableDevices = 0;

    //Step through node list
    node *temp = head;

    if (temp == NULL)
      SerialPrintln(F("**No devices detected on Qwiic bus**"));

    while (temp != NULL)
    {
      //Exclude multiplexers from the list
      if (temp->deviceType != DEVICE_MULTIPLEXER)
      {
        char strAddress[50];
        if (temp->muxAddress == 0)
          sprintf(strAddress, "(0x%02X)", temp->address);
        else
          sprintf(strAddress, "(0x%02X)(Mux:0x%02X Port:%d)", temp->address, temp->muxAddress, temp->portNumber);

        char strDeviceMenu[10];
        sprintf(strDeviceMenu, "%d)", availableDevices++ + 1);

        switch (temp->deviceType)
        {
          case DEVICE_MULTIPLEXER:
            //SerialPrintf3("%s Multiplexer %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_LOADCELL_NAU7802:
            SerialPrintf3("%s NAU7802 Weight Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_DISTANCE_VL53L1X:
            SerialPrintf3("%s VL53L1X Distance Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_GPS_UBLOX:
            SerialPrintf3("%s u-blox GPS Receiver %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_PROXIMITY_VCNL4040:
            SerialPrintf3("%s VCNL4040 Proximity Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_TEMPERATURE_TMP117:
            SerialPrintf3("%s TMP117 High Precision Temperature Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_PRESSURE_MS5637:
            SerialPrintf3("%s MS5637 Pressure Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_PRESSURE_LPS25HB:
            SerialPrintf3("%s LPS25HB Pressure Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_PRESSURE_LPS28DFW:
            SerialPrintf3("%s LPS28DFW Pressure Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_PHT_BME280:
            SerialPrintf3("%s BME280 Pressure/Humidity/Temp (PHT) Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_UV_VEML6075:
            SerialPrintf3("%s VEML6075 UV Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_LIGHT_VEML7700:
            SerialPrintf3("%s VEML7700 Ambient Light Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_VOC_CCS811:
            SerialPrintf3("%s CCS811 tVOC and CO2 Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_VOC_SGP30:
            SerialPrintf3("%s SGP30 tVOC and CO2 Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_CO2_SCD30:
            SerialPrintf3("%s SCD30 CO2 Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_PHT_MS8607:
            SerialPrintf3("%s MS8607 Pressure/Humidity/Temp (PHT) Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_TEMPERATURE_MCP9600:
            SerialPrintf3("%s MCP9600 Thermocouple Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_HUMIDITY_AHT20:
            SerialPrintf3("%s AHT20 Humidity Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_HUMIDITY_SHTC3:
            SerialPrintf3("%s SHTC3 Humidity Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_ADC_ADS122C04:
            SerialPrintf3("%s ADS122C04 ADC (Qwiic PT100) %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_PRESSURE_MPR0025PA1:
            SerialPrintf3("%s MPR MicroPressure Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_PARTICLE_SNGCJA5:
            SerialPrintf3("%s SN-GCJA5 Particle Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_VOC_SGP40:
            SerialPrintf3("%s SGP40 VOC Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_PRESSURE_SDP3X:
            SerialPrintf3("%s SDP3X Differential Pressure Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_PRESSURE_MS5837:
            SerialPrintf3("%s MS5837 (BAR30 / BAR02) Pressure Sensor %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_QWIIC_BUTTON:
            SerialPrintf3("%s Qwiic Button %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_BIO_SENSOR_HUB:
            SerialPrintf3("%s Bio Sensor Pulse Oximeter %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_ISM330DHCX:
            SerialPrintf3("%s ISM330DHCX IMU %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_MMC5983MA:
            SerialPrintf3("%s MMC5983MA Magnetometer %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_KX134:
            SerialPrintf3("%s KX134 Accelerometer %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_ADS1015:
            SerialPrintf3("%s ADS1015 ADC %s\r\n", strDeviceMenu, strAddress);
            break;
          case DEVICE_PCF8575:
            SerialPrintf3("%s PCF8575 GPIO expander %s\r\n", strDeviceMenu, strAddress);
            break;

          default:
            SerialPrintf2("Unknown device type %d in menuAttachedDevices\r\n", temp->deviceType);
            break;
        }
      }

      temp = temp->next;
    }

    availableDevices++;
    SerialPrintf2("%d) Configure Qwiic Settings\r\n", availableDevices);
    availableDevices++;
    if (settings.identifyBioSensorHubs == true)
    {
      SerialPrintf2("%d) Detect Bio Sensor Pulse Oximeter: Enabled\r\n", availableDevices);
    }
    else
    {
      SerialPrintf2("%d) Detect Bio Sensor Pulse Oximeter: Disabled\r\n", availableDevices);
    }

    SerialPrintln(F("x) Exit"));

    int nodeNumber = getNumber(menuTimeout); //Timeout after x seconds
    if (nodeNumber > 0 && nodeNumber < availableDevices - 1)
    {
      //Lookup the function we need to call based the node number
      FunctionPointer functionPointer = getConfigFunctionPtr(nodeNumber - 1);

      //Get the configPtr for this given node
      void *deviceConfigPtr = getConfigPointer(nodeNumber - 1);
      functionPointer(deviceConfigPtr); //Call the appropriate config menu with a pointer to this node's configPtr

      configureDevice(nodeNumber - 1); //Reconfigure this device with the new settings
    }
    else if (nodeNumber == availableDevices - 1)
    {
      menuConfigure_QwiicBus();
    }
    else if (nodeNumber == availableDevices)
    {
      if (settings.identifyBioSensorHubs)
      {
        SerialPrintln(F(""));
        SerialPrintln(F("\"Detect Bio Sensor Pulse Oximeter\" can only be disabled by \"Reset all settings to default\""));
        SerialPrintln(F(""));
      }
      else
      {
        SerialPrintln(F(""));
        SerialPrintln(F("\"Detect Bio Sensor Pulse Oximeter\" requires exclusive use of pins 32 and 11"));
        SerialPrintln(F("Once enabled, \"Detect Bio Sensor Pulse Oximeter\" can only be disabled by \"Reset all settings to default\""));
        SerialPrintln(F("Pin 32 must be connected to the sensor RST pin"));
        SerialPrintln(F("Pin 11 must be connected to the sensor MFIO pin"));
        SerialPrintln(F("This means you cannot use pins 32 and 11 for: analog logging; triggering; fast/slow logging; stop logging; etc."));
        SerialPrintln(F("Are you sure? Press 'y' to confirm: "));
        byte bContinue = getByteChoice(menuTimeout);
        if (bContinue == 'y')
        {
          settings.identifyBioSensorHubs = true;
          settings.logA11 = false;
          settings.logA32 = false;
          if (settings.useGPIO11ForTrigger == true) // If interrupts are enabled, we need to disable and then re-enable
          {
            detachInterrupt(PIN_TRIGGER); // Disable the interrupt
            settings.useGPIO11ForTrigger = false;
          }
          settings.useGPIO11ForFastSlowLogging = false;
          if (settings.useGPIO32ForStopLogging == true)
          {
            // Disable stop logging
            settings.useGPIO32ForStopLogging = false;
            detachInterrupt(PIN_STOP_LOGGING); // Disable the interrupt
          }

          recordSystemSettings(); //Record the new settings to EEPROM and config file now in case the user resets before exiting the menus

          if (detectQwiicDevices() == true) //Detect the oximeter
          {
            beginQwiicDevices(); //Begin() each device in the node list
            configureQwiicDevices(); //Apply config settings to each device in the node list
            recordDeviceSettingsToFile(); //Record the current devices settings to device config file now in case the user resets before exiting the menus
          }

          recordSystemSettings(); //Record the new settings to EEPROM and config file now in case the user resets before exiting the menus

          if (detectQwiicDevices() == true) //Detect the oximeter
          {
            beginQwiicDevices(); //Begin() each device in the node list
            configureQwiicDevices(); //Apply config settings to each device in the node list
            recordDeviceSettingsToFile(); //Record the current devices settings to device config file now in case the user resets before exiting the menus
          }
        }
        else
          SerialPrintln(F("\"Detect Bio Sensor Pulse Oximeter\"  aborted"));
      }
    }
    else if (nodeNumber == STATUS_PRESSED_X)
      break;
    else if (nodeNumber == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(nodeNumber);
  }
}

void menuConfigure_QwiicBus()
{
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Qwiic Bus"));

    SerialPrint(F("1) Turn off bus power between readings (>2s): "));
    if (settings.powerDownQwiicBusBetweenReads == true) SerialPrintln(F("Yes"));
    else SerialPrintln(F("No"));

    SerialPrintf2("2) Set Max Qwiic Bus Speed: %d Hz\r\n", settings.qwiicBusMaxSpeed);

    SerialPrintf2("3) Set minimum Qwiic bus power up delay: %d ms\r\n", settings.qwiicBusPowerUpDelayMs);

    SerialPrint(F("4) Qwiic bus pull-ups (internal to the Artemis): "));
    if (settings.qwiicBusPullUps == 1)
      SerialPrintln(F("1.5k"));
    else if (settings.qwiicBusPullUps == 6)
      SerialPrintln(F("6k"));
    else if (settings.qwiicBusPullUps == 12)
      SerialPrintln(F("12k"));
    else if (settings.qwiicBusPullUps == 24)
      SerialPrintln(F("24k"));
    else
      SerialPrintln(F("None"));

    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      settings.powerDownQwiicBusBetweenReads ^= 1;
    else if (incoming == '2')
    {
      if (settings.qwiicBusMaxSpeed == 100000)
        settings.qwiicBusMaxSpeed = 400000;
      else
        settings.qwiicBusMaxSpeed = 100000;
    }
    else if (incoming == '3')
    {
      // 60 seconds is more than long enough for a ZED-F9P to do a warm start after being powered cycled, so that seems a sensible maximum
      // minimumQwiicPowerOnDelay is defined in settings.h
      SerialPrintf2("Enter the minimum number of milliseconds to wait for Qwiic VCC to stabilize before communication: (%d to 60000): ", minimumQwiicPowerOnDelay);
      unsigned long amt = getNumber(menuTimeout);
      if ((amt >= minimumQwiicPowerOnDelay) && (amt <= 60000))
        settings.qwiicBusPowerUpDelayMs = amt;
      else
        SerialPrintln(F("Error: Out of range"));
    }
    else if (incoming == '4')
    {
      SerialPrint(F("Enter the Artemis pull-up resistance (0 = None; 1 = 1.5k; 6 = 6k; 12 = 12k; 24 = 24k): "));
      uint32_t pur = (uint32_t)getNumber(menuTimeout);
      if ((pur == 0) || (pur == 1) || (pur == 6) || (pur == 12) || (pur == 24))
        settings.qwiicBusPullUps = pur;
      else
        SerialPrintln(F("Error: Invalid resistance. Possible values are 0,1,6,12,24."));
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_Multiplexer(void *configPtr)
{
  //struct_multiplexer *sensor = (struct_multiplexer*)configPtr;

  SerialPrintln(F(""));
  SerialPrintln(F("Menu: Configure Multiplexer"));

  SerialPrintln(F("There are currently no configurable options for this device."));
  for (int i = 0; i < 500; i++)
  {
    checkBattery();
    delay(1);
  }
}

//There is short and long range mode
//The Intermeasurement period seems to set the timing budget (PLL of the device)
//Setting the Intermeasurement period too short causes the device to freeze up
//The intermeasurement period that gets written as X gets read as X+1 so we get X and write X-1.
void menuConfigure_VL53L1X(void *configPtr)
{
  struct_VL53L1X *sensorSetting = (struct_VL53L1X*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure VL53L1X Distance Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Distance: "));
      if (sensorSetting->logDistance == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Range Status: "));
      if (sensorSetting->logRangeStatus == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Log Signal Rate: "));
      if (sensorSetting->logSignalRate == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("5) Set Distance Mode: "));
      if (sensorSetting->distanceMode == VL53L1X_DISTANCE_MODE_SHORT)
        SerialPrint(F("Short"));
      else
        SerialPrint(F("Long"));
      SerialPrintln(F(""));

      SerialPrintf2("6) Set Intermeasurement Period: %d ms\r\n", sensorSetting->intermeasurementPeriod);
      SerialPrintf2("7) Set Offset: %d mm\r\n", sensorSetting->offset);
      SerialPrintf2("8) Set Cross Talk (counts per second): %d cps\r\n", sensorSetting->crosstalk);
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logDistance ^= 1;
      else if (incoming == '3')
        sensorSetting->logRangeStatus ^= 1;
      else if (incoming == '4')
        sensorSetting->logSignalRate ^= 1;
      else if (incoming == '5')
      {
        if (sensorSetting->distanceMode == VL53L1X_DISTANCE_MODE_SHORT)
          sensorSetting->distanceMode = VL53L1X_DISTANCE_MODE_LONG;
        else
          sensorSetting->distanceMode = VL53L1X_DISTANCE_MODE_SHORT;

        //Error check
        if (sensorSetting->distanceMode == VL53L1X_DISTANCE_MODE_LONG && sensorSetting->intermeasurementPeriod < 140)
        {
          sensorSetting->intermeasurementPeriod = 140;
          SerialPrintln(F("Intermeasurement Period increased to 140ms"));
        }
      }
      else if (incoming == '6')
      {
        int min = 20;
        if (sensorSetting->distanceMode == VL53L1X_DISTANCE_MODE_LONG)
          min = 140;


        SerialPrintf2("Set timing budget (%d to 1000ms): ", min);
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt < min || amt > 1000)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->intermeasurementPeriod = amt;
      }
      else if (incoming == '7')
      {
        SerialPrint(F("Set Offset in mm (0 to 4000mm): "));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt < 0 || amt > 4000)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->offset = amt;
      }
      else if (incoming == '8')
      {
        SerialPrint(F("Set Crosstalk in Counts Per Second: "));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt < 0 || amt > 4000)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->crosstalk = amt;
      }
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }

}

void menuConfigure_BME280(void *configPtr)
{
  struct_BME280 *sensorSetting = (struct_BME280*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure BME280 Pressure/Humidity/Temperature Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Pressure: "));
      if (sensorSetting->logPressure == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Humidity: "));
      if (sensorSetting->logHumidity == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Log Altitude: "));
      if (sensorSetting->logAltitude == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("5) Log Temperature: "));
      if (sensorSetting->logTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logPressure ^= 1;
      else if (incoming == '3')
        sensorSetting->logHumidity ^= 1;
      else if (incoming == '4')
        sensorSetting->logAltitude ^= 1;
      else if (incoming == '5')
        sensorSetting->logTemperature ^= 1;
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_CCS811(void *configPtr)
{
  struct_CCS811 *sensorSetting = (struct_CCS811*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure CCS811 tVOC and CO2 Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log tVOC: "));
      if (sensorSetting->logTVOC == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log CO2: "));
      if (sensorSetting->logCO2 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logTVOC ^= 1;
      else if (incoming == '3')
        sensorSetting->logCO2 ^= 1;
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_LPS25HB(void *configPtr)
{
  struct_LPS25HB *sensorSetting = (struct_LPS25HB*)configPtr;
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure LPS25HB Pressure Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Pressure: "));
      if (sensorSetting->logPressure == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Temperature: "));
      if (sensorSetting->logTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logPressure ^= 1;
      else if (incoming == '3')
        sensorSetting->logTemperature ^= 1;
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }

}

void menuConfigure_LPS28DFW(void *configPtr)
{
  struct_LPS28DFW *sensorSetting = (struct_LPS28DFW*)configPtr;
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure LPS28DFW Pressure Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Pressure: "));
      if (sensorSetting->logPressure == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Temperature: "));
      if (sensorSetting->logTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logPressure ^= 1;
      else if (incoming == '3')
        sensorSetting->logTemperature ^= 1;
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }

}

void menuConfigure_NAU7802(void *configPtr)
{
  //Search the list of nodes looking for the one with matching config pointer
  node *temp = head;
  while (temp != NULL)
  {
    if (temp->configPtr == configPtr)
      break;

    temp = temp->next;
  }
  if (temp == NULL)
  {
    SerialPrintln(F("NAU7802 node not found. Returning."));
    for (int i = 0; i < 1000; i++)
    {
      checkBattery();
      delay(1);
    }
    return;
  }

  NAU7802 *sensor = (NAU7802 *)temp->classPtr;
  struct_NAU7802 *sensorConfig = (struct_NAU7802*)configPtr;

  openConnection(temp->muxAddress, temp->portNumber); //Connect to this device through muxes as needed

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure NAU7802 Load Cell Amplifier"));

    if (sensorConfig->log == true)
    {
      char tempStr[16];
      olaftoa(sensorConfig->calibrationFactor, tempStr, 6, sizeof(tempStr) / sizeof(char));
      SerialPrintf2("\r\nScale calibration factor: %s\r\n", tempStr);

      SerialPrintf2("Scale zero offset: %d\r\n", sensorConfig->zeroOffset);
      SerialPrintf2("Scale offset register: %d\r\n", sensor->get24BitRegister(NAU7802_OCAL1_B2));

      sensor->getWeight(true, 10); //Flush
      olaftoa(sensor->getWeight(true, sensorConfig->averageAmount), tempStr, sensorConfig->decimalPlaces, sizeof(tempStr) / sizeof(char));
      SerialPrintf2("Weight currently on scale: %s\r\n\r\n", tempStr);
    }

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorConfig->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorConfig->log == true)
    {
      SerialPrintln(F("2) Zero scale"));
      char tempStr[16];
      olaftoa(sensorConfig->calibrationWeight, tempStr, 6, sizeof(tempStr) / sizeof(char));
      SerialPrintln(F("3) Calibrate scale"));
      SerialPrintf2("4) Calibration weight: %s\r\n", tempStr);
      SerialPrintf2("5) Number of decimal places: %d\r\n", sensorConfig->decimalPlaces);
      SerialPrintf2("6) Average number of readings to take per weight read: %d\r\n", sensorConfig->averageAmount);
      int gain;
      switch (sensorConfig->gain)
      {
        case 0:
          gain = 1;
          break;
        case 1:
          gain = 2;
          break;
        case 2:
          gain = 4;
          break;
        case 3:
          gain = 8;
          break;
        case 4:
          gain = 16;
          break;
        case 5:
          gain = 32;
          break;
        case 6:
          gain = 64;
          break;
        case 7:
          gain = 128;
          break;
      }
      SerialPrintf2("7) Gain: %d\r\n", gain);
      int rate;
      switch (sensorConfig->sampleRate)
      {
        case 0:
          rate = 10;
          break;
        case 1:
          rate = 20;
          break;
        case 2:
          rate = 40;
          break;
        case 3:
          rate = 80;
          break;
        case 7:
          rate = 320;
          break;
      }
      SerialPrintf2("8) Sample rate: %d\r\n", rate);
      float LDO;
      switch (sensorConfig->LDO)
      {
        case 4:
          LDO = 3.3;
          break;
        case 5:
          LDO = 3.0;
          break;
        case 6:
          LDO = 2.7;
          break;
        case 7:
          LDO = 2.4;
          break;
      }
      olaftoa(LDO, tempStr, 1, sizeof(tempStr) / sizeof(char));
      SerialPrintf2("9) LDO voltage: %s\r\n", tempStr);
      SerialPrint(F("10) Calibration mode: "));
      if (sensorConfig->calibrationMode == 0) SerialPrintln(F("None"));
      else if (sensorConfig->calibrationMode == 1) SerialPrintln(F("Internal"));
      else SerialPrintln(F("External"));
    }

    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after 10 seconds

    if (incoming == 1)
    {
      sensorConfig->log ^= 1;
    }
    else if (sensorConfig->log == true)
    {
      if (incoming == 2)
      {
        //Gives user the ability to set a known weight on the scale and calculate a calibration factor
        SerialPrintln(F(""));
        SerialPrintln(F("Zero scale"));

        SerialPrintln(F("Setup scale with no weight on it. Press a key when ready."));
        waitForInput();

        sensor->getWeight(true, 10); //Flush

        if (sensorConfig->calibrationMode == 2) //External calibration
        {
          sensor->calibrateAFE(NAU7802_CALMOD_OFFSET); //External offset calibration

          sensorConfig->offsetReg = sensor->get24BitRegister(NAU7802_OCAL1_B2); // Save new offset
          sensorConfig->gainReg = sensor->get32BitRegister(NAU7802_GCAL1_B3); // This should not have changed, but read it anyway

          sensor->getWeight(true, 10); //Flush
        }

        sensor->calculateZeroOffset(sensorConfig->averageAmount); //Zero or Tare the scale. With external calibration, this should be ~zero

        sensorConfig->zeroOffset = sensor->getZeroOffset();
      }
      else if (incoming == 3)
      {
        //Gives user the ability to set a known weight on the scale and calculate a calibration factor
        SerialPrintln(F(""));
        SerialPrintln(F("Scale calibration"));

        SerialPrintln(F("Place calibration weight on scale. Press a key when weight is in place and stable."));
        waitForInput();

        sensor->getWeight(true, 10); //Flush

        sensor->calculateCalibrationFactor(sensorConfig->calibrationWeight, sensorConfig->averageAmount); //Tell the library how much weight is currently on it

        sensorConfig->calibrationFactor = sensor->getCalibrationFactor();
      }
      else if (incoming == 4)
      {
        SerialPrint(F("Please enter the weight, without units, for scale calibration (3) - (for example '100.0'): "));

        //Read user input
        double newWeight = getDouble(menuTimeout); //Timeout after x seconds
        if ((newWeight != STATUS_GETNUMBER_TIMEOUT) && (newWeight != STATUS_PRESSED_X))
          sensorConfig->calibrationWeight = (float)newWeight;

        SerialPrintln(F(""));
      }
      else if (incoming == 5)
      {
        SerialPrint(F("Enter number of decimal places to print (1 to 10): "));
        int places = getNumber(menuTimeout);
        if (places < 1 || places > 10)
        {
          SerialPrintln(F("Error: Decimal places out of range"));
        }
        else
        {
          sensorConfig->decimalPlaces = places;
        }
      }
      else if (incoming == 6)
      {
        //Limit number of readings to the sample rate so that the getWeight doesn't time out
        SerialPrint(F("Enter number of readings to take per weight read (>= 1, < Sample Rate): "));
        int rate;
        switch (sensorConfig->sampleRate)
        {
          case 0:
            rate = 10;
            break;
          case 1:
            rate = 20;
            break;
          case 2:
            rate = 40;
            break;
          case 3:
            rate = 80;
            break;
          case 7:
            rate = 320;
            break;
        }
        int amt = getNumber(menuTimeout);
        if (amt < 1 || amt >= rate)
        {
          SerialPrintln(F("Error: Average number of readings out of range"));
        }
        else
        {
          sensorConfig->averageAmount = amt;
        }
      }
      else if (incoming == 7)
      {
        sensorConfig->gain += 1;
        if (sensorConfig->gain == 8)
          sensorConfig->gain = 0;

        sensor->setGain(sensorConfig->gain);

        if (sensorConfig->calibrationMode == 1) //Internal calibration
        {
          sensor->getWeight(true, 10); //Flush

          sensor->calibrateAFE(NAU7802_CALMOD_INTERNAL); //Recalibrate after changing gain / sample rate          
        }

        SerialPrintln(F("\r\n\r\nGain updated. Please zero and calibrate the scale\r\n\r\n"));
      }
      else if (incoming == 8)
      {
        sensorConfig->sampleRate += 1;
        if (sensorConfig->sampleRate == 4)
          sensorConfig->sampleRate = 7;
        if (sensorConfig->sampleRate == 8)
          sensorConfig->sampleRate = 0;

        sensor->setSampleRate(sensorConfig->sampleRate);

        if (sensorConfig->calibrationMode == 1) //Internal calibration
        {
          sensor->getWeight(true, 10); //Flush

          sensor->calibrateAFE(NAU7802_CALMOD_INTERNAL); //Recalibrate after changing gain / sample rate          
        }

        // Limit averageAmount (to prevent getWeight timing out after 1s)
        if ((sensorConfig->sampleRate) == 0 && (sensorConfig->averageAmount > 9))
          sensorConfig->averageAmount = 9;
        else if ((sensorConfig->sampleRate) == 1 && (sensorConfig->averageAmount > 19))
          sensorConfig->averageAmount = 19;
        else if ((sensorConfig->sampleRate) == 2 && (sensorConfig->averageAmount > 39))
          sensorConfig->averageAmount = 39;
        else if ((sensorConfig->sampleRate) == 3 && (sensorConfig->averageAmount > 79))
          sensorConfig->averageAmount = 79;
        else if (sensorConfig->averageAmount > 319)
          sensorConfig->averageAmount = 319;

        SerialPrintln(F("\r\n\r\nSample rate updated. Please zero and calibrate the scale\r\n\r\n"));
      }
      else if (incoming == 9)
      {
        sensorConfig->LDO += 1;
        if (sensorConfig->LDO == 8)
          sensorConfig->LDO = 4;

        sensor->setLDO(sensorConfig->LDO);

        if (sensorConfig->calibrationMode == 1) //Internal calibration
        {
          delay(sensor->getLDORampDelay()); // Wait for LDO to ramp before attempting calibrateAFE

          sensor->getWeight(true, 10); //Flush

          sensor->calibrateAFE(NAU7802_CALMOD_INTERNAL); //Recalibrate after changing gain / sample rate          
        }

        SerialPrintln(F("\r\n\r\nLDO updated. Please zero and calibrate the scale\r\n\r\n"));
      }
      else if (incoming == 10)
      {
        sensorConfig->calibrationMode += 1;
        if (sensorConfig->calibrationMode == 3)
          sensorConfig->calibrationMode = 0;

        sensor->reset();
        sensor->powerUp();
        sensor->setLDO(sensorConfig->LDO);
        sensor->setGain(sensorConfig->gain);
        sensor->setSampleRate(sensorConfig->sampleRate);
        //Turn off CLK_CHP. From 9.1 power on sequencing.
        uint8_t adc = sensor->getRegister(NAU7802_ADC);
        adc |= 0x30;
        sensor->setRegister(NAU7802_ADC, adc);
        sensor->setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR); //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
        sensor->clearBit(NAU7802_PGA_LDOMODE, NAU7802_PGA); //Ensure LDOMODE bit is clear - improved accuracy and higher DC gain, with ESR < 1 ohm
        sensor->setCalibrationFactor(sensorConfig->calibrationFactor);
        sensor->setZeroOffset(sensorConfig->zeroOffset);

        delay(sensor->getLDORampDelay()); // Wait for LDO to ramp before attempting calibrateAFE

        if (sensorConfig->calibrationMode == 1) //Internal calibration
        {
          sensor->getWeight(true, 10); //Flush

          sensor->calibrateAFE(NAU7802_CALMOD_INTERNAL); //Recalibrate after changing gain / sample rate          
        }

        sensor->getWeight(true, 10); //Flush

        SerialPrintln(F("\r\n\r\nCalibration updated. Please zero and calibrate the scale\r\n\r\n"));
      }
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_ublox(void *configPtr)
{
  struct_ublox *sensorSetting = (struct_ublox*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure u-blox GPS Receiver"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log GPS Date: "));
      if (sensorSetting->logDate == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log GPS Time: "));
      if (sensorSetting->logTime == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Log Longitude/Latitude: "));
      if (sensorSetting->logPosition == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("5) Log Altitude: "));
      if (sensorSetting->logAltitude == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("6) Log Altitude Mean Sea Level: "));
      if (sensorSetting->logAltitudeMSL == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("7) Log Satellites In View: "));
      if (sensorSetting->logSIV == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("8) Log Fix Type: "));
      if (sensorSetting->logFixType == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("9) Log Carrier Solution: "));
      if (sensorSetting->logCarrierSolution == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("10) Log Ground Speed: "));
      if (sensorSetting->logGroundSpeed == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("11) Log Heading of Motion: "));
      if (sensorSetting->logHeadingOfMotion == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("12) Log Position Dilution of Precision (pDOP): "));
      if (sensorSetting->logpDOP == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("13) Log Interval Time Of Week (iTOW): "));
      if (sensorSetting->logiTOW == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrintf2("14) Set I2C Interface Speed (u-blox modules have pullups built in. Remove *all* I2C pullups to achieve 400kHz): %d\r\n", sensorSetting->i2cSpeed);

      SerialPrint(F("15) Use autoPVT: "));
      if (sensorSetting->useAutoPVT == true) SerialPrintln(F("Yes"));
      else SerialPrintln(F("No"));

      SerialPrintln(F("16) Reset GNSS to factory defaults"));

      SerialFlush();
    }
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after 10 seconds

    if (incoming == 1)
    {
      sensorSetting->log ^= 1;
    }
    else if (sensorSetting->log == true)
    {
      if (incoming == 2)
        sensorSetting->logDate ^= 1;
      else if (incoming == 3)
        sensorSetting->logTime ^= 1;
      else if (incoming == 4)
        sensorSetting->logPosition ^= 1;
      else if (incoming == 5)
        sensorSetting->logAltitude ^= 1;
      else if (incoming == 6)
        sensorSetting->logAltitudeMSL ^= 1;
      else if (incoming == 7)
        sensorSetting->logSIV ^= 1;
      else if (incoming == 8)
        sensorSetting->logFixType ^= 1;
      else if (incoming == 9)
        sensorSetting->logCarrierSolution ^= 1;
      else if (incoming == 10)
        sensorSetting->logGroundSpeed ^= 1;
      else if (incoming == 11)
        sensorSetting->logHeadingOfMotion ^= 1;
      else if (incoming == 12)
        sensorSetting->logpDOP ^= 1;
      else if (incoming == 13)
        sensorSetting->logiTOW ^= 1;
      else if (incoming == 14)
      {
        if (sensorSetting->i2cSpeed == 100000)
          sensorSetting->i2cSpeed = 400000;
        else
          sensorSetting->i2cSpeed = 100000;
      }
      else if (incoming == 15)
        sensorSetting->useAutoPVT ^= 1;
      else if (incoming == 16)
      {
        SerialPrintln(F("Reset GNSS module to factory defaults. This will take 5 seconds to complete."));
        SerialPrintln(F("Are you sure? Press 'y' to confirm: "));
        byte bContinue = getByteChoice(menuTimeout);
        if (bContinue == 'y')
        {
          gnssFactoryDefault();
        }
        else
          SerialPrintln(F("Reset GNSS aborted"));
      }
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

bool isUbloxAttached()
{
  //Step through node list
  node *temp = head;

  while (temp != NULL)
  {
    switch (temp->deviceType)
    {
      case DEVICE_GPS_UBLOX:
        return (true);
    }
    temp = temp->next;
  }

  return (false);
}

void getUbloxDateTime(int &year, int &month, int &day, int &hour, int &minute, int &second, int &millisecond, bool &dateValid, bool &timeValid)
{
  //Step through node list
  node *temp = head;

  while (temp != NULL)
  {
    switch (temp->deviceType)
    {
      case DEVICE_GPS_UBLOX:
        {
          openConnection(temp->muxAddress, temp->portNumber); //Connect to this device through muxes as needed

          setQwiicPullups(0); //Disable pullups to minimize CRC issues

          SFE_UBLOX_GNSS *nodeDevice = (SFE_UBLOX_GNSS *)temp->classPtr;
          struct_ublox *nodeSetting = (struct_ublox *)temp->configPtr;

          //If autoPVT is enabled, flush the data to make sure we get fresh date and time
          if (nodeSetting->useAutoPVT) nodeDevice->flushPVT();

          //Get latested date/time from GPS
          //These will be extracted from a single PVT packet
          year = nodeDevice->getYear();
          month = nodeDevice->getMonth();
          day = nodeDevice->getDay();
          hour = nodeDevice->getHour();
          minute = nodeDevice->getMinute();
          second = nodeDevice->getSecond();
          dateValid = nodeDevice->getDateValid();
          timeValid = nodeDevice->getTimeValid();
          millisecond = nodeDevice->getMillisecond();

          setQwiicPullups(settings.qwiicBusPullUps); //Re-enable pullups
        }
    }
    temp = temp->next;
  }
}

void gnssFactoryDefault(void)
{
  //Step through node list
  node *temp = head;

  while (temp != NULL)
  {
    switch (temp->deviceType)
    {
      case DEVICE_GPS_UBLOX:
        {
          openConnection(temp->muxAddress, temp->portNumber); //Connect to this device through muxes as needed
          
          setQwiicPullups(0); //Disable pullups to minimize CRC issues

          SFE_UBLOX_GNSS *nodeDevice = (SFE_UBLOX_GNSS *)temp->classPtr;
          struct_ublox *nodeSetting = (struct_ublox *)temp->configPtr;

          //Reset the module to the factory defaults
          nodeDevice->factoryDefault();

          delay(5000); //Blocking delay to allow module to reset

          nodeDevice->setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
          nodeDevice->saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the current ioPortsettings to flash and BBR

          setQwiicPullups(settings.qwiicBusPullUps); //Re-enable pullups
        }
    }
    temp = temp->next;
  }
}

void menuConfigure_MCP9600(void *configPtr)
{
  struct_MCP9600 *sensorSetting = (struct_MCP9600*)configPtr;
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure MCP9600 Thermocouple Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Thermocouple Temperature: "));
      if (sensorSetting->logTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Ambient Temperature: "));
      if (sensorSetting->logAmbientTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logTemperature ^= 1;
      else if (incoming == '3')
        sensorSetting->logAmbientTemperature ^= 1;
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }

}

void menuConfigure_VCNL4040(void *configPtr)
{
  struct_VCNL4040 *sensorSetting = (struct_VCNL4040*)configPtr;
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure VCNL4040 Proximity Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Proximity: "));
      if (sensorSetting->logProximity == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Ambient Light: "));
      if (sensorSetting->logAmbientLight == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrintf2("4) Set LED Current: %d\r\n", sensorSetting->LEDCurrent);
      SerialPrintf2("5) Set IR Duty Cycle: %d\r\n", sensorSetting->IRDutyCycle);
      SerialPrintf2("6) Set Proximity Integration Time: %d\r\n", sensorSetting->proximityIntegrationTime);
      SerialPrintf2("7) Set Ambient Integration Time: %d\r\n", sensorSetting->ambientIntegrationTime);
      SerialPrintf2("8) Set Resolution (bits): %d\r\n", sensorSetting->resolution);
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logProximity ^= 1;
      else if (incoming == '3')
        sensorSetting->logAmbientLight ^= 1;
      else if (incoming == '4')
      {
        SerialPrint(F("Enter current (mA) for IR LED drive (50 to 200mA): "));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt < 50 || amt > 200)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->LEDCurrent = amt;
      }
      else if (incoming == '5')
      {
        SerialPrint(F("Enter IR Duty Cycle (40 to 320): "));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt < 40 || amt > 320)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->IRDutyCycle = amt;
      }
      else if (incoming == '6')
      {
        SerialPrint(F("Enter Proximity Integration Time (1 to 8): "));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt < 1 || amt > 8)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->proximityIntegrationTime = amt;
      }
      else if (incoming == '7')
      {
        SerialPrint(F("Enter Ambient Light Integration Time (80 to 640ms): "));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt < 80 || amt > 640)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->ambientIntegrationTime = amt;
      }
      else if (incoming == '8')
      {
        SerialPrint(F("Enter Proximity Resolution (12 or 16 bit): "));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt == 12 || amt == 16)
          sensorSetting->resolution = amt;
        else
          SerialPrintln(F("Error: Out of range"));
      }
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }

}

void menuConfigure_TMP117(void *configPtr)
{
  struct_TMP117 *sensorSetting = (struct_TMP117*)configPtr;
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure TMP117 Precision Temperature Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }

}


void menuConfigure_SGP30(void *configPtr)
{
  struct_SGP30 *sensorSetting = (struct_SGP30*)configPtr;
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure SGP30 tVOC and CO2 Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log tVOC: "));
      if (sensorSetting->logTVOC == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log CO2: "));
      if (sensorSetting->logCO2 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Log H2: "));
      if (sensorSetting->logH2 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("5) Log Ethanol: "));
      if (sensorSetting->logEthanol == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logTVOC ^= 1;
      else if (incoming == '3')
        sensorSetting->logCO2 ^= 1;
      else if (incoming == '4')
        sensorSetting->logH2 ^= 1;
      else if (incoming == '5')
        sensorSetting->logEthanol ^= 1;
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_VEML6075(void *configPtr)
{
  struct_VEML6075 *sensorSetting = (struct_VEML6075*)configPtr;
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure VEML6075 UV Index Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log UVA: "));
      if (sensorSetting->logUVA == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log UVB: "));
      if (sensorSetting->logUVB == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Log UV Index: "));
      if (sensorSetting->logUVIndex == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logUVA ^= 1;
      else if (incoming == '3')
        sensorSetting->logUVB ^= 1;
      else if (incoming == '4')
        sensorSetting->logUVIndex ^= 1;
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }

}

void menuConfigure_VEML7700(void *configPtr)
{
  struct_VEML7700 *sensorSetting = (struct_VEML7700*)configPtr;
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure VEML7700 Ambient Light Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }

}

void menuConfigure_MS5637(void *configPtr)
{
  struct_MS5637 *sensorSetting = (struct_MS5637*)configPtr;
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure MS5637 Pressure Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Pressure: "));
      if (sensorSetting->logPressure == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Temperature: "));
      if (sensorSetting->logTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logPressure ^= 1;
      else if (incoming == '3')
        sensorSetting->logTemperature ^= 1;
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }

}


void menuConfigure_SCD30(void *configPtr)
{
  //Search the list of nodes looking for the one with matching config pointer
  node *temp = head;
  while (temp != NULL)
  {
    if (temp->configPtr == configPtr)
      break;

    temp = temp->next;
  }
  if (temp == NULL)
  {
    SerialPrintln(F("SCD30 node not found. Returning."));
    for (int i = 0; i < 1000; i++)
    {
      checkBattery();
      delay(1);
    }
    return;
  }

  SCD30 *sensor = (SCD30 *)temp->classPtr;
  struct_SCD30 *sensorSetting = (struct_SCD30*)configPtr;

  openConnection(temp->muxAddress, temp->portNumber); //Connect to this device through muxes as needed

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure SCD30 CO2 and Humidity Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log CO2: "));
      if (sensorSetting->logCO2 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Humidity: "));
      if (sensorSetting->logHumidity == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Log Temperature: "));
      if (sensorSetting->logTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrintf2("5) Set Measurement Interval: %d\r\n", sensorSetting->measurementInterval);
      SerialPrintf2("6) Set Altitude Compensation: %d\r\n", sensorSetting->altitudeCompensation);
      SerialPrintf2("7) Set Ambient Pressure: %d\r\n", sensorSetting->ambientPressure);
      SerialPrintf2("8) Set Temperature Offset: %d\r\n", sensorSetting->temperatureOffset);
      SerialPrintln(F("9) Set FRC Calibration CO2"));
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logCO2 ^= 1;
      else if (incoming == '3')
        sensorSetting->logHumidity ^= 1;
      else if (incoming == '4')
        sensorSetting->logTemperature ^= 1;
      else if (incoming == '5')
      {
        SerialPrint(F("Enter the seconds between measurements (2 to 1800): "));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt < 2 || amt > 1800)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->measurementInterval = amt;
      }
      else if (incoming == '6')
      {
        SerialPrint(F("Enter the Altitude Compensation in meters (0 to 10000): "));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt < 0 || amt > 10000)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->altitudeCompensation = amt;
      }
      else if (incoming == '7')
      {
        SerialPrint(F("Enter Ambient Pressure in mBar (700 to 1200): "));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt < 700 || amt > 1200)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->ambientPressure = amt;
      }
      else if (incoming == '8')
      {
        SerialPrint(F("The current temperature offset read from the sensor is: "));
        Serial.print(sensor->getTemperatureOffset(), 2);
        if (settings.useTxRxPinsForTerminal == true)
          Serial1.print(sensor->getTemperatureOffset(), 2);
        SerialPrintln(F("C"));
        SerialPrint(F("Enter new temperature offset in C (-50 to 50): "));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt < -50 || amt > 50)
          sensorSetting->temperatureOffset = amt;
        else
          SerialPrintln(F("Error: Out of range"));
      }
      else if (incoming == '9')
      {
        SerialPrint(F("Enter Calibration CO2 in ppm (400 to 2000): "));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt < 400 || amt > 2000)
          SerialPrintln(F("Error: Out of range"));
        else
        {
          sensorSetting->calibrationConcentration = amt;
          sensorSetting->applyCalibrationConcentration = true;
        }
      }
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }

}


void menuConfigure_MS8607(void *configPtr)
{
  struct_MS8607 *sensorSetting = (struct_MS8607*)configPtr;
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure MS8607 Pressure Humidity Temperature (PHT) Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Pressure: "));
      if (sensorSetting->logPressure == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Humidity: "));
      if (sensorSetting->logHumidity == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Log Temperature: "));
      if (sensorSetting->logTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("5) Heater: "));
      if (sensorSetting->enableHeater == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("6) Set Pressure Resolution: "));
      if (sensorSetting->pressureResolution == MS8607_pressure_resolution_osr_256)
        SerialPrint(F("0.11"));
      else if (sensorSetting->pressureResolution == MS8607_pressure_resolution_osr_512)
        SerialPrint(F("0.062"));
      else if (sensorSetting->pressureResolution == MS8607_pressure_resolution_osr_1024)
        SerialPrint(F("0.039"));
      else if (sensorSetting->pressureResolution == MS8607_pressure_resolution_osr_2048)
        SerialPrint(F("0.028"));
      else if (sensorSetting->pressureResolution == MS8607_pressure_resolution_osr_4096)
        SerialPrint(F("0.021"));
      else if (sensorSetting->pressureResolution == MS8607_pressure_resolution_osr_8192)
        SerialPrint(F("0.016"));
      SerialPrintln(F(" mbar"));

      SerialPrint(F("7) Set Humidity Resolution: "));
      if (sensorSetting->humidityResolution == MS8607_humidity_resolution_8b)
        SerialPrint(F("8"));
      else if (sensorSetting->humidityResolution == MS8607_humidity_resolution_10b)
        SerialPrint(F("10"));
      else if (sensorSetting->humidityResolution == MS8607_humidity_resolution_11b)
        SerialPrint(F("11"));
      else if (sensorSetting->humidityResolution == MS8607_humidity_resolution_12b)
        SerialPrint(F("12"));
      SerialPrintln(F(" bits"));
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logPressure ^= 1;
      else if (incoming == '3')
        sensorSetting->logHumidity ^= 1;
      else if (incoming == '4')
        sensorSetting->logTemperature ^= 1;
      else if (incoming == '5')
        sensorSetting->enableHeater ^= 1;
      else if (incoming == '6')
      {
        SerialPrintln(F("Set Pressure Resolution:"));
        SerialPrintln(F("1) 0.11 mbar"));
        SerialPrintln(F("2) 0.062 mbar"));
        SerialPrintln(F("3) 0.039 mbar"));
        SerialPrintln(F("4) 0.028 mbar"));
        SerialPrintln(F("5) 0.021 mbar"));
        SerialPrintln(F("6) 0.016 mbar"));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt >= 1 && amt <= 6)
          sensorSetting->pressureResolution = (MS8607_pressure_resolution)(amt - 1);
        else
          SerialPrintln(F("Error: Out of range"));
      }
      else if (incoming == '7')
      {
        SerialPrintln(F("Set Humidity Resolution:"));
        SerialPrintln(F("1) 8 bit"));
        SerialPrintln(F("2) 10 bit"));
        SerialPrintln(F("3) 11 bit"));
        SerialPrintln(F("4) 12 bit"));
        int amt = getNumber(menuTimeout); //x second timeout
        if (amt >= 1 && amt <= 4)
        {
          //Unfortunately these enums aren't sequential so we have to lookup
          if (amt == 1) sensorSetting->humidityResolution = MS8607_humidity_resolution_8b;
          if (amt == 2) sensorSetting->humidityResolution = MS8607_humidity_resolution_10b;
          if (amt == 3) sensorSetting->humidityResolution = MS8607_humidity_resolution_11b;
          if (amt == 4) sensorSetting->humidityResolution = MS8607_humidity_resolution_12b;
        }
        else
          SerialPrintln(F("Error: Out of range"));
      }
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_AHT20(void *configPtr)
{
  struct_AHT20 *sensorSetting = (struct_AHT20*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure AHT20 Humidity Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Humidity: "));
      if (sensorSetting->logHumidity == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Temperature: "));
      if (sensorSetting->logTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logHumidity ^= 1;
      else if (incoming == '3')
        sensorSetting->logTemperature ^= 1;
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_SHTC3(void *configPtr)
{
  struct_SHTC3 *sensorSetting = (struct_SHTC3*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure SHTC3 Humidity Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Humidity: "));
      if (sensorSetting->logHumidity == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Temperature: "));
      if (sensorSetting->logTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == '2')
        sensorSetting->logHumidity ^= 1;
      else if (incoming == '3')
        sensorSetting->logTemperature ^= 1;
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_ADS122C04(void *configPtr)
{
  struct_ADS122C04 *sensorSetting = (struct_ADS122C04*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure ADS122C04 ADC (Qwiic PT100)"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Centigrade: "));
      if (sensorSetting->logCentigrade == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Fahrenheit: "));
      if (sensorSetting->logFahrenheit == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Log Internal Temperature: "));
      if (sensorSetting->logInternalTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("5) Log Raw Voltage: "));
      if (sensorSetting->logRawVoltage == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("6) Use 4-Wire Mode: "));
      if (sensorSetting->useFourWireMode == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("7) Use 3-Wire Mode: "));
      if (sensorSetting->useThreeWireMode == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("8) Use 2-Wire Mode: "));
      if (sensorSetting->useTwoWireMode == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("9) Use 4-Wire High Temperature Mode: "));
      if (sensorSetting->useFourWireHighTemperatureMode == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("10) Use 3-Wire High Temperature Mode: "));
      if (sensorSetting->useThreeWireHighTemperatureMode == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("11) Use 2-Wire High Temperature Mode: "));
      if (sensorSetting->useTwoWireHighTemperatureMode == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == 2)
        sensorSetting->logCentigrade ^= 1;
      else if (incoming == 3)
        sensorSetting->logFahrenheit ^= 1;
      else if (incoming == 4)
        sensorSetting->logInternalTemperature ^= 1;
      else if (incoming == 5)
        sensorSetting->logRawVoltage ^= 1;
      else if (incoming == 6)
      {
        sensorSetting->useFourWireMode = true;
        sensorSetting->useThreeWireMode = false;
        sensorSetting->useTwoWireMode = false;
        sensorSetting->useFourWireHighTemperatureMode = false;
        sensorSetting->useThreeWireHighTemperatureMode = false;
        sensorSetting->useTwoWireHighTemperatureMode = false;
      }
      else if (incoming == 7)
      {
        sensorSetting->useFourWireMode = false;
        sensorSetting->useThreeWireMode = true;
        sensorSetting->useTwoWireMode = false;
        sensorSetting->useFourWireHighTemperatureMode = false;
        sensorSetting->useThreeWireHighTemperatureMode = false;
        sensorSetting->useTwoWireHighTemperatureMode = false;
      }
      else if (incoming == 8)
      {
        sensorSetting->useFourWireMode = false;
        sensorSetting->useThreeWireMode = false;
        sensorSetting->useTwoWireMode = true;
        sensorSetting->useFourWireHighTemperatureMode = false;
        sensorSetting->useThreeWireHighTemperatureMode = false;
        sensorSetting->useTwoWireHighTemperatureMode = false;
      }
      else if (incoming == 9)
      {
        sensorSetting->useFourWireMode = false;
        sensorSetting->useThreeWireMode = false;
        sensorSetting->useTwoWireMode = false;
        sensorSetting->useFourWireHighTemperatureMode = true;
        sensorSetting->useThreeWireHighTemperatureMode = false;
        sensorSetting->useTwoWireHighTemperatureMode = false;
      }
      else if (incoming == 10)
      {
        sensorSetting->useFourWireMode = false;
        sensorSetting->useThreeWireMode = false;
        sensorSetting->useTwoWireMode = false;
        sensorSetting->useFourWireHighTemperatureMode = false;
        sensorSetting->useThreeWireHighTemperatureMode = true;
        sensorSetting->useTwoWireHighTemperatureMode = false;
      }
      else if (incoming == 11)
      {
        sensorSetting->useFourWireMode = false;
        sensorSetting->useThreeWireMode = false;
        sensorSetting->useTwoWireMode = false;
        sensorSetting->useFourWireHighTemperatureMode = false;
        sensorSetting->useThreeWireHighTemperatureMode = false;
        sensorSetting->useTwoWireHighTemperatureMode = true;
      }
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_MPR0025PA1(void *configPtr)
{
  struct_MPR0025PA1 *sensorSetting = (struct_MPR0025PA1*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure MPR MicroPressure Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrintf2("2) Minimum PSI: %d\r\n", sensorSetting->minimumPSI);

      SerialPrintf2("3) Maximum PSI: %d\r\n", sensorSetting->maximumPSI);

      SerialPrint(F("4) Use PSI: "));
      if (sensorSetting->usePSI == true) SerialPrintln(F("Yes"));
      else SerialPrintln(F("No"));

      SerialPrint(F("5) Use Pa: "));
      if (sensorSetting->usePA == true) SerialPrintln(F("Yes"));
      else SerialPrintln(F("No"));

      SerialPrint(F("6) Use kPa: "));
      if (sensorSetting->useKPA == true) SerialPrintln(F("Yes"));
      else SerialPrintln(F("No"));

      SerialPrint(F("7) Use torr: "));
      if (sensorSetting->useTORR == true) SerialPrintln(F("Yes"));
      else SerialPrintln(F("No"));

      SerialPrint(F("8) Use inHg: "));
      if (sensorSetting->useINHG == true) SerialPrintln(F("Yes"));
      else SerialPrintln(F("No"));

      SerialPrint(F("9) Use atm: "));
      if (sensorSetting->useATM == true) SerialPrintln(F("Yes"));
      else SerialPrintln(F("No"));

      SerialPrint(F("10) Use bar: "));
      if (sensorSetting->useBAR == true) SerialPrintln(F("Yes"));
      else SerialPrintln(F("No"));

    }
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == 2)
      {
        SerialPrint(F("Enter the sensor minimum pressure in PSI (this should be 0 for the MPR0025PA): "));
        int minPSI = getNumber(menuTimeout); //x second timeout
        if (minPSI < 0 || minPSI > 30)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->minimumPSI = minPSI;
      }
      else if (incoming == 3)
      {
        SerialPrint(F("Enter the sensor maximum pressure in PSI (this should be 25 for the MPR0025PA): "));
        int maxPSI = getNumber(menuTimeout); //x second timeout
        if (maxPSI < 0 || maxPSI > 30)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->maximumPSI = maxPSI;
      }
      else if (incoming == 4)
      {
        sensorSetting->usePSI = true;
        sensorSetting->usePA = false;
        sensorSetting->useKPA = false;
        sensorSetting->useTORR = false;
        sensorSetting->useINHG = false;
        sensorSetting->useATM = false;
        sensorSetting->useBAR = false;
      }
      else if (incoming == 5)
      {
        sensorSetting->usePSI = false;
        sensorSetting->usePA = true;
        sensorSetting->useKPA = false;
        sensorSetting->useTORR = false;
        sensorSetting->useINHG = false;
        sensorSetting->useATM = false;
        sensorSetting->useBAR = false;
      }
      else if (incoming == 6)
      {
        sensorSetting->usePSI = false;
        sensorSetting->usePA = false;
        sensorSetting->useKPA = true;
        sensorSetting->useTORR = false;
        sensorSetting->useINHG = false;
        sensorSetting->useATM = false;
        sensorSetting->useBAR = false;
      }
      else if (incoming == 7)
      {
        sensorSetting->usePSI = false;
        sensorSetting->usePA = false;
        sensorSetting->useKPA = false;
        sensorSetting->useTORR = true;
        sensorSetting->useINHG = false;
        sensorSetting->useATM = false;
        sensorSetting->useBAR = false;
      }
      else if (incoming == 8)
      {
        sensorSetting->usePSI = false;
        sensorSetting->usePA = false;
        sensorSetting->useKPA = false;
        sensorSetting->useTORR = false;
        sensorSetting->useINHG = true;
        sensorSetting->useATM = false;
        sensorSetting->useBAR = false;
      }
      else if (incoming == 9)
      {
        sensorSetting->usePSI = false;
        sensorSetting->usePA = false;
        sensorSetting->useKPA = false;
        sensorSetting->useTORR = false;
        sensorSetting->useINHG = false;
        sensorSetting->useATM = true;
        sensorSetting->useBAR = false;
      }
      else if (incoming == 10)
      {
        sensorSetting->usePSI = false;
        sensorSetting->usePA = false;
        sensorSetting->useKPA = false;
        sensorSetting->useTORR = false;
        sensorSetting->useINHG = false;
        sensorSetting->useATM = false;
        sensorSetting->useBAR = true;
      }
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_SNGCJA5(void *configPtr)
{
  struct_SNGCJA5 *sensorSetting = (struct_SNGCJA5*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure SNGCJA5 Particle Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Particle Mass Density 1.0um (ug/m^3): "));
      if (sensorSetting->logPM1 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Particle Mass Density 2.5um (ug/m^3): "));
      if (sensorSetting->logPM25 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Log Particle Mass Density 10.0um (ug/m^3): "));
      if (sensorSetting->logPM10 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("5) Log Particle Count 0.5um: "));
      if (sensorSetting->logPC05 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("6) Log Particle Count 1.0um: "));
      if (sensorSetting->logPC1 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("7) Log Particle Count 2.5um: "));
      if (sensorSetting->logPC25 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("8) Log Particle Count 5.0um: "));
      if (sensorSetting->logPC50 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("9) Log Particle Count 7.5um: "));
      if (sensorSetting->logPC75 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("10) Log Particle Count 10.0um: "));
      if (sensorSetting->logPC10 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("11) Log Combined Sensor Status: "));
      if (sensorSetting->logSensorStatus == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("12) Log PhotoDiode Status: "));
      if (sensorSetting->logPDStatus == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("13) Log LaserDiode Status: "));
      if (sensorSetting->logLDStatus == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("14) Log Fan Status: "));
      if (sensorSetting->logFanStatus == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == 2)
        sensorSetting->logPM1 ^= 1;
      else if (incoming == 3)
        sensorSetting->logPM25 ^= 1;
      else if (incoming == 4)
        sensorSetting->logPM10 ^= 1;
      else if (incoming == 5)
        sensorSetting->logPC05 ^= 1;
      else if (incoming == 6)
        sensorSetting->logPC1 ^= 1;
      else if (incoming == 7)
        sensorSetting->logPC25 ^= 1;
      else if (incoming == 8)
        sensorSetting->logPC50 ^= 1;
      else if (incoming == 9)
        sensorSetting->logPC75 ^= 1;
      else if (incoming == 10)
        sensorSetting->logPC10 ^= 1;
      else if (incoming == 11)
        sensorSetting->logSensorStatus ^= 1;
      else if (incoming == 12)
        sensorSetting->logPDStatus ^= 1;
      else if (incoming == 13)
        sensorSetting->logLDStatus ^= 1;
      else if (incoming == 14)
        sensorSetting->logFanStatus ^= 1;
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_SGP40(void *configPtr)
{
  struct_SGP40 *sensorSetting = (struct_SGP40*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure SGP40 VOC Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log VOC: "));
      if (sensorSetting->logVOC == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrintf2("3) Sensor Compensation: Relative Humidity (%): %d\r\n", sensorSetting->RH);

      SerialPrintf2("4) Sensor Compensation: Temperature (C): %d\r\n", sensorSetting->T);
    }
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == 2)
        sensorSetting->logVOC ^= 1;
      else if (incoming == 3)
      {
        SerialPrint(F("Enter the %RH for sensor compensation (0 to 100): "));
        int RH = getNumber(menuTimeout); //x second timeout
        if (RH < 0 || RH > 100)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->RH = RH;
      }
      else if (incoming == 4)
      {
        SerialPrint(F("Enter the temperature (C) for sensor compensation (-45 to 130): "));
        int T = getNumber(menuTimeout); //x second timeout
        if (T < -45 || T > 130)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->T = T;
      }
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_SDP3X(void *configPtr)
{
  struct_SDP3X *sensorSetting = (struct_SDP3X*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure SDP3X Differential Pressure Sensor"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Pressure: "));
      if (sensorSetting->logPressure == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Temperature: "));
      if (sensorSetting->logTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Temperature Compensation: "));
      if (sensorSetting->massFlow == true) SerialPrintln(F("Mass Flow"));
      else SerialPrintln(F("Differential Pressure"));

      SerialPrint(F("5) Measurement Averaging: "));
      if (sensorSetting->averaging == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == 2)
        sensorSetting->logPressure ^= 1;
      else if (incoming == 3)
        sensorSetting->logTemperature ^= 1;
      else if (incoming == 4)
        sensorSetting->massFlow ^= 1;
      else if (incoming == 5)
        sensorSetting->averaging ^= 1;
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_MS5837(void *configPtr)
{
  struct_MS5837 *sensorSetting = (struct_MS5837*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure MS5837 Pressure Sensor"));

    SerialPrint(F("Sensor Model: "));
    if (sensorSetting->model == 1) SerialPrintln(F("MS5837-02BA / BlueRobotics Bar02: 2 Bar Absolute / 10m Depth"));
    else SerialPrintln(F("MS5837-30BA / BlueRobotics Bar30: 30 Bar Absolute / 300m Depth"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      char tempStr[16];

      SerialPrint(F("2) Log Pressure: "));
      if (sensorSetting->logPressure == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Temperature: "));
      if (sensorSetting->logTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Log Depth: "));
      if (sensorSetting->logDepth == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("5) Log Altitude: "));
      if (sensorSetting->logAltitude == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      olaftoa(sensorSetting->fluidDensity, tempStr, 1, sizeof(tempStr) / sizeof(char));
      SerialPrintf2("6) Fluid Density (kg/m^3): %s\r\n", tempStr);

      olaftoa(sensorSetting->conversion, tempStr, 3, sizeof(tempStr) / sizeof(char));
      SerialPrintf2("7) Pressure Conversion Factor: %s\r\n", tempStr);
    }
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == 2)
        sensorSetting->logPressure ^= 1;
      else if (incoming == 3)
        sensorSetting->logTemperature ^= 1;
      else if (incoming == 4)
        sensorSetting->logDepth ^= 1;
      else if (incoming == 5)
        sensorSetting->logAltitude ^= 1;
      else if (incoming == 6)
      {
        SerialPrint(F("Enter the Fluid Density (kg/m^3): "));
        double FD = getDouble(menuTimeout); //x second timeout
        sensorSetting->fluidDensity = (float)FD;
      }
      else if (incoming == 7)
      {
        SerialPrint(F("Enter the Pressure Conversion Factor: "));
        double PCF = getDouble(menuTimeout); //x second timeout
        sensorSetting->conversion = (float)PCF;
      }
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_QWIIC_BUTTON(void *configPtr)
{
  struct_QWIIC_BUTTON *sensorSetting = (struct_QWIIC_BUTTON*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Qwiic Button"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Button Presses: "));
      if (sensorSetting->logPressed == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Button Clicks: "));
      if (sensorSetting->logClicked == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Toggle LED on each click (and log the LED state): "));
      if (sensorSetting->toggleLEDOnClick == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrintf2("5) LED Brightness: %d\r\n", sensorSetting->ledBrightness);
    }
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == 2)
        sensorSetting->logPressed ^= 1;
      else if (incoming == 3)
        sensorSetting->logClicked ^= 1;
      else if (incoming == 4)
        sensorSetting->toggleLEDOnClick ^= 1;
      else if (incoming == 5)
      {
        SerialPrint(F("Enter the LED brightness (0 to 255): "));
        int bright = getNumber(menuTimeout); //x second timeout
        if (bright < 0 || bright > 255)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->ledBrightness = bright;
      }
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_BIO_SENSOR_HUB(void *configPtr)
{
  struct_BIO_SENSOR_HUB *sensorSetting = (struct_BIO_SENSOR_HUB*)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Bio Sensor Hub (Pulse Oximeter)"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Heart Rate: "));
      if (sensorSetting->logHeartrate == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Confidence %: "));
      if (sensorSetting->logConfidence == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Log Oxygen %: "));
      if (sensorSetting->logOxygen == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("5) Log Status: "));
      if (sensorSetting->logStatus == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("6) Log Extended Status: "));
      if (sensorSetting->logExtendedStatus == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("7) Log Oxygen R Value: "));
      if (sensorSetting->logRValue == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == 2)
        sensorSetting->logHeartrate ^= 1;
      else if (incoming == 3)
        sensorSetting->logConfidence ^= 1;
      else if (incoming == 4)
        sensorSetting->logOxygen ^= 1;
      else if (incoming == 5)
        sensorSetting->logStatus ^= 1;
      else if (incoming == 6)
        sensorSetting->logExtendedStatus ^= 1;
      else if (incoming == 7)
        sensorSetting->logRValue ^= 1;
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_ISM330DHCX(void *configPtr)
{
  struct_ISM330DHCX *sensorSetting = (struct_ISM330DHCX *)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure ISM330DHCX IMU"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Accelerometer: "));
      if (sensorSetting->logAccel == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Gyro: "));
      if (sensorSetting->logGyro == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Log Data Ready: "));
      if (sensorSetting->logDataReady == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrintf2("5) Accel Scale: %d\r\n", sensorSetting->accelScale);
      SerialPrintf2("6) Accel Rate: %d\r\n", sensorSetting->accelRate);
      SerialPrint(F("7) Accel Filter LP2: "));
      if (sensorSetting->accelFilterLP2 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
      SerialPrintf2("8) Accel Slope Filter: %d\r\n", sensorSetting->accelSlopeFilter);
      SerialPrintf2("9) Gyro Scale: %d\r\n", sensorSetting->gyroScale);
      SerialPrintf2("10) Gyro Rate: %d\r\n", sensorSetting->gyroRate);
      SerialPrint(F("11) Gyro Filter LP1: "));
      if (sensorSetting->gyroFilterLP1 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
      SerialPrintf2("12) Gyro LP1 Bandwidth: %d\r\n", sensorSetting->gyroLP1BW);
    }
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == 2)
        sensorSetting->logAccel ^= 1;
      else if (incoming == 3)
        sensorSetting->logGyro ^= 1;
      else if (incoming == 4)
        sensorSetting->logDataReady ^= 1;
      else if (incoming == 5)
      {
        SerialPrintln(F("2g : 0"));
        SerialPrintln(F("16g: 1"));
        SerialPrintln(F("4g : 2"));
        SerialPrintln(F("8g : 3"));
        SerialPrint(F("Enter the Accel Full Scale (0 to 3): "));
        int newNum = getNumber(menuTimeout); //x second timeout
        if (newNum < 0 || newNum > 3)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->accelScale = newNum;
      }
      else if (incoming == 6)
      {
        SerialPrintln(F("OFF   : 0"));
        SerialPrintln(F("12.5Hz: 1"));
        SerialPrintln(F("26Hz  : 2"));
        SerialPrintln(F("52Hz  : 3"));
        SerialPrintln(F("104Hz : 4"));
        SerialPrintln(F("208Hz : 5"));
        SerialPrintln(F("416Hz : 6"));
        SerialPrintln(F("833Hz : 7"));
        SerialPrintln(F("1666Hz: 8"));
        SerialPrintln(F("3332Hz: 9"));
        SerialPrintln(F("6667Hz: 10"));
        SerialPrintln(F("1Hz6  : 11"));
        SerialPrint(F("Enter the Accel Rate (0 to 11): "));
        int newNum = getNumber(menuTimeout); //x second timeout
        if (newNum < 0 || newNum > 11)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->accelRate = newNum;
      }
      else if (incoming == 7)
        sensorSetting->accelFilterLP2 ^= 1;
      else if (incoming == 8)
      {
        SerialPrintln(F("HP_PATH_DISABLE_ON_OUT: 0"));
        SerialPrintln(F("LP_ODR_DIV_10         : 1"));
        SerialPrintln(F("LP_ODR_DIV_20         : 2"));
        SerialPrintln(F("LP_ODR_DIV_45         : 3"));
        SerialPrintln(F("LP_ODR_DIV_100        : 4"));
        SerialPrintln(F("LP_ODR_DIV_200        : 5"));
        SerialPrintln(F("LP_ODR_DIV_400        : 6"));
        SerialPrintln(F("LP_ODR_DIV_800        : 7"));
        SerialPrintln(F("SLOPE_ODR_DIV_4       : 16"));
        SerialPrintln(F("HP_ODR_DIV_10         : 17"));
        SerialPrintln(F("HP_ODR_DIV_20         : 18"));
        SerialPrintln(F("HP_ODR_DIV_45         : 19"));
        SerialPrintln(F("HP_ODR_DIV_100        : 20"));
        SerialPrintln(F("HP_ODR_DIV_200        : 21"));
        SerialPrintln(F("HP_ODR_DIV_400        : 22"));
        SerialPrintln(F("HP_ODR_DIV_800        : 23"));
        SerialPrintln(F("HP_REF_MD_ODR_DIV_10  : 49"));
        SerialPrintln(F("HP_REF_MD_ODR_DIV_20  : 50"));
        SerialPrintln(F("HP_REF_MD_ODR_DIV_45  : 51"));
        SerialPrintln(F("HP_REF_MD_ODR_DIV_100 : 52"));
        SerialPrintln(F("HP_REF_MD_ODR_DIV_200 : 53"));
        SerialPrintln(F("HP_REF_MD_ODR_DIV_400 : 54"));
        SerialPrintln(F("HP_REF_MD_ODR_DIV_800 : 55"));
        SerialPrint(F("Enter the Accel Slope Filter setting (0 to 55): "));
        int newNum = getNumber(menuTimeout); //x second timeout
        if (newNum < 0 || newNum > 55)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->accelSlopeFilter = newNum;
      }
      else if (incoming == 9)
      {
        SerialPrintln(F("125dps : 2"));
        SerialPrintln(F("250dps : 0"));
        SerialPrintln(F("500dps : 4"));
        SerialPrintln(F("1000dps: 8"));
        SerialPrintln(F("2000dps: 12"));
        SerialPrintln(F("4000dps: 1"));
        SerialPrint(F("Enter the Gyro Full Scale (0 to 12): "));
        int newNum = getNumber(menuTimeout); //x second timeout
        if (newNum < 0 || newNum > 12)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->gyroScale = newNum;
      }
      else if (incoming == 10)
      {
        SerialPrintln(F("OFF   : 0"));
        SerialPrintln(F("12Hz  : 1"));
        SerialPrintln(F("26Hz  : 2"));
        SerialPrintln(F("52Hz  : 3"));
        SerialPrintln(F("104Hz : 4"));
        SerialPrintln(F("208Hz : 5"));
        SerialPrintln(F("416Hz : 6"));
        SerialPrintln(F("833Hz : 7"));
        SerialPrintln(F("1666Hz: 8"));
        SerialPrintln(F("3332Hz: 9"));
        SerialPrintln(F("6667Hz: 10"));
        SerialPrint(F("Enter the Gyro Rate (0 to 10): "));
        int newNum = getNumber(menuTimeout); //x second timeout
        if (newNum < 0 || newNum > 10)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->gyroRate = newNum;
      }
      else if (incoming == 11)
        sensorSetting->gyroFilterLP1 ^= 1;
      else if (incoming == 12)
      {
        SerialPrintln(F("ULTRA_LIGHT: 0"));
        SerialPrintln(F("VERY_LIGHT : 1"));
        SerialPrintln(F("LIGHT      : 2"));
        SerialPrintln(F("MEDIUM     : 3"));
        SerialPrintln(F("STRONG     : 4"));
        SerialPrintln(F("VERY_STRONG: 5"));
        SerialPrintln(F("AGGRESSIVE : 6"));
        SerialPrintln(F("XTREME     : 7"));
        SerialPrintln(F("Enter the Gyro LP1 Bandwidth (0 to 7): "));
        int newNum = getNumber(menuTimeout); //x second timeout
        if (newNum < 0 || newNum > 7)
          SerialPrintln(F("Error: Out of range"));
        else
          sensorSetting->gyroLP1BW = newNum;
      }
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_MMC5983MA(void *configPtr)
{
  struct_MMC5983MA *sensorSetting = (struct_MMC5983MA *)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure MMC5983MA Magnetometer"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log Magnetometer: "));
      if (sensorSetting->logMag == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log Temperature: "));
      if (sensorSetting->logTemperature == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == 2)
        sensorSetting->logMag ^= 1;
      else if (incoming == 3)
        sensorSetting->logTemperature ^= 1;
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_KX134(void *configPtr)
{
  struct_KX134 *sensorSetting = (struct_KX134 *)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure KX134 Accelerometer"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Range 8G: "));
      if (sensorSetting->range8G == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Range 16G: "));
      if (sensorSetting->range16G == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Range 32G: "));
      if (sensorSetting->range32G == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("5) Range 64G: "));
      if (sensorSetting->range64G == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("6) High Speed (400Hz): "));
      if (sensorSetting->highSpeed == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

    }
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == 2)
      {
        sensorSetting->range8G = true;
        sensorSetting->range16G = false;
        sensorSetting->range32G = false;
        sensorSetting->range64G = false;
      }
      else if (incoming == 3)
      {
        sensorSetting->range8G = false;
        sensorSetting->range16G = true;
        sensorSetting->range32G = false;
        sensorSetting->range64G = false;
      }
      else if (incoming == 3)
      {
        sensorSetting->range8G = false;
        sensorSetting->range16G = false;
        sensorSetting->range32G = true;
        sensorSetting->range64G = false;
      }
      else if (incoming == 5)
      {
        sensorSetting->range8G = false;
        sensorSetting->range16G = false;
        sensorSetting->range32G = false;
        sensorSetting->range64G = true;
      }
      else if (incoming == 6)
        sensorSetting->highSpeed ^= 1;
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_ADS1015(void *configPtr)
{
  struct_ADS1015 *sensorSetting = (struct_ADS1015 *)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure ADS1015 ADC"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (sensorSetting->log == true)
    {
      SerialPrint(F("2) Log A0: "));
      if (sensorSetting->logA0 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("3) Log A1: "));
      if (sensorSetting->logA1 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("4) Log A2: "));
      if (sensorSetting->logA2 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("5) Log A3: "));
      if (sensorSetting->logA3 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("6) Log A0-A1: "));
      if (sensorSetting->logA0A1 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("7) Log A0-A3: "));
      if (sensorSetting->logA0A3 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("8) Log A1-A3: "));
      if (sensorSetting->logA1A3 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("9) Log A2-A3: "));
      if (sensorSetting->logA2A3 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("10) Gain x2/3: "));
      if (sensorSetting->gain23 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("11) Gain x1: "));
      if (sensorSetting->gain1 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("12) Gain x2: "));
      if (sensorSetting->gain2 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("13) Gain x4: "));
      if (sensorSetting->gain4 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("14) Gain x8: "));
      if (sensorSetting->gain8 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      SerialPrint(F("15) Gain x16: "));
      if (sensorSetting->gain16 == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

    }
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      sensorSetting->log ^= 1;
    else if (sensorSetting->log == true)
    {
      if (incoming == 2)
        sensorSetting->logA0 ^= 1;
      else if (incoming == 3)
        sensorSetting->logA1 ^= 1;
      else if (incoming == 4)
        sensorSetting->logA2 ^= 1;
      else if (incoming == 5)
        sensorSetting->logA3 ^= 1;
      else if (incoming == 6)
        sensorSetting->logA0A1 ^= 1;
      else if (incoming == 7)
        sensorSetting->logA0A3 ^= 1;
      else if (incoming == 8)
        sensorSetting->logA1A3 ^= 1;
      else if (incoming == 9)
        sensorSetting->logA2A3 ^= 1;
      else if (incoming == 10)
      {
        sensorSetting->gain23 = true;
        sensorSetting->gain1 = false;
        sensorSetting->gain2 = false;
        sensorSetting->gain4 = false;
        sensorSetting->gain8 = false;
        sensorSetting->gain16 = false;
      }
      else if (incoming == 11)
      {
        sensorSetting->gain23 = false;
        sensorSetting->gain1 = true;
        sensorSetting->gain2 = false;
        sensorSetting->gain4 = false;
        sensorSetting->gain8 = false;
        sensorSetting->gain16 = false;
      }
      else if (incoming == 12)
      {
        sensorSetting->gain23 = false;
        sensorSetting->gain1 = false;
        sensorSetting->gain2 = true;
        sensorSetting->gain4 = false;
        sensorSetting->gain8 = false;
        sensorSetting->gain16 = false;
      }
      else if (incoming == 13)
      {
        sensorSetting->gain23 = false;
        sensorSetting->gain1 = false;
        sensorSetting->gain2 = false;
        sensorSetting->gain4 = true;
        sensorSetting->gain8 = false;
        sensorSetting->gain16 = false;
      }
      else if (incoming == 14)
      {
        sensorSetting->gain23 = false;
        sensorSetting->gain1 = false;
        sensorSetting->gain2 = false;
        sensorSetting->gain4 = false;
        sensorSetting->gain8 = true;
        sensorSetting->gain16 = false;
      }
      else if (incoming == 15)
      {
        sensorSetting->gain23 = false;
        sensorSetting->gain1 = false;
        sensorSetting->gain2 = false;
        sensorSetting->gain4 = false;
        sensorSetting->gain8 = false;
        sensorSetting->gain16 = true;
      }
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

void menuConfigure_PCF8575(void *configPtr)
{
  struct_PCF8575 *sensorSetting = (struct_PCF8575 *)configPtr;

  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure PCF8575 ADC"));

    SerialPrint(F("1) Sensor Logging: "));
    if (sensorSetting->log == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    // Flip the option 1 setting vairable if 1 input
    if (incoming == 1)
      sensorSetting->log ^= 1;
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\menuDebug.ino"
void menuDebug()
{
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Debug Settings"));

    SerialPrint(F("1) Debug Messages: "));
    if (settings.printDebugMessages == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("2) Reset on Zero Device Count: "));
    if (settings.resetOnZeroDeviceCount == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("3) GNSS Debug Messages: "));
    if (settings.printGNSSDebugMessages == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("4) Only Open Main Menu With Printable Char: "));
    if (settings.openMenuWithPrintable == true) SerialPrintln(F("Yes"));
    else SerialPrintln(F("No"));

    SerialPrintln(F("5) Reboot the logger"));

    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
    {
      settings.printDebugMessages ^= 1;
    }
    else if (incoming == '2')
    {
      if (settings.resetOnZeroDeviceCount == false)
      {
        SerialPrintln(F(""));
        SerialPrintln(F("Enabling resetOnZeroDeviceCount will cause the OLA to completely reset if no devices are found on the Qwiic bus."));
        SerialPrintln(F("Do not enable this option if you are only logging IMU or Serial data."));
        SerialPrintln(F("Are you sure? Press 'y' to confirm: "));
        byte bContinue = getByteChoice(menuTimeout);
        if (bContinue == 'y')
        {
          settings.resetOnZeroDeviceCount ^= 1;
        }
        else
          SerialPrintln(F("\"resetOnZeroDeviceCount\"  aborted"));
      }
      else
      {
        settings.resetOnZeroDeviceCount ^= 1;
      }
    }
    else if (incoming == '3')
    {
      settings.printGNSSDebugMessages ^= 1;
    }
    else if (incoming == '4')
    {
      settings.openMenuWithPrintable ^= 1;
    }
    else if (incoming == '5')
    {
        SerialPrint(F("Are you sure? Press 'y' to confirm: "));
        byte bContinue = getByteChoice(menuTimeout);
        if (bContinue == 'y')
        {
          SerialPrintln(F("y"));
          resetArtemis();
        }
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\menuIMU.ino"
// Return true if IMU requires a restart
bool menuIMU()
{
  bool restartIMU = false;
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure IMU"));

    SerialPrint(F("1) Sensor Logging: "));
    if (settings.enableIMU == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (settings.enableIMU == true)
    {
      if (settings.imuUseDMP == false)
      {
        SerialPrint(F("2) Accelerometer Logging: "));
        if (settings.logIMUAccel) SerialPrintln(F("Enabled"));
        else SerialPrintln(F("Disabled"));
  
        SerialPrint(F("3) Gyro Logging: "));
        if (settings.logIMUGyro) SerialPrintln(F("Enabled"));
        else SerialPrintln(F("Disabled"));
  
        SerialPrint(F("4) Magnetometer Logging: "));
        if (settings.logIMUMag) SerialPrintln(F("Enabled"));
        else SerialPrintln(F("Disabled"));
  
        SerialPrint(F("5) Temperature Logging: "));
        if (settings.logIMUTemp) SerialPrintln(F("Enabled"));
        else SerialPrintln(F("Disabled"));
  
        if (online.IMU == true)
        {
          SerialPrint(F("6) Accelerometer Full Scale: +/- "));
          switch (settings.imuAccFSS)
          {
            case 0:
              SerialPrintln(F("2g"));
              break;
            case 1:
              SerialPrintln(F("4g"));
              break;
            case 2:
              SerialPrintln(F("8g"));
              break;
            case 3:
              SerialPrintln(F("16g"));
              break;
            default:
              SerialPrintln(F("UNKNOWN"));
              break;
          }
  
          SerialPrint(F("7) Accelerometer Digital Low Pass Filter: "));
          if (settings.imuAccDLPF)
          {
            SerialPrintln(F("Enabled"));
            SerialPrint(F("8) Accelerometer DLPF Bandwidth (Hz): "));
            switch (settings.imuAccDLPFBW)
            {
              case 0:
                SerialPrintln(F("246.0 (3dB)  265.0 (Nyquist)"));
                break;
              case 1:
                SerialPrintln(F("246.0 (3dB)  265.0 (Nyquist)"));
                break;
              case 2:
                SerialPrintln(F("111.4 (3dB)  136.0 (Nyquist)"));
                break;
              case 3:
                SerialPrintln(F("50.4 (3dB)  68.8 (Nyquist)"));
                break;
              case 4:
                SerialPrintln(F("23.9 (3dB)  34.4 (Nyquist)"));
                break;
              case 5:
                SerialPrintln(F("11.5 (3dB)  17.0 (Nyquist)"));
                break;
              case 6:
                SerialPrintln(F("5.7 (3dB)  8.3 (Nyquist)"));
                break;
              case 7:
                SerialPrintln(F("473 (3dB)  499 (Nyquist)"));
                break;
              default:
                SerialPrintln(F("UNKNOWN"));
                break;
            }
          }
          else
          {
            SerialPrintln(F("Disabled  (Bandwidth is 1209 Hz (3dB) 1248 Hz (Nyquist))"));
          }
  
          SerialPrint(F("9) Gyro Full Scale: +/- "));
          switch (settings.imuGyroFSS)
          {
            case 0:
              SerialPrintln(F("250dps"));
              break;
            case 1:
              SerialPrintln(F("500dps"));
              break;
            case 2:
              SerialPrintln(F("1000dps"));
              break;
            case 3:
              SerialPrintln(F("2000dps"));
              break;
            default:
              SerialPrintln(F("UNKNOWN"));
              break;
          }
  
          SerialPrint(F("10) Gyro Digital Low Pass Filter: "));
          if (settings.imuGyroDLPF)
          {
            SerialPrintln(F("Enabled"));
            SerialPrint(F("11) Gyro DLPF Bandwidth (Hz): "));
            switch (settings.imuGyroDLPFBW)
            {
              case 0:
                SerialPrintln(F("196.6 (3dB)  229.8 (Nyquist)"));
                break;
              case 1:
                SerialPrintln(F("151.8 (3dB)  187.6 (Nyquist)"));
                break;
              case 2:
                SerialPrintln(F("119.5 (3dB)  154.3 (Nyquist)"));
                break;
              case 3:
                SerialPrintln(F("51.2 (3dB)  73.3 (Nyquist)"));
                break;
              case 4:
                SerialPrintln(F("23.9 (3dB)  35.9 (Nyquist)"));
                break;
              case 5:
                SerialPrintln(F("11.6 (3dB)  17.8 (Nyquist)"));
                break;
              case 6:
                SerialPrintln(F("5.7 (3dB)  8.9 (Nyquist)"));
                break;
              case 7:
                SerialPrintln(F("361.4 (3dB)  376.5 (Nyquist)"));
                break;
              default:
                SerialPrintln(F("UNKNOWN"));
                break;
            }
          }
          else
          {
            SerialPrintln(F("Disabled  (Bandwidth is 12106 Hz (3dB) 12316 Hz (Nyquist))"));
          }
        }
      }

      SerialPrint(F("12) Digital Motion Processor (DMP): "));
      if (settings.imuUseDMP) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));

      if (settings.imuUseDMP == true)
      {
        SerialPrint(F("13) Game Rotation Vector (Quat6) Logging: "));
        if (settings.imuLogDMPQuat6) SerialPrintln(F("Enabled"));
        else SerialPrintln(F("Disabled"));
        SerialPrint(F("14) Rotation Vector (Quat9) Logging: "));
        if (settings.imuLogDMPQuat9) SerialPrintln(F("Enabled"));
        else SerialPrintln(F("Disabled"));
        SerialPrint(F("15) Accelerometer Logging: "));
        if (settings.imuLogDMPAccel) SerialPrintln(F("Enabled"));
        else SerialPrintln(F("Disabled"));
        SerialPrint(F("16) Gyro Logging: "));
        if (settings.imuLogDMPGyro) SerialPrintln(F("Enabled"));
        else SerialPrintln(F("Disabled"));
        SerialPrint(F("17) Compass Logging: "));
        if (settings.imuLogDMPCpass) SerialPrintln(F("Enabled"));
        else SerialPrintln(F("Disabled"));
      }
    }
    
    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
    {
      settings.enableIMU ^= 1;
      if (settings.enableIMU == true) beginIMU();
      else online.IMU = false;
    }
    else if (settings.enableIMU == true)
    {
      if (settings.imuUseDMP == false)
      {
        if (incoming == 2)
          settings.logIMUAccel ^= 1;
        else if (incoming == 3)
          settings.logIMUGyro ^= 1;
        else if (incoming == 4)
          settings.logIMUMag ^= 1;
        else if (incoming == 5)
          settings.logIMUTemp ^= 1;
        else if ((incoming == 6) && (online.IMU == true))
        {
          SerialPrintln(F("Enter Accelerometer Full Scale (0 to 3): "));
          SerialPrintln(F("0: +/- 2g"));
          SerialPrintln(F("1: +/- 4g"));
          SerialPrintln(F("2: +/- 8g"));
          SerialPrintln(F("3: +/- 16g"));
          int afs = getNumber(menuTimeout); //x second timeout
          if (afs < 0 || afs > 3)
            SerialPrintln(F("Error: Out of range"));
          else
          {
            settings.imuAccFSS = afs;
            ICM_20948_fss_t FSS;
            FSS.a = settings.imuAccFSS;
            FSS.g = settings.imuGyroFSS;
            ICM_20948_Status_e retval = myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
            if (retval != ICM_20948_Stat_Ok)
            {
              SerialPrintln(F("Error: Could not configure the IMU!"));
            }
          }
        }
        else if ((incoming == 7) && (online.IMU == true))
        {
          settings.imuAccDLPF ^= 1;
          ICM_20948_Status_e retval = myICM.enableDLPF(ICM_20948_Internal_Acc, settings.imuAccDLPF);
          if (retval != ICM_20948_Stat_Ok)
          {
            SerialPrintln(F("Error: Could not configure the IMU!"));
          }
        }
        else if ((incoming == 8) && (online.IMU == true) && (settings.imuAccDLPF == true))
        {
          SerialPrintln(F("Enter Accelerometer DLPF Bandwidth (0 to 7): "));
          SerialPrintln(F("0: 246.0 (3dB)  265.0 (Nyquist) (Hz)"));
          SerialPrintln(F("1: 246.0 (3dB)  265.0 (Nyquist) (Hz)"));
          SerialPrintln(F("2: 111.4 (3dB)  136.0 (Nyquist) (Hz)"));
          SerialPrintln(F("3: 50.4  (3dB)  68.8  (Nyquist) (Hz)"));
          SerialPrintln(F("4: 23.9  (3dB)  34.4  (Nyquist) (Hz)"));
          SerialPrintln(F("5: 11.5  (3dB)  17.0  (Nyquist) (Hz)"));
          SerialPrintln(F("6: 5.7   (3dB)  8.3   (Nyquist) (Hz)"));
          SerialPrintln(F("7: 473   (3dB)  499   (Nyquist) (Hz)"));
          int afbw = getNumber(menuTimeout); //x second timeout
          if (afbw < 0 || afbw > 7)
            SerialPrintln(F("Error: Out of range"));
          else
          {
            settings.imuAccDLPFBW = afbw;
            ICM_20948_dlpcfg_t dlpcfg;
            dlpcfg.a = settings.imuAccDLPFBW;
            dlpcfg.g = settings.imuGyroDLPFBW;
            ICM_20948_Status_e retval = myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
            if (retval != ICM_20948_Stat_Ok)
            {
              SerialPrintln(F("Error: Could not configure the IMU!"));
            }
          }
        }
        else if ((incoming == 9) && (online.IMU == true))
        {
          SerialPrintln(F("Enter Gyro Full Scale (0 to 3): "));
          SerialPrintln(F("0: +/- 250dps"));
          SerialPrintln(F("1: +/- 500dps"));
          SerialPrintln(F("2: +/- 1000dps"));
          SerialPrintln(F("3: +/- 2000dps"));
          int gfs = getNumber(menuTimeout); //x second timeout
          if (gfs < 0 || gfs > 3)
            SerialPrintln(F("Error: Out of range"));
          else
          {
            settings.imuGyroFSS = gfs;
            ICM_20948_fss_t FSS;
            FSS.a = settings.imuAccFSS;
            FSS.g = settings.imuGyroFSS;
            ICM_20948_Status_e retval = myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
            if (retval != ICM_20948_Stat_Ok)
            {
              SerialPrintln(F("Error: Could not configure the IMU!"));
            }
          }
        }
        else if ((incoming == 10) && (online.IMU == true))
        {
          settings.imuGyroDLPF ^= 1;
          ICM_20948_Status_e retval = myICM.enableDLPF(ICM_20948_Internal_Gyr, settings.imuGyroDLPF);
          if (retval != ICM_20948_Stat_Ok)
          {
            SerialPrintln(F("Error: Could not configure the IMU!"));
          }
        }
        else if ((incoming == 11) && (online.IMU == true) && (settings.imuGyroDLPF == true))
        {
          SerialPrintln(F("Enter Gyro DLPF Bandwidth (0 to 7): "));
          SerialPrintln(F("0: 196.6 (3dB)  229.8 (Nyquist) (Hz)"));
          SerialPrintln(F("1: 151.8 (3dB)  187.6 (Nyquist) (Hz)"));
          SerialPrintln(F("2: 119.5 (3dB)  154.3 (Nyquist) (Hz)"));
          SerialPrintln(F("3: 51.2  (3dB)  73.3  (Nyquist) (Hz)"));
          SerialPrintln(F("4: 23.9  (3dB)  35.9  (Nyquist) (Hz)"));
          SerialPrintln(F("5: 11.6  (3dB)  17.8  (Nyquist) (Hz)"));
          SerialPrintln(F("6: 5.7   (3dB)  8.9   (Nyquist) (Hz)"));
          SerialPrintln(F("7: 361.4 (3dB)  376.5 (Nyquist) (Hz)"));
          int gfbw = getNumber(menuTimeout); //x second timeout
          if (gfbw < 0 || gfbw > 7)
            SerialPrintln(F("Error: Out of range"));
          else
          {
            settings.imuGyroDLPFBW = gfbw;
            ICM_20948_dlpcfg_t dlpcfg;
            dlpcfg.a = settings.imuAccDLPFBW;
            dlpcfg.g = settings.imuGyroDLPFBW;
            ICM_20948_Status_e retval = myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
            if (retval != ICM_20948_Stat_Ok)
            {
              SerialPrintln(F("Error: Could not configure the IMU!"));
            }
          }
        }
        else if (incoming == 12)
        {
          settings.imuUseDMP ^= 1;
          restartIMU = true;
        }
        else if (incoming == STATUS_PRESSED_X)
          break;
        else if (incoming == STATUS_GETNUMBER_TIMEOUT)
          break;
        else
          printUnknown(incoming);
      }
      else if (settings.imuUseDMP == true)
      {
        if (incoming == 12)
        {
          settings.imuUseDMP ^= 1;
          restartIMU = true;
        }
        else if (incoming == 13)
        {
          if (settings.imuLogDMPQuat6 == true)
          {
            settings.imuLogDMPQuat6 = false;
          }
          else
          {
            settings.imuLogDMPQuat6 = true;
            settings.imuLogDMPQuat9 = false;
          }
          restartIMU = true;
        }
        else if (incoming == 14)
        {
          if (settings.imuLogDMPQuat9 == true)
          {
            settings.imuLogDMPQuat9 = false;
          }
          else
          {
            settings.imuLogDMPQuat9 = true;
            settings.imuLogDMPQuat6 = false;
          }
          restartIMU = true;
        }
        else if (incoming == 15)
        {
          settings.imuLogDMPAccel ^= 1;
          restartIMU = true;
        }
        else if (incoming == 16)
        {
          settings.imuLogDMPGyro ^= 1;
          restartIMU = true;
        }
        else if (incoming == 17)
        {
          settings.imuLogDMPCpass ^= 1;
          restartIMU = true;
        }
        else if (incoming == STATUS_PRESSED_X)
          break;
        else if (incoming == STATUS_GETNUMBER_TIMEOUT)
          break;
        else
          printUnknown(incoming);
      }
      else if (incoming == STATUS_PRESSED_X)
        break;
      else if (incoming == STATUS_GETNUMBER_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == STATUS_PRESSED_X)
      break;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
  return (restartIMU);
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\menuMain.ino"


#include "Sensors.h"

//Display the options
//If user doesn't respond within a few seconds, return to main loop
void menuMain(bool alwaysOpen)
{
  bool restartIMU = false;

  if (settings.openMenuWithPrintable) // If settings.openMenuWithPrintable is true, eat the first character. Return if < 9 (Tab)
  {
    if ((settings.useTxRxPinsForTerminal == true) && (Serial1.available()))
    {
      uint8_t firstChar = Serial1.read();
      if (firstChar < 9)
        return;
    }
    else if (Serial.available())
    {
      uint8_t firstChar = Serial.read();
      if (firstChar < 9)
        return;
    }
    else if (!alwaysOpen)
      return;
  }
  
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Main Menu"));

    SerialPrintln(F("1) Configure Terminal Output"));

    SerialPrintln(F("2) Configure Time Stamp"));

    if (online.IMU)
      SerialPrintln(F("3) Configure IMU Logging"));

    if (settings.useTxRxPinsForTerminal == false)
      SerialPrintln(F("4) Configure Serial Logging"));

    SerialPrintln(F("5) Configure Analog Logging"));

    SerialPrintln(F("6) Detect / Configure Attached Devices"));

    SerialPrintln(F("7) Configure Power Options"));

    SerialPrintln(F("h) Print Sensor Helper Text (and return to logging)"));

    if (online.microSD)
      SerialPrintln(F("s) SD Card File Transfer"));

    SerialPrintln(F("r) Reset all settings to default"));

    SerialPrintln(F("q) Quit: Close log files and power down"));

    //SerialPrintln(F("d) Debug Menu"));

    SerialPrintln(F("x) Return to logging"));

    byte incoming = getByteChoice(menuTimeout, true); //Get byte choice and set DSERIAL & ZSERIAL

    if (incoming == '1')
      menuLogRate();
    else if (incoming == '2')
      menuTimeStamp();
    else if ((incoming == '3') && (online.IMU))
      restartIMU = menuIMU();
    else if ((incoming == '4') && (settings.useTxRxPinsForTerminal == false))
      menuSerialLogging();
    else if (incoming == '5')
      menuAnalogLogging();
    else if (incoming == '6')
      menuAttachedDevices();
    else if (incoming == '7')
      menuPower();
    else if (incoming == 'h')
    {
      printHelperText(OL_OUTPUT_SERIAL); //printHelperText to terminal only
      break; //return to logging
    }
    else if (incoming == 'd')
    {
      menuDebug();
    }
    else if (incoming == 's')
    {
      if (online.microSD)
      {
        //Close log files before showing sdCardMenu
        if (online.dataLogging == true)
        {
          sensorDataFile.sync();
          updateDataFileAccess(&sensorDataFile); // Update the file access time & date
          sensorDataFile.close();
        }
        if (online.serialLogging == true)
        {
          serialDataFile.sync();
          updateDataFileAccess(&serialDataFile); // Update the file access time & date
          serialDataFile.close();
        }
  
        SerialPrintln(F(""));
        SerialPrintln(F(""));
        sdCardMenu(sdCardMenuTimeout); // Located in zmodem.ino
        SerialPrintln(F(""));
        SerialPrintln(F(""));
        
        if (online.dataLogging == true)
        {
          // Check if the current datafile was deleted
          if (sd.exists(sensorDataFileName) == false)
            strcpy(sensorDataFileName, findNextAvailableLog(settings.nextDataLogNumber, "dataLog"));
          beginDataLogging(); //180ms
          if (settings.showHelperText == true) 
            printHelperText(OL_OUTPUT_SERIAL | OL_OUTPUT_SDCARD); //printHelperText to terminal and sensor file
        }
        if (online.serialLogging == true)
        {
          // Check if the current serial file was deleted
          if (sd.exists(serialDataFileName) == false)
            strcpy(serialDataFileName, findNextAvailableLog(settings.nextSerialLogNumber, "serialLog"));
          beginSerialLogging();
        }
      }
    }
    else if (incoming == 'r')
    {
      SerialPrintln(F("\r\nResetting to factory defaults. Press 'y' to confirm: "));
      byte bContinue = getByteChoice(menuTimeout);
      if (bContinue == 'y')
      {
        EEPROM.erase();
        if (sd.exists("OLA_settings.txt"))
          sd.remove("OLA_settings.txt");
        if (sd.exists("OLA_deviceSettings.txt"))
          sd.remove("OLA_deviceSettings.txt");

        SerialPrint(F("Settings erased. Please reset OpenLog Artemis and open a terminal at "));
        Serial.print((String)settings.serialTerminalBaudRate);
        if (settings.useTxRxPinsForTerminal == true)
          Serial1.print((String)settings.serialTerminalBaudRate);
        SerialPrintln(F("bps..."));
        while (1);
      }
      else
        SerialPrintln(F("Reset aborted"));
    }
    else if (incoming == 'q')
    {
      SerialPrintln(F("\r\nQuit? Press 'y' to confirm:"));
      byte bContinue = getByteChoice(menuTimeout);
      if (bContinue == 'y')
      {
        //Save files before going to sleep
        if (online.dataLogging == true)
        {
          sensorDataFile.sync();
          updateDataFileAccess(&sensorDataFile); // Update the file access time & date
          sensorDataFile.close(); //No need to close files. https://forum.arduino.cc/index.php?topic=149504.msg1125098#msg1125098
        }
        if (online.serialLogging == true)
        {
          serialDataFile.sync();
          updateDataFileAccess(&serialDataFile); // Update the file access time & date
          serialDataFile.close();
        }
        SerialPrint(F("Log files are closed. Please reset OpenLog Artemis and open a terminal at "));
        Serial.print((String)settings.serialTerminalBaudRate);
        if (settings.useTxRxPinsForTerminal == true)
          Serial1.print((String)settings.serialTerminalBaudRate);
        SerialPrintln(F("bps..."));
        delay(sdPowerDownDelay); // Give the SD card time to shut down
        powerDownOLA();
      }
      else
        SerialPrintln(F("Quit aborted"));
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }

  recordSystemSettings(); //Once all menus have exited, record the new settings to EEPROM and config file

  recordDeviceSettingsToFile(); //Record the current devices settings to device config file

  configureQwiicDevices(); //Reconfigure the qwiic devices in case any settings have changed

  if (restartIMU == true)
    beginIMU(); // Restart the IMU if required

  while (Serial.available()) Serial.read(); //Empty buffer of any newline chars

  if (settings.useTxRxPinsForTerminal == true)
    while (Serial1.available()) Serial1.read(); //Empty buffer of any newline chars

  //Reset measurements
  measurementCount = 0;
  totalCharactersPrinted = 0;
  //If we are sleeping between readings then we cannot rely on millis() as it is powered down
  //Use RTC instead
  measurementStartTime = rtcMillis();

  //Edge case: after 10Hz reading, user sets the log rate above 2s mark. We never go to sleep because 
  //takeReading is not true. And since we don't wake up, takeReading never gets set to true.
  //So we force it here.
  takeReading = true; 
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\menuPower.ino"
void menuPower()
{
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Power Options"));

    SerialPrint(F("1) Turn off Qwiic bus power between readings (>2s): "));
    if (settings.powerDownQwiicBusBetweenReads == true) SerialPrintln(F("Yes"));
    else SerialPrintln(F("No"));

    SerialPrint(F("2) Use pin 32 to Stop Logging: "));
    if (settings.useGPIO32ForStopLogging == true) SerialPrintln(F("Yes"));
    else SerialPrintln(F("No"));

#if(HARDWARE_VERSION_MAJOR >= 1)
    SerialPrint(F("3) Power LED During Sleep: "));
    if (settings.enablePwrLedDuringSleep == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("4) Low Battery Voltage Detection: "));
    if (settings.enableLowBatteryDetection == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("5) Low Battery Threshold (V): "));
    char tempStr[16];
    olaftoa(settings.lowBatteryThreshold, tempStr, 2, sizeof(tempStr) / sizeof(char));
    SerialPrintf2("%s\r\n", tempStr);

    SerialPrint(F("6) VIN measurement correction factor: "));
    olaftoa(settings.vinCorrectionFactor, tempStr, 3, sizeof(tempStr) / sizeof(char));
    SerialPrintf2("%s\r\n", tempStr);
#endif

    SerialPrint(F("7) Serial Tx and Rx pins during sleep are: "));
    if (settings.serialTxRxDuringSleep == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
    {
      settings.powerDownQwiicBusBetweenReads ^= 1;
    }
    else if (incoming == '2')
    {
      if (settings.identifyBioSensorHubs == false)
      {
        if (settings.useGPIO32ForStopLogging == true)
        {
          // Disable stop logging
          settings.useGPIO32ForStopLogging = false;
          detachInterrupt(PIN_STOP_LOGGING); // Disable the interrupt
          pinMode(PIN_STOP_LOGGING, INPUT); // Remove the pull-up
          pin_config(PinName(PIN_STOP_LOGGING), g_AM_HAL_GPIO_INPUT); // Make sure the pin does actually get re-configured
          stopLoggingSeen = false; // Make sure the flag is clear
        }
        else
        {
          // Enable stop logging
          settings.useGPIO32ForStopLogging = true;
          pinMode(PIN_STOP_LOGGING, INPUT_PULLUP);
          pin_config(PinName(PIN_STOP_LOGGING), g_AM_HAL_GPIO_INPUT_PULLUP); // Make sure the pin does actually get re-configured
          delay(1); // Let the pin stabilize
          attachInterrupt(PIN_STOP_LOGGING, stopLoggingISR, FALLING); // Enable the interrupt
          am_hal_gpio_pincfg_t intPinConfig = g_AM_HAL_GPIO_INPUT_PULLUP;
          intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
          pin_config(PinName(PIN_STOP_LOGGING), intPinConfig); // Make sure the pull-up does actually stay enabled
          stopLoggingSeen = false; // Make sure the flag is clear
          settings.logA32 = false; // Disable analog logging on pin 32
        }
      }
      else
      {
        SerialPrintln(F(""));
        SerialPrintln(F("Stop logging via pin 32 is not possible. \"Detect Bio Sensor Pulse Oximeter\" is enabled."));
        SerialPrintln(F(""));
      }              
    }
#if(HARDWARE_VERSION_MAJOR >= 1)
    else if (incoming == '3')
    {
      settings.enablePwrLedDuringSleep ^= 1;
    }
    else if (incoming == '4')
    {
      settings.enableLowBatteryDetection ^= 1;
    }
    else if (incoming == '5')
    {
      SerialPrintln(F("Please enter the new low battery threshold:"));
      float tempBT = (float)getDouble(menuTimeout); //Timeout after x seconds
      if ((tempBT < 3.0) || (tempBT > 6.0))
        SerialPrintln(F("Error: Threshold out of range"));
      else
        settings.lowBatteryThreshold = tempBT;
    }
    else if (incoming == '6')
    {
      SerialPrintln(F("Please measure the voltage on the MEAS pin and enter it here:"));
      float tempCF = (float)getDouble(menuTimeout); //Timeout after x seconds
      int div3 = analogRead(PIN_VIN_MONITOR); //Read VIN across a 1/3 resistor divider
      float vin = (float)div3 * 3.0 * 2.0 / 16384.0; //Convert 1/3 VIN to VIN (14-bit resolution)
      tempCF = tempCF / vin; //Calculate the new correction factor
      if ((tempCF < 1.0) || (tempCF > 2.0))
        SerialPrintln(F("Error: Correction factor out of range"));
      else
        settings.vinCorrectionFactor = tempCF;
    }
#endif
    else if (incoming == '7')
    {
      settings.serialTxRxDuringSleep ^= 1;
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\menuSerialLogging.ino"
//TODO Add time stamp option after a certain timeout

void menuSerialLogging()
{
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Serial Logging"));

    SerialPrint(F("1) Log serial data: "));
    if (settings.logSerial == true) SerialPrintln(F("Enabled, analog logging on RX/A13 pin disabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("2) Output serial data to TX pin: "));
    if (settings.outputSerial == true) SerialPrintln(F("Enabled, analog logging on TX/A12 pin disabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("3) zmodem start delay: "));
    Serial.print(settings.zmodemStartDelay);
    if (settings.useTxRxPinsForTerminal == true)
      Serial1.print(settings.zmodemStartDelay);
    SerialPrintln(F(" seconds"));

    if ((settings.logSerial == true) || (settings.outputSerial == true))
    {
      SerialPrint(F("4) Set serial baud rate: "));
      Serial.print(settings.serialLogBaudRate);
      if (settings.useTxRxPinsForTerminal == true)
        Serial1.print(settings.zmodemStartDelay);
      SerialPrintln(F(" bps"));
    }

    if (settings.logSerial == true) // Suggested by @DennisMelamed in Issue #63
    {
      SerialPrint(F("5) Add RTC timestamp when token is received: "));
      if (settings.timestampSerial == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
  
      SerialPrint(F("6) Timestamp token: "));
      Serial.print(settings.timeStampToken);
      if (settings.useTxRxPinsForTerminal == true)
        Serial1.print(settings.timeStampToken);
      SerialPrint(F(" (Decimal)"));
      switch (settings.timeStampToken)
      {
        case 0x00:
          SerialPrintln(F(" = NULL"));
          break;
        case 0x03:
          SerialPrintln(F(" = End of Text"));
          break;
        case 0x0A:
          SerialPrintln(F(" = Line Feed"));
          break;
        case 0x0D:
          SerialPrintln(F(" = Carriage Return"));
          break;
        case 0x1B:
          SerialPrintln(F(" = Escape"));
          break;
        default:
          SerialPrintln(F(""));
          break;
      }
    }

    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
    {
      if (settings.logSerial == false)
      {
        settings.logSerial = true;
        settings.logA13 = false; //Disable analog readings on RX pin

        beginSerialLogging(); //Start up port and log file (this will set online.serialLogging to true if successful)
      }
      else
      {
        if (online.serialLogging)
        {
          //Shut it all down
          updateDataFileAccess(&serialDataFile); // Update the file access time & date
          serialDataFile.close();

          online.serialLogging = false;
        }
        settings.logSerial = false;
      }
    }
    else if (incoming == '2')
    {
      if (settings.outputSerial == false)
      {
        settings.outputSerial = true;
        settings.logA12 = false; //Disable analog readings on TX pin

        beginSerialOutput(); //Start up port (this will set online.serialOutput to true if successful)
      }
      else
      {
        online.serialOutput = false;
        settings.outputSerial = false;
      }
    }
    else if (incoming == '3')
    {
      SerialPrint(F("Enter zmodem start delay (5 to 60): "));
      int newDelay = getNumber(menuTimeout); //Timeout after x seconds
      if (newDelay < 5 || newDelay > 60)
      {
        SerialPrintln(F("Error: start delay out of range"));
      }
      else
      {
        settings.zmodemStartDelay = (uint8_t)newDelay;
      }
    }
    else if((settings.logSerial == true) || (settings.outputSerial == true))
    {
      if (incoming == '4')
      {
        SerialPrint(F("Enter baud rate (1200 to 500000): "));
        int newBaud = getNumber(menuTimeout); //Timeout after x seconds
        if (newBaud < 1200 || newBaud > 500000)
        {
          SerialPrintln(F("Error: baud rate out of range"));
        }
        else
        {
          configureSerial1TxRx();
          settings.serialLogBaudRate = newBaud;
          Serial1.begin(settings.serialLogBaudRate);
        }
      }
      else if (incoming == '5')
        settings.timestampSerial ^= 1;
      else if (incoming == '6')
      {
        SerialPrint(F("Enter the timestamp token in decimal (0 to 255): "));
        int newToken = getNumber(menuTimeout); //Timeout after x seconds
        if (newToken < 0 || newToken > 255)
        {
          SerialPrintln(F("Error: token out of range"));
        }
        else
        {
          settings.timeStampToken = (uint8_t)newToken;
        }
      }
      else if (incoming == 'x')
        return;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        return;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      return;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      return;
    else
      printUnknown(incoming);
  }
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\menuTerminal.ino"
void menuLogRate()
{
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Terminal Output"));

    SerialPrint(F("1) Log to microSD: "));
    if (settings.logData == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("2) Log to Terminal: "));
    if (settings.enableTerminalOutput == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("3) Set Serial Terminal Baud Rate: "));
    Serial.print(settings.serialTerminalBaudRate);
    if (settings.useTxRxPinsForTerminal == true)
      Serial1.print(settings.serialTerminalBaudRate);
    SerialPrintln(F(" bps"));

    if (settings.useGPIO11ForTrigger == false)
    {
      SerialPrint(F("4) Set Log Rate in Hz: "));
      if (settings.logMaxRate == true) SerialPrintln(F("Max rate enabled"));
      else
      {
        if (settings.usBetweenReadings < 1000000ULL) //Take more than one measurement per second
        {
          //Display Integer Hertz
          int logRate = (int)(1000000ULL / settings.usBetweenReadings);
          SerialPrintf2("%d\r\n", logRate);
        }
        else
        {
          //Display fractional Hertz
          uint32_t logRateSeconds = (uint32_t)(settings.usBetweenReadings / 1000000ULL);
          char tempStr[16];
          olaftoa(1.0 / logRateSeconds, tempStr, 6, sizeof(tempStr) / sizeof(char));
          SerialPrintf2("%s\r\n", tempStr);
        }
      }
  
      SerialPrint(F("5) Set Log Rate in seconds between readings: "));
      if (settings.logMaxRate == true) SerialPrintln(F("Max rate enabled"));
      else
      {
        if (settings.usBetweenReadings > 1000000ULL) //Take more than one measurement per second
        {
          uint32_t interval = (uint32_t)(settings.usBetweenReadings / 1000000ULL);
          SerialPrintf2("%d\r\n", interval);
        }
        else
        {
          float rate = (float)(settings.usBetweenReadings / 1000000.0);
          char tempStr[16];
          olaftoa(rate, tempStr, 6, sizeof(tempStr) / sizeof(char));
          SerialPrintf2("%s\r\n", tempStr);
        }
      }
  
      SerialPrint(F("6) Enable maximum logging: "));
      if (settings.logMaxRate == true) SerialPrintln(F("Enabled"));
      else SerialPrintln(F("Disabled"));
    }

    SerialPrint(F("7) Output Actual Hertz: "));
    if (settings.logHertz == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("8) Output Column Titles: "));
    if (settings.showHelperText == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("9) Output Measurement Count: "));
    if (settings.printMeasurementCount == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("10) Open New Log Files After (s): "));
    SerialPrintf2("%d", settings.openNewLogFilesAfter);
    if (settings.openNewLogFilesAfter == 0) SerialPrintln(F(" (Never)"));
    else SerialPrintln(F(""));

    SerialPrint(F("11) Frequent log file access timestamps: "));
    if (settings.frequentFileAccessTimestamps == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("12) Use pin 11 to trigger logging: "));
    if (settings.useGPIO11ForTrigger == true) SerialPrintln(F("Yes"));
    else SerialPrintln(F("No"));

    SerialPrint(F("13) Logging is triggered when the signal on pin 11 is: "));
    if (settings.fallingEdgeTrigger == true) SerialPrintln(F("Falling"));
    else SerialPrintln(F("Rising"));

    SerialPrint(F("14) Use TX and RX pins for Terminal: "));
    if (settings.useTxRxPinsForTerminal == true)
    {
      SerialPrintln(F("Enabled"));
      SerialPrintln(F("                                     Analog logging on TX/A12 and RX/A13 is permanently disabled"));
      SerialPrintln(F("                                     Serial logging on RX/A13 is permanently disabled"));
    }
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("15) Use Pin 11 to control fast/slow logging: "));
    if (settings.useGPIO11ForFastSlowLogging == true) SerialPrintln(F("Yes"));
    else SerialPrintln(F("No"));

    if (settings.useGPIO11ForFastSlowLogging == true)
    {
      SerialPrint(F("16) Log slowly when Pin 11 is: "));
      if (settings.slowLoggingWhenPin11Is == true) SerialPrintln(F("High"));
      else SerialPrintln(F("Low"));
    }

    SerialPrint(F("17) Use RTC to control fast/slow logging: "));
    if (settings.useRTCForFastSlowLogging == true) SerialPrintln(F("Yes"));
    else SerialPrintln(F("No"));

    if ((settings.useGPIO11ForFastSlowLogging == true) || (settings.useRTCForFastSlowLogging == true))
    {
      SerialPrint(F("18) Slow logging interval (seconds): "));
      SerialPrintf2("%d\r\n", settings.slowLoggingIntervalSeconds);
    }

    if (settings.useRTCForFastSlowLogging == true)
    {
      SerialPrint(F("19) Slow logging starts at: "));
      int slowHour = settings.slowLoggingStartMOD / 60;
      int slowMin = settings.slowLoggingStartMOD % 60;
      char hourStr[3];
      char minStr[3];
      if (slowHour < 10)
        sprintf(hourStr, "0%d", slowHour);
      else
        sprintf(hourStr, "%d", slowHour);
      if (slowMin < 10)
        sprintf(minStr, "0%d", slowMin);
      else
        sprintf(minStr, "%d", slowMin);
      SerialPrintf3("%s:%s\r\n", hourStr, minStr);

      SerialPrint(F("20) Slow logging ends at: "));
      slowHour = settings.slowLoggingStopMOD / 60;
      slowMin = settings.slowLoggingStopMOD % 60;
      if (slowHour < 10)
        sprintf(hourStr, "0%d", slowHour);
      else
        sprintf(hourStr, "%d", slowHour);
      if (slowMin < 10)
        sprintf(minStr, "0%d", slowMin);
      else
        sprintf(minStr, "%d", slowMin);
      SerialPrintf3("%s:%s\r\n", hourStr, minStr);
    }

    if ((settings.useGPIO11ForTrigger == false) && (settings.usBetweenReadings >= maxUsBeforeSleep))
    {
      SerialPrint(F("21) Minimum awake time between sleeps: "));
      SerialPrintf2("%dms\r\n", settings.minimumAwakeTimeMillis);
    }

    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      settings.logData ^= 1;
    else if (incoming == 2)
      settings.enableTerminalOutput ^= 1;
    else if (incoming == 3)
    {
      SerialPrint(F("Enter baud rate (1200 to 500000): "));
      int newBaud = getNumber(menuTimeout); //Timeout after x seconds
      if (newBaud < 1200 || newBaud > 500000)
      {
        SerialPrintln(F("Error: baud rate out of range"));
      }
      else
      {
        settings.serialTerminalBaudRate = newBaud;
        recordSystemSettings(); //Normally recorded upon all menu exits
        recordDeviceSettingsToFile(); //Normally recorded upon all menu exits
        SerialPrintf2("Terminal now set at %dbps. Please reset device and open terminal at new baud rate. Freezing...\r\n", settings.serialTerminalBaudRate);
        while (1);
      }
    }
    else if (incoming == 4)
    {
      if (settings.useGPIO11ForTrigger == false)
      {
        int maxOutputRate = settings.serialTerminalBaudRate / 10 / (totalCharactersPrinted / measurementCount);
        maxOutputRate = (maxOutputRate * 90) / 100; //Fudge reduction of 10%
  
        if (maxOutputRate < 10) maxOutputRate = 10; //TODO this is forced. Needed when multi seconds between readings.
  
        SerialPrintf2("How many readings per second would you like to log? (Current max is %d): ", maxOutputRate);
        int tempRPS = getNumber(menuTimeout); //Timeout after x seconds
        if (tempRPS < 1 || tempRPS > maxOutputRate)
          SerialPrintln(F("Error: Readings Per Second out of range"));
        else
          settings.usBetweenReadings = 1000000ULL / ((uint64_t)tempRPS);
      }
    }
    else if (incoming == 5)
    {
      if (settings.useGPIO11ForTrigger == false)
      {
        //The Deep Sleep duration is set with am_hal_stimer_compare_delta_set, the duration of which is uint32_t
        //So the maximum we can sleep for is 2^32 / 32768 = 131072 seconds = 36.4 hours
        //Let's limit this to 36 hours = 129600 seconds
        SerialPrintln(F("How many seconds would you like to wait between readings? (1 to 129,600):"));
        int64_t tempSeconds = getNumber(menuTimeout); //Timeout after x seconds
        if (tempSeconds < 1 || tempSeconds > 129600)
          SerialPrintln(F("Error: logging interval out of range"));
        else
        {
          settings.usBetweenReadings = 1000000ULL * ((uint64_t)tempSeconds);

          if (settings.usBetweenReadings >= maxUsBeforeSleep) // Check if minimumAwakeTimeMillis needs to be reduced
          {
            // Limit minimumAwakeTimeMillis to usBetweenReadings minus one second
            if (settings.minimumAwakeTimeMillis > ((settings.usBetweenReadings / 1000ULL) - 1000ULL))
              settings.minimumAwakeTimeMillis = (unsigned long)((settings.usBetweenReadings / 1000ULL) - 1000ULL);
          }
        }
      }
    }
    else if (incoming == 6)
    {
      if (settings.useGPIO11ForTrigger == false)
      {
        if (settings.logMaxRate == false)
        {
          SerialPrintln(F("\r\nEnabling max log rate will disable the IMU, \r\nterminal output, and serial logging. \r\nOnly analog values will be logged. Continue?"));
          byte bContinue = getByteChoice(menuTimeout);
          if (bContinue == 'y')
          {
            settings.logMaxRate = true;
            settings.logSerial = false;
            settings.enableTerminalOutput = false;
            settings.enableIMU = false;
  
            //Close files on SD to be sure they are recorded fully
            updateDataFileAccess(&serialDataFile); // Update the file access time & date
            serialDataFile.close();
            updateDataFileAccess(&sensorDataFile); // Update the file access time & date          
            sensorDataFile.close();
  
            recordSystemSettings(); //Normally recorded upon all menu exits
            recordDeviceSettingsToFile(); //Normally recorded upon all menu exits
  
            SerialPrintln(F("OpenLog Artemis configured for max data rate. Please reset. Freezing..."));
            while (1);
          }
        }
        else
        {
          settings.logMaxRate = false;
          //settings.usBetweenReadings = 100000ULL; //Default to 100,000us = 100ms = 10 readings per second.
        }
      }
    }
    else if (incoming == 7)
      settings.logHertz ^= 1;
    else if (incoming == 8)
      settings.showHelperText ^= 1;
    else if (incoming == 9)
      settings.printMeasurementCount ^= 1;
    else if (incoming == 10)
    {
      SerialPrintln(F("Open new log files after this many seconds (0 or 10 to 129,600) (0 = Never):"));
      int64_t tempSeconds = getNumber(menuTimeout); //Timeout after x seconds
      if ((tempSeconds < 0) || ((tempSeconds > 0) && (tempSeconds < 10)) || (tempSeconds > 129600ULL))
        SerialPrintln(F("Error: Invalid interval"));
      else
        settings.openNewLogFilesAfter = tempSeconds;
    }
    else if (incoming == 11)
      settings.frequentFileAccessTimestamps ^= 1;
    else if (incoming == 12)
    {
      if (settings.identifyBioSensorHubs == false)
      {
        if (settings.useGPIO11ForTrigger == true)
        {
          // Disable triggering
          settings.useGPIO11ForTrigger = false;
          detachInterrupt(PIN_TRIGGER); // Disable the interrupt
          pinMode(PIN_TRIGGER, INPUT); // Remove the pull-up
          pin_config(PinName(PIN_TRIGGER), g_AM_HAL_GPIO_INPUT); // Make sure the pin does actually get re-configured
          triggerEdgeSeen = false; // Make sure the flag is clear
        }
        else
        {
          // Enable triggering
          settings.useGPIO11ForTrigger = true;
          pinMode(PIN_TRIGGER, INPUT_PULLUP);
          pin_config(PinName(PIN_TRIGGER), g_AM_HAL_GPIO_INPUT_PULLUP); // Make sure the pin does actually get re-configured
          delay(1); // Let the pin stabilize
          am_hal_gpio_pincfg_t intPinConfig = g_AM_HAL_GPIO_INPUT_PULLUP;
          if (settings.fallingEdgeTrigger == true)
          {
            attachInterrupt(PIN_TRIGGER, triggerPinISR, FALLING); // Enable the interrupt
            intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
          }
          else
          {
            attachInterrupt(PIN_TRIGGER, triggerPinISR, RISING); // Enable the interrupt
            intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
          }
          pin_config(PinName(PIN_TRIGGER), intPinConfig); // Make sure the pull-up does actually stay enabled
          triggerEdgeSeen = false; // Make sure the flag is clear
          settings.logA11 = false; // Disable analog logging on pin 11
          settings.logMaxRate = false; // Disable max rate logging
          settings.useGPIO11ForFastSlowLogging = false;
          settings.useRTCForFastSlowLogging = false;
        }
      }
      else
      {
        SerialPrintln(F(""));
        SerialPrintln(F("Triggering on pin 11 is not possible. \"Detect Bio Sensor Pulse Oximeter\" is enabled."));
        SerialPrintln(F(""));
      }      
    }
    else if (incoming == 13)
    {
      if (settings.identifyBioSensorHubs == false)
      {
        if (settings.useGPIO11ForTrigger == true) // If interrupts are enabled, we need to disable and then re-enable
        {
          detachInterrupt(PIN_TRIGGER); // Disable the interrupt
          settings.fallingEdgeTrigger ^= 1; // Invert the flag
          am_hal_gpio_pincfg_t intPinConfig = g_AM_HAL_GPIO_INPUT_PULLUP;
          if (settings.fallingEdgeTrigger == true)
          {
            attachInterrupt(PIN_TRIGGER, triggerPinISR, FALLING); // Enable the interrupt
            intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
          }
          else
          {
            attachInterrupt(PIN_TRIGGER, triggerPinISR, RISING); // Enable the interrupt
            intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
          }
          pin_config(PinName(PIN_TRIGGER), intPinConfig); // Make sure the pull-up does actually stay enabled
          triggerEdgeSeen = false; // Make sure the flag is clear
        }
        else
          settings.fallingEdgeTrigger ^= 1; // Interrupt is not currently enabled so simply invert the flag
      }
      else
      {
        SerialPrintln(F(""));
        SerialPrintln(F("Triggering on pin 11 is not possible. \"Detect Bio Sensor Pulse Oximeter\" is enabled."));
        SerialPrintln(F(""));
      }      
    }
    else if (incoming == 14)
    {
      if (settings.useTxRxPinsForTerminal == false)
      {
        SerialPrintln(F(""));
        SerialPrintln(F("\"Use TX and RX pins for terminal\" can only be disabled by \"Reset all settings to default\"."));
        SerialPrintln(F("Analog logging on TX/A12 and RX/A13 will be disabled."));
        SerialPrintln(F("Serial logging will be disabled."));
        SerialPrintln(F("Are you sure? Press 'y' to confirm: "));
        byte bContinue = getByteChoice(menuTimeout);
        if (bContinue == 'y')
        {
          settings.useTxRxPinsForTerminal = true;
          if (online.serialLogging == true)
          {
            serialDataFile.sync();
            updateDataFileAccess(&serialDataFile); // Update the file access time & date
            serialDataFile.close();
          }
          online.serialLogging = false;
          settings.logSerial = false;
          settings.outputSerial = false;
          online.serialOutput = false;
          settings.logA12 = false;
          settings.logA13 = false;

          //We need to manually restore the Serial1 TX and RX pins before we can use Serial1
          configureSerial1TxRx();

          Serial1.begin(settings.serialTerminalBaudRate); // (Re)Start the serial port using the terminal baud rate
        }
        else
          SerialPrintln(F("\"Use TX and RX pins for terminal\"  aborted"));
      }
      else
      {
        SerialPrintln(F(""));
        SerialPrintln(F("\"Use TX and RX pins for terminal\" can not be disabled."));
        SerialPrintln(F("You need to use \"Reset all settings to default\" from the main menu."));
        SerialPrintln(F(""));
      }
    }
    else if (incoming == 15)
    {
      if (settings.identifyBioSensorHubs == false)
      {
        if (settings.useGPIO11ForFastSlowLogging == false) // If the user is trying to enable Pin 11 fast / slow logging
        {
          settings.useGPIO11ForFastSlowLogging = true;
          settings.useRTCForFastSlowLogging = false;
          settings.logA11 = false; // Disable analog logging on pin 11
          pinMode(PIN_TRIGGER, INPUT_PULLUP);
          pin_config(PinName(PIN_TRIGGER), g_AM_HAL_GPIO_INPUT_PULLUP); // Make sure the pin does actually get re-configured
          delay(1); // Let the pin stabilize
          // Disable triggering
          if (settings.useGPIO11ForTrigger == true)
          {
            detachInterrupt(PIN_TRIGGER); // Disable the interrupt
            triggerEdgeSeen = false; // Make sure the flag is clear
          }
          settings.useGPIO11ForTrigger = false;
        }
        else // If the user is trying to disable Pin 11 fast / slow logging
        {
          settings.useGPIO11ForFastSlowLogging = false;        
          pinMode(PIN_TRIGGER, INPUT); // Remove the pull-up
          pin_config(PinName(PIN_TRIGGER), g_AM_HAL_GPIO_INPUT); // Make sure the pin does actually get re-configured
        }
      }
      else
      {
        SerialPrintln(F(""));
        SerialPrintln(F("Fast / slow logging via pin 11 is not possible. \"Detect Bio Sensor Pulse Oximeter\" is enabled."));
        SerialPrintln(F(""));
      }              
    }
    else if (incoming == 16)
    {
      if (settings.identifyBioSensorHubs == false)
      {
        if (settings.useGPIO11ForFastSlowLogging == true)
        {
          settings.slowLoggingWhenPin11Is ^= 1;
        }
      }
      else // If the user is trying to disable Pin 11 fast / slow logging
      {
        settings.useGPIO11ForFastSlowLogging = false;        
        pinMode(PIN_TRIGGER, INPUT); // Remove the pull-up
        pin_config(PinName(PIN_TRIGGER), g_AM_HAL_GPIO_INPUT); // Make sure the pin does actually get re-configured
      }
    }
    else if (incoming == 17)
    {
      if (settings.useRTCForFastSlowLogging == false) // If the user is trying to enable RTC fast / slow logging
      {
        settings.useRTCForFastSlowLogging = true;
        if (settings.useGPIO11ForFastSlowLogging == true)
        {
          pinMode(PIN_TRIGGER, INPUT); // Remove the pull-up
          pin_config(PinName(PIN_TRIGGER), g_AM_HAL_GPIO_INPUT); // Make sure the pin does actually get re-configured
        }
        settings.useGPIO11ForFastSlowLogging = false;
        settings.logA11 = false; // Disable analog logging on pin 11
        // Disable triggering
        if (settings.useGPIO11ForTrigger == true)
        {
          detachInterrupt(PIN_TRIGGER); // Disable the interrupt
          pinMode(PIN_TRIGGER, INPUT); // Remove the pull-up
          pin_config(PinName(PIN_TRIGGER), g_AM_HAL_GPIO_INPUT); // Make sure the pin does actually get re-configured
          triggerEdgeSeen = false; // Make sure the flag is clear
        }
        settings.useGPIO11ForTrigger = false;
      }
      else // If the user is trying to disable RTC fast / slow logging
      {
        settings.useRTCForFastSlowLogging = false;        
      }
    }
    else if (incoming == 18)
    {
      if ((settings.useGPIO11ForFastSlowLogging == true) || (settings.useRTCForFastSlowLogging == true))
      {
        //The Deep Sleep duration is set with am_hal_stimer_compare_delta_set, the duration of which is uint32_t
        //So the maximum we can sleep for is 2^32 / 32768 = 131072 seconds = 36.4 hours
        //Let's limit this to 36 hours = 129600 seconds
        SerialPrintln(F("How many seconds would you like to sleep between readings? (5 to 129,600):"));
        int64_t tempSeconds = getNumber(menuTimeout); //Timeout after x seconds
        if (tempSeconds < 5 || tempSeconds > 129600)
          SerialPrintln(F("Error: sleep interval out of range"));
        else
          settings.slowLoggingIntervalSeconds = (int)tempSeconds;
      }      
    }
    else if (incoming == 19)
    {
      if (settings.useRTCForFastSlowLogging == true)
      {
        SerialPrintln(F("Enter the Hour slow logging should start (0 to 23):"));
        int64_t tempMOD = getNumber(menuTimeout); //Timeout after x seconds
        if (tempMOD < 0 || tempMOD > 23)
          SerialPrintln(F("Error: time out of range"));
        else
        {
          settings.slowLoggingStartMOD = (int)tempMOD * 60; // Convert to minutes
          SerialPrintln(F("\r\nEnter the Minute slow logging should start (0 to 59):"));
          tempMOD = getNumber(menuTimeout); //Timeout after x seconds
          if (tempMOD < 0 || tempMOD > 59)
            SerialPrintln(F("Error: time out of range"));
          else
            settings.slowLoggingStartMOD += (int)tempMOD;
        }
      }      
    }
    else if (incoming == 20)
    {
      if (settings.useRTCForFastSlowLogging == true)
      {
        SerialPrintln(F("Enter the Hour slow logging should end (0 to 23):"));
        int64_t tempMOD = getNumber(menuTimeout); //Timeout after x seconds
        if (tempMOD < 0 || tempMOD > 23)
          SerialPrintln(F("Error: time out of range"));
        else
        {
          settings.slowLoggingStopMOD = (int)tempMOD * 60; // Convert to minutes
          SerialPrintln(F("\r\nEnter the Minute slow logging should end (0 to 59):"));
          tempMOD = getNumber(menuTimeout); //Timeout after x seconds
          if (tempMOD < 0 || tempMOD > 59)
            SerialPrintln(F("Error: time out of range"));
          else
            settings.slowLoggingStopMOD += (int)tempMOD;
        }
      }      
    }
    else if (incoming == 21)
    {
      if (settings.useGPIO11ForTrigger == false)
      {
        if (settings.usBetweenReadings >= maxUsBeforeSleep)
        {
          // Limit minimumAwakeTimeMillis to usBetweenReadings minus one second
          unsigned long maxAwakeMillis = (unsigned long)((settings.usBetweenReadings / 1000ULL) - 1000ULL);
          SerialPrintf2("Enter minimum awake time (ms: 0 to %d): : ", maxAwakeMillis);
          int newAwake = getNumber(menuTimeout); //Timeout after x seconds
          if (newAwake < 0 || newAwake > maxAwakeMillis)
          {
            SerialPrintln(F("Error: awake time out of range"));
          }
          else
          {
            settings.minimumAwakeTimeMillis = newAwake;
          }
        }
      }
    }
    else if (incoming == STATUS_PRESSED_X)
      return;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      return;
    else
      printUnknown(incoming);
  }
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\menuTimeStamp.ino"
void menuTimeStamp()
{
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Time Stamp"));
    SerialPrint(F("Current date/time: "));
    myRTC.getTime();

    char rtcDate[11]; // 10/12/2019
    char rtcDay[3];
    char rtcMonth[3];
    char rtcYear[5];
    if (myRTC.dayOfMonth < 10)
      sprintf(rtcDay, "0%d", myRTC.dayOfMonth);
    else
      sprintf(rtcDay, "%d", myRTC.dayOfMonth);
    if (myRTC.month < 10)
      sprintf(rtcMonth, "0%d", myRTC.month);
    else
      sprintf(rtcMonth, "%d", myRTC.month);
    if (myRTC.year < 10)
      sprintf(rtcYear, "200%d", myRTC.year);
    else
      sprintf(rtcYear, "20%d", myRTC.year);
    if (settings.dateStyle == 0)
      sprintf(rtcDate, "%s/%s/%s,", rtcMonth, rtcDay, rtcYear);
    else if (settings.dateStyle == 1)
      sprintf(rtcDate, "%s/%s/%s,", rtcDay, rtcMonth, rtcYear);
    else
      sprintf(rtcDate, "%s/%s/%s,", rtcYear, rtcMonth, rtcDay);

    SerialPrint(rtcDate);
    SerialPrint(F(" "));

    char rtcTime[13]; //09:14:37.41,
    int adjustedHour = myRTC.hour;
    if (settings.hour24Style == false)
    {
      if (adjustedHour > 12) adjustedHour -= 12;
    }
    char rtcHour[3];
    char rtcMin[3];
    char rtcSec[3];
    char rtcHundredths[3];
    if (adjustedHour < 10)
      sprintf(rtcHour, "0%d", adjustedHour);
    else
      sprintf(rtcHour, "%d", adjustedHour);
    if (myRTC.minute < 10)
      sprintf(rtcMin, "0%d", myRTC.minute);
    else
      sprintf(rtcMin, "%d", myRTC.minute);
    if (myRTC.seconds < 10)
      sprintf(rtcSec, "0%d", myRTC.seconds);
    else
      sprintf(rtcSec, "%d", myRTC.seconds);
    if (myRTC.hundredths < 10)
      sprintf(rtcHundredths, "0%d", myRTC.hundredths);
    else
      sprintf(rtcHundredths, "%d", myRTC.hundredths);
    sprintf(rtcTime, "%s:%s:%s.%s", rtcHour, rtcMin, rtcSec, rtcHundredths);

    SerialPrintln(rtcTime);

    SerialPrint(F("1) Log Date: "));
    if (settings.logDate == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("2) Log Time: "));
    if (settings.logTime == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    if (settings.logDate == true || settings.logTime == true)
    {
      SerialPrintln(F("3) Set RTC to compiler macro time"));
    }

    if (settings.logDate == true)
    {
      SerialPrintln(F("4) Manually set RTC date"));

      SerialPrint(F("5) Toggle date style: "));
      if (settings.dateStyle == 0) SerialPrintln(F("mm/dd/yyyy"));
      else if (settings.dateStyle == 1) SerialPrintln(F("dd/mm/yyyy"));
      else if (settings.dateStyle == 2) SerialPrintln(F("yyyy/mm/dd"));
      else SerialPrintln(F("ISO 8601"));
    }

    if (settings.logTime == true)
    {
      SerialPrintln(F("6) Manually set RTC time"));

      SerialPrint(F("7) Toggle time style: "));
      if (settings.hour24Style == true) SerialPrintln(F("24 hour"));
      else SerialPrintln(F("12 hour"));
    }

    if (settings.logDate == true || settings.logTime == true)
    {
      if (isUbloxAttached() == true)
      {
        SerialPrintln(F("8) Synchronize RTC to GPS"));
      }
      SerialPrint(F("9) Local offset from UTC: "));
      Serial.println(settings.localUTCOffset);
      if (settings.useTxRxPinsForTerminal == true)
        Serial1.println(settings.localUTCOffset);
    }

    SerialPrint(F("10) Log Microseconds: "));
    if (settings.logMicroseconds == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
      settings.logDate ^= 1;
    else if (incoming == 2)
      settings.logTime ^= 1;
    else if (incoming == 10)
      settings.logMicroseconds ^= 1;
    else if (incoming == STATUS_PRESSED_X)
      return;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      return;

    if ((settings.logDate == true) || (settings.logTime == true))
    {
      //Options 3, 8, 9
      if (incoming == 3)
      {
        myRTC.setToCompilerTime(); //Set RTC using the system __DATE__ and __TIME__ macros from compiler
        SerialPrintln(F("RTC set to compiler time"));
      }
      else if ((incoming == 8) && (isUbloxAttached() == true))
      {
        myRTC.getTime(); // Get the RTC date and time (just in case getGPSDateTime fails)
        int dd = myRTC.dayOfMonth, mm = myRTC.month, yy = myRTC.year, h = myRTC.hour, m = myRTC.minute, s = myRTC.seconds, ms = (myRTC.hundredths * 10);
        bool dateValid, timeValid;
        getGPSDateTime(yy, mm, dd, h, m, s, ms, dateValid, timeValid); // Get the GPS date and time, corrected for localUTCOffset
        myRTC.setTime((ms / 10), s, m, h, dd, mm, (yy - 2000)); //Manually set RTC
        lastSDFileNameChangeTime = rtcMillis(); // Record the time of the file name change
        SerialPrintln(F("RTC set to GPS (UTC) time"));
        if ((dateValid == false) || (timeValid == false))
        {
          SerialPrintln(F("\r\nWarning: the GPS time or date was not valid. Please try again.\r\n"));
        }
      }
      else if (incoming == 9)
      {
        SerialPrint(F("Enter the local hour offset from UTC (-12 to 14): "));
        float offset = (float)getDouble(menuTimeout); //Timeout after x seconds
        if (offset < -12 || offset > 14)
          SerialPrintln(F("Error: Offset is out of range"));
        else
          settings.localUTCOffset = offset;
      }
    }

    if (settings.logDate == true)
    {
      //4 and 5
      if (incoming == 4) //Manually set RTC date
      {
        myRTC.getTime();
        int dd = myRTC.dayOfMonth, mm = myRTC.month, yy = myRTC.year, h = myRTC.hour, m = myRTC.minute, s = myRTC.seconds;

        SerialPrint(F("Enter current two digit year: "));
        yy = getNumber(menuTimeout); //Timeout after x seconds
        if (yy > 2000 && yy < 2100) yy -= 2000;

        SerialPrint(F("Enter current month (1 to 12): "));
        mm = getNumber(menuTimeout); //Timeout after x seconds

        SerialPrint(F("Enter current day (1 to 31): "));
        dd = getNumber(menuTimeout); //Timeout after x seconds

        myRTC.getTime();
        h = myRTC.hour; m = myRTC.minute; s = myRTC.seconds;
        myRTC.setTime(0, s, m, h, dd, mm, yy); //Manually set RTC
        lastSDFileNameChangeTime = rtcMillis(); // Record the time of the file name change
      }
      else if (incoming == 5)
      {
        settings.dateStyle += 1;
        if (settings.dateStyle == 4)
          settings.dateStyle = 0;
      }
    }

    if (settings.logTime == true)
    {
      //6 and 7
      if (incoming == 6) //Manually set time
      {
        myRTC.getTime();
        int dd = myRTC.dayOfMonth, mm = myRTC.month, yy = myRTC.year, h = myRTC.hour, m = myRTC.minute, s = myRTC.seconds;

        SerialPrint(F("Enter current hour (0 to 23): "));
        h = getNumber(menuTimeout); //Timeout after x seconds

        SerialPrint(F("Enter current minute (0 to 59): "));
        m = getNumber(menuTimeout); //Timeout after x seconds

        SerialPrint(F("Enter current second (0 to 59): "));
        s = getNumber(menuTimeout); //Timeout after x seconds

        myRTC.setTime(0, s, m, h, dd, mm, yy); //Manually set RTC
        lastSDFileNameChangeTime = rtcMillis(); // Record the time of the file name change
      }
      else if (incoming == 7)
      {
        settings.hour24Style ^= 1;
      }
    }
  }
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\nvm.ino"
void loadSettings()
{
  //First load any settings from NVM
  //After, we'll load settings from config file if available
  //We'll then re-record settings so that the settings from the file over-rides internal NVM settings

  //Check to see if EEPROM is blank
  uint32_t testRead = 0;
  if (EEPROM.get(0, testRead) == 0xFFFFFFFF)
  {
    SerialPrintln(F("EEPROM is blank. Default settings applied"));
    recordSystemSettings(); //Record default settings to EEPROM and config file. At power on, settings are in default state
  }

  //Check that the current settings struct size matches what is stored in EEPROM
  //Misalignment happens when we add a new feature or setting
  int tempSize = 0;
  EEPROM.get(0, tempSize); //Load the sizeOfSettings
  if (tempSize != sizeof(settings))
  {
    SerialPrintln(F("Settings wrong size. Default settings applied"));
    recordSystemSettings(); //Record default settings to EEPROM and config file. At power on, settings are in default state
  }

  //Check that the olaIdentifier is correct
  //(It is possible for two different versions of the code to have the same sizeOfSettings - which causes problems!)
  int tempIdentifier = 0;
  EEPROM.get(sizeof(int), tempIdentifier); //Load the identifier from the EEPROM location after sizeOfSettings (int)
  if (tempIdentifier != OLA_IDENTIFIER)
  {
    SerialPrintln(F("Settings are not valid for this variant of the OLA. Default settings applied"));
    recordSystemSettings(); //Record default settings to EEPROM and config file. At power on, settings are in default state
  }

  //Read current settings
  EEPROM.get(0, settings);

  loadSystemSettingsFromFile(); //Load any settings from config file. This will over-write any pre-existing EEPROM settings.
  //Record these new settings to EEPROM and config file to be sure they are the same
  //(do this even if loadSystemSettingsFromFile returned false)
  recordSystemSettings();
}

//Record the current settings struct to EEPROM and then to config file
void recordSystemSettings()
{
  settings.sizeOfSettings = sizeof(settings);
  EEPROM.put(0, settings);
  recordSystemSettingsToFile();
}

//Export the current settings to a config file
void recordSystemSettingsToFile()
{
  if (online.microSD == true)
  {
    if (sd.exists("OLA_settings.txt"))
      sd.remove("OLA_settings.txt");

    #if SD_FAT_TYPE == 1
    File32 settingsFile;
    #elif SD_FAT_TYPE == 2
    ExFile settingsFile;
    #elif SD_FAT_TYPE == 3
    FsFile settingsFile;
    #else // SD_FAT_TYPE == 0
    File settingsFile;
    #endif  // SD_FAT_TYPE
    if (settingsFile.open("OLA_settings.txt", O_CREAT | O_APPEND | O_WRITE) == false)
    {
      SerialPrintln(F("Failed to create settings file"));
      return;
    }

    settingsFile.println("sizeOfSettings=" + (String)settings.sizeOfSettings);
    settingsFile.println("olaIdentifier=" + (String)settings.olaIdentifier);
    settingsFile.println("nextSerialLogNumber=" + (String)settings.nextSerialLogNumber);
    settingsFile.println("nextDataLogNumber=" + (String)settings.nextDataLogNumber);

    // Convert uint64_t to string
    // Based on printLLNumber by robtillaart
    // https://forum.arduino.cc/index.php?topic=143584.msg1519824#msg1519824
    char tempTimeRev[20]; // Char array to hold to usBetweenReadings (reversed order)
    char tempTime[20]; // Char array to hold to usBetweenReadings (correct order)
    uint64_t usBR = settings.usBetweenReadings;
    unsigned int i = 0;
    if (usBR == 0ULL) // if usBetweenReadings is zero, set tempTime to "0"
    {
      tempTime[0] = '0';
      tempTime[1] = 0;
    }
    else
    {
      while (usBR > 0)
      {
        tempTimeRev[i++] = (usBR % 10) + '0'; // divide by 10, convert the remainder to char
        usBR /= 10; // divide by 10
      }
      unsigned int j = 0;
      while (i > 0)
      {
        tempTime[j++] = tempTimeRev[--i]; // reverse the order
        tempTime[j] = 0; // mark the end with a NULL
      }
    }
    
    settingsFile.println("usBetweenReadings=" + (String)tempTime);

    //printDebug(F("Saving usBetweenReadings to SD card: "));
    //printDebug((String)tempTime);
    //printDebug(F("\r\n"));

    settingsFile.println("logMaxRate=" + (String)settings.logMaxRate);
    settingsFile.println("enableRTC=" + (String)settings.enableRTC);
    settingsFile.println("enableIMU=" + (String)settings.enableIMU);
    settingsFile.println("enableTerminalOutput=" + (String)settings.enableTerminalOutput);
    settingsFile.println("logDate=" + (String)settings.logDate);
    settingsFile.println("logTime=" + (String)settings.logTime);
    settingsFile.println("logData=" + (String)settings.logData);
    settingsFile.println("logSerial=" + (String)settings.logSerial);
    settingsFile.println("logIMUAccel=" + (String)settings.logIMUAccel);
    settingsFile.println("logIMUGyro=" + (String)settings.logIMUGyro);
    settingsFile.println("logIMUMag=" + (String)settings.logIMUMag);
    settingsFile.println("logIMUTemp=" + (String)settings.logIMUTemp);
    settingsFile.println("logRTC=" + (String)settings.logRTC);
    settingsFile.println("logHertz=" + (String)settings.logHertz);
    settingsFile.println("correctForDST=" + (String)settings.correctForDST);
    settingsFile.println("dateStyle=" + (String)settings.dateStyle);
    settingsFile.println("hour24Style=" + (String)settings.hour24Style);
    settingsFile.println("serialTerminalBaudRate=" + (String)settings.serialTerminalBaudRate);
    settingsFile.println("serialLogBaudRate=" + (String)settings.serialLogBaudRate);
    settingsFile.println("showHelperText=" + (String)settings.showHelperText);
    settingsFile.println("logA11=" + (String)settings.logA11);
    settingsFile.println("logA12=" + (String)settings.logA12);
    settingsFile.println("logA13=" + (String)settings.logA13);
    settingsFile.println("logA32=" + (String)settings.logA32);
    settingsFile.println("logAnalogVoltages=" + (String)settings.logAnalogVoltages);
    settingsFile.print("localUTCOffset="); settingsFile.println(settings.localUTCOffset);
    settingsFile.println("printDebugMessages=" + (String)settings.printDebugMessages);
    settingsFile.println("powerDownQwiicBusBetweenReads=" + (String)settings.powerDownQwiicBusBetweenReads);
    settingsFile.println("qwiicBusMaxSpeed=" + (String)settings.qwiicBusMaxSpeed);
    settingsFile.println("qwiicBusPowerUpDelayMs=" + (String)settings.qwiicBusPowerUpDelayMs);
    settingsFile.println("printMeasurementCount=" + (String)settings.printMeasurementCount);
    settingsFile.println("enablePwrLedDuringSleep=" + (String)settings.enablePwrLedDuringSleep);
    settingsFile.println("logVIN=" + (String)settings.logVIN);
    settingsFile.println("openNewLogFilesAfter=" + (String)settings.openNewLogFilesAfter);
    settingsFile.print("vinCorrectionFactor="); settingsFile.println(settings.vinCorrectionFactor);
    settingsFile.println("useGPIO32ForStopLogging=" + (String)settings.useGPIO32ForStopLogging);
    settingsFile.println("qwiicBusPullUps=" + (String)settings.qwiicBusPullUps);
    settingsFile.println("outputSerial=" + (String)settings.outputSerial);
    settingsFile.println("zmodemStartDelay=" + (String)settings.zmodemStartDelay);
    settingsFile.println("enableLowBatteryDetection=" + (String)settings.enableLowBatteryDetection);
    settingsFile.print("lowBatteryThreshold="); settingsFile.println(settings.lowBatteryThreshold);
    settingsFile.println("frequentFileAccessTimestamps=" + (String)settings.frequentFileAccessTimestamps);
    settingsFile.println("useGPIO11ForTrigger=" + (String)settings.useGPIO11ForTrigger);
    settingsFile.println("fallingEdgeTrigger=" + (String)settings.fallingEdgeTrigger);
    settingsFile.println("imuAccDLPF=" + (String)settings.imuAccDLPF);
    settingsFile.println("imuGyroDLPF=" + (String)settings.imuGyroDLPF);
    settingsFile.println("imuAccFSS=" + (String)settings.imuAccFSS);
    settingsFile.println("imuAccDLPFBW=" + (String)settings.imuAccDLPFBW);
    settingsFile.println("imuGyroFSS=" + (String)settings.imuGyroFSS);
    settingsFile.println("imuGyroDLPFBW=" + (String)settings.imuGyroDLPFBW);
    settingsFile.println("logMicroseconds=" + (String)settings.logMicroseconds);
    settingsFile.println("useTxRxPinsForTerminal=" + (String)settings.useTxRxPinsForTerminal);
    settingsFile.println("timestampSerial=" + (String)settings.timestampSerial);
    settingsFile.println("timeStampToken=" + (String)settings.timeStampToken);
    settingsFile.println("useGPIO11ForFastSlowLogging=" + (String)settings.useGPIO11ForFastSlowLogging);
    settingsFile.println("slowLoggingWhenPin11Is=" + (String)settings.slowLoggingWhenPin11Is);
    settingsFile.println("useRTCForFastSlowLogging=" + (String)settings.useRTCForFastSlowLogging);
    settingsFile.println("slowLoggingIntervalSeconds=" + (String)settings.slowLoggingIntervalSeconds);
    settingsFile.println("slowLoggingStartMOD=" + (String)settings.slowLoggingStartMOD);
    settingsFile.println("slowLoggingStopMOD=" + (String)settings.slowLoggingStopMOD);
    settingsFile.println("resetOnZeroDeviceCount=" + (String)settings.resetOnZeroDeviceCount);
    settingsFile.println("imuUseDMP=" + (String)settings.imuUseDMP);
    settingsFile.println("imuLogDMPQuat6=" + (String)settings.imuLogDMPQuat6);
    settingsFile.println("imuLogDMPQuat9=" + (String)settings.imuLogDMPQuat9);
    settingsFile.println("imuLogDMPAccel=" + (String)settings.imuLogDMPAccel);
    settingsFile.println("imuLogDMPGyro=" + (String)settings.imuLogDMPGyro);
    settingsFile.println("imuLogDMPCpass=" + (String)settings.imuLogDMPCpass);
    settingsFile.println("minimumAwakeTimeMillis=" + (String)settings.minimumAwakeTimeMillis);
    settingsFile.println("identifyBioSensorHubs=" + (String)settings.identifyBioSensorHubs);
    settingsFile.println("serialTxRxDuringSleep=" + (String)settings.serialTxRxDuringSleep);
    settingsFile.println("printGNSSDebugMessages=" + (String)settings.printGNSSDebugMessages);
    settingsFile.println("openMenuWithPrintable=" + (String)settings.openMenuWithPrintable);
    updateDataFileAccess(&settingsFile); // Update the file access time & date
    settingsFile.close();
  }
}

//If a config file exists on the SD card, load them and overwrite the local settings
//Heavily based on ReadCsvFile from SdFat library
//Returns true if some settings were loaded from a file
//Returns false if a file was not opened/loaded
bool loadSystemSettingsFromFile()
{
  if (online.microSD == true)
  {
    if (sd.exists("OLA_settings.txt"))
    {
      SdFile settingsFile; //FAT32
      if (settingsFile.open("OLA_settings.txt", O_READ) == false)
      {
        SerialPrintln(F("Failed to open settings file"));
        return (false);
      }

      char line[60];
      int lineNumber = 0;

      while (settingsFile.available()) {
        int n = settingsFile.fgets(line, sizeof(line));
        if (n <= 0) {
          SerialPrintf2("Failed to read line %d from settings file\r\n", lineNumber);
        }
        else if (line[n - 1] != '\n' && n == (sizeof(line) - 1)) {
          SerialPrintf2("Settings line %d too long\r\n", lineNumber);
          if (lineNumber == 0)
          {
            //If we can't read the first line of the settings file, give up
            SerialPrintln(F("Giving up on settings file"));
            settingsFile.close();
            return (false);
          }
        }
        else if (parseLine(line) == false) {
          SerialPrintf3("Failed to parse line %d: %s\r\n", lineNumber, line);
          if (lineNumber == 0)
          {
            //If we can't read the first line of the settings file, give up
            SerialPrintln(F("Giving up on settings file"));
            settingsFile.close();
            return (false);
          }
        }

        lineNumber++;
      }

      //SerialPrintln(F("Config file read complete"));
      settingsFile.close();
      return (true);
    }
    else
    {
      SerialPrintln(F("No config file found. Using settings from EEPROM."));
      //The defaults of the struct will be recorded to a file later on.
      return (false);
    }
  }

  SerialPrintln(F("Config file read failed: SD offline"));
  return (false); //SD offline
}

// Check for extra characters in field or find minus sign.
char* skipSpace(char* str) {
  while (isspace(*str)) str++;
  return str;
}

//Convert a given line from file into a settingName and value
//Sets the setting if the name is known
bool parseLine(char* str) {
  char* ptr;

  //Debug
  //SerialPrintf2("Line contents: %s", str);
  //SerialFlush();

  // Set strtok start of line.
  str = strtok(str, "=");
  if (!str) return false;

  //Store this setting name
  char settingName[40];
  sprintf(settingName, "%s", str);

  //Move pointer to end of line
  str = strtok(nullptr, "\n");
  if (!str) return false;

  //SerialPrintf2("s = %s\r\n", str);
  //SerialFlush();

  // Convert string to double.
  double d = strtod(str, &ptr);

  //SerialPrintf2("d = %lf\r\n", d);
  //SerialFlush();

  if (str == ptr || *skipSpace(ptr)) return false;

  // Get setting name
  if (strcmp(settingName, "sizeOfSettings") == 0)
  {
    //We may want to cause a factory reset from the settings file rather than the menu
    //If user sets sizeOfSettings to -1 in config file, OLA will factory reset
    if (d == -1)
    {
      EEPROM.erase();
      sd.remove("OLA_settings.txt");
      SerialPrintln(F("OpenLog Artemis has been factory reset. Freezing. Please restart and open terminal at 115200bps."));
      while (1);
    }

    //Check to see if this setting file is compatible with this version of OLA
    if (d != sizeof(settings))
      SerialPrintf3("Warning: Settings size is %d but current firmware expects %d. Attempting to use settings from file.\r\n", d, sizeof(settings));

  }
  else if (strcmp(settingName, "olaIdentifier") == 0)
    settings.olaIdentifier = d;
  else if (strcmp(settingName, "nextSerialLogNumber") == 0)
    settings.nextSerialLogNumber = d;
  else if (strcmp(settingName, "nextDataLogNumber") == 0)
    settings.nextDataLogNumber = d;
  else if (strcmp(settingName, "usBetweenReadings") == 0)
  {
    settings.usBetweenReadings = d;
    //printDebug(F("Read usBetweenReadings from SD card: "));
    //printDebug(String(d));
    //printDebug(F("\r\n"));
  }
  else if (strcmp(settingName, "logMaxRate") == 0)
    settings.logMaxRate = d;
  else if (strcmp(settingName, "enableRTC") == 0)
    settings.enableRTC = d;
  else if (strcmp(settingName, "enableIMU") == 0)
    settings.enableIMU = d;
  else if (strcmp(settingName, "enableTerminalOutput") == 0)
    settings.enableTerminalOutput = d;
  else if (strcmp(settingName, "logDate") == 0)
    settings.logDate = d;
  else if (strcmp(settingName, "logTime") == 0)
    settings.logTime = d;
  else if (strcmp(settingName, "logData") == 0)
    settings.logData = d;
  else if (strcmp(settingName, "logSerial") == 0)
    settings.logSerial = d;
  else if (strcmp(settingName, "logIMUAccel") == 0)
    settings.logIMUAccel = d;
  else if (strcmp(settingName, "logIMUGyro") == 0)
    settings.logIMUGyro = d;
  else if (strcmp(settingName, "logIMUMag") == 0)
    settings.logIMUMag = d;
  else if (strcmp(settingName, "logIMUTemp") == 0)
    settings.logIMUTemp = d;
  else if (strcmp(settingName, "logRTC") == 0)
    settings.logRTC = d;
  else if (strcmp(settingName, "logHertz") == 0)
    settings.logHertz = d;
  else if (strcmp(settingName, "correctForDST") == 0)
    settings.correctForDST = d;
  else if (strcmp(settingName, "dateStyle") == 0)
    settings.dateStyle = d;
  else if (strcmp(settingName, "americanDateStyle") == 0) // Included for backward-compatibility
    settings.dateStyle = d;
  else if (strcmp(settingName, "hour24Style") == 0)
    settings.hour24Style = d;
  else if (strcmp(settingName, "serialTerminalBaudRate") == 0)
    settings.serialTerminalBaudRate = d;
  else if (strcmp(settingName, "serialLogBaudRate") == 0)
    settings.serialLogBaudRate = d;
  else if (strcmp(settingName, "showHelperText") == 0)
    settings.showHelperText = d;
  else if (strcmp(settingName, "logA11") == 0)
    settings.logA11 = d;
  else if (strcmp(settingName, "logA12") == 0)
    settings.logA12 = d;
  else if (strcmp(settingName, "logA13") == 0)
    settings.logA13 = d;
  else if (strcmp(settingName, "logA32") == 0)
    settings.logA32 = d;
  else if (strcmp(settingName, "logAnalogVoltages") == 0)
    settings.logAnalogVoltages = d;
  else if (strcmp(settingName, "localUTCOffset") == 0)
    settings.localUTCOffset = d;
  else if (strcmp(settingName, "printDebugMessages") == 0)
    settings.printDebugMessages = d;
  else if (strcmp(settingName, "powerDownQwiicBusBetweenReads") == 0)
    settings.powerDownQwiicBusBetweenReads = d;
  else if (strcmp(settingName, "qwiicBusMaxSpeed") == 0)
    settings.qwiicBusMaxSpeed = d;
  else if (strcmp(settingName, "qwiicBusPowerUpDelayMs") == 0)
    settings.qwiicBusPowerUpDelayMs = d;
  else if (strcmp(settingName, "printMeasurementCount") == 0)
    settings.printMeasurementCount = d;
  else if (strcmp(settingName, "enablePwrLedDuringSleep") == 0)
    settings.enablePwrLedDuringSleep = d;
  else if (strcmp(settingName, "logVIN") == 0)
    settings.logVIN = d;
  else if (strcmp(settingName, "openNewLogFilesAfter") == 0)
    settings.openNewLogFilesAfter = d;
  else if (strcmp(settingName, "vinCorrectionFactor") == 0)
    settings.vinCorrectionFactor = d;
  else if (strcmp(settingName, "useGPIO32ForStopLogging") == 0)
    settings.useGPIO32ForStopLogging = d;
  else if (strcmp(settingName, "qwiicBusPullUps") == 0)
    settings.qwiicBusPullUps = d;
  else if (strcmp(settingName, "outputSerial") == 0)
    settings.outputSerial = d;
  else if (strcmp(settingName, "zmodemStartDelay") == 0)
    settings.zmodemStartDelay = d;
  else if (strcmp(settingName, "enableLowBatteryDetection") == 0)
    settings.enableLowBatteryDetection = d;
  else if (strcmp(settingName, "lowBatteryThreshold") == 0)
    settings.lowBatteryThreshold = d;
  else if (strcmp(settingName, "frequentFileAccessTimestamps") == 0)
    settings.frequentFileAccessTimestamps = d;
  else if (strcmp(settingName, "useGPIO11ForTrigger") == 0)
    settings.useGPIO11ForTrigger = d;
  else if (strcmp(settingName, "fallingEdgeTrigger") == 0)
    settings.fallingEdgeTrigger = d;
  else if (strcmp(settingName, "imuAccDLPF") == 0)
    settings.imuAccDLPF = d;
  else if (strcmp(settingName, "imuGyroDLPF") == 0)
    settings.imuGyroDLPF = d;
  else if (strcmp(settingName, "imuAccFSS") == 0)
    settings.imuAccFSS = d;
  else if (strcmp(settingName, "imuAccDLPFBW") == 0)
    settings.imuAccDLPFBW = d;
  else if (strcmp(settingName, "imuGyroFSS") == 0)
    settings.imuGyroFSS = d;
  else if (strcmp(settingName, "imuGyroDLPFBW") == 0)
    settings.imuGyroDLPFBW = d;
  else if (strcmp(settingName, "logMicroseconds") == 0)
    settings.logMicroseconds = d;
  else if (strcmp(settingName, "useTxRxPinsForTerminal") == 0)
    settings.useTxRxPinsForTerminal = d;
  else if (strcmp(settingName, "timestampSerial") == 0)
    settings.timestampSerial = d;
  else if (strcmp(settingName, "timeStampToken") == 0)
    settings.timeStampToken = d;
  else if (strcmp(settingName, "useGPIO11ForFastSlowLogging") == 0)
    settings.useGPIO11ForFastSlowLogging = d;
  else if (strcmp(settingName, "slowLoggingWhenPin11Is") == 0)
    settings.slowLoggingWhenPin11Is = d;
  else if (strcmp(settingName, "useRTCForFastSlowLogging") == 0)
    settings.useRTCForFastSlowLogging = d;
  else if (strcmp(settingName, "slowLoggingIntervalSeconds") == 0)
    settings.slowLoggingIntervalSeconds = d;
  else if (strcmp(settingName, "slowLoggingStartMOD") == 0)
    settings.slowLoggingStartMOD = d;
  else if (strcmp(settingName, "slowLoggingStopMOD") == 0)
    settings.slowLoggingStopMOD = d;
  else if (strcmp(settingName, "resetOnZeroDeviceCount") == 0)
    settings.resetOnZeroDeviceCount = d;
  else if (strcmp(settingName, "imuUseDMP") == 0)
    settings.imuUseDMP = d;
  else if (strcmp(settingName, "imuLogDMPQuat6") == 0)
    settings.imuLogDMPQuat6 = d;
  else if (strcmp(settingName, "imuLogDMPQuat9") == 0)
    settings.imuLogDMPQuat9 = d;
  else if (strcmp(settingName, "imuLogDMPAccel") == 0)
    settings.imuLogDMPAccel = d;
  else if (strcmp(settingName, "imuLogDMPGyro") == 0)
    settings.imuLogDMPGyro = d;
  else if (strcmp(settingName, "imuLogDMPCpass") == 0)
    settings.imuLogDMPCpass = d;
  else if (strcmp(settingName, "minimumAwakeTimeMillis") == 0)
    settings.minimumAwakeTimeMillis = d;
  else if (strcmp(settingName, "identifyBioSensorHubs") == 0)
    settings.identifyBioSensorHubs = d;
  else if (strcmp(settingName, "serialTxRxDuringSleep") == 0)
    settings.serialTxRxDuringSleep = d;
  else if (strcmp(settingName, "printGNSSDebugMessages") == 0)
    settings.printGNSSDebugMessages = d;
  else if (strcmp(settingName, "openMenuWithPrintable") == 0)
    settings.openMenuWithPrintable = d;
  else
    {
      SerialPrintf2("Unknown setting %s. Ignoring...\r\n", settingName);
      return(false);
    }

  return (true);
}

//Export the current device settings to a config file
void recordDeviceSettingsToFile()
{
  if (online.microSD == true)
  {
    if (sd.exists("OLA_deviceSettings.txt"))
      sd.remove("OLA_deviceSettings.txt");

    #if SD_FAT_TYPE == 1
    File32 settingsFile;
    #elif SD_FAT_TYPE == 2
    ExFile settingsFile;
    #elif SD_FAT_TYPE == 3
    FsFile settingsFile;
    #else // SD_FAT_TYPE == 0
    File settingsFile;
    #endif  // SD_FAT_TYPE
    if (settingsFile.open("OLA_deviceSettings.txt", O_CREAT | O_APPEND | O_WRITE) == false)
    {
      SerialPrintln(F("Failed to create device settings file"));
      return;
    }

    //Step through the node list, recording each node's settings
    char base[75];
    node *temp = head;
    while (temp != NULL)
    {
      sprintf(base, "%s.%d.%d.%d.%d.", getDeviceName(temp->deviceType), temp->deviceType, temp->address, temp->muxAddress, temp->portNumber);

      switch (temp->deviceType)
      {
        case DEVICE_MULTIPLEXER:
          {
            //Currently, no settings for multiplexer to record
            //struct_multiplexer *nodeSetting = (struct_multiplexer *)temp->configPtr; //Create a local pointer that points to same spot as node does
            //settingsFile.println((String)base + "log=" + nodeSetting->log);
          }
          break;
        case DEVICE_LOADCELL_NAU7802:
          {
            struct_NAU7802 *nodeSetting = (struct_NAU7802 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.print((String)base + "calibrationFactor="); settingsFile.println(nodeSetting->calibrationFactor);
            settingsFile.println((String)base + "zeroOffset=" + nodeSetting->zeroOffset);
            settingsFile.println((String)base + "decimalPlaces=" + nodeSetting->decimalPlaces);
            settingsFile.println((String)base + "averageAmount=" + nodeSetting->averageAmount);
            settingsFile.print((String)base + "calibrationWeight="); settingsFile.println(nodeSetting->calibrationWeight);
            settingsFile.println((String)base + "sampleRate=" + nodeSetting->sampleRate);
            settingsFile.println((String)base + "gain=" + nodeSetting->gain);
            settingsFile.println((String)base + "LDO=" + nodeSetting->LDO);
            settingsFile.println((String)base + "calibrationMode=" + nodeSetting->calibrationMode);
            settingsFile.println((String)base + "offsetReg=" + nodeSetting->offsetReg);
            settingsFile.println((String)base + "gainReg=" + nodeSetting->gainReg);
          }
          break;
        case DEVICE_DISTANCE_VL53L1X:
          {
            struct_VL53L1X *nodeSetting = (struct_VL53L1X *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logDistance=" + nodeSetting->logDistance);
            settingsFile.println((String)base + "logRangeStatus=" + nodeSetting->logRangeStatus);
            settingsFile.println((String)base + "logSignalRate=" + nodeSetting->logSignalRate);
            settingsFile.println((String)base + "distanceMode=" + nodeSetting->distanceMode);
            settingsFile.println((String)base + "intermeasurementPeriod=" + nodeSetting->intermeasurementPeriod);
            settingsFile.println((String)base + "offset=" + nodeSetting->offset);
            settingsFile.println((String)base + "crosstalk=" + nodeSetting->crosstalk);
          }
          break;
        case DEVICE_GPS_UBLOX:
          {
            struct_ublox *nodeSetting = (struct_ublox *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logDate=" + nodeSetting->logDate);
            settingsFile.println((String)base + "logTime=" + nodeSetting->logTime);
            settingsFile.println((String)base + "logPosition=" + nodeSetting->logPosition);
            settingsFile.println((String)base + "logAltitude=" + nodeSetting->logAltitude);
            settingsFile.println((String)base + "logAltitudeMSL=" + nodeSetting->logAltitudeMSL);
            settingsFile.println((String)base + "logSIV=" + nodeSetting->logSIV);
            settingsFile.println((String)base + "logFixType=" + nodeSetting->logFixType);
            settingsFile.println((String)base + "logCarrierSolution=" + nodeSetting->logCarrierSolution);
            settingsFile.println((String)base + "logGroundSpeed=" + nodeSetting->logGroundSpeed);
            settingsFile.println((String)base + "logHeadingOfMotion=" + nodeSetting->logHeadingOfMotion);
            settingsFile.println((String)base + "logpDOP=" + nodeSetting->logpDOP);
            settingsFile.println((String)base + "logiTOW=" + nodeSetting->logiTOW);
            settingsFile.println((String)base + "i2cSpeed=" + nodeSetting->i2cSpeed);
            settingsFile.println((String)base + "useAutoPVT=" + nodeSetting->useAutoPVT);
          }
          break;
        case DEVICE_PROXIMITY_VCNL4040:
          {
            struct_VCNL4040 *nodeSetting = (struct_VCNL4040 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logProximity=" + nodeSetting->logProximity);
            settingsFile.println((String)base + "logAmbientLight=" + nodeSetting->logAmbientLight);
            settingsFile.println((String)base + "LEDCurrent=" + nodeSetting->LEDCurrent);
            settingsFile.println((String)base + "IRDutyCycle=" + nodeSetting->IRDutyCycle);
            settingsFile.println((String)base + "proximityIntegrationTime=" + nodeSetting->proximityIntegrationTime);
            settingsFile.println((String)base + "ambientIntegrationTime=" + nodeSetting->ambientIntegrationTime);
            settingsFile.println((String)base + "resolution=" + nodeSetting->resolution);
          }
          break;
        case DEVICE_TEMPERATURE_TMP117:
          {
            struct_TMP117 *nodeSetting = (struct_TMP117 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logTemperature=" + nodeSetting->logTemperature);
          }
          break;
        case DEVICE_PRESSURE_MS5637:
          {
            struct_MS5637 *nodeSetting = (struct_MS5637 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logPressure=" + nodeSetting->logPressure);
            settingsFile.println((String)base + "logTemperature=" + nodeSetting->logTemperature);
          }
          break;
        case DEVICE_PRESSURE_LPS25HB:
          {
            struct_LPS25HB *nodeSetting = (struct_LPS25HB *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logPressure=" + nodeSetting->logPressure);
            settingsFile.println((String)base + "logTemperature=" + nodeSetting->logTemperature);
          }
          break;
        case DEVICE_PRESSURE_LPS28DFW:
          {
            struct_LPS28DFW *nodeSetting = (struct_LPS28DFW *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logPressure=" + nodeSetting->logPressure);
            settingsFile.println((String)base + "logTemperature=" + nodeSetting->logTemperature);
          }
          break;
        case DEVICE_PHT_BME280:
          {
            struct_BME280 *nodeSetting = (struct_BME280 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logHumidity=" + nodeSetting->logHumidity);
            settingsFile.println((String)base + "logPressure=" + nodeSetting->logPressure);
            settingsFile.println((String)base + "logAltitude=" + nodeSetting->logAltitude);
            settingsFile.println((String)base + "logTemperature=" + nodeSetting->logTemperature);
          }
          break;
        case DEVICE_UV_VEML6075:
          {
            struct_VEML6075 *nodeSetting = (struct_VEML6075 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logUVA=" + nodeSetting->logUVA);
            settingsFile.println((String)base + "logUVB=" + nodeSetting->logUVB);
            settingsFile.println((String)base + "logUVIndex=" + nodeSetting->logUVIndex);
          }
          break;
        case DEVICE_LIGHT_VEML7700:
          {
            struct_VEML7700 *nodeSetting = (struct_VEML7700 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
          }
          break;
        case DEVICE_VOC_CCS811:
          {
            struct_CCS811 *nodeSetting = (struct_CCS811 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logTVOC=" + nodeSetting->logTVOC);
            settingsFile.println((String)base + "logCO2=" + nodeSetting->logCO2);
          }
          break;
        case DEVICE_VOC_SGP30:
          {
            struct_SGP30 *nodeSetting = (struct_SGP30 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logTVOC=" + nodeSetting->logTVOC);
            settingsFile.println((String)base + "logCO2=" + nodeSetting->logCO2);
            settingsFile.println((String)base + "logH2=" + nodeSetting->logH2);
            settingsFile.println((String)base + "logEthanol=" + nodeSetting->logEthanol);
          }
          break;
        case DEVICE_CO2_SCD30:
          {
            struct_SCD30 *nodeSetting = (struct_SCD30 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logCO2=" + nodeSetting->logCO2);
            settingsFile.println((String)base + "logHumidity=" + nodeSetting->logHumidity);
            settingsFile.println((String)base + "logTemperature=" + nodeSetting->logTemperature);
            settingsFile.println((String)base + "measurementInterval=" + nodeSetting->measurementInterval);
            settingsFile.println((String)base + "altitudeCompensation=" + nodeSetting->altitudeCompensation);
            settingsFile.println((String)base + "ambientPressure=" + nodeSetting->ambientPressure);
            settingsFile.println((String)base + "temperatureOffset=" + nodeSetting->temperatureOffset);
          }
          break;
        case DEVICE_PHT_MS8607:
          {
            struct_MS8607 *nodeSetting = (struct_MS8607 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logHumidity=" + nodeSetting->logHumidity);
            settingsFile.println((String)base + "logPressure=" + nodeSetting->logPressure);
            settingsFile.println((String)base + "logTemperature=" + nodeSetting->logTemperature);
            settingsFile.println((String)base + "enableHeater=" + nodeSetting->enableHeater);
            settingsFile.println((String)base + "pressureResolution=" + nodeSetting->pressureResolution);
            settingsFile.println((String)base + "humidityResolution=" + nodeSetting->humidityResolution);
          }
          break;
        case DEVICE_TEMPERATURE_MCP9600:
          {
            struct_MCP9600 *nodeSetting = (struct_MCP9600 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logTemperature=" + nodeSetting->logTemperature);
            settingsFile.println((String)base + "logAmbientTemperature=" + nodeSetting->logAmbientTemperature);
          }
          break;
        case DEVICE_HUMIDITY_AHT20:
          {
            struct_AHT20 *nodeSetting = (struct_AHT20 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logHumidity=" + nodeSetting->logHumidity);
            settingsFile.println((String)base + "logTemperature=" + nodeSetting->logTemperature);
          }
          break;
        case DEVICE_HUMIDITY_SHTC3:
          {
            struct_SHTC3 *nodeSetting = (struct_SHTC3 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logHumidity=" + nodeSetting->logHumidity);
            settingsFile.println((String)base + "logTemperature=" + nodeSetting->logTemperature);
          }
          break;
        case DEVICE_ADC_ADS122C04:
          {
            struct_ADS122C04 *nodeSetting = (struct_ADS122C04 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logCentigrade=" + nodeSetting->logCentigrade);
            settingsFile.println((String)base + "logFahrenheit=" + nodeSetting->logFahrenheit);
            settingsFile.println((String)base + "logInternalTemperature=" + nodeSetting->logInternalTemperature);
            settingsFile.println((String)base + "logRawVoltage=" + nodeSetting->logRawVoltage);
            settingsFile.println((String)base + "useFourWireMode=" + nodeSetting->useFourWireMode);
            settingsFile.println((String)base + "useThreeWireMode=" + nodeSetting->useThreeWireMode);
            settingsFile.println((String)base + "useTwoWireMode=" + nodeSetting->useTwoWireMode);
            settingsFile.println((String)base + "useFourWireHighTemperatureMode=" + nodeSetting->useFourWireHighTemperatureMode);
            settingsFile.println((String)base + "useThreeWireHighTemperatureMode=" + nodeSetting->useThreeWireHighTemperatureMode);
            settingsFile.println((String)base + "useTwoWireHighTemperatureMode=" + nodeSetting->useTwoWireHighTemperatureMode);
          }
          break;
        case DEVICE_PRESSURE_MPR0025PA1:
          {
            struct_MPR0025PA1 *nodeSetting = (struct_MPR0025PA1 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "minimumPSI=" + nodeSetting->minimumPSI);
            settingsFile.println((String)base + "maximumPSI=" + nodeSetting->maximumPSI);
            settingsFile.println((String)base + "usePSI=" + nodeSetting->usePSI);
            settingsFile.println((String)base + "usePA=" + nodeSetting->usePA);
            settingsFile.println((String)base + "useKPA=" + nodeSetting->useKPA);
            settingsFile.println((String)base + "useTORR=" + nodeSetting->useTORR);
            settingsFile.println((String)base + "useINHG=" + nodeSetting->useINHG);
            settingsFile.println((String)base + "useATM=" + nodeSetting->useATM);
            settingsFile.println((String)base + "useBAR=" + nodeSetting->useBAR);
          }
          break;
        case DEVICE_PARTICLE_SNGCJA5:
          {
            struct_SNGCJA5 *nodeSetting = (struct_SNGCJA5 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logPM1=" + nodeSetting->logPM1);
            settingsFile.println((String)base + "logPM25=" + nodeSetting->logPM25);
            settingsFile.println((String)base + "logPM10=" + nodeSetting->logPM10);
            settingsFile.println((String)base + "logPC05=" + nodeSetting->logPC05);
            settingsFile.println((String)base + "logPC1=" + nodeSetting->logPC1);
            settingsFile.println((String)base + "logPC25=" + nodeSetting->logPC25);
            settingsFile.println((String)base + "logPC50=" + nodeSetting->logPC50);
            settingsFile.println((String)base + "logPC75=" + nodeSetting->logPC75);
            settingsFile.println((String)base + "logPC10=" + nodeSetting->logPC10);
            settingsFile.println((String)base + "logSensorStatus=" + nodeSetting->logSensorStatus);
            settingsFile.println((String)base + "logPDStatus=" + nodeSetting->logPDStatus);
            settingsFile.println((String)base + "logLDStatus=" + nodeSetting->logLDStatus);
            settingsFile.println((String)base + "logFanStatus=" + nodeSetting->logFanStatus);
          }
          break;
        case DEVICE_VOC_SGP40:
          {
            struct_SGP40 *nodeSetting = (struct_SGP40 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logVOC=" + nodeSetting->logVOC);
            settingsFile.print((String)base + "RH="); settingsFile.println(nodeSetting->RH);
            settingsFile.print((String)base + "T="); settingsFile.println(nodeSetting->T);
          }
          break;
        case DEVICE_PRESSURE_SDP3X:
          {
            struct_SDP3X *nodeSetting = (struct_SDP3X *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logPressure=" + nodeSetting->logPressure);
            settingsFile.println((String)base + "logTemperature=" + nodeSetting->logTemperature);
            settingsFile.println((String)base + "massFlow=" + nodeSetting->massFlow);
            settingsFile.println((String)base + "averaging=" + nodeSetting->averaging);
          }
          break;
        case DEVICE_PRESSURE_MS5837:
          {
            struct_MS5837 *nodeSetting = (struct_MS5837 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logPressure=" + nodeSetting->logPressure);
            settingsFile.println((String)base + "logTemperature=" + nodeSetting->logTemperature);
            settingsFile.println((String)base + "logDepth=" + nodeSetting->logDepth);
            settingsFile.println((String)base + "logAltitude=" + nodeSetting->logAltitude);
            settingsFile.println((String)base + "model=" + nodeSetting->model);
            settingsFile.print((String)base + "fluidDensity="); settingsFile.println(nodeSetting->fluidDensity);
            settingsFile.print((String)base + "conversion="); settingsFile.println(nodeSetting->conversion);
          }
          break;
        case DEVICE_QWIIC_BUTTON:
          {
            struct_QWIIC_BUTTON *nodeSetting = (struct_QWIIC_BUTTON *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logPressed=" + nodeSetting->logPressed);
            settingsFile.println((String)base + "logClicked=" + nodeSetting->logClicked);
            settingsFile.println((String)base + "toggleLEDOnClick=" + nodeSetting->toggleLEDOnClick);
            settingsFile.println((String)base + "ledBrightness=" + nodeSetting->ledBrightness);
          }
          break;
        case DEVICE_BIO_SENSOR_HUB:
          {
            struct_BIO_SENSOR_HUB *nodeSetting = (struct_BIO_SENSOR_HUB *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logHeartrate=" + nodeSetting->logHeartrate);
            settingsFile.println((String)base + "logConfidence=" + nodeSetting->logConfidence);
            settingsFile.println((String)base + "logOxygen=" + nodeSetting->logOxygen);
            settingsFile.println((String)base + "logStatus=" + nodeSetting->logStatus);
            settingsFile.println((String)base + "logExtendedStatus=" + nodeSetting->logExtendedStatus);
            settingsFile.println((String)base + "logRValue=" + nodeSetting->logRValue);
          }
          break;
        case DEVICE_ISM330DHCX:
          {
            struct_ISM330DHCX *nodeSetting = (struct_ISM330DHCX *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logAccel=" + nodeSetting->logAccel);
            settingsFile.println((String)base + "logGyro=" + nodeSetting->logGyro);
            settingsFile.println((String)base + "logDataReady=" + nodeSetting->logDataReady);
            settingsFile.println((String)base + "accelScale=" + nodeSetting->accelScale);
            settingsFile.println((String)base + "gyroScale=" + nodeSetting->gyroScale);
            settingsFile.println((String)base + "accelRate=" + nodeSetting->accelRate);
            settingsFile.println((String)base + "gyroRate=" + nodeSetting->gyroRate);
            settingsFile.println((String)base + "accelFilterLP2=" + nodeSetting->accelFilterLP2);
            settingsFile.println((String)base + "gyroFilterLP1=" + nodeSetting->gyroFilterLP1);
            settingsFile.println((String)base + "gyroLP1BW=" + nodeSetting->gyroLP1BW);
            settingsFile.println((String)base + "accelSlopeFilter=" + nodeSetting->accelSlopeFilter);
          }
          break;
        case DEVICE_MMC5983MA:
          {
            struct_MMC5983MA *nodeSetting = (struct_MMC5983MA *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logMag=" + nodeSetting->logMag);
            settingsFile.println((String)base + "logTemperature=" + nodeSetting->logTemperature);
          }
          break;
        case DEVICE_KX134:
          {
            struct_KX134 *nodeSetting = (struct_KX134 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logAccel=" + nodeSetting->logAccel);
            settingsFile.println((String)base + "logDataReady=" + nodeSetting->logDataReady);
            settingsFile.println((String)base + "range8G=" + nodeSetting->range8G);
            settingsFile.println((String)base + "range16G=" + nodeSetting->range16G);
            settingsFile.println((String)base + "range32G=" + nodeSetting->range32G);
            settingsFile.println((String)base + "range64G=" + nodeSetting->range64G);
            settingsFile.println((String)base + "highSpeed=" + nodeSetting->highSpeed);
          }
          break;
        case DEVICE_ADS1015:
          {
            struct_ADS1015 *nodeSetting = (struct_ADS1015 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
            settingsFile.println((String)base + "logA0=" + nodeSetting->logA0);
            settingsFile.println((String)base + "logA1=" + nodeSetting->logA1);
            settingsFile.println((String)base + "logA2=" + nodeSetting->logA2);
            settingsFile.println((String)base + "logA3=" + nodeSetting->logA3);
            settingsFile.println((String)base + "logA0A1=" + nodeSetting->logA0A1);
            settingsFile.println((String)base + "logA0A3=" + nodeSetting->logA0A3);
            settingsFile.println((String)base + "logA1A3=" + nodeSetting->logA1A3);
            settingsFile.println((String)base + "logA2A3=" + nodeSetting->logA2A3);
            settingsFile.println((String)base + "gain23=" + nodeSetting->gain23);
            settingsFile.println((String)base + "gain1=" + nodeSetting->gain1);
            settingsFile.println((String)base + "gain2=" + nodeSetting->gain2);
            settingsFile.println((String)base + "gain4=" + nodeSetting->gain4);
            settingsFile.println((String)base + "gain8=" + nodeSetting->gain8);
            settingsFile.println((String)base + "gain16=" + nodeSetting->gain16);
          }
          break;
        case DEVICE_PCF8575:
          {
            struct_PCF8575 *nodeSetting = (struct_PCF8575 *)temp->configPtr;
            settingsFile.println((String)base + "log=" + nodeSetting->log);
          }
          break;
        default:
          SerialPrintf2("recordSettingsToFile Unknown device: %s\r\n", base);
          //settingsFile.println((String)base + "=UnknownDeviceSettings");
          break;
      }
      temp = temp->next;
    }
    updateDataFileAccess(&settingsFile); // Update the file access time & date
    settingsFile.close();
  }
}

//If a device config file exists on the SD card, load them and overwrite the local settings
//Heavily based on ReadCsvFile from SdFat library
//Returns true if some settings were loaded from a file
//Returns false if a file was not opened/loaded
bool loadDeviceSettingsFromFile()
{
  if (online.microSD == true)
  {
    if (sd.exists("OLA_deviceSettings.txt"))
    {
      #if SD_FAT_TYPE == 1
      File32 settingsFile;
      #elif SD_FAT_TYPE == 2
      ExFile settingsFile;
      #elif SD_FAT_TYPE == 3
      FsFile settingsFile;
      #else // SD_FAT_TYPE == 0
      File settingsFile;
      #endif  // SD_FAT_TYPE

      if (settingsFile.open("OLA_deviceSettings.txt", O_READ) == false)
      {
        SerialPrintln(F("Failed to open device settings file"));
        return (false);
      }

      char line[150];
      int lineNumber = 0;

      while (settingsFile.available()) {
        int n = settingsFile.fgets(line, sizeof(line));
        if (n <= 0) {
          SerialPrintf2("Failed to read line %d from settings file\r\n", lineNumber);
        }
        else if (line[n - 1] != '\n' && n == (sizeof(line) - 1)) {
          SerialPrintf2("Settings line %d too long\n", lineNumber);
        }
        else if (parseDeviceLine(line) == false) {
          SerialPrintf3("Failed to parse line %d: %s\r\n", lineNumber + 1, line);
        }

        lineNumber++;
      }

      //SerialPrintln(F("Device config file read complete"));
      //updateDataFileAccess(&settingsFile); // Update the file access time & date
      settingsFile.close();
      return (true);
    }
    else
    {
      SerialPrintln(F("No device config file found. Creating one with device defaults."));
      recordDeviceSettingsToFile(); //Record the current settings to create the initial file
      return (false);
    }
  }

  SerialPrintln(F("Device config file read failed: SD offline"));
  return (false); //SD offline
}

//Convert a given line from device setting file into a settingName and value
//Immediately applies the setting to the appropriate node
bool parseDeviceLine(char* str) {
  char* ptr;

  //Debug
  //SerialPrintf2("Line contents: %s", str);
  //SerialFlush();

  // Set strtok start of line.
  str = strtok(str, "=");
  if (!str) return false;

  //Store this setting name
  char settingName[150];
  sprintf(settingName, "%s", str);

  //Move pointer to end of line
  str = strtok(nullptr, "\n");
  if (!str) return false;

  //SerialPrintf2("s = %s\r\n", str);
  //SerialFlush();

  // Convert string to double.
  double d = strtod(str, &ptr);
  if (str == ptr || *skipSpace(ptr)) return false;

  //SerialPrintf2("d = %lf\r\n", d);
  //SerialFlush();

  //Break device setting into its constituent parts
  char deviceSettingName[50];
  deviceType_e deviceType;
  uint8_t address;
  uint8_t muxAddress;
  uint8_t portNumber;
  uint8_t count = 0;
  char *split = strtok(settingName, ".");
  while (split != NULL)
  {
    if (count == 0)
      ; //Do nothing. This is merely the human friendly device name
    else if (count == 1)
      deviceType = (deviceType_e)atoi(split);
    else if (count == 2)
      address = atoi(split);
    else if (count == 3)
      muxAddress = atoi(split);
    else if (count == 4)
      portNumber = atoi(split);
    else if (count == 5)
      sprintf(deviceSettingName, "%s", split);
    split = strtok(NULL, ".");
    count++;
  }

  if (count < 5)
  {
    SerialPrintf2("Incomplete setting: %s\r\n", settingName);
    return false;
  }

  //SerialPrintf6("%d: %d.%d.%d - %s\r\n", deviceType, address, muxAddress, portNumber, deviceSettingName);
  //SerialFlush();

  //Find the device in the list that has this device type and address
  void *deviceConfigPtr = getConfigPointer(deviceType, address, muxAddress, portNumber);
  if (deviceConfigPtr == NULL)
  {
    //SerialPrintf2("Setting in file found but no matching device on bus is available: %s\r\n", settingName);
    //SerialFlush();
  }
  else
  {
    switch (deviceType)
    {
      case DEVICE_MULTIPLEXER:
        {
          SerialPrintln(F("There are no known settings for a multiplexer to load."));
        }
        break;
      case DEVICE_LOADCELL_NAU7802:
        {
          struct_NAU7802 *nodeSetting = (struct_NAU7802 *)deviceConfigPtr;

          //Apply the appropriate settings
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "calibrationFactor") == 0)
            nodeSetting->calibrationFactor = d;
          else if (strcmp(deviceSettingName, "zeroOffset") == 0)
            nodeSetting->zeroOffset = d;
          else if (strcmp(deviceSettingName, "decimalPlaces") == 0)
            nodeSetting->decimalPlaces = d;
          else if (strcmp(deviceSettingName, "averageAmount") == 0)
            nodeSetting->averageAmount = d;
          else if (strcmp(deviceSettingName, "calibrationWeight") == 0)
            nodeSetting->calibrationWeight = d;
          else if (strcmp(deviceSettingName, "sampleRate") == 0)
            nodeSetting->sampleRate = d;
          else if (strcmp(deviceSettingName, "gain") == 0)
            nodeSetting->gain = d;
          else if (strcmp(deviceSettingName, "LDO") == 0)
            nodeSetting->LDO = d;
          else if (strcmp(deviceSettingName, "calibrationMode") == 0)
            nodeSetting->calibrationMode = d;
          else if (strcmp(deviceSettingName, "offsetReg") == 0)
            nodeSetting->offsetReg = d;
          else if (strcmp(deviceSettingName, "gainReg") == 0)
            nodeSetting->gainReg = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_DISTANCE_VL53L1X:
        {
          struct_VL53L1X *nodeSetting = (struct_VL53L1X *)deviceConfigPtr;
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logDistance") == 0)
            nodeSetting->logDistance = d;
          else if (strcmp(deviceSettingName, "logRangeStatus") == 0)
            nodeSetting->logRangeStatus = d;
          else if (strcmp(deviceSettingName, "logSignalRate") == 0)
            nodeSetting->logSignalRate = d;
          else if (strcmp(deviceSettingName, "distanceMode") == 0)
            nodeSetting->distanceMode = d;
          else if (strcmp(deviceSettingName, "intermeasurementPeriod") == 0)
            nodeSetting->intermeasurementPeriod = d;
          else if (strcmp(deviceSettingName, "offset") == 0)
            nodeSetting->offset = d;
          else if (strcmp(deviceSettingName, "crosstalk") == 0)
            nodeSetting->crosstalk = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_GPS_UBLOX:
        {
          struct_ublox *nodeSetting = (struct_ublox *)deviceConfigPtr;

          //Apply the appropriate settings
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logDate") == 0)
            nodeSetting->logDate = d;
          else if (strcmp(deviceSettingName, "logTime") == 0)
            nodeSetting->logTime = d;
          else if (strcmp(deviceSettingName, "logPosition") == 0)
            nodeSetting->logPosition = d;
          else if (strcmp(deviceSettingName, "logAltitude") == 0)
            nodeSetting->logAltitude = d;
          else if (strcmp(deviceSettingName, "logAltitudeMSL") == 0)
            nodeSetting->logAltitudeMSL = d;
          else if (strcmp(deviceSettingName, "logSIV") == 0)
            nodeSetting->logSIV = d;
          else if (strcmp(deviceSettingName, "logFixType") == 0)
            nodeSetting->logFixType = d;
          else if (strcmp(deviceSettingName, "logCarrierSolution") == 0)
            nodeSetting->logCarrierSolution = d;
          else if (strcmp(deviceSettingName, "logGroundSpeed") == 0)
            nodeSetting->logGroundSpeed = d;
          else if (strcmp(deviceSettingName, "logHeadingOfMotion") == 0)
            nodeSetting->logHeadingOfMotion = d;
          else if (strcmp(deviceSettingName, "logpDOP") == 0)
            nodeSetting->logpDOP = d;
          else if (strcmp(deviceSettingName, "logiTOW") == 0)
            nodeSetting->logiTOW = d;
          else if (strcmp(deviceSettingName, "i2cSpeed") == 0)
            nodeSetting->i2cSpeed = d;
          else if (strcmp(deviceSettingName, "useAutoPVT") == 0)
            nodeSetting->useAutoPVT = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_PROXIMITY_VCNL4040:
        {
          struct_VCNL4040 *nodeSetting = (struct_VCNL4040 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logProximity") == 0)
            nodeSetting->logProximity = d;
          else if (strcmp(deviceSettingName, "logAmbientLight") == 0)
            nodeSetting->logAmbientLight = d;
          else if (strcmp(deviceSettingName, "LEDCurrent") == 0)
            nodeSetting->LEDCurrent = d;
          else if (strcmp(deviceSettingName, "IRDutyCycle") == 0)
            nodeSetting->IRDutyCycle = d;
          else if (strcmp(deviceSettingName, "proximityIntegrationTime") == 0)
            nodeSetting->proximityIntegrationTime = d;
          else if (strcmp(deviceSettingName, "ambientIntegrationTime") == 0)
            nodeSetting->ambientIntegrationTime = d;
          else if (strcmp(deviceSettingName, "resolution") == 0)
            nodeSetting->resolution = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_TEMPERATURE_TMP117:
        {
          struct_TMP117 *nodeSetting = (struct_TMP117 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logTemperature") == 0)
            nodeSetting->logTemperature = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_PRESSURE_MS5637:
        {
          struct_MS5637 *nodeSetting = (struct_MS5637 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logPressure") == 0)
            nodeSetting->logPressure = d;
          else if (strcmp(deviceSettingName, "logTemperature") == 0)
            nodeSetting->logTemperature = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_PRESSURE_LPS25HB:
        {
          struct_LPS25HB *nodeSetting = (struct_LPS25HB *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logPressure") == 0)
            nodeSetting->logPressure = d;
          else if (strcmp(deviceSettingName, "logTemperature") == 0)
            nodeSetting->logTemperature = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_PRESSURE_LPS28DFW:
        {
          struct_LPS28DFW *nodeSetting = (struct_LPS28DFW *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logPressure") == 0)
            nodeSetting->logPressure = d;
          else if (strcmp(deviceSettingName, "logTemperature") == 0)
            nodeSetting->logTemperature = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_PHT_BME280:
        {
          struct_BME280 *nodeSetting = (struct_BME280 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logHumidity") == 0)
            nodeSetting->logHumidity = d;
          else if (strcmp(deviceSettingName, "logPressure") == 0)
            nodeSetting->logPressure = d;
          else if (strcmp(deviceSettingName, "logAltitude") == 0)
            nodeSetting->logAltitude = d;
          else if (strcmp(deviceSettingName, "logTemperature") == 0)
            nodeSetting->logTemperature = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_UV_VEML6075:
        {
          struct_VEML6075 *nodeSetting = (struct_VEML6075 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logUVA") == 0)
            nodeSetting->logUVA = d;
          else if (strcmp(deviceSettingName, "logUVB") == 0)
            nodeSetting->logUVB = d;
          else if (strcmp(deviceSettingName, "logUVIndex") == 0)
            nodeSetting->logUVIndex = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_LIGHT_VEML7700:
        {
          struct_VEML7700 *nodeSetting = (struct_VEML7700 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_VOC_CCS811:
        {
          struct_CCS811 *nodeSetting = (struct_CCS811 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logTVOC") == 0)
            nodeSetting->logTVOC = d;
          else if (strcmp(deviceSettingName, "logCO2") == 0)
            nodeSetting->logCO2 = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_VOC_SGP30:
        {
          struct_SGP30 *nodeSetting = (struct_SGP30 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logTVOC") == 0)
            nodeSetting->logTVOC = d;
          else if (strcmp(deviceSettingName, "logCO2") == 0)
            nodeSetting->logCO2 = d;
          else if (strcmp(deviceSettingName, "logH2") == 0)
            nodeSetting->logH2 = d;
          else if (strcmp(deviceSettingName, "logEthanol") == 0)
            nodeSetting->logEthanol = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_CO2_SCD30:
        {
          struct_SCD30 *nodeSetting = (struct_SCD30 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logCO2") == 0)
            nodeSetting->logCO2 = d;
          else if (strcmp(deviceSettingName, "logHumidity") == 0)
            nodeSetting->logHumidity = d;
          else if (strcmp(deviceSettingName, "logTemperature") == 0)
            nodeSetting->logTemperature = d;
          else if (strcmp(deviceSettingName, "measurementInterval") == 0)
            nodeSetting->measurementInterval = d;
          else if (strcmp(deviceSettingName, "altitudeCompensation") == 0)
            nodeSetting->altitudeCompensation = d;
          else if (strcmp(deviceSettingName, "ambientPressure") == 0)
            nodeSetting->ambientPressure = d;
          else if (strcmp(deviceSettingName, "temperatureOffset") == 0)
            nodeSetting->temperatureOffset = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_PHT_MS8607:
        {
          struct_MS8607 *nodeSetting = (struct_MS8607 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logHumidity") == 0)
            nodeSetting->logHumidity = d;
          else if (strcmp(deviceSettingName, "logPressure") == 0)
            nodeSetting->logPressure = d;
          else if (strcmp(deviceSettingName, "logTemperature") == 0)
            nodeSetting->logTemperature = d;
          else if (strcmp(deviceSettingName, "enableHeater") == 0)
            nodeSetting->enableHeater = d;
          else if (strcmp(deviceSettingName, "pressureResolution") == 0)
            nodeSetting->pressureResolution = (MS8607_pressure_resolution)d;
          else if (strcmp(deviceSettingName, "humidityResolution") == 0)
            nodeSetting->humidityResolution = (MS8607_humidity_resolution)d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_TEMPERATURE_MCP9600:
        {
          struct_MCP9600 *nodeSetting = (struct_MCP9600 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logTemperature") == 0)
            nodeSetting->logTemperature = d;
          else if (strcmp(deviceSettingName, "logAmbientTemperature") == 0)
            nodeSetting->logAmbientTemperature = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_HUMIDITY_AHT20:
        {
          struct_AHT20 *nodeSetting = (struct_AHT20 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logHumidity") == 0)
            nodeSetting->logHumidity = d;
          else if (strcmp(deviceSettingName, "logTemperature") == 0)
            nodeSetting->logTemperature = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_HUMIDITY_SHTC3:
        {
          struct_SHTC3 *nodeSetting = (struct_SHTC3 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logHumidity") == 0)
            nodeSetting->logHumidity = d;
          else if (strcmp(deviceSettingName, "logTemperature") == 0)
            nodeSetting->logTemperature = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_ADC_ADS122C04:
        {
          struct_ADS122C04 *nodeSetting = (struct_ADS122C04 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logCentigrade") == 0)
            nodeSetting->logCentigrade = d;
          else if (strcmp(deviceSettingName, "logFahrenheit") == 0)
            nodeSetting->logFahrenheit = d;
          else if (strcmp(deviceSettingName, "logInternalTemperature") == 0)
            nodeSetting->logInternalTemperature = d;
          else if (strcmp(deviceSettingName, "logRawVoltage") == 0)
            nodeSetting->logRawVoltage = d;
          else if (strcmp(deviceSettingName, "useFourWireMode") == 0)
            nodeSetting->useFourWireMode = d;
          else if (strcmp(deviceSettingName, "useThreeWireMode") == 0)
            nodeSetting->useThreeWireMode = d;
          else if (strcmp(deviceSettingName, "useTwoWireMode") == 0)
            nodeSetting->useTwoWireMode = d;
          else if (strcmp(deviceSettingName, "useFourWireHighTemperatureMode") == 0)
            nodeSetting->useFourWireHighTemperatureMode = d;
          else if (strcmp(deviceSettingName, "useThreeWireHighTemperatureMode") == 0)
            nodeSetting->useThreeWireHighTemperatureMode = d;
          else if (strcmp(deviceSettingName, "useTwoWireHighTemperatureMode") == 0)
            nodeSetting->useTwoWireHighTemperatureMode = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_PRESSURE_MPR0025PA1:
        {
          struct_MPR0025PA1 *nodeSetting = (struct_MPR0025PA1 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "minimumPSI") == 0)
            nodeSetting->minimumPSI = d;
          else if (strcmp(deviceSettingName, "maximumPSI") == 0)
            nodeSetting->maximumPSI = d;
          else if (strcmp(deviceSettingName, "usePSI") == 0)
            nodeSetting->usePSI = d;
          else if (strcmp(deviceSettingName, "usePA") == 0)
            nodeSetting->usePA = d;
          else if (strcmp(deviceSettingName, "useKPA") == 0)
            nodeSetting->useKPA = d;
          else if (strcmp(deviceSettingName, "useTORR") == 0)
            nodeSetting->useTORR = d;
          else if (strcmp(deviceSettingName, "useINHG") == 0)
            nodeSetting->useINHG = d;
          else if (strcmp(deviceSettingName, "useATM") == 0)
            nodeSetting->useATM = d;
          else if (strcmp(deviceSettingName, "useBAR") == 0)
            nodeSetting->useBAR = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_PARTICLE_SNGCJA5:
        {
          struct_SNGCJA5 *nodeSetting = (struct_SNGCJA5 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logPM1") == 0)
            nodeSetting->logPM1 = d;
          else if (strcmp(deviceSettingName, "logPM25") == 0)
            nodeSetting->logPM25 = d;
          else if (strcmp(deviceSettingName, "logPM10") == 0)
            nodeSetting->logPM10 = d;
          else if (strcmp(deviceSettingName, "logPC05") == 0)
            nodeSetting->logPC05 = d;
          else if (strcmp(deviceSettingName, "logPC1") == 0)
            nodeSetting->logPC1 = d;
          else if (strcmp(deviceSettingName, "logPC25") == 0)
            nodeSetting->logPC25 = d;
          else if (strcmp(deviceSettingName, "logPC50") == 0)
            nodeSetting->logPC50 = d;
          else if (strcmp(deviceSettingName, "logPC75") == 0)
            nodeSetting->logPC75 = d;
          else if (strcmp(deviceSettingName, "logPC10") == 0)
            nodeSetting->logPC10 = d;
          else if (strcmp(deviceSettingName, "logSensorStatus") == 0)
            nodeSetting->logSensorStatus = d;
          else if (strcmp(deviceSettingName, "logPDStatus") == 0)
            nodeSetting->logPDStatus = d;
          else if (strcmp(deviceSettingName, "logLDStatus") == 0)
            nodeSetting->logLDStatus = d;
          else if (strcmp(deviceSettingName, "logFanStatus") == 0)
            nodeSetting->logFanStatus = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_VOC_SGP40:
        {
          struct_SGP40 *nodeSetting = (struct_SGP40 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logVOC") == 0)
            nodeSetting->logVOC = d;
          else if (strcmp(deviceSettingName, "RH") == 0)
            nodeSetting->RH = d;
          else if (strcmp(deviceSettingName, "T") == 0)
            nodeSetting->T = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_PRESSURE_SDP3X:
        {
          struct_SDP3X *nodeSetting = (struct_SDP3X *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logPressure") == 0)
            nodeSetting->logPressure = d;
          else if (strcmp(deviceSettingName, "logTemperature") == 0)
            nodeSetting->logTemperature = d;
          else if (strcmp(deviceSettingName, "massFlow") == 0)
            nodeSetting->massFlow = d;
          else if (strcmp(deviceSettingName, "averaging") == 0)
            nodeSetting->averaging = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_PRESSURE_MS5837:
        {
          struct_MS5837 *nodeSetting = (struct_MS5837 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logPressure") == 0)
            nodeSetting->logPressure = d;
          else if (strcmp(deviceSettingName, "logTemperature") == 0)
            nodeSetting->logTemperature = d;
          else if (strcmp(deviceSettingName, "logDepth") == 0)
            nodeSetting->logDepth = d;
          else if (strcmp(deviceSettingName, "logAltitude") == 0)
            nodeSetting->logAltitude = d;
          else if (strcmp(deviceSettingName, "model") == 0)
            nodeSetting->model = d;
          else if (strcmp(deviceSettingName, "fluidDensity") == 0)
            nodeSetting->fluidDensity = d;
          else if (strcmp(deviceSettingName, "conversion") == 0)
            nodeSetting->conversion = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_QWIIC_BUTTON:
        {
          struct_QWIIC_BUTTON *nodeSetting = (struct_QWIIC_BUTTON *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logPressed") == 0)
            nodeSetting->logPressed = d;
          else if (strcmp(deviceSettingName, "logClicked") == 0)
            nodeSetting->logClicked = d;
          else if (strcmp(deviceSettingName, "toggleLEDOnClick") == 0)
            nodeSetting->toggleLEDOnClick = d;
          else if (strcmp(deviceSettingName, "ledBrightness") == 0)
            nodeSetting->ledBrightness = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_BIO_SENSOR_HUB:
        {
          struct_BIO_SENSOR_HUB *nodeSetting = (struct_BIO_SENSOR_HUB *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logHeartrate") == 0)
            nodeSetting->logHeartrate = d;
          else if (strcmp(deviceSettingName, "logConfidence") == 0)
            nodeSetting->logConfidence = d;
          else if (strcmp(deviceSettingName, "logOxygen") == 0)
            nodeSetting->logOxygen = d;
          else if (strcmp(deviceSettingName, "logStatus") == 0)
            nodeSetting->logStatus = d;
          else if (strcmp(deviceSettingName, "logExtendedStatus") == 0)
            nodeSetting->logExtendedStatus = d;
          else if (strcmp(deviceSettingName, "logRValue") == 0)
            nodeSetting->logRValue = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_ISM330DHCX:
        {
          struct_ISM330DHCX *nodeSetting = (struct_ISM330DHCX *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logAccel") == 0)
            nodeSetting->logAccel = d;
          else if (strcmp(deviceSettingName, "logGyro") == 0)
            nodeSetting->logGyro = d;
          else if (strcmp(deviceSettingName, "logDataReady") == 0)
            nodeSetting->logDataReady = d;
          else if (strcmp(deviceSettingName, "accelScale") == 0)
            nodeSetting->accelScale = d;
          else if (strcmp(deviceSettingName, "gyroScale") == 0)
            nodeSetting->gyroScale = d;
          else if (strcmp(deviceSettingName, "accelRate") == 0)
            nodeSetting->accelRate = d;
          else if (strcmp(deviceSettingName, "gyroRate") == 0)
            nodeSetting->gyroRate = d;
          else if (strcmp(deviceSettingName, "accelFilterLP2") == 0)
            nodeSetting->accelFilterLP2 = d;
          else if (strcmp(deviceSettingName, "gyroFilterLP1") == 0)
            nodeSetting->gyroFilterLP1 = d;
          else if (strcmp(deviceSettingName, "gyroLP1BW") == 0)
            nodeSetting->gyroLP1BW = d;
          else if (strcmp(deviceSettingName, "accelSlopeFilter") == 0)
            nodeSetting->accelSlopeFilter = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_MMC5983MA:
        {
          struct_MMC5983MA *nodeSetting = (struct_MMC5983MA *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logMag") == 0)
            nodeSetting->logMag = d;
          else if (strcmp(deviceSettingName, "logTemperature") == 0)
            nodeSetting->logTemperature = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_KX134:
        {
          struct_KX134 *nodeSetting = (struct_KX134 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logAccel") == 0)
            nodeSetting->logAccel = d;
          else if (strcmp(deviceSettingName, "logDataReady") == 0)
            nodeSetting->logDataReady = d;
          else if (strcmp(deviceSettingName, "range8G") == 0)
            nodeSetting->range8G = d;
          else if (strcmp(deviceSettingName, "range16G") == 0)
            nodeSetting->range16G = d;
          else if (strcmp(deviceSettingName, "range32G") == 0)
            nodeSetting->range32G = d;
          else if (strcmp(deviceSettingName, "range64G") == 0)
            nodeSetting->range64G = d;
          else if (strcmp(deviceSettingName, "highSpeed") == 0)
            nodeSetting->highSpeed = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_ADS1015:
        {
          struct_ADS1015 *nodeSetting = (struct_ADS1015 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else if (strcmp(deviceSettingName, "logA0") == 0)
            nodeSetting->logA0 = d;
          else if (strcmp(deviceSettingName, "logA1") == 0)
            nodeSetting->logA1 = d;
          else if (strcmp(deviceSettingName, "logA2") == 0)
            nodeSetting->logA2 = d;
          else if (strcmp(deviceSettingName, "logA3") == 0)
            nodeSetting->logA3 = d;
          else if (strcmp(deviceSettingName, "logA0A1") == 0)
            nodeSetting->logA0A1 = d;
          else if (strcmp(deviceSettingName, "logA0A3") == 0)
            nodeSetting->logA0A3 = d;
          else if (strcmp(deviceSettingName, "logA1A3") == 0)
            nodeSetting->logA1A3 = d;
          else if (strcmp(deviceSettingName, "logA2A3") == 0)
            nodeSetting->logA2A3 = d;
          else if (strcmp(deviceSettingName, "gain23") == 0)
            nodeSetting->gain23 = d;
          else if (strcmp(deviceSettingName, "gain1") == 0)
            nodeSetting->gain1 = d;
          else if (strcmp(deviceSettingName, "gain2") == 0)
            nodeSetting->gain2 = d;
          else if (strcmp(deviceSettingName, "gain4") == 0)
            nodeSetting->gain4 = d;
          else if (strcmp(deviceSettingName, "gain8") == 0)
            nodeSetting->gain8 = d;
          else if (strcmp(deviceSettingName, "gain16") == 0)
            nodeSetting->gain16 = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      case DEVICE_PCF8575:
        {
          struct_PCF8575 *nodeSetting = (struct_PCF8575 *)deviceConfigPtr; //Create a local pointer that points to same spot as node does
          if (strcmp(deviceSettingName, "log") == 0)
            nodeSetting->log = d;
          else
            SerialPrintf2("Unknown device setting: %s\r\n", deviceSettingName);
        }
        break;
      default:
        SerialPrintf2("Unknown device type: %d\r\n", deviceType);
        SerialFlush();
        break;
    }
  }
  return (true);
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\productionTest.ino"
/*
  OpenLog Artemis: Production Test Code

  To begin, the OLA checks to see if all four breakout pins are being held low (at reset).
  If they are, the OLA waits for up to five seconds for all four to be released.
  If they are not released, then the OLA assumes they are being used for analog input and continues with the normal OLA code.
  If the pins are released, the OLA waits for serial command bytes via the RX breakout pin (115200 baud).
  The command bytes instruct the OLA to test a thing and report success/fail.
  The OLA responds by: echoing the command byte if the test was successful; echoing the command byte with the most significant bit set if the test failed.
  E.g.: if the Flying Jalapeno sends 0x01: the OLA will return 0x01 if VIN is in range; or 0x81 if VIN is out of range.
  Implememted command bytes are:
    0x01: VIN/3 Divider: The OLA reads VIN/3 and checks that it is 5V +/- 0.25V
    0x02: IMU Temperature: The OLA powers up the IMU and reads its temperature. Success: 25C +/- 15C
    0x03: IMU Accelerometer: The OLA powers up the IMU and reads the accel axes. Success: -1g +/- 0.1g on Z; 0g +/- 0.1g on X and Y
    0x04: IMU Magnetometer: The OLA powers up the IMU and reads the mag axes. Success: 25nT +/- 5nT on Z; vector product of X and Y is 25nT +/- 5nT
    0x05: RTC Crystal: The OLA goes into deep sleep for 5 seconds and checks that the RTC has incremented. Success: RTC increments by 4.9-5.1 seconds
    0x06: Qwiic Power On: The OLA enables Qwiic power. Success: always
    0x07: Qwiic Power Off: The OLA disables Qwiic power. Success: always
    0x08: PWR LED On: The OLA turns on the PWR LED. Success: always
    0x09: PWR LED Off: The OLA turns off the PWR LED. Success: always
    0x0A: STAT LED On: The OLA turns on the STAT LED. Success: always
    0x0B: STAT LED Off: The OLA turns off the STAT LED. Success: always
    0x0C: Pin 32 High: The OLA pulls breakout pin 32 high. Success: always
    0x0D: Pin 32 Low: The OLA pulls breakout pin 32 low. Success: always
    0x0E: Qwiic SCL High: The OLA pulls the Qwiic bus SCL pin high. Success: always
    0x0F: Qwiic SCL Low: The OLA pulls the Qwiic bus SCL pin low. Success: always
    0x10: Qwiic SDA High: The OLA pulls the Qwiic bus SDA pin high. Success: always
    0x11: Qwiic SDA Low: The OLA pulls the Qwiic bus SDA pin low. Success: always
    0x12: Pin 11 High: The OLA pulls breakout pin 11 high. Success: always
    0x13: Pin 11 Low: The OLA pulls breakout pin 11 low. Success: always
    0x14: USB TX Test Start: The OLA sends "HelloHelloHello" (repeating continuously) on USB as 115200 serial. Success: always
    0x15: USB TX Test Stop: The OLA stops sending. Success: always
    0x16: USB Echo Test Start: The OLA will echo back on USB TX whatever it receives on USB RX. Success: always
    0x17: USB Echo Test Stop: The OLA stops echoing on USB. Success: always
    0x18: Read RTC Time: Special case - see below
    0x19: microSD Card Test: The OLA writes to a file on microSD and then reads the contents back. Success: file written successfully
    0x55: Deep Sleep: Special case - see below
  Special cases are:
    0x18: Read RTC Time: The OLA will respond with 0x18 followed by the RTC time in HH:MM:SS.SS format (ASCII text)
    0x55: Deep Sleep: The OLA goes immediately into deep sleep and does not respond
*/

#define verboseProdTest // Comment this line to disable the helpful Serial messages

void productionTest()
{
  pinMode(BREAKOUT_PIN_32, INPUT_PULLUP); // Make pin 32 an input with pull-up
  pinMode(BREAKOUT_PIN_TX, INPUT_PULLUP); // Make pin TX an input with pull-up
  pinMode(BREAKOUT_PIN_RX, INPUT_PULLUP); // Make pin RX an input with pull-up
  pinMode(BREAKOUT_PIN_11, INPUT_PULLUP); // Make pin 11 an input with pull-up

  delay(10); // Wait for the pins to settle
  
  int all_low = LOW; // Flag to indicate the combined (OR) status of the four breakout pins
  all_low |= digitalRead(BREAKOUT_PIN_32); // Read all four pins. If any one is high all_low will be high
  all_low |= digitalRead(BREAKOUT_PIN_TX);
  all_low |= digitalRead(BREAKOUT_PIN_RX);
  all_low |= digitalRead(BREAKOUT_PIN_11);

  if (all_low == HIGH)
  {
    // One or more pins are high so exit now
    pinMode(BREAKOUT_PIN_32, INPUT); // Remove the pull-ups
    pinMode(BREAKOUT_PIN_TX, INPUT);
    pinMode(BREAKOUT_PIN_RX, INPUT);
    pinMode(BREAKOUT_PIN_11, INPUT);
    return;
  }

  unsigned long start_time = millis(); // Record what time we started waiting
  int all_high = LOW; // Flag to indicate the combined (AND) status of the four breakout pins
  while (((millis() - start_time) < 5000) && (all_high == LOW)) // Wait for up to five seconds for all the pins to be released
  {
    all_high = HIGH;
    all_high &= digitalRead(BREAKOUT_PIN_32); // Read all four pins. If any one is low all_high will be low
    all_high &= digitalRead(BREAKOUT_PIN_TX);
    all_high &= digitalRead(BREAKOUT_PIN_RX);
    all_high &= digitalRead(BREAKOUT_PIN_11);
  }

  // Either the five second timeout expired or all four pins are high
  if (all_high == LOW) // If any one pin is still low - timeout must have expired so exit now
  {
    pinMode(BREAKOUT_PIN_32, INPUT); // Remove the pull-ups
    pinMode(BREAKOUT_PIN_TX, INPUT);
    pinMode(BREAKOUT_PIN_RX, INPUT);
    pinMode(BREAKOUT_PIN_11, INPUT);
    return;
  }

  // OK. The breakout pins were held low and then released (pulled-up) within five seconds so let's go into production test mode!

  //We need to manually restore the Serial1 TX and RX pins before we can use Serial1
  configureSerial1TxRx();

  Serial.begin(115200); //Default for initial debug messages if necessary

  //detachInterrupt(PIN_POWER_LOSS); // Disable power loss interrupt
  digitalWrite(PIN_STAT_LED, LOW); // Turn the STAT LED off
  powerLEDOff(); // Turn the power LED on - if the hardware supports it

  analogReadResolution(14); //Increase from default of 10

  readVIN(); // Read VIN now to initialise the analog pin

  Serial1.begin(115200); // Begin the serial port using the TX and RX breakout pins

#ifdef verboseProdTest
  Serial.println(F("OLA Production Test initiated!"));
  Serial.println(F("Waiting for command bytes on the RX breakout pin..."));
#endif

  bool sendHellos = false; // Flag to indicate if we should be sending repeated Hellos via USB (command 0x14/0x15)
  unsigned long lastHelloSent = 0; // Use this to record the last time a Hello was sent
  bool echoUSB = false; // Flag to indicate if we should be echoing on USB (command 0x16/0x17)

  char tempData1[16];
  char tempData2[16];
  char tempData3[16];
  
  while (1) // Do this loop forever!
  {
    while (!Serial1.available()) // Wait until we receive a command byte
    {
      if ((sendHellos == true) && (millis() > lastHelloSent)) // Is it time to send a Hello? (5 x 10 / 115200 = 0.434ms)
      {
        Serial.print(F("Hello"));
        lastHelloSent = millis();
      }
      if (echoUSB == true) // Should we echo everything received via USB?
      {
        while (Serial.available())
        {
          Serial.write(Serial.read()); // Echo
        }
      }
    }

    // Command byte received! Let's process it.
    uint8_t commandByte = Serial1.read();
#ifdef verboseProdTest
    Serial.printf("Processing command byte: 0x%02X\r\n", commandByte);
#endif
    
    switch (commandByte)
    {
      case 0x01: // VIN/3 Divider
      {
        float vin = readVIN(); // Read VIN
#ifdef verboseProdTest
        olaftoa(vin, tempData1, 2, sizeof(tempData1) / sizeof(char));
        Serial.printf("VIN is %sV\r\n", tempData1);
#endif
        if ((vin >= 4.75) && (vin <= 5.25)) // Success
        {
          Serial1.write(0x01);
        }
        else
        {
          Serial1.write(0x81);         
        }
      } // / 0x01: VIN/3
        break;
      case 0x02: // IMU Temperature
      {
        enableCIPOpullUp(); // Enable CIPO pull-up
        beginIMU();
        if (online.IMU)
        {
          delay(100); // Give the IMU time to get going
          if (myICM.dataReady())
          {
            myICM.getAGMT(); //Update values
            delay(10);
            myICM.getAGMT(); //Update values
#ifdef verboseProdTest
            olaftoa(myICM.temp(), tempData1, 2, sizeof(tempData1) / sizeof(char));
            Serial.printf("IMU Temp is: %sC\r\n", tempData1);
#endif
            if ((myICM.temp() >= 10.0) && (myICM.temp() <= 40.0))
            {
              Serial1.write(0x02); // Test passed
            }
            else
            {
              Serial1.write(0x82); // Test failed - readings are out of range
            }
          }
          else
          {
            Serial1.write(0x82); // Test failed - IMU data is not ready
#ifdef verboseProdTest
            Serial.println(F("IMU data not ready!"));
#endif
          }
        }
        else
        {
          Serial1.write(0x82); // Test failed - IMU is not online
#ifdef verboseProdTest
          Serial.println(F("IMU is not online!"));
#endif
        }
        imuPowerOff();
      } // / 0x02: IMU Temperature
        break;
      case 0x03: // IMU Accelerometer
      {
        enableCIPOpullUp(); // Enable CIPO pull-up
        beginIMU();
        if (online.IMU)
        {
          delay(100); // Give the IMU time to get going
          if (myICM.dataReady())
          {
            myICM.getAGMT(); //Update values
            delay(10);
            myICM.getAGMT(); //Update values
#ifdef verboseProdTest
            olaftoa(myICM.accX(), tempData1, 2, sizeof(tempData1) / sizeof(char));
            olaftoa(myICM.accY(), tempData2, 2, sizeof(tempData2) / sizeof(char));
            olaftoa(myICM.accZ(), tempData3, 2, sizeof(tempData3) / sizeof(char));
            Serial.printf("IMU Accel readings are: %s %s %s mg\r\n", tempData1, tempData2, tempData3);
#endif
            if (((myICM.accX() > -100) && (myICM.accX() < 100)) && // Check the readings are in range
                ((myICM.accY() > -100) && (myICM.accY() < 100)) &&
                ((myICM.accZ() > -1100) && (myICM.accZ() < -900)))
            {
              Serial1.write(0x03); // Test passed
            }
            else
            {
              Serial1.write(0x83); // Test failed - readings are out of range
            }
          }
          else
          {
            Serial1.write(0x83); // Test failed - IMU data is not ready
#ifdef verboseProdTest
            Serial.println(F("IMU data not ready!"));
#endif
          }
        }
        else
        {
          Serial1.write(0x83); // Test failed - IMU is not online
#ifdef verboseProdTest
          Serial.println(F("IMU is not online!"));
#endif
        }
        imuPowerOff();
      } // / 0x03: IMU Accelerometer
        break;
      case 0x04: // IMU Magnetometer
      {
        enableCIPOpullUp(); // Enable CIPO pull-up
        beginIMU();
        if (online.IMU)
        {
          delay(100); // Give the IMU time to get going
          if (myICM.dataReady())
          {
            myICM.getAGMT(); //Update values
            delay(10);
            myICM.getAGMT(); //Update values
#ifdef verboseProdTest
            olaftoa(myICM.magX(), tempData1, 2, sizeof(tempData1) / sizeof(char));
            olaftoa(myICM.magY(), tempData2, 2, sizeof(tempData2) / sizeof(char));
            olaftoa(myICM.magZ(), tempData3, 2, sizeof(tempData3) / sizeof(char));
            Serial.printf("IMU Mag readings are: %s %s %s nT\r\n", tempData1, tempData2, tempData3);
#endif
            float magVectorProduct = sqrt((myICM.magX() * myICM.magX()) + (myICM.magY() * myICM.magY())); // Calculate the vector product of magX and magY
#ifdef verboseProdTest
            olaftoa(magVectorProduct, tempData1, 2, sizeof(tempData1) / sizeof(char));
            Serial.printf("IMU Mag XY vector product is: %s nT\r\n", tempData1);
#endif
            if (((myICM.magZ() >= 20.0) && (myICM.magZ() <= 30.0)) && // Check the readings are in range
                ((magVectorProduct >= 20.0) && (magVectorProduct <= 30.0)))
            {
              Serial1.write(0x04); // Test passed
            }
            else
            {
              Serial1.write(0x84); // Test failed - readings are out of range
            }
          }
          else
          {
            Serial1.write(0x84); // Test failed - IMU data is not ready
#ifdef verboseProdTest
            Serial.println(F("IMU data not ready!"));
#endif
          }
        }
        else
        {
          Serial1.write(0x84); // Test failed - IMU is not online
#ifdef verboseProdTest
          Serial.println(F("IMU is not online!"));
#endif
        }
        imuPowerOff();
      } // / 0x04: IMU Magnetometer
        break;
      case 0x05: // RTC Crystal
      {
        myRTC.getTime(); // Read the RTC
        unsigned long hundredthsBeforeSleep = (myRTC.hour * 360000) +  (myRTC.minute * 6000) + (myRTC.seconds * 100) + myRTC.hundredths;
#ifdef verboseProdTest
        Serial.printf("RTC time in hundredths is %d\r\n", hundredthsBeforeSleep);
        Serial.println(F("Going into deep sleep for 5 seconds..."));
#endif
        Serial.flush(); //Finish any prints
        //qwiic.end(); //Power down I2C
        SPI.end(); //Power down SPI
        powerControlADC(false); //Power down ADC. It it started by default before setup().
        Serial.end(); //Power down UART
        Serial1.end();
        //Force the peripherals off
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1);
        //Use the lower power 32kHz clock. Use it to run CT6 as well.
        am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
        am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_G_ENABLE);
        //Setup interrupt to trigger when the number of ms have elapsed
        am_hal_stimer_compare_delta_set(6, 163840); // Sleep for 5 seconds @ 32768 kHz
        //We use counter/timer 6 to cause us to wake up from sleep but 0 to 7 are available
        //CT 7 is used for Software Serial. All CTs are used for Servo.
        am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREG);  //Clear CT6
        am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREG); //Enable C/T G=6
        //Enable the timer interrupt in the NVIC.
        NVIC_EnableIRQ(STIMER_CMPR6_IRQn);
        //Deep Sleep
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        //Turn off interrupt
        NVIC_DisableIRQ(STIMER_CMPR6_IRQn);
        am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREG); //Disable C/T G=6
        //Go back to using the main clock
        //am_hal_stimer_int_enable(AM_HAL_STIMER_INT_OVERFLOW);
        //NVIC_EnableIRQ(STIMER_IRQn);
        am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
        am_hal_stimer_config(AM_HAL_STIMER_HFRC_3MHZ);
        //Turn on ADC
        powerControlADC(true); //ap3_adc_setup();
        Serial.begin(115200);
        Serial1.begin(115200);
        SPI.begin();
        myRTC.getTime(); // Read the RTC
        unsigned long hundredthsAfterSleep = (myRTC.hour * 360000) +  (myRTC.minute * 6000) + (myRTC.seconds * 100) + myRTC.hundredths;
        unsigned long elapsedHundredths = hundredthsAfterSleep - hundredthsBeforeSleep;
#ifdef verboseProdTest
        Serial.println(F("Awake!"));
        Serial.printf("RTC time in hundredths is now %d\r\n", hundredthsAfterSleep);
        Serial.printf("Elapsed RTC time in hundredths is %d\r\n", elapsedHundredths);
#endif
        if ((elapsedHundredths > 490) && (elapsedHundredths < 510))
        {
          Serial1.write(0x05); // Test passed
        }
        else
        {
          Serial1.write(0x85); // Test failed
        }
      } // /0x05: RTC Crystal
        break;
      case 0x06:
        qwiicPowerOn();
        Serial1.write(0x06);
        break;
      case 0x07:
        qwiicPowerOff();
        Serial1.write(0x07);
        break;
      case 0x08:
        powerLEDOn();
        Serial1.write(0x08);
        break;
      case 0x09:
        powerLEDOff();
        Serial1.write(0x09);
        break;
      case 0x0A:
        digitalWrite(PIN_STAT_LED, HIGH);
        Serial1.write(0x0A);
        break;
      case 0x0B:
        digitalWrite(PIN_STAT_LED, LOW);
        Serial1.write(0x0B);
        break;
      case 0x0C:
        pinMode(BREAKOUT_PIN_32, OUTPUT);
        digitalWrite(BREAKOUT_PIN_32, HIGH);
        Serial1.write(0x0C);
        break;
      case 0x0D:
        pinMode(BREAKOUT_PIN_32, OUTPUT);
        digitalWrite(BREAKOUT_PIN_32, LOW);
        Serial1.write(0x0D);
        break;
      case 0x0E:
        pinMode(PIN_QWIIC_SCL, OUTPUT);
        digitalWrite(PIN_QWIIC_SCL, HIGH);
        Serial1.write(0x0E);
        break;
      case 0x0F:
        pinMode(PIN_QWIIC_SCL, OUTPUT);
        digitalWrite(PIN_QWIIC_SCL, LOW);
        Serial1.write(0x0F);
        break;
      case 0x10:
        pinMode(PIN_QWIIC_SDA, OUTPUT);
        digitalWrite(PIN_QWIIC_SDA, HIGH);
        Serial1.write(0x10);
        break;
      case 0x11:
        pinMode(PIN_QWIIC_SDA, OUTPUT);
        digitalWrite(PIN_QWIIC_SDA, LOW);
        Serial1.write(0x11);
        break;
      case 0x12:
        pinMode(BREAKOUT_PIN_11, OUTPUT);
        digitalWrite(BREAKOUT_PIN_11, HIGH);
        Serial1.write(0x12);
        break;
      case 0x13:
        pinMode(BREAKOUT_PIN_11, OUTPUT);
        digitalWrite(BREAKOUT_PIN_11, LOW);
        Serial1.write(0x13);
        break;
      case 0x14:
        sendHellos = true;
        Serial1.write(0x14);
        break;
      case 0x15:
        sendHellos = false;
        Serial1.write(0x15);
        break;
      case 0x16:
        echoUSB = true;
        Serial1.write(0x16);
        break;
      case 0x17:
        echoUSB = false;
        Serial1.write(0x17);
        break;
      case 0x18:
        myRTC.getTime(); // Read the RTC
        Serial1.write(0x18);
        char rtcHour[3];
        char rtcMin[3];
        char rtcSec[3];
        char rtcHundredths[3];
        if (myRTC.hour < 10)
          sprintf(rtcHour, "0%d", myRTC.hour);
        else
          sprintf(rtcHour, "%d", myRTC.hour);
        if (myRTC.minute < 10)
          sprintf(rtcMin, "0%d", myRTC.minute);
        else
          sprintf(rtcMin, "%d", myRTC.minute);
        if (myRTC.seconds < 10)
          sprintf(rtcSec, "0%d", myRTC.seconds);
        else
          sprintf(rtcSec, "%d", myRTC.seconds);
        if (myRTC.hundredths < 10)
          sprintf(rtcHundredths, "0%d", myRTC.hundredths);
        else
          sprintf(rtcHundredths, "%d", myRTC.hundredths);
        
        Serial1.printf("%s:%s:%s.%s", rtcHour, rtcMin, rtcSec, rtcHundredths);
#ifdef verboseProdTest
        Serial.printf("RTC time is %s:%s:%s.%s\r\n", rtcHour, rtcMin, rtcSec, rtcHundredths);
#endif
        break;
      case 0x19: // SD Card Test
      {
        beginSD(); //285 - 293ms
        enableCIPOpullUp(); // Enable CIPO pull-up after beginSD
        if (online.microSD == true)
        {
          if (sd.exists("OLA_prod_test.txt"))
            sd.remove("OLA_prod_test.txt");
      
          SdFile testFile; //FAT32
          if (testFile.open("OLA_prod_test.txt", O_CREAT | O_APPEND | O_WRITE) == true)
          {
#ifdef verboseProdTest
            Serial.println(F("Test file created"));
#endif                              
            testFile.println(F("112358132134")); // Write the Fibonacci sequence - just for fun
            testFile.close(); // Close the file
            if (testFile.open("OLA_prod_test.txt", O_READ) == true)
            {
#ifdef verboseProdTest
              Serial.println(F("Test file reopened"));
#endif                              
              char line[60];
              int n = testFile.fgets(line, sizeof(line));
              if (n == 13)
              {
                if (strcmp(line, "112358132134\n") == 0) // Look for the correct sequence
                {
                  testFile.close(); // Close the file
                  Serial1.write(0x19); // Test passed
#ifdef verboseProdTest
                  Serial.println(F("SD card test passed - file contents are correct"));
#endif                              
                }
                else
                {
                  testFile.close(); // Close the file
                  Serial1.write(0x99); // Test failed - data did not compare
#ifdef verboseProdTest
                  Serial.println(F("Test file contents incorrect!"));
                  for (int l = 0; l < 13; l++)
                    Serial.printf("0x%02X ", line[l]);
                  Serial.println();
#endif                              
                }
              }
              else
              {
                testFile.close(); // Close the file
                Serial1.write(0x99); // Test failed - test file contents are not the correct length
#ifdef verboseProdTest
                Serial.printf("Test file contents incorrect length (%d)!\r\n", n);
#endif                              
              }
            }
            else
            {
              Serial1.write(0x99); // Test failed - could not reopen the test file
#ifdef verboseProdTest
              Serial.println(F("Failed to reopen test file"));
#endif              
            }
          }
          else
          {
            Serial1.write(0x99); // Test failed - could not create the test file
#ifdef verboseProdTest
            Serial.println(F("Failed to create test file"));
#endif
          }
        }
        else
        {
          Serial1.write(0x99); // Test failed - SD is not online
#ifdef verboseProdTest
          Serial.println(F("SD card is not online!"));
#endif          
        }
        microSDPowerOff();
      } // / 0x19: SD Card Test
        break;
      case 0x55: // Deep sleep
      {
#ifndef noPowerLossProtection // Probably redundant - included just in case detachInterrupt causes badness when it has not been attached
        detachInterrupt(PIN_POWER_LOSS); // Disable power loss interrupt
#endif
        Serial.end(); //Power down UART
        //Force the peripherals off
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1);      
        //Disable pads
        for (int x = 0; x < 50; x++)
        {
          if ((x != PIN_POWER_LOSS) &&
            //(x != PIN_LOGIC_DEBUG) &&
            (x != PIN_MICROSD_POWER) &&
            (x != PIN_QWIIC_POWER) &&
            (x != PIN_IMU_POWER))
          {
            am_hal_gpio_pinconfig(x, g_AM_HAL_GPIO_DISABLE);
          }
        }
        //We can't leave these power control pins floating
        imuPowerOff();
        microSDPowerOff();
        qwiicPowerOff();
        
        //Power down cache, flash, SRAM
        am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_ALL); // Power down all flash and cache
        am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_384K); // Retain all SRAM
      
        //Use the lower power 32kHz clock. Use it to run CT6 as well.
        am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
        am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_G_ENABLE);

        while (1) // Stay in deep sleep until we get reset
        {
          am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP); //Sleep
        }
      } // / 0x55: Deep sleep
        break;
      default:
#ifdef verboseProdTest
        Serial.printf("Unrecognised command byte: 0x%02X\r\n", commandByte);
#endif
        break;
    }
  }
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\support.ino"


#include "Sensors.h"

bool useRTCmillis(void)
{
  return (((settings.useGPIO11ForTrigger == false) && (settings.usBetweenReadings >= maxUsBeforeSleep))
  || (settings.useGPIO11ForFastSlowLogging == true)
  || (settings.useRTCForFastSlowLogging == true));
}

uint64_t bestMillis(void)
{
  if (useRTCmillis())
    return(rtcMillis());
  else
    return(millis());
}

void printDebug(String thingToPrint)
{
  if(settings.printDebugMessages == true)
  {
    Serial.print(thingToPrint);
    if (settings.useTxRxPinsForTerminal == true)
      Serial1.print(thingToPrint);
  }
}

//Option not known
void printUnknown(uint8_t unknownChoice)
{
  SerialPrint(F("Unknown choice: "));
  Serial.write(unknownChoice);
  if (settings.useTxRxPinsForTerminal == true)
      Serial1.write(unknownChoice);
  SerialPrintln(F(""));
}
void printUnknown(int unknownValue)
{
  SerialPrint(F("Unknown value: "));
  Serial.write(unknownValue);
  if (settings.useTxRxPinsForTerminal == true)
      Serial1.write(unknownValue);
  SerialPrintln(F(""));
}

//Blocking wait for user input
void waitForInput()
{
  for (int i = 0; i < 10; i++) //Wait for any incoming chars to hit buffer
  {
    checkBattery();
    delay(1);
  }
  
  while (Serial.available() > 0) Serial.read(); //Clear buffer
  
  if (settings.useTxRxPinsForTerminal == true)
    while (Serial1.available() > 0) Serial1.read(); //Clear buffer

  bool keepChecking = true;    
  while (keepChecking)
  {
    if (Serial.available())
      keepChecking = false;
      
    if (settings.useTxRxPinsForTerminal == true)
    {
      if (Serial1.available())
        keepChecking = false;
    }
    
    checkBattery();
  }
}

//Get single byte from user
//Waits for and returns the character that the user provides
//Returns STATUS_GETNUMBER_TIMEOUT if input times out
//Returns 'x' if user presses 'x'
uint8_t getByteChoice(int numberOfSeconds, bool updateDZSERIAL)
{
  SerialFlush();

  for (int i = 0; i < 50; i++) //Wait for any incoming chars to hit buffer
  {
    checkBattery();
    delay(1);
  }
  
  while (Serial.available() > 0) Serial.read(); //Clear buffer

  if (settings.useTxRxPinsForTerminal == true)
    while (Serial1.available() > 0) Serial1.read(); //Clear buffer

  unsigned long startTime = millis();
  byte incoming;
  while (1)
  {
    if (Serial.available() > 0)
    {
      incoming = Serial.read();
      if (updateDZSERIAL)
      {
        DSERIAL = &Serial;
        ZSERIAL = &Serial;
      }
//      SerialPrint(F("byte: 0x"));
//      Serial.println(incoming, HEX);
//      if (settings.useTxRxPinsForTerminal == true)
//        Serial1.println(incoming, HEX);
      if (incoming >= 'a' && incoming <= 'z') break;
      if (incoming >= 'A' && incoming <= 'Z') break;
      if (incoming >= '0' && incoming <= '9') break;
    }

    if ((settings.useTxRxPinsForTerminal == true) && (Serial1.available() > 0))
    {
      incoming = Serial1.read();
      if (updateDZSERIAL)
      {
        DSERIAL = &Serial1;
        ZSERIAL = &Serial1;
      }
//      SerialPrint(F("byte: 0x"));
//      Serial.println(incoming, HEX);
//      if (settings.useTxRxPinsForTerminal == true)
//        Serial1.println(incoming, HEX);
      if (incoming >= 'a' && incoming <= 'z') break;
      if (incoming >= 'A' && incoming <= 'Z') break;
      if (incoming >= '0' && incoming <= '9') break;
    }

    if ( (millis() - startTime) / 1000 >= numberOfSeconds)
    {
      SerialPrintln(F("No user input received."));
      return (STATUS_GETBYTE_TIMEOUT); //Timeout. No user input.
    }

    checkBattery();
    delay(1);
  }

  return (incoming);
}

//Get a string/value from user, remove all non-numeric values
//Returns STATUS_GETNUMBER_TIMEOUT if input times out
//Returns STATUS_PRESSED_X if user presses 'x'
int64_t getNumber(int numberOfSeconds)
{
  for (int i = 0; i < 10; i++) //Wait for any incoming chars to hit buffer
  {
    checkBattery();
    delay(1);
  }
  
  while (Serial.available() > 0) Serial.read(); //Clear buffer

  if (settings.useTxRxPinsForTerminal == true)
    while (Serial1.available() > 0) Serial1.read(); //Clear buffer

  //Get input from user
  char cleansed[20]; //Good for very large numbers: 123,456,789,012,345,678\0

  unsigned long startTime = millis();
  int spot = 0;
  while (spot < 20 - 1) //Leave room for terminating \0
  {
    bool serialAvailable = false;
    while (serialAvailable == false) //Wait for user input
    {
      if (Serial.available())
        serialAvailable = true;

      if ((settings.useTxRxPinsForTerminal == true) && (Serial1.available()))
        serialAvailable = true;
        
      checkBattery();
      
      if ( (millis() - startTime) / 1000 >= numberOfSeconds)
      {
        if (spot == 0)
        {
          SerialPrintln(F("No user input received. Do you have line endings turned on?"));
          return (STATUS_GETNUMBER_TIMEOUT); //Timeout. No user input.
        }
        else if (spot > 0)
        {
          break; //Timeout, but we have data
        }
      }
    }

    //See if we timed out waiting for a line ending
    if (spot > 0 && (millis() - startTime) / 1000 >= numberOfSeconds)
    {
      SerialPrintln(F("Do you have line endings turned on?"));
      break; //Timeout, but we have data
    }

    byte incoming;

    if (Serial.available())
      incoming = Serial.read();
    
    else
      incoming = Serial1.read();

    if (incoming == '\n' || incoming == '\r')
    {
      SerialPrintln(F(""));
      break;
    }

    if ((isDigit(incoming) == true) || ((incoming == '-') && (spot == 0))) // Check for digits and a minus sign
    {
      Serial.write(incoming); //Echo user's typing

      if (settings.useTxRxPinsForTerminal == true)
        Serial1.write(incoming); //Echo user's typing
      
      cleansed[spot++] = (char)incoming;
    }

    if (incoming == 'x')
    {
      return (STATUS_PRESSED_X);
    }
  }

  cleansed[spot] = '\0';

  int64_t largeNumber = 0;
  int x = 0;
  if (cleansed[0] == '-') // If our number is negative
  {
    x = 1; // Skip the minus
  }
  for( ; x < spot ; x++)
  {
    largeNumber *= 10;
    largeNumber += (cleansed[x] - '0');
  }
  if (cleansed[0] == '-') // If our number is negative
  {
    largeNumber = 0 - largeNumber; // Make it negative
  }
  return (largeNumber);
}

//Get a string/value from user, remove all non-numeric values
//Returns STATUS_GETNUMBER_TIMEOUT if input times out
//Returns STATUS_PRESSED_X if user presses 'x'
double getDouble(int numberOfSeconds)
{
  for (int i = 0; i < 10; i++) //Wait for any incoming chars to hit buffer
  {
    checkBattery();
    delay(1);
  }
  
  while (Serial.available() > 0) Serial.read(); //Clear buffer

  if (settings.useTxRxPinsForTerminal == true)
    while (Serial1.available() > 0) Serial1.read(); //Clear buffer

  //Get input from user
  char cleansed[20]; //Good for very large numbers: 123,456,789,012,345,678\0

  unsigned long startTime = millis();
  int spot = 0;
  bool dpSeen = false;
  while (spot < 20 - 1) //Leave room for terminating \0
  {
    bool serialAvailable = false;
    while (serialAvailable == false) //Wait for user input
    {
      if (Serial.available())
        serialAvailable = true;

      if ((settings.useTxRxPinsForTerminal == true) && (Serial1.available()))
        serialAvailable = true;

      checkBattery();
      
      if ( (millis() - startTime) / 1000 >= numberOfSeconds)
      {
        if (spot == 0)
        {
          SerialPrintln(F("No user input received. Do you have line endings turned on?"));
          return (STATUS_GETNUMBER_TIMEOUT); //Timeout. No user input.
        }
        else if (spot > 0)
        {
          break; //Timeout, but we have data
        }
      }
    }

    //See if we timed out waiting for a line ending
    if (spot > 0 && (millis() - startTime) / 1000 >= numberOfSeconds)
    {
      SerialPrintln(F("Do you have line endings turned on?"));
      break; //Timeout, but we have data
    }

    byte incoming;

    if (Serial.available())
      incoming = Serial.read();
    
    else
      incoming = Serial1.read();

    if (incoming == '\n' || incoming == '\r')
    {
      SerialPrintln(F(""));
      break;
    }

    if ((isDigit(incoming) == true) || ((incoming == '-') && (spot == 0)) || ((incoming == '.') && (dpSeen == false))) // Check for digits/minus/dp
    {
      Serial.write(incoming); //Echo user's typing
      
      if (settings.useTxRxPinsForTerminal == true)
        Serial1.write(incoming); //Echo user's typing
      
      cleansed[spot++] = (char)incoming;
    }

    if (incoming == '.')
      dpSeen = true;

    if (incoming == 'x')
    {
      return (STATUS_PRESSED_X);
    }
  }

  cleansed[spot] = '\0';

  double largeNumber = 0;
  int x = 0;
  if (cleansed[0] == '-') // If our number is negative
  {
    x = 1; // Skip the minus
  }
  for( ; x < spot ; x++)
  {
    if (cleansed[x] == '.')
      break;
    largeNumber *= 10;
    largeNumber += (cleansed[x] - '0');
  }
  if (x < spot) // Check if we found a '.'
  {
    x++;
    double divider = 0.1;
    for( ; x < spot ; x++)
    {
      largeNumber += (cleansed[x] - '0') * divider;
      divider /= 10;
    }
  }
  if (cleansed[0] == '-') // If our number is negative
  {
    largeNumber = 0 - largeNumber; // Make it negative
  }
  return (largeNumber);
}

//*****************************************************************************
//
//  Divide an unsigned 32-bit value by 10.
//
//  Note: Adapted from Ch10 of Hackers Delight (hackersdelight.org).
//
//*****************************************************************************
static uint64_t divu64_10(uint64_t ui64Val)
{
    uint64_t q64, r64;
    uint32_t q32, r32, ui32Val;

    //
    // If a 32-bit value, use the more optimal 32-bit routine.
    //
    if ( ui64Val >> 32 )
    {
        q64 = (ui64Val>>1) + (ui64Val>>2);
        q64 += (q64 >> 4);
        q64 += (q64 >> 8);
        q64 += (q64 >> 16);
        q64 += (q64 >> 32);
        q64 >>= 3;
        r64 = ui64Val - q64*10;
        return q64 + ((r64 + 6) >> 4);
    }
    else
    {
        ui32Val = (uint32_t)(ui64Val & 0xffffffff);
        q32 = (ui32Val>>1) + (ui32Val>>2);
        q32 += (q32 >> 4);
        q32 += (q32 >> 8);
        q32 += (q32 >> 16);
        q32 >>= 3;
        r32 = ui32Val - q32*10;
        return (uint64_t)(q32 + ((r32 + 6) >> 4));
    }
}

//*****************************************************************************
//
// Converts ui64Val to a string.
// Note: pcBuf[] must be sized for a minimum of 21 characters.
//
// Returns the number of decimal digits in the string.
//
// NOTE: If pcBuf is NULL, will compute a return ui64Val only (no chars
// written).
//
//*****************************************************************************
static int uint64_to_str(uint64_t ui64Val, char *pcBuf)
{
    char tbuf[25];
    int ix = 0, iNumDig = 0;
    unsigned uMod;
    uint64_t u64Tmp;

    do
    {
        //
        // Divide by 10
        //
        u64Tmp = divu64_10(ui64Val);

        //
        // Get modulus
        //
        uMod = ui64Val - (u64Tmp * 10);

        tbuf[ix++] = uMod + '0';
        ui64Val = u64Tmp;
    } while ( ui64Val );

    //
    // Save the total number of digits
    //
    iNumDig = ix;

    //
    // Now, reverse the buffer when saving to the caller's buffer.
    //
    if ( pcBuf )
    {
        while ( ix-- )
        {
            *pcBuf++ = tbuf[ix];
        }

        //
        // Terminate the caller's buffer
        //
        *pcBuf = 0x00;
    }

    return iNumDig;
}

//*****************************************************************************
//
//  Float to ASCII text. A basic implementation for providing support for
//  single-precision %f.
//
//  param
//      fValue     = Float value to be converted.
//      pcBuf      = Buffer to place string AND input of buffer size.
//      iPrecision = Desired number of decimal places.
//      bufSize    = The size (in bytes) of the buffer.
//                   The recommended size is at least 16 bytes.
//
//  This function performs a basic translation of a floating point single
//  precision value to a string.
//
//  return Number of chars printed to the buffer.
//
//*****************************************************************************
#define OLA_FTOA_ERR_VAL_TOO_SMALL   -1
#define OLA_FTOA_ERR_VAL_TOO_LARGE   -2
#define OLA_FTOA_ERR_BUFSIZE         -3

typedef union
{
    int32_t I32;
    float F;
} ola_i32fl_t;

static int olaftoa(float fValue, char *pcBuf, int iPrecision, int bufSize)
{
    ola_i32fl_t unFloatValue;
    int iExp2, iBufSize;
    int32_t i32Significand, i32IntPart, i32FracPart;
    char *pcBufInitial, *pcBuftmp;

    iBufSize = bufSize; // *(uint32_t*)pcBuf;
    if (iBufSize < 4)
    {
        return OLA_FTOA_ERR_BUFSIZE;
    }

    if (fValue == 0.0f)
    {
        // "0.0"
        *(uint32_t*)pcBuf = 0x00 << 24 | ('0' << 16) | ('.' << 8) | ('0' << 0);
        return 3;
    }

    pcBufInitial = pcBuf;

    unFloatValue.F = fValue;

    iExp2 = ((unFloatValue.I32 >> 23) & 0x000000FF) - 127;
    i32Significand = (unFloatValue.I32 & 0x00FFFFFF) | 0x00800000;
    i32FracPart = 0;
    i32IntPart = 0;

    if (iExp2 >= 31)
    {
        return OLA_FTOA_ERR_VAL_TOO_LARGE;
    }
    else if (iExp2 < -23)
    {
        return OLA_FTOA_ERR_VAL_TOO_SMALL;
    }
    else if (iExp2 >= 23)
    {
        i32IntPart = i32Significand << (iExp2 - 23);
    }
    else if (iExp2 >= 0)
    {
        i32IntPart = i32Significand >> (23 - iExp2);
        i32FracPart = (i32Significand << (iExp2 + 1)) & 0x00FFFFFF;
    }
    else // if (iExp2 < 0)
    {
        i32FracPart = (i32Significand & 0x00FFFFFF) >> -(iExp2 + 1);
    }

    if (unFloatValue.I32 < 0)
    {
        *pcBuf++ = '-';
    }

    if (i32IntPart == 0)
    {
        *pcBuf++ = '0';
    }
    else
    {
        if (i32IntPart > 0)
        {
            uint64_to_str(i32IntPart, pcBuf);
        }
        else
        {
            *pcBuf++ = '-';
            uint64_to_str(-i32IntPart, pcBuf);
        }
        while (*pcBuf)    // Get to end of new string
        {
            pcBuf++;
        }
    }

    //
    // Now, begin the fractional part
    //
    *pcBuf++ = '.';

    if (i32FracPart == 0)
    {
        *pcBuf++ = '0';
    }
    else
    {
        int jx, iMax;

        iMax = iBufSize - (pcBuf - pcBufInitial) - 1;
        iMax = (iMax > iPrecision) ? iPrecision : iMax;

        for (jx = 0; jx < iMax; jx++)
        {
            i32FracPart *= 10;
            *pcBuf++ = (i32FracPart >> 24) + '0';
            i32FracPart &= 0x00FFFFFF;
        }

        //
        // Per the printf spec, the number of digits printed to the right of the
        // decimal point (i.e. iPrecision) should be rounded.
        // Some examples:
        // Value        iPrecision          Formatted value
        // 1.36399      Unspecified (6)     1.363990
        // 1.36399      3                   1.364
        // 1.36399      4                   1.3640
        // 1.36399      5                   1.36399
        // 1.363994     Unspecified (6)     1.363994
        // 1.363994     3                   1.364
        // 1.363994     4                   1.3640
        // 1.363994     5                   1.36399
        // 1.363995     Unspecified (6)     1.363995
        // 1.363995     3                   1.364
        // 1.363995     4                   1.3640
        // 1.363995     5                   1.36400
        // 1.996        Unspecified (6)     1.996000
        // 1.996        2                   2.00
        // 1.996        3                   1.996
        // 1.996        4                   1.9960
        //
        // To determine whether to round up, we'll look at what the next
        // decimal value would have been.
        //
        if ( ((i32FracPart * 10) >> 24) >= 5 )
        {
            //
            // Yes, we need to round up.
            // Go back through the string and make adjustments as necessary.
            //
            pcBuftmp = pcBuf - 1;
            while ( pcBuftmp >= pcBufInitial )
            {
                if ( *pcBuftmp == '.' )
                {
                }
                else if ( *pcBuftmp == '9' )
                {
                    *pcBuftmp = '0';
                }
                else
                {
                    *pcBuftmp += 1;
                    break;
                }
                pcBuftmp--;
            }
        }
    }

    //
    // Terminate the string and we're done
    //
    *pcBuf = 0x00;

    return (pcBuf - pcBufInitial);
} // ftoa()

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\timeStamp.ino"
//Query the RTC and put the appropriately formatted (according to settings) 
//string into the passed buffer. timeStringBuffer should be at least 37 chars long
//Code modified by @DennisMelamed in PR #70
void getTimeString(char timeStringBuffer[])
{
  //reset the buffer
  timeStringBuffer[0] = '\0';

  myRTC.getTime();

  if (settings.logDate)
  {
    char rtcDate[12]; // 10/12/2019,
    char rtcDay[3];
    char rtcMonth[3];
    char rtcYear[5];
    if (myRTC.dayOfMonth < 10)
      sprintf(rtcDay, "0%d", myRTC.dayOfMonth);
    else
      sprintf(rtcDay, "%d", myRTC.dayOfMonth);
    if (myRTC.month < 10)
      sprintf(rtcMonth, "0%d", myRTC.month);
    else
      sprintf(rtcMonth, "%d", myRTC.month);
    if (myRTC.year < 10)
      sprintf(rtcYear, "200%d", myRTC.year);
    else
      sprintf(rtcYear, "20%d", myRTC.year);
    if (settings.dateStyle == 0)
      sprintf(rtcDate, "%s/%s/%s,", rtcMonth, rtcDay, rtcYear);
    else if (settings.dateStyle == 1)
      sprintf(rtcDate, "%s/%s/%s,", rtcDay, rtcMonth, rtcYear);
    else if (settings.dateStyle == 2)
      sprintf(rtcDate, "%s/%s/%s,", rtcYear, rtcMonth, rtcDay);
    else // if (settings.dateStyle == 3)
      sprintf(rtcDate, "%s-%s-%sT", rtcYear, rtcMonth, rtcDay);
    strcat(timeStringBuffer, rtcDate);
  }

  if ((settings.logTime) || ((settings.logDate) && (settings.dateStyle == 3)))
  {
    char rtcTime[16]; //09:14:37.41, or 09:14:37+00:00,
    int adjustedHour = myRTC.hour;
    if (settings.hour24Style == false)
    {
      if (adjustedHour > 12) adjustedHour -= 12;
    }
    char rtcHour[3];
    char rtcMin[3];
    char rtcSec[3];
    char rtcHundredths[3];
    char timeZoneH[4];
    char timeZoneM[4];
    if (adjustedHour < 10)
      sprintf(rtcHour, "0%d", adjustedHour);
    else
      sprintf(rtcHour, "%d", adjustedHour);
    if (myRTC.minute < 10)
      sprintf(rtcMin, "0%d", myRTC.minute);
    else
      sprintf(rtcMin, "%d", myRTC.minute);
    if (myRTC.seconds < 10)
      sprintf(rtcSec, "0%d", myRTC.seconds);
    else
      sprintf(rtcSec, "%d", myRTC.seconds);
    if (myRTC.hundredths < 10)
      sprintf(rtcHundredths, "0%d", myRTC.hundredths);
    else
      sprintf(rtcHundredths, "%d", myRTC.hundredths);
    if (settings.localUTCOffset >= 0)
    {
      if (settings.localUTCOffset < 10)
        sprintf(timeZoneH, "+0%d", (int)settings.localUTCOffset);
      else
        sprintf(timeZoneH, "+%d", (int)settings.localUTCOffset);
    }
    else
    {
      if (settings.localUTCOffset <= -10)
        sprintf(timeZoneH, "-%d", 0 - (int)settings.localUTCOffset);
      else
        sprintf(timeZoneH, "-0%d", 0 - (int)settings.localUTCOffset);
    }
    int tzMins = (int)((settings.localUTCOffset - (float)((int)settings.localUTCOffset)) * 60.0);
    if (tzMins < 0)
      tzMins = 0 - tzMins;
    if (tzMins < 10)
      sprintf(timeZoneM, ":0%d", tzMins);
    else
      sprintf(timeZoneM, ":%d", tzMins);
    if ((settings.logDate) && (settings.dateStyle == 3))
    {
      sprintf(rtcTime, "%s:%s:%s%s%s,", rtcHour, rtcMin, rtcSec, timeZoneH, timeZoneM);
      strcat(timeStringBuffer, rtcTime);      
    }
    if (settings.logTime)
    {
      sprintf(rtcTime, "%s:%s:%s.%s,", rtcHour, rtcMin, rtcSec, rtcHundredths);
      strcat(timeStringBuffer, rtcTime);
    }
  }
  
  if (settings.logMicroseconds)
  {
    // Convert uint64_t to string
    // Based on printLLNumber by robtillaart
    // https://forum.arduino.cc/index.php?topic=143584.msg1519824#msg1519824
    char microsecondsRev[20]; // Char array to hold to microseconds (reversed order)
    char microseconds[20]; // Char array to hold to microseconds (correct order)
    uint64_t microsNow = micros();
    unsigned int i = 0;
    
    if (microsNow == 0ULL) // if usBetweenReadings is zero, set tempTime to "0"
    {
      microseconds[0] = '0';
      microseconds[1] = ',';
      microseconds[2] = 0;
    }
    
    else
    {
      while (microsNow > 0)
      {
        microsecondsRev[i++] = (microsNow % 10) + '0'; // divide by 10, convert the remainder to char
        microsNow /= 10; // divide by 10
      }
      unsigned int j = 0;
      while (i > 0)
      {
        microseconds[j++] = microsecondsRev[--i]; // reverse the order
        microseconds[j] = ',';
        microseconds[j+1] = 0; // mark the end with a NULL
      }
    }
    
    strcat(timeStringBuffer, microseconds);
  }
}

//Gets the current time from GPS
//Adjust the hour by local hour offset
//Adjust the date as necessary
//
//Note: this function should only be called if we know that a u-blox GNSS is actually connected
//
void getGPSDateTime(int &year, int &month, int &day, int &hour, int &minute, int &second, int &millisecond, bool &dateValid, bool &timeValid) {
  //Get latested date/time from GPS
  //These will be extracted from a single PVT packet
  getUbloxDateTime(year, month, day, hour, minute, second, millisecond, dateValid, timeValid);
  //Do it twice - to make sure the data is fresh
  getUbloxDateTime(year, month, day, hour, minute, second, millisecond, dateValid, timeValid);

  adjustToLocalDateTime(year, month, day, hour, minute, settings.localUTCOffset);
}

//Given the date and hour, calculate local date/time
//Adjust the hour by local hour offset
//Adjust the hour by DST as necessary
//Adjust the date as necessary
//Leap year is taken into account but does not interact with DST (DST happens later in March)
void adjustToLocalDateTime(int &year, int &month, int &day, int &hour, int &minute, float localUTCOffset) {

  //Apply any offset to UTC
  hour += (int)localUTCOffset;

  //Apply minutes offset
  int tzMins = (int)((localUTCOffset - (float)((int)localUTCOffset)) * 60.0);
  minute += tzMins;
  if (minute >= 60)
  {
    hour += 1;
    minute -= 60;
  }
  else if (minute < 0)
  {
    hour -= 1;
    minute += 60;
  }

  //If the adjusted hour is outside 0 to 23, then adjust date as necessary
  correctDate(year, month, day, hour);

  //Should we correct for daylight savings time?
  if (settings.correctForDST == true)
  {
    //Calculate DST adjustment based on date and local offset
    hour += findUSDSTadjustment(year, month, day, hour);

    //DST may have pushed a date change so do one more time
    correctDate(year, month, day, hour);
  }
}

//If the given hour is outside 0 to 23, then adjust date and hour as necessary
void correctDate(int &year, int &month, int &day, int &hour)
{
  //Adjust date forwards if the local hour offset causes it
  if (hour > 23)
  {
    hour -= 24;
    day++;
    bool adjustMonth = false;
    if (month == 1 && day == 32)
      adjustMonth = true;
    else if (month == 2)
    {
      if (year % 4 == 0 && day == 30) // Note: this will fail in 2100. 2100 is not a leap year.
        adjustMonth = true;
      else if (day == 29)
        adjustMonth = true;
    }
    else if (month == 3 && day == 32)
      adjustMonth = true;
    else if (month == 4 && day == 31)
      adjustMonth = true;
    else if (month == 5 && day == 32)
      adjustMonth = true;
    else if (month == 6 && day == 31)
      adjustMonth = true;
    else if (month == 7 && day == 32)
      adjustMonth = true;
    else if (month == 8 && day == 32)
      adjustMonth = true;
    else if (month == 9 && day == 31)
      adjustMonth = true;
    else if (month == 10 && day == 32)
      adjustMonth = true;
    else if (month == 11 && day == 31)
      adjustMonth = true;
    else if (month == 11 && day == 32)
      adjustMonth = true;

    if (adjustMonth == true)
    {
      month++;
      day = 1;
      if (month == 13)
      {
        month = 1;
        year++;
      }
    }
  }

  //Adjust date backwards if the local hour offset causes it
  if (hour < 0)
  {
    hour += 24;
    day--;
    if (day == 0)
    {
      //Move back a month and reset day to the last day of the new month
      month--;
      switch (month)
      {
        case 0: //December
          year--;
          month = 12;
          day = 31;
          break;
        case 1: //January
          day = 31;
          break;
        case 2: //February
          if (year % 4 == 0) day = 29; // Note: this will fail in 2100. 2100 is not a leap year.
          else day = 28;
          break;
        case 3: //March
          day = 31;
          break;
        case 4: //April
          day = 30;
          break;
        case 5: //May
          day = 31;
          break;
        case 6: //June
          day = 30;
          break;
        case 7: //July
          day = 31;
          break;
        case 8: //August
          day = 31;
          break;
        case 9: //September
          day = 30;
          break;
        case 10: //October
          day = 31;
          break;
        case 11: //November
          day = 30;
          break;
      }
    }
  }
}

//Given a year/month/day/current UTC/local offset give me the amount to adjust the current hour
//Clocks adjust at 2AM so we need the local hour as well
int findUSDSTadjustment(int year, byte month, byte day, byte localHour)
{
  //Since 2007 DST starts on the second Sunday in March and ends the first Sunday of November
  //Let's just assume it's going to be this way for awhile (silly US government!)
  //Example from: http://stackoverflow.com/questions/5590429/calculating-daylight-savings-time-from-only-date

  //boolean dst = false; //Assume we're not in DST
  if (month > 3 && month < 11) return (1); //DST is happening!
  if (month < 3 || month > 11) return (0); //DST is not happening

  int firstSunday = getFirstSunday(year, month);
  int secondSunday = firstSunday + 7;

  //In March, we are in DST if we are on or after the second sunday.
  if (month == 3)
  {
    if (day > secondSunday) return (1); //We are in later march
    if (day < secondSunday) return (0); //No DST
    if (day == secondSunday)
    {
      if (localHour >= 2) return (1); //It's after 2AM, spring forward, add hour to clock
      else return (0); //It's before 2AM, no DST
    }
  }

  //In November we must be before the first Sunday to be DST.
  if (month == 11)
  {
    if (day < firstSunday) return (1); //Still in DST
    if (day > firstSunday) return (0); //No DST
    if (day == firstSunday) //Today is the last day of DST
    {
      //At 2AM the clock gets moved back to 1AM
      //When we check the local hour we need to assume DST is still being applied (at least for the 12, 1AM, 2AM checks)
      if ((localHour + 1) >= 2) return (0); //It's 2AM or later, fall back, remove hour from clock
      else return (1); //It's before 2AM, continue adding DST
    }
  }

  return (0); //We should not get here
}

//Given year/month, find day of first Sunday
byte getFirstSunday(int year, int month)
{
  int day = 1;
  while (dayOfWeek(year, month, day) != 0)
    day++;
  return (day);
}

//Given the current year/month/day
//Returns 0 (Sunday) through 6 (Saturday) for the day of the week
//From: http://en.wikipedia.org/wiki/Calculating_the_day_of_the_week
//This function assumes the month from the caller is 1-12
char dayOfWeek(int year, int month, int day)
{
  //Devised by Tomohiko Sakamoto in 1993, it is accurate for any Gregorian date:
  static int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4  };
  year -= month < 3;
  return (year + year / 4 - year / 100 + year / 400 + t[month - 1] + day) % 7;
}

#line 1 "G:\\My Drive\\Programming\\Arduino\\Projects\\Projects 2024\\OpenLog_Artemis_Vehicle-Log\\Firmware\\OpenLog_Artemis\\zmodem.ino"
// ecm-bitflipper's Arduino ZModem:
// https://github.com/ecm-bitflipper/Arduino_ZModem

// V2.1.3
// 2020-09-02
//  - Updated for the OLA by Paul Clark
//  - CD, MD, RD, PWD and RZ disabled
//  - added CAT/TYPE

#define ultoa utoa

// Arghhh. These three links have disappeared!
// See this page for the original code:
// http://www.raspberryginger.com/jbailey/minix/html/dir_acf1a49c3b8ff2cb9205e4a19757c0d6.html
// From: http://www.raspberryginger.com/jbailey/minix/html/zm_8c-source.html
// docs at: http://www.raspberryginger.com/jbailey/minix/html/zm_8c.html
// The minix files here might be the same thing:
// http://www.cise.ufl.edu/~cop4600/cgi-bin/lxr/http/source.cgi/commands/zmodem/

#include "zmodem_config.h"
#include "zmodem.h"
#include "zmodem_zm.h"

/*
  Originally was an example by fat16lib of reading a directory
  and listing its files by directory entry number.
See: http://forum.arduino.cc/index.php?topic=173562.0

  Heavily modified by Pete (El Supremo) to recursively list the files
  starting at a specified point in the directory structure and then
  use zmodem to transmit them to the PC via the ZSERIAL port

  Further heavy modifications by Dylan (monte_carlo_ecm, bitflipper, etc.)
  to create a user driven "file manager" of sorts.
  Many thanks to Pete (El Supremo) who got this started.  Much work remained
  to get receive (rz) working, mostly due to the need for speed because of the
  very small (64 bytes) Serial buffer in the Arduino.

  I have tested this with an Arduino Mega 2560 R3 interfacing with Windows 10
  using Hyperterminal, Syncterm and TeraTerm.  All of them seem to work, though
  their crash recovery (partial file transfer restart) behaviours vary.
  Syncterm kicks out a couple of non-fatal errors at the beginning of sending
  a file to the Arduino, but appears to always recover and complete the transfer.

  This sketch should work on any board with at least 30K of flash and 2K of RAM.
  Go to zmodem_config.h and disable some of the ARDUINO_SMALL_MEMORY_* macros
  for maximum peace of mind and stability if you don't need all the features
  (send, receive and file management).

V2.1.2
2018-05-11
  - Fixes for Arduino IDE 1.8.5
  - Attempted to patch for use on Teensy

V2.1
2015-03-06
  - Large scale code clean-up, reduction of variable sizes where they were
    unnecessarily large, sharing variables previously unshared between sz and
    rz, and creative use of the send/receive buffer allowed this sketch to
    BARELY fit and run with all features enabled on a board with 30K flash and
    2K of RAM.  Uno & Nano users - enjoy.
  - Some boards were unstable at baud rates above 9600.  I tracked this back
    to overrunning the SERIAL_TX_BUFFER_SIZE to my surprise.  Added a check
    if a flush() is required both in the help and directory listings, as well
    as the sendline() macro.

V2.0
2015-02-23
  - Taken over by Dylan (monte_carlo_ecm, bitflipper, etc.)
  - Added Serial based user interface
  - Added support for SparkFun MP3 shield based SDCard (see zmodem_config.h)
  - Moved CRC tables to PROGMEM to lighten footprint on dynamic memory (zmodem_crc16.cpp)
  - Added ZRQINIT at start of sz.  All terminal applications I tested didn't strictly need it, but it's
    super handy for getting the terminal application to auto start the download
  - Completed adaptation of rz to Arduino
  - Removed directory recursion for sz in favour of single file or entire current directory ("*") for sz
  - Optimized zdlread, readline, zsendline and sendline
      into macros for rz speed - still only up to 57600 baud
  - Enabled "crash recovery" for both sz and rz.  Various terminal applications may respond differently
      to restarting partially completed transfers; experiment with yours to see how it behaves.  This
      feature could be particularly useful if you have an ever growing log file and you just need to
      download the entries since your last download from your Arduino to your computer.

V1.03
140913
  - remove extraneous code such as the entire main() function
    in sz and rz and anything dependent on the vax, etc.
  - moved purgeline, sendline, readline and bttyout from rz to zm
    so that the the zmodem_rz.cpp file is not required when compiling
    sz 
    
V1.02
140912
  - yup, sz transfer still works.
    10 files -- 2853852 bytes
    Time = 265 secs
    
V1.01
140912
This was originally working on a T++2 and now works on T3
  - This works on a T3 using the RTC/GPS/uSD breadboard
    It sent multiple files - see info.h
  - both rz and sz sources compile together here but have not
    yet ensured that transmit still works.
    
V1.00
130630
  - it compiles. It even times out. But it doesn't send anything
    to the PC - the TTYUSB LEDs don't blink at all
  - ARGHH. It does help to open the Serial1 port!!
  - but now it sends something to TTerm but TTerm must be answering
    with a NAK because they just repeat the same thing over
    and over again.

V2.00
130702
  - IT SENT A FILE!!!!
    It should have sent two, but I'll take it!
  - tried sending 2012/09 at 115200 - it sent the first file (138kB!)
    but hangs when it starts on the second one. The file is created
    but is zero length.
    
  - THIS VERSION SENDS MULTIPLE FILES

*/

#define error(s) printDebug(s)

extern int Filesleft;
extern long Totalleft;

extern SdFile fout;

static bool oneTime = false; // Display the Tera Term Change Directory note only once (so it does not get on people's nerves!)

// Dylan (monte_carlo_ecm, bitflipper, etc.) - This function was added because I found
// that SERIAL_TX_BUFFER_SIZE was getting overrun at higher baud rates.  This modified
// Serial.print() function ensures we are not overrunning the buffer by flushing if
// it gets more than half full.

size_t DSERIALprint(const __FlashStringHelper *ifsh)
{
  PGM_P p = reinterpret_cast<PGM_P>(ifsh);
  size_t n = 0;
  while (1) {
    unsigned char c = pgm_read_byte(p++);
    if (c == 0) break;
    if (DSERIAL->availableForWrite() > SERIAL_TX_BUFFER_SIZE / 2) DSERIAL->flush();
    if (DSERIAL->write(c)) n++;
    else break;
  }
  return n;
}

#define DSERIALprintln(_p) ({ DSERIALprint(_p); DSERIAL->write("\r\n"); })

void sdCardHelp(void)
{
  DSERIALprint(F("\r\n"));
  DSERIALprint(Progname);
  DSERIALprint(F(" - Transfer rate: "));
  DSERIAL->flush(); DSERIAL->println(settings.serialTerminalBaudRate); DSERIAL->flush();
  DSERIALprintln(F("Available Commands:")); DSERIAL->flush();
  DSERIALprintln(F("HELP     - Print this list of commands")); DSERIAL->flush();
  DSERIALprintln(F("DIR      - List files in current working directory - alternate LS")); DSERIAL->flush();
  DSERIALprintln(F("DEL file - Delete file - alternate RM")); DSERIAL->flush();
  DSERIALprintln(F("SZ  file - Send file from OLA to terminal using ZModem (\"SZ *\" will send all files)")); DSERIAL->flush();
  DSERIALprintln(F("SS  file - Send file from OLA using serial TX pin")); DSERIAL->flush();
  DSERIALprintln(F("CAT file - Type file to this terminal - alternate TYPE")); DSERIAL->flush();
  DSERIALprintln(F("X        - Exit to OpenLog Artemis Main Menu")); DSERIAL->flush();
  DSERIALprint(F("\r\n"));
}

SdFile root; // Copied from SdFat OpenNext example
SdFile fout;
//dir_t *dir ;

int count_files(int *file_count, long *byte_count)
{
  *file_count = 0;
  *byte_count = 0;

  if (!root.open("/"))
    return 0;

  root.rewind();

  while (fout.openNext(&root, O_RDONLY))
  {
    // read next directory entry in current working directory
    if (!fout.isDir())
    {
      *file_count = *file_count + 1;
      *byte_count = *byte_count + fout.fileSize();
    }
    fout.close();
  }

  root.close();
  
  return 0;
}

void sdCardMenu(int numberOfSeconds)
{
  sdCardHelp(); // Display the help
  
  unsigned long startTime = millis();

  bool keepGoing = true;
  
  while (keepGoing)
  {
    char *cmd = oneKbuf;
    char *param;
  
    *cmd = 0;

    while (DSERIAL->available()) DSERIAL->read();
    
    char c = 0;
    while(1)
    {
      if (DSERIAL->available() > 0)
      {
        startTime = millis(); // reset the start time if we receive a char
        c = DSERIAL->read();
        if ((c == 8 or c == 127) && strlen(cmd) > 0) cmd[strlen(cmd)-1] = 0;
        if (c == '\n' || c == '\r') break;
        DSERIAL->write(c);
        if (c != 8 && c != 127) strncat(cmd, &c, 1);
      }
      else
      {
        // Dylan (monte_carlo_ecm, bitflipper, etc.) -
        // This delay is required because I found that if I hard loop with DSERIAL.available,
        // in certain circumstances the Arduino never sees a new character.  Various forum posts
        // seem to confirm that a short delay is required when using this style of reading
        // from Serial
        checkBattery();
        delay(1);
        if ( (millis() - startTime) / 1000 >= numberOfSeconds)
        {
          SerialPrintln(F("No user input received."));
          return;
        }        
      }
    }
     
    param = strchr(cmd, 32);
    if (param > 0)
    {
      *param = 0;
      param = param + 1;
    }
    else
    {
      param = &cmd[strlen(cmd)];
    }
  
    strupr(cmd);
    DSERIAL->println();
  //  DSERIALprintln(command);
  //  DSERIALprintln(parameter);
  
    if (!strcmp_P(cmd, PSTR("HELP")))
    {     
      sdCardHelp();
    }
    
    else if (!strcmp_P(cmd, PSTR("DIR")) || !strcmp_P(cmd, PSTR("LS"))) // DIRectory
    {
      DSERIALprintln(F("\r\nRoot Directory Listing:"));

      if (DSERIAL == &Serial)
        sd.ls("/", LS_DATE | LS_SIZE); // Do a non-recursive LS of the root directory showing file modification dates and sizes
      else
        sd.ls(&Serial1, "/", LS_DATE | LS_SIZE);
  
      DSERIALprintln(F("End of Directory\r\n"));
    }
    
    else if (!strcmp_P(cmd, PSTR("DEL")) || !strcmp_P(cmd, PSTR("RM"))) // ReMove / DELete file
    {
      if (!sd.remove(param))
      {
        DSERIALprint(F("\r\nFailed to delete file "));
        DSERIAL->flush(); DSERIAL->println(param); DSERIAL->flush();
        DSERIALprintln(F("\r\n"));
      }
      else
      {
        DSERIALprint(F("\r\nFile "));
        DSERIAL->flush(); DSERIAL->print(param); DSERIAL->flush();
        DSERIALprintln(F(" deleted\r\n"));
      }
    }

    else if (!strcmp_P(cmd, PSTR("SZ"))) // Send file(s) using zmodem
    {
      if (!strcmp_P(param, PSTR("*")))
      {
       
        count_files(&Filesleft, &Totalleft);
        DSERIALprint(F("\r\nTransferring ")); DSERIAL->print(Filesleft); DSERIALprint(F(" files (")); DSERIAL->print(Totalleft); DSERIALprintln(F(" bytes)")); 
        
        root.open("/"); // (re)open the root directory
        root.rewind(); // rewind

        if (Filesleft > 0)
        {
          DSERIALprint(F("Starting zmodem transfer in ")); DSERIAL->print(settings.zmodemStartDelay); DSERIALprintln(F(" seconds..."));
          DSERIALprintln(F("(If you are using Tera Term, you need to start your File\\Transfer\\ZMODEM\\Receive now!)"));
          if (oneTime == false)
          {
            DSERIALprintln(F("(Also, if you are using Tera Term, you need to change the directory to something sensible."));
            DSERIALprintln(F(" Use File\\Change directory... to change where the received files are stored.)"));
            oneTime = true;
          }
          for (int i = 0; i < (((int)settings.zmodemStartDelay) * 1000); i++)
          {
            checkBattery();
            delay(1);
          }  
          
          sendzrqinit();
          for (int i = 0; i < 200; i++)
          {
            checkBattery();
            delay(1);
          }  
          
          //while (sd.vwd()->readDir(dir) == sizeof(*dir)) {
          while (fout.openNext(&root, O_RDONLY))
          {
            // read next directory entry in current working directory
            if (!fout.isDir()) {
              char fname[30];
              size_t fsize = 30;
              fout.getName(fname, fsize);
              //DSERIAL->print("fname: "); DSERIAL->println(fname);
              if (wcs(fname) == ERROR)
              {
                for (int i = 0; i < 500; i++)
                {
                  checkBattery();
                  delay(1);
                }  
                fout.close();
                break;
              }
              else
              {
                for (int i = 0; i < 500; i++)
                {
                  checkBattery();
                  delay(1);
                }  
              }
            }
            fout.close();
          }
          saybibi();
          DSERIALprintln(F("\r\nzmodem transfer complete!\r\n"));
        }
        else
        {
          DSERIALprintln(F("\r\nNo files found to send\r\n"));
        }

        root.close();
      }
      else
      {
        if (!fout.open(param, O_READ))
        {
          DSERIALprintln(F("\r\nfile.open failed!\r\n"));
        }
        else
        {
          DSERIALprint(F("\r\nStarting zmodem transfer in ")); DSERIAL->print(settings.zmodemStartDelay); DSERIALprintln(F(" seconds..."));
          DSERIALprintln(F("(If you are using Tera Term, you need to start your File\\Transfer\\ZMODEM\\Receive now!)"));
          if (oneTime == false)
          {
            DSERIALprintln(F("(Also, if you are using Tera Term, you need to change the directory to something sensible."));
            DSERIALprintln(F(" Use File\\Change directory... to change where the received files are stored.)"));
            oneTime = true;
          }
          for (int i = 0; i < (((int)settings.zmodemStartDelay) * 1000); i++)
          {
            checkBattery();
            delay(1);
          }  
            
          // Start the ZMODEM transfer
          Filesleft = 1;
          Totalleft = fout.fileSize();
          //ZSERIAL->print(F("rz\r"));
          sendzrqinit();
          for (int i = 0; i < 200; i++)
          {
            checkBattery();
            delay(1);
          }  
          wcs(param);
          saybibi();
          fout.close();
          DSERIALprintln(F("\r\nzmodem transfer complete!\r\n"));
        }
      }
    }

    else if (!strcmp_P(cmd, PSTR("SS"))) // Send file to serial TX pin
    {
      if (!fout.open(param, O_READ))
      {
        DSERIALprintln(F("\r\nfile.open failed!\r\n"));
      }
      else
      {
        settings.logA12 = false; //Disable analog readings on TX pin
        if (settings.useTxRxPinsForTerminal == false)
        {
          //We need to manually restore the Serial1 TX and RX pins
          configureSerial1TxRx();

          Serial1.begin(settings.serialLogBaudRate); // (Re)start the serial port
        }

        DSERIALprint(F("\r\nSending ")); DSERIAL->print(param); DSERIALprint(F(" to the TX pin at ")); DSERIAL->print(settings.serialLogBaudRate); DSERIALprintln(F(" baud"));

        while (fout.available())
        {
          char ch;
          if (fout.read(&ch, 1) == 1) // Read a single char
            Serial1.write(ch); // Send it via SerialLog (TX pin)
        }
        
        fout.close();
        DSERIALprintln(F("\r\nFile sent!\r\n"));
      }
    }

    else if (!strcmp_P(cmd, PSTR("CAT")) || !strcmp_P(cmd, PSTR("TYPE"))) // concatenate / type file to the terminal
    {
      if (!fout.open(param, O_READ))
      {
        DSERIALprintln(F("\r\nfile.open failed!\r\n"));
      }
      else
      {
        DSERIALprint(F("\r\n"));

        while (fout.available())
        {
          char ch;
          if (fout.read(&ch, 1) == 1) // Read a single char
            DSERIAL->write(ch); // Send it via SerialLog (TX pin)
        }
        
        fout.close();
        DSERIALprintln(F("\r\n"));
      }
    }

    else if (!strcmp_P(cmd, PSTR("X"))) // eXit to main menu
    {
      keepGoing = false;
    }
    
    startTime = millis(); // reset the start time after commanded action completes
  }
}

