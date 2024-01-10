/*
  Receiving the NMEA 0183 data from u-Blox GNSS receiver via I2C
  By: Kazimierz Wilk
  Date: January, 2024
  License: GNU Lesser General Public License. See license file for more information.

  This example shows how to configure the u-blox GNSS receiver via I2C and receive NMEA data for the ultimateGNSSParser library.

  Open the serial monitor at 115200 baud to see the output
*/


// This example has been tested with these modules: MAX-M10S, NEO-M9N, ZED-F9P, NEO-M8P-2, SAM-M8Q

// The 1st board: https://www.sparkfun.com/products/18037   SparkFun GNSS Receiver Breakout - MAX-M10S (Qwiic)
// The 2nd board: https://www.sparkfun.com/products/15712   SparkFun GPS Breakout - NEO-M9N, U.FL (Qwiic)
// The 3rd board: https://www.sparkfun.com/products/22560   SparkFun GNSS Combo Breakout - ZED-F9P, NEO-D9S (Qwiic)
// The 4th board: https://www.sparkfun.com/products/15005   SparkFun GPS-RTK Board - NEO-M8P-2 (Qwiic)
// The 5th board: https://store.arduino.cc/products/arduino-mkr-gps-shield  and https://store.arduino.cc/products/arduino-mkr-wan-1310 connected via included I2C cable

// The first four boards was connected to the Arduino UNO R4 WiFi https://store.arduino.cc/products/uno-r4-wifi by qwiic cable


/*****************************************************************************************************************/
// this is correct for Arduino UNO R4 WiFi using qwiic connector
// qwiic connector is wired to the Arduino board's second I2C bus, so there in the C++ code is Wire1 instead of default Wire
#define MYWIREBUS Wire1

// this is correct for Arduino MKR series using i2c cable
//#define MYWIREBUS Wire
/*****************************************************************************************************************/

// This example should work with: MAX-M10S, NEO-M8P-2, NEO-M9N, SAM-M8Q, ZED-F9K, ZED-F9P, ZED-F9R, ZED-F9T, ZOE-M8Q  -> !!!! CONNECTED BY I2C !!!!


#include <ultimateGNSSParser.h>

#include <SparkFun_u-blox_GNSS_v3.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3

class SparkFun_UBLOX_GNSS::SfeI2C *myGNSSModule;


uint8_t localReadI2CBuffer[50];
volatile uint8_t localDataSize = 0;
volatile uint8_t localHeadPtr = 0;

int8_t data_available_callback(void) {
  if (0 < localDataSize)
    return (1);
    if (0 < myGNSSModule->available()) {
      return (1);
    }
    return (0);
}

int8_t data_read_callback(void) {
  if (0 < localDataSize) {
    localDataSize--;
    return (localReadI2CBuffer[localHeadPtr++]);
  } else {
    localDataSize = myGNSSModule->readBytes(localReadI2CBuffer, 40); // 40 is optimal, don't use huge transmissions via i2c, it will not work
    localHeadPtr = 0;
    if (0 < localDataSize) {
      localDataSize--;
      return(localReadI2CBuffer[localHeadPtr++]);
    } else {
      return('\n'); // it shouldn't happen
    }
  }
}

class GNSSCollector myParser(&data_available_callback, &data_read_callback);

void setup() {
//  Serial.begin(230400);
  Serial.begin(115200);
  delay(300);
  Serial.println("Ultimate GNSS Parser with uBlox module");
  
  MYWIREBUS.begin();
  myGNSSModule = new SparkFun_UBLOX_GNSS::SfeI2C();
  if (NULL == myGNSSModule) {
    Serial.println("The I2C connection can not work");
    while (1);
  }
  
  if (false == myGNSSModule->init(MYWIREBUS, 0x42) ) {
    Serial.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  
  // This magic code (see the manual for more details) is useful only for uBlox modules. It doesn't work with the Mediatek, ST or other one.
  // This code turns on the GBS message for current session only. After the power off sequence, it is necessary to send it again to the module.
  const uint8_t Enable_GBS_NMEA_Message[] = {0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x09, 0x01, 0x04, 0x22};
  
  // This magic code (see the manual for more details) is useful only for uBlox modules. It doesn't work with the Mediatek, ST or other one.
  // This code turns on the GST message for current session only. After the power off sequence, it is necessary to send it again to the module.
  const uint8_t Enable_GST_NMEA_Message[] = {0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x07, 0x01, 0x02, 0x1e};
  
  // This magic code (see the manual for more details) is useful only for uBlox modules. It doesn't work with the Mediatek, ST or other one.
  // This code turns on the high precision for latitude and longitude fields - adds 2 extra digits for current session only. After the power off sequence, it is necessary to send it again to the module.
  const uint8_t Enable_CFG_NMEA_HIGHPREC_RAM_uBlox_ZEDF9P[]  = {0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x06, 0x00, 0x93, 0x10, 0x01, 0x44, 0x29};
  
  
  // the GBS and GST messages are very useful. They present the expected position errors in latitude, longitude and altitude in meters
  // this message could be present even with NEO-6M (the original uBlox module - there are many fake NEO-6M on the market actually) - tested with NEO-6M and it works
  myGNSSModule->writeBytes((uint8_t *)Enable_GBS_NMEA_Message, sizeof(Enable_GBS_NMEA_Message));
  
  delay(200); // this is the delay for GNSS receiver to process the magic code
  
  // ... and again for the NMEA GST message enable
  myGNSSModule->writeBytes((uint8_t *)Enable_GST_NMEA_Message, sizeof(Enable_GST_NMEA_Message));
  
  delay(200); // this is the delay for GNSS receiver to process the magic code
  
  // ... and again for the CFG_NMEA_HIGHPREC_RAM
  myGNSSModule->writeBytes((uint8_t *)Enable_CFG_NMEA_HIGHPREC_RAM_uBlox_ZEDF9P, sizeof(Enable_CFG_NMEA_HIGHPREC_RAM_uBlox_ZEDF9P));
  
  delay(200); // this is the delay for GNSS receiver to process the magic code

  // The GSV ("GNSS Satellites in View") messages are irrelevant in most projects and consumes huge RAM amount, so use them carefully.
  // This can be used to graphic imaging satellites positions (linux applications ???), or for some type of analysis
  myParser.GSVSwitch(true);
}

void loop1() {
  uint8_t localBuf[80];
  uint8_t loRead;
  while(myGNSSModule->available()) {
    loRead = myGNSSModule->readBytes(localBuf, 40);
    Serial.write(localBuf, loRead);
  }
}

void loop2() {
  while(data_available_callback()) {
    Serial.write(data_read_callback());
  }
}

void loop3() {
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  // the callback functions "data_available_callback()" and "data_read_callback()" are connected to the object myParser
  myParser.collectData(true,true); // these boolean parameters you can use for debug mode to print the processed data
  
  // this function presents the GSV data - it's very sophisticated and irrelevant in most GNSS projects
  myParser.printGSVData(true);
  myParser.printGNSSData(true); // this method prints all collected data (except GSV - "GNSS Satellites in View") - it's useful for debug mode, but in your project you can use getGNSSData and take the only data you need
  
  // Here is the example how to access the collected data:
    const struct GNSS_data  *myData;
    myData = myParser.getGNSSData(); // look at this method and feel free to use any parameter from the storage structure
    Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> The speed is "); Serial.print(myData->speed, DEC); Serial.println(" [kmph]");
  Serial.println("____________________________________________________________________________________________________________________________________________________________");  
}

void loop() {
  
  //  You can use one of the loops presented to better understand data flow:
  
  loop2();
  
}
