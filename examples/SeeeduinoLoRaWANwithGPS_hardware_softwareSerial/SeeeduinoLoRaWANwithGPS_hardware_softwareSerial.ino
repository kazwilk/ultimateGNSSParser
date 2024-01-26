/*
  Receiving the NMEA 0183 data from GNSS receiver via hardware serial (UART2) or SoftwareSerial from external module e.g. Grove-GPS module connected by Grove A0 or Grove A2 socket
  By: Kazimierz Wilk
  Date: January, 2024
  License: GNU Lesser General Public License. See license file for more information.

  This example shows how to receive the NMEA messages via hardware serial port from the UART output of GNSS receiver and SoftwareSerial from Grove modules.

  Open the serial monitor at 115200 baud to see the output
*/

// This example is tested with the three GPS/GNSS receivers:
// 1st Quectel L70 module placed on https://wiki.seeedstudio.com/Seeeduino_LoRAWAN/   connected by hardware UART to https://wiki.seeedstudio.com/Seeeduino_LoRAWAN/
// 2nd https://wiki.seeedstudio.com/Grove-GPS-Air530/                                 connected by Grove connector (SoftwareSerial) to https://wiki.seeedstudio.com/Seeeduino_LoRAWAN/
// 3rd https://wiki.seeedstudio.com/Grove-GPS/                                        connected by Grove connector (SoftwareSerial) to https://wiki.seeedstudio.com/Seeeduino_LoRAWAN/
// The Arduino board used in all these cases is: https://wiki.seeedstudio.com/Seeeduino_LoRAWAN/

/*
   There is the 2nd version of Seeeduino LoRaWAN board without GNSS Quectel L70 module.
   You can use some other module in that case, e.g. Grove module:
   https://wiki.seeedstudio.com/GPS-Modules-Selection-Guide/
   this one: https://wiki.seeedstudio.com/Grove-GPS/
   or this:  https://wiki.seeedstudio.com/Grove-GPS-Air530/
*/

// comment this line if you use onboard Quectel L70 module
//#define GROVE


#ifdef GROVE
#include <SoftwareSerial.h>
#endif

#include <ultimateGNSSParser.h>

#ifdef GROVE
SoftwareSerial SoftSerial(A0, A1);  // Grove connector A0
//SoftwareSerial SoftSerial(A2, A3);  // Grove connector A2
#endif

int8_t data_available_callback(void) {
#ifdef GROVE  
  return (SoftSerial.available());
#else
  return (Serial2.available());
#endif
}

int8_t data_read_callback(void) {
#ifdef GROVE
  return (SoftSerial.read());
#else
  return (Serial2.read());
#endif
}

class GNSSCollector myGPS(&data_available_callback, &data_read_callback);

void setup() {
#ifdef GROVE
    SoftSerial.begin(9600);                 // the SoftSerial baud rate for Grove modules
#else
    Serial2.begin(9600);                    // the Hardware UART baud rate for Quectel L70 onboard module
#endif
    Serial.begin(115200);                   // the Serial port of Arduino baud rate.
    digitalWrite(ATN,HIGH);                 // Power on GROVE connector
}


const struct GNSS_data *all_GNSS_data;
char double_string[100];


void loop1() { // The DEMO of ultimateGNSSParser library
  Serial.println("............................................................................................................................................................");
  Serial.println("............................................................................................................................................................");
  Serial.println("............................................................................................................................................................");
  myGPS.collectData(true, true);
  all_GNSS_data = myGPS.getGNSSData();
  myGPS.printGNSSData(true);
  Serial.println("____________________________________________________________________________________________________________________________________________________________");
  Serial.println("____________________________________________________________________________________________________________________________________________________________");
  Serial.println("____________________________________________________________________________________________________________________________________________________________");
  
  if ('A' != all_GNSS_data->pos_status) {
    return;
  }
  
  Serial.println(('A' == all_GNSS_data->pos_status)?"Position is valid":"Unknown position");
  double velocity = all_GNSS_data->nautical_speed*1.852;
  sprintf(double_string,"https://www.google.com/maps?q=%0.5lf%c,%0.5lf%c&v=%0.1lf,a=%0.0lf",all_GNSS_data->lat, all_GNSS_data->lat_dir, all_GNSS_data->lon, all_GNSS_data->lon_dir,velocity,all_GNSS_data->alt);
  //sprintf(double_string,"{\"p\":\"%0.5lf%c,%0.5lf%c\",\"v\":\"%0.1lf\",\"a\":\"%0.0lf\",\"t\":\"%d:%02d:%02d\"}",all_GNSS_data->lat, all_GNSS_data->lat_dir, all_GNSS_data->lon, all_GNSS_data->lon_dir,velocity,all_GNSS_data->alt,all_GNSS_data->UTC_H,all_GNSS_data->UTC_M,all_GNSS_data->UTC_S);
  Serial.print("Example message (");
  Serial.print(strlen(double_string));
  Serial.print("): ");
  Serial.println(double_string);
}


// You can observe what data is received from the GPS/GNSS module
void loop2() { // PASSTHROUGH
    if (data_available_callback()) {             // if date is coming from software serial port ==> data is coming from SoftSerial shield
      Serial.write(data_read_callback());
    }
    if (Serial.available())                   // if data is available on hardware serial port ==> data is coming from PC or notebook
#ifdef GROVE
      SoftSerial.write(Serial.read());        // write it to the SoftSerial shield
#else
      Serial2.write(Serial.read());
#endif
}

void loop () {
  loop1();
}
