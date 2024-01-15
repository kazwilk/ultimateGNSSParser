/*
  Receiving the NMEA 0183 data from ST Teseo-LIV3F GNSS receiver via software serial
  By: Kazimierz Wilk
  Date: January, 2024
  License: GNU Lesser General Public License. See license file for more information.

  This example shows how to receive the NMEA messages via software serial port from the UART of GNSS receiver.

  Open the serial monitor at 115200 baud to see the output
*/

/************************************************************************************************
 * this example was tested using the Nucleo GNSS1A1 board:      https://www.st.com/en/ecosystems/x-nucleo-gnss1a1.html
 * connected to the Arduino UNO board:                          https://store.arduino.cc/products/arduino-uno-rev3
 ************************************************************************************************/

#include <SoftwareSerial.h>
#include <ultimateGNSSParser.h>

SoftwareSerial mySerial(2, 8); // RX, TX


char outMessage[100];


// The macro SEND_COMMAND_TO_X_NUCLEO_GNSS1A1 is used to send every single NMEA command to the receiver:

#define SEND_COMMAND_TO_X_NUCLEO_GNSS1A1 \
  completeTheNMEAMessage(outMessage); \
  mySerial.print(outMessage); \
  delay(30); /* this is the delay for GNSS receiver to process the command */ \
  \
  sprintf(outMessage, "%s", "$PSTMSAVEPAR"); \
  \
  completeTheNMEAMessage(outMessage); \
  mySerial.print(outMessage); \
  delay(30); /* this is the delay for GNSS receiver to process the command */


int data_available_callback(void) {
  return (mySerial.available());
}

int data_read_callback(void) {
  return (mySerial.read());
}

class GNSSCollector myGPS(&data_available_callback, &data_read_callback);

// The 
int myNMEAParser (const struct NMEA_fields *paSlices) {
  
  if (!strcmp ("RMC",myGPS.get_field(paSlices,0)+3)) {
    //GNSSCollector::printFieldsStorage(paSlices); // for DEBUG purpose only
    return (0);  // the library will try to parse this message (if there is the particular parser)
  }
  
  if (!strcmp ("GNS",myGPS.get_field(paSlices,0)+3)) {
    //GNSSCollector::printFieldsStorage(paSlices); // for DEBUG purpose only
    return (1); // the library will not parse this message
  }
  
  if (!strcmp ("PSTMCPU",myGPS.get_field(paSlices,0)+1)) {
    //GNSSCollector::printFieldsStorage(paSlices); // for DEBUG purpose only
    return (1);  // the library will not parse this message
  }
  
  if (!strcmp ("ZDA",myGPS.get_field(paSlices,0)+3)) {
    //GNSSCollector::printFieldsStorage(paSlices); // for DEBUG purpose only
    return (1);  // the library will not parse this message
  }
  
  return (0);
}





void setup() {
  // Open serial communications and wait for port to open:
  //Serial.begin(230400);
  Serial.begin(115200);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  pinMode(4, OUTPUT);    // wakeup
  digitalWrite(4, HIGH);
  pinMode(7, OUTPUT);    // reset
  digitalWrite(7, HIGH);
  
  pinMode(2, INPUT);     // RxD
  
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  
  myGPS.setCustomParser(myNMEAParser);
  
  /*
  // my experiment proves that the Arduino UNO is too slow for bigger bitrates for NMEA messages
  // from the GNSS receiver (e.g. X_NUCLEO_GNSS1A1). The MAX bitrate when it still works is 19200
  // change the default bitrate 9600 to the 19200 sequence for the X_NUCLEO_GNSS1A1 board:
  sprintf(outMessage, "%s", "$PSTMSETPAR,3102,0x7");   // set the baudrate to the 19200 bps
  SEND_COMMAND_TO_X_NUCLEO_GNSS1A1
  sprintf(outMessage, "%s", "$PSTMSRR");   // system Reset - make sure the new bitrate is set
  SEND_COMMAND_TO_X_NUCLEO_GNSS1A1
  // the next step is to comment again this section and change the bitrate of mySerial to 19200,
  // then compile and flash again to the Arduino UNO board
  */
  
// These commands you can use as you wish (remember to run the MACRO SEND_COMMAND_TO_X_NUCLEO_GNSS1A1 after every single command to take effect):
  
//  sprintf(outMessage, "%s", "$PSTMSETPAR,1228,0x00002000,1"); // add GBS message to the output
//  sprintf(outMessage, "%s", "$PSTMSETPAR,1228,0x00002000,2"); // remove GBS message from the output
  
//  sprintf(outMessage, "%s", "$PSTMSETPAR,1201,0x00000008,1"); // add GST message to the output
//  sprintf(outMessage, "%s", "$PSTMSETPAR,1201,0x00000008,2"); // remove GST message from the output
  
  
//  sprintf(outMessage, "%s", "$PSTMSETPAR,1201,0x01000000,1"); // add ZDA message to the output
//  sprintf(outMessage, "%s", "$PSTMSETPAR,1201,0x01000000,2"); // remove ZDA message from the output

//  sprintf(outMessage, "%s", "$PSTMSETPAR,1201,0x00100000,1"); // add $GPGLL message to the output
//  sprintf(outMessage, "%s", "$PSTMSETPAR,1201,0x00100000,2"); // remove $GPGLL message from the output


//  sprintf(outMessage, "%s", "$PSTMSETPAR,1201,0x00000100,2"); // remove $PSTMTG message from the output
//  sprintf(outMessage, "%s", "$PSTMSETPAR,1201,0x00000200,2"); // remove $PSTMTS message from the output
//  sprintf(outMessage, "%s", "$PSTMSETPAR,1201,0x00004000,2"); // remove $PSTMWAAS message from the output
//  sprintf(outMessage, "%s", "$PSTMSETPAR,1201,0x00020000,2"); // remove $PSTMSBAS message from the output
//  sprintf(outMessage, "%s", "$PSTMSETPAR,1201,0x02000000,2"); // remove $PSTMTRAIMSTATUS message from the output



//  SEND_COMMAND_TO_X_NUCLEO_GNSS1A1
}


// loop1 is PASTHROOUGH
void loop1() { // run over and over
  if (mySerial.available()) {
    //Serial.print(mySerial.read(), HEX);Serial.print(" ");
    Serial.write(mySerial.read());
  }
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
}


void loop2() {
  char local_string[100];
  const struct GNSS_data *all_GNSS_data;
  
//  myGPS.GSVSwitch(true); // this will not work on Arduino UNO with 2 KB of RAM

  Serial.println("............................................................................................................................................................");
  myGPS.collectData(false,false); // false arguments turns off the debug messages from the output due to slow Arduino UNO CPU
                                  // if you have some stronger CPU on your Arduino board, you can give double true to the method
                                  // Note that these flags turn on the messages for debug purpose only

  // this is the method for debug purpose only. You can reach the data by the struct GNSS_data given by the GNSSCollector::getGNSSData() method
  myGPS.printGNSSData(true);
  Serial.println("____________________________________________________________________________________________________________________________________________________________");
  
  all_GNSS_data = myGPS.getGNSSData();
  Serial.println(('A' == all_GNSS_data->pos_status)?"Position is valid":">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> The position is unknown <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  
  if ('A' != all_GNSS_data->pos_status) {
    return;
  }
  
  double velocity = all_GNSS_data->nautical_speed*1.852;
  
  sprintf(local_string,"https://www.google.com/maps?q=%s%c,%s%c,v=%s,a=%s",String(all_GNSS_data->lat, 5).c_str(), all_GNSS_data->lat_dir, String(all_GNSS_data->lon, 5).c_str(), all_GNSS_data->lon_dir,String(velocity, 1).c_str(),String(all_GNSS_data->alt, 1).c_str());
  //sprintf(local_string,"{\"p\":\"%s%c,%s%c\",\"v\":\"%s\",\"a\":\"%s\",\"t\":\"%d:%02d:%02d\"}",String(all_GNSS_data->lat, 5).c_str(), all_GNSS_data->lat_dir, String(all_GNSS_data->lon, 5).c_str(), all_GNSS_data->lon_dir,String(velocity, 1).c_str(),String(all_GNSS_data->alt, 1).c_str(),all_GNSS_data->UTC_H,all_GNSS_data->UTC_M,all_GNSS_data->UTC_S);
  Serial.print("Position Info: (");
  Serial.print(strlen(local_string));
  Serial.print("): ");
  Serial.println(local_string);
}

void loop () {
  loop2();
}
