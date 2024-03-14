#include <TimeOut.h>
#include <ultimateGNSSParser.h>  // https://github.com/kazwilk/ultimateGNSSParser


static volatile int TOGNSSSemaphore=0;  // TimeOut GNSS Semaphore
const int GNSSResponseTimeout = 3000;   // 3000 miliseconds

void callbackGNSSTO() {
  Serial.println("GNSS timeout");
  TOGNSSSemaphore=1;
}

TimeOut timeoutGNSSTO;

int8_t data_available_callback(void) {
  int loAvailable = Serial2.available();
  if (0 < loAvailable) {
    return (1);
  }
  
  TimeOut::handler();
  if (1 == TOGNSSSemaphore) {
    Serial.println("+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+- TIMEOUT has occured while waiting for data from GNSS");
    return (-1); // We are waiting too long - there is no response from GNSS module any more
  }
  return (0);
}

int8_t data_read_callback(void) {
  return (Serial2.read());
}

class GNSSCollector myGNSS(&data_available_callback, &data_read_callback);

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  
  Serial.println("GNSS UART is ready to work");
}

void loop (void) {
  const struct GNSS_data *all_GNSS_data;
  
  // We have to empty input buffer from old data, because we need very fresh time data
  while (0 != Serial2.available()) {
    Serial2.read();
  }
  
  timeoutGNSSTO.cancel(); // Do it just before TOSemaphore settings !!!
  TOGNSSSemaphore = 0;
  
  timeoutGNSSTO.timeOut(GNSSResponseTimeout, callbackGNSSTO); // Timeout after some milliseconds
  
  myGNSS.collectData(true, true);
  timeoutGNSSTO.cancel(); // Something is received by external UART from GNSS
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> received GNSS data >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  myGNSS.printGNSSData(true);
  Serial.println("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  
  all_GNSS_data = myGNSS.getGNSSData();
  
  if (all_GNSS_data->msgs_rcvd[MSG_GGA]) {
    Serial.println("$GPGGA message received");
  }
}