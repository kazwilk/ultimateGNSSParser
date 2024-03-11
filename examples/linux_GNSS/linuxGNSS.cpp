/*
  Receiving the NMEA 0183 data from GNSS receiver serial port device in linux environment (e.g. /dev/ttyS0, /dev/ttyACM0, /dev/TTYUSB0, etc.)
  By: Kazimierz Wilk
  Date: January, 2024
  License: GNU Lesser General Public License. See license file for more information.
  
  This example shows how to receive the NMEA messages via hardware serial port in the linux environment.
  
  The default serial transmission for the most receivers is 9600 bps.
  Run the program with -h or --help option to check the list of its arguments
*/


#include <getopt.h>
#include <fcntl.h>


#include <ctype.h>
#include <stdlib.h>
#include <time.h>

#include <math.h>


#include <ultimateGNSSParser.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#include "serial_port_control.h"


int fd = -1;    // Serial port file descriptor



/************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************/

void help_screen (const char * const progname) {
  fprintf (stderr, "This tool receive data from GPS receiver connected to pointed serial port.\n");
  fprintf (stderr, "Program usage: %s [options sequence]\n\n", progname);
  fprintf (stderr, "Program options:\n");
  fprintf (stderr, "\t\t-D\t\t--device\t\tthe serial port device (e.g. /dev/ttyACM0, /dev/ttyS0 or /dev/ttyUSB0; - is stdin) program will receive data from\n");
  fprintf (stderr, "\t\t-s\t\t--speed\t\tthe serial port baudrate (9600, 115200)\r\n");
  fprintf (stderr, "\t\t-h\t\t--help\t\t\tprint this help screen\n");
  fprintf (stderr, "\t\t-m\t\t--monitor\t\ttwo modes of serial port monitoring:\n\t\t\t\t\t0\t-\tprint every byte received from serial port.\n\t\t\t\t\t1\t-\tASCII text\n");
  fprintf (stderr, "\t\t-v\t\t--verbosity\tincreasing verbosity level\n\n");
}


/************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************/

void print_character(const char paCharacter, bool paNewLine) {
  switch (paCharacter) {
    case '\r':
      fprintf(stderr, "\\r");
      break;
    case '\n':
      fprintf(stderr, "\\n");
      if (paNewLine)
        fprintf(stderr, "\n");
      break;
    default:
      if (! paNewLine)
        fprintf(stderr, " ");
      fprintf(stderr, "%c", paCharacter );
  }
}

void serial_port_monitor (int mode) {
  
  char loBuf [5];
  int n ;
  int bufn;
  struct timespec tp_old, tp_now;
  double result;
  
  bzero(&tp_now, sizeof(struct timespec));
  bzero(&tp_old, sizeof(struct timespec));
  
  //###########################################################################################################################
  if (0 == mode) {
    do {
      n = read (fd, loBuf, 1);  // read 1 character if ready to read
      
      if (0 < n ) {
        clock_gettime(CLOCK_MONOTONIC, &tp_now);
        bufn = (int)((char)*loBuf);
        
        fprintf(stderr, "Read: %4u (0x%02x)  (", (unsigned char)bufn, (unsigned char)bufn );
        print_character(bufn, false);
        fprintf(stderr, ")   ");
        if ((0 == tp_old.tv_sec) && (0 == tp_old.tv_nsec)) {
          memcpy(&tp_old, &tp_now, sizeof (struct timespec));
          continue;
          fprintf(stderr, "\n");
        }
        result = (tp_now.tv_sec - tp_old.tv_sec) * 1e3 + (tp_now.tv_nsec - tp_old.tv_nsec) / 1e6; // time in milliseconds
        fprintf(stderr, " received %8.3lf milliseconds since previous one\n", result);
        memcpy(&tp_old, &tp_now, sizeof (struct timespec));
      }
    } while (1);
  } else {
    do {
      n = read (fd, loBuf, 1);  // read 1 character if ready to read
      
      if (0 < n ) {
        print_character(*loBuf,true);
      }
    } while (1);
  }
  return;
}

/************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************/

int verbosity;

static struct option long_options[] = {
                                        {"device",    required_argument, 0, 'D'},
                                        {"speed",     required_argument, 0, 's'},
                                        {"help",      no_argument,       0, 'h'},
                                        {"monitor",   required_argument, 0, 'm'},
                                        {"verbosity", no_argument,       0, 'v'},
                                        {0,           0,                 0,  0 }
};


int program_parameters_parser (const int argc, char * const argv[]) {
  
  int c;
  unsigned int serial_speed = 9600;
  
  const char *loFileName;
  
  int loMonitor = -1;
  
  if (argc < 2) {
    help_screen(argv[0]);
    return(0);
  }
  
  while (1) {
    int option_index = 0;
    
    c = getopt_long(argc, argv, "D:s:hm:v", long_options, &option_index);
    if (c == -1)
      break;
    
    switch (c) {
      case 'D':
              loFileName = optarg;
              break;
      case 'h':
              help_screen(argv[0]);
              exit(0);
              break;
      case 's':
              if (NULL == optarg) {
                fprintf(stderr, "The speed parameter needs the argument\r\n");
                break;
              }
              serial_speed = atoi(optarg);
              break;
      case 'm':
              loMonitor = ((NULL==optarg)?0:(atoi(optarg)?1:0));
              break;
      case 'v':
              verbosity += 1;
              if (1 < verbosity) fprintf(stderr, "Verbosity level is now %u\n", verbosity);
              break;
      case '?':
              printf("??? read carefully how to give me the parameters\n");
              help_screen(argv[0]);
              exit(0);
              break;
      default:
              printf("?? getopt returned character code 0%o ??\n", c); // it shouldn't happen
    } /* switch(c) */
  } /* while(1) */
  
  
  printf("Using device %s\n", loFileName);
  if (!strcmp("-", loFileName)) {
    fd = STDIN_FILENO;
  } else {
    fd = open (loFileName, O_RDWR | O_NOCTTY | O_SYNC);
  }
  if (0 > fd) {
    fprintf (stderr, "Error %d opening %s: %s\r\n", errno, loFileName, strerror (errno));
    exit (-1);
  }
  
  if (spc_set_interface_attribs (fd, serial_speed ) ) {// set speed to (default=9,600) bps, 8n1 (no parity)
    exit (-1);
  }
  if ( spc_set_blocking (fd, 0) ) {                // set  non blocking
    exit (-1);
  }
  
  if (-1 != loMonitor)
    serial_port_monitor(loMonitor);
  
  return (0);
}


/************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************/

int8_t data_available_callback(void) {
  int bytes_available;
  
  if (0 != ioctl (fd, FIONREAD, &bytes_available))
    bytes_available = 0;
  
  if (0 < bytes_available)
    return (1);
  else
    return (0);
}

/************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************/

int8_t data_read_callback(void) {
  static int lobuf;
  
  read(fd, &lobuf, 1);
  return ((int8_t)lobuf);
}

/************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************/

class GNSSCollector myGPS(&data_available_callback, &data_read_callback);


int8_t myNMEAParser (const struct NMEA_fields *paSlices) {

//return (1); // Do not parse any sentence by the library, but library checks the checksums and slices the messages,
// then you can write any parser in your own
  
  if (!strcmp ("TXT",myGPS.get_field(paSlices,0)+3)) {
//    GNSSCollector::printFieldsStorage(paSlices); // for DEBUG purpose only
    return (1);
  }
  
  if (!strcmp ("RMC",myGPS.get_field(paSlices,0)+3)) {
//    GNSSCollector::printFieldsStorage(paSlices); // for DEBUG purpose only
    return (0);
  }
  
  if (!strcmp ("GGA",myGPS.get_field(paSlices,0)+3)) {
//    GNSSCollector::printFieldsStorage(paSlices); // for DEBUG purpose only
    return (0);
  }
  
  if (!strcmp ("EPE",myGPS.get_field(paSlices,0)+3)) {
//    GNSSCollector::printFieldsStorage(paSlices); // for DEBUG purpose only
    return (1);
  }
  
  if (!strcmp ("ZDA",myGPS.get_field(paSlices,0)+3)) {
//    GNSSCollector::printFieldsStorage(paSlices); // for DEBUG purpose only
    return (1);
  }
  
  if (!strcmp ("GST",myGPS.get_field(paSlices,0)+3)) {
//    GNSSCollector::printFieldsStorage(paSlices); // for DEBUG purpose only
    return (0);
  }
  
  if (!strcmp ("ACCURACY",myGPS.get_field(paSlices,0)+3)) {
//    GNSSCollector::printFieldsStorage(paSlices); // for DEBUG purpose only
    return (1);
  }
  
  return (0);
}


/************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************
 ************************************************************************************************************************/

int main (int argc, char *argv[]) {
  
  const struct GNSS_data *all_GNSS_data;
  char double_string[100];
  double max_error;
  
  
  if (program_parameters_parser(argc, argv)) {
    return(-5);
  }
  
//#include "NMEA_commands.h"
  
  myGPS.GSVSwitch(true);
  myGPS.setBreakTime(35); // The ATGM336H needs longer period here, default value is correct for most receivers
  
  myGPS.setCustomParser(myNMEAParser);
  
  while (1) {
    fprintf(stderr,"............................................................................................................................................................\r\n");
    myGPS.collectData((0 < verbosity)?true:false,(1<verbosity)?true:false);
    all_GNSS_data = myGPS.getGNSSData();
    if (2 < verbosity)
      myGPS.printGSVData(true);
    myGPS.printGNSSData(true);
    fprintf(stderr,"____________________________________________________________________________________________________________________________________________________________\r\n");
    
    max_error = sqrt((all_GNSS_data->lat_err*all_GNSS_data->lat_err) + (all_GNSS_data->lon_err*all_GNSS_data->lon_err));
    fprintf(stderr, "Expected horizontal error    : %lf [m]\r\n", max_error);
    
    max_error = sqrt((all_GNSS_data->lat_std_dev*all_GNSS_data->lat_std_dev) + (all_GNSS_data->lon_std_dev*all_GNSS_data->lon_std_dev));
    fprintf(stderr, "Horizontal standard deviation: %lf [m]\r\n", max_error);
    
    if (all_GNSS_data->msgs_rcvd[MSG_RMC] && all_GNSS_data->msgs_rcvd[MSG_GGA]) {
      fprintf(stderr, ('A' == all_GNSS_data->pos_status)?"\033[92mPosition is valid\033[39m\r\n":"\033[91mUnknown position\033[39m\r\n");
      double velocity = all_GNSS_data->nautical_speed*1.852;
      sprintf(double_string,"https://www.google.com/maps?q=%0.11lf%c,%0.11lf%c",all_GNSS_data->lat, all_GNSS_data->lat_dir, all_GNSS_data->lon, all_GNSS_data->lon_dir);
      //sprintf(double_string,"https://www.google.com/maps?q=%0.11lf%c,%0.11lf%c&v=%0.1lf&a=%0.0lf",all_GNSS_data->lat, all_GNSS_data->lat_dir, all_GNSS_data->lon, all_GNSS_data->lon_dir,velocity,all_GNSS_data->alt);
      //sprintf(double_string,"{\"p\":\"%0.5lf%c,%0.5lf%c\",\"v\":\"%0.1lf\",\"a\":\"%0.0lf\",\"t\":\"%d:%02d:%02d\"}",all_GNSS_data->lat, all_GNSS_data->lat_dir, all_GNSS_data->lon, all_GNSS_data->lon_dir,velocity,all_GNSS_data->alt,all_GNSS_data->UTC_H,all_GNSS_data->UTC_M,all_GNSS_data->UTC_S);
      fprintf(stderr, "Example message (");
      fprintf(stderr, "%ld", strlen(double_string));
      fprintf(stderr, "): ");
      fprintf(stderr, "%s\r\n", double_string);
    }
  } // while (1)
  return (0);
}

