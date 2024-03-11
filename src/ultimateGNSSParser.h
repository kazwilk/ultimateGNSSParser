/*
  This file is a part of the ultimateGNSSParser library.
  Copyright (c) 2024 Kazimierz Wilk. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef GNSS_PARSER_H
#define GNSS_PARSER_H

#ifdef ARDUINO
#include <Arduino.h>
#define DBG(msg)        Serial.print(F(msg));
#define DBGV(msg)       Serial.print(msg);
#define DBGC(msg)       Serial.print(msg);
#define DBGT(msg, TYPE) Serial.print(msg, TYPE);

#define NOCOLOR             {;}
#define SETCOLORRED         {;}
#define SETCOLORGREEN       {;}
#define SETCOLORYELLOW      {;}
#define SETCOLORBLUE        {;}
#define SETCOLORPURPLE      {;}
#define SETCOLORCYAN        {;}
#define SETCOLORLIGHTGRAY   {;}
#define SETCOLORBLACK       {;}
#define SETCOLORDEFAULT     {;}


#elif __linux__
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>    // stderr
#include <unistd.h>   // usleep
#define DBG(msg)    fprintf(stderr, "%s", msg);
#define DBGV(msg)   fprintf(stderr, "%s", msg);
#define DBGC(msg)   fprintf(stderr, "%c", msg);
#define DEC "%d"
#define HEX "%X"
#define DBGT(msg, TYPE)  fprintf(stderr, TYPE, msg);


#define NOCOLOR             fprintf(stderr, "\033[39m");
#define SETCOLORRED         fprintf(stderr, "\033[91m");
#define SETCOLORGREEN       fprintf(stderr, "\033[92m");
#define SETCOLORYELLOW      fprintf(stderr, "\033[93m");
#define SETCOLORBLUE        fprintf(stderr, "\033[94m");
#define SETCOLORPURPLE      fprintf(stderr, "\033[95m");
#define SETCOLORCYAN        fprintf(stderr, "\033[96m");
#define SETCOLORLIGHTGRAY   fprintf(stderr, "\033[97m");
#define SETCOLORBLACK       fprintf(stderr, "\033[98m");
#define SETCOLORDEFAULT     fprintf(stderr, "\033[99m");


#else

#error "The library works on ARDUINO or LINUX platform"

#endif


inline void printPreciselyDouble(double paValue) {
// The Latitude and Longitude are given with the precision of 0.00001 minutes.
// This is the 0.0000001(6) of degrees, so we have to print the position with 8 digits of fraction,
// but in RTK mode it's good to turn on two extra digits for better angle precision.
// The u-blox RTK module ZED-F9P allows to do it with CFG_NMEA_HIGHPREC,
// That's the reason for printing 10 digits of the fraction
// In that case the NMEA messages give the angles with precision of 0.0000001 minutes.
// This is the 0.000000001(6) of degrees.
// Note that the precision can be reached only on platforms with 8 bytes size of the double type.
// The double type on Arduino UNO has size of 4 bytes.
// Some receivers (e.g. Unicorecomm UM980 RTK with 8mm precision) prints the Latitude and Longitude
// with the precision of 0.00000001 minutes, so that's the 0.0000000001(6) of degrees.

#ifdef ARDUINO
  DBGV(String(paValue, 11).c_str());
#elif __linux__
  fprintf(stderr, "%.11lf", paValue);
#endif
  return;
}

inline void printDouble(double paValue) {
#ifdef ARDUINO
  DBGV(String(paValue, 3).c_str());
#elif __linux__
  fprintf(stderr, "%2.3lf", paValue);
#endif
  return;
}


#define PRN_SATS_MAX 6  // we have 6 GNSS constellations (GPS, GLONASS, Galileo, BeiDou, QZSS, NavIC)

const uint8_t MAXMESSAGELENGTH   = 100;  // Maximum sentence length is limited to 82 characters according to the NMEA restrictions, but sometimes they're longer
const uint8_t MAXFIELDSINMESSAGE = 30;

struct NMEA_fields {
  char message[MAXMESSAGELENGTH];
  uint8_t field_index[MAXFIELDSINMESSAGE]; // indexes of the first bytes of the fields
  uint8_t cnt=0;                           // fields counter
  int8_t chSum;
};

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 *************************************************** the data structures to store the main data   **************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

// $xxGSA message data storage:
struct PRN_sats {
  uint16_t prn_gps[12];  // PRN numbers of satellites
                         // used in solution (null for unused fields),
                         // total of 12 sats
  int16_t systemID;      // system ID is not available in every GNSS receiver (-1 means it's unavailable)
  char talker[3];        // talker ID from message header
};


// the order depends on the frequency of abbreviation to save CPU time
const uint8_t MSG_GSV=0;
const uint8_t MSG_GSA=1;
const uint8_t MSG_RMC=2;
const uint8_t MSG_GGA=3;
const uint8_t MSG_VTG=4;
const uint8_t MSG_GLL=5;
const uint8_t MSG_GBS=6;
const uint8_t MSG_GST=7;
const uint8_t MSG_MAX=8;


// This is the main structure containing whole NMEA messages data except of $xxGSV data
struct GNSS_data {
  uint8_t   UTC_H;         // hour                                        $xxRMC, $xxGGA,                 $xxGBS, $xxGLL
  uint8_t   UTC_M;         // minute                                      $xxRMC, $xxGGA,                 $xxGBS, $xxGLL
  uint8_t   UTC_S;         // seconds (not float storage)                 $xxRMC, $xxGGA,                 $xxGBS, $xxGLL
  uint16_t  UTC_fract;     //                                             $xxRMC, $xxGGA,                 $xxGBS, $xxGLL
  char    pos_status;      // position status  A=active or V=void         $xxRMC,                                 $xxGLL
  double  lat;             // Latitude                                    $xxRMC, $xxGGA,                         $xxGLL
  char    lat_dir;         // latitude direction                          $xxRMC, $xxGGA,                         $xxGLL
  double  lon;             // longitude                                   $xxRMC, $xxGGA,                         $xxGLL
  char    lon_dir;         // longitude direction                         $xxRMC, $xxGGA,                         $xxGLL

  uint16_t year;           //                                             $xxRMC
  uint8_t  month;          //                                             $xxRMC
  uint8_t  day;            //                                             $xxRMC
  double   mag_var;        // magnetic variation deg                      $xxRMC
  char     var_dir;        // magnetic variation direction E/W            $xxRMC
  
  char quality;            // quality                                             $xxGGA
     // 0: Fix not valid
     // 1: Single point fix
     // 2: Differential GNSS fix (DGNSS), SBAS
     // 3: Not applicable
     // 4: RTK Fixed
     // 5: RTK Float
     // 6: INS Dead reckoning
  
  uint8_t       sats;       // number of satellites in view                       $xxGGA
  double        alt;        // antenna altitude                                   $xxGGA
  char          a_units;    // antenna units                                      $xxGGA
  double        undulation; // the relationship btwn geoid and WGS84 ellipsoid    $xxGGA
  char          u_units;    // undulation units                                   $xxGGA
  //unsigned char age;        // age of correction data                             $xxGGA   unused
  //unsigned int  stn_ID;     // differential base station ID                       $xxGGA   unused
  
  char          modeMA;     // M-Manual, A-Automatic                                      $xxGSA
  char          mode123;    // 1-not available, 2-2D fix, 3-3D fix                        $xxGSA
  
  struct PRN_sats prn_sats[PRN_SATS_MAX]; //                                              $xxGSA
  
  double pdop;          // Position dilution of precision                                 $xxGSA
  double hdop;          // Horizontal dilution of precision                       $xxGGA  $xxGSA
  double vdop;          // Vertical dilution of precision                                 $xxGSA
  
  double speed;              // speed over ground km/h                                            $xxVTG
  char   speed_ind;          // speed indicator (KM/H)                                            $xxVTG
  double nautical_speed;     // speed over ground knots                   $xxRMC,                 $xxVTG
  char   nautical_speed_ind; // nautical speed indicator                                          $xxVTG
  double true_track;         // Track made good (degrees true)            $xxRMC,                 $xxVTG
  char   true_track_ind;     // Track made good is relative to true north                         $xxVTG
  double magnetic_track;     // Track made good (degrees magnetic)                                $xxVTG
  char   magnetic_track_ind; // Magnetic track indicator                                          $xxVTG
  char   mode_ind;           // Positioning system mode indicator         $xxRMC,                 $xxVTG,         $xxGLL
                             // (A - Autonomous, D - Differential,
                             // E - Estimated (dead reckoning) mode,
                             // M - Manual input, N - Data not valid

        // Expected error in lat/lon/alt, in meters, due to bias, with noise = 0
  double lat_err;           // Expected error in latitude                                                 $xxGBS
  double lon_err;           // Expected error in longitude                                                $xxGBS
  double alt_err;           // Expected error in altitude                                                 $xxGBS
 
  double RMS;               // RMS value of the standard deviation of the ranges.                         $xxGST
                            // Includes carrier phase residuals during periods
                            // of RTK (float) and RTK (fixed) processing.
  
  double lat_std_dev;       // Standard deviation of latitude error                                       $xxGST
  double lon_std_dev;       // Standard deviation of longitude error                                      $xxGST
  double alt_std_dev;       // Standard deviation of altitude error                                       $xxGST
  
  uint8_t msgs_rcvd [MSG_MAX]; // the number of messages received, grouped by type: GSV, GSA, RMC, GGA, VTG, GLL, GBS, GST
  // To be sure the data (two angles:longitude/latitude of position) is calculated correctly, check "pos_status", "quality" and "mode123"
};


/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 *************************************************** the structures for alternate satellites information *******************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/


// some modules (e.g. u-blox MAX-M10S or u-blox ZED-F9P) group GSV messages by Signal ID (it corresponds to the frequency bands), so we need four groups for GPS, four groups for Galileo, etc.
#define MAXGSVSYSTEMSTORAGE 16

struct GSV_Sat_data {
  uint16_t  prn;
  uint8_t   elev;
  uint16_t  azimuth;
  uint8_t   SNR;
};

struct GSV_message {          // single GSV message content
  uint8_t msgs;               // the number of messages for current system
  uint8_t msgNo;              // the message number for current system
  uint8_t sats;               // the number of satellites for current system
  struct GSV_Sat_data sat[4]; // satellites data from current message
  uint8_t signalID;           // available only for NMEA 4.11+
};

struct GSV_data_by_systemID {
  char    talker[3];          // talker ("GP", "GL", "GA", "BD", etc. - terminated with \0 byte)
  uint8_t msgs;               // messages received already for single talker
  struct  GSV_message GSV[9]; // the GSV message content
};

struct GSV_manager {
  struct GSV_data_by_systemID system[MAXGSVSYSTEMSTORAGE];
  uint8_t recSystems;   // system index of the last message received
};// GSVData;



class GNSSCollector {
private:
  // the main GNSS data storage:
  struct GNSS_data atDataStorage;
  struct GSV_manager *atGSVData;
  
  // the user callbacks:
  int8_t (*avl_callback)(void);
  int8_t (*read_callback)(void);
  int8_t (*atCustomParser)(const struct NMEA_fields *paSlices);
  
  // processing data storage:
  struct NMEA_parsers_table {
    const char *header; // NMEA header for parser function
    int8_t (GNSSCollector::*parser_method)(const struct NMEA_fields *);
  } const NMEA_p_t[9] = {
                          {"GSV", &GNSSCollector::GSV_parser},
                          {"GSA", &GNSSCollector::GSA_parser},
                          {"RMC", &GNSSCollector::RMC_parser},
                          {"GGA", &GNSSCollector::GGA_parser},
                          {"GLL", &GNSSCollector::GLL_parser},
                          {"VTG", &GNSSCollector::VTG_parser},
                          {"TXT", &GNSSCollector::TXT_parser},
                          {"GBS", &GNSSCollector::GBS_parser},
                          {"GST", &GNSSCollector::GST_parser}
                        };
  uint8_t atMessagesBreakLength;   // the time we wait to check if the message pack from single timestamp is complete or not
  
  // data processing methods:
  int8_t check_and_slice_NMEA_message(const char *pa_single_line, struct NMEA_fields *paSlices);
  int8_t parse_NMEA_fields_for_particular_message(const struct NMEA_fields *paSlices);
  
  // the particular parsers:
  int8_t RMC_parser(const struct NMEA_fields  *paSlices);
  int8_t GGA_parser(const struct NMEA_fields  *paSlices);
  int8_t VTG_parser(const struct NMEA_fields  *paSlices);
  int8_t GSA_parser(const struct NMEA_fields  *paSlices);
  int8_t GSV_parser(const struct NMEA_fields  *paSlices);
  int8_t GLL_parser(const struct NMEA_fields  *paSlices);
  int8_t GBS_parser(const struct NMEA_fields  *paSlices);
  int8_t GST_parser(const struct NMEA_fields  *paSlices);
  int8_t TXT_parser(const struct NMEA_fields  *paSlices) { SETCOLORCYAN DBG("                                   ... some info\r\n"); NOCOLOR; return(paSlices->cnt - paSlices->cnt);};
  
  // extra tools:
  static inline void printTalkerName (const char *paTalker, bool paAlign);
  static inline uint8_t getSystemIDByTalker(const char *paTalker);

public:
  // constructor/destructor:
  // the constructor needs the two callbacks used to obtain data from the data source
  // Thanks to this solution, the library can work with devices connected via any port (e.g. UART, i2c, USB, etc.)
  // the first callback returns the number of bytes waiting in the input buffer or the negative value if timeout occured
  // the timeout mechanism you have to implement with your own using the available_check_callback function
  // it may be implemented in different way on linux and arduino
  // the second callback returns one byte from the input buffer
  GNSSCollector(int8_t (*available_check_callback)(void), int8_t (*read_check_callback)(void));
  ~GNSSCollector(void);
  
  // the data processing methods:
  
  // the method converts the angle (latitude or longitude) given with some NMEA message in sophisticated string format
  // to the double type degrees value
  
/* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
 * WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
 * WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
 * 
 * this method works only for the architectures with 8 bytes size of double
 * the calculations on 4 bytes size of double (e.g. Arduino UNO) will have no full precision
 * check the double type size on your platform to have full precision especially for RTK applications
 * 
 * WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
 * WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
 * WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING */
  static inline double convertAngle(const char *paNMEA_AngleFormat);
  
  
  // make sure you give the correct NMEA time format to this function
  static inline void parseTime(const char *paSlice, uint8_t &paHour, uint8_t &paMinutes, uint8_t &paSeconds, uint16_t &paFraction);
  
  // the data processing customization methods:
  // this method sets the time to estimate if the NMEA message belongs to the same time stamp messages pack or different time stamp messages pack
  // use them very carefully, the time shall be as short as possible to receive all NMEA messages correctly
  // the given value is the equivalent of milliseconds
  //
  // if the value is too small, it may result in collecting only fragments of packages with the same timestamp using the collectData() function
  // if the value is too big, some data may be lost
  // the default values have been selected experimentally and should be satisfactory for most applications,
  // but don't hesitate to experiment on your own
  void setBreakTime(uint8_t);

  // default the GSV data is not collected because it needs huge RAM space. It is impossible to parse GSV data
  // using Arduino UNO with 2KB of RAM
  // GSV data is very sophisticated and it is irrelevant in the vast majority of applications,
  // but you can collect them in linux applications or other Arduino boards equipped with large RAM space
  int8_t GSVSwitch(bool turnOn);

  // Using this method you can write your own parser. Your parser can control if the message received shall be processed by library parsers.
  // When your own parser returns the non zero value, the library parsers will be cancelled for the actual processed single NMEA message.
  // You can set NULL pointer with this method at any time to cancel your custom parser
  // You can also change the pointer to any item of your collection of parsers at any time
  // Your parser will receive sliced NMEA message ready to analyse any fields from the whole NMEA message
  // You can use them to filter library operations to the messages you are interested in and save CPU time
  void setCustomParser (int8_t (*paParser)(const struct NMEA_fields *paSlices)) {this->atCustomParser = paParser;};
  
  // the data access methods:
  inline const struct GNSS_data   *getGNSSData(void) {return ( &this->atDataStorage); }
  inline const struct GSV_manager *getGSVData(void) {return ( this->atGSVData); }
  inline uint8_t                   getBreakTime(void) { return this->atMessagesBreakLength; };
  
  // !!!! WARNING !!!, the paIndex value shall be less than paSlices->cnt value and less than MAXFIELDSINMESSAGE
  // !!! that is not checked due to CPU time saving by inline function !!!
  // !!! WARNING !!! paSlices can not be NULL - that is not checked due to CPU time saving by inline function !!!
  static inline const char        *get_field(const struct NMEA_fields *paSlices, unsigned char paIndex) { return (paSlices->message + paSlices->field_index[paIndex]); }
  
  // the main method:
  // this method refreshes the GNSS data structures, so it is necessary to call them every time you want to have actual GNSS data
  int8_t collectData(bool paShowReceivedMessage, bool paShowCRNLVisible);
  
  /****************************************************************************************************
   ****************************************************************************************************
   ****************************************************************************************************
   ****************************************************************************************************
   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   debug purpose methods: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   ****************************************************************************************************
   ****************************************************************************************************
   ****************************************************************************************************
   ****************************************************************************************************/
  
  // this function prints the slices from single message 
  static inline void printFieldsStorage(const struct NMEA_fields *paSlices) { // for debug purpose only
    int8_t i;
    
    if (NULL == paSlices) {
      DBG ("The \"printFieldsStorage method\" can not print data stored inside NULL pointed area !!!\r\n");
      return;
    }
    DBG("============================= FIELDS BEGIN =========================================================\r\n")
    
    for (i=0; i<paSlices->cnt; i++) {
      DBG("field ("); DBGT(i,DEC); DBG("): [at ");DBGT(paSlices->field_index[i], DEC);
      DBG("] ("); DBGV(GNSSCollector::get_field(paSlices,i)); DBG(")\r\n");
    }
    DBG("============================= FIELDS END =========================================================\r\n")
    
    return;
  }
  
  // this function prints the whole collected data from all parsed messages except of the GSV messages
  void printGNSSData(bool);
  
  // this function prints the satellites in view informations like SAT PRN, elevation, azimuth, SNR
  // To collect GSV data, you have to turn on the parser for this messages disabled by default
  // the GSVSwitch(bool turnOn) function is intended for this purpose
  void printGSVData(bool);
};

// this function can be used to prepare NMEA format message to be send to the receiver.
// there must be the buffer space allocated for the message and filled with the message data
// This function adds at the end of message these 5 bytes:
// 1) the "*" character
// 2) the two bytes of NMEA message checksum calculated from the given message
// 3) the "\r\n" characters at the end
// then you can put the message to the receiver to change some settings on them
int8_t completeTheNMEAMessage(char *paMessage);

  const struct talkerID_Names {
    const char talker[3];
    const char name[8];
    const uint8_t systemID;
  } talkerNames [] = { {"??","    ???", 0}, /* some unrecognized message (e.g. Teseo-LIV3F has the $PSTMCPU message)*/
                       {"GP","    GPS", 1}, /* USA - Global Positioning System */
                       {"GL","GLONASS", 2}, /* Ru - Глобальная навигационная спутниковая система */
                       {"GA","Galileo", 3}, /* EU system - more precise than GLONASS or GPS */
                       {"BD"," BeiDou", 4}, /* BeiDou (BDS); Chinese: 北斗卫星导航系统; pinyin: Běidǒu Wèixīng Dǎoháng Xìtǒng */
                       {"GB"," BeiDou", 4}, /* BeiDou (BDS); Chinese: 北斗卫星导航系统; pinyin: Běidǒu Wèixīng Dǎoháng Xìtǒng */
                       {"QZ","   QZSS", 5}, /* QZSS (Quasi-Zenith Satellite System), Japan */ /* QZSS regional GPS augmentation system (Japan) */
                       {"GQ","   QZSS", 5}, /* QZSS (Quasi-Zenith Satellite System), Japan */
                       {"GI","  NavIC", 6}, /* Indian Regional Navigation Satellite System (IRNSS) */
                       {"PQ","QZSS-QQ", 5}, /* QZSS (Quasi-Zenith Satellite System), Japan */  /* QZSS (Quectel Quirk) */
                       {"GN","   GNSS", 0} }; /* Multi constellation - has to be at the end of the table due to presentation layer */

/* there are 6 constellations: GPS, GLONASS, Galileo, BeiDou, QZSS, NavIC */
#define MAX_SYSTEM_ID 6

extern const char *GNSSsignalIDNames[MAX_SYSTEM_ID + 1][16];

#endif
