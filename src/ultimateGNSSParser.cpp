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

#ifndef GNSS_PARSER_CPP
#define GNSS_PARSER_CPP

#include "ultimateGNSSParser.h"



/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ************************************************ this method calculates the angle from the NMEA format DD(D)MM.MMM to DD.DDDDD ********************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

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

inline double GNSSCollector::convertAngle(const char *paNMEA_AngleFormat) {
  // the IMPORTANT assumption is that the string contains only numbers and dot sign !!!!!! - it is not checked not to waste the CPU time
  // The easiest way to calculate the degrees is given below, but it is not mathematically correct calculation and has lower accuracy
  //double loNMEA_Angle=atof(paNMEA_AngleFormat);
  //return ((loNMEA_Angle - (double)(((uint32_t)(loNMEA_Angle/100))*100))/60.0 + (uint32_t)(loNMEA_Angle/100));
  
  if (NULL == paNMEA_AngleFormat) // no parameter is given
    return((double)0.0);
  
  if (0 == *paNMEA_AngleFormat) // empty string is given
    return((double)0.0);
  
  // it is not checked but there shall be two digits or more before the dot character
  const char *loPtr = strchr(paNMEA_AngleFormat, '.')-2; // two digits for the minutes before fraction
                                                         // we have to count left from the dot, because
                                                         // the degrees can be given with 2 or 3 digits
  if (NULL != loPtr) {
    double loMinutes = atof(loPtr);
    uint16_t loDegrees = 0;
    uint8_t i, loDegDigits = loPtr-paNMEA_AngleFormat; // the number of the full degrees digits (2 or 3)
    for (i=0; i<loDegDigits; i++) {
      loDegrees *=10;
      loDegrees += *(paNMEA_AngleFormat+i)-0x30; // 0x30 -> look at the ASCII table
    }
    loDegrees*=60; // now the number of degrees shows the amount of minutes
    loMinutes+= (double)loDegrees; // and now we have full angle in minutes
    loMinutes/=(double)60.0; // now we have the number of degrees with the last single operation of division (the most accurate value)
    return (loMinutes);
  } else { // this should not happen - the angle is given with degrees and minutes without fraction !!!
    return ((double)(atoi(paNMEA_AngleFormat)));
  }
}


/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ********************* this method wraps the NMEA time format and saves the values of hour, min, sec to the given params  **************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

inline void GNSSCollector::parseTime(const char *paSlice, uint8_t &paHour, uint8_t &paMinutes, uint8_t &paSeconds, uint16_t &paFraction) {
  const char *loFractPtr = strchr(paSlice, '.') + 1;
  
  if (NULL == paSlice) {
    DBG("parseTime: the given slice is NULL\r\n");
    return;
  }
  double loSlice = atof(paSlice);
  paHour     = ((uint8_t) (   ( ( (uint32_t)loSlice )  /10000)      ));
  paMinutes  = ((uint8_t) (   ( ( (uint32_t)loSlice )  /100)    %100));
  paSeconds  = ((uint8_t) (     ( (uint32_t)loSlice )           %100));
  if (0 == *paSlice) { // the empty string
    paFraction = 0;
  } else {
    // be sure you give the correct NMEA time format to this function
    /* sometimes the fraction of seconds is given with only two digits (e.g. UM980)
     * so we make sure that the number is in milliseconds: */
    paFraction = atoi(loFractPtr);
    if (3 > strlen (loFractPtr)) paFraction *= 10;
    if (2 > strlen (loFractPtr)) paFraction *= 10;
    // TODO - check if the fraction is more accurate than 1ms in the string
  }
  // paFraction = (uint16_t) ((paSlice - ((double)((uint32_t)paSlice)))*1000); // this doesn't work due to rounding the last digit
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 *************************************************************** the class constructor  ************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

GNSSCollector::GNSSCollector(int8_t (*available_check_callback)(void), int8_t (*read_check_callback)(void)) {
  memset((void*)&(this->atDataStorage), 0, sizeof(this->atDataStorage));
  avl_callback = available_check_callback;
  read_callback = read_check_callback;
  atGSVData = NULL;
  atCustomParser = NULL;
#ifdef ARDUINO
  atMessagesBreakLength = 2;
#elif __linux__
  atMessagesBreakLength =20;
#endif
  return;
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 *************************************************************** the class destructor  *************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

GNSSCollector::~GNSSCollector(void) {
  if (this->atGSVData) {
    delete this->atGSVData;
    this->atGSVData = NULL;
  }
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 **************** the method sets the time break period for estimating the bunch of messages emited in single timestamp  ***************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

void GNSSCollector::setBreakTime(uint8_t paNewTime) {
  if (0 == paNewTime) {
    DBG("The delay time shall be greater than 0 ms\r\n");
    return;
  }
  
  if (200 < paNewTime) {
    DBG("The delay time shall be less than 200 ms\r\n");
    return;
  }
  
  this->atMessagesBreakLength = paNewTime;
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ************************************ this method turns ON(true)/OFF(false) the GSV parser and GSV data structures *********************************
 **************** The GSV parser and data structures are OFF by default due to save RAM space (e.g. Arduino UNO) and CPU time. *********************
 ******************************* The GSV data is very sophisticated and unrelevant in most GNSS projects. ******************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

int8_t GNSSCollector::GSVSwitch(bool turnOn) {
  if (turnOn) { // turn ON
    if (NULL == this->atGSVData) {
      this->atGSVData = new struct GSV_manager;
      memset((void*)(this->atGSVData), 0, sizeof(*(this->atGSVData)));
      if (NULL == this->atGSVData) {
        SETCOLORRED DBG("Insufficient RAM space for GSV collector\r\n"); NOCOLOR
        return (-1);
      }
    }
  } else { // turn OFF
    if (this->atGSVData) {
      delete this->atGSVData;
      this->atGSVData = NULL;
    }
  }
  return (0);
}


/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ************************************************ this is the parsers manager  *********************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

int8_t GNSSCollector::parse_NMEA_fields_for_particular_message(const struct NMEA_fields *paSlices) {

  uint8_t i;
  uint8_t loConstTableSize = sizeof(this->NMEA_p_t)/sizeof(this->NMEA_p_t[0]);
  
  for (i=0; i< loConstTableSize;i+=1) {
    if (!strncmp(GNSSCollector::get_field(paSlices,0)+3, this->NMEA_p_t[i].header,3)) {
      return ((this->*NMEA_p_t[i].parser_method)(paSlices));
    }
  }
  SETCOLORRED
  DBG("                                   Unrecognized NMEA message - no particular parser\r\n");
  NOCOLOR
  return(0);
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 *************************************************** talker ID names to be printed in debug mode  **************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/


inline void GNSSCollector::printTalkerName (const char *paTalker, bool paAlign) {
  
  const struct talkerID_Names {
    const char talker[3];
    const char name[8];
  } talkerNames [] = { {"??","    ???"}, /* some unrecognized message (e.g. Teseo-LIV3F has the $PSTMCPU message)*/
                       {"GP","    GPS"}, /* USA - Global Positioning System */
                       {"GL","GLONASS"}, /* Ru - Глобальная навигационная спутниковая система */
                       {"GA","Galileo"}, /* EU system - more precise than GLONASS or GPS */
                       {"BD"," BeiDou"}, /* BeiDou (BDS); Chinese: 北斗卫星导航系统; pinyin: Běidǒu Wèixīng Dǎoháng Xìtǒng */
                       {"GB"," BeiDou"}, /* BeiDou (BDS); Chinese: 北斗卫星导航系统; pinyin: Běidǒu Wèixīng Dǎoháng Xìtǒng */
                       {"QZ","   QZSS"}, /* QZSS (Quasi-Zenith Satellite System), Japan */ /* QZSS regional GPS augmentation system (Japan) */
                       {"GQ","   QZSS"}, /* QZSS (Quasi-Zenith Satellite System), Japan */
                       {"GI","  NavIC"}, /* Indian Regional Navigation Satellite System (IRNSS) */
                       {"PQ","QZSS-QQ"}, /* QZSS (Quasi-Zenith Satellite System), Japan */  /* QZSS (Quectel Quirk) */
                       {"GN","   GNSS"} }; /* Multi constellation - has to be at the end of the table due to presentation layer */
  
  uint8_t loTindex;
  const uint8_t loTalkersNumber = sizeof(talkerNames)/sizeof(talkerNames[0]);
  const char *loTalkerFound;
  
  for (loTindex = 0; loTindex < loTalkersNumber; loTindex++ ) {
    if (! strncmp(talkerNames[loTindex].talker, paTalker, 2)) {
      loTalkerFound = talkerNames[loTindex].name;
      break;
    }
  }
  
  if (loTalkersNumber == loTindex) {
    loTalkerFound = talkerNames[0].name;
  }
  
  if (!paAlign) {
    while (' ' == *loTalkerFound)
      loTalkerFound++;
  }
  
  DBGV(loTalkerFound);
}



inline uint8_t GNSSCollector::getSystemIDByTalker(const char *paTalker) {
  
  uint8_t loSindex;
  const uint8_t loTalkersNumber = sizeof(talkerNames)/sizeof(talkerNames[0]);
  uint8_t loSystemID = 0;
  
  for (loSindex = 0; loSindex < loTalkersNumber; loSindex++ ) {
    if (! strncmp(talkerNames[loSindex].talker, paTalker, 2)) {
      loSystemID = talkerNames[loSindex].systemID;
      break;
    }
  }
  
  if (loTalkersNumber == loSindex) {
    loSystemID = talkerNames[0].systemID;
  }
  
  return (loSystemID);
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 *********************************** The method prints the collected data from the GSV messages in extended mode ***********************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

/*    0             1            2            3            4            5            6           7            8            9            A            B             C            D           E            F    */
const char *GNSSsignalIDNames[MAX_SYSTEM_ID + 1][16] = {
  {"Undefined",   "Undefined","Undefined", "Undefined", "Undefined",  "Undefined","Undefined", "Undefined", "Undefined", "Undefined", "Undefined", "Undefined", "Undefined", "Undefined", "Undefined", "Undefined"},  // Undefined
  {"All signals", "L1 C/A",   "L1 P(Y)"  ,   "L1 M",     "L2 P(Y)",    "L2C-M",    "L2C-L",      "L5-I",      "L5-Q",    "Reserved",  "Reserved",  "Reserved",   "Reserved", "Reserved",  "Reserved",  "Reserved"},   // GPS
  {"All signals", "L1 C/A",   "L1 P",       "L2 C/A",     "L2 P",     "Reserved", "Reserved",  "Reserved",  "Reserved",  "Reserved",  "Reserved",  "Reserved",   "Reserved", "Reserved",  "Reserved",  "Reserved"},   // GLONASS
  {"All signals", "E5a",      "E5b",        "E5a+b",      "E6-A",      "E6-BC",     "L1-A",      "L1-BC",   "Reserved",  "Reserved",  "Reserved",  "Reserved",   "Reserved", "Reserved",  "Reserved",  "Reserved"},   // Galileo
  {"All signals", "B1I",      "B1Q",         "B1C",       "B1A",       "B2-a",      "B2-b",     "B2 a+b",      "B3I",       "B3Q",       "B3A",       "B2I",       "B2Q"  ,  "Reserved",  "Reserved",  "Reserved"},   // BeiDou
  {"All signals", "L1 C/A",   "L1C (D)",   "L1C (P)",     "LIS",      "L2C-M",     "L2C-L",      "L5-I",      "L5-Q",       "L6D",       "L6E",    "Reserved",   "Reserved", "Reserved",  "Reserved",  "Reserved"},   // QZSS
  {"All signals", "L5-SPS",   "S-SPS",      "L5-RS",      "S-RS",     "L1-SPS",   "Reserved",  "Reserved",  "Reserved",  "Reserved",  "Reserved",  "Reserved",   "Reserved", "Reserved",  "Reserved",  "Reserved"}    // NavIC
};


void GNSSCollector::printGSVData(bool paShowDebugInfo = false) {
  uint8_t i,m,s; /* iterators: generic, message, system */
  uint8_t loSystemID;
  
  if (NULL == this->atGSVData) {
    if (paShowDebugInfo) {
      SETCOLORRED DBG("THE GSV data is not collected !!!\r\n"); NOCOLOR
    }
    return;
  }
  
  DBG("\r\n\r\nThe GSV data: "); SETCOLORBLUE DBG("GNSS Satellites in View"); NOCOLOR DBG(" - the number of GNSS systems is "); DBGT(atGSVData->recSystems, DEC); DBG("\r\n");
  for (s=0; s<MAXGSVSYSTEMSTORAGE; s++) {
    if (0 == *(atGSVData->system[s].talker))
      continue;
    DBG("************************************************************\r\n");
    DBG("System No "); DBGT(s+1, DEC); DBG("\t");
    DBG ("talker is \""); DBGV(atGSVData->system[s].talker); DBG("\" ("); SETCOLORCYAN GNSSCollector::printTalkerName(atGSVData->system[s].talker,false); NOCOLOR DBG(")   ");
    DBG ("received messages: "); DBGT(atGSVData->system[s].msgs,DEC); DBG("\r\n");
    
    loSystemID = getSystemIDByTalker(atGSVData->system[s].talker);
    
    for (m=0; m<9;m++) {
      if (0 == atGSVData->system[s].GSV[m].msgNo)
        continue;
      DBG("--------------------------------------------------\r\n");
      DBG("Msg: ");   DBGT(atGSVData->system[s].GSV[m].msgNo,DEC); DBG("/"); DBGT(atGSVData->system[s].GSV[m].msgs,DEC);  DBG("\t");
      DBG("sats in view: ");  SETCOLORBLUE DBGT(atGSVData->system[s].GSV[m].sats,DEC); NOCOLOR DBG("\r\n");
      SETCOLORBLUE DBG ("                        PRN   ");   DBG ("elev   ");   DBG ("azimuth   "); DBG ("SNR"); NOCOLOR DBG("\r\n");
      for (i=0;i<4;i++) {
        if (0 == atGSVData->system[s].GSV[m].sat[i].prn)
          continue;
        DBG ("\tSat No.: "); DBGT(i, DEC); DBG("      ");
        if ( 0 == atGSVData->system[s].GSV[m].sat[i].SNR) SETCOLORRED else SETCOLORGREEN
        if (100 > atGSVData->system[s].GSV[m].sat[i].prn)     DBG(" "); if (10 > atGSVData->system[s].GSV[m].sat[i].prn)     DBG(" "); DBGT(atGSVData->system[s].GSV[m].sat[i].prn,    DEC); DBG("    ");
        if ( 10 > atGSVData->system[s].GSV[m].sat[i].elev)    DBG(" ");                                                                DBGT(atGSVData->system[s].GSV[m].sat[i].elev,   DEC); DBG("     ");
        if (100 > atGSVData->system[s].GSV[m].sat[i].azimuth) DBG(" "); if (10 > atGSVData->system[s].GSV[m].sat[i].azimuth) DBG(" "); DBGT(atGSVData->system[s].GSV[m].sat[i].azimuth,DEC); DBG("      ");
        if ( 10 > atGSVData->system[s].GSV[m].sat[i].SNR)     DBG(" ");                                                                DBGT(atGSVData->system[s].GSV[m].sat[i].SNR,    DEC); DBG("\r\n");
        NOCOLOR
      }
      
      DBG("SignalID "); if (255 != atGSVData->system[s].GSV[m].signalID) { SETCOLORGREEN 
                                                                           DBGT(atGSVData->system[s].GSV[m].signalID,HEX); DBG("\t"); 
                                                                           DBG(GNSSsignalIDNames[loSystemID][atGSVData->system[s].GSV[m].signalID]); 
                                                                           NOCOLOR
                                                                         } else {SETCOLORRED DBG(" not present"); NOCOLOR}

      DBG("\r\n");
    }
    DBG("--------------------------------------------------\r\n");
  }
  DBG("************************************************************\r\n");
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ************************************* The method prints the collected data from the struct GNSS_data attribute ************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

void GNSSCollector::printGNSSData(bool paShowDebugInfo = false) {
  double loField;
  const struct SystemIDByName {
    const int8_t systemID;
    const char name[8];
  } sysIdName [] ={{0, "QZSS"},{1, "GPS"},{2, "GLONASS"},{3, "Galileo"},{4, "BeiDou"},{5,"QZSS"},{6,"NavIC"/*IRNSS*/},{7, "Empty"}}; // the "for" loop needs the last empty name
  
  DBG("Time:\t"); DBGT(this->atDataStorage.year, DEC);  DBG("-"); DBGT(this->atDataStorage.month, DEC); DBG("-"); DBGT(this->atDataStorage.day, DEC);
  DBG("        "); DBGT(this->atDataStorage.UTC_H, DEC); DBG(":"); DBGT(this->atDataStorage.UTC_M,DEC);  DBG(":"); DBGT(this->atDataStorage.UTC_S,DEC);
  DBG("."); if (100 > this->atDataStorage.UTC_fract) DBG("0"); if (10 > this->atDataStorage.UTC_fract) DBG("0"); DBGT(this->atDataStorage.UTC_fract,DEC); DBG("\r\n");
  
  DBG("Position: ");
  printPreciselyDouble(this->atDataStorage.lat);
  DBGC(this->atDataStorage.lat_dir); DBG(",");
  printPreciselyDouble(this->atDataStorage.lon);
  DBGC(this->atDataStorage.lon_dir); DBG("\r\n");
  DBG("Antenna altitude: "); printDouble(this->atDataStorage.alt); DBG(" [");DBGC(this->atDataStorage.a_units); DBG("]\r\n");
  
  DBG("\r\nPosition status: "); DBGC(this->atDataStorage.pos_status);
  switch (this->atDataStorage.pos_status) {
    case 'A':
            SETCOLORGREEN DBG(" - Valid position (2D or 3D fix)"); NOCOLOR
            break;
    case 'V':
            SETCOLORPURPLE DBG(" - Warning: Invalid position"); NOCOLOR
            break;
    default:
            SETCOLORRED DBG("    Unknown status !!!"); NOCOLOR
            break;
  }
  DBG("\r\n");
  
  DBG("Mode Indicator:  "); DBGC(this->atDataStorage.mode_ind);
  switch (this->atDataStorage.mode_ind) {
    case 'A':
      DBG(" - Autonomous");
      break;
    case 'C':
      SETCOLORPURPLE DBG(" - Quectel Querk, \"Caution\""); NOCOLOR
      break;
    case 'D':
      DBG(" - Differential");
      break;
    case 'E':
      DBG(" - Estimated (dead reckoning)");
      break;
    case 'F':
      DBG(" - RTK Float");
      break;
    case 'M':
      DBG(" - Manual input");
      break;
    case 'N':
      SETCOLORPURPLE DBG(" - Data not valid"); NOCOLOR
      break;
    case 'P':
      DBG(" - Precise (4.00 and later)");
      break;
    case 'R':
      DBG(" - RTK Integer");
      break;
    case 'S':
      DBG(" - Simulated");
      break;
    case 'U':
      SETCOLORPURPLE DBG(" - Quectel Querk, \"Unsafe\""); NOCOLOR
      break;
    default:
      SETCOLORRED DBG("    Unknown mode !!!"); NOCOLOR
  }
  DBG("\r\n");
  
  DBG("Quality:         "); DBGC(this->atDataStorage.quality);
  switch (this->atDataStorage.quality) {
     case '0': SETCOLORPURPLE DBG(" - Fix not valid"); NOCOLOR break;
     case '1': DBG(" - Single point fix"); break;
     case '2': DBG(" - Differential fix"); break;
     case '3': SETCOLORRED DBG(" - Not applicable"); NOCOLOR break;
     case '4': DBG(" - RTK Fixed, ambiguity solution"); break;
     case '5': DBG(" - RTK Floating, ambiguity solution"); break;
     case '6': DBG(" - Dead reckoning"); break;
     case '7': SETCOLORYELLOW DBG(" - Manual input mode (fixed position)"); NOCOLOR break;
     case '8': SETCOLORYELLOW DBG(" - Simulator mode"); NOCOLOR break;
     case '9': DBG(" - WAAS (SBAS)"); break;
     default: SETCOLORRED DBG("    Unknown quality");NOCOLOR
  }
  DBG("\r\n");
  
  DBG("mode 123:        "); DBGC(this->atDataStorage.mode123);
  switch ((int)this->atDataStorage.mode123) {
    case '1': SETCOLORPURPLE DBG(" - Fix not available"); NOCOLOR
      break;
    case '2': SETCOLORYELLOW DBG(" - 2D Fix"); NOCOLOR
      break;
    case '3': SETCOLORGREEN  DBG(" - 3D Fix"); NOCOLOR
      break;
    default:  SETCOLORRED    DBG("    Unknown mode"); NOCOLOR
  }
  DBG("\r\n");
  
  DBG("mode MA:         "); if (0 != this->atDataStorage.modeMA) DBGC(this->atDataStorage.modeMA);
  switch (this->atDataStorage.modeMA) {
    case 'M': DBG(" - Manual, forced to operate in 2D or 3D"); break;
    case 'A': DBG (" - Automatic 2D/3D"); break;
    default: SETCOLORRED DBG("    Unknown mode"); NOCOLOR
  }
  DBG("\r\n");
  
  DBG("RMS value of the pseudorange residuals: ");  printDouble(this->atDataStorage.RMS); DBG(" [m]\r\n");
  loField = this->atDataStorage.lat_err;
  DBG("Expected latitude  error / Latitude  standard deviation: "); if (((double)0.0) == loField) {DBG("----- / ");} else { printDouble(loField); DBG(" [m] / ");}
  loField = this->atDataStorage.lat_std_dev;
  if (((double)0.0) == loField) {DBG("-----\r\n");} else { printDouble(loField); DBG(" [m]\r\n");}

  loField = this->atDataStorage.lon_err;
  DBG("Expected longitude error / Longitude standard deviation: "); if (((double)0.0) == loField) {DBG("----- / ");} else { printDouble(loField); DBG(" [m] / ");}
  loField = this->atDataStorage.lon_std_dev;
  if (((double)0.0) == loField) {DBG("-----\r\n");} else { printDouble(loField); DBG(" [m]\r\n");}

  loField = this->atDataStorage.alt_err;
  DBG("Expected altitude  error / Altitude  standard deviation: "); if (((double)0.0) == loField) {DBG("----- / ");} else { printDouble(loField); DBG(" [m] / ");}
  loField = this->atDataStorage.alt_std_dev;
  if (((double)0.0) == loField) {DBG("-----\r\n");} else { printDouble(loField); DBG(" [m]\r\n");}
  
  DBG("(PDOP): "); printDouble(this->atDataStorage.pdop); DBG("\t");
  DBG("(HDOP): "); printDouble(this->atDataStorage.hdop); DBG("\t");
  DBG("(VDOP): "); printDouble(this->atDataStorage.vdop); DBG("\r\n");
  
  DBG("Nautical speed:      "); printDouble(this->atDataStorage.nautical_speed); DBG(" ["); DBGC(this->atDataStorage.nautical_speed_ind); DBG("] [knot] ( => "); printDouble((double)(this->atDataStorage.nautical_speed * (double)1.852)); DBG(" [km/h])\r\n");
  DBG("Speed over ground:   "); printDouble(this->atDataStorage.speed); DBG(" ["); DBGC(this->atDataStorage.speed_ind);  DBG("] [km/h]\r\n");

  DBG("True track:          "); printDouble(this->atDataStorage.true_track); DBG(" ["); DBGC(this->atDataStorage.true_track_ind); DBG("]\r\n");
  DBG("Magnetic track:      "); printDouble(this->atDataStorage.magnetic_track); DBG(" ["); DBGC(this->atDataStorage.magnetic_track_ind); DBG("]\r\n");
  DBG("Magnetic variation:  "); printDouble(this->atDataStorage.mag_var); DBG(" ["); DBGC(this->atDataStorage.var_dir); DBG("]\r\n");
  DBG("Undulation:          "); printDouble(this->atDataStorage.undulation); DBG(" ["); DBGC(this->atDataStorage.u_units); DBG("]\r\n");
  DBG("Satellites in view: "); DBGT(this->atDataStorage.sats, DEC); DBG("\r\n");
  DBG("Satellites used in solution:\r\n");
  
  for (int j=0; (0 != this->atDataStorage.prn_sats[j].prn_gps[0]) && (PRN_SATS_MAX>j); j+=1) {
    DBG("\t\t\t\t(");
    for (int i=0; 12>i; i+=1) {
      if (10 > this->atDataStorage.prn_sats[j].prn_gps[i])
        DBG(" ");
      if (0 != this->atDataStorage.prn_sats[j].prn_gps[i]) {
        SETCOLORGREEN
      } else {
        SETCOLORRED
      }
      DBGT(this->atDataStorage.prn_sats[j].prn_gps[i],DEC);
      NOCOLOR
      if (11>i) {
        DBG(", ");
      }
    }
    DBG(")");
    if (-1 != this->atDataStorage.prn_sats[j].systemID) {
      SETCOLORGREEN DBG("\tSystem ID = ");
      DBGT(this->atDataStorage.prn_sats[j].systemID,DEC);
      for ( uint8_t it=0; it<(sizeof(sysIdName) /sizeof(sysIdName[0])); it++) {
        if (sysIdName[it].systemID == this->atDataStorage.prn_sats[j].systemID) {
          DBG("\t");
          DBGV(sysIdName[it].name);
          break;
        }
      }
      NOCOLOR
    } else if (strncmp("GN", this->atDataStorage.prn_sats[j].talker, 2)) {
      SETCOLORGREEN
      DBG("\tSystem ("); GNSSCollector::printTalkerName(this->atDataStorage.prn_sats[j].talker, false); DBG(")");
      NOCOLOR
    } else { // system ID is not present in $xxGSA message, so we try to read it from GSV messages if present
      if (NULL != this->atGSVData) { // the GSV data could be present
        if (j < this->atGSVData->recSystems) {
          SETCOLORYELLOW DBG("\tprobably ("); GNSSCollector::printTalkerName(this->atGSVData->system[j].talker, false); DBG(")");  NOCOLOR
        }
      }
    }
    DBG("\r\n");
  }
  
  DBG("The numbers of received messages by type: \r\n");
  
  DBG(  "\tGSV: "); DBGT(this->atDataStorage.msgs_rcvd[MSG_GSV],DEC);
  DBG("\t\tGSA: "); DBGT(this->atDataStorage.msgs_rcvd[MSG_GSA],DEC);
  DBG("\t\tRMC: "); DBGT(this->atDataStorage.msgs_rcvd[MSG_RMC],DEC);
  DBG("\t\tGGA: "); DBGT(this->atDataStorage.msgs_rcvd[MSG_GGA],DEC);
  DBG("\t\tVTG: "); DBGT(this->atDataStorage.msgs_rcvd[MSG_VTG],DEC);
  DBG("\t\tGLL: "); DBGT(this->atDataStorage.msgs_rcvd[MSG_GLL],DEC);
  DBG("\t\tGBS: "); DBGT(this->atDataStorage.msgs_rcvd[MSG_GBS],DEC);
  DBG("\t\tGST: "); DBGT(this->atDataStorage.msgs_rcvd[MSG_GST],DEC); DBG("\r\n");
}


/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ****** The parser for $GPGSV message. Takes data from the sliced message and stores them into the struct GNSS_data  *****
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/


#define GSV_CURR_SYS this->atGSVData->system[this->atGSVData->recSystems]

int8_t GNSSCollector::GSV_parser(const struct NMEA_fields  *paSlices) {
  uint8_t i;
  bool NMEA_ver411; // the version 4.11+ provides the signalID information
  
  if (strcmp("GSV", GNSSCollector::get_field(paSlices,0)+3)) {
    DBG("$__GSV message parser received different message: ");
    DBGV(GNSSCollector::get_field(paSlices,0)); DBG("\r\n");
    return (-1);
  }
  this->atDataStorage.msgs_rcvd[MSG_GSV]+=1;
  if (NULL == this->atGSVData) {
//    SETCOLORRED DBG("The GSV collector is turned off\r\n"); NOCOLOR
    return(0);
  }
  
  NMEA_ver411 = true;
  if (  ( 5 != paSlices->cnt)
     && ( 9 != paSlices->cnt)
     && (13 != paSlices->cnt)
     && (17 != paSlices->cnt)
     && (21 != paSlices->cnt)
     ) {
    NMEA_ver411 = false;
    if (  ( 4 != paSlices->cnt)
       && ( 8 != paSlices->cnt)
       && (12 != paSlices->cnt)
       && (16 != paSlices->cnt)
       && (20 != paSlices->cnt)
       ) {
      DBG("$__GSV message shall consists 5,9,13,17 or 21 fields in NMEA version \"4.11+\", 4,8,12,16 or 20 in older version but there are recognized the ");
      DBGT(paSlices->cnt, DEC); DBG("\r\n");
      return (-1);
    }
  }
  
  if ((MAXGSVSYSTEMSTORAGE) == this->atGSVData->recSystems) {
    SETCOLORRED DBG("The data storage space for GSV information is full. Cannot save this one and subsequent messages\r\n"); NOCOLOR
    return(-2);
  }
  
  if (0 == GSV_CURR_SYS.msgs) { // this is the first message for the current system pack
    strncpy(GSV_CURR_SYS.talker, GNSSCollector::get_field(paSlices,0)+1, 2);
  } else { // subsequent message for the single system pack
    if (strncmp( GSV_CURR_SYS.talker, GNSSCollector::get_field(paSlices,0)+1, 2)) {
      // subsequent message for the single system pack doesn't match the talker name to the first one
      SETCOLORRED DBG("\tThe talker name of subsequent GSV message ("); SETCOLORCYAN GNSSCollector::printTalkerName(GNSSCollector::get_field(paSlices,0)+1, false); SETCOLORRED DBG(") doesn't match to the first message from the current system pack ("); SETCOLORCYAN GNSSCollector::printTalkerName(GSV_CURR_SYS.talker ,false); SETCOLORRED DBG(")\r\n"); NOCOLOR
      if ((MAXGSVSYSTEMSTORAGE) == ++this->atGSVData->recSystems) {
        SETCOLORRED DBG("The data storage space for GSV information is full. Can not save this and subsequent messages\r\n"); NOCOLOR
        return(-2);
      }
      strncpy(GSV_CURR_SYS.talker, GNSSCollector::get_field(paSlices,0)+1, 2);
    } else if ( !(atoi(GNSSCollector::get_field(paSlices,2)) > GSV_CURR_SYS.GSV[GSV_CURR_SYS.msgs-1].msgNo )) { // this is the message from the same talker but new pack (probably different SignalID - e.g. u-blox MAX-M10S)
      SETCOLORRED DBG("\tThe last message(s) from previous system pack was/were omitted. This is the message from the same talker ("); SETCOLORCYAN GNSSCollector::printTalkerName(GNSSCollector::get_field(paSlices,0)+1, false); SETCOLORRED DBG("), but new system pack - probably different SignalID\r\n"); NOCOLOR
      if ((MAXGSVSYSTEMSTORAGE) == ++this->atGSVData->recSystems) {
        SETCOLORRED DBG("The data storage space for GSV information is full. Can not save this and subsequent messages\r\n"); NOCOLOR
        return(-2);
      }
      strncpy(GSV_CURR_SYS.talker, GNSSCollector::get_field(paSlices,0)+1, 2);
    }
  }
  
  if (atoi(GNSSCollector::get_field(paSlices,2)) != (GSV_CURR_SYS.msgs+1)) {    // field 2 is the message number in the pack for single system pack
    SETCOLORRED DBG("\tAt least one GSV message was omitted from system pack nr "); DBGT(this->atGSVData->recSystems,DEC); DBG(" (");  SETCOLORCYAN GNSSCollector::printTalkerName(GSV_CURR_SYS.talker,false); SETCOLORRED DBG(")\r\n"); NOCOLOR
  }
  
  GSV_CURR_SYS.GSV[GSV_CURR_SYS.msgs].msgs  = atoi(GNSSCollector::get_field(paSlices, 1));
  GSV_CURR_SYS.GSV[GSV_CURR_SYS.msgs].msgNo = atoi(GNSSCollector::get_field(paSlices, 2));
  GSV_CURR_SYS.GSV[GSV_CURR_SYS.msgs].sats  = atoi(GNSSCollector::get_field(paSlices, 3));
  
  i=0;
  while ((paSlices->cnt) >= (4+(4*i)+4)) { /* sats starts at index of 4 + 4*i(one sats takes 4 fields) + 4 (4 fields for the current sat data) */
    //DBG("fieldsCnt="); DBGT(paSlices->cnt, DEC); DBG(" i+4="); DBGT(4+(4*i)+4, DEC); DBG("\r\n");
    GSV_CURR_SYS.GSV[GSV_CURR_SYS.msgs].sat[i].prn     = atoi(GNSSCollector::get_field(paSlices, 4+(4*i)));
    GSV_CURR_SYS.GSV[GSV_CURR_SYS.msgs].sat[i].elev    = atoi(GNSSCollector::get_field(paSlices, 5+(4*i)));
    GSV_CURR_SYS.GSV[GSV_CURR_SYS.msgs].sat[i].azimuth = atoi(GNSSCollector::get_field(paSlices, 6+(4*i)));
    GSV_CURR_SYS.GSV[GSV_CURR_SYS.msgs].sat[i].SNR     = atoi(GNSSCollector::get_field(paSlices, 7+(4*i)));
    i++;
  }
  
  if (NMEA_ver411) {
    GSV_CURR_SYS.GSV[GSV_CURR_SYS.msgs].signalID = (uint8_t) strtol(GNSSCollector::get_field(paSlices, 4+(4*i)), NULL, 16);
  } else {
    GSV_CURR_SYS.GSV[GSV_CURR_SYS.msgs].signalID = -1;
  }
  
  GSV_CURR_SYS.msgs++;
  
  if (atoi(GNSSCollector::get_field(paSlices,1)) == atoi(GNSSCollector::get_field(paSlices,2))) { // the last message from the current GNSS system pack
    this->atGSVData->recSystems++;
  }
  return(0);
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ****** The parser for $GPRMC message. Takes data from the sliced message and stores them into the struct GNSS_data  *****
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

int8_t GNSSCollector::RMC_parser(const struct NMEA_fields  *paSlices) {
  
  double loField;
  
  if (strcmp("RMC", GNSSCollector::get_field(paSlices,0)+3)) {
    DBG("$__RMC message parser received different message: ");
    DBGV(GNSSCollector::get_field(paSlices,0)); DBG("\r\n");
    return (-1);
  }
  
  if ((13 != paSlices->cnt) && (14 != paSlices->cnt)) {
    DBG("$__RMC message shall consists 13 or 14 fields, but there are recognized the ");
    DBGT(paSlices->cnt, DEC); DBG("\r\n");
    return (-1);
  }
  
  this->atDataStorage.msgs_rcvd[MSG_RMC]+=1;
  
  // UTC time status of position (hours/minutes/seconds/ decimal seconds)  hhmmss.fff
  GNSSCollector::parseTime(GNSSCollector::get_field(paSlices,1), this->atDataStorage.UTC_H, this->atDataStorage.UTC_M, this->atDataStorage.UTC_S, this->atDataStorage.UTC_fract);
  
  this->atDataStorage.pos_status = *GNSSCollector::get_field(paSlices,2); // Position status (A = data valid, V = data invalid) - Status A=active or V=void
  
  this->atDataStorage.lat = GNSSCollector::convertAngle(GNSSCollector::get_field(paSlices,3)); // Latitude (DDmm.mm)
  
  if (0 == *GNSSCollector::get_field(paSlices,4)) {
    this->atDataStorage.lat_dir = 'U'; // Undefined
  } else {
    this->atDataStorage.lat_dir = *GNSSCollector::get_field(paSlices,4); // Latitude direction (N = North, S = South)
  }
  
  this->atDataStorage.lon = GNSSCollector::convertAngle(GNSSCollector::get_field(paSlices,5)); // Longitude (DDDmm.mm)
  
  if (0 == *GNSSCollector::get_field(paSlices,6)) {
    this->atDataStorage.lon_dir = 'U'; // Undefined
  } else {
    this->atDataStorage.lon_dir = *GNSSCollector::get_field(paSlices,6); // Longitude direction (E = East, W = West)
  }
  
  this->atDataStorage.nautical_speed = atof(GNSSCollector::get_field(paSlices,7)); // Speed over ground, knots
  
  this->atDataStorage.true_track = atof(GNSSCollector::get_field(paSlices,8)); // Track made good, degrees True
  
  loField = atof(GNSSCollector::get_field(paSlices,9)); // Date: dd/mm/yy
  this->atDataStorage.day   = ((uint8_t)  (   ( ( (uint32_t)loField )  /10000)       ));
  this->atDataStorage.month = ((uint8_t)  ((  ( ( (uint32_t)loField )  /100)   ) %100));
  this->atDataStorage.year  = ((uint32_t) (     ( (uint32_t)loField )            %100));
  this->atDataStorage.year += 2000;
  
  this->atDataStorage.mag_var = atof(GNSSCollector::get_field(paSlices,10)); // Magnetic variation, degrees
  
  if (0 == *GNSSCollector::get_field(paSlices,11)) {
    this->atDataStorage.var_dir = 'U'; // Undefined
  } else {
    this->atDataStorage.var_dir = *GNSSCollector::get_field(paSlices,11); // Magnetic variation direction E/W
  }
  
  this->atDataStorage.mode_ind = *GNSSCollector::get_field(paSlices,12); // Positioning system mode indicator (A - Autonomous, D - Differential, E - Estimated (dead reckoning) mode, M - Manual input, N - Data not valid)
  
  return 0;
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ****** The parser for $GGA message. Takes data from the sliced message and stores them into the struct GNSS_data  *****
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

int8_t GNSSCollector::GGA_parser(const struct NMEA_fields *paSlices) {
  
  if (strcmp("GGA", GNSSCollector::get_field(paSlices,0)+3)) {
    DBG("$__GGA message parser received different message: ");
    DBGV(GNSSCollector::get_field(paSlices,0)); DBG("\r\n");
    return (-1);
  }
  
  if (15 != paSlices->cnt) {
    DBG("$__GGA message shall consists 15 fields, but there are recognized the ");
    DBGT(paSlices->cnt, DEC);DBG("\r\n");
    return (-1);
  }
  
  this->atDataStorage.msgs_rcvd[MSG_GGA]+=1;
  
  // UTC time status of position (hours/minutes/seconds/decimal seconds)  hhmmss.fff
  GNSSCollector::parseTime(GNSSCollector::get_field(paSlices,1), this->atDataStorage.UTC_H, this->atDataStorage.UTC_M, this->atDataStorage.UTC_S, this->atDataStorage.UTC_fract);
  
  this->atDataStorage.lat = GNSSCollector::convertAngle(GNSSCollector::get_field(paSlices,2)); // Latitude (DDmm.mm)
  
  if (0 == *GNSSCollector::get_field(paSlices,3)) {
    this->atDataStorage.lat_dir = 'U'; // Undefined
  } else {
    this->atDataStorage.lat_dir = *GNSSCollector::get_field(paSlices,3); // Latitude direction (N = North, S = South)
  }
  
  this->atDataStorage.lon = GNSSCollector::convertAngle(GNSSCollector::get_field(paSlices,4)); // Longitude (DDDmm.mm)
  
  if (0 == *GNSSCollector::get_field(paSlices,5)) {
    this->atDataStorage.lon_dir = 'U'; // Undefined
  } else {
    this->atDataStorage.lon_dir = *GNSSCollector::get_field(paSlices,5); // Longitude direction (E = East, W = West)
  }
  
  this->atDataStorage.quality = *GNSSCollector::get_field(paSlices,6); // GNSS Quality Indicators
  
  this->atDataStorage.sats = (uint8_t)atoi(GNSSCollector::get_field(paSlices,7)); // Number of satellites in use. May be different to the number in view
  
  this->atDataStorage.hdop = atof(GNSSCollector::get_field(paSlices,8)); // Horizontal dilution of precision
  
  this->atDataStorage.alt = atof(GNSSCollector::get_field(paSlices,9)); // Antenna altitude above/below mean sea level
  
  this->atDataStorage.a_units = *GNSSCollector::get_field(paSlices,10); // Units of antenna altitude (M = metres)
  if (0 == this->atDataStorage.a_units)
    this->atDataStorage.a_units = 'U'; // Unknown
  
  this->atDataStorage.undulation = atof(GNSSCollector::get_field(paSlices,11)); // Undulation - the relationship between the geoid and the WGS84 ellipsoid
  
  this->atDataStorage.u_units = *GNSSCollector::get_field(paSlices,12); // Units of undulation (M = metres)
  
  return 0;
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ****** The parser for $__VTG message. Takes data from the sliced message and stores them into the struct GNSS_data  *****
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/
int8_t GNSSCollector::VTG_parser(const struct NMEA_fields *paSlices) {
  
  if (strcmp("VTG", GNSSCollector::get_field(paSlices,0)+3)) {
    DBG("$__VTG message parser received different message: ");
    DBGV(GNSSCollector::get_field(paSlices,0)); DBG("\r\n");
    return (-1);
  }
  
  if (10 != paSlices->cnt) {
    DBG("$__VTG message shall consists 10 fields, but there are recognized the ");
    DBGT(paSlices->cnt, DEC);DBG("\r\n");
    return (-1);
  }
  
  this->atDataStorage.msgs_rcvd[MSG_VTG]+=1;
  
  this->atDataStorage.true_track         = atof(GNSSCollector::get_field(paSlices,1));  // Track made good, degrees True
  this->atDataStorage.true_track_ind     = *GNSSCollector::get_field(paSlices,2);       // true track indicator (track made good is relative to true north)
  this->atDataStorage.magnetic_track     = atof(GNSSCollector::get_field(paSlices,3));  // Track made good, degrees Magnetic
  this->atDataStorage.magnetic_track_ind = *GNSSCollector::get_field(paSlices,4);       // magnetic track indicator (track made good is relative to magnetic north)
  this->atDataStorage.nautical_speed     = atof(GNSSCollector::get_field(paSlices,5));  // speed value [knots]
  this->atDataStorage.nautical_speed_ind = *GNSSCollector::get_field(paSlices,6);       // nautical speed indicator (N = knots)
  this->atDataStorage.speed              = atof(GNSSCollector::get_field(paSlices,7));  // speed value [kmph]
  this->atDataStorage.speed_ind          = *GNSSCollector::get_field(paSlices,8);       // speed indicator (K = km/hr)
  this->atDataStorage.mode_ind           = *GNSSCollector::get_field(paSlices,9);       // Positioning system mode indicator (A - Autonomous, D - Differential, E - Estimated (dead reckoning) mode, M - Manual input, N - Data not valid)
  
  return 0;
}


/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ****** The parser for $__GSA message. Takes data from the sliced message and stores them into the struct GNSS_data  *****
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

int8_t GNSSCollector::GSA_parser(const struct NMEA_fields *paSlices) {
  uint8_t i,j;
  
  if (strcmp("GSA", GNSSCollector::get_field(paSlices,0)+3)) { // $GP...  +3 at the end
    DBG("$__GSA message parser received different message: ");
    DBGV(GNSSCollector::get_field(paSlices,0)); DBG("\r\n");
    return (-1);
  }
  
  if ((18 != paSlices->cnt) && (19 != paSlices->cnt)) { // sometimes system ID is not present
    DBG("$__GSA message shall consists 18 (or +1 system ID) fields, but there are recognized the ");
    DBGT(paSlices->cnt, DEC);DBG("\r\n");
    return (-1);
  }
  
  this->atDataStorage.msgs_rcvd[MSG_GSA]+=1;
  
  this->atDataStorage.modeMA = *GNSSCollector::get_field(paSlices,1);  // A = Automatic 2D/3D; M = Manual, forced to operate in 2D or 3D
  
  this->atDataStorage.mode123 = *GNSSCollector::get_field(paSlices,2); // Mode: 1 = Fix not available; 2 = 2D; 3 = 3D
  
  // PRN numbers of satellites used in solution (null for unused fields), total of 12 fields
  
  for (j = 0; PRN_SATS_MAX>j; j++) {
    // we are looking for the first empty row
    if (0 != this->atDataStorage.prn_sats[j].prn_gps[0])
      continue;
    
    for (i =0; i<12; i+=1) {
      this->atDataStorage.prn_sats[j].prn_gps[i] = (uint16_t)atoi(GNSSCollector::get_field(paSlices,i+3));
    }
    strncpy(this->atDataStorage.prn_sats[j].talker, GNSSCollector::get_field(paSlices,0)+1, 2);
    this->atDataStorage.prn_sats[j].systemID = -1; // We don't know yet if there is system ID available
    if (19 == paSlices->cnt) {
      if (0 != strlen(GNSSCollector::get_field(paSlices,18))) // it is necessary because QZSS has the number of 0 - atoi returns 0 if we read empty string
        this->atDataStorage.prn_sats[j].systemID = (uint16_t)atoi(GNSSCollector::get_field(paSlices,18));
    }
    break;
  }
  
  if ((PRN_SATS_MAX-1) < j) {
    DBG("There are too many $xxGSA messages - message ignored\r\n");
  }
  
  this->atDataStorage.pdop = atof(GNSSCollector::get_field(paSlices,15)); // Position dilution of precision
  this->atDataStorage.hdop = atof(GNSSCollector::get_field(paSlices,16)); // Horizontal dilution of precision
  this->atDataStorage.vdop = atof(GNSSCollector::get_field(paSlices,17)); // Vertical dilution of precision
  
  return 0;
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ****** The parser for $__GBS message. Takes data from the sliced message and stores them into the struct GNSS_data  *****
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

int8_t GNSSCollector::GBS_parser(const struct NMEA_fields *paSlices) {
  
  if (strcmp("GBS", GNSSCollector::get_field(paSlices,0)+3)) { // $GP...  +3 at the end
    DBG("$__GBS message parser received different message: ");
    DBGV(GNSSCollector::get_field(paSlices,0)); DBG("\r\n");
    return (-1);
  }
  
  if ((9 != paSlices->cnt) && (11 != paSlices->cnt)) { // sometime system ID is not present
    DBG("$__GBS message shall consists 9 (or +2 system ID/signal ID for NMEA 4.10+) fields, but there are recognized the ");
    DBGT(paSlices->cnt, DEC);DBG("\r\n");
    return (-1);
  }
  
  this->atDataStorage.msgs_rcvd[MSG_GBS]+=1;
  
  // UTC time status of position (hours/minutes/seconds/decimal seconds)  hhmmss.fff
  GNSSCollector::parseTime(GNSSCollector::get_field(paSlices,1), this->atDataStorage.UTC_H, this->atDataStorage.UTC_M, this->atDataStorage.UTC_S, this->atDataStorage.UTC_fract);
  
  this->atDataStorage.lat_err = atof(GNSSCollector::get_field(paSlices,2));
  this->atDataStorage.lon_err = atof(GNSSCollector::get_field(paSlices,3));
  this->atDataStorage.alt_err = atof(GNSSCollector::get_field(paSlices,4));
  
  return 0;
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ****** The parser for $__GST message. Takes data from the sliced message and stores them into the struct GNSS_data  *****
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

int8_t GNSSCollector::GST_parser(const struct NMEA_fields *paSlices) {
  
  if (strcmp("GST", GNSSCollector::get_field(paSlices,0)+3)) { // $GP...  +3 at the end
    DBG("$__GST message parser received different message: ");
    DBGV(GNSSCollector::get_field(paSlices,0)); DBG("\r\n");
    return (-1);
  }
  
  if (9 != paSlices->cnt) {
    DBG("$__GST message shall consists 9 fields, but there are recognized the ");
    DBGT(paSlices->cnt, DEC);DBG("\r\n");
    return (-1);
  }
  
  this->atDataStorage.msgs_rcvd[MSG_GST]+=1;
  
  // UTC time status of position (hours/minutes/seconds/decimal seconds)  hhmmss.fff
  GNSSCollector::parseTime(GNSSCollector::get_field(paSlices,1), this->atDataStorage.UTC_H, this->atDataStorage.UTC_M, this->atDataStorage.UTC_S, this->atDataStorage.UTC_fract);
  
  this->atDataStorage.RMS         = atof(GNSSCollector::get_field(paSlices,2)); // RMS value of the standard deviation of the ranges.
                                                                                // Includes carrier phase residuals during periods of RTK (float) and RTK (fixed) processing.
  this->atDataStorage.lat_std_dev = atof(GNSSCollector::get_field(paSlices,6));
  this->atDataStorage.lon_std_dev = atof(GNSSCollector::get_field(paSlices,7));
  this->atDataStorage.alt_std_dev = atof(GNSSCollector::get_field(paSlices,8));
  
  return 0;
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ****** The parser for $__GLL message. Takes data from the sliced message and stores them into the struct GNSS_data  *****
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

int8_t GNSSCollector::GLL_parser(const struct NMEA_fields  *paSlices) {
  
  if (strcmp("GLL", GNSSCollector::get_field(paSlices,0)+3)) { // $GP...  +3 at the end
    DBG("$__GLL message parser received different message: ");
    DBGV(GNSSCollector::get_field(paSlices,0)); DBG("\r\n");
    return (-1);
  }
  
  if (8 != paSlices->cnt) {
    DBG("$__GLL message shall consists 8 fields, but there are recognized the ");
    DBGT(paSlices->cnt, DEC);DBG("\r\n");
    return (-1);
  }
  
  this->atDataStorage.msgs_rcvd[MSG_GLL]+=1;
  
  this->atDataStorage.lat = GNSSCollector::convertAngle(GNSSCollector::get_field(paSlices,1)); // Latitude (DDmm.mm)
  
  if (0 == *GNSSCollector::get_field(paSlices,2)) {
    this->atDataStorage.lat_dir = 'U'; // Undefined
  } else {
    this->atDataStorage.lat_dir = *GNSSCollector::get_field(paSlices,2); // Latitude direction (N = North, S = South)
  }
  
  this->atDataStorage.lon = GNSSCollector::convertAngle(GNSSCollector::get_field(paSlices,3)); // Longitude (DDDmm.mm)
  
  if (0 == *GNSSCollector::get_field(paSlices,4)) {
    this->atDataStorage.lon_dir = 'U'; // Undefined
  } else {
    this->atDataStorage.lon_dir = *GNSSCollector::get_field(paSlices,4); // Longitude direction (E = East, W = West)
  }
  
  // UTC time status of position (hours/minutes/seconds/decimal seconds)  hhmmss.fff
  GNSSCollector::parseTime(GNSSCollector::get_field(paSlices,5), this->atDataStorage.UTC_H, this->atDataStorage.UTC_M, this->atDataStorage.UTC_S, this->atDataStorage.UTC_fract);
  
  this->atDataStorage.pos_status = *GNSSCollector::get_field(paSlices,6);
  this->atDataStorage.mode_ind = *GNSSCollector::get_field(paSlices,7); // Positioning system mode indicator (A - Autonomous, D - Differential, E - Estimated (dead reckoning) mode, M - Manual input, N - Data not valid)
  
  return(0);
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 *************** this function collects the data from the data source callback and then parses every line with the particular parser  **************
 ***************************************************************************************************************************************************
 ***** the first parameter is the flag to show the processed data in debug mode, the second one is useful for very weak CPU - e.g. Arduino UNO *****
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

int8_t GNSSCollector::collectData(bool paShowReceivedMessage = false, bool paShowCRNLVisible = true) {
  
  struct NMEA_fields loSlices;
  
  bool loSequenceStarted   = false;
  bool loSequenceCompleted = false;
  bool loHaveSomeDataFlag  = false;
  
  char loOneFullRow[MAXMESSAGELENGTH];
  uint8_t i;
  int8_t avl_result;
  
  memset((void *)&(this->atDataStorage), 0, sizeof(this->atDataStorage));
  if (NULL != this->atGSVData) {
    memset((void *)this->atGSVData, 0, sizeof(struct GSV_manager));
  }
  
  i = 0; // we will read every single character of the string and put them to the buffer from the index of 0
  
  while (! loSequenceCompleted) {
    
    while (((int8_t)0) < (avl_result = avl_callback())) {
      if ((MAXMESSAGELENGTH-3) < i) { // the 3 bytes space is needed for \r\n\0 terminating the string
        loOneFullRow[i] = 0;
        i=0;
        DBGV(loOneFullRow); DBG("\r\n");
        SETCOLORRED DBG("Received too long NMEA message (or some junk) for processing, so ignored\r\n"); NOCOLOR
        continue;
      }
      
      loOneFullRow[i] = (char)read_callback();
      
      if ('\n' == loOneFullRow[i++]) {
        loOneFullRow[i] = 0; // terminating the received string
        
        i = 0;  // reading will be continued as a new NMEA message
        
        if (paShowReceivedMessage) { // this is for debug purpose only
          uint8_t loFullRowLength; // declared here due to save RAM space on weak CPUs when DEBUG is turned off
          DBG("The NMEA msg -> ");
          loFullRowLength = strlen(loOneFullRow);
          if (3 < loFullRowLength) { // the second and third characters is the talkerID sequence to be decoded
            GNSSCollector::printTalkerName(loOneFullRow+1, true);
          }
          DBG(" - ("); DBGT((int)loFullRowLength,DEC); DBG("): ");
          
          if (paShowCRNLVisible) {
            char loOneCharacter;
            for (uint8_t loIT=0; loIT<loFullRowLength;loIT++) {
              loOneCharacter = loOneFullRow[loIT];
              switch (loOneCharacter) {
                case '\r':
                  DBG("\\r");
                  break;
                case '\n':
                  DBG("\\n");
                  break;
                default:
                  DBGC(loOneCharacter);
              }
            }
            DBG("\r\n");
          } else {
            DBGV(loOneFullRow);
          }
        } // paShowReceivedMessage - for debug purpose only
        if (check_and_slice_NMEA_message(loOneFullRow, &loSlices)) {
          // this is faulty NMEA message - we will not parse them and collect its data
          continue;
        } else {
          // this is correctly formatted NMEA message
          if (loSequenceStarted) {
            int8_t loFlag = 0;
            if (NULL != atCustomParser) {
              loFlag = atCustomParser(&loSlices);
            }
            if (loFlag) {
            } else if (0 > parse_NMEA_fields_for_particular_message(&loSlices)) {
              DBG("particular message parser returned error code\r\n");
              continue;
            }
            loHaveSomeDataFlag = true;
          }
        }
      } // '\n' occured
    } // while available callback
    if (0 > avl_result) {
      DBG("collectData timeout\r\n");
      return (-5);
    }
#ifdef ARDUINO
    delay(atMessagesBreakLength);
#elif __linux__
    usleep(atMessagesBreakLength * 1000);
#endif
    
    if (((int8_t)0) == (avl_result = avl_callback())) {
      
      // we are in the break between the packs of messages
      
      if (!loSequenceStarted) { // Now we will start collecting data
        if (paShowReceivedMessage) {
          SETCOLORBLUE DBG("Waiting for new sequence ...\r\n"); NOCOLOR
        }
        loSequenceStarted = 1;
      } else if (loHaveSomeDataFlag) { // we have all data of finished pack
        loSequenceCompleted = 1;
      } else { // we still are waiting for the first message of the new pack of messages (new sequence has not started yet)
        ;
      }
      
    } else if (((int8_t)0) > avl_result) {
      DBG("collectData timeout\r\n");
      return (-6);
    } else {
      ; // the subsequent message but not the first one in current pack of messages
    }
    
  } // while ! loSequenceCompleted
  return (0);
} /***** collectData ****/

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ** this function checks the integrity of NMEA message and slices the string on every single NMEA field (slice) to be ready for particular parser **
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/


int8_t GNSSCollector::check_and_slice_NMEA_message(const char *paSingleLine, struct NMEA_fields *paSlices) {
  
  uint8_t it;
  const char *ptrCurrentField;
  char chSumChar;
  char cSum;
  const uint8_t loLineLength = strlen(paSingleLine);
  
  if (NULL == paSingleLine) {
    SETCOLORRED DBG("The NMEA message is not given - NULL pointer\r\n"); NOCOLOR
    return (-1);
  }
  
  if (NULL == paSlices) {
    SETCOLORRED DBG("The memory area for NMEA message fields storage is not given.\r\n"); NOCOLOR
    return (-1);
  }
  
  if (MAXMESSAGELENGTH <= loLineLength) {
    SETCOLORRED DBG("The NMEA message is too long to be processed\r\n"); NOCOLOR
    return (-1);
  }
  
  if (11 > loLineLength) { // $__XXX,...,*CHECKSUM\r\n
    SETCOLORRED DBG("The NMEA message is too short for NMEA message format\r\n"); NOCOLOR
    return (-1);
  }
  
  if ('$' != *paSingleLine) {
    SETCOLORRED DBG("The NMEA message has no initial character for NMEA message format\r\n"); NOCOLOR
    return (-1);
  }
  
  if ( ('\n' != *(paSingleLine + loLineLength - 1))
    || ('\r' != *(paSingleLine + loLineLength - 2)) ) {
    SETCOLORRED DBG("The NMEA message has incorrect terminating characters for NMEA message format (\"\\r\\n\")\r\n"); NOCOLOR
    return (-1);
  }
  
  memset ((void *)paSlices, 0, sizeof(*paSlices));
  
  //  calculating check sum from the string excluding initial '$' and terminating checksum + "\r\n"
  paSlices->chSum = 0;
  for (it = 1; it < (loLineLength-5); it++) { // whole line except initial '$' and terminating "*<2_bytes_of_check_sum>\r\n"
    paSlices->chSum ^= *(paSingleLine+it);
  }
  
  ptrCurrentField = paSingleLine;
  paSlices->cnt = 0;
  while (strchr(ptrCurrentField, ',')) {
    if (MAXFIELDSINMESSAGE <= paSlices->cnt) {
      SETCOLORRED DBG("There are too many fields in the NMEA message1\r\n"); NOCOLOR
      return (-1);
    }
    paSlices->field_index[paSlices->cnt] = ptrCurrentField - paSingleLine;
    paSlices->cnt += 1;
    ptrCurrentField = strchr(ptrCurrentField, ',') + 1;
  }
  
  if (strchr(ptrCurrentField, '*')) { // the last one is check sum preceded with '*' character
    if (MAXFIELDSINMESSAGE <= paSlices->cnt) {
      SETCOLORRED DBG("There are too many fields in the NMEA message2\r\n"); NOCOLOR
      return (-1);
    }
    paSlices->field_index[paSlices->cnt] = ptrCurrentField - paSingleLine;
    paSlices->cnt += 1;
    ptrCurrentField = strchr(ptrCurrentField, '*') + 1;
  } else {
    SETCOLORRED DBG("The NMEA message has no '*' character preceding the check sum\r\n"); NOCOLOR
    return (-1);
  }
  
  strcpy(paSlices->message, paSingleLine);
  
  for (it=0; it<loLineLength; it++) {
    if (',' == paSlices->message[it] || '*'== paSlices->message[it]) {
      paSlices->message[it] = 0;
    }
  }
  
  // TODO - to check if the checksum could be one digit length
  if (4 != strlen(ptrCurrentField)) {  // two hex digits + \r\n
    SETCOLORRED DBG("The NMEA message has incorrect check sum string: \""); NOCOLOR // two HEX digits ended with "\r\n" are expected
    DBGV(ptrCurrentField); // the string is terminated with "\r\n"
    DBG("\"\\r\\n\tLength: ");
    DBGT((unsigned int)strlen(ptrCurrentField),DEC); DBG("\r\n");
    return (-1);
  }
  
  // reading check sum from the string (the last two bytes of the string terminated with \r\n
  cSum = 0;
  chSumChar = *(ptrCurrentField);
  if (('0'<=chSumChar) && ('9'>=chSumChar))
    cSum = (chSumChar-'0');
  else if (('A'<=toupper(chSumChar)) && ('F'>=toupper(chSumChar)))
    cSum = toupper(chSumChar) - 'A' + 10;
  else {
    SETCOLORRED DBG("The NMEA message terminated with incorrect character in the check sum field\r\n"); NOCOLOR
    return(-1);
  }
  cSum*=16;
  chSumChar = *(ptrCurrentField+1);
  
  if (('0'<=chSumChar) && ('9'>=chSumChar))
    cSum += (chSumChar-'0');
  else if (('A'<=toupper(chSumChar)) && ('F'>=toupper(chSumChar)))
    cSum += toupper(chSumChar) - 'A' + 10;
  else {
    SETCOLORRED DBG("The NMEA message terminated with incorrect character in the check sum field\r\n"); NOCOLOR
    return(-1);
  }
  
  if (cSum != paSlices->chSum) {
    SETCOLORRED
    DBG("Inconsistent check sum - calculated: 0x");
    DBGT((unsigned int)(paSlices->chSum), HEX);
    DBG(", from the string: 0x");
    DBGT((unsigned int)cSum, HEX); DBG("\r\n");
    NOCOLOR
    return (-2);
  }
  return (0); // successfull
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ************************************** this function adds the correct "*<CHckSum>\r\n" to the given NMEA message string ***************************
 **************************** there has to be allocated RAM space for additional 5 bytes (this function doesn't check it !!!) **********************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

// WARNING: !!!!! Make sure you have the extra 5 bytes space at the end of the given by paMessage string
//  to put there the 5 bytes: "*<2bytesChckSum>\r\n"

int8_t completeTheNMEAMessage(char *paMessage) {
  
  if (NULL == paMessage) {
    DBG("The message has to be given to be completed\r\n");
    return (-1);
  }
  
  if ('$' != *paMessage) {
    DBG("The NMEA message must begin with a \"$\" character\r\n");
    return (-2);
  }
  
  const uint8_t loLen = strlen(paMessage);
  char loChSum;
  uint8_t i;
  
  loChSum = 0;
  for (i = 1; i < (loLen); i++) { // whole line except the initial '$' character
    loChSum ^= *(paMessage+i);
  }
  sprintf(paMessage+loLen, "*%02X\r\n", (int)loChSum);
  return (0);
}

/***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ******   *****
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************
 ***************************************************************************************************************************************************/

#endif
