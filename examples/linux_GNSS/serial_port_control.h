#ifndef SERIAL_PORT_CONTROL_H
#define SERIAL_PORT_CONTROL_H


/*
  The Serial port control utility file
  By: Kazimierz Wilk
  Date: January, 2024
  License: GNU Lesser General Public License. See license file for more information.

  This module is the part of for the ultimateGNSSParser library.
*/





#include <termios.h>  /* struct termios */
#include <unistd.h>   /* struct termios */

#include <stdio.h>    /* fprintf */
#include <string.h>   /* memset */

#include <errno.h>    /* errno */


/*========================================================================================================================*/

unsigned int spc_parse_speed(const speed_t paSpeed) {
  switch (paSpeed) {
    case 1200:
            fprintf(stderr, "The serial port bitrate is 1200 [bps]\r\n");
            return(B1200);
            break;
    case 1800:
            fprintf(stderr, "The serial port bitrate is 1800 [bps]\r\n");
            return(B1800);
            break;
    case 2400:
            fprintf(stderr, "The serial port bitrate is 2400 [bps]\r\n");
            return (B2400);
            break;
    case 4800:
            fprintf(stderr, "The serial port bitrate is 4800 [bps]\r\n");
            return(B4800);
            break;
    case 9600:
            fprintf(stderr, "The serial port bitrate is 9600 [bps]\r\n");
            return(B9600);
            break;
    case 19200:
            fprintf(stderr, "The serial port bitrate is 19200 [bps]\r\n");
            return(B19200);
            break;
    case 38400:
            fprintf(stderr, "The serial port bitrate is 38400 [bps]\r\n");
            return(B38400);
            break;
    case 57600:
            fprintf(stderr, "The serial port bitrate is 57600 [bps]\r\n");
            return(B57600);
            break;
    case 115200:
            fprintf(stderr, "The serial port bitrate is 115200 [bps]\r\n");
            return(B115200);
            break;
    case 230400:
            fprintf(stderr, "The serial port bitrate is 230400 [bps]\r\n");
            return(B230400);
            break;
    case 460800:
            fprintf(stderr, "The serial port bitrate is 460800 [bps]\r\n");
            return(B460800);
            break;
    case 500000:
            fprintf(stderr, "The serial port bitrate is 500000 [bps]\r\n");
            return(B500000);
            break;
    case 576000:
            fprintf(stderr, "The serial port bitrate is 576000 [bps]\r\n");
            return(B576000);
            break;
    case 921600:
            fprintf(stderr, "The serial port bitrate is 921600 [bps]\r\n");
            return(B921600);
            break;
    default:
            fprintf(stderr, "The given bitrate %d is illegal, 9600 [bps] is set\r\n", paSpeed);
            return (B9600);
  }
}

/*========================================================================================================================*/

int spc_set_interface_attribs (const int fd, const speed_t paSpeed) {
  struct termios tty;
  char *error_str;
  
  memset (&tty, 0, sizeof (tty));
  if ( 0 != tcgetattr (fd, &tty) ) {
    error_str = strerror(errno);
    fprintf (stderr, "%s:%d Error tcgetattr %d: %s\n", __FILE__, __LINE__, errno, error_str);
    return (-1);
  }
  
  /* Speed settings */
  if ( 0 != cfsetspeed (&tty, spc_parse_speed(paSpeed)) ) { // This function sets the input and output speed
    error_str = strerror(errno);
    fprintf (stderr, "%s:%d Error cfsetspeed %d: %s\n", __FILE__, __LINE__, errno, error_str);
    return (-2);
  }
  
  cfmakeraw(&tty);
  
  if ( 0 != tcsetattr (fd, TCSANOW, &tty) ) {
    error_str = strerror(errno);
    fprintf (stderr, "%s:%d Error tcsetattr %d: %s\n", __FILE__, __LINE__, errno, error_str);
    return (-4);
  }
  return 0;
}

/*========================================================================================================================*/

int spc_set_blocking (const int fd, const int should_block) {
  
  struct termios tty;
  char *error_str;
  
  memset ( &tty, 0, sizeof(tty) );
  if ( 0 != tcgetattr (fd, &tty) ) {
    error_str = strerror(errno);
    fprintf (stderr, "%s:%d Error tcgetattr %d: %s\n", __FILE__, __LINE__, errno, error_str);
    return (-1);
  }
  
  tty.c_cc[VMIN]  = (0 != should_block) ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
  
  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    error_str = strerror(errno);
    fprintf (stderr, "%s:%d Error tcsetattr %d: %s\n", __FILE__, __LINE__, errno, error_str);
    return (-2);
  }
  return 0;
}


/*========================================================================================================================*/

#endif /* #define SERIAL_PORT_CONTROL_H */
