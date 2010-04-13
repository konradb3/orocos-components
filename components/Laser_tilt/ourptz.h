/*
 * ourptz.h
 *
 *  Created on: Jan 8, 2010
 *      Author: Konrad Banachowicz
 */

#ifndef OURPTZ_H_
#define OURPTZ_H_

// serial includes
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <strings.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <math.h>

#define T_STOP 0x80
#define T_SCAN 0x90
#define T_GETPOS 0xa0
#define T_CALIBRATE 0xb0
#define T_SETSPEED 0xe0
#define T_GOTO 0xf0

//serial defines
#define Ourptz_DEFAULT_BAUD B38400
#define Ourptz_BUFFER_LEN 255

// command defines
#define Ourptz_MIN 'n'
#define Ourptz_MAX 'x'
#define Ourptz_MIN_SPEED 'l'
#define Ourptz_MAX_SPEED 'u'
#define Ourptz_VELOCITY 'v'
#define Ourptz_POSITION 'i'

#define Ourptz_DEFAULT_OFFSET 227

#define DEFAULT_PTZ_PORT "/dev/ttyS0"

//
// Pan-Tilt Control Class
//

struct Ourptz_settings{
        int offset; /// Pozycja początkowa w jednostkach enkodera
        int pos_res; /// Ilość jednostek na obrót enkodera
};

class Ourptz {
  public:

    Ourptz(const char *port, int rate, Ourptz_settings settings);
    ~Ourptz();

    // get/set position in degrees
    double GetPos();
    bool SetPos(double pos, bool Block = false);
    bool SetStop();

    // get/set speed in degrees/sec
    bool SetSpeed(double speed);
    int GetSpeed();

    // get/set move mode
    bool SetMode(char type);
    char GetMode();

    bool Open() {
        return fd > 0;
    };

    // Position Limits
    double TMin, TMax;
    // Speed Limits
    double TSMin, TSMax;

  private:
    // pan and tilt resolution
    float tr, pr;

    // serial port descriptor
    int fd;
    struct termios oldtio;

    // read buffer
    char buffer[Ourptz_BUFFER_LEN + 1];

    // write data to serial port
    int Write(char *data, int length = 0);

    int prev_pos;

    Ourptz_settings m_settings;

};


#endif /* OURPTZ_H_ */
