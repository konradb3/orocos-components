/*
 * ourptz.cpp
 *
 *  Created on: Jan 8, 2010
 *      Author: konradb3
 */

#include <stdio.h>

#include <math.h>
#include "ourptz.h"

double DTOR(double angle)
{
  return angle*M_PI/180;
}

double RTOD(double angle)
{
  return angle*180/M_PI;
}

// Constructor opens the serial port, and read the config info from it
Ourptz::Ourptz(const char *port, int rate, Ourptz_settings settings)
{
    tr = pr = 1;
    fd = -1;
    m_settings = settings;
    // open the serial port

    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        fprintf(stderr, "Could not open serial device %s\n", port);
        return;
    }
    //fcntl(fd,F_SETFL, 0);

    // save the current io settings
    tcgetattr(fd, &oldtio);

    // rtv - CBAUD is pre-POSIX and doesn't exist on OS X
    // should replace this with ispeed and ospeed instead.

    // set up new settings
    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = CBAUD | CS8 | CLOCAL | CREAD | CSTOPB;
    newtio.c_iflag = INPCK; //IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    if (cfsetispeed(&newtio, rate) < 0 || cfsetospeed(&newtio, rate) < 0) {
        fprintf(stderr, "Failed to set serial baud rate: %d\n", rate);
        tcsetattr(fd, TCSANOW, &oldtio);
        close(fd);
        fd = -1;
        return;
    }
    // activate new settings
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    SetStop();
    SetStop();
    SetStop();

    TMin = DTOR(-10.0);
    TMax = DTOR(92.0);
}

bool Ourptz::SetStop()
{
    unsigned char stop = T_STOP;
    Write((char *) &stop, 1);
    return true;
}

Ourptz::~Ourptz()
{
    // restore old port settings
    if (fd > 0) tcsetattr(fd, TCSANOW, &oldtio);

    close(fd);
}

int
Ourptz::Write(char *data, int length)
{

    if (fd < 0)
        return -1;

    // autocalculate if using short form
    if (length == 0)
        length = strlen(data);

    tcflush(fd, TCIOFLUSH);
#if 0
    // ugly error handling, if write fails then shut down unit
    while (length--) {
        if (write(fd, data++, 1) != 1) {
            fprintf(stderr, "Error writing to Pan Tilt Unit, disabling\n");
            tcsetattr(fd, TCSANOW, &oldtio);
            fd = -1;
            return -1;
        }
        usleep(1);
    }
#else
    if (write(fd, data, length) < length) {
        fprintf(stderr, "Error writing to Pan Tilt Unit, disabling\n");
        tcsetattr(fd, TCSANOW, &oldtio);
        fd = -1;
        return -1;
    }
#endif
    tcdrain(fd);
    return 0;
}

// get position in degrees
double
Ourptz::GetPos()
{
    unsigned char buffer[4];

    if (fd < 0)
        return -1;

    char cmd = T_GETPOS;

    fd_set rfds;

    struct timeval tv;
    int len = 0;
    int tries = 10;

    // get pan pos
    Write(&cmd, 1);

    while (len < 2) {
        tv.tv_sec = 0;
        tv.tv_usec = 30000;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        int s = select(fd + 1, &rfds, NULL, NULL, &tv);

        if (s < 0) {
            perror("select()");
            break;
        }

        if (s == 0) {
            fprintf(stderr, "ourptz: no reply (%d)\n", tries);
            if (tries--) {
                continue;
            } else {
                break;
            }
        }

        if (!FD_ISSET(fd, &rfds)) {
            fprintf(stderr, "ourptz: no reply from port\n");
        }

        int r = read(fd, &buffer[len], 2 - len);

        if (r > 0) {
            len += r;
        }
    }

    if (len != 2 || ((buffer[0] & 0xf0) != 0xf0)) {
        fprintf(stderr, "Error getting pan-tilt pos (len=%d)\n", len);

        printf("%d:", len);
        for (int i = 0; i < len; i++) {
            printf(" %02x", buffer[i] & 0xff);
        }
        printf("\n");

        if (prev_pos > 0) {
            int ret = prev_pos;
            prev_pos = -1;
            return ret;
        }

        return -1;
    }

    /*
    printf("%d:", len);
    for(int i = 0; i < len; i++) {
      printf(" %02x", buffer[i] & 0xff);
    }
    printf("\n");
    */

    int wynik;

    wynik=(buffer[1]) + ((buffer[0]&0x0f)<<7);
    //printf("%d:\n", wynik);

    double wynikd = M_PI_2 * double(wynik - m_settings.offset) / double(m_settings.pos_res);
    //wynik*=2; // mechatronika

    //printf("wynik = %d, wynikd = %f RAD\n", wynik, wynikd);

    prev_pos = wynik;
    //printf("%4.2f:\n", wynikd);
    //printf("%+d.%02d\n", wynik/100, abs(wynik%100));

    return wynikd;
}

// set position in degrees
bool
Ourptz::SetPos(double pos, bool Block)
{
   //printf("SetPos(%f)\n", pos);
    //pos/=2; // mechatronika
   // printf("%f:\n", pos);
    //printf("%+2d.%02d %c %c\n", pos/100, abs(pos%100), type, Block ? 'b' : 'n');

    if (fd < 0)
        return false;

    // Check limits
    if (pos < TMin || pos > TMax) {
        fprintf(stderr, "Tilt Value out of Range: %f <%+.2f:%+.2f>\n",
                RTOD(pos), RTOD(TMin), RTOD(TMax));
        return false;
    }

    double cposd;
    int cpos;

    cposd = double(m_settings.offset) + pos * double(m_settings.pos_res) / M_PI_2;
    //printf("%4.2f:\n", cposd);
    cpos = lrint(cposd);
   // printf("%d\n",cpos);
    char cmd[2];
    cmd[1] = cpos & 0x7f;
    cpos = cpos >> 7;
    cmd[0] = T_GOTO | (cpos&0x0f);

    /*
     * printf("-> (%d -> %.2f) %02x %02x\n",
     * pos, cposd,
     * cmd[0] & 0xff, cmd[1] & 0xff);
     */

    // set pos
    Write(cmd, 2);

    if (Block) {
        while (GetPos() != pos);
    }
    return true;
}

// get speed in degrees/sec
int
Ourptz::GetSpeed()
{
// Nie ma zastosowania w naszym przypadku
/*    if (fd < 0)
        return -1;

    char cmd[4] = " s ";
    cmd[0] = type;

    // get speed
    int len = 0;
    Write(cmd);
    len = read(fd, buffer, Ourptz_BUFFER_LEN);

    if (len < 3 || buffer[0] != '*') {
        fprintf(stderr, "Error getting pan-tilt speed\n");
        return -1;
    }

    buffer[len] = '\0';
*/
    return -1;
}

// set speed in degrees/sec
bool
Ourptz::SetSpeed(double speed)
{
    printf("SetSpeed: %3f\n", speed);

    if (fd < 0)
        return false;
    int ispeed = lrint(speed);

    char cmd[2];
    cmd[1] = ispeed & 0x7f;
    ispeed = ispeed >> 7;
    cmd[0] = T_SETSPEED | (ispeed&0x0f);

    /*
      printf("-> (%d -> %.2f) %02x %02x\n",
      ispeed, speed,
      cmd[0] & 0xff, cmd[1] & 0xff);
     */

    // set speed
    Write(cmd, 2);

    return true;
}

// set movement mode (position/velocity)
bool
Ourptz::SetMode(char type)
{
    if (fd < 0)
        return false;
// To też nie ma zastosowania
/*   char cmd[4] = "c  ";
    cmd[1] = type;

    // set mode
    int len = 0;
    Write(cmd);
    len = read(fd, buffer, Ourptz_BUFFER_LEN);

    if (len <= 0 || buffer[0] != '*') {
        fprintf(stderr, "Error setting pan-tilt move mode\n");
        return false;
    }*/
    return true;
}

// get position in degrees
char
Ourptz::GetMode()
{
    if (fd < 0)
        return -1;

    // get pan tilt mode
    int len = 0;
    Write("c ");
    len = read(fd, buffer, Ourptz_BUFFER_LEN);

    if (len < 3 || buffer[0] != '*') {
        fprintf(stderr, "Error getting pan-tilt pos\n");
        return -1;
    }

    if (buffer[2] == 'p')
        return Ourptz_VELOCITY;
    else if (buffer[2] == 'i')
        return Ourptz_POSITION;
    else
        return -1;
}


