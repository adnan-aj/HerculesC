/* herc-comm.c
 * 
 * 2017-03-04 catmaker@github.com Adnan Jalaludin
 *
 * Functions to open comm port and interface to Seeedstudio Hercules motor controller
 *
 * Ref: http://tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html
 */
  

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "hercules.h"

#define DEFAULTDEVICE	"/dev/ttyMFD1"
#define HERC_AVR_BPS	B57600
//#define _POSIX_SOURCE	1

int hfd;
static int inlen;
char herc_inbuf[128];
static struct termios oldtio, newtio;

int herc_open(char *devpathname)
{
    hfd = open(devpathname, O_RDWR | O_NOCTTY | O_NDELAY);
    if (hfd < 0) {
	perror(devpathname);
	return -1;
    }

    tcgetattr(hfd, &oldtio);
    bzero(&newtio, sizeof(newtio));

    // newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtio.c_cflag = HERC_AVR_BPS | CS8 | CLOCAL | CREAD;
    // if reading binary packets, below may have to remove ICRNL
    newtio.c_iflag = IGNPAR | ICRNL;
    // raw output
    newtio.c_oflag = 0;
    newtio.c_lflag = ICANON;
    // left out all other control char params of reference code
    newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
    newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 char arrives */
    tcflush(hfd, TCIFLUSH);
    tcsetattr(hfd, TCSANOW, &newtio);
    fcntl(hfd, F_SETFL, O_NONBLOCK);
    return hfd;
}

int motor_speed(int speedA, int speedB)
{
    char outbuf[128];
    if (hfd <= 0) return -1;
    sprintf(outbuf, "mot %d %d\n", speedA, speedB);
    write(hfd, outbuf, strlen(outbuf));
    return 0;
}

void herc_close()
{
    motor_speed(0, 0);
    tcsetattr(hfd, TCSANOW, &oldtio);
    close(hfd);
}

int herc_poll()
{
    if (hfd <= 0) return -1;
    if (read(hfd, herc_inbuf + inlen, 1) <= 0)
	return 0;
    int ch = herc_inbuf[inlen++];
    if ((ch == '\n') || (ch == '\r') || (inlen > 127)) {
	int rvalue = inlen;
	herc_inbuf[inlen-1] = 0;
	inlen = 0;
	return rvalue;
    }
    return 0;
}

int herc_piden(int onoff)
{
    char outbuf[128];
    if (hfd <= 0) return -1;
    sprintf(outbuf,"pid %s\n", onoff ? "on" : "off");
    write(hfd, outbuf, strlen(outbuf));
}

int herc_getpid()
{
    return 0;
}

int herc_setpid()
{
    return 0;
}

int herc_setodolimit()
{
    return 0;
}

int herc_getbatt()
{
    return 0;
}
