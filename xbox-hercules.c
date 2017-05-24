/*
 * Adnan's Intel Edison 4 wheel drive xbox controlled rover
 *
 * adnan@singnet.com.sg
 *
 * uses pwm0 - pwm3 for motor speeds
 * uses gpio128, gpio129, gpio48, gpio49 for motor directions
 *
 * Nice joystick-to-differential drive reference:
 * http://www.phidgets.com/docs/Mobile_Robot_(MURVV)#Calculate_Wheel_Speeds
 *
 * uses evtest.c for debugging info
 *
 * gcc -o xbox-4wd evtest.c xbox-4wd.c
 *
 */
#define _GNU_SOURCE


#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <linux/input.h>
#include <poll.h>
#include <time.h>

#include "hercules.h"

extern char *events[];
extern char **names[];
extern char *absval[];

#define BITS_PER_LONG (sizeof(long) * 8)
#define NBITS(x) ((((x)-1)/BITS_PER_LONG)+1)
#define OFF(x)  ((x)%BITS_PER_LONG)
#define BIT(x)  (1UL<<OFF(x))
#define LONG(x) ((x)/BITS_PER_LONG)
#define test_bit(bit, array)	((array[LONG(bit)] >> OFF(bit)) & 1)


int mfd = 0;	// motor serial port file desc
int xfd = 0;	// external xbox controller
int joystickX = 0;
int joystickY = 0;
int motorA_setting = 0;
int motorB_setting = 0;
int diff_steering(int joyX, int joyY);
volatile bool interrupted = false;


void int_handler(int signumxs) {

    interrupted = true;
}

int open_xbox()
{
    int fd, i;
    char devpath[64];
    char devname[256];
    int version;
    unsigned short id[4];
	
    for (i = 0; i < 64; i++) {
	sprintf(devpath, "/dev/input/event%d", i);
	printf("Testing %s...  ", devname);
	if ((fd = open(devpath, O_RDONLY)) >= 0) {
	    strcpy(devname, "Unknown");
	    ioctl(fd, EVIOCGNAME(sizeof(devname)), devname);
	    printf("found: %s\n", devname);
	    if (strcasestr(devname, "xbox")) {
		printf("Using this one!\n");
		break;
	    }
	    close(fd);
	}
    }
    if (ioctl(fd, EVIOCGVERSION, &version)) {
	perror("evtest: can't get version");
	return -1;
    }
    printf("Input driver version is %d.%d.%d\n",
	   version >> 16, (version >> 8) & 0xff, version & 0xff);

    //fcntl(fd, F_SETFL, O_NONBLOCK);
    
    ioctl(fd, EVIOCGID, id);
    printf("Input device ID: bus 0x%x vendor 0x%x product 0x%x version 0x%x\n",
	   id[ID_BUS], id[ID_VENDOR], id[ID_PRODUCT], id[ID_VERSION]);
    
    return fd;
}

void print_event(char *typename, struct input_event ev)
{
    printf("%sEvent: time %ld.%06ld, type %d (%s), code %d (%s), value %d\n",
	   typename, ev.time.tv_sec, ev.time.tv_usec, ev.type,
	   events[ev.type] ? events[ev.type] : "?",
	   ev.code,
	   names[ev.type] ? (names[ev.type][ev.code] ? names[ev.type][ev.code] : "?") : "?",
	   ev.value);
}

void usage(char *appname)
{
    printf("usage: %s [logfilename]\n", appname);
    printf("\tOn Xbox controller:\n");
    printf("\tend: small back-button + main mode button\n");
    printf("\tstop: red \"B\" button\n");
    printf("\ttoggle PID: yellow \"Y\" button\n");
    printf("\txboxtext: button select (?)\n");
}




int main(int argc, char **argv)
{
    FILE *logfp;
    int rd, i;
    struct sigaction act;
    struct input_event ev[64];
    int btnselect = 0, btnstart = 0, btnmode = 0, xboxtext = 0;
    bool motor_update = false;
    int piden = 1;
    //unsigned long bit[EV_MAX][NBITS(KEY_MAX)];
    //int abs[5];


    time_t current_time;
    struct tm * time_info;
    char timeString[40];
    int oldsec = 0;


    //http://beej.us/guide/bgnet/output/html/multipage/pollman.html
    int rv;
    struct pollfd ufds[2];
    
    if (argc > 1) {
	if (strcmp(argv[1], "--help") == 0) {
	    usage(argv[0]);
	    return 0;
	}
	
	if ((logfp = fopen(argv[1], "a")) == NULL) {
	    printf("Log file %s cannot open\n", argv[1]);
	    return 1;
	}
    }
    else
	logfp = stdout;

    if ((xfd = open_xbox()) < 0) {
	printf("Xbox driver not found fd = %d\n", xfd);
	return 1;
    }

    if ((mfd = herc_open("/dev/ttyMFD1")) < 0) {
	printf("Hercules motor serial port cannot open fd = %d\n", mfd);
	return 1;
    }

    memset(&act, 0, sizeof(act));
    act.sa_handler = int_handler;
    sigaction(SIGINT, &act, NULL);
    printf("Testing ... (interrupt to exit)\n");
    
    ufds[0].fd = mfd;
    ufds[0].events = POLLIN | POLLPRI; // check for normal or out-of-band
    ufds[1].fd = xfd;
    ufds[1].events = POLLIN | POLLPRI; // check for just normal data

    while (!interrupted) {
	rv = poll(ufds, 2, 500);

	if (rv == -1) {
	    perror("poll"); // error occurred in poll()
	} else if (rv == 0) {
	    printf("Timeout occurred!  No data after 3.5 seconds.\n");
	} else {
	    if (ufds[0].revents & (POLLIN | POLLPRI)) {
		if (herc_poll()) {

		    time(&current_time);
		    time_info = localtime(&current_time);
		    if (time_info->tm_sec != oldsec) {
			oldsec = time_info->tm_sec;
			// format is 2017-01-01 13:48:56
			strftime(timeString, sizeof(timeString), "%F %T", time_info);
			fprintf(logfp, "Time: %s\n", timeString);
		    }

		    if (strstr(herc_inbuf, "odo:") != NULL)
			fprintf(logfp, "%s \n", herc_inbuf);
		    else
			printf("Log: %s\n", herc_inbuf);
		}
	    }

	    if (ufds[1].revents & (POLLIN | POLLPRI)) {
		// external xbox events
		//recv(s1, buf2, sizeof buf2, 0);
		rd = read(xfd, ev, sizeof(struct input_event) * 64);
		if (rd < (int) sizeof(struct input_event)) {
		    printf("yyy\n");
		    perror("\nevtest: error reading");
		}
		else {
		    for (i = 0; (i < rd / sizeof(struct input_event)) && (!interrupted); i++) {
			
			if (ev[i].type == EV_SYN) {
			    if (xboxtext)
				print_event("Sync", ev[i]);
			    
			} else if (ev[i].type == EV_MSC && (ev[i].code == MSC_RAW || ev[i].code == MSC_SCAN)) {
			    if (xboxtext)
				print_event("MSC_", ev[i]);
			    
			} else {
			    if (xboxtext)
				print_event("Else", ev[i]);
			    
			    if (ev[i].type == EV_KEY && ev[i].code == BTN_SELECT) {
				btnselect = ev[i].value;
				printf("Interesting %d!!!\n", btnselect);
				if (btnselect == 0)
				    xboxtext = 1 - xboxtext;
			    }
			    if (ev[i].type == EV_KEY && ev[i].code == BTN_START) {
				btnstart = ev[i].value;
				printf("Interesting %d!!!\n", btnstart);
			    }
			    if (ev[i].type == EV_KEY && ev[i].code == BTN_Y) {
				printf("Interesting %d!!!\n", btnstart);
				if (ev[i].value == 1) {
				    piden = 1 - piden;
				    herc_piden(piden);
				}
			    }
			    if (ev[i].type == EV_ABS && ev[i].code == ABS_RX) {
				//motorB_setting = -ev[i].value w/ 64;
				joystickX = -ev[i].value;
				motor_update = true;
			    }
			    
			    if (ev[i].type == EV_ABS && ev[i].code == ABS_Y) {
				//motorA_setting = -ev[i].value / 64;
				joystickY = -ev[i].value;
				motor_update = true;
			    }
			    
			    if (ev[i].type == EV_KEY && ev[i].code == BTN_B) {
				joystickX = joystickY = 0;
				motor_update = true;
			    }

			    if (ev[i].type == EV_KEY && ev[i].code == BTN_MODE) {
				if (btnselect) {
				    interrupted = 1;
				}
			    }
			    
			    if (interrupted) {
				joystickX = joystickY = 0;
				motor_update = true;
			    }

			    if (motor_update) {
				diff_steering(joystickX, joystickY);
				motor_speed(motorA_setting, motorB_setting);
				motor_update = false;
			    }
			    
			}	
		    }
		}
	    }
	}
    }
    herc_close();
    fclose(logfp);
    return 0;
}

int diff_steering(int joyX, int joyY)
{
    // Differential Steering Joystick Algorithm
    // ========================================
    //   by Calvin Hass
    //   http://www.impulseadventure.com/elec/
    //
    // Converts a single dual-axis joystick into a differential
    // drive motor control, with support for both drive, turn
    // and pivot operations.
    //
    
    // INPUTS
    int     nJoyX = joyX / 256;              // Joystick X input                     (-128..+127)
    int     nJoyY = joyY / 256;              // Joystick Y input                     (-128..+127)

    // OUTPUTS
    int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
    int     nMotMixR;           // Motor (right) mixed output           (-128..+127)

    // CONFIG
    // - fPivYLimt  : The threshold at which the pivot action starts
    //                This threshold is measured in units on the Y-axis
    //                away from the X-axis (Y=0). A greater value will assign
    //                more of the joystick's range to pivot actions.
    //                Allowable range: (0..+127)
    float fPivYLimit = 32.0;
    
    // TEMP VARIABLES
    float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
    float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
    int     nPivSpeed;      // Pivot Speed                          (-128..+127)
    float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
    
    
    // Calculate Drive Turn output due to Joystick X input
    if (nJoyY >= 0) {
	// Forward
	nMotPremixL = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
	nMotPremixR = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
    } else {
	// Reverse
	nMotPremixL = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
	nMotPremixR = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
    }
    
    // Scale Drive output due to Joystick Y input (throttle)
    nMotPremixL = nMotPremixL * nJoyY/128.0;
    nMotPremixR = nMotPremixR * nJoyY/128.0;
    
    // Now calculate pivot amount
    // - Strength of pivot (nPivSpeed) based on Joystick X input
    // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
    nPivSpeed = nJoyX;
    fPivScale = (abs(nJoyY)>fPivYLimit)? 0.0 : (1.0 - abs(nJoyY)/fPivYLimit);
    
    // Calculate final mix of Drive and Pivot
    nMotMixL = (1.0-fPivScale)*nMotPremixL + fPivScale*( nPivSpeed);
    nMotMixR = (1.0-fPivScale)*nMotPremixR + fPivScale*(-nPivSpeed);
    
    // Convert to Motor PWM range
    // ...
    motorB_setting = nMotMixL;
    motorA_setting = nMotMixR;
    return 0;
}
