/* hercules.h
 * 
 * 2017-03-04 catmaker@github.com Adnan Jalaludin
 *
 * Defines for inter modules paramter passing and functions declarations
 *
 * Ref: http://tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html
 */

#ifndef HERCULES_H
#define HERCULES_H

extern char herc_inbuf[];

extern int herc_open(char *devpathname);
extern int motor_speed(int speedA, int speedB);
extern void herc_close();
extern int herc_poll();
extern int herc_getpid();
extern int herc_setpid();
extern int herc_piden(int onoff);
extern int herc_setodolimit();
extern int herc_getbatt();

#endif
