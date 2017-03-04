/*
 * HerculesC.c
 *
 * Created: 2017-02-13 04:28:36 PM
 * Author : Adnan
 */ 

#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#include "hercules.h"
#include "usart.h"
#include "Arduino.h"
#include "seeed_pwm.h"
#include "pid.h"

static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar, NULL, _FDEV_SETUP_WRITE);

const long PWM_FREQ = 10000;

byte odoPtr = 0;
bool odoFull = false;
uint16_t odoA[16], odoB[16];

volatile unsigned int downtimer, timer1sec;
volatile bool odoUpdated = false;
volatile int motorA_count, motorB_count;	// pulse counts (in 100ms period)
int motorA_speed, motorB_speed;				// last counter record
int motorA_setting, motorB_setting;			// signed speed setting
struct PID_DATA motorA_PidData;
struct PID_DATA motorB_PidData;

bool pid_en = true;
bool adc_en = false;
bool odo_en = true;

#define K_P		0.2
#define K_I		0.1
#define K_D		0.02

void doResetPID(void)
{
	// Uses factory-set PID constants
	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &motorA_PidData);
	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &motorB_PidData);
}


void motorAdir(bool forward)
{
	if (forward) {
		PORTD |= _BV(PORTD4);
		PORTD &= ~_BV(PORTD5);
	}
	else {
		PORTD &= ~_BV(PORTD4);
		PORTD |= _BV(PORTD5);
	}
}

void motorBdir(bool forward)
{
	if (forward) {
		PORTD |= _BV(PORTD7);
		PORTB &= ~_BV(PORTB0);
	}
	else {
		PORTD &= ~_BV(PORTD7);
		PORTB |= _BV(PORTB0);
	}
}

void motorA_setPWM(int value)
{
	if (value > 0) {
		motorAdir(false);
	}
	else {
		value = -value;
		motorAdir(true);
	}
	setPwm(9, value, PWM_FREQ);
}

void motorB_setPWM(int value)
{
	if (value > 0) {
		motorBdir(true);
	}
	else {
		value = -value;
		motorBdir(false);
	}
	setPwm(10, value, PWM_FREQ);
}

ISR(INT0_vect) //INT0 = D2 = PD2
{
	if (PINC & (1<<PINC2))
		motorA_count--;
	else
		motorA_count++;
}

ISR(INT1_vect) //INT1 = D3 = PD3
{
	if (PINC & (1<<PINC3))
		motorB_count++;
	else
		motorB_count--;
}

ISR(TIMER1_OVF_vect)
{
	if (downtimer) {
		downtimer--;
	}
	else {
		D11_ON();
		downtimer = 1000;
		motorA_speed = motorA_count;
		motorB_speed = motorB_count;
		motorA_count = motorB_count = 0;
		odoUpdated = true;
		D11_OFF();
	}
}

char inbuf[128];
char *inptr = inbuf;

int readline()
{
	return 0;
}

int readline_nb()
{
	byte c;
	while (usart_havechar() && (inptr - inbuf) < sizeof(inbuf)) {
		RED_ON();
		c = usart_getc();
		if (c == '\r' || c == '\n') {
			*inptr = 0;
			inptr = inbuf;
			RED_OFF();
			return 1;
			} else {
			*inptr++ = c;
		}
	}
	RED_OFF();
	return 0;
}


void doCommandProcessing(void);

int main(void)
{
	int correctionA, correctionB;

 	HERCULES_PCBA_PINS_INIT();
 	usart_init(57600);
 	stdout = &mystdout;
	eeprom_busy_wait();
	analogReference(0);
	pwm_init();  // init
	setPwm(9, 0, PWM_FREQ);        // pin: 9,  duty: 20%, freq: 5kHz
	setPwm(10, 0, PWM_FREQ);      // pin: 10, duty: 80%, freq: 5kHz
	IR210_ON();
	TIMSK1 |= (1<<TOIE1);		// using PWM module as timer interrupt source
	EICRA = 0b1111;
	EIMSK = 0b11;
	sei();

	doResetPID();
	//#define SAVE_DEFAULT_PID_EEPROM
	#ifdef SAVE_DEFAULT_PID_EEPROM
	doSavePID();
	#endif
	#ifdef RECALL_SAVED_PID_EEPROM
	doRecallPID();
	#endif

 	printf_P(PSTR("\n\nCPP menu test application 0.02\n"));

 	/* Replace with your application code */
 	while (1) {
		 
		if (readline_nb()) {
			doCommandProcessing();
		}

		if (odoUpdated) {

			if (adc_en) {
				motorA_setting = analogRead(PIN_A0) - 512;
				motorB_setting = analogRead(PIN_A1) - 512;
			}

			if (pid_en) {
				if (motorA_setting == 0) {
					motorA_setPWM(0);
					pid_Reset_Integrator(&motorA_PidData);
				}
				else {
					correctionA = pid_Controller(motorA_setting, motorA_speed, &motorA_PidData);
					motorA_setPWM(correctionA);
				}

				if (motorB_setting == 0) {
					motorB_setPWM(0);
					pid_Reset_Integrator(&motorA_PidData);
				}
				else {
					correctionB = pid_Controller(motorB_setting, motorB_speed, &motorB_PidData);
					motorB_setPWM(correctionB);
				}
			}
			else {
				// Set raw motor settings to PWM
				motorA_setPWM(motorA_setting / 5);
				motorB_setPWM(motorB_setting / 5);
			}
			if (odo_en)
				//printf("odo: %-3d %-3d\n", motorA_speed, motorB_speed);
				printf("odo: %4d %4d %4d %4d %4d %4d\n", motorA_setting, motorA_speed, correctionA,
				 motorB_setting, motorB_speed, correctionB);

			odoUpdated = false;
		}

		
 	}

}


void doSavePID(void)
{
	// Saves the current PID into EEPROM
	eeprom_busy_wait();
	eeprom_write_word((uint16_t *) 2, (motorA_PidData.P_Factor));
	eeprom_busy_wait();
	eeprom_write_word((uint16_t *) 4, (motorA_PidData.I_Factor));
	eeprom_busy_wait();
	eeprom_write_word((uint16_t *) 6, (motorA_PidData.D_Factor));
	eeprom_busy_wait();
	eeprom_write_word((uint16_t *) 8, (motorB_PidData.P_Factor));
	eeprom_busy_wait();
	eeprom_write_word((uint16_t *)10, (motorB_PidData.I_Factor));
	eeprom_busy_wait();
	eeprom_write_word((uint16_t *)12, (motorB_PidData.D_Factor));
}

void doRecallPID(struct PID_DATA *motorA_PidData, struct PID_DATA *motorB_PidData)
{
	int scaled_P, scaled_I, scaled_D;
	eeprom_busy_wait();
	scaled_P = eeprom_read_word((uint16_t *)2);
	scaled_I = eeprom_read_word((uint16_t *)4);
	scaled_D = eeprom_read_word((uint16_t *)6);
	pid_Init(scaled_P, scaled_I, scaled_D, motorA_PidData);
	scaled_P = eeprom_read_word((uint16_t *)8);
	scaled_I = eeprom_read_word((uint16_t *)10);
	scaled_D = eeprom_read_word((uint16_t *)12);
	pid_Init(scaled_P, scaled_I, scaled_D, motorB_PidData);
}

bool isParamOnOff(char *p)
{
	if (strcasecmp(p, "on") == 0)
		return 1;
	else if (strcasecmp(p, "off") == 0)
		return 0;
	return -1;
}

void doCommandProcessing(void)
{
	char *ptab[16],*p, *p0;
	int j, i = 0;
	int onoff;
	float fp, fi, fd;
	long volts;
	
	printf("The command was: '%s'\n", inbuf);
	p = strtok(inbuf, " ,");
	do {
		ptab[i++] = p;
		p = strtok(NULL, " ,");
	} while (p && (i < 16));
	for (j = 0; j < i; j++)
		printf("%s,", ptab[j]);
	printf("\n");
	
	p0 = ptab[0];
	if (strcasecmp(p0, "odo") == 0) {
		onoff = isParamOnOff(ptab[1]);
		if (onoff >= 0) {
			odo_en = onoff;
			printf("ODO %s\n", onoff ? "On" : "Off");
		}
	}
	else if (strcasecmp(p0, "adc") == 0) {
		onoff = isParamOnOff(ptab[1]);
		if (onoff >= 0) {
			adc_en = onoff;
			if (onoff == 0) {
				motorA_setting = motorB_setting = 0;
			}
			printf("ADC %s\n", onoff ? "On" : "Off");
		}
	}
	else if (strcasecmp(p0, "mot") == 0) {
		motorA_setting = atoi(ptab[1]);
		motorB_setting = atoi(ptab[2]);
	}
	else if (strcasecmp(p0, "getm") == 0) {
		printf("motor: %d,%d\n", motorA_setting, motorB_setting);
	}
	else if (strcasecmp(p0, "pid") == 0) {
		onoff = isParamOnOff(ptab[1]);
		if (onoff >= 0) {
			pid_en = onoff;
			printf("PID %s\n", onoff ? "On" : "Off");
		}
	}
	else if (strcasecmp(p0, "getp") == 0) {
		printf("pid: %f,%f,%f, %f,%f,%f\n",
			(float)motorA_PidData.P_Factor / SCALING_FACTOR,
			(float)motorA_PidData.I_Factor / SCALING_FACTOR,
			(float)motorA_PidData.D_Factor / SCALING_FACTOR,
			(float)motorB_PidData.P_Factor / SCALING_FACTOR,
			(float)motorB_PidData.I_Factor / SCALING_FACTOR,
			(float)motorB_PidData.D_Factor / SCALING_FACTOR);
	}
	else if (strcasecmp(p0, "setp") == 0) {
		printf("Trying new PID params\n");
		fp = atof(ptab[1]);
		fi = atof(ptab[2]);
		fd = atof(ptab[3]);
		printf("New PID = %f, %f, %f\n", fp, fi, fd);
		pid_Init(fp * SCALING_FACTOR, fi * SCALING_FACTOR , fd * SCALING_FACTOR , &motorA_PidData);
		pid_Init(fp * SCALING_FACTOR, fi * SCALING_FACTOR , fd * SCALING_FACTOR , &motorB_PidData);
	}
	else if (strcasecmp(p0, "savep") == 0) {
		doSavePID();
	}
	else if (strcasecmp(p0, "recallp") == 0) {
		doRecallPID(&motorA_PidData, &motorB_PidData);
	}
	else if (strcasecmp(p0, "getb") == 0) {
		volts = (long) analogRead(PIN_A6) * 1000 / 384;;
		printf("Vbatt:%d\n", (int)volts);
	}
}
