/*
 * hercules.h
 *
 * Created: 2017-02-14 12:00:41 PM
 *  Author: Adnan
 *
 * SeeedStudio Hercules 4WD Robot Controller PCBA
 * ATmega328p with 16MHz crystal, with Arduino-style bootloader
 */ 

#ifndef HERCULES_H_
#define HERCULES_H_

/*
 * PD7: D7 / CTR2A output
 * PD6: D6 / IR210SD output, low=shutdown
 * PD5: D5 / CTR1B output
 * PD4: D4 / CTR1A output
 * PD3: D3 / INT1 expansion input
 * PD2: D2 / INT0 expansion input
 * PD1: M-TXD output
 * PD0: M-RXD input
 *
 * PC6: Used for RESET#
 * PC5: SCL0 master OC, set as input
 * PC4: SDA0 slave OC, set as input
 * PC3: A3 / quad exp input (with D3)
 * PC2: A2 / quad exp input (with D2)
 * PC1: A1 / ADC1 exp input
 * PC0: A0 / ADC0 exp input
 *
 * PB7: Used for XTAL2
 * PB6: Used for XTAL1
 * PB5: D13 / SCK / red LED output
 * PB4: D12 / MISO input
 * PB3: D11 / MOSI output
 * PB2: D10 / PWM2 output
 * PB1:  D9 / PWM1 output
 * PB0:  D8 / CTR2B output
 *
 * ADC6: Battery ADC input (430||100 kohms)
 * ADC7: Unconnected ADC input
 */

#define HERCULES_PCBA_PINS_INIT() { \
						DDRD =  0b11110010; \
						PORTD = 0b00001111; \
						DDRC =  0b00000000; \
						PORTC = 0b00111100; \
						DDRB =  0b00101111; \
						PORTB = 0b00000000; }

#define RED_ON()			(PORTB |=  (1<<PORTB5))
#define RED_OFF()			(PORTB &= ~(1<<PORTB5))
#define IR210_ON()			(PORTD |=  (1<<PORTD6))
#define IR210_OFF()			(PORTD &= ~(1<<PORTD6))
/* Ability to change SPI-SDcard to input (hi-Z) for safe insertion/removal */
#define SDSPI_OFF()			(DDRB &= ~( (1<<PB3) | (1<<PB5) | (1<<PB2) ) )
#define SDSPI_ON()			(DDRB |=  ( (1<<PB3) | (1<<PB5) | (1<<PB2) ) )

#define D11_ON()			(PORTB |=  (1<<PORTB3))
#define D11_OFF()			(PORTB &= ~(1<<PORTB3))

#endif /* HERCULES_H_ */