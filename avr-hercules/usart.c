/*
 * usart.c
 *
 * Created: 2016-02-09 10:49:39 AM
 *  Author: adnan
 */ 

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <stdio.h>

void usart_init(unsigned long baud)
{
	/* Set baud rate */
	unsigned long ubrr = (F_CPU / 16 / baud - 1);
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)(ubrr>>0);
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: Async,8,N,1 */
	UCSR0C = 0b00000110;
}

void usart_off(void)
{
	UCSR0B = 0;
}

char usart_havechar(void)
{
	return (UCSR0A & (1<<RXC0));
}

char usart_getc(void)
{
	while (!usart_havechar())
		;
	return UDR0;
}

void usart_putc(char c)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
}

void usart_puts(char *s)
{
	while(*s)
		usart_putc(*s++);
}

int usart_putchar(char ch, FILE *stream) {
	// translate \n to \r for br@y++ terminal
	if (ch == '\n')
		usart_putc('\r');
	usart_putc(ch);
	return 0;
}