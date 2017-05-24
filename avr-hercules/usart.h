/*
 * usart.h
 *
 * Created: 2016-02-09 10:57:50 AM
 *  Author: adnan
 */ 


#ifndef USART_H_
#define USART_H_

void usart_init(unsigned long baud);
void usart_off(void);
char usart_havechar(void);
char usart_getc(void);
void usart_putc(char c);
void usart_puts(char *s);
int usart_putchar(char ch, FILE *stream);

#endif /* USART_H_ */