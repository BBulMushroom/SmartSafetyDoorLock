#include <avr/io.h>
#include <myatmega128.h>
#include <stdint.h>

/*
 * USART0
 */
#if USART0_ACTIVATE
void init_USART0()
{
	unsigned int ubrr = F_CPU/16/BAUD-1;
	UBRR0H = (unsigned char) (ubrr >> 8);	// Baud rate : 9600bps
	UBRR0L = (unsigned char) ubrr;
	UCSR0C |= (1<<UCSZ00)|(1<<UCSZ01);	// Character size : 8bit
	UCSR0C &= ~(1<<USBS0);	//stop  bit : 1비트
	UCSR0C &= ~((1<<UPM01)|(1<<UPM00));	// no parity mode
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);	// Rx, Tx enable
	UCSR0B |= (1 << RXCIE0);
}
void USART0_Tx(unsigned char data)
{
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}
void USART0_Tx_String(char *str)
{
	while(*str != '\0')
	USART0_Tx(*str++);
}
char USART0_Rx()
{
	while(!(UCSR0A&(1<<RXC0)));	// 수신 되기를 기다림
	return UDR0;
}
void USART0_Rx_String(char str[], uint8_t maxLength)
{
	UCSR0B &= ~(1 << RXCIE0);
	char response;
	uint8_t i;
	i = 0;
	while (i < (maxLength - 1)) {                   /* prevent over-runs */
		response = USART0_Rx();
		if (response == '\r' || response == '\n') {                     /* enter marks the end */
			break;
		}
		else {
			str[i] = response;                       /* add in a letter */
			i++;
		}
	}
	str[i] = 0;                          /* terminal NULL character */
	UCSR0B |= (1 << RXCIE0);
}
#endif

/*
 * USART1
 */
#if USART1_ACTIVATE
void init_USART1()
{
	unsigned int ubrr = F_CPU/16/BAUD-1;
	UBRR1H = (unsigned char) (ubrr >> 8);	// Baud rate : 9600bps
	UBRR1L = (unsigned char) ubrr;
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);	// Rx, Tx enable
	UCSR1B |= (1 << RXCIE1) | (1 < TXCIE1);	// Rx interrupt enable
	UCSR1C |= (1<<UCSZ10)|(1<<UCSZ11);	// Character size : 8bit
}
void USART1_Tx(unsigned char data)
{
	while (!(UCSR1A & (1 << UDRE1)));
	UDR1 = data;
}
void USART1_Tx_String(char *str)
{
	while(*str != '\0')
	USART1_Tx(*str++);
}
#endif

/*
 * ADC
 */
#if ADC_ACTIVATE
void init_ADC()	// adc 초기화
{
	ADCSRA = (0x07 << ADPS0);    //ADC를 사용함, 128의 분주비사용.
}
int readADC(int channel)	// adc 읽기
{
	ADMUX = channel;
	int output;
	ADCSRA |= (1<<ADSC) | (1<<ADEN);	// start single conversion
	while(!(ADCSRA & (1 << ADIF)));    // ADC변환이 완료될때까지 대기함.
	output = ADCL + (ADCH << 8);
	ADCSRA |= (1 << ADIF);    // 인터럽트 플래그를 클리어함.
	ADCSRA &= ~(1<<ADEN);	// disable
	return output;
}
#endif

/*
 * TIMER2
 */
#if TIMER2_ACTIVATE
void init_Timer2()
{
	TCCR2 = (0<<CS02)|(1<<CS01)|(1<<CS00);	// 8bit 64 prescaler 설정
	TCNT2 = 6;								// (1/16M)*1024*250=16ms
	TIMSK |= (1<<TOIE2);
}

#endif