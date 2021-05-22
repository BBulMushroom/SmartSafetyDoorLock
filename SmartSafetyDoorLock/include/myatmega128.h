#ifndef MYATMEGA128_H_
#define MYATMEGA128_H_

#ifndef BAUD
#define BAUD 9600
#endif

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/*
 * Set to 1, activate
 */
#define USART0_ACTIVATE 1
#define USART1_ACTIVATE 0
#define SPI_ACTIVATE 1
#define ADC_ACTIVATE 1
#define TIMER2_ACTIVATE 1

#if USART0_ACTIVATE
void init_USART0();
void USART0_Tx(unsigned char data);
void USART0_Tx_String(char *str);
char USART0_Rx();
void USART0_Rx_String(char str[], uint8_t maxLength);
#endif

#if USART1_ACTIVATE
void init_USART1();
void USART1_Tx(unsigned char data);
void USART1_Tx_String(char *str);
#endif

#if ADC_ACTIVATE
void init_ADC();
int readADC(int channel);
#endif

#if TIMER2_ACTIVATE
void init_Timer2();
#endif

#endif /* MYATMEGA128_H_ */