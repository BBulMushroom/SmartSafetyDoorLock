#include <spi.h>

void init_SPI()
{
	DDRB |= (1<<PB2)|(1<<PB1)|(1<<PB0);
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);//prescaler 16
}

uint8_t spi_transmit(uint8_t data)
{
	SPDR = data;
	while(!(SPSR & (1<<SPIF)));
	
	return SPDR;
}