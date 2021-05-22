#ifndef SPI_H
#define SPI_H

#include <avr/io.h>

void init_SPI();
uint8_t spi_transmit(uint8_t data);

#define ENABLE_CHIP() (PORTB &= (~(1<<PB0)))
#define DISABLE_CHIP() (PORTB |= (1<<PB0))

#endif