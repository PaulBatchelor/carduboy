#ifndef PINS_H
#define PINS_H

#define TX_RX_LED_INIT DDRD |= (1<<5), DDRB |= (1<<0)
#define TXLED0 PORTD |= (1<<5)
#define TXLED1 PORTD &= ~(1<<5)
#define RXLED0 PORTB |= (1<<0)
#define RXLED1 PORTB &= ~(1<<0)

#define PIN_WIRE_SDA (2)
#define PIN_WIRE_SCL (3)

#define PIN_SPI_SS (17)
#define PIN_SPI_MOSI (16)
#define PIN_SPI_MISO (14)
#define PIN_SPI_SCK (15)

#define PIN_A0 (18)
#define PIN_A1 (19)
#define PIN_A2 (20)
#define PIN_A3 (21)
#define PIN_A4 (22)
#define PIN_A5 (23)
#define PIN_A6 (24)
#define PIN_A7 (25)
#define PIN_A8 (26)
#define PIN_A9 (27)
#define PIN_A10 (28)
#define PIN_A11 (29)
#endif
