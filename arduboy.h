#ifndef ARDUBOY2_H
#define ARDUBOY2_H

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define HIGH 0x1
#define LOW  0x0

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

unsigned long millis(void);
void delay(unsigned long);

#include "pins.h"

#define RGB_ON LOW
#define RGB_OFF HIGH

#define PIN_CS 12
#define CS_PORT PORTD
#define CS_BIT PORTD6

#define PIN_DC 4
#define DC_PORT PORTD
#define DC_BIT PORTD4

#define PIN_RST 6
#define RST_PORT PORTD
#define RST_BIT PORTD7

#define RED_LED 10
#define GREEN_LED 11
#define BLUE_LED 9

#define RED_LED_PORT PORTB
#define RED_LED_BIT PORTB6

#define GREEN_LED_PORT PORTB
#define GREEN_LED_BIT PORTB7

#define BLUE_LED_PORT PORTB
#define BLUE_LED_BIT PORTB5

/* bit values for button states */
#define LEFT_BUTTON _BV(5)
#define RIGHT_BUTTON _BV(6)
#define UP_BUTTON _BV(7)
#define DOWN_BUTTON _BV(4)
#define A_BUTTON _BV(3)
#define B_BUTTON _BV(2)

#define PIN_LEFT_BUTTON A2
#define LEFT_BUTTON_PORT PORTF
#define LEFT_BUTTON_PORTIN PINF
#define LEFT_BUTTON_DDR DDRF
#define LEFT_BUTTON_BIT PORTF5

#define PIN_RIGHT_BUTTON A1
#define RIGHT_BUTTON_PORT PORTF
#define RIGHT_BUTTON_PORTIN PINF
#define RIGHT_BUTTON_DDR DDRF
#define RIGHT_BUTTON_BIT PORTF6

#define PIN_UP_BUTTON A0
#define UP_BUTTON_PORT PORTF
#define UP_BUTTON_PORTIN PINF
#define UP_BUTTON_DDR DDRF
#define UP_BUTTON_BIT PORTF7

#define PIN_DOWN_BUTTON A3
#define DOWN_BUTTON_PORT PORTF
#define DOWN_BUTTON_PORTIN PINF
#define DOWN_BUTTON_DDR DDRF
#define DOWN_BUTTON_BIT PORTF4

#define PIN_A_BUTTON 7
#define A_BUTTON_PORT PORTE
#define A_BUTTON_PORTIN PINE
#define A_BUTTON_DDR DDRE
#define A_BUTTON_BIT PORTE6

#define PIN_B_BUTTON 8
#define B_BUTTON_PORT PORTB
#define B_BUTTON_PORTIN PINB
#define B_BUTTON_DDR DDRB
#define B_BUTTON_BIT PORTB4

#define PIN_SPEAKER_1 5
#define PIN_SPEAKER_2 13

#define SPEAKER_1_PORT PORTC
#define SPEAKER_1_DDR DDRC
#define SPEAKER_1_BIT PORTC6

#define SPEAKER_2_PORT PORTC
#define SPEAKER_2_DDR DDRC
#define SPEAKER_2_BIT PORTC7

#define RAND_SEED_IN A4
#define RAND_SEED_IN_PORT PORTF
#define RAND_SEED_IN_BIT PORTF1
#define RAND_SEED_IN_ADMUX (_BV(REFS0) | _BV(REFS1) | _BV(MUX0))

/* SPI interface */
#define SPI_MISO_PORT PORTB
#define SPI_MISO_BIT PORTB3

#define SPI_MOSI_PORT PORTB
#define SPI_MOSI_BIT PORTB2

#define SPI_SCK_PORT PORTB
#define SPI_SCK_BIT PORTB1

#define SPI_SS_PORT PORTB
#define SPI_SS_BIT PORTB0

#define OLED_PIXELS_INVERTED 0xA7
#define OLED_PIXELS_NORMAL 0xA6

#define OLED_ALL_PIXELS_ON 0xA5
#define OLED_PIXELS_FROM_RAM 0xA4

#define OLED_VERTICAL_FLIPPED 0xC0
#define OLED_VERTICAL_NORMAL 0xC8

#define OLED_HORIZ_FLIPPED 0xA0
#define OLED_HORIZ_NORMAL 0xA1

#define WIDTH 128
#define HEIGHT 64

#define COLUMN_ADDRESS_END (WIDTH - 1) & 127
#define PAGE_ADDRESS_END ((HEIGHT/8)-1) & 7

#define ARDUBOY_LIB_VER 50200

#define ARDUBOY_UNIT_NAME_LEN 6

#define EEPROM_VERSION 0
#define EEPROM_SYS_FLAGS 1
#define EEPROM_AUDIO_ON_OFF 2
#define EEPROM_UNIT_ID 8
#define EEPROM_UNIT_NAME 10
#define SYS_FLAG_UNAME 0
#define SYS_FLAG_UNAME_MASK _BV(SYS_FLAG_UNAME)
#define SYS_FLAG_SHOW_LOGO 1
#define SYS_FLAG_SHOW_LOGO_MASK _BV(SYS_FLAG_SHOW_LOGO)
#define SYS_FLAG_SHOW_LOGO_LEDS 2
#define SYS_FLAG_SHOW_LOGO_LEDS_MASK _BV(SYS_FLAG_SHOW_LOGO_LEDS)

#define EEPROM_STORAGE_SPACE_START 16

#define PIXEL_SAFE_MODE

#define BLACK 0
#define WHITE 1
#define INVERT 2

#define CLEAR_BUFFER true

typedef struct ArduboyC ArduboyC;

struct ArduboyC {
  uint8_t eachFrameMillis;
  uint8_t thisFrameStart;
  bool justRendered;
  uint8_t lastFrameDurationMs;
  uint8_t *sBuffer;
};

void arduboy_init(ArduboyC *ab);

uint8_t arduboy_buttons_state(void);

void arduboy_display(ArduboyC *ab, bool clear);

void arduboy_draw_rect(ArduboyC *ab,
                       int16_t x, int16_t y,
                       uint8_t w, uint8_t h,
                       uint8_t color);

void arduboy_clear(ArduboyC *ab);

void arduboy_begin(ArduboyC *ab);

int arduboy_nextframe(ArduboyC *ab);

void arduboy_setframerate(ArduboyC *ab, uint8_t rate);

#endif
