#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "arduboy.h"

#define clockCyclesPerMicrosecond() (F_CPU / 1000000L)
#define clockCyclesToMicroseconds(a) ((a) / clockCyclesPerMicrosecond() )

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;
static void idle();
static uint8_t buf[1024];

void arduboy_setframerate(ArduboyC *ab, uint8_t rate)
{
    ab->eachFrameMillis = 1000 / rate;
}

int arduboy_nextframe(ArduboyC *ab)
{
    uint8_t now = (uint8_t) millis();
    uint8_t frameDurationMs = now - ab->thisFrameStart;

    if (ab->justRendered) {
        ab->lastFrameDurationMs = frameDurationMs;
        ab->justRendered = false;
        return 0;
    } else if (frameDurationMs < ab->eachFrameMillis) {
        if (++frameDurationMs < ab->eachFrameMillis) {
            idle();
        }
        return 0;
    }

    ab->justRendered = true;
    ab->thisFrameStart = now;

    return 1;
}

static void draw_pixel(uint8_t *sBuffer, int16_t x, int16_t y, uint8_t color)
{
    uint16_t row_offset;
    uint8_t bit;

    __asm__ volatile (
        /* bit = 1; */
        "ldi  %[bit], 1\n"
        /* if (y & _BV(1)) bit = 4; */
        "sbrc %[y], 1\n"
        "ldi  %[bit], 4\n"
        /* if (y & _BV(0)) bit = bit << 1; */
        "sbrc %[y], 0\n"
        "lsl  %[bit]\n"

        /* if (y & _BV(2)) bit = (bit << 4) | (bit >> 4); */
        "sbrc %[y], 2\n"
        "swap %[bit]\n"

        /* row_offset = (y & 0xF8) * WIDTH / 8 */
        "andi %A[y], 0xf8\n"
        "mul  %[width_offset], %A[y]\n"
        "movw %[row_offset], r0\n"
        "clr  __zero_reg__\n"

        /* row_offset += x */
        "add  %A[row_offset], %[x]\n"
        : [row_offset]   "=&x" (row_offset),   // upper register (ANDI)
          [bit]          "=&d" (bit),          // upper register (LDI)
          [y]            "+d"  (y)             // upper register (ANDI), must be writable
        : [width_offset] "r"   ((uint8_t)(WIDTH/8)),
          [x]            "r"   ((uint8_t)x)
        :
        );

    uint8_t data = sBuffer[row_offset] | bit;

    if (!(color & _BV(0))) data ^= bit;
    sBuffer[row_offset] = data;
}

static void draw_vline(uint8_t *sBuffer,
                       int16_t x, int16_t y, uint8_t h,
                       uint8_t color);

static void draw_hline(uint8_t *sBuffer,
                       int16_t x, int16_t y,
                       uint8_t w, uint8_t color);


static void draw_rect(uint8_t *sBuffer,
                      int16_t x, int16_t y,
                      uint8_t w, uint8_t h,
                      uint8_t color)
{

    draw_hline(sBuffer, x, y, w, color);
    draw_hline(sBuffer, x, y+h-1, w, color);
    draw_vline(sBuffer, x, y, h, color);
    draw_vline(sBuffer, x+w-1, y, h, color);
}

void arduboy_draw_rect(ArduboyC *ab,
                       int16_t x, int16_t y,
                       uint8_t w, uint8_t h,
                       uint8_t color)
{
    draw_rect(ab->sBuffer, x, y, w, h, color);
}

static void draw_pixel(uint8_t *sBuffer, int16_t x, int16_t y, uint8_t color);

static void draw_vline(uint8_t *sBuffer,
                       int16_t x, int16_t y, uint8_t h,
                       uint8_t color)
{
    int end = y+h;
    for (int a = max(0,y); a < min(end,HEIGHT); a++) {
        draw_pixel(sBuffer, x,a,color);
    }
}

static void draw_hline(uint8_t *sBuffer,
                       int16_t x, int16_t y,
                       uint8_t w, uint8_t color)
{
    int16_t xEnd;

    if (y < 0 || y >= HEIGHT) return;

    xEnd = x + w;

    if (xEnd <= 0 || x >= WIDTH) return;

    if (x < 0) x = 0;

    if (xEnd > WIDTH) xEnd = WIDTH;

    w = xEnd - x;

    register uint8_t *pBuf = sBuffer + ((y / 8) * WIDTH) + x;
    register uint8_t mask = 1 << (y & 7);

    switch (color) {
        case WHITE:
            while (w--) *pBuf++ |= mask;
            break;

        case BLACK:
            mask = ~mask;
            while (w--) *pBuf++ &= mask;
            break;
    }
}

const uint8_t PROGMEM lcdBootProgram[] = {
    /* Set Display Clock Divisor v = 0xF0 */
    /* default is 0x80 */
    0xD5, 0xF0,

    /* Charge Pump Setting v = enable (0x14) */
    /* default is disabled */
    0x8D, 0x14,

    /* Set Segment Re-map (A0) | (b0001) */
    /* default is (b0000) */
    0xA1,

    /* Set COM Output Scan Direction */
    0xC8,

    /* Set Contrast v = 0xCF */
    0x81, 0xCF,

    /* Set Precharge = 0xF1 */
    0xD9, 0xF1,

    /* Display On */
    0xAF,

    /* set display mode = horizontal addressing mode (0x00) */
    0x20, 0x00,
};

static void boot_pins(void)
{
    /* Port B INPUT_PULLUP or HIGH */
    PORTB |= _BV(RED_LED_BIT) | _BV(GREEN_LED_BIT) | _BV(BLUE_LED_BIT) |
        _BV(B_BUTTON_BIT);

    /* Port B INPUT or LOW (none) */
    /* Port B inputs */
    DDRB &= ~(_BV(B_BUTTON_BIT) | _BV(SPI_MISO_BIT));
    // Port B outputs
    DDRB |= _BV(RED_LED_BIT) | _BV(GREEN_LED_BIT) | _BV(BLUE_LED_BIT) |
        _BV(SPI_MOSI_BIT) | _BV(SPI_SCK_BIT) | _BV(SPI_SS_BIT);

    /* Port D INPUT_PULLUP or HIGH */
    PORTD |= _BV(CS_BIT);
    /* Port D INPUT or LOW */
    PORTD &= ~(_BV(RST_BIT));
    /* Port D inputs (none) */
    // Port D outputs
    DDRD |= _BV(RST_BIT) | _BV(CS_BIT) | _BV(DC_BIT);

    /* Port E INPUT_PULLUP or HIGH */
    PORTE |= _BV(A_BUTTON_BIT);
    /* Port E INPUT or LOW (none) */
    /* Port E inputs */
    DDRE &= ~(_BV(A_BUTTON_BIT));
    /* Port E outputs (none) */

    // Port F INPUT_PULLUP or HIGH
    PORTF |= _BV(LEFT_BUTTON_BIT) | _BV(RIGHT_BUTTON_BIT) |
        _BV(UP_BUTTON_BIT) | _BV(DOWN_BUTTON_BIT);
    /* Port F INPUT or LOW */
    PORTF &= ~(_BV(RAND_SEED_IN_BIT));
    /* Port F inputs */
    DDRF &= ~(_BV(LEFT_BUTTON_BIT) | _BV(RIGHT_BUTTON_BIT) |
              _BV(UP_BUTTON_BIT) | _BV(DOWN_BUTTON_BIT) |
              _BV(RAND_SEED_IN_BIT));
}

static void boot_spi(void)
{
    /* master, mode 0, MSB first, CPU clock / 2 (8MHz) */
    SPCR = _BV(SPE) | _BV(MSTR);
    SPSR = _BV(SPI2X);
}


static void lcd_data_mode(void)
{
    bitSet(DC_PORT, DC_BIT);
}

static void lcd_command_mode(void)
{
    bitClear(DC_PORT, DC_BIT);
}

// delay in ms with 16 bit duration
static void delay_short(uint16_t ms)
{
    delay((unsigned long) ms);
}



/*
 * The following NOP introduces a small delay that can prevent the wait
 * loop form iterating when running at the maximum speed. This gives
 * about 10% more speed, even if it seems counter-intuitive. At lower
 * speeds it is unnoticed.
 */
static void spi_transfer(uint8_t data)
{
    SPDR = data;
    __asm__ volatile("nop");
    while (!(SPSR & _BV(SPIF))) { } // wait
}

void boot_oled(void)
{

    /* reset pin should be low here. let it stay low a while */
    delay_short(5);
    bitSet(RST_PORT, RST_BIT); // set high to come out of reset
    delay_short(5); // wait a while

    /* select the display (permanently, since nothing else is using SPI) */
    bitClear(CS_PORT, CS_BIT);

    /* run our customized boot-up command sequence against the */
    /* OLED to initialize it properly for Arduboy */
    lcd_command_mode();
    for (uint8_t i = 0; i < sizeof(lcdBootProgram); i++) {
        spi_transfer(pgm_read_byte(lcdBootProgram + i));
    }
    lcd_data_mode();
}

static void boot_powersaving(void)
{
    /* disable Two Wire Interface (I2C) and the ADC */
    /* All other bits will be written with 0 so will be enabled */
    PRR0 = _BV(PRTWI) | _BV(PRADC);
    /* disable USART1 */
    PRR1 |= _BV(PRUSART1);
}

static void bootup(void)
{
    ADMUX = RAND_SEED_IN_ADMUX;

    boot_pins();
    boot_spi();
    boot_oled();
    boot_powersaving();
}

static void idle()
{
    /* select idle mode and enable sleeping */
    SMCR = _BV(SE);
    sleep_cpu();
    /* disable sleeping */
    SMCR = 0;
}

uint8_t arduboy_buttons_state(void)
{
    uint8_t buttons;

    buttons = ((~PINF) &
               (_BV(UP_BUTTON_BIT) |
                _BV(RIGHT_BUTTON_BIT) |
                _BV(LEFT_BUTTON_BIT) |
                _BV(DOWN_BUTTON_BIT)));

    if (bitRead(A_BUTTON_PORTIN, A_BUTTON_BIT) == 0) {
        buttons |= A_BUTTON;
    }

    if (bitRead(B_BUTTON_PORTIN, B_BUTTON_BIT) == 0) {
        buttons |= B_BUTTON;
    }

    return buttons;
}

void arduboy_display(ArduboyC *ab, bool clear)
{
  uint16_t count;
  uint8_t *image;

  image = ab->sBuffer;

  __asm__ volatile (
    "   ldi   %A[count], %[len_lsb]\n" //for (len = WIDTH * HEIGHT / 8)
    "   ldi   %B[count], %[len_msb]\n"
    "1: ld    __tmp_reg__, %a[ptr] ;2\n" //tmp = *(image)
    "   out   %[spdr], __tmp_reg__ ;1\n" //SPDR = tmp
    "   cpse  %[clear], __zero_reg__ ;1/2\n" //if (clear) tmp = 0;
    "   mov   __tmp_reg__, __zero_reg__ ;1\n"
    "2: sbiw  %A[count], 1 ;2\n\t" //len --
    "   sbrc  %A[count], 0 ;1/2\n\t" //loop twice for cheap delay
    "   rjmp  2b ;2\n\t"
    "   st    %a[ptr]+, __tmp_reg__ ;2\n" //*(image++) = tmp
    "   brne  1b ;1/2 :18\n\t" //len > 0
    "   in    __tmp_reg__, %[spsr]\n" //read SPSR to clear SPIF
    : [ptr]     "+&e" (image),
      [count]   "=&w" (count)
    : [spdr]    "I"   (_SFR_IO_ADDR(SPDR)),
      [spsr]    "I"   (_SFR_IO_ADDR(SPSR)),
      [len_msb] "M"   (WIDTH * (HEIGHT / 8 * 2) >> 8),   // 8: pixels per byte
      [len_lsb] "M"   (WIDTH * (HEIGHT / 8 * 2) & 0xFF), // 2: for delay loop multiplier
      [clear]   "r"   (clear)
  );
}

/* C version: */
/* if (color != BLACK) */
/* { */
/*   color = 0xFF; // all pixels on */
/* } */
/* for (int16_t i = 0; i < WIDTH * HEIGTH / 8; i++) */
/* { */
/*    sBuffer[i] = color; */
/* } */

/* This asm version is hard coded for 1024 bytes. It doesn't use the defined */
/* WIDTH and HEIGHT values. It will have to be modified for a different */
/* screen buffer size. */
/* It also assumes color value for BLACK is 0. */

/* local variable for screen buffer pointer, */
/* which can be declared a read-write operand */

static void fillscreen(uint8_t color, uint8_t *sBuffer)
{
    uint8_t* bPtr = sBuffer;

    __asm__ volatile (
        /* if value is zero, skip assigning to 0xff */
        "cpse %[color], __zero_reg__\n"
        "ldi %[color], 0xFF\n"
        /* counter = 0 */
        "clr __tmp_reg__\n"
        "loopto:\n"
        /* (4x) push zero into screen buffer, */
        /* then increment buffer position */
        "st Z+, %[color]\n"
        "st Z+, %[color]\n"
        "st Z+, %[color]\n"
        "st Z+, %[color]\n"
        /* increase counter */
        "inc __tmp_reg__\n"
        /* repeat for 256 loops */
        /* (until counter rolls over back to 0) */
        "brne loopto\n"
        : [color] "+d" (color),
          "+z" (bPtr)
        :
        :
    );
}

void arduboy_clear(ArduboyC *ab)
{
    fillscreen(BLACK, ab->sBuffer);
}

void arduboy_begin(ArduboyC *ab)
{
    bootup();
    arduboy_display(ab, false);
}

void arduboy_init(ArduboyC *ab)
{
    ab->justRendered = false;
    ab->sBuffer = buf;
}

ISR(TIMER0_OVF_vect)
{
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

unsigned long millis()
{
	unsigned long m;
	uint8_t oldSREG = SREG;

	cli();
	m = timer0_millis;
	SREG = oldSREG;

	return m;
}

unsigned long micros() {
	unsigned long m;
	uint8_t oldSREG = SREG, t;

	cli();
	m = timer0_overflow_count;
	t = TCNT0;

	if ((TIFR0 & _BV(TOV0)) && (t < 255))
		m++;

	SREG = oldSREG;

	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

static void __empty() {
}

void yield(void) __attribute__ ((weak, alias("__empty")));

void delay(unsigned long ms)
{
	uint32_t start = micros();

	while (ms > 0) {
		yield();
		while ( ms > 0 && (micros() - start) >= 1000) {
			ms--;
			start += 1000;
		}
	}
}

void init()
{
	sei();

	sbi(TCCR0A, WGM01);
	sbi(TCCR0A, WGM00);

	sbi(TCCR0B, CS01);
	sbi(TCCR0B, CS00);

	sbi(TIMSK0, TOIE0);


	TCCR1B = 0;

	sbi(TCCR1B, CS11);
	sbi(TCCR1B, CS10);

	sbi(TCCR1A, WGM10);

	sbi(TCCR3B, CS31);
	sbi(TCCR3B, CS30);
	sbi(TCCR3A, WGM30);

	sbi(TCCR4B, CS42);
	sbi(TCCR4B, CS41);
	sbi(TCCR4B, CS40);
	sbi(TCCR4D, WGM40);
	sbi(TCCR4A, PWM4A);
	sbi(TCCR4C, PWM4D);


    sbi(ADCSRA, ADPS2);
    sbi(ADCSRA, ADPS1);
    sbi(ADCSRA, ADPS0);
	sbi(ADCSRA, ADEN);
}
