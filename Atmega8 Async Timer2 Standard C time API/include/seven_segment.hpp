#pragma once
#include <stdint.h>

#define DOT_0 1
#define DOT_1 2
#define DOT_2 4
#define DOT_3 8

#define DIGIT_0 1
#define DIGIT_1 2
#define DIGIT_2 4
#define DIGIT_3 8

#define ALL_DIGITS 0x00
#define NO_DIGITS 0x0F
#define HOUR_DIGITS 0x0C
#define MINUTE_DIGITS 0x03


typedef struct SevSeg{
    volatile uint8_t values[4]{0};
    volatile uint8_t enabled_digits{0};
    volatile uint8_t enabled_dots{0};
    volatile uint8_t hours_blink{0};
    volatile uint8_t minutes_blink{0};
    volatile uint8_t digit_counter{0};
}SevSeg;

// volatile SevSeg main_seven_segment{0};

extern void fill_digits(uint8_t value, volatile uint8_t *const buffer, uint8_t buffer_size);

const extern uint8_t seven_segment_cc[];

const extern uint8_t seven_segment_ca[];