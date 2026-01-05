#include "seven_segment.hpp"
#include "common.hpp"
void fill_digits(uint8_t value, volatile uint8_t *const buffer, uint8_t buffer_size)
{
  uint8_t i = 0;
  while (i < buffer_size)
  {
    buffer[buffer_size - i - 1] = value % 10;
    value /= 10;
    i++;
  }
}


const uint8_t seven_segment_cc[] = {
    0b00111111, //0
    0b00000110, //1
    0b01011011, //2
    0b01001111, //3
    0b01100110, //4
    0b01101101, //5
    0b01111101, //6
    0b00000111, //7
    0b01111111, //8
    0b01101111, //9
    0b01110111, //A
    0b01111111, //B
    0b00111001, //C
    0b00111111, //D
    0b01111001, //E
    0b01110001  //F
};


const uint8_t seven_segment_ca[] = {
    0b11000000, //0
    0b11111001, //1
    0b10100100, //2
    0b10110000, //3
    0b10011001, //4
    0b10010010, //5
    0b10000010, //6
    0b11111000, //7
    0b10000000, //8
    0b10010000, //9
    0b10001000, //A
    0b10000000, //B
    0b11000110, //C
    0b11000000, //D
    0b10000110, //E
    0b10000001  //F
};