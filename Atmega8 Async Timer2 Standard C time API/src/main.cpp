#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <time.h>
#include <seven_segment.h>

#define SEV_DATA_PORT PORTD
#define SEV_MUX_PORT PORTC
#define SEV_MUX_PIN0 PINC0
#define SEV_MUX_PIN1 PINC1
#define SEV_MUX_PIN2 PINC2
#define SEV_MUX_PIN3 PINC3
#define SEV_MUX_DDR DDRC
#define SEV_DATA_DDR DDRD

#define set_bit(byte, bit) (byte) |= (1 << (bit))
#define clear_bit(byte, bit) (byte) &= ~(1 << (bit))
#define write_bit(byte, bit, value) (value) == 0 ? clear_bit((byte), (bit)) : set_bit((byte), (bit))
#define read_bit(byte, bit) (byte) & (1 << bit)
#define toggle_bit(byte, bit) read_bit((byte), (bit)) ? write_bit(byte, bit, 0) : write_bit(byte, bit, 1)

volatile int8_t seconds_digits[2]{0};
volatile int8_t minutes_digits[2]{0};
volatile int8_t hours_digits[2]{0};
volatile uint8_t sev_counter = 0;

void fill_digits(int8_t value, volatile int8_t *const buffer, uint8_t buffer_size)
{
  int8_t i = 0;
  while (i < buffer_size)
  {
    buffer[buffer_size - i - 1] = value % 10;
    value /= 10;
    i++;
  }
}

ISR(TIMER0_OVF_vect)
{
  sev_counter++;
  if (sev_counter > 3)
    sev_counter = 0;

  SEV_MUX_PORT = ~(1 << sev_counter);
  SEV_DATA_PORT = sev_counter > 1 ? seven_segment_cc[minutes_digits[sev_counter % 2]] : seven_segment_cc[hours_digits[sev_counter % 2]];
}

ISR(TIMER2_OVF_vect, ISR_NAKED)
{
  toggle_bit(PORTB, PINB0);
  system_tick();
  reti();
}

int main()
{
  cli();

  // Timer2 Asynchronous configuration with overflow interrupt
  ASSR |= (1 << AS2);
  TCCR2 = 0b00000101; 
  TIMSK |= (1 << TOIE2); //Enabling Overflow Interrupt/
  while (!(bit_is_clear(ASSR, TCN2UB) && bit_is_clear(ASSR, TCR2UB)))
  {
  }

  // Timer0 configuration for 7segment
  TCCR0 = 0;
  TCCR0 = (1 << CS02); // Setting prescaler to clk_io/256
  TIMSK |= (1 << TOIE0); //Enabling Overflow Interrupt/

  SEV_MUX_DDR = 0xFF;
  SEV_DATA_DDR = 0x0F;
  SEV_DATA_PORT = 0x00;
  SEV_MUX_PORT = 0x00;
  DDRB = 0x01;

  set_system_time(CURRENT_TIME - UNIX_OFFSET);  //CURRENT_TIME get defined at compile time using the -D flag. Check platformio.ini file
  set_zone(3.5 * ONE_HOUR);
  set_position(35.7219 * ONE_DEGREE, 51.3347 * ONE_DEGREE);

  // set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  // sleep_mode();

  int8_t hours = 0;
  int8_t minutes = 0;
  // int8_t seconds = 0;
  time_t current_time = 0;
  sei();

  while (1)
  {
    current_time = time(NULL);
    hours = localtime(&current_time)->tm_hour;
    minutes = localtime(&current_time)->tm_min;
    // seconds = localtime(&current_time)->tm_sec;
    fill_digits(hours, hours_digits, 2);
    fill_digits(minutes, minutes_digits, 2);
    // fill_digits(seconds, seconds_digits, 2);

  }
  return 0;
}