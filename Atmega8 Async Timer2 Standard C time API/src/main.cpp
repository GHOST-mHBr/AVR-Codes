#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <time.h>
#include <seven_segment.hpp>
#include <util/delay.h>
#include <string.h>

#define SEV_DATA_PORT PORTD
#define SEV_MUX_PORT PORTC
#define SEV_MUX_PIN0 PINC0
#define SEV_MUX_PIN1 PINC1
#define SEV_MUX_PIN2 PINC2
#define SEV_MUX_PIN3 PINC3
#define SEV_MUX_DDR DDRC
#define SEV_DATA_DDR DDRD
#define SEV_DP_DDR DDRD
#define SEV_DP_PIN PIND7

#define KEYS_DDR DDRB
#define KEYS_PIN_REG PINB
#define KEY_SET_PIN PINB1
#define KEY_UP_PIN PINB2
#define KEY_DOWN_PIN PINB3

#define ALARM_PORT PORTB
#define ALARM_PIN PORTB0
#define ALARM_DDR DDRB

#define ONE_MINUTE 60

#define set_bit(byte, bit) (byte) |= (1 << (bit))
#define clear_bit(byte, bit) (byte) &= ~(1 << (bit))
#define write_bit(byte, bit, value) (value) == 0 ? clear_bit((byte), (bit)) : set_bit((byte), (bit))
#define read_bit(byte, bit) (((byte) & (1 << bit)) > 0)
#define toggle_bit(byte, bit) read_bit((byte), (bit)) ? write_bit(byte, bit, 0) : write_bit(byte, bit, 1)

volatile SevSeg main_seven_segment;

enum State
{
  NORMAL,
  SET_CLOCK_HOUR,
  SET_CLOCK_MINUTE,
  SET_ALARM_HOUR,
  SET_ALARM_MINUTE,
  // END_SETTING
} programState;

struct hour_of_day
{
  volatile uint8_t value = 0;
  const uint8_t MAX = 23;
  const uint8_t MIN = 0;
};

struct min_of_hour
{
  volatile uint8_t value = 0;
  const uint8_t MAX = 59;
  const uint8_t MIN = 0;
};

struct seven_segment_value
{
  volatile uint8_t value = 0;
  const uint8_t max_value = 0;
  const uint8_t min_value = 0;
};

volatile uint8_t timer_cnt = 0;

void set_button_work()
{
  programState = (State)(programState + 1);
  if (programState > SET_ALARM_MINUTE)
  {
    programState = NORMAL;
  }
}

// BLOCKING function that checks if a button is clicked
uint8_t is_clicked(volatile uint8_t *pin_reg, uint8_t pin)
{
  if (read_bit(*pin_reg, pin))
  {
    _delay_ms(200);
    while (read_bit(*pin_reg, pin))
    {
    } // Wait until the button gets released
    return true;
  }
  return false;
}

/**
 * configuring timer0 for seven segment with Normal mode and prescaler = CLK_IO/256.
 * We are going to use its overflow interrupt.
 * Sine CLK_IO = 8MHz, and this timer is an 8bit-one, we will get 1/(8M/(256*256)) = 32ms ~ 30fps
 */
void config_timer0()
{
  TCCR0 = 0;
  TCCR0 = (1 << CS02); // Setting prescaler to clk_io/256
}

/**
 * Configuring Timer2 to operate in Asynchronous mode. You have to directly connect a watch crystal(32.768KHz)
 * to TOSC1 and TOSC2 for suitable operation.
 * Prescaler is set to CLK_TS2/128, so since this is an 8bit timer, we will get 32768/128*256 = 1Hz.
 */
void config_timer2()
{
  ASSR |= (1 << AS2);                 // Asynchronous mode selection
  TCCR2 |= (1 << CS21) | (1 << CS20); // Set prescaler to CLK_ASYN/128 which results in 32768/(128*256)=1Hz
  while (!(bit_is_clear(ASSR, TCN2UB) && bit_is_clear(ASSR, TCR2UB)))
  {
  } // Recommended by data sheet to wait until changes get applied
}

/**
 * Configuring interrupts mask to be enabled for Timer0 and Timer2 overflow
 */
void config_interrupts()
{
  TIMSK |= (1 << TOIE2) | (1 << TOIE0);
}

void config_gpio()
{
  SEV_MUX_DDR = 0x0F;
  SEV_MUX_PORT = 0x00;
  SEV_DATA_DDR = 0xFF;
  SEV_DATA_PORT = 0x00;

  write_bit(SEV_DP_DDR, SEV_DP_PIN, 1);

  write_bit(KEYS_DDR, KEY_SET_PIN, 0);
  write_bit(KEYS_DDR, KEY_UP_PIN, 0);
  write_bit(KEYS_DDR, KEY_DOWN_PIN, 0);
  write_bit(ALARM_DDR, ALARM_PIN, 1);
}

void config_time()
{
  set_system_time(CURRENT_TIME - UNIX_OFFSET); // CURRENT_TIME get defined at compile time using the -D flag. Check platformio.ini file
  set_zone(3.5 * ONE_HOUR);
  set_position(35.7219 * ONE_DEGREE, 51.3347 * ONE_DEGREE);
}

void config_seven_segment()
{
  main_seven_segment.enabled_digits = ALL_DIGITS;
  main_seven_segment.enabled_dots = 0x02;
  // Other fields will fill with zero due to the their default values
}

ISR(TIMER0_OVF_vect)
{
  main_seven_segment.digit_counter++;
  if (main_seven_segment.digit_counter > 3)
    main_seven_segment.digit_counter = 0;

  SEV_MUX_PORT = ~(1 << main_seven_segment.digit_counter);
  volatile uint8_t value = seven_segment_cc[main_seven_segment.values[main_seven_segment.digit_counter]];
  value &= (read_bit(main_seven_segment.enabled_digits, main_seven_segment.digit_counter) ? 0xFF : 0x00);
  write_bit(value, 7, read_bit(main_seven_segment.enabled_dots, main_seven_segment.digit_counter));
  SEV_DATA_PORT = value;
}

ISR(TIMER2_OVF_vect, ISR_NAKED)
{
  timer_cnt++;
  if (timer_cnt > 3)
  {
    timer_cnt = 0;
    system_tick();
    toggle_bit(main_seven_segment.enabled_dots, 1);
  }

  // Check for hours blinking
  if (main_seven_segment.hours_blink)
  {
    if (main_seven_segment.enabled_digits == MINUTE_DIGITS)
    {
      main_seven_segment.enabled_digits = ALL_DIGITS;
    }
    else
    {
      main_seven_segment.enabled_digits = MINUTE_DIGITS;
    }
  }

  // Check for minutes blinking
  if (main_seven_segment.minutes_blink)
  {
    if (main_seven_segment.enabled_digits == HOUR_DIGITS)
    {
      main_seven_segment.enabled_digits = ALL_DIGITS;
    }
    else
    {
      main_seven_segment.enabled_digits = HOUR_DIGITS;
    }
  }

  reti();
}

int main()
{
  cli();
  config_timer0();
  config_timer2();
  config_gpio();
  config_interrupts();
  config_time();
  config_seven_segment();
  programState = NORMAL;
  sei(); // Enabling interrupts

  time_t current_time = 0;
  volatile struct hour_of_day hours;
  volatile struct hour_of_day prev_hours;
  volatile struct min_of_hour minutes;
  volatile struct min_of_hour prev_minutes;
  volatile struct hour_of_day alarm_hour;
  volatile struct min_of_hour alarm_minute;
  volatile struct seven_segment_value *seven_disp01 = NULL;
  volatile struct seven_segment_value *seven_disp23 = NULL;
  volatile struct seven_segment_value **current_disp = NULL;
  volatile time_t prev_time = 0;
  // seven_disp01->value = 10;
  // seven_disp23->value = 20;

  seven_disp01 = (volatile seven_segment_value *)&hours;
  seven_disp23 = (volatile seven_segment_value *)&minutes;

  while (1)
  {
    // Note that the seven segment will get updated using the timer.
    // We don't need to do that things inside the main function

    if (is_clicked(&KEYS_PIN_REG, KEY_SET_PIN))
    {
      switch (programState)
      {
      case NORMAL:
        seven_disp01 = (volatile seven_segment_value *)&hours;
        seven_disp23 = (volatile seven_segment_value *)&minutes;
        current_disp = &seven_disp01;
        main_seven_segment.hours_blink = 1;
        main_seven_segment.minutes_blink = 0;
        prev_time = time(NULL);
        prev_hours.value = hours.value;
        prev_minutes.value = minutes.value;
        break;

      case SET_CLOCK_HOUR:
        seven_disp01 = (volatile seven_segment_value *)&hours;
        seven_disp23 = (volatile seven_segment_value *)&minutes;
        current_disp = &seven_disp23;
        main_seven_segment.hours_blink = 0;
        main_seven_segment.minutes_blink = 1;
        break;

      case SET_CLOCK_MINUTE:
        current_disp = &seven_disp01;
        main_seven_segment.hours_blink = 1;
        main_seven_segment.minutes_blink = 0;
        set_system_time(prev_time + ONE_HOUR * (seven_disp01->value - prev_hours.value) + ONE_MINUTE*(seven_disp23->value-prev_minutes.value));
        seven_disp23 = (volatile seven_segment_value *)&alarm_minute;
        seven_disp01 = (volatile seven_segment_value *)&alarm_hour;
        break;

      case SET_ALARM_HOUR:
        seven_disp01 = (volatile seven_segment_value *)&alarm_hour;
        seven_disp23 = (volatile seven_segment_value *)&alarm_minute;
        current_disp = &seven_disp23;
        main_seven_segment.hours_blink = 0;
        main_seven_segment.minutes_blink = 1;
        break;

      case SET_ALARM_MINUTE:
        seven_disp01 = (volatile seven_segment_value *)&hours;
        seven_disp23 = (volatile seven_segment_value *)&minutes;
        current_disp = NULL;
        main_seven_segment.enabled_digits = ALL_DIGITS;
        main_seven_segment.hours_blink = 0;
        main_seven_segment.minutes_blink = 0;
        // alarm_hour.value = seven_disp01->value;
        // alarm_minute.value = seven_disp23->value;
        break;
      }
      set_button_work();
    }

    if (programState == NORMAL)
    {
      current_time = time(NULL);
      hours.value = localtime(&current_time)->tm_hour;
      minutes.value = localtime(&current_time)->tm_min;
      _delay_ms(500);
    }

    if (current_disp != NULL)
    {
      if (is_clicked(&KEYS_PIN_REG, KEY_UP_PIN))
      {
        if ((*current_disp)->value < (*current_disp)->max_value)
          (*current_disp)->value++;
        else
          (*current_disp)->value = (*current_disp)->min_value;
      }

      if (is_clicked(&KEYS_PIN_REG, KEY_DOWN_PIN))
      {
        if ((*current_disp)->value > (*current_disp)->min_value)
          (*current_disp)->value--;
        else
          (*current_disp)->value = (*current_disp)->max_value;
      }
    }
    fill_digits(seven_disp01->value, main_seven_segment.values, 2);
    fill_digits(seven_disp23->value, main_seven_segment.values + 2, 2);
    write_bit(ALARM_PORT, ALARM_PIN, alarm_minute.value == minutes.value && alarm_hour.value == hours.value);
  }
  return 0;
}