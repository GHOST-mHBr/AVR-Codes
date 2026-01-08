#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <time.h>
#include <seven_segment.hpp>
#include <util/delay.h>
#include <string.h>
#include <common.hpp>

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

volatile SevSeg main_seven_segment;
volatile uint32_t second_quarters = 0;

enum __attribute__((packed)) State
{
  NORMAL,
  SET_CLOCK_HOUR,
  SET_CLOCK_MINUTE,
  SET_ALARM_HOUR,
  SET_ALARM_MINUTE,
  // END_SETTING
} programState;

volatile time_t epoch = 0;

volatile uint8_t seconds_ = 0;
volatile uint8_t minutes_ = 0;
volatile uint8_t hours_ = 0;

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

void epoch_to_time(time_t epoch_, volatile uint8_t* seconds, volatile uint8_t* minutes, volatile uint8_t* hours){
  epoch_ = epoch_ % ONE_DAY;
  *hours = epoch_/ONE_HOUR;
  epoch_ = epoch_-(*hours)*ONE_HOUR;
  
  *minutes = epoch_/ONE_MINUTE;
  *seconds = epoch_ - (*minutes)*ONE_MINUTE;
}

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
    _delay_ms(100);
    uint32_t current_second_quarter = second_quarters;
    while (read_bit(*pin_reg, pin) && (second_quarters - current_second_quarter) < 2)
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
  // TCCR0 = 0;
  TCCR0 = (1 << CS01) | (1 << CS00); // Setting prescaler to clk_io/64
}

/**
 * Configuring Timer2 to operate in Asynchronous mode. You have to directly connect a watch crystal(32.768KHz)
 * to TOSC1 and TOSC2 for suitable operation.
 * Prescaler is set to CLK_TS2/128, so since this is an 8bit timer, we will get 32768/128*256 = 1Hz.
 */
void config_timer2()
{
  ASSR = (1 << AS2);                 // Asynchronous mode selection
  TCCR2 = (1 << CS22) | (1 << CS20); // Set prescaler to CLK_ASYN/128 which results in 32768/(128*256)=1Hz
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

  // write_bit(SEV_DP_DDR, SEV_DP_PIN, 1);

  write_bit(KEYS_DDR, KEY_SET_PIN, 0);
  write_bit(KEYS_DDR, KEY_UP_PIN, 0);
  write_bit(KEYS_DDR, KEY_DOWN_PIN, 0);
  write_bit(ALARM_DDR, ALARM_PIN, 1);
}

void config_time()
{
  epoch = CURRENT_TIME;
  epoch_to_time(epoch, &seconds_, &minutes_, &hours_);
}

void config_seven_segment()
{
  main_seven_segment.enabled_digits = ALL_DIGITS;
  main_seven_segment.enabled_dots = 0x02;
  // Other fields will fill with zero due to the their default values
}

ISR(TIMER0_OVF_vect)
{
  // SEV_MUX_PORT = NO_DIGITS;
  show_cc(&main_seven_segment, &SEV_DATA_PORT, &SEV_MUX_PORT);
}

ISR(TIMER2_OVF_vect)
{
  seconds_++;
  if(seconds_ > 59){
    seconds_ = 0;
    minutes_++;
  }
  if(minutes_ > 59){
    minutes_ = 0;
    hours_++;
  }
  if(hours_ > 23){
    hours_ = 0;
  }

  toggle_bit(main_seven_segment.enabled_dots, 1);

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

  TIFR |= (1<<TOV2);
  // reti();
}

int main()
{
  _delay_ms(2000);
  cli();
  config_timer0();
  config_timer2();
  config_gpio();
  config_interrupts();
  config_time();
  config_seven_segment();
  programState = NORMAL;
  sei(); // Enabling interrupts

  struct hour_of_day hours;
  struct min_of_hour minutes;
  struct hour_of_day alarm_hour;
  struct min_of_hour alarm_minute;
  struct seven_segment_value *seven_disp01 = NULL;
  struct seven_segment_value *seven_disp23 = NULL;
  struct seven_segment_value **current_disp = NULL;

  seven_disp01 = ( seven_segment_value *)&hours;
  seven_disp23 = ( seven_segment_value *)&minutes;

  while (1)
  {
    // Note that the seven segment will get updated using the timer.
    // We don't need to do that things inside the main function

    if (is_clicked(&KEYS_PIN_REG, KEY_SET_PIN))
    {
      switch (programState)
      {
      case NORMAL:
      {
        seven_disp01 = ( seven_segment_value *)&hours;
        seven_disp23 = ( seven_segment_value *)&minutes;
        current_disp = &seven_disp01;
        main_seven_segment.hours_blink = 1;
        main_seven_segment.minutes_blink = 0;
        break;
      }

      case SET_CLOCK_HOUR:
      {
        seven_disp01 = ( seven_segment_value *)&hours;
        seven_disp23 = ( seven_segment_value *)&minutes;
        current_disp = &seven_disp23;
        main_seven_segment.hours_blink = 0;
        main_seven_segment.minutes_blink = 1;
        break;
      }

      case SET_CLOCK_MINUTE:
      {
        current_disp = &seven_disp01;
        main_seven_segment.hours_blink = 1;
        main_seven_segment.minutes_blink = 0;
        hours_ = seven_disp01->value;
        minutes_ = seven_disp23->value;
        epoch = hours_*ONE_HOUR + minutes_*ONE_MINUTE;
        seven_disp01 = ( seven_segment_value *)&alarm_hour;
        seven_disp23 = ( seven_segment_value *)&alarm_minute;
        break;
      }

      case SET_ALARM_HOUR:
      {
        seven_disp01 = ( seven_segment_value *)&alarm_hour;
        seven_disp23 = ( seven_segment_value *)&alarm_minute;
        current_disp = &seven_disp23;
        main_seven_segment.hours_blink = 0;
        main_seven_segment.minutes_blink = 1;
        break;
      }

      case SET_ALARM_MINUTE:
      {
        seven_disp01 = ( seven_segment_value *)&hours;
        seven_disp23 = ( seven_segment_value *)&minutes;
        current_disp = NULL;
        main_seven_segment.enabled_digits = ALL_DIGITS;
        main_seven_segment.hours_blink = 0;
        main_seven_segment.minutes_blink = 0;
        break;
      }
      }
      set_button_work();
    }

    if (programState == NORMAL)
    {
      cli();
      hours.value = hours_;
      minutes.value = minutes_;
      sei();
      _delay_ms(20);
    }
    _delay_ms(20);

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
    
    cli();
    fill_digits(seven_disp01->value, main_seven_segment.values, 2);
    fill_digits(seven_disp23->value, main_seven_segment.values + 2, 2);
    sei();
    
    write_bit(ALARM_PORT, ALARM_PIN, alarm_minute.value == minutes_ && alarm_hour.value == hours_);
  }
  return 0;
}