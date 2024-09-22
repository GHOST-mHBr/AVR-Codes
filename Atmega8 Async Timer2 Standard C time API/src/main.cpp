#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <time.h>


#define set_bit(byte,bit) (byte) |= (1<<(bit))  
#define clear_bit(byte,bit) (byte) &= ~(1<<(bit))  
#define write_bit(byte, bit , value) (value) == 0 ? clear_bit((byte),(bit)) : set_bit((byte),(bit))
#define read_bit(byte, bit) (byte) & (1<<bit)
#define toggle_bit(byte,bit) read_bit((byte), (bit)) ? write_bit(byte,bit,0) : write_bit(byte,bit,1)


ISR(TIMER2_OVF_vect, ISR_NAKED)
{
  // toggle_bit(PORTC, PINC0);
  system_tick();
  reti();
}

int main()
{
  cli();
  ASSR |= (1 << AS2);
  TCCR2 = 0b00000101;
  TCNT2 = 0;
  TIFR |= (1 << TOV2);
  TIMSK |= (1 << TOIE2);
  while (!(bit_is_clear(ASSR, TCN2UB) && bit_is_clear(ASSR, TCR2UB))){}
  sei();

  DDRC = 0xFF;
  PORTC = 0x00;

  struct tm t{0};
  t.tm_isdst=0;
  t.tm_min=1;
  set_system_time(mktime(&t));

  // set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  // sleep_mode();

  while (1){
    //We are in power save mode
    time_t current_time = time(NULL);
    PORTC = (localtime(&current_time) -> tm_min << 4) | ((localtime(&current_time) -> tm_sec)&(0x0F));
  }
  return 0;
}