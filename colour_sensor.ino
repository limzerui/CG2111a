#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#define S0_PORT PORTL
#define S0_DDR  DDRL
#define S0_BIT  6

#define S1_PORT PORTG
#define S1_DDR  DDRG
#define S1_BIT  0

#define S2_PORT PORTC
#define S2_DDR  DDRC
#define S2_BIT  0

#define S3_PORT PORTG
#define S3_DDR  DDRG
#define S3_BIT  2

#define SENSOR_PORT PORTD
#define SENSOR_DDR  DDRD
#define SENSOR_PIN  PIND
#define SENSOR_BIT  1

#define COLOR_RED 1
#define COLOR_GREEN 2
#define COLOR_UNCERTAIN 0

volatile uint16_t pulse_start = 0;
volatile uint16_t pulse_width = 0;
volatile bool pulse_measured = false;
volatile bool waiting_for_falling = true;
volatile unsigned long milliseconds = 0;

ISR(TIMER0_COMPA_vect) {
    milliseconds++;
}

ISR(INT1_vect) {
  if (waiting_for_falling) {
    if (!(SENSOR_PIN & (1 << SENSOR_BIT))) {
      pulse_start = TCNT1;
      waiting_for_falling = false;
    }
  } else {
    if (SENSOR_PIN & (1 << SENSOR_BIT)) {
      uint16_t pulse_end = TCNT1;
      pulse_width = pulse_end - pulse_start;
      pulse_measured = true;
      waiting_for_falling = true;
    }
  }
}

void setupTimer1(void) {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << CS11);
  TCNT1 = 0;
}

void setupTimer0(void) {
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS01) | (1 << CS00);
  OCR0A = 249;
  TIMSK0 = (1 << OCIE0A);
}

void setupInterrupt(void) {
  EICRA |= (1 << ISC10);
  EICRA &= ~(1 << ISC11);
  EIMSK |= (1 << INT1);
  sei();
}

void setFilter(bool s2, bool s3) {
  if (s2)
    S2_PORT |= (1 << S2_BIT);
  else
    S2_PORT &= ~(1 << S2_BIT);
    
  if (s3)
    S3_PORT |= (1 << S3_BIT);
  else
    S3_PORT &= ~(1 << S3_BIT);
    
  _delay_ms(50);
}

unsigned long millis(void) {
  unsigned long m;
  uint8_t oldSREG = SREG;
  cli();
  m = milliseconds;
  SREG = oldSREG;
  return m;
}

void uart_init(void) {
  UBRR0H = 0;
  UBRR0L = 103;
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (3 << UCSZ00);
}

void uart_transmit(unsigned char data) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;
}

void uart_print_string(const char* str) {
  while (*str)
    uart_transmit(*str++);
}

void uart_print_int(long val) {
  char buffer[16];
  ltoa(val, buffer, 10);
  uart_print_string(buffer);
}

void uart_print_float(float val, uint8_t precision) {
  int int_part = (int)val;
  uart_print_int(int_part);
  uart_transmit('.');
  
  if (val < 0 && int_part == 0)
    uart_transmit('-');
    
  float decimal_part = val - int_part;
  if (decimal_part < 0) decimal_part = -decimal_part;
  
  for (uint8_t i = 0; i < precision; i++) {
    decimal_part *= 10;
    uart_transmit('0' + ((int)decimal_part % 10));
  }
}

float getPulsePeriod(void) {
  pulse_measured = false;
  unsigned long start_wait = millis();
  
  while (!pulse_measured && (millis() - start_wait < 100)) {
  }
  
  if (!pulse_measured || pulse_width == 0) return 0;
  
  // Convert timer ticks to milliseconds (timer runs with prescaler 8, so each tick is 0.5 µs)
  // pulse_width * 0.5 gives period in µs, then / 1000 to convert to ms
  return (pulse_width * 0.5) / 1000.0;
}

uint8_t detectColor(float red, float green, float blue) {
  // Higher period means lower frequency which means less light reflected
  // So we need to invert our logic compared to frequency-based detection
  
  // If any reading is too small (meaning too high frequency or too bright reflection)
  // or too large (meaning sensor error or very dark)
  if (red < 0.01 || green < 0.01 || blue < 0.01 || 
      red > 10 || green > 10 || blue > 10) {
    return COLOR_UNCERTAIN;
  }
  
  // For period, higher values mean less light, so we want to look for the smallest period
  // Red object will reflect more red light = smaller red period
  if (red < green * 0.7 && red < blue * 0.7) {
    return COLOR_RED;
  }
  // Green object will reflect more green light = smaller green period
  else if (green < red * 0.7 && green < blue * 0.7) {
    return COLOR_GREEN;
  }
  
  return COLOR_UNCERTAIN;
}

void setup(void) {
  S0_DDR |= (1 << S0_BIT);
  S1_DDR |= (1 << S1_BIT);
  S2_DDR |= (1 << S2_BIT);
  S3_DDR |= (1 << S3_BIT);
  
  SENSOR_DDR &= ~(1 << SENSOR_BIT);
  
  S0_PORT |= (1 << S0_BIT);
  S1_PORT |= (1 << S1_BIT);
  
  setupTimer0();
  setupTimer1();
  setupInterrupt();
  uart_init();
}

void loop(void) {
  float red, green, blue, clear;
  
  setFilter(0, 0);
  red = getPulsePeriod();
  
  setFilter(1, 1);
  green = getPulsePeriod();
  
  setFilter(0, 1);
  blue = getPulsePeriod();
  
  setFilter(1, 0);
  clear = getPulsePeriod();
  
  uart_print_string("Red: ");
  uart_print_float(red, 3);
  uart_print_string(" ms, Green: ");
  uart_print_float(green, 3);
  uart_print_string(" ms, Blue: ");
  uart_print_float(blue, 3);
  uart_print_string(" ms, Clear: ");
  uart_print_float(clear, 3);
  
  uint8_t color = detectColor(red, green, blue);
  
  if (color == COLOR_RED) {
    uart_print_string(" - RED DETECTED\r\n");
  } else if (color == COLOR_GREEN) {
    uart_print_string(" - GREEN DETECTED\r\n");
  } else {
    uart_print_string(" - UNCERTAIN\r\n");
  }
  
  _delay_ms(500);
}

int main(void) {
  setup();
  
  while (1) {
    loop();
  }
  
  return 0;
}