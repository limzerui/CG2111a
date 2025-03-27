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

// Add a function to get elapsed time based on Timer1
unsigned long micros_timer1() {
  // Timer1 is running with prescaler 8, so each tick is 0.5 microseconds
  // Multiply by 0.5 to get microseconds
  return (unsigned long)(TCNT1) * 0.5;
}

// Calculate a timeout in Timer1 ticks (at 0.5us per tick)
uint16_t timeout_ticks(unsigned long milliseconds) {
  // Convert milliseconds to Timer1 ticks (1ms = 2000 ticks at 0.5us per tick)
  return milliseconds * 2000;
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
  TCCR1B |= (1 << CS11);  // Prescaler = 8
  TCNT1 = 0;
}

void setupInterrupt(void) {
  EICRA |= (1 << ISC10);  // Any logical change on INT1
  EICRA &= ~(1 << ISC11);
  EIMSK |= (1 << INT1);   // Enable INT1 interrupt
  sei();                  // Enable global interrupts
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
    
  _delay_ms(50);  // Use _delay_ms which doesn't depend on timers
}

float getPulsePeriod(void) {
  pulse_measured = false;
  uint16_t start_ticks = TCNT1;  // Use Timer1 directly
  
  // Wait for pulse to be measured or timeout after 100ms
  while (!pulse_measured) {
    uint16_t current_ticks = TCNT1;
    uint16_t elapsed;
    
    // Handle timer overflow
    if (current_ticks < start_ticks) {
      elapsed = (65535 - start_ticks) + current_ticks;
    } else {
      elapsed = current_ticks - start_ticks;
    }
    
    // Check if we've exceeded 100ms timeout (200,000 timer ticks at 0.5us per tick)
    if (elapsed > 200000) {
      break;
    }
  }
  
  if (!pulse_measured || pulse_width == 0) return 0;
  
  // Convert timer ticks to milliseconds (timer runs with prescaler 8, so each tick is 0.5 µs)
  // pulse_width * 0.5 gives period in µs, then / 1000 to convert to ms
  return (pulse_width * 0.5) / 1000.0;
}

uint8_t detectColor(float red, float green, float blue) {
  // If any reading is too small or too large
  if (red < 0.01 || green < 0.01 || blue < 0.01 || 
      red > 10 || green > 10 || blue > 10) {
    return COLOR_UNCERTAIN;
  }
  
  // For period, higher values mean less light, so we want to look for the smallest period
  if (red < green * 0.7 && red < blue * 0.7) {
    return COLOR_RED;
  }
  else if (green < red * 0.7 && green < blue * 0.7) {
    return COLOR_GREEN;
  }
  
  return COLOR_UNCERTAIN;
}

void setup(void) {
  // Initialize Serial communication
  Serial.begin(9600);
  
  S0_DDR |= (1 << S0_BIT);
  S1_DDR |= (1 << S1_BIT);
  S2_DDR |= (1 << S2_BIT);
  S3_DDR |= (1 << S3_BIT);
  
  SENSOR_DDR &= ~(1 << SENSOR_BIT);
  
  S0_PORT |= (1 << S0_BIT);
  S1_PORT |= (1 << S1_BIT);
  
  setupTimer1();
  setupInterrupt();
  
  Serial.println("Color sensor initialized (single timer version)");
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
  
  Serial.print("Red: ");
  Serial.print(red, 3);
  Serial.print(" ms, Green: ");
  Serial.print(green, 3);
  Serial.print(" ms, Blue: ");
  Serial.print(blue, 3);
  Serial.print(" ms, Clear: ");
  Serial.print(clear, 3);
  
  uint8_t color = detectColor(red, green, blue);
  
  if (color == COLOR_RED) {
    Serial.println(" - RED DETECTED");
  } else if (color == COLOR_GREEN) {
    Serial.println(" - GREEN DETECTED");
  } else {
    Serial.println(" - UNCERTAIN");
  }
  
  // Use _delay_ms which doesn't rely on timers for timing
  _delay_ms(500);
} 