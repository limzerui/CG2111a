#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <AFMotor.h>
#include <Arduino.h>

// Color sensor pins
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

#define COMMAND_TIMEOUT 500 // 500ms timeout for heartbeat

// Motor control
AF_DCMotor right(1), left(4);

// Color sensor variables
volatile uint16_t pulse_start = 0;
volatile uint16_t pulse_width = 0;
volatile bool pulse_measured = false;
volatile bool waiting_for_falling = true;
volatile unsigned long milliseconds = 0;

// Encoder variables
volatile unsigned long leftTicks = 0, rightTicks = 0;
volatile unsigned long lastCommandTime = 0;

// RGB values
uint16_t red_value = 0;
uint16_t green_value = 0;
uint16_t blue_value = 0;

// Timer interrupt for millisecond tracking
ISR(TIMER0_COMPA_vect) {
    milliseconds++;
}

// Color sensor pulse measurement
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

// Encoder interrupt handlers
ISR(INT2_vect) {
  leftTicks++;
}

ISR(INT3_vect) {
  rightTicks++;
}

// Motor control functions
void translate(bool direction, int speed) {
  right.run(direction ? FORWARD : BACKWARD);
  left.run(direction ? FORWARD : BACKWARD);
  right.setSpeed(speed);
  left.setSpeed(speed);
}

void rotate(bool direction, int speed) {
  right.run(direction ? FORWARD : BACKWARD);
  left.run(direction ? BACKWARD : FORWARD);
  right.setSpeed(speed);
  left.setSpeed(speed);
}

void stop() {
  right.setSpeed(0);
  left.setSpeed(0);
}

// Set servo positions (0-180 degrees)
void setServos(int pos1, int pos2) {
  // Convert degrees to pulse width (1000-2000 µs)
  int pulse1 = map(pos1, 0, 180, 1000, 2000);
  int pulse2 = map(pos2, 0, 180, 1000, 2000);
  
  OCR5B = pulse1;
  OCR5C = pulse2;
}

// Read color sensor values
uint16_t readColorValue(bool s2, bool s3) {
  // Set filter type
  if (s2) S2_PORT |= (1 << S2_BIT);
  else S2_PORT &= ~(1 << S2_BIT);
  
  if (s3) S3_PORT |= (1 << S3_BIT);
  else S3_PORT &= ~(1 << S3_BIT);
  
  // Reset measurement flag
  pulse_measured = false;
  
  // Wait until we get a measurement or timeout
  unsigned long startTime = milliseconds;
  while (!pulse_measured) {
    if (milliseconds - startTime > 100) return 65535; // Timeout after 100ms
  }
  
  return pulse_width;
}

// Determine color based on RGB readings
uint8_t detectColor() {
  // Read red (S2 low, S3 low)
  red_value = readColorValue(false, false);
  
  // Read green (S2 high, S3 high)
  green_value = readColorValue(true, true);
  
  // Read blue (S2 low, S3 high)
  blue_value = readColorValue(false, true);
  
  // Simple color detection logic
  if (red_value < green_value && red_value < blue_value) {
    return COLOR_RED;
  } else if (green_value < red_value && green_value < blue_value) {
    return COLOR_GREEN;
  } else {
    return COLOR_UNCERTAIN;
  }
}

// Process serial commands
void processSerialCommands() {
  if (Serial.available() > 0) {
    int command = Serial.read();
    bool trBit = (command & 0b10000000) > 0;
    bool direction = (command & 0b01000000) > 0;
    int speed = (command & 0b00111111) * 4; // Scale to 0-252
    
    lastCommandTime = milliseconds; // Reset heartbeat timer
    
    if (trBit) {
      // Translate (forward/backward)
      translate(direction, speed);
      Serial.print("Translate: ");
      Serial.print(direction ? "Forward" : "Reverse");
    } else {
      // Rotate (left/right)
      rotate(direction, speed);
      Serial.print("Rotate: ");
      Serial.print(direction ? "Right" : "Left");
    }
    Serial.print(", Speed: ");
    Serial.println(speed);
  }
}

// Send color sensor data
void transmitColorData() {
  Serial.print("R:");
  Serial.print(red_value);
  Serial.print(" G:");
  Serial.print(green_value);
  Serial.print(" B:");
  Serial.print(blue_value);
  Serial.print(" Color:");
  
  uint8_t color = detectColor();
  switch(color) {
    case COLOR_RED:
      Serial.println("RED");
      break;
    case COLOR_GREEN:
      Serial.println("GREEN");
      break;
    default:
      Serial.println("UNCERTAIN");
      break;
  }
}

void setup() {
  cli();  // Disable interrupts during setup
  
  // Set up color sensor pins
  S0_DDR |= (1 << S0_BIT);
  S1_DDR |= (1 << S1_BIT);
  S2_DDR |= (1 << S2_BIT);
  S3_DDR |= (1 << S3_BIT);
  
  // Set sensor input pin
  SENSOR_DDR &= ~(1 << SENSOR_BIT);
  
  // Set frequency scaling to 20% (S0=high, S1=low)
  S0_PORT |= (1 << S0_BIT);
  S1_PORT &= ~(1 << S1_BIT);
  
  // Set up encoder pin interrupts
  EICRA = 0b01010000;  // Fire interrupt on signal change for INT2 and INT3
  EIMSK = 0b00001110;  // Enable INT1, INT2 & INT3 interrupts
  
  // Set encoder pins (PD2 and PD3) to input with pullups enabled
  DDRD &= ~0b00001100;
  PORTD |= 0b00001100;
  
  // Set sensor interrupt (INT1/PD1) to trigger on both edges
  EICRA |= 0b00000100;  // Any edge triggers INT1
  
  // Set servo pins (PL4 and PL5) to output
  DDRL |= 0b00110000;
  
  // Set Timer5 for servo control
  TCCR5A = 0b10100010;  // Set non-inverting mode on channels B and C
  TCCR5B = 0b00010010;  // Mode: Phase Correct PWM, TOP is ICR5, prescaler 8
  ICR5 = 20000;         // 50Hz PWM (20ms period)
  OCR5B = 1500;         // Initial position (neutral)
  OCR5C = 1500;         // Initial position (neutral)
  
  // Set Timer1 for pulse width measurement
  TCCR1A = 0;
  TCCR1B = 0b00000010;  // Prescaler 8
  TCNT1 = 0;           // Reset counter
  
  // Set Timer0 for millisecond counting
  TCCR0A = 0b00000010;  // CTC mode
  TCCR0B = 0b00000011;  // Prescaler 64
  OCR0A = 249;         // 1ms interrupt at 16MHz
  TIMSK0 = 0b00000010;  // Enable compare match interrupt
  
  // Standard serial setup
  Serial.begin(9600);
  
  sei();  // Enable interrupts
  
  // Initialize motors
  stop();
  
  // Initialize the lastCommandTime
  lastCommandTime = milliseconds;
  
  // Initial servo position
  setServos(90, 90);
  
  // Print startup message
  Serial.println("Robot initialized. Send commands to move:");
  Serial.println("- 192-255: Forward (bit 7=1, bit 6=1)");
  Serial.println("- 128-191: Reverse (bit 7=1, bit 6=0)");
  Serial.println("- 64-127: Turn Right (bit 7=0, bit 6=1)");
  Serial.println("- 0-63: Turn Left (bit 7=0, bit 6=0)");
  Serial.println("Speed is in bits 0-5 (0-63, scaled to 0-252)");
}

void loop() {
  // Check heartbeat - stop motors if no command received
  if (milliseconds - lastCommandTime > COMMAND_TIMEOUT) {
    stop();
  }
  
  // Process incoming commands
  processSerialCommands();
  
  // Transmit color data periodically
  static unsigned long lastTransmitTime = 0;
  if (milliseconds - lastTransmitTime > 500) { // Every 500ms
    transmitColorData();
    lastTransmitTime = milliseconds;
  }
}