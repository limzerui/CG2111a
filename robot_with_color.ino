#include <AFMotor.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

// Color sensor pin definitions
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

// Color definitions
#define COLOR_RED 1
#define COLOR_GREEN 2
#define COLOR_UNCERTAIN 0

// Motor setup
AF_DCMotor right(1), left(4);

// Command variables
const int MAX_COMMAND_LENGTH = 32;
char command[MAX_COMMAND_LENGTH];
int commandIndex = 0;

// Encoder variables
volatile unsigned long leftTicks = 0, rightTicks = 0;

// Color sensor variables
volatile uint16_t pulse_start = 0;
volatile uint16_t pulse_width = 0;
volatile bool pulse_measured = false;
volatile bool waiting_for_falling = true;

//=========== INTERRUPT SERVICE ROUTINES ===========//

// Color sensor interrupt for pulse measurement
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

// Left encoder interrupt
ISR(INT2_vect) {
  leftTicks++;
}

// Right encoder interrupt
ISR(INT3_vect) {
  rightTicks++;
}

//=========== MOTOR CONTROL FUNCTIONS ===========//

void translate(bool direction, int speed) {
  Serial.print("Setting motors: direction=");
  Serial.print(direction ? "FORWARD" : "BACKWARD");
  Serial.print(", speed=");
  Serial.println(speed);
  
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
  right.run(RELEASE);
  left.run(RELEASE);
  right.setSpeed(0);
  left.setSpeed(0);
}

void setServos(int pos1, int pos2) {
  OCR5B = constrain(pos1, 1000, 2000);
  OCR5C = constrain(pos2, 1000, 2000);
}

//=========== COLOR SENSOR FUNCTIONS ===========//

void setupTimer1(void) {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << CS11);  // Prescaler = 8
  TCNT1 = 0;
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

float getPulsePeriod(void) {
  pulse_measured = false;
  uint16_t start_ticks = TCNT1;
  
  // Wait for pulse to be measured or timeout after ~100ms
  const uint16_t MAX_WAIT_TICKS = 32768;
  
  for (uint8_t timeoutCounter = 0; timeoutCounter < 7 && !pulse_measured; timeoutCounter++) {
    uint16_t timeout_end = start_ticks + MAX_WAIT_TICKS;
    
    while (((uint16_t)TCNT1 != timeout_end) && !pulse_measured) {
      // Just waiting
    }
    
    start_ticks = TCNT1;
  }
  
  if (!pulse_measured || pulse_width == 0) return 0;
  
  return (pulse_width * 0.5) / 1000.0;  // Convert to milliseconds
}

uint8_t detectColor(float red, float green, float blue) {
  // Validity check
  if (red < 0.01 || green < 0.01 || blue < 0.01 || 
      red > 10 || green > 10 || blue > 10) {
    return COLOR_UNCERTAIN;
  }
  
  // For period, smaller means more reflection
  if (red < green * 0.7 && red < blue * 0.7) {
    return COLOR_RED;
  }
  else if (green < red * 0.7 && green < blue * 0.7) {
    return COLOR_GREEN;
  }
  
  return COLOR_UNCERTAIN;
}

void readColor() {
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
}

//=========== COMMAND PROCESSING ===========//

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Echo characters back
    Serial.write(c);
    
    // Process command when newline is received
    if (c == '\n' || c == '\r') {
      if (commandIndex > 0) {
        command[commandIndex] = '\0';
        Serial.print("Processing command: ");
        Serial.println(command);
        processCommand();
      }
      commandIndex = 0;
    } else if (commandIndex < MAX_COMMAND_LENGTH - 1) {
      command[commandIndex++] = c;
    }
  }
}

void processCommand() {
  char cmdCopy[MAX_COMMAND_LENGTH];
  strcpy(cmdCopy, command);
  
  char commandType = cmdCopy[0];
  int value1 = 0, value2 = 0;
  
  // Extract parameters
  if (strlen(cmdCopy) > 1) {
    char *firstParam = strchr(cmdCopy, ',');
    if (firstParam != NULL) {
      firstParam++;
      value1 = atoi(firstParam);
      
      char *secondParam = strchr(firstParam, ',');
      if (secondParam != NULL) {
        secondParam++;
        value2 = atoi(secondParam);
      }
    }
  }
  
  // Process based on command type
  switch (commandType) {
    // Motor Commands
    case 'F': // Forward: F,speed
      Serial.println("Moving forward");
      translate(true, value1);
      break;
      
    case 'B': // Backward: B,speed
      Serial.println("Moving backward");
      translate(false, value1);
      break;
      
    case 'R': // Rotate: R,direction,speed (1=right, 0=left)
      Serial.println("Rotating");
      rotate(value1 == 1, value2);
      break;
      
    case 'S': // Stop: S
      Serial.println("Stopping");
      stop();
      break;
      
    case 'V': // Servos: V,pos1,pos2
      Serial.println("Setting servos");
      setServos(value1, value2);
      break;
      
    case 'E': // Read encoders: E
      Serial.print("Encoders: Left=");
      Serial.print(leftTicks);
      Serial.print(" Right=");
      Serial.println(rightTicks);
      break;
      
    case 'C': // Clear encoders: C
      leftTicks = 0;
      rightTicks = 0;
      Serial.println("Encoders cleared");
      break;
    
    // Color Sensor Command
    case 'D': // Detect color: D
      Serial.println("Reading color");
      readColor();
      break;
      
    default:
      Serial.print("Unknown command: ");
      Serial.println(commandType);
      break;
  }
}

//=========== SETUP & MAIN LOOP ===========//

void setup() {
  // Initialize Serial
  Serial.begin(9600);
  delay(1000);
  
  // Initialize motor control
  right.run(RELEASE);
  left.run(RELEASE);
  stop();
  
  // Initialize color sensor pins
  S0_DDR |= (1 << S0_BIT);
  S1_DDR |= (1 << S1_BIT);
  S2_DDR |= (1 << S2_BIT);
  S3_DDR |= (1 << S3_BIT);
  SENSOR_DDR &= ~(1 << SENSOR_BIT);
  S0_PORT |= (1 << S0_BIT);
  S1_PORT |= (1 << S1_BIT);
  
  cli(); // Disable interrupts during setup
  
  // Set up timers
  setupTimer1(); // Timer1 for color sensor
  
  // Set up Timer5 for servo control
  TCCR5A = 0b00101000;
  TCCR5B = 0b00010010;
  ICR5 = 20000;
  OCR5B = 1500;
  OCR5C = 1500;
  
  // Set up interrupts
  EICRA = 0b01010000; // Trigger on any edge for INT2 and INT3
  EIMSK = 0b00001100; // Enable INT2 and INT3
  DDRD &= ~0b00001100;
  PORTD |= 0b00001100;
  
  EICRA |= (1 << ISC10);  // Any logical change on INT1
  EICRA &= ~(1 << ISC11);
  EIMSK |= (1 << INT1);   // Enable INT1 interrupt
  
  // Set up servo pins
  DDRL |= 0b00110000;
  
  sei(); // Re-enable interrupts
  
  Serial.println("Robot with Color Detection initialized!");
  Serial.println("Commands:");
  Serial.println("  F,speed - Forward");
  Serial.println("  B,speed - Backward");
  Serial.println("  R,dir,speed - Rotate (dir: 1=right, 0=left)");
  Serial.println("  S - Stop");
  Serial.println("  V,pos1,pos2 - Servos");
  Serial.println("  E - Read encoders");
  Serial.println("  C - Clear encoders");
  Serial.println("  D - Detect color");
}

void loop() {
  // Check for serial commands
  readSerialCommands();
  delay(10);
} 