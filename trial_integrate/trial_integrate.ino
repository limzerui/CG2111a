#include <AFMotor.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

// Motor setup
AF_DCMotor right(4);
AF_DCMotor left(1);

// Command variables
const int MAX_COMMAND_LENGTH = 32;
char command[MAX_COMMAND_LENGTH];
int commandIndex = 0;

// Encoder variables
volatile unsigned long leftTicks = 0, rightTicks = 0;

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

// Color sensor variables
volatile uint16_t pulse_start = 0;
volatile uint16_t pulse_width = 0;
volatile bool pulse_measured = false;
volatile bool waiting_for_falling = true;

// Motor control functions - using DIRECT control
void moveForward(int speed) {
  Serial.println("Forward - Setting both motors");

  // Set directions individually
  right.run(FORWARD);
  left.run(FORWARD);  
  // Set speeds individually
  right.setSpeed(speed);
  left.setSpeed(speed);
}

void moveBackward(int speed) {
  Serial.println("Backward - Setting both motors");
  
  // Set directions individually
  right.run(BACKWARD);
  left.run(BACKWARD);  
  // Set speeds individually
  right.setSpeed(speed);
  left.setSpeed(speed);
}

void turnRight(int speed) {
  Serial.println("Right Turn - Setting both motors");
  
  // First reset both motors
  right.run(RELEASE);
  left.run(RELEASE);
  
  // Set directions individually
  right.run(BACKWARD);
  left.run(FORWARD);
  
  // Set speeds individually
  right.setSpeed(speed);
  left.setSpeed(speed);
}

void turnLeft(int speed) {
  Serial.println("Left Turn - Setting both motors");
  
  // Set directions individually
  right.run(FORWARD);
  left.run(BACKWARD);  
  // Set speeds individually
  right.setSpeed(speed);
  left.setSpeed(speed);
}

void stopMotors() {
  Serial.println("Stopping motors");
  right.setSpeed(0);
  left.setSpeed(0);
}

void setServos(int pos1, int pos2) {
  OCR5B = constrain(pos1, 1000, 2000);
  OCR5C = constrain(pos2, 1000, 2000);
}

// Interrupt handlers - carefully separated
// ISR(INT1_vect) {
//   // Color sensor pulse measurement
//   if (waiting_for_falling) {
//     if (!(SENSOR_PIN & (1 << SENSOR_BIT))) {
//       pulse_start = TCNT1;
//       waiting_for_falling = false;
//     }
//   } else {
//     if (SENSOR_PIN & (1 << SENSOR_BIT)) {
//       uint16_t pulse_end = TCNT1;
//       pulse_width = pulse_end - pulse_start;
//       pulse_measured = true;
//       waiting_for_falling = true;
//     }
//   }
// }

// ISR(INT2_vect) {
//   // Motor encoder
//   leftTicks++;
// }

// ISR(INT3_vect) {
//   // Motor encoder
//   rightTicks++;
// }

// Timer setup functions
void setupTimer1(void) {
  // For color sensor pulse measurement
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << CS11);  // Prescaler 8
  TCNT1 = 0;
}

void setupTimer5(void) {
  // For servo control
  TCCR5A = 0b00101000;    // Phase correct PWM
  TCCR5B = 0b00010010;    // Prescaler 8
  ICR5 = 20000;
  OCR5B = 1500;
  OCR5C = 1500;
}

// Color sensor functions
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
  unsigned long start_wait = millis();
  
  while (!pulse_measured && (millis() - start_wait < 100)) {
    // Wait for pulse measurement
  }
  
  if (!pulse_measured || pulse_width == 0) return 0;
  
  // Convert timer ticks to milliseconds
  return (pulse_width * 0.5) / 1000.0;
}

uint8_t detectColor(float red, float green, float blue) {
  Serial.print("Color values - Red: ");
  Serial.print(red, 3);
  Serial.print(", Green: ");
  Serial.print(green, 3);
  Serial.print(", Blue: ");
  Serial.println(blue, 3);
  
  // If any reading is too small or too large
  if (red < 0.01 || green < 0.01 || blue < 0.01 || 
      red > 10 || green > 10 || blue > 10) {
    return COLOR_UNCERTAIN;
  }
  
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

// Serial command processing
void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Process command when newline is received
    if (c == '\n' || c == '\r') {
      if (commandIndex > 0) {
        command[commandIndex] = '\0'; // Null terminate
        Serial.print("Command received: ");
        Serial.println(command);
        processCommand();
      }
      commandIndex = 0; // Reset for next command
    } else if (commandIndex < MAX_COMMAND_LENGTH - 1) {
      command[commandIndex++] = c;
    }
  }
}

void processCommand() {
  // Make a copy of the command for processing
  char cmdCopy[MAX_COMMAND_LENGTH];
  strcpy(cmdCopy, command);
  
  char commandType = cmdCopy[0];
  int value1 = 0, value2 = 0;
  
  // Extract parameters if they exist
  if (strlen(cmdCopy) > 1) {
    char *firstParam = strchr(cmdCopy, ',');
    if (firstParam != NULL) {
      firstParam++; // Skip the comma
      value1 = atoi(firstParam);
      
      char *secondParam = strchr(firstParam, ',');
      if (secondParam != NULL) {
        secondParam++; // Skip the comma
        value2 = atoi(secondParam);
      }
    }
  }
  
  // Process commands
  switch (commandType) {
    case 'F': // Forward: F,speed
      moveForward(value1);
      break;
      
    case 'B': // Backward: B,speed
      moveBackward(value1);
      break;
      
    case 'L': // Left turn: L,speed
      turnLeft(value1);
      break;
      
    case 'R': // Right turn: R,speed
      turnRight(value1);
      break;
      
    case 'S': // Stop: S
      stopMotors();
      break;
      
    case 'V': // Servos: V,pos1,pos2
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
      
    case 'D': // Detect color: D
      detectAndReportColor();
      break;
      
    case 'T': // Test motors: T
      testMotors();
      break;
      
    default:
      Serial.print("Unknown command: ");
      Serial.println(commandType);
      break;
  }
}

void detectAndReportColor() {
  float red, green, blue, clear;
  
  Serial.println("Color detection started...");
  
  setFilter(0, 0); // Red filter
  red = getPulsePeriod();
  
  setFilter(1, 1); // Green filter
  green = getPulsePeriod();
  
  setFilter(0, 1); // Blue filter
  blue = getPulsePeriod();
  
  setFilter(1, 0); // Clear (no filter)
  clear = getPulsePeriod();
  
  Serial.print("Readings - Red: ");
  Serial.print(red, 3);
  Serial.print(" ms, Green: ");
  Serial.print(green, 3);
  Serial.print(" ms, Blue: ");
  Serial.print(blue, 3);
  Serial.print(" ms, Clear: ");
  Serial.print(clear, 3);
  Serial.println(" ms");
  
  uint8_t color = detectColor(red, green, blue);
  
  if (color == COLOR_RED) {
    Serial.println("RED DETECTED");
  } else if (color == COLOR_GREEN) {
    Serial.println("GREEN DETECTED");
  } else {
    Serial.println("COLOR UNCERTAIN");
  }
}

void testMotors() {
  Serial.println("===== MOTOR TEST SEQUENCE =====");
  
  // Forward test
  Serial.println("1. Testing FORWARD");
  moveForward(150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  // Backward test
  Serial.println("2. Testing BACKWARD");
  moveBackward(150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  // Left turn test
  Serial.println("3. Testing LEFT TURN");
  turnLeft(150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  // Right turn test
  Serial.println("4. Testing RIGHT TURN");
  turnRight(150);
  delay(2000);
  stopMotors();
  
  Serial.println("===== TEST COMPLETE =====");
}

void setup() {
  // Initialize Serial
  Serial.begin(9600);
  delay(2000);
  Serial.println("Robot with color sensor ready");
  
  // Disable interrupts during setup
  cli();

  // Setup color sensor pins
  S0_DDR |= (1 << S0_BIT);
  S1_DDR |= (1 << S1_BIT);
  S2_DDR |= (1 << S2_BIT);
  S3_DDR |= (1 << S3_BIT);
  
  SENSOR_DDR &= ~(1 << SENSOR_BIT);
  
  S0_PORT |= (1 << S0_BIT);
  S1_PORT |= (1 << S1_BIT);

  // Set up pin interrupts carefully
  EICRA = 0b01010100; // INT1 on any change, INT2 & INT3 on signal change
  //EIMSK = 0b00001110; // Enable INT1, INT2 & INT3 interrupts
  
  // Set encoder pins (PD2 and PD3) to input with pullups enabled
  DDRD &= ~0b00001100;
  PORTD |= 0b00001100;

  // Set servo pins (PL4 and PL5) to output
  DDRL |= 0b00110000;

  // Set up timersjh
  setupTimer1(); // For color sensor
  setupTimer5(); // For servos

  // Re-enable interrupts
  sei();
  
  // Initialize motors
  right.run(RELEASE);
  left.run(RELEASE);
  right.setSpeed(0);
  left.setSpeed(0);
  
  Serial.println("Type commands followed by Enter:");
  Serial.println("F,speed - Move forward");
  Serial.println("B,speed - Move backward");
  Serial.println("L,speed - Turn left");
  Serial.println("R,speed - Turn right");
  Serial.println("S - Stop");
  Serial.println("V,pos1,pos2 - Set servos");
  Serial.println("E - Read encoders");
  Serial.println("C - Clear encoders");
  Serial.println("D - Detect color");
  Serial.println("T - Test motors");
}

void loop() {
  // Check for serial commands
  readSerialCommands();
  delay(10);
}