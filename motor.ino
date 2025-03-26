#include <AFMotor.h>

// Motor setup
AF_DCMotor right(1), left(4);

// Command variables
const int MAX_COMMAND_LENGTH = 32;
char command[MAX_COMMAND_LENGTH];
int commandIndex = 0;

// Encoder variables
volatile unsigned long leftTicks = 0, rightTicks = 0;

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

ISR(INT2_vect) {
  leftTicks++;
}

ISR(INT3_vect) {
  rightTicks++;
}

void setup() {
  // Initialize Serial with higher baud rate
  Serial.begin(9600);
  
  // Wait for serial connection to establish (important for USB-based Arduinos)
  delay(1000);
  
  Serial.println("Robot ready for commands");
  
  cli();

  //set up encoder pin interrupts
  EICRA = 0b01010000; //fire interrupt on signal change
  EIMSK = 0b00001100; //enable INT2 & INT3 interrupts
  
  //set encoder pins (PD2 and PD3) to input with pullups enabled
  DDRD &= ~0b00001100;
  PORTD |= 0b00001100;

  //set servo pins (PL4 and PL5) to output
  DDRL |= 0b00110000;

    // Set up Timer 5 for servo control
  TCCR5A = 0b00101000;  //Clock Prescaler: 8
  TCCR5B = 0b00010010;  //Mode: Phase Correct PWM, TOP is ICR5
  ICR5 = 20000;
  OCR5B = 1500;
  OCR5C = 1500;

  sei();
  
  // Initialize motors - ensure stopped
  right.run(RELEASE);
  left.run(RELEASE);
  stop();
  
  Serial.println("Type commands followed by Enter. Example: F,150");
}

void loop() {
  // Check for serial commands
  readSerialCommands();
  delay(10); // Small delay to avoid hogging CPU
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Echo characters back for debugging
    Serial.write(c);
    
    // Process command when newline is received
    if (c == '\n' || c == '\r') {
      if (commandIndex > 0) { // Only process non-empty commands
        command[commandIndex] = '\0'; // Null terminate
        Serial.print("Processing command: ");
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
  // Make a copy of the command for strtok (since it modifies the string)
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
  
  Serial.print("Command: ");
  Serial.print(commandType);
  Serial.print(" Value1: ");
  Serial.print(value1);
  Serial.print(" Value2: ");
  Serial.println(value2);
  
  // Process based on command type
  switch (commandType) {
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
      
    default:
      Serial.print("Unknown command: ");
      Serial.println(commandType);
      break;
  }
}