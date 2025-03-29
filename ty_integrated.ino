#include <AFMotor.h>

#define S0_PORT PORTL
#define S0_DDR DDRL
#define S0_BIT 6

#define S1_PORT PORTG
#define S1_DDR DDRG
#define S1_BIT 0

#define S2_PORT PORTC
#define S2_DDR DDRC
#define S2_BIT 0

#define S3_PORT PORTG
#define S3_DDR DDRG
#define S3_BIT 2

#define SENSOR_PORT PORTD
#define SENSOR_DDR DDRD
#define SENSOR_PIN PIND
#define SENSOR_BIT 1

AF_DCMotor right(1), left(4);

#define DEFAULT_SPEED 150

bool isClawOpen = false; // Assume starts closed

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

volatile unsigned int lastEdge, pulse, green, red, state, edges;

ISR(INT1_vect) {
  unsigned int currentEdge = TCNT4;
  long delta = currentEdge - lastEdge;

  if (delta < 0)
    pulse = delta + 0xFFFF;
  else
    pulse = delta;

  lastEdge = currentEdge;
  edges++;
}

ISR(TIMER3_COMPA_vect) {
  bool greenStatusUpdated = false;
  switch (state) {
    case 0: // Just finished reading red, starting green measurement
      if (edges >= 3)
        red = pulse;
      else
        red = 20000;
      setFilter(1, 1);
           break;

    case 1: 
      if (edges >= 3)
        green = pulse;
      else
        green = 20000;
      setFilter(0, 0); // Set filter for RED
      greenStatusUpdated = true; // Green/Red values are now updated
      break;
  }

  edges = 0;
  state = (state + 1) % 2;

  // Send status update AFTER both colors have been measured (when state flips back to 0)
  if (greenStatusUpdated) {
    bool isGreenDominant = (green < red);
    Serial3.write(isGreenDominant ? '1' : '0'); // Send '1' for green, '0' for red/equal
  }
}

// volatile unsigned long leftTicks = 0, rightTicks = 0;

// ISR(INT2_vect)
// {
//   leftTicks++;
// }

// ISR(INT3_vect)
// {
//   rightTicks++;
// }

void setFilter(bool s2, bool s3) {
  if (s2)
    S2_PORT |= (1 << S2_BIT);
  else
    S2_PORT &= ~(1 << S2_BIT);

  if (s3)
    S3_PORT |= (1 << S3_BIT);
  else
    S3_PORT &= ~(1 << S3_BIT);
}

void setup() {
  cli();

  //set up encoder and col. sensor pin interrupts
  // EICRA = 0b01010100; //fire interrupt on signal change
  // EIMSK = 0b00001110; //enable INT1, INT2, INT3

  //set up col. sensor pin interrupt
  EICRA = 0b00000100;  //fire interrupt on signal change
  EIMSK = 0b00000010;  //enable INT1 interrupt

  //set encoder pins (PD2 and PD3) to input with pullups enabled
  // DDRD &= ~0b00001100;
  // PORTD |= 0b00001100;

  //set servo pins (PL3, PL4, PL5) to output
  DDRL |= 0b00111000;

  S0_DDR |= (1 << S0_BIT);
  S1_DDR |= (1 << S1_BIT);
  S2_DDR |= (1 << S2_BIT);
  S3_DDR |= (1 << S3_BIT);
  SENSOR_DDR &= ~(1 << SENSOR_BIT);
  S0_PORT |= (1 << S0_BIT);
  S1_PORT |= (1 << S1_BIT);

  //set Timer3 to trigger every 10ms
  TCCR3A = 0b00000000;
  TCCR3B = 0b00001010;
  TIMSK3 = 0b00000011;
  OCR3A = 20000;
  TCNT3 = 0;

  //set Timer4 to count at 2Mhz
  TCCR4A = 0b00000000;
  TCCR4B = 0b00000010;
  TCNT4 = 0;

  //set Timer5 interrupts for PWM pins in Phase-Correct PWM Mode
  TCCR5A = 0b10101000;  //Clock Prescaler: 8
  TCCR5B = 0b00010010;  //Mode: Phase Correct PWM, TOP is ICR5
  ICR5 = 20000;
  OCR5A = 1500;   //Medpack Servo
  OCR5B = 1500;   //Left Claw Arm
  OCR5C = 1500;   //Right Claw Arm

  //Activate Serial RX Interrupts (7N1)


  //Activate Serial TX Interrupts (Transmit Colour Sensor data)

  // Setup Serial3 for Raspberry Pi communication
  // Baud rate: 9600 (ensure RPi matches)
  // Config: SERIAL_7N1 means 7 data bits, No parity, 1 stop bit
  Serial3.begin(9600, SERIAL_7N1);

  // Keep Serial (USB) for debugging if needed
  Serial.begin(9600); 

  sei();

  Serial.println("Setup Complete. Serial3 configured for 7N1 communication.");
  // Initialize claw to a known state (e.g., closed)
  closeClaw();
  isClawOpen = false;
}

void openClaw() {
  OCR5B = 1000;
  OCR5C = 2000;
}

void closeClaw() {
  OCR5B = 1420;
  OCR5C = 1580;
}

void keepMedpack() {
  OCR5A = 1650;
}

void releaseMedpack() {
  OCR5A = 1050;
}

void loop() {
  // Check for incoming commands from Raspberry Pi via Serial3
  if (Serial3.available() > 0) {
    // Read the incoming byte (7 data bits)
    byte receivedData = Serial3.read();

    // Extract the lower 3 bits for the command
    // Use bitwise AND with 0x07 (binary 00000111)
    byte command = receivedData & 0x07;

    // Debugging: Print received data and command
    // Serial.print("Received Byte: "); Serial.print(receivedData, BIN);
    // Serial.print(" | Extracted Command: "); Serial.println(command, BIN);

    // Execute command based on the 3-bit value
    switch (command) {
      case 0: // 000: Stop
        Serial.println("Command: Stop");
        stop();
        break;
      case 1: // 001: Forward
        Serial.println("Command: Forward");
        translate(true, DEFAULT_SPEED); // true for FORWARD
        break;
      case 2: // 010: Reverse
        Serial.println("Command: Reverse");
        translate(false, DEFAULT_SPEED); // false for BACKWARD
        break;
      case 3: // 011: Left
        Serial.println("Command: Left");
        rotate(false, DEFAULT_SPEED); // false for Left turn
        break;
      case 4: // 100: Right
        Serial.println("Command: Right");
        rotate(true, DEFAULT_SPEED); // true for Right turn
        break;
      case 5: // 101: Toggle Claw
        Serial.println("Command: Toggle Claw");
        if (isClawOpen) {
          closeClaw();
          isClawOpen = false;
        } else {
          openClaw();
          isClawOpen = true;
        }
        break;
     
      default:
        
        Serial.print("Unknown command: "); Serial.println(command);
        break;
    }
  }

  // keepMedpack();
  // delay(1000);
  // releaseMedpack();
  // delay(1000);

  // openClaw();
  // delay(2000);
  // closeClaw();
  // delay(2000);
}
