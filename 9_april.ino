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

volatile unsigned long lastSendTime = 0;
volatile uint8_t receivedData = 0;
const unsigned long SEND_INTERVAL = 1000;  // 1 second

AF_DCMotor br(1), bl(2), fr(4), fl(3);

void translate(bool direction, int speed) {
  br.run(direction ? FORWARD : BACKWARD);
  bl.run(direction ? BACKWARD : FORWARD);
  fr.run(direction ? FORWARD : BACKWARD);
  fl.run(direction ? BACKWARD : FORWARD);
  br.setSpeed(speed);
  bl.setSpeed(speed);
  fr.setSpeed(speed);
  fl.setSpeed(speed);
}

void rotate(bool direction, int speed) {
  br.run(direction ? FORWARD : BACKWARD);
  bl.run(direction ? FORWARD : BACKWARD);
  fr.run(direction ? FORWARD : BACKWARD);
  fl.run(direction ? FORWARD : BACKWARD);
  br.setSpeed(speed);
  bl.setSpeed(speed);
  fr.setSpeed(speed);
  fl.setSpeed(speed);
}

void stop() {
  br.setSpeed(0);
  bl.setSpeed(0);
  fr.setSpeed(0);
  fl.setSpeed(0);
}

volatile unsigned int lastEdge, pulse, green, red, state, edges;

ISR(INT1_vect) {
  unsigned int currentEdge = 2000;
  long delta = currentEdge - lastEdge;

  if (delta < 0)
    pulse = delta + 0xFFFF;
  else
    pulse = delta;

  lastEdge = currentEdge;
  edges++;
}

ISR(TIMER4_COMPA_vect) {
  switch (state) {
    case 0:
      if (edges >= 3)
        green = pulse;
      else
        green = 3000;
      setFilter(1, 1);
      break;

    case 1:
      if (edges >= 3)
        red = pulse;
      else
        red = 3000;
      setFilter(0, 0);
      break;
  }

  edges = 0;
  state = (state + 1) % 2;
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
}

uint8_t getColourStatus() {
  const unsigned int threshold = 500;  // Adjust threshold as needed
  // Lower pulse value means stronger detection.
  if ((green < threshold) && (red < threshold) && (green < red)) {
    return 0b01;  // Green detected
  } else if ((green < threshold) && (red < threshold) && (red < green)) {
    return 0b00;  // Red detected
  } else {
    return 0b10;  // Black or no significant color detected
  }
}

void setup() {
  cli();

  //set up col. sensor pin interrupt
  EICRA = 0b00000100;  //fire interrupt on signal change
  EIMSK = 0b00000010;  //enable INT1 interrupt

  //set servo pins (PL3, PL4, PL5) to output
  DDRL |= 0b00111000;

  S0_DDR |= (1 << S0_BIT);
  S1_DDR |= (1 << S1_BIT);
  S2_DDR |= (1 << S2_BIT);
  S3_DDR |= (1 << S3_BIT);
  SENSOR_DDR &= ~(1 << SENSOR_BIT);
  S0_PORT |= (1 << S0_BIT);
  S1_PORT |= (1 << S1_BIT);

  //set Timer4 to trigger every 10ms
  TCCR4A = 0b00000000;
  TCCR4B = 0b00001010;
  TIMSK4 = 0b00000010;
  OCR4A = 20000;
  TCNT4 = 0;

  //set Timer5 interrupts for PWM pins in Phase-Correct PWM Mode
  TCCR5A = 0b10101000;  //Clock Prescaler: 8
  TCCR5B = 0b00010010;  //Mode: Phase Correct PWM, TOP is ICR5
  ICR5 = 20000;
  OCR5A = 1500;  //Medpack Servo
  OCR5B = 1500;  //Left Claw Arm
  OCR5C = 1500;  //Right Claw Arm

  sei();

  setFilter(0, 0);
  Serial.begin(115200);
  Serial2.begin(115200);

  // openClaw();
  // keepMedpack();
}

void openClaw() {
  OCR5B = 1100;
  OCR5C = 2000;
}

void closeClaw() {
  OCR5B = 1600;
  OCR5C = 1500;
}

void keepMedpack() {
  OCR5A = 1650;
}


void releaseMedpack() {
  OCR5A = 1050;
}

void loop() {
  // while (1) {
  // Serial.print(green);
  // Serial.print(' ');
  // Serial.println(red);
  // Serial.println(getColourStatus());
  // }

  // while (!Serial2.available()) {};
  // uint8_t cmd = Serial2.read();

  // Serial.println(cmd);

  int cmd = 0;
  switch (cmd) {
    // case 0:
    //   releaseMedpack();
    //   break;
    case 1:
      translate(true, 255);
      break;
    case 2:
      stop();
      break;
    case 3:
      rotate(true, 255);
      break;
    case 4:
      rotate(false, 255);
      break;
    case 5:
      translate(false, 255);
      break;
    // case 6:
    //   openClaw();
    //   break;
    // case 7:
    //   closeClaw();
    //   break;
    default:
      break;
  }

  // uint8_t colorResponse = getColourStatus();    // lower 2 bits indicate sensor state
  // uint8_t responseByte = colorResponse & 0x03;  // mask to only 2 bits

  // Serial2.write(colorResponse);
}