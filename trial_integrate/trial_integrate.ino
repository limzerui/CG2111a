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
const unsigned long SEND_INTERVAL = 1000; // 1 second


AF_DCMotor right(1), left(4);

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
  switch (state) {
    case 0:
      if (edges >= 3)
        green = pulse;
      else
        green = 20000;
      setFilter(1,1);
      break;

    case 1:
      if (edges >= 3)
        red = pulse;
      else
        red = 20000;
      setFilter(0, 0);
      break;
  }

  edges = 0;
  state = (state + 1) % 2;
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

ISR(USART3_RX_vect){
  uint8_t cmd = UDR3;
  switch (cmd) {
    case 0x01:
      translate(true, 180);
      break;
    case 0x02:
      stop();
      break;
    case 0x03:
      rotate(true, 180);
      break;
    case 0x04:
      rotate(false, 180);
      break;
    case 0x05:
      translate(false, 180);
      break;
    default:
      // Unrecognized command – optionally handle error.
      break;
}
  uint8_t colorResponse = getColorStatus();  // lower 2 bits indicate sensor state
  uint8_t responseByte = colorResponse & 0x03; // mask to only 2 bits
  
  // Wait until the transmit buffer is empty, then send the response.
  while (!(UCSR3A & (1 << UDRE3))) ;  // Wait for UDRE flag
  UDR3 = responseByte;
}

uint8_t getColorStatus() {
  const unsigned int threshold = 18000; // Adjust threshold as needed
  // Lower pulse value means stronger detection.
  if ((green < threshold) && (green < red)) {
    return 0b01; // Green detected
  } else if ((red < threshold) && (red < green)) {
    return 0b00; // Red detected
  } else {
    return 0b10; // Black or no significant color detected
  }
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

  UCSR3B |= (1<<RXCIE3); //Enable RX interrupt
  //Serial3.begin(115200);
  
  //Activate Serial TX Interrupts (Transmit Colour Sensor data)

  sei();
  Serial.begin(9600);

}


// void handleSerialSend() {
//   unsigned long currentTime = millis();
  
//   // Check if it's time to send data
//   if (currentTime - lastSendTime >= SEND_INTERVAL) {
//     Serial3.write(1);
//     Serial.println("Sent '1' via Serial3");
//     lastSendTime = currentTime;
//   }
// }

// Function to check for incoming data - call this in your loop()
// void handleSerialReceive() {
//   if (Serial3.available() > 0) {
//     // Read the incoming byte
//     receivedData = Serial3.read();
    
//     // Print to main serial for debugging
//     Serial.print("Received data: ");
//     Serial.println(receivedData + '0');
//   }
// }

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

  bool isGreen = green > red;
  // handleSerialSend();
  // handleSerialReceive();


  // OCR5A = 1000;
  // OCR5B = 1000;
  // OCR5C = 1000;
  // delay(2000);

  // OCR5A = 2000;
  // OCR5B = 2000;
  // OCR5C = 2000;
  // delay(2000);

  // if (Serial.available() >= 3)
  // {
  //   int speed = (Serial.read() - '0') * 100;
  //   speed += (Serial.read() - '0') * 10;
  //   speed += (Serial.read() - '0') * 1;

  //   translate(1, speed);
  // }

  // keepMedpack();
  // delay(1000);
  // releaseMedpack();
  // delay(1000);

  // openClaw();
  // delay(2000);
  // closeClaw();
  // delay(2000);
}
