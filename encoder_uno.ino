#include <PinChangeInt.h>




// Define the pins for the first encoder
const int encoder1PinA = 2;
const int encoder1PinB = 3;

// Define the pins for the second encoder
const int encoder2PinA = 4;
const int encoder2PinB = 5;

// Variables to store encoder values
volatile int encoder1Value = 0;
volatile int encoder2Value = 0;

// Variables to store last state of the encoder pins
volatile int encoder1LastStateA = LOW;
volatile int encoder1LastStateB = LOW;
volatile int encoder2LastStateA = LOW;
volatile int encoder2LastStateB = LOW;

void setup() {
  // Set encoder pins as INPUT_PULLUP
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);

  // Attach PinChange interrupts for the encoder pins
  PCintPort::attachInterrupt(encoder1PinA, encoder1ISR, CHANGE);
  PCintPort::attachInterrupt(encoder1PinB, encoder1ISR, CHANGE);
  PCintPort::attachInterrupt(encoder2PinA, encoder2ISR, CHANGE);
  PCintPort::attachInterrupt(encoder2PinB, encoder2ISR, CHANGE);

  // Initialize Serial communication
  Serial.begin(9600);
}

void loop() {
  // Main loop, you can add your code here

  // Print encoder values to Serial monitor
  Serial.print("Encoder 1: ");
  Serial.print(encoder1Value);
  Serial.print("\tEncoder 2: ");
  Serial.println(digitalRead(encoder1PinB));

  delay(100); // Adjust the delay as needed
}

// Interrupt service routine for encoder 1
void encoder1ISR() {
  int currentStateA = digitalRead(encoder1PinA);
  int currentStateB = digitalRead(encoder1PinB);

  if (currentStateA != encoder1LastStateA || currentStateB != encoder1LastStateB) {
    if (currentStateA == currentStateB) {
      encoder1Value++;
    } else {
      encoder1Value--;
    }
    encoder1LastStateA = currentStateA;
    encoder1LastStateB = currentStateB;
  }
}

// Interrupt service routine for encoder 2
void encoder2ISR() {
  int currentStateA = digitalRead(encoder2PinA);
  int currentStateB = digitalRead(encoder2PinB);

  if (currentStateA != encoder2LastStateA || currentStateB != encoder2LastStateB) {
    if (currentStateA == currentStateB) {
      encoder2Value++;
    } else {
      encoder2Value--;
    }
    encoder2LastStateA = currentStateA;
    encoder2LastStateB = currentStateB;
  }
}
