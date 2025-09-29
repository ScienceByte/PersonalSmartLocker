// SERVO: Define the pin for where the servo plugs in
const int outputPinServo = 9;

// SERVO: Define the time intervals in microseconds for the pulses for servo control
const unsigned long pulseOpenHighTime = 2500;
const unsigned long pulseOpenLowTime = 17500;
const unsigned long pulseLockHighTime = 500;
const unsigned long pulseLockLowTime = 19500;

// SERVO: Define the delay interval in milliseconds between pulse sequences for servo control
const unsigned long delayTime = 500;

// SERVOstate machine FOR PWM, using enums for that.
enum ServoState {
  PULSE_OPEN_HIGH,
  PULSE_OPEN_LOW,
  HOLDING_OPEN, 
  PULSE_LOCK_HIGH,
  PULSE_LOCK_LOW,
  HOLDING_LOCKED  
};

// SERVO: 0 means we want to be locked, 1 means we want to be open.
int servoTargetState = 0; 

// SERVO: Variable to hold the current SERVOstate
// Let's start in the locked position.
ServoState currentServoState = HOLDING_LOCKED;

// SERVO: store the last time an event happened
unsigned long previousMicros = 0;
unsigned long previousMillis = 0;

// for DEMONSTRATION: timer to toggle the servo in demonstration
unsigned long previousToggleMillis = 0;

void setup() {
  // Set the Servo output pin as an output
  pinMode(outputPinServo, OUTPUT);
  // Initialize timers
  previousMicros = micros();
  previousMillis = millis();
}

void loop() {
  // DEMONSTRATION ~~
  // This is an example of openServo() and lockServo() being called
  // to be replaced with password checking logic.
  if (millis() - previousToggleMillis >= 500) { // 
    if (servoTargetState == 0) {
      openServo(); // Tell the servo to open
    } else {
      lockServo(); // Tell the servo to lock
    }
    previousToggleMillis = millis(); // Reset the toggle timer
  }
  // END OF DEMONSTRATION ~~


  // The servo state machine has to be part of this loop here.
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();

  switch (currentServoState) {
    case PULSE_OPEN_HIGH:
      digitalWrite(outputPinServo, HIGH);
      if (currentMicros - previousMicros >= pulseOpenHighTime) {
        currentServoState = PULSE_OPEN_LOW;
        previousMicros = currentMicros;
      }
      break;

    case PULSE_OPEN_LOW:
      digitalWrite(outputPinServo, LOW);
      if (currentMicros - previousMicros >= pulseOpenLowTime) {
        currentServoState = HOLDING_OPEN; // Now hold this position
        previousMillis = currentMillis; 
      }
      break;

    case HOLDING_OPEN: //delay
      if (servoTargetState == 0) { // check if the command is to lock
        currentServoState = PULSE_LOCK_HIGH; //will go do that ^
        previousMicros = currentMicros; 
      }
      break;

    case PULSE_LOCK_HIGH:
      digitalWrite(outputPinServo, HIGH);
      if (currentMicros - previousMicros >= pulseLockHighTime) {
        currentServoState = PULSE_LOCK_LOW;
        previousMicros = currentMicros;
      }
      break;

    case PULSE_LOCK_LOW:
      digitalWrite(outputPinServo, LOW);
      if (currentMicros - previousMicros >= pulseLockLowTime) {
        currentServoState = HOLDING_LOCKED; // Now hold this position
        previousMillis = currentMillis; 
      }
      break;

    case HOLDING_LOCKED: //delay
      if (servoTargetState == 1) { // check if the command is to open.
        currentServoState = PULSE_OPEN_HIGH; // will go do that ^
        previousMicros = currentMicros; 
      }
      break;
  }
}

// This function sets the TARGET for the state machine.
void lockServo() {
  servoTargetState = 0;
}

// This function sets the TARGET for the state machine.
void openServo() {
  servoTargetState = 1;
}
