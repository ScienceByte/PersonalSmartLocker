unsigned long previousSerialMillis = 0;
const long serialInterval = 4000; // Print, and check if it's obstructed that often too
int obstructionThreshold = 100;


const int maxPulseNum = 20;
int curPulseNum = 0;

// SERVO: Define the pin for where the servo plugs in
const int outputPinServo = 9;

// SERVO: Define the time intervals in microseconds for the pulses for servo control
const unsigned long pulseOpenHighTime = 2000;
const unsigned long pulseOpenLowTime = 18000;
const unsigned long pulseLockHighTime = 600;
const unsigned long pulseLockLowTime = 19400;

// SERVO: Define the delay interval in milliseconds between pulse sequences for servo control
const unsigned long delayTime = 1000;

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
int servoIntendedState = 0;
//SERVO: checks if obstructed. used to keep track of what servotargetstate it should return to
bool isObstructed = false;

// SERVO: Variable to hold the current SERVOstate
// Let's start in the locked position.
ServoState currentServoState = HOLDING_LOCKED;

// SERVO: store the last time an event happened
unsigned long previousMicros = 0;
unsigned long previousMillis = 0;

// for DEMONSTRATION: timer to toggle the servo in demonstration
unsigned long previousToggleMillis = 0;

int anval;

void setup() {
  // Set the Servo output pin as an output
  Serial.begin(115200);
  pinMode(outputPinServo, OUTPUT);
  pinMode(A0, INPUT);
  // Initialize timers
  previousMicros = micros();
  previousMillis = millis();

  lockServo();
}

void loop() {
  //NON-BLOCKING SERIAL PRINT
  if (millis() - previousSerialMillis >= serialInterval) {
    previousSerialMillis = millis(); // Reset the print timer
    // anval = analogRead(A0);
    // Serial.println(anval);
    //obstructionReturn();
  }


  // DEMONSTRATION ~~
  // This is an example of openServo() and lockServo() being called
  // to be replaced with password checking logic.
  if (millis() - previousToggleMillis >= delayTime) { // 
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
        curPulseNum = 0;
      }
      else{

        if(curPulseNum < maxPulseNum){
          curPulseNum++;
          currentServoState = PULSE_OPEN_HIGH;
          previousMicros = currentMicros;
        }

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
        curPulseNum = 0;
      }
      else{
        if(curPulseNum < maxPulseNum){
          curPulseNum++;
          currentServoState = PULSE_LOCK_HIGH;
          previousMicros = currentMicros;
        }
      }
    break;
  }
}

// This function sets the TARGET for the state machine.
void lockServo() {
  //pulseCounter = 50;
  servoTargetState = 0;
  servoIntendedState = 0;
}

// This function sets the TARGET for the state machine.
void openServo() {
  //pulseCounter = 50;
  servoTargetState = 1;
  servoIntendedState = 1;
}

void obstructionReturn() {
  anval = analogRead(A0);
  Serial.println(anval);

  // Check if an obstruction has just appeared
  if (anval > obstructionThreshold && !isObstructed) {
    isObstructed = true; // Set the flag so this only runs once
    Serial.println("Obstruction Detected! Reversing.");
    
    // Reverse the current target state
    if (servoTargetState == 1) {
      servoTargetState = 0; // If moving open, now move to lock
    } else {
      servoTargetState = 1; // If moving to lock, now move to open
    }
  } 
  // Check if an obstruction has just been removed
  else if (anval <= obstructionThreshold && isObstructed) {
    isObstructed = false; // Clear the flag
    Serial.println("Obstruction Cleared. Returning to original position.");
    
    // Restore the target state to the original intended state
    servoTargetState = servoIntendedState;
  }
}  