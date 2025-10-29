//Servo Set-up___________________________________________
unsigned long previousSerialMillis = 0;
const long serialInterval = 200; // Print every 1000 ms, and check if it's obstructed that often too
int obstructionThreshold = 70;
// SERVO: 0 means we want to be locked, 1 means we want to be open.
int servoIntendedState = 0;
bool isObstructed = false;
int anval;

const int maxPulseNum = 50;
int curPulseNum = 0;


// SERVO: Define the pin for where the servo plugs in
const int outputPinServo = 9;

// SERVO: Define the time intervals in microseconds for the pulses for servo control
const unsigned long pulseOpenHighTime = 2000;
const unsigned long pulseOpenLowTime = 18000;
const unsigned long pulseLockHighTime = 600;
const unsigned long pulseLockLowTime = 19400;

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

//KeyPad Setup_____________________________________
  // Store each number pressed
char passInput[5];
int input = 0;
char approvedPass[5];
bool passwordSet = false;

// Key mapping for the 4x3 keypad
char KEYS[] = { '1','2','3','4','5','6','7','8','9','*','0','#' };

// Voltage ranges
const double voltages[][2] = {
  {4.05, 4.15},   // '1' 
  {3.75, 3.8},   // '2' 
  {3.00, 3.18},   // '3' 
  {3.40, 3.50},   // '4' 
  {3.19, 3.25},   // '5' 
  {2.75, 2.78},   // '6' 
  {2.79, 2.85},   // '7' 
  {2.55, 2.65},   // '8' 
  {2.30, 2.40},   // '9' 
  {2.15, 2.20},   // '*' 
  {2.00, 2.10},   // '0' 
  {1.80, 1.90}    // '#' 
};

// These variables are added to replace delay() with a non-blocking timer.
unsigned long lastKeypressMillis = 0;
const unsigned long KEYPAD_DEBOUNCE_DELAY = 300;

void setup() {
  Serial.begin(115200);
  //Keypad input_________________________________
  //Prompts the user to input password
  Serial.println("Set passcode: "); 

  //Servo Motor__________________________________
  // Set the Servo output pin as an output
  pinMode(outputPinServo, OUTPUT);

  pinMode(A0, INPUT); //Set pin A0, as arduino side of stunt resistor
  // pinMode(A1, INPUT); //Set pin A1, as ground side of stunt resistor

  // Initialize timers
  previousMicros = micros();
  previousMillis = millis();

  lockServo();
}

//Servo Functions_____________________________________________________
  // This function sets the TARGET for the state machine.
  void lockServo() {
    servoTargetState = 0;
    servoIntendedState = 0;
    Serial.println("lock servo");
 //   if (currentServoState == HOLDING_LOCKED)
   // {
//      currentServoState = PULSE_LOCK_HIGH;
//      previousMicros = micros();
 //   }
  }

  // This function sets the TARGET for the state machine.
  void openServo() {
    servoTargetState = 1;
    servoIntendedState = 1;
    Serial.println("open servo");

 //   if (currentServoState == HOLDING_LOCKED)
 //   {
 //     currentServoState = PULSE_OPEN_HIGH;
 //     previousMicros = micros();
 //   }
  }

void obstructionReturn() {
  anval = analogRead(A0);
  //Serial.println(anval);

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

//EEPROM Functions_________________________________________________
unsigned char EEPROM_read(unsigned int uiAddress) { 
  /* Wait for completion of previous write */ 
  while(EECR & (1<<EEPE)) ; 
  /* Set up address register */ 
  EEAR = uiAddress; 
  /* Start eeprom read by writing EERE */ 
  EECR |= (1<<EERE); 
  /* Return data from Data Register */ 
  return EEDR; 
}

void EEPROM_write(unsigned int uiAddress, unsigned char ucData) {
  /* Wait for completion of previous write */ while(EECR & (1<<EEPE)) ; 
  /* Set up address and Data Registers */ 
  EEAR = uiAddress; EEDR = ucData; 

  cli(); //disable interrupts
  /* Write logical one to EEMPE */ 
  EECR |= (1<<EEMPE); 
  /* Start eeprom write by setting EEPE */ 
  EECR |= (1<<EEPE); 
  sei(); //enable interrupts
}

void loop() {
  // This check ensures the keypad is only read if 300ms have passed since the last press.

  if (millis() - lastKeypressMillis > KEYPAD_DEBOUNCE_DELAY) {
    int keyPressed = analogRead(A5);
    double voltage = keyPressed * (5.0 / 1023.0);
    //Serial.println(voltage);

    // Find which key matches the measured voltage
    for (int j = 0; j < 12; j++) {
      if (voltage >= voltages[j][0] && voltage <= voltages[j][1]) {
        passInput[input] = KEYS[j];
        input++;
        Serial.print("Key pressed: ");
        Serial.println(KEYS[j]);
        // The delay(300) is replaced by resetting the timer.
        lastKeypressMillis = millis();

        if (input == 4) {
          passInput[4] = '\0';
          Serial.print("Entered passcode: ");
          Serial.println(passInput);

          if (!passwordSet) {
            // Save the password to EEPROM
            for (int i = 0; i < 4; i++) {
              EEPROM_write(i, passInput[i]);
            }
            passwordSet = true;
            Serial.println("Password is saved");
          } else {
            // Check password
            bool correct = true;
            for (int i = 0; i < 4; i++) {
              if (passInput[i] != EEPROM_read(i)) {
                correct = false;
                break;
              }
            }
            if (correct) {
              Serial.println("Correct Password");
              openServo();
            } else {
              Serial.println("Incorrect Password");
              lockServo();
            }
          }

          input = 0;  // reset for next entry
        }
        break; // Exit the for-loop once a key is found
      }
    }
  }

  // The servo state machine has to be part of this loop here.
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();
    switch (currentServoState) {
     // Serial.println(curPulseNum);
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

  if (millis() - previousSerialMillis >= serialInterval) {
    previousSerialMillis = millis(); // Reset the print timer
     anval = analogRead(A0);
     Serial.println(anval);
    obstructionReturn();
  }
}

attachInterrupt(digitalPinInterrupt(interruptPin), powerDownMode, CHANGE);
//Use interupts and test it by hooking up the arduino again and printing the statement if the loop is true
void powerDownMode()
{
  Serial.println("Going to sleep"); //prints the statement when it calls this function
  SMCR = (1 << SE);  //Enables sleep mode by setting bit 0 of SMCR to 1
  SMCR |= (1 << SM1);  //Sets bit 2 of SMCR to 1

  EICRA |= (1 << ISC00); //Sets bit 0 of EICRA to 1
  EICRA &= ~(1 << ISC01); //Sets bit 1 of EICRA to 0
  EIMSK |= (1 << INT0); //Enables INT0
}