//Servo Set-up___________________________________________
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
  {4.900, 4.910},   // '1' 
  {4.840, 4.850},   // '2' 
  {4.730, 4.736},   // '3' 
  {4.795, 4.800},   // '4' 
  {4.741, 4.746},   // '5' 
  {4.620, 4.638},   // '6' 
  {4.638, 4.655},   // '7' 
  {4.584, 4.600},   // '8' 
  {4.480, 4.492},   // '9' 
  {4.420, 4.440},   // '*' 
  {4.365, 4.400},   // '0' 
  {4.270, 4.300}    // '#' 
};

// These variables are added to replace delay() with a non-blocking timer.
unsigned long lastKeypressMillis = 0;
const unsigned long KEYPAD_DEBOUNCE_DELAY = 300;

void setup() {
  Serial.begin(9600);
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

  //Set anVal = 0
  float anval = 0;
}

//Servo Functions_____________________________________________________
  // This function sets the TARGET for the state machine.
  void lockServo() {
    servoTargetState = 0;
    if (currentServoState == HOLDING_LOCKED)
    {
      currentServoState = PULSE_LOCK_HIGH;
      previousMicros = micros();
    }
  }

  // This function sets the TARGET for the state machine.
  void openServo() {
    servoTargetState = 1;
    if (currentServoState == HOLDING_LOCKED)
    {
      currentServoState = PULSE_OPEN_HIGH;
      previousMicros = micros();
    }
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
      else{
        currentServoState = PULSE_OPEN_HIGH;
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
      else{
        currentServoState = PULSE_LOCK_HIGH;
        previousMicros = currentMicros;
      }
    break;
  }
  obstructionReturn();
}