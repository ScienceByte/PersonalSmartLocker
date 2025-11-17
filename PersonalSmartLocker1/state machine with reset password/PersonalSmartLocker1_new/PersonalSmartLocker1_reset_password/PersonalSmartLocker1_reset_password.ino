//used to initialize pins that control the RED, YELLOW, and GREEN LEDs
int POWER_LED_PIN = 8;//POWER_LED_PIN 8
int YELLOW_LED_PIN = 7; //YELLOW_LED_PIN 7
int GRN_LED_PIN = 6; //GRN_LED_PIN 6

//Pin to send signal to transistor to turn on and off
int Transistor_Pin = 10;
const unsigned long TransistorLowTime = 10000;
const unsigned long TransistorHighTime = 5000;
unsigned long currentTransistorMillis = millis();
unsigned long previousTransistorMillis = millis();
bool Transistor_State = true;

bool yellowBlinkState = LOW; //used for the blinking of yellow LED
bool greenBlinkState = LOW; //used for green blinking

// states for LED status
enum LEDState {
  SET_PASSWORD,     // solid green and yellow
  IDLE,             // All lights off, waiting for input
  TYPING,           // yellow solid (flickers on keypress)
  CORRECT_PASSWORD, // solid green
  WRONG_PASSWORD    // yellow blinks rapidly
};

LEDState currentLEDstate = SET_PASSWORD; // Start in idle mode //PASSWORD RESET



//Servo Set-up___________________________________________
unsigned long previousSerialMillis = 0;
const long serialInterval = 200; // Print every 1000 ms, and check if it's obstructed that often too
int obstructionThreshold = 100;

// SERVO: 0 means we want to be locked, 1 means we want to be open.
int servoIntendedState = 0;
bool isObstructed = false;
int anval;

const int maxPulseNum = 50;
int curPulseNum = 0;

//SLEEP VARIABLES
const unsigned long SLEEP_TIMEOUT = 30000; // 30 seconds of no keypress
const int wakeUpPin = 2; // This is INT0 (Digital Pin 2)
volatile bool justWoke = false; // Flag to run code once on wake-up

//Password Setup
// bool reset = false;

// SERVO: Define the pin for where the servo plugs in
const int outputPinServo = 9;

// SERVO: Define the time intervals in microseconds for the pulses for servo control
const unsigned long pulseOpenHighTime =  1500;
const unsigned long pulseOpenLowTime = 18500;
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
bool passwordSet = false; //start with assuming it's set for //PASSWORD RESET

// Key mapping for the 4x3 keypad
char KEYS[] = { '1','2','3','4','5','6','7','8','9','*','0','#' };

// Voltage ranges
const double voltages[][2] = {
  {3.92, 4.15},   // '1' sometims it's 3.61
  {3.67, 3.8},   // '2' 
  {3.08, 3.11},   // '3' 
  {3.28, 3.50},   // '4' 
  {3.12, 3.14},   // '5' 
  {2.55, 2.67},   // '6' 
  {2.68, 2.72},   // '7' 
  {2.53, 2.56},   // '8' 
  {2.15, 2.25},   // '9' 
  {1.98, 2.07},   // '*' 
  {1.85, 1.97},   // '0' 
  {1.60, 1.80}    // '#' 
};

// These variables are added to replace delay() with a non-blocking timer.
unsigned long lastKeypressMillis = 0;
const unsigned long KEYPAD_DEBOUNCE_DELAY = 300;

void setup() {
  Serial.begin(115200);

  //Servo Motor__________________________________
  // Set the Servo output pin as an output
  pinMode(outputPinServo, OUTPUT);

  pinMode(A0, INPUT); //Set pin A0, as arduino side of stunt resistor
  // pinMode(A1, INPUT); //Set pin A1, as ground side of stunt resistor

  // Initialize timers
  previousMicros = micros();
  previousMillis = millis();

  //Initialize timer for transistor
  previousTransistorMillis = millis();

  //SLEEP mode pin. Digital Pin 2
  pinMode(wakeUpPin, INPUT); // Set D2 as an input, this is wired to a pushbutton which is wired with a pullup resistor.
  //SLEEP mode mask creating
  EICRA = EICRA & ~((1<<ISC01)|(1<<ISC00)); // interrupt on low level of INT0. since only level interrupt can be a wake up signal for INT0 anyway.
  //ISC00 and ISC01 should both be set to 0. this creates a mask for ISC01 and ISC00 and uses it to clear those two values

pinMode(A3, INPUT);

pinMode(POWER_LED_PIN, OUTPUT); //POWER_LED_PIN 8
pinMode(YELLOW_LED_PIN, OUTPUT); //YELLOW_LED_PIN 7
pinMode(GRN_LED_PIN, OUTPUT); //GRN_LED_PIN 6
pinMode(Transistor_Pin, OUTPUT); //Transistor pin

  //Keypad input_________________________________
  //Prompts the user to input password
  Serial.println("Ready to go. "); //PASSWORD RESET
  //turn on YELLOW and GREEN to signify it's set password mode
  digitalWrite(GRN_LED_PIN, HIGH); //PASSWORD RESET
  digitalWrite(YELLOW_LED_PIN, HIGH); //PASSWORD RESET

  //Test battery level to start
  digitalWrite(Transistor_Pin, HIGH);


  lockServo();
}

//SLEEP functions
ISR(INT0_vect){ 
  justWoke = true;
}

void goToSleep() {
  // Simple guard - if already LOW, abort
  // NOTE: This check happens BEFORE cli(), slightly less safe but simpler test
  if (digitalRead(wakeUpPin) == LOW) {
     Serial.println("Pin already LOW, aborting sleep cycle.");
     Serial.flush();
     // Reset flag just in case it got set erratically
     justWoke = false;
     return;
  }

  Serial.println("Entering Standby Mode...");
  Serial.flush();

  cli(); // Disable interrupts

  // Set sleep mode to STANDBY (110)
  SMCR = (SMCR & ~(1 << SM0)) | (1 << SM2) | (1 << SM1);

  EIMSK |= (1 << INT0); // Enable INT0

  do {
    SMCR |= (1 << SE); // Enable Sleep
    sei();             // Interrupts ON for one cycle
    __asm__ __volatile__("sleep" ::); // Sleep Now
    // --- WAKE UP --- (Interrupts OFF)
    SMCR &= ~(1 << SE); // Disable Sleep
  } while (0);

//haven't disabled EIMSK not immediately necessary
  sei(); // Re-enable interrupts
  Serial.println("...Woke up from sleep function."); // Debug message
  Serial.flush();
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

  if (justWoke) {
    Serial.println("Woke up.");
    
    // Reset the inactivity timer so we don't 
    // immediately go back to sleep.
    lastKeypressMillis = millis(); 
    
    // Reset the flag
    justWoke = false; 

      EIMSK &= ~(1<<INT0); // external interrupt disable (INT0)

  }

  if (millis() - lastKeypressMillis > KEYPAD_DEBOUNCE_DELAY) {

    int keyPressed = analogRead(A5);
    double voltage = keyPressed * (5.0 / 1023.0);
    Serial.println("voltage:");
    Serial.println(voltage);


    // Find which key matches the measured voltage
    for (int j = 0; j < 12; j++) {
      if (voltage >= voltages[j][0] && voltage <= voltages[j][1]) {
              
              char pressedKey = KEYS[j];    // Get the key that was pressed
              lastKeypressMillis = millis(); // Reset the non-blocking debounce timer
              
              // --- NEW LOGIC: Password Reset ---
              if (pressedKey == '*') {
                // Check if we are in the "unlocked" state
                if (currentLEDstate == CORRECT_PASSWORD) {
                  Serial.println("Password reset initiated.");
                  Serial.println("Please enter a new 4-digit password:");

                  // Change state to SET_PASSWORD
                  currentLEDstate = SET_PASSWORD;
                  passwordSet = false; // This flag will make the 4-digit logic save the new pass
                  input = 0;           // Reset the password input buffer

                  // Set LEDs to "set password" mode (solid green & yellow)
                  digitalWrite(GRN_LED_PIN, HIGH);
                  digitalWrite(YELLOW_LED_PIN, HIGH);
                }
                // Optional: Keep '*' as a "clear" button if typing
                else if (currentLEDstate == TYPING) {
                  Serial.println("Input cleared.");
                  input = 0; // Reset input buffer
                }
                // In any other state, '*' does nothing.
                
                break; // We're done handling the '*' press, exit the for-loop
              }
              
              // --- EXISTING LOGIC: Handle Digits (0-9, #) ---
              // This code only runs if the key was NOT '*'
              
              passInput[input] = pressedKey;
              input++;
              
              Serial.print("Key pressed: ");
              Serial.println(pressedKey);

              // If we were in the "Correct" (unlocked) state, pressing a new digit
              // (or being in SET_PASSWORD) should move us to the "Typing" state.
              currentLEDstate = TYPING; 
              digitalWrite(YELLOW_LED_PIN, HIGH); 
              
              if(passwordSet) { 
                // If passwordSet is true, we are *checking* a password.
                // Turn off the green "correct" light.
                digitalWrite(GRN_LED_PIN, LOW); 
              } 
              // If passwordSet is false (i.e., we are setting a new pass),
              // the green LED state is handled by the blinking logic
              // in the serialInterval check, which is correct.

              // Check if 4 digits have been entered
              if (input == 4) {
                passInput[4] = '\0'; // Null-terminate the string
                Serial.print("Entered passcode: ");
                Serial.println(passInput);

                if (!passwordSet) {
                  // --- SAVE NEW PASSWORD ---
                  // This block now runs for the *first-time* setup AND
                  // for the new password reset.
                  for (int i = 0; i < 4; i++) {
                    EEPROM_write(i, passInput[i]);
                  }
                  passwordSet = true; // The password is now set (or reset)
                  Serial.println("Password is saved");
                  
                  // After saving, go to the IDLE (locked) state.
                  // The user must re-enter the new password to unlock.
                  currentLEDstate = IDLE; 
                  digitalWrite(GRN_LED_PIN, LOW);
                  digitalWrite(YELLOW_LED_PIN, LOW);
                  lockServo(); // Ensure we are locked
                  
                } else {
                  // --- CHECK PASSWORD ---
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
                    currentLEDstate = CORRECT_PASSWORD; // UNLOCKED state
                    digitalWrite(GRN_LED_PIN, HIGH);
                    digitalWrite(YELLOW_LED_PIN, LOW);
                  } else {
                    Serial.println("Incorrect Password");
                    lockServo();
                    digitalWrite(GRN_LED_PIN, LOW); 
                    currentLEDstate = WRONG_PASSWORD; // Blinking yellow
                  }
                }
                input = 0;  // reset for next entry
              }
              
              break; // Exit the for-loop once a key is found
            }
    }
  }
  if (millis() - lastKeypressMillis > SLEEP_TIMEOUT) { //if it's been idle for more than the SLEEP_TIMEOUT
    Serial.println("sleep");
    //shut off LEDs before sleeping
    digitalWrite(POWER_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GRN_LED_PIN, LOW);
    Serial.flush(); //waits until message is sent then goes to sleep
    goToSleep();
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
    if(currentLEDstate == WRONG_PASSWORD){
      yellowBlinkState = !yellowBlinkState;
      digitalWrite(YELLOW_LED_PIN, yellowBlinkState);
    }
    if((!passwordSet)&&(currentLEDstate == TYPING)){ //blink if setting password still
      greenBlinkState = !greenBlinkState;
      digitalWrite(GRN_LED_PIN, greenBlinkState);
    }

    previousSerialMillis = millis(); // Reset the print timer
     anval = analogRead(A0);
      // Serial.println("anval:");
      // Serial.println(anval);
    obstructionReturn();

    currentTransistorMillis = millis();
    if (Transistor_State)
    {
      digitalWrite(Transistor_Pin, HIGH);
      //Check if battery is running low. the serial interval here is for things that don't need to be continuously sampled
      int batteryLife = analogRead(A3);
      double batteryVoltage = batteryLife * (5.0 / 1023.0);
      //Serial.println(batteryVoltage);
      if (batteryVoltage <= 3.27)
      {
        digitalWrite(POWER_LED_PIN, HIGH);
      }
      else 
      {
        digitalWrite(POWER_LED_PIN, LOW);
      }
      if (currentTransistorMillis - previousTransistorMillis >= TransistorHighTime)
      {
        Transistor_State = false;
        previousTransistorMillis = currentTransistorMillis;
        digitalWrite(POWER_LED_PIN, LOW);
        digitalWrite(Transistor_Pin, LOW);
      }
    }
    else if (!Transistor_State)
    {
      digitalWrite(Transistor_Pin, LOW);
      digitalWrite(POWER_LED_PIN, LOW);

      if (currentTransistorMillis - previousTransistorMillis >= TransistorLowTime)
      {
        Transistor_State = true;
        previousTransistorMillis = currentTransistorMillis;
      }
    }
  }
}