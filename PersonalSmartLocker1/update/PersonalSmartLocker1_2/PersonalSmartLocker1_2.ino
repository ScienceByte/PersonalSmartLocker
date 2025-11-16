/*
 * Full-featured lockbox code with a Finite State Machine (FSM).
 * * Features:
 * - 4-digit keypad password (stored in EEPROM)
 * - Servo motor control for lock/unlock
 * - LED status indicators (Power, Yellow, Green)
 * - Obstruction detection and reversal
 * - Low battery voltage warning
 * - Keypad-based password reset ('*' when unlocked)
 * - Automatic sleep after 30s of inactivity
 * - Wakes from sleep on interrupt (D2)
 */

// --- Pin Definitions ---
// LEDs
const int POWER_LED_PIN = 8;
const int YELLOW_LED_PIN = 7;
const int GRN_LED_PIN = 6;
// Servo
const int outputPinServo = 9;
// Keypad (Analog)
const int KEYPAD_PIN = A5;
// Obstruction Sensor (Analog)
const int OBSTRUCTION_PIN = A0;
// Battery Voltage (Analog)
const int BATTERY_PIN = A3;
// Sleep Interrupt (Digital)
const int wakeUpPin = 2; // This is INT0

// --- State Machine ---
// This is the single "brain" for the lock's state
enum LockState {
  STATE_SET_PASSWORD,   // Initial setup or reset (Solid G+Y, G blinks)
  STATE_LOCKED,         // Idle and locked (All LEDs off)
  STATE_TYPING,         // User is entering a password (Solid Y)
  STATE_UNLOCKED,       // Correct password entered (Solid G)
  STATE_WRONG_PASSWORD  // Incorrect password entered (Blinking Y)
};
LockState currentLockState = STATE_SET_PASSWORD; // Start in setup mode

// --- LED Blinking ---
bool yellowBlinkState = LOW;
bool greenBlinkState = LOW;
unsigned long previousBlinkMillis = 0;
const long BLINK_INTERVAL = 200; // How fast to blink

// --- Servo ---
enum ServoState {
  PULSE_OPEN_HIGH,
  PULSE_OPEN_LOW,
  HOLDING_OPEN,
  PULSE_LOCK_HIGH,
  PULSE_LOCK_LOW,
  HOLDING_LOCKED
};
ServoState currentServoState = HOLDING_LOCKED;
int servoTargetState = 0; // 0 = lock, 1 = open
int servoIntendedState = 0; // 0 = lock, 1 = open
bool isObstructed = false;
int obstructionThreshold = 100;
const unsigned long pulseOpenHighTime = 2000;
const unsigned long pulseOpenLowTime = 18000;
const unsigned long pulseLockHighTime = 600;
const unsigned long pulseLockLowTime = 19400;
const int maxPulseNum = 50;
int curPulseNum = 0;
unsigned long previousServoMicros = 0;

// --- Sleep ---
const unsigned long SLEEP_TIMEOUT = 30000; // 30 seconds
volatile bool justWoke = false;

// --- Keypad ---
char passInput[5]; // 4 digits + null terminator
int input = 0;
bool passwordSet = false; // We'll check EEPROM in setup()
char KEYS[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9', '*', '0', '#' };
const double voltages[][2] = {
  {4.05, 4.15}, // '1'
  {3.70, 3.8},  // '2'
  {3.00, 3.12}, // '3'
  {3.35, 3.50}, // '4'
  {3.13, 3.25}, // '5'
  {2.65, 2.69}, // '6'
  {2.70, 2.77}, // '7'
  {2.50, 2.65}, // '8'
  {2.20, 2.35}, // '9'
  {2.04, 2.15}, // '*'
  {1.90, 2.00}, // '0'
  {1.74, 1.85}  // '#'
};
unsigned long lastKeypressMillis = 0;
const unsigned long KEYPAD_DEBOUNCE_DELAY = 300; // ms

// --- Timers ---
unsigned long previousSerialMillis = 0;
const long serialInterval = 200; // For less frequent tasks


//==================================================================
// SETUP
//==================================================================
void setup() {
  Serial.begin(115200);

  // Pin Modes
  pinMode(outputPinServo, OUTPUT);
  pinMode(OBSTRUCTION_PIN, INPUT);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(KEYPAD_PIN, INPUT);

  pinMode(POWER_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GRN_LED_PIN, OUTPUT);

  // Sleep Setup
  pinMode(wakeUpPin, INPUT); // D2
  EICRA = EICRA & ~((1 << ISC01) | (1 << ISC00)); // Interrupt on low level

  // Check if a password is already set
  // Simple check: is the first char a valid digit?
  // (A more robust check might be a "magic number" in another address)
  char firstChar = EEPROM_read(0);
  if (firstChar >= '0' && firstChar <= '9') {
    passwordSet = true;
    currentLockState = STATE_LOCKED; // Go to normal locked mode
    Serial.println("Password loaded. System locked.");
  } else {
    // No password set, stay in STATE_SET_PASSWORD
    passwordSet = false;
    Serial.println("Set passcode: ");
    digitalWrite(GRN_LED_PIN, HIGH);
    digitalWrite(YELLOW_LED_PIN, HIGH);
  }

  // Initialize timers
  previousServoMicros = micros();
  lastKeypressMillis = millis(); // For sleep timer
  
  lockServo(); // Ensure we start in a locked state
}

//==================================================================
// MAIN LOOP - The "Conductor"
// This is now very clean. It just calls the helper functions.
//==================================================================
void loop() {
  // Handle wake-up logic first
  if (justWoke) {
    handleWakeUp();
  }

  // Handle the main logic FSM (keypad, states)
  handleLockMachine();

  // Handle the servo FSM (runs in parallel)
  handleServo();

  // Handle less-frequent tasks (blinking, battery)
  handleTimedTasks();

  // Check if it's time to go to sleep
  checkSleep();
}

//==================================================================
// STATE MACHINE - The "Brain"
//==================================================================
void handleLockMachine() {
  // --- 1. Check for Keypress ---
  // This only runs if the debounce delay has passed.
  if (millis() - lastKeypressMillis > KEYPAD_DEBOUNCE_DELAY) {
    int keyPressed = analogRead(KEYPAD_PIN);
    double voltage = keyPressed * (5.0 / 1023.0);
    char key = '\0'; // No key pressed by default

    // Find which key matches the measured voltage
    for (int j = 0; j < 12; j++) {
      if (voltage >= voltages[j][0] && voltage <= voltages[j][1]) {
        key = KEYS[j];
        lastKeypressMillis = millis(); // Reset the inactivity/sleep timer
        Serial.print("Key pressed: ");
        Serial.println(key);
        break; // Exit the for-loop once a key is found
      }
    }

    // --- 2. Feed the Key to the State Machine ---
    if (key != '\0') {
      
      switch (currentLockState) {
        
        // --- SET PASSWORD STATE ---
        case STATE_SET_PASSWORD:
          passInput[input] = key;
          input++;
          
          if (input == 4) {
            passInput[4] = '\0';
            Serial.print("Setting new passcode: ");
            Serial.println(passInput);
            
            // Save the password to EEPROM
            for (int i = 0; i < 4; i++) {
              EEPROM_write(i, passInput[i]);
            }
            passwordSet = true;
            Serial.println("Password is saved.");
            
            // Transition to the LOCKED state
            input = 0;
            currentLockState = STATE_LOCKED;
            digitalWrite(GRN_LED_PIN, LOW);
            digitalWrite(YELLOW_LED_PIN, LOW);
          }
          break; // end SET_PASSWORD case
          
        // --- LOCKED STATE ---
        case STATE_LOCKED:
          // Any key press in the LOCKED state moves us to TYPING
          digitalWrite(YELLOW_LED_PIN, HIGH); // Turn on typing light
          currentLockState = STATE_TYPING;
          
          // Store the first key
          passInput[input] = key;
          input++;
          break; // end LOCKED case

        // --- TYPING STATE ---
        case STATE_TYPING:
          passInput[input] = key;
          input++;

          if (input == 4) {
            passInput[4] = '\0';
            Serial.print("Entered passcode: ");
            Serial.println(passInput);
            
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
              currentLockState = STATE_UNLOCKED;
              digitalWrite(GRN_LED_PIN, HIGH);
              digitalWrite(YELLOW_LED_PIN, LOW);
            } else {
              Serial.println("Incorrect Password");
              lockServo();
              currentLockState = STATE_WRONG_PASSWORD;
              digitalWrite(GRN_LED_PIN, LOW);
            }
            input = 0; // reset for next entry
          }
          break; // end TYPING case
          
        // --- UNLOCKED STATE ---
        case STATE_UNLOCKED:
          // Check for the '*' key to reset password
          if (key == '*') {
            Serial.println("Password reset activated. Enter new 4-digit password.");
            passwordSet = false; // Flag for a new password
            input = 0;
            currentLockState = STATE_SET_PASSWORD;
            // Set LEDs for setup mode
            digitalWrite(GRN_LED_PIN, HIGH);
            digitalWrite(YELLOW_LED_PIN, HIGH);
          }
          break; // end UNLOCKED case

        // --- WRONG PASSWORD STATE ---
        case STATE_WRONG_PASSWORD:
          // Any keypress will clear the "wrong" state and go back to locked
          input = 0;
          currentLockState = STATE_LOCKED;
          digitalWrite(YELLOW_LED_PIN, LOW); // Stop blinking
          break; // end WRONG_PASSWORD case
      }
    }
  } // end if(debounce delay)
}

//==================================================================
// HELPER FUNCTIONS
//==================================================================

// --- Servo FSM ---
void handleServo() {
  unsigned long currentMicros = micros();

  switch (currentServoState) {
    case PULSE_OPEN_HIGH:
      digitalWrite(outputPinServo, HIGH);
      if (currentMicros - previousServoMicros >= pulseOpenHighTime) {
        currentServoState = PULSE_OPEN_LOW;
        previousServoMicros = currentMicros;
      }
      break;

    case PULSE_OPEN_LOW:
      digitalWrite(outputPinServo, LOW);
      if (currentMicros - previousServoMicros >= pulseOpenLowTime) {
        currentServoState = HOLDING_OPEN;
      }
      break;

    case HOLDING_OPEN:
      if (servoTargetState == 0) { // If command is to lock
        currentServoState = PULSE_LOCK_HIGH;
        previousServoMicros = currentMicros;
        curPulseNum = 0;
      } else if (curPulseNum < maxPulseNum) { // Keep pulsing to hold
        curPulseNum++;
        currentServoState = PULSE_OPEN_HIGH;
        previousServoMicros = currentMicros;
      }
      break;

    case PULSE_LOCK_HIGH:
      digitalWrite(outputPinServo, HIGH);
      if (currentMicros - previousServoMicros >= pulseLockHighTime) {
        currentServoState = PULSE_LOCK_LOW;
        previousServoMicros = currentMicros;
      }
      break;

    case PULSE_LOCK_LOW:
      digitalWrite(outputPinServo, LOW);
      if (currentMicros - previousServoMicros >= pulseLockLowTime) {
        currentServoState = HOLDING_LOCKED;
      }
      break;

    case HOLDING_LOCKED:
      if (servoTargetState == 1) { // If command is to open
        currentServoState = PULSE_OPEN_HIGH;
        previousServoMicros = currentMicros;
        curPulseNum = 0;
      } else if (curPulseNum < maxPulseNum) { // Keep pulsing to hold
        curPulseNum++;
        currentServoState = PULSE_LOCK_HIGH;
        previousServoMicros = currentMicros;
      }
      break;
  }
}

// --- Timed Tasks (Blink, Battery, Obstruction) ---
void handleTimedTasks() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousSerialMillis >= serialInterval) {
    previousSerialMillis = currentMillis; // Reset the timer

    // --- Blinking Logic ---
    if (currentLockState == STATE_WRONG_PASSWORD) {
      yellowBlinkState = !yellowBlinkState;
      digitalWrite(YELLOW_LED_PIN, yellowBlinkState);
    }
    if (currentLockState == STATE_SET_PASSWORD) { // Blink green while setting
      greenBlinkState = !greenBlinkState;
      digitalWrite(GRN_LED_PIN, greenBlinkState);
    }

    // --- Obstruction Check ---
    obstructionReturn();

    // --- Battery Check ---
    int batteryLife = analogRead(BATTERY_PIN);
    double batteryVoltage = batteryLife * (5.0 / 1023.0);
    
    if (batteryVoltage <= 3.27) {
      digitalWrite(POWER_LED_PIN, HIGH); // Turn on low power warning
    } else {
      digitalWrite(POWER_LED_PIN, LOW);
    }
  }
}

// --- Servo Commands ---
void lockServo() {
  servoTargetState = 0;
  servoIntendedState = 0;
  Serial.println("Servo: Locking");
}

void openServo() {
  servoTargetState = 1;
  servoIntendedState = 1;
  Serial.println("Servo: Opening");
}

// --- Obstruction Logic ---
void obstructionReturn() {
  int anval = analogRead(OBSTRUCTION_PIN);

  // Check if an obstruction has just appeared
  if (anval > obstructionThreshold && !isObstructed) {
    isObstructed = true; // Set the flag so this only runs once
    Serial.println("Obstruction Detected! Reversing.");
    
    // Reverse the current target state
    servoTargetState = (servoTargetState == 1) ? 0 : 1;

  } 
  // Check if an obstruction has just been removed
  else if (anval <= obstructionThreshold && isObstructed) {
    isObstructed = false; // Clear the flag
    Serial.println("Obstruction Cleared. Returning to original position.");
    
    // Restore the target state to the original intended state
    servoTargetState = servoIntendedState;
  }
}

// --- Sleep Functions ---
void checkSleep() {
  if (millis() - lastKeypressMillis > SLEEP_TIMEOUT) {
    Serial.println("Going to sleep...");
    // Shut off LEDs before sleeping
    digitalWrite(POWER_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GRN_LED_PIN, LOW);
    Serial.flush(); // Wait for Serial to finish
    goToSleep();
  }
}

void handleWakeUp() {
  Serial.println("Woke up.");
  lastKeypressMillis = millis(); // Reset inactivity timer
  justWoke = false; // Reset the flag
  EIMSK &= ~(1 << INT0); // Disable INT0 interrupt
}

ISR(INT0_vect) {
  justWoke = true;
}

void goToSleep() {
  if (digitalRead(wakeUpPin) == LOW) {
     Serial.println("Pin already LOW, aborting sleep cycle.");
     Serial.flush();
     justWoke = false;
     return;
   }

  Serial.println("Entering Standby Mode...");
  Serial.flush();

  cli(); // Disable interrupts

  // Set sleep mode to STANDBY
  SMCR = (SMCR & ~(1 << SM0)) | (1 << SM2) | (1 << SM1);

  EIMSK |= (1 << INT0); // Enable INT0

  do {
    SMCR |= (1 << SE); // Enable Sleep
    sei();             // Interrupts ON for one cycle
    __asm__ __volatile__("sleep" ::); // Sleep Now
    // --- WAKE UP ---
    SMCR &= ~(1 << SE); // Disable Sleep
  } while (0);

  sei(); // Re-enable interrupts
  Serial.println("...Woke up from sleep function.");
  Serial.flush();
}

// --- EEPROM Functions ---
unsigned char EEPROM_read(unsigned int uiAddress) {
  while (EECR & (1 << EEPE)) ;
  EEAR = uiAddress;
  EECR |= (1 << EERE);
  return EEDR;
}

void EEPROM_write(unsigned int uiAddress, unsigned char ucData) {
  while (EECR & (1 << EEPE)) ;
  EEAR = uiAddress; EEDR = ucData;
  cli(); //disable interrupts
  EECR |= (1 << EEMPE);
  EECR |= (1 << EEPE);
  sei(); //enable interrupts
}