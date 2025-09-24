// Define the pin for the output
const int outputPin = 9;

// Define the time intervals in microseconds for the pulses
const unsigned long pulse1HighTime = 2500;
const unsigned long pulse1LowTime = 17500;
const unsigned long pulse2HighTime = 500;
const unsigned long pulse2LowTime = 19500;

// Define the delay interval in milliseconds between pulse sequences
const unsigned long delayTime = 500;

// will be using a state machine, using enums for that
enum State {
  PULSE_1_HIGH,
  PULSE_1_LOW,
  DELAY_1,
  PULSE_2_HIGH,
  PULSE_2_LOW,
  DELAY_2
};

// Variable to hold the current state
State currentState = PULSE_1_HIGH;

// Variables to store the last time an event happened
unsigned long previousMicros = 0;
unsigned long previousMillis = 0;

void setup() {
  // Set the pin as an output
  pinMode(outputPin, OUTPUT);
  
  // Initialize timers
  previousMicros = micros();
  previousMillis = millis();
}

void loop() {
  // current time at the start of the loop
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();

  switch (currentState) {
    case PULSE_1_HIGH:
      digitalWrite(outputPin, HIGH);
      
      // Check if the required time has passed
      if (currentMicros - previousMicros >= pulse1HighTime) {
        currentState = PULSE_1_LOW;
        // Save the current time for the next interval
        previousMicros = currentMicros;
      }
      break;

    case PULSE_1_LOW:
      digitalWrite(outputPin, LOW);
      
      // Check if the required time has passed
      if (currentMicros - previousMicros >= pulse1LowTime) {
        currentState = DELAY_1;
        previousMillis = currentMillis; // using millis() for the long delay
      }
      break;

    case DELAY_1:
      // Check if the delay has passed
      if (currentMillis - previousMillis >= delayTime) {
        currentState = PULSE_2_HIGH;
        previousMicros = currentMicros; // Switch back to micros() for the pulse
      }
      break;

    case PULSE_2_HIGH:
      digitalWrite(outputPin, HIGH);

      // Check if the required time has passed
      if (currentMicros - previousMicros >= pulse2HighTime) {
        currentState = PULSE_2_LOW;
        previousMicros = currentMicros;
      }
      break;

    case PULSE_2_LOW:
      digitalWrite(outputPin, LOW);

      // Check if the required time has passed
      if (currentMicros - previousMicros >= pulse2LowTime) {
        currentState = DELAY_2;
        previousMillis = currentMillis; // switch to millis() for the long delay
      }
      break;

    case DELAY_2:
      // Check if the delay has passed
      if (currentMillis - previousMillis >= delayTime) {
        // loop back to the beginning
        currentState = PULSE_1_HIGH;
        previousMicros = currentMicros; // switch back to micros() for the pulse
      }
      break;
  }

//other code: password checking, etc, will be added here
}
