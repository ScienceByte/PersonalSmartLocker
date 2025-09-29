// Store each number pressed
char passInput[4];
int input = 0;
char approvedPass[5];
bool passwordSet = false;

// Key mapping for the 4x3 keypad
char KEYS[] = { '1','2','3','4','5','6','7','8','9','*','0','#' };

// Voltage ranges
const double voltages[][2] = {
  {0.63, 0.66},   // '1' Row1, Col150
  {1.36, 1.38},   // '2' Row1, Col390
  {1.93, 1.94},   // '3' Row1, Col680
  {0.21, 0.22},   // '4' Row2, Col150
  {0.51, 0.52},   // '5' Row2, Col390
  {0.80, 0.81},   // '6' Row2, Col680
  {0.10, 0.11},   // '7' Row3, Col150
  {0.26, 0.27},   // '8' Row3, Col390
  {0.42, 0.43},   // '9' Row3, Col680
  {0.048, 0.05},  // '*' Row4, Col150
  {0.12, 0.13},   // '0' Row4, Col390
  {0.20, 0.21}    // '#' Row4, Col680
};


void setup() {
  Serial.begin(9600);
  Serial.println("Set passcode: "); //Prompts the user to input password
}

void loop() {
  int keyPressed = analogRead(A5);
  double voltage = keyPressed * (5.0 / 1023.0);

  // Find which key matches the measured voltage
  for (int j = 0; j < 12; j++) {
    if (voltage >= voltages[j][0] && voltage <= voltages[j][1]) {
      passInput[input] = KEYS[j];
      input++;
      Serial.print("Key pressed: ");
      Serial.println(KEYS[j]);
      delay(300); 

      if (input == 4) {
        Serial.print("Passcode: ");
        for (int i = 0; i < 4; i++) {
          Serial.print(passInput[i]);
        }
        Serial.println();
        input = 0; // reset for next code
      }
      break;
    }
  }
}