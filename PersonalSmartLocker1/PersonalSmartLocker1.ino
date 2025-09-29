// Store each number pressed
char passInput[4];
int input = 0;
char approvedPass[5];
bool passwordSet = false;

// Key mapping for the 4x3 keypad
char KEYS[] = { '1','2','3','4','5','6','7','8','9','*','0','#' };

// Voltage ranges
const double voltages[][2] = {
  {4.900, 4.910},   // '1' Row1,C1
  {4.840, 4.850},   // '2' Row1,C2 
  {4.730, 4.736},   // '3' Row1,C3 
  {4.795, 4.800},   // '4' Row2,C1 
  {4.741, 4.746},   // '5' Row2,C2 
  {4.620, 4.638},   // '6' Row2,C3 
  {4.638, 4.655},   // '7' Row3,C1
  {4.584, 4.600},   // '8' Row3,C2
  {4.480, 4.492},   // '9' Row3,C3
  {4.420, 4.440},   // '*' Row4,C1
  {4.365, 4.400},   // '0' Row4,C2
  {4.270, 4.300}    // '#' Row4,C3
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