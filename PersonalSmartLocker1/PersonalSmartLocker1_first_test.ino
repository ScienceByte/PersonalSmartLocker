  //Set an array for each number pressed to be stored in
  int passInput[4];
  int input = 0;
  //Set each key as a character
  char KEYS[] 
    {
      '1','2', '3', '4', '5', '6', '7', '8', '9', '*', '0', '#'
    };
  //Set matrix to identify each voltage range
  const double voltages[][2] = 
  {
    {0.60, 0.70}, {2.0, 2.2}, {2.95, 3.05}, {0.17, 0.27}, {0.77, 0.87}, 
    {1.51, 1.61}, {0.06, 0.16}, {0.40, 0.50}, {0.88, 0.98}, {1.0, 1.1}, 
    {2.66, 2.86}, {3.56, 3.76}, {0.0, 0.22}
  };

void setup() {
  //Serial.begin(9600);
}

void loop() {
  //Use for loop to iterate until the correct number of buttons are pressed. 
  for (int i = 0; i < 4; i++)
  {
    //Read the value of input A5 to determine which key was pressed
    int keyPressed = analogRead(A5);

    //Convert the signal from A5 into a voltage
    //Use double because it yields more decimal points than float
    double voltage = keyPressed * (5.0 / 1023.0);

    for (int j = 0; j < 13; j++)
    {
      if (voltage >= voltages[j][0] && voltage <= voltages[j][1])
      {
        passInput[input] = KEYS[j];
        input++;
        break;
      }
    }
  }
}
