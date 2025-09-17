void setup() {
pinMode(9, OUTPUT);
}

void loop() {

digitalWrite(9, HIGH);
delayMicroseconds(2500);
digitalWrite(9, LOW);
delayMicroseconds(17500);

delay(500);

digitalWrite(9, HIGH);
delayMicroseconds(500);
digitalWrite(9, LOW);
delayMicroseconds(19500);

delay(500);

}
