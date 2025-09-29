
void setup() {
  Serial.begin(9600);

  EEPROM_write(0, 2);
  EEPROM_write(1, 8);
  EEPROM_write(2, 0);
  EEPROM_write(3, 4);

  int valueRead0 = EEPROM_read(0);
  int valueRead1 = EEPROM_read(1);
  int valueRead2 = EEPROM_read(2);
  int valueRead3 = EEPROM_read(3);

  Serial.println(valueRead0);
  Serial.println(valueRead1);
  Serial.println(valueRead2);
  Serial.println(valueRead3);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  
}


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
