
#define setHigh(port, pin) ((port) |= (1 << (pin)))
void init() {
  ADMUX  = 0;//(1<<REFS0);							// external reference voltage
  // ADCSRA  = (1<<ADEN);   							// enable ADC
  //ADCSRA |= (1<<ADPS2) | (1<<ADPS1) |(1<<ADPS0);// | (1<<ADIE);

  bitWrite(ADCSRA,ADEN,1); //adc enabled

  bitWrite(ADCSRA,ADPS2,1); // set prescaler
  bitWrite(ADCSRA,ADPS1,1);
  bitWrite(ADCSRA,ADPS0,1);

  bitWrite(ADCSRA,ADIE,1); //enable conversion finished interupt
  bitWrite(SREG,7,1);
  // prescaler = highest division
  //     ADCSRB |= (1<<BIN); //bipolar mode
}


// channel 8 can be used to measure the temperature of the chip
void connectChannel(uint8_t number) {
  ADMUX &= (10110000);
  ADMUX |= number;
  //DIDR0  =  (1<<number);
}

void startConversion() {
  //setHigh(ADCSRA,ADSC);
  bitWrite(ADCSRA,ADSC,1); //start conversion
}

bool isConversionFinished() {
  return (ADCSRA & (1<<ADIF));
}

bool isConversionRunning() {
  return !(ADCSRA & (1<<ADIF));
}

uint16_t getConversionResult() {
  uint16_t result = ADCL;
  return result | (ADCH<<8);
}
