/*
  KASTLE ARP
  -melodic beast
  -product page:

  OPERATION:
  any change of the note will trigger a decay envelope
  DECAY knob from centre to the right (CW) is normal decay. from centre CCW it is non-retrigable envelope that also only updates the pitch of the oscillator when triggered.
  this is usefull for creating variating patterns and have interference patterns between the lfo rate and decay of the envelope
  NOTE knob selects a note
  TIMBRE knob adds XOR timbral modulation
  CHORD patchpoint accesses 5 different chords from lowest voltage to highest they are the following (when root note is C)
  -F major "-" F A C
  -A minor (in between) A C E
  -C major (unconnected) C E G
  - E minor (in between) E G B
  - G major "+" G B D

  
  BOOT MODE:
  unplugg all patch cords
  to enter the boot mode patch CHORD to "+", turn NOTE, TIMBRE and DECAY fully CW and power up Kastle Arp
  in boot mode NOTE previews the root triad chord of the root key
  use TIMBRE to set the ROOT key (C is fully left-CCW)
  use DECAY to set the FINE-TUNE (around center should be A=440hz)
  
  to leave the boot mode unplugg the patchcable from CHORD input - ROOT and FINE-TUNE will be stored in the memory to be reused after powercycle
  
  #########
  Writen by Vaclav Pelousek 2023
  based on the earlier kastle v1.5 and kastleDrum
  open source license: CC BY SA
  http://www.bastl-instruments.com

  -this is the code for the VCO chip of the Kastle
  -software written in Arduino 1.8.19 - used to flash ATTINY 85 running at 8mHz
  http://highlowtech.org/?p=1695
  -created with help of the heavenly powers of internet and several tutorials that you can google out
  -i hope somebody finds this code usefull (i know it is a mess :( )

  thanks to
  -Lennart Schierling for making some work on the register access
  -Uwe Schuller for explaining the capacitance of zener diodes
  -Peter Edwards for making the inspireing bitRanger
  -Ondrej Merta for being the best boss
  -and the whole bastl crew that made this project possible
  -Arduino community and the forums for picking ups bits of code to setup the timers & interrupts
  -kastle arp, v1.5 and kastle drum uses bits of code from miniMO DCO http://www.minimosynth.com/

*/

#define F_CPU 8000000  // This is used by delay.h library
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>
#include <EEPROM.h>

#define LOW_THRES 150
#define HIGH_THRES 162
//defines for synth parameters
#define PITCH 2
#define WS_1 3
#define WS_2 1

const char PROGMEM sinetable[128] = {
  0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 9, 10, 11, 12, 14, 15, 17, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35, 37, 40, 42, 44, 47, 49, 52, 54, 57, 59, 62, 65, 67, 70, 73, 76, 79, 82, 85, 88, 90, 93, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124,
  128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173, 176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244, 245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255,
};

//the actual table that is read to generate the sound
unsigned char wavetable[256];


//tuning related
const PROGMEM uint16_t dcoTable[97] = { //tuned pitches
  69 /*C*/, 73, 77, 81, 86, 91, 96, 102 /*G*/, 108, 115, 122, 129,
  137 /*C*/, 145, 153, 162, 172, 182, 193, 205 /*G*/, 217, 230, 243, 258,
  273 /*C*/, 289, 306, 325, 344, 365, 386, 409 /*G*/, 434, 460, 487, 516,
  547 /*C*/, 579, 614, 650, 689, 730, 773, 819 /*G*/, 867, 919, 973, 1031,
  1092 /*C*/, 1157, 1227, 1298, 1378, 1459, 1546, 1638 /*G*/, 1734, 1837, 1947, 2063,
  2187 /*C*/, 2314, 2453, 2599, 2753, 2915, 3089, 3273 /*G*/, 3466, 3674, 3894, 4126,
  4374 /*C*/, 4628, 4906, 5198, 5506, 5830, 6178, 6546 /*G*/, 6932, 7348, 7788, 8252,
  8748 /*C*/, 9256, 9812, 10396, 11012, 11660, 12356, 13092 /*G*/, 13864, 14696, 15578, 16504,
  17496 /*C*/
};

//100010000100 2180 //Gdur
//100010010000 2192 //Emol
//000010010001 145 //Cdur
//001000010001 529 //Amol
//001000100001 545 //Fdur
//001000100100 548 //Dmol

const uint16_t scales[8] = {548, 548, 545, 529, 145, 2192, 2180, 2180};
uint16_t _useScale = scales[4]; //scales[4];
uint16_t pitchAverage, lastPitchAverage;
uint8_t semitone, lastSemitone;
uint8_t root;
uint8_t lastPitch, pitch;
uint8_t transpose = 0; //default used when formating memory
uint8_t fineTune = 120; //default used when formating memory
const uint8_t rootToPitch[8] = {2, 2, 5, 9, 0, 4, 7, 7}; //bass tone for each chord

//envelope related
uint8_t decayVolume;
uint16_t decayTime = 50;
uint8_t decayCount;
uint16_t decayRun[8];
uint8_t runningDecay;
uint16_t decayCounter = 0;

//oscillator related
byte sample;
unsigned int _phase;
unsigned int frequency;
byte sample2;
unsigned int _phase2;
unsigned int frequency2;
uint8_t _phs;
uint8_t _xor = 0;

//general
//uint8_t mapLookup[256];
bool bootMode;
uint8_t startupRead = 0;
uint8_t _clocks;


//analog input related
const uint8_t analogToDigitalPinMapping[4] = {
  PORTB5, PORTB2, PORTB4, PORTB3
};

uint8_t lastAnalogChannelRead;
bool firstRead = false;
uint8_t analogChannelRead = 1;
uint8_t analogValues[4];
uint8_t lastAnalogValues[4];
uint8_t analogChannelSequence[6] = {0, 1, 0, 2, 0, 3};
uint8_t analogChannelReadIndex;

#define PITCHMAP_POINTS 5
uint16_t pitchMap[10] = {
  0, 63, 127, 191, 235,   0, 40, 50, 60, 84
};

uint32_t curveMap(uint8_t value, uint8_t numberOfPoints, uint16_t * tableMap) {
  uint32_t inMin = 0, inMax = 255, outMin = 0, outMax = 255;
  for (int i = 0; i < numberOfPoints - 1; i++) {
    if (value >= tableMap[i] && value <= tableMap[i + 1]) {
      inMax = tableMap[i + 1];
      inMin = tableMap[i];
      outMax = tableMap[numberOfPoints + i + 1];
      outMin = tableMap[numberOfPoints + i];
      i = numberOfPoints + 10;
    }
  }
  return map(value, inMin, inMax, outMin, outMax);
}

void createLookup() {
  for (uint16_t i = 0; i < 256; i++) {
    // mapLookup[i]=curveMap(i,WSMAP_POINTS,wsMap);
  }
}

uint16_t clocks() {
  //return _clocks;
  return TCNT0 | (_clocks << 8);
}
void writeWave(int wave) {
  switch (wave) {
    case 0:
      sineWave();
      break;
    case 1:
      triangleWave();
      break;
    case 2:
      squareWave();
      break;
    case 3:
      sawtoothWave();
      break;
    case 4:
      digitalWrite(0, LOW);
      zeroWave();
      break;
  }
}

//functions to populate the wavetable
void sineWave() {  //too costly to calculate on the fly, so it reads from the sine table. We use 128 values, then mirror them to get the whole cycle
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = pgm_read_byte_near(sinetable + i);
  }
  wavetable[128] = 255;
  for (int i = 129; i < 256; ++i) {
    wavetable[i] = wavetable[256 - i] ;
  }
}
void sawtoothWave() {
  for (int i = 0; i < 256; ++i) {
    wavetable[i] = i; // sawtooth
  }
}
void triangleWave() {
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = i * 2;
  }
  int value = 255;
  for (int i = 128; i < 256; ++i) {
    wavetable[i] = value;
    value -= 2;
  }
}
void squareWave() {
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = 255;
  }
  for (int i = 128; i < 256; ++i) {
    wavetable[i] = 1;                  //0 gives problems (offset and different freq), related to sample  = ((wavetable[phase >> 8]*amplitude)>>8);
  }
}
void zeroWave() {
  for (int i = 0; i < 256; ++i) {
    wavetable[i] = 1;                  //0 gives problems
  }
}

void setTimers(void)
{

  PLLCSR |= (1 << PLLE);               // Enable PLL (64 MHz)
  _delay_us(100);                      // Wait for a steady state
  while (!(PLLCSR & (1 << PLOCK)));    // Ensure PLL lock
  PLLCSR |= (1 << PCKE);               // Enable PLL as clock source for timer 1
  PLLCSR |= (1 << LSM); //low speed mode 32mhz
  cli();                               // Interrupts OFF (disable interrupts globally)

  TCCR0A = 2 << COM0A0 | 2 << COM0B0 | 3 << WGM00;
  TCCR0B = 0 << WGM02 | 1 << CS00;

  //  setup timer 0 to run fast for audiorate interrupt
  TCCR1 = 0;                  //stop the timer
  TCNT1 = 0;                  //zero the timer
  GTCCR = _BV(PSR1);          //reset the prescaler
  OCR1A = 255;                //set the compare value
  OCR1C = 255;

  TIMSK = _BV(OCIE1A);  //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1
  TCCR1 = _BV(CTC1) | _BV(CS12);
  sei();
}


void setFrequency2(uint16_t input) {
  frequency2 = input;
}

uint8_t quantizeNote(uint8_t _note) {
  uint8_t _semitone = _note % 12;
  if (bitRead(_useScale, (_semitone) % 12) ) {
    return _note;
  }
  else {
    uint8_t findSemitoneUp = 0;
    while (!bitRead(_useScale, (_note + findSemitoneUp) % 12)) findSemitoneUp++;
    _note += findSemitoneUp;
    return _note;
  }
}

void setSemitone(uint16_t _semitone) {
  lastSemitone = semitone;
  semitone = _semitone;
  //setFrequency(dcoTable[_semitone] << 1);
  int semiSize = (pgm_read_word_near(dcoTable + _semitone + 1) << 1) - (pgm_read_word_near(dcoTable + _semitone) << 1);
  int fine = map(fineTune, 0, 255, -semiSize, semiSize);
  setFrequency((pgm_read_word_near(dcoTable + _semitone + transpose) << 1) + fine);
  setFrequency2((pgm_read_word_near(dcoTable + rootToPitch[root] + transpose) << 1) + fine);
}

void setFrequency(uint16_t input) {
  frequency = input  ;
}


uint8_t trigger() {
  decayVolume = 255;//, decayVolume2 = 150;
}

void setDecay() {
  if (analogValues[WS_2] > 100) decayTime = constrain(analogValues[WS_2] - 120, 1, 255);//, pitchEnv = 0;
  else decayTime = (100 - analogValues[WS_2]);//, pitchEnv = 255; //decayTime;
}

void renderDecay() {
  if (decayTime != 0) {
    if (1) {
      decayCounter += 6;
      if (decayCounter >= decayTime)
      {
        decayCounter = 0;
        if (decayVolume > 0) decayVolume -= ((decayVolume >> 6) + 1); //the decayVolume is here to make the shape a bit exponential
        //else decayVolume=255;//, _phase=0;
      }
    }
  }
  //add a bit of averaging on top of the envelope to lower clicking in the attack phase
  decayCount++;
  if (decayCount > 7) decayCount = 0;
  decayRun[decayCount] = decayVolume;
  uint16_t decaySum = 0;
  for (uint8_t i = 0; i < 8; i++) decaySum += decayRun[i];
  runningDecay = decaySum >> 3;

}

void setup()  { //happends at the startup
  writeWave(0);
  digitalWrite(5, HIGH); //turn on pull up resistor for the reset pin
  // createLookup(); //mapping of knob values is done thru a lookuptable which is generated at startup from defined values
  //set outputs
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);

  
  if ( EEPROM.read(8) == 123 && EEPROM.read(9) == 81 &&  EEPROM.read(10) == 214) { //check 3 addresses if the chips memory was formated
    //if yes load the date
    transpose = EEPROM.read(0) % 12;
    fineTune = EEPROM.read(1);
  }
  else {
    //if no format the memory and write controll addresses
    EEPROM.write(0, transpose);
    EEPROM.write(1, fineTune);
    EEPROM.write(8,123);
    EEPROM.write(9,81); 
    EEPROM.write(10,214);
  }

  //serial for debugging only
  //mySerial.begin(9600);

  setTimers(); //setup interrupts

  //setup ADC and run it in interrupt
  init();
  connectChannel(analogChannelRead);
  startConversion();
  _delay_us(100);
  
  while (startupRead < 12) { //wait for all analog inputs to be read
    loop();
  }
  
  bootMode = true; //if all pots are hight and mode is HIGH than render boot mode instead
  if (analogValues[0] < HIGH_THRES)  bootMode = false;
  for (uint8_t i = 1; i < 4; i++) if (analogValues[i] < 200) bootMode = false; //HIGH_THRES

}

void loop() {

  if (bootMode) { // BOOT MODE
    decayVolume = 255;
    _xor = 0;
    _useScale = scales[4];
    if (analogValues[0] > HIGH_THRES) {
      fineTune = analogValues[WS_2];
      transpose = map( analogValues[WS_1], 0, 255, 0, 13);
      pitch = quantizeNote(curveMap(pitchAverage, PITCHMAP_POINTS, pitchMap));
      setSemitone( pitch);
    }
    else {
      bootMode = false;
      EEPROM.write(0, transpose);
      EEPROM.write(1, fineTune);
    }
  }
  else { // NORMAL LOOP
    if (analogValues[0] < (LOW_THRES - 10)) root = 2; //f major
    else if (analogValues[0] < LOW_THRES) root = 3;  // a minor
    else if (analogValues[0] > HIGH_THRES + 3) root = 6;  // g major
    else if (analogValues[0] > HIGH_THRES - 6) root = 5; //e minor
    else root = 4; //c major
    _useScale = scales[root];
    if (lastAnalogChannelRead == WS_1 && lastAnalogValues[WS_1] != analogValues[WS_1]) {
      _xor = analogValues[WS_1];
    }
    if (lastAnalogChannelRead == WS_2 && lastAnalogValues[WS_2] != analogValues[WS_2]) {
      analogValues[WS_2] = analogValues[WS_2], setDecay();
    }

    lastPitch = pitch;
    pitch = quantizeNote(curveMap(pitchAverage, PITCHMAP_POINTS, pitchMap));
    
    if (analogValues[WS_2] < 100) { 
      //when decay CCW
      //re-trigger envelope only when low enough - also update pitch only then
      if (pitch != lastPitch) {
        if (decayVolume < 100) {
          setSemitone( pitch);
          trigger();
        }
      }
    }
    else {
      //when decay CW trigger envelope at every pitch change
      setSemitone( pitch);
      if (pitch != lastPitch) {
        trigger();
      }
    }
    renderDecay();
  }

}


ISR(TIMER1_COMPA_vect)  // render both oscillators in the interupt
{
  OCR0A = sample;
  OCR0B = sample2;
  _phase += frequency;
  _phase2 += frequency2;
  _phs = _phase >> 8;
  sample = ((wavetable[_phs] ^ _xor) * runningDecay) >> 8; 
  //  sample = (wavetable[_phs] ^ _xor) ;
  sample2 = wavetable[_phase2 >> 8] ; // ^ _xor;
}



// #### FUNCTIONS TO ACCES ADC REGISTERS
void init() {

  ADMUX  = 0;
  bitWrite(ADCSRA, ADEN, 1); //adc enabled
  bitWrite(ADCSRA, ADPS2, 1); // set prescaler
  bitWrite(ADCSRA, ADPS1, 1); // set prescaler
  bitWrite(ADCSRA, ADPS0, 1); // set prescaler
  bitWrite(ADCSRA, ADIE, 1); //enable conversion finished interupt
  bitWrite(SREG, 7, 1);
  // prescaler = highest division
}

void connectChannel(uint8_t number) {
  ADMUX &= (11110000);
  ADMUX |= number;
}

void startConversion() {
  bitWrite(ADCSRA, ADSC, 1); //start conversion
}

bool isConversionFinished() {
  return (ADCSRA & (1 << ADIF));
}

bool isConversionRunning() {
  return !(ADCSRA & (1 << ADIF));
}

uint16_t getConversionResult() {
  uint16_t result = ADCL;
  return result | (ADCH << 8);
}
uint8_t pitchCount = 0;
uint16_t pitchRun[4];

ISR(ADC_vect) { // interupt triggered ad completion of ADC counter
  startupRead++;
  if (!firstRead) { // discard first reading due to ADC multiplexer crosstalk
    //update values and remember last values
    lastAnalogValues[analogChannelRead] = analogValues[analogChannelRead];
    analogValues[analogChannelRead] = getConversionResult() >> 2;
    if (analogChannelRead == PITCH) {
      lastPitchAverage = pitchAverage;
      pitchCount++;
      if (pitchCount > 3) pitchCount = 0;
      pitchRun[pitchCount] = analogValues[PITCH];
      uint16_t pitchSum = 0;
      for (uint8_t i = 0; i < 4; i++) {
        pitchSum += pitchRun[i];
      }
      pitchAverage = pitchSum >> 2;
    }
    //set ADC MULTIPLEXER to read the next channel
    lastAnalogChannelRead = analogChannelRead;
    analogChannelReadIndex++;
    if (analogChannelReadIndex > 5) analogChannelReadIndex = 0;
    analogChannelRead = analogChannelSequence[analogChannelReadIndex];

    connectChannel(analogChannelRead);
    firstRead = true;
    //start the ADC - at completion the interupt will be called again
    startConversion();

  }
  else {
    /*
      at the first reading off the ADX (which will not used)
      something else will happen the input pin will briefly turn to output to
      discarge charge built up in passive mixing ciruit using zener diodes
      because zeners have some higher unpredictable capacitance, various voltages might get stuck on the pin
    */
    if (analogValues[analogChannelRead] < 200) bitWrite(DDRB, analogToDigitalPinMapping[analogChannelRead], 1);
    bitWrite(DDRB, analogToDigitalPinMapping[analogChannelRead], 0);
    bitWrite(PORTB, analogToDigitalPinMapping[analogChannelRead], 0);
    firstRead = false;
    startConversion();
  }
}
