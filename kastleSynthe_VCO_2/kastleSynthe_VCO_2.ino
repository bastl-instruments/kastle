
/*
KASTLE VCO v 1.5
 
 
 Features
 -5 synthesis modes = phase modulation, phase distortion, tarck & hold modulation, formant synthesis, noise wtf
 -regular & alternative waveform output by 2 PWM channels
 -3 sound parameters controlled by voltage inputs
 -voltage selectable synthesis modes on weak I/O Reset pin

 
 Writen by Vaclav Pelousek 2016
 open source license: CC BY SA
 http://www.bastl-instruments.com
 
 
 -software written in Arduino 1.0.6 - used to flash ATTINY 85 running at 8mHz
 -created with help of the heavenly powers of internet and several tutorials that you can google out
 -i hope somebody finds this code usefull (i know it is a mess :( )
 
 thanks to 
 -Lennart Schierling for making some work on the register access
 -Uwe Schuller for explaining the capacitance of zener diodes
 -Peter Edwards for making the inspireing bitRanger
 -Ondrej Merta for being the best boss
 -and the whole bastl crew that made this project possible
 -Arduino community and the forums for picking ups bits of code to setup the timers & interrupts
 -v1.5 uses bits of code from miniMO DCO http://www.minimosynth.com/
 
 
 */

#define F_CPU 8000000  // This is used by delay.h library
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>

//#include <CB_AT.h>
//#include <CB2_AT.h>
//#include <CB3_AT.h>
//#include <CB4_AT.h>
//#include <GB_BLIP_AT.h>
//#include <GB_BZZ_AT.h>
//#include <GB_GLITCH_AT.h>
//#include <GB_HH_AT.h>
//#include <GB_KICK_AT.h>
//#include <GB_SNARE_AT.h>
//#include <GB_TUI_AT.h>
//#include <GL_A_AT.h>
//#include <GL_B_AT.h>
//#include <GL_C_AT.h>
//#include <GL_D_AT.h>
//#include <GL_E_AT.h>
//#include <GL_F_AT.h>
//#include <GL_G_AT.h>
//#include <GL_H_AT.h>
//#include <GL_I_AT.h>
//#include <HAT_AT.h>
//#include <HAT2_AT.h>
//#include <KICK_AT.h>
//#include <KICK2_AT.h>
//#include <RIDE_AT.h>
//#include <SNARE_AT.h>
//#include <SNARE2_AT.h>
//#include <TR_CB_AT.h>
//#include <TR_CLAP_AT.h>
#include <TR_HH_AT.h>
//#include <TR_KICK_AT.h>
//#include <TR_OH_AT.h>
//#include <TR_RIM_AT.h>
//#include <TR_SNARE_AT.h>
//#include <TR_TOM_AT.h>

//#include "SINE.h" //sinewave wavetable - modified but originnaly from the Mozzi library
//#include <SAW.h>
//#include <CHEB4.h>


//for debugging purposes
//#include "SoftwareSerial.h" 
//const int Rx = 8; 
//const int Tx = 3; 
//SoftwareSerial mySerial(Rx, Tx); //use only for debugging

//global variables
#define WSMAP_POINTS 5
uint16_t wsMap[10]={
  0,63,127,191,234,   15,100,160,210,254};


uint8_t _out;
uint16_t time;
uint8_t mode;
uint8_t analogChannelRead=1;
uint8_t analogValues[4];
uint8_t lastAnalogValues[4];
uint8_t out;
uint8_t pwmCounter;
//uint8_t mapLookup[256];
uint8_t _clocks;
bool flop;
uint8_t incr=6,_incr=6;
uint8_t lastOut;
uint8_t bitShift=3;
uint16_t osc2offset=255;
uint8_t lastAnalogChannelRead;
bool firstRead=false;

uint8_t pwmIncrement,_upIncrement,_downIncrement,upIncrement,downIncrement;
bool quantizer;
const uint8_t analogToDigitalPinMapping[4]={ 
  PORTB5,PORTB2,PORTB4,PORTB3};


//defines for synth types
//all are dual oscillator setups - 
#define NOISE 1 // phase distortion -
#define FM 0 //aka phase modulation
#define TAH 2 //aka track & hold modulation (downsampling with T&H)


#define LOW_THRES 150
#define HIGH_THRES 162
#define LOW_MIX 300
#define HIGH_MIX 900

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


uint32_t curveMap(uint8_t value, uint8_t numberOfPoints, uint16_t * tableMap){
  uint32_t inMin=0, inMax=255, outMin=0, outMax=255;
  for(int i=0;i<numberOfPoints-1;i++){
    if(value >= tableMap[i] && value <= tableMap[i+1]) {
      inMax=tableMap[i+1];
      inMin=tableMap[i];
      outMax=tableMap[numberOfPoints+i+1];
      outMin=tableMap[numberOfPoints+i];
      i=numberOfPoints+10;
    }
  }
  return map(value,inMin,inMax,outMin,outMax);
}


void createLookup(){
  for(uint16_t i=0;i<256;i++){
    // mapLookup[i]=curveMap(i,WSMAP_POINTS,wsMap);
  }
}


bool XYmode;
uint8_t startupRead=0;
void setup()  { //happends at the startup
  writeWave(0);
  digitalWrite(5,HIGH); //turn on pull up resistor for the reset pin
  // createLookup(); //mapping of knob values is done thru a lookuptable which is generated at startup from defined values
  //set outputs
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  //serial for debugging only
  //mySerial.begin(9600);

  setTimers(); //setup interrupts

    //setup ADC and run it in interrupt
  init();
  connectChannel(analogChannelRead);
  startConversion();
  //long _time=millis();
  /*
while(millis()-_time<4){
   loop(); 
   }
   */
      _delay_us(4000);
  // while(startupRead<12){ _delay_us(10); }
  //  } //read voltages
  XYmode=true; //if all pots are hight and mode is HIGH than render XY mode instead
  if(analogValues[0]<HIGH_THRES)  XYmode=false;
  for(uint8_t i=1;i<4;i++) if(analogValues[i]<200) XYmode=false; //HIGH_THRES

} 

void setTimers(void)
{
  /*
  TCCR0A=0;
   TCCR0B=0;
   bitWrite(TCCR0A,COM0A0,0);
   bitWrite(TCCR0A,COM0A1,1);
   bitWrite(TCCR0A,COM0B0,0);
   bitWrite(TCCR0A,COM0B1,1);
   bitWrite(TCCR0A,WGM00,1);
   bitWrite(TCCR0A,WGM01,1);
   bitWrite(TCCR0B,WGM02,0);
   bitWrite(TCCR0B,CS00,1);
   */
  PLLCSR |= (1 << PLLE);               // Enable PLL (64 MHz)
  _delay_us(100);                      // Wait for a steady state
  while (!(PLLCSR & (1 << PLOCK)));    // Ensure PLL lock
  PLLCSR |= (1 << PCKE);               // Enable PLL as clock source for timer 1
  PLLCSR |= (1 << LSM); //low speed mode 32mhz
  cli();                               // Interrupts OFF (disable interrupts globally)


  TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
  TCCR0B = 0<<WGM02 | 1<<CS00;

  //  setup timer 0 to run fast for audiorate interrupt 

  TCCR1 = 0;                  //stop the timer
  TCNT1 = 0;                  //zero the timer
  GTCCR = _BV(PSR1);          //reset the prescaler
  OCR1A = 255;                //set the compare value
  OCR1C = 255;    
  // OCR1C = 31;
  TIMSK = _BV(OCIE1A);// | _BV(TOIE0);    //TOIE0    //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1    
  TCCR1 = _BV(CTC1) | _BV(CS12);// | _BV(CS10) ;//| _BV(CS11);//  | _BV(CS11) ;// //| _BV(CS13) | _BV(CS12) | _BV(CS11) |

  // GTCCR  = (1 << PWM1B) | (1 << COM1B1); // PWM, output on pb4, compare with OCR1B (see interrupt below), reset on match with OCR1C
  //OCR1C  = 0xff;                         // 255
  // TCCR1  = (1 << CS10);                  // no prescale

  sei();
  // TIMSK |=_BV(TOIE0);   
}

uint16_t clocks(){
  //return _clocks;
  return TCNT0|(_clocks<<8);
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
void sineWave() {                                       //too costly to calculate on the fly, so it reads from the sine table. We use 128 values, then mirror them to get the whole cycle
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
/*
ISR(TIMER0_OVF_vect){ // increment _clocks at PWM interrupt overflow - this gives 16bit time consciousnes to the chip (aproxx 2 seconds before overflow)
 _clocks++; 
 }
 */
byte sample, sample90;
unsigned int _phase, _lastPhase;
unsigned int frequency;
byte sample2;
unsigned int _phase2, _phase4, _phase5,_phase6;
unsigned int frequency2, frequency4, frequency5, frequency6;
uint8_t _phs,_phs90;
uint8_t _phase3;
/*
ISR(TIMER0_OVF_vect){    //not used               //Timer 0 interruption - changes the width of timer 1's pulse to generate waves
 //  OCR1B
 
 
 
 
 // analogValues[WS_2]
 //phase accumulator
 }
 */
ISR(TIMER1_COMPA_vect)  // render primary oscillator in the interupt
{
  OCR0A = sample;//(sample+sample2)>>1;
  OCR0B = sample2;//_phs;// sample90;

  //_lastPhase=_phase;
  _phase += frequency;   

  _phase2 += frequency2;
  _phase4 += frequency4;
  _phase5 += frequency5;
  if(mode){ // other than FM, FM=0
    _phase3=_phase2>>5;
    //(frequency2+1)<<1;

    //(frequency2-1)*3;
    _phase6 += frequency6;
  }
  else{
    _phs=(_phase+(analogValues[WS_2]*wavetable[_phase2 >> 8])) >> 6; 
    sample = (wavetable[_phs] );
  }



  /*
  out+=incr; // increment the oscillator saw core
   if((analogValues[WS_2])<pwmCounter) OCR0B=255; //render pulse oscillator output
   else OCR0B=0;
   
   pwmCounter=out<<1; // speed up the frequency of the square wave output twice
   TCNT1 = 0; //reset interupt couter
   */
}

uint16_t sampleEnd;
const uint8_t multiplier[24]={
  2,2,2,3,1,2,4,4, 3,4,5,2,1,5,6,8, 3,8,7,8,7,6,8,16};
void setFrequency2(uint16_t input) {
  if(   mode==NOISE) frequency2 = (((input-300)<<2)+1)/2;//sampleEnd=map(input,300,1024,0,sampleLength);//
  else if (   mode==TAH){
    uint8_t multiplierIndex=analogValues[WS_2]>>5;
    frequency2 = (input<<2)+1;
    frequency4=(frequency2+1)*multiplier[multiplierIndex]; //+analogValues[WS_2]>>4
    frequency5=(frequency2-3)*multiplier[multiplierIndex+8];//+analogValues[WS_2]>>3
    frequency6=(frequency2+7)*multiplier[multiplierIndex+16];//+analogValues[WS_2]>>2
  }
  else{
    frequency2 = (input<<2)+1;
    frequency4=frequency2;
    frequency5=frequency2;
  }
}

void setLength(uint8_t _length){
  sampleEnd=map(_length,0,255,0,sampleLength);
}

void setFrequency(uint16_t input) {
  if(   mode==NOISE) frequency = ((input-200)<<2)+1;
  else frequency = (input<<2)+1;
  /*
  coarseVolChange = false;                            //reset the control condition for volume
   if (coarseFreqChange == false) {
   byte coarsefreqRead = analogRead(pin) >> 2;
   if (coarsefreqRead == potPosFreqRef) {
   coarseFreqChange = true;
   }
   }
   if (coarseFreqChange == true) {
   int tempRead = analogRead(pin);
   byte freqRead = tempRead >> 2;
   potPosFreqRef = freqRead;
   //frequencymap(tempRead, sensorMin, sensorMax, 1, 3600); //map the calibrated values (by default 0-1023) to the frequency range we want
   }
   */
}

int ultimateFold(int _input){
  int _output=_input;
  while(_output>255 || _output<0){
    if(_output>255) _output=255-(_output-255);
    if(_output<0) _output=0-_output;
  }
  return _output;
}
uint8_t decayVolume;
uint8_t decayTime;
uint8_t _sample;
uint8_t _saw, _lastSaw;
void synthesis(){
  if(mode==FM){

    if(XYmode){
      _phs90=_phs+64;
      sample2 = (wavetable[_phs90] );
    }
    else{
      _lastSaw=_saw;
      _saw=(((255-(_phase>>8))*(analogValues[WS_2]))>>8);
      // uint8_t _p=(_phase4 >> 8)+128;
      sample2 = ((_saw*wavetable[_phase4 >> 8] )>>8)+((wavetable[_phase5 >> 8]*(255-analogValues[WS_2]))>>8);
      if(_lastSaw<_saw) _phase4=64<<8;
      uint8_t shft=abs(_saw-_lastSaw);
      if(shft>3) _phase5+=shft<<8;
    }
    
  }

  if(mode==NOISE){
    if((_phase>>2)>=(analogValues[WS_2]-100)<<5){
      _phase=0;
    }
    _sample = (char)pgm_read_byte_near(sampleTable+(_phase>>2));
    _sample=(_sample*wavetable[_phase2>>8])>>8;
    sample=_sample;
    sample2 = (wavetable[_phase3+(_phase>>8)]);
  }


  if(mode==TAH){
    if((_phase2 >> 8)>analogValues[WS_2])  _phs=_phase>>8, sample = (wavetable[_phs] );
    if(XYmode){
      _phs90=_phs+64;
      sample2 = (wavetable[_phs90] );
    }
    else{
      sample2 = (wavetable[_phase2 >>8]+ wavetable[_phase4 >>8] + wavetable[_phase5 >>8]+ wavetable[_phase6 >>8])>>2;
    }
  } 
}
void loop() { 
//if(analogValues[WS_2]<20) analogValues[WS_2]=0;
  //FM + PHS DIST
  // t&h + chord
  // noise sample 
  synthesis();

  modeDetect();

}
void modeDetect(){
  if(analogValues[0]<LOW_THRES)  mode=NOISE; //, incr=11,_incr=6, bitShift=2, osc2offset=270; 
  else if(analogValues[0]>HIGH_THRES) mode=TAH; //, incr=24,_incr=6,bitShift=4,osc2offset=255;
  else mode = FM; //, incr=11,_incr=5,bitShift=4,osc2offset=255; 
}


uint8_t analogChannelSequence[6]={0,1,0,2,0,3};
uint8_t analogChannelReadIndex;
ISR(ADC_vect){ // interupt triggered ad completion of ADC counter
  startupRead++;
  if(!firstRead){ // discard first reading due to ADC multiplexer crosstalk
    //update values and remember last values
    lastAnalogValues[analogChannelRead]=analogValues[analogChannelRead];
    analogValues[analogChannelRead]= getConversionResult()>>2;
    //set ADC MULTIPLEXER to read the next channel
    lastAnalogChannelRead=analogChannelRead;
    if(!analogChannelRead) modeDetect();
    analogChannelReadIndex++;
    if(analogChannelReadIndex>5) analogChannelReadIndex=0;
    analogChannelRead=analogChannelSequence[analogChannelReadIndex];
    connectChannel(analogChannelRead);
    // set controll values if relevant (value changed)
    if(lastAnalogChannelRead==PITCH && lastAnalogValues[PITCH]!=analogValues[PITCH]) setFrequency(analogValues[PITCH]<<2);//constrain(mapLookup[,0,1015));
    if(lastAnalogChannelRead==WS_1 && lastAnalogValues[WS_1]!=analogValues[WS_1])  setFrequency2(analogValues[WS_1]<<2);
    if(lastAnalogChannelRead==WS_2 && lastAnalogValues[WS_2]!=analogValues[WS_2]) analogValues[WS_2]= analogValues[WS_2];
    firstRead=true;
    //start the ADC - at completion the interupt will be called again
    startConversion();

  }
  else{ 
    /* 
     at the first reading off the ADX (which will not used) 
     something else will happen the input pin will briefly turn to output to 
     discarge charge built up in passive mixing ciruit using zener diodes
     because zeners have some higher unpredictable capacitance, various voltages might get stuck on the pin
     */
    //  if(analogChannelRead!=0) - by mohlo vyresit celou situaci
    // if( mode==PHASE_DIST && analogChannelRead!=PITCH){  // //this would somehow make the  reset pin trigger because the reset pin is already pulled to ground



    if( mode==NOISE){// && analogChannelRead!=PITCH){//if(analogChannelRead==0){  
      //  bitWrite(PORTB,analogToDigitalPinMapping[analogChannelRead],1);
      // renderDecay();
    }

    else{ 
      if(analogValues[analogChannelRead]<200) bitWrite(DDRB,analogToDigitalPinMapping[analogChannelRead],1);
      bitWrite(DDRB,analogToDigitalPinMapping[analogChannelRead],0);
      bitWrite(PORTB,analogToDigitalPinMapping[analogChannelRead],0);
    } 
    firstRead=false;
    startConversion();

  }
}


uint8_t decayCounter=0;
void renderDecay(){
  if(decayTime!=0){
    if(0){
      decayCounter+=8;
      if(decayCounter>=decayTime)
      {
        decayCounter=0;
        if(decayVolume>0) decayVolume--;
        else decayVolume=255, _phase=0;
      }
    }
  } 
}

/*
void setFrequency(int _freq){ //set frequency of the interupt for primary oscillator
 _freq=1024-_freq;
 uint8_t preScaler=_freq>>7;
 preScaler+=2; //*2
 pwmIncrement=4;
 _upIncrement=pwmIncrement*upIncrement;
 _downIncrement=pwmIncrement*downIncrement;
 for(uint8_t i=0;i<4;i++) bitWrite(TCCR1,CS10+i,bitRead(preScaler,i)); 
 uint8_t compare=_freq;
 bitWrite(compare,7,0);
 if(quantizer){ // if quantizer was implemented
 //compare=tuneTable[map(compare,0,127,0,6)];
 }
 OCR1A=compare+128; 
 }
 */


// #### FUNCTIONS TO ACCES ADC REGISTERS
void init() {

  ADMUX  = 0;
  bitWrite(ADCSRA,ADEN,1); //adc enabled
  bitWrite(ADCSRA,ADPS2,1); // set prescaler
  bitWrite(ADCSRA,ADPS1,1); // set prescaler
  bitWrite(ADCSRA,ADPS0,1); // set prescaler
  bitWrite(ADCSRA,ADIE,1); //enable conversion finished interupt
  bitWrite(SREG,7,1);
  // prescaler = highest division
}


// channel 8 can be used to measure the temperature of the chip
void connectChannel(uint8_t number) {
  ADMUX &= (11110000);
  ADMUX |= number;
}

void startConversion() {
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









