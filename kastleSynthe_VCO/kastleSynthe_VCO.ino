
/*
KASTLE VCO v 1.0


Features
-3 synthesis modes = phase modulation, phase distortion, tarck & hold modulation
-regular waveform output by PWM
-square wave output
-3 sound parameters controlled by voltage inputs
-voltage selectable synthesis modes on weak I/O Reset pin


ideas 
 -transfer wavetable to ram and see if it is faster
 -update parameters at the beginning of waveoform cycle to see if it improoves stability
 -dynamicly lower the resolution of the VCO
 
Writen by Vaclav Pelousek 2016
open source license: CC BY SA
http://www.bastl-instruments.com
 
 
-software written in Arduino 1.0.6 - used to flash ATTINY 85 running at 8mHz
-created with help of the heavenly powers of internet and several tutorials that you can google out
-i hope somebody finds this code usefull

thanks to 
-Lennart Schierling for making some work on the register access
-Uwe Schuller for explaining the capacitance of zener diodes
-Peter Edwards for making the inspireing bitRanger
-Ondrej Merta for being the best boss
-and the whole bastl crew that made this project possible

 */
 
#define F_CPU 8000000  // This is used by delay.h library
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>

#include "SINE.h" //sinewave wavetable - modified but originnaly from the Mozzi library
//#include <SAW.h>
//#include <CHEB4.h>


//for debugging purposes
#include "SoftwareSerial.h" 
const int Rx = 8; 
const int Tx = 3; 
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
uint8_t mapLookup[256];
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
  6,PORTB2,PORTB4,PORTB3};
  
  
//defines for synth types
//all are dual oscillator setups - 
#define PHASE_DIST 0 // phase distortion -
#define FM 1 //aka phase modulation
#define TAH 2 //aka track & hold modulation (downsampling with T&H)


#define LOW_THRES 150
#define HIGH_THRES 162
#define LOW_MIX 300
#define HIGH_MIX 900

//defines for synth parameters
#define PITCH 2
#define WS_1 3
#define WS_2 1


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
    mapLookup[i]=curveMap(i,WSMAP_POINTS,wsMap);
  }
}



void setup()  { //happends at the startup
  digitalWrite(5,HIGH); //turn on pull up resistor for the reset pin
  createLookup(); //mapping of knob values is done thru a lookuptable which is generated at startup from defined values
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

  cli();                               // Interrupts OFF (disable interrupts globally)
  
  
  TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
  TCCR0B = 0<<WGM02 | 1<<CS00;

  //  setup timer 0 to run fast for audiorate interrupt 
  TCCR1 = 0;                  //stop the timer
  TCNT1 = 0;                  //zero the timer
  GTCCR = _BV(PSR1);          //reset the prescaler
  OCR1A = 250;                //set the compare value
  // OCR1C = 31;
  TIMSK = _BV(OCIE1A);// | _BV(TOIE0);    //TOIE0    //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1    
  TCCR1 = _BV(CTC1) | _BV(CS12) ;//| _BV(CS11);//  | _BV(CS11) ;// //| _BV(CS13) | _BV(CS12) | _BV(CS11) |
  sei();
  TIMSK |=_BV(TOIE0);   
}

uint16_t clocks(){
  //return _clocks;
  return TCNT0|(_clocks<<8);
}

ISR(TIMER0_OVF_vect){ // increment _clocks at PWM interrupt overflow - this gives 16bit time consciousnes to the chip (aproxx 2 seconds before overflow)
  _clocks++; 
}


ISR(TIMER1_COMPA_vect)  // render primary oscillator in the interupt
{
 
  out+=incr; // increment the oscillator saw core
  if((analogValues[WS_2])<pwmCounter) OCR0B=255; //render pulse oscillator output
  else OCR0B=0;
  
  pwmCounter=out<<1; // speed up the frequency of the square wave output twice
  TCNT1 = 0; //reset interupt couter
}



void loop() { 

  if(mode==PHASE_DIST){

    if(out<incr){ //sync oscillators
      _out=0;
    }
    uint8_t waveShape=(analogValues[WS_2])<<1;
    if(analogValues[WS_2]>170) {
      if(out!=lastOut) OCR0A =   (((256-out)|waveShape) * (((char)pgm_read_byte_near(SIN256_DATA+_out))+128))>>8;  
    }
    else{
      OCR0A =   (((256-out)|waveShape) * (((char)pgm_read_byte_near(SIN256_DATA+_out))+128))>>8;
    }
    lastOut=out;
  }

  if(mode==FM){
    uint8_t _mod= ((((analogValues[WS_2])+1)*((char)pgm_read_byte_near(SIN256_DATA+_out)+128))>>6) +out;
    OCR0A=  (char)pgm_read_byte_near(SIN256_DATA+_mod)+128;
  }

  if(mode==TAH){
    if(out>(analogValues[WS_2]-15)){
      OCR0A=  (char)pgm_read_byte_near(SIN256_DATA+_out)+128;
    }
  }

  if(clocks()-time>((osc2offset-analogValues[WS_1])<<bitShift)){ // render secondary oscillator in the loop
   time=clocks(); 
    _out+=_incr; // increment the oscillator saw core
  }
  else{
    if(analogValues[0]<LOW_THRES)  mode=PHASE_DIST, incr=11,_incr=6, bitShift=2, osc2offset=270; 
    else if(analogValues[0]>HIGH_THRES) mode=TAH, incr=24,_incr=6,bitShift=4,osc2offset=255;
    else mode = FM, incr=11,_incr=5,bitShift=4,osc2offset=255;
  }
  //_delay_us(250);
}




ISR(ADC_vect){ // interupt triggered ad completion of ADC counter
  if(!firstRead){ // discard first reading due to ADC multiplexer crosstalk
  //update values and remember last values
    lastAnalogValues[analogChannelRead]=analogValues[analogChannelRead];
    analogValues[analogChannelRead]= getConversionResult()>>2;
  //set ADC MULTIPLEXER to read the next channel
    lastAnalogChannelRead=analogChannelRead;
    analogChannelRead++;
    if(analogChannelRead>3) analogChannelRead=0;
    connectChannel(analogChannelRead);
  // set controll values if relevant (value changed)
    if(lastAnalogChannelRead==PITCH && lastAnalogValues[PITCH]!=analogValues[PITCH]) setFrequency(constrain(mapLookup[analogValues[PITCH]]<<2,0,1015));
    if(lastAnalogChannelRead==WS_1 && lastAnalogValues[WS_1]!=analogValues[WS_1]) analogValues[WS_1]= mapLookup[analogValues[WS_1]];
    if(lastAnalogChannelRead==WS_2 && lastAnalogValues[WS_2]!=analogValues[WS_2]) analogValues[WS_2]= mapLookup[analogValues[WS_2]];
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
    if( mode==PHASE_DIST && analogChannelRead!=PITCH){ //this would somehow make the  reset pin trigger because the reset pin is already pulled to ground
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



// #### FUNCTIONS TO ACCESS ADC REGISTERS
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
