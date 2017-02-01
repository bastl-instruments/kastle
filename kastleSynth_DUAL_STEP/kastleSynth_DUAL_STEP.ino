
/*
KASTLE LFO v 1.0
 
 
 Features
 
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
 -Rob Hordijk for inventing the amazing concept of the Rungler
 -Ondrej Merta for being the best boss
 -and the whole bastl crew that made this project possible
 
 */

#define F_CPU 8000000  // This is used by delay.h library
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>    // Adds delay_ms and delay_us functions


//debugging purposes
//#include <SoftwareSerial.h>
//#define rxPin 5    
//#define txPin 3
//SoftwareSerial serial(rxPin, txPin);

uint8_t analogChannelRead=1;
uint16_t analogValues[3];
uint16_t lastAnalogValues[3];
//uint8_t runglerByte;
int pwmCounter;
uint16_t upIncrement=0;
uint16_t downIncrement=255;
uint32_t _upIncrement=0;
uint32_t _downIncrement=255;
uint8_t pwmIncrement;
uint8_t waveshape,lastWaveshape;
long _value;
bool goingUp;
uint16_t counter;

bool resetState=false;
//uint16_t runglerOut;
bool lastDoReset;
const uint8_t runglerMap[8]={
  0,80,120,150,180,200,220,255};

//uint16_t wsMap[10]={ 0,120,150,180,255,   20,60,120,190,254};


uint8_t analogPins[3]={
  A1,A2,A3};

uint8_t _xor;
int _val;
bool _gate;
int out,in;
bool render;
bool cycle;
const bool usePin[4]={
  true,false,true,false};
uint8_t lfoValue=0;
bool lfoFlop=true;
bool doReset=false;
bool firstRead=false;
const uint8_t analogToDigitalPinMapping[4]={
  7,PORTB2,PORTB4,PORTB3};


uint16_t wsMap[10]={ 
  0,60,127,191,255,   50,127,190,230,254};

#define WSMAP_POINTS 5

uint8_t mapLookup[256];

void createLookup(){
  for(uint16_t i=0;i<256;i++){
    mapLookup[i]=curveMap(i,WSMAP_POINTS,wsMap);
  }
}


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

uint8_t byte1, byte2, out1, out2;
void setup()  { 
  digitalWrite(5,HIGH);
  pinMode(4, INPUT);
  digitalWrite(4,HIGH);
  createLookup();
  setTimers(); //setup audiorate interrupt
  out1=random(255);
  out2=random(255);
  pinMode(0, OUTPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  digitalWrite(4,LOW);
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
  TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
  TCCR0B = 0<<WGM02 | 1<<CS00;

  //  setup timer 0 to run fast for audiorate interrupt 
  TCCR1 = 0;                  //stop the timer
  TCNT1 = 0;                  //zero the timer
  GTCCR = _BV(PSR1);          //reset the prescaler
  OCR1A = 250;                //set the compare value
  // OCR1C = 31;
  TIMSK = _BV(OCIE1A);        //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1    
  TCCR1 = _BV(CTC1) | _BV(CS12);//  | _BV(CS11) ;//| _BV(CS10); //| _BV(CS13) | _BV(CS12) | _BV(CS11) |
  bitWrite(TCCR1,CS12,0);
  sei();
}

uint8_t bitInState;
ISR(ADC_vect){
  if(!firstRead){
    lastAnalogValues[analogChannelRead]=analogValues[analogChannelRead];
    analogValues[analogChannelRead]= getConversionResult();
    if(analogChannelRead==2 &&  lastAnalogValues[2]!= analogValues[2]) setFrequency(mapLookup[analogValues[2]>>2]);
    if(analogChannelRead==0 &&  lastAnalogValues[0]!= analogValues[0]){
      if((analogValues[0]>>2)<150){
        bitInState=0;
      }
      else if((analogValues[0]>>2)>162){
        bitInState=2;
      }
      else{
        bitInState=1;
      }
    }
    analogChannelRead++;
    while(!usePin[analogChannelRead]){
      analogChannelRead++;
      if(analogChannelRead>3) analogChannelRead=0;
    }

    if(analogChannelRead>2) analogChannelRead=2;
    connectChannel(analogChannelRead);
    firstRead=true;
    startConversion();

  }
  else {
    if(analogChannelRead==2){
      if(analogValues[analogChannelRead]<750) bitWrite(DDRB,analogToDigitalPinMapping[analogChannelRead],1);
      bitWrite(DDRB,analogToDigitalPinMapping[analogChannelRead],0);
      bitWrite(PORTB,analogToDigitalPinMapping[analogChannelRead],0);
    }
    firstRead=false;
    startConversion();
  }

}


void loop() { 
  //pure nothingness
}

uint8_t processRunglerByte(uint8_t runglerByte){

  bool newBit= bitRead(runglerByte,7) ;
  runglerByte=runglerByte<<1;
  if(bitInState==0){
    newBit=newBit;
  }
  else if(bitInState==2){
    newBit=TCNT0>>7;
  }
  else newBit=!newBit;
  bitWrite(runglerByte,0,newBit);
  return runglerByte;
}
uint8_t calculateRunglerOut(uint8_t runglerByte){
  uint8_t runglerOut=0;
  bitWrite(runglerOut,0,bitRead(runglerByte,0));
  bitWrite(runglerOut,1,bitRead(runglerByte,3));
  bitWrite(runglerOut,2,bitRead(runglerByte,5));
  runglerOut=runglerMap[runglerOut]; 
  return runglerOut;
}

uint16_t pattern[3]={
  word(B00010000,B01000001), word(B00010001,B00010001),word(B0000001,B00000001)} 
;
uint8_t lastLfoValue;
ISR(TIMER1_COMPA_vect)  //audiorate interrupt
{
  lastLfoValue=lfoValue;
  lfoValue++;
  if(lfoValue>15) lfoValue=0;

  bitWrite(PORTB,PINB2, bitRead(pattern[bitInState],lfoValue));


  doReset=bitRead(PINB,PINB3);
  if(!lastDoReset && doReset) {
    lfoValue=0, lfoFlop=0;
  }
  lastDoReset=doReset;

  if(lfoValue==0){
    byte1=processRunglerByte(byte1);
    out1=calculateRunglerOut(byte1);
    OCR0A= out1;
  }
  if(lfoValue>>3 != lastLfoValue>>3){
    byte2=processRunglerByte(byte2);
    out2=calculateRunglerOut(byte2);
  }
  OCR0B= out2;//constrain(out,0,255);

  TCNT1 = 0; 
}

void setFrequency(int _freq){
  _freq=(2048-(_freq<<3))+20;
  uint8_t preScaler=_freq>>7;
  preScaler+=1; //*2
  for(uint8_t i=0;i<4;i++) bitWrite(TCCR1,CS10+i,bitRead(preScaler,i)); 
  uint8_t compare=_freq;
  bitWrite(compare,7,0);
  OCR1A=compare+128; 
}


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





