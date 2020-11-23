
/*
KASTLE LFO v 1.0


Kastle Drum Features 
  -8 drum synthesis styles
  -”noises” output for less tonal content
  -DRUM selects drum sounds
  -acceleration charge dynamic envelope
  -decay time
  -PITCH control with offset and CV input with attenuator
  -voltage-controllable clock with square and triangle output
  -stepped voltage generator with random, 8 step and 16 step loop mode
  -2 I/O CV ports that can be routed to any patch point 
  -the main output can drive headphones
  -3x AA battery operation or USB power selectable by a switch
  -open source
  -durable black & gold PCB enclosure



  Writen by Vaclav Pelousek 2020
  based on the earlier kastle v1.5
  open source license: CC BY SA
  http://www.bastl-instruments.com

  -this is the code for the LFO chip of the Kastle
  -software written in Arduino 1.8.12 - used to flash ATTINY 85 running at 8mHz
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
  -v1.5 and kastle drum uses bits of code from miniMO DCO http://www.minimosynth.com/

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
uint8_t runglerByte;
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
uint16_t runglerOut;
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
  0,50,127,191,255,   80,157,180,220,254};

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


void setup()  { 
  digitalWrite(5,HIGH);
  pinMode(4, INPUT);
  digitalWrite(4,HIGH);
  createLookup();
  setTimers(); //setup audiorate interrupt
  runglerByte=random(255);
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
  TCCR1 = _BV(CTC1) |   _BV(CS12);//_BV(CS10)  | _BV(CS11) ;// //| _BV(CS10); //| _BV(CS13) | _BV(CS12) | _BV(CS11) |
  bitWrite(TCCR1,CS12,0);
  sei();
}

  
ISR(ADC_vect){
  if(!firstRead){
    lastAnalogValues[analogChannelRead]=analogValues[analogChannelRead];
    analogValues[analogChannelRead]= getConversionResult();
    if(analogChannelRead==2 &&  lastAnalogValues[2]!= analogValues[2]) setFrequency(mapLookup[analogValues[2]>>2]);

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

uint32_t resetHappened=0;
bool makeReset=false;


void renderRungler(){
  bool newBit= bitRead(runglerByte,7) ;
    runglerByte=runglerByte<<1;
    if((analogValues[0]>>2)<150){
      newBit=newBit;
    }
    else if((analogValues[0]>>2)>162){
      newBit=TCNT0>>7;
    }
    else newBit=!newBit;
    bitWrite(runglerByte,0,newBit);
    runglerOut=0;
    bitWrite(runglerOut,0,bitRead(runglerByte,0));
    bitWrite(runglerOut,1,bitRead(runglerByte,3));
    bitWrite(runglerOut,2,bitRead(runglerByte,5));
    runglerOut=runglerMap[runglerOut];
}

bool lastSquareState, squareState;
void loop() { 
 //pure nothingness
 doReset=bitRead(PINB,PINB3);
  if(!lastDoReset && doReset) {
    resetHappened++;
    if(resetHappened>8) makeReset=false;
    if(makeReset) lfoValue=0, lfoFlop=0;
    renderRungler();
  }
  lastDoReset=doReset;

  lastSquareState=squareState;
  if(lfoValue<128) squareState=1;
  else squareState=0;

  if(lastSquareState!=squareState){
    if(makeReset) renderRungler();
    bitWrite(PORTB,PINB2, squareState);
  }

  
  OCR0B= constrain(out,0,255);
  OCR0A= runglerOut;
  
}



ISR(TIMER1_COMPA_vect)  //audiorate interrupt
{
  lfoValue++;
  
  
  if(lfoValue==0){
    
    lfoFlop=!lfoFlop;
    if(lfoFlop){
      if(resetHappened>4) makeReset=false, resetHappened=0;
      else makeReset=true, resetHappened=0;
    }
   
    
  }
  if(lfoFlop) out =255-lfoValue;
  else out=lfoValue;
  
  
  
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
