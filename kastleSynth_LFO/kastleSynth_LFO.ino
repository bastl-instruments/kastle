

#define F_CPU 8000000  // This is used by delay.h library
//#include "fastAnalogRead.h"
#include <stdlib.h>
//#include <EEPROM.h>
#include <avr/interrupt.h>
//#include <expADSR.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>    // Adds delay_ms and delay_us functions
//#include <Oscil.h>
// Only use Serial if using ATTiny85
// Serial output connections:
//#include <SoftwareSerial.h>
//#define rxPin 5    // We use a non-existant pin as we are not interested in receiving data
//#define txPin 3
//SoftwareSerial serial(rxPin, txPin);
uint8_t analogChannelRead=1;
uint16_t analogValues[3];
uint16_t lastAnalogValues[3];
uint8_t runglerByte;

uint16_t wsMap[10]={
  0,120,150,180,255,   20,60,120,190,254};
#define WSMAP_POINTS 5

uint8_t mapLookup[256];
void createLookup(){
  for(uint16_t i=0;i<256;i++){
    mapLookup[i]=curveMap(i,WSMAP_POINTS,wsMap);
  }
  // mapLookup[255]=curveMap(255,WSMAP_POINTS,wsMap);
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
  // digitalWrite(0,LOW);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, INPUT);
   digitalWrite(4,LOW);
  pinMode(4, INPUT);

  // digitalWrite(1,LOW);

  //  pinMode(1, INPUT_PULLUP);
  // digitalWrite(1,LOW);
  //  analogRead(A3);

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
ISR(ADC_vect){

  // if(isConversionFinished()){


  // if(analogChannelRead==0 &&  analogValues[0]>650 && lastAnalogValues[0]<=650) doReset=true;
  //if(analogChannelRead==PITCH && lastAnalogValues[PITCH]!=analogValues[PITCH]) setFrequency(analogValues[PITCH]);
  // if(analogChannelRead==WS_1 && lastAnalogValues[WS_1]!=analogValues[WS_1]) analogValues[WS_1]=map(analogValues[WS_1],LOW_MIX,HIGH_MIX,0,1024);
  // if(analogChannelRead==WS_2 && lastAnalogValues[WS_2]!=analogValues[WS_2]) analogValues[WS_2]=map(analogValues[WS_2],LOW_MIX,800,0,1024);

  if(!firstRead){
    lastAnalogValues[analogChannelRead]=analogValues[analogChannelRead];
    analogValues[analogChannelRead]= getConversionResult();
    if(analogChannelRead==2 &&  lastAnalogValues[2]!= analogValues[2]) setFrequency(mapLookup[analogValues[2]>>2]);

    analogChannelRead++;
    while(!usePin[analogChannelRead]){
      analogChannelRead++;
      if(analogChannelRead>3) analogChannelRead=0;
    }
    /*
    while(readADC[analogChannelRead]){
     analogChannelRead++;
     if(analogChannelRead>3) analogChannelRead=1;
     }
     */
    if(analogChannelRead>2) analogChannelRead=2;
    firstRead=true;
  }
  else{
    firstRead=false;
  }
  connectChannel(analogChannelRead);
  startConversion();


  //  }

}


void loop() { 
  // bitRead(PINB,PINB1);
  /*
  if(isConversionFinished()){
   lastAnalogValues[analogChannelRead-1]=analogValues[analogChannelRead-1];
   analogValues[analogChannelRead-1]= getConversionResult();
   analogChannelRead++;
   if(analogChannelRead>2) analogChannelRead=2;
   connectChannel(analogChannelRead);
   startConversion();
   }
   */

  // if( lastAnalogValues[2]!= analogValues[2]) setFrequency(analogValues[2]);



  // _delay_us(250);


}
void _digitalWrite(uint8_t _state){
  bitWrite(PORTB,PB1,_state);
  //if(_state==0) PORTB &= ~(1<<PB1);
  // else  PORTB |= 1<<PB1;
}

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

//divider++;
// if(dividerCounter
/*
  switch(divider){
 case 0:
 if(counter>127) counter=0;
 break; 
 case 1:
 break;
 }
 */
//render square output




bool resetState=false;
uint16_t runglerOut;
bool lastDoReset;
const uint8_t runglerMap[8]={
  0,80,120,150,180,200,220,255};
ISR(TIMER1_COMPA_vect)  //audiorate interrupt
{


  lfoValue++;
  //if(lfoFloú)
  //else
  if(lfoFlop && lfoValue<200) bitWrite(PORTB,PINB2, 1);
  else bitWrite(PORTB,PINB2, 0);
  //(lfoValue>>6)%2);
  /*
  bool newValue= 
   if(newValue && !resetState){ //reset
   lfoValue=0;
   lfoFlop=false;
   }
   resetState=newValue;
   */
  doReset=bitRead(PINB,PINB3);
  if(!lastDoReset && doReset) {
    lfoValue=0, lfoFlop=0;
  }
  lastDoReset=doReset;
  if(lfoValue==0){
    lfoFlop=!lfoFlop;
    // if(lfoFlop) { //rungler
    bool newBit= bitRead(runglerByte,7) ;
    runglerByte=runglerByte<<1;
    //  if(analogChannelRead==0 &&  analogValues[0]>650 && lastAnalogValues[0]<=650) doReset=true;
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
    runglerOut=runglerMap[runglerOut];//map(runglerOut,0,7,0,255);

    // }
  }

  if(lfoFlop) out =255-lfoValue;
  else out=lfoValue;


  OCR0B= constrain(out,0,255);//constrain(renderVco()^_xor,0,255); //pwm output
  OCR0A= runglerOut;
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
  // OCR1A=compare<<1; 


  // OCR1A=tuneTable[map(compare,0,127,0,12)]; //quantized

}

void _setFrequency(int _freq){
  if(_freq<256){
    bitWrite(TCCR1,CS13,1);
    bitWrite(TCCR1,CS12,0);
    bitWrite(TCCR1,CS11,1);
    bitWrite(TCCR1,CS10,0);
    OCR1A=map(_freq,0,255,255,1);//constrain(255-(_freq>>1),1,255);
  }
  else if(_freq<750){
    bitWrite(TCCR1,CS13,1);
    bitWrite(TCCR1,CS12,0);
    bitWrite(TCCR1,CS11,0);
    bitWrite(TCCR1,CS10,0);
    OCR1A=map(_freq,256,750,255,20);
  }
  else{
    bitWrite(TCCR1,CS13,0);
    bitWrite(TCCR1,CS12,1);
    bitWrite(TCCR1,CS11,0);
    bitWrite(TCCR1,CS10,0);
    OCR1A=map(_freq,751,1024,255,50);
  }
  /*
  _freq=map(_freq,0,1024,384,0);
   uint8_t prescaler = _freq>>7;
   switch(prescaler){
   case 0: //fastest prescaler =0
   bitWrite(TCCR1,CS13,1);
   bitWrite(TCCR1,CS12,0);
   bitWrite(TCCR1,CS11,0);
   bitWrite(TCCR1,CS10,0);
   break; 
   case 1: //prescaler 128 times slower
   bitWrite(TCCR1,CS13,1);
   bitWrite(TCCR1,CS12,0);
   bitWrite(TCCR1,CS11,0);
   bitWrite(TCCR1,CS10,1);
   break; 
   case 2:
   bitWrite(TCCR1,CS13,1);
   bitWrite(TCCR1,CS12,0);
   bitWrite(TCCR1,CS11,1);
   bitWrite(TCCR1,CS10,0);
   break; 
  /*
   case 3:
   bitWrite(TCCR1,CS13,0);
   bitWrite(TCCR1,CS12,0);
   bitWrite(TCCR1,CS11,0);
   bitWrite(TCCR1,CS10,0);
   break; 
   case 4:
   bitWrite(TCCR1,CS13,0);
   bitWrite(TCCR1,CS12,0);
   bitWrite(TCCR1,CS11,0);
   bitWrite(TCCR1,CS10,0);
   break; 
   */
  /*
  }
   // bitWrite(TCCR1,CS13,bitRead(prescaler,2));
   // bitWrite(TCCR1,CS11,bitRead(prescaler,1));
   //  bitWrite(TCCR1,CS10,bitRead(prescaler,0));
   uint8_t compare=_freq;
   OCR1A=(compare>>1)<<1;
   */

}












