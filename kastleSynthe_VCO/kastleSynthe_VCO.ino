
/*
TODO
 -create lookup table using multipoint map in ram 
 -than transfer it to progmem
 -transfer wavetable to ram and see if it is faster
 -update parameters at the beginning of waveoform cycle to see if it improoves stability
 -try to keep constant timing everywhere
 
 -dynamicly lower the resolution of the VCO
 */
#define F_CPU 8000000  // This is used by delay.h library
//#include "fastAnalogRead.h"
#include <stdlib.h>
//#include <EEPROM.h>
#include <avr/interrupt.h>
//#include <expADSR.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>
#include "SoftwareSerial.h"
const int Rx = 8; // this is physical pin 2
const int Tx = 3; // this is physical pin 3
//SoftwareSerial mySerial(Rx, Tx);
uint16_t wsMap[10]={
  0,63,107,191,225,   1,5,150,200,254};
#define WSMAP_POINTS 5

uint8_t _out;
uint16_t time;
uint8_t mode;

#define PHASE_DIST 0
#define FM 1
#define TAH 2
#define LOW_THRES 150
#define HIGH_THRES 162
#define LOW_MIX 300
#define HIGH_MIX 900

#define PITCH 2
#define WS_1 3
#define WS_2 1
uint8_t pwmCounter;

uint32_t curveMap(uint8_t value, uint8_t numberOfPoints, uint16_t * tableMap){
  //  if(value>tableMap[numberOfPoints-1]) return tableMap[4+numberOfPoints];
  // else{
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
  // }
}

uint8_t mapLookup[256];
void createLookup(){
  for(uint16_t i=0;i<256;i++){
    mapLookup[i]=curveMap(i,WSMAP_POINTS,wsMap);
  }
  // mapLookup[255]=curveMap(255,WSMAP_POINTS,wsMap);
}

//curveMap(,WSMAP_POINTS,wsMap)

uint8_t analogChannelRead=1;
uint8_t analogValues[4];
uint8_t lastAnalogValues[4];
uint8_t out;
#include <SINE.h>
#include <SAW.h>
#include <CHEB4.h>
void setup()  { 

  createLookup();
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  //  pinMode(2, OUTPUT);
  //  pinMode(A1, INPUT);
  // pinMode(A2, INPUT);
  // pinMode(A3, INPUT);
  digitalWrite(5,HIGH);
  //  pinMode(1, INPUT_PULLUP);
  // digitalWrite(1,LOW);
  //  analogRead(A3);
  //mySerial.begin(9600);
  setTimers(); //setup audiorate interrupt
  // setFrequency(constrain(mapLookup[analogValues[PITCH]]<<2,0,1000));
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
  TIMSK = _BV(OCIE1A);// | _BV(TOIE0);    //TOIE0    //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1    
  TCCR1 = _BV(CTC1) | _BV(CS12) ;//| _BV(CS11);//  | _BV(CS11) ;// //| _BV(CS13) | _BV(CS12) | _BV(CS11) |
  sei();
  TIMSK |=_BV(TOIE0);   
}
uint8_t _clocks;
uint16_t clocks(){
  //return _clocks;
  return TCNT0|(_clocks<<8);
}
ISR(TIMER0_OVF_vect){
  _clocks++;
}
bool flop;
uint8_t incr=6,_incr=6;
ISR(TIMER1_COMPA_vect)  //audiorate interrupt
{
  //OCR0B=0;


  out+=incr;

  if((analogValues[WS_2])<pwmCounter) OCR0B=255;
  else OCR0B=0;
  pwmCounter=out<<1;//+=12;
  /*
  switch(analogValues[WS_1]>>6){
   case 0:
   pwmCounter+=2;
   break;
   case 1: 
   pwmCounter+=3;
   break;
   case 2: 
   
   break;
   case 3:  
   pwmCounter+=12; 
   break;
   }
   */


  /*FM
   
   */

  //PD
  //  //pd HP


  // OCR0A= out+_out;//constrain(renderVco()^_xor,0,255); //pwm output
  // OCR0A= ((256-_out) * ((char)pgm_read_byte_near(SIN256_DATA+out)+128))>>8;
  //OCR0A= ((256-out) * ((char)pgm_read_byte_near(SIN256_DATA+_out)+128))>>8;

  // OCR0A=   (((256-out)|(analogValues[3]>>2)) * (((char)pgm_read_byte_near(SIN256_DATA+_out))+128))>>8; //+((analogValues[3]>>2)

  // OCR0A=   (((256-out)) * ((((char)pgm_read_byte_near(SIN256_DATA+_out))+128)|(analogValues[3]>>2)))>>8; //pd  RES
  //    OCR0A=   (((256-_out)) * ((((char)pgm_read_byte_near(SIN256_DATA+out))+128)|(analogValues[3]>>2)))>>8; //pd  RES INV
  //OCR0A=   ((256-(_out&(analogValues[3]>>2)+out&(256-analogValues[3]>>2))) * (((char)pgm_read_byte_near(SIN256_DATA+(out&(analogValues[3]>>2)+_out&(256-analogValues[3]>>2))))+128))>>8;
  //  OCR0A=   (((256-out)&(analogValues[3]>>2)) * (((char)pgm_read_byte_near(SAW256_DATA+_out))+128))>>8;// + (((256-out)&(256-(analogValues[3]>>2))) * (((char)pgm_read_byte_near(SIN256_DATA+_out))+128))>>8; //+((analogValues[3]>>2)
  // OCR0A=   (((256-out)) * (((char)pgm_read_byte_near(SIN256_DATA+_out))+128)&(analogValues[3]>>2) + ((((char)pgm_read_byte_near(SAW256_DATA+_out))+128)&(256-(analogValues[3]>>2))) )>>8; 
  // OCR0A=((256-out) *(((char)pgm_read_byte_near(SIN256_DATA+_out)+128)*(analogValues[3]>>2)>>8 +  ((char)pgm_read_byte_near(CHEBYSHEV_4TH_256_DATA+_out)+128)*(256-(analogValues[3]>>2))>>8))>>8;


  TCNT1 = 0; 
}


uint8_t lastOut;
uint8_t bitShift=3;
void loop() { 


  if(mode==PHASE_DIST){

    if(out<incr){
      _out=0;
      //if(analogValues[WS_2]<=20)  _out=0;
      //else _out+=analogValues[WS_2]>>2;
    }
    // out=(out
    //  OCR0A=   (((256-out)|analogValues[WS_2]) * (((char)pgm_read_byte_near(SIN256_DATA+_out))+128))>>8; //
    /*
    if(analogValues[WS_2]<40){
     OCR0A=   (((256-out)) * (((char)pgm_read_byte_near(SIN256_DATA+_out))+128))>>8;
     }
     else if(analogValues[WS_2]>100){
     if(out!=lastOut)    OCR0A=   (((256-out)^((analogValues[WS_2]-100)<<1)) * (((char)pgm_read_byte_near(SIN256_DATA+_out))+128))>>8; 
     }
     else{
     
     if(out!=lastOut)    OCR0A=   (((256-out)|(analogValues[WS_2]<<1)) * (((char)pgm_read_byte_near(SIN256_DATA+_out))+128))>>8;
     }
     */
    // if(out!=lastOut)
    uint8_t waveShape=(analogValues[WS_2]-30)<<1;
    if(analogValues[WS_2]>170) {
      if(out!=lastOut) OCR0A =   (((256-out)|waveShape) * (((char)pgm_read_byte_near(SIN256_DATA+_out))+128))>>8;  
    }
    else{
      OCR0A =   (((256-out)|waveShape) * (((char)pgm_read_byte_near(SIN256_DATA+_out))+128))>>8;
      ;
    }
     lastOut=out;
  }

  if(mode==FM){
    uint8_t _mod= ((((analogValues[WS_2])+1)*((char)pgm_read_byte_near(SIN256_DATA+_out)+128))>>6) +out;
    OCR0A=  (char)pgm_read_byte_near(SIN256_DATA+_mod)+128;
  }

  if(mode==TAH){
    if(out>analogValues[WS_2]){
      OCR0A=  (char)pgm_read_byte_near(SIN256_DATA+_out)+128;
    }
    // OCR0A=  (char)pgm_read_byte_near(SIN256_DATA+out)+128;
    //  OCR0A=   (((256-out)) * (((char)pgm_read_byte_near(SIN256_DATA+_out))+128))>>8;
  }
  // bitRead(PINB,PINB1);
  // out=analogValues[2]>>2;
  // mySerial.println(analogValues[2]); 

  //  mySerial.print(analogValues[2]); 
  // mySerial.print(" "); 
  // mySerial.println(analogValues[1]); 
  //mySerial.println(clocks());

  if(clocks()-time>((255-analogValues[WS_1])<<bitShift)){
    time=clocks(); 
    _out+=_incr;

    // OCR0B= _out;
  }
  else{
    if(analogValues[0]<LOW_THRES)  mode=PHASE_DIST, incr=11,_incr=6, bitShift=2; 
    else if(analogValues[0]>HIGH_THRES) mode=TAH, incr=24,_incr=6,bitShift=4;
    else mode = FM, incr=11,_incr=5,bitShift=4;
  }
  //_delay_us(250);
}
uint8_t lastAnalogChannelRead;
ISR(ADC_vect){

  // if(isConversionFinished()){
  lastAnalogValues[analogChannelRead]=analogValues[analogChannelRead];
  analogValues[analogChannelRead]= getConversionResult()>>2;
  lastAnalogChannelRead=analogChannelRead;
  analogChannelRead++;
  /*
    while(readADC[analogChannelRead]){
   analogChannelRead++;
   if(analogChannelRead>3) analogChannelRead=1;
   }
   */
  if(analogChannelRead>3) analogChannelRead=0;
  connectChannel(analogChannelRead);

  if(lastAnalogChannelRead==PITCH && lastAnalogValues[PITCH]!=analogValues[PITCH]) setFrequency(constrain(mapLookup[analogValues[PITCH]]<<2,0,1015));//constrain(map(analogValues[PITCH],100,HIGH_MIX,0,1024),0,1000));////
  if(lastAnalogChannelRead==WS_1 && lastAnalogValues[WS_1]!=analogValues[WS_1]) analogValues[WS_1]= mapLookup[analogValues[WS_1]];//constrain(map(analogValues[WS_1],LOW_MIX,HIGH_MIX,0,1024),0,1023);////
  if(lastAnalogChannelRead==WS_2 && lastAnalogValues[WS_2]!=analogValues[WS_2]) analogValues[WS_2]= mapLookup[analogValues[WS_2]];//constrain(map(analogValues[WS_2],LOW_MIX,HIGH_MIX,0,1024),0,1023); //



  startConversion();


  //  }

}
uint8_t pwmIncrement,_upIncrement,_downIncrement,upIncrement,downIncrement;
bool quantizer;
void setFrequency(int _freq){
  _freq=1024-_freq;
  uint8_t preScaler=_freq>>7;
  preScaler+=2; //*2
  pwmIncrement=4;

  _upIncrement=pwmIncrement*upIncrement;
  _downIncrement=pwmIncrement*downIncrement;
  for(uint8_t i=0;i<4;i++) bitWrite(TCCR1,CS10+i,bitRead(preScaler,i)); 

  uint8_t compare=_freq;
  bitWrite(compare,7,0);
  if(quantizer){
    //compare=tuneTable[map(compare,0,127,0,6)];
  }
  OCR1A=compare+128; 


}

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
  ADMUX &= (11110000);
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
















