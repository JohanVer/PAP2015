#include <ros.h>
#include <pap_common/ArduinoMsg.h>
#include <pap_common/Status.h>

#include "FastLED.h"
#include <Stepper.h>

#define LED_PIN   5
#define BOT_LED_PIN 3
#define TOP_LED_PIN 10
#define NUM_LEDS  59
#define NUM_TOP_LEDS  12
#define NUM_BOT_LEDS 24
#define CHIPSET   WS2812B

#define SPEEDBACK 2
#define SPEEDRING 1

#define MOTORSTEPS 200
Stepper stepper1(MOTORSTEPS, A0, A1, A2 ,A3);
int previousSteps1 = 0;

CRGB leds[NUM_LEDS];
CRGB bottom_leds[NUM_BOT_LEDS];
CRGB top_leds[NUM_TOP_LEDS];

void messageCb( const pap_common::ArduinoMsg& arduinoMsg);

pap_common::ArduinoMsg arduMsg;
ros::NodeHandle  nh;
ros::Subscriber<pap_common::ArduinoMsg> arduinoMessageSub("arduinoTx", messageCb );

bool ringBlinking = false;
bool backLightBlinking = false;
int backColor = 0;
int ringColor = 96;

int brightnessRing = 255;
int brightnessBack = 255;
int brightnessTop = 255;
int counter1 = 0;
int blinkIteration = 0;
bool down = false;
int delayMs = 0;

enum ARDUINO_TASK {
  SETRELAIS = 1,
  RESETRELAIS = 2,
  RUNSTEPPER1 = 3,
  RUNSTEPPER2 = 4, 
  RESETSTEPPERS = 5,
  SETLED = 6,
  RESETLED = 7,
  SETBOTTOMLED = 8,
  RESETBOTTOMLED = 9,
  RESETALLLED = 10,
  RINGBLINK = 11,
  BACKLIGHTBLINK = 12,
  SETBRIGHTNESSRING = 13,
  SETBRIGHTNESSBACK = 14,
  SETRINGCOLOR = 15,
  SETTOPLED = 16,
  RESETTOPLED = 17,
  SETBRIGHTNESSTOP = 18
};

enum LED_STATE{
  LED_IDLE,
  LED_BLINKRING,
  LED_BLINKBACK
}ledState;


void messageCb( const pap_common::ArduinoMsg& arduinoMsg){
  
  if(arduinoMsg.command == RUNSTEPPER1) {
    
    int steps = arduinoMsg.data;    
    if((previousSteps1 + steps) > 100) {
      steps = steps - 200;                     // full rotation = 200 Steps
    } else if((previousSteps1 + steps) < -100) {
      steps = 200 - steps;
    }
    previousSteps1 = previousSteps1 + steps;
    stepper1.step(steps);
  }
  
  if(arduinoMsg.command == RESETSTEPPERS ){
    stepper1.step(-previousSteps1);
    previousSteps1 = 0;
  }
  
  
  if(arduinoMsg.command == SETLED ){  
    //leds[arduinoMsg.data] = CRGB::Green;
    leds[arduinoMsg.data].setHSV( 96, 255, brightnessBack);
    FastLED.show(); 
  }

  if(arduinoMsg.command == RESETLED ){
    leds[arduinoMsg.data] = CRGB::Black;
    FastLED.show(); 
  } 
  
  if(arduinoMsg.command == SETBOTTOMLED ){
    showRingLeds();
  }
  
  if(arduinoMsg.command == RESETBOTTOMLED ){
    for (int i = 0; i < NUM_BOT_LEDS; i++) {
      bottom_leds[i] = CRGB::Black;
      FastLED.show();
    } 
  }
  
  if(arduinoMsg.command == SETTOPLED ){
    for (int i = 0; i < NUM_TOP_LEDS; i++) {
      top_leds[i].setRGB( 31, 31, 31);
      FastLED.show();
    } 
  }
  
  if(arduinoMsg.command == RESETTOPLED ){
    for (int i = 0; i < NUM_TOP_LEDS; i++) {
      top_leds[i] = CRGB::Black;
      FastLED.show();
    } 
  }
  
  if(arduinoMsg.command == RESETALLLED ){
    resetAllLEDs();
  }
  
  if(arduinoMsg.command == RESETALLLED ){
    resetAllLEDs();
  }
  
  if(arduinoMsg.command == RINGBLINK ){
    if(arduinoMsg.data){
      ledState = LED_BLINKRING;
    }
    else{
     ledState = LED_IDLE;
     resetAllLEDs();
    }
    blinkIteration = 0;
    delayMs = 0;
    down = false;
  }
  
  if(arduinoMsg.command == BACKLIGHTBLINK ){
    if(arduinoMsg.data){
      ledState = LED_BLINKBACK;
    }
    else{
     ledState = LED_IDLE;
     resetAllLEDs();
    }
    blinkIteration = 0;
    delayMs = 0;
    down = false;
  }
  
  if(arduinoMsg.command == SETBRIGHTNESSRING ){
    brightnessRing = arduinoMsg.data;
    showRingLeds();
  }
  
  if(arduinoMsg.command == SETBRIGHTNESSTOP ){
    brightnessTop = arduinoMsg.data;
    for (int i = 0; i < NUM_TOP_LEDS; i++) {
      top_leds[i].setHSV( 0, 255, brightnessTop);
      FastLED.show();
    } 
  }
  
  if(arduinoMsg.command == SETBRIGHTNESSBACK ){
    brightnessBack = arduinoMsg.data;
  }
  
  if(arduinoMsg.command == SETRINGCOLOR ){
    ringColor = arduinoMsg.data;
    showRingLeds();
  }
}

void setup()
{
  // sanity check delay - allows reprogramming if accidently blowing power w/leds
  delay(200);
  
  FastLED.addLeds<CHIPSET, LED_PIN, RGB>(leds, NUM_LEDS);
  FastLED.addLeds<CHIPSET, BOT_LED_PIN,RGB>(bottom_leds, NUM_BOT_LEDS);
  FastLED.addLeds<CHIPSET, TOP_LED_PIN,RGB>(top_leds, NUM_TOP_LEDS);
  stepper1.setSpeed(60); // RPM
  nh.initNode();
  
  nh.subscribe(arduinoMessageSub);
  
  resetAllLEDs();
}

void loop()
{  
  nh.spinOnce();
  delay(1);
  if(delayMs > 0 ){
    delayMs--;
  }
  switch(ledState){
    case LED_IDLE:
      
    break;
    
    case LED_BLINKRING:
    
    if(delayMs == 0){
    delayMs = ((SPEEDRING/2.0)/51.0)*1000.0;
    
    if(!down){
      blinkIteration += 5;
    }
    else{
     blinkIteration -= 5; 
    }
    
    if(!down && blinkIteration == 255){
     down = true;
    }
    else if ( down && blinkIteration == 0){
      down = false;
    }
    
    for (int i = 0; i < NUM_BOT_LEDS; i++) {
        bottom_leds[i].setHSV( 96, 255, blinkIteration);
      }
      FastLED.show();
      
    }
    
    break;
    
    case LED_BLINKBACK:
    
    if(delayMs == 0){
    delayMs = ((SPEEDBACK/2.0)/51.0)*1000.0;
    
    if(!down){
      blinkIteration += 5;
    }
    else{
     blinkIteration -= 5; 
    }
    
    if(!down && blinkIteration == 255){
     down = true;
    }
    else if ( down && blinkIteration == 0){
      down = false;
    }
    
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i].setHSV( 0, 255, blinkIteration);
      }
    FastLED.show();
    }
    break;
  }
}

void showRingLeds(){
 for (int i = 0; i < NUM_BOT_LEDS; i++) {
      bottom_leds[i].setHSV( ringColor, 255, brightnessRing);// = CRGB::Green;
    }  
    FastLED.show();
}

void LEDtest() {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Red;
      FastLED.show();
      delay(200);    
    } 
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Black;
      FastLED.show();
    } 
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Blue;
      FastLED.show();
      delay(200);
    } 
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Black;
      FastLED.show();
    } 
}

void resetAllLEDs() {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Black;
      FastLED.show();
    }   
    for (int i = 0; i < NUM_BOT_LEDS; i++) {
      bottom_leds[i] = CRGB::Black;
      FastLED.show();
    }   
    for (int i = 0; i < NUM_TOP_LEDS; i++) {
      top_leds[i] = CRGB::Black;
      FastLED.show();
    } 
}


