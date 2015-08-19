#include <ros.h>
#include <pap_common/ArduinoMsg.h>
#include <pap_common/Status.h>
#include "FastLED.h"

#define LED_PIN   5
#define BOT_LED_PIN 3
#define NUM_LEDS  59
#define NUM_BOT_LEDS 24
#define CHIPSET   WS2812B

#define SPEEDBACK 2
#define SPEEDRING 1

CRGB leds[NUM_LEDS];
CRGB bottom_leds[NUM_BOT_LEDS];

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
int counter1 = 0;
int blinkIteration = 0;
bool down = false;
int delayMs = 0;
//ros::Publisher statusPublisher("arduStatus", &arduMsg);

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
  SETRINGCOLOR = 15
};

enum LED_STATE{
  LED_IDLE,
  LED_BLINKRING,
  LED_BLINKBACK
}ledState;


void messageCb( const pap_common::ArduinoMsg& arduinoMsg){
  
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

  nh.initNode();
  //nh.advertise(statusPublisher);
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
}


