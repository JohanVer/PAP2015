#include <ros.h>
#include <pap_common/ArduinoMsg.h>
#include <pap_common/Status.h>
#include "FastLED.h"

#define LED_PIN   5
#define BOT_LED_PIN 3
#define NUM_LEDS  59
#define NUM_BOT_LEDS 24
#define CHIPSET   WS2812B

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
      ringBlinking = true;
    }
    else{
     ringBlinking = false; 
    }
  }
  
  if(arduinoMsg.command == BACKLIGHTBLINK ){
    if(arduinoMsg.data){
      backLightBlinking = true;
    }
    else{
     backLightBlinking = false; 
    }
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
  //LEDtest();
}

void loop()
{  
  nh.spinOnce();
  delay(1);
  if(ringBlinking){
    blinkRing(1.0);
  }
  
  if(backLightBlinking){
    blinkBacklight(2.0,backColor);
  }
}

void showRingLeds(){
 for (int i = 0; i < NUM_BOT_LEDS; i++) {
      if(bottom_leds[i])
      bottom_leds[i].setHSV( ringColor, 255, brightnessRing);// = CRGB::Green;
      FastLED.show();
    }  
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

void blinkRing(float speed){
  for (int blinkIter = 0; blinkIter <= 255; blinkIter+=5 ){
    for (int i = 0; i < NUM_BOT_LEDS; i++) {
        bottom_leds[i].setHSV( 96, 255, blinkIter);
        FastLED.show();
      }
      delay((speed/2.0)/51.0);
  }
  
  for (int blinkIter = 255; blinkIter >= 0; blinkIter -= 5 ){
    for (int i = 0; i < NUM_BOT_LEDS; i++) {
        bottom_leds[i].setHSV( 96, 255, blinkIter);
        FastLED.show();
      }
      delay((speed/2.0)/51.0);
  }
}

void blinkBacklight(float speed, int color){
  for (int blinkIter = 0; blinkIter <= 255; blinkIter+=5 ){
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i].setHSV( color, 255, blinkIter);
        FastLED.show();
      }
      delay((speed/2.0)/51.0);
  }
  
  for (int blinkIter = 255; blinkIter >= 0; blinkIter -= 5 ){
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i].setHSV( color, 255, blinkIter);
        FastLED.show();
      }
      delay((speed/2.0)/51.0);
  }
}

