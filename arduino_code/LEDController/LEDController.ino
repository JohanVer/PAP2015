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
  RESETBOTTOMLED = 9
};


void messageCb( const pap_common::ArduinoMsg& arduinoMsg){
  
  if(arduinoMsg.command == SETLED ){  
    //leds[arduinoMsg.data] = CRGB::Green;
    leds[arduinoMsg.data].setHSV( 96, 255, 255);
    FastLED.show(); 
  }

  if(arduinoMsg.command == RESETLED ){
    leds[arduinoMsg.data] = CRGB::Black;
    FastLED.show(); 
  } 
  
  if(arduinoMsg.command == SETBOTTOMLED ){
    for (int i = 0; i < NUM_BOT_LEDS; i++) {
      bottom_leds[i] = CRGB::Green;
      FastLED.show();
    } 
  }
  
  if(arduinoMsg.command == RESETBOTTOMLED ){
    for (int i = 0; i < NUM_BOT_LEDS; i++) {
      bottom_leds[i] = CRGB::Black;
      FastLED.show();
    } 
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
//  while(initialize) {
//    LEDtest();    
//  }

  //LEDtest(); 
  nh.spinOnce();
  delay(1);
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

