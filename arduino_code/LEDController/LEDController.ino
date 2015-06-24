#include <ros.h>
#include <pap_common/ArduinoMsg.h>
#include <pap_common/Status.h>
#include "FastLED.h"

#define LED_PIN   5
#define NUM_LEDS  59
#define CHIPSET   WS2812B

CRGB leds[NUM_LEDS];

void messageCb( const pap_common::ArduinoMsg& arduinoMsg);

pap_common::ArduinoMsg arduMsg;
ros::NodeHandle  nh;
ros::Subscriber<pap_common::ArduinoMsg> arduinoMessageSub("arduinoTx", messageCb );
ros::Publisher statusPublisher("arduStatus", &arduMsg);

enum ARDUINO_TASK {
  SETRELAIS = 1,
  RESETRELAIS = 2,
  RUNSTEPPER1 = 3,
  RUNSTEPPER2 = 4, 
  RESETSTEPPERS = 5,
  SETLED = 6,
  RESETLED = 7
};


void messageCb( const pap_common::ArduinoMsg& arduinoMsg){
  
  if(arduinoMsg.command == SETLED ){  
    leds[arduinoMsg.data] = CRGB::White;
    FastLED.show(); 
  }

  if(arduinoMsg.command == RESETLED ){
    leds[arduinoMsg.data] = CRGB::Black;
    FastLED.show(); 
  }  
}

void setup()
{
  // sanity check delay - allows reprogramming if accidently blowing power w/leds
  delay(2000);
  
  FastLED.addLeds<CHIPSET, LED_PIN, RGB>(leds, NUM_LEDS);

  nh.initNode();
  nh.advertise(statusPublisher);
  nh.subscribe(arduinoMessageSub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}



