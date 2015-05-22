#include <ros.h>
#include <pap_common/ArduinoMsg.h>
#include <pap_common/Status.h>


void messageCb( const pap_common::ArduinoMsg& arduinoMsg);

pap_common::ArduinoMsg arduMsg;
ros::NodeHandle  nh;
ros::Subscriber<pap_common::ArduinoMsg> arduinoMessageSub("arduinoTx", messageCb );
ros::Publisher statusPublisher("arduStatus", &arduMsg);

enum ARDUINO_TASK {
  SETRELAIS = 1,
  RESETRELAIS = 2
};

enum RELAIS {
  RELAIS1 = 1,
  RELAIS2 = 2,
  RELAIS3 = 3,
  RELAIS4 = 4,
  RELAIS5 = 5,
  RELAIS6 = 6,
  RELAIS7 = 7,
  RELAIS8 = 8
};

void messageCb( const pap_common::ArduinoMsg& arduinoMsg){
  if(arduinoMsg.command == SETRELAIS ){
    switch(arduinoMsg.data){
    case 1:
      digitalWrite(13, HIGH);
      break;

    case 2:
    digitalWrite(12, HIGH);
      break;
      
    case 3:
      break;

    case 4:
      break;

    case 5:
      break;

    case 6:
      break;

    case 7:
      break;

    case 8:
      break;
    }
  }

  if(arduinoMsg.command == RESETRELAIS ){
    switch(arduinoMsg.data){
    case 1:
      digitalWrite(13, LOW);
      break;

    case 2:
      digitalWrite(12, LOW);
      break;

    case 3:
      break;

    case 4:
      break;

    case 5:
      break;

    case 6:
      break;

    case 7:
      break;

    case 8:
      break;
    }
  }

}

void setup()
{
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  nh.initNode();
  nh.advertise(statusPublisher);
  nh.subscribe(arduinoMessageSub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}



