#include <ros.h>
#include <pap_common/ArduinoMsg.h>
#include <pap_common/Status.h>
#include <Stepper.h>

#define MOTORSTEPS 200
Stepper stepper1(MOTORSTEPS, 10, 11, 12, 13);
//Stepper stepper2(MOTORSTEPS, 10, 11, 12, 13);
int previousSteps1 = 0;
int previousSteps2 = 0;

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
  RESETSTEPPERS = 5
};

enum RELAIS {
  RELAIS1 = 1,
  RELAIS2 = 2,
  RELAIS3 = 3,
  RELAIS4 = 4,
  RELAIS5 = 5,
  RELAIS6 = 6,
  RELAIS7 = 7,
  RELAIS8 = 8,
  RELAIS9 = 9,
  RELAIS10 = 10,
};

void messageCb( const pap_common::ArduinoMsg& arduinoMsg){
  if(arduinoMsg.command == RESETRELAIS ){
    switch(arduinoMsg.data){
    case 1:
      digitalWrite(2, HIGH);
      break;

    case 2:
    digitalWrite(3, HIGH);
      break;
      
    case 3:
      digitalWrite(4, HIGH); 
      break;

    case 4:
      digitalWrite(5, HIGH);
      break;

    case 5:
      digitalWrite(6, HIGH);
      break;

    case 6:
      digitalWrite(7, HIGH);
      break;

    case 7:
      digitalWrite(8, HIGH);
      break;

    case 8:
      digitalWrite(9, HIGH);
      break;
    case 9:
      digitalWrite(10, HIGH);
      break;
    case 10:
      digitalWrite(11, HIGH);
      break;      
    }
  }

  if(arduinoMsg.command == SETRELAIS ){
    switch(arduinoMsg.data){
    case 1:
      digitalWrite(2, LOW);
      break;

    case 2:
      digitalWrite(3, LOW);
      break;

    case 3:
      digitalWrite(4, LOW);
      break;

    case 4:
      digitalWrite(5, LOW);
      break;

    case 5:
      digitalWrite(6, LOW);
      break;

    case 6:
      digitalWrite(7, LOW);
      break;

    case 7:
      digitalWrite(8, LOW);
      break;

    case 8:
      digitalWrite(9, LOW);
      break;
    case 9:
      digitalWrite(10, LOW);
      break;
    case 10:
      digitalWrite(11, LOW);
      break;
    }
  }
  
  if(arduinoMsg.command == RUNSTEPPER1) {
    
    int steps = round(arduinoMsg.data/1.8);    // Resolution of 1.8°
  
    if((previousSteps1 + steps) > 100) {
      steps = steps - 200;                     // Rotation = 200 Steps
    }     
    if((previousSteps1 + steps) < 100) {
      steps = 200 - steps;
    }         
    previousSteps1 = previousSteps1 + steps;   
    stepper1.step(steps);
  }
  
 /* 
  if(arduinoMsg.command == RUNSTEPPER2 ){
    
    int steps = round(arduinoMsg.data/1.8);    // Resolution of 1.8°
  
    if((previousSteps2 + steps) > 100) {
      steps = steps - 200;                     // Rotation = 200 Steps
    }  
    if((previousSteps2 + steps) < 100) {
      steps = 200 - steps;
    }  
    previousSteps2 = previousSteps2 + steps;   
    stepper2.step(steps);
  } 
  */
  if(arduinoMsg.command == RESETSTEPPERS ){
    stepper1.step(-previousSteps1);
    //stepper2.step(-previousSteps2);
    previousSteps1 = 0;
    previousSteps2 = 0;
  }
   
}

void setup()
{
  pinMode(2, OUTPUT);  
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  stepper1.setSpeed(30); // RPM
  //stepper2.setSpeed(30);
  nh.initNode();
  nh.advertise(statusPublisher);
  nh.subscribe(arduinoMessageSub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}



