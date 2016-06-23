      #include <ros.h>
      #include <pap_common/ArduinoMsg.h>
      #include <pap_common/Status.h>
      #include <pap_common/CalibrationSignal.h>
      #include <pap_common/Pressure.h>
      #include <Stepper.h>
      
      #define MOTORSTEPS 200
      
      Stepper stepper2(MOTORSTEPS, A4, A5, 12, 13);
      
      void messageCb( const pap_common::ArduinoMsg& arduinoMsg);
      
      pap_common::ArduinoMsg arduMsg;
      ros::NodeHandle  nh;
      ros::Subscriber<pap_common::ArduinoMsg> arduinoMessageSub("arduinoTx", messageCb );
      pap_common::CalibrationSignal cal_needle_flag;
      pap_common::Pressure pressure_msg;
      
      ros::Publisher calibration_signal("cal_signal", &cal_needle_flag);
      ros::Publisher pressure_signal("pressure", &pressure_msg);
      
      enum ARDUINO_TASK {
        SETRELAIS = 1,
        RESETRELAIS = 2,
        RUNSTEPPER1 = 3,
        RUNSTEPPER2 = 4, 
        RESETSTEPPERS = 5,
        SETLED = 6,
        RESETLED = 7
        
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
            //digitalWrite(11, HIGH);
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
            //digitalWrite(11, LOW);
            break;
          }
        } 
      
        if(arduinoMsg.command == RUNSTEPPER2 ){ 
          int steps = arduinoMsg.data;  
          stepper2.step(steps);
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
        pinMode(11, INPUT_PULLUP);
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
        stepper2.setSpeed(60);  //RPM
        nh.initNode();
        //nh.advertise(statusPublisher);
        nh.subscribe(arduinoMessageSub);
        nh.advertise(calibration_signal);
        nh.advertise(pressure_signal);
        
      }
      
      void loop()
      {
        static unsigned int pressure_counter = 0;
        
        nh.spinOnce();
        bool reading = digitalRead(11);
      
      if(!reading){
         cal_needle_flag.touched = reading;
         calibration_signal.publish(&cal_needle_flag);
      }
      
      if(pressure_counter == 500){
       pressure_counter = 0;
       float volt = analogRead(A0) * 0.0049;
       float pressure = (volt)  / 0.033333333;
       pressure = (pressure-15) / 9;
       pressure_msg.pressure = pressure;
       pressure_signal.publish(&pressure_msg);              
      }else{
       pressure_counter++; 
      }
      
      
      
        delay(2);
      }
      
      

