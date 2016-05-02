#ifndef ARDUINOSENDFUNCTIONS_H
#define ARDUINOSENDFUNCTIONS_H

#include "ros/ros.h"
#include "pap_common/ArduinoMsg.h"
#include <pap_common/arduino_message_def.h>

namespace arduino_send_functions{

class ArduinoSender
{
public:
    ArduinoSender(ros::NodeHandle &node_handle);
    void sendRelaisTask(int relaisNumber, bool value);
    void sendStepperTask(int StepperNumber, int rotationAngle);
    void resetStepper();
    void setLEDTask(int LEDnumber);
    void resetLEDTask(int LEDnumber);
    void LEDTask(int task, int data);

private:
    ros::Publisher pub_;
    ros::NodeHandle nh_;
};

}
#endif // ARDUINOSENDFUNCTIONS_H
