#include <pap_common/arduinosendfunctions.h>

namespace arduino_send_functions{

ArduinoSender::ArduinoSender(ros::NodeHandle &node_handle) : nh_(node_handle)
{
    pub_ = nh_.advertise<pap_common::ArduinoMsg>("arduinoTx", 1000);
}

/*****************************************************************************
 ** Send arduino taks functions - Implementation
 *****************************************************************************/
void ArduinoSender::sendRelaisTask(int relaisNumber, bool value) {
    pap_common::ArduinoMsg arduinoMsg;
    if (value) {
        arduinoMsg.command = pap_common::SETRELAIS;
        arduinoMsg.data = relaisNumber;
    } else {
        arduinoMsg.command = pap_common::RESETRELAIS;
        arduinoMsg.data = relaisNumber;
    }
    pub_.publish(arduinoMsg);
}

void ArduinoSender::sendStepperTask(int StepperNumber, int rotationAngle) {
    pap_common::ArduinoMsg arduinoMsg;
    if (StepperNumber == 2) {
        arduinoMsg.command = pap_common::RUNSTEPPER1;
        arduinoMsg.data = rotationAngle;
        pub_.publish(arduinoMsg);
    }
    if (StepperNumber == 1) {
        arduinoMsg.command = pap_common::RUNSTEPPER2;
        arduinoMsg.data = rotationAngle;
        pub_.publish(arduinoMsg);
    }
}

void ArduinoSender::resetStepper() {
    pap_common::ArduinoMsg arduinoMsg;
    arduinoMsg.command = pap_common::RESETSTEPPERS;
    pub_.publish(arduinoMsg);
}

void ArduinoSender::setLEDTask(int LEDnumber) {
    pap_common::ArduinoMsg arduinoMsg;
    arduinoMsg.command = pap_common::SETLED;
    arduinoMsg.data = LEDnumber;
    pub_.publish(arduinoMsg);
}

void ArduinoSender::resetLEDTask(int LEDnumber) {
    pap_common::ArduinoMsg arduinoMsg;
    arduinoMsg.command = pap_common::RESETLED;
    arduinoMsg.data = LEDnumber;
    pub_.publish(arduinoMsg);
}

void ArduinoSender::LEDTask(int task, int data) {
    pap_common::ArduinoMsg arduinoMsg;
    arduinoMsg.command = task;
    arduinoMsg.data = data;
    pub_.publish(arduinoMsg);
}

}
