#ifndef CONTROLLERNODE_H
#define CONTROLLERNODE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <motorController/controllerClass.hpp>
#include <pap_common/MotorControllerActionAction.h>
#include <pap_common/Status.h>
#include <actionlib/server/simple_action_server.h>
#include <sstream>
#include <memory>


namespace controllerNode{

typedef actionlib::SimpleActionServer<pap_common::MotorControllerActionAction> ActionServer;

enum STATE {
    IDLE, POSITION, STATUS
} state;

void insertStatusInStatusMsg(int num_of_controller, const controllerStatus& c_status, bool connected, pap_common::Status& msg);


class ControllerNode{
public:


    ControllerNode();
    void publishControllerStatus(const ros::Publisher& publisher ,const controllerStatus& c1, const controllerStatus& c2, const controllerStatus& c3);
    void init();
    void run();
    bool checkAllControllers(controllerStatus& c1, controllerStatus& c2, controllerStatus& c3);

private:

    void parseTask(const pap_common::TaskConstPtr& taskMsg);
    bool checkStatusController(int numberOfController,
                               controllerStatus* controllerStatusAct);

    void execute_action(const pap_common::MotorControllerActionGoalConstPtr& command, ActionServer* as);
    bool sendHomeOffset;
    int xTimeOutTimer, yTimeOutTimer, zTimeOutTimer;

    motorController controller;
    controllerStatus controllerState1, controllerState2, controllerState3,
    oldControllerState1, oldControllerState2, oldControllerState3;
    ros::Publisher statusPublisher;
    ros::Subscriber taskSubscriber_;

    std::unique_ptr<ActionServer> actionServer_;
};

}


#endif // CONTROLLERNODE_H
