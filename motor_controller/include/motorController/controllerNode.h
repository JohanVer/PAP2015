#ifndef CONTROLLERNODE_H
#define CONTROLLERNODE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <motorController/controllerClass.hpp>
#include <pap_common/MotorControllerActionAction.h>
#include <pap_common/Status.h>
#include <actionlib/server/simple_action_server.h>
#include <sstream>
#include <motorController/controller_interface.h>
#include <memory>

class ControllerNode{
public:


    ControllerNode();


private:


};


#endif // CONTROLLERNODE_H
