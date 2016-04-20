#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "math.h"
#include "../../motor_controller/include/motorController/controllerClass.hpp"
#include <motorController/controllerNode.h>
#include <motorController/controller_interface.h>
#include <pap_common/Status.h>
#include <robot_simulation/controller_simulator.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "motorSimulation");

    controller_simulator::ControllerSimulator simulator;
    motor_controller::ControllerInterface* basePtr;
    basePtr =& simulator;
    basePtr->initInterface();
    basePtr->startInterface();

    return 0;
}
