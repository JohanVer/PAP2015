#include <motorController/controllerNode.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "motorController");

    motor_controller::motorController controller;
    motor_controller::ControllerInterface* basePtr;
    basePtr =& controller;
    basePtr->initInterface();
    basePtr->startInterface();

    return 0;
}
