#include <motorController/controllerNode.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "motorController");

    motor_controller::ControllerInterface node;

    node.init();
    node.startInterface();

    return 0;
}
