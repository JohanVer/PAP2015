#include <motorController/controllerNode.h>

#define TIMEOUT 10

namespace controllerNode{


void insertStatusInStatusMsg(int num_of_controller, const controllerStatus& c_status, bool connected, pap_common::Status& msg){
    switch (num_of_controller) {
    case pap_common::XMOTOR:
        msg.connected[0] = connected;
        msg.energized[0] = c_status.energized;
        msg.error[0] = c_status.error;
        msg.reached[0] = c_status.positionReached;
        msg.pos[0] = c_status.position;
        break;

    case pap_common::YMOTOR:
        msg.connected[1] = connected;
        msg.energized[1] = c_status.energized;
        msg.error[1] = c_status.error;
        msg.reached[1] = c_status.positionReached;
        msg.pos[1] = c_status.position;
        break;

    case pap_common::ZMOTOR:
        msg.connected[2] = connected;
        msg.energized[2] = c_status.energized;
        msg.error[2] = c_status.error;
        msg.reached[2] = c_status.positionReached;
        msg.pos[2] = c_status.position;
        break;
    }
}

void ControllerNode::publishControllerStatus(const ros::Publisher& publisher ,const controllerStatus& c1, const controllerStatus& c2, const controllerStatus& c3){
    pap_common::Status msg;
    insertStatusInStatusMsg(pap_common::XMOTOR, c1, controller.controllerConnected_1_, msg);
    insertStatusInStatusMsg(pap_common::YMOTOR, c2, controller.controllerConnected_2_, msg);
    insertStatusInStatusMsg(pap_common::ZMOTOR, c3, controller.controllerConnected_3_, msg);
    publisher.publish(msg);
}



bool ControllerNode::checkStatusController(int numberOfController,
                                           controllerStatus* controllerStatusAct) {
    float position = 0.0;
    while(1){
        *controllerStatusAct = controller.getStatusController(numberOfController);
        position = controller.getPosition(numberOfController);
        size_t tries = 0;

        if (controllerStatusAct->failed || position == -1){
            ROS_ERROR("Get status of %d axis failed, trying again...",
                      numberOfController);

            if (tries > TIMEOUT) {

                ROS_ERROR("Axis: %d reconnect timeout, not connected anymore", numberOfController);
                switch(numberOfController){
                case pap_common::XMOTOR:
                    controller.controllerConnected_1_ = false;
                    break;

                case pap_common::YMOTOR:
                    controller.controllerConnected_2_ = false;
                    break;

                case pap_common::ZMOTOR:
                    controller.controllerConnected_3_ = false;
                    break;
                }
                return false;
            }else{
                tries++;
            }
        }else {
            break;
        }
    }

    controllerStatusAct->position = position;
    return controllerStatusAct->failed;
}


bool ControllerNode::checkAllControllers(controllerStatus& c1, controllerStatus& c2, controllerStatus& c3){
    if (controller.controllerConnected_1_) {
        if(!checkStatusController(1, &c1))
            return false;
    }else return false;

    if (controller.controllerConnected_2_) {
        if(!checkStatusController(2, &c2))
            return false;
    }else return false;

    if (controller.controllerConnected_3_) {
        if(!checkStatusController(3, &c3))
            return false;
    }else return false;

    return true;
}

void ControllerNode::execute_action(const pap_common::MotorControllerActionGoalConstPtr& command, ActionServer* as)
{
    switch (command->task) {
    case pap_common::HOMING:
        if (!controller.sendHoming()) {
            ROS_ERROR("Error while sending homing command");
        }

        while(1){
            if(checkAllControllers(controllerState1, controllerState2, controllerState3)){
                if(controllerState1.positionReached && controllerState2.positionReached && controllerState3.positionReached){
                    controller.gotoCoord(5, 5,
                                         0.1,50.0,300.0,100.0);
                    as->setSucceeded();
                    break;
                }
            }else{
                as->setAborted();
                break;
            }
        }
    }
}

void ControllerNode::parseTask(const pap_common::TaskConstPtr& taskMsg) {
    int coordError = 0;
    bool cmdExecuted = true;
    switch (taskMsg->destination) {
    case pap_common::CONTROLLER:
        switch (taskMsg->task) {
        case pap_common::HOMING:
            if (!controller.sendHoming()) {
                ROS_ERROR("Error while sending homing command");
                cmdExecuted = false;

            }
            else{
                sendHomeOffset = true;
            }
            break;
        case pap_common::CURRENT:
            if (!controllerState3.energized) {
                if (!controller.energizeAxis(1, true)) {
                    ROS_ERROR("Error while switching current on x-axis");
                    cmdExecuted = false;
                }

                if (!controller.energizeAxis(2, true)) {
                    ROS_ERROR("Error while switching current on y-axis");
                    cmdExecuted = false;
                }

                if (!controller.energizeAxis(3, true)) {
                    ROS_ERROR("Error while switching current on z-axis");
                    cmdExecuted = false;
                }
            } else {
                if (!controller.energizeAxis(1, false)) {
                    ROS_ERROR("Error while switching current on x-axis");
                    cmdExecuted = false;
                }
                if (!controller.energizeAxis(2, false)) {
                    ROS_ERROR("Error while switching current on y-axis");
                    cmdExecuted = false;
                }
                if (!controller.energizeAxis(3, false)) {
                    ROS_ERROR("Error while switching current on z-axis");
                    cmdExecuted = false;
                }
            }
            break;
        case pap_common::COORD:
            coordError = controller.gotoCoord(taskMsg->data1, taskMsg->data2,
                                              taskMsg->data3,50.0,300.0,100.0);
            if (coordError != error_codes::NO_ERROR) {
                if (coordError == error_codes::X_ERROR) {
                    ROS_ERROR("Error while setting x-axis");
                    cmdExecuted = false;
                } else if (coordError == error_codes::Y_ERROR) {
                    ROS_ERROR("Error while setting y-axis");
                    cmdExecuted = false;
                } else if (coordError == error_codes::Z_ERROR) {
                    ROS_ERROR("Error while setting z-axis");
                    cmdExecuted = false;
                }
            }
            break;

        case pap_common::COORD_VEL:
            coordError = controller.gotoCoord(taskMsg->data1, taskMsg->data2,
                                              taskMsg->data3, taskMsg->velX, taskMsg->velY,100.0);
            if (coordError != error_codes::NO_ERROR) {
                if (coordError == error_codes::X_ERROR) {
                    ROS_ERROR("Error while setting x-axis");
                    cmdExecuted = false;
                } else if (coordError == error_codes::Y_ERROR) {
                    ROS_ERROR("Error while setting y-axis");
                    cmdExecuted = false;
                } else if (coordError == error_codes::Z_ERROR) {
                    ROS_ERROR("Error while setting z-axis");
                    cmdExecuted = false;
                }
            }

            break;

        case pap_common::MANUAL:
            if (taskMsg->data1 == (float) (pap_common::XMOTOR)) {
                if (taskMsg->data2 == (float) (pap_common::FORWARD)) {
                    controller.manual(1, 1);
                } else {
                    controller.manual(1, 0);
                }
            } else if (taskMsg->data1 == (float) (pap_common::YMOTOR)) {
                if (taskMsg->data2 == (float) (pap_common::FORWARD)) {
                    controller.manual(2, 1);
                } else {
                    controller.manual(2, 0);
                }
            } else if (taskMsg->data1 == (float) (pap_common::ZMOTOR)) {
                if (taskMsg->data2 == (float) (pap_common::FORWARD)) {
                    controller.manual(3, 1);
                } else {
                    controller.manual(3, 0);
                }
            }
            break;

        case pap_common::CONNECT:
            ROS_INFO("Searching for devices...");
            controller.searchForDevices();
            break;

        case pap_common::STOP:
            if (taskMsg->data1 == (float) (pap_common::XMOTOR)) {
                controller.stop(1);
            } else if (taskMsg->data1 == (float) (pap_common::YMOTOR)) {
                controller.stop(2);
            } else if (taskMsg->data1 == (float) (pap_common::ZMOTOR)) {
                controller.stop(3);
            }
            break;
        }
        break;
    }
}

void ControllerNode::init()
{
    ros::NodeHandle n;
    taskSubscriber_ = n.subscribe("task", 10, &parseTask, this);
    ActionServer actionServer(n, "motor_controller_actions", boost::bind(&ControllerNode::execute_action, this, _1, &actionServer), false);
    actionServer.start();
    statusPublisher = n.advertise<pap_common::Status>("status", 1000);

    ROS_INFO("Motor controller started...");
    controller.connectToBus();

}

void ControllerNode::run()
{
    ros::Rate loop_rate(10);
    state = STATUS;

    while (ros::ok()) {
        switch (state) {
        case IDLE:
            break;
        case STATUS:

            if(checkAllControllers(controllerState1, controllerState2, controllerState3)){
                publishControllerStatus(statusPublisher, controllerState1, controllerState2, controllerState3);
            }

            break;

        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motorController");




    return 0;
}
