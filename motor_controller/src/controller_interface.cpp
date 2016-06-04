#include <motorController/controller_interface.h>

#define TIMEOUT 10

namespace motor_controller {

void insertStatusInStatusMsg(enum pap_common::MOTOR num_of_controller, const controllerStatus& c_status, bool connected, pap_common::Status& msg){
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

ControllerInterface::ControllerInterface(): as_(nh_, "motor_controller_actions", boost::bind(&ControllerInterface::execute_action, this, _1),false){
    xTimeOutTimer = 0;
    yTimeOutTimer = 0;
    zTimeOutTimer = 0;
    block_status_ = false;
    needle_touched_ = false;
}

void ControllerInterface::publishControllerStatus(const ros::Publisher& publisher ,const controllerStatus& c1, const controllerStatus& c2, const controllerStatus& c3){
    pap_common::Status msg;
    bool connectedX = isConnected(pap_common::XMOTOR);
    bool connectedY = isConnected(pap_common::YMOTOR);
    bool connectedZ = isConnected(pap_common::ZMOTOR);
    insertStatusInStatusMsg(pap_common::XMOTOR, c1, connectedX, msg);
    insertStatusInStatusMsg(pap_common::YMOTOR, c2, connectedY, msg);
    insertStatusInStatusMsg(pap_common::ZMOTOR, c3, connectedZ, msg);
    publisher.publish(msg);
}

bool ControllerInterface::checkStatusController(enum pap_common::MOTOR device_address,
                                                controllerStatus& controllerStatusAct) {
    if(!isConnected(device_address)){
        std::cerr << "Not connected...\n";
        return false;
    }

    size_t tries = 0;

    while(1){
        controllerStatusAct = getFullStatusController(device_address);

        if (controllerStatusAct.failed || controllerStatusAct.position == -1){
            ROS_ERROR("Get status of %d axis failed, trying again...",
                      device_address);

            if (tries > TIMEOUT) {

                ROS_ERROR("Axis: %d reconnect timeout, not connected anymore", device_address);
                switch(device_address){
                case pap_common::XMOTOR:
                    disconnect(pap_common::XMOTOR);
                    break;

                case pap_common::YMOTOR:
                    disconnect(pap_common::YMOTOR);
                    break;

                case pap_common::ZMOTOR:
                    disconnect(pap_common::ZMOTOR);
                    break;
                }
                return false;
            }else{
                tries++;
                ros::Duration(0.1).sleep();
            }
        }else {
            break;
        }
    }
    return true;
}

bool ControllerInterface::checkAllControllers(controllerStatus& c1, controllerStatus& c2, controllerStatus& c3){
    if (isConnected(pap_common::XMOTOR)) {
        if(!checkStatusController(pap_common::XMOTOR, c1)){
            return false;
        }
    }else{
        return false;
    }
    if (isConnected(pap_common::YMOTOR)) {
        if(!checkStatusController(pap_common::YMOTOR, c2))
        {
            return false;
        }
    }else{
        return false;
    }

    if (isConnected(pap_common::ZMOTOR)) {
        if(!checkStatusController(pap_common::ZMOTOR, c3)){
            return false;
        }
    }else{
        return false;
    }
    return true;
}

bool ControllerInterface::checkTimeout(ros::WallTime start_t, double timeout){
    double secs = ros::WallTime::now().toSec() - start_t.toSec();
    if(secs > timeout) return true;
    return false;
}


bool ControllerInterface::waitForArrival(double timeout){
    ros::WallTime start_t = ros::WallTime::now();
    ros::Rate looprate(20);

    while(1){
        looprate.sleep();

        if(checkTimeout(start_t, timeout)){
            std::cerr << "Timeout or device not responding...\n";
            return false;
        }

        if(checkAllControllers(controllerState1, controllerState2, controllerState3)){
            publishControllerStatus(statusPublisher, controllerState1, controllerState2, controllerState3);
            if(controllerState1.positionReached && controllerState2.positionReached && controllerState3.positionReached){
                return true;
            }
        }else{
            std::cerr << "Timeout or device not responding...\n";
            return false;
        }

    }

    return false;
}

void ControllerInterface::execute_action(const pap_common::MotorControllerActionGoalConstPtr& command)
{

    block_status_ = true;
    int coordError = 0;
    bool cmdExecuted = true;
    switch (command->task) {
    case pap_common::HOMING:

        if (!sendHoming()) {
            ROS_ERROR("Error while sending homing command");
        }

        if(!waitForArrival(50)){
            as_.setAborted();
            break;
        }

        gotoCoord(5, 5,
                  0.1,50.0,300.0,100.0);

        as_.setSucceeded();
        break;

    case pap_common::CURRENT:
        if (!controllerState3.energized) {
            if (!energizeAxis(pap_common::XMOTOR, true)) {
                ROS_ERROR("Error while switching current on x-axis");
                cmdExecuted = false;
            }

            if (!energizeAxis(pap_common::YMOTOR, true)) {
                ROS_ERROR("Error while switching current on y-axis");
                cmdExecuted = false;
            }

            if (!energizeAxis(pap_common::ZMOTOR, true)) {
                ROS_ERROR("Error while switching current on z-axis");
                cmdExecuted = false;
            }
        } else {
            if (!energizeAxis(pap_common::XMOTOR, false)) {
                ROS_ERROR("Error while switching current on x-axis");
                cmdExecuted = false;
            }
            if (!energizeAxis(pap_common::YMOTOR, false)) {
                ROS_ERROR("Error while switching current on y-axis");
                cmdExecuted = false;
            }
            if (!energizeAxis(pap_common::ZMOTOR, false)) {
                ROS_ERROR("Error while switching current on z-axis");
                cmdExecuted = false;
            }
        }

        if(!cmdExecuted) as_.setAborted();
        else as_.setSucceeded();

        break;

    case pap_common::COORD:
        coordError = gotoCoord(command->data1, command->data2,
                               command->data3,50.0,300.0,100.0);

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

        if(!cmdExecuted || !waitForArrival(15)){
            as_.setAborted();
            break;
        }

        as_.setSucceeded();
        break;

    case pap_common::HEIGHT_CAL:{

        ros::Rate loop_rate(100);
        coordError = gotoCoord(controllerState1.position, controllerState2.position,
                               command->data3, command->velX, command->velY,command->velZ);
        if (coordError != error_codes::NO_ERROR) {
            ros::Time start_time = ros::Time::now();
            needle_touched_ = false;
            // Wait until needle touched
            while(!needle_touched_){
                if(ros::Time::now().toSec() - start_time.toSec() > 15.0){
                    std::cerr << "Needle not touched within timer...\n";
                    break;
                }
                ros::spinOnce();
                loop_rate.sleep();
            }

            if(needle_touched_){
                stop(pap_common::ZMOTOR);
                if(checkAllControllers(controllerState1, controllerState2, controllerState3)){
                    pap_common::MotorControllerActionResult res;
                    res.height = controllerState3.position;
                    as_.setSucceeded(res);
                }else{
                    as_.setAborted();
                }
            }
        }else{
            as_.setAborted();
        }
    }
        break;

    case pap_common::COORD_VEL:
        coordError = gotoCoord(command->data1, command->data2,
                               command->data3, command->velX, command->velY,100.0);
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

        if(!cmdExecuted || !waitForArrival(15)){
            as_.setAborted();
            break;
        }

        as_.setSucceeded();

        break;
    }

    block_status_ = false;
}

void ControllerInterface::calSignalCallback(const pap_common::CalibrationSignalConstPtr& msg){
    if(!msg->touched){
        needle_touched_ = true;
        std::cerr << "Cal Signal received...." <<(size_t) msg->touched << std::endl;
    }
}

void ControllerInterface::parseTask(const pap_common::TaskConstPtr& taskMsg) {
    int coordError = 0;
    bool cmdExecuted = true;
    std::cerr << "Got command..." << taskMsg->destination << " " << taskMsg->task << std::endl;

    switch (taskMsg->destination) {
    case pap_common::CONTROLLER:
        switch (taskMsg->task) {

        case pap_common::COORD_VEL:
            coordError = gotoCoord(taskMsg->data1, taskMsg->data2,
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
                    manual(pap_common::XMOTOR, 1);
                } else {
                    manual(pap_common::XMOTOR, 0);
                }
            } else if (taskMsg->data1 == (float) (pap_common::YMOTOR)) {
                if (taskMsg->data2 == (float) (pap_common::FORWARD)) {
                    manual(pap_common::YMOTOR, 1);
                } else {
                    manual(pap_common::YMOTOR, 0);
                }
            } else if (taskMsg->data1 == (float) (pap_common::ZMOTOR)) {
                if (taskMsg->data2 == (float) (pap_common::FORWARD)) {
                    manual(pap_common::ZMOTOR, 1);
                } else {
                    manual(pap_common::ZMOTOR, 0);
                }
            }
            break;

        case pap_common::CONNECT:
            ROS_INFO("Searching for devices...");
            searchForDevices();
            break;

        case pap_common::STOP:
            if (taskMsg->data1 == (float) (pap_common::XMOTOR)) {
                stop(pap_common::XMOTOR);
            } else if (taskMsg->data1 == (float) (pap_common::YMOTOR)) {
                stop(pap_common::YMOTOR);
            } else if (taskMsg->data1 == (float) (pap_common::ZMOTOR)) {
                stop(pap_common::ZMOTOR);
            }
            break;
        }

        break;

    }
}

void ControllerInterface::initInterface()
{
    taskSubscriber_ = nh_.subscribe("/task", 100, &ControllerInterface::parseTask, this);
    calSignalSub_ = nh_.subscribe("cal_signal", 1, &ControllerInterface::calSignalCallback, this);
    as_.start();
    statusPublisher = nh_.advertise<pap_common::Status>("status", 1000);

    ROS_INFO("Motor controller started...");
    connectToBus();

}

void ControllerInterface::startInterface()
{
    ros::Rate loop_rate(20);

    while (ros::ok()) {
        loop_rate.sleep();

        if(!block_status_){
            if(checkAllControllers(controllerState1, controllerState2, controllerState3)){
                publishControllerStatus(statusPublisher, controllerState1, controllerState2, controllerState3);
            }else{
                //std::cerr << "Checking devices failed...\n";
            }
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}

// VIRTUAL DEFAULT FUNCTIONS
bool ControllerInterface::stop(enum pap_common::MOTOR deviceAddress){
    std::cerr << "This function is currently not implemented...\n";
    return false;
}


bool ControllerInterface::manual(enum pap_common::MOTOR, unsigned char direction){
    std::cerr << "This function is currently not implemented...\n";
    return false;
}

bool ControllerInterface::disconnect(enum pap_common::MOTOR device){
    std::cerr << "This function is currently not implemented...\n";
    return false;
}

}
