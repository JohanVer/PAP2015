#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pap_common/MotorControllerActionAction.h>
#include <pap_common/Status.h>
#include <pap_common/Task.h>
#include <pap_common/task_message_def.h>
#include <pap_common/status_message_def.h>
#include <actionlib/server/simple_action_server.h>
#include <sstream>
#include <memory>

namespace motor_controller {
typedef actionlib::SimpleActionServer<pap_common::MotorControllerActionAction> ActionServer;

namespace error_codes {
enum ERROR_CODES {
    NO_ERROR, X_ERROR, Y_ERROR, Z_ERROR
};
}

class controllerStatus {
public:
    controllerStatus() {
        error = false;
        energized = false;
        positionReached = false;
        failed = false;
    }

    bool error;
    bool energized;
    bool positionReached;
    bool failed;
    float position;

private:
};

void insertStatusInStatusMsg(int num_of_controller, const controllerStatus& c_status, bool connected, pap_common::Status& msg);

class ControllerInterface
{
public:

    ControllerInterface();
    void publishControllerStatus(const ros::Publisher& publisher ,const controllerStatus& c1, const controllerStatus& c2, const controllerStatus& c3);
    void initInterface();
    void startInterface();
    bool checkAllControllers(controllerStatus& c1, controllerStatus& c2, controllerStatus& c3);
    bool checkTimeout(ros::WallTime start_t, double timeout);
    bool waitForArrival(double timeout);

    // Necessary virtual functions
    virtual bool sendHoming() = 0;
    virtual controllerStatus getFullStatusController(enum pap_common::MOTOR addressDevice) = 0;
    virtual int  gotoCoord(float x, float y, float z, float velX = 50.0, float velY = 300.0, float velZ = 100.0 ) = 0;
    virtual bool energizeAxis(enum pap_common::MOTOR adressDevice, bool trigger) = 0;
    virtual void searchForDevices() = 0;
    virtual void connectToBus() = 0;
    virtual bool isConnected(enum pap_common::MOTOR device) = 0;

    // Optional virtual functions
    virtual bool stop(enum pap_common::MOTOR deviceAddress);
    virtual bool manual(enum pap_common::MOTOR, unsigned char direction);
    virtual bool disconnect(enum pap_common::MOTOR device);

private:

    ros::NodeHandle nh_;
    ActionServer as_;
    void parseTask(const pap_common::TaskConstPtr& taskMsg);
    bool checkStatusController(pap_common::MOTOR device_address,
                               controllerStatus &controllerStatusAct);

    void execute_action(const pap_common::MotorControllerActionGoalConstPtr& command);
    int xTimeOutTimer, yTimeOutTimer, zTimeOutTimer;

    controllerStatus controllerState1, controllerState2, controllerState3,
    oldControllerState1, oldControllerState2, oldControllerState3;
    ros::Publisher statusPublisher;
    ros::Subscriber taskSubscriber_;

    std::unique_ptr<ActionServer> actionServer_;
    bool block_status_;
};


}
#endif // CONTROLLER_INTERFACE_H
