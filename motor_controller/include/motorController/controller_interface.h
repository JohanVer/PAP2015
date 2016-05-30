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

//!
//! \brief The ControllerInterface class implements the ros interface for motor control.
//! This class serves as module which can be used together with plugins
//! which are implementing various virtual functions.
//!
class ControllerInterface
{
public:

    ControllerInterface();

    //!
    //! \brief publishControllerStatus publishes controller status via ros topic
    //! \param publisher ros publisher
    //! \param c1 controller status message for controller 1
    //! \param c2 controller status message for controller 2
    //! \param c3 controller status message for controller 3
    //!
    void publishControllerStatus(const ros::Publisher& publisher ,const controllerStatus& c1, const controllerStatus& c2, const controllerStatus& c3);

    //!
    //! \brief initInterface initializes interface
    //!
    void initInterface();

    //!
    //! \brief startInterface starts interface
    //!
    void startInterface();

    //!
    //! \brief checkAllControllers gets all controller status messages
    //! \param[out] c1 controller status of controller 1
    //! \param[out] c2 controller status of controller 2
    //! \param[out] c3 controller status of controller 3
    //! \return true if successfull
    //!
    bool checkAllControllers(controllerStatus& c1, controllerStatus& c2, controllerStatus& c3);

    //!
    //! \brief checkTimeout checks if timer has expired
    //! \param start_t start time of timer
    //! \param timeout timer value
    //! \return true if expired
    //!
    bool checkTimeout(ros::WallTime start_t, double timeout);

    //!
    //! \brief waitForArrival blocks until all axes have arrived
    //! \param timeout timeout value
    //! \return true if successfull
    //!
    bool waitForArrival(double timeout);

    // Necessary virtual functions/////////////////////////////////////////////////////////////////////////////////////
    //!
    //! \brief sendHoming sends homing command to parker device
    //! \return true if successfull
    //!
    virtual bool sendHoming() = 0;

    //!
    //! \brief getFullStatusController gets full status of controller (with position data)
    //! \param addressDevice device address
    //! \return full status message
    //!
    virtual controllerStatus getFullStatusController(enum pap_common::MOTOR addressDevice) = 0;

    //!
    //! \brief gotoCoord drives with all three axis controller to specified position
    //! \param x x-position
    //! \param y y-position
    //! \param z z-position
    //! \param velX x-velocity
    //! \param velY y-velocity
    //! \param velZ z-velocity
    //! \return error code (ERROR_CODES)
    //!
    virtual int  gotoCoord(float x, float y, float z, float velX = 50.0, float velY = 300.0, float velZ = 100.0 ) = 0;

    //!
    //! \brief energizeAxis energize or deenergize axis controller
    //! \param adressDevice device address
    //! \param trigger if true axis is energized
    //! \return true if successfull
    //!
    virtual bool energizeAxis(enum pap_common::MOTOR adressDevice, bool trigger) = 0;

    //!
    //! \brief searchForDevices searches available devices and connects to them
    //!
    virtual void searchForDevices() = 0;

    //!
    //! \brief connectToBus connects to serial bus
    //!
    virtual void connectToBus() = 0;

    //!
    //! \brief isConnected checks if specified device is connected
    //! \param device device name
    //! \return true if connected
    //!
    virtual bool isConnected(enum pap_common::MOTOR device) = 0;

    // Optional virtual functions/////////////////////////////////////////////////////////////////////////////////////
    //!
    //! \brief stop stops motion of specified device
    //! \param deviceAddress device address
    //! \return true if successfull
    //!
    virtual bool stop(enum pap_common::MOTOR deviceAddress);

    //!
    //! \brief manual manual control
    //! \param deviceAddress device address
    //! \param direction forward = 1 backward = 0
    //! \return true if successfull
    //!
    virtual bool manual(enum pap_common::MOTOR, unsigned char direction);

    //!
    //! \brief disconnect disconnects from specified device
    //! \param device device name
    //! \return true if disconnected successfully
    //!
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
