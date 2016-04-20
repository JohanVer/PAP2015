#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "math.h"
#include "../../motor_controller/include/motorController/controllerClass.hpp"
#include <motorController/controllerNode.h>
#include <pap_common/Status.h>
#include <robot_simulation/controller_simulator.h>

/*****************************************************************************
 * Parameter
 *****************************************************************************/

motor_controller::controllerStatus controllerState1, controllerState2, controllerState3;
ros::Publisher statusPublisher;
controller_simulator::ControllerSimulator controller_sim;
bool energized = false;

void insertStatusInStatusMsg(enum pap_common::MOTOR num_of_controller, const motor_controller::controllerStatus& c_status, bool connected, pap_common::Status& msg){
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

void parseTask(const pap_common::TaskConstPtr& taskMsg) {
    switch (taskMsg->destination) {
    case pap_common::CONTROLLER:
        switch (taskMsg->task) {
        case pap_common::HOMING:
            controller_sim.sendHoming();
            break;
        case pap_common::CURRENT:
            if(energized){
                controller_sim.energizeAllAxis(false);
                energized = false;
            }
            else{
                controller_sim.energizeAllAxis(true);
                energized = true;
            }
            break;
        case pap_common::COORD:
            controller_sim.gotoCoord(taskMsg->data1, taskMsg->data2, taskMsg->data3);
            break;

        case pap_common::COORD_VEL:
            controller_sim.gotoCoord(taskMsg->data1, taskMsg->data2, taskMsg->data3);
            break;

        case pap_common::CONNECT:
            controller_sim.connectController(true);
            break;

        }
        break;
    }
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_sender");
    ros::NodeHandle n;
    ros::Subscriber taskSubscriber_ = n.subscribe("task", 1, &parseTask);
    statusPublisher = n.advertise<pap_common::Status>("status", 1000);
    ros::Rate loop_rate(10);

    // start simulation thread
    controller_sim.start();

    while (ros::ok()) {

        pap_common::Status msg;
        bool controllerConnected = controller_sim.isConnected(pap_common::XMOTOR);
        controllerState1 = controller_sim.getFullStatusController(pap_common::MOTOR::XMOTOR);
        controllerState2 = controller_sim.getFullStatusController(pap_common::MOTOR::YMOTOR);
        controllerState3 = controller_sim.getFullStatusController(pap_common::MOTOR::ZMOTOR);
        insertStatusInStatusMsg(pap_common::XMOTOR, controllerState1, controllerConnected, msg);
        insertStatusInStatusMsg(pap_common::YMOTOR, controllerState2, controllerConnected, msg);
        insertStatusInStatusMsg(pap_common::ZMOTOR, controllerState3, controllerConnected, msg);
        statusPublisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

