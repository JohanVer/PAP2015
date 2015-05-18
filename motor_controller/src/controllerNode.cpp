#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../include/motorController/controllerClass.hpp"

#include <sstream>

void parseTask(const pap_common::TaskConstPtr& taskMsg);

motorController controller;
controllerStatus controllerState1, controllerState2, controllerState3,
		oldControllerState1, oldControllerState2, oldControllerState3;
ros::Publisher statusPublisher;

enum STATE {
	IDLE, POSITION, STATUS
} state;

void checkStatusController(int numberOfController,
		controllerStatus* controllerStatusAct,
		controllerStatus* controllerStatusOld) {
	*controllerStatusAct = controller.getStatusController(numberOfController);
	pap_common::Status stateMessage;
	stateMessage.data1 = numberOfController;

	if (controllerStatusAct->energized != controllerStatusOld->energized) {

		if (controllerStatusAct->energized) {
			stateMessage.status = pap_common::ENERGIZED;
		} else {
			stateMessage.status = pap_common::NOENERGY;
		}
		statusPublisher.publish(stateMessage);
		controllerStatusOld->energized = controllerStatusAct->energized;
	}

	if (controllerStatusAct->error != controllerStatusOld->error) {

		if (controllerStatusAct->error) {
			stateMessage.status = pap_common::ERROR;
		} else {
			stateMessage.status = pap_common::NOERROR;
		}
		statusPublisher.publish(stateMessage);
		controllerStatusOld->error = controllerStatusAct->error;
	}

	if (controllerStatusAct->positionReached
			!= controllerStatusOld->positionReached) {

		if (controllerStatusAct->positionReached) {
			stateMessage.status = pap_common::POSITIONREACHED;
		} else {
			stateMessage.status = pap_common::POSITIONNOTREACHED;
		}
		statusPublisher.publish(stateMessage);
		controllerStatusOld->positionReached =
				controllerStatusAct->positionReached;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "motorController");

	ros::NodeHandle n;
	ros::Subscriber taskSubscriber_ = n.subscribe("task", 1, &parseTask);
	statusPublisher = n.advertise<pap_common::Status>("status", 1000);

	ROS_INFO("Motor controller started...");
	ros::Rate loop_rate(5);
	controller.connectToBus();

	state = STATUS;

	while (ros::ok()) {
		switch (state) {
		case IDLE:
			break;
		case STATUS:

			if (controller.controllerConnected_1_) {
				checkStatusController(1, &controllerState1,
						&oldControllerState1);
			}

			if (controller.controllerConnected_2_) {
				checkStatusController(2, &controllerState2,
						&oldControllerState2);
			}

			if (controller.controllerConnected_3_) {
				checkStatusController(3, &controllerState3,
						&oldControllerState3);
			}
			break;

		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
void parseTask(const pap_common::TaskConstPtr& taskMsg) {

	ROS_INFO("Got msg");
	int coordError = 0;

	switch (taskMsg->destination) {
	case pap_common::CONTROLLER:
		switch (taskMsg->task) {
		case pap_common::HOMING:
			if (!controller.sendHoming()) {
				ROS_ERROR("Error while sending homing command");
			}
			break;
		case pap_common::CURRENT:
			if (!controllerState1.energized) {
				if (!controller.energizeAxis(1, true)) {
					ROS_ERROR("Error while switching current on x-axis");
				}

				if (!controller.energizeAxis(2, true)) {
					ROS_ERROR("Error while switching current on y-axis");
				}

				if (!controller.energizeAxis(3, true)) {
					ROS_ERROR("Error while switching current on z-axis");
				}
			} else {
				if (!controller.energizeAxis(1, false)) {
					ROS_ERROR("Error while switching current on x-axis");
				}
				if (!controller.energizeAxis(2, false)) {
					ROS_ERROR("Error while switching current on y-axis");
				}
				if (!controller.energizeAxis(3, false)) {
					ROS_ERROR("Error while switching current on z-axis");
				}
			}
			break;
		case pap_common::COORD:
			coordError = controller.gotoCoord(taskMsg->data1, taskMsg->data2,
					taskMsg->data3);
			if (coordError != error_codes::NO_ERROR) {
				if (coordError == error_codes::X_ERROR) {
					ROS_ERROR("Error while setting x-axis");
				} else if (coordError == error_codes::Y_ERROR) {
					ROS_ERROR("Error while setting y-axis");
				} else if (coordError == error_codes::Z_ERROR) {
					ROS_ERROR("Error while setting z-axis");
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
		}
		break;
	}
}
