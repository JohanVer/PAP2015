#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../include/motorController/controllerClass.hpp"

#include <sstream>

#define TIMEOUT 10

void parseTask(const pap_common::TaskConstPtr& taskMsg);

motorController controller;
controllerStatus controllerState1, controllerState2, controllerState3,
		oldControllerState1, oldControllerState2, oldControllerState3;
ros::Publisher statusPublisher;

int xTimeOutTimer, yTimeOutTimer, zTimeOutTimer = 0;

enum STATE {
	IDLE, POSITION, STATUS
} state;

void checkStatusController(int numberOfController,
		controllerStatus* controllerStatusAct,
		controllerStatus* controllerStatusOld) {
	*controllerStatusAct = controller.getStatusController(numberOfController);
	float Position = controller.getPosition(numberOfController);

	pap_common::Status stateMessage;
	stateMessage.data1 = numberOfController;
	if (xTimeOutTimer > TIMEOUT) {
		ROS_ERROR("X-Axis reconnect timeout, not connected anymore");
		controller.controllerConnected_1_ = false;
		stateMessage.status = pap_common::DISCONNECTED;
		stateMessage.data1 = pap_common::XMOTOR;
		statusPublisher.publish(stateMessage);
		xTimeOutTimer = 0;
		return;
	}

	if (yTimeOutTimer > TIMEOUT) {
		ROS_ERROR("Y-Axis reconnect timeout, not connected anymore");
		controller.controllerConnected_2_ = false;
		stateMessage.status = pap_common::DISCONNECTED;
		stateMessage.data1 = pap_common::YMOTOR;
		statusPublisher.publish(stateMessage);
		yTimeOutTimer = 0;
		return;
	}

	if (zTimeOutTimer > TIMEOUT) {
		ROS_ERROR("Z-Axis reconnect timeout, not connected anymore");
		controller.controllerConnected_3_ = false;
		stateMessage.status = pap_common::DISCONNECTED;
		stateMessage.data1 = pap_common::ZMOTOR;
		statusPublisher.publish(stateMessage);
		zTimeOutTimer = 0;
		return;
	}

	if (controllerStatusAct->failed) {
		ROS_ERROR("Get status of %d axis failed, trying again...",
				numberOfController);

		switch (numberOfController) {
		case pap_common::XMOTOR:
			xTimeOutTimer++;
			break;
		case pap_common::YMOTOR:
			yTimeOutTimer++;
			break;
		case pap_common::ZMOTOR:
			zTimeOutTimer++;
			break;
		}
		return;
	}

	switch (numberOfController) {
	case pap_common::XMOTOR:
		stateMessage.posX = Position;
		//ROS_INFO("posX: %f", Position);
		break;

	case pap_common::YMOTOR:
		stateMessage.posY = Position;
		//ROS_INFO("posY: %f", Position);
		break;

	case pap_common::ZMOTOR:
		stateMessage.posZ = Position;
		//ROS_INFO("posZ: %f", Position);
		break;
	}

//	if (controllerStatusAct->energized != controllerStatusOld->energized) {

	if (controllerStatusAct->energized) {
		stateMessage.status = pap_common::ENERGIZED;
	} else {
		stateMessage.status = pap_common::NOENERGY;
	}
	statusPublisher.publish(stateMessage);
	controllerStatusOld->energized = controllerStatusAct->energized;
//	}

//	if (controllerStatusAct->error != controllerStatusOld->error) {

	if (controllerStatusAct->error) {
		stateMessage.status = pap_common::ERROR;
	} else {
		stateMessage.status = pap_common::NOERROR;
	}
	statusPublisher.publish(stateMessage);
	controllerStatusOld->error = controllerStatusAct->error;
//	}

//	if (controllerStatusAct->positionReached
//			!= controllerStatusOld->positionReached) {

	if (controllerStatusAct->positionReached) {
		stateMessage.status = pap_common::POSITIONREACHED;
	} else {
		stateMessage.status = pap_common::POSITIONNOTREACHED;
	}
	statusPublisher.publish(stateMessage);

	controllerStatusOld->positionReached = controllerStatusAct->positionReached;
//	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "motorController");

	ros::NodeHandle n;
	ros::Subscriber taskSubscriber_ = n.subscribe("task", 10, &parseTask);
	statusPublisher = n.advertise<pap_common::Status>("status", 1000);

	ROS_INFO("Motor controller started...");
	ros::Rate loop_rate(10);
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
					taskMsg->data3);
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
					taskMsg->data3, taskMsg->velX, taskMsg->velY);
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
		case pap_common::GETPOSITION:

			if (controller.controllerConnected_1_
					&& controller.controllerConnected_2_
					&& controller.controllerConnected_3_) {
				float xPosition = controller.getPosition(1);
				float yPosition = controller.getPosition(2);
				float zPosition = controller.getPosition(3);
				pap_common::Status positionMessage;
				positionMessage.status = pap_common::POSITION;
				positionMessage.posX = xPosition;
				positionMessage.posY = yPosition;
				positionMessage.posZ = zPosition;
				statusPublisher.publish(positionMessage);
			}

			break;
		}
		break;
	}

	if (!cmdExecuted) {
		pap_common::Status stateMessage;
		stateMessage.status = pap_common::LAST_CMD_FAILED;
		statusPublisher.publish(stateMessage);
	}
}
