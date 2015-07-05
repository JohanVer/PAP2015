#include "ros/ros.h"
#include "std_msgs/String.h"

#include "pap_common/Task.h"
#include "pap_common/Status.h"
#include "pap_common/VisionStatus.h"
#include "pap_common/ArduinoMsg.h"
#include "../../pap_common/include/pap_common/task_message_def.h"
#include "../../pap_common/include/pap_common/status_message_def.h"
#include "../../pap_common/include/pap_common/arduino_message_def.h"
#include "../../pap_common/include/pap_common/vision_message_def.h"
#include "../../pap_placer/include/pap_placer/placerNode.hpp"
//#include "../include/pap_placer/placerNode.hpp"
#include "../include/pap_placer/placerClass.hpp"

#define TIMEOUT 10
int xTimeOutTimer, yTimeOutTimer, zTimeOutTimer = 0;

PlaceController placeController;
ComponentPlacerData currentComponent;

/* Call back functions */
void statusCallback(const pap_common::StatusConstPtr&  statusMsg);
void visionStatusCallback(const pap_common::VisionStatusConstPtr&  statusMsg);
void placerCallback(const pap_common::TaskConstPtr& taskMsg);

ros::Publisher task_publisher, arduino_publisher_;
ros::Subscriber statusSubsriber_;
ros::Subscriber visionStatusSubsriber_;
ros::Subscriber placerTaskSubscriber_;

bool visionEnabled = false;

enum STATE {
	IDLE, CALIBRATE, GOTOBOX, PICKUP, GOTOPCB, PLACEMENT, HOMING, ERROR
} state;


int main(int argc, char **argv) {

	ros::init(argc, argv, "motorController");
	if (!ros::master::check()) {
		ROS_INFO("PlacerNode did not start...");
		return 0;
	}
	ROS_INFO("PlacerNode started...");

	ros::NodeHandle n_;

	task_publisher = n_.advertise<pap_common::Task>("task", 1000);
	arduino_publisher_ = n_.advertise<pap_common::ArduinoMsg>("arduinoTx", 1000);
	statusSubsriber_ = n_.subscribe("status", 100, &statusCallback);
	visionStatusSubsriber_ = n_.subscribe("visionStatus", 100, &visionStatusCallback);
	placerTaskSubscriber_ = n_.subscribe("task", 10, &placerCallback);

	ros::Rate loop_rate(5);
	state = HOMING;

	while (ros::ok()) {
		switch (state) {
		case IDLE:
			break;
		case CALIBRATE:

		case GOTOBOX:
/*
			if (controller.controllerConnected_1_) {
				checkStatusController(1, &controllerState1,
						&oldControllerState1);
			}*/
			break;

		case PICKUP:
			break;
		case GOTOPCB:
			break;
		case PLACEMENT:
			break;
		case HOMING:
			break;
		case ERROR:
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

/*****************************************************************************
 ** Callback functions for motor status, vision status and placer tasks
 *****************************************************************************/
void statusCallback(const pap_common::StatusConstPtr& statusMsg) {
/*
	int index = statusMsg->data1;
	if (statusMsg->status == pap_common::ENERGIZED) {
		motorcontrollerStatus[index].energized = true;
	}
	if (statusMsg->status == pap_common::NOENERGY) {
		motorcontrollerStatus[index].energized = false;
	}

	if (statusMsg->status == pap_common::POSITIONREACHED) {
		motorcontrollerStatus[index].positionReached = true;
	}

	if (statusMsg->status == pap_common::POSITIONNOTREACHED) {
		motorcontrollerStatus[index].positionReached = false;
	}

	if (statusMsg->status == pap_common::ERROR) {
		motorcontrollerStatus[index].error = true;
	}

	if (statusMsg->status == pap_common::NOERROR) {
		motorcontrollerStatus[index].error = false;
	}*/
}

void visionStatusCallback(const pap_common::VisionStatusConstPtr& statusMsg) {
	//Q_EMIT smdCoordinates(statusMsg->data1, statusMsg->data2, statusMsg->data3);
}

void placerCallback(const pap_common::TaskConstPtr& taskMsg) {

	bool cmdExecuted = true;

	switch (taskMsg->destination) {
	case pap_common::PLACER:
		switch (taskMsg->task) {

		case pap_common::PLACECOMPONENT:
			state = GOTOBOX;				// Start placement process with state GOTOBOX
			ROS_INFO("placer called...");
			break;
		case pap_common::PICKUPCOMPONENT:
			state = PICKUP;
			ROS_INFO("Pick-up component called...");
			break;
		case pap_common::GOTOBOX:
			state = GOTOBOX;
			ROS_INFO("go to box called...");
			break;
		case pap_common::GOTOPCB:
			state = GOTOPCB;
			ROS_INFO("go to pcb called...");
			break;
		case pap_common::PLACEMENT:
			state = PLACEMENT;
			ROS_INFO("Placement called...");
			break;
		break;
		}
	}
}

/*****************************************************************************
 ** Send task functions
 *****************************************************************************/
void sendTask(pap_common::DESTINATION destination,
		pap_common::TASK task) {
	pap_common::Task taskMsg;
	taskMsg.destination = destination;
	taskMsg.task = task;
	task_publisher.publish(taskMsg);
}

void sendTask(pap_common::DESTINATION destination,
		pap_vision::VISION task) {
	pap_common::Task taskMsg;
	taskMsg.destination = destination;
	taskMsg.task = task;
	task_publisher.publish(taskMsg);
}

void sendTask(pap_common::DESTINATION destination,
		pap_vision::VISION task, float x, float y, float z) {
	pap_common::Task taskMsg;
	taskMsg.destination = destination;
	taskMsg.task = task;
	taskMsg.data1 = x;
	taskMsg.data2 = y;
	taskMsg.data3 = z;
	task_publisher.publish(taskMsg);
}

void sendTask(pap_common::DESTINATION destination, pap_common::TASK task,
		float x, float y, float z) {
	pap_common::Task taskMsg;
	taskMsg.destination = destination;
	taskMsg.task = task;
	taskMsg.data1 = x;
	taskMsg.data2 = y;
	taskMsg.data3 = z;
	task_publisher.publish(taskMsg);
}


/*****************************************************************************
 ** Send arduino taks functions
 *****************************************************************************/
void sendRelaisTask(int relaisNumber, bool value) {
	pap_common::ArduinoMsg arduinoMsg;
	if (value) {
		arduinoMsg.command = pap_common::SETRELAIS;
		arduinoMsg.data = relaisNumber;
	} else {
		arduinoMsg.command = pap_common::RESETRELAIS;
		arduinoMsg.data = relaisNumber;
	}
	arduino_publisher_.publish(arduinoMsg);
}

void sendStepperTask(int StepperNumber, int rotationAngle) {
	pap_common::ArduinoMsg arduinoMsg;
	if (StepperNumber == 1) {
		arduinoMsg.command = pap_common::RUNSTEPPER1;
		arduinoMsg.data = rotationAngle;
		arduino_publisher_.publish(arduinoMsg);
	}
	if (StepperNumber == 2) {
		arduinoMsg.command = pap_common::RUNSTEPPER2;
		arduinoMsg.data = rotationAngle;
		arduino_publisher_.publish(arduinoMsg);
	}
}

void resetStepper() {
	pap_common::ArduinoMsg arduinoMsg;
	arduinoMsg.command = pap_common::RESETSTEPPERS;
	arduino_publisher_.publish(arduinoMsg);
}

void setLEDTask(int LEDnumber) {
	pap_common::ArduinoMsg arduinoMsg;
	arduinoMsg.command = pap_common::SETLED;
	arduinoMsg.data = LEDnumber;
	arduino_publisher_.publish(arduinoMsg);
}

void resetLEDTask(int LEDnumber) {
	pap_common::ArduinoMsg arduinoMsg;
	arduinoMsg.command = pap_common::RESETLED;
	arduinoMsg.data = LEDnumber;
	arduino_publisher_.publish(arduinoMsg);
}

