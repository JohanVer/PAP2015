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
//#include "../../motor_controller/include/motorController/controllerClass.hpp"

#define TIMEOUT 1
#define PICKUPDELAY1 5
#define PICKUPDELAY2 5
#define PLACEMENTDELAY1 5
#define PLACEMENTDELAY2 5

int pickupwait_counter, placementwait_counter = 0;

PlaceController placeController;
ComponentPlacerData currentComponent;
controllerStatus motorcontrollerStatus[3];

/* Call back functions */
void statusCallback(const pap_common::StatusConstPtr&  statusMsg);
void visionStatusCallback(const pap_common::VisionStatusConstPtr&  statusMsg);
void placerCallback(const pap_common::TaskConstPtr& taskMsg);

/* Send task functions */
void sendTask(pap_common::DESTINATION,pap_common::TASK);
void sendTask(pap_common::DESTINATION,pap_vision::VISION);
void sendTask(pap_common::DESTINATION destination,pap_common::TASK task, float x, float y, float z );
void sendTask(pap_common::DESTINATION destination,pap_vision::VISION task,float x, float y, float z);

/* Send arduino task funcionts */
void sendRelaisTask(int relaisNumber,bool value);
void sendStepperTask(int StepperNumber, int rotationAngle);
void resetStepper();
void setLEDTask(int LEDnumber);
void resetLEDTask(int LEDnumber);

ros::Publisher task_publisher, arduino_publisher_;
ros::Subscriber statusSubsriber_;
ros::Subscriber visionStatusSubsriber_;
ros::Subscriber placerTaskSubscriber_;

bool visionEnabled = false;
bool placerNodeBusy = false;

bool manualOperation = true;
//bool componentPickUpFinished = false;
bool boxCoordinatesSent = false;
bool componentPickUpStarted = false;
bool pcbCoordinatesSent = false;
bool componentPlacementStarted = false;
//bool componentPlacementFinished = false;

enum ERROR_CODE {
	MOTORFAILED_BOX, MOTORERROR_BOX, MOTORFAILED_PCB, MOTORERROR_PCB
} error_code;

enum STATE {
	IDLE, CALIBRATE, GOTOBOX, FINDCOMPONENT, STARTPICKUP, PICKUPWAIT, PICKUP1, PICKUP2, CHECKCOMPONENTPICKUP, GOTOPCB,
	CHECKCOMPONENTPOSITION, STARTPLACEMENT, PLACEMENTWAIT, PLACEMENT1, PLACEMENT2, CHECKCOMPONENTPLACEMENT, HOMING, ERROR
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

	ros::Rate loop_rate(20);
	state = HOMING;

	while (ros::ok()) {
		switch (state) {
		case IDLE:
			break;

		case CALIBRATE:
			break;

		case GOTOBOX:			// Send coordinates to motor controller and wait until position reached

			if (!placerNodeBusy && !boxCoordinatesSent && placeController.getCalibrationStatus()) {
				Offset destination = placeController.getBoxCoordinates();
				sendTask(pap_common::CONTROLLER, pap_common::COORD, destination.x, destination.y, destination.z);
				setLEDTask(placeController.getBoxNumber());
				boxCoordinatesSent = true;
				placerNodeBusy = true;
			}

			if (motorcontrollerStatus[1].positionReached && motorcontrollerStatus[2].positionReached && motorcontrollerStatus[3].positionReached && boxCoordinatesSent) {
				placerNodeBusy = false;
				if (manualOperation) {
					state = IDLE;
				} else {
					state = STARTPICKUP;
				}
				break;
			}
			else if (motorcontrollerStatus[1].error || motorcontrollerStatus[2].error || motorcontrollerStatus[3].error) {
				error_code = MOTORERROR_BOX;
				state = ERROR;
				break;
			}
			else if (motorcontrollerStatus[1].failed || motorcontrollerStatus[2].failed || motorcontrollerStatus[3].failed) {
				error_code = MOTORFAILED_BOX;
				state = ERROR;
				break;
			}
			break;

		case STARTPICKUP:	// Start pick-up process by activating vacuum

			// Make sure that box position of component has been reached before trying to pick up
			if (boxCoordinatesSent && !placerNodeBusy && !componentPickUpStarted) {
				sendRelaisTask(1, false);		// Turn on vacuum - Früher is wohl besser!!!
				sendRelaisTask(2, true);
				if (placeController.getSelcetedTip()) {
					sendRelaisTask(5, true);	// Turn on left vacuum valve
				} else {
					sendRelaisTask(4, true);	// Turn on right vacuum valve
				}
				state = PICKUPWAIT;				// Wait until vacuum has been built up
				componentPickUpStarted = true;
				placerNodeBusy = true;
			}
			break;

		case PICKUPWAIT:
			if (pickupwait_counter == PICKUPDELAY1) {
				pickupwait_counter++;
				state = PICKUP1;
			} else if (pickupwait_counter == (PICKUPDELAY1+PICKUPDELAY2)) {
				pickupwait_counter = 0;
				state = PICKUP2;
			} else {
				pickupwait_counter++;
			}
			break;

		case PICKUP1:								// Activate cylinder to pick-up component
			if (placeController.getSelcetedTip()) {
				sendRelaisTask(3, false);			// Activate left cylinder
				sendRelaisTask(6, true);
			} else {
				sendRelaisTask(7, true);			// Activate right cylinder
			}
			state = PICKUPWAIT;						// Wait until cylinder extended and component picked up
			break;

		case PICKUP2:								// Release tip
			if (placeController.getSelcetedTip()) {
				sendRelaisTask(6, false);
				sendRelaisTask(3, true);			// Release left tip

			} else {
				sendRelaisTask(7, false);			// Release right tip
			}

			if (manualOperation) {
				if (visionEnabled) {
					placerNodeBusy = false;
					state = CHECKCOMPONENTPICKUP;
				}
				else {
					placerNodeBusy = false;
					state = IDLE;
				}
			} else {
				if (visionEnabled) {
					state = CHECKCOMPONENTPICKUP;	// INFO: Reset placerNodeBusy in next state
				}
				else {
					placerNodeBusy = false;
					state = GOTOPCB;
				}
			}
			break;

		case CHECKCOMPONENTPICKUP:

		case GOTOPCB:

			if (!placerNodeBusy && !pcbCoordinatesSent && placeController.getCalibrationStatus()) {
				Offset destination = placeController.getPCBCoordinates();
				sendTask(pap_common::CONTROLLER, pap_common::COORD, destination.x, destination.y, destination.z);
				resetLEDTask(placeController.getBoxNumber());
				pcbCoordinatesSent = true;
				placerNodeBusy = true;
			}

			if (motorcontrollerStatus[1].positionReached && motorcontrollerStatus[2].positionReached && motorcontrollerStatus[3].positionReached && pcbCoordinatesSent) {
				placerNodeBusy = false;
				if (manualOperation) {
					state = IDLE;
				} else {
					state = STARTPLACEMENT;
				}
				break;
			}
			else if (motorcontrollerStatus[1].error || motorcontrollerStatus[2].error || motorcontrollerStatus[3].error) {
				error_code = MOTORERROR_PCB;
				state = ERROR;
				break;
			}
			else if (motorcontrollerStatus[1].failed || motorcontrollerStatus[2].failed || motorcontrollerStatus[3].failed) {
				error_code = MOTORFAILED_PCB;
				state = ERROR;
				break;
			}
			break;

		case STARTPLACEMENT:

			// Make sure that pcb position of component has been reached before trying to place
			if (pcbCoordinatesSent && !placerNodeBusy && !componentPlacementStarted) {
				if (placeController.getSelcetedTip()) {
					sendRelaisTask(3, false);			// Activate left cylinder
					sendRelaisTask(6, true);
				} else {
					sendRelaisTask(7, true);			// Activate right cylinder
				}
				state = PLACEMENTWAIT;
				componentPickUpStarted = true;
				placerNodeBusy = true;
				break;
			}
			break;

		case PLACEMENT1:							//
			sendRelaisTask(2, false);				// Turn off vacuum
			sendRelaisTask(1, true);
			if (placeController.getSelcetedTip()) {
				sendRelaisTask(5, false);			// Turn off left vacuum valve
			} else {
				sendRelaisTask(4, false);			// Turn off right vacuum valve
			}
			state = PLACEMENTWAIT;					// Wait until vacuum has been gone
			break;

		case PLACEMENT2:							// Release tip to end placement process
			if (placeController.getSelcetedTip()) {
				sendRelaisTask(6, false);
				sendRelaisTask(3, true);			// Release left tip

			} else {
				sendRelaisTask(7, false);			// Release right tip
			}

			placerNodeBusy = false;
			if (manualOperation) {
				if (visionEnabled) {
					state = CHECKCOMPONENTPLACEMENT;
				} else {
					state = IDLE;
				break;
				}
			} else {
				if (visionEnabled) {
					state = CHECKCOMPONENTPLACEMENT;
				}
				else {
					state = HOMING;
				}
			}
			break;

		case PLACEMENTWAIT:
			if (placementwait_counter == PLACEMENTDELAY1) {
				placementwait_counter++;
				state = PLACEMENT1;
			} else if (placementwait_counter == (PLACEMENTDELAY1+PLACEMENTDELAY2)) {
				pickupwait_counter = 0;
				state = PLACEMENT2;
			} else {
				placementwait_counter++;
			}
			break;

		case HOMING:

			break;
		case ERROR:				// Stop and publish error code

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
	}
}

void visionStatusCallback(const pap_common::VisionStatusConstPtr& statusMsg) {
	//Q_EMIT smdCoordinates(statusMsg->data1, statusMsg->data2, statusMsg->data3);
}

void placerCallback(const pap_common::TaskConstPtr& taskMsg) {

	switch (taskMsg->destination) {
	case pap_common::PLACER:

		if(manualOperation) {
			ComponentPlacerData tempComponent;
			tempComponent.destX = taskMsg->data1;
			tempComponent.destY = taskMsg->data2;
			tempComponent.box = taskMsg->box;
			tempComponent.height = taskMsg->height;
			tempComponent.length = taskMsg->length;
			tempComponent.width = taskMsg->width;
			tempComponent.rotation = taskMsg->data3;
			placeController.updatePlacementData(&tempComponent);
		}

		switch (taskMsg->task) {
		case pap_common::PLACECOMPONENT:
			if(!manualOperation) {
				ComponentPlacerData tempComponent;
				tempComponent.destX = taskMsg->data1;
				tempComponent.destY = taskMsg->data2;
				tempComponent.box = taskMsg->box;
				tempComponent.height = taskMsg->height;
				tempComponent.length = taskMsg->length;
				tempComponent.width = taskMsg->width;
				tempComponent.rotation = taskMsg->data3;
				placeController.updatePlacementData(&tempComponent);

				state = GOTOBOX;				// Start placement process with state GOTOBOX
				ROS_INFO("Complete placement process called...");
			}
			break;
		case pap_common::PICKUPCOMPONENT:
			state = STARTPICKUP;
			ROS_INFO("Pick-up component called...");
			break;
		case pap_common::GOTOBOX:
			boxCoordinatesSent = false;		// Has to be set to able to go to a new position!
			state = GOTOBOX;
			ROS_INFO("go to box called...");
			break;
		case pap_common::GOTOPCB:
			state = GOTOPCB;
			ROS_INFO("go to pcb called...");
			break;
		case pap_common::PLACEMENT:
			state = STARTPLACEMENT;
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

