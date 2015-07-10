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

#define PICKUPDELAY1 5
#define PICKUPDELAY2 5
#define PLACEMENTDELAY1 5
#define PLACEMENTDELAY2 5
#define COMPFINDERTIMEOUT 10

int pickupwait_counter, placementwait_counter = 0;
int componentFinder_counter = 0;

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

void resetProcessVariables();

ros::Publisher task_publisher, arduino_publisher_;
ros::Subscriber statusSubsriber_;
ros::Subscriber visionStatusSubsriber_;
ros::Subscriber placerTaskSubscriber_;

bool visionEnabled = false;
bool manualOperation = true;

bool placerNodeBusy = false;

bool boxCoordinatesSent = false;
bool componentPickUpStarted = false;
bool pcbCoordinatesSent = false;
bool pcbOrigCoordinatesSent = false;
bool componentPlacementStarted = false;
bool componentFinderStarted = false;
bool componentFound = false;
bool compPickUpCoordinatesSent = false;
bool bottomCamCoordinatesSent = false;
bool pcbPlaceCoordinatesSent = false;
bool homingCoordinatesSent = false;

bool IDLE_called = true;

enum ERROR_CODE {
	MOTORFAILED_BOX, MOTORERROR_BOX, MOTORERROR_PICKUPCOOR, MOTORFAILED_PICKUPCOOR, MOTORFAILED_PCB, MOTORERROR_PCB, NOCOMPONENTFOUND,
	MOTORERROR_HOMING, MOTORFAILED_HOMING, MOTORERROR_BOTTOM, MOTORFAILED_BOTTOM, MOTORERROR_PLACE, MOTORFAILED_PLACE
} error_code;

enum STATE {
	IDLE, CALIBRATE, GOTOPCBORIGIN, FINDPADS, GOTOBOX, FINDCOMPONENT, GOTOPICKUPCOOR, STARTPICKUP, GOTOBOTTOMCAM, CHECKCOMPONENTPICKUP, GOTOPCBCOMP,
	CHECKCOMPPOSITON, GOTOPLACECOORD,
	GOTOPCB,
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

	ros::Rate loop_rate(5);
	state = IDLE;

	while (ros::ok()) {
		switch (state) {
		case IDLE:
			if (IDLE_called) {
				ROS_INFO("PlacerState: IDLE");
				IDLE_called = false;
			}
			break;

		case CALIBRATE:
			ROS_INFO("PlacerState: CALIBRATE");
			break;

		case GOTOPCBORIGIN:
			ROS_INFO("PlacerState: GOTOPCBORIGIN");

			if (!placerNodeBusy) {
				Offset destination = placeController.getPCBCalibCoordinates();
				sendTask(pap_common::CONTROLLER, pap_common::COORD, destination.x, destination.y, destination.z);
				pcbOrigCoordinatesSent = true;
				placerNodeBusy = true;
			}

			if (motorcontrollerStatus[1].positionReached && motorcontrollerStatus[2].positionReached && motorcontrollerStatus[3].positionReached && boxCoordinatesSent) {
				placerNodeBusy = false;
				if (manualOperation) {
					if (visionEnabled) {
						state = FINDPADS;
					} else {
						IDLE_called = true;
						state = IDLE;
					}
				} else {
					if (visionEnabled) {
						state = FINDPADS;
					} else {
						state = IDLE;
					}
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

		case GOTOBOX:			// Send coordinates to motor controller and wait until position reached
			ROS_INFO("PlacerState: GOTOBOX");

			if (!placerNodeBusy && !boxCoordinatesSent && placeController.getCalibrationStatus()) {
				Offset destination = placeController.getBoxCoordinates();
				sendTask(pap_common::CONTROLLER, pap_common::COORD, destination.x, destination.y, destination.z);
				setLEDTask(placeController.getBoxNumber());
				boxCoordinatesSent = true;
				placerNodeBusy = true;
			}

			// TODO: Add a function to decide if vision is needed for this process step
			if (motorcontrollerStatus[1].positionReached && motorcontrollerStatus[2].positionReached && motorcontrollerStatus[3].positionReached && boxCoordinatesSent) {
				placerNodeBusy = false;
				if (manualOperation) {
					if (visionEnabled) {
						state = FINDCOMPONENT;
					} else {
						//IDLE_called = true;
						//state = IDLE;
						ros::Duration(5).sleep();
						state = GOTOPICKUPCOOR;
					}
				} else {
					if (visionEnabled) {
						state = FINDCOMPONENT;
					} else {
						state = GOTOPICKUPCOOR;
					}
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

		case FINDCOMPONENT:
			ROS_INFO("PlacerState: FINDCOMPONENT");

			if (!componentFinderStarted && !placerNodeBusy) {
				sendTask(pap_common::VISION, pap_vision::START_VISION);
				float length = placeController.getComponentLenth();
				float width = placeController.getComponentWidth();
				if(placeController.selectFinder() == 3) {
					sendTask(pap_common::VISION, pap_vision::START_CHIP_FINDER);
				} else if (placeController.selectFinder() == 4) {
					sendTask(pap_common::VISION, pap_vision::START_SMALL_FINDER, width, length, 0.0);
				} else if (placeController.selectFinder() == 5) {
					sendTask(pap_common::VISION, pap_vision::START_TAPE_FINDER, width, length, 0.0);
				}
				componentFinderStarted = true;
				placerNodeBusy = true;
			}

			if (componentFound) {
				sendTask(pap_common::VISION, pap_vision::STOP_VISION);
				placerNodeBusy = false;
				state = GOTOPICKUPCOOR;		// Component position has been update in vision callback function
			} else if (componentFinder_counter == COMPFINDERTIMEOUT) {
				sendTask(pap_common::VISION, pap_vision::STOP_VISION);
				placerNodeBusy = false;
				error_code = NOCOMPONENTFOUND;
				state = ERROR;
			} else {
				componentFinder_counter++;
			}
			break;


		case GOTOPICKUPCOOR:			// Send coordinates to motor controller and wait until position reached
			ROS_INFO("PlacerState: GOTOPICKUPCOOR");

			if (!compPickUpCoordinatesSent && !placerNodeBusy) {
				Offset destination = placeController.getCompPickUpCoordinates();
				sendTask(pap_common::CONTROLLER, pap_common::COORD, destination.x, destination.y, destination.z);
				compPickUpCoordinatesSent = true;
				placerNodeBusy = true;
			}

			if (motorcontrollerStatus[1].positionReached && motorcontrollerStatus[2].positionReached && motorcontrollerStatus[3].positionReached && compPickUpCoordinatesSent) {
				placerNodeBusy = false;
				if (manualOperation) {
					//IDLE_called = true;
					//state = IDLE;
					ros::Duration(5).sleep();
					state = STARTPICKUP;
				} else {
					state = STARTPICKUP;
				}
				break;
			}
			else if (motorcontrollerStatus[1].error || motorcontrollerStatus[2].error || motorcontrollerStatus[3].error) {
				error_code = MOTORERROR_PICKUPCOOR;
				state = ERROR;
				break;
			}
			else if (motorcontrollerStatus[1].failed || motorcontrollerStatus[2].failed || motorcontrollerStatus[3].failed) {
				error_code = MOTORFAILED_PICKUPCOOR;
				state = ERROR;
				break;
			}
			break;


		case STARTPICKUP:			// Start pick-up process by activating vacuum
			ROS_INFO("PlacerState: STARTPICKUP");

			// Make sure that box position of component has been reached before trying to pick up
			if (boxCoordinatesSent && !placerNodeBusy && !componentPickUpStarted) {
				componentPickUpStarted = true;

				sendRelaisTask(1, false);				// Turn on vacuum
				sendRelaisTask(2, true);
				ros::Duration(1).sleep();				// Wait until vacuum has been built up

				if (placeController.selectTip()) {		// Activate tip
					sendRelaisTask(3, false);			// Left tip
					sendRelaisTask(6, true);
				} else {
					sendRelaisTask(7, true);			// Right tip
				}
				ros::Duration(1).sleep();				// Wait until cylinder activated

				if (placeController.selectTip()) {
					sendRelaisTask(5, true);			// Turn on left vacuum valve
				} else {
					sendRelaisTask(4, true);			// Turn on right vacuum valve
				}
				ros::Duration(1).sleep();				// Wait until component fixed

				if (placeController.selectTip()) {		// Release tip
					sendRelaisTask(6, false);
					sendRelaisTask(3, true);			// Left tip

				} else {
					sendRelaisTask(7, false);			// Right tip
				}
				ros::Duration(1).sleep();				// Wait until cylinder released

				Offset destination = placeController.getCompPickUpCoordinates();
				sendStepperTask(2, (int)destination.rot);	// Turn component

				if (manualOperation) {
					if (visionEnabled) {
						state = CHECKCOMPONENTPICKUP;
					}
					else {
						//IDLE_called = true;
						//state = IDLE;
						ros::Duration(5).sleep();
						state = GOTOBOTTOMCAM;
					}
				} else {
					if (visionEnabled) {
						state = GOTOBOTTOMCAM;
					}
					else {
						state = GOTOPCB;
					}
				}
			}
			break;

		case GOTOBOTTOMCAM:
			ROS_INFO("PlacerState: GOTOBOTTOMCAM");
			if (!placerNodeBusy && !bottomCamCoordinatesSent) {
				Offset destination = placeController.getPCBCompCoordinates();
				sendTask(pap_common::CONTROLLER, pap_common::COORD, destination.x, destination.y, destination.z);
				// Set bottom camera light
				bottomCamCoordinatesSent = true;
				placerNodeBusy = true;
			}

			if (motorcontrollerStatus[1].positionReached && motorcontrollerStatus[2].positionReached && motorcontrollerStatus[3].positionReached && bottomCamCoordinatesSent) {
				placerNodeBusy = false;
				if (!visionEnabled) {
					//IDLE_called = true;
					//state = IDLE;
					ros::Duration(5).sleep();
					state = CHECKCOMPONENTPICKUP;
				} else {
					state = CHECKCOMPONENTPICKUP;
				}
				break;
			}
			else if (motorcontrollerStatus[1].error || motorcontrollerStatus[2].error || motorcontrollerStatus[3].error) {
				error_code = MOTORERROR_BOTTOM;
				state = ERROR;
				break;
			}
			else if (motorcontrollerStatus[1].failed || motorcontrollerStatus[2].failed || motorcontrollerStatus[3].failed) {
				error_code = MOTORFAILED_BOTTOM;
				state = ERROR;
				break;
			}
			break;


		case CHECKCOMPONENTPICKUP:
			ROS_INFO("PlacerState: CHECKCOMPONENTPICKUP");
			ros::Duration(5).sleep();
			IDLE_called = true;
			state = IDLE;
			break;



		case GOTOPCBCOMP:
			ROS_INFO("PlacerState: GOTOPCBCOMP");

			if (!placerNodeBusy && !pcbCoordinatesSent && placeController.getCalibrationStatus()) {
				Offset destination = placeController.getPCBCompCoordinates();
				sendTask(pap_common::CONTROLLER, pap_common::COORD, destination.x, destination.y, destination.z);
				resetLEDTask(placeController.getBoxNumber());
				pcbCoordinatesSent = true;
				placerNodeBusy = true;
			}

			if (motorcontrollerStatus[1].positionReached && motorcontrollerStatus[2].positionReached && motorcontrollerStatus[3].positionReached && pcbCoordinatesSent) {
				placerNodeBusy = false;
				if (manualOperation) {
					//IDLE_called = true;
					//state = IDLE;
					state = CHECKCOMPPOSITON;
				} else {
					state = CHECKCOMPPOSITON;
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

		case CHECKCOMPPOSITON:
			ROS_INFO("PlacerState: CHECKCOMPPOSITON");
			ros::Duration(5).sleep();
			state = GOTOPLACECOORD;
			break;




		case GOTOPLACECOORD:			// Component coord + correction offset and camToTip offset
			ROS_INFO("PlacerState: GOTOPLACECOORD");

			if (!placerNodeBusy && !pcbPlaceCoordinatesSent) {
				Offset destination = placeController.getCompPlaceCoordinates();
				sendTask(pap_common::CONTROLLER, pap_common::COORD, destination.x, destination.y, destination.z);
				pcbPlaceCoordinatesSent = true;
				placerNodeBusy = true;
			}

			if (motorcontrollerStatus[1].positionReached && motorcontrollerStatus[2].positionReached && motorcontrollerStatus[3].positionReached && pcbPlaceCoordinatesSent) {
				placerNodeBusy = false;
				if (manualOperation) {
					IDLE_called = true;
					state = IDLE;
				} else {
					state = STARTPLACEMENT;
				}
				break;
			}
			else if (motorcontrollerStatus[1].error || motorcontrollerStatus[2].error || motorcontrollerStatus[3].error) {
				error_code = MOTORERROR_PLACE;
				state = ERROR;
				break;
			}
			else if (motorcontrollerStatus[1].failed || motorcontrollerStatus[2].failed || motorcontrollerStatus[3].failed) {
				error_code = MOTORFAILED_PLACE;
				state = ERROR;
				break;
			}
			break;


		case STARTPLACEMENT:
			ROS_INFO("PlacerState: STARTPLACEMENT");

			// Make sure that pcb position of component has been reached before trying to place
			if (pcbPlaceCoordinatesSent && !placerNodeBusy && !componentPlacementStarted) {
				placerNodeBusy = true;
				componentPickUpStarted = true;

				if (placeController.selectTip()) {
					sendRelaisTask(3, false);			// Activate left cylinder
					sendRelaisTask(6, true);
				} else {
					sendRelaisTask(7, true);			// Activate right cylinder
				}
				ros::Duration(1).sleep();

				sendRelaisTask(2, false);				// Turn off vacuum
				sendRelaisTask(1, true);
				if (placeController.selectTip()) {
					sendRelaisTask(5, false);			// Turn off left vacuum valve
				} else {
					sendRelaisTask(4, false);			// Turn off right vacuum valve
				}
				ros::Duration(1).sleep();

				if (placeController.selectTip()) {
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
						//IDLE_called = true;
						//state = IDLE;
						state = HOMING;
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
			}
			break;


		case HOMING:
			ROS_INFO("PlacerState: HOMING");
			if (!placerNodeBusy && !homingCoordinatesSent) {
				homingCoordinatesSent = true;
				placerNodeBusy = true;
				sendTask(pap_common::CONTROLLER, pap_common::HOMING);
			}

			if (motorcontrollerStatus[1].positionReached && motorcontrollerStatus[2].positionReached && motorcontrollerStatus[3].positionReached && pcbCoordinatesSent) {
				resetProcessVariables();
				resetStepper();
				placerNodeBusy = false;
				IDLE_called = true;
				state = IDLE;
			}
			else if (motorcontrollerStatus[1].error || motorcontrollerStatus[2].error || motorcontrollerStatus[3].error) {
				error_code = MOTORERROR_HOMING;
				state = ERROR;
			}
			else if (motorcontrollerStatus[1].failed || motorcontrollerStatus[2].failed || motorcontrollerStatus[3].failed) {
				error_code = MOTORFAILED_HOMING;
				state = ERROR;
			}
			break;

		case ERROR:				// Stop and publish error code
			ROS_INFO("PlacerState: ERROR");
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

	switch (statusMsg->task) {
	case pap_vision::START_PAD_FINDER:

		float xDiff = statusMsg->data1;
		float yDiff = statusMsg->data2;
		float rotDiff = statusMsg->data3;
		placeController.setPlaceCorrectionOffset(xDiff, yDiff, rotDiff);

	//case pap_vision::
	}
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
			//state = STARTPICKUP;
			ROS_INFO("Pick-up component called...");
			break;
		case pap_common::GOTOBOX:
			boxCoordinatesSent = false;		// Has to be set to able to go to a new position!
			compPickUpCoordinatesSent = false;
			componentPickUpStarted = false;
			bottomCamCoordinatesSent = false;
			state = GOTOBOX;
			ROS_INFO("go to box called...");
			break;
		case pap_common::GOTOPCB:
			state = GOTOPCBCOMP;
			pcbPlaceCoordinatesSent = false;
			componentPlacementStarted = false;
			ROS_INFO("go to pcb called...");
			break;
		case pap_common::PLACEMENT:
			state = STARTPLACEMENT;
			ROS_INFO("Placement called...");
			break;
		case pap_common::CALIBRATION:
			state = CALIBRATE;
			ROS_INFO("Calibration called...");
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

void resetProcessVariables() {
	boxCoordinatesSent = false;
	componentPickUpStarted = false;
	pcbCoordinatesSent = false;
	componentPlacementStarted = false;
	componentFinderStarted = false;
	componentFound = false;
	placerNodeBusy = false;
	pickupwait_counter = 0;
	placementwait_counter = 0;
	componentFinder_counter = 0;
}

