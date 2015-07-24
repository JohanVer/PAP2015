#include "ros/ros.h"
#include "std_msgs/String.h"

#include "pap_common/Task.h"
#include "pap_common/Status.h"
#include "pap_common/VisionStatus.h"
#include "pap_common/ArduinoMsg.h"
#include "pap_common/PlacerStatus.h"
#include "../../pap_common/include/pap_common/task_message_def.h"
#include "../../pap_common/include/pap_common/status_message_def.h"
#include "../../pap_common/include/pap_common/arduino_message_def.h"
#include "../../pap_common/include/pap_common/vision_message_def.h"
#include "../../pap_common/include/pap_common/placer_message_def.h"

#include "../../pap_placer/include/pap_placer/placerNode.hpp"
//#include "../include/pap_placer/placerNode.hpp"
#include "../include/pap_placer/placerClass.hpp"
//#include "../../motor_controller/include/motorController/controllerClass.hpp"

#define PICKUPDELAY1 5
#define PICKUPDELAY2 5
#define PLACEMENTDELAY1 5
#define PLACEMENTDELAY2 5
#define COMPFINDERTIMEOUT 300
#define MOTORCONTROLLER_TIMEOUT 2000

#define TIP1_DIAMETER_VISION 20.0
#define TIP2_DIAMETER_VISION 20.0
#define DISPENSER_DIAMETER_VISION 1.0
#define CAMERA_DIAMETER_VISION 11.0

int pickupwait_counter, placementwait_counter = 0;
int componentFinder_counter = 0;
unsigned int counterMean = 0;
PlaceController placeController;
ComponentPlacerData currentComponent;
controllerStatus motorcontrollerStatus[3];

/* Call back functions */
void statusCallback(const pap_common::StatusConstPtr& statusMsg);
void visionStatusCallback(const pap_common::VisionStatusConstPtr& statusMsg);
void placerCallback(const pap_common::TaskConstPtr& taskMsg);

/* Send task functions */
void sendTask(pap_common::DESTINATION, pap_common::TASK);
void sendTask(pap_common::DESTINATION, pap_vision::VISION);
void sendTask(pap_common::DESTINATION destination, pap_common::TASK task,
		float x, float y, float z);
void sendTask(pap_common::DESTINATION destination, pap_vision::VISION task,
		float x, float y, float z);
void sendPlacerStatus(pap_common::PROCESS process,
		pap_common::PLACER_STATUS status);
void sendPlacerInfo(int state);

/* Send arduino task funcionts */
void sendRelaisTask(int relaisNumber, bool value);
void sendStepperTask(int StepperNumber, int rotationAngle);
void resetStepper();
void setLEDTask(int LEDnumber);
void resetLEDTask(int LEDnumber);

void resetProcessVariables();

ros::Publisher task_publisher, arduino_publisher_, placerStatus_publisher_;
ros::Subscriber statusSubsriber_;
ros::Subscriber visionStatusSubsriber_;
ros::Subscriber placerTaskSubscriber_;

bool visionEnabled = true;
bool manualOperation = true;

bool placerNodeBusy = false;
bool positionSend = false;
bool visionStarted = false;
int zPosReached = 0;

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
bool pcbBottomCamCoordinatesSent = false;
bool cameraPositionReceived = false;

bool IDLE_called = true;
int motorcontroller_counter = 0;
enum CALIBRATION_STATE {
	CAMERA, TIP1, DISPENSER, TIP2
} calibration_state;

enum ERROR_CODE {
	MOTOR_TIMEOUT,
	MOTOR_FAILED,
	MOTOR_ERROR,
	MOTORFAILED_BOX,
	MOTORERROR_BOX,
	MOTORERROR_PICKUPCOOR,
	MOTORFAILED_PICKUPCOOR,
	MOTORFAILED_PCB,
	MOTORERROR_PCB,
	NOCOMPONENTFOUND,
	MOTORERROR_HOMING,
	MOTORFAILED_HOMING,
	MOTORERROR_BOTTOM,
	MOTORFAILED_BOTTOM,
	MOTORERROR_PLACE,
	MOTORFAILED_PLACE
} error_code;

enum STATE {
	IDLE,
	CALIBRATE,
	GOTOPCBORIGIN,
	FINDPADS,
	GOTOBOX,
	FINDCOMPONENT,
	GOTOPICKUPCOOR,
	STARTPICKUP,
	GOTOBOTTOMCAM,
	CHECKCOMPONENTPICKUP,
	GOTOPCBCOMP,
	CHECKCOMPPOSITON,
	GOTOPLACECOORD,
	CHECKCOMPONENTPOSITION,
	STARTPLACEMENT,
	CHECKCOMPONENTPLACEMENT,
	GOTOCOORD,
	HOMING,
	ERROR
} state, last_state;

int main(int argc, char **argv) {

	ros::init(argc, argv, "motorController");
	if (!ros::master::check()) {
		ROS_INFO("PlacerNode did not start...");
		return 0;
	}
	ROS_INFO("PlacerNode started...");

	ros::NodeHandle n_;
	task_publisher = n_.advertise<pap_common::Task>("task", 1000);
	arduino_publisher_ = n_.advertise<pap_common::ArduinoMsg>("arduinoTx",
			1000);
	placerStatus_publisher_ = n_.advertise<pap_common::PlacerStatus>(
			"placerStatus", 1000);
	statusSubsriber_ = n_.subscribe("status", 100, &statusCallback);
	visionStatusSubsriber_ = n_.subscribe("visionStatus", 100,
			&visionStatusCallback);
	placerTaskSubscriber_ = n_.subscribe("task", 10, &placerCallback);

	ros::Rate loop_rate(100);
	state = IDLE;
	calibration_state = CAMERA;

	while (ros::ok()) {
		switch (state) {
		case IDLE:
			if (IDLE_called) {
				ROS_INFO("PlacerState: IDLE");
				sendPlacerInfo(IDLE);
				IDLE_called = false;
			}
			sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_ACTIVE);
			break;

		case CALIBRATE:
			//ROS_INFO("PlacerState: CALIBRATE");
			sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_IDLE);
			sendPlacerStatus(pap_common::CALIBRATION_STATE,
					pap_common::PLACER_ACTIVE);

			switch (calibration_state) {
			case CAMERA:
				if (!positionSend) {
					ROS_INFO("PlacerState: CAMERA");
					placeController.currentDestination_ =
							placeController.getBottomCamCoordinates();
					ROS_INFO("Go to: x:%f y:%f z:%f",
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.currentDestination_.z);
					positionSend = true;
					last_state = state;
					state = GOTOCOORD;
				} else {

					if (!visionStarted) {
						ROS_INFO("Camera: Vision started");
						ros::Duration(3).sleep();
						counterMean = 0;
						placeController.camClibrationOffset_.x = 0.0;
						placeController.camClibrationOffset_.y = 0.0;
						sendTask(pap_common::VISION, pap_vision::START_VISION);
						sendTask(pap_common::VISION, pap_vision::SEARCH_CIRCLE,
						CAMERA_DIAMETER_VISION, 0.0, 0.0);
						visionStarted = true;

					}

					if (cameraPositionReceived) {
						ROS_INFO("Camera: cameraOffsetreceived");
						sendTask(pap_common::VISION, pap_vision::STOP_VISION);
						//ros::Duration(1).sleep();
						//IDLE_called = true;
						//state = IDLE;
						calibration_state = TIP1;
						cameraPositionReceived = false;
						positionSend = false;
						visionStarted = false;
					}
				}
				break;

			case TIP1:
				if (!positionSend) {
					ROS_INFO("PlacerState: TIP1 ");
					placeController.currentDestination_ =
							placeController.getTip1Coordinates();
					ROS_INFO("Go to: x:%f y:%f z:%f",
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.currentDestination_.z);
					positionSend = true;
					last_state = state;
					state = GOTOCOORD;
				} else {
					if (!visionStarted) {
						ros::Duration(3).sleep();
						counterMean = 0;
						placeController.tip1ClibrationOffset_.x = 0.0;
						placeController.tip1ClibrationOffset_.y = 0.0;
						sendTask(pap_common::VISION, pap_vision::START_VISION);
						sendTask(pap_common::VISION, pap_vision::SEARCH_CIRCLE,
						TIP1_DIAMETER_VISION, 0.0, 0.0);
						visionStarted = true;

					}

					if (cameraPositionReceived) {
						sendTask(pap_common::VISION, pap_vision::STOP_VISION);
						calibration_state = DISPENSER;
						cameraPositionReceived = false;
						positionSend = false;
						visionStarted = false;
					}
				}
				break;
			case DISPENSER:
				if (!positionSend) {
					ROS_INFO("PlacerState: DISPENSER ");
					placeController.currentDestination_ =
							placeController.getDispenserCoordinates();
					ROS_INFO("Go to: x:%f y:%f z:%f",
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.currentDestination_.z);
					positionSend = true;
					last_state = state;
					state = GOTOCOORD;
				} else {
					if (!visionStarted) {
						ros::Duration(3).sleep();
						sendTask(pap_common::VISION, pap_vision::START_VISION);
						sendTask(pap_common::VISION, pap_vision::SEARCH_CIRCLE,
						DISPENSER_DIAMETER_VISION, 0.0, 0.0);
						visionStarted = true;
					}

					if (cameraPositionReceived) {
						sendTask(pap_common::VISION, pap_vision::STOP_VISION);
						calibration_state = TIP2;
						cameraPositionReceived = false;
						positionSend = false;
						visionStarted = false;
					}
				}
				break;

			case TIP2:

				if (!positionSend) {
					ROS_INFO("PlacerState: TIP2 ");
					placeController.currentDestination_ =
							placeController.getTip2Coordinates();
					ROS_INFO("Go to: x:%f y:%f z:%f",
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.currentDestination_.z);
					positionSend = true;
					last_state = state;
					state = GOTOCOORD;
				} else {
					if (!visionStarted) {
						ros::Duration(3).sleep();
						sendTask(pap_common::VISION, pap_vision::START_VISION);
						sendTask(pap_common::VISION, pap_vision::SEARCH_CIRCLE,
						TIP2_DIAMETER_VISION, 0.0, 0.0);
						visionStarted = true;
					}

					if (cameraPositionReceived) {
						sendTask(pap_common::VISION, pap_vision::STOP_VISION);
						calibration_state = CAMERA;
						cameraPositionReceived = false;
						positionSend = false;
						visionStarted = false;
						IDLE_called = true;
						state = IDLE;
					}
				}
				break;
			}

			break;

		case GOTOPCBORIGIN:
			ROS_INFO("PlacerState: GOTOPCBORIGIN");
			if (!placerNodeBusy) {
				Offset destination = placeController.getPCBCalibCoordinates();
				sendTask(pap_common::CONTROLLER, pap_common::COORD,
						destination.x, destination.y, destination.z);
				ros::Duration(0.1).sleep();
				pcbOrigCoordinatesSent = true;
				placerNodeBusy = true;
			}

			if (motorcontrollerStatus[1].positionReached
					&& motorcontrollerStatus[2].positionReached
					&& motorcontrollerStatus[3].positionReached
					&& boxCoordinatesSent) {
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
			} else if (motorcontrollerStatus[1].error
					|| motorcontrollerStatus[2].error
					|| motorcontrollerStatus[3].error) {
				error_code = MOTORERROR_BOX;
				state = ERROR;
				break;
			} else if (motorcontrollerStatus[1].failed
					|| motorcontrollerStatus[2].failed
					|| motorcontrollerStatus[3].failed) {
				error_code = MOTORFAILED_BOX;
				state = ERROR;
				break;
			}

			break;

			// Send coordinates to motor controller and wait until position reached
		case GOTOBOX:

			sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_IDLE);
			sendPlacerStatus(pap_common::GOTOBOX_STATE,
					pap_common::PLACER_ACTIVE);

			if (!positionSend) {
				setLEDTask(placeController.getBoxNumber());
				placeController.currentDestination_ =
						placeController.getBoxCoordinates();
				ROS_INFO("Go to: x:%f y:%f z:%f",
						placeController.currentDestination_.x,
						placeController.currentDestination_.y,
						placeController.currentDestination_.z);
				positionSend = true;
				last_state = state;
				state = GOTOCOORD;
			} else {
				if (!visionStarted) {
					sendTask(pap_common::VISION, pap_vision::START_VISION);
					float length = placeController.getComponentLenth();
					float width = placeController.getComponentWidth();
					if (placeController.selectFinder() == 3) {
						sendTask(pap_common::VISION,
								pap_vision::START_CHIP_FINDER, width, length,
								0);
					} else if (placeController.selectFinder() == 4) {
						sendTask(pap_common::VISION,
								pap_vision::START_SMALL_FINDER, width, length,
								0);
					} else if (placeController.selectFinder() == 5) {
						sendTask(pap_common::VISION,
								pap_vision::START_TAPE_FINDER, width, length,
								0);
					}
					visionStarted = true;
					ros::Duration(3).sleep();
				}

				if (cameraPositionReceived) {
					sendTask(pap_common::VISION, pap_vision::STOP_VISION);
					cameraPositionReceived = false;
					positionSend = false;
					visionStarted = false;
					state = GOTOPICKUPCOOR;
				}
			}

			break;

		case GOTOPICKUPCOOR:
			// Send coordinates to motor controller and wait until position reached

			if (!positionSend) {
				placeController.currentDestination_ =
						placeController.getCompPickUpCoordinates();
				ROS_INFO("Go to: x:%f y:%f z:%f",
						placeController.currentDestination_.x,
						placeController.currentDestination_.y,
						placeController.currentDestination_.z);
				positionSend = true;
				last_state = state;
				state = GOTOCOORD;
			} else {
				positionSend = false;
				IDLE_called = true;
				state = IDLE;
			}
			break;

		case GOTOCOORD:

			if (!placerNodeBusy) {

				if (zPosReached == 0) {
					sendTask(pap_common::CONTROLLER, pap_common::COORD,
							placeController.lastDestination_.x,
							placeController.lastDestination_.y,
							placeController.zMoveheight_);

					ROS_INFO("PlacerState: GOTOCOORD: x=%f y=%f z=%f",
							placeController.lastDestination_.x,
							placeController.lastDestination_.y,
							placeController.zMoveheight_);
					ROS_INFO("zPos: %d", zPosReached);

				} else if (zPosReached == 1) {
					sendTask(pap_common::CONTROLLER, pap_common::COORD,
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.zMoveheight_);

					ROS_INFO("PlacerState: GOTOCOORD: x=%f y=%f z=%f",
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.zMoveheight_);
					ROS_INFO("zPos: %d", zPosReached);

				} else if (zPosReached == 2) {
					sendTask(pap_common::CONTROLLER, pap_common::COORD,
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.currentDestination_.z);

					ROS_INFO("PlacerState: GOTOCOORD: x=%f y=%f z=%f",
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.currentDestination_.z);

					ROS_INFO("zPos: %d", zPosReached);
				}
				ros::Duration(0.3).sleep();
				motorcontroller_counter = 0;
				placerNodeBusy = true;
			}

			if (motorcontrollerStatus[1].positionReached
					&& motorcontrollerStatus[2].positionReached
					&& motorcontrollerStatus[3].positionReached
					&& placerNodeBusy) {
				placerNodeBusy = false;
				if (zPosReached == 0 || zPosReached == 1) {
					zPosReached++;
					state = GOTOCOORD;
				} else if (zPosReached == 2) {
					zPosReached = 0;
					state = last_state;
					ROS_INFO("Last state: %d", last_state);
				}
//				ros::Duration(3).sleep();
				break;

			} else if (motorcontrollerStatus[1].error
					|| motorcontrollerStatus[2].error
					|| motorcontrollerStatus[3].error) {
				sendPlacerStatus(pap_common::GOTOBOX_STATE,
						pap_common::PLACER_ERROR);
				error_code = MOTOR_ERROR;
				state = ERROR;
				break;

			} else if (motorcontrollerStatus[1].failed
					|| motorcontrollerStatus[2].failed
					|| motorcontrollerStatus[3].failed) {
				sendPlacerStatus(pap_common::GOTOBOX_STATE,
						pap_common::PLACER_ERROR);
				error_code = MOTOR_FAILED;
				state = ERROR;
				break;

			} else if (motorcontroller_counter == MOTORCONTROLLER_TIMEOUT) {
				error_code = MOTOR_TIMEOUT;
				state = ERROR;
				break;
			} else {
				motorcontroller_counter++;
				break;
			}
			break;

		case STARTPICKUP:
			// Start pick-up process by activating vacuum
			ROS_INFO("PlacerState: STARTPICKUP");
			sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_IDLE);
			sendPlacerStatus(pap_common::STARTPICKUP_STATE,
					pap_common::PLACER_ACTIVE);

			// Make sure that box position of component has been reached before trying to pick up
			if (compPickUpCoordinatesSent && !placerNodeBusy
					&& !componentPickUpStarted) {
				componentPickUpStarted = true;

				sendRelaisTask(1, false);				// Turn on vacuum
				sendRelaisTask(2, true);
				ros::Duration(1).sleep();// Wait until vacuum has been built up

				if (placeController.selectTip()) {		// Activate tip
					sendRelaisTask(3, false);			// Left tip
					sendRelaisTask(6, true);
				} else {
					sendRelaisTask(7, true);			// Right tip
				}
				ros::Duration(1).sleep();	// Wait until cylinder activated

				if (placeController.selectTip()) {
					sendRelaisTask(5, true);	// Turn on left vacuum valve
				} else {
					sendRelaisTask(4, true);	// Turn on right vacuum valve
				}
				ros::Duration(1).sleep();	// Wait until component fixed

				if (placeController.selectTip()) {		// Release tip
					sendRelaisTask(6, false);
					sendRelaisTask(3, true);			// Left tip

				} else {
					sendRelaisTask(7, false);			// Right tip
				}
				ros::Duration(1).sleep();	// Wait until cylinder released

				Offset destination = placeController.getCompPickUpCoordinates();
				//sendStepperTask(2, (int) destination.rot);	// Turn component

				if (manualOperation) {
					if (visionEnabled) {
						state = CHECKCOMPONENTPICKUP;
					} else {
						//IDLE_called = true;
						//state = IDLE;
						ros::Duration(2).sleep();
						state = GOTOBOTTOMCAM;
					}
				} else {
					if (visionEnabled) {
						state = GOTOBOTTOMCAM;
					} else {
						state = GOTOPCBCOMP;
					}
				}
			}
			break;

		case GOTOBOTTOMCAM:
			ROS_INFO("PlacerState: GOTOBOTTOMCAM");
			if (!placerNodeBusy && !bottomCamCoordinatesSent) {
				Offset destination = placeController.getBottomCamCoordinates();
				sendTask(pap_common::CONTROLLER, pap_common::COORD,
						destination.x, destination.y, destination.z);
				// Set bottom camera light
				bottomCamCoordinatesSent = true;
				placerNodeBusy = true;
			}

			if (motorcontrollerStatus[1].positionReached
					&& motorcontrollerStatus[2].positionReached
					&& motorcontrollerStatus[3].positionReached
					&& bottomCamCoordinatesSent) {
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
			} else if (motorcontrollerStatus[1].error
					|| motorcontrollerStatus[2].error
					|| motorcontrollerStatus[3].error) {
				sendPlacerStatus(pap_common::STARTPICKUP_STATE,
						pap_common::PLACER_ERROR);
				error_code = MOTORERROR_BOTTOM;
				state = ERROR;
				break;
			} else if (motorcontrollerStatus[1].failed
					|| motorcontrollerStatus[2].failed
					|| motorcontrollerStatus[3].failed) {
				sendPlacerStatus(pap_common::STARTPICKUP_STATE,
						pap_common::PLACER_ERROR);
				error_code = MOTORFAILED_BOTTOM;
				state = ERROR;
				break;
			}
			break;

		case CHECKCOMPONENTPICKUP:
			ROS_INFO("PlacerState: CHECKCOMPONENTPICKUP");
			ros::Duration(2).sleep();
			sendPlacerStatus(pap_common::STARTPICKUP_STATE,
					pap_common::PLACER_FINISHED);
			IDLE_called = true;
			state = IDLE;
			break;

		case GOTOPCBCOMP:
			ROS_INFO("PlacerState: GOTOPCBCOMP");
			sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_IDLE);
			sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE,
					pap_common::PLACER_ACTIVE);

			if (!placerNodeBusy && !pcbCoordinatesSent) {
				Offset destination = placeController.getPCBCompCoordinates();
				sendTask(pap_common::CONTROLLER, pap_common::COORD,
						destination.x, destination.y, destination.z);
				resetLEDTask(placeController.getBoxNumber());
				pcbCoordinatesSent = true;
				placerNodeBusy = true;
			}

			if (motorcontrollerStatus[1].positionReached
					&& motorcontrollerStatus[2].positionReached
					&& motorcontrollerStatus[3].positionReached
					&& pcbCoordinatesSent) {
				placerNodeBusy = false;
				if (manualOperation) {
					//IDLE_called = true;
					//state = IDLE;
					state = CHECKCOMPPOSITON;
				} else {
					state = CHECKCOMPPOSITON;
				}
				break;
			} else if (motorcontrollerStatus[1].error
					|| motorcontrollerStatus[2].error
					|| motorcontrollerStatus[3].error) {
				error_code = MOTORERROR_PCB;
				state = ERROR;
				break;
			} else if (motorcontrollerStatus[1].failed
					|| motorcontrollerStatus[2].failed
					|| motorcontrollerStatus[3].failed) {
				error_code = MOTORFAILED_PCB;
				state = ERROR;
				break;
			}
			break;

		case CHECKCOMPPOSITON:
			ROS_INFO("PlacerState: CHECKCOMPPOSITON");
			ros::Duration(2).sleep();
			state = GOTOPLACECOORD;
			break;

		case GOTOPLACECOORD:
			// Component coord + correction offset and camToTip offset
			ROS_INFO("PlacerState: GOTOPLACECOORD");

			if (!placerNodeBusy && !pcbPlaceCoordinatesSent) {
				Offset destination = placeController.getCompPlaceCoordinates();
				sendTask(pap_common::CONTROLLER, pap_common::COORD,
						destination.x, destination.y, destination.z);
				pcbPlaceCoordinatesSent = true;
				placerNodeBusy = true;
			}

			if (motorcontrollerStatus[1].positionReached
					&& motorcontrollerStatus[2].positionReached
					&& motorcontrollerStatus[3].positionReached
					&& pcbPlaceCoordinatesSent) {
				sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE,
						pap_common::PLACER_FINISHED);
				placerNodeBusy = false;
				if (manualOperation) {
					IDLE_called = true;
					state = IDLE;
				} else {
					state = STARTPLACEMENT;
				}
				break;
			} else if (motorcontrollerStatus[1].error
					|| motorcontrollerStatus[2].error
					|| motorcontrollerStatus[3].error) {
				sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE,
						pap_common::PLACER_ERROR);
				error_code = MOTORERROR_PLACE;
				state = ERROR;
				break;
			} else if (motorcontrollerStatus[1].failed
					|| motorcontrollerStatus[2].failed
					|| motorcontrollerStatus[3].failed) {
				sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE,
						pap_common::PLACER_ERROR);
				error_code = MOTORFAILED_PLACE;
				state = ERROR;
				break;
			}
			break;

		case STARTPLACEMENT:
			ROS_INFO("PlacerState: STARTPLACEMENT");
			sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_IDLE);
			sendPlacerStatus(pap_common::STARTPLACEMET_STATE,
					pap_common::PLACER_ACTIVE);

			// Make sure that pcb position of component has been reached before trying to place
			if (pcbPlaceCoordinatesSent && !placerNodeBusy
					&& !componentPlacementStarted) {
				placerNodeBusy = true;
				componentPickUpStarted = true;

				if (placeController.selectTip()) {
					sendRelaisTask(3, false);	// Activate left cylinder
					sendRelaisTask(6, true);
				} else {
					sendRelaisTask(7, true);	// Activate right cylinder
				}
				ros::Duration(1).sleep();

				sendRelaisTask(2, false);				// Turn off vacuum
				sendRelaisTask(1, true);
				if (placeController.selectTip()) {
					sendRelaisTask(5, false);	// Turn off left vacuum valve
				} else {
					sendRelaisTask(4, false);	// Turn off right vacuum valve
				}
				ros::Duration(1).sleep();

				if (placeController.selectTip()) {
					sendRelaisTask(6, false);
					sendRelaisTask(3, true);			// Release left tip
				} else {
					sendRelaisTask(7, false);			// Release right tip
				}

				placerNodeBusy = false;
				sendPlacerStatus(pap_common::STARTPLACEMET_STATE,
						pap_common::PLACER_FINISHED);
				if (manualOperation) {
					if (visionEnabled) {
						state = CHECKCOMPONENTPLACEMENT;
					} else {

						IDLE_called = true;
						state = IDLE;
						break;
					}
				} else {
					if (visionEnabled) {
						state = CHECKCOMPONENTPLACEMENT;
					} else {
						state = HOMING;
					}
				}
			}
			break;

		case HOMING:
			ROS_INFO("PlacerState: HOMING");
			sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_IDLE);
			sendPlacerStatus(pap_common::HOMING_STATE,
					pap_common::PLACER_ACTIVE);
			if (!placerNodeBusy && !homingCoordinatesSent) {
				homingCoordinatesSent = true;
				placerNodeBusy = true;
				sendTask(pap_common::CONTROLLER, pap_common::HOMING);
			}

			if (motorcontrollerStatus[1].positionReached
					&& motorcontrollerStatus[2].positionReached
					&& motorcontrollerStatus[3].positionReached
					&& pcbCoordinatesSent) {
				sendPlacerStatus(pap_common::GOTOBOX_STATE,
						pap_common::PLACER_FINISHED);
				resetProcessVariables();
				resetStepper();
				placerNodeBusy = false;
				IDLE_called = true;
				state = IDLE;
			} else if (motorcontrollerStatus[1].error
					|| motorcontrollerStatus[2].error
					|| motorcontrollerStatus[3].error) {
				error_code = MOTORERROR_HOMING;
				state = ERROR;
			} else if (motorcontrollerStatus[1].failed
					|| motorcontrollerStatus[2].failed
					|| motorcontrollerStatus[3].failed) {
				error_code = MOTORFAILED_HOMING;
				state = ERROR;
			}
			break;

		case ERROR:
			// Stop and publish error code
			ROS_INFO("PlacerState: ERROR %d", error_code);
			IDLE_called = true;
			state = IDLE;
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

	if (statusMsg->posX != 0.0) {
		placeController.lastDestination_.x = statusMsg->posX;
	}
	if (statusMsg->posY != 0.0) {
		placeController.lastDestination_.y = statusMsg->posY;
	}
	if (statusMsg->posZ != 0.0) {
		placeController.lastDestination_.z = statusMsg->posZ;
	}
}

void visionStatusCallback(const pap_common::VisionStatusConstPtr& statusMsg) {

	float xDiff = statusMsg->data1;
	float yDiff = statusMsg->data2;
	float rotDiff = statusMsg->data3;

	switch (statusMsg->task) {
	case pap_vision::START_PAD_FINDER:
		break;

	case pap_vision::START_CHIP_FINDER:
		ROS_INFO("CHip finder called");

		if (!(xDiff == 0 && yDiff == 0) && !componentFound) {
			placeController.setPickUpCorrectionOffset(xDiff, yDiff, rotDiff);
			ROS_INFO("correction offset: x:%f y:%f", xDiff, yDiff);
			cameraPositionReceived = true;
		}
		break;

	case pap_vision::START_TAPE_FINDER:
		placeController.setPlaceCorrectionOffset(xDiff, yDiff, rotDiff);
		cameraPositionReceived = true;
		break;

	case pap_vision::SEARCH_CIRCLE:
		if (visionStarted && !cameraPositionReceived) {
			switch (calibration_state) {
			case CAMERA:

				counterMean++;
				placeController.camClibrationOffset_.x += xDiff;
				placeController.camClibrationOffset_.y += yDiff;

				if (counterMean == 50) {
					placeController.camClibrationOffset_.x /= 50;
					placeController.camClibrationOffset_.y /= 50;
					cameraPositionReceived = true;
					ROS_INFO("Cam Calibration Offset: X: %f Y: %f", placeController.camClibrationOffset_.x,
							placeController.camClibrationOffset_.y);
				}
				break;
			case TIP1:
				counterMean++;
				placeController.tip1ClibrationOffset_.x += xDiff;
				placeController.tip1ClibrationOffset_.y += yDiff;

				if (counterMean == 50) {
					placeController.tip1ClibrationOffset_.x /= 50;
					placeController.tip1ClibrationOffset_.y /= 50;
					cameraPositionReceived = true;
					ROS_INFO("Tip1 Calibration Offset: X: %f Y: %f", placeController.tip1ClibrationOffset_.x,
							placeController.tip1ClibrationOffset_.y);
				}
				break;
			case DISPENSER:
				placeController.setDispenserOffset(xDiff, yDiff);
				cameraPositionReceived = true;
				break;
			case TIP2:
				placeController.setTip2Offset(xDiff, yDiff);
				cameraPositionReceived = true;
				break;
			}

		}
		break;
	}
}

void placerCallback(const pap_common::TaskConstPtr& taskMsg) {

	switch (taskMsg->destination) {
	case pap_common::PLACER:

		if (manualOperation) {
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
			if (!manualOperation) {
				ComponentPlacerData tempComponent;
				tempComponent.destX = taskMsg->data1;
				tempComponent.destY = taskMsg->data2;
				tempComponent.box = taskMsg->box;
				tempComponent.height = taskMsg->height;
				tempComponent.length = taskMsg->length;
				tempComponent.width = taskMsg->width;
				tempComponent.rotation = taskMsg->data3;
				placeController.updatePlacementData(&tempComponent);

				state = GOTOBOX;// Start placement process with state GOTOBOX
				ROS_INFO("Complete placement process called...");
			}
			break;
		case pap_common::GOTOBOX:
			sendPlacerStatus(pap_common::GOTOBOX_STATE,
					pap_common::PLACER_IDLE);
			boxCoordinatesSent = false;
			compPickUpCoordinatesSent = false;
			resetProcessVariables();
			state = GOTOBOX;
			ROS_INFO("go to box called...");
			break;
		case pap_common::PICKUPCOMPONENT:
			sendPlacerStatus(pap_common::STARTPICKUP_STATE,
					pap_common::PLACER_IDLE);
			componentPickUpStarted = false;
			bottomCamCoordinatesSent = false;
			state = STARTPICKUP;
			ROS_INFO("Pick-up component called...");
			break;
		case pap_common::GOTOPCB:
			sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE,
					pap_common::PLACER_IDLE);
			pcbCoordinatesSent = false;
			pcbPlaceCoordinatesSent = false;
			state = GOTOPCBCOMP;
			ROS_INFO("go to pcb called...");
			break;
		case pap_common::PLACEMENT:
			sendPlacerStatus(pap_common::STARTPLACEMET_STATE,
					pap_common::PLACER_IDLE);
			componentPlacementStarted = false;
			state = STARTPLACEMENT;
			ROS_INFO("Placement called...");
			break;
		case pap_common::CALIBRATION:
			sendPlacerStatus(pap_common::CALIBRATION_STATE,
					pap_common::PLACER_IDLE);
			state = CALIBRATE;
			ROS_INFO("Calibration started...");
			break;
		case pap_common::HOMING:
			sendPlacerStatus(pap_common::HOMING_STATE, pap_common::PLACER_IDLE);
			homingCoordinatesSent = false;
			state = HOMING;
			ROS_INFO("Calibration called...");
			break;
			break;
		}
	}
}

/*****************************************************************************
 ** Send task functions
 *****************************************************************************/
void sendTask(pap_common::DESTINATION destination, pap_common::TASK task) {
	pap_common::Task taskMsg;
	taskMsg.destination = destination;
	taskMsg.task = task;
	task_publisher.publish(taskMsg);
}

void sendTask(pap_common::DESTINATION destination, pap_vision::VISION task) {
	pap_common::Task taskMsg;
	taskMsg.destination = destination;
	taskMsg.task = task;
	task_publisher.publish(taskMsg);
}

void sendTask(pap_common::DESTINATION destination, pap_vision::VISION task,
		float x, float y, float z) {
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

void sendPlacerStatus(pap_common::PROCESS process,
		pap_common::PLACER_STATUS status) {
	pap_common::PlacerStatus statusMsg;
	statusMsg.process = process;
	statusMsg.status = status;
	placerStatus_publisher_.publish(statusMsg);
}

void sendPlacerInfo(int state) {
	pap_common::PlacerStatus statusMsg;
	statusMsg.process = pap_common::INFO;
	statusMsg.status = state;
	placerStatus_publisher_.publish(statusMsg);
}

//void sendPlacerInfo()

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
	compPickUpCoordinatesSent = false;
	componentFinderStarted = false;
	componentFound = false;
	placerNodeBusy = false;
	pickupwait_counter = 0;
	placementwait_counter = 0;
	componentFinder_counter = 0;
}
