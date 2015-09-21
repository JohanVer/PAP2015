#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pap_common/Task.h"
#include "pap_common/Status.h"
#include "pap_common/VisionStatus.h"
#include "pap_common/ArduinoMsg.h"
#include "pap_common/PlacerStatus.h"
#include "pap_common/DispenseTask.h"
#include "../../pap_common/include/pap_common/task_message_def.h"
#include "../../pap_common/include/pap_common/status_message_def.h"
#include "../../pap_common/include/pap_common/arduino_message_def.h"
#include "../../pap_common/include/pap_common/vision_message_def.h"
#include "../../pap_common/include/pap_common/placer_message_def.h"
#include "../../pap_placer/include/pap_placer/placerNode.hpp"
#include "../include/pap_placer/placerClass.hpp"

/* Constant parameter definitions */
#define posTolerance 0.01 // Deviation of position in mm
#define DISPENSER_TOLERANCE 0.1
#define MOTORCONTROLLER_TIMEOUT 3000
#define TIP1_DIAMETER_VISION 15.0 //130
#define TIP2_DIAMETER_VISION 20.0
#define DISPENSER_DIAMETER_VISION 130
#define CAMERA_DIAMETER_VISION 165.0
#define DISPENSER_HEIGHT 22.2 //12,2

/* Call back functions */
void statusCallback(const pap_common::StatusConstPtr& statusMsg);
void visionStatusCallback(const pap_common::VisionStatusConstPtr& statusMsg);
void placerCallback(const pap_common::TaskConstPtr& taskMsg);
void dispenserCallback(const pap_common::DispenseTaskConstPtr& taskMsg);

/* Send task functions */
void sendTask(pap_common::DESTINATION, pap_common::TASK);
void sendTask(pap_common::DESTINATION, pap_vision::VISION);
void sendTask(pap_common::DESTINATION destination, pap_common::TASK task,
		float x, float y, float z);
void sendTask(pap_common::DESTINATION destination, pap_vision::VISION task,
		float x, float y, float z);
void sendPlacerStatus(pap_common::PROCESS process,
		pap_common::PLACER_STATUS status);
void sendTask(pap_common::DESTINATION destination, pap_common::TASK task,
		float x, float y, float z, float velx, float vely);
void sendTask(pap_common::DESTINATION destination, pap_vision::VISION task,
		int type, int camera);
void sendPlacerInfo(int state);

/* Send arduino task funcionts */
void sendRelaisTask(int relaisNumber, bool value);
void sendStepperTask(int StepperNumber, int rotationAngle);
void resetStepper();
void setLEDTask(int LEDnumber);
void resetLEDTask(int LEDnumber);
void LEDTask(int task, int data);

/* General local functions */
void resetMotorState(bool x, bool y, bool z);
void resetMotorState(int index, bool value);
void checkIfOverThreshold(int numberOfAxis, float zValue);

ros::Publisher task_publisher, arduino_publisher_, placerStatus_publisher_;
ros::Subscriber statusSubsriber_;
ros::Subscriber visionStatusSubsriber_;
ros::Subscriber placerTaskSubscriber_;
ros::Subscriber dispenserTaskSubscriber_;

/* Variable definitions and initializations*/
bool visionEnabled = true;
bool placerNodeBusy = false;
bool positionSend = false;
bool visionStarted = false;
int zPosReached = 0;
bool componentFound = false;
bool cameraFeedbackReceived = false;
bool pickUpCompleted = false;
bool dispensed = false;
unsigned int counterMean = 0;
bool manualGoto = false;
bool IDLE_called = true;
int motorcontroller_counter = 0;
int timeoutValue = 500;

PlaceController placeController;
ComponentPlacerData currentComponent;
static controllerStatus motorcontrollerStatus[3];

enum QR_CALIBRATION_PROCESS {
	GOTO_QR, PICKUP_QR, CAM_QR, RELEASE_QR, GOBACKTO_QR, START_QR_VISION
} qr_calibration_state;

enum CALIBRATION_STATE {
	CAMERA, TIP1, DISPENSER, TIP2, BOTTOM_CAM_QR, SLOT_QR, PCB_QR, TAPE_QR, CORRECTED_CAMERA, CORRECTED_TIP1
} calibration_state;

enum ERROR_CODE {
	MOTOR_TIMEOUT, MOTOR_FAILED, MOTOR_ERROR, TIMEOUT
} error_code;

enum STATE {
	IDLE,
	CALIBRATE,
	GOTOPCBORIGIN,
	GOTOBOX,
	FINDCOMPONENT,
	GOTOPICKUPCOOR,
	STARTPICKUP,
	GOTOBOTTOMCAM,
	CHECKCOMPONENTPICKUP,
	GOTOPCBCOMP,
	CHECKCOMPPOSITON,
	GOTOPLACECOORD,
	STARTPLACEMENT,
	GOTOCOORD,
	HOMING,
	ERROR,
	DISPENSE,
	DISPENSETASK
} state, last_state, last_qr_state;

/******************************************************************************************
 *  Main function - Implements entire placer state machine
 ******************************************************************************************/
int main(int argc, char **argv) {

	// Initialize state machine
	ros::init(argc, argv, "motorController");
	if (!ros::master::check()) {
		ROS_INFO("PlacerNode did not start...");
		return 0;
	}
	ROS_INFO("Place controller started");

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
	dispenserTaskSubscriber_ = n_.subscribe("dispenseTask", 10,
			&dispenserCallback);

	ros::Rate loop_rate(100);
	state = IDLE;
	calibration_state = SLOT_QR;
	bool outTolerance = false;

	// Run state machine forever
	while (ros::ok()) {
		switch (state) {
		case IDLE:
			timeoutValue = 500;
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
					LEDTask(pap_common::SETRINGCOLOR, 0);
					ros::Duration(0.3).sleep();
					LEDTask(pap_common::RESETBOTTOMLED, 0);

					ROS_INFO("PlacerState: CAMERA");
					placeController.currentDestination_ =
							placeController.getBottomCamCoordinates();
					ROS_INFO("Go to: x:%f y:%f z:%f",
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.currentDestination_.z);
					timeoutValue = 500;
					positionSend = true;
					last_state = state;
					state = GOTOCOORD;
				} else {

					if (!visionStarted) {
						ROS_INFO("Camera: Vision started");
						ros::Duration(3).sleep();
						counterMean = 0;
						placeController.camClibrationOffset_.x = 0.0;	//needed ??
						placeController.camClibrationOffset_.y = 0.0;	//needed ??
						LEDTask(pap_common::SETBOTTOMLED, 0);
						sendTask(pap_common::VISION, pap_vision::START_VISION);
						sendTask(pap_common::VISION, pap_vision::SEARCH_CIRCLE,
						CAMERA_DIAMETER_VISION, 0.0, 0.0);
						visionStarted = true;

					}

					if (cameraFeedbackReceived) {
						LEDTask(pap_common::RESETBOTTOMLED, 0);
						ROS_INFO("Camera: cameraOffsetreceived");
						sendTask(pap_common::VISION, pap_vision::STOP_VISION);
						//ros::Duration(1).sleep();
						//IDLE_called = true;
						//state = IDLE;
						calibration_state = CORRECTED_CAMERA;
						timeoutValue = 500;
						cameraFeedbackReceived = false;
						positionSend = false;
						visionStarted = false;
					}
				}
				break;

			case CORRECTED_CAMERA:
				if (!positionSend) {
					ROS_INFO("PlacerState: CORRECTED_CAMERA");
					placeController.currentDestination_ =
							placeController.getBottomCamCoordinates();
					ROS_INFO("Go to: x:%f y:%f z:%f",
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.currentDestination_.z);
					timeoutValue = 500;
					positionSend = true;
					last_state = state;
					state = GOTOCOORD;
				} else {
					IDLE_called = true;
					ros::Duration(5).sleep();
					//state = IDLE;
					calibration_state = TIP1;
					timeoutValue = 500;
					positionSend = false;
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
					timeoutValue = 500;
					positionSend = true;
					last_state = state;
					state = GOTOCOORD;
				} else {
					if (!visionStarted) {
						ros::Duration(1).sleep();
						sendRelaisTask(7, true);			// tip 2
						sendPlacerStatus(pap_common::INFO,
								pap_common::LEFT_TIP_DOWN);
						ros::Duration(1).sleep();
						counterMean = 0;
						placeController.dispenserCalibrationOffset_.x = 0.0;
						placeController.dispenserCalibrationOffset_.y = 0.0;
						LEDTask(pap_common::SETBOTTOMLED, 0);
						sendTask(pap_common::VISION, pap_vision::START_VISION);
						sendTask(pap_common::VISION, pap_vision::SEARCH_CIRCLE,
						TIP1_DIAMETER_VISION, 0.0, 0.0);
						visionStarted = true;

					}

					if (cameraFeedbackReceived) {
						LEDTask(pap_common::RESETBOTTOMLED, 0);
						sendTask(pap_common::VISION, pap_vision::STOP_VISION);
						calibration_state = CORRECTED_TIP1;
						timeoutValue = 500;
						cameraFeedbackReceived = false;
						positionSend = false;
						visionStarted = false;

					}
				}
				break;
			case CORRECTED_TIP1:
				if (!positionSend) {
					ROS_INFO("PlacerState: CORRECTED_TIP1");
					placeController.currentDestination_ =
							placeController.getTip1Coordinates();
					ROS_INFO("Go to: x:%f y:%f z:%f",
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.currentDestination_.z);
					timeoutValue = 500;
					positionSend = true;
					last_state = state;
					state = GOTOCOORD;
				} else {
					ros::Duration(5).sleep();
					sendRelaisTask(7, false);			// tip 2
					sendPlacerStatus(pap_common::INFO,
							pap_common::LEFT_TIP_UP);
					calibration_state = SLOT_QR;
					IDLE_called = true;
					state = IDLE;
					timeoutValue = 500;
					positionSend = false;
				}
				break;
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
					timeoutValue = 500;
					positionSend = true;
					last_state = state;
					state = GOTOCOORD;
				} else {
					if (!visionStarted) {
						counterMean = 0;
						placeController.dispenserTipOffset.x = 0;
						placeController.dispenserTipOffset.x = 0;
						ros::Duration(3).sleep();
						LEDTask(pap_common::SETBOTTOMLED, 0);
						sendTask(pap_common::VISION, pap_vision::START_VISION);
						sendTask(pap_common::VISION, pap_vision::SEARCH_CIRCLE,
						DISPENSER_DIAMETER_VISION, 0.0, 0.0);
						visionStarted = true;
					}

					if (cameraFeedbackReceived) {
						LEDTask(pap_common::RESETBOTTOMLED, 0);
						sendTask(pap_common::VISION, pap_vision::STOP_VISION);
						calibration_state = CAMERA;
						timeoutValue = 500;
						cameraFeedbackReceived = false;
						positionSend = false;
						visionStarted = false;
						IDLE_called = true;
						state = IDLE;
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
					timeoutValue = 500;
					positionSend = true;
					last_state = state;
					state = GOTOCOORD;
				} else {
					if (!visionStarted) {
						ros::Duration(3).sleep();
						LEDTask(pap_common::SETBOTTOMLED, 0);
						sendTask(pap_common::VISION, pap_vision::START_VISION);
						sendTask(pap_common::VISION, pap_vision::SEARCH_CIRCLE,
						TIP2_DIAMETER_VISION, 0.0, 0.0);
						visionStarted = true;
					}

					if (cameraFeedbackReceived) {
						LEDTask(pap_common::RESETBOTTOMLED, 0);
						sendTask(pap_common::VISION, pap_vision::STOP_VISION);
						calibration_state = CAMERA;
						timeoutValue = 500;
						cameraFeedbackReceived = false;
						positionSend = false;
						visionStarted = false;
						IDLE_called = true;
						state = IDLE;
					}
				}
				break;

			case SLOT_QR:
				if (!positionSend) {
					ROS_INFO("PlacerState: SLOT");
					placeController.currentDestination_ =
							placeController.SLOT_QR_Offset_;
					ROS_INFO("Go to: x:%f y:%f z:%f",
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.currentDestination_.z);
					timeoutValue = 500;
					positionSend = true;
					last_state = state;
					state = GOTOCOORD;
				} else {

					if (!visionStarted) {
						ROS_INFO("SLOT calibration: Vision started");
						ros::Duration(3).sleep();
						sendTask(pap_common::VISION, pap_vision::START_VISION);
						// Start QR Code Finder (Arg1: 1 = SLOT, Arg2: 0 = camera top)
						sendTask(pap_common::VISION,
								pap_vision::START__QRCODE_FINDER, 1, 0);
						visionStarted = true;
					}

					if (cameraFeedbackReceived) {
						sendTask(pap_common::VISION, pap_vision::STOP_VISION);
						//IDLE_called = true;
						//state = IDLE;
						calibration_state = PCB_QR;
						timeoutValue = 500;
						cameraFeedbackReceived = false;
						positionSend = false;
						visionStarted = false;
					}
				}
				break;

			case PCB_QR:
				if (!positionSend) {
					ROS_INFO("PlacerState: PCB_QR");
					placeController.currentDestination_ =
							placeController.PCB_QR_Offset_;
					ROS_INFO("Go to: x:%f y:%f z:%f",
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.currentDestination_.z);
					timeoutValue = 500;
					positionSend = true;
					last_state = state;
					state = GOTOCOORD;
				} else {

					if (!visionStarted) {
						ROS_INFO("PCB calibration: Vision started");
						ros::Duration(3).sleep();
						sendTask(pap_common::VISION, pap_vision::START_VISION);
						// Start QR Code Finder (Arg1: 3 = PCB, Arg2: 0 = camera top)
						sendTask(pap_common::VISION,
								pap_vision::START__QRCODE_FINDER, 3, 0);
						visionStarted = true;
					}

					if (cameraFeedbackReceived) {
						sendTask(pap_common::VISION, pap_vision::STOP_VISION);
						//IDLE_called = true;
						//state = IDLE;
						calibration_state = TAPE_QR;
						timeoutValue = 500;
						cameraFeedbackReceived = false;
						positionSend = false;
						visionStarted = false;
					}
				}
				break;

			case TAPE_QR:
				if (!positionSend) {
					ROS_INFO("PlacerState: TAPE_QR");
					placeController.currentDestination_ =
							placeController.TAPE_QR_Offset_;
					placeController.currentDestination_.z += 0.8;
					timeoutValue = 500;
					positionSend = true;
					last_state = state;
					state = GOTOCOORD;
				} else {

					if (!visionStarted) {
						ROS_INFO("Tape calibration: Vision started");
						ros::Duration(3).sleep();
						sendTask(pap_common::VISION, pap_vision::START_VISION);
						// Start QR Code Finder (Arg1: 2= TAPE, Arg2: 0 = camera top)
						sendTask(pap_common::VISION,
								pap_vision::START__QRCODE_FINDER, 2, 0);
						visionStarted = true;
					}

					if (cameraFeedbackReceived) {
						sendTask(pap_common::VISION, pap_vision::STOP_VISION);
						//IDLE_called = true;
						//state = IDLE;
						calibration_state = BOTTOM_CAM_QR;
						timeoutValue = 500;
						cameraFeedbackReceived = false;
						positionSend = false;
						visionStarted = false;
					}
				}
				break;

			case BOTTOM_CAM_QR:
				switch (qr_calibration_state) {
				case GOTO_QR:
					ROS_INFO("PlacerState: CAM_QR");
					placeController.currentDestination_ =
							placeController.BottomCam_QR_Offset_;
					timeoutValue = 500;
					last_state = state;
					state = GOTOCOORD;
					qr_calibration_state = PICKUP_QR;
					break;
				case PICKUP_QR:
					placeController.pickRelQR_ = true;
					last_qr_state = state;
					timeoutValue = 500;
					state = STARTPICKUP;
					qr_calibration_state = CAM_QR;
					break;
				case CAM_QR:
					ROS_INFO("PlacerState: CAM_QR");
					placeController.currentDestination_ =
							placeController.getTip1Coordinates();
					placeController.currentDestination_.z += 1.0;
					timeoutValue = 500;
					last_state = state;
					state = GOTOCOORD;
					qr_calibration_state = START_QR_VISION;

					break;

				case START_QR_VISION:
					ros::Duration(2.0).sleep();
					sendRelaisTask(7, true);			// tip 2
					sendPlacerStatus(pap_common::INFO,
							pap_common::LEFT_TIP_DOWN);
					ros::Duration(2.0).sleep();
					cameraFeedbackReceived = false;
					LEDTask(pap_common::SETBOTTOMLED, 0);
					sendTask(pap_common::VISION, pap_vision::START_VISION);
					// Start QR Code Finder (Arg1: 4=CAM, Arg2: 1 = camera bottom)
					sendTask(pap_common::VISION,
							pap_vision::START__QRCODE_FINDER, 4, 1);
					qr_calibration_state = GOBACKTO_QR;
					break;
				case GOBACKTO_QR:
					if (cameraFeedbackReceived) {
						sendRelaisTask(7, false);			// tip 2
						sendPlacerStatus(pap_common::INFO,
								pap_common::LEFT_TIP_UP);
						LEDTask(pap_common::RESETBOTTOMLED, 0);
						sendTask(pap_common::VISION, pap_vision::STOP_VISION);
						ROS_INFO("PlacerState: CAM_QR");
						placeController.currentDestination_ =
								placeController.BottomCam_QR_Offset_;
						timeoutValue = 500;
						last_state = state;
						state = GOTOCOORD;
						qr_calibration_state = RELEASE_QR;
						cameraFeedbackReceived = false;
					}
					break;
				case RELEASE_QR:
					placeController.pickRelQR_ = true;
					timeoutValue = 500;
					state = STARTPLACEMENT;
					calibration_state = SLOT_QR;
					qr_calibration_state = GOTO_QR;
					break;
				}

				break;
			}
			break;

		case GOTOPCBORIGIN:

			if (!positionSend) {
				placeController.currentDestination_ =
						placeController.getPCBCalibCoordinates();
				ROS_INFO("PlacerState: GOTOPCBORIGIN");
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

			// Send coordinates to motor last_statecontroller and wait until position reached
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

				if (cameraFeedbackReceived) {
					sendTask(pap_common::VISION, pap_vision::STOP_VISION);
					ROS_INFO("Got feedback from vision for chip correction...");
					cameraFeedbackReceived = false;
					positionSend = false;
					visionStarted = false;
					state = GOTOPICKUPCOOR;
				}
			}

			break;

		case GOTOPICKUPCOOR:
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
				//IDLE_called = true;
				state = STARTPICKUP; //IDLE;
			}
			break;

		case DISPENSETASK:
			if (!positionSend) {
				placeController.currentDestination_.x =
						placeController.dispenseTask.xPos;
				placeController.currentDestination_.y =
						placeController.dispenseTask.yPos;
				placeController.currentDestination_.z = DISPENSER_HEIGHT;
				positionSend = true;
				last_state = state;
				state = GOTOCOORD;
			} else if (!dispensed) {
				last_state = state;
				state = DISPENSE;
				dispensed = true;
			} else {
				dispensed = false;
				positionSend = false;
				IDLE_called = true;
				sendPlacerStatus(pap_common::INFO,
						pap_common::DISPENSER_FINISHED);
				state = IDLE;
			}
			break;

		case STARTPICKUP:
			if (!positionSend) {
				ROS_INFO("PlacerState: STARTPICKUP");
				sendPlacerStatus(pap_common::IDLE_STATE,
						pap_common::PLACER_IDLE);
				sendPlacerStatus(pap_common::STARTPICKUP_STATE,
						pap_common::PLACER_ACTIVE);

				ros::Duration(1).sleep();

				if (placeController.selectTip()) {		// Activate tip
					sendRelaisTask(3, false);			// tip 1
					sendRelaisTask(6, true);
					sendPlacerStatus(pap_common::INFO,
							pap_common::RIGHT_TIP_DOWN);
				} else {
					sendRelaisTask(7, true);			// tip 2
					sendPlacerStatus(pap_common::INFO,
							pap_common::LEFT_TIP_DOWN);
				}
				ros::Duration(1).sleep();

				placeController.currentDestination_.z =
						placeController.getCompSuckingHeight();
				ROS_INFO("PlacerState: STARTPICKUP");
				ROS_INFO("Go to: x:%f y:%f z:%f",
						placeController.currentDestination_.x,
						placeController.currentDestination_.y,
						placeController.currentDestination_.z);
				positionSend = true;
				last_state = state;
				state = GOTOCOORD;
			} else {
				sendRelaisTask(1, false);				// Turn on vacuum
				sendRelaisTask(2, true);
				if (placeController.selectTip()) {	// Turn on vacuum
					sendRelaisTask(5, true);		// Tip 1
				} else {
					sendRelaisTask(4, true);		// Tip 2
				}
				ros::Duration(1).sleep();

				if (placeController.selectTip()) {		// Release tip
					sendRelaisTask(6, false);
					sendRelaisTask(3, true);			// Tip 1
					sendPlacerStatus(pap_common::INFO,
							pap_common::RIGHT_TIP_UP);
				} else {
					sendRelaisTask(7, false);			// Tip 2
					sendPlacerStatus(pap_common::INFO, pap_common::LEFT_TIP_UP);
				}
				ros::Duration(1).sleep();
				if (!placeController.pickRelQR_) {
					int rotation =
							(int) placeController.getCompPickUpCoordinates().rot;
					sendStepperTask((placeController.selectTip() + 1),
							rotation);	// Turn component
					ROS_INFO("Placer - Rotation: rot:%d", rotation);
					ros::Duration(1).sleep();
					//IDLE_called = true;
					state = GOTOPLACECOORD; //GOTOPLACECOORD;
				} else {
					placeController.pickRelQR_ = false;
					state = last_qr_state;
				}
				positionSend = false;

			}
			break;

		case GOTOBOTTOMCAM:

			if (!positionSend) {
				placeController.currentDestination_ =
						placeController.getBottomCamCoordinates();
				ROS_INFO("PlacerState: GOTOBOTTOMCAM");
				ROS_INFO("Go to: x:%f y:%f z:%f",
						placeController.currentDestination_.x,
						placeController.currentDestination_.y,
						placeController.currentDestination_.z);
				positionSend = true;
				last_state = state;
				state = GOTOCOORD;
			} else {
				if (!visionStarted) {
					ROS_INFO("PlacerState: CHECKCOMPONENTPICKUP");
					sendTask(pap_common::VISION, pap_vision::START_VISION);
					float length = placeController.getComponentLenth();
					float width = placeController.getComponentWidth();
					/*sendTask(pap_common::VISION,						// Start appropriate vision here!
					 pap_vision::CHIP_BOTTOM, width, length,
					 0);*/
					visionStarted = true;
					ros::Duration(3).sleep();
				}

				if (cameraFeedbackReceived) {
					sendTask(pap_common::VISION, pap_vision::STOP_VISION);
					cameraFeedbackReceived = false;
					positionSend = false;
					visionStarted = false;
					IDLE_called = true;
					state = IDLE;
				}
			}
			break;

		case GOTOPLACECOORD:
			sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_IDLE);
			sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE,
					pap_common::PLACER_ACTIVE);

			if (!positionSend) {
				placeController.currentDestination_ =
						placeController.getCompPlaceCoordinates();
				ROS_INFO("PlacerState: GOTOPLACECOORD");
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

		case STARTPLACEMENT:
			ROS_INFO("PlacerState: STARTPLACEMENT");
			sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_IDLE);
			sendPlacerStatus(pap_common::STARTPLACEMET_STATE,
					pap_common::PLACER_ACTIVE);

			if (!positionSend) {
				if (placeController.selectTip()) {
					sendRelaisTask(3, false);	// Activate left cylinder
					sendRelaisTask(6, true);
					sendPlacerStatus(pap_common::INFO,
							pap_common::RIGHT_TIP_DOWN);
				} else {
					sendRelaisTask(7, true);	// Activate right cylinder
					sendPlacerStatus(pap_common::INFO,
							pap_common::LEFT_TIP_DOWN);
				}
				ros::Duration(1).sleep();

				placeController.currentDestination_.z =
						placeController.getCompPlaceHeight();
				ROS_INFO("PlacerState: GOTORELEASECOORD");
				ROS_INFO("Go to: x:%f y:%f z:%f",
						placeController.currentDestination_.x,
						placeController.currentDestination_.y,
						placeController.currentDestination_.z);
				positionSend = true;
				last_state = state;
				state = GOTOCOORD;
			} else {
				sendRelaisTask(2, false);		// Turn off vacuum
				sendRelaisTask(1, true);
				if (placeController.selectTip()) {
					sendRelaisTask(5, false);	// Tip 1
				} else {
					sendRelaisTask(4, false);	// Tip 2
				}
				ros::Duration(1).sleep();

				if (placeController.selectTip()) {		// Release tip
					sendRelaisTask(6, false);
					sendRelaisTask(3, true);			// Tip 1
					sendPlacerStatus(pap_common::INFO,
							pap_common::RIGHT_TIP_UP);
				} else {
					sendRelaisTask(7, false);			// Tip 2
					sendPlacerStatus(pap_common::INFO, pap_common::LEFT_TIP_UP);
				}
				sendPlacerStatus(pap_common::STARTPLACEMET_STATE,
						pap_common::PLACER_FINISHED);
				if (placeController.pickRelQR_) {
					placeController.pickRelQR_ = false;
				}

				positionSend = false;
				IDLE_called = true;
				state = IDLE;
			}
			break;

		case HOMING:
			sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_IDLE);
			sendPlacerStatus(pap_common::HOMING_STATE,
					pap_common::PLACER_ACTIVE);

			if (!positionSend) {
				placeController.currentDestination_ =
						placeController.idleCoordinates_;
				ROS_INFO("PlacerState: HOMING");
				ROS_INFO("Go to: x:%f y:%f z:%f",
						placeController.currentDestination_.x,
						placeController.currentDestination_.y,
						placeController.currentDestination_.z);
				positionSend = true;
				last_state = state;
				state = GOTOCOORD;
			} else {
				//endTask(pap_common::CONTROLLER, pap_common::HOMING);
				positionSend = false;
				IDLE_called = true;
				state = IDLE;
			}
			break;

		case GOTOCOORD:
			if (fabs(
					placeController.lastDestination_.x
							- placeController.currentDestination_.x)
					> (posTolerance)
					|| fabs(
							placeController.lastDestination_.y
									- placeController.currentDestination_.y)
							> (posTolerance)) {
				outTolerance = true;
			} else {
				outTolerance = false;
				//ROS_INFO("In Tolerance");
			}
			if (!placerNodeBusy) {
				//ROS_INFO("lastX: %f", placeController.lastDestination_.x);
				//ROS_INFO("lastY: %f", placeController.lastDestination_.y);
				//ROS_INFO("currentX: %f", placeController.currentDestination_.x);
				//ROS_INFO("currentY: %f", placeController.currentDestination_.y);
				if (zPosReached == 0 && outTolerance) {
					checkIfOverThreshold(3, placeController.MovingHeight_);
					sendTask(pap_common::CONTROLLER, pap_common::COORD,
							placeController.lastDestination_.x,
							placeController.lastDestination_.y,
							placeController.MovingHeight_);
					//ros::Duration(0.3).sleep();
					ROS_INFO("PlacerState: GOTOCOORD: x=%f y=%f z=%f",
							placeController.lastDestination_.x,
							placeController.lastDestination_.y,
							placeController.MovingHeight_);
					//ROS_INFO("zPos: %d", zPosReached);

				} else if (zPosReached == 1 && outTolerance) {
					checkIfOverThreshold(1, placeController.MovingHeight_);
					checkIfOverThreshold(2, placeController.MovingHeight_);
					sendTask(pap_common::CONTROLLER, pap_common::COORD,
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.MovingHeight_);
					//ros::Duration(0.3).sleep();
					ROS_INFO("PlacerState: GOTOCOORD: x=%f y=%f z=%f",
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.MovingHeight_);
					//ROS_INFO("zPos: %d", zPosReached);

				} else if (zPosReached == 2 || outTolerance) {
					checkIfOverThreshold(3,
							placeController.currentDestination_.z);
					//resetMotorState(true, true, false);
					sendTask(pap_common::CONTROLLER, pap_common::COORD,
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.currentDestination_.z);
					//ros::Duration(0.3).sleep();

					ROS_INFO("PlacerState: GOTOCOORD: x=%f y=%f z=%f",
							placeController.currentDestination_.x,
							placeController.currentDestination_.y,
							placeController.currentDestination_.z);
					zPosReached = 2;
					//ROS_INFO("zPos: %d", zPosReached);
				}
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
					if (manualGoto) {
						sendPlacerStatus(pap_common::GOTO_STATE,
								pap_common::PLACER_FINISHED);
						manualGoto = false;
					}
					//ROS_INFO("Last state: %d", last_state);
				}
				//ros::Duration(3).sleep();
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

		case DISPENSE:
			if (!placerNodeBusy) {

				// Turn on dispenser
				sendRelaisTask(8, true);
				ros::Duration(placeController.dispenseTask.time).sleep();
				float diffX = fabs(
						placeController.lastDestination_.x
								- placeController.dispenseTask.xPos2);
				float diffY = fabs(
						placeController.lastDestination_.y
								- placeController.dispenseTask.yPos2);

				if (diffX > DISPENSER_TOLERANCE) {
					resetMotorState(1, false);
				}

				if (diffY > DISPENSER_TOLERANCE) {
					resetMotorState(2, false);
				}

				sendTask(pap_common::CONTROLLER, pap_common::COORD_VEL,
						placeController.dispenseTask.xPos2,
						placeController.dispenseTask.yPos2, DISPENSER_HEIGHT,
						placeController.dispenseTask.velocity,
						placeController.dispenseTask.velocity);
				//ros::Duration(0.3).sleep();
				ROS_INFO("PlacerState: GOTOCOORD: x=%f y=%f z=%f",
						placeController.dispenseTask.xPos2,
						placeController.dispenseTask.yPos2, DISPENSER_HEIGHT);

				motorcontroller_counter = 0;
				placerNodeBusy = true;
			}

			if (motorcontrollerStatus[1].positionReached
					&& motorcontrollerStatus[2].positionReached
					&& motorcontrollerStatus[3].positionReached
					&& placerNodeBusy) {
				// Turn off dispenser
				sendRelaisTask(8, false);
				placerNodeBusy = false;
				state = last_state;
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

		case ERROR:
			// Stop and publish error code
			ROS_INFO("PlacerState: ERROR %d", error_code);
			IDLE_called = true;
			state = IDLE;
			break;
		}
		//ROS_INFO("Reached 1: %d Reached 2: %d Reached 3: %d",motorcontrollerStatus[1].positionReached,motorcontrollerStatus[2].positionReached,motorcontrollerStatus[3].positionReached);
		ros::spinOnce();
		loop_rate.sleep();
	}

	if (timeoutValue >= 0) {
		timeoutValue -= 1;
	} else {
		error_code = TIMEOUT;
		state = ERROR;
	}

	return 0;
}

/*****************************************************************************
 * General local functions - Implementation
 *****************************************************************************/
void checkIfOverThreshold(int numberOfAxis, float zValue) {
	if (numberOfAxis == 1) {
		float diffX = fabs(
				placeController.lastDestination_.x
						- placeController.currentDestination_.x);
		if (diffX > DISPENSER_TOLERANCE) {
			resetMotorState(1, false);
		}
	} else if (numberOfAxis == 2) {
		float diffY = fabs(
				placeController.lastDestination_.y
						- placeController.currentDestination_.y);
		if (diffY > DISPENSER_TOLERANCE) {
			resetMotorState(2, false);
		}
	} else if (numberOfAxis == 3) {
		float diffZ = fabs(placeController.lastDestination_.z - zValue);
		if (diffZ > DISPENSER_TOLERANCE) {
			resetMotorState(3, false);
		}
	}
}

void resetMotorState(bool x, bool y, bool z) {
	motorcontrollerStatus[1].positionReached = x;
	motorcontrollerStatus[2].positionReached = y;
	motorcontrollerStatus[3].positionReached = z;
}

void resetMotorState(int index, bool value) {
	motorcontrollerStatus[index].positionReached = value;
}

/*****************************************************************************
 * Callback functions for motor PLACECOMPONENTstatus, vision status and placer tasks
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
		//ROS_INFO("Reached %d", index);
	}

	if (statusMsg->status == pap_common::POSITIONNOTREACHED) {
		motorcontrollerStatus[index].positionReached = false;
		//ROS_INFO("NotReached");
	}

	if (statusMsg->status == pap_common::ERROR) {
		motorcontrollerStatus[index].error = true;
	}

	if (statusMsg->status == pap_common::NOERROR) {
		motorcontrollerStatus[index].error = false;
	}

	if (statusMsg->posX != 0.0) {
		placeController.lastDestination_.x = fabs(statusMsg->posX);
	}
	if (statusMsg->posY != 0.0) {
		placeController.lastDestination_.y = fabs(statusMsg->posY);
	}
	if (statusMsg->posZ != 0.0) {
		placeController.lastDestination_.z = fabs(statusMsg->posZ);
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
		//ROS_INFO("CHip finder called");

		if (!(xDiff == 0 && yDiff == 0) && !componentFound) {
			placeController.setPickUpCorrectionOffset(xDiff, yDiff, rotDiff);
			ROS_INFO("correction offset: x:%f y:%f, rot:%f", xDiff, yDiff,
					rotDiff);
			cameraFeedbackReceived = true;
		}
		break;

	case pap_vision::START_TAPE_FINDER:
		cameraFeedbackReceived = true;
		break;

	case pap_vision::START__QRCODE_FINDER:
		cameraFeedbackReceived = true;
		break;

	case pap_vision::SEARCH_CIRCLE:
		if (visionStarted && !cameraFeedbackReceived) {
			switch (calibration_state) {
			case CAMERA:

				counterMean++;
				placeController.camClibrationOffset_.x += xDiff;
				placeController.camClibrationOffset_.y += yDiff;

				if (counterMean == 50) {
					placeController.camClibrationOffset_.x /= 50;
					placeController.camClibrationOffset_.y /= 50;
					cameraFeedbackReceived = true;
					ROS_INFO("Cam Calibration Offset: X: %f Y: %f",
							placeController.camClibrationOffset_.x,
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
					cameraFeedbackReceived = true;
					ROS_INFO("Tip1 Calibration Offset: X: %f Y: %f",
							placeController.tip1ClibrationOffset_.x,
							placeController.tip1ClibrationOffset_.y);
				}
				break;
			case DISPENSER:
				counterMean++;
				placeController.dispenserCalibrationOffset_.x += xDiff;
				placeController.dispenserCalibrationOffset_.y += yDiff;

				if (counterMean == 50) {
					placeController.dispenserCalibrationOffset_.x /= 50;
					placeController.dispenserCalibrationOffset_.y /= 50;
					cameraFeedbackReceived = true;
					ROS_INFO("Dispenser Calibration Offset: X: %f Y: %f",
							placeController.dispenserCalibrationOffset_.x,
							placeController.dispenserCalibrationOffset_.y);
				}
				break;
			case TIP2:
				placeController.setTip2Offset(xDiff, yDiff);
				cameraFeedbackReceived = true;
				break;
			}

		}
		break;
	}
}

void placerCallback(const pap_common::TaskConstPtr& taskMsg) {

	switch (taskMsg->destination) {
	case pap_common::PLACER: {
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
		case pap_common::PLACECOMPONENT: {

			ComponentPlacerData tempComponent;
			tempComponent.destX = taskMsg->data1;
			tempComponent.destY = taskMsg->data2;
			tempComponent.box = taskMsg->box;
			tempComponent.height = taskMsg->height;
			tempComponent.length = taskMsg->length;
			tempComponent.width = taskMsg->width;
			tempComponent.rotation = taskMsg->data3;
			tempComponent.tapeX = taskMsg->velX;
			tempComponent.tapeY = taskMsg->velY;
			tempComponent.tapeRot = taskMsg->velZ;
			placeController.updatePlacementData(&tempComponent);

			if ((tempComponent.box >= 67) && (tempComponent.box <= 86)) {
				// Its a tape - no GOTOBOX, VISION states needed
				state = GOTOPICKUPCOOR;
				ROS_INFO("Placer - GOTOPICKUPCOOR");
			} else {
				state = GOTOBOX;	// Start placer with state GOTOBOX
				ROS_INFO("Placer - GOTOBOX");
			}
		}
			break;
		case pap_common::GOTOBOX:
			sendPlacerStatus(pap_common::GOTOBOX_STATE,
					pap_common::PLACER_IDLE);
			state = GOTOBOX;
			ROS_INFO("go to box called...");
			break;
		case pap_common::PICKUPCOMPONENT:
			sendPlacerStatus(pap_common::STARTPICKUP_STATE,
					pap_common::PLACER_IDLE);
			state = STARTPICKUP;
			ROS_INFO("Pick-up component called...");
			break;

		case pap_common::GOTOPCB:
			sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE,
					pap_common::PLACER_IDLE);
			state = GOTOPCBORIGIN;
			break;

		case pap_common::PLACEMENT:
			sendPlacerStatus(pap_common::STARTPLACEMET_STATE,
					pap_common::PLACER_IDLE);
			state = STARTPLACEMENT;
			ROS_INFO("Placement called...");
			break;

		case pap_common::CALIBRATION_OFFSET:
			state = CALIBRATE;
			calibration_state = CAMERA;

			ROS_INFO("Calibration started...");
			break;

		case pap_common::CALIBRATION_RATIO:
			state = CALIBRATE;
			calibration_state = SLOT_QR;
			ROS_INFO("Calibration started...");
			break;

		case pap_common::HOMING:
			sendPlacerStatus(pap_common::HOMING_STATE, pap_common::PLACER_IDLE);
			state = HOMING;
			break;

		case pap_common::GOTO:
			placeController.currentDestination_.x = taskMsg->data1;
			placeController.currentDestination_.y = taskMsg->data2;
			placeController.currentDestination_.z = taskMsg->data3;
			last_state = IDLE;
			state = GOTOCOORD;
			manualGoto = true;
			break;
		}
	}
}

void dispenserCallback(const pap_common::DispenseTaskConstPtr& taskMsg) {
	placeController.dispenseTask.xPos = taskMsg->xPos1;
	//+ placeController.dispenserTipOffset.x
	//- placeController.camClibrationOffset_.x
	//+ placeController.dispenserCalibrationOffset_.x;
	placeController.dispenseTask.xPos2 = taskMsg->xPos2;
	//+ placeController.dispenserTipOffset.x
	//- placeController.camClibrationOffset_.x
	//+ placeController.dispenserCalibrationOffset_.x;
	placeController.dispenseTask.yPos = taskMsg->yPos1;
	//+ placeController.dispenserTipOffset.y
	//- placeController.camClibrationOffset_.y
	//+ placeController.dispenserCalibrationOffset_.y;
	placeController.dispenseTask.yPos2 = taskMsg->yPos2;
	//+ placeController.dispenserTipOffset.y
	//- placeController.camClibrationOffset_.y
	//+ placeController.dispenserCalibrationOffset_.y;
	placeController.dispenseTask.velocity = taskMsg->velocity;
	placeController.dispenseTask.time = taskMsg->waitTime;
	state = DISPENSETASK;
	//ROS_INFO("Dispensing...");
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

void sendTask(pap_common::DESTINATION destination, pap_common::TASK task,
		float x, float y, float z, float velx, float vely) {
	pap_common::Task taskMsg;
	taskMsg.destination = destination;
	taskMsg.task = task;
	taskMsg.data1 = x;
	taskMsg.data2 = y;
	taskMsg.data3 = z;
	taskMsg.velX = velx;
	taskMsg.velY = vely;
	task_publisher.publish(taskMsg);
}

// QR Code type: 1=SLOT, 2=TAPE, 3=PCB, 4=BottomCam
void sendTask(pap_common::DESTINATION destination, pap_vision::VISION task,
		int type, int camera) {
	pap_common::Task taskMsg;
	taskMsg.destination = destination;
	taskMsg.task = task;
	taskMsg.data1 = type;
	taskMsg.data2 = camera;
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
	if (StepperNumber == 2) {
		arduinoMsg.command = pap_common::RUNSTEPPER1;
		arduinoMsg.data = rotationAngle;
		arduino_publisher_.publish(arduinoMsg);
	}
	if (StepperNumber == 1) {
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

void LEDTask(int task, int data) {
	pap_common::ArduinoMsg arduinoMsg;
	arduinoMsg.command = task;
	arduinoMsg.data = data;
	arduino_publisher_.publish(arduinoMsg);
}
