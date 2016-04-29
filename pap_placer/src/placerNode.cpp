#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pap_common/Task.h"
#include "pap_common/Status.h"
#include "pap_common/VisionStatus.h"
#include "pap_common/ArduinoMsg.h"
#include "pap_common/PlacerStatus.h"
#include "pap_common/DispenseTask.h"
#include <pap_common/task_message_def.h>
#include <pap_common/status_message_def.h>
#include <pap_common/arduino_message_def.h>
#include <pap_common/vision_message_def.h>
#include <pap_common/placer_message_def.h>

#include <pap_placer/placerNode.hpp>
#include <pap_placer/placerClass.hpp>

#include <motorController/controller_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <pap_common/MotorControllerActionAction.h>
#include <motorController/sendfunctions.h>

#include <pap_common/VisionAction.h>
#include <pcb_cv/sendfunctions.h>

/* Constant parameter definitions */
#define posTolerance 0.01 // Deviation of position in mm
#define DISPENSER_TOLERANCE 0.1
#define MOTORCONTROLLER_TIMEOUT 3000
#define TIP1_DIAMETER_VISION 20.0 //130
#define TIP2_DIAMETER_VISION 20.0
#define DISPENSER_DIAMETER_VISION 130
#define CAMERA_DIAMETER_VISION 210.0//165.0
#define DISPENSER_HEIGHT 22.2 //12,2

//typedef actionlib::SimpleActionClient<pap_common::MotorControllerActionAction> Client;
//typedef actionlib::SimpleActionClient<pap_common::VisionAction> Client;     // NEW

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
bool driveAroundPosition(Offset position, int distance);

ros::Publisher task_publisher, arduino_publisher_, placerStatus_publisher_;
ros::Subscriber statusSubsriber_;
ros::Subscriber visionStatusSubsriber_;
ros::Subscriber placerTaskSubscriber_;
ros::Subscriber dispenserTaskSubscriber_;

//std::unique_ptr<Client> action_client;
std::unique_ptr<motor_send_functions::Client> motor_action_client;
std::unique_ptr<vision_send_functions::Client> vision_action_client;


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
bool completePlacement = false;

PlaceController placeController;
ComponentPlacerData currentComponent;
static motor_controller::controllerStatus motorcontrollerStatus[2];

enum QR_CALIBRATION_PROCESS {
    GOTO_QR, PICKUP_QR, CAM_QR, RELEASE_QR, GOBACKTO_QR, START_QR_VISION
} qr_calibration_state;

enum CALIBRATION_STATE {
    CAMERA,
    CORRECTED_CAMERA,
    TIP1,
    CORRECTED_TIP1,
    DISPENSER,
    CORRECTED_DISPENSER,
    TIP2,
    CORRECTED_TIP2,
    BOTTOM_CAM_QR,
    SLOT_QR,
    PCB_QR,
    TAPE_QR,
    CHECKERBOARD
} calibration_state;

enum ERROR_CODE {
    MOTOR_TIMEOUT, MOTOR_FAILED, MOTOR_ERROR, TIMEOUT, VISION_ERROR
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
    PLACECOMPONENT,
    GOTOCOORD,
    HOMING,
    ERROR,
    DISPENSE,
    DISPENSETASK
} state, last_state, last_qr_state;

/******************************************************************************************
 *  Main function - Implements entire placer state machine
 ******************************************************************************************/

bool driveToCoord(const double &x, const double &y, const double &z){
    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                  placeController.lastDestination_.x,
                                  placeController.lastDestination_.y,
                                  placeController.MovingHeight_)){
        return false;
    }

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                  x,
                                  y,
                                  placeController.MovingHeight_)){
        return false;
    }

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                  x,
                                  y,
                                  z)){
        return false;
    }

    return true;
}

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

   // motor_action_client = std::unique_ptr<Client>(new Client("motor_controller_actions", true));
    motor_action_client = std::unique_ptr<motor_send_functions::Client>(new motor_send_functions::Client("motor_controller_actions", true));
    vision_action_client = std::unique_ptr<vision_send_functions::Client>(new vision_send_functions::Client("vision_actions", true));

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
                sendPlacerStatus(pap_common::IDLE_STATE,
                                 pap_common::PLACER_ACTIVE);
                // Keep state indicators to see error state
                ROS_INFO("Last state: %d", last_state);
                if (last_state != ERROR) {
                    sendPlacerStatus(pap_common::GOTOBOX_STATE,
                                     pap_common::PLACER_IDLE);
                    sendPlacerStatus(pap_common::STARTPICKUP_STATE,
                                     pap_common::PLACER_IDLE);
                    sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE,
                                     pap_common::PLACER_IDLE);
                    sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                                     pap_common::PLACER_IDLE);
                    sendPlacerStatus(pap_common::HOMING_STATE,
                                     pap_common::PLACER_IDLE);
                }
                IDLE_called = false;
            }
            break;

        case CALIBRATE:

            switch (calibration_state) {
            case CAMERA: {

                ROS_INFO("PlacerState: CAMERA");
                sendPlacerStatus(pap_common::IDLE_STATE,
                                 pap_common::PLACER_IDLE);
                sendPlacerStatus(pap_common::CALIBRATION_STATE,
                                 pap_common::PLACER_ACTIVE);

                LEDTask(pap_common::SETRINGCOLOR, 0);
                ros::Duration(0.3).sleep();
                LEDTask(pap_common::RESETBOTTOMLED, 0);

                Offset bottomCam = placeController.getBottomCamCoordinates();
                ROS_INFO("Go to: x:%f y:%f z:%f", bottomCam.x, bottomCam.y, bottomCam.z);

                if(!driveToCoord(bottomCam.x, bottomCam.y, bottomCam.z)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                ros::Duration(1).sleep();
                LEDTask(pap_common::SETBOTTOMLED, 0);

                ROS_INFO("Placerstate: CAMERA - Start Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }

                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, CAMERA_DIAMETER_VISION, 0.0, 0.0, res, 50)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }
                ROS_INFO("Placerstate: CAMERA - cameraOffset received");
                placeController.updateCameraBottomOffset(res.data2, res.data1);
                /*      res.data1 = -tip.y;
                        res.data2 = tip.x;
                        res.data3 = tip.rot;
                        res.cameraSelect = cameraSelect;    */

                LEDTask(pap_common::RESETBOTTOMLED, 0);
                ROS_INFO("Placerstate: CAMERA - Stop Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::STOP_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }
                calibration_state = CORRECTED_CAMERA;
                break;
            }

            case CORRECTED_CAMERA: {

                ROS_INFO("PlacerState: CORRECTED_CAMERA");
                Offset bottomCam = placeController.getBottomCamCoordinates();
                ROS_INFO("Go to: x:%f y:%f z:%f", bottomCam.x, bottomCam.y, bottomCam.z);

                if(!driveToCoord(bottomCam.x, bottomCam.y, bottomCam.z)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                ros::Duration(5).sleep();
                calibration_state = TIP1;
                break;
            }

            case TIP1: {

                ROS_INFO("PlacerState: TIP1 ");
                Offset tip1 = placeController.getTip1Coordinates();
                ROS_INFO("Go to: x:%f y:%f z:%f", tip1.x, tip1.y, tip1.z);

                if(!driveToCoord(tip1.x, tip1.y, tip1.z)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                ros::Duration(1).sleep();
                sendRelaisTask(7, true);			// tip 2
                sendPlacerStatus(pap_common::INFO, pap_common::LEFT_TIP_DOWN);
                ros::Duration(1).sleep();

                ROS_INFO("Placerstate: TIP1 - Start Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }

                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, TIP1_DIAMETER_VISION, 0.0, 0.0, res, 50)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }
                ROS_INFO("Placerstate: TIP1 - cameraOffset received");
                placeController.updateTip1Offset(res.data2, res.data1);

                LEDTask(pap_common::RESETBOTTOMLED, 0);
                ROS_INFO("Placerstate: TIP1 - Stop Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::STOP_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }

                calibration_state = CORRECTED_TIP1;
                break;
            }

            case CORRECTED_TIP1: {

                ROS_INFO("PlacerState: CORRECTED_TIP1");
                Offset tip1 = placeController.getTip1Coordinates();
                ROS_INFO("Go to: x:%f y:%f z:%f", tip1.x, tip1.y, tip1.z);

                if(!driveToCoord(tip1.x, tip1.y, tip1.z)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                ros::Duration(5).sleep();
                sendRelaisTask(7, false);   // Left tip
                sendPlacerStatus(pap_common::INFO, pap_common::LEFT_TIP_UP);

                calibration_state = CAMERA;
                IDLE_called = true;
                state = IDLE;
                break;
            }

            case DISPENSER: {

                ROS_INFO("PlacerState: DISPENSER ");
                Offset dispenser = placeController.getDispenserCoordinates();
                ROS_INFO("Go to: x:%f y:%f z:%f", dispenser.x, dispenser.y, dispenser.z);

                if(!driveToCoord(dispenser.x, dispenser.y, dispenser.z)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                ros::Duration(3).sleep();
                LEDTask(pap_common::SETBOTTOMLED, 0);

                ROS_INFO("Placerstate: DISPENSER - Start Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }

                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, DISPENSER_DIAMETER_VISION, 0.0, 0.0, res, 50)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }
                ROS_INFO("Placerstate: DISPENSER - cameraOffset received");
                placeController.updatedispenserTipOffset(res.data2, res.data1);

                LEDTask(pap_common::RESETBOTTOMLED, 0);
                ROS_INFO("Placerstate: DISPENSER - Stop Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::STOP_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }

                calibration_state = CORRECTED_DISPENSER;
                break;
            }

            case CORRECTED_DISPENSER: {

                ROS_INFO("PlacerState: CORRECTED_DISPENSER");
                Offset dispenser = placeController.getDispenserCoordinates();
                ROS_INFO("Go to: x:%f y:%f z:%f", dispenser.x, dispenser.y, dispenser.z);

                if(!driveToCoord(dispenser.x, dispenser.y, dispenser.z)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                ros::Duration(5).sleep();
                IDLE_called = true;
                state = IDLE;
                break;
            }

            case TIP2: {

                ROS_INFO("PlacerState: TIP2 ");
                Offset tip2 = placeController.getTip2Coordinates();
                ROS_INFO("Go to: x:%f y:%f z:%f", tip2.x, tip2.y, tip2.z);

                if(!driveToCoord(tip2.x, tip2.y, tip2.z)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                ros::Duration(1).sleep();
                // TODO: Activate right tip
                //sendRelaisTask(7, true);
                sendPlacerStatus(pap_common::INFO, pap_common::RIGHT_TIP_DOWN);
                ros::Duration(1).sleep();
                LEDTask(pap_common::SETBOTTOMLED, 0);

                ROS_INFO("Placerstate: TIP2 - Start Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }

                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, TIP2_DIAMETER_VISION, 0.0, 0.0, res, 50)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }
                ROS_INFO("Placerstate: TIP2 - cameraOffset received");
                placeController.updateTip2Offset(res.data2, res.data1);

                LEDTask(pap_common::RESETBOTTOMLED, 0);
                ROS_INFO("Placerstate: TIP2 - Stop Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::STOP_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }

                calibration_state = CAMERA;
                IDLE_called = true;
                state = IDLE;
                break;
            }

            case CORRECTED_TIP2: {

                ROS_INFO("PlacerState: CORRECTED_TIP2");
                Offset tip2 = placeController.getTip2Coordinates();
                ROS_INFO("Go to: x:%f y:%f z:%f", tip2.x, tip2.y, tip2.z);

                if(!driveToCoord(tip2.x, tip2.y, tip2.z)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                ros::Duration(5).sleep();
                // TODO: Activate corresponding tip!
                //sendRelaisTask(7, false);
                //sendPlacerStatus(pap_common::INFO, pap_common::LEFT_TIP_UP);

                IDLE_called = true;
                state = IDLE;
                break;
            }

            case CHECKERBOARD: {

                ROS_INFO("PlacerState: CHECKERBOAR 1");
                Offset checker = placeController.CHECKERBOARD_1_Offset_;
                ROS_INFO("Go to: x:%f y:%f z:%f", checker.x, checker.y, checker.z);
                if(!driveAroundPosition(checker, 10)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                ROS_INFO("PlacerState: CHECKERBOAR 2");
                checker = placeController.CHECKERBOARD_2_Offset_;
                ROS_INFO("Go to: x:%f y:%f z:%f", checker.x, checker.y, checker.z);
                if(!driveAroundPosition(checker, 10)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                calibration_state = CAMERA;
                IDLE_called = true;
                state = IDLE;
                break;
            }

            case SLOT_QR: {

                ROS_INFO("PlacerState: SLOT_QR");
                Offset slot_qr = placeController.SLOT_QR_Offset_;
                ROS_INFO("Go to: x:%f y:%f z:%f", slot_qr.x, slot_qr.y, slot_qr.z);

                if(!driveToCoord(slot_qr.x, slot_qr.y, slot_qr.z)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                ros::Duration(3).sleep();
                ROS_INFO("PlacerState: SLOT_QR - Vision started");

                // TODO: Reset slot_ratio !!!!???
                ROS_INFO("Placerstate: SLOT_QR - Start Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }

                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START__QRCODE_FINDER, pap_vision::TOP_SLOT, pap_vision::CAMERA_TOP, res, 2)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                } else {
                    ROS_INFO("Placerstate: SLOT_QR - cameraFeedback received");
                    // TODO: Handle res?
                }

                ROS_INFO("Placerstate: SLOT_QR - Stop Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::STOP_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }
                calibration_state = PCB_QR;
                break;
            }

            case PCB_QR: {

                ROS_INFO("PlacerState: PCB_QR");
                Offset pcb_qr = placeController.PCB_QR_Offset_;
                ROS_INFO("Go to: x:%f y:%f z:%f", pcb_qr.x, pcb_qr.y, pcb_qr.z);

                if(!driveToCoord(pcb_qr.x, pcb_qr.y, pcb_qr.z)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                ros::Duration(3).sleep();
                ROS_INFO("PlacerState: PCB_QR - Vision started");

                // TODO: Reset slot_ratio !!!!???
                ROS_INFO("Placerstate: PCB_QR - Start Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }

                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START__QRCODE_FINDER, pap_vision::TOP_PCB, pap_vision::CAMERA_TOP, res, 2)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                } else {
                    ROS_INFO("Placerstate: PCB_QR - cameraFeedback received");
                    // TODO: Handle res?
                }

                ROS_INFO("Placerstate: PCB_QR - Stop Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::STOP_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }

                calibration_state = TAPE_QR;
                break;
            }

            case TAPE_QR: {

                ROS_INFO("PlacerState: TAPE_QR");
                Offset tape_qr = placeController.TAPE_QR_Offset_;
                // TODO: check!
                tape_qr.z += 0.8;
                ROS_INFO("Go to: x:%f y:%f z:%f", tape_qr.x, tape_qr.y, tape_qr.z);

                if(!driveToCoord(tape_qr.x, tape_qr.y, tape_qr.z)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                ros::Duration(3).sleep();
                ROS_INFO("PlacerState: TAPE_QR - Vision started");

                // TODO: Reset slot_ratio !!!!???
                ROS_INFO("Placerstate: TAPE_QR - Start Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }

                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START__QRCODE_FINDER, pap_vision::TOP_TAPE, pap_vision::CAMERA_TOP, res, 2)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                } else {
                    ROS_INFO("Placerstate: TAPE_QR - cameraFeedback received");
                    // TODO: Handle res?
                }

                ROS_INFO("Placerstate: TAPE_QR - Stop Vision");
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::STOP_VISION)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }

                calibration_state = BOTTOM_CAM_QR;
                break;
            }

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
                    ros::Duration(1).sleep();
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
                    state = PLACECOMPONENT;
                    calibration_state = SLOT_QR;
                    qr_calibration_state = GOTO_QR;
                    break;
                }

                break;
            }
            break;

        case GOTOPCBORIGIN: {
            ROS_INFO("PlacerState: GOTOPCBORIGIN");
            Offset pcbCoords = placeController.getPCBCalibCoordinates();
            ROS_INFO("Go to: x:%f y:%f z:%f", pcbCoords.x, pcbCoords.y, pcbCoords.z);

            if(!driveToCoord(pcbCoords.x, pcbCoords.y, pcbCoords.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }

            IDLE_called = true;
            state = IDLE;
            break;
        }

        case GOTOBOX: {

            ROS_INFO("Placerstate: GOTOBOX");
            sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_IDLE);
            sendPlacerStatus(pap_common::GOTOBOX_STATE, pap_common::PLACER_ACTIVE);

            setLEDTask(placeController.getBoxNumber());
            Offset boxCoords = placeController.getBoxCoordinates();
            ROS_INFO("Go to: x:%f y:%f z:%f", boxCoords.x, boxCoords.y, boxCoords.z);

            if(!driveToCoord(boxCoords.x, boxCoords.y, boxCoords.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }

            ROS_INFO("Placerstate: GOTOBOX - Vision started");
            vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START_VISION);

            float length = placeController.currentComponent.length;
            float width = placeController.currentComponent.width;
            float height = placeController.currentComponent.height;
            pap_common::VisionResult result;

            ROS_INFO("Placer: componentFinder started");
            if(!vision_send_functions::sendVisionTask(*vision_action_client, placeController.finderType,
                                          pap_vision::CAMERA_TOP, length, width, height, result)) {
                error_code = VISION_ERROR;
                state = ERROR;
                break;
            }

            ros::Duration(3).sleep();

            ROS_INFO("Placerstate: GOTOBOX - Got feedback from vision for chip correction");
            vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::STOP_VISION);

            state = GOTOPICKUPCOOR;
            break;
        }

        case GOTOPICKUPCOOR: {

            ROS_INFO("Placerstate: GOTOPICKUPCOOR");
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
        }
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
                state = IDLE;
            }
            break;

        case STARTPICKUP:
            if (!positionSend) {
                ROS_INFO("PlacerState: STARTPICKUP");
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
                    //sendStepperTask((placeController.selectTip() + 1),
                    //		rotation);	// Turn component
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
            if (!positionSend) {
                ROS_INFO("PlacerState: GOTOPLACECOORD");
                sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE,
                                 pap_common::PLACER_ACTIVE);
                placeController.currentDestination_ =
                        placeController.getCompPlaceCoordinates();
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
                //state = IDLE;
                state = PLACECOMPONENT;
            }
            break;

        case PLACECOMPONENT:
            if (!positionSend) {
                ROS_INFO("PlacerState: PLACECOMPONENT");
                sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                                 pap_common::PLACER_ACTIVE);

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
                ros::Duration(1).sleep();
                positionSend = false;
                if (placeController.pickRelQR_) {
                    placeController.pickRelQR_ = false;
                }

                // Distinguish single/complete placement
                if (completePlacement) {
                    // Go to idle and wait for next component infos
                    IDLE_called = true;
                    state = IDLE;
                } else {
                    state = HOMING;
                }

                // Indicates placement finished & if complPlacement gui send new data and restarts process
                sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                                 pap_common::PLACER_FINISHED);
            }
            break;

        case HOMING:

            sendPlacerStatus(pap_common::IDLE_STATE,
                             pap_common::PLACER_IDLE);
            sendPlacerStatus(pap_common::GOTOBOX_STATE,
                             pap_common::PLACER_IDLE);
            sendPlacerStatus(pap_common::STARTPICKUP_STATE,
                             pap_common::PLACER_IDLE);
            sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE,
                             pap_common::PLACER_IDLE);
            sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                             pap_common::PLACER_IDLE);
            sendPlacerStatus(pap_common::HOMING_STATE,
                             pap_common::PLACER_ACTIVE);

            // Make sure tip is in upper position again!
            // TIP 1
            sendRelaisTask(6, false);
            sendRelaisTask(3, true);			// Tip 1
            sendPlacerStatus(pap_common::INFO,
                             pap_common::RIGHT_TIP_UP);
            // TIP 2
            sendRelaisTask(7, false);			// Tip 2
            sendPlacerStatus(pap_common::INFO, pap_common::LEFT_TIP_UP);
            ros::Duration(0.5).sleep();

            if(!driveToCoord(placeController.idleCoordinates_.x, placeController.idleCoordinates_.y, placeController.idleCoordinates_.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }

            motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::HOMING);
            IDLE_called = true;
            state = IDLE;

            break;

        case GOTOCOORD:


            if(!driveToCoord(placeController.idleCoordinates_.x, placeController.idleCoordinates_.y, placeController.idleCoordinates_.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }


            if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                          placeController.lastDestination_.x,
                                          placeController.lastDestination_.y,
                                          placeController.MovingHeight_)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }

            if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                          placeController.currentDestination_.x,
                                          placeController.currentDestination_.y,
                                          placeController.MovingHeight_)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }

            if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                          placeController.currentDestination_.x,
                                          placeController.currentDestination_.y,
                                          placeController.currentDestination_.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }

            state = last_state;
            placerNodeBusy = false;

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

            if (motorcontrollerStatus[0].positionReached
                    && motorcontrollerStatus[1].positionReached
                    && motorcontrollerStatus[2].positionReached
                    && placerNodeBusy) {
                // Turn off dispenser
                sendRelaisTask(8, false);
                placerNodeBusy = false;
                state = last_state;
                break;

            } else if (motorcontrollerStatus[0].error
                       || motorcontrollerStatus[1].error
                       || motorcontrollerStatus[2].error) {
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;

            } else if (motorcontrollerStatus[0].failed
                       || motorcontrollerStatus[1].failed
                       || motorcontrollerStatus[2].failed) {
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
            switch (last_state) {
            case GOTOBOX:
                sendPlacerStatus(pap_common::GOTOBOX_STATE,
                                 pap_common::PLACER_ERROR);
                break;
            case STARTPICKUP:
                sendPlacerStatus(pap_common::STARTPICKUP_STATE,
                                 pap_common::PLACER_ERROR);
                break;
            case GOTOPLACECOORD:
                sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE,
                                 pap_common::PLACER_ERROR);
                break;
            case PLACECOMPONENT:
                sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                                 pap_common::PLACER_ERROR);
                break;
            case HOMING:
                sendPlacerStatus(pap_common::HOMING_STATE,
                                 pap_common::PLACER_ERROR);
                break;
            }
            last_state = state;
            IDLE_called = true;
            state = IDLE;
            break;
        }
        //ROS_INFO("Reached 1: %d Reached 2: %d Reached 3: %d",motorcontrollerStatus[0].positionReached,motorcontrollerStatus[1].positionReached,motorcontrollerStatus[2].positionReached);
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
bool driveAroundPosition(Offset position, int distance) {

    if(!driveToCoord(position.x, position.y, position.z)){
        return false;
    }

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                  (position.x - (distance/2)), position.y, position.z)){
        return false;
    }

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                  (position.x - (distance/2)), (position.y - (distance/2)), position.z)){
        return false;
    }

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                  (position.x + (distance/2)), (position.y - (distance/2)), position.z)){
        return false;
    }

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                  (position.x + (distance/2)), (position.y + (distance/2)), position.z)){
        return false;
    }

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                  (position.x - (distance/2)), (position.y + (distance/2)), position.z)){
        return false;
    }

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                  (position.x - (distance/2)), (position.y - (distance/2)), position.z)){
        return false;
    }
    return true;
}


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
    motorcontrollerStatus[0].positionReached = x;
    motorcontrollerStatus[1].positionReached = y;
    motorcontrollerStatus[2].positionReached = z;
}

void resetMotorState(int index, bool value) {
    motorcontrollerStatus[index].positionReached = value;
}

/*****************************************************************************
 * Callback functions for motor PLACECOMPONENTstatus, vision status and placer tasks
 *****************************************************************************/
void statusCallback(const pap_common::StatusConstPtr& statusMsg) {

    motorcontrollerStatus[0].energized = statusMsg->energized[0];
    motorcontrollerStatus[1].energized = statusMsg->energized[1];
    motorcontrollerStatus[2].energized = statusMsg->energized[2];

    motorcontrollerStatus[0].error = statusMsg->error[0];
    motorcontrollerStatus[1].error = statusMsg->error[1];
    motorcontrollerStatus[2].error = statusMsg->error[2];



    motorcontrollerStatus[0].positionReached = statusMsg->reached[0];
    motorcontrollerStatus[1].positionReached = statusMsg->reached[1];
    motorcontrollerStatus[2].positionReached = statusMsg->reached[2];

    motorcontrollerStatus[0].position = statusMsg->pos[0];
    motorcontrollerStatus[1].position = statusMsg->pos[1];
    motorcontrollerStatus[2].position = statusMsg->pos[2];


    if (statusMsg->pos[0] != 0.0) {
        placeController.lastDestination_.x = fabs(statusMsg->pos[0]);
    }
    if (statusMsg->pos[1] != 0.0) {
        placeController.lastDestination_.y = fabs(statusMsg->pos[1]);
    }
    if (statusMsg->pos[2] != 0.0) {
        placeController.lastDestination_.z = fabs(statusMsg->pos[2]);
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

        if (taskMsg->task == pap_common::SINGLEPLACEMENT
                || taskMsg->task == pap_common::COMPLETEPLACEMENT
                || taskMsg->task == pap_common::PLACECOMPONENT) {

            ComponentPlacerData tempComponent;
            tempComponent.destX = taskMsg->data1;
            tempComponent.destY = taskMsg->data2;
            tempComponent.rotation = taskMsg->data3;
            tempComponent.box = taskMsg->box;
            tempComponent.height = taskMsg->height;
            tempComponent.length = taskMsg->length;
            tempComponent.width = taskMsg->width;
            tempComponent.tapeX = taskMsg->velX;
            tempComponent.tapeY = taskMsg->velY;
            tempComponent.tapeRot = taskMsg->velZ;
            placeController.updatePlacementData(&tempComponent);
            if ((tempComponent.box >= 67) && (tempComponent.box <= 86)) {
                // Its a tape - no GOTOBOX, VISION states needed
                state = GOTOPICKUPCOOR;
            } else {
                state = GOTOBOX;
            }
        }

        switch (taskMsg->task) {

        // Need to make sure placement data is set correctly!!!
        case pap_common::SINGLEPLACEMENT: {
            ROS_INFO("Placer: SinglePlacement called.");
            completePlacement = false;
        }
            break;

        case pap_common::COMPLETEPLACEMENT: {
            ROS_INFO("Placer: CompletePlacement called.");
            completePlacement = true;
        }
            break;

        case pap_common::IDLE: {
            ROS_INFO("Placer: Idle called.");
            IDLE_called = true;
            state = IDLE;
        }
            break;
        case pap_common::STOP: {
            ROS_INFO("Placer: Stop called.");
            completePlacement = false;
        }
            break;

        case pap_common::PLACECOMPONENT: {
            ROS_INFO("Placer: PlaceComponent called.");
            completePlacement = false;
        }
            break;

        case pap_common::GOTOBOX: {
            ROS_INFO("Placer: gotobox called.");
            sendPlacerStatus(pap_common::GOTOBOX_STATE,
                             pap_common::PLACER_IDLE);
            state = GOTOBOX;
        }
            break;

        case pap_common::PICKUPCOMPONENT: {
            state = STARTPICKUP;
            ROS_INFO("Pick-up component called...");
        }
            break;

        case pap_common::GOTOPCB: {
            ROS_INFO("Placer: gotoPCB called.");
            sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE,
                             pap_common::PLACER_IDLE);
            state = GOTOPCBORIGIN;
        }
            break;

        case pap_common::PLACEMENT:
            ROS_INFO("Placer: Placement called.");
            sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                             pap_common::PLACER_IDLE);
            state = PLACECOMPONENT;
            break;

        case pap_common::CALIBRATION_OFFSET:
            ROS_INFO("Placer: Calibration_offset called.");
            state = CALIBRATE;
            calibration_state = CAMERA;
            break;

        case pap_common::CALIBRATION_RATIO:
            ROS_INFO("Placer: Calibration_ratio called.");
            state = CALIBRATE;
            calibration_state = SLOT_QR;
            break;

        case pap_common::HOMING:
            ROS_INFO("Placer: Homing called.");
            completePlacement = false;
            positionSend = false;
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

        case pap_common::PRINT_OFFSET:
            ROS_INFO("Placer: PrintOffset");
            Offset temp;
            temp = placeController.getBottomCamCoordinates();
            ROS_INFO("BottomCam: x=%f, y=%f, z=%f", temp.x, temp.y, temp.z);
            temp = placeController.getTip1Coordinates();
            ROS_INFO("Tip1Coord: x=%f, y=%f, z=%f", temp.x, temp.y, temp.z);
            temp = placeController.getTip2Coordinates();
            ROS_INFO("Tip2Coord: x=%f, y=%f, z=%f", temp.x, temp.y, temp.z);
            temp = placeController.getDispenserCoordinates();
            ROS_INFO("DispCoord: x=%f, y=%f, z=%f", temp.x, temp.y, temp.z);
            break;

        case pap_common::CALIBRATION_CHECKERBOARD:
            ROS_INFO("Placer: CALIBRATION_CHECKERBOARD");
            state = CALIBRATE;
            calibration_state = CHECKERBOARD;
            break;
        }
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

// Normal Messages

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
