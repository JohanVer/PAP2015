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
#include <pap_common/arduinosendfunctions.h>

/* Constant parameter definitions */
#define DISPENSER_TOLERANCE 0.1
#define MOTORCONTROLLER_TIMEOUT 3000
#define TIP1_DIAMETER_VISION 20.0 //130
#define TIP2_DIAMETER_VISION 20.0
#define DISPENSER_DIAMETER_VISION 130
#define CAMERA_DIAMETER_VISION 210.0//165.0
#define DISPENSER_HEIGHT 22.2 //12,2

/* Call back functions */
void statusCallback(const pap_common::StatusConstPtr& statusMsg);
void visionStatusCallback(const pap_common::VisionStatusConstPtr& statusMsg);
void placerCallback(const pap_common::TaskConstPtr& taskMsg);
void dispenserCallback(const pap_common::DispenseTaskConstPtr& taskMsg);

void sendPlacerStatus(pap_common::PROCESS process,
                      pap_common::PLACER_STATUS status);

void sendPlacerInfo(int state);

/* General local functions */
void switchDispenser(bool activate);
void switchVacuum(bool activate);
void forwardVacuum(enum TIP tip_select, bool activate);
void moveTip(enum TIP tip_select, bool down);
bool driveAroundPosition(Offset position, int distance);
bool driveToCoord(const double &x, const double &y, const double &z);

ros::Publisher task_publisher, arduino_publisher_, placerStatus_publisher_;
ros::Subscriber statusSubsriber_;
ros::Subscriber visionStatusSubsriber_;
ros::Subscriber placerTaskSubscriber_;
ros::Subscriber dispenserTaskSubscriber_;

//std::unique_ptr<Client> action_client;
std::unique_ptr<motor_send_functions::Client> motor_action_client;
std::unique_ptr<vision_send_functions::Client> vision_action_client;
std::unique_ptr<arduino_send_functions::ArduinoSender> arduino_client;

/* Variable definitions and initializations*/
bool pickUpCompleted = false;
bool dispensed = false;
bool IDLE_called = true;
bool completePlacement = false;

PlaceController placeController;
ComponentPlacerData currentComponent;
static motor_controller::controllerStatus motorcontrollerStatus[2];

enum QR_CALIBRATION_PROCESS {
    GOTO_QR, CAM_QR
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
    placerStatus_publisher_ = n_.advertise<pap_common::PlacerStatus>("placerStatus", 1000);
    statusSubsriber_ = n_.subscribe("status", 100, &statusCallback);
    placerTaskSubscriber_ = n_.subscribe("task", 10, &placerCallback);
    dispenserTaskSubscriber_ = n_.subscribe("dispenseTask", 10, &dispenserCallback);

    // motor_action_client = std::unique_ptr<Client>(new Client("motor_controller_actions", true));
    motor_action_client = std::unique_ptr<motor_send_functions::Client>(new motor_send_functions::Client("motor_controller_actions", true));
    vision_action_client = std::unique_ptr<vision_send_functions::Client>(new vision_send_functions::Client("vision_actions", true));
    arduino_client = std::unique_ptr<arduino_send_functions::ArduinoSender>(new arduino_send_functions::ArduinoSender(n_));

    ros::Rate loop_rate(100);
    state = IDLE;
    calibration_state = SLOT_QR;

    // Run state machine forever
    while (ros::ok()) {
        switch (state) {
        case IDLE: {
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
        }

        case CALIBRATE: {

            switch (calibration_state) {
            case CAMERA: {

                ROS_INFO("PlacerState: CAMERA");
                sendPlacerStatus(pap_common::IDLE_STATE,
                                 pap_common::PLACER_IDLE);
                sendPlacerStatus(pap_common::CALIBRATION_STATE,
                                 pap_common::PLACER_ACTIVE);

                arduino_client->LEDTask(pap_common::SETRINGCOLOR, 0);
                ros::Duration(0.3).sleep();
                arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

                Offset bottomCam = placeController.getBottomCamCoordinates();
                ROS_INFO("Go to: x:%f y:%f z:%f", bottomCam.x, bottomCam.y, bottomCam.z);

                if(!driveToCoord(bottomCam.x, bottomCam.y, bottomCam.z)){
                    error_code = MOTOR_ERROR;
                    state = ERROR;
                    break;
                }

                ros::Duration(1).sleep();
                arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);

                ROS_INFO("Placerstate: CAMERA - Start Vision");
                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, CAMERA_DIAMETER_VISION, 0.0, 0.0, res, 50)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }
                ROS_INFO("Placerstate: CAMERA - cameraOffset received");
                placeController.updateCameraBottomOffset(res.data1, res.data2);

                ROS_INFO("Placerstate: CAMERA - Stop Vision");
                arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);
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
                moveTip(TIP::LEFT_TIP, true);
                ros::Duration(1).sleep();

                ROS_INFO("Placerstate: TIP1 - Start Vision");

                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, TIP1_DIAMETER_VISION, 0.0, 0.0, res, 50)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }
                ROS_INFO("Placerstate: TIP1 - cameraOffset received");
                placeController.updateTip1Offset(res.data2, res.data1);

                arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

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
                moveTip(TIP::LEFT_TIP, false);

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
                arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);

                ROS_INFO("Placerstate: DISPENSER - Start Vision");
                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, DISPENSER_DIAMETER_VISION, 0.0, 0.0, res, 50)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }
                ROS_INFO("Placerstate: DISPENSER - cameraOffset received");
                placeController.updatedispenserTipOffset(res.data2, res.data1);

                arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

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
                moveTip(TIP::RIGHT_TIP, true);
                ros::Duration(1).sleep();
                arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);

                ROS_INFO("Placerstate: TIP2 - Start Vision");

                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, TIP2_DIAMETER_VISION, 0.0, 0.0, res, 50)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }
                ROS_INFO("Placerstate: TIP2 - cameraOffset received");
                placeController.updateTip2Offset(res.data2, res.data1);

                arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

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
                moveTip(TIP::RIGHT_TIP, false);

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
                ROS_INFO("Placerstate: SLOT_QR - Start Vision");

                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START__QRCODE_FINDER, pap_vision::TOP_SLOT, pap_vision::CAMERA_TOP, res, 100)) {
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
                ROS_INFO("Placerstate: PCB_QR - Start Vision");

                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START__QRCODE_FINDER, pap_vision::TOP_PCB, pap_vision::CAMERA_TOP, res, 100)) {
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
                ROS_INFO("Placerstate: TAPE_QR - Start Vision");

                pap_common::VisionResult res;
                if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START__QRCODE_FINDER, pap_vision::TOP_TAPE, pap_vision::CAMERA_TOP, res, 100)) {
                    error_code = VISION_ERROR;
                    state = ERROR;
                    break;
                }

                calibration_state = BOTTOM_CAM_QR;
                break;
            }

            case BOTTOM_CAM_QR: {
                switch (qr_calibration_state) {
                case GOTO_QR: {
                    ROS_INFO("PlacerState: GOTO_QR");
                    Offset gotoOffset = placeController.BottomCam_QR_Offset_;
                    ROS_INFO("Go to: x:%f y:%f z:%f", gotoOffset.x, gotoOffset.y, gotoOffset.z);

                    if(!driveToCoord(gotoOffset.x, gotoOffset.y, gotoOffset.z)){
                        error_code = MOTOR_ERROR;
                        state = ERROR;
                        break;
                    }

                    placeController.pickRelQR_ = true;
                    last_qr_state = state;
                    state = STARTPICKUP;
                    qr_calibration_state = CAM_QR;
                }
                    break;

                case CAM_QR: {
                    ROS_INFO("PlacerState: CAM_QR");
                    Offset QROffset = placeController.getTip1Coordinates();
                    QROffset.z += 1.0;
                    ROS_INFO("Go to: x:%f y:%f z:%f", QROffset.x, QROffset.y, QROffset.z);

                    if(!driveToCoord(QROffset.x, QROffset.y, QROffset.z)){
                        error_code = MOTOR_ERROR;
                        state = ERROR;
                        break;
                    }

                    ros::Duration(2.0).sleep();
                    moveTip(TIP::LEFT_TIP, true);
                    ros::Duration(1).sleep();
                    arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);

                    ROS_INFO("Placerstate: CAM_QR - Start Vision");
                    pap_common::VisionResult res;
                    if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START__QRCODE_FINDER, pap_vision::BOTTOM_CAM, pap_vision::CAMERA_BOTTOM, res, 100)) {
                        error_code = VISION_ERROR;
                        state = ERROR;
                        break;
                    }

                    ROS_INFO("Placerstate: PCB_QR - cameraFeedback received");
                    // TODO: Handle res?

                    moveTip(TIP::LEFT_TIP, false);
                    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

                    QROffset = placeController.BottomCam_QR_Offset_;
                    ROS_INFO("Go to: x:%f y:%f z:%f", QROffset.x, QROffset.y, QROffset.z);
                    if(!driveToCoord(QROffset.x, QROffset.y, QROffset.z)){
                        error_code = MOTOR_ERROR;
                        state = ERROR;
                        break;
                    }

                    placeController.pickRelQR_ = true;
                    state = PLACECOMPONENT;
                    qr_calibration_state = GOTO_QR;
                }
                    break;
                }

            }
                break;
            }
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
        }
            break;

        case GOTOBOX: {

            ROS_INFO("Placerstate: GOTOBOX");
            sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_IDLE);
            sendPlacerStatus(pap_common::GOTOBOX_STATE, pap_common::PLACER_ACTIVE);

            arduino_client->setLEDTask(placeController.getBoxNumber());
            Offset boxCoords = placeController.getBoxCoordinates();
            ROS_INFO("Go to: x:%f y:%f z:%f", boxCoords.x, boxCoords.y, boxCoords.z);

            if(!driveToCoord(boxCoords.x, boxCoords.y, boxCoords.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }

            ROS_INFO("Placerstate: GOTOBOX - Vision started");

            float length = placeController.currentComponent.length;
            float width = placeController.currentComponent.width;
            float height = placeController.currentComponent.height;
            pap_common::VisionResult res;

            ROS_INFO("Placer: componentFinder started");
            if(!vision_send_functions::sendVisionTask(*vision_action_client, placeController.finderType,
                                                      pap_vision::CAMERA_TOP, length, width, height, res)) {
                error_code = VISION_ERROR;
                state = ERROR;
                break;
            }

            placeController.setPickUpCorrectionOffset(res.data2, res.data2, res.data3);
            ROS_INFO("PickUp correction offset: x:%f y:%f, rot:%f",res.data2, res.data2, res.data3);

            ros::Duration(3).sleep();
            ROS_INFO("Placerstate: GOTOBOX - Got feedback from vision");

            state = GOTOPICKUPCOOR;
            break;
        }

        case GOTOPICKUPCOOR: {

            ROS_INFO("Placerstate: GOTOPICKUPCOOR");
            Offset pickupCoord = placeController.getCompPickUpCoordinates();
            ROS_INFO("Go to: x:%f y:%f z:%f", pickupCoord.x, pickupCoord.y, pickupCoord.z);
            if(!driveToCoord(pickupCoord.x, pickupCoord.y, pickupCoord.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }
            state = STARTPICKUP;
            break;
        }

        case STARTPICKUP: {

            ROS_INFO("PlacerState: STARTPICKUP");
            sendPlacerStatus(pap_common::STARTPICKUP_STATE,
                             pap_common::PLACER_ACTIVE);
            ros::Duration(1).sleep();

            if (placeController.getTip()) {		// Activate tip
                moveTip(TIP::RIGHT_TIP, true);
            } else {
                moveTip(TIP::LEFT_TIP, true);
            }
            ros::Duration(1).sleep();

            Offset suckCoord = placeController.getCompPickUpCoordinates();
            suckCoord.z = placeController.getCompSuckingHeight();
            ROS_INFO("Go to: x:%f y:%f z:%f", suckCoord.x, suckCoord.y, suckCoord.z);

            if(!driveToCoord(suckCoord.x, suckCoord.y, suckCoord.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }

            switchVacuum(true);

            if (placeController.getTip()) {
                forwardVacuum(TIP::RIGHT_TIP, true);
            } else {
                forwardVacuum(TIP::LEFT_TIP, true);
            }

            ros::Duration(1).sleep();

            if (placeController.getTip()) {		// Release tip
                moveTip(TIP::RIGHT_TIP, false);
            } else {
                moveTip(TIP::LEFT_TIP, false);
            }
            ros::Duration(1).sleep();

            if (!placeController.pickRelQR_) {
                int rotation = (int) placeController.getCompPickUpCoordinates().rot;
                //sendStepperTask((placeController.getTip() + 1), rotation);	// Turn component
                ROS_INFO("Placer - Rotation: rot:%d", rotation);
                ros::Duration(1).sleep();
                state = GOTOPLACECOORD;
            } else {
                placeController.pickRelQR_ = false;
                state = last_qr_state;
            }
            break;
        }

        case GOTOBOTTOMCAM: {

            ROS_INFO("PlacerState: GOTOBOTTOMCAM");
            Offset botCam = placeController.getBottomCamCoordinates();
            ROS_INFO("Go to: x:%f y:%f z:%f", botCam.x, botCam.y, botCam.z);

            if(!driveToCoord(botCam.x, botCam.y, botCam.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }

            ROS_INFO("Placerstate: GOTOBOTTOMCAM - Vision started");
            float length = placeController.getComponentLenth();
            float width = placeController.getComponentWidth();

            /*sendTask(pap_common::VISION,						// Start appropriate vision here!
             pap_vision::CHIP_BOTTOM, width, length,
             0);*/

            ROS_INFO("Placerstate: GOTOBOTTOMCAM - Got feedback from vision");
            state = GOTOPLACECOORD;
            break;
        }

        case GOTOPLACECOORD: {

            ROS_INFO("Placerstate: GOTOPLACECOORD");
            sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE, pap_common::PLACER_ACTIVE);
            Offset placeCoord = placeController.getCompPlaceCoordinates();
            ROS_INFO("Go to: x:%f y:%f z:%f", placeCoord.x, placeCoord.y, placeCoord.z);
            if(!driveToCoord(placeCoord.x, placeCoord.y, placeCoord.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }
            state = PLACECOMPONENT;
            break;
        }

        case PLACECOMPONENT: {

            ROS_INFO("PlacerState: PLACECOMPONENT");
            sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                             pap_common::PLACER_ACTIVE);

            if (placeController.getTip()) {  // Activate cylinder
                moveTip(TIP::RIGHT_TIP, true);
            } else {
                moveTip(TIP::LEFT_TIP, true);
            }
            ros::Duration(1).sleep();

            Offset placeCoord = placeController.getCompPlaceCoordinates();
            placeCoord.z = placeController.getCompPlaceHeight();
            ROS_INFO("Go to: x:%f y:%f z:%f", placeCoord.x, placeCoord.y, placeCoord.z);

            if(!driveToCoord(placeCoord.x, placeCoord.y, placeCoord.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }

            switchVacuum(false);

            if (placeController.getTip()) {
                forwardVacuum(TIP::RIGHT_TIP, false);
            } else {
                forwardVacuum(TIP::LEFT_TIP, false);
            }
            ros::Duration(1).sleep();

            if (placeController.getTip()) {		// Release tip
                moveTip(TIP::RIGHT_TIP, false);
            } else {
                moveTip(TIP::LEFT_TIP, false);
            }

            ros::Duration(1).sleep();

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

            // Indicates placement finished & if complPlacement gui sends new data and restarts process
            sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                             pap_common::PLACER_FINISHED);
            break;
        }

        case HOMING: {

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
            moveTip(TIP::RIGHT_TIP, false);
            moveTip(TIP::LEFT_TIP, false);

            if(!driveToCoord(placeController.idleCoordinates_.x, placeController.idleCoordinates_.y, placeController.idleCoordinates_.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }

            motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::HOMING);
            IDLE_called = true;
            state = IDLE;
            break;
        }

        case GOTOCOORD: {

            ROS_INFO("Go to: x:%f y:%f z:%f", placeController.currentDestination_.x, placeController.currentDestination_.y, placeController.currentDestination_.z);
            if(!driveToCoord(placeController.currentDestination_.x, placeController.currentDestination_.y, placeController.currentDestination_.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }
            state = last_state;
            break;
        }

        case DISPENSETASK: {

            Offset dispCoord;
            dispCoord.x = placeController.dispenseTask.xPos;
            dispCoord.y = placeController.dispenseTask.yPos;
            dispCoord.z = DISPENSER_HEIGHT;
            ROS_INFO("Go to: x:%f y:%f z:%f", dispCoord.x, dispCoord.y, dispCoord.z);
            if(!driveToCoord(dispCoord.x, dispCoord.y, dispCoord.z)){
                error_code = MOTOR_ERROR;
                state = ERROR;
                break;
            }

            state = DISPENSE;
            break;
        }

        case DISPENSE: {

            // Turn on dispenser
            switchDispenser(true);
            ros::Duration(placeController.dispenseTask.time).sleep();

            ROS_INFO("PlacerState: GOTOCOORD: x=%f y=%f z=%f",
                     placeController.dispenseTask.xPos2,
                     placeController.dispenseTask.yPos2, DISPENSER_HEIGHT);

            // TODO: Check

            //sendTask(pap_common::CONTROLLER, pap_common::COORD_VEL,
            //         placeController.dispenseTask.xPos2,
            //         placeController.dispenseTask.yPos2, DISPENSER_HEIGHT,
            //         placeController.dispenseTask.velocity,
            //         placeController.dispenseTask.velocity);

            // Turn off dispenser
            switchDispenser(false);
            state = HOMING; //(last_state;)
            break;
        }

        case ERROR: {
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
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

/*****************************************************************************
 * General local functions - Implementation
 *****************************************************************************/
void moveTip(enum TIP tip_select, bool down){
    switch(tip_select){
    case TIP::LEFT_TIP:
        if(down){
            arduino_client->sendRelaisTask(7, true);
            sendPlacerStatus(pap_common::INFO,
                             pap_common::LEFT_TIP_DOWN);
        }else
        {
            arduino_client->sendRelaisTask(7, false);
            sendPlacerStatus(pap_common::INFO, pap_common::LEFT_TIP_UP);
        }
        break;
    case TIP::RIGHT_TIP:
        if(down){
            arduino_client->sendRelaisTask(3, false);
            arduino_client->sendRelaisTask(6, true);
            sendPlacerStatus(pap_common::INFO,
                             pap_common::RIGHT_TIP_DOWN);
        }else
        {
            arduino_client->sendRelaisTask(6, false);
            arduino_client->sendRelaisTask(3, true);
            sendPlacerStatus(pap_common::INFO,
                             pap_common::RIGHT_TIP_UP);
        }
        break;
    }
}

void switchVacuum(bool activate){
    if(activate){
        arduino_client->sendRelaisTask(1, false);			// Turn on vacuum
        arduino_client->sendRelaisTask(2, true);
    }else
    {
        arduino_client->sendRelaisTask(2, false);		// Turn off vacuum
        arduino_client->sendRelaisTask(1, true);
    }
}

void forwardVacuum(enum TIP tip_select, bool activate){
    switch(tip_select){
    case TIP::LEFT_TIP:
        if(activate){
            arduino_client->sendRelaisTask(4, true);
        }
        else{
            arduino_client->sendRelaisTask(4, false);
        }
        break;

    case TIP::RIGHT_TIP:
        if(activate){
            arduino_client->sendRelaisTask(5, true);
        }else{
            arduino_client->sendRelaisTask(5, false);
        }
        break;
    }
}

void switchDispenser(bool activate){
    if(activate){
        arduino_client->sendRelaisTask(8, true);
    }else{
        arduino_client->sendRelaisTask(8, false);
    }
}

bool driveAroundPosition(Offset position, int distance) {

    if(!driveToCoord(position.x, position.y, position.z)){
        return false;
    }

    ros::Duration(1).sleep();
    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        (position.x - (distance/2)), position.y, position.z)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        (position.x - (distance/2)), (position.y - (distance/2)), position.z)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        (position.x + (distance/2)), (position.y - (distance/2)), position.z)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        (position.x + (distance/2)), (position.y + (distance/2)), position.z)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        (position.x - (distance/2)), (position.y + (distance/2)), position.z)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        (position.x - (distance/2)), (position.y - (distance/2)), position.z)){
        return false;
    }
    ros::Duration(1).sleep();
    return true;
}

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

/*****************************************************************************
 * Callback functions - Implementation
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

        case pap_common::IDLE: {
            ROS_INFO("Placer: Idle called.");
            IDLE_called = true;
            state = IDLE;
            break;
        }

        case pap_common::STOP: {
            ROS_INFO("Placer: Stop called.");
            completePlacement = false;
            break;
        }

        case pap_common::CALIBRATION_OFFSET: {
            ROS_INFO("Placer: CALIBRATION_OFFSET");
            state = CALIBRATE;
            calibration_state = CAMERA;
            break;
        }

        case pap_common::CALIBRATION_RATIO: {
            ROS_INFO("Placer: CALIBRATION_RATIO");
            state = CALIBRATE;
            calibration_state = SLOT_QR;
            break;
        }

        case pap_common::CALIBRATION_CHECKERBOARD: {
            ROS_INFO("Placer: CALIBRATION_CHECKERBOARD");
            state = CALIBRATE;
            calibration_state = CHECKERBOARD;
            break;
        }

        case pap_common::PLACECOMPONENT: {
            ROS_INFO("Placer: PlaceComponent called.");
            completePlacement = false;
            break;
        }

        case pap_common::SINGLEPLACEMENT: {
            ROS_INFO("Placer: SinglePlacement called.");
            completePlacement = false;
            break;
        }

        case pap_common::COMPLETEPLACEMENT: {
            ROS_INFO("Placer: CompletePlacement called.");
            completePlacement = true;
            break;
        }

        case pap_common::GOTOBOX: {
            ROS_INFO("Placer: gotobox called.");
            sendPlacerStatus(pap_common::GOTOBOX_STATE,
                             pap_common::PLACER_IDLE);
            state = GOTOBOX;
            break;
        }

        case pap_common::PICKUPCOMPONENT: {
            state = STARTPICKUP;
            ROS_INFO("Pick-up component called...");
            break;
        }

        case pap_common::GOTOPCB: {
            ROS_INFO("Placer: gotoPCB called.");
            sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE,
                             pap_common::PLACER_IDLE);
            state = GOTOPCBORIGIN;
            break;
        }

        case pap_common::PLACEMENT: {
            ROS_INFO("Placer: Placement called.");
            sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                             pap_common::PLACER_IDLE);
            state = PLACECOMPONENT;
            break;
        }

        case pap_common::HOMING: {
            ROS_INFO("Placer: Homing called.");
            completePlacement = false;
            state = HOMING;
            break;
        }

        case pap_common::GOTO: {
            placeController.currentDestination_.x = taskMsg->data1;
            placeController.currentDestination_.y = taskMsg->data2;
            placeController.currentDestination_.z = taskMsg->data3;
            last_state = IDLE;
            state = GOTOCOORD;
            break;
        }

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


