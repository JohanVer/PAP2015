#ifndef PLACER_NODE_H_
#define PLACER_NODE_H_

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
#define CAMERA_DIAMETER_VISION 190.0
#define DISPENSER_HEIGHT 27

enum QR_CALIBRATION_PROCESS {
    GOTO_QR, CAM_QR
};

enum ERROR_CODE {
    MOTOR_TIMEOUT, MOTOR_FAILED, MOTOR_ERROR, TIMEOUT, VISION_ERROR
};

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
};

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
};

/* Call back functions */
void statusCallback(const pap_common::StatusConstPtr& statusMsg);
void visionStatusCallback(const pap_common::VisionStatusConstPtr& statusMsg);
void placerCallback(const pap_common::TaskConstPtr& taskMsg);
void dispenserCallbackPlacer(const pap_common::DispenseTaskConstPtr& taskMsg);

void sendPlacerStatus(pap_common::PROCESS process,
                      pap_common::PLACER_STATUS status);
void sendPlacerInfo(int state);


// Placer functions
bool pickUp(double height);
bool placeComp(double height);

bool calibrateOffsets();
bool calibrateCamera();
bool calibrateTip1();
bool calibrateDispenser();
bool calibrateTip2();

bool calibrateTopCamDistortion();
bool calibrateBottomCamDistortion();

bool singleCompPlacement();
bool goToBox();
bool pickUpComponent();
bool checkCompPickUp();
bool placeComponent();

bool dispensePCB();
bool goToPCBOrigin();

/* General local functions */
void switchDispenser(bool activate);
void switchVacuum(bool activate);
void forwardVacuum(enum TIP tip_select, bool activate);
void moveTip(enum TIP tip_select, bool down);
bool driveAroundPosition(Offset position, int distance_x, int distance_y);
bool driveToCoord(const double &x, const double &y, const double &z);
void processAllStatusCallbacks();

#endif // PLACER_NODE_H_
