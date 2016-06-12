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
#include <pap_common/DispenseTasks.h>
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
#include <pap_common/CommonDataClasses.hpp>

#include <eigen3/Eigen/Eigen>

/* Constant parameter definitions */
#define DISPENSER_TOLERANCE 0.1
#define MOTORCONTROLLER_TIMEOUT 3000
#define TIP1_DIAMETER_VISION 1.0
#define TIP2_DIAMETER_VISION 1.0
#define DISPENSER_DIAMETER_VISION 0.8
#define CAMERA_DIAMETER_VISION 9.3596
#define DISPENSER_HEIGHT 10.302
#define DISPENSER_CONN_SPEED 20

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

//!
//! \brief statusCallback process feedback from motor controller
//! \param statusMsg received status message
//!
void statusCallback(const pap_common::StatusConstPtr& statusMsg);

//!
//! \brief visionStatusCallback processes feedback from vision
//! \param statusMsg received status message
//!
void visionStatusCallback(const pap_common::VisionStatusConstPtr& statusMsg);

//!
//! \brief placerCallback handles new placement commands
//! \param taskMsg new placement controller task
//!
void placerCallback(const pap_common::TaskConstPtr& taskMsg);

//!
//! \brief dispenserCallbackPlacer handles new dispensing commands
//! \param taskMsg new dispenser command
//!
void dispenserCallbackPlacer(const pap_common::DispenseTasksConstPtr &taskMsg);

//!
//! \brief sendPlacerStatus broadcasts current placer status
//! \param process process the placer want to update
//! \param status placer status within this process
//
void sendPlacerStatus(pap_common::PROCESS process, pap_common::PLACER_STATUS status);


// Placer functions
bool singleCompPlacement();
bool multipleCompPlacement();
bool goToBox(TIP usedTip);
bool pickUp(double height, TIP usedTip);
bool placeComp(double height, TIP usedTip);
bool goToPCBOrigin();

bool dispenseDots(std::vector<dispenseInfo>  &dots, double dispense_height);
bool dispenseLines(std::vector<dispenseInfo>  &lines, double dispense_height);
bool dispensePCB();

//!
//! \brief calibrateOffsets starts offset calibration procedure of both tips and
//! dispenser tip relative to top camera
//! \return true if calibration succesfull, otherwise false
//!
bool calibrateOffsets();


//!
//! \brief calibrateCamera
//! \return
//!
bool calibrateCamera();

//!
//! \brief calibrateTip1
//! \return
//!
bool calibrateTip1();

//!
//! \brief calibrateDispenser
//! \param diameter
//! \return
//!
bool calibrateDispenser(double diameter);

//!
//! \brief calibrateTip2
//! \return
//!
bool calibrateTip2();

//!
//!//! \brief calibrateTopCamDistortion
//!//! \return
//!
bool calibrateTopCamDistortion();

//!
//! \brief calibrateBottomCamDistortion
//! \return
//!
bool calibrateBottomCamDistortion();


/* General local functions */
void switchDispenser(bool activate);
void switchVacuum(bool activate);
void forwardVacuum(enum TIP tip_select, bool activate);
void moveTip(enum TIP tip_select, bool down);
bool driveAroundPosition(Offset position, int distance_x, int distance_y);
bool driveToCoord(const double &x, const double &y, const double &z, const double moving_height = 45.0);
void processAllStatusCallbacks();

bool dispensePCB(std::vector<dispenseInfo> dispense_task, double dispense_height);

#endif // PLACER_NODE_H_
