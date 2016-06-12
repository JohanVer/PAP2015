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
#include <pap_placer/placerClass.hpp>
#include <motorController/controller_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <pap_common/MotorControllerActionAction.h>
#include <motorController/sendfunctions.h>
#include <pap_common/VisionAction.h>
#include <pcb_cv/sendfunctions.h>
#include <pap_common/arduinosendfunctions.h>
#include <pap_common/CommonDataClasses.hpp>

#define DISPENSER_TOLERANCE 0.1
#define MOTORCONTROLLER_TIMEOUT 3000
#define TIP1_DIAMETER_VISION 20.0 //130
#define TIP2_DIAMETER_VISION 20.0
#define DISPENSER_DIAMETER_VISION 17
#define CAMERA_DIAMETER_VISION 190.0
#define DISPENSER_HEIGHT 10.302
#define DISPENSER_CONN_SPEED 20


//!
//! \brief statusCallback processes feedback from motor controller
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
//! \param status placer status within corresponding process
//
void sendPlacerStatus(pap_common::PROCESS process, pap_common::PLACER_STATUS status);


//!
//! \brief singleCompPlacement performs an entire placement process with one component
//! \return true if all placement steps executed successfully, otherwise false
//!
bool singleCompPlacement();

//!
//! \brief multipleCompPlacement performs an entire placement process with both tips
//! \return true if all placement steps executed successfully, otherwise false
//!
bool multipleCompPlacement();

//!
//! \brief goToBox moves placement head to selected component box, starts vision to find the
//! corresponding component in that box and updates the pick-up correction offset
//! \param usedTip selected left or right tip component
//! \return true if all steps executed successfully, otherwise false
//!
bool goToBox(TIP usedTip);

//!
//! \brief pickUp performs a component pick-up task by moving to corrected
//! component position and taking care of all pneumatic and vacuum tasks necessary
//! \param height determines pick-up height, height to extend active cylinder
//! \param usedTip selects left or right tip - cylinder/valves to activate
//! \return true if all steps executed successfully, otherwise false
//!
bool pickUp(double height, TIP usedTip);

//!
//! \brief placeComp performs a placement task by moving to the component placement
//! position (PCB) and taking care of all pneumatic and vacuum tasks necessary
//! \param height determines placement height, height to extend active cylinder
//! \param usedTip selects left or right tip - cylinder/valves to activate
//! \return
//!
bool placeComp(double height, TIP usedTip);

//!
//! \brief goToPCBOrigin moves placement head to PCB
//! \return true if all steps executed successfully, otherwise false
//!
bool goToPCBOrigin();

//!
//! \brief dispensePCB starts dispensing solder paste onto current PCB
//! \return true if all steps executed successfully, otherwise false
//!
bool dispensePCB();

//!
//! \brief calibrateOffsets starts offset calibration procedure of both tips and
//! dispenser tip relative to top camera
//! \return true if calibration successfull, otherwise false
//!
bool calibrateOffsets();

//!
//! \brief calibrateCamera performs
//! \return
//!
bool calibrateCamera();

//!
//! \brief calibrateTip1 calibrates relative offset between top camera and tip1, based on bottom cam
//! \return true if calibration successfull, otherwise false
//!
bool calibrateTip1();

//!
//! \brief calibrateDispenser calibrates relative offset between top camera and dispenser tip, based on bottom cam
//! \return true if calibration successfull, otherwise false
//!
bool calibrateDispenser();

//!
//! \brief calibrateTip2 calibrates relative offset between top camera and tip2, based on bottom cam
//! \return true if calibration successfull, otherwise false
//!
bool calibrateTip2();

//!
//!//! \brief calibrateTopCamDistortion performs a checkerboard calibration of top camera
//!           !! calibTopCamera-Node needs to be started as well !!
//!//! \return true if calibration successfull, otherwise false
//!
bool calibrateTopCamDistortion();

//!
//! \brief calibrateBottomCamDistortion performs a checkerboard calibration of bottom camera
//!           !! calibBottomCamera-Node needs to be started as well !!
//! \return true if calibration successfull, otherwise false
//!
bool calibrateBottomCamDistortion();

//!
//! \brief switchDispenser sets dispenser valve state
//! \param activate - value state is set to
//!
void switchDispenser(bool activate);

//!
//! \brief switchVacuum sets vacuum ejector state
//! \param activate - value state is set to
//!
void switchVacuum(bool activate);

//!
//! \brief forwardVacuum sets selected vacuum valve state
//! \param tip_select selectes left or right tip vacuum valve
//! \param activate - value state is set to
//!
void forwardVacuum(enum TIP tip_select, bool activate);

//!
//! \brief moveTip activates/deactivates selected cylinder
//! \param tip_select selected left or right tip cylinder
//! \param down - state the cylinder is set to
//!
void moveTip(enum TIP tip_select, bool down);

//!
//! \brief driveAroundPosition moves placement head along rectangular trajectory around position
//! \param position center of the rectangular trajectory
//! \param distance_x max trajectory distance in x direction
//! \param distance_y max trajectory distance in y direction
//! \return true if all movement steps performed successfully, otherwise false
//!
bool driveAroundPosition(Offset position, int distance_x, int distance_y);

//!
//! \brief driveToCoord moves placement head to a desired position
//! \param x coordinate of destination in x direction
//! \param y coordinate of destination in y direction
//! \param z coordinate of destination in z direction
//! \return true if all movement steps performed successfully, otherwise false
//!
bool driveToCoord(const double &x, const double &y, const double &z);

//!
//! \brief processAllStatusCallbacks
//!
void processAllStatusCallbacks();

//!
//! \brief dispensePCB starts dispensing solder paste onto current PCB
//! \param dispense_task vector of separate dispender tasks, performed sequentially
//! \param dispense_height sets desired dispenser needle height
//! \return
//!
bool dispensePCB(std::vector<dispenseInfo> dispense_task, double dispense_height);

#endif // PLACER_NODE_H_
