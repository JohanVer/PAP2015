#include <pap_placer/placerNode.hpp>

ros::Publisher task_publisher, arduino_publisher_, placerStatus_publisher_;
ros::Subscriber statusSubsriber_;
ros::Subscriber visionStatusSubsriber_;
ros::Subscriber placerTaskSubscriber_;
ros::Subscriber dispenserTaskSubscriber_;

//std::unique_ptr<Client> action_client;
std::unique_ptr<motor_send_functions::Client> motor_action_client;
std::unique_ptr<vision_send_functions::Client> vision_action_client;
std::unique_ptr<arduino_send_functions::ArduinoSender> arduino_client;

PlaceController placeController;
static motor_controller::controllerStatus motorcontrollerStatus[2];

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
    placerStatus_publisher_ = n_.advertise<pap_common::PlacerStatus>("placerStatus", 100);
    statusSubsriber_ = n_.subscribe("status", 100, &statusCallback);
    placerTaskSubscriber_ = n_.subscribe("task", 10, &placerCallback);
    dispenserTaskSubscriber_ = n_.subscribe("/dispenseTask", 100, &dispenserCallbackPlacer);

    // motor_action_client = std::unique_ptr<Client>(new Client("motor_controller_actions", true));
    motor_action_client = std::unique_ptr<motor_send_functions::Client>(new motor_send_functions::Client("motor_controller_actions", true));
    vision_action_client = std::unique_ptr<vision_send_functions::Client>(new vision_send_functions::Client("vision_actions", true));
    arduino_client = std::unique_ptr<arduino_send_functions::ArduinoSender>(new arduino_send_functions::ArduinoSender(n_));

    ros::Rate loop_rate(100);

    // Run node forever
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



bool homeSystem() {

    ROS_INFO("Placer: System is homing");
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

    if(!driveToCoord(placeController.idleCoordinates_.x, placeController.idleCoordinates_.y, placeController.idleCoordinates_.z))
        return false;

    motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::HOMING);
    sendPlacerStatus(pap_common::HOMING_STATE,
                     pap_common::PLACER_IDLE);
    return true;
}


// Offset calibration methods
bool calibrateOffsets() {
    if(!calibrateCamera())
        return false;

    if(!calibrateTip1())
        return false;

    if(!calibrateDispenser())
        return false;

    if(!calibrateTip2())
        return false;

    return true;
}

bool calibrateCamera() {
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

    if(!driveToCoord(bottomCam.x, bottomCam.y, bottomCam.z))
        return false;

    ros::Duration(1).sleep();
    ROS_INFO("Placerstate: CAMERA - Start Vision");
    arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);

    pap_common::VisionResult res;
    if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, CAMERA_DIAMETER_VISION, 0.0, 0.0, res, 50))
        return false;

    ROS_INFO("Placerstate: CAMERA - cameraOffset received");
    placeController.updateCameraBottomOffset(res.data1, res.data2);
    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

    ROS_INFO("PlacerState: CORRECTED_CAMERA");
    bottomCam = placeController.getBottomCamCoordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", bottomCam.x, bottomCam.y, bottomCam.z);

    if(!driveToCoord(bottomCam.x, bottomCam.y, bottomCam.z))
        return false;

    ros::Duration(5).sleep();
    return true;
}

bool calibrateTip1() {
    ROS_INFO("PlacerState: TIP1 ");
    Offset tip1 = placeController.getTip1Coordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", tip1.x, tip1.y, tip1.z);

    if(!driveToCoord(tip1.x, tip1.y, tip1.z))
        return false;

    ros::Duration(1).sleep();
    moveTip(TIP::LEFT_TIP, true);
    ros::Duration(1).sleep();

    ROS_INFO("Placerstate: TIP1 - Start Vision");

    pap_common::VisionResult res;
    if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, TIP1_DIAMETER_VISION, 0.0, 0.0, res, 50))
        return true;

    ROS_INFO("Placerstate: TIP1 - cameraOffset received");
    placeController.updateTip1Offset(res.data2, res.data1);

    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

    ROS_INFO("PlacerState: CORRECTED_TIP1");
    tip1 = placeController.getTip1Coordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", tip1.x, tip1.y, tip1.z);

    if(!driveToCoord(tip1.x, tip1.y, tip1.z))
        return false;

    ros::Duration(5).sleep();
    moveTip(TIP::LEFT_TIP, false);
    return true;
}

bool calibrateDispenser() {
    ROS_INFO("PlacerState: DISPENSER ");
    Offset dispenser = placeController.getDispenserCoordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", dispenser.x, dispenser.y, dispenser.z);

    if(!driveToCoord(dispenser.x, dispenser.y, dispenser.z))
        return false;

    ros::Duration(3).sleep();
    arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);

    ROS_INFO("Placerstate: DISPENSER - Start Vision");
    pap_common::VisionResult res;
    if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, DISPENSER_DIAMETER_VISION, 0.0, 0.0, res, 50))
        return false;

    ROS_INFO("Placerstate: DISPENSER - cameraOffset received");
    placeController.updatedispenserTipOffset(res.data2, res.data1);

    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

    ROS_INFO("PlacerState: CORRECTED_DISPENSER");
    dispenser = placeController.getDispenserCoordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", dispenser.x, dispenser.y, dispenser.z);

    if(!driveToCoord(dispenser.x, dispenser.y, dispenser.z))
        return false;

    ros::Duration(5).sleep();
    return true;
}

bool calibrateTip2() {
    ROS_INFO("PlacerState: TIP2 ");
    Offset tip2 = placeController.getTip2Coordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", tip2.x, tip2.y, tip2.z);

    if(!driveToCoord(tip2.x, tip2.y, tip2.z))
        return false;

    ros::Duration(1).sleep();
    moveTip(TIP::RIGHT_TIP, true);
    ros::Duration(1).sleep();
    arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);

    ROS_INFO("Placerstate: TIP2 - Start Vision");

    pap_common::VisionResult res;
    if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, TIP2_DIAMETER_VISION, 0.0, 0.0, res, 50))
        return false;

    ROS_INFO("Placerstate: TIP2 - cameraOffset received");
    placeController.updateTip2Offset(res.data2, res.data1);

    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

    ROS_INFO("PlacerState: CORRECTED_TIP2");
    tip2 = placeController.getTip2Coordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", tip2.x, tip2.y, tip2.z);

    if(!driveToCoord(tip2.x, tip2.y, tip2.z))
        return false;

    ros::Duration(5).sleep();
    moveTip(TIP::RIGHT_TIP, false);
    return true;
}

// Calibrate ratios using QR-Codes
bool calibrateQR() {
    ROS_INFO("PlacerState: SLOT_QR");
    Offset slot_qr = placeController.SLOT_QR_Offset_;
    ROS_INFO("Go to: x:%f y:%f z:%f", slot_qr.x, slot_qr.y, slot_qr.z);

    if(!driveToCoord(slot_qr.x, slot_qr.y, slot_qr.z))
        return false;

    ros::Duration(3).sleep();
    ROS_INFO("Placerstate: SLOT_QR - Start Vision");

    pap_common::VisionResult res;
    if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START__QRCODE_FINDER, pap_vision::TOP_SLOT, pap_vision::CAMERA_TOP, res, 100))
        return false;


    ROS_INFO("PlacerState: PCB_QR");
    Offset pcb_qr = placeController.PCB_QR_Offset_;
    ROS_INFO("Go to: x:%f y:%f z:%f", pcb_qr.x, pcb_qr.y, pcb_qr.z);

    if(!driveToCoord(pcb_qr.x, pcb_qr.y, pcb_qr.z))
        return false;

    ros::Duration(3).sleep();
    ROS_INFO("Placerstate: PCB_QR - Start Vision");

    if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START__QRCODE_FINDER, pap_vision::TOP_PCB, pap_vision::CAMERA_TOP, res, 100))
        return false;

    ROS_INFO("PlacerState: TAPE_QR");
    Offset tape_qr = placeController.TAPE_QR_Offset_;
    // TODO: check!
    tape_qr.z += 0.8;
    ROS_INFO("Go to: x:%f y:%f z:%f", tape_qr.x, tape_qr.y, tape_qr.z);

    if(!driveToCoord(tape_qr.x, tape_qr.y, tape_qr.z))
        return false;

    ros::Duration(3).sleep();
    ROS_INFO("Placerstate: TAPE_QR - Start Vision");

    if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START__QRCODE_FINDER, pap_vision::TOP_TAPE, pap_vision::CAMERA_TOP, res, 100))
        return false;

    ROS_INFO("PlacerState: GOTO_QR");
    Offset gotoOffset = placeController.BottomCam_QR_Offset_;
    ROS_INFO("Go to: x:%f y:%f z:%f", gotoOffset.x, gotoOffset.y, gotoOffset.z);

    if(!driveToCoord(gotoOffset.x, gotoOffset.y, gotoOffset.z))
        return false;

    pickUp(placeController.largeBoxHeight_+1);

    ROS_INFO("PlacerState: CAM_QR");
    std::cerr << "Goto tip1\n";
    Offset QROffset = placeController.getTip1Coordinates();
    QROffset.z += 1.0;
    ROS_INFO("Go to: x:%f y:%f z:%f", QROffset.x, QROffset.y, QROffset.z);

    if(!driveToCoord(QROffset.x, QROffset.y, QROffset.z))
        return false;

    ros::Duration(2.0).sleep();
    moveTip(TIP::LEFT_TIP, true);
    ros::Duration(1).sleep();
    arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);

    ROS_INFO("Placerstate: CAM_QR - Start Vision");
    if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START__QRCODE_FINDER, pap_vision::BOTTOM_CAM, pap_vision::CAMERA_BOTTOM, res, 100))
        return false;

    ROS_INFO("Placerstate: PCB_QR - cameraFeedback received");
    moveTip(TIP::LEFT_TIP, false);
    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

    QROffset = placeController.BottomCam_QR_Offset_;
    ROS_INFO("Go to: x:%f y:%f z:%f", QROffset.x, QROffset.y, QROffset.z);
    if(!driveToCoord(QROffset.x, QROffset.y, QROffset.z))
        return false;

    placeComp(placeController.largeBoxHeight_+1);
    return true;
}

// Calibrate cam distortion using checkerboard
bool calibrateTopCamDistortion() {

    ROS_INFO("PlacerState: Calibrate top cam distortion - Checkerboard 1");
    Offset gotoOffset = placeController.Checkerboard_top1_Offset_;
    ROS_INFO("Go to: x:%f y:%f z:%f", gotoOffset.x, gotoOffset.y, gotoOffset.z);
    if(!driveToCoord(gotoOffset.x, gotoOffset.y, gotoOffset.z))
        return false;

    if(!driveAroundPosition(gotoOffset, 6, 6))
        return false;
    if(!driveAroundPosition(gotoOffset, 10, 12))
        return false;
    ROS_INFO("Placerstate: First checkerboard finished");

    ROS_INFO("PlacerState: Calibrate top cam distortion - Checkerboard 2");
    gotoOffset = placeController.Checkerboard_bottom2_Offset_;
    ROS_INFO("Go to: x:%f y:%f z:%f", gotoOffset.x, gotoOffset.y, gotoOffset.z);
    if(!driveToCoord(gotoOffset.x, gotoOffset.y, gotoOffset.z))
        return false;

    if(!driveAroundPosition(gotoOffset, 6, 6))
        return false;
    if(!driveAroundPosition(gotoOffset, 8, 10))
        return false;
    ROS_INFO("Placerstate: Second checkerboard finished");

    if(!homeSystem())
        return false;

    return true;
}

// Calibration bottom cam
bool calibrateBottomCamDistortion() {

    ROS_INFO("PlacerState: Calibrate bottom cam distortion - Checkerboard 1");
    Offset gotoOffset = placeController.Checkerboard_bottom1_Offset_;
    ROS_INFO("Go to: x:%f y:%f z:%f", gotoOffset.x, gotoOffset.y, gotoOffset.z);
    if(!driveToCoord(gotoOffset.x, gotoOffset.y, gotoOffset.z))
        return false;

    pickUp(placeController.largeBoxHeight_+1);

    gotoOffset = placeController.getTip1Coordinates();
    gotoOffset.z += 1.0;
    ROS_INFO("Go to: x:%f y:%f z:%f", gotoOffset.x, gotoOffset.y, gotoOffset.z);
    if(!driveToCoord(gotoOffset.x, gotoOffset.y, gotoOffset.z))
        return false;

    ros::Duration(2.0).sleep();
    moveTip(TIP::LEFT_TIP, true);
    arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);
    ros::Duration(1).sleep();

    if(!driveAroundPosition(gotoOffset, 6, 6))
        return false;
    if(!driveAroundPosition(gotoOffset, 8, 10))
        return false;

    ROS_INFO("Placerstate: First checkerboard finished");
    moveTip(TIP::LEFT_TIP, false);
    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

    gotoOffset = placeController.Checkerboard_bottom1_Offset_;
    ROS_INFO("Go to: x:%f y:%f z:%f", gotoOffset.x, gotoOffset.y, gotoOffset.z);
    if(!driveToCoord(gotoOffset.x, gotoOffset.y, gotoOffset.z))
        return false;

    placeComp(placeController.largeBoxHeight_+1);


    ROS_INFO("PlacerState: Calibrate bottom cam distortion - Checkerboard 2");
    gotoOffset = placeController.Checkerboard_bottom2_Offset_;
    ROS_INFO("Go to: x:%f y:%f z:%f", gotoOffset.x, gotoOffset.y, gotoOffset.z);
    if(!driveToCoord(gotoOffset.x, gotoOffset.y, gotoOffset.z))
        return false;

    pickUp(placeController.largeBoxHeight_+1);

    gotoOffset = placeController.getTip1Coordinates();
    gotoOffset.z += 1.0;
    ROS_INFO("Go to: x:%f y:%f z:%f", gotoOffset.x, gotoOffset.y, gotoOffset.z);
    if(!driveToCoord(gotoOffset.x, gotoOffset.y, gotoOffset.z))
        return false;

    ros::Duration(2.0).sleep();
    moveTip(TIP::LEFT_TIP, true);
    arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);
    ros::Duration(1).sleep();

    if(!driveAroundPosition(gotoOffset, 6, 6))
        return false;
    if(!driveAroundPosition(gotoOffset, 8, 10))
        return false;

    ROS_INFO("Placerstate: Second checkerboard finished");
    moveTip(TIP::LEFT_TIP, false);
    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

    gotoOffset = placeController.Checkerboard_bottom2_Offset_;
    ROS_INFO("Go to: x:%f y:%f z:%f", gotoOffset.x, gotoOffset.y, gotoOffset.z);
    if(!driveToCoord(gotoOffset.x, gotoOffset.y, gotoOffset.z))
        return false;

    placeComp(placeController.largeBoxHeight_+1);

    if(!homeSystem())
        return false;

    return true;
}


// Placement process
bool singleCompPlacement() {

    int box = placeController.getBoxNumber();
    if(box < 67) {
        if(!goToBox())
            return false;
    }

    if(!pickUpComponent())
        return false;

    /*if(!checkCompPickUp())
        return false;*/

    if(!placeComponent())
        return false;

    if(!homeSystem())
        return false;

    return true;
}

bool goToBox() {
    ROS_INFO("Placerstate: GOTOBOX");
    sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_IDLE);
    sendPlacerStatus(pap_common::GOTOBOX_STATE, pap_common::PLACER_ACTIVE);

    arduino_client->setLEDTask(placeController.getBoxNumber());
    Offset boxCoords = placeController.getBoxCoordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", boxCoords.x, boxCoords.y, boxCoords.z);

    if(!driveToCoord(boxCoords.x, boxCoords.y, boxCoords.z))
        return false;

    ROS_INFO("Placerstate: GOTOBOX - Vision started");
    float length = placeController.currentComponent.length;
    float width = placeController.currentComponent.width;
    float height = placeController.currentComponent.height;
    pap_common::VisionResult res;

    ROS_INFO("Placer: componentFinder started");
    if(!vision_send_functions::sendVisionTask(*vision_action_client, placeController.finderType,
                                              pap_vision::CAMERA_TOP, length, width, height, res))
        return false;

    placeController.setPickUpCorrectionOffset(res.data2, res.data2, res.data3);
    ROS_INFO("PickUp correction offset: x:%f y:%f, rot:%f",res.data2, res.data2, res.data3);

    ros::Duration(3).sleep();
    ROS_INFO("Placerstate: GOTOBOX - Got feedback from vision");
    return true;
}

bool pickUpComponent() {

    ROS_INFO("Placerstate: GOTOPICKUPCOOR");
    Offset pickupCoord = placeController.getCompPickUpCoordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", pickupCoord.x, pickupCoord.y, pickupCoord.z);
    if(!driveToCoord(pickupCoord.x, pickupCoord.y, pickupCoord.z))
        return false;

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

    if(!driveToCoord(suckCoord.x, suckCoord.y, suckCoord.z))
        return false;

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

    int rotation = (int) placeController.getCompPickUpCoordinates().rot;
    //sendStepperTask((placeController.getTip() + 1), rotation);	// Turn component
    ROS_INFO("Placer - Rotation: rot:%d", rotation);
    ros::Duration(1).sleep();
    return true;
}

bool checkCompPickUp() {

    ROS_INFO("PlacerState: GOTOBOTTOMCAM");
    Offset botCam = placeController.getBottomCamCoordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", botCam.x, botCam.y, botCam.z);

    if(!driveToCoord(botCam.x, botCam.y, botCam.z))
        return false;

    ROS_INFO("Placerstate: GOTOBOTTOMCAM - Vision started");
    float length = placeController.getComponentLenth();
    float width = placeController.getComponentWidth();

    /*sendTask(pap_common::VISION,						// TODO: Start appropriate vision here!
     pap_vision::CHIP_BOTTOM, width, length,
     0);*/

    ROS_INFO("Placerstate: CHECKCOMPPICKUP - Got feedback from vision");
    return true;
}

bool placeComponent() {
    ROS_INFO("Placerstate: GOTOPLACECOORD");
    sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE, pap_common::PLACER_ACTIVE);
    Offset placeCoord = placeController.getCompPlaceCoordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", placeCoord.x, placeCoord.y, placeCoord.z);
    if(!driveToCoord(placeCoord.x, placeCoord.y, placeCoord.z))
        return false;

    ROS_INFO("PlacerState: PLACECOMPONENT");
    sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                     pap_common::PLACER_ACTIVE);

    if (placeController.getTip()) {  // Activate cylinder
        moveTip(TIP::RIGHT_TIP, true);
    } else {
        moveTip(TIP::LEFT_TIP, true);
    }
    ros::Duration(1).sleep();

    placeCoord = placeController.getCompPlaceCoordinates();
    placeCoord.z = placeController.getCompPlaceHeight();
    ROS_INFO("Go to: x:%f y:%f z:%f", placeCoord.x, placeCoord.y, placeCoord.z);

    if(!driveToCoord(placeCoord.x, placeCoord.y, placeCoord.z))
            return false;

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

    // Indicates placement finished & if complPlacement gui sends new data and restarts process
    sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                     pap_common::PLACER_FINISHED);
    return true;
}

// Dispense process
bool dispensePCB() {

    Offset dispCoord;
    dispCoord.x = placeController.dispenseTask.xPos;
    dispCoord.y = placeController.dispenseTask.yPos;
    dispCoord.z = DISPENSER_HEIGHT;
    ROS_INFO("Go to: x:%f y:%f z:%f", dispCoord.x, dispCoord.y, dispCoord.z);
    if(!driveToCoord(dispCoord.x, dispCoord.y, dispCoord.z))
        return false;

    // Turn on dispenser
    switchDispenser(true);
    ros::Duration(placeController.dispenseTask.time).sleep();

    ROS_INFO("PlacerState: GOTOCOORD: x=%f y=%f z=%f",
             placeController.dispenseTask.xPos2,
             placeController.dispenseTask.yPos2, DISPENSER_HEIGHT);


    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD_VEL,
                                                        placeController.dispenseTask.xPos2,
                                                        placeController.dispenseTask.yPos2,
                                                        DISPENSER_HEIGHT,
                                                        placeController.dispenseTask.velocity,
                                                        placeController.dispenseTask.velocity)){
        return false;
    }

    // Turn off dispenser
    switchDispenser(false);

    sendPlacerStatus(pap_common::INFO,
                     pap_common::DISPENSER_FINISHED);
    return true;
}

bool goToPCBOrigin() {
    ROS_INFO("Placer: GOTOPCBORIGIN");
    Offset pcbCoords = placeController.getPCBCalibCoordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", pcbCoords.x, pcbCoords.y, pcbCoords.z);
    if(!driveToCoord(pcbCoords.x, pcbCoords.y, pcbCoords.z))
        return false;

    return true;
}


bool pickUp(double height){
    processAllStatusCallbacks();
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

    if(!driveToCoord(placeController.lastDestination_.x, placeController.lastDestination_.y, height)){
       return false;
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

    return true;
}

bool placeComp(double height){
    processAllStatusCallbacks();

    sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                     pap_common::PLACER_ACTIVE);

    if (placeController.getTip()) {  // Activate cylinder
        moveTip(TIP::RIGHT_TIP, true);
    } else {
        moveTip(TIP::LEFT_TIP, true);
    }
    ros::Duration(1).sleep();

    if(!driveToCoord(placeController.lastDestination_.x, placeController.lastDestination_.y, height)){
       return false;
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

    // Indicates placement finished & if complPlacement gui sends new data and restarts process
    sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                     pap_common::PLACER_FINISHED);

    return true;
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

bool driveAroundPosition(Offset position, int distance_x, int distance_y) {

    float velX = 0.0001;
    float velY = 0.0001;

    ros::Duration(1).sleep();

    for(int i = 0; i <= (distance_x/2); i++) {
        if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                            (position.x - i), position.y, position.z, velX, velY)){
            return false;
        }
        ros::Duration(1).sleep();
    }

    for(int i = 0; i <= (distance_y/2); i++) {
        if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                            (position.x - (distance_x/2)), (position.y - i), position.z, velX, velY)){
            return false;
        }
        ros::Duration(1).sleep();
    }

    for(int i = -(distance_x/2); i <= (distance_x/2); i++) {
        if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                            (position.x + i), (position.y - (distance_y/2)), position.z, velX, velY)){
            return false;
        }
        ros::Duration(1).sleep();
    }

    for(int i = -(distance_y/2); i <= (distance_y/2); i++) {
        if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                            (position.x + (distance_x/2)), (position.y + i), position.z, velX, velY)){
            return false;
        }
        ros::Duration(1).sleep();
    }

    for(int i = -(distance_x/2); i <= (distance_x/2); i++) {
        if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                            (position.x - i), (position.y + (distance_y/2)), position.z, velX, velY)){
            return false;
        }
        ros::Duration(1).sleep();
    }

    for(int i = -(distance_y/2); i <= 0; i++) {
        if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                            (position.x - (distance_x/2)), (position.y - i), position.z, velX, velY)){
            return false;
        }
        ros::Duration(1).sleep();
    }

    for(int i = -(distance_x/2); i <= 0; i++) {
        if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                            (position.x + i), position.y, position.z, velX, velY)){
            return false;
        }
        ros::Duration(1).sleep();
    }

    /*if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        (position.x - (distance/2)), position.y, position.z, velX, velY)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        (position.x - (distance/2)), (position.y - (distance/2)), position.z, velX, velY)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        (position.x + (distance/2)), (position.y - (distance/2)), position.z, velX, velY)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        (position.x + (distance/2)), (position.y + (distance/2)), position.z, velX, velY)){
        return false;
    }
    ros::Duration(1).sleep();



    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        (position.x - (distance/2)), (position.y + (distance/2)), position.z, velX, velY)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        (position.x - (distance/2)), (position.y - (distance/2)), position.z, velX, velY)){
        return false;
    }
    ros::Duration(1).sleep();*/

    return true;
}

bool driveToCoord(const double &x, const double &y, const double &z){
    processAllStatusCallbacks();
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

void processAllStatusCallbacks(){
    for(size_t i = 0; i < 100; i++){
        ros::spinOnce();
    }
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
                || taskMsg->task == pap_common::COMPLETEPLACEMENT) {

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
        }

        switch (taskMsg->task) {

        case pap_common::HOMING: {
            if(! homeSystem()) {
                ROS_ERROR("Placer: HOMING failed");
                // TODO: Handle result
            }
            break;
        }

        case pap_common::STOP: {
            ROS_INFO("Placer: Stop called.");
            //completePlacement = false; ??
            // TODO: Handle
            break;
        }

        case pap_common::CALIBRATION_OFFSET: {
            if(!calibrateOffsets()) {
                ROS_ERROR("Placer: Offset calibration failed");
                // TODO: Handle
            }
            break;
        }

        case pap_common::CALIBRATION_RATIO: {
            if(!calibrateQR()) {
                ROS_ERROR("Placer: Ratio calibration failed");
                // TODO: Handle
            }
            break;
        }

        case pap_common::CALIBRATION_TOPCAM: {
            if(!calibrateTopCamDistortion()) {
                ROS_ERROR("Placer: Top cam calibration failed");
                // TODO: Handle
            }
            break;
        }

        case pap_common::CALIBRATION_BOTTOMCAM: {
           if(!calibrateBottomCamDistortion()) {
                ROS_ERROR("Placer: Bottom cam calibration failed");
                // TODO: Handle
            }
            break;
        }

        case pap_common::SINGLEPLACEMENT: {
            if(!singleCompPlacement()) {
                ROS_ERROR("Placer: Single component placement failed");
                // TODO: Handle
                break;
            }
            if(! homeSystem()) {
                ROS_ERROR("Placer: HOMING failed");
                // TODO: Handle result
            }
            break;
        }

        case pap_common::COMPLETEPLACEMENT: {
            if(!singleCompPlacement()) {
                ROS_ERROR("Placer: Complete component placement failed");
                // TODO: Handle
            }
            break;
        }

        case pap_common::GOTOPCB: {
            if(!goToPCBOrigin()) {
                ROS_ERROR("GOTOPCB failed");
                // TODO: Handle result
            }
            break;
        }

        case pap_common::GOTO: {            
            ROS_INFO("Go to: x:%f y:%f z:%f", taskMsg->data1, taskMsg->data2, taskMsg->data3);
            if(!driveToCoord(taskMsg->data1, taskMsg->data2, taskMsg->data3)){
                ROS_ERROR("GOTO-Command failed");
                //TODO: Handle result
            }
            break;
        }
        }
    }
    }
}

void dispenserCallbackPlacer(const pap_common::DispenseTaskConstPtr& taskMsg) {
    std::cerr << "received dispenser task\n";
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
    //ROS_INFO("Dispensing...");
    dispensePCB();
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


