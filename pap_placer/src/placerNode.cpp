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
    placeController.loadParams();

    ros::NodeHandle n_;
    task_publisher = n_.advertise<pap_common::Task>("task", 1000);
    placerStatus_publisher_ = n_.advertise<pap_common::PlacerStatus>("placerStatus", 100);
    statusSubsriber_ = n_.subscribe("status", 100, &statusCallback);
    placerTaskSubscriber_ = n_.subscribe("task", 10, &placerCallback);
    dispenserTaskSubscriber_ = n_.subscribe("/dispenseTask", 100, &dispenserCallbackPlacer);

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

    if(!driveToCoord(placeController.lastDestination_.x, placeController.lastDestination_.y, 40))
        return false;

    motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::HOMING);
    sendPlacerStatus(pap_common::HOMING_STATE,
                     pap_common::PLACER_IDLE);
    return true;
}


// Offset calibration methods
bool calibrateOffsets(double leftTipRadius, double rightTipRadius) {
    if(!calibrateCamera())
        return false;

    if(!calibrateTip(TIP::LEFT_TIP))
        return false;

    //if(!calibrateTip2())
    //    return false;

    return true;
}

bool calibrateCamera() {
    ROS_INFO("PlacerState: CAMERA");
    sendPlacerStatus(pap_common::IDLE_STATE,
                     pap_common::PLACER_IDLE);
    sendPlacerStatus(pap_common::CALIBRATION_STATE,
                     pap_common::PLACER_ACTIVE);

    arduino_client->LEDTask(pap_common::SETRINGCOLOR, 0);
    ros::Duration(0.1).sleep();
    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

    Offset bottomCam = placeController.getBottomCamCoordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", bottomCam.x, bottomCam.y, bottomCam.z);

    if(!driveToCoord(bottomCam.x, bottomCam.y, bottomCam.z))
        return false;

    ROS_INFO("Placerstate: CAMERA - Start Vision");
    arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);
    ros::Duration(0.1).sleep();
    arduino_client->LEDTask(pap_common::SETTOPLED, 0);
    ros::Duration(0.1).sleep();
    arduino_client->LEDTask(pap_common::SETBRIGHTNESSRING, 45);
    ros::Duration(0.5).sleep();

    pap_common::VisionResult res;
    if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, CAMERA_DIAMETER_VISION, 0.0, (float) true, res, 50))
        return false;

    ROS_INFO("Placerstate: CAMERA - cameraOffset received");
    placeController.updateCameraBottomOffset(res.data1, res.data2);

    ROS_INFO("PlacerState: CORRECTED_CAMERA");
    bottomCam = placeController.getBottomCamCoordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", bottomCam.x, bottomCam.y, bottomCam.z);

    if(!driveToCoord(bottomCam.x, bottomCam.y, bottomCam.z))
        return false;

    ros::Duration(1).sleep();

    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);
    arduino_client->LEDTask(pap_common::RESETTOPLED, 0);
    return true;
}

bool calibrateTip1() {
    ROS_INFO("PlacerState: TIP1 ");
    Offset tip1 = placeController.getTipCoordinates(TIP::LEFT_TIP);
    ROS_INFO("Go to: x:%f y:%f z:%f", tip1.x, tip1.y, tip1.z);

    if(!driveToCoord(tip1.x, tip1.y, tip1.z))
        return false;

    moveTip(TIP::LEFT_TIP, true);

    arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);
    ros::Duration(0.1).sleep();
    arduino_client->LEDTask(pap_common::SETBRIGHTNESSRING, 200);
    ros::Duration(0.5).sleep();
    ROS_INFO("Placerstate: TIP1 - Start Vision");

    pap_common::VisionResult res;
    if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, TIP1_DIAMETER_VISION, 0.0, (float) true, res, 50))
        return true;

    ROS_INFO("Placerstate: TIP1 - cameraOffset received");
    placeController.updateTip1Offset(res.data1, res.data2);

    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

    ROS_INFO("PlacerState: CORRECTED_TIP1");
    tip1 = placeController.getTipCoordinates(TIP::LEFT_TIP);
    ROS_INFO("Go to: x:%f y:%f z:%f", tip1.x, tip1.y, tip1.z);

    if(!driveToCoord(tip1.x, tip1.y, tip1.z))
        return false;

    ros::Duration(1).sleep();
    moveTip(TIP::LEFT_TIP, false);
    return true;
}

bool calibrateDispenser(double diameter) {

    Eigen::Vector3d point_A;
    Eigen::Vector3d point_B;
    Eigen::Vector3d point_C;

    if(!driveToCoord(placeController.dispenserCalibOffsetA.x, placeController.dispenserCalibOffsetA.y, placeController.dispenserCalibOffsetA.z))
        return false;

    pap_common::MotorControllerActionResult res_motor;
    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::HEIGHT_CAL,
                                                        placeController.dispenserCalibOffsetA.x,
                                                        placeController.dispenserCalibOffsetA.y,
                                                        5,
                                                        100,
                                                        100,
                                                        1,
                                                        res_motor)){
        return false;
    }

    point_A[0] = placeController.dispenserCalibOffsetA.x;
    point_A[1] = placeController.dispenserCalibOffsetA.y;
    point_A[2] = res_motor.height;


    if(!driveToCoord(placeController.dispenserCalibOffsetB.x, placeController.dispenserCalibOffsetB.y, placeController.dispenserCalibOffsetB.z))
        return false;

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::HEIGHT_CAL,
                                                        placeController.dispenserCalibOffsetB.x,
                                                        placeController.dispenserCalibOffsetB.y,
                                                        5,
                                                        100,
                                                        100,
                                                        1,
                                                        res_motor)){
        return false;
    }

    std::cerr << "Dispenser calibration height B: " << res_motor.height << std::endl;
    point_B[0] = placeController.dispenserCalibOffsetB.x;
    point_B[1] = placeController.dispenserCalibOffsetB.y;
    point_B[2] = res_motor.height;


    if(!driveToCoord(placeController.dispenserCalibOffsetC.x, placeController.dispenserCalibOffsetC.y, placeController.dispenserCalibOffsetC.z))
        return false;

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::HEIGHT_CAL,
                                                        placeController.dispenserCalibOffsetC.x,
                                                        placeController.dispenserCalibOffsetC.y,
                                                        5,
                                                        100,
                                                        100,
                                                        1,
                                                        res_motor)){
        return false;
    }

    std::cerr << "Dispenser calibration height C: " << res_motor.height << std::endl;
    point_C[0] = placeController.dispenserCalibOffsetC.x;
    point_C[1] = placeController.dispenserCalibOffsetC.y;
    point_C[2] = res_motor.height;

    // Compute plane coordinate form
    Eigen::Vector3d vec_AB = point_B - point_A;
    Eigen::Vector3d vec_AC = point_C - point_A;

    Eigen::Vector3d cross = vec_AB.cross(vec_AC);

    // ax + by + az + d = 0
    // compute d by inserting point A in equation
    double a = cross[0];
    double b = cross[1];
    double c = cross[2];

    double d = -(point_A[0] * a + point_A[1] * b + point_A[2] * c);

    // Insert new plane
    placeController.dispenser_cal_plane_ = pap_placer::plane(a, b, c, d);
    placeController.plane_calibrated_ = true;

    double test = placeController.dispenser_cal_plane_.getHeight(point_C[0], point_C[1]);
    std::cerr << "Calculated height: " << test << " expected: " << res_motor.height << std::endl;

    placeController.dispenserTipOffset.z = res_motor.height + placeController.dispenser_height_offset_;

    ROS_INFO("PlacerState: DISPENSER ");
    Offset dispenser = placeController.getDispenserCoordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", dispenser.x, dispenser.y, dispenser.z);

    if(!driveToCoord(dispenser.x, dispenser.y, dispenser.z))
        return false;

    ros::Duration(1).sleep();
    arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);
    ros::Duration(0.5).sleep();
    arduino_client->LEDTask(pap_common::SETBRIGHTNESSRING, 40);
    ros::Duration(0.5).sleep();

    ROS_INFO("Placerstate: DISPENSER - Start Vision");
    pap_common::VisionResult res;
    if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, diameter, 0.0, (float) true, res, 50))
        return false;

    ROS_INFO("Placerstate: DISPENSER - cameraOffset received");
    placeController.updatedispenserTipOffset(res.data1, res.data2);

    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

    ROS_INFO("PlacerState: CORRECTED_DISPENSER");
    dispenser = placeController.getDispenserCoordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", dispenser.x, dispenser.y, dispenser.z);

    if(!driveToCoord(dispenser.x, dispenser.y, dispenser.z))
        return false;

    ros::Duration(5).sleep();

    return true;
}

bool calibrateTip(TIP selectedTip) {


    Offset tip = placeController.getTipCoordinates(selectedTip);
    ROS_INFO("Go to: x:%f y:%f z:%f", tip.x, tip.y, tip.z);
    if(!driveToCoord(tip.x, tip.y, tip.z))
        return false;

    moveTip(selectedTip, true);
    arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);
    ros::Duration(0.1).sleep();
    arduino_client->LEDTask(pap_common::SETBRIGHTNESSRING, 200);
    ros::Duration(0.5).sleep();
    ROS_INFO("Placerstate: TIP - Start Vision");

    pap_common::VisionResult res;
    if(selcetedTip == TIP::LEFT_TIP) {
        if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, TIP1_DIAMETER_VISION, 0.0, (float) true, res, 50))
            return true;
        placeController.updateTip1Offset(res.data1, res.data2);
    } else {
        if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::SEARCH_CIRCLE, pap_vision::CAMERA_BOTTOM, TIP1_DIAMETER_VISION, 0.0, (float) true, res, 50))
            return true;
        placeController.updateTip2Offset(res.data1, res.data2);
    }

    ROS_INFO("Placerstate: TIP - offset updated");
    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

    ROS_INFO("PlacerState: CORRECTED_TIP");
    tip = placeController.getTipCoordinates(selectedTip);
    ROS_INFO("Go to: x:%f y:%f z:%f", tip.x, tip.y, tip.z);
    if(!driveToCoord(tip.x, tip.y, tip.z))
        return false;

    ros::Duration(1).sleep();
    moveTip(selectedTip, false);
    return true;
}

bool calibrateTip2() {
    ROS_INFO("PlacerState: TIP2 ");
    Offset tip2 = placeController.getTipCoordinates(TIP::RIGHT_TIP);
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
    placeController.updateTip2Offset(res.data1, res.data2);

    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

    ROS_INFO("PlacerState: CORRECTED_TIP2");
    tip2 = placeController.getTipCoordinates(TIP::RIGHT_TIP);
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


    arduino_client->LEDTask(pap_common::SETTOPLED, 0);
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

    pickUp(placeController.largeBoxHeight_+1, TIP::LEFT_TIP);

    ROS_INFO("PlacerState: CAM_QR");
    std::cerr << "Goto tip1\n";
    Offset QROffset = placeController.getTipCoordinates(TIP::LEFT_TIP);
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

    arduino_client->LEDTask(pap_common::RESETTOPLED, 0);
    ROS_INFO("Placerstate: PCB_QR - cameraFeedback received");
    moveTip(TIP::LEFT_TIP, false);
    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);

    QROffset = placeController.BottomCam_QR_Offset_;
    ROS_INFO("Go to: x:%f y:%f z:%f", QROffset.x, QROffset.y, QROffset.z);
    if(!driveToCoord(QROffset.x, QROffset.y, QROffset.z))
        return false;

    placeComp(placeController.largeBoxHeight_+1, TIP::LEFT_TIP);

    if(!homeSystem())
        return false;

    return true;
}

bool calibrateDispenserAmount(double tip_diameter, double init_vel){
    ROS_INFO("PlacerState: Calibrate dispensing amount/n");

    Offset begin_despensing_p;
    begin_despensing_p.x = 30;
    begin_despensing_p.y = 5;
    begin_despensing_p.z = 50;

    const size_t num_blobs = 5;
    const double length_blobs = 3.0;
    const double gap_length = 4.0;
    const double init_wait = 0;
    const double des_width = tip_diameter;
    const double learning_rate = 0.5;

    Offset tipOffset = placeController.dispenserTipOffset;

    double velocity = init_vel;
    double error_width = 0;
    // Generate blobs
    for(size_t i = 0; i < num_blobs; i++){
        dispenseInfo blob;
        blob.time = init_wait;
        blob.velocity = velocity;
        blob.type = dispenser_types::LINE_DISPENSE;

        blob.xPos = begin_despensing_p.x + i*length_blobs + i *gap_length;
        blob.yPos = begin_despensing_p.y;

        blob.xPos2 = begin_despensing_p.x + (i+1) * length_blobs + i *gap_length;
        blob.yPos2 = begin_despensing_p.y;

        std::cerr << "New blob: " << blob.xPos << " / " << blob.yPos << " /2/ " << blob.xPos2 << " / " << blob.yPos2 << " / " <<  std::endl;

        std::vector<dispenseInfo> blobs;
        blobs.push_back(blob);

        if(placeController.plane_calibrated_){
            if(!dispensePCB(blobs, placeController.dispenser_surface_offset_)) return false;
        }else{
            std::cerr << "Plane not yet calibrated...\n";
        }

        // Drive with camera to center of blob
        //double blob_center_x = blob.xPos + (blob.xPos2 -blob.xPos) / 2;
        //double blob_center_y = blob.yPos;

        //if(!driveToCoord(blob_center_x, blob_center_y, 27.0))
        //    return false;

        ros::Duration(5.0).sleep();

        // Measure width of blob with pcv_cv
        pap_common::VisionResult res;

        ROS_INFO("Placerstate: componentFinder started");
        //if(!vision_send_functions::sendVisionTask(*vision_action_client, pap_vision::START_CHIP_FINDER,
        //                                          pap_vision::CAMERA_TOP, 0, 0, 0, res))
        //    return false;

        //error_width = res.data4 -  des_width;

        //std::cerr << "Detected dispenser blob width: " << res.data4 << " ... error is: " << error_width;
        //std::cerr << "Old velocity: " << velocity << " new velocity: " << velocity + learning_rate * error_width << std::endl;

        //velocity = velocity + learning_rate * error_width;
    }

    std::cerr << "Width error was: " << error_width << std::endl;

    //placeController.setDispenserVel(velocity);

    processAllStatusCallbacks();
    if(!driveToCoord(placeController.lastDestination_.x, placeController.lastDestination_.y, 27.0))
        return false;


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

    pickUp(placeController.largeBoxHeight_+1, TIP::LEFT_TIP);

    gotoOffset = placeController.getTipCoordinates(TIP::LEFT_TIP);
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

    placeComp(placeController.largeBoxHeight_+1, TIP::LEFT_TIP);


    ROS_INFO("PlacerState: Calibrate bottom cam distortion - Checkerboard 2");
    gotoOffset = placeController.Checkerboard_bottom2_Offset_;
    ROS_INFO("Go to: x:%f y:%f z:%f", gotoOffset.x, gotoOffset.y, gotoOffset.z);
    if(!driveToCoord(gotoOffset.x, gotoOffset.y, gotoOffset.z))
        return false;

    pickUp(placeController.largeBoxHeight_+1, TIP::LEFT_TIP);

    gotoOffset = placeController.getTipCoordinates(TIP::LEFT_TIP);
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

    placeComp(placeController.largeBoxHeight_+1, TIP::LEFT_TIP);

    if(!homeSystem())
        return false;

    return true;
}

bool multipleCompPlacement() {

    if(placeController.leftTipComponent.isWaiting && placeController.rightTipComponent.isWaiting) {

        // Pick-up component with left tip
        ROS_INFO("PLACER: Going to pickup coords of left tip");
        Offset pickupCoord = placeController.getCompPickUpCoordinates(TIP::LEFT_TIP);
        ROS_INFO("PLACER: Go to x:%f y:%f z:%f", pickupCoord.x, pickupCoord.y, pickupCoord.z);
        if(!driveToCoord(pickupCoord.x, pickupCoord.y, pickupCoord.z))
            return false;

        ROS_INFO("PLACER: Picking up component with left tip");
        pickupCoord.z = placeController.getCompSuckingHeight(TIP::LEFT_TIP);
        pickUp(pickupCoord.z, TIP::LEFT_TIP);

        int rotation = (int) placeController.getCompPickUpCoordinates(TIP::LEFT_TIP).rot;
        ROS_INFO("PLACER: Rotate component by %d", rotation);
        arduino_client->sendStepperTask(TIP::LEFT_TIP, rotation);
        ros::Duration(1).sleep();

        // Pick-up component with right tip
        ROS_INFO("PLACER: Going to pickup coords of right tip");
        pickupCoord = placeController.getCompPickUpCoordinates(TIP::RIGHT_TIP);
        ROS_INFO("PLACER: Go to x:%f y:%f z:%f", pickupCoord.x, pickupCoord.y, pickupCoord.z);
        if(!driveToCoord(pickupCoord.x, pickupCoord.y, pickupCoord.z))
            return false;

        ROS_INFO("PLACER: Picking up component with right tip");
        pickupCoord.z = placeController.getCompSuckingHeight(TIP::RIGHT_TIP);
        pickUp(pickupCoord.z, TIP::RIGHT_TIP);

        rotation = (int) placeController.getCompPickUpCoordinates(TIP::LEFT_TIP).rot;
        ROS_INFO("PLACER: Rotate component by %d", rotation);
        arduino_client->sendStepperTask(TIP::RIGHT_TIP, rotation);
        ros::Duration(1).sleep();

        // Check left tip pick-up
        ROS_INFO("PLACER: Check left tip pickup");
        Offset checkCoord = placeController.getTipCoordinates(TIP::LEFT_TIP);
        checkCoord.z += placeController.getComponentHeight(TIP::LEFT_TIP);
        ROS_INFO("PLACER: Go to x:%f y:%f z:%f", checkCoord.x, checkCoord.y, checkCoord.z);
        if(!driveToCoord(checkCoord.x, checkCoord.y, checkCoord.z))
            return false;

        ros::Duration(2.0).sleep();
        moveTip(TIP::LEFT_TIP, true);           // Set tip

        ros::Duration(1).sleep();
        arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);
        ros::Duration(5).sleep();
        arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);
        moveTip(TIP::LEFT_TIP, false);           // Release tip

        // Check right tip pick-up
        ROS_INFO("PLACER: Check right tip pickup");
        checkCoord = placeController.getTipCoordinates(TIP::RIGHT_TIP);
        checkCoord.z += placeController.getComponentHeight(TIP::RIGHT_TIP);
        ROS_INFO("PLACER: Go to x:%f y:%f z:%f", checkCoord.x, checkCoord.y, checkCoord.z);
        if(!driveToCoord(checkCoord.x, checkCoord.y, checkCoord.z))
            return false;

        ros::Duration(2.0).sleep();
        moveTip(TIP::RIGHT_TIP, true);           // Set tip

        ros::Duration(1).sleep();
        arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);
        ros::Duration(5).sleep();
        arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);
        moveTip(TIP::RIGHT_TIP, false);           // Release tip

        // Place left tip component
        ROS_INFO("PLACER: Going to left tip place coord");
        sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE, pap_common::PLACER_ACTIVE);
        Offset placeCoord = placeController.getCompPlaceCoordinates(TIP::LEFT_TIP);
        ROS_INFO("PLACER: Go to x:%f y:%f z:%f", placeCoord.x, placeCoord.y, placeCoord.z);
        if(!driveToCoord(placeCoord.x, placeCoord.y, placeCoord.z))
            return false;

        ROS_INFO("PLACER: Placing component");
        placeCoord.z = placeController.getCompPlaceHeight(TIP::LEFT_TIP);
        placeComp(placeCoord.z, TIP::LEFT_TIP);
        placeController.leftTipComponent.isWaiting = false;

        // Place right tip component
        ROS_INFO("PLACER: Going to right tip place coord");
        sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE, pap_common::PLACER_ACTIVE);
        placeCoord = placeController.getCompPlaceCoordinates(TIP::RIGHT_TIP);
        ROS_INFO("PLACER: Go to x:%f y:%f z:%f", placeCoord.x, placeCoord.y, placeCoord.z);
        if(!driveToCoord(placeCoord.x, placeCoord.y, placeCoord.z))
            return false;

        ROS_INFO("PLACER: Placing component");
        placeCoord.z = placeController.getCompPlaceHeight(TIP::RIGHT_TIP);
        placeComp(placeCoord.z, TIP::RIGHT_TIP);
        placeController.rightTipComponent.isWaiting = false;

        // Indicates placement finished & if complPlacement gui sends new data and restarts process
        sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                         pap_common::PLACER_FINISHED);

    } else {
        if(!singleCompPlacement()) {
            std::cerr << "Single component placement failed" << std::endl;
            return false;
        }
    }
    return true;
}


// Placement process
bool singleCompPlacement() {

    TIP activeTip;
    if(placeController.leftTipComponent.isWaiting) {
        activeTip = TIP::LEFT_TIP;
        std::cerr << "PlaceController: Left tip used" << std::endl;
    } else if(placeController.rightTipComponent.isWaiting) {
        activeTip = TIP::RIGHT_TIP;
        std::cerr << "PlaceController: Right tip used" << std::endl;
    } else {
        std::cerr << "PlaceController: No component data waiting for placement" << std::endl;
        return false;
    }

    int box = placeController.getBoxNumber(activeTip);
    if(box < 67) {
        if(!goToBox(activeTip))
            return false;
    }

    ROS_INFO("PLACER: Going to pickup coords");
    Offset pickupCoord = placeController.getCompPickUpCoordinates(activeTip);
    ROS_INFO("PLACER: Go to x:%f y:%f z:%f", pickupCoord.x, pickupCoord.y, pickupCoord.z);
    if(!driveToCoord(pickupCoord.x, pickupCoord.y, pickupCoord.z))
        return false;

    ROS_INFO("PLACER: Picking up component");
    pickupCoord.z = placeController.getCompSuckingHeight(activeTip);
    pickUp(pickupCoord.z, activeTip);

    ROS_INFO("PLACER: Check pickup");
    Offset checkCoord = placeController.getTipCoordinates(activeTip);
    checkCoord.z += placeController.getComponentHeight(activeTip);
    ROS_INFO("PLACER: Go to x:%f y:%f z:%f", checkCoord.x, checkCoord.y, checkCoord.z);
    if(!driveToCoord(checkCoord.x, checkCoord.y, checkCoord.z))
        return false;

    moveTip(activeTip, true);           // Set tip
    arduino_client->LEDTask(pap_common::SETBOTTOMLED, 0);
    ros::Duration(1.0).sleep();

    ros::Duration(5).sleep();
    float rotation = placeController.getCompPickUpCoordinates(activeTip).rot;
    int steps = placeController.angleToSteps(rotation);
    arduino_client->sendStepperTask(activeTip, steps);
    ros::Duration(5).sleep();

    arduino_client->LEDTask(pap_common::RESETBOTTOMLED, 0);
    moveTip(activeTip, false);           // Release tip

    ROS_INFO("PLACER: Going to place coord");
    sendPlacerStatus(pap_common::GOTOPCBCOMP_STATE, pap_common::PLACER_ACTIVE);
    Offset placeCoord = placeController.getCompPlaceCoordinates(activeTip);
    ROS_INFO("PLACER: Go to x:%f y:%f z:%f", placeCoord.x, placeCoord.y, placeCoord.z);
    if(!driveToCoord(placeCoord.x, placeCoord.y, placeCoord.z))
        return false;

    ROS_INFO("PLACER: Placing component");
    placeCoord.z = placeController.getCompPlaceHeight(activeTip);
    placeComp(placeCoord.z, activeTip);

    if(activeTip == TIP::LEFT_TIP) {
        placeController.leftTipComponent.isWaiting = false;
    } else {
        placeController.rightTipComponent.isWaiting = false;
    }

    // Indicates placement finished
    sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                     pap_common::PLACER_FINISHED);
    return true;
}

bool goToBox(TIP usedTip) {
    ROS_INFO("Placerstate: GOTOBOX");
    sendPlacerStatus(pap_common::IDLE_STATE, pap_common::PLACER_IDLE);
    sendPlacerStatus(pap_common::GOTOBOX_STATE, pap_common::PLACER_ACTIVE);

    arduino_client->setLEDTask(placeController.getBoxNumber(usedTip));
    Offset boxCoords = placeController.getBoxCoordinates(usedTip);
    std::cerr << "Got box coordinates: " << boxCoords.x << ", " << boxCoords.y  << std::endl;
    ROS_INFO("Go to: x:%f y:%f z:%f", boxCoords.x, boxCoords.y, boxCoords.z);

    if(!driveToCoord(boxCoords.x, boxCoords.y, boxCoords.z))
        return false;

    std::cerr << "Got box coordinates" << std::endl;
    ROS_INFO("Placerstate: GOTOBOX - Vision started");
    float length = placeController.getComponentLenth(usedTip);
    float width = placeController.getComponentWidth(usedTip);
    float height = placeController.getComponentHeight(usedTip);
    pap_common::VisionResult res;

    std::cerr << "Sizes passed to vision: " << length << ", " << width << ", " << height << std::endl;

    ROS_INFO("Placerstate: componentFinder started");
    if(!vision_send_functions::sendVisionTask(*vision_action_client, placeController.getFinderType(usedTip),
                                              pap_vision::CAMERA_TOP, length, width, height, res, 100))
        return false;

    const Offset &camera_offset = placeController.getCameraProjectionOffset();

    placeController.setPickUpCorrectionOffset(res.data1 + camera_offset.x , res.data2 + camera_offset.y, res.data3);
    ROS_INFO("PickUp correction offset: x:%f y:%f, rot:%f",res.data1, res.data2, res.data3);

    ros::Duration(3).sleep();
    return true;
}


bool dispenseLines(std::vector<dispenseInfo>  &lines, double dispense_height_offset){
    Offset dispCoord;
    double height_1 = 0;
    double height_2 = 0;
    dispenseInfo begin = lines.front();
    dispCoord.x = begin.xPos;
    dispCoord.y = begin.yPos;

    height_1 = placeController.dispenser_cal_plane_.getHeight(dispCoord.x, dispCoord.y);
    dispCoord.z = height_1 + dispense_height_offset;

    if(!driveToCoord(dispCoord.x, dispCoord.y, dispCoord.z))
        return false;

    for(size_t i = 0; i < lines.size(); i++){
        dispenseInfo goal = lines.at(i);

        double velocity = goal.velocity;

        if(goal.type == dispenser_types::LINE_DISPENSE ){
            switchDispenser(true);
        }
        else{
            velocity = DISPENSER_CONN_SPEED;
            switchDispenser(false);
        }

        if(i == 0 && goal.type == dispenser_types::LINE_DISPENSE){
            ros::Duration(begin.time).sleep();
        }


        height_2 = placeController.dispenser_cal_plane_.getHeight(goal.xPos2, goal.yPos2);

        ROS_INFO("PlacerState: GOTOCOORD: x=%f y=%f z=%f",
                 goal.xPos2,
                 goal.yPos2, height_2 + dispense_height_offset);


        if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD_VEL,
                                                            goal.xPos2,
                                                            goal.yPos2,
                                                            height_2 + dispense_height_offset,
                                                            velocity,
                                                            velocity)){
            return false;
        }
    }
    switchDispenser(false);
    return true;
}

bool dispenseDots(std::vector<dispenseInfo>  &dots, double dispense_height_offset){
    for(size_t i = 0; i < dots.size(); i++){
        dispenseInfo goal = dots.at(i);

        Offset dispCoord;
        dispCoord.x = goal.xPos;
        dispCoord.y = goal.yPos;
        double height = placeController.dispenser_cal_plane_.getHeight(dispCoord.x, dispCoord.y);
        dispCoord.z = height + dispense_height_offset;

        if(!driveToCoord(dispCoord.x, dispCoord.y, dispCoord.z, dispCoord.z + 3))
            return false;

        switchDispenser(true);
        ros::Duration(goal.time).sleep();
        switchDispenser(false);

    }
    switchDispenser(false);
    return true;
}

// Dispense process
bool dispensePCB(std::vector<dispenseInfo> dispense_task, double dispense_height_offset) {
    dispenseInfo begin = dispense_task.front();

    Offset begin_offset;
    begin_offset.x = begin.xPos;
    begin_offset.y = begin.yPos;
    begin_offset.z = placeController.MovingHeight_;

    if(!driveToCoord(begin_offset.x, begin_offset.y, begin_offset.z))
        return false;
    /*
    if(begin.type == dispenser_types::LINE_DISPENSE ){
        dispenseLines(dispense_task, dispense_height_offset);
    }else if (begin.type == dispenser_types::DOT_DISPENSE) {
        dispenseDots(dispense_task, dispense_height_offset);
    }

    processAllStatusCallbacks();
    if(!driveToCoord( placeController.lastDestination_.x,  placeController.lastDestination_.y, 45))
        return false;

    sendPlacerStatus(pap_common::INFO,
                     pap_common::DISPENSER_FINISHED);
                     */
    return true;
}

bool goToPCBOrigin() {
    ROS_INFO("Placer: GOTOPCBORIGIN");
    Offset pcbCoords = placeController.getPCBCalibCoordinates();
    ROS_INFO("Go to: x:%f y:%f z:%f", pcbCoords.x, pcbCoords.y, pcbCoords.z);

    if(!driveToCoord(pcbCoords.x, pcbCoords.y, pcbCoords.z))
        return false;
    arduino_client->LEDTask(pap_common::SETTOPLED, 0);
    return true;
}


bool pickUp(double height, TIP usedTip){
    processAllStatusCallbacks();
    sendPlacerStatus(pap_common::STARTPICKUP_STATE,
                     pap_common::PLACER_ACTIVE);
    ros::Duration(1).sleep();
    moveTip(usedTip, true);         // Activate tip
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        placeController.lastDestination_.x,
                                                        placeController.lastDestination_.y, height)){
        return false;
    }

    switchVacuum(true);
    forwardVacuum(usedTip, true);
    ros::Duration(1).sleep();

    moveTip(usedTip, false);		// Release tip
    ros::Duration(1).sleep();
    return true;
}


bool placeComp(double height, TIP usedTip){
    processAllStatusCallbacks();
    sendPlacerStatus(pap_common::PLACECOMPONENT_STATE,
                     pap_common::PLACER_ACTIVE);

    moveTip(usedTip, true);         // Activate cylinder
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        placeController.lastDestination_.x,
                                                        placeController.lastDestination_.y, height)){
        return false;
    }

    switchVacuum(false);
    forwardVacuum(usedTip, false);
    ros::Duration(1).sleep();

    moveTip(usedTip, false);		// Release tip
    ros::Duration(1).sleep();
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

    float velX = 1;
    float velY = 1;

    ros::Duration(1).sleep();


    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD_VEL,
                                                        (position.x - (distance_x/2)), position.y, position.z, velX, velY)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD_VEL,
                                                        (position.x - (distance_x/2)), (position.y - (distance_y/2)), position.z, velX, velY)){
        return false;
    }
    ros::Duration(1).sleep();


    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD_VEL,
                                                        (position.x + (distance_x/2)), (position.y - (distance_y/2)), position.z, velX, velY)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD_VEL,
                                                        (position.x + (distance_x/2)), (position.y + (distance_y/2)), position.z, velX, velY)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD_VEL,
                                                        (position.x - (distance_x/2)), (position.y + (distance_y/2)), position.z, velX, velY)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD_VEL,
                                                        (position.x - (distance_x/2)), position.y, position.z, velX, velY)){
        return false;
    }
    ros::Duration(1).sleep();

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD_VEL,
                                                        position.x, position.y, position.z, velX, velY)){
        return false;
    }
    ros::Duration(1).sleep();

    return true;
}

bool driveToCoord(const double &x, const double &y, const double &z, const double moving_height){
    processAllStatusCallbacks();
    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        placeController.lastDestination_.x,
                                                        placeController.lastDestination_.y,
                                                        moving_height)){
        return false;
    }

    if(!motor_send_functions::sendMotorControllerAction(*motor_action_client, pap_common::COORD,
                                                        x,
                                                        y,
                                                        moving_height)){
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

        switch (taskMsg->task) {
        case pap_common::UPDATE_PLACER: {

            const Offset &camera_offset = placeController.getCameraProjectionOffset();

            ComponentPlacerData tempComponent;
            tempComponent.destX = taskMsg->data1 + camera_offset.x;
            tempComponent.destY = taskMsg->data2 + camera_offset.y;
            tempComponent.rotation = taskMsg->data3;
            tempComponent.box = taskMsg->box;
            tempComponent.height = taskMsg->height;
            tempComponent.length = taskMsg->length;
            tempComponent.width = taskMsg->width;
            tempComponent.tapeX = taskMsg->velX;
            tempComponent.tapeY = taskMsg->velY;
            tempComponent.tapeRot = taskMsg->velZ;
            placeController.updatePlacementData(tempComponent, (TIP)taskMsg->tip);
        } break;

        case pap_common::START_SINGLE_PLACEMENT: {
            if(!singleCompPlacement()) {
                placeController.leftTipComponent.isWaiting = false;
                placeController.leftTipComponent.isWaiting = false;
                ROS_ERROR("Placer: Single component placement failed");
            }
        } break;

        case pap_common::START_COMPLETE_PLACEMENT: {
            if(!multipleCompPlacement()) {
                placeController.leftTipComponent.isWaiting = false;
                placeController.leftTipComponent.isWaiting = false;
                ROS_ERROR("Placer: Multiple component placement failed");
            }
        } break;

        case pap_common::HOMING: {
            if(! homeSystem()) {
                ROS_ERROR("Placer: HOMING failed");
                // TODO: Handle result
            }
            break;
        }

        case pap_common::STOP: {
            placeController.leftTipComponent.isWaiting = false;
            placeController.rightTipComponent.isWaiting = false;
            ROS_INFO("Placer: Stop called.");
            break;
        }

        case pap_common::CALIBRATION_OFFSET: {
            //std::cerr << "Tip radius: " << taskMsg->data1 << " (tip1), " << taskMsg->data2 << " (tip2)" << std::endl;
            //if(!calibrateOffsets(taskMsg->data1, taskMsg->data2)) {
            if(!calibrateOffsets(0.0, 0.0)) {
                ROS_ERROR("Placer: Offset calibration failed");
            } else {
                sendPlacerStatus(pap_common::OFFSET_CALIBRATION,
                                 pap_common::PLACER_FINISHED);
            }
            break;
        }

        case pap_common::CALIBRATION_RATIO: {
            if(!calibrateQR()) {
                ROS_ERROR("Placer: Ratio calibration failed");
                // TODO: Handle
            } else {
                sendPlacerStatus(pap_common::RATIO_CALIBRATION,
                                 pap_common::PLACER_FINISHED);
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
            break;
        }

        case pap_common::COMPLETEPLACEMENT: {
            //if(!singleCompPlacement()) {
            //    ROS_ERROR("Placer: Complete component placement failed");
            // TODO: Handle
            //}
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

        case pap_common::ADJUST_DISPENSER: {
            std::cerr << "Starting dispenser amount init...\n";
            if(!calibrateDispenserAmount(taskMsg->data1, taskMsg->data2)){
                ROS_ERROR("Error while calibrating dispenser amount.../n");
            }
        }
            break;

        case pap_common::CALIBRATE_DISPENSER: {
            std::cerr << "Starting dispenser calibration ...\n";
            if(!calibrateDispenser(taskMsg->data1))
                std::cerr << "Failed to calibrate dispenser nozzle ...\n";
        }
            break;
        }
    }
    }
}

void dispenserCallbackPlacer(const pap_common::DispenseTasksConstPtr& taskMsg) {
    std::cerr << "received dispenser task\n";

    Offset tipOffset = placeController.dispenserTipOffset + placeController.getCameraProjectionOffset();

    std::vector<dispenseInfo> dispense_task;
    for(size_t i = 0; i < taskMsg->lines.size(); i++){
        dispenseInfo di;
        pap_common::DispenseTask act = taskMsg->lines.at(i);
        di.xPos = act.xPos1 + tipOffset.x;
        di.yPos = act.yPos1 + tipOffset.y;
        di.xPos2 = act.xPos2 + tipOffset.x;
        di.yPos2 = act.yPos2 + tipOffset.y;
        di.velocity = act.velocity;
        di.time = act.waitTime;
        di.type = act.type;
        dispense_task.push_back(di);
    }

    placeController.dispenser_surface_offset_ = taskMsg->heightOffset;

    //ROS_INFO("Dispensing...");
    if(placeController.plane_calibrated_){
        dispensePCB(dispense_task, placeController.dispenser_surface_offset_);
    }
    else{
        std::cerr << "Plane not yet calibrated...\n";
    }
}

void sendPlacerStatus(pap_common::PROCESS process,
                      pap_common::PLACER_STATUS status) {
    pap_common::PlacerStatus statusMsg;
    statusMsg.process = process;
    statusMsg.status = status;
    placerStatus_publisher_.publish(statusMsg);
}
