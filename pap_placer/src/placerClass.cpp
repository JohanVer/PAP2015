/**
 * @file /src/placerClass.cpp
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <pap_placer/placerClass.hpp>
#include <pap_placer/offsetTable.hpp>

template<typename T>
bool getPAPParam(ros::NodeHandle &nh,const std::string &param_name, T &param){

    if (!nh.getParam(param_name, param))
    {
        std::cerr << "Could not load param: " << param_name << std::endl;
        return false;
    }else{
        std::cout << "Loaded param: " << param_name << " : " << param << std::endl;
        return true;
    }
}

bool PlaceController::saveOffsetsToFile(){
    ofstream param_file;
    std::string package_path = ros::package::getPath("PAP_resources");
    param_file.open(package_path + "/parameters/offsets.yaml");
    param_file << "tip1Offset/x" << ": " << tip1Offset.x << std::endl;
    param_file << "tip1Offset/y" << ": " << tip1Offset.y << std::endl;
    param_file << "tip1Offset/z" << ": " << tip1Offset.z << std::endl;

    param_file << "tip2Offset/x" << ": " << tip2Offset.x << std::endl;
    param_file << "tip2Offset/y" << ": " << tip2Offset.y << std::endl;
    param_file << "tip2Offset/z" << ": " << tip2Offset.z << std::endl;

    param_file << "camOffset/x" << ": " << cameraBottomOffset.x << std::endl;
    param_file << "camOffset/y" << ": " << cameraBottomOffset.y << std::endl;
    param_file << "camOffset/z" << ": " << cameraBottomOffset.z << std::endl;

    param_file << "dispenserOffset/x" << ": " << dispenserTipOffset.x << std::endl;
    param_file << "dispenserOffset/y" << ": " << dispenserTipOffset.y << std::endl;
    param_file << "dispenserOffset/z" << ": " << dispenserTipOffset.z << std::endl;

    param_file.close();
    std::cout << "Saved all offsets to file...\n";
    return true;
}

void PlaceController::loadParams(){
    ros::NodeHandle nh;
    //Load parameters
    getPAPParam<double>(nh, "tip1Offset/x", tip1Offset.x);
    getPAPParam<double>(nh, "tip1Offset/y", tip1Offset.y);
    getPAPParam<double>(nh, "tip1Offset/z", tip1Offset.z);

    getPAPParam<double>(nh, "tip2Offset/x", tip2Offset.x);
    getPAPParam<double>(nh, "tip2Offset/y", tip2Offset.y);
    getPAPParam<double>(nh, "tip2Offset/z", tip2Offset.z);

    getPAPParam<double>(nh, "camOffset/x", cameraBottomOffset.x);
    getPAPParam<double>(nh, "camOffset/y", cameraBottomOffset.y);
    getPAPParam<double>(nh, "camOffset/z", cameraBottomOffset.z);

    getPAPParam<double>(nh, "dispenserOffset/x", dispenserTipOffset.x);
    getPAPParam<double>(nh, "dispenserOffset/y", dispenserTipOffset.y);
    getPAPParam<double>(nh, "dispenserOffset/z", dispenserTipOffset.z);
}

/*****************************************************************************
 ** Constructor
 *****************************************************************************/
PlaceController::PlaceController() {
    //Relative offsets to camera
    tip1Offset.x = -95;
    tip1Offset.y = 0;
    tip1Offset.z = 50;

    tip2Offset.x = -94.08;
    tip2Offset.y = 70.2;
    tip2Offset.z = 50;

    cameraBottomOffset.x = 236;
    cameraBottomOffset.y = 195.65;
    cameraBottomOffset.z = 8.17;

    dispenserTipOffset.x = -52.499;
    dispenserTipOffset.y = 38.988;
    dispenserTipOffset.z = 36;

    corr_dispenser_vel_ = 0.0;

    MovingHeight_ = 45.0;
    dispenserHeight_ = 20.0;
    // CAL Point A
    dispenserCalibOffsetA.x = 175;
    dispenserCalibOffsetA.y = 117;
    dispenserCalibOffsetA.z = dispenserHeight_;
    // CAL Point B
    dispenserCalibOffsetB.x = 240;
    dispenserCalibOffsetB.y = 153;
    dispenserCalibOffsetB.z = dispenserHeight_;
    // CAL Point C
    dispenserCalibOffsetC.x = 240;
    dispenserCalibOffsetC.y = 90;
    dispenserCalibOffsetC.z = dispenserHeight_;

    dispenser_height_offset_ = 27.15;
    dispenser_surface_offset_ = 0;

    idleCoordinates_.x = 5.0;
    idleCoordinates_.y = 5.0;
    idleCoordinates_.z = 0.0;

    camera_projection_offset_.x = 0.06;
    camera_projection_offset_.y = -0.35;

    // Height for sucking a component (normal chip, not a tape)
    largeBoxHeight_ = 18.2;

    leftTipPCBHeight_ = 25.6;
    rightTipPCBHeight_ = 25.6;
    leftTipSuckingHeight_ = 21.0;
    rightTipSuckingHeight_ = 21.0;

    tipHeightCalibrationOffset_.x = 280;
    tipHeightCalibrationOffset_.y = 41.35;
    tipHeightCalibrationOffset_.z = 30.0;

    // Absolut offests
    pcbOriginOffset.x = 300;
    pcbOriginOffset.y = 145;
    pcbOriginOffset.z = 25.6;
    pickUpAreaOffset.x = 108.42;	// + tape_x -> 449.85 = max.x destination
    pickUpAreaOffset.y = 261;
    pickUpAreaOffset.z = 45;

    // Calibration offsets - QR Code positions
    SLOT_QR_Offset_.x = 109.92;
    SLOT_QR_Offset_.y = 261.008;
    SLOT_QR_Offset_.z = 45;
    PCB_QR_Offset_.x = 293.04;
    PCB_QR_Offset_.y = 133.027;
    PCB_QR_Offset_.z = 45;
    TAPE_QR_Offset_.x = 389.554;
    TAPE_QR_Offset_.y = 110.429;
    TAPE_QR_Offset_.z = 45;
    BottomCam_QR_Offset_.x = 218;
    BottomCam_QR_Offset_.y = 177;
    BottomCam_QR_Offset_.z = 20.15;

    /*
    Checkerboard_top1_Offset_.x = 377;
    Checkerboard_top1_Offset_.y = 77;
    Checkerboard_top1_Offset_.z = pickUpAreaOffset.z;
    Checkerboard_top2_Offset_.x = 377.5;
    Checkerboard_top2_Offset_.y = 53;
    Checkerboard_top2_Offset_.z = pickUpAreaOffset.z;

    Checkerboard_bottom1_Offset_.x = 327.5;
    Checkerboard_bottom1_Offset_.y = 186.4;
    Checkerboard_bottom1_Offset_.z = 18.6;
    Checkerboard_bottom2_Offset_.x = 324.15;
    Checkerboard_bottom2_Offset_.y = 155.2;
    Checkerboard_bottom2_Offset_.z = 18.6;
*/
    // Correction offsets
    PickUpCorrection.x = 0;
    PickUpCorrection.y = 0;
    PickUpCorrection.z = 0;
    PickUpCorrection.rot = 0.0;

    pickRelQR_ = false;
    plane_calibrated_ = false;
}

PlaceController::~PlaceController() {
    saveOffsetsToFile();
}



/******************************************************
* Update offsets based on calibration vision feedback
******************************************************/
// absolut
void PlaceController::updateCameraBottomOffset(float update_x, float update_y) {
    cameraBottomOffset.x += update_x;
    cameraBottomOffset.y += update_y;
}

// relativ to top cam
void PlaceController::updateTip1Offset(float update_x, float update_y) {
    tip1Offset.x += update_x;
    tip1Offset.y += update_y;
}

// relativ to top cam
void PlaceController::updateTip2Offset(float update_x, float update_y) {
    tip2Offset.x += update_x;
    tip2Offset.y += update_y;
}

// relativ to top cam
void PlaceController::updatedispenserTipOffset(float update_x, float update_y) {
    dispenserTipOffset.x += update_x;
    dispenserTipOffset.y += update_y;
}


/******************************************************
* Get coordinates
******************************************************/
Offset PlaceController::getBottomCamCoordinates() {
    Offset temp;
    temp.x = cameraBottomOffset.x;
    temp.y = cameraBottomOffset.y;
    temp.z = cameraBottomOffset.z;
    return temp;
}

Offset PlaceController::getTipCoordinates(TIP usedTip) {

    Offset tipCoordinate;
    if(usedTip == TIP::LEFT_TIP) {
        tipCoordinate.x = cameraBottomOffset.x + tip1Offset.x;
        tipCoordinate.y = cameraBottomOffset.y + tip1Offset.y;
        tipCoordinate.z = tip1Offset.z;
    } else {
        tipCoordinate.x = cameraBottomOffset.x + tip2Offset.x;
        tipCoordinate.y = cameraBottomOffset.y + tip2Offset.y;
        tipCoordinate.z = tip2Offset.z;
    }
    return tipCoordinate;
}

Offset PlaceController::getDispenserCoordinates() {
    Offset dispenserCoordinate;
    dispenserCoordinate.x = cameraBottomOffset.x + dispenserTipOffset.x;
    dispenserCoordinate.y = cameraBottomOffset.y + dispenserTipOffset.y;
    dispenserCoordinate.z = dispenserTipOffset.z;
    return dispenserCoordinate;
}

Offset PlaceController::getPCBCalibCoordinates() {
    return pcbOriginOffset;
}

Offset PlaceController::getBoxCoordinates(TIP usedTip) {

    Offset temp;
    temp.x = pickUpAreaOffset.x;
    temp.y = pickUpAreaOffset.y;
    temp.z = pickUpAreaOffset.z;

    ComponentPlacerData* currentComp;
    if(usedTip == TIP::LEFT_TIP) {
        currentComp = &leftTipComponent;
    } else {
        currentComp = &rightTipComponent;
    }

    if (currentComp->box < 67) { //47
        temp.x += BoxOffsetTable[currentComp->box].x;
        temp.y += BoxOffsetTable[currentComp->box].y;
    } else if ((currentComp->box >= 67) && (currentComp->box <= 86)) {
        // Its a tape
        if(TapeOffsetTable[currentComp->box - 67].x+temp.x < 450.0) temp.x += TapeOffsetTable[currentComp->box - 67].x; else temp.x = 449.9;
        temp.y += TapeOffsetTable[currentComp->box - 67].y;
    }

    return temp;
}

Offset PlaceController::getCompPickUpCoordinates(TIP usedTip) {

    Offset temp;
    ComponentPlacerData* currentComp;

    if(usedTip == TIP::LEFT_TIP) {
        currentComp = &leftTipComponent;
        temp.z = leftTipSuckingHeight_ + 10.0;
        temp.x = tip1Offset.x;
        temp.y = tip1Offset.y;
    } else {
        currentComp = &rightTipComponent;
        temp.z = rightTipSuckingHeight_ + 10.0;
        temp.x = tip2Offset.x;
        temp.y = tip2Offset.y;
    }

    if (currentComp->box < 67) {
        temp.x += (pickUpAreaOffset.x
                   + BoxOffsetTable[currentComp->box].x
                + PickUpCorrection.x);
        temp.y += (pickUpAreaOffset.y
                   + BoxOffsetTable[currentComp->box].y
                + PickUpCorrection.y);
        temp.rot = PickUpCorrection.rot + fmod(currentComp->rotation,180);

    } else if ((currentComp->box >= 67) && (currentComp->box <= 86)) {
        // Its a tape
        temp.x += currentComp->tapeX;
        temp.y += currentComp->tapeY;
        temp.rot = currentComp->tapeRot + fmod(currentComp->rotation,180);
    }
    return temp;
}

float PlaceController::getCompSuckingHeight(TIP usedTip) {
    if(pickRelQR_){
        return largeBoxHeight_;
    }

    if(usedTip == TIP::LEFT_TIP) {
        if ((leftTipComponent.box >= 67) && (leftTipComponent.box <= 86)) {
            return leftTipSuckingHeight_;
        } else {
            //return pickUpAreaOffset.z + leftTipComponent.height;
            return leftTipSuckingHeight_ + leftTipComponent.height;
        }
    } else {
        if ((rightTipComponent.box >= 67) && (rightTipComponent.box <= 86)) {
            return rightTipSuckingHeight_;
        } else {
            //return pickUpAreaOffset.z + rightTipComponent.height;
            return rightTipSuckingHeight_ + rightTipComponent.height;
        }
    }
}

Offset PlaceController::getCompPlaceCoordinates(TIP usedTip) {
    Offset temp;
    switch (usedTip) {
    case LEFT_TIP:
        temp.x = leftTipComponent.destX + tip1Offset.x;
        temp.y = leftTipComponent.destY + tip1Offset.y;
        temp.z = leftTipPCBHeight_ + leftTipComponent.height + 10;
        temp.rot = leftTipComponent.rotation;   // Not used
        break;
    case RIGHT_TIP:
        temp.x = rightTipComponent.destX + tip2Offset.x;
        temp.y = rightTipComponent.destY + tip2Offset.y;
        temp.z = rightTipPCBHeight_ + rightTipComponent.height + 10;
        temp.rot = rightTipComponent.rotation;   // Not used
        break;
    }
    return temp;
}

float PlaceController::getCompPlaceHeight(TIP usedTip) {
    if(pickRelQR_){
        return largeBoxHeight_;
    }

    if(usedTip == TIP::LEFT_TIP) {
        ROS_INFO("PCBOrigin: %f, CompHeight: %f",leftTipPCBHeight_ ,leftTipComponent.height);
        return leftTipPCBHeight_ + leftTipComponent.height;
    } else {
        ROS_INFO("PCBOrigin: %f, CompHeight: %f",rightTipPCBHeight_ ,rightTipComponent.height);
        return rightTipPCBHeight_ + rightTipComponent.height;
    }
}

Offset PlaceController::getCameraProjectionOffset(){
    return camera_projection_offset_;
}

/******************************************************
* Set correction offset for pickup
******************************************************/
void PlaceController::setPickUpCorrectionOffset(float xDiff, float yDiff, float rotDiff) {
    PickUpCorrection.x = xDiff;
    PickUpCorrection.y = yDiff;
    PickUpCorrection.rot = rotDiff;
}


/******************************************************
* Getter functions - dimensions, boxNum
******************************************************/
float PlaceController::getComponentHeight(TIP usedTip) {
    if(usedTip == TIP::LEFT_TIP) {
        return leftTipComponent.height;
    } else {
        return rightTipComponent.height;
    }
}

float PlaceController::getComponentLenth(TIP usedTip) {
    if(usedTip == TIP::LEFT_TIP) {
        return leftTipComponent.length;
    } else {
        return rightTipComponent.length;
    }
}

float PlaceController::getComponentWidth(TIP usedTip) {
    if(usedTip == TIP::LEFT_TIP) {
        return leftTipComponent.width;
    } else {
        return rightTipComponent.width;
    }
}

int PlaceController::getBoxNumber(TIP usedTip) {
    if(usedTip == TIP::LEFT_TIP) {
        return leftTipComponent.box;
    } else {
        return rightTipComponent.box;
    }
}

pap_vision::VISION PlaceController::getFinderType(TIP usedTip) {
    if(usedTip == TIP::LEFT_TIP) {
        return leftTipComponent.finderType;
    } else {
        return rightTipComponent.finderType;
    }
}

/******************************************************
* Transforms angle to stepps
******************************************************/
int PlaceController::angleToSteps(float angle) {
    int steps = round(angle / 1.8);
    steps = steps%200;				// 200 steps = full rotation
    ROS_INFO("Angle: %f, Steps: %d", angle, steps);
    return steps;
}

/******************************************************
* Update placement data
******************************************************/
void PlaceController::updatePlacementData(ComponentPlacerData& data, TIP usedTip) {

    ComponentPlacerData* compToUpdate;
    if(usedTip == TIP::LEFT_TIP) {
        compToUpdate = &leftTipComponent;
        std::cerr << "PlaceController: Placement data for left tip is updated" << std::endl;
    } else {
        compToUpdate = &rightTipComponent;
        std::cerr << "PlaceController: Placement data for right tip is updated" << std::endl;
    }

    compToUpdate->box = data.box;
    compToUpdate->destX = data.destX;
    compToUpdate->destY = data.destY;
    compToUpdate->height = data.height;
    compToUpdate->length = data.length;
    compToUpdate->width = data.width;
    compToUpdate->rotation = data.rotation;
    compToUpdate->tapeX = data.tapeX;
    compToUpdate->tapeY = data.tapeY;
    compToUpdate->tapeRot = data.tapeRot;
    compToUpdate->isWaiting = true;

    // Select corresponding vision
    if(compToUpdate->box <= 66) {
        compToUpdate->finderType = pap_vision::START_CHIP_FINDER;
        std::cerr << "Chiper finder selected" << std::endl;
    } else {
        compToUpdate->finderType = pap_vision::START_TAPE_FINDER;
        std::cerr << "Tape finder selected" << std::endl;
    }
}


void PlaceController::setDispenserVel(double vel){
    corr_dispenser_vel_ = vel;
}

double PlaceController::getDispenserVel(){
    return corr_dispenser_vel_;
}


