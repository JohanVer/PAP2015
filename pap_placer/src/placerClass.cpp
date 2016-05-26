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

/*****************************************************************************
 ** Constructor
 *****************************************************************************/
PlaceController::PlaceController() {
    corr_dispenser_vel_ = 0.0;

	MovingHeight_ = 45.0;

	idleCoordinates_.x = 5.0;
	idleCoordinates_.y = 5.0;
	idleCoordinates_.z = 0.0;

	// Height for sucking a component (normal chip, not a tape)
	suckingHeight_ = 20.2;
    largeBoxHeight_ = 18.2;

    // Absolut offests
	pcbOriginOffset.x = 300;
	pcbOriginOffset.y = 145;
    pcbOriginOffset.z = 25.6;
	pickUpAreaOffset.x = 108.42;	// + tape_x -> 449.85 = max.x destination
	pickUpAreaOffset.y = 261;
	pickUpAreaOffset.z = suckingHeight_;
    cameraBottomOffset.x = 236;
    cameraBottomOffset.y = 195.65;
    cameraBottomOffset.z = 8.17;

    //Relative offsets to camera
	tip2Offset.x = -94.08;
	tip2Offset.y = 64.709;
    tip2Offset.z = 50;
    tip1Offset.x = -95;
    tip1Offset.y = 0;
    tip1Offset.z = 50;
    dispenserTipOffset.x = -52.499;
    dispenserTipOffset.y = 38.988;
    dispenserTipOffset.z = 36;

	// Calibration offsets - QR Code positions
	SLOT_QR_Offset_.x = 109.92;
	SLOT_QR_Offset_.y = 261.008;
	SLOT_QR_Offset_.z = pickUpAreaOffset.z;
	PCB_QR_Offset_.x = 293.04;
	PCB_QR_Offset_.y = 133.027;
	PCB_QR_Offset_.z = pcbOriginOffset.z;
	TAPE_QR_Offset_.x = 389.554;
	TAPE_QR_Offset_.y = 110.429;
	TAPE_QR_Offset_.z = pickUpAreaOffset.z;
    BottomCam_QR_Offset_.x = 218;
    BottomCam_QR_Offset_.y = 177;
	BottomCam_QR_Offset_.z = 20.15;

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

	// Correction offsets
	PickUpCorrection.x = 0;
	PickUpCorrection.y = 0;
	PickUpCorrection.z = 0;
	PickUpCorrection.rot = 0.0;

	// Calibration offsets
//	camClibrationOffset_.x = 0.0;
//	camClibrationOffset_.y = 0.0;
//	tip1ClibrationOffset_.x = 0.0;
//	tip1ClibrationOffset_.y = 0.0;

	// Default tip/visual finder
    finderType = pap_vision::START_CHIP_FINDER;
	pickRelQR_ = false;
}

PlaceController::~PlaceController() {
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
    temp.z = suckingHeight_ + 10.0; // Sucking Height plus 10 mm

    if(usedTip == TIP::LEFT_TIP) {
        currentComp = &leftTipComponent;
        temp.x = tip1Offset.x;
        temp.y = tip1Offset.y;
    } else {
        currentComp = &rightTipComponent;
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

    if ((currentComponent.box >= 67) && (currentComponent.box <= 86)) {
        return pickUpAreaOffset.z;
    } else {
        if(usedTip == TIP::LEFT_TIP) {
            return pickUpAreaOffset.z + leftTipComponent.height;
        } else {
            return pickUpAreaOffset.z + rightTipComponent.height;
        }
    }
}

Offset PlaceController::getCompPlaceCoordinates(TIP usedTip) {
    Offset temp;
    switch (usedTip) {
    case LEFT_TIP:
        temp.x = leftTipComponent.destX + tip1Offset.x;
        temp.y = leftTipComponent.destY + tip1Offset.y;
        temp.z = 45.0;
        temp.rot = leftTipComponent.rotation;   // Not used
        break;
    case RIGHT_TIP:
        temp.x = rightTipComponent.destX + tip2Offset.x;
        temp.y = rightTipComponent.destY + tip2Offset.y;
        temp.z = 45.0;
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
        ROS_INFO("PCBOrigin: %f, CompHeight: %f",pcbOriginOffset.z ,leftTipComponent.height);
        return pcbOriginOffset.z + leftTipComponent.height;
    } else {
        ROS_INFO("PCBOrigin: %f, CompHeight: %f",pcbOriginOffset.z ,rightTipComponent.height);
        return pcbOriginOffset.z + rightTipComponent.height;
    }
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
    } else {
        compToUpdate->finderType = pap_vision::START_TAPE_FINDER;
    }
}



void PlaceController::setTip1Offset(float xDiff, float yDiff) {

	tip1Offset.x = tip1Offset.x + camClibrationOffset_.x + xDiff;
	tip1Offset.y = tip1Offset.y + camClibrationOffset_.y + yDiff;
	ROS_INFO("x/yDiff:: x:%f y:%f", xDiff, yDiff);
	ROS_INFO("camClibrationOffset_: x:%f y:%f", camClibrationOffset_.x,
			camClibrationOffset_.y);
	ROS_INFO("tip1offset: x:%f y:%f", tip1Offset.x, tip1Offset.y);
}

void PlaceController::setTip2Offset(float xDiff, float yDiff) {
	tip2Offset.x = tip2Offset.x + xDiff;
	tip2Offset.y = tip2Offset.y + yDiff;
}

void PlaceController::setDispenserVel(double vel){
    corr_dispenser_vel_ = vel;
}

double PlaceController::getDispenserVel(){
    return corr_dispenser_vel_;
}


