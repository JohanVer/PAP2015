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
#include "../include/pap_placer/placerClass.hpp"
#include "../include/pap_placer/offsetTable.hpp"

/*****************************************************************************
 ** Implementation
 *****************************************************************************/
PlaceController::PlaceController() {
	MovingHeight_ = 45.0;

	idleCoordinates_.x = 5.0;
	idleCoordinates_.y = 5.0;
	idleCoordinates_.z = 0.0;

	// Height for sucking a component (normal chip, not a tape)
	suckingHeight_ = 20.2;
	largeBoxHeight_ = 18.2;

	// Rough offset values - need to be refined
	pcbOriginOffset.x = 300;
	pcbOriginOffset.y = 145;
	pcbOriginOffset.z = 27.0;//25.6;
	pickUpAreaOffset.x = 108.42;	// + tape_x -> 449.85 = max.x destination
	pickUpAreaOffset.y = 261;
	pickUpAreaOffset.z = suckingHeight_;
	cameraBottomOffset.x = 237.6; //239.15; //238.74+1.0;
	cameraBottomOffset.y = 195.65; //195.21+1.0;
	cameraBottomOffset.z = 23.17-15; //13;

	// Offsets relative to camera
	tip2Offset.x = -94.08;
	tip2Offset.y = 64.709;
	tip2Offset.z = 45+15-10;
	tip1Offset.x = -92,817; //-93,817;
	tip1Offset.y = 2,598;
	tip1Offset.z = 45+15-10;
	dispenserTipOffset.x = -53.42;
	dispenserTipOffset.y = 37.89;
	dispenserTipOffset.z = 49-10;

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
	BottomCam_QR_Offset_.x = 292.09;
	BottomCam_QR_Offset_.y = 259.35;
	BottomCam_QR_Offset_.z = 20.15;
	CHECKERBOARD_1_Offset_.x = 387;
	CHECKERBOARD_1_Offset_.y = 80;
	CHECKERBOARD_1_Offset_.z = pickUpAreaOffset.z;

	// Correction offsets
	PickUpCorrection.x = 0;
	PickUpCorrection.y = 0;
	PickUpCorrection.z = 0;
	PickUpCorrection.rot = 0.0;
	PlaceCorrection.x = 0;
	PlaceCorrection.y = 0;
	PlaceCorrection.z = 0;
	PlaceCorrection.rot = 0.0;

	// Calibration offsets
	camClibrationOffset_.x = 0.0;
	camClibrationOffset_.y = 0.0;
	tip1ClibrationOffset_.x = 0.0;
	tip1ClibrationOffset_.y = 0.0;

	// Default tip/visual finder
	tip = LEFT_TIP;
	visualFinder = 2;	//(ChipFinder)
	pickRelQR_ = false;
};

PlaceController::~PlaceController() {
};

Offset PlaceController::getBoxCoordinates() {

	Offset temp;
	temp.x = pickUpAreaOffset.x;
	temp.y = pickUpAreaOffset.y;
	temp.z = pickUpAreaOffset.z;
	if (currentComponent.box < 67) { //47
		temp.x += BoxOffsetTable[currentComponent.box].x;
		temp.y += BoxOffsetTable[currentComponent.box].y;
	} else if ((currentComponent.box >= 67) && (currentComponent.box <= 86)) {
		// Its a tape
		if(TapeOffsetTable[currentComponent.box - 67].x+temp.x < 450.0) temp.x += TapeOffsetTable[currentComponent.box - 67].x; else temp.x = 449.9;
		temp.y += TapeOffsetTable[currentComponent.box - 67].y;
	}

	return temp;
};

Offset PlaceController::getCompPickUpCoordinates() {
	Offset temp;
	temp.z = suckingHeight_ + 10.0; // Sucking Height plus 10 mm for safety

	if (currentComponent.box < 67) {
		temp.x = pickUpAreaOffset.x
				+ BoxOffsetTable[currentComponent.box].x
				+ PickUpCorrection.x;
		temp.y = pickUpAreaOffset.y
				+ BoxOffsetTable[currentComponent.box].y
				+ PickUpCorrection.y;
		temp.rot = PickUpCorrection.rot;
	} else if ((currentComponent.box >= 67) && (currentComponent.box <= 86)) {
		// Its a tape
		temp.x = currentComponent.tapeX;
		temp.y = currentComponent.tapeY;
		temp.rot = currentComponent.tapeRot;
	}

	switch (tip) {
	case LEFT_TIP:
		temp.x += (tip1Offset.x - camClibrationOffset_.x
				+ tip1ClibrationOffset_.x +0.532 );
		temp.y += (tip1Offset.y - camClibrationOffset_.y
				+ tip1ClibrationOffset_.y -0.374);
		break;
	case RIGHT_TIP:
		temp.x += tip2Offset.x;
		temp.y += tip2Offset.y;
		break;
	}
	return temp;
}
;

float PlaceController::getCompSuckingHeight() {
	if(pickRelQR_){
		return largeBoxHeight_;
	}

	if ((currentComponent.box >= 67) && (currentComponent.box <= 86)) {
		return pickUpAreaOffset.z;
	} else {
		return pickUpAreaOffset.z + currentComponent.height;
	}
}
;

void PlaceController::setPickUpCorrectionOffset(float xDiff, float yDiff,
		float rotDiff) {
	PickUpCorrection.x = xDiff;
	PickUpCorrection.y = yDiff;
	PickUpCorrection.rot = rotDiff;
}

Offset PlaceController::getBottomCamCoordinates() {
	Offset temp;
	temp.x = cameraBottomOffset.x + camClibrationOffset_.x;
	temp.y = cameraBottomOffset.y + camClibrationOffset_.y;
	temp.z = cameraBottomOffset.z;
	return temp;
}

Offset PlaceController::getPCBCalibCoordinates() {
	return pcbOriginOffset;
}

Offset PlaceController::getPCBCompCoordinates() {
	Offset temp;
	temp.x = pcbOriginOffset.x + currentComponent.destX;
	temp.y = pcbOriginOffset.y + currentComponent.destY;
	temp.z = pcbOriginOffset.z + currentComponent.height;
	temp.rot = pcbOriginOffset.rot + currentComponent.rotation;
	return temp;
}

Offset PlaceController::getCompPlaceCoordinates() {
	Offset temp;
	switch (tip) {
	case LEFT_TIP:
		temp.x = currentComponent.destX
				+ (tip1Offset.x - camClibrationOffset_.x
						+ tip1ClibrationOffset_.x) + 0.7;
		temp.y = currentComponent.destY
				+ (tip1Offset.y - camClibrationOffset_.y
						+ tip1ClibrationOffset_.y) - 0.4;
		temp.z = 45.0;
		temp.rot = currentComponent.rotation;
		break;
	case RIGHT_TIP:
		temp.x = currentComponent.destX + tip2Offset.x;
		temp.y = currentComponent.destY + tip2Offset.y;
		temp.z = 45.0;
		temp.rot = currentComponent.rotation;
		break;
	}
	return temp;
}
;

float PlaceController::getCompPlaceHeight() {
	if(pickRelQR_){
		return largeBoxHeight_;
	}

	ROS_INFO("PCBOrigin: %f, CompHeight: %f",pcbOriginOffset.z ,currentComponent.height);
	return pcbOriginOffset.z + currentComponent.height;
}
;


// NOT USED!
void PlaceController::setPlaceCorrectionOffset(float xDiff, float yDiff,
		float rotDiff) {
	PlaceCorrection.x = xDiff;
	PlaceCorrection.y = yDiff;
	PlaceCorrection.rot = rotDiff;
	ROS_INFO("Placer - Place correction: x: %f y: %f", xDiff, yDiff);
}

void PlaceController::setBottomCamCorrectionOffset(float xDiff, float yDiff) {
	cameraBottomOffset.x = cameraBottomOffset.x + xDiff;
	cameraBottomOffset.y = cameraBottomOffset.y + yDiff;
}

Offset PlaceController::getTip1Coordinates() {
	Offset tip1Coordinate;
	tip1Coordinate.x = cameraBottomOffset.x + tip1Offset.x + tip1ClibrationOffset_.x;
	tip1Coordinate.y = cameraBottomOffset.y + tip1Offset.y + tip1ClibrationOffset_.y;
	tip1Coordinate.z = tip1Offset.z;
	return tip1Coordinate;
}

Offset PlaceController::getTip2Coordinates() {
	Offset tip2Coordinate;
	tip2Coordinate.x = cameraBottomOffset.x + tip2Offset.x;
	tip2Coordinate.y = cameraBottomOffset.y + tip2Offset.y;
	tip2Coordinate.z = tip2Offset.z;
	return tip2Coordinate;
}

Offset PlaceController::getDispenserCoordinates() {
	Offset dispenserCoordinate;
	dispenserCoordinate.x = cameraBottomOffset.x + dispenserTipOffset.x;
	dispenserCoordinate.y = cameraBottomOffset.y + dispenserTipOffset.y;
	dispenserCoordinate.z = dispenserTipOffset.z;
	return dispenserCoordinate;
}

void PlaceController::setDispenserOffset(float xDiff, float yDiff) {
	dispenserTipOffset.x = dispenserTipOffset.x + xDiff;
	dispenserTipOffset.y = dispenserTipOffset.y + yDiff;
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

float PlaceController::getComponentLenth() {
	return currentComponent.length;
}

float PlaceController::getComponentWidth() {
	return currentComponent.width;
}

int PlaceController::getBoxNumber() {
	return currentComponent.box;
}
;

int PlaceController::selectTip() {
	return tip;	// Left tip
};

void PlaceController::updatePlacementData(ComponentPlacerData * data) {
	currentComponent.box = data->box;
	currentComponent.destX = data->destX;
	currentComponent.destY = data->destY;
	currentComponent.height = data->height;
	currentComponent.length = data->length;
	currentComponent.width = data->width;
	currentComponent.rotation = data->rotation;
	currentComponent.tapeX = data->tapeX;
	currentComponent.tapeY = data->tapeY;
	currentComponent.tapeRot = data->tapeRot;

	// Select corresponding visual finder
	if(currentComponent.box <= 46) {
		visualFinder = 1;	// SmallFinder
	} else if (currentComponent.box <= 66) {
		visualFinder = 2;	// ChipFinder
	} else {
		visualFinder = 3;	// TapeFinder
	}
};
