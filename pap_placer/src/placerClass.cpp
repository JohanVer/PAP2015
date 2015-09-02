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


/*****************************************************************************
 ** Implementation
 *****************************************************************************/
PlaceController::PlaceController() {



	MovingHeight_ = 45.0;

	idleCoordinates_.x = 1.0;
	idleCoordinates_.y = 1.0;
	idleCoordinates_.z = 1.0;

	// Rough offset values - need to be refined
	pcbOriginOffset.x = 300;
	pcbOriginOffset.y = 145;
	pcbOriginOffset.z = 22.0;
	pickUpAreaOffset.x = 109;
	pickUpAreaOffset.y = 261;
	pickUpAreaOffset.z = 20.1;
	cameraBottomOffset.x = 238.74;
	cameraBottomOffset.y = 195.21;
	cameraBottomOffset.z = 13;

	// Offsets relative to camera
	tip2Offset.x = -94.08;
	tip2Offset.y = 64.709;
	tip2Offset.z = 45;
	tip1Offset.x = -94.08;
	tip1Offset.y = -1,8;
	tip1Offset.z = 45;
	dispenserTipOffset.x = -53.42;
	dispenserTipOffset.y = 37,89;
	dispenserTipOffset.z = 49;

	// 101 , 338 // 600, 336

	// Correction offsets
	PickUpCorrection.x = 0;
	PickUpCorrection.y = 0;
	PickUpCorrection.z = 0;
	PickUpRotCorrection = 0;
	PlaceCorrection.x = 0;
	PlaceCorrection.y = 0;
	PlaceCorrection.z = 0;
	PlaceRotCorrection = 0;

	// selceted Tip for placing current component
	tip = LEFT_TIP;
};
PlaceController::~PlaceController(){

};

const Offset SmallBoxOffsetTable[59] = 	{	{0.0, 0.0}, {16.66, 0.0}, {33.32, 0.0}, {49.98, 0.0}, {66.64, 0.0}, {83.30, 0.0}, {99.96, 0.0}, {116.62, 0.0}, {133.28, 0.0}, {149.94, 0.0}, {166.60, 0.0},
											{166.60, -14.0}, {149.94, -14.0}, {133.28, -14.0}, {116.62, -14.0}, {99.96, -14.0}, {83.30, -14.0}, {66.64, -14.0}, {49.98, -14.0}, {33.32, -14.0}, {16.66, -14.0}, {0, -14.0},
											{0.0, -28.0}, {16.66, -28.0}, {33.32, -28.0}, {49.98, -28.0}, {66.64, -28.0},
											{66.64, -42.0}, {49.98, -42.0}, {33.32, -42.0}, {16.66, -42.0}, {0.0, -42.0},
											{0, -56.0}, {16.66, -56.0}, {33.32, -56.0}, {49.98, -56.0}, {66.64, -56.0},
											{66.64, -70.0}, {49.98, -70.0}, {33.32, -70.0}, {16.66, -70.0}, {0.0, -70.0},
											{0, -84.0}, {16.66, -84.0}, {33.32, -84.0}, {49.98, -84.0}, {66.64, -84.0},
											{187.65, -2.0}, {187.65, -18.6}, {187.65, -35.2}, {187.65, -51.8}, {187.65, -68.4}, {187.65, -85},
											{204.25, -2.0}, {204.25, -18.6}, {204.25, -35.2}, {204.25, -51.8}, {204.25, -68.4}, {204.25, -85}
								 	 	 };

const Offset MiddleBoxOffsetTable[12] = {	{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
											{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
											{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}
										};
const Offset LargeBoxOffsetTable[8] = 	{	{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
											{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}
										};

const Offset TapeOffsetTable[20] = {	{340,9, -40.0}, {340,9, -51.0}, {340,9, -62.0}, {340,9, -73.0}, {340,9, -84.0},
						{340,9, -95.0}, {340,9, -106.0}, {340,9, -117.0}, {340,9, -128.0}, {340,9, -139.0},
						{340,9, -150.0}, {340,9, -161.0}, {340,9, -172.0}, {340,9, -183.0}, {340,9, -194.0},
						{340,9, -205.0}, {340,9, -216.0}, {340,9, -227.0}, {340,9, -238.0}, {340,9, -249.0}};



Offset PlaceController::getBoxCoordinates() {

	Offset temp;
	temp.x = pickUpAreaOffset.x;
	temp.y = pickUpAreaOffset.y;
	temp.z = pickUpAreaOffset.z;
	if(currentComponent.box < 59) { //47
		temp.x += SmallBoxOffsetTable[currentComponent.box].x;
		temp.y += SmallBoxOffsetTable[currentComponent.box].y;
	//} else if((currentComponent.box >= 47) && (currentComponent.box < 59)) {
		//temp.x += MiddleBoxOffsetTable[currentComponent.box - 47].x;
		//temp.y += MiddleBoxOffsetTable[currentComponent.box - 47].y;
	} else if((currentComponent.box >= 59) && (currentComponent.box < 67)) {
		temp.x += LargeBoxOffsetTable[currentComponent.box - 59].x;
		temp.y += LargeBoxOffsetTable[currentComponent.box - 59].y;
	} else if((currentComponent.box >= 67) && (currentComponent.box <= 86)) {
		// Its a tape
		temp.x += TapeOffsetTable[currentComponent.box - 67].x;
		temp.y += TapeOffsetTable[currentComponent.box - 67].y;
	}

	return temp;
};

Offset PlaceController::getCompPickUpCoordinates() {
	Offset temp;
	temp.z = 20.2;	// Same height for all box pickups

	if(currentComponent.box < 59) { //47
		temp.x = pickUpAreaOffset.x + SmallBoxOffsetTable[currentComponent.box].x + PickUpCorrection.x;
		temp.y = pickUpAreaOffset.y + SmallBoxOffsetTable[currentComponent.box].y + PickUpCorrection.y;
		temp.rot = PickUpCorrection.rot;
	//} else if((currentComponent.box >= 47) && (currentComponent.box < 59)) {
		//temp.x += MiddleBoxOffsetTable[currentComponent.box - 47].x;
		//temp.y += MiddleBoxOffsetTable[currentComponent.box - 47].y;
	//} else if((currentComponent.box >= 59) && (currentComponent.box < 67)) {
		//temp.x = LargeBoxOffsetTable[currentComponent.box - 59].x;
		//temp.y = LargeBoxOffsetTable[currentComponent.box - 59].y;
	} else if((currentComponent.box >= 67) && (currentComponent.box <= 86)) {
		// Its a tape
		temp.x = currentComponent.tapeX;
		temp.y = currentComponent.tapeY;
		temp.rot = currentComponent.tapeRot;
		temp.z = 19.2;
	}

	switch (tip) {
	case LEFT_TIP:
		temp.x += (tip1Offset.x + camClibrationOffset_.x + tip1ClibrationOffset_.x);
		temp.y += (tip1Offset.y - camClibrationOffset_.y + tip1ClibrationOffset_.y);
		break;
	case RIGHT_TIP:
		temp.x += tip2Offset.x;
		temp.y += tip2Offset.y;
		break;
	}
	return temp;
};

float PlaceController::getCompSuckingHeight() {
	return pickUpAreaOffset.z + currentComponent.height;
};

void PlaceController::setPickUpCorrectionOffset(float xDiff, float yDiff, float rotDiff) {
	PickUpCorrection.x = xDiff;
	PickUpCorrection.y = yDiff;
	PickUpCorrection.rot = rotDiff;
}


Offset PlaceController::getBottomCamCoordinates() {
	return cameraBottomOffset;
};

Offset PlaceController::getPCBCalibCoordinates() {
	return pcbOriginOffset;
};

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

		temp.x = currentComponent.destX + PlaceCorrection.x + (tip1Offset.x + camClibrationOffset_.x + tip1ClibrationOffset_.x);
		temp.y = currentComponent.destY + PlaceCorrection.y + (tip1Offset.y - camClibrationOffset_.y + tip1ClibrationOffset_.y);
		temp.z = pcbOriginOffset.z + tip1Offset.z;
		temp.rot = currentComponent.rotation + PlaceCorrection.rot;
		break;
	case RIGHT_TIP:
		temp.x = currentComponent.destX + PlaceCorrection.x + tip2Offset.x;
		temp.y = currentComponent.destY + PlaceCorrection.y + tip2Offset.y;
		temp.z = pcbOriginOffset.z + tip2Offset.z;
		temp.rot = currentComponent.rotation + PlaceCorrection.rot;
		break;
	}
	return temp;
};

float PlaceController::getCompPlaceHeight() {
	return pcbOriginOffset.z + currentComponent.height;
};

void PlaceController::setPlaceCorrectionOffset(float xDiff, float yDiff, float rotDiff) {
	PlaceCorrection.x = xDiff;
	PlaceCorrection.y = yDiff;
	PlaceCorrection.rot = rotDiff;
}

void PlaceController::setBottomCamCorrectionOffset(float xDiff, float yDiff) {
	cameraBottomOffset.x = cameraBottomOffset.x + xDiff;
	cameraBottomOffset.y = cameraBottomOffset.y + yDiff;
}

Offset PlaceController::getTip1Coordinates() {
	Offset tip1Coordinate;
	tip1Coordinate.x = cameraBottomOffset.x + tip1Offset.x;
	tip1Coordinate.y = cameraBottomOffset.y + tip1Offset.y;
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
	ROS_INFO("camClibrationOffset_: x:%f y:%f", camClibrationOffset_.x, camClibrationOffset_.y);
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
};


// TODO: Choose tip according to component type and position!
int PlaceController::selectFinder() {
	return 3;	// Chipfinder = 3, smallFinder = 4, tapeFinder = 5
};
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
};
