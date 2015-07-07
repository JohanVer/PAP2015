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
	calibrationStatus = true;

	pcbOriginOffset.x = 100;
	pcbOriginOffset.y = 160;
	pcbOriginOffset.z = 50;
	pickUpAreaOffset.x = 15;
	pickUpAreaOffset.y = 350;
	pickUpAreaOffset.z = 60;
	cameraBottomOffset.x = 0;
	cameraBottomOffset.y = 0;
	cameraBottomOffset.z = 0;

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

//const Offset LargeBoxOffsetTable[10];
//const Offset TapeOffsetTable[15];


void PlaceController::systemCalibration() {

};

float PlaceController::getComponentLenth() {
	return currentComponent.length;
}

float PlaceController::getComponentWidth() {
	return currentComponent.width;
}

int PlaceController::getSelectedFinder() {
	return 3;	// Chipfinder = 3, smallFinder = 4, tapeFinder = 5
};

bool PlaceController::getSelcetedTip() {
	return true;	// Left tip
};

int PlaceController::getBoxNumber() {
	return currentComponent.box;
};


bool PlaceController::getCalibrationStatus() {
	return calibrationStatus;
};

Offset PlaceController::getBoxCoordinates() {
	Offset temp;
	temp.x = pickUpAreaOffset.x + componentOffset.x + SmallBoxOffsetTable[currentComponent.box].x;
	temp.y = pickUpAreaOffset.y + componentOffset.y + SmallBoxOffsetTable[currentComponent.box].y;
	temp.z = pickUpAreaOffset.z;	// Add offset of corresponding tip, cylinder offset
	return temp;
};

void PlaceController::setCompOffset(float xDiff, float yDiff, float rotDiff) {
	componentOffset.x = xDiff;
	componentOffset.y = yDiff;
	componentRotDiff = rotDiff;
}

Offset PlaceController::getPCBCoordinates() {
	Offset temp;
	temp.x = pcbOriginOffset.x + currentComponent.destX;
	temp.y = pcbOriginOffset.y + currentComponent.destY;
	temp.z = pcbOriginOffset.z;		// Add offset of corresponding tip, cylinder offset
	return temp;
};

void PlaceController::updatePlacementData(ComponentPlacerData * data) {
	currentComponent.box = data->box;
	//currentComponent.box = 12;
	currentComponent.destX = data->destX;
	currentComponent.destY = data->destY;
	currentComponent.height = data->height;
	currentComponent.length = data->length;
	currentComponent.width = data->width;
	currentComponent.rotation = data->rotation;
};
