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

	// Move:  	z = 30.0;

	// Rough offset values - need to be refined
	pcbOriginOffset.x = 282;
	pcbOriginOffset.y = 149;
	pcbOriginOffset.z = 0.1;
	pickUpAreaOffset.x = 109;
	pickUpAreaOffset.y = 261;
	pickUpAreaOffset.z = 0.1;
	cameraBottomOffset.x = 236;
	cameraBottomOffset.y = 197;
	cameraBottomOffset.z = 0.1;

	// Offsets relative to camera
	tipRightOffset.x = -50;
	tipRightOffset.y = 50;
	tipRightOffset.z = 0;
	tipLeftOffset.x = -50;
	tipLeftOffset.y = 5;
	tipLeftOffset.z = 0;
	dispenserTipOffset.x = -40;
	dispenserTipOffset.y = 25;
	dispenserTipOffset.z = 0;

	// Correction offsets
	PickUpCorrection.x = 5;
	PickUpCorrection.y = 5;
	PickUpCorrection.z = 5;
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

//const Offset LargeBoxOffsetTable[10];
//const Offset TapeOffsetTable[15];


Offset PlaceController::getBoxCoordinates() {
	Offset temp;
	temp.x = pickUpAreaOffset.x + SmallBoxOffsetTable[currentComponent.box].x;
	temp.y = pickUpAreaOffset.y + SmallBoxOffsetTable[currentComponent.box].y;
	temp.z = pickUpAreaOffset.z;
	return temp;
};

Offset PlaceController::getCompPickUpCoordinates() {
	Offset temp;
	switch (tip) {
	case LEFT_TIP:
		temp.x = pickUpAreaOffset.x + PickUpCorrection.x + SmallBoxOffsetTable[currentComponent.box].x + tipLeftOffset.x;
		temp.y = pickUpAreaOffset.y + PickUpCorrection.y + SmallBoxOffsetTable[currentComponent.box].y + tipLeftOffset.y;
		temp.z = pickUpAreaOffset.z + tipLeftOffset.z;
		temp.rot = PickUpCorrection.rot;
		break;
	case RIGHT_TIP:
		temp.x = pickUpAreaOffset.x + PickUpCorrection.x + SmallBoxOffsetTable[currentComponent.box].x + tipRightOffset.x;
		temp.y = pickUpAreaOffset.y + PickUpCorrection.y + SmallBoxOffsetTable[currentComponent.box].y + tipRightOffset.y;
		temp.z = pickUpAreaOffset.z + tipRightOffset.z;
		temp.rot = PickUpCorrection.rot;
		break;
	}
	return temp;
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
		temp.x = pcbOriginOffset.x + currentComponent.destX + PlaceCorrection.x + tipLeftOffset.x;
		temp.y = pcbOriginOffset.y + currentComponent.destY + PlaceCorrection.y + tipLeftOffset.y;
		temp.z = pcbOriginOffset.z + currentComponent.height + tipLeftOffset.z;
		temp.rot = currentComponent.rotation + PlaceCorrection.rot;
		break;
	case RIGHT_TIP:
		temp.x = pcbOriginOffset.x + currentComponent.destX + PlaceCorrection.x + tipRightOffset.x;
		temp.y = pcbOriginOffset.y + currentComponent.destY + PlaceCorrection.y + tipRightOffset.y;
		temp.z = pcbOriginOffset.z + currentComponent.height + tipRightOffset.z;
		temp.rot = currentComponent.rotation + PlaceCorrection.rot;
		break;
	}
	return temp;
};

void PlaceController::setPickUpCorrectionOffset(float xDiff, float yDiff, float rotDiff) {
	PickUpCorrection.x = xDiff;
	PickUpCorrection.y = yDiff;
	PickUpCorrection.rot = rotDiff;
}

void PlaceController::setPlaceCorrectionOffset(float xDiff, float yDiff, float rotDiff) {
	PlaceCorrection.x = xDiff;
	PlaceCorrection.y = yDiff;
	PlaceCorrection.rot = rotDiff;
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








void PlaceController::systemCalibration() {

};

bool PlaceController::getCalibrationStatus() {
	return calibrationStatus;
};

void PlaceController::updatePlacementData(ComponentPlacerData * data) {
	currentComponent.box = data->box;
	currentComponent.destX = data->destX;
	currentComponent.destY = data->destY;
	currentComponent.height = data->height;
	currentComponent.length = data->length;
	currentComponent.width = data->width;
	currentComponent.rotation = data->rotation;
};
