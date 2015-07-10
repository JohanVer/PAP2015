#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <vector>
#include <cmath>
#include "pap_common/Task.h"
#include "pap_common/Status.h"
#include "../../../pap_common/include/pap_common/task_message_def.h"
#include "../../../pap_common/include/pap_common/status_message_def.h"
#include <stdio.h>
#include <cstdio>

/*****************************************************************************
** Namespace
*****************************************************************************/
using namespace std;
/*****************************************************************************
** Class implementation
*****************************************************************************/

class ComponentPlacerData {
public:
	float destX;
	float destY;
	float rotation;
	float length, width, height;
	int box;

	ComponentPlacerData() {
		destX = 0.0;
		destY = 0.0;
		rotation = 0.0;
		length = 0.0;
		width = 0.0;
		height = 0.0;
		box = 0;
	}
private:
};

class controllerStatus {
public:
	controllerStatus() {
		error = false;
		energized = false;
		positionReached = false;
		failed = false;
	}

	bool error;
	bool energized;
	bool positionReached;
	bool failed;
private:
};

class Offset {
public:
	float x;
	float y;
	float z;
	float rot;
private:
};


class PlaceController {
public:
	PlaceController();
	~PlaceController();

	Offset getBoxCoordinates();
	Offset getCompPickUpCoordinates();
	Offset getPCBCalibCoordinates();
	Offset getPCBCompCoordinates();
	Offset getCompPlaceCoordinates();
	float getComponentLenth();
	float getComponentWidth();

	int selectFinder();
	int selectTip();
	void setPickUpCorrectionOffset(float xDiff, float yDiff, float rotDiff);
	void setPlaceCorrectionOffset(float xDiff, float yDiff, float rotDiff);


	void systemCalibration();

	void updatePlacementData(ComponentPlacerData *data);
	bool getCalibrationStatus();
	int getBoxNumber();




private:
	ComponentPlacerData currentComponent;
	bool calibrationStatus;

	enum TIP {
		LEFT_TIP, RIGHT_TIP
	} tip;

	// These offsets are relative to camera
	Offset tipRightOffset, tipLeftOffset, dispenserTipOffset;

	// These offsets are relative to homing position
	Offset pcbOriginOffset, pickUpAreaOffset;
	Offset cameraBottomOffset;

	// Correction feedback from vision for pick-up and place
	Offset PickUpCorrection, PlaceCorrection;
	int PickUpRotCorrection, PlaceRotCorrection;
};
