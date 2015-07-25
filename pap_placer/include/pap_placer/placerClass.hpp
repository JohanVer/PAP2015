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

class dispenseInfos{
public:
	dispenseInfos(){
		xPos = 0.0;
		yPos = 0.0;
		xPos2 = 0.0;
		yPos2 = 0.0;
		type = Point;
		rotation = 0.0;
		velocity = 0.0;
		time = 0.0;
	}

	enum dispensePadType {
		Point, Long
	};

	int type;
	float xPos, yPos, xPos2, yPos2;
	float rotation;
	float velocity;
	float time;
};

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

	dispenseInfos dispenseTask;
	// These offsets are relative to camera
	Offset tip2Offset, tip1Offset, dispenserTipOffset;
	// Current destination coordinates for gotocoord state
	Offset currentDestination_, lastDestination_;
	Offset idleCoordinates_;
	float MovingHeight_;

	Offset camClibrationOffset_;
	Offset tip1ClibrationOffset_;

	Offset getBoxCoordinates();
	Offset getCompPickUpCoordinates();
	Offset getPCBCalibCoordinates();
	Offset getPCBCompCoordinates();
	Offset getCompPlaceCoordinates();
	Offset getBottomCamCoordinates();
	float getCompSuckingHeight();
	float getCompPlaceHeight();
	float getComponentLenth();
	float getComponentWidth();

	int selectFinder();
	int selectTip();
	void setPickUpCorrectionOffset(float xDiff, float yDiff, float rotDiff);
	void setPlaceCorrectionOffset(float xDiff, float yDiff, float rotDiff);
	void setBottomCamCorrectionOffset(float xDiff, float yDiff);
	void setTip1Offset(float xDiff, float yDiff);
	void setTip2Offset(float xDiff, float yDiff);
	void setDispenserOffset(float xDiff, float yDiff);

	Offset getTip1Coordinates();
	Offset getTip2Coordinates();
	Offset getDispenserCoordinates();

	void systemCalibration();

	void updatePlacementData(ComponentPlacerData *data);
	bool getCalibrationStatus();
	int getBoxNumber();

private:
	ComponentPlacerData currentComponent;

	enum TIP {
		LEFT_TIP, RIGHT_TIP
	} tip;

	// These offsets are relative to homing position
	Offset pcbOriginOffset, pickUpAreaOffset;
	Offset cameraBottomOffset;


	// Correction feedback from vision for pick-up and place
	Offset PickUpCorrection, PlaceCorrection;
	int PickUpRotCorrection, PlaceRotCorrection;

	// Current destination coordinates for gotocoord state
	Offset currentDestination;
};
