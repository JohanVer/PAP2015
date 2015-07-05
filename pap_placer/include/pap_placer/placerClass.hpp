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

class Offset {
public:
	float x;
	float y;
private:
};


class PlaceController {
public:
	PlaceController();
	~PlaceController();

private:

};
