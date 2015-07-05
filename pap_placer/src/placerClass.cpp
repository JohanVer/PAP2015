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

};
PlaceController::~PlaceController(){

};



Offset PlaceController::getBoxOffset(int boxNumber) {
	return boxOffsetVector.at(boxNumber);
};
