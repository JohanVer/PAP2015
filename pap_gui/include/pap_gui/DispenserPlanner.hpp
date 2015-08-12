/*
 * DispenserPlanner.hpp
 *
 *  Created on: 24.07.2015
 *      Author: johan
 */

#ifndef PAP2015_PAP_GUI_SRC_DISPENSERPLANNER_HPP_
#define PAP2015_PAP_GUI_SRC_DISPENSERPLANNER_HPP_

#include <CommonDataClasses.hpp>
#include <vector>
#include <math.h>
#include <ros/ros.h>

#define PERCENTAGE_EDGE_DISTANCE 0.1
#define VELOCITY_DISPENSE 0.5

class DispenserPlanner {
public:
	DispenserPlanner();
	virtual ~DispenserPlanner();

	// Planer function for dispensing one pad
	// Input: 1. Size of pad,2. Position of pad, 3. Rotation of pad 4. Nozzle diameter
	// Output:Start/Stop positions , Velocity/Time
	std::vector<dispenseInfo> planDispensing(PadInformation padInfo, float nozzleDiameter);
};

#endif /* PAP2015_PAP_GUI_SRC_DISPENSERPLANNER_HPP_ */
