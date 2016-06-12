/*
 * DispenserPlanner.hpp
 *
 *  Created on: 24.07.2015
 *      Author: johan
 */

#ifndef PAP2015_PAP_GUI_SRC_DISPENSERPLANNER_HPP_
#define PAP2015_PAP_GUI_SRC_DISPENSERPLANNER_HPP_

#include <pap_common/CommonDataClasses.hpp>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace DispenserPlanner{

bool isPadCompatibleToNozzle(const PadInformation &pad, double nozzle_diameter, double percentage_edge_dist);

enum DOT_ALIGN{
    CENTER,
    LEFT,
    RIGHT
};

enum PLANNER_SELECT{
    LINE_PLANNER,
    DOT_PLANNER
};

class DotPlanner{
public:
    DotPlanner();
    virtual ~DotPlanner();
    std::vector<dispenseInfo> planDispensing(PadInformation padInfo, float nozzleDiameter, double percentage_edge_dist = 0.1, double wait_time = 1, double alpha =0.5, enum DOT_ALIGN alignment = DOT_ALIGN::RIGHT);
};

class DispenserPlanner {
public:
	DispenserPlanner();
	virtual ~DispenserPlanner();

	// Planer function for dispensing one pad
	// Input: 1. Size of pad,2. Position of pad, 3. Rotation of pad 4. Nozzle diameter
	// Output:Start/Stop positions , Velocity/Time
    std::vector<dispenseInfo> planDispensing(PadInformation padInfo, float nozzleDiameter, double percentage_edge_dist = 0.1, double velocity = 1, double wait_time = 1);

};

}
#endif /* PAP2015_PAP_GUI_SRC_DISPENSERPLANNER_HPP_ */
