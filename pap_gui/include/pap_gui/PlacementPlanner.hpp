/*
 * PlacementPlanner.hpp
 *
 *  Created on: 25.05.2016
 *      Author: nikolas
 */

#ifndef PAP2015_PAP_GUI_SRC_PLACEMENTPLANNER_HPP_
#define PAP2015_PAP_GUI_SRC_PLACEMENTPLANNER_HPP_

#include <pap_common/CommonDataClasses.hpp>
#include "qnode.hpp"
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>


class PlacementPlanner {
public:
    // Set qnode reference in constructor
    PlacementPlanner();
	virtual ~PlacementPlanner();

    //void updatePlacementData();

    // Function to update current tip diameters
    void setTipDiameters(float leftTip, float rightTip);

    //
    //void startSingleCompPlacement(ComponentPlacerData& compToPlace);


    // void startCompletePlacement(vector<ComponentPlacerData>& allCompToPlace);

    // PlaceController feedback -> planner can select next components
    // and send update/placement commands to controller
    //bool placementStepFinished();

private:
    float leftTipDiameter, rightTipDiameter;


};

#endif /* PAP2015_PAP_GUI_SRC_PLACEMENTPLANNER_HPP_ */
