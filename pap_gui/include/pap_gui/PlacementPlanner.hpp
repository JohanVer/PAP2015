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

    // Gets componentToPlace, selects best tip available and sends componentData +
    // startSinglePlacement command to placeController.
    bool startSingleCompPlacement(pap_gui::QNode& node, ComponentPlacerData& compToPlace, bool& singlePlacementRunning);

    // Gets list of componentsToPlace, ...,
    void startCompletePlacement(pap_gui::QNode& node, vector<ComponentPlacerData>& allCompToPlace, bool& completePlacementRunning);

    // PlaceController feedback -> planner can select next components
    // and send update/placement commands to controller
    //bool placementStepFinished();

private:
     bool checkTipSize(float tipDiameter, float length, float width);
     bool boxReachable(int box, TIP usedTip);

    float leftTipDiameter, rightTipDiameter;

    vector<ComponentPlacerData> plannerCompList;
    //vector<bool>
    //std::queue<bool> tip1Comps;
    //std::queue<bool> tip2Comps;

};

#endif /* PAP2015_PAP_GUI_SRC_PLACEMENTPLANNER_HPP_ */
