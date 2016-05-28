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
#include <queue>          // std::queue


class PlacementPlanner {
public:
    PlacementPlanner();
	virtual ~PlacementPlanner();

    // Update both tip diameters
    void setTipDiameters(float leftTip, float rightTip);

    // Gets componentToPlace, selects best tip available and sends componentData +
    // start placement command to placeController.
    // Returns true if process successfully started, false otherwise
    bool startSingleCompPlacement(pap_gui::QNode& node, ComponentPlacerData& compToPlace, bool& singlePlacementRunning);

    // Gets list of componentsToPlace, sorts components to best suitable tips, sends componentData +
    // start placement command to placeController.
    // Returns true if process successfully started, false otherwise
    bool startCompletePlacement(pap_gui::QNode& node, vector<ComponentPlacerData>& allCompToPlace, bool& completePlacementRunning);

    // Last task finished -> update components and start next placement task
    // Returns false if no components left to place, otherwise true
    bool sendNextTask(pap_gui::QNode& node, int& compNumLeftTip, int& compNumRightTip);

    // Clears component placement queues of both tips
    void resetQueues();

private:
     // Returns true if tipDiameter suits component diameters for a safe pick-up,
     // otherwise returns false
     bool checkTipSize(float tipDiameter, float length, float width);

     // Checks if box can be reach by usedTip, if thats the case returns
     // true, otherwise false
     bool boxReachable(int box, TIP usedTip);

    float leftTipDiameter, rightTipDiameter;

    std::queue<ComponentPlacerData> leftTipQueue;
    std::queue<ComponentPlacerData> rightTipQueue;

};

#endif /* PAP2015_PAP_GUI_SRC_PLACEMENTPLANNER_HPP_ */
