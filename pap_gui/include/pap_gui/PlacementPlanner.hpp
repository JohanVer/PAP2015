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
#include <queue>

//!
//! \brief PlacementPlanner takes a single component or an entire component list, initiates corresponding placement process if possible
//! and sends following placement tasks if necessary
//!
class PlacementPlanner {
public:

    PlacementPlanner();
	virtual ~PlacementPlanner();

    //!
    //! \brief setTipDiameters updates tip radius of both tips
    //! \param leftTip current left tip radius
    //! \param rightTip current left tip radius
    //!
    void setTipDiameters(float leftTip, float rightTip);

    //!
    //! \brief startSingleCompPlacement takes care of starting a singel component placement process
    //! \param node ROS communication node reference passed to be able to send messages
    //! \param compToPlace component to be placed
    //! \param singlePlacementRunning flag reference for GUI
    //! \return true if placement process successfully started, otherwise false
    //!
    bool startSingleCompPlacement(pap_gui::QNode& node, ComponentPlacerData& compToPlace, bool& singlePlacementRunning);

    //!
    //! \brief startCompletePlacement takes care of starting a complete placement process
    //! \param node ROS communication node reference passed to be able to send messages
    //! \param allCompToPlace vector of all components to be placed, already transformed into global coordinate system
    //! \param completePlacementRunning flage reference for GUI
    //! \return true if placement process successfully started, otherwise false
    //!
    bool startCompletePlacement(pap_gui::QNode& node, vector<ComponentPlacerData>& allCompToPlace, bool& completePlacementRunning);

    //!
    //! \brief sendNextTask send next components to be placed in case there are some left
    //! \param node ROS communication node reference passed to be able to send messages
    //! \return true if next components send successfully, otherwise false
    //!
    bool sendNextTask(pap_gui::QNode& node);

    //!
    //! \brief resetQueues clears both component queues
    //!
    void resetQueues();

    ComponentPlacerData compToPlaceLeft, compToPlaceRight;
    std::queue<ComponentPlacerData> leftTipQueue;
    std::queue<ComponentPlacerData> rightTipQueue;

private:
    //!
    //! \brief checkTipSize checks if tipDiameter suits component size for a safe pick-up
    //! \param tipDiameter of currenlty used tip
    //! \param length of component to be placed
    //! \param width of component to be placed
    //! \return true if tip suitable, otherwise false
    //!
    bool checkTipSize(float tipDiameter, float length, float width);

    //!
    //! \brief boxReachable checks if box can be reached by usedTip
    //! \param box current component box
    //! \param usedTip left or right tip
    //! \return true if box can be reached, otherwise false
    //!
    bool boxReachable(int box, TIP usedTip);

    //!
    //! \brief resetCompData sets component variables to default values
    //! \param comp selected component
    //!
    void resetCompData(ComponentPlacerData& comp);

    float leftTipDiameter, rightTipDiameter;
};

#endif /* PAP2015_PAP_GUI_SRC_PLACEMENTPLANNER_HPP_ */
