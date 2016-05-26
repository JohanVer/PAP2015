/*
 * PlacementPlanner.cpp
 *
 *  Created on: 25.05.2016
 *      Author: nikolas
 */

#include <pap_gui/PlacementPlanner.hpp>
#include <QMessageBox>

PlacementPlanner::PlacementPlanner() {
    // TODO Auto-generated constructor stub
    //plannerQnode = &qnode;

}

PlacementPlanner::~PlacementPlanner() {
    // TODO Auto-generated destructor stub
}

void PlacementPlanner::setTipDiameters(float leftTip, float rightTip) {
    leftTipDiameter = leftTip;
    rightTipDiameter = rightTip;
    std::cerr << "Tip diameters update to: " << leftTipDiameter << ", " << rightTipDiameter << std::endl;
}


bool PlacementPlanner::startSingleCompPlacement(pap_gui::QNode& node, ComponentPlacerData& compToPlace, bool& singlePlacementRunning) {

    float length = compToPlace.length;
    float width = compToPlace.width;
    int box = compToPlace.box;

    // Select tip, update placement data and start process
    if(checkTipSize(leftTipDiameter, length, width) && boxReachable(box, TIP::LEFT_TIP)) {               // Left nozzle suitable
        node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                               compToPlace, TIP::LEFT_TIP);
        node.sendTask(pap_common::PLACER, pap_common::START_SINGLE_PLACEMENT);
        singlePlacementRunning = true;
        std::cerr << "PlacementPlanner: Singe placement process started - Left tip used" << std::endl;
        return true;

    } else if(checkTipSize(rightTipDiameter, length, width) && boxReachable(box, TIP::RIGHT_TIP)) {       // Right nozzle suitable
        node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                               compToPlace, TIP::RIGHT_TIP);
        node.sendTask(pap_common::PLACER, pap_common::START_SINGLE_PLACEMENT);
        singlePlacementRunning = true;
        std::cerr << "PlacementPlanner: Singe placement process started - Right tip used" << std::endl;
        return true;

    } else {       // No nozzles suitable
        QMessageBox msgBox;
        msgBox.setText("No suitable tips available or box cannot be reached.");
        msgBox.exec();
        msgBox.close();
        return false;
    }
}

bool PlacementPlanner::boxReachable(int box, TIP usedTip) {

    if(usedTip == TIP::LEFT_TIP) {
        if(box < 86)
            return true;

    } else {
        if(((box >= 42) && (box <= 46))             // Small boxes
                || (box == 52) || (box == 58)       // Middle boxes
                || (box == 62) || (box == 66)       // Large boxes
                || ((box >= 70) && (box <= 86)))    // Tapes
            return true;
    }
    return false;
}

bool PlacementPlanner::checkTipSize(float tipDiameter, float length, float width) {

    if(tipDiameter > 0.0) {
        // Consider shorter component edge
        if(width <= length) {
            if(width >= 0.8 * tipDiameter)
                return true;
        } else {
            if(length >= 0.8 * tipDiameter)
                return true;
        }
    }
    return false;
}


bool PlacementPlanner::startCompletePlacement(pap_gui::QNode& node, vector<ComponentPlacerData>& allCompToPlace, bool& completePlacementRunning) {

    // Update planner component list
    plannerCompList.clear();
    plannerCompList = allCompToPlace;

    // Check list size etc.
    std::cerr << "Planner compList size: " << plannerCompList.size() << std::endl;

//    node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
//                           compToPlace, TIP::LEFT_TIP);
//    node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
//                           compToPlace, TIP::RIGHT_TIP);
//    node.sendTask(pap_common::PLACER, pap_common::START_COMPLETE_PLACEMENT);

    return true;
}
