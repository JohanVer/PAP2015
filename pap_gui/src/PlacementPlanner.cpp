/*
 * PlacementPlanner.cpp
 *
 *  Created on: 25.05.2016
 *      Author: nikolas
 */

#include <pap_gui/PlacementPlanner.hpp>
#include "../../pap_placer/include/pap_placer/offsetTable.hpp"
#include <QMessageBox>

#define TIP_SIZE_SAFETY_FACTOR 0.7

#define TIP_DISTANCE_Y 100.0
#define MAX_DOUBLE_PLACEMENT_DIST 150.0

PlacementPlanner::PlacementPlanner() {
    // TODO Auto-generated constructor stub
}

PlacementPlanner::~PlacementPlanner() {
    // TODO Auto-generated destructor stub
}

void PlacementPlanner::setTipDiameters(float leftTip, float rightTip) {
    leftTipDiameter = 2*leftTip;
    rightTipDiameter = 2*rightTip;
    std::cerr << "PlacementPlanner: Tip diameters updated to: " << leftTipDiameter << ", " << rightTipDiameter << std::endl;
}


bool PlacementPlanner::startSingleCompPlacement(pap_gui::QNode& node, ComponentPlacerData& compToPlace, bool& singlePlacementRunning) {

    QMessageBox msgBox;
    float length = compToPlace.length;
    float width = compToPlace.width;
    int box = compToPlace.box;
    compToPlaceRight.isWaiting = false;
    compToPlaceLeft.isWaiting = false;

    // Select tip, update placement data and start process
    if(checkTipSize(leftTipDiameter, length, width) && boxReachable(box, TIP::LEFT_TIP)) {               // Left nozzle suitable
        QString message =
                QString("You are about to start a single placement process. Please make sure the component is in slot %1. \n\nClick yes to start process.").arg(compToPlace.box);
        msgBox.setWindowTitle("Confirm to start placement");
        msgBox.setText(message);
        msgBox.setStandardButtons(QMessageBox::Yes);
        msgBox.addButton(QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);
        if (msgBox.exec() == QMessageBox::Yes) {
            compToPlaceLeft = compToPlace;
            compToPlaceLeft.isWaiting = true;
            node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                                   compToPlace, TIP::LEFT_TIP);
            node.sendTask(pap_common::PLACER, pap_common::START_SINGLE_PLACEMENT);
            singlePlacementRunning = true;
            std::cerr << "PlacementPlanner: Singe placement process started - Left tip used" << std::endl;
            return true;
        }

    } else if(checkTipSize(rightTipDiameter, length, width) && boxReachable(box, TIP::RIGHT_TIP)) {       // Right nozzle suitable
        QString message =
                QString("You are about to start a single placement process. Please make sure the component is in slot %1. \n\nClick yes to start process.").arg(compToPlace.box);
        msgBox.setWindowTitle("Confirm to start placement");
        msgBox.setText(message);
        msgBox.setStandardButtons(QMessageBox::Yes);
        msgBox.addButton(QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);
        if (msgBox.exec() == QMessageBox::Yes) {
            compToPlaceRight = compToPlace;
            compToPlaceRight.isWaiting = true;
            node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                                   compToPlace, TIP::RIGHT_TIP);
            node.sendTask(pap_common::PLACER, pap_common::START_SINGLE_PLACEMENT);
            singlePlacementRunning = true;
            std::cerr << "PlacementPlanner: Singe placement process started - Right tip used" << std::endl;
            return true;
        }

    } else {       // No nozzles suitable
        QMessageBox msgBox;
        msgBox.setText("No suitable tips available or box cannot be reached.");
        msgBox.exec();
        msgBox.close();
    }
    return false;
}

bool PlacementPlanner::startCompletePlacement(pap_gui::QNode& node, vector<ComponentPlacerData>& allCompToPlace, bool& completePlacementRunning) {

    resetQueues();
    std::cerr << "PlacementPlanner: compList size " << allCompToPlace.size() << std::endl;
    int nonSuitableComps = 0;

    vector<bool> componentAssigned(allCompToPlace.size(), false);
    float length, width;
    int box;
    float minDist = MAX_DOUBLE_PLACEMENT_DIST;
    int compIndex = -1;

    // Iterate over all components not assigned to a tip
    for(int i = 0; i <  allCompToPlace.size(); i++) {

        // If component has not been assigned to a tip
        if(!componentAssigned[i]) {
            length = allCompToPlace[i].length;
            width = allCompToPlace[i].width;
            box = allCompToPlace[i].box;

            // Check if right tip can be used for this component and box is reachable
            if(checkTipSize(rightTipDiameter, length, width) && boxReachable(box, TIP::RIGHT_TIP)) {
                rightTipQueue.push(allCompToPlace[i]);
                componentAssigned[i]  = true;

                QPointF optLeftTipBoxPos = getBoxPosition(box);
                optLeftTipBoxPos.setY(optLeftTipBoxPos.y() + TIP_DISTANCE_Y);
                minDist = MAX_DOUBLE_PLACEMENT_DIST;
                compIndex = -1;

                // Find closest component for left tip by iterating over all missing components
                for(int j = 0; j <  allCompToPlace.size(); j++) {

                    // If component has not been assigned yet
                    if(!componentAssigned[j]) {
                        length = allCompToPlace[j].length;
                        width = allCompToPlace[j].width;
                        box = allCompToPlace[j].box;

                        // Check if left tip can be used for this component and box is reachable
                        if(checkTipSize(leftTipDiameter, length, width) && boxReachable(box, TIP::LEFT_TIP)) {

                            // Compute euclidean distance
                            QPointF boxPos = getBoxPosition(box);
                            float dist = computeDistance(optLeftTipBoxPos, boxPos);

                            if(dist < minDist) {
                                minDist = dist;
                                compIndex = j;
                            }
                        }
                    }
                }

                if(compIndex != -1) {
                    // Check if left tip can be used for this component and box is reachable
                    leftTipQueue.push(allCompToPlace[compIndex]);
                    componentAssigned[compIndex]  = true;
                } else {
                    // Include NOP entry to wait with left tip until right tip placement finished
                    ComponentPlacerData nop;
                    nop.box = -1;
                    leftTipQueue.push(nop);
                }
            }
        }
    }

    std::cerr << "PlacementPlanner: left tip queue size " << leftTipQueue.size() << std::endl;
    std::cerr << "PlacementPlanner: right tip queue size " << rightTipQueue.size() << std::endl;

    // Iterate over all components not assigned yet - right tip not suitable!
    for(int i = 0; i <  allCompToPlace.size(); i++) {

        // If component has not been assigned to a tip
        if(!componentAssigned[i]) {
            length = allCompToPlace[i].length;
            width = allCompToPlace[i].width;
            box = allCompToPlace[i].box;

            // Check if left tip can be used for this component and box is reachable
            if(checkTipSize(leftTipDiameter, length, width) && boxReachable(box, TIP::LEFT_TIP)) {
                leftTipQueue.push(allCompToPlace[i]);
                componentAssigned[i]  = true;
            } else {
                nonSuitableComps++;
            }
        }
    }


    // Push components to suitable tip queue
//    for(int i = 0; i <  allCompToPlace.size(); i++) {

//        float length = allCompToPlace[i].length;
//        float width = allCompToPlace[i].width;
//        int box = allCompToPlace[i].box;

//        // Select tip and push component to corresponding queue
//        if(checkTipSize(rightTipDiameter, length, width) && boxReachable(box, TIP::RIGHT_TIP)) {            // Right nozzle suitable
//            // Right tip can be used for this component and box is reachable
//            rightTipQueue.push(allCompToPlace[i]);

//        } else if(checkTipSize(leftTipDiameter, length, width) && boxReachable(box, TIP::LEFT_TIP)) {       // Left nozzle suitable
//            // Left tip can be used for this component and box is reachable
//            leftTipQueue.push(allCompToPlace[i]);

//        } else {       // No nozzles suitable
//            nonSuitableComps++;
//        }
//    }

    std::cerr << "PlacementPlanner: left tip queue size " << leftTipQueue.size() << std::endl;
    std::cerr << "PlacementPlanner: right tip queue size " << rightTipQueue.size() << std::endl;

    QMessageBox msgBox;
    compToPlaceRight.isWaiting = false;
    compToPlaceLeft.isWaiting = false;

    if((nonSuitableComps > 0) && (nonSuitableComps < allCompToPlace.size())) {
        QString message =
                QString("There are %1 out of %2 components that cannot be placed with the current nozzle configuration.\n\nIf you want to start the placement process anyways press yes.\n").arg(nonSuitableComps).arg(allCompToPlace.size());
        msgBox.setWindowTitle("Confirm to start placement");
        msgBox.setText(message);
        msgBox.setStandardButtons(QMessageBox::Yes);
        msgBox.addButton(QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::Yes);
        if (msgBox.exec() == QMessageBox::No) {
            std::cerr << "Placement process aborted.\n" << std::endl;
            return false;
        }
    } else if(nonSuitableComps == allCompToPlace.size()) {      // Both queues are empty!!
        msgBox.setText("Not able to place any components \n(tip cannot be used or box not reachable).");
        msgBox.exec();
        msgBox.close();
        return false;
    } else {
        QString message =
                QString("You are about to start a complete placement proces with %1 components.\n\nClick yes to start process.\n").arg(allCompToPlace.size());
        msgBox.setWindowTitle("Confirm to start placement");
        msgBox.setText(message);
        msgBox.setStandardButtons(QMessageBox::Yes);
        msgBox.addButton(QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::Yes);
        if (msgBox.exec() == QMessageBox::No) {
            std::cerr << "Placement process aborted.\n" << std::endl;
            return false;
        }
    }

    // Check queues and send data to placeController
    if(!(rightTipQueue.empty())) {                              // Right tip queue is non-empty
        compToPlaceRight = rightTipQueue.front();
        rightTipQueue.pop();
        compToPlaceRight.isWaiting = true;
        node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                               compToPlaceRight, TIP::RIGHT_TIP);
    } else {
        resetCompData(compToPlaceRight);
    }

    if(!(leftTipQueue.empty())) {                               // Left tip queue is non-empty
        ComponentPlacerData next = leftTipQueue.front();
        leftTipQueue.pop();
        if(next.box != -1) {
            compToPlaceLeft = next;
            compToPlaceLeft.isWaiting = true;
            node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                          compToPlaceLeft, TIP::LEFT_TIP);
        }
    } else {
        resetCompData(compToPlaceLeft);
    }

    completePlacementRunning = true;
    node.sendTask(pap_common::PLACER, pap_common::START_COMPLETE_PLACEMENT);
    return true;
}

QPointF PlacementPlanner::getBoxPosition(const int box) {

    QPointF boxPos(-1.0,-1.0);
    if (box < 67) {
        boxPos.setX(BoxOffsetTable[box].x);
        boxPos.setY(BoxOffsetTable[box].y);
    } else if ((box >= 67) && (box <= 86)) {
        boxPos.setX(TapeOffsetTable[box-67].x);
        boxPos.setY(TapeOffsetTable[box-67].y);
    }
    return boxPos;
}

float PlacementPlanner::computeDistance(const QPointF& p1, const QPointF& p2) {
    float diffY = p1.y() - p2.y();
    float diffX = p1.x() - p2.x();
    return sqrt((diffY * diffY) + (diffX * diffX));
}

void PlacementPlanner::resetCompData(ComponentPlacerData& comp) {
        comp.destX = 0.0;
        comp.destY = 0.0;
        comp.rotation = 0.0;
        comp.length = 0.0;
        comp.width = 0.0;
        comp.height = 0.0;
        comp.box = 0;
        comp.tapeX = 0.0;
        comp.tapeY = 0.0;
        comp.tapeRot = 0.0;
        comp.name = "-";
        comp.isWaiting = false;
}


bool PlacementPlanner::boxReachable(int box, TIP usedTip) {

    if(usedTip == TIP::LEFT_TIP) {
        if(box < 86)
            return true;

    } else {
        if(((box >= 37) && (box <= 46))             // Small boxes
                || (box == 51) || (box == 57)       // Middle boxes
                || (box == 52) || (box == 58)       // Middle boxes
                || (box == 61) || (box == 65)       // Large boxes
                || (box == 62) || (box == 66)       // Large boxes
                || ((box >= 70) && (box <= 86)))    // Tapes
            return true;
    }
    return false;
}

bool PlacementPlanner::checkTipSize(float tipDiameter, float length, float width) {

    if(tipDiameter > 0.0) {
        // Consider shorter component edge
        //std::cerr << "width: " << width << " - length:  " << length << std::endl;
        if(width <= length) {
            if(width >= TIP_SIZE_SAFETY_FACTOR * tipDiameter)
                return true;
        } else {
            if(length >= TIP_SIZE_SAFETY_FACTOR * tipDiameter)
                return true;
        }
    }
    return false;
}

void PlacementPlanner::resetQueues() {
    while(!leftTipQueue.empty()) leftTipQueue.pop();
    while(!rightTipQueue.empty()) rightTipQueue.pop();
    compToPlaceRight.isWaiting = false;
    compToPlaceLeft.isWaiting = false;
}

bool PlacementPlanner::sendNextTask(pap_gui::QNode& node) {

    // Pop next components to place from queues
    compToPlaceRight.isWaiting = false;
    compToPlaceLeft.isWaiting = false;

    // Check queues
    if(leftTipQueue.empty() && rightTipQueue.empty()) {     // No more components to place
        return false;

    } else if(leftTipQueue.empty()) {                       // Right tip queue is non-empty
        compToPlaceRight = rightTipQueue.front();
        rightTipQueue.pop();
        compToPlaceRight.isWaiting = true;
        node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                               compToPlaceRight, TIP::RIGHT_TIP);

    } else if(rightTipQueue.empty()) {                      // Left tip queue is non-empty
        ComponentPlacerData next = leftTipQueue.front();
        leftTipQueue.pop();
        if(next.box != -1) {
            compToPlaceLeft = next;
            compToPlaceLeft.isWaiting = true;
            node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                          compToPlaceLeft, TIP::LEFT_TIP);
        }
    } else {                                                // Both queues are non-empty
        ComponentPlacerData next = leftTipQueue.front();
        leftTipQueue.pop();
        if(next.box != -1) {
            compToPlaceLeft = next;
            compToPlaceLeft.isWaiting = true;
            node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                          compToPlaceLeft, TIP::LEFT_TIP);
        }
        compToPlaceRight = rightTipQueue.front();
        rightTipQueue.pop();
        compToPlaceRight.isWaiting = true;
        node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                               compToPlaceRight, TIP::RIGHT_TIP);
    }

    node.sendTask(pap_common::PLACER, pap_common::START_COMPLETE_PLACEMENT);
    return true;
}
