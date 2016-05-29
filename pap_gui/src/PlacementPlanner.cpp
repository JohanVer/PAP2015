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
}

PlacementPlanner::~PlacementPlanner() {
    // TODO Auto-generated destructor stub
}

void PlacementPlanner::setTipDiameters(float leftTip, float rightTip) {
    leftTipDiameter = leftTip;
    rightTipDiameter = rightTip;
    std::cerr << "Tip diameters updated to: " << leftTipDiameter << ", " << rightTipDiameter << std::endl;
}


bool PlacementPlanner::startSingleCompPlacement(pap_gui::QNode& node, ComponentPlacerData& compToPlace, bool& singlePlacementRunning) {

    QMessageBox msgBox;
    float length = compToPlace.length;
    float width = compToPlace.width;
    int box = compToPlace.box;

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
    std::cerr << "Planner compList size: " << allCompToPlace.size() << std::endl;
    int nonSuitableComps = 0;

    // Push components to suitable tip queue
    for(int i = 0; i <  allCompToPlace.size(); i++) {

        float length = allCompToPlace[i].length;
        float width = allCompToPlace[i].width;
        int box = allCompToPlace[i].box;

        // Select tip and push component to corresponding queue
        if(checkTipSize(rightTipDiameter, length, width) && boxReachable(box, TIP::RIGHT_TIP)) {            // Right nozzle suitable
            // Right tip can be used for this component and box is reachable
            rightTipQueue.push(allCompToPlace[i]);

        } else if(checkTipSize(leftTipDiameter, length, width) && boxReachable(box, TIP::LEFT_TIP)) {       // Left nozzle suitable
            // Left tip can be used for this component and box is reachable
            leftTipQueue.push(allCompToPlace[i]);

        } else {       // No nozzles suitable
            nonSuitableComps++;
        }
    }

    QMessageBox msgBox;
    compToPlaceRight.isWaiting = false;
    compToPlaceLeft.isWaiting = false;

    if((nonSuitableComps > 0) && (nonSuitableComps < allCompToPlace.size())) {
        QString message =
                QString("There are %1 out of %2 components that cannot be placed with the current nozzle configuration. If you want to start the placement process anyways press yes.\n").arg(nonSuitableComps).arg(allCompToPlace.size());
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
        msgBox.setText("Not able to place any components.");
        msgBox.exec();
        msgBox.close();
        return false;
    } else {
        QString message =
                QString("You are about to start a complete placement proces with %1 components.\n \nClick yes to start process.\n").arg(allCompToPlace.size());
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

    // Check queues and send data to palceController
    if(leftTipQueue.empty()) {                              // Right tip queue is non-empty
        resetCompData(compToPlaceLeft);
        compToPlaceRight = rightTipQueue.front();
        rightTipQueue.pop();
        compToPlaceRight.isWaiting = true;
        node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                               compToPlaceRight, TIP::RIGHT_TIP);

    } else if(rightTipQueue.empty()) {                      // Left tip queue is non-empty
        resetCompData(compToPlaceRight);
        compToPlaceLeft = leftTipQueue.front();
        leftTipQueue.pop();
        compToPlaceLeft.isWaiting = true;
        node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                               compToPlaceLeft, TIP::LEFT_TIP);

    } else {                                                // Both queues are non-empty
        compToPlaceLeft = leftTipQueue.front();
        leftTipQueue.pop();
        compToPlaceRight = rightTipQueue.front();
        rightTipQueue.pop();
        compToPlaceRight.isWaiting = true;
        compToPlaceLeft.isWaiting = true;
        node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                               compToPlaceLeft, TIP::LEFT_TIP);
        node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                               compToPlaceRight, TIP::RIGHT_TIP);
    }

    completePlacementRunning = true;
    return true;
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

void PlacementPlanner::resetQueues() {
    while(!leftTipQueue.empty()) leftTipQueue.pop();
    while(!rightTipQueue.empty()) rightTipQueue.pop();
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
        compToPlaceLeft = leftTipQueue.front();
        leftTipQueue.pop();
        compToPlaceLeft.isWaiting = true;
        node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                               compToPlaceLeft, TIP::LEFT_TIP);

    } else {                                                // Both queues are non-empty
        compToPlaceLeft = leftTipQueue.front();
        leftTipQueue.pop();
        compToPlaceRight = rightTipQueue.front();
        rightTipQueue.pop();
        compToPlaceRight.isWaiting = true;
        compToPlaceLeft.isWaiting = true;
        node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                               compToPlaceLeft, TIP::LEFT_TIP);
        node.sendTask(pap_common::PLACER, pap_common::UPDATE_PLACER,
                               compToPlaceRight, TIP::RIGHT_TIP);
    }

    node.sendTask(pap_common::PLACER, pap_common::START_COMPLETE_PLACEMENT);
    return true;
}
