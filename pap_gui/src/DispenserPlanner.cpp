/*
 * DispenserPlanner.cpp
 *
 *  Created on: 24.07.2015
 *      Author: johan
 */

#include <pap_gui/DispenserPlanner.hpp>

DispenserPlanner::DispenserPlanner() {
    // TODO Auto-generated constructor stub

}

DispenserPlanner::~DispenserPlanner() {
    // TODO Auto-generated destructor stub
}

std::vector<dispenseInfo> DispenserPlanner::planDispensing(
        PadInformation padInfoIn, float nozzleDiameter) {
    float inX = padInfoIn.rect.x();
    float inY = padInfoIn.rect.y();
    float inRot = padInfoIn.rotation;
    float inWidth = padInfoIn.rect.width();
    float inHeight = padInfoIn.rect.height();

    std::vector<dispenseInfo> outVector;
    float distanceFromEdge = PERCENTAGE_EDGE_DISTANCE * nozzleDiameter;
    bool useHeightLines = false;

    tf::Quaternion rotQuat;
    tf::Transform rotation_;
    rotQuat.setEuler(0.0, 0.0, inRot*(M_PI/180.0));
    rotation_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    rotation_.setRotation(rotQuat);

    float widthDrive = inWidth - 2 * distanceFromEdge - nozzleDiameter;
    float heightDrive = inHeight - 2 * distanceFromEdge - nozzleDiameter;

    if (widthDrive < 0 || heightDrive < 0) {
        ROS_ERROR("Pad too small for dispensing with current nozzle");
    }

    unsigned int numberOfLines = 0;
    float distanceFromEdgeCal = 0.0;

    if (inWidth > inHeight) {
        numberOfLines = inHeight / (nozzleDiameter + distanceFromEdge * 2);
        distanceFromEdgeCal = fmod(inHeight,
                                   (nozzleDiameter + distanceFromEdge * 2));
        useHeightLines = true;
        // Dispenser Nozzle is too big for the pad
        if (numberOfLines <= 0) {
            ROS_INFO("Dispenser Nozzle is too big for pad...");
            numberOfLines = 1;
            distanceFromEdgeCal = fmod(inHeight, nozzleDiameter);
            distanceFromEdgeCal /= 2.0;
            if (nozzleDiameter > inHeight) {
                distanceFromEdgeCal = -distanceFromEdgeCal;
            }
            //	distanceFromEdgeCal = 0.0;
        }
    } else {
        numberOfLines = inWidth / (nozzleDiameter + distanceFromEdge * 2);
        distanceFromEdgeCal = fmod(inWidth,
                                   (nozzleDiameter + distanceFromEdge * 2));
        useHeightLines = false;

        // Dispenser Nozzle is too big for the pad
        if (numberOfLines <= 0) {
            ROS_INFO("Dispenser Nozzle is too big for pad...");
            numberOfLines = 1;
            distanceFromEdgeCal = fmod(inWidth, nozzleDiameter);
            distanceFromEdgeCal /= 2;
            if (nozzleDiameter > inWidth) {
                distanceFromEdgeCal = -distanceFromEdgeCal;
            }
            //distanceFromEdgeCal = 0.0;
        }
    }

    //ROS_INFO("DistanceFromEdgeCal : %f DistanceFromEdge: %f",
    //distanceFromEdgeCal, distanceFromEdge);
    for (size_t i = 0; i < numberOfLines; i++) {
        if (useHeightLines) {
            dispenseInfo outInfo;
            // Width is higher than height
            float heightOffset =
                    i
                    * ((2 * distanceFromEdge) + nozzleDiameter
                       + distanceFromEdgeCal
                       / (float) (numberOfLines + 1));
            float xCoord =  - (inWidth / 2) + distanceFromEdge
                    + nozzleDiameter / 2.0;
            float yCoord =  - (inHeight / 2) + distanceFromEdge
                    + distanceFromEdgeCal / (float) (numberOfLines + 1)
                    + nozzleDiameter / 2.0;
            yCoord += heightOffset;

            float xCoord2 =  - (inWidth / 2) + distanceFromEdge
                    + nozzleDiameter / 2.0 + widthDrive;
            float yCoord2 = yCoord;

            ROS_INFO("Generated line:X1: %f Y1: %f X2: %f Y2: %f", xCoord,
                     yCoord, xCoord2, yCoord2);
            outInfo.xPos = xCoord;
            outInfo.yPos = yCoord;
            outInfo.xPos2 = xCoord2;
            outInfo.yPos2 = yCoord2;
            outInfo.type = 1;
            outInfo.velocity = VELOCITY_DISPENSE;
            outInfo.time = 1;
            outVector.push_back(outInfo);
        } else {
            dispenseInfo outInfo;
            // Height is higher than width
            float widthOffset =
                    i
                    * ((2 * distanceFromEdge) + nozzleDiameter
                       + distanceFromEdgeCal
                       / (float) (numberOfLines + 1));
            float yCoord =  - (inHeight / 2) + distanceFromEdge
                    + nozzleDiameter / 2.0;
            float xCoord =  - (inWidth / 2) + distanceFromEdge
                    + distanceFromEdgeCal / (float) (numberOfLines + 1)
                    + nozzleDiameter / 2.0;
            xCoord += widthOffset;

            float yCoord2 =  - (inHeight / 2) + distanceFromEdge
                    + nozzleDiameter / 2.0 + heightDrive;
            float xCoord2 = xCoord;

            //ROS_INFO("Generated line:X1: %f Y1: %f X2: %f Y2: %f",xCoord,yCoord,xCoord2,yCoord2);
            outInfo.xPos = xCoord;
            outInfo.yPos = yCoord;
            outInfo.xPos2 = xCoord2;
            outInfo.yPos2 = yCoord2;
            outInfo.type = 1;
            outInfo.velocity = VELOCITY_DISPENSE;
            outInfo.time = 1;
            outVector.push_back(outInfo);
        }
    }

    for(size_t i = 0; i < outVector.size(); i++){
        tf::Point to_tf_1(outVector.at(i).xPos, outVector.at(i).yPos, 0);
        tf::Point to_tf_2(outVector.at(i).xPos2, outVector.at(i).yPos2, 0);

        tf::Point transformed_1 = rotation_ * to_tf_1;
        tf::Point transformed_2 = rotation_ * to_tf_2;

        outVector.at(i).xPos = transformed_1.x() + inX;
        outVector.at(i).yPos = transformed_1.y() + inY;

        outVector.at(i).xPos2 = transformed_2.x() + inX;
        outVector.at(i).yPos2 = transformed_2.y() + inY;
    }

    ROS_INFO("Planned %d lines...", numberOfLines);
    return outVector;
}
