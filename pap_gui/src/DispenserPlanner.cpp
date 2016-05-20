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
        PadInformation padInfoIn, float nozzleDiameter, double percentage_edge_dist, double velocity) {
    float inX = padInfoIn.rect.x();
    float inY = padInfoIn.rect.y();
    float inRot = padInfoIn.rotation;
    float inWidth = padInfoIn.rect.width();
    float inHeight = padInfoIn.rect.height();

    std::vector<dispenseInfo> outVector;
    float distanceFromEdge = percentage_edge_dist * nozzleDiameter;
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
            outInfo.type = dispenser_types::DISPENSE;
            outInfo.velocity = velocity;
            outInfo.time = 0;
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
            outInfo.type = dispenser_types::DISPENSE;
            outInfo.velocity = velocity;
            outInfo.time = 0;
            outVector.push_back(outInfo);
        }
    }

    // Create connecting lines
    std::vector<dispenseInfo> conn_outVector;
    for(size_t i = 0; i < outVector.size()-1; i++){

        // Actual line
        dispenseInfo act = outVector.at(i);
        conn_outVector.push_back(act);

        // Next line
        dispenseInfo &next = outVector.at(i+1);

        // Switch necessary ?
        if(i % 2 == 0){
            // Switch direction of next line
            double x_t = next.xPos2;
            double y_t = next.yPos2;
            next.xPos2 = next.xPos;
            next.yPos2 = next.yPos;
            next.xPos = x_t;
            next.yPos = y_t;
        }

        // Connect line
        dispenseInfo new_dis;
        new_dis.xPos = act.xPos2;
        new_dis.yPos = act.yPos2;
        new_dis.xPos2 = next.xPos;
        new_dis.yPos2 = next.yPos;
        new_dis.type = dispenser_types::NOT_DISPENSE;
        new_dis.time = 0;
        new_dis.velocity = act.velocity;
        conn_outVector.push_back(new_dis);
    }

    conn_outVector.push_back(outVector.back());

    // Rotate coordinates
    for(size_t i = 0; i < conn_outVector.size(); i++){
        tf::Point to_tf_1(conn_outVector.at(i).xPos, conn_outVector.at(i).yPos, 0);
        tf::Point to_tf_2(conn_outVector.at(i).xPos2, conn_outVector.at(i).yPos2, 0);

        tf::Point transformed_1 = rotation_ * to_tf_1;
        tf::Point transformed_2 = rotation_ * to_tf_2;

        conn_outVector.at(i).xPos = transformed_1.x() + inX;
        conn_outVector.at(i).yPos = transformed_1.y() + inY;

        conn_outVector.at(i).xPos2 = transformed_2.x() + inX;
        conn_outVector.at(i).yPos2 = transformed_2.y() + inY;
    }

    ROS_INFO("Planned %d lines...", numberOfLines);
    return conn_outVector;
}
