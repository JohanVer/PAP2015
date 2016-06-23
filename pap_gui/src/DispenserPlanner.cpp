/*
 * DispenserPlanner.cpp
 *
 *  Created on: 24.07.2015
 *      Author: johan
 */

#include <pap_gui/DispenserPlanner.hpp>
namespace DispenserPlanner{
DotPlanner::DotPlanner(){

}

DotPlanner::~DotPlanner() {
    // TODO Auto-generated destructor stub
}


std::vector<dispenseInfo> DotPlanner::planDispensing(PadInformation padInfoIn, float nozzleDiameter, double percentage_edge_dist, double wait_time, double alpha, DOT_ALIGN alignment){
    float inX = padInfoIn.rect.x();
    float inY = padInfoIn.rect.y();
    float inRot = padInfoIn.rotation;
    float inWidth = padInfoIn.rect.width();
    float inHeight = padInfoIn.rect.height();

    std::vector<dispenseInfo> outVector;
    float distanceFromEdge = percentage_edge_dist * nozzleDiameter;

    tf::Quaternion rotQuat;
    tf::Transform rotation_;
    rotQuat.setEuler(0.0, 0.0, inRot*(M_PI/180.0));
    rotation_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    rotation_.setRotation(rotQuat);

    double nozzle_side_total = 2 * distanceFromEdge + nozzleDiameter;

    unsigned int number_of_lines = 0;
    unsigned int number_of_cols = 0;

    number_of_lines =  inHeight / nozzle_side_total;
    number_of_cols = inWidth / nozzle_side_total;

    double dyn_offset_x = 0;
    double dyn_offset_y = 0;

    double stat_offset_x = 0;
    double stat_offset_y = 0;

    double nozzle_side_total_x = 0;
    double nozzle_side_total_y = 0;

    double max_overlap_length = 0;
    double max_underlap_length = 0;

    if(number_of_lines == 0){
        number_of_lines = 1;
        double overlap = (nozzleDiameter - inHeight);
        dyn_offset_y = - (overlap) / 2;
        nozzle_side_total_y = nozzleDiameter;
        if(overlap > max_overlap_length) max_overlap_length = overlap;
    }else{
        if(alignment == DOT_ALIGN::CENTER){
            dyn_offset_y = std::fmod(inHeight, nozzle_side_total) / (number_of_lines + 1);
            stat_offset_y = 0;
        }else if(alignment == DOT_ALIGN::LEFT){
            dyn_offset_y = 0;
            stat_offset_y = 0;
        }else if(alignment == DOT_ALIGN::RIGHT){
            dyn_offset_y = 0;
            stat_offset_y = std::fmod(inHeight, nozzle_side_total);
        }
        nozzle_side_total_y = nozzle_side_total;
    }

    if(number_of_cols == 0){
        number_of_cols = 1;
        double overlap = (nozzleDiameter - inWidth);
        dyn_offset_x = - (overlap) / 2;
        nozzle_side_total_x = nozzleDiameter;
        if(overlap > max_overlap_length) max_overlap_length = overlap;
    }else{
        if(alignment == DOT_ALIGN::CENTER){
            dyn_offset_x = std::fmod(inWidth, nozzle_side_total)  / (number_of_cols + 1) ;
            stat_offset_x = 0;
        }else if(alignment == DOT_ALIGN::LEFT){
            dyn_offset_x = 0;
            stat_offset_x = 0;
        }else if(alignment == DOT_ALIGN::RIGHT){
            dyn_offset_x = 0;
            stat_offset_x = std::fmod(inWidth, nozzle_side_total);
        }
        nozzle_side_total_x = nozzle_side_total;
    }

    double overlap_ratio = max_overlap_length / nozzle_side_total;

    std::cerr << "Wait time reducing factor: " << (alpha * overlap_ratio) << " with alpha: " << alpha << " and overlap_ratio: " << overlap_ratio << std::endl;

    for(auto line = 0; line < number_of_lines; line++){
        for(auto col = 0; col < number_of_cols; col++){
            double x = col * (nozzle_side_total_x) + (nozzle_side_total_x / 2) + (col + 1) * dyn_offset_x + stat_offset_x;
            double y = line * (nozzle_side_total_y) + (nozzle_side_total_y / 2) + (line + 1) * dyn_offset_y + stat_offset_y;

            dispenseInfo point;
            point.xPos = x - (inWidth / 2);
            point.xPos2 = point.xPos;

            point.yPos = y - (inHeight / 2);
            point.yPos2 = point.yPos;

            point.type = dispenser_types::DOT_DISPENSE;

            point.time = wait_time - ((alpha * overlap_ratio) * wait_time);

            outVector.push_back(point);
        }
    }

    // Rotate coordinates
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

    return outVector;
}

DispenserPlanner::DispenserPlanner() {
    // TODO Auto-generated constructor stub

}

DispenserPlanner::~DispenserPlanner() {
    // TODO Auto-generated destructor stub
}

bool isPadCompatibleToNozzle(const PadInformation &pad, double nozzle_diameter, double percentage_edge_dist){
    float inWidth = pad.rect.width();
    float inHeight = pad.rect.height();

    double distanceFromEdge = percentage_edge_dist * nozzle_diameter;

    double nozzle_side_total = 2 * distanceFromEdge + nozzle_diameter;

    unsigned int number_of_lines = 0;
    unsigned int number_of_cols = 0;

    number_of_lines =  inHeight / nozzle_side_total;
    number_of_cols = inWidth / nozzle_side_total;

    if(number_of_cols == 0){
        double rest_width = nozzle_side_total - inWidth;
        double rest_ratio = rest_width / nozzle_side_total;
        if(rest_ratio > 0.3) return false;
    }

    if(number_of_lines == 0){
        double rest_height = nozzle_side_total-  inHeight;
        double rest_ratio = rest_height / nozzle_side_total;
        if(rest_ratio > 0.3) return false;
    }

    return true;

}

std::vector<dispenseInfo> DispenserPlanner::planDispensing(
        PadInformation padInfoIn, float nozzleDiameter, double percentage_edge_dist, double velocity, double wait_time) {
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
            outInfo.type = dispenser_types::LINE_DISPENSE;
            outInfo.velocity = velocity;
            outInfo.time = wait_time;
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
            outInfo.type = dispenser_types::LINE_DISPENSE;
            outInfo.velocity = velocity;
            outInfo.time = wait_time;
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

}
