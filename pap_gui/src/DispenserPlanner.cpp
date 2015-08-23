/*
 * DispenserPlanner.cpp
 *
 *  Created on: 24.07.2015
 *      Author: johan
 */

#include "../include/pap_gui/DispenserPlanner.hpp"

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

	if (inRot == 270.0 || inRot == 90.0) {
		float buffer;
		buffer = inWidth;
		inWidth = inHeight;
		inHeight = buffer;
	}

	float widthDrive = inWidth - 2 * distanceFromEdge - nozzleDiameter;
	float heightDrive = inHeight - 2 * distanceFromEdge - nozzleDiameter;

	if (widthDrive < 0 || heightDrive < 0) {
		ROS_ERROR("Pad too small for dispensing with current nozzle");
	}

	//ROS_INFO("WidthDrive : %f , HeightDrive: %f", widthDrive, heightDrive);

	/*
	if (widthDrive < nozzleDiameter || heightDrive < nozzleDiameter) {
		dispenseInfo outInfo;
		ROS_INFO(
				"Pad too small for dispensing but I will try to dispense a dot...");
		/outInfo.xPos = inX;
		outInfo.yPos = inY;
		outInfo.type = 0;
		outInfo.time = 0.5;
		//return outVector;
		//outVector.push_back(outInfo);
	}
	*/

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
			float xCoord = inX - (inWidth / 2) + distanceFromEdge
					+ nozzleDiameter / 2.0;
			float yCoord = inY - (inHeight / 2) + distanceFromEdge
					+ distanceFromEdgeCal / (float) (numberOfLines + 1)
					+ nozzleDiameter / 2.0;
			yCoord += heightOffset;

			float xCoord2 = inX - (inWidth / 2) + distanceFromEdge
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
			float yCoord = inY - (inHeight / 2) + distanceFromEdge
					+ nozzleDiameter / 2.0;
			float xCoord = inX - (inWidth / 2) + distanceFromEdge
					+ distanceFromEdgeCal / (float) (numberOfLines + 1)
					+ nozzleDiameter / 2.0;
			xCoord += widthOffset;

			float yCoord2 = inY - (inHeight / 2) + distanceFromEdge
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

	ROS_INFO("Planned %d lines...", numberOfLines);
	return outVector;
}
