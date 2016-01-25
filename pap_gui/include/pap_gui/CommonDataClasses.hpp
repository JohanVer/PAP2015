/*
 * CommonDataClasses.hpp
 *
 *  Created on: 30.07.2015
 *      Author: johan
 */

#ifndef PAP2015_PAP_GUI_INCLUDE_PAP_GUI_COMMONDATACLASSES_HPP_
#define PAP2015_PAP_GUI_INCLUDE_PAP_GUI_COMMONDATACLASSES_HPP_

#include <string>
#include <QRectF>
#include <QString>

class databaseEntry {
public:
	QString package;
	float length, width, height;
	int pins;
	databaseEntry(){
		length = 0.0;
		width = 0.0;
		height = 0.0;
		pins = 0;
	}
};

class PartEntry{
public:
	std::string package, value;
	int count, slot;
	PartEntry(){
		count = 0;
		slot = 0;
	}
};



class componentEntry {
public:
	std::string name, package, side, value;
	float posX, posY, length, width, height;
	int box, rotation, pins, index;
};

class dispenseInfo{
public:
	dispenseInfo(){
		xPos = 0.0;
		yPos = 0.0;
		xPos2 = 0.0;
		yPos2 = 0.0;
		type = Point;
		rotation = 0.0;
		velocity = 0.0;
		time = 0.0;
	}

	enum dispensePadType {
		Point, Long
	};

	int type;
	float xPos, yPos, xPos2, yPos2;
	float rotation;
	float velocity;
	float time;
};

class PadInformation {
public:
	PadInformation() {
		rotation = 0.0;
	}
	QRectF rect;
	std::string shapeStr;
	float rotation;
	bool dispensed;
};

class ShapeInformation {
public:
	ShapeInformation() {
		shapeIndex = 0;
		shapeStr = "";
		rotation = 0.0;
	}
	int shapeIndex;
	std::string shapeStr;
	QPointF padDimensions;
	float rotation;
};

#endif /* PAP2015_PAP_GUI_INCLUDE_PAP_GUI_COMMONDATACLASSES_HPP_ */
