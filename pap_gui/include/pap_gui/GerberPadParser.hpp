/*
 * GerberPadParser.hpp
 *
 *  Created on: Jul 5, 2015
 *      Author: johan
 */

#ifndef PAP_GUI_SRC_GERBERPADPARSER_HPP_
#define PAP_GUI_SRC_GERBERPADPARSER_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <QPoint>
#include <ros/ros.h>
#include <QString>
#include <QStandardItemModel>
#include <QTableWidget>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "cv.h"
#include "highgui.h"
#include <cmath>
#include "PadView.hpp"
#include <QGraphicsRectItem>

class PadInformation{
	public:
	PadInformation(){
		rotation = 0.0;
	}

	QPointF padPosition;
	QPointF padSize;
	std::string shapeStr;
	float rotation;
};

class ShapeInformation{
public:
	ShapeInformation(){
		shapeIndex = 0;
		shapeStr = "";
		rotation = 0.0;
	}
	int shapeIndex;
	std::string shapeStr;
	QPointF padDimensions;
	float rotation;
};

class GerberPadParser {
public:
	GerberPadParser();
	virtual ~GerberPadParser();
	float parseFloat(std::string line,std::size_t start, std::size_t end);
	void loadFile(std::string fileName);
	void parseShapes(std::string fileName);
	bool searchShape(int shapeIndex,PadInformation* pad);
	void setSize(float height, float width);
	void renderImage(QGraphicsScene* scene,int width, int height);
	void setTable(QTableWidget* table);

private:
	std::vector<PadInformation> padInformationArray_;
	std::vector<ShapeInformation> shapeInformationArray_;


	float height_,width_;
};

#endif /* PAP_GUI_SRC_GERBERPADPARSER_HPP_ */
