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
#include <QGraphicsObject>
#include <QRectF>

class PadInformation{
	public:
	PadInformation(){
		rotation = 0.0;
	}
	QRectF rect;
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
	int searchId(QPointF position,int height);

	// TODO: Make getter/setter for this public variable array
	std::vector<PadInformation> padInformationArray_;

private:
	std::vector<ShapeInformation> shapeInformationArray_;
	std::vector<QRectF> printedRects;
	double pixelConversionFactor;
	cv::Rect pcbSize;
	float height_,width_;
};

#endif /* PAP_GUI_SRC_GERBERPADPARSER_HPP_ */
