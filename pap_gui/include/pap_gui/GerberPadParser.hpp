/*
 * GerberPadParser.hpp
 *
 *  Created on: Jul 5, 2015
 *      Author: johan
 */

#ifndef PAP_GUI_SRC_GERBERPADPARSER_HPP_
#define PAP_GUI_SRC_GERBERPADPARSER_HPP_

//#include "DispenserPlanner.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <QPoint>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
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
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "CommonDataClasses.hpp"


class componentEntry {
public:
	std::string name, package, side, value;
	float posX, posY, length, width, height;
	int box, rotation, pins;
};

class GerberPadParser {
public:
	GerberPadParser();
	virtual ~GerberPadParser();
	float parseFloat(std::string line, std::size_t start, std::size_t end);
	void loadFile(std::string fileName,bool bottomLayer);
	void parseShapes(std::string fileName);
	bool searchShape(int shapeIndex, PadInformation* pad);
	void setSize(float height, float width);
	QRectF renderImage(QGraphicsScene* scene, int width, int height);
	void setTable(QTableWidget* table);
	int searchId(QPointF position, int height);
	float calibratePads(QPointF local1, QPointF local2, QPointF global1,
			QPointF global2);
	void rotatePads(void);
	void transformComponent(componentEntry *componentInformation);
	visualization_msgs::MarkerArray* getMarkerList(void);
	void transformDispenserInfo(dispenseInfo *info);
	// TODO: Make getter/setter for this public variable array
	std::vector<PadInformation> padInformationArray_,padInformationArrayPrint_;
	double pixelConversionFactor;
	float height_, width_,outerRectRot_;
	int heightPixel_;

private:
	std::vector<ShapeInformation> shapeInformationArray_;
	std::vector<QRectF> printedRects;
	QRectF pcbSize;

	tf::Transform transformIntoGlobalPoint_, transTransformIntoRobot_,rotation_;
	float differenceAngle_;
	visualization_msgs::MarkerArray markerArray;
	bool bottomLayer_;
};

#endif /* PAP_GUI_SRC_GERBERPADPARSER_HPP_ */
