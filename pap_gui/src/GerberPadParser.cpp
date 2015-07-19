/*
 * GerberPadParser.cpp
 *
 *  Created on: Jul 5, 2015
 *      Author: johan
 */

#define CHANGE_SHAPE 54
#define ENDOFFILE_GERBER 02
#include "../include/pap_gui/GerberPadParser.hpp"

GerberPadParser::GerberPadParser() {
	height_ = 0.0;
	width_ = 0.0;
	outerRectRot_ = 0.0;
	pcbSize.setX(0.0);
	pcbSize.setY(0.0);

}

GerberPadParser::~GerberPadParser() {

}

void GerberPadParser::parseShapes(std::string fileName) {
	std::ifstream infile;
	infile.open(fileName.c_str(), std::ios::in);
	std::string line;
	while (std::getline(infile, line)) {
		if (line[0] == '%') {
			continue;
		}

		// D-Codes
		if (line[0] == 'D') {
			std::size_t emptyFound = line.find(" ");
			if (emptyFound != std::string::npos) {
				QString* dCodeIndexStr = new QString(
						line.substr(1, emptyFound - 1).c_str());
				int dCode = dCodeIndexStr->toInt();
				//ROS_INFO("D-Code : %d ", dCode);
				std::size_t wordFound = line.find_first_not_of(" ", emptyFound);
				if (wordFound != std::string::npos) {
					std::size_t emptyFound2 = line.find(" ", wordFound);
					if (emptyFound2 != std::string::npos) {

						// String
						std::string shapeStr = line.substr(wordFound,
								emptyFound2 - wordFound);
						//ROS_INFO("Shapestr : %s", shapeStr.c_str());

						if (shapeStr == "square") {
							size_t xFound = line.find("x", emptyFound2 + 1);
							if (xFound != std::string::npos) {

								// Arguments
								QString* firstArgStr = new QString(
										line.substr(emptyFound2,
												xFound - emptyFound2).c_str());
								float firstArg = firstArgStr->toFloat();
								QString* secondArgStr =
										new QString(
												line.substr(xFound + 1,
														line.size()
																- (xFound + 1)).c_str());
								float secondArg = secondArgStr->toFloat();
								ROS_INFO(
										"D-Code: %d Square X: %f, Rotation: %f",
										dCode, firstArg, secondArg);
								ShapeInformation shape;
								shape.shapeIndex = dCode;
								shape.shapeStr = shapeStr;
								shape.padDimensions.setX(firstArg * 25.4);
								shape.padDimensions.setY(firstArg * 25.4);
								shape.rotation = secondArg;
								shapeInformationArray_.push_back(shape);
							} else {
								// Arguments
								QString* firstArgStr = new QString(
										line.substr(emptyFound2,
												xFound - emptyFound2).c_str());
								float firstArg = firstArgStr->toFloat();

								ROS_INFO("D-Code: %d Square X: %f, Rotation: 0",
										dCode, firstArg);
								ShapeInformation shape;
								shape.shapeIndex = dCode;
								shape.shapeStr = shapeStr;
								shape.padDimensions.setX(firstArg * 25.4);
								shape.padDimensions.setY(firstArg * 25.4);
								shape.rotation = 0.0;
								shapeInformationArray_.push_back(shape);
							}
						}

						else if (shapeStr == "draw") {
							QString* drawArgStr = new QString(
									line.substr(emptyFound2,
											line.size() - emptyFound2).c_str());
							float drawArg = drawArgStr->toFloat();
							//ROS_INFO("Arg : %f", drawArg);
							ShapeInformation shape;
							shape.shapeIndex = dCode;
							shape.shapeStr = shapeStr;
							shape.padDimensions.setX(drawArg);
							shapeInformationArray_.push_back(shape);
						}

						else if (shapeStr == "rectangle"
								|| shapeStr == "oval") {
							size_t xFound = line.find("x", emptyFound2 + 1);
							float secondArg = 0.0;
							float thirdArg = 0.0;

							if (xFound != std::string::npos) {

								// Arguments
								QString* firstArgStr = new QString(
										line.substr(emptyFound2,
												xFound - emptyFound2).c_str());
								float firstArg = firstArgStr->toFloat();

								size_t xFound2 = line.find("x", xFound + 1);
								if (xFound2 != std::string::npos) {

									QString* secondArgStr =
											new QString(
													line.substr(xFound + 1,
															xFound2
																	- (xFound
																			+ 1)).c_str());
									secondArg = secondArgStr->toFloat();

									QString* thirdArgStr =
											new QString(
													line.substr(xFound2 + 1,
															line.size()
																	- (xFound2
																			+ 1)).c_str());
									thirdArg = thirdArgStr->toFloat();

								} else {
									QString* secondArgStr =
											new QString(
													line.substr(xFound + 1,
															line.size()
																	- (xFound
																			+ 1)).c_str());
									secondArg = secondArgStr->toFloat();
								}
								ROS_INFO(
										"D-Code: %d Rectangle X: %f, Y: %f, Rotation %f",
										dCode, firstArg, secondArg, thirdArg);
								ShapeInformation shape;
								shape.shapeIndex = dCode;
								shape.shapeStr = shapeStr;
								shape.padDimensions.setX(firstArg * 25.4);
								shape.padDimensions.setY(secondArg * 25.4);
								shape.rotation = thirdArg;
								shapeInformationArray_.push_back(shape);
							}
						}
					}
				}
			}
		}
	}
}

float GerberPadParser::parseFloat(std::string line, std::size_t start,
		std::size_t end) {
	int numberLength = end - start - 1;
	std::string floatStr;
	if (numberLength <= 5) {
		int zeros = 5 - numberLength;
		floatStr = "0.";
		for (int i = 0; i < zeros; i++) {
			floatStr = floatStr + "0";
		}
		floatStr = floatStr + line.substr(start + 1, numberLength);
	} else {
		floatStr = line.substr(start + 1, (numberLength - 5)) + "."
				+ line.substr(start + 1 + (numberLength - 5), 5);
	}
	QString* qtstring = new QString(floatStr.c_str());

	return qtstring->toFloat();
}

bool GerberPadParser::searchShape(int shapeIndex, PadInformation* pad) {
	for (int i = 0; i < shapeInformationArray_.size(); i++) {
		if (shapeInformationArray_[i].shapeIndex == shapeIndex) {
			pad->rect.setWidth(shapeInformationArray_[i].padDimensions.x());
			pad->rect.setHeight(shapeInformationArray_[i].padDimensions.y());
			pad->rotation = shapeInformationArray_[i].rotation;
			pad->shapeStr = shapeInformationArray_[i].shapeStr;
			return true;
		}
	}
	ROS_ERROR("Required Shape not found %d", shapeIndex);
	return false;
}

void GerberPadParser::setTable(QTableWidget* table) {

	table->setRowCount(padInformationArray_.size());
	QStringList vLabels;
	for (int i = 1; i <= padInformationArray_.size(); i++) {
		vLabels << QString::number(i);
	}
	table->setVerticalHeaderLabels(vLabels);
	for (std::size_t i = 0; i < padInformationArray_.size(); i++) {
		PadInformation pad;
		pad = padInformationArray_[i];
		table->setItem(i, 0,
				new QTableWidgetItem(QString::number(pad.rect.x())));

		table->setItem(i, 1,
				new QTableWidgetItem(QString::number(pad.rect.y())));

		table->setItem(i, 2,
				new QTableWidgetItem(QString::number(pad.rect.width())));

		table->setItem(i, 3,
				new QTableWidgetItem(QString::number(pad.rect.height())));

		table->setItem(i, 4,
				new QTableWidgetItem(QString::number(pad.rotation)));

		table->setItem(i, 5,
				new QTableWidgetItem(QString::fromStdString(pad.shapeStr)));
	}
}

void GerberPadParser::loadFile(std::string fileName) {

	std::ifstream infile;
	infile.open(fileName.c_str(), std::ios::in);
	padInformationArray_.clear();

	// Outlines are part of the first entry!

	PadInformation outerRect;
	outerRect.rect.setX(width_ / 2 + pcbSize.x());
	outerRect.rect.setY(height_ / 2 + pcbSize.y());
	outerRect.rotation = outerRectRot_;
	outerRect.rect.setHeight(height_);
	outerRect.rect.setWidth(width_);
	padInformationArray_.push_back(outerRect);

	float xParsed, yParsed = 0.0;
	std::string line;
	int gCode, dCodeShape = 0;
	while (std::getline(infile, line)) {
		std::istringstream iss(line);
		if (line[0] == '%') {
			continue;
		}

		if (line[0] == 'G') {
			// G-code
			QString* gCodeIndexStr = new QString(line.substr(1, 2).c_str());
			gCode = gCodeIndexStr->toInt();
			// D-Code
			std::size_t dFound = line.find("D");
			if (dFound != std::string::npos) {
				std::size_t endFound = line.find("*");
				if (endFound != std::string::npos) {
					QString* dCodeIndexStr =
							new QString(
									line.substr(dFound + 1,
											endFound - (dFound + 1)).c_str());
					if (gCode == CHANGE_SHAPE) {
						dCodeShape = dCodeIndexStr->toInt();
					}
					ROS_INFO("D-Code %d ", dCodeShape);
				}
			}
			continue;
		}

		// Misc codes
		if (line[0] == 'M') {
			QString* mCodeIndexStr = new QString(line.substr(1, 2).c_str());
			int mCodeIndex = mCodeIndexStr->toInt();
			if (mCodeIndex == ENDOFFILE_GERBER) {
				ROS_INFO("End of file reached");
				break;
			}
			continue;
		}

		std::size_t xFound = line.find("X");
		if (xFound != std::string::npos) {
			std::size_t yFound = line.find("Y");
			if (yFound != std::string::npos) {
				xParsed = parseFloat(line, xFound, yFound);
			}
			std::size_t dFound = line.find("D");
			if (dFound != std::string::npos) {
				yParsed = parseFloat(line, yFound, dFound);
				PadInformation newPad;
				newPad.rect.setX(width_ - (xParsed * 25.4));
				newPad.rect.setY(yParsed * 25.4);
				// Convert d-code using tabular out of whl file
				if (searchShape(dCodeShape, &newPad)) {
					padInformationArray_.push_back(newPad);
					ROS_INFO(
							"Shape: %d , X-pos: %f , Y-pos: %f, X-Size: %f, Y-Size: %f, Rotation %f, Type: %s  ",
							dCodeShape, newPad.rect.x(), newPad.rect.y(),
							newPad.rect.width(), newPad.rect.height(),
							newPad.rotation, newPad.shapeStr.c_str());
				}
			}
		}

	}

	ROS_INFO("Parsed %d pads...", (int )padInformationArray_.size());
}

void GerberPadParser::setSize(float height, float width) {
	width_ = width;
	height_ = height;
}

QRectF GerberPadParser::renderImage(QGraphicsScene* scene, int width,
		int height) {

	printedRects.clear();
	scene->clear();

//pcbSize.x = 0;
//pcbSize.y = 0;

	pixelConversionFactor = 0.0;
	double pixelWidth = (double) width / (double) width_;
	double pixelHeight = (double) height / (double) height_;
//ROS_INFO("PixW %f PixH %f Cols %d Rows %d", pixelWidth, pixelHeight,
//		dst->cols, dst->rows);

	if (pixelWidth > pixelHeight) {
		pixelConversionFactor = pixelHeight;
	} else {
		pixelConversionFactor = pixelWidth;
	}

	pcbSize.setWidth((unsigned int) (width_ * pixelConversionFactor));
	pcbSize.setHeight((unsigned int) (height_ * pixelConversionFactor));

	float pcbSizeY = (pcbSize.y());

	// Draw Pads
	for (std::size_t i = 0; i < padInformationArray_.size(); i++) {
		QRectF pad;
		PadInformation padInfo;
		padInfo = padInformationArray_[i];

		double upperCornerPadX =
				((padInfo.rect.x() - padInfo.rect.width() / 2.0)
						* pixelConversionFactor);
		double upperCornerPadY = (double) height
				- ((padInfo.rect.y() + padInfo.rect.height() / 2.0)
						* pixelConversionFactor);

		pad.setX(upperCornerPadX);
		pad.setY(upperCornerPadY);	// + pcbSizeY);

		pad.setWidth(
				(unsigned int) (padInfo.rect.width() * pixelConversionFactor));
		pad.setHeight(
				(unsigned int) (padInfo.rect.height() * pixelConversionFactor));

		ROS_INFO("X: %f Y: %f W: %f H: %f", pad.x(), pad.y(), pad.width(),
				pad.height());
		printedRects.push_back(pad);
		QGraphicsRectItem *rect = new QGraphicsRectItem(pad.x(), pad.y(),
				pad.width(), pad.height());
		rect->setPen(QPen(Qt::red, 1, Qt::SolidLine));
		if (i != 0) {
			rect->setBrush(Qt::red);
		} else {
			rect->setBrush(Qt::green);
		}
		rect->setTransformOriginPoint(padInfo.rect.x() * pixelConversionFactor,
				(double) height - (padInfo.rect.y() * pixelConversionFactor));
		rect->setRotation(padInfo.rotation);
		ROS_INFO("ROTATION: %f", padInfo.rotation);
		scene->addItem(rect);

	}

	return pcbSize;
}

int GerberPadParser::searchId(QPointF position, int height) {
	QPointF convPoint;
	convPoint.setX(position.x());
	convPoint.setY(position.y());
	// First rect is outline
	for (std::size_t i = 1; i < printedRects.size(); i++) {
		if (printedRects[i].contains(convPoint)) {
			return i;
		}
	}
	return -1;
}

visualization_msgs::MarkerArray* GerberPadParser::getMarkerList(void) {
	markerArray.markers.clear();

	for (std::size_t i = 0; i < padInformationArray_.size(); i++) {
		PadInformation padInfo;
		padInfo = padInformationArray_[i];
		// Create Marker

		visualization_msgs::Marker marker;
		marker.header.frame_id = "world";
		marker.header.stamp = ros::Time();
		marker.id = i + 1;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = (padInfo.rect.x())
				* 0.001;
		marker.pose.position.y = (padInfo.rect.y())
				* 0.001;
		if (i != 0) {
			marker.pose.position.z = 0.003;
		} else {
			marker.pose.position.z = 0.002;
		}
		tf::Quaternion rotQuat;
		rotQuat.setEuler(0.0, 0.0, -padInfo.rotation * (M_PI / 180.0));
		marker.pose.orientation.w = rotQuat.getW();
		marker.pose.orientation.x = rotQuat.getX();
		marker.pose.orientation.y = rotQuat.getY();
		marker.pose.orientation.z = rotQuat.getZ();

		marker.scale.x = padInfo.rect.width() * 0.001;
		marker.scale.y = padInfo.rect.height() * 0.001;
		marker.scale.z = 1 * 0.001;
		if (i != 0) {
			marker.scale.z = 0.01;
			marker.color.a = 1.0;
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
		} else {
			marker.scale.z = 0.01;
			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
		}
		markerArray.markers.push_back(marker);

	}
	return &markerArray;
}

float GerberPadParser::calibratePads(QPointF local1, QPointF local2,
		QPointF global1, QPointF global2) {

	float fixedAngle = atan2(global2.y() - global1.y(),
			global2.x() - global1.x());
	float cvAngle = atan2(local2.y() - local1.y(), local2.x() - local1.x());

	differenceAngle_ = M_PI / 3.0;		//fixedAngle - cvAngle;
	ROS_INFO("Calibration : Difference Angle %f", differenceAngle_);

// This calculates the translation of the pads in the global pad frame
	transformIntoGlobalPoint_.setOrigin(
			tf::Vector3(-global1.x(), -global1.y(), 0.0));
	transformIntoGlobalPoint_.setRotation(tf::Quaternion(0, 0, 0, 1));

// This transforms the pads into the robot frame
	transTransformIntoRobot_.setOrigin(
			tf::Vector3(local1.x() - global1.x(), local1.y() - global1.y(),
					0.0));
	transTransformIntoRobot_.setRotation(tf::Quaternion(0, 0, 0, 1));

// This rotates the pads in the global pad frame
	tf::Quaternion rotQuat;
	rotQuat.setEuler(0.0, 0.0, differenceAngle_);
	rotation_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	rotation_.setRotation(rotQuat);
}

void GerberPadParser::rotatePads(void) {
// The postitions of the pad are stored in it's own reference coordinate system
// 1. Rotate the pad positions around the global1 point about the angle which is
// calculated from the local positions
// 2. Transform all points of the global pad coordinate system into the local robot
// coordinate system

	tf::Transform backTransform = transformIntoGlobalPoint_;
	for (size_t i = 0; i < padInformationArray_.size(); i++) {

		tf::Point pointToTransform;
		// This point should be transformed
		pointToTransform.setX(padInformationArray_[i].rect.x());
		pointToTransform.setY(padInformationArray_[i].rect.y());
		pointToTransform.setZ(0.0);

		float width, height = 0.0;
		width = padInformationArray_[i].rect.width();
		height = padInformationArray_[i].rect.height();

		pointToTransform = transformIntoGlobalPoint_ * pointToTransform;
		pointToTransform = rotation_ * pointToTransform;
		pointToTransform = backTransform.inverse() * pointToTransform;

		padInformationArray_[i].rect.setX(pointToTransform.x());
		padInformationArray_[i].rect.setY(pointToTransform.y());
		padInformationArray_[i].rect.setWidth(width);
		padInformationArray_[i].rect.setHeight(height);
		padInformationArray_[i].rotation = padInformationArray_[i].rotation
				- (differenceAngle_ * (180.0 / M_PI));
		ROS_INFO("After X: %f Y: %f Width: %f Height %f",
				padInformationArray_[i].rect.x(),
				padInformationArray_[i].rect.y(),
				padInformationArray_[i].rect.width(),
				padInformationArray_[i].rect.height());
	}

	ROS_INFO("Calibration: Transformed %d pads...",
			(int )padInformationArray_.size());
}
