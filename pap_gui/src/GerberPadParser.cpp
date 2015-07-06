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

						else if (shapeStr == "rectangle") {
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
			pad->padSize.setX(shapeInformationArray_[i].padDimensions.x());
			pad->padSize.setY(shapeInformationArray_[i].padDimensions.y());
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
				new QTableWidgetItem(QString::number(pad.padPosition.x())));

		table->setItem(i, 1,
				new QTableWidgetItem(QString::number(pad.padPosition.y())));

		table->setItem(i, 2,
				new QTableWidgetItem(QString::number(pad.padSize.x())));

		table->setItem(i, 3,
				new QTableWidgetItem(QString::number(pad.padSize.y())));

		table->setItem(i, 4,
						new QTableWidgetItem(QString::number(pad.rotation)));

		table->setItem(i, 5,
						new QTableWidgetItem(QString::fromStdString(pad.shapeStr)));
	}
}

void GerberPadParser::loadFile(std::string fileName) {

	std::ifstream infile;
	infile.open(fileName.c_str(), std::ios::in);

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
				newPad.padPosition.setX(xParsed * 25.4);
				newPad.padPosition.setY(yParsed * 25.4);
				// Convert d-code using tabular out of whl file
				searchShape(dCodeShape, &newPad);
				padInformationArray_.push_back(newPad);
				ROS_INFO(
						"Shape: %d , X-pos: %f , Y-pos: %f, X-Size: %f, Y-Size: %f, Rotation %f, Type: %s  ",
						dCodeShape, newPad.padPosition.x(),
						newPad.padPosition.y(), newPad.padSize.x(),
						newPad.padSize.y(), newPad.rotation,
						newPad.shapeStr.c_str());
			}
		}

	}
	ROS_INFO("Parsed %d pads...", (int )padInformationArray_.size());
}

void GerberPadParser::setSize(float height, float width) {
	width_ = width;
	height_ = height;
}

void GerberPadParser::renderImage(QGraphicsScene* scene, int width, int height) {
	cv::Rect pcbSize;

	pcbSize.x = 0;
	pcbSize.y = 0;
	double pixelConversionFactor = 0.0;
	double pixelWidth = (double) width / (double) width_;
	double pixelHeight = (double) height / (double) height_;
	//ROS_INFO("PixW %f PixH %f Cols %d Rows %d", pixelWidth, pixelHeight,
	//		dst->cols, dst->rows);

	if (pixelWidth > pixelHeight) {
		pixelConversionFactor = pixelHeight;
	} else {
		pixelConversionFactor = pixelWidth;
	}

	//ROS_INFO("PixConv %f", pixelConversionFactor);

	pcbSize.width = (unsigned int) (width_ * pixelConversionFactor);
	pcbSize.height = (unsigned int) (height_ * pixelConversionFactor);

	pcbSize.y = (unsigned int) (height - pcbSize.height) / 2;
	//ROS_INFO("X: %d Y: %d W: %d H: %d", pcbSize.x, pcbSize.y, pcbSize.width,
	//		pcbSize.height);

		QGraphicsRectItem *rect = new QGraphicsRectItem(pcbSize.x,pcbSize.y,pcbSize.width,pcbSize.height);
		rect->setPen( QPen( Qt::red, 3, Qt::DashDotLine ) );
		//rect->setBrush( Qt::gray );
		scene->addItem(rect);
		// Draw Pads

	for (std::size_t i = 0; i < padInformationArray_.size(); i++) {
		cv::Rect pad;
		PadInformation padInfo;
		padInfo = padInformationArray_[i];

		double upperCornerPadX = ((padInfo.padPosition.x()
				- padInfo.padSize.x() / 2.0) * pixelConversionFactor);
		double upperCornerPadY = (double) height
				- ((padInfo.padPosition.y() + padInfo.padSize.y() / 2.0)
						* pixelConversionFactor);
		pad.x = upperCornerPadX;
		pad.y = upperCornerPadY + pcbSize.y;
		//cv::circle(*dst,cv::Point2f(padInfo.padPosition.x()*pixelConversionFactor,(double)dst->rows-padInfo.padPosition.y()*pixelConversionFactor),4,cv::Scalar(255,0,0));
		pad.width =
				(unsigned int) (padInfo.padSize.x() * pixelConversionFactor);
		pad.height =
				(unsigned int) (padInfo.padSize.y() * pixelConversionFactor);
		//ROS_INFO("X: %d Y: %d W: %d H: %d", pad.x, pad.y, pad.width,
		//		pad.height);
		//cv::rectangle(*dst, pad, cv::Scalar(0, 0, 255), CV_FILLED);

		rectObject *rect = new rectObject(pad.x,pad.y,pad.width,pad.height);
		rect->setPen( QPen( Qt::red, 1, Qt::SolidLine ) );
		rect->setBrush( Qt::red );
		rect->id_ = i;
		scene->addItem(rect);

	}

}
