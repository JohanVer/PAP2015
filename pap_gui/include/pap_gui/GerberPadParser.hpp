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

class PadInformation{
	public:
	PadInformation(){

	}

	QPointF padPosition;
	QPointF padSize;
};

class GerberPadParser {
public:
	GerberPadParser();
	virtual ~GerberPadParser();
	float parseFloat(std::string line,std::size_t start, std::size_t end);
	void loadFile(std::string fileName);

private:
	std::vector<PadInformation> padInformationArray_;
};

#endif /* PAP_GUI_SRC_GERBERPADPARSER_HPP_ */
