/*
 * GerberPadParser.cpp
 *
 *  Created on: Jul 5, 2015
 *      Author: johan
 */

#include "../include/pap_gui/GerberPadParser.hpp"

GerberPadParser::GerberPadParser() {
	// TODO Auto-generated constructor stub

}

GerberPadParser::~GerberPadParser() {
	// TODO Auto-generated destructor stub
}

float GerberPadParser::parseFloat(std::string line, std::size_t start,
		std::size_t end) {
	int numberLength = end - start - 1;
	std::string floatStr;
	if (numberLength <= 5) {
		floatStr = "0." + line.substr(start + 1, numberLength);
	} else {
		 floatStr = line.substr(start + 1, (numberLength - 5)) + "."
				+ line.substr(start + 1 + (numberLength - 5), 5);
	}
	QString* qtstring = new QString(floatStr.c_str());

	return qtstring->toFloat();
}

void GerberPadParser::loadFile(std::string fileName) {

	std::ifstream infile;
	infile.open(fileName.c_str(), std::ios::in);

	float xParsed,yParsed = 0.0;
	std::string line;
	int gCode,dCodeShape  = 0;
	while (std::getline(infile, line)) {
		std::istringstream iss(line);
		if (line[0] == '%') {
			continue;
		}

		if (line[0] == 'G'){
			// G-code
			QString* gCodeIndexStr = new QString(line.substr(1,2).c_str());
			gCode = gCodeIndexStr->toInt();
			std::size_t dFound = line.find("D");
			if(dFound != std::string::npos){
				QString* dCodeIndexStr = new QString(line.substr(dFound+1,2).c_str());
				dCodeShape = dCodeIndexStr->toInt();
			}
		}

		std::size_t xFound = line.find("X");
		if (xFound != std::string::npos) {
			std::size_t yFound = line.find("Y");
			if (yFound != std::string::npos) {
				xParsed = parseFloat(line,xFound,yFound);
			}
			std::size_t dFound = line.find("D");
			if(dFound != std::string::npos){
				yParsed = parseFloat(line,yFound,dFound);
				PadInformation newPad;
				newPad.padPosition.setX(xParsed*25.4);
				newPad.padPosition.setY(yParsed*25.4);
				padInformationArray_.push_back(newPad);
				ROS_INFO("Shape: %d",dCodeShape);
			}
		}

	}
	ROS_INFO("Parsed %d pads...",(int)padInformationArray_.size());
}
