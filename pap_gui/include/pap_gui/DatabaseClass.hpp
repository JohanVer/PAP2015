/*
 * DatabaseClass.hpp
 *
 *  Created on: Dec. 5, 2015
 *      Author: nikolas
 */

#ifndef PAP_GUI_SRC_DATABASECLASS_HPP_
#define PAP_GUI_SRC_DATABASECLASS_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <QString>
#include <QStandardItemModel>
#include <QTableWidget>
#include "CommonDataClasses.hpp"


class DatabaseClass {
public:
	DatabaseClass();
	virtual ~DatabaseClass();
	void load(void);
	void getAll(QVector<databaseEntry> *database);

private:
	std::vector<ShapeInformation> shapeInformationArray_;
	QVector<databaseEntry> databaseVector;
};

#endif /* PAP_GUI_SRC_DATABASECLASS_HPP_ */
