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
#include <pap_common/CommonDataClasses.hpp>


class DataIO {
public:
    DataIO();
    virtual ~DataIO();
    void loadDatabase(QVector<databaseEntry>& database);
    //void loadGerberFile(QVector<componentEntry>& components);

private:
	std::vector<ShapeInformation> shapeInformationArray_;
	QVector<databaseEntry> databaseVector;
};

#endif /* PAP_GUI_SRC_DATABASECLASS_HPP_ */
