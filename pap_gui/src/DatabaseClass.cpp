/*
 * DatabaseClass.cpp
 *
 *  Created on: Dec. 5, 2015
 *      Author: nikoas
 */

#include <pap_gui/DatabaseClass.hpp>
#include <string>
#include <QMessageBox>

DataIO::DataIO() {
	// Init global variables
}

DataIO::~DataIO() {
}

void DataIO::loadDatabase(QVector<databaseEntry>& database) {

	std::fstream databaseFile;
	std::string fileName = std::string(getenv("PAPRESOURCES"))
			+ "database/database.txt";
	databaseFile.open(fileName.c_str(),
			std::fstream::in | std::fstream::out | std::fstream::app);

	/* ok, proceed  */
	if (databaseFile.is_open()) {

        std::string lineString;
		while (getline(databaseFile, lineString)) {

			QString componentString = QString::fromStdString(lineString);
			QRegExp sep(",");
			bool ok;

			/* Filter only valid database entries */
			if (!(componentString.at(0) == (char) 42)) {

				databaseEntry newDatabaseEntry;
				newDatabaseEntry.package = componentString.section(sep, 0, 0);
				newDatabaseEntry.length =
						componentString.section(sep, 1, 1).toFloat(&ok);
				newDatabaseEntry.width =
						componentString.section(sep, 2, 2).toFloat(&ok);
				newDatabaseEntry.height =
						componentString.section(sep, 3, 3).toFloat(&ok);
				newDatabaseEntry.pins =
						componentString.section(sep, 4, 4).toInt(&ok);
                database.append(newDatabaseEntry);
			}
		}

	} else {
		ROS_INFO("Could not open database!");
	}

	databaseFile.close();

    if (database.isEmpty()) {
		QMessageBox msgBox;
		msgBox.setText("No database found or not able to read database.");
		msgBox.exec();
		msgBox.close();
	}
}


