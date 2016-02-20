/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <QtGui>
#include <QPixmap>
#include <QPainter>
#include <QKeyEvent>
#include <QMessageBox>
#include <iostream>
#include "../include/pap_gui/main_window.hpp"
#include "../include/pap_gui/GerberPadParser.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <QStandardItemModel>
#include <QVector>
#include <QString>
#include <QFile>
#include <tf/transform_broadcaster.h>
#include "../../pap_placer/include/pap_placer/offsetTable.hpp"
#include "../include/pap_gui/DatabaseClass.hpp"
#include "../include/pap_gui/packageDialog.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace pap_gui {

using namespace Qt;
using namespace std;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

bool bottomLEDon = false;
int componentCount = 0;
float xTapeCalibration, yTapeCalibration, rotTapeCalibration = 0;

MainWindow::MainWindow(int version, int argc, char** argv, QWidget *parent) :
		QMainWindow(parent), qnode(argc, argv) {
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	ui.centralwidget->setMouseTracking(true);

	QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
			SLOT(aboutQt())); // qApp is a global variable for the application

	ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));

	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	 ** Logging
	 **********************/
	ui.view_logging->setModel(qnode.loggingModel());
	QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
			SLOT(updateLoggingView()));

	/*********************
	 ** Auto Start
	 **********************/
	if (ui.checkbox_remember_settings->isChecked()) {
		on_button_connect_clicked(true);
	}

	//Vision
	QObject::connect(&qnode,
			SIGNAL(smdCoordinates(float ,float ,float ,unsigned int)), this,
			SLOT(displaySMDCoords(float ,float ,float,unsigned int )));
	QWidget::connect(ui.camera1, SIGNAL(sendMousePoint(QPointF)), this,
			SLOT(setCamera1Point(QPointF)));
	QWidget::connect(ui.camera1, SIGNAL(setFiducial(QPointF)), this,
			SLOT(setFiducial(QPointF)));

	QWidget::connect(&qnode, SIGNAL(signalPosition(float,float)), this,
			SLOT(signalPosition(float,float)));

	QWidget::connect(ui.fiducialTable, SIGNAL(sendGotoFiducial(int)), this,
			SLOT(sendGotoFiducial(int)));

	QWidget::connect(&scenePads_, SIGNAL(sendMousePoint(int ,QPointF)), this,
			SLOT(padPressed(int,QPointF)));

	QWidget::connect(&scenePads_, SIGNAL(gotoPad(QPointF)), this,
			SLOT(gotoPad(QPointF)));

	QWidget::connect(&scenePads_, SIGNAL(deletePad(QPointF)), this,
			SLOT(deletePad(QPointF)));

	QWidget::connect(&scenePads_, SIGNAL(dispensePad(QPointF)), this,
			SLOT(dispenseSinglePad(QPointF)));

	// Cameras
	QObject::connect(&qnode, SIGNAL(cameraUpdated(int )), this,
			SLOT(cameraUpdated(int )));

	// Status
	QObject::connect(&qnode, SIGNAL(statusUpdated(int)), this,
			SLOT(statusUpdated(int)));

	// Tip status
	QObject::connect(&qnode, SIGNAL(tipToggled(int,bool)), this,
			SLOT(tipToggled(int,bool)));

	// Placer status / QR Code scanner
	QObject::connect(&qnode, SIGNAL(placerStatusUpdated(int, int)), this,
			SLOT(placerStatusUpdated(int, int)));

	//LED Slider
	connect(ui.ringBrightnessSlider, SIGNAL(valueChanged(int)), this,
			SLOT(changeRingLEDBrightness(int)));

	connect(ui.backBrightnessSlider, SIGNAL(valueChanged(int)), this,
			SLOT(changeBackLEDBrightness(int)));

	connect(ui.topBrightnessSlider_2, SIGNAL(valueChanged(int)), this,
			SLOT(changeTopLEDBrightness(int)));

	// Color Combo LED Ring
	connect(ui.ringColorCombo, SIGNAL(activated(int)), this,
			SLOT(changeRingColor(int)));

	connect(ui.xManPos, SIGNAL(released()), this, SLOT(releasexManPos()));
	connect(ui.xManNeg, SIGNAL(released()), this, SLOT(releasexManNeg()));
	connect(ui.YManPos, SIGNAL(released()), this, SLOT(releaseyManPos()));

	connect(ui.YManNeg, SIGNAL(released()), this, SLOT(releaseyManNeg()));
	connect(ui.ZManPos, SIGNAL(released()), this, SLOT(releasezManPos()));
	connect(ui.ZManNeg, SIGNAL(released()), this, SLOT(releasezManNeg()));

	valve1Active_ = false;
	valve2Active_ = false;
	valve3Active_ = false;
	valve4Active_ = false;
	valve5Active_ = false;
	valve6Active_ = false;
	valve7Active_ = false;
	valve8Active_ = false;
	alreadyFlipped_ = false;

	tip1Pos_ = 0.0;
	tip2Pos_ = 0.0;

	completePlacementRunning = false;
	componentIndicator = 0;

	xTapeCalibration = 0.0;
	yTapeCalibration = 0.0;
	rotTapeCalibration = 0.0;

	/* Load database */
	//database.load();
	//database.getAll(&databaseVector);
	loadDatabaseContent();
	updateDatabaseTable();

	initFiducialTable();
	initPadTable(1);
	id_ = 0;
	sizeDefined_ = false;
	padFileLoaded_ = false;
	dispenserPaused = false;
	lastDispenserId = 0;

	currentPosition.x = 0.0;
	currentPosition.y = 0.0;
	//ui.checkBox_box->setDisabled(true);
	for (size_t i = 0; i < 20; i++) {
		tapeCompCounter[i] = 0;
	}

	// Init placer status
	for (int i = 1; i <= 7; i++) {
		placerStatusUpdated(i, pap_common::PLACER_IDLE);
	}

	// Disable developer tabs if user version selected
	if (version == 1) {
		ui.tab_manager->setTabEnabled(3, false);
		ui.tab_manager->setTabEnabled(4, false);
		ui.tab_manager->setTabEnabled(5, false);
		ui.tab_manager->setCurrentIndex(0);
		on_button_connect_clicked(true);
		ros::Duration(1).sleep();
		qnode.sendTask(pap_common::PLACER, pap_common::IDLE);
	} else {
		// Show Ros status dockwidget
		ui.dock_status->show();
	}
}

MainWindow::~MainWindow() {
}

/********************************************************************"*********
 ** Implementation "Complete PCB"-Tab
 *****************************************************************************/
void MainWindow::on_loadGerberFileButton_clicked() {

	/* Clear component vector and reset component number*/
	componentList.clear();
	packageList.clear();
	missingPackageList.clear();
	updateMissingPackageList();
	updateComponentTable();
	updateComponentInformation();
	componentCount = 0;

	//get a filename to open
	QString gerberFile = QFileDialog::getOpenFileName(this,
			tr("Open Gerber file"), "/home", tr("Text Files (*.txt *.csv)"));
	std::cout << "Got filename: " << gerberFile.toStdString() << std::endl;

	/* Load gerber file and add components to componentList */
	std::fstream datafile;
	const char *filename = gerberFile.toLatin1().data();
	datafile.open(filename,
			std::fstream::in | std::fstream::out | std::fstream::app);

	/* ok, proceed  */
	if (datafile.is_open()) {

		string componentString;
		int comp_index = 0;
		while (getline(datafile, componentString)) {

			/* Filter only component data */
			if (!(componentString.at(0) == (char) 42)) {

				componentEntry newComponent;
				//newComponent.value = componentString.section(sep, 0, 0);

				int pos1 = componentString.find('"');
				componentString = componentString.substr(pos1 + 1,
						componentString.size() - pos1);

				// Be careful - neglecting one position!!!
				pos1 = componentString.find('//');
				newComponent.value = componentString.substr(0, pos1);
				componentString = componentString.substr(pos1 + 2,
						componentString.size() - pos1);

				pos1 = componentString.find('"');
				newComponent.package = componentString.substr(0, pos1);
				componentString = componentString.substr(pos1 + 3,
						componentString.size() - pos1);

				pos1 = componentString.find('"');
				QString* posX = new QString(
						componentString.substr(0, pos1).c_str());
				newComponent.posX = posX->toFloat();
				componentString = componentString.substr(pos1 + 3,
						componentString.size() - pos1);

				pos1 = componentString.find('"');
				QString* posY = new QString(
						componentString.substr(0, pos1).c_str());
				newComponent.posY = posY->toFloat();
				componentString = componentString.substr(pos1 + 3,
						componentString.size() - pos1);

				pos1 = componentString.find('"');
				newComponent.rotation = atoi(
						(componentString.substr(0, pos1)).c_str());
				componentString = componentString.substr(pos1 + 2,
						componentString.size() - pos1);

				pos1 = componentString.find(',');
				newComponent.side = componentString.substr(0, pos1);
				componentString = componentString.substr(pos1 + 2,
						componentString.size() - pos1);

				pos1 = componentString.find('"');
				newComponent.name = componentString.substr(0, pos1);

				// Set box number of component
				newComponent.box = -1;

				newComponent.index = comp_index;
				comp_index++;

				componentList.append(newComponent);
				componentCount++;
			}
		}

	} else {
		std::cout << "Could not open file!" << std::endl;
	}

	datafile.close();
	updateComponentTable();

	/* Check if components in database */
	updatePackageList();
	updateMissingPackageList();

	if (!missingPackageList.isEmpty()) {
		QMessageBox msgBox;
		QString message =
				QString(
						"There are %1 unknown packages which are needed to perform a complete PCB placement.").arg(
						missingPackageList.size());
		msgBox.setText(message);
		msgBox.exec();
		msgBox.close();

		/* Jump to database tab and show missing packages*/
		ui.tab_manager->setCurrentIndex(2);
	}
}

void MainWindow::on_clearTableButton_clicked() {

	componentCount = 0;
	componentList.clear();
	packageList.clear();
	missingPackageList.clear();
	updateMissingPackageList();
	updateComponentTable();
	updateComponentInformation();
}

void MainWindow::on_startSlotWizard_clicked() {

	// Check if table empty
	if (!componentList.isEmpty()) {

		// All types of components need to be known
		if (missingPackageList.isEmpty()) {
			SlotSelectorDialog slotWizard(&componentList, &databaseVector);
			connect(&slotWizard, SIGNAL(setLed(int)), this,
					SLOT(setLedFromSelection(int)));
			slotWizard.exec();
			updateComponentInformation();
		} else {
			QMessageBox msgBox;
			QString message =
					QString(
							"There are still %1 unknown packages which are needed to perform a complete PCB placement.").arg(
							missingPackageList.size());
			msgBox.setText(message);
			msgBox.exec();
			msgBox.close();
		}

	} else {
		QMessageBox msgBox;
		msgBox.setText("Component table empty. Please load gerber file first.");
		msgBox.exec();
		msgBox.close();
	}
}

void MainWindow::on_compDeleteButton_clicked() {

	/* Get current component */
	int currentComp = ui.tableWidget->currentRow();
	if (currentComp == -1) {
		QMessageBox msgBox;
		msgBox.setText("Please select a component.");
		msgBox.exec();
		msgBox.close();
	} else { /* Delete component from list */
		componentList.remove(currentComp);
		updateComponentTable();
		updateComponentInformation();
	}
	updateMissingPackageList();
	updateMissingPackageTable();
}

void MainWindow::on_compOrientButton_clicked() {

	/* Get current component */
	int currentComponent = ui.tableWidget->currentRow();

	/* If no component selected */
	if (currentComponent == -1) {
		QMessageBox msgBox;
		msgBox.setText("Please select a component.");
		msgBox.exec();
		msgBox.close();

	} else {

		bool ok = false;
		QInputDialog* inputDialog = new QInputDialog();
		inputDialog->setOptions(QInputDialog::NoButtons);

		/* Get boxNumber input */
		int currentRotation = componentList.at(currentComponent).rotation;
		int rotation = inputDialog->getInt(this, "Change component orientation",
				"Enter component orientation:", currentRotation);
		componentList[currentComponent].rotation = rotation;
		ui.label_compOrient->setText(QString::number(rotation));
	}
}

void MainWindow::on_compPackageButton_clicked() {

	/* Get current component */
	int currentComponent = ui.tableWidget->currentRow();

	/* If no component selected */
	if (currentComponent == -1) {
		QMessageBox msgBox;
		msgBox.setText("Please select a component.");
		msgBox.exec();
		msgBox.close();

	} else {

		bool ok = false;
		QInputDialog* inputDialog = new QInputDialog();
		inputDialog->setOptions(QInputDialog::NoButtons);

		/* Get boxNumber input */
		QString currentPackage = QString::fromStdString(
				componentList.at(currentComponent).package);
		QString text = inputDialog->getText(this, "Change package",
				"Enter new package:", QLineEdit::Normal, currentPackage, &ok);
		if (ok && !text.isEmpty()) {
			ui.label_compPackage->setText(text);
			componentList[currentComponent].package = text.toStdString();
		}
		updateComponentTable();
		updateMissingPackageList();
		updateMissingPackageTable();
	}
}

void MainWindow::on_startPlacementButton_clicked() {

	// Components to place?
	if (componentList.isEmpty()) {
		QMessageBox msgBox;
		msgBox.setText("Component table empty. Please load gerber file first.");
		msgBox.exec();
		msgBox.close();
		return;
	}
	// Missing packages?
	updateMissingPackageList();
	if (!missingPackageList.isEmpty()) {
		QMessageBox msgBox;
		QString message = QString("There are still %1 unknown packages.").arg(
				missingPackageList.size());
		msgBox.setText(message);
		msgBox.exec();
		msgBox.close();
		return;
	}
	// Missing slots?
	if (emptySlots()) {
		QMessageBox msgBox;
		msgBox.setText("There are still components without assigned slot.");
		msgBox.exec();
		msgBox.close();
		return;
	}
	// Start placement process
	if (!completePlacementRunning) {
		completePlacementRunning = true;
		componentIndicator = 0;
		updatePlacementData(componentList[componentIndicator]);
		qnode.sendTask(pap_common::PLACER, pap_common::COMPLETEPLACEMENT,
				placementData);
		ui.label_compLeft->setText(QString::number(componentList.size()-componentIndicator));
		ui.label_currentComp->setText(QString::fromStdString(componentList.at(componentIndicator).name));
	} else {
		QMessageBox msgBox;
		msgBox.setText("Placement process already running - stop first.");
		msgBox.exec();
		msgBox.close();
	}
}

bool MainWindow::emptySlots() {
	for (size_t k = 0; k < componentList.size(); k++) {
		if (componentList.at(k).box == -1)
			return true;
	}
	return false;
}

void MainWindow::on_stopPlacementButton_clicked() {
	// Placer will stop/home once current comp placed
	qnode.sendTask(pap_common::PLACER, pap_common::STOP);
}

/********************************************************************"*********
 ** Implementation "Package Database"-Tab
 *****************************************************************************/
void MainWindow::on_replaceButton_clicked() {

	int database_package = ui.packageTableWidget->currentRow();
	int missing_package = ui.missingPackageTableWidget->currentRow();
	int replaceCounter = 0;

	if (database_package == -1) {
		QMessageBox msgBox;
		msgBox.setText("No package from database selected.");
		msgBox.exec();
		msgBox.close();
	} else if (missing_package == -1) {
		QMessageBox msgBox;
		msgBox.setText("No missing package selected.");
		msgBox.exec();
		msgBox.close();
	} else {

		QString databasePackage = databaseVector.at(database_package).package;
		QString missingPackage = QString::fromStdString(
				missingPackageList.at(missing_package));
		QString message =
				QString("To replace package '%1' by '%2'n click yes.").arg(
						missingPackage).arg(databasePackage);

		QMessageBox::StandardButton reply;
		reply = QMessageBox::question(this, "Check replacement", message,
				QMessageBox::Yes | QMessageBox::No);
		if (reply == QMessageBox::Yes) {
			// Replace missing package by the given database package.
			for (size_t i = 0; i < componentList.size(); i++) {
				if (componentList.at(i).package.compare(
						missingPackageList.at(missing_package)) == 0) {
					componentList[i].package = databasePackage.toStdString();
					replaceCounter++;
				}
			}
			updateComponentTable();
			updateComponentInformation();
			updateMissingPackageList();
			updateMissingPackageTable();
			ROS_INFO("%d components have been replaced.", replaceCounter);
		}
	}
}

void MainWindow::on_addPackageButton_clicked() {
	int missing_package = ui.missingPackageTableWidget->currentRow();
	PackageDialog* packageDialog = new PackageDialog(&databaseVector, &missingPackageList, missing_package);
	packageDialog->exec();
	updateDatabaseTable();
	updateMissingPackageList();
	updateMissingPackageTable();
}

void MainWindow::on_editPackageButton_clicked() {

	int current_package = ui.packageTableWidget->currentRow();
	if (current_package != -1) {
		PackageDialog* packageDialog = new PackageDialog(&databaseVector, NULL, current_package);
		packageDialog->exec();
		updateDatabaseTable();
		updateMissingPackageList();
		updateMissingPackageTable();
	} else {
		QMessageBox msgBox;
		msgBox.setText("No package selected.");
		msgBox.exec();
		msgBox.close();
	}
}

void MainWindow::on_deletePackageButton_clicked() {

	int current_package = ui.packageTableWidget->currentRow();
	if (current_package != -1) {
		databaseVector.remove(current_package);
		updateDatabaseTable();
		updateMissingPackageList();
		updateMissingPackageTable();
	} else {
		QMessageBox msgBox;
		msgBox.setText("No package selected.");
		msgBox.exec();
		msgBox.close();
	}
}

/********************************************************************"*********
 ** Implementation "Single Component"-Tab
 *****************************************************************************/

void MainWindow::setLedFromSelection(int selection) {
	if (selection != -1) {
		ROS_INFO("Set led number %d", selection);
		qnode.setLEDTask(selection);
	} else {
		qnode.LEDTask(pap_common::RESETALLLED, 0);
	}
}

void MainWindow::loadDatabaseContent() {

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
				databaseVector.append(newDatabaseEntry);
			}
		}

	} else {
		QMessageBox msgBox;
		msgBox.setText("Could not open database!");
		msgBox.exec();
		msgBox.close();
	}

	databaseFile.close();
	if (databaseVector.isEmpty()) {
		QMessageBox msgBox;
		msgBox.setText("Empty database.");
		msgBox.exec();
		msgBox.close();
	}
}


void MainWindow::updateComponentInformation() {

	int currentComp = ui.tableWidget->currentRow();

	if (currentComp == -1) {
		ui.label_compName->setText("-");
		ui.label_compValue->setText("-");
		ui.label_compPackage->setText("-");
		ui.label_compLength->setText("-");
		ui.label_compWidth->setText("-");
		ui.label_compHeight->setText("-");
		ui.label_compPins->setText("-");
		ui.label_compPos->setText("-");
		ui.label_compOrient->setText("-");
		ui.label_compSide->setText("-");
		ui.label_compBox->setText("-");

	} else {

		ui.label_compName->setText(componentList.at(currentComp).name.c_str());
		ui.label_compValue->setText(
				componentList.at(currentComp).value.c_str());
		ui.label_compPackage->setText(
				componentList.at(currentComp).package.c_str());

		int packageID = -1;
		for (int i = 0; i < databaseVector.size(); i++) {
			string currentPackage = componentList.at(currentComp).package;

			if (databaseVector.at(i).package
					== QString::fromStdString(currentPackage)) {
				packageID = i;
				break;
			}
		}

		if (packageID == -1) {		// Package not found
			ui.label_compLength->setText("not found");
			ui.label_compWidth->setText("not found");
			ui.label_compHeight->setText("not found");
			ui.label_compPins->setText("not found");
		} else {					// Package found
			ui.label_compLength->setText(
					QString::number(databaseVector.at(packageID).length, 'f',
							2));
			componentList[currentComp].length =
					databaseVector.at(packageID).length;
			ui.label_compWidth->setText(
					QString::number(databaseVector.at(packageID).width, 'f',
							2));
			componentList[currentComp].width =
					databaseVector.at(packageID).width;
			ui.label_compHeight->setText(
					QString::number(databaseVector.at(packageID).height, 'f',
							2));
			ui.label_compPins->setText(
					QString::number(databaseVector.at(packageID).pins));
		}

		ui.label_compPos->setText(
				QString::number(componentList.at(currentComp).posX, 'f', 2)
						+ " / "
						+ QString::number(componentList.at(currentComp).posY,
								'f', 2));
		ui.label_compOrient->setText(
				QString::number(componentList.at(currentComp).rotation));
		ui.label_compSide->setText(componentList.at(currentComp).side.c_str());

		if (componentList.at(currentComp).box == -1) {
			ui.label_compBox->setText("unknown");
		} else {
			ui.label_compBox->setText(
					QString::number(componentList.at(currentComp).box));
		}
	}
}

void MainWindow::on_tableWidget_clicked() {

	updateComponentInformation();
}

void MainWindow::updatePackageList() {
	bool inPackageList;
	packageList.clear();

	/* Iterate over all components*/
	for (size_t i = 0; i < componentList.size(); i++) {

		inPackageList = false;
		for (size_t k = 0; k < packageList.size(); k++) {
			/* Check if package already in List */
			if (componentList.at(i).package.compare(packageList.at(k)) == 0) {
				inPackageList = true;
				break;
			}
		}

		/* If package not yet in packageList, add it */
		if (!inPackageList) {
			packageList.push_back(componentList.at(i).package);
		}
	}
}

void MainWindow::updateMissingPackageList() {
	updatePackageList();
	bool inDatabase;
	missingPackageList.clear();

	/* Iterate over all parts */
	for (size_t i = 0; i < packageList.size(); i++) {

		inDatabase = false;
		for (size_t k = 0; k < databaseVector.size(); k++) {
			/* Check if part is in database */
			if (packageList.at(i).compare(
					databaseVector.at(k).package.toStdString()) == 0) {
				inDatabase = true;
				break;
			}
		}

		/* If part is not in database -> add */
		if (!inDatabase) {
			missingPackageList.push_back(packageList.at(i));
		}
	}
	updateMissingPackageTable();
}

void MainWindow::updateMissingPackageTable() {

	/* Set size of table */
	ui.missingPackageTableWidget->setRowCount(missingPackageList.size());
	ui.missingPackageTableWidget->setColumnCount(1);

	// Set labels
	QStringList hLabels, vLabels;
	hLabels << "Package";
	for (int i = 1; i < missingPackageList.size(); i++) {
		vLabels << QString::number(i);
	}
	ui.missingPackageTableWidget->setHorizontalHeaderLabels(hLabels);
	ui.missingPackageTableWidget->setVerticalHeaderLabels(vLabels);

	// Set content
	for (int i = 0; i < ui.missingPackageTableWidget->rowCount(); i++) {
		ui.missingPackageTableWidget->setItem(i, 0,
				new QTableWidgetItem(missingPackageList.at(i).c_str()));
	}

	// Table settings
	ui.missingPackageTableWidget->setSelectionMode(
			QAbstractItemView::SingleSelection);
	ui.missingPackageTableWidget->setSelectionBehavior(
			QAbstractItemView::SelectRows);
	ui.missingPackageTableWidget->setSizePolicy(QSizePolicy::Expanding,
			QSizePolicy::Maximum);
	ui.missingPackageTableWidget->horizontalHeader()->setResizeMode(
			QHeaderView::Stretch);
	ui.missingPackageTableWidget->show();
}

void MainWindow::updateComponentTable() {

	// Set size of table
	ui.tableWidget->setRowCount(componentList.size());
	ui.tableWidget->setColumnCount(3);

	// Set labels
	QStringList hLabels, vLabels;
	hLabels << "Name" << "Value" << "Package";
	for (int i = 1; i < componentList.size(); i++) {
		vLabels << QString::number(i);
	}
	ui.tableWidget->setHorizontalHeaderLabels(hLabels);
	ui.tableWidget->setVerticalHeaderLabels(vLabels);

	// Set content
	for (int i = 0; i < ui.tableWidget->rowCount(); i++) {

		ui.tableWidget->setItem(i, 0,
				new QTableWidgetItem((componentList.at(i).name).c_str()));
		ui.tableWidget->setItem(i, 1,
				new QTableWidgetItem((componentList.at(i).value).c_str()));
		ui.tableWidget->setItem(i, 2,
				new QTableWidgetItem((componentList.at(i).package).c_str()));

	}

	// Table settings
	ui.tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
	ui.tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
	//ui.tableWidget->verticalHeader()->setVisible(false);
	ui.tableWidget->setSizePolicy(QSizePolicy::Expanding,
			QSizePolicy::Expanding);
	ui.tableWidget->horizontalHeader()->setResizeMode(QHeaderView::Stretch);

	//ui.tableWidget->setWindowTitle("QTableWidget");
	ui.tableWidget->show();

	// Set number of components
	ui.label_compTotal->setText(QString::number(componentList.size()));
	ui.label_compLeft->setText(QString::number(componentList.size()));
}

void MainWindow::updateDatabaseTable() {

	// Set size of table
	ui.packageTableWidget->setRowCount(databaseVector.size());
	ui.packageTableWidget->setColumnCount(5);

	// Set labels
	QStringList hLabels, vLabels;
	hLabels << "Package" << "Length" << "Width" << "Height" << "# of Pins";
	for (int i = 1; i < componentList.size(); i++) {
		vLabels << QString::number(i);
	}
	ui.packageTableWidget->setHorizontalHeaderLabels(hLabels);
	ui.packageTableWidget->setVerticalHeaderLabels(vLabels);

	// Set content
	for (int i = 0; i < ui.packageTableWidget->rowCount(); i++) {

		ui.packageTableWidget->setItem(i, 0,
				new QTableWidgetItem(databaseVector.at(i).package));
		ui.packageTableWidget->setItem(i, 1,
				new QTableWidgetItem(
						QString::number(databaseVector.at(i).length)));
		ui.packageTableWidget->setItem(i, 2,
				new QTableWidgetItem(
						QString::number(databaseVector.at(i).width)));
		ui.packageTableWidget->setItem(i, 3,
				new QTableWidgetItem(
						QString::number(databaseVector.at(i).height)));
		ui.packageTableWidget->setItem(i, 4,
				new QTableWidgetItem(
						QString::number(databaseVector.at(i).pins)));
		ui.packageTableWidget->item(i, 1)->setTextAlignment(Qt::AlignCenter);
		ui.packageTableWidget->item(i, 2)->setTextAlignment(Qt::AlignCenter);
		ui.packageTableWidget->item(i, 3)->setTextAlignment(Qt::AlignCenter);
		ui.packageTableWidget->item(i, 4)->setTextAlignment(Qt::AlignCenter);
	}

	// Table settings
	ui.packageTableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
	ui.packageTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
	ui.tableWidget->verticalHeader()->setVisible(false);
	ui.packageTableWidget->setSizePolicy(QSizePolicy::Expanding,
			QSizePolicy::Expanding);
	ui.packageTableWidget->horizontalHeader()->setResizeMode(
			QHeaderView::Stretch);

	//ui.tableWidget->setWindowTitle("QTableWidget");
	ui.tableWidget->show();
}

/*************************************************************************************************************
 ** Placement control buttons
 **************************************************************************************************************/

void MainWindow::on_goToPCBButton_clicked() {
	ui.tab_manager->setCurrentIndex(1);
	qnode.sendTask(pap_common::PLACER, pap_common::GOTOPCB, placementData);
}

// Package that is going to be sent to placeController
void MainWindow::updatePlacementData(componentEntry &singleComponentIn) {

	componentEntry entryToTransform;
	entryToTransform = singleComponentIn;

	ROS_INFO("placerInfo - before: x%f, y=%f", entryToTransform.posX,
			entryToTransform.posY);
	padParser.transformComponent(&entryToTransform);
	ROS_INFO("placerInfo - after: x%f, y=%f", entryToTransform.posX,
			entryToTransform.posY);

	// Is it a tape?
	if ((entryToTransform.box >= 67) && (entryToTransform.box <= 86)) {
		unsigned int tape_nr = entryToTransform.box - 67;
		tapeCalibrationValue tapePartPos = calculatePosOfTapePart(tape_nr,
				tapeCompCounter[tape_nr]);
		placementData.tapeX = tapePartPos.x;
		placementData.tapeY = tapePartPos.y;
		placementData.tapeRot = tapePartPos.rot;
		tapeCompCounter[tape_nr]++;
	} else {
		placementData.tapeX = 0.0;
		placementData.tapeY = 0.0;
		placementData.tapeRot = 0.0;
	}

	placementData.destX = entryToTransform.posX;
	placementData.destY = entryToTransform.posY;
	placementData.box = entryToTransform.box;
	placementData.height = entryToTransform.height;
	placementData.length = entryToTransform.length;
	placementData.width = entryToTransform.width;
	placementData.rotation = entryToTransform.rotation;
}

void MainWindow::on_placeSingleComponentButton_clicked() {

	int currentComp = ui.tableWidget->currentRow();
	if (currentComp == -1) {
		QMessageBox msgBox;
		msgBox.setText("Please select a component.");
		msgBox.exec();
		msgBox.close();
	} else {

/////////////////// CHANGE //////////////////////////////
		if (singleComponent.box == -1) {
			QMessageBox msgBox;
			msgBox.setText("Please set box number first.");
			msgBox.exec();
			msgBox.close();
		} else {
			updatePlacementData(singleComponent);
			qnode.sendTask(pap_common::PLACER, pap_common::PLACECOMPONENT,
					placementData);
		}
		singleComponent = componentList.at(currentComp);
		//updateSingleComponentInformation();
	}
}

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
	close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check) {
	if (ui.checkbox_use_environment->isChecked()) {
		if (!qnode.init()) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if (!qnode.init(ui.line_edit_master->text().toStdString(),
				ui.line_edit_host->text().toStdString())) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if (state == 0) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

void MainWindow::updateLoggingView() {
	ui.view_logging->scrollToBottom();
}

void MainWindow::on_actionAbout_triggered() {
	QMessageBox::about(this, tr("About ..."),
			tr(
					"<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

void MainWindow::ReadSettings() {
	QSettings settings("Qt-Ros Package", "pap_gui");
	restoreGeometry(settings.value("geometry").toByteArray());
	restoreState(settings.value("windowState").toByteArray());
	QString master_url = settings.value("master_url",
			QString("http://192.168.1.2:11311/")).toString();
	QString host_url =
			settings.value("host_url", QString("192.168.1.3")).toString();
	//QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
	ui.line_edit_master->setText(master_url);
	ui.line_edit_host->setText(host_url);
	//ui.line_edit_topic->setText(topic_name);
	bool remember = settings.value("remember_settings", false).toBool();
	ui.checkbox_remember_settings->setChecked(remember);
	bool checked = settings.value("use_environment_variables", false).toBool();
	ui.checkbox_use_environment->setChecked(checked);
	if (checked) {
		ui.line_edit_master->setEnabled(false);
		ui.line_edit_host->setEnabled(false);
		//ui.line_edit_topic->setEnabled(false);
	}
}

void MainWindow::WriteSettings() {
	QSettings settings("Qt-Ros Package", "pap_gui");
	settings.setValue("master_url", ui.line_edit_master->text());
	settings.setValue("host_url", ui.line_edit_host->text());
	//settings.setValue("topic_name",ui.line_edit_topic->text());
	settings.setValue("use_environment_variables",
			QVariant(ui.checkbox_use_environment->isChecked()));
	settings.setValue("geometry", saveGeometry());
	settings.setValue("windowState", saveState());
	settings.setValue("remember_settings",
			QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event) {
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::cameraUpdated(int index) {
	if (index == 1) {
		int width = ui.camera1->width();
		int height = ui.camera1->height() - 2;

		QImage camera1Scaled = qnode.getCamera1()->scaled(width, height,
				Qt::KeepAspectRatio);
		cameraPicture1 = QPixmap::fromImage(camera1Scaled);
		scene_.clear();
		scene_.addPixmap(cameraPicture1);
		ui.camera1->setScene(&scene_);
		ui.camera1->show();
	} else if (index == 2) {
		int width = ui.camera2->width();
		int height = ui.camera2->height() - 2;
		QImage camera2Scaled = qnode.getCamera2()->scaled(width, height,
				Qt::KeepAspectRatio);
		cameraPicture2 = QPixmap::fromImage(camera2Scaled);
		scene2_.clear();
		scene2_.addPixmap(cameraPicture2);
		ui.camera2->setScene(&scene2_);
		ui.camera2->show();
	}
}

void MainWindow::on_startHoming_clicked(bool check) {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::COORD, currentPosition.x,
			currentPosition.y, 45.0);
	ros::Duration(2.0).sleep();
	qnode.sendTask(pap_common::CONTROLLER, pap_common::HOMING);
}

void MainWindow::on_switchCurrent_clicked(bool check) {
	ROS_INFO("Sending current switch command...");
	qnode.sendTask(pap_common::CONTROLLER, pap_common::CURRENT);
}

void MainWindow::on_gotoCoord_clicked(bool check) {
	//qnode.sendTask(pap_common::CONTROLLER, pap_common::COORD,
	//		(ui.xLineEdit->text()).toFloat(), (ui.yLineEdit->text()).toFloat(),
	//		(ui.zLineEdit->text()).toFloat());

	qnode.sendTask(pap_common::PLACER, pap_common::GOTO,
			(ui.xLineEdit->text()).toFloat(), (ui.yLineEdit->text()).toFloat(),
			(ui.zLineEdit->text()).toFloat());
}

void MainWindow::on_xManPos_pressed() {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
			(float) pap_common::XMOTOR, (float) pap_common::FORWARD, 0.0);
}

void MainWindow::releasexManPos() {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
			(float) pap_common::XMOTOR, 0.0, 0.0);
}

void MainWindow::on_xManNeg_pressed() {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
			(float) pap_common::XMOTOR, (float) pap_common::BACKWARD, 0.0);
}

void MainWindow::releasexManNeg() {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
			(float) pap_common::XMOTOR, 0.0, 0.0);
}

void MainWindow::on_YManPos_pressed() {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
			(float) pap_common::YMOTOR, (float) pap_common::FORWARD, 0.0);
}

void MainWindow::releaseyManPos() {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
			(float) pap_common::YMOTOR, 0.0, 0.0);
}

void MainWindow::on_YManNeg_pressed() {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
			(float) pap_common::YMOTOR, (float) pap_common::BACKWARD, 0.0);
}

void MainWindow::releaseyManNeg() {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
			(float) pap_common::YMOTOR, 0.0, 0.0);
}

void MainWindow::on_ZManPos_pressed() {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
			(float) pap_common::ZMOTOR, (float) pap_common::FORWARD, 0.0);
}

void MainWindow::releasezManPos() {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
			(float) pap_common::ZMOTOR, 0.0, 0.0);
}

void MainWindow::on_ZManNeg_pressed() {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
			(float) pap_common::ZMOTOR, (float) pap_common::BACKWARD, 0.0);
}

void MainWindow::releasezManNeg() {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
			(float) pap_common::ZMOTOR, 0.0, 0.0);
}

void MainWindow::placerStatusUpdated(int state, int status) {

	ROS_INFO("PlacerStatusUpdated: %d, %d !!", state, status);

	// Complete placement - send next component
	if (state == pap_common::PLACECOMPONENT_STATE
			&& status == pap_common::PLACER_FINISHED
			&& completePlacementRunning) {
		if (componentIndicator < componentList.size()) {
			componentIndicator++;
			updatePlacementData(componentList[componentIndicator]);
			qnode.sendTask(pap_common::PLACER, pap_common::COMPLETEPLACEMENT,
					placementData);
			ui.label_compLeft->setText(QString::number(componentList.size()-componentIndicator));
			ui.label_currentComp->setText(QString::fromStdString(componentList.at(componentIndicator).name));
		} else {
			// no more components - stop placer (Homing)
			qnode.sendTask(pap_common::PLACER, pap_common::HOMING);
			completePlacementRunning = false;
			ui.label_currentComp->setText("-");
			ui.label_compLeft->setText(QString::number(0));
		}
	}

	QPixmap statePixmap(QSize(20, 20));
	statePixmap.fill(Qt::transparent);
	QPainter p(&statePixmap);
	p.setRenderHint(QPainter::Antialiasing, true);
	QPen pen(Qt::black, 1);
	p.setPen(pen);
	QBrush brushWhite(Qt::white);
	QBrush brushGreen(Qt::green);
	QBrush brushYellow(Qt::yellow);
	QBrush brushRed(Qt::red);

	switch (status) {
	case pap_common::PLACER_IDLE:
		p.setBrush(brushWhite);
		break;
	case pap_common::PLACER_ACTIVE:
		p.setBrush(brushGreen);
		break;
	case pap_common::PLACER_FINISHED:
		p.setBrush(brushYellow);
		break;
	case pap_common::PLACER_ERROR:
		p.setBrush(brushRed);
	}

	p.drawEllipse(5, 5, 10, 10);

	switch (state) {
	case 1:
		ui.label_indicator1->setPixmap(statePixmap);
		break;
	case 2:
		break;
	case 3:
		ui.label_indicator3->setPixmap(statePixmap);
		break;
	case 4:
		ui.label_indicator4->setPixmap(statePixmap);
		break;
	case 5:
		ui.label_indicator5->setPixmap(statePixmap);
		break;
	case 6:
		ui.label_indicator6->setPixmap(statePixmap);
		break;
	case 7:
		ui.label_indicator7->setPixmap(statePixmap);
		break;
/*
	case pap_common::INFO:
		switch (status) {
		case 1:
			ui.label_Info->setText("IDLE");
			break;
		case 2:
			ui.label_Info->setText("CALIBRATE");
			break;
		case 3:
			ui.label_Info->setText("GOTOPCBORIGIN");
			break;
		case 4:
			ui.label_Info->setText("FINDPADS");
			break;
		case 5:
			ui.label_Info->setText("GOTOBOX");
			break;
		case 6:
			ui.label_Info->setText("FINDCOMPONENT");
			break;
		case 7:
			ui.label_Info->setText("GOTOPICKUPCOOR");
			break;
		case 8:
			ui.label_Info->setText("STARTPICKUP");
			break;
		case 9:
			ui.label_Info->setText("GOTOBOTTOMCAM");
			break;
		case 10:
			ui.label_Info->setText("CHECKCOMPONENTPICKUP");
			break;
		case 11:
			ui.label_Info->setText("GOTOPCBCOMP");
			break;
		case 12:
			ui.label_Info->setText("CHECKCOMPPOSITON");
			break;
		case 13:
			ui.label_Info->setText("GOTOPLACECOORD");
			break;
		case 14:
			ui.label_Info->setText("CHECKCOMPONENTPOSITION");
			break;
		case 15:
			ui.label_Info->setText("STARTPLACEMENT");
			break;
			break;
		}*/
	}
}

void MainWindow::statusUpdated(int index) {
	switch (index) {
	case 1:
		if ((qnode.getStatus())[index].energized) {
			ui.powerLabel1->setText("on");
		} else {
			ui.powerLabel1->setText("off");
		}

		if ((qnode.getStatus())[index].error) {
			ui.errorLabel1->setText("ERROR");
		} else {
			ui.errorLabel1->setText("OK");
		}

		if ((qnode.getStatus())[index].positionReached) {
			ui.posLabel1->setText("reached");
		} else {
			ui.posLabel1->setText("busy");
		}
		if (qnode.getStatus()[index].pos != 0.0) {
			ui.label_posX->setText(
					QString::number((qnode.getStatus())[index].pos));
			currentPosition.x = (qnode.getStatus())[index].pos;
		}
		break;

	case 2:
		if ((qnode.getStatus())[index].energized) {
			ui.powerLabel2->setText("on");
		} else {
			ui.powerLabel2->setText("off");
		}

		if ((qnode.getStatus())[index].error) {
			ui.errorLabel2->setText("ERROR");
		} else {
			ui.errorLabel2->setText("OK");
		}

		if ((qnode.getStatus())[index].positionReached) {
			ui.posLabel2->setText("reached");
		} else {
			ui.posLabel2->setText("busy");
		}

		if (qnode.getStatus()[index].pos != 0.0) {
			ui.label_posY->setText(
					QString::number((qnode.getStatus())[index].pos));
			currentPosition.y = (qnode.getStatus())[index].pos;
		}
		break;

	case 3:
		if ((qnode.getStatus())[index].energized) {
			ui.powerLabel3->setText("on");
		} else {
			ui.powerLabel3->setText("off");
		}

		if ((qnode.getStatus())[index].error) {
			ui.errorLabel3->setText("ERROR");
		} else {
			ui.errorLabel3->setText("OK");
		}

		if ((qnode.getStatus())[index].positionReached) {
			ui.posLabel3->setText("reached");
		} else {
			ui.posLabel3->setText("busy");
		}
		if (qnode.getStatus()[index].pos != 0.0) {
			ui.label_posZ->setText(
					QString::number((qnode.getStatus())[index].pos));
			currentPosition.z = (qnode.getStatus())[index].pos;
		}
		break;
	}
	// Send transforms to rviz
	sendTransforms(currentPosition.x / 1000.0, currentPosition.y / 1000.0,
			currentPosition.z / 1000.0, tip1Pos_ / 1000.0, tip2Pos_ / 1000.0);
}

// Vacuum valve 1
void MainWindow::on_connectButton_clicked(bool check) {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::CONNECT);
}

void MainWindow::on_valveToggle1_clicked(bool check) {
	if (!valve1Active_) {
		qnode.sendRelaisTask(5, true);
		ui.valveToggle1->setText("On");
		valve1Active_ = true;
	} else {
		qnode.sendRelaisTask(5, false);
		ui.valveToggle1->setText("Off");
		valve1Active_ = false;
	}
}

void MainWindow::on_valveToggle2_clicked(bool check) {
	if (!valve2Active_) {
		qnode.sendRelaisTask(4, true);
		ui.valveToggle2->setText("On");
		valve2Active_ = true;
	} else {
		qnode.sendRelaisTask(4, false);
		ui.valveToggle2->setText("Off");
		valve2Active_ = false;
	}
}

void MainWindow::on_valveToggle3_clicked(bool check) {
	if (!valve3Active_) {
		qnode.sendRelaisTask(8, true);
		ui.valveToggle3->setText("On");
		valve3Active_ = true;
	} else {
		qnode.sendRelaisTask(8, false);
		ui.valveToggle3->setText("Off");
		valve3Active_ = false;
	}
}
void MainWindow::on_valveToggle4_clicked(bool check) {
	if (!valve4Active_) {
		qnode.sendRelaisTask(3, false);
		qnode.sendRelaisTask(6, true);
		ui.valveToggle4->setText("On");
		tip2Pos_ = -20;
		valve4Active_ = true;
	} else {
		qnode.sendRelaisTask(6, false);
		qnode.sendRelaisTask(3, true);
		ui.valveToggle4->setText("Off");
		tip2Pos_ = 0.0;
		valve4Active_ = false;
	}
}
void MainWindow::on_valveToggle5_clicked(bool check) {
	if (!valve5Active_) {
		qnode.sendRelaisTask(7, true);
		ui.valveToggle5->setText("On");
		tip1Pos_ = -20;
		valve5Active_ = true;
	} else {
		qnode.sendRelaisTask(7, false);
		ui.valveToggle5->setText("Off");
		tip1Pos_ = 0.0;
		valve5Active_ = false;
	}
}
void MainWindow::on_valveToggle6_clicked(bool check) {
	if (!valve6Active_) {
		qnode.sendRelaisTask(1, false);
		qnode.sendRelaisTask(2, true);
		ui.valveToggle6->setText("On");
		valve6Active_ = true;
	} else {
		qnode.sendRelaisTask(2, false);
		qnode.sendRelaisTask(1, true);
		ui.valveToggle6->setText("Off");
		valve6Active_ = false;
	}
}
void MainWindow::on_valveToggle7_clicked(bool check) {
	if (!valve7Active_) {
		qnode.sendRelaisTask(9, true);
		ui.valveToggle7->setText("On");
		valve7Active_ = true;
	} else {
		qnode.sendRelaisTask(9, false);
		ui.valveToggle7->setText("Off");
		valve7Active_ = false;
	}
}
void MainWindow::on_valveToggle8_clicked(bool check) {
	if (!valve8Active_) {
		qnode.sendRelaisTask(10, true);
		ui.valveToggle8->setText("On");
		valve8Active_ = true;
	} else {
		qnode.sendRelaisTask(10, false);
		ui.valveToggle8->setText("Off");
		valve8Active_ = false;
	}
}
void MainWindow::on_turnLeftTipButton_clicked() {
	// Get angle from line edit
	bool ok;
	int requestedAngle = ui.rotationAngleLeft->text().toInt(&ok, 10);
	//float requestedAngle = ui.rotationAngleRight->text().toFloat(&ok);
	if (ok == true) {
		qnode.sendStepperTask(1, requestedAngle);
	} else {
		QMessageBox msgBox;
		const QString title = "Conversion failed!";
		msgBox.setWindowTitle(title);
		msgBox.setText("Only fixed-point angles accepted");
		msgBox.exec();
		msgBox.close();
	}
}

void MainWindow::on_turnRightTipButton_clicked() {
	// Get angle from line edit
	bool ok;
	int requestedAngle = ui.rotationAngleRight->text().toInt(&ok, 10);
	//float requestedAngle = ui.rotationAngleRight->text().toFloat(&ok);
	if (ok == true) {
		qnode.sendStepperTask(2, requestedAngle);
	} else {
		QMessageBox msgBox;
		const QString title = "Conversion failed!";
		msgBox.setWindowTitle(title);
		msgBox.setText("Only fixed-point angles accepted");
		msgBox.exec();
		msgBox.close();
	}
}

void MainWindow::on_setLEDButton_clicked() {
	// Get LED number from line edit
	bool ok;
	int requestedLED = ui.LEDnumber->text().toInt(&ok, 10);

	if (ok == true) {

		if (requestedLED <= 59) {
			qnode.setLEDTask(requestedLED);
		} else {
			QMessageBox msgBox;
			const QString title = "Wrong LED number";
			msgBox.setWindowTitle(title);
			msgBox.setText("Led numbers are in the range of 0 to 58.");
			msgBox.exec();
			msgBox.close();
		}
	} else {
		QMessageBox msgBox;
		const QString title = "Conversion failed!";
		msgBox.setWindowTitle(title);
		msgBox.setText("Only numbers accepted.");
		msgBox.exec();
		msgBox.close();
	}
}

void MainWindow::on_resetLEDButton_clicked() {
	// Get LED number from line edit
	bool ok;
	int requestedLED = ui.LEDnumber->text().toInt(&ok, 10);

	if (ok == true) {

		if (requestedLED <= 59) {
			qnode.resetLEDTask(requestedLED);
		} else {
			QMessageBox msgBox;
			const QString title = "Wrong LED number";
			msgBox.setWindowTitle(title);
			msgBox.setText("Led numbers are in the range of 0 to 58.");
			msgBox.exec();
			msgBox.close();
		}
	} else {
		QMessageBox msgBox;
		const QString title = "Conversion failed!";
		msgBox.setWindowTitle(title);
		msgBox.setText("Only numbers accepted.");
		msgBox.exec();
		msgBox.close();
	}

}

void MainWindow::on_ledResetButton_clicked() {
	qnode.LEDTask(pap_common::RESETALLLED, 0);
}

void MainWindow::on_blinkBackButton_clicked() {
	static unsigned int backBlinking = 0;
	if (backBlinking) {
		qnode.LEDTask(pap_common::BACKLIGHTBLINK, 0);
		backBlinking = 0;
	} else {
		qnode.LEDTask(pap_common::BACKLIGHTBLINK, 1);
		backBlinking = 1;
	}
}

void MainWindow::on_blinkRingButton_clicked() {
	static unsigned int ringBlinking = 0;
	if (ringBlinking) {
		qnode.LEDTask(pap_common::RINGBLINK, 0);
		ringBlinking = 0;
	} else {
		qnode.LEDTask(pap_common::RINGBLINK, 1);
		ringBlinking = 1;
	}
}

void MainWindow::changeRingLEDBrightness(int brightness) {
	qnode.LEDTask(pap_common::SETBRIGHTNESSRING, brightness);
}

void MainWindow::changeBackLEDBrightness(int brightness) {
	qnode.LEDTask(pap_common::SETBRIGHTNESSBACK, brightness);
}

void MainWindow::changeTopLEDBrightness(int brightness) {
	qnode.LEDTask(pap_common::SETBRIGHTNESSTOP, brightness);
}

void MainWindow::changeRingColor(int comboValue) {
	switch (comboValue) {
	// Green
	case 0:
		qnode.LEDTask(pap_common::SETRINGCOLOR, 0);
		break;
		// Red
	case 1:
		qnode.LEDTask(pap_common::SETRINGCOLOR, 96);
		break;

		//Blue
	case 2:
		qnode.LEDTask(pap_common::SETRINGCOLOR, 171);
		break;

		// White
	case 3:
		qnode.LEDTask(pap_common::SETRINGCOLOR, 128);
		break;
	}
}

void MainWindow::on_startChipFinder_Button_clicked() {
	unsigned int cameraSelect = 3;

	if (ui.cameraSelect_Chip->currentText() == "Top") {
		cameraSelect = 0;
	} else if (ui.cameraSelect_Chip->currentText() == "Bottom") {
		cameraSelect = 1;
	} else {
		return;
	}

	qnode.sendTask(pap_common::VISION, pap_vision::START_CHIP_FINDER,
			ui.widthChipFinder_LineEdit->text().toFloat(),
			ui.heightChipFinder_LineEdit->text().toFloat(),
			(float) cameraSelect);
	displaySMDCoords(0.0, 0.0, 0.0, 0);
	displaySMDCoords(0.0, 0.0, 0.0, 1);
}

void MainWindow::on_startSmallSMDFinder_Button_clicked() {
	if (ui.SmallSMD_Combo->currentText() == "0805") {
		qnode.sendTask(pap_common::VISION, pap_vision::START_SMALL_FINDER, 1.25,
				2.0, 0.0);
	} else if (ui.SmallSMD_Combo->currentText() == "0603") {
		qnode.sendTask(pap_common::VISION, pap_vision::START_SMALL_FINDER, 0.8,
				1.6, 0.0);
	} else if (ui.SmallSMD_Combo->currentText() == "0402") {
		qnode.sendTask(pap_common::VISION, pap_vision::START_SMALL_FINDER, 0.5,
				1.0, 0.0);
	}
	displaySMDCoords(0.0, 0.0, 0.0, 0);
	displaySMDCoords(0.0, 0.0, 0.0, 1);
}

void MainWindow::on_startTapeFinder_Button_clicked() {
	if (ui.tapeFinder_Combo->currentText() == "0805") {
		qnode.sendTask(pap_common::VISION, pap_vision::START_TAPE_FINDER, 1.25,
				2.0, 0.0);
	} else if (ui.tapeFinder_Combo->currentText() == "0603") {
		qnode.sendTask(pap_common::VISION, pap_vision::START_TAPE_FINDER, 0.8,
				1.6, 0.0);
	} else if (ui.tapeFinder_Combo->currentText() == "0402") {
		qnode.sendTask(pap_common::VISION, pap_vision::START_TAPE_FINDER, 0.5,
				1.0, 0.0);
	}
	displaySMDCoords(0.0, 0.0, 0.0, 0);
	displaySMDCoords(0.0, 0.0, 0.0, 1);
}

void MainWindow::on_startPadFinder_Button_clicked() {
	displaySMDCoords(0.0, 0.0, 0.0, 0);
	displaySMDCoords(0.0, 0.0, 0.0, 1);
	qnode.sendTask(pap_common::VISION, pap_vision::START_PAD_FINDER);
}

void MainWindow::on_StartStopVision_Button_clicked() {

	if (!visionStarted_) {
		qnode.sendTask(pap_common::VISION, pap_vision::START_VISION);
		ui.visionStatus_Label->setText("On");
		visionStarted_ = true;
	} else {
		qnode.sendTask(pap_common::VISION, pap_vision::STOP_VISION);
		ui.visionStatus_Label->setText("Off");
		visionStarted_ = false;
	}
	displaySMDCoords(0.0, 0.0, 0.0, 0);
	displaySMDCoords(0.0, 0.0, 0.0, 1);
}

void MainWindow::displaySMDCoords(float x, float y, float rot,
		unsigned int cameraSelect) {
	if (cameraSelect == 0) {
		xTapeCalibration = x;
		yTapeCalibration = y;
		rotTapeCalibration = rot;
		ui.smdXTop_Label->setText(QString("X: ") + QString::number(x));
		ui.smdYTop_Label->setText(QString("Y: ") + QString::number(y));
		ui.smdRotTop_Label->setText(QString("Rot: ") + QString::number(rot));
	} else if (cameraSelect == 1) {
		ui.smdXBottom_Label->setText(QString("X: ") + QString::number(x));
		ui.smdYBottom_Label->setText(QString("Y: ") + QString::number(y));
		ui.smdRotBottom_Label->setText(QString("Rot: ") + QString::number(rot));
	}
}

void MainWindow::setCamera1Point(QPointF point) {
	float percentageX = (100.0 / (float) ui.camera1->width()) * point.x();
	float percentageY = (100.0 / (float) ui.camera1->height()) * point.y();
}

void MainWindow::on_scanQRButton_clicked() {
	qnode.sendTask(pap_common::VISION, pap_vision::START__QRCODE_FINDER, 0, 1,
			0);
	/*
	 QEventLoop loop;
	 QTimer *timer = new QTimer(this);

	 //connect(&qnode, SIGNAL(signalPosition(float,float)), &compOrientButton_2loop, SLOT(quit()));
	 connect(timer, SIGNAL(timeout()), &loop, SLOT(quit()));
	 timer->setSingleShot(true);
	 timer->start(1000);

	 loop.exec(); //blocks untill either signalPosition or timeout was fired

	 // Is timeout ocurred?
	 if (!timer->isActive()) {
	 //qnode.sendTask(pap_common::VISION, pap_vision::START_PAD_FINDER);
	 QMessageBox msgBox;
	 const QString title = "No QR Code detected";
	 msgBox.setWindowTitle(title);
	 msgBox.setText("No QR Code found within the last 5 seconds");
	 msgBox.exec();
	 msgBox.close();
	 return;
	 }
	 */
	//qnode.sendTask(pap_common::VISION, pap_vision::START_PAD_FINDER);
	// Update component info
}

void MainWindow::setFiducial(QPointF point) {
	float percentageX = (100.0 / (float) ui.camera1->width()) * point.x();
	float percentageY = (100.0 / (float) ui.camera1->height()) * point.y();

	float indexXFull = (640.0 / 100.0) * percentageX;
	float indexYFull = (480.0 / 100.0) * percentageY;

	qnode.sendTask(pap_common::VISION, pap_vision::START_PAD_FINDER, 1.0,
			indexXFull, indexYFull);

	QEventLoop loop;
	QTimer *timer = new QTimer(this);
	connect(&qnode, SIGNAL(signalPosition(float,float)), &loop, SLOT(quit()));
	connect(timer, SIGNAL(timeout()), &loop, SLOT(quit()));
	timer->setSingleShot(true);
	timer->start(1000);

	loop.exec(); //blocks untill either signalPosition or timeout was fired

	// Is timeout ocurred?
	if (!timer->isActive()) {
		qnode.sendTask(pap_common::VISION, pap_vision::START_PAD_FINDER);
		QMessageBox msgBox;
		const QString title = "Nothing received";
		msgBox.setWindowTitle(title);
		msgBox.setText("No pad found on given position");
		msgBox.exec();
		msgBox.close();
		return;
	}

	qnode.sendTask(pap_common::VISION, pap_vision::START_PAD_FINDER);
	ros::Duration(0.5).sleep();
	if (ui.fiducialTable->fiducialSize_ == 0) {
		setFiducialTable(0, padPosition_.x() + currentPosition.x,
				padPosition_.y() + currentPosition.y);
	} else if (ui.fiducialTable->fiducialSize_ == 1) {
		setFiducialTable(1, padPosition_.x() + currentPosition.x,
				padPosition_.y() + currentPosition.y);
	} else {
		ui.fiducialTable->clear();
		ui.fiducialTable->fiducialSize_ = 0;
		initFiducialTable();
		setFiducialTable(0, padPosition_.x() + currentPosition.x,
				padPosition_.y() + currentPosition.y);
	}

}

void MainWindow::initFiducialTable(void) {
	// Set size of table
	ui.fiducialTable->setRowCount(2);
	ui.fiducialTable->setColumnCount(4);

	// Set labels
	QStringList hLabels, vLabels;
	hLabels << "X-PCB" << "Y-PCB" << "X-Global" << "Y-Global";
	for (int i = 1; i <= 2; i++) {
		vLabels << QString::number(i);
	}
	ui.fiducialTable->setHorizontalHeaderLabels(hLabels);
	ui.fiducialTable->setVerticalHeaderLabels(vLabels);
}

void MainWindow::initPadTable(int rows) {
	// Set size of table
	ui.padTable->setRowCount(rows);
	ui.padTable->setColumnCount(6);

	// Set labels
	QStringList hLabels, vLabels;
	hLabels << "X" << "Y" << "Width" << "Height" << "Rotation" << "Type";
	for (int i = 1; i <= rows; i++) {
		vLabels << QString::number(i);
	}
	ui.padTable->setHorizontalHeaderLabels(hLabels);
	ui.padTable->setVerticalHeaderLabels(vLabels);
}

void MainWindow::setFiducialTable(int number, float xGlobal, float yGlobal) {
	ui.fiducialTable->setItem(number, 2,
			new QTableWidgetItem(QString::number(xGlobal)));
	ui.fiducialTable->setItem(number, 3,
			new QTableWidgetItem(QString::number(yGlobal)));
	ui.fiducialTable->fiducialSize_++;
}

// This function is fired from the PadView Class
void MainWindow::setFiducialPads(int number, float x, float y) {
	ui.fiducialTable->setItem(number, 0,
			new QTableWidgetItem(QString::number(x)));
	ui.fiducialTable->setItem(number, 1,
			new QTableWidgetItem(QString::number(y)));
}

void MainWindow::signalPosition(float x, float y) {
	padPosition_.setX(x);
	padPosition_.setY(y);
	//ROS_INFO("PadPos: %f %f", padPosition_.x(), padPosition_.y());
}

void MainWindow::tipToggled(int select, bool status) {
	if (select == 0) {
		if (status) {
			tip1Pos_ = -20.0;
		} else {
			tip1Pos_ = 0.0;
		}
	} else if (select == 1) {
		if (status) {
			tip2Pos_ = -20.0;
		} else {
			tip2Pos_ = 0.0;
		}

	}
}

void MainWindow::sendGotoFiducial(int indexOfFiducial) {
	if (qnode.getStatus()[0].positionReached
			&& qnode.getStatus()[1].positionReached
			&& qnode.getStatus()[2].positionReached) {
		if (indexOfFiducial == 0 && ui.fiducialTable->fiducialSize_ < 1) {
			return;
		} else if (indexOfFiducial == 1
				&& ui.fiducialTable->fiducialSize_ < 2) {
			return;
		}
		float x = ui.fiducialTable->item(indexOfFiducial, 2)->text().toFloat();
		float y = ui.fiducialTable->item(indexOfFiducial, 3)->text().toFloat();
		ROS_INFO("Goto position x: %f y: %f", x, y);
		//qnode.sendTask(pap_common::CONTROLLER, pap_common::COORD, x, y, 22.0);
		qnode.sendTask(pap_common::PLACER, pap_common::GOTO, x, y, 22.0);
	}
}

void MainWindow::on_inputPad_Button_clicked() {
	//get a filename to open
	if (alreadyFlipped_) {
		ui.padView_Image->scale(-1, -1);
		alreadyFlipped_ = false;
	}

	QString gerberFile = QFileDialog::getOpenFileName(this, tr("Open Whl file"),
			"/home", tr("Text Files (*.txt  *.Whl)"));

	const char *filenameWhl = gerberFile.toLatin1().data();

	padParser.parseShapes(filenameWhl);

	//get a filename to open
	gerberFile = QFileDialog::getOpenFileName(this, tr("Open PasteBot file"),
			"/home", tr("Text Files (*.txt  *.PasteBot *.PasteTop)"));

	bottomLayer_ = false;
	if (gerberFile.contains(".PasteBot")) {
		bottomLayer_ = true;
		ui.padView_Image->scale(-1, -1);
		alreadyFlipped_ = true;
	} else {
		bottomLayer_ = false;
	}
	const char *filenamePaste = gerberFile.toLatin1().data();

	// First load shape data from .Whl file
	padParser.loadFile(filenamePaste, bottomLayer_);
	// Then load cad file .PasteBot
	padParser.setTable(ui.padTable);
	padFileLoaded_ = true;
}

void MainWindow::on_padViewSetSize_button_clicked() {
	float width = ui.padViewWidth_text->text().toFloat();
	float height = ui.padViewHeight_text->text().toFloat();
	padParser.setSize(height, width);
	qnode.pcbHeight_ = height;
	qnode.pcbWidth_ = width;
	sizeDefined_ = true;

	QMessageBox msgBox;
	const QString title = "Size Definition";
	msgBox.setWindowTitle(title);
	msgBox.setText("Size saved...");
	msgBox.exec();
	msgBox.close();
	return;
}

void MainWindow::on_padViewGenerate_button_clicked() {

	if (!sizeDefined_ | !padFileLoaded_) {
		QMessageBox msgBox;
		const QString title = "Not initialized";
		msgBox.setWindowTitle(title);
		msgBox.setText(
				"Make sure that size is set and the pad file is loaded!");
		msgBox.exec();
		msgBox.close();
		return;
	}

	// Build image with QGraphicsItem
	QRectF pcbSize = padParser.renderImage(&scenePads_,
			ui.padView_Image->width() - 20, ui.padView_Image->height() - 20);

	ui.padView_Image->setScene(&scenePads_);
	ui.padView_Image->show();
	/*
	 QImage *image = new QImage(pcbSize.width(), pcbSize.height(),
	 QImage::Format_RGB888);
	 QPainter painter(image);
	 painter.setRenderHint(QPainter::Antialiasing);
	 scenePads_.render(&painter);
	 if (!image->save("/home/johan/Schreibtisch/file_name.png")) {
	 ROS_ERROR("Error while saving image");
	 }
	 */
	qnode.sendPcbImage(padParser.getMarkerList());

}

void MainWindow::padPressed(int numberOfFiducial, QPointF padPos) {
	id_ = padParser.searchId(padPos, ui.padView_Image->width() - 20);
	setFiducialPads(numberOfFiducial,
			padParser.padInformationArray_[id_].rect.x(),
			padParser.padInformationArray_[id_].rect.y());
}

void MainWindow::gotoPad(QPointF padPos) {
	ROS_INFO("Goto Pad....");
	id_ = padParser.searchId(padPos, ui.padView_Image->width() - 20);
	ROS_INFO("ID: %d", id_);
	//if (qnode.getStatus()[0].positionReached
	//		&& qnode.getStatus()[1].positionReached
	//	&& qnode.getStatus()[2].positionReached) {

	float x = padParser.padInformationArray_[id_].rect.x();
	float y = padParser.padInformationArray_[id_].rect.y();
	ROS_INFO("Goto position x: %f y: %f", x, y);
	//qnode.sendTask(pap_common::CONTROLLER, pap_common::COORD, x, y, 22);
	qnode.sendTask(pap_common::PLACER, pap_common::GOTO, x, y, 22.0);
	//}
}

void MainWindow::deletePad(QPointF padPos) {
	id_ = padParser.searchId(padPos, ui.padView_Image->width() - 20);
	padParser.deleteEntry(id_);
	on_padViewGenerate_button_clicked();
}

void MainWindow::on_calibrationButton_offsets_clicked() {
	ui.tab_manager->setCurrentIndex(1);
	qnode.sendTask(pap_common::PLACER, pap_common::CALIBRATION_OFFSET);
}

void MainWindow::on_calibrationButton_ratios_clicked() {
	ui.tab_manager->setCurrentIndex(1);
	qnode.sendTask(pap_common::PLACER, pap_common::CALIBRATION_RATIO);
}

void MainWindow::on_calcOrientation_Button_clicked() {
	QPointF local1, global1, local2, global2;

	float xCamera = 0.0;
	float yCamera = 0.0;
	if (qnode.fakePadPos_) {
		ROS_INFO("Simulation active: I will fake the pad positions...");
		xCamera = 180.0;
		yCamera = 140.0;

		local1.setX(xCamera);
		local1.setY(yCamera);
		local2.setX(xCamera);
		local2.setY(yCamera);

		global1.setX(0);
		global1.setY(0);
		global2.setX(0);
		global2.setY(0);
	} else {

		local1.setX(ui.fiducialTable->item(0, 2)->text().toFloat() + xCamera);
		local1.setY(ui.fiducialTable->item(0, 3)->text().toFloat() + yCamera);
		local2.setX(ui.fiducialTable->item(1, 2)->text().toFloat() + xCamera);
		local2.setY(ui.fiducialTable->item(1, 3)->text().toFloat() + yCamera);

		global1.setX(ui.fiducialTable->item(0, 0)->text().toFloat());
		global1.setY(ui.fiducialTable->item(0, 1)->text().toFloat());
		global2.setX(ui.fiducialTable->item(1, 0)->text().toFloat());
		global2.setY(ui.fiducialTable->item(1, 1)->text().toFloat());
	}
	padParser.calibratePads(local1, local2, global1, global2,
			qnode.fakePadPos_);
	padParser.rotatePads();

	QMessageBox msgBox;
	const QString title = "Calibration";
	msgBox.setWindowTitle(title);
	msgBox.setText("Calibration finished");
	msgBox.exec();
	msgBox.close();
}

void MainWindow::keyPressEvent(QKeyEvent *e) {

	if (e->isAutoRepeat()) {
		return;
	}

	switch (e->key()) {
	case Qt::Key_S:
		qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
				(float) pap_common::XMOTOR, (float) pap_common::FORWARD, 0.0);
		//ROS_INFO("Pressed down xmotor");
		break;
	case Qt::Key_W:
		qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
				(float) pap_common::XMOTOR, (float) pap_common::BACKWARD, 0.0);
		//ROS_INFO("Pressed up xmotor");
		break;
	case Qt::Key_A:
		qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
				(float) pap_common::YMOTOR, (float) pap_common::FORWARD, 0.0);
		break;
	case Qt::Key_D:
		qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
				(float) pap_common::YMOTOR, (float) pap_common::BACKWARD, 0.0);
		break;
	case Qt::Key_PageDown:
		qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
				(float) pap_common::ZMOTOR, (float) pap_common::BACKWARD, 0.0);
		break;
	case Qt::Key_PageUp:
		qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
				(float) pap_common::ZMOTOR, (float) pap_common::FORWARD, 0.0);
		break;
	}
}

void MainWindow::keyReleaseEvent(QKeyEvent *e) {

	if (e->isAutoRepeat()) {
		return;
	}
	switch (e->key()) {
	case Qt::Key_S:
		qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
				(float) pap_common::XMOTOR, 0.0, 0.0);
		qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
				(float) pap_common::XMOTOR, (float) pap_common::BACKWARD, 0.0);
		qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
				(float) pap_common::XMOTOR, 0.0, 0.0);
		break;
	case Qt::Key_W:
		qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
				(float) pap_common::XMOTOR, 0.0, 0.0);
		qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
				(float) pap_common::XMOTOR, (float) pap_common::FORWARD, 0.0);
		qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
				(float) pap_common::XMOTOR, 0.0, 0.0);
		break;
	case Qt::Key_A:
		qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
				(float) pap_common::YMOTOR, 0.0, 0.0);
		qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
				(float) pap_common::YMOTOR, (float) pap_common::BACKWARD, 0.0);
		qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
				(float) pap_common::YMOTOR, 0.0, 0.0);
		break;
	case Qt::Key_D:
		qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
				(float) pap_common::YMOTOR, 0.0, 0.0);
		qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
				(float) pap_common::YMOTOR, (float) pap_common::FORWARD, 0.0);
		qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
				(float) pap_common::YMOTOR, 0.0, 0.0);
		break;
	case Qt::Key_PageDown:
		qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
				(float) pap_common::ZMOTOR, 0.0, 0.0);
		break;
	case Qt::Key_PageUp:
		qnode.sendTask(pap_common::CONTROLLER, pap_common::STOP,
				(float) pap_common::ZMOTOR, 0.0, 0.0);
		break;
	}
}

void MainWindow::on_startTipFinder_Button_clicked() {
	qnode.sendTask(pap_common::VISION, pap_vision::SEARCH_CIRCLE,
			ui.radius_edit->text().toFloat(), 0.0, 0.0);
	displaySMDCoords(0.0, 0.0, 0.0, 0);
	displaySMDCoords(0.0, 0.0, 0.0, 1);
}

void MainWindow::on_bottomLEDButton_clicked() {
	if (!bottomLEDon) {
		qnode.setBottomLEDTask();
		bottomLEDon = true;
	} else {
		qnode.resetBottomLEDTask();
		bottomLEDon = false;
	}
}

void MainWindow::on_topLedButton_clicked() {
	static bool topLEDon = false;
	if (!topLEDon) {
		qnode.LEDTask(pap_common::SETTOPLED, 0);
		topLEDon = true;
	} else {
		qnode.LEDTask(pap_common::RESETTOPLED, 0);
		topLEDon = false;
	}
}

struct compareClass {
	//compareClass(float paramA, float paramB) { this->xPos = paramA; this->yPos = paramB; }
	bool operator()(PadInformation i, PadInformation j) {
		//float arg1 = sqrt(pow(i.rect.x()-yPos,2) + pow(i.rect.y()-xPos,2));
		//float arg2 = sqrt(pow(j.rect.x()-yPos,2) + pow(j.rect.y()-xPos,2));
		float arg1 = i.rect.x();
		float arg2 = j.rect.x();
		return (arg1 < arg2);
	}
	//float xPos,yPos;
};

void MainWindow::on_stopDispense_button_clicked() {
	dispenserPaused = true;
	QMessageBox msgBox;
	const QString title = "Dispensing information";
	msgBox.setWindowTitle(title);
	msgBox.setText("Dispensing paused");
	msgBox.exec();
	msgBox.close();
}

void MainWindow::on_resetDispense_button_clicked() {
	dispenserPaused = false;
	lastDispenserId = 0;
	on_padViewGenerate_button_clicked();
	QMessageBox msgBox;
	const QString title = "Dispensing information";
	msgBox.setWindowTitle(title);
	msgBox.setText("Dispensing reseted");
	msgBox.exec();
	msgBox.close();
}

void MainWindow::on_startDispense_button_clicked() {
	float pxFactor = padParser.pixelConversionFactor;
	float nozzleDiameter = ui.nozzleDispCombo->currentText().toFloat();

	size_t initialIter = 0;
	if (dispenserPaused) {
		dispenserPaused = false;
		initialIter = lastDispenserId;
	}
	std::vector<PadInformation> copy;
	copy = padParser.padInformationArrayPrint_;

	// Number 0 is background
	copy.erase(copy.begin());

	std::sort(copy.begin(), copy.end(), compareClass());

	for (size_t i = initialIter; i < copy.size(); i++) {

		if (dispenserPaused) {
			lastDispenserId = i;
			qnode.sendTask(pap_common::PLACER, pap_common::GOTO,
					currentPosition.x, currentPosition.y, 45.0);
			return;
		}
		std::vector<dispenseInfo> dispInfo = dispenserPlanner.planDispensing(
				copy[i], nozzleDiameter);

		//copy.erase(copy.begin());

		//std::sort(copy.begin(), copy.end(), compareClass(currentPosition.x,currentPosition.y));

		for (size_t j = 0; j < dispInfo.size(); j++) {
			scenePads_.addLine(
					QLineF(dispInfo[j].yPos * pxFactor,
							padParser.heightPixel_
									- (dispInfo[j].xPos * pxFactor),
							dispInfo[j].yPos2 * pxFactor,
							padParser.heightPixel_
									- (dispInfo[j].xPos2 * pxFactor)),
					QPen(Qt::blue, nozzleDiameter * pxFactor, Qt::SolidLine));

			padParser.transformDispenserInfo(&dispInfo[j]);
			qnode.sendDispenserTask(dispInfo[j]);

			QEventLoop loop;
			QTimer *timer = new QTimer(this);

			connect(&qnode, SIGNAL(dispenserFinished()), &loop, SLOT(quit()));
			connect(timer, SIGNAL(timeout()), &loop, SLOT(quit()));
			timer->setSingleShot(true);
			timer->start(10000);

			loop.exec(); //blocks untill either signalPosition or timeout was fired

			// Is timeout ocurred?
			if (!timer->isActive()) {
				return;
			}
		}
		scenePads_.addEllipse((copy[i].rect.y()) * pxFactor - 1.0,
				padParser.heightPixel_ - (copy[i].rect.x() * pxFactor), 1, 1,
				QPen(Qt::green, 2, Qt::SolidLine));

	}
	ROS_INFO("Dispensing finished....");
	qnode.sendTask(pap_common::PLACER, pap_common::GOTO, currentPosition.x,
			currentPosition.y, 45.0);
}

void MainWindow::dispenseSinglePad(QPointF point) {
	id_ = padParser.searchId(point, ui.padView_Image->width() - 20);
	float nozzleDiameter = ui.nozzleDispCombo->currentText().toFloat();
	float pxFactor = padParser.pixelConversionFactor;
	if (id_ != -1) {
		std::vector<dispenseInfo> dispInfo = dispenserPlanner.planDispensing(
				padParser.padInformationArrayPrint_[id_], nozzleDiameter);

		for (size_t j = 0; j < dispInfo.size(); j++) {
			scenePads_.addLine(
					QLineF(dispInfo[j].yPos * pxFactor,
							padParser.heightPixel_
									- (dispInfo[j].xPos * pxFactor),
							dispInfo[j].yPos2 * pxFactor,
							padParser.heightPixel_
									- (dispInfo[j].xPos2 * pxFactor)),
					QPen(Qt::blue, nozzleDiameter * pxFactor, Qt::SolidLine));

			padParser.transformDispenserInfo(&dispInfo[j]);
			qnode.sendDispenserTask(dispInfo[j]);

			QEventLoop loop;
			QTimer *timer = new QTimer(this);

			connect(&qnode, SIGNAL(dispenserFinished()), &loop, SLOT(quit()));
			connect(timer, SIGNAL(timeout()), &loop, SLOT(quit()));
			timer->setSingleShot(true);
			timer->start(10000);

			loop.exec(); //blocks untill either signalPosition or timeout was fired

			// Is timeout ocurred?
			if (!timer->isActive()) {
				return;
			}
			//ROS_INFO("Print: X %f Y %f X2 %f Y2 %f",dispInfo[j].xPos *pxFactor ,(padParser.height_-dispInfo[j].yPos)*pxFactor,dispInfo[j].xPos2*pxFactor,(padParser.height_-dispInfo[j].yPos2)*pxFactor);
		}
	} else {
		ROS_ERROR("No pad selected...");
	}
}

void MainWindow::sendTransforms(double x, double y, double z, double nozzle_1,
		double nozzle_2) {
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Transform transformReference;
	tf::Transform transformX;
	tf::Transform transformY, transformZ, transformS1, transformS2;

	// Reference frame
	transformReference.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	tf::Quaternion qRef;
	qRef.setRPY(0, 0, 0);
	transformReference.setRotation(qRef);

	// Base_link frame
	transform.setOrigin(tf::Vector3(0, 0, 0));
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);

	transformX.setOrigin(tf::Vector3(0.023026, 0.11628 + x, 0.11244));
	tf::Quaternion qX;
	qX.setX(-0.699941);
	qX.setY(0.000131839);
	qX.setZ(0.000619792);
	qX.setW(0.7142);
	transformX.setRotation(qX);

	// Y-Link
	transformY.setOrigin(tf::Vector3(-0.11097 + y, 0.14813 + x, 0.16924));
	tf::Quaternion qY;
	qY.setX(-0.504923);
	qY.setY(0.494495);
	qY.setZ(0.505109);
	qY.setW(-0.495371);
	transformY.setRotation(qY);

	// Z-Link
	transformZ.setOrigin(tf::Vector3(-0.032259 + y, 0.16839 + x, 0.15228 + z));
	tf::Quaternion qZ;
	qZ.setX(0.000131839);
	qZ.setY(0.699941);
	qZ.setZ(0.7142);
	qZ.setW(-0.000619792);
	transformZ.setRotation(qZ);

	// Stepper1-Link
	//transformS1.setOrigin(tf::Vector3(-0.044085 + y, 0.2214 + x, 0.10935+z));
	transformS1.setOrigin(
			tf::Vector3(-0.044085 + y, 0.2214 + x, 0.10935 + z + nozzle_1));
	tf::Quaternion qS1;
	qS1.setX(0.00737794);
	qS1.setY(-0.706695);
	qS1.setZ(-0.00688299);
	qS1.setW(0.707446);
	transformS1.setRotation(qS1);

	// Stepper2-Link
	//transformS2.setOrigin(tf::Vector3(-0.11908 + y, 0.22134 + x, 0.10935 +z));
	transformS2.setOrigin(
			tf::Vector3(-0.11908 + y, 0.22134 + x, 0.10935 + z + nozzle_2));// Change Niko
	tf::Quaternion qS2;
	qS2.setX(0.00737794);
	qS2.setY(-0.706695);
	qS2.setZ(-0.00688299);
	qS2.setW(0.707446);
	transformS2.setRotation(qS2);

	br.sendTransform(
			tf::StampedTransform(transformReference, ros::Time::now(),
					"/base_link", "/world"));
	//br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/world","/base_link"));

	// Axes
	br.sendTransform(
			tf::StampedTransform(transformX, ros::Time::now(), "/base_link",
					"/x-axis"));
	br.sendTransform(
			tf::StampedTransform(transformY, ros::Time::now(), "/base_link",
					"/y-axis"));

	br.sendTransform(
			tf::StampedTransform(transformZ, ros::Time::now(), "/base_link",
					"/z-axis"));

	br.sendTransform(
			tf::StampedTransform(transformS1, ros::Time::now(), "/base_link",
					"/nozzle_1"));

	br.sendTransform(
			tf::StampedTransform(transformS2, ros::Time::now(), "/base_link",
					"/nozzle_2"));
}

void MainWindow::on_calibrateTapeButton_clicked(void) {

	// TODO{Niko}: Use these functions in gui and placer node
	QVector<int> calibratedTapes;
	for (size_t i = 0; i < componentList.size(); i++) {
		if ((componentList.at(i).box >= 67)
				&& (componentList.at(i).box <= 86)) {
			ROS_INFO("Calibrated tape: %d", componentList.at(i).box);
			int tape_nr = componentList.at(i).box - 67;
			if (calibratedTapes.indexOf(tape_nr) == -1) {
				calibratedTapes.append(tape_nr);
				calibrateTape(tape_nr, componentList.at(i).width,
						componentList.at(i).length);
				ROS_INFO("Index : %d Width: %f Height: %f", tape_nr,
						componentList.at(i).width, componentList.at(i).length);
			}
		}
	}

	// EXAMPLE: Get 4th position of component in 1st tape
	//tapeCalibrationValue positionOfComponent = calculatePosOfTapePart(1,4);
}

tapeCalibrationValue MainWindow::calculatePosOfTapePart(int numOfTape,
		int numOfPart) {
	tf::Transform rotation_;
	tapeCalibrationValue out;

	int indexInVector = -1;
	for (size_t i = 0; i < tapeCalibrationValues.size(); i++) {
		if (tapeCalibrationValues[i].index == numOfTape) {
			indexInVector = i;
		}
	}

	if (indexInVector == -1) {
		ROS_ERROR("Tape calibration values not found!");
		return out;
	}

	tf::Point pointToTransform;
	// This point should be transformed
	// Distance between comp. on tape is 2 mm (0402 comp.)
	pointToTransform.setX(0.0);
	pointToTransform.setY(numOfPart * 2.0);
	pointToTransform.setZ(0.0);

	// This rotates the component to the tape orientation
	tf::Quaternion rotQuat;
	rotQuat.setEuler(0.0, 0.0,
			tapeCalibrationValues[indexInVector].rot * (M_PI / 180.0));
	rotation_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	rotation_.setRotation(rotQuat);

	pointToTransform = rotation_ * pointToTransform;

	out.x = pointToTransform.x() + tapeCalibrationValues[indexInVector].x;
	out.y = pointToTransform.y() + tapeCalibrationValues[indexInVector].y;
	out.rot = tapeCalibrationValues[indexInVector].rot;

	ROS_INFO("Calculated Pos of part in Tape: x %f y %f rot %f", out.x, out.y,
			out.rot);
	return out;
}

void MainWindow::calibrateTape(int tapeNumber, float componentWidth,
		float componentHeight) {
	// TODO{Johan}: Goto tape with index: tapeNumber

	Offset temp = TapeOffsetTable[tapeNumber];
	temp.x += 108.42;
	temp.y += 261;
	temp.z = 20.1;
	qnode.sendTask(pap_common::PLACER, pap_common::GOTO, temp.x, temp.y,
			temp.z);

	QEventLoop loopPos;
	QTimer *timerPos = new QTimer(this);
	connect(&qnode, SIGNAL(positionGotoReached()), &loopPos, SLOT(quit()));
	connect(timerPos, SIGNAL(timeout()), &loopPos, SLOT(quit()));
	timerPos->setSingleShot(true);
	timerPos->start(20000);
	loopPos.exec(); //blocks untill either signalPosition or timeout was fired

	// Is timeout ocurred?
	if (!timerPos->isActive()) {
		QMessageBox msgBox;
		const QString title = "Timeout";
		msgBox.setWindowTitle(title);
		msgBox.setText("Position not reached");
		msgBox.exec();
		msgBox.close();
		return;
	}

	tapeCalibrationValue calibrationVal;
	calibrationVal.index = tapeNumber;

	// Search for component in tape
	qnode.sendTask(pap_common::VISION, pap_vision::START_VISION);
	qnode.sendTask(pap_common::VISION, pap_vision::START_TAPE_FINDER,
			componentWidth, componentHeight, 0);

	QEventLoop loop;
	QTimer *timer = new QTimer(this);
	connect(&qnode, SIGNAL(smdCoordinates(float ,float ,float,unsigned int )),
			&loop, SLOT(quit()));
	connect(timer, SIGNAL(timeout()), &loop, SLOT(quit()));
	timer->setSingleShot(true);
	timer->start(1000);

	loop.exec(); //blocks untill either signalPosition or timeout was fired

	// Is timeout ocurred?
	if (!timer->isActive()) {
		qnode.sendTask(pap_common::VISION, pap_vision::STOP_VISION);
		QMessageBox msgBox;
		const QString title = "Nothing received";
		msgBox.setWindowTitle(title);
		msgBox.setText("No component found in tape");
		msgBox.exec();
		msgBox.close();
		return;
	}
	qnode.sendTask(pap_common::VISION, pap_vision::STOP_VISION);

	ROS_INFO("TapeCal: %f CurrentPos: %f", xTapeCalibration, currentPosition.x);
	calibrationVal.x = xTapeCalibration + currentPosition.x;
	calibrationVal.y = yTapeCalibration + currentPosition.y;
	ros::Duration(1).sleep();
	// Search tape dimensions and rotation
	qnode.sendTask(pap_common::VISION, pap_vision::START_VISION);
	qnode.sendTask(pap_common::VISION, pap_vision::START_TAPE_FINDER,
			componentWidth, componentHeight, 1);

	QEventLoop loop2;
	QTimer *timer2 = new QTimer(this);
	connect(&qnode, SIGNAL(smdCoordinates(float ,float ,float,unsigned int )),
			&loop2, SLOT(quit()));
	connect(timer2, SIGNAL(timeout()), &loop, SLOT(quit()));
	timer2->setSingleShot(true);
	timer2->start(1000);

	loop2.exec(); //blocks untill either signalPosition or timeout was fired

	// Is timeout ocurred?
	if (!timer2->isActive()) {
		qnode.sendTask(pap_common::VISION, pap_vision::STOP_VISION);
		QMessageBox msgBox;
		const QString title = "Nothing received";
		msgBox.setWindowTitle(title);
		msgBox.setText("No tape dimensions found");
		msgBox.exec();
		msgBox.close();
		return;
	}
	calibrationVal.rot = rotTapeCalibration;
	if (calibrationVal.rot == -90 || calibrationVal.rot == 90) {
		calibrationVal.rot = 0;
	}
	qnode.sendTask(pap_common::VISION, pap_vision::STOP_VISION);
	ROS_INFO("Tape Calibration: Got x: %f y: %f rot: %f", calibrationVal.x,
			calibrationVal.y, calibrationVal.rot);
	tapeCalibrationValues.push_back(calibrationVal);
}

}
// namespace pap_gui

