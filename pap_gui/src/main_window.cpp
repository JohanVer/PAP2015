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

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace pap_gui {

using namespace Qt;
using namespace std;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

bool componentTableEmpty = true;
bool singleComponentSelected = false;
bool placementProcessRunning = false;
int componentCount = 0;
int boxNumberMax = 59;
int boxNumberMin = 0;
int boxNumberSug = 1;

struct databaseEntry {
	QString package;
	float length, width, height;
	int pins;
};

ComponentPlacerData placementData;

struct componentEntry {
	string name, package, side, value;
	float posX, posY, length, width, height;
	int box, rotation, pins;
};

QVector<componentEntry> componentVector;
QVector<databaseEntry> databaseVector;
componentEntry singleComponent;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
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
	QObject::connect(&qnode, SIGNAL(smdCoordinates(float ,float ,float )), this,
			SLOT(displaySMDCoords(float ,float ,float )));
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
	// Cameras
	QObject::connect(&qnode, SIGNAL(cameraUpdated(int )), this,
			SLOT(cameraUpdated(int )));

	// Status
	QObject::connect(&qnode, SIGNAL(statusUpdated(int)), this,
			SLOT(statusUpdated(int)));

	connect(ui.xManPos, SIGNAL(released()), this, SLOT(releasexManPos()));
	connect(ui.xManNeg, SIGNAL(released()), this, SLOT(releasexManNeg()));
	connect(ui.YManPos, SIGNAL(released()), this, SLOT(releaseyManPos()));
	// Show Ros status dockwidget
	ui.dock_status->show();

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

	/* Load database */
	loadDatabaseContent();
	updateDatabaseTable();
	initFiducialTable();
	initPadTable(1);
	id_ = 0;
	sizeDefined_ = false;
	padFileLoaded_ = false;
	//ui.checkBox_box->setDisabled(true);
}

MainWindow::~MainWindow() {
}

/********************************************************************"*********
 ** Implementation [Slots]
 *****************************************************************************/

void MainWindow::on_scanPCBButton_clicked() {

	// Go with camera 1 to PCB fence

	// take picture / start stitching process

	// store image
}

void MainWindow::on_setCompBoxNrButton_2_clicked() {

	if (singleComponentSelected) {

		bool ok = false;
		QInputDialog* inputDialog = new QInputDialog();
		inputDialog->setOptions(QInputDialog::NoButtons);

		/* Get boxNumber input */
		int currentBox = singleComponent.box;
		if (currentBox == -1) {
			currentBox = 1;
		}

		int boxNumber = inputDialog->getInt(this, "Set new box number",
				"Enter new box number:", currentBox);

		if (boxNumber <= boxNumberMax && boxNumber >= boxNumberMin) {
			singleComponent.box = boxNumber;
			ui.checkBox_box->setChecked(true);
			updatePlacementData();
			ui.label_compBox_2->setText(QString::number(boxNumber));

		} else {
			QMessageBox msgBox;
			msgBox.setText("BoxNumberMin = 0, BoxNumberMax = 59");
			msgBox.exec();
			msgBox.close();

		}
	} else {
		QMessageBox msgBox;
		msgBox.setText("No component information available.");
		msgBox.exec();
		msgBox.close();
	}
}

void MainWindow::on_pickupComponentButton_clicked() {

	// Check box number, length, width and height

	// If all within limits, no component is currently picked-up, not within process -> go to box and pick it up.
	qnode.sendTask(pap_common::PLACER, pap_common::PICKUPCOMPONENT,
			placementData);
}

void MainWindow::on_goToComponentButton_clicked() {
	if (singleComponent.box == -1) {
		QMessageBox msgBox;
		msgBox.setText("Please set box number first.");
		msgBox.exec();
		msgBox.close();
	} else {
		qnode.sendTask(pap_common::PLACER, pap_common::GOTOBOX, placementData);
	}
}

void MainWindow::on_goToPCBButton_clicked() {
	qnode.sendTask(pap_common::PLACER, pap_common::GOTOPCB, placementData);
}

void MainWindow::on_placeComponentButton_clicked() {
	qnode.sendTask(pap_common::PLACER, pap_common::PLACEMENT, placementData);
}

void MainWindow::on_setCompBoxNrButton_clicked() {

	// Check if table empty
	if (!componentTableEmpty) {

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
			int currentBox = componentVector.at(currentComponent).box;
			if (currentBox == -1) {
				currentBox = boxNumberSug;
			}

			//int boxNumber = inputDialog->getInt(this, "enter number", "enter number", currentBox, boxNumberMax, boxNumberMin, boxNumberStep, &ok, 0);
			int boxNumber = inputDialog->getInt(this, "enter number",
					"enter number", currentBox);

			if (boxNumber <= boxNumberMax && boxNumber >= boxNumberMin) {
				componentVector[currentComponent].box = boxNumber;
				ui.label_compBox->setText(QString::number(boxNumber));
				boxNumberSug++;

			} else {
				QMessageBox msgBox;
				msgBox.setText("BoxNumberMin = 1, BoxNumberMax = 50");
				msgBox.exec();
				msgBox.close();

			}
		}
		/* If table empty */
	} else {
		QMessageBox msgBox;
		msgBox.setText(
				"Component table empty. Please select gerber file first.");
		msgBox.exec();
		msgBox.close();
	}
}

void MainWindow::updateSingleComponentInformation() {

	ui.label_compName_2->setText(singleComponent.name.c_str());
	ui.label_compValue_2->setText(singleComponent.value.c_str());
	ui.label_compPackage_2->setText(singleComponent.package.c_str());

	// Check if package exsists in database
	int packageID = -1;
	for (int i = 0; i < databaseVector.size(); i++) {
		string currentPackage = singleComponent.package;

		if (databaseVector.at(i).package
				== QString::fromStdString(currentPackage)) {
			packageID = i;
			break;
		}
	}

	if (packageID == -1) {		// Package not found
		ui.label_compLength_2->setText("not found");
		ui.label_compWidth_2->setText("not found");
		ui.label_compHeight_2->setText("not found");
		ui.label_compPins_2->setText("not found");
	} else {					// Package found
		ui.label_compLength_2->setText(
				QString::number(databaseVector.at(packageID).length, 'f', 2));
		singleComponent.length = databaseVector.at(packageID).length;
		ui.label_compWidth_2->setText(
				QString::number(databaseVector.at(packageID).width, 'f', 2));
		singleComponent.width = databaseVector.at(packageID).width;
		ui.label_compHeight_2->setText(
				QString::number(databaseVector.at(packageID).height, 'f', 2));
		singleComponent.height = databaseVector.at(packageID).height;
		ui.label_compPins_2->setText(
				QString::number(databaseVector.at(packageID).pins));
	}

	ui.label_compPos_2->setText(
			QString::number(singleComponent.posX, 'f', 2) + " / "
					+ QString::number(singleComponent.posY, 'f', 2));
	ui.label_compOrient_2->setText(QString::number(singleComponent.rotation));
	ui.label_compSide_2->setText(singleComponent.side.c_str());

	if (singleComponent.box == -1) {
		ui.label_compBox_2->setText("unknown");
	} else {
		ui.label_compBox_2->setText(QString::number(singleComponent.box));
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

		ui.label_compName->setText(
				componentVector.at(currentComp).name.c_str());
		ui.label_compValue->setText(
				componentVector.at(currentComp).value.c_str());
		ui.label_compPackage->setText(
				componentVector.at(currentComp).package.c_str());

		int packageID = -1;
		for (int i = 0; i < databaseVector.size(); i++) {
			string currentPackage = componentVector.at(currentComp).package;

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
			ui.label_compWidth->setText(
					QString::number(databaseVector.at(packageID).width, 'f',
							2));
			ui.label_compHeight->setText(
					QString::number(databaseVector.at(packageID).height, 'f',
							2));
			ui.label_compPins->setText(
					QString::number(databaseVector.at(packageID).pins));
		}

		ui.label_compPos->setText(
				QString::number(componentVector.at(currentComp).posX, 'f', 2)
						+ " / "
						+ QString::number(componentVector.at(currentComp).posY,
								'f', 2));
		ui.label_compOrient->setText(
				QString::number(componentVector.at(currentComp).rotation));
		ui.label_compSide->setText(
				componentVector.at(currentComp).side.c_str());

		if (componentVector.at(currentComp).box == -1) {
			ui.label_compBox->setText("unknown");
		} else {
			ui.label_compBox->setText(
					QString::number(componentVector.at(currentComp).box));
		}
	}
}

void MainWindow::on_tableWidget_clicked() {
	updateComponentInformation();
}

void MainWindow::on_clearTableButton_clicked() {

	if (componentTableEmpty) {						// Table already empty
		QMessageBox msgBox;
		msgBox.setText("No content to delete.");
		msgBox.exec();
		msgBox.close();
	} else {										// Clear table
		while (!componentVector.isEmpty()) {
			componentVector.remove(0);
		}
		updateComponentTable();
		updateComponentInformation();
	}
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
		int currentRotation = componentVector.at(currentComponent).rotation;
		int rotation = inputDialog->getInt(this, "Change component orientation",
				"Enter component orientation:", currentRotation);
		componentVector[currentComponent].rotation = rotation;
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
				componentVector.at(currentComponent).package);
		//string currentPackage = componentVector.at(currentComponent).package;
		//string package = inputDialog->getText(this, "Enter", "Enter", currentPackage);

		QString text = inputDialog->getText(this, "Change package",
				"Enter new package:", QLineEdit::Normal, currentPackage, &ok);

		if (ok && !text.isEmpty()) {
			ui.label_compPackage->setText(text);
			componentVector[currentComponent].package = text.toStdString();
		}
	}
}

void MainWindow::on_compDeleteButton_clicked() {

	/* Get current component */
	int currentComp = ui.tableWidget->currentRow();

	if (currentComp == -1) {
		QMessageBox msgBox;
		msgBox.setText("No component selected.");
		msgBox.exec();
		msgBox.close();

	} else { /* Delete component from vector */
		componentVector.remove(currentComp);
		updateComponentTable();
		updateComponentInformation();
	}
}

void MainWindow::loadDatabaseContent() {

	std::fstream databaseFile;
	databaseFile.open(
			"/home/nikolas/catkin_ws/src/PAP2015/pap_gui/src/database.txt",
			std::fstream::in | std::fstream::out | std::fstream::app);

	/* ok, proceed  */
	if (databaseFile.is_open()) {

		string lineString;
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
				/*qDebug() << newDatabaseEntry.package << " "
				 << newDatabaseEntry.length << " "
				 << newDatabaseEntry.width << " "
				 << newDatabaseEntry.height << " "
				 << newDatabaseEntry.pins;*/
				databaseVector.append(newDatabaseEntry);
			}
		}

	} else {
		qDebug() << "Could not open database!";
	}

	databaseFile.close();

	if (databaseVector.isEmpty()) {
		QMessageBox msgBox;
		msgBox.setText("No database found or not able to read database.");
		msgBox.exec();
		msgBox.close();
	}
}

void MainWindow::on_loadGerberFileButton_clicked() {

	/* Clear component vector and reset component number*/
	componentVector.clear();
	componentCount = 0;

	//get a filename to open
	QString gerberFile = QFileDialog::getOpenFileName(this,
			tr("Open Gerber file"), "/home", tr("Text Files (*.txt *.csv)"));
	std::cout << "Got filename: " << gerberFile.toStdString() << std::endl;

	/* Load gerber file and add new components to vector */
	std::fstream datafile;
	const char *filename = gerberFile.toLatin1().data();
	datafile.open(filename,
			std::fstream::in | std::fstream::out | std::fstream::app);

	/* ok, proceed  */
	if (datafile.is_open()) {

		string componentString;
		//string lineString;
		while (getline(datafile, componentString)) {

			//QString componentString = QString::fromStdString(lineString);
			//QRegExp sep1('"');
			//QRegExp sep2("//");
			//bool ok;

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
				newComponent.posX = atof(
						(componentString.substr(0, pos1)).c_str());
				componentString = componentString.substr(pos1 + 3,
						componentString.size() - pos1);

				pos1 = componentString.find('"');
				newComponent.posY = atof(
						(componentString.substr(0, pos1)).c_str());
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

				componentVector.append(newComponent);
				componentCount++;
			}
		}

	} else {
		std::cout << "Could not open file!" << std::endl;
	}

	datafile.close();

	if (componentCount != 0) {
		componentTableEmpty = false;
		updateComponentTable();
	}
}

void MainWindow::updateComponentTable() {

	// Set size of table
	ui.tableWidget->setRowCount(componentVector.size());
	ui.tableWidget->setColumnCount(3);

	// Set labels
	QStringList hLabels, vLabels;
	hLabels << "Name" << "Value" << "Package";
	for (int i = 1; i < componentVector.size(); i++) {
		vLabels << QString::number(i);
	}
	ui.tableWidget->setHorizontalHeaderLabels(hLabels);
	ui.tableWidget->setVerticalHeaderLabels(vLabels);

	// Set content
	for (int i = 0; i < ui.tableWidget->rowCount(); i++) {

		ui.tableWidget->setItem(i, 0,
				new QTableWidgetItem((componentVector.at(i).name).c_str()));
		ui.tableWidget->setItem(i, 1,
				new QTableWidgetItem((componentVector.at(i).value).c_str()));
		ui.tableWidget->setItem(i, 2,
				new QTableWidgetItem((componentVector.at(i).package).c_str()));

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
	ui.label_compTotal->setText(QString::number(componentVector.size()));
	ui.label_compLeft->setText(QString::number(componentVector.size()));
}

void MainWindow::updateDatabaseTable() {

	// Set size of table
	ui.packageTableWidget->setRowCount(databaseVector.size());
	ui.packageTableWidget->setColumnCount(5);

	// Set labels
	QStringList hLabels, vLabels;
	hLabels << "Package" << "Length" << "Width" << "Height" << "# of Pins";
	for (int i = 1; i < componentVector.size(); i++) {
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

void MainWindow::on_startSinglePlacementButton_clicked() {

	// Check all prerequesits
	if (ui.checkBox_pcbPlaced->isChecked()) {
		if (ui.checkBox_position->isChecked()
				&& ui.checkBox_orientation->isChecked()
				&& ui.checkBox_box->isChecked()) {

		} else {
			QMessageBox msgBox;
			msgBox.setText("Component information not complete.");
			msgBox.exec();
			msgBox.close();
		}
	} else {
		QMessageBox msgBox;
		msgBox.setText("Please place a circuit board.");
		msgBox.exec();
		msgBox.close();
	}

}

// Package that is going to be sent to placeController
void MainWindow::updatePlacementData() {
	placementData.destX = singleComponent.posX;
	placementData.destY = singleComponent.posY;
	placementData.box = singleComponent.box;
	placementData.height = singleComponent.height;
	placementData.length = singleComponent.length;
	placementData.width = singleComponent.width;
	placementData.rotation = singleComponent.rotation;
}

void MainWindow::on_placeSingleComponentButton_clicked() {

	int currentComp = ui.tableWidget->currentRow();

	if (currentComp == -1) {
		QMessageBox msgBox;
		msgBox.setText("Please select a component.");
		msgBox.exec();
		msgBox.close();

	} else {

		singleComponent = componentVector.at(currentComp);
		updateSingleComponentInformation();
		singleComponentSelected = true;

		ui.checkBox_position->setChecked(true);
		ui.checkBox_orientation->setChecked(true);
		if (singleComponent.box != -1) {
			ui.checkBox_box->setChecked(true);
		}

		ui.tab_manager->setCurrentIndex(4);

		updatePlacementData();

		/*QWizard wizard;
		 wizard.addPage(createIntroPage());

		 wizard.setWindowTitle("Trivial Wizard");
		 wizard.show();*/

		// Dialog - sure to place selceted component? - yes/no
		// if yes - switch window to single component, update information box
		// If all boxes checked, placement process can bet started
	}
}
/*
 QWizardPage *MainWindow::createIntroPage()
 {
 QWizardPage *page = new QWizardPage;
 page->setTitle("Introduction");

 QLabel *label = new QLabel("This wizard will help you register your copy "
 "of Super Product Two.");
 label->setWordWrap(true);

 QVBoxLayout *layout = new QVBoxLayout;
 layout->addWidget(label);
 page->setLayout(layout);

 return page;
 }*/

void MainWindow::on_startPlacementButton_clicked() {

	if (componentTableEmpty) {
		QMessageBox msgBox;
		msgBox.setText(
				"Component table empty. Please select gerber file first.");
		msgBox.exec();
		msgBox.close();
	} else {

	}
}

void MainWindow::on_pausePlacementButton_clicked() {

	if (!placementProcessRunning) {
		QMessageBox msgBox;
		msgBox.setText(
				"Component table empty. Please select Gerber file first.");
		msgBox.exec();
		msgBox.close();
	} else {

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
	int width = ui.camera1->width();
	int height = ui.camera1->height() - 2;
	QImage camera1Scaled = qnode.getCamera1()->scaled(width, height,
			Qt::KeepAspectRatio);
	cameraPicture1 = QPixmap::fromImage(camera1Scaled);
	scene_.clear();
	scene_.addPixmap(cameraPicture1);
	ui.camera1->setScene(&scene_);
	ui.camera1->show();
}

void MainWindow::on_startHoming_clicked(bool check) {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::HOMING);
}

void MainWindow::on_switchCurrent_clicked(bool check) {
	ROS_INFO("Sending current switch command...");
	qnode.sendTask(pap_common::CONTROLLER, pap_common::CURRENT);
}

void MainWindow::on_gotoCoord_clicked(bool check) {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::COORD,
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
		break;
	}
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
		valve4Active_ = true;
	} else {
		qnode.sendRelaisTask(6, false);
		qnode.sendRelaisTask(3, true);
		ui.valveToggle4->setText("Off");
		valve4Active_ = false;
	}
}
void MainWindow::on_valveToggle5_clicked(bool check) {
	if (!valve5Active_) {
		qnode.sendRelaisTask(7, true);
		ui.valveToggle5->setText("On");
		valve5Active_ = true;
	} else {
		qnode.sendRelaisTask(7, false);
		ui.valveToggle5->setText("Off");
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

void MainWindow::on_turnRightTipButton_clicked() {
	// Get angle from line edit
	bool ok;
	int requestedAngle = ui.rotationAngleRight->text().toInt(&ok, 10);
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

void MainWindow::on_startChipFinder_Button_clicked() {
	qnode.sendTask(pap_common::VISION, pap_vision::START_CHIP_FINDER);
	displaySMDCoords(0.0, 0.0, 0.0);
}

void MainWindow::on_startSmallSMDFinder_Button_clicked() {
	// TODO: Take size values from the database!
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
	displaySMDCoords(0.0, 0.0, 0.0);
}

void MainWindow::on_startTapeFinder_Button_clicked() {
	// TODO: Take size values from the database!
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
	displaySMDCoords(0.0, 0.0, 0.0);
}

void MainWindow::on_startPadFinder_Button_clicked() {
	displaySMDCoords(0.0, 0.0, 0.0);
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
	displaySMDCoords(0.0, 0.0, 0.0);
}

void MainWindow::displaySMDCoords(float x, float y, float rot) {
	ui.smdXTop_Label->setText(QString("X: ") + QString::number(x));
	ui.smdYTop_Label->setText(QString("Y: ") + QString::number(y));
	ui.smdRotTop_Label->setText(QString("Rot: ") + QString::number(rot));
}

void MainWindow::setCamera1Point(QPointF point) {
	float percentageX = (100.0 / (float) ui.camera1->width()) * point.x();
	float percentageY = (100.0 / (float) ui.camera1->height()) * point.y();
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
	if (ui.fiducialTable->fiducialSize_ == 0) {
		setFiducialTable(0, padPosition_.x(), padPosition_.y());
	} else if (ui.fiducialTable->fiducialSize_ == 1) {
		setFiducialTable(1, padPosition_.x(), padPosition_.y());
	} else {
		ui.fiducialTable->clear();
		ui.fiducialTable->fiducialSize_ = 0;
		initFiducialTable();
		setFiducialTable(0, padPosition_.x(), padPosition_.y());
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
		qnode.sendTask(pap_common::CONTROLLER, pap_common::COORD, x, y, 50.0);
	}
}

void MainWindow::on_inputPad_Button_clicked() {
	//get a filename to open
	QString gerberFile = QFileDialog::getOpenFileName(this, tr("Open Whl file"),
			"/home", tr("Text Files (*.txt  *.Whl)"));

	const char *filenameWhl = gerberFile.toLatin1().data();

	padParser.parseShapes(filenameWhl);

	//get a filename to open
	gerberFile = QFileDialog::getOpenFileName(this, tr("Open PasteBot file"),
			"/home", tr("Text Files (*.txt  *.PasteBot)"));

	const char *filenamePaste = gerberFile.toLatin1().data();

	// First load shape data from .Whl file
	padParser.loadFile(filenamePaste);
	// Then load cad file .PasteBot
	padParser.setTable(ui.padTable);
	padFileLoaded_ = true;
}

void MainWindow::on_padViewSetSize_button_clicked() {
	padParser.setSize(ui.padViewHeight_text->text().toFloat(),
			ui.padViewWidth_text->text().toFloat());
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
	padParser.renderImage(&scenePads_, ui.padView_Image->width() - 20,
			ui.padView_Image->height() - 20);

	ui.padView_Image->setScene(&scenePads_);
	ui.padView_Image->show();
}

void MainWindow::padPressed(int numberOfFiducial, QPointF padPos) {
	id_ = padParser.searchId(padPos, ui.padView_Image->width() - 20);
	setFiducialPads(numberOfFiducial,
			padParser.padInformationArray_[id_].rect.x(),
			padParser.padInformationArray_[id_].rect.y());
}

void MainWindow::on_calcOrientation_Button_clicked(){
	QPointF local1,global1,local2,global2;
	local1.setX(0.0);
	local1.setY(0.0);
	local2.setX(0.0);
	local2.setY(0.0);
	global1.setX(ui.fiducialTable->item(0, 0)->text().toFloat());
	global1.setY(ui.fiducialTable->item(0, 1)->text().toFloat());
	global2.setX(ui.fiducialTable->item(1, 0)->text().toFloat());
	global2.setY(ui.fiducialTable->item(1, 1)->text().toFloat());
	padParser.calibratePads(local1,local2,global1,global2);
	padParser.rotatePads();
}
}
// namespace pap_gui

