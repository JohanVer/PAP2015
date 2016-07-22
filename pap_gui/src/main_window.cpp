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
#include <pap_gui/main_window.hpp>
#include <pap_gui/GerberPadParser.hpp>
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
#include <pap_gui/DatabaseClass.hpp>
#include <pap_gui/packageDialog.hpp>

/*****************************************************************************
 ** Namespaces
 ****************************************************************************/
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
    QWidget::connect(&scene_, SIGNAL(sendMousePoint(QPointF)), this,
                     SLOT(setCamera1Point(QPointF)));
    QWidget::connect(&scene_, SIGNAL(setFiducial(QPointF)), this,
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
    QObject::connect(&qnode, SIGNAL(statusUpdated()), this,
                     SLOT(statusUpdated()));

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

    // Pressure
    connect(&qnode, SIGNAL(updatePressure(float)) , this, SLOT(updatePressure(float)));

    connect(ui.xManPos, SIGNAL(released()), this, SLOT(releasexManPos()));
    connect(ui.xManNeg, SIGNAL(released()), this, SLOT(releasexManNeg()));
    connect(ui.YManPos, SIGNAL(released()), this, SLOT(releaseyManPos()));
    connect(ui.YManNeg, SIGNAL(released()), this, SLOT(releaseyManNeg()));
    connect(ui.ZManPos, SIGNAL(released()), this, SLOT(releasezManPos()));
    connect(ui.ZManNeg, SIGNAL(released()), this, SLOT(releasezManNeg()));

    // New pad creation
    QWidget::connect(&scenePads_, SIGNAL(createPad(QRectF)), this,
                     SLOT(createNewPad(QRectF)));

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

    leftTipRadius = 0.0;
    rightTipRadius = 0.0;

    completePlacementRunning = false;
    singlePlacementRunning = false;
    componentIndicator = 0;
    completeCalibrationRunning = false;
    ui.pushButton_recalibrate_left_tip->setEnabled(false);
    ui.pushButton_recalibrate_right_tip->setEnabled(false);

    xTapeCalibration = 0.0;
    yTapeCalibration = 0.0;
    rotTapeCalibration = 0.0;

    /* Load database */
    dataIO.loadDatabase(databaseVector);
    updateDatabaseTable();

    initFiducialTable();
    id_ = 0;
    sizeDefined_ = false;
    padFileLoaded_ = false;
    dispenserPaused = false;
    lastDispenserId = 0;

    ui.padView_Image->setScene(&scenePads_);
    ui.padView_Image->show();
    ui.camera1->setScene(&scene_);
    ui.camera1->show();
    ui.camera1_2->setScene(&scene_);
    ui.camera1_2->show();
    ui.camera2->setScene(&scene2_);
    ui.camera2->show();

    currentPosition.x = 0.0;
    currentPosition.y = 0.0;
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
        ros::Duration(0.5).sleep();
        on_switchCurrent_clicked(true);
        ros::Duration(0.5).sleep();
        on_switchCurrent_clicked(true
                                 );
        ui.dock_status->hide();
        ros::Duration(1).sleep();
    } else {
        // Show Ros status dockwidget
        ui.dock_status->show();
    }

    edge_percentage_ = 0.1;
    dispenser_velocity_ = 1.0;
    nozzle_diameter_ = 0.41;
    alpha_ = 0.2;
    planner_selection_ = DispenserPlanner::PLANNER_SELECT::DOT_PLANNER;
    wait_time_ = 1;
    alignment_ = DispenserPlanner::DOT_ALIGN::CENTER;

    tip_thresholding_on = false;
    dispenser_height_offset_ = 0;
    PCBTransformCalibrated_ = false;
    tapeCalibrated_ = false;

    padParser.setDispenserInfo(nozzle_diameter_, edge_percentage_);
    tape_calibrater_ = std::unique_ptr<pap_gui::TapeCalibrater>(new TapeCalibrater(qnode));
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
                                                      tr("Open Gerber file"), "/home/johan/catkin_ws/src/PAP2015/PAP/pickPlace", tr("Text Files (*.txt *.csv *.TXT)"));
    std::cout << "Got filename: " << gerberFile.toStdString() << std::endl;

    /* Load gerber file and add components to componentList */
    std::fstream datafile;
    const char *filename = gerberFile.toLatin1().data();
    datafile.open(filename,
                  std::fstream::in | std::fstream::out | std::fstream::app);

    /* ok, proceed  */
    if (datafile.is_open()) {

        string componentString;
        bool inDatabase;
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
                pos1 = componentString.find("//");
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
    updateCompDimensions();
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

void MainWindow::updateCompDimensions() {

    int databasePos = -1;
    /* Iterate over all parts */
    for (size_t i = 0; i < componentList.size(); i++) {

        databasePos = -1;
        for (size_t k = 0; k < databaseVector.size(); k++) {
            /* Check if part is in database */
            if (componentList.at(i).package.compare(
                        databaseVector.at(k).package.toStdString()) == 0) {
                databasePos = k;
                break;
            }
        }

        /* If part is not in database -> add */
        if (databasePos == -1) {
            componentList[i].length = -1.0;
            componentList[i].width = -1.0;
            componentList[i].height = -1.0;
            componentList[i].pins = -1;
        } else {
            componentList[i].length = databaseVector.at(databasePos).length;
            componentList[i].width = databaseVector.at(databasePos).width;
            componentList[i].height = databaseVector.at(databasePos).height;
            componentList[i].pins = databaseVector.at(databasePos).pins;
        }
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
    QMessageBox msgBox;
    // Check if table empty
    if (!componentList.isEmpty()) {

        // All types of components need to be known
        if (missingPackageList.isEmpty()) {
            SlotSelectorDialog slotWizard(&componentList, &databaseVector);
            connect(&slotWizard, SIGNAL(setLed(int)), this,
                    SLOT(setLedFromSelection(int)));
            slotWizard.exec();
            updateComponentInformation();
            return;
        } else {
            QString message = QString(
                        "There are still %1 unknown packages which are needed to perform a complete PCB placement.\nPlease add, delete or replace them first.").arg(
                        missingPackageList.size());
            msgBox.setText(message);
        }
    } else {
        msgBox.setText("Component table empty. Please load gerber file first.");
    }
    msgBox.exec();
    msgBox.close();
}

void MainWindow::on_compDeleteButton_clicked() {

    /* Get current component */
    int currentComp = ui.tableWidget->currentRow();
    if (currentComp == -1) {
        showSelectCompMessage();
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
        showSelectCompMessage();
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

    // Get current component
    int currentComponent = ui.tableWidget->currentRow();

    /* If no component selected */
    if (currentComponent == -1) {
        showSelectCompMessage();
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
        updateCompDimensions();
        updateComponentInformation();
        updateComponentTable();
        updateMissingPackageList();
        updateMissingPackageTable();
    }
}

bool MainWindow::isPCBCalibrated(){
    if(PCBTransformCalibrated_)
       return true;

    QMessageBox msgBox;
    msgBox.setText("Please compute PCB transform first.");
    msgBox.exec();
    msgBox.close();
    return false;
}

bool MainWindow::isTapeCalibrated(){
    if(tapeCalibrated_)
       return true;

    QMessageBox msgBox;
    msgBox.setText("Please calibrate tapes first.");
    msgBox.exec();
    msgBox.close();
    return false;
}

bool MainWindow::checkSlotsAndTapeCalibration() {
    for (size_t k = 0; k < componentList.size(); k++) {
        int box = componentList.at(k).box;
        if (box == -1) {
            QMessageBox msgBox;
            msgBox.setText("There are still components without assigned slot.");
            msgBox.exec();
            msgBox.close();
            return false;
        } else if((box >= 67) && (box <= 86)) {
            if(!isTapeCalibrated())
                return false;
        }
    }
    return true;
}

void MainWindow::on_stopPlacementButton_clicked() {

    qnode.sendTask(pap_common::PLACER, pap_common::STOP);

    // Clear component queues
    placementPlanner.resetQueues();
    // Reset flags
    completePlacementRunning = false;
    singlePlacementRunning = false;
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
            updateCompDimensions();
            updateComponentInformation();
            updateMissingPackageList();
            updateMissingPackageTable();
            ROS_INFO("GUI: %d components have been replaced.", replaceCounter);
        }
    }
}

void MainWindow::on_addPackageButton_clicked() {
    int missing_package = ui.missingPackageTableWidget->currentRow();
    PackageDialog* packageDialog = new PackageDialog(&databaseVector,
                                                     &missingPackageList, missing_package);
    packageDialog->exec();
    updateDatabaseTable();
    updateCompDimensions();
    updateComponentInformation();
    updateMissingPackageList();
    updateMissingPackageTable();
}

void MainWindow::on_editPackageButton_clicked() {

    int current_package = ui.packageTableWidget->currentRow();
    if (current_package != -1) {
        PackageDialog* packageDialog = new PackageDialog(&databaseVector, NULL,
                                                         current_package);
        packageDialog->exec();
        updateDatabaseTable();
        updateCompDimensions();
        updateComponentInformation();
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
        updateCompDimensions();
        updateComponentInformation();
        updateMissingPackageList();
        updateMissingPackageTable();
    } else {
        QMessageBox msgBox;
        msgBox.setText("No package selected.");
        msgBox.exec();
        msgBox.close();
    }
}

void MainWindow::setLedFromSelection(int selection) {
    if (selection != -1) {
        ROS_INFO("GUI: Set led number %d", selection);
        qnode.setLEDTask(selection);
    } else {
        qnode.LEDTask(pap_common::RESETALLLED, 0);
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

        // No package info available
        if (componentList.at(currentComp).length == -1.0
                && componentList.at(currentComp).width == -1.0) {
            ui.label_compLength->setText("not found");
            ui.label_compWidth->setText("not found");
            ui.label_compHeight->setText("not found");
            ui.label_compPins->setText("not found");
            // Package info found
        } else {
            ui.label_compLength->setText(
                        QString::number(componentList.at(currentComp).length, 'f',
                                        2));
            ui.label_compWidth->setText(
                        QString::number(componentList.at(currentComp).width, 'f',
                                        2));
            ui.label_compHeight->setText(
                        QString::number(componentList.at(currentComp).height, 'f',
                                        2));
            ui.label_compPins->setText(
                        QString::number(componentList.at(currentComp).pins));
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
    ui.missingPackageTableWidget->horizontalHeader()->setSectionResizeMode(
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
    ui.tableWidget->setSizePolicy(QSizePolicy::Expanding,
                                  QSizePolicy::Expanding);
    ui.tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui.tableWidget->show();
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
    ui.packageTableWidget->horizontalHeader()->setSectionResizeMode(
                QHeaderView::Stretch);

    //ui.tableWidget->setWindowTitle("QTableWidget");
    ui.tableWidget->show();
}

/*************************************************************************************************************
 ** Placement control buttons
 **************************************************************************************************************/

void MainWindow::on_goToPCBButton_clicked() {
    ui.tab_manager->setCurrentIndex(1);
    qnode.sendTask(pap_common::PLACER, pap_common::GOTOPCB);
}

void MainWindow::transformSingleComp(int currentComp, ComponentPlacerData& singleCompData) {

    componentEntry entryToTransform;
    entryToTransform = componentList[currentComp];
    padParser.transformComponent(&entryToTransform);

    // Is it a tape?
    if ((entryToTransform.box >= 67) && (entryToTransform.box <= 86)) {
        unsigned int tape_nr = entryToTransform.box - 67;
        Offset tapePartPos;
        if(!tape_calibrater_->calculatePosOfTapePart(tape_nr, tapeCompCounter[tape_nr], tapePartPos )){
            std::cerr << "Error while caluclating tape part position...\n";
        }
        singleCompData.tapeX = tapePartPos.x;
        singleCompData.tapeY = tapePartPos.y;
        singleCompData.tapeRot = tapePartPos.rot;
        tapeCompCounter[tape_nr]++;
    } else {
        singleCompData.tapeX = 0.0;
        singleCompData.tapeY = 0.0;
        singleCompData.tapeRot = 0.0;
    }

    singleCompData.destX = entryToTransform.posX;
    singleCompData.destY = entryToTransform.posY;
    singleCompData.box = entryToTransform.box;
    singleCompData.height = entryToTransform.height;
    singleCompData.length = entryToTransform.length;
    singleCompData.width = entryToTransform.width;
    singleCompData.rotation = entryToTransform.rotation;
    singleCompData.name = entryToTransform.name;
}

void MainWindow::transformAllComp(vector<ComponentPlacerData>& allCompData) {

    for(int i = 0; i < componentList.size(); i++) {

        componentEntry tmpCompEntry;
        ComponentPlacerData tmpCompPlaceData;

        tmpCompEntry = componentList.at(i);
        padParser.transformComponent(&tmpCompEntry);

        // Is it a tape?
        if ((tmpCompEntry.box >= 67) && (tmpCompEntry.box <= 86)) {
            unsigned int tape_nr = tmpCompEntry.box - 67;
            Offset tapePartPos;
            if(!tape_calibrater_->calculatePosOfTapePart(tape_nr, tapeCompCounter[tape_nr], tapePartPos )){
                std::cerr << "Error while caluclating tape part position...\n";
            }
            tmpCompPlaceData.tapeX = tapePartPos.x;
            tmpCompPlaceData.tapeY = tapePartPos.y;
            tmpCompPlaceData.tapeRot = tapePartPos.rot;
            tapeCompCounter[tape_nr]++;
        } else {
            tmpCompPlaceData.tapeX = 0.0;
            tmpCompPlaceData.tapeY = 0.0;
            tmpCompPlaceData.tapeRot = 0.0;
        }

        tmpCompPlaceData.destX = tmpCompEntry.posX;
        tmpCompPlaceData.destY = tmpCompEntry.posY;
        tmpCompPlaceData.box = tmpCompEntry.box;
        tmpCompPlaceData.height = tmpCompEntry.height;
        tmpCompPlaceData.length = tmpCompEntry.length;
        tmpCompPlaceData.width = tmpCompEntry.width;
        tmpCompPlaceData.rotation = tmpCompEntry.rotation;
        tmpCompPlaceData.name = tmpCompEntry.name;
        allCompData.push_back(tmpCompPlaceData);
    }
}

void MainWindow::on_startPlacementButton_clicked() {

    if(!isPressureEnough()) return;
    if(!isPCBCalibrated()) return;

    // Components to place?
    if (componentList.isEmpty()) {
        QMessageBox msgBox;
        msgBox.setText("Component table empty. Please load gerber file first.");
        msgBox.exec();
        msgBox.close();
        return;
    }
    if (!checkSlotsAndTapeCalibration()) return;

    // Start placement process
    if (!completePlacementRunning && !singlePlacementRunning) {
        vector<ComponentPlacerData> allCompData;
        transformAllComp(allCompData);
        updateCurrentNozzles();
        if(placementPlanner.startCompletePlacement(qnode, allCompData, completePlacementRunning)) {
            if(placementPlanner.leftTipQueue.size() > 0) {
                ui.label_compTotalLeft->setText(QString::number(placementPlanner.leftTipQueue.size()+1));
            }
            if(placementPlanner.rightTipQueue.size() > 0) {
                ui.label_compTotalRight->setText(QString::number(placementPlanner.rightTipQueue.size()+1));
            }
            updatePlacementInfo();
        }
    } else {
        QMessageBox msgBox;
        msgBox.setText("Placement process already running - stop first.");
        msgBox.exec();
        msgBox.close();
    }
}

void MainWindow::on_placeSingleComponentButton_clicked() {

    if(!isPressureEnough()) return;
    if(!isPCBCalibrated()) return;

    int currentComp = ui.tableWidget->currentRow();

    // No component selected
    if (currentComp == -1) {
        showSelectCompMessage();
        return;
    }

    // Missing slots?
    int box = componentList.at(currentComp).box;
    if (box == -1) {
        QMessageBox msgBox;
        msgBox.setText("Please assign slot first.");
        msgBox.exec();
        msgBox.close();
        return;
    } else if((box >= 67) && (box <= 86)) {
        if(!isTapeCalibrated()) return;
    }

    // Start placement process
    if (!completePlacementRunning && !singlePlacementRunning) {
        ComponentPlacerData compPlaceData;
        transformSingleComp(currentComp, compPlaceData);
        updateCurrentNozzles();
        if(placementPlanner.startSingleCompPlacement(qnode, compPlaceData, singlePlacementRunning)) {
            if(placementPlanner.compToPlaceLeft.isWaiting) {
                ui.label_compTotalLeft->setText(QString::number(1));
            }
            if(placementPlanner.compToPlaceRight.isWaiting) {
                ui.label_compTotalRight->setText(QString::number(1));
            }
            updatePlacementInfo();
        }
    } else {
        QMessageBox msgBox;
        msgBox.setText("Another placement process has not finished yet.");
        msgBox.exec();
        msgBox.close();
    }
}

void MainWindow::updatePlacementInfo() {
    if(placementPlanner.compToPlaceLeft.isWaiting) {
        ui.label_placementLeft->setText("Running");
        ui.label_currentCompLeft->setText(QString::fromStdString(placementPlanner.compToPlaceLeft.name));
        ui.label_missingCompLeft->setText(QString::number(placementPlanner.leftTipQueue.size()+1));
    } else {
        ui.label_placementLeft->setText("Finished");
        ui.label_currentCompLeft->setText("-");
        ui.label_missingCompLeft->setText(QString::number(0));
        ui.label_compTotalLeft->setText(QString::number(0));
    }

    if(placementPlanner.compToPlaceRight.isWaiting) {
        ui.label_placementRight->setText("Running");
        ui.label_currentCompRight->setText(QString::fromStdString(placementPlanner.compToPlaceRight.name));
        ui.label_missingCompRight->setText(QString::number(placementPlanner.rightTipQueue.size()+1));
    } else {
        ui.label_placementRight->setText("Finished");
        ui.label_currentCompRight->setText("-");
        ui.label_missingCompRight->setText(QString::number(0));
        ui.label_compTotalRight->setText(QString::number(0));
    }
}

void MainWindow::updateCurrentNozzles() {
    // Get current nozzles
    QString leftNozzle = ui.leftNozzle_comboBox->currentText();
    QString rightNozzle = ui.rightNozzle_comboBox->currentText();

    if(leftNozzle == "-") {
        leftTipRadius = 0.0;
    } else {
        leftTipRadius = (leftNozzle.toFloat())/2;
    }

    if(rightNozzle == "-") {
        rightTipRadius = 0.0;
    } else {
        rightTipRadius = (rightNozzle.toFloat())/2;
    }
    placementPlanner.setTipDiameters(leftTipRadius, rightTipRadius);
}

void MainWindow::showSelectCompMessage() {
    QMessageBox msgBox;
    msgBox.setText("No component selected.");
    msgBox.exec();
    msgBox.close();
}

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

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
    ui.dock_status->hide();
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
        scene_.clear();
        QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(*(qnode.getCamera1())));
        scene_.addItem(item);

        //ui.camera1->fitInView(scene_.sceneRect(), Qt::KeepAspectRatioByExpanding);

    } else if (index == 2) {
        scene2_.clear();
        QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(*(qnode.getCamera2())));
        scene2_.addItem(item);
        //ui.camera2->fitInView(scene2_.sceneRect(), Qt::KeepAspectRatioByExpanding);
    }

}

void MainWindow::on_startHoming_clicked(bool check) {
    qnode.sendTask(pap_common::PLACER, pap_common::HOMING);
}

void MainWindow::on_switchCurrent_clicked(bool check) {
    ROS_INFO("GUI: Sending current switch command...");
    if(!motor_send_functions::sendMotorControllerAction(qnode.getMotorClientRef(), pap_common::CURRENT)){
        std::cerr << "Failed to send current command...\n";
    }
}

void MainWindow::on_gotoCoord_clicked(bool check) {
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

    //ROS_INFO("GUI: placerStatusUpdated: %d, %d !!", state, status);

    // Calibration running
    if(state == pap_common::RATIO_CALIBRATION
            && status == pap_common::PLACER_FINISHED) {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Ratio calibration finished");
        msgBox.setText(QString("All ratios are now calibrated.\nWant to continue with offset calibration?"));
        msgBox.setStandardButtons(QMessageBox::Yes);
        msgBox.addButton(QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);
        if (msgBox.exec() == QMessageBox::Yes) {
            on_calibrationButton_offsets_clicked();
            //qnode.sendTask(pap_common::PLACER, pap_common::CALIBRATION_OFFSET);
            //completeCalibrationRunning = false;
        }
    }

    if(state == pap_common::OFFSET_CALIBRATION
            && status == pap_common::PLACER_FINISHED) {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Offset calibration finished");
        msgBox.setText(QString("All offset are now calibrated.\nWant to home system now?"));
        msgBox.setStandardButtons(QMessageBox::Yes);
        msgBox.addButton(QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);
        if (msgBox.exec() == QMessageBox::Yes) {
            qnode.sendTask(pap_common::PLACER, pap_common::HOMING);
        }
        ui.pushButton_recalibrate_left_tip->setEnabled(true);
        ui.pushButton_recalibrate_right_tip->setEnabled(true);
    }


    // Complete PCB placement running
    if (state == pap_common::PLACECOMPONENT_STATE
            && status == pap_common::PLACER_FINISHED) {

        //Remove placed components from table
        for(int i = 0; i  < componentList.size(); i++) {
            if((componentList.at(i).name.compare(placementPlanner.compToPlaceLeft.name) == 0) ||
                    (componentList.at(i).name.compare(placementPlanner.compToPlaceRight.name) == 0)){
                componentList.remove(i);
            }
        }
        updateComponentTable();
        updateComponentInformation();

        if(completePlacementRunning) {
            if(!placementPlanner.sendNextTask(qnode)){
                completePlacementRunning = false;
            }
        }
        updatePlacementInfo();
    }

    // Single component placement running
    if (state == pap_common::PLACECOMPONENT_STATE
            && status == pap_common::PLACER_FINISHED
            && singlePlacementRunning) {
        singlePlacementRunning = false;
        placementPlanner.resetQueues();
        updatePlacementInfo();
    }

    QPixmap statePixmap(QSize(20, 20));
    statePixmap.fill(Qt::transparent);
    QPainter p(&statePixmap);
    p.setRenderHint(QPainter::Antialiasing, true);
    QPen pen(Qt::black, 1);
    p.setPen(pen);
    QBrush brushWhite(Qt::white);
    QBrush brushGreen(Qt::green);
    QBrush brushRed(Qt::red);

    switch (status) {
    case pap_common::PLACER_IDLE:
        p.setBrush(brushWhite);
        break;
    case pap_common::PLACER_ACTIVE:
        p.setBrush(brushGreen);
        break;
    case pap_common::PLACER_FINISHED:
        p.setBrush(brushGreen);
        break;
    case pap_common::PLACER_ERROR:
        p.setBrush(brushRed);
    }

    p.drawEllipse(5, 5, 10, 10);

    switch (state) {
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
    }
}

void MainWindow::statusUpdated() {

    // Energized

    if ((qnode.getStatus(0)).energized) {
        ui.powerLabel1->setText("on");
    } else {
        ui.powerLabel1->setText("off");
    }

    if ((qnode.getStatus(1)).energized) {
        ui.powerLabel2->setText("on");
    } else {
        ui.powerLabel2->setText("off");
    }
    if ((qnode.getStatus(2)).energized) {
        ui.powerLabel3->setText("on");
    } else {
        ui.powerLabel3->setText("off");
    }

    // Error

    if ((qnode.getStatus(0)).error) {
        ui.errorLabel1->setText("ERROR");
    } else {
        ui.errorLabel1->setText("OK");
    }

    if ((qnode.getStatus(1)).error) {
        ui.errorLabel2->setText("ERROR");
    } else {
        ui.errorLabel2->setText("OK");
    }

    if ((qnode.getStatus(2)).error) {
        ui.errorLabel3->setText("ERROR");
    } else {
        ui.errorLabel3->setText("OK");
    }

    // Reached

    if ((qnode.getStatus(0)).positionReached) {
        ui.posLabel1->setText("reached");
    } else {
        ui.posLabel1->setText("busy");
    }

    if ((qnode.getStatus(1)).positionReached) {
        ui.posLabel2->setText("reached");
    } else {
        ui.posLabel2->setText("busy");
    }

    if ((qnode.getStatus(2)).positionReached) {
        ui.posLabel3->setText("reached");
    } else {
        ui.posLabel3->setText("busy");
    }

    // Position

    float x_pos = qnode.getStatus(0).position;
    if (x_pos != 0.0) {
        ui.label_posX->setText(
                    QString::number(x_pos));
        currentPosition.x = x_pos;
    }

    float y_pos = qnode.getStatus(1).position;
    if (y_pos != 0.0) {
        ui.label_posY->setText(
                    QString::number(y_pos));
        currentPosition.y = y_pos;
    }

    float z_pos = qnode.getStatus(2).position;
    if (z_pos != 0.0) {
        ui.label_posZ->setText(
                    QString::number(z_pos));
        currentPosition.z = z_pos;
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
        qnode.sendRelaisTask(4, true);
        ui.valveToggle1->setText("On");
        valve1Active_ = true;
        ROS_INFO("Toggle");
    } else {
        qnode.sendRelaisTask(4, false);
        ui.valveToggle1->setText("Off");
        valve1Active_ = false;
    }
}

void MainWindow::on_valveToggle2_clicked(bool check) {
    if (!valve2Active_) {
        qnode.sendRelaisTask(5, true);
        ui.valveToggle2->setText("On");
        valve2Active_ = true;
    } else {
        qnode.sendRelaisTask(5, false);
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
    float requestedAngle = ui.rotationAngleLeft->text().toFloat(&ok);
    if (ok == true) {
        qnode.sendTask(pap_common::PLACER, pap_common::STEPPER_ROTATION, requestedAngle, TIP::LEFT_TIP, 0.0);
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
    float requestedAngle = ui.rotationAngleRight->text().toFloat(&ok);
    if (ok == true) {
        qnode.sendTask(pap_common::PLACER, pap_common::STEPPER_ROTATION, requestedAngle, TIP::RIGHT_TIP, 0.0);
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

void MainWindow::updatePressure(float pressure){
    if(qnode.fakePadPos_) {
        pressure_ = 15.0;
    } else {
        pressure_ = pressure;
    }
    ui.pressure_label->setText(QString::fromStdString(std::to_string(pressure_)));
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


void MainWindow::on_startTapeFinder_Button_clicked() {

    bool search_tape = false;
    search_tape = ui.search_tape_radio_button->isChecked();
    if (ui.tapeFinder_Combo->currentText() == "0805") {
        qnode.sendTask(pap_common::VISION, pap_vision::START_TAPE_FINDER, 1.25,
                       2.0, (float) search_tape);
    } else if (ui.tapeFinder_Combo->currentText() == "0603") {
        qnode.sendTask(pap_common::VISION, pap_vision::START_TAPE_FINDER, 0.8,
                       1.6, (float) search_tape);
    } else if (ui.tapeFinder_Combo->currentText() == "0402") {
        qnode.sendTask(pap_common::VISION, pap_vision::START_TAPE_FINDER, 0.5,
                       1.0, (float) search_tape);
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
        vision_send_functions::sendVisionTask(qnode.getVisionClientRef(), pap_vision::START_VISION);
        ui.visionStatus_Label->setText("On");
        visionStarted_ = true;
    } else {
        vision_send_functions::sendVisionTask(qnode.getVisionClientRef(), pap_vision::STOP_VISION);
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
}

void MainWindow::setFiducial(QPointF point) {
    float indexXFull = point.x();
    float indexYFull = point.y();

    pap_common::VisionResult res;
    std::cerr << "Sending fiducial search task \n";
    if(vision_send_functions::sendVisionTask(qnode.getVisionClientRef(), pap_vision::START_PAD_FINDER, pap_vision::CAMERA_TOP, 1, indexXFull, indexYFull ,res)){
        QPointF padPos (res.data1, res.data2);

        if (ui.fiducialTable->fiducialSize_ == 0) {
            setFiducialTable(0, padPos.x() + currentPosition.x,
                             padPos.y() + currentPosition.y);
        } else if (ui.fiducialTable->fiducialSize_ == 1) {
            setFiducialTable(1, padPos.x() + currentPosition.x,
                             padPos.y() + currentPosition.y);
        } else {
            ui.fiducialTable->clear();
            ui.fiducialTable->fiducialSize_ = 0;
            initFiducialTable();
            setFiducialTable(0, padPos.x() + currentPosition.x,
                             padPos.y() + currentPosition.y);
        }

    }else{
        QMessageBox msgBox;
        const QString title = "Nothing received";
        msgBox.setWindowTitle(title);
        msgBox.setText("No pad found on given position");
        msgBox.exec();
        msgBox.close();
        return;
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
    /*
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
    */
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
    //ROS_INFO("GUI: PadPos: %f %f", padPosition_.x(), padPosition_.y());
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
    if (qnode.getStatus(0).positionReached
            && qnode.getStatus(1).positionReached
            && qnode.getStatus(2).positionReached) {
        if (indexOfFiducial == 0 && ui.fiducialTable->fiducialSize_ < 1) {
            return;
        } else if (indexOfFiducial == 1
                   && ui.fiducialTable->fiducialSize_ < 2) {
            return;
        }
        float x = ui.fiducialTable->item(indexOfFiducial, 2)->text().toFloat();
        float y = ui.fiducialTable->item(indexOfFiducial, 3)->text().toFloat();
        ROS_INFO("GUI: Goto position x: %f y: %f", x, y);
        //qnode.sendTask(pap_common::CONTROLLER, pap_common::COORD, x, y, 22.0);
        qnode.sendTask(pap_common::PLACER, pap_common::GOTO, x, y, 22.0);
    }
}

void MainWindow::on_inputPad_Button_clicked() {
    //get a filename to open

    dispensed_ids_.clear();
    padParser.reset();

    if (!sizeDefined_) {
        QMessageBox msgBox;
        const QString title = "Not initialized";
        msgBox.setWindowTitle(title);
        msgBox.setText(
                    "Make sure that size is set!");
        msgBox.exec();
        msgBox.close();
        return;
    }

    std::string package_path = ros::package::getPath("PAP_resources");
    QString gerberFile = QFileDialog::getOpenFileName(this, tr("Open Whl file"),
                                                      QString((package_path+"/dispensing").c_str()), tr("Text Files (*.txt  *.Whl)"));

    const char *filenameWhl = gerberFile.toLatin1().data();

    padParser.parseShapes(filenameWhl);

    //get a filename to open
    gerberFile = QFileDialog::getOpenFileName(this, tr("Open PasteBot/Top file"),
                                              QString((package_path+"/dispensing").c_str()), tr("Text Files (*.txt  *.PasteBot *.PasteTop)"));

    bottomLayer_ = false;
    if (gerberFile.contains(".PasteBot")) {
        bottomLayer_ = true;
    } else {
        bottomLayer_ = false;
    }
    const char *filenamePaste = gerberFile.toLatin1().data();

    padParser.pixelConversionFactor = 30;
    // First load shape data from .Whl file
    padParser.loadFile(filenamePaste, bottomLayer_);

    // Build image with QGraphicsItem
    pic_offset_.width = 0;
    pic_offset_.height =  -qnode.pcbHeight_ * padParser.pixelConversionFactor;
    std::cerr << "Background size: " << pic_offset_ << std::endl;
    padParser.deleteBackground();
    QImage background = QImage(qnode.pcbWidth_*padParser.pixelConversionFactor, qnode.pcbHeight_ * padParser.pixelConversionFactor, QImage::Format_ARGB32);
    background.fill(qRgba(0,255,0,200));
    padParser.setBackGround(background.copy());

    //padParser.setTable(ui.padTable);
    padFileLoaded_ = true;

    //Render image
    redrawPadView();
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
    padParser.pixelConversionFactor = 30;
    padParser.deleteBackground();
    pic_offset_.width = ui.padView_Image->width();
    pic_offset_.height = ui.padView_Image->height();

    redrawPadView();

}

void MainWindow::padPressed(int numberOfFiducial, QPointF padPos) {
    id_ = padParser.searchId(padPos);
    setFiducialPads(numberOfFiducial,
                    padParser.padInformationArrayPrint_[id_].rect.x(),
                    padParser.padInformationArrayPrint_[id_].rect.y());
}

void MainWindow::gotoPad(QPointF padPos) {
    ROS_INFO("GUI: Goto Pad....");
    id_ = padParser.searchId(padPos);
    ROS_INFO("GUI: ID: %d", id_);

    float x = padParser.padInformationArray_[id_].rect.x();
    float y = padParser.padInformationArray_[id_].rect.y();
    ROS_INFO("GUI: Goto position x: %f y: %f", x, y);
    qnode.sendTask(pap_common::PLACER, pap_common::GOTO, x, y, 27.0);
    //}
}

void MainWindow::redrawPadView(){
    padParser.renderImage(&scenePads_, pic_offset_.width, pic_offset_.height, dispensed_ids_);
    //padParser.setTable(ui.padTable);
    qnode.sendPcbImage(padParser.getMarkerList());
}

void MainWindow::deletePad(QPointF padPos) {
    id_ = padParser.searchId(padPos);
    padParser.deleteEntry(id_);
    redrawPadView();
}

void MainWindow::on_calibrationButton_offsets_clicked() {
    if(isPressureEnough()){
        ui.tab_manager->setCurrentIndex(4);
        qnode.sendTask(pap_common::PLACER, pap_common::GOTO,
                       currentPosition.x, currentPosition.y, 45.0);
        QMessageBox msgBox;
        msgBox.setWindowTitle("Preparing offset calibration");
        msgBox.setText(QString("Insert desired nozzles and confirm to start offset calibration."));
        msgBox.setStandardButtons(QMessageBox::Yes);
        msgBox.addButton(QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);
        if (msgBox.exec() == QMessageBox::Yes) {
            updateCurrentNozzles();
            qnode.sendTask(pap_common::PLACER, pap_common::CALIBRATION_OFFSET, leftTipRadius, rightTipRadius, 0.0);
        }
    }
}

bool MainWindow::isPressureEnough(){
    if(pressure_ > MIN_PRESSURE){
        return true;
    }else{
        QMessageBox msgBox;
        const QString title = "Pressure warning";
        msgBox.setWindowTitle(title);
        msgBox.setText("Pressure is too low");
        msgBox.exec();
        msgBox.close();
        return false;
    }
}

void MainWindow::on_calibrationButton_ratios_clicked() {
    ui.tab_manager->setCurrentIndex(4);
    if(isPressureEnough()){
        qnode.sendTask(pap_common::PLACER, pap_common::GOTO,
                       currentPosition.x, currentPosition.y, 30.0);
        QMessageBox msgBox;
        msgBox.setWindowTitle("Preparing ratio calibration");
        msgBox.setText(QString("Insert a nozzle (2.5 mm diameter) into tip1 and confirm to start ratio calibration."));
        msgBox.setStandardButtons(QMessageBox::Yes);
        msgBox.addButton(QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);
        if (msgBox.exec() == QMessageBox::Yes) {
            updateCurrentNozzles();
            qnode.sendTask(pap_common::PLACER, pap_common::CALIBRATION_RATIO);
        }
    }
}

void MainWindow::on_calcOrientation_Button_clicked() {
    QPointF local1, global1, local2, global2;

    float xCamera = 0.0;
    float yCamera = 0.0;
    if (qnode.fakePadPos_) {
        ROS_INFO("GUI: Simulation active: I will fake the pad positions...");
        xCamera = 180.0;
        yCamera = 140.0;

        pressure_ = 6;

        local1.setX(xCamera);
        local1.setY(yCamera);
        local2.setX(xCamera);
        local2.setY(yCamera);

        global1.setX(0);
        global1.setY(0);
        global2.setX(0);
        global2.setY(0);
    } else {

        for(size_t row = 0; row < ui.fiducialTable->rowCount(); row++){
            for(size_t col = 0; col < ui.fiducialTable->columnCount(); col++){
                QTableWidgetItem* item = ui.fiducialTable->item(row,col);
                if (!item || item->text().isEmpty())
                {
                    QMessageBox msgBox;
                    const QString title = "Calibration failed";
                    msgBox.setWindowTitle(title);
                    msgBox.setText("Calibration table not complete");
                    msgBox.exec();
                    msgBox.close();
                    return;
                }
            }
        }

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

    qnode.sendPcbImage(padParser.getMarkerList());
    PCBTransformCalibrated_ = true;
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
        //ROS_INFO("GUI: Pressed down xmotor");
        break;
    case Qt::Key_W:
        qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
                       (float) pap_common::XMOTOR, (float) pap_common::BACKWARD, 0.0);
        //ROS_INFO("GUI: Pressed up xmotor");
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
    float camera_select = 0;
    if(ui.camera_select_tip->currentText() == "Bottom"){
        camera_select = 1;
    }
    qnode.sendTask(pap_common::VISION, pap_vision::SEARCH_CIRCLE,
                   ui.radius_edit->text().toFloat(), (float)tip_thresholding_on, camera_select );
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
    dispensed_ids_.clear();
    redrawPadView();
    QMessageBox msgBox;
    const QString title = "Dispensing information";
    msgBox.setWindowTitle(title);
    msgBox.setText("Dispensing reseted");
    msgBox.exec();
    msgBox.close();
}

void MainWindow::on_startDispense_button_clicked() {
    float pxFactor = padParser.pixelConversionFactor;
    float nozzleDiameter = nozzle_diameter_;

    if(!isPressureEnough()) return;

    if(dispenserPaused){
        dispenserPaused = false;
    }

    size_t initialIter = 0;

    std::vector<PadInformation> copy;
    copy = padParser.padInformationArrayPrint_;

    std::sort(copy.begin(), copy.end(), compareClass());

    bool left_align = true;

    for (size_t i = initialIter; i < copy.size(); i++) {

        if (dispenserPaused) {
            qnode.sendTask(pap_common::PLACER, pap_common::GOTO,
                           currentPosition.x, currentPosition.y, 45.0);
            return;
        }

        if(dispensed_ids_.find(copy.at(i).id) != dispensed_ids_.end()){
            continue;
        }

        if(!DispenserPlanner::isPadCompatibleToNozzle(copy.at(i), nozzle_diameter_, edge_percentage_)){
            continue;
        }

        std::vector<dispenseInfo> dispInfo;
        if(planner_selection_ == DispenserPlanner::PLANNER_SELECT::DOT_PLANNER){
            enum DispenserPlanner::DOT_ALIGN align_temp;
            //if(left_align){
            align_temp = DispenserPlanner::DOT_ALIGN::CENTER;
            //    left_align = false;
            //}else{
            //    align_temp = DispenserPlanner::DOT_ALIGN::RIGHT;
            //    left_align = true;
            //}

            dispInfo = dotPlanner.planDispensing(
                        copy[i], nozzleDiameter, edge_percentage_, wait_time_, alpha_, align_temp);
        }else if(planner_selection_ == DispenserPlanner::PLANNER_SELECT::LINE_PLANNER){
            dispInfo = dispenserPlanner.planDispensing(
                        copy[i], nozzleDiameter, edge_percentage_, dispenser_velocity_, wait_time_);
        }

        //copy.erase(copy.begin());

        //std::sort(copy.begin(), copy.end(), compareClass(currentPosition.x,currentPosition.y));

        for (size_t j = 0; j < dispInfo.size(); j++) {
            if(dispInfo.at(j).type == dispenser_types::DOT_DISPENSE){

                double radius = nozzleDiameter * pxFactor;
                double x = dispInfo[j].yPos * pxFactor - radius/2 ;
                double y = - (dispInfo[j].xPos * pxFactor) - radius/2;
                scenePads_.addEllipse(x, y, radius, radius,QPen(Qt::blue, 0, Qt::SolidLine), QBrush(Qt::blue) );

            }else if(dispInfo.at(j).type == dispenser_types::LINE_DISPENSE){
                scenePads_.addLine(
                            QLineF(dispInfo[j].yPos * pxFactor,
                                   - (dispInfo[j].xPos * pxFactor),
                                   dispInfo[j].yPos2 * pxFactor,
                                   -(dispInfo[j].xPos2 * pxFactor)),
                            QPen(Qt::blue, nozzleDiameter * pxFactor, Qt::SolidLine));

            }
            else{
                scenePads_.addLine(
                            QLineF(dispInfo[j].yPos * pxFactor,
                                   - (dispInfo[j].xPos * pxFactor),
                                   dispInfo[j].yPos2 * pxFactor,
                                   -(dispInfo[j].xPos2 * pxFactor)),
                            QPen(Qt::gray, nozzleDiameter * pxFactor, Qt::SolidLine));
            }

            padParser.transformDispenserInfo(&dispInfo[j]);

        }

        if(i == 0 || i == initialIter){
            std::cerr << "X: " << dispInfo.front().xPos << " Y: " << dispInfo.front().xPos << std::endl;
            if(!driveToCoord(dispInfo.front().xPos  -52 , dispInfo.front().yPos +39, 45)){
                std::cerr << "Could not drive to initial dispense pos...\n";
                continue;
            }
        }

        qnode.sendDispenserTask(dispInfo, dispenser_height_offset_);

        QEventLoop loop;
        QTimer *timer = new QTimer(this);

        connect(&qnode, SIGNAL(dispenserFinished()), &loop, SLOT(quit()));
        connect(timer, SIGNAL(timeout()), &loop, SLOT(quit()));
        timer->setSingleShot(true);
        timer->start(60000);

        loop.exec(); //blocks untill either signalPosition or timeout was fired

        // Is timeout ocurred?
        if (!timer->isActive()) {
            std::cerr << "Dispensing timeout...\n";
            return;
        }

        dispensed_ids_.insert(copy.at(i).id);

        scenePads_.addEllipse((copy[i].rect.y()) * pxFactor - 1.0,
                              -(copy[i].rect.x() * pxFactor), 1, 1,
                              QPen(Qt::green, 2, Qt::SolidLine));

    }
    processAllCallbacks();
    ROS_INFO("GUI: Dispensing finished....");
    qnode.sendTask(pap_common::PLACER, pap_common::GOTO, currentPosition.x,
                   currentPosition.y, 45.0);
}

void MainWindow::dispenseSinglePad(QPointF point) {
    id_ = padParser.searchId(point);
    float nozzleDiameter = nozzle_diameter_;
    float pxFactor = padParser.pixelConversionFactor;

    if(!isPressureEnough()) return;
    if (id_ != -1) {

        if(!DispenserPlanner::isPadCompatibleToNozzle(padParser.padInformationArrayPrint_[id_], nozzle_diameter_, edge_percentage_)){
            return;
        }

        std::vector<dispenseInfo> dispInfo;
        if(planner_selection_ == DispenserPlanner::PLANNER_SELECT::DOT_PLANNER){
            dispInfo = dotPlanner.planDispensing(
                        padParser.padInformationArrayPrint_[id_], nozzleDiameter, edge_percentage_, wait_time_, alpha_, alignment_);
        }else if(planner_selection_ == DispenserPlanner::PLANNER_SELECT::LINE_PLANNER){
            dispInfo = dispenserPlanner.planDispensing(
                        padParser.padInformationArrayPrint_[id_], nozzleDiameter, edge_percentage_, dispenser_velocity_, wait_time_);
        }

        for (size_t j = 0; j < dispInfo.size(); j++) {
            if(dispInfo.at(j).type == dispenser_types::DOT_DISPENSE){

                double diameter = nozzleDiameter * pxFactor;

                double x = dispInfo[j].yPos * pxFactor - diameter/2 ;
                double y = - (dispInfo[j].xPos * pxFactor) - diameter/2;
                scenePads_.addEllipse(x, y, diameter, diameter,QPen(Qt::blue, 0, Qt::SolidLine), QBrush(Qt::blue) );

            }else if(dispInfo.at(j).type == dispenser_types::LINE_DISPENSE){
                scenePads_.addLine(
                            QLineF(dispInfo[j].yPos * pxFactor,
                                   - (dispInfo[j].xPos * pxFactor),
                                   dispInfo[j].yPos2 * pxFactor,
                                   -(dispInfo[j].xPos2 * pxFactor)),
                            QPen(Qt::blue, nozzleDiameter * pxFactor, Qt::SolidLine));

            }
            else{
                scenePads_.addLine(
                            QLineF(dispInfo[j].yPos * pxFactor,
                                   - (dispInfo[j].xPos * pxFactor),
                                   dispInfo[j].yPos2 * pxFactor,
                                   -(dispInfo[j].xPos2 * pxFactor)),
                            QPen(Qt::gray, nozzleDiameter * pxFactor, Qt::SolidLine));
            }

            padParser.transformDispenserInfo(&dispInfo[j]);
            std::cerr << "Point in PAP: " << dispInfo[j].xPos << " / " << dispInfo[j].yPos << std::endl;

        }

        dispensed_ids_.insert(padParser.padInformationArrayPrint_[id_].id);

        if(!driveToCoord(dispInfo.front().xPos  -52 , dispInfo.front().yPos +39, 45)){
            std::cerr << "Could not drive to initial dispense pos...\n";
            return;
        }

        qnode.sendDispenserTask(dispInfo, dispenser_height_offset_);

        QEventLoop loop;
        QTimer *timer = new QTimer(this);

        connect(&qnode, SIGNAL(dispenserFinished()), &loop, SLOT(quit()));
        connect(timer, SIGNAL(timeout()), &loop, SLOT(quit()));
        timer->setSingleShot(true);
        timer->start(60000);

        loop.exec(); //blocks untill either signalPosition or timeout was fired

        processAllCallbacks();
        ROS_INFO("GUI: Dispensing finished....");
        qnode.sendTask(pap_common::PLACER, pap_common::GOTO, currentPosition.x,
                       currentPosition.y, 45.0);

        // Is timeout ocurred?
        if (!timer->isActive()) {
            return;
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

bool MainWindow::startTapePartSelector(int numOfTape){
    TapeCalibrationDialog tape_dialog(qnode, *tape_calibrater_, numOfTape );
    connect(&qnode, SIGNAL(cameraUpdated(int)), &tape_dialog, SLOT(cameraUpdated(int)));
    tape_dialog.exec();

    int pos = tape_dialog.getTapePos();

    if(pos != -1){
        tapeCompCounter[numOfTape] = pos;
        return true;
    }else{
        return false;
    }
}

void MainWindow::on_calibrateTapeButton_clicked(void) {
    QMessageBox msgBox;
    QString message =
            QString("You are about to start a tape calibration process. Please make sure all tapes are placed correctly.");
    msgBox.setWindowTitle("Confirm to start calibration");
    msgBox.setText(message);
    msgBox.setStandardButtons(QMessageBox::Yes);
    msgBox.addButton(QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);
    if (msgBox.exec() == QMessageBox::Yes) {
        QVector<int> calibratedTapes;
        for (size_t i = 0; i < componentList.size(); i++) {
            if ((componentList.at(i).box >= 67)
                    && (componentList.at(i).box <= 86)) {
                int tape_nr = componentList.at(i).box - 67;
                if (calibratedTapes.indexOf(tape_nr) == -1) {
                    calibratedTapes.append(tape_nr);
                    if(tape_calibrater_->calibrateTape(tape_nr, componentList.at(i).width,
                                                       componentList.at(i).length)){
                        startTapePartSelector(tape_nr);
                        ROS_INFO("GUI: Calibrated tape: %d", componentList.at(i).box);
                    }
                    ROS_INFO("GUI: Tape nr: %d, Width: %f Height: %f", tape_nr,
                             componentList.at(i).width, componentList.at(i).length);
                }
            }
        }
        tapeCalibrated_ = true;
    }
}

void MainWindow::on_printButton_offsets_clicked() {
    qnode.sendTask(pap_common::PLACER, pap_common::PRINT_OFFSET);
}

MainWindow::~MainWindow() {
}

}
// namespace pap_gui

void pap_gui::MainWindow::on_calibrateTopCamButton_clicked() {
    QMessageBox msgBox;
    msgBox.setWindowTitle("Confirm to start calibration");
    msgBox.setText(QString("You are about to calibrate the top camera. Make sure 'calibTopCamera.launch'' has been started."));
    msgBox.setStandardButtons(QMessageBox::Yes);
    msgBox.addButton(QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::Yes);
    if (msgBox.exec() == QMessageBox::Yes) {
        qnode.sendTask(pap_common::PLACER, pap_common::CALIBRATION_TOPCAM);
    }
}

void pap_gui::MainWindow::on_calibrateBottomCamButton_clicked()
{
    QMessageBox msgBox;
    msgBox.setWindowTitle("Confirm to start calibration");
    msgBox.setText(QString("You are about to calibrate the bottom camera. Make sure 'calibBottomCamera.launch has been started."));
    msgBox.setStandardButtons(QMessageBox::Yes);
    msgBox.addButton(QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::Yes);
    if (msgBox.exec() == QMessageBox::Yes) {
        qnode.sendTask(pap_common::PLACER, pap_common::CALIBRATION_BOTTOMCAM);
    }
}


void pap_gui::MainWindow::on_take_img_button_clicked()
{
    pap_common::VisionResult res;
    if(vision_send_functions::sendVisionTask(qnode.getVisionClientRef(), pap_vision::FEED_STITCH_PIC, pap_vision::CAMERA_TOP, currentPosition.x, currentPosition.y, currentPosition.z ,res)){
        QMessageBox msgBox;
        msgBox.setText("Image was appended");
        msgBox.exec();
    }
}

void pap_gui::MainWindow::on_pushButton_2_clicked()
{
    if(vision_send_functions::sendVisionTask(qnode.getVisionClientRef(), pap_vision::STITCH_PICTURES)){
        QMessageBox msgBox;
        msgBox.setText("Images were saved");
        msgBox.exec();
    }
}

void pap_gui::MainWindow::on_pushButton_clicked()
{
    qnode.sendTask(pap_common::VISION, pap_vision::START__QRCODE_FINDER, (float)pap_vision::VISION_QR_CALIBRATION::BOTTOM_CAM, (float)pap_vision::CAMERA_SELECT::CAMERA_BOTTOM, 0.0);
}

void pap_gui::MainWindow::on_pushButton_startQRTop_clicked()
{
    qnode.sendTask(pap_common::VISION, pap_vision::START__QRCODE_FINDER, (float)pap_vision::VISION_QR_CALIBRATION::TOP_SLOT, (float)pap_vision::CAMERA_SELECT::CAMERA_TOP, 0.0);
}

void pap_gui::MainWindow::on_calibrateSystemButton_clicked()
{
    QMessageBox msgBox;
    QString message = QString("Please make sure a large nozzle is used for this calibration process.");
    msgBox.setWindowTitle("Confirm to start calibration");
    msgBox.setText(message);
    msgBox.setStandardButtons(QMessageBox::Yes);
    msgBox.addButton(QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);
    if (msgBox.exec() == QMessageBox::Yes) {
        completeCalibrationRunning = true;
        qnode.sendTask(pap_common::PLACER, pap_common::CALIBRATION_RATIO);
        // TODO: Disable other functions/buttons
        // TODO: Show calibrationIndicator!
    }
}

void pap_gui::MainWindow::processAllCallbacks(){
    for(size_t i = 0; i < 1000; i++){
        ros::spinOnce();
    }
}

void pap_gui::MainWindow::createNewPad(QRectF pad){
    padParser.createPadFromView(pad);
    redrawPadView();
}

void pap_gui::MainWindow::on_scanButton_clicked()
{
    if(qnode.pcbHeight_ == 0 || qnode.pcbWidth_ == 0) return;

    // Lower left corner of pcb holder
    const QVector3D init(311.204, 153.019, 25.6);
    const QVector2D pcb_size(qnode.pcbHeight_, qnode.pcbWidth_);

    // Calculate waypoints for gathering all images for the stitching process
    std::vector<QVector3D> waypoints = stitch_waypoint_maker::generateWaypoints(init, 50, pcb_size, 31 , 31, 25.6);

    qnode.LEDTask(pap_common::SETTOPLED, 0);
    // Drive along waypoints
    for(size_t i = 0; i < waypoints.size(); i++){
        std::cerr << "Drive to waypoint... " << i << std::endl;
        QVector3D &p = waypoints.at(i);
        if(!motor_send_functions::sendMotorControllerAction(qnode.getMotorClientRef(), pap_common::COORD, p.x(), p.y(), p.z() )){
            std::cerr << "Failed to send current command...\n";
            return;
        }

        // This is needed to update the positions via the callbacks
        processAllCallbacks();

        ros::Duration(0.3);

        // Make picture and align it with the previous taken picture
        pap_common::VisionResult res;
        if(!vision_send_functions::sendVisionTask(qnode.getVisionClientRef(), pap_vision::FEED_STITCH_PIC, pap_vision::CAMERA_TOP, currentPosition.x, currentPosition.y, currentPosition.z ,res)){
            std::cerr << "Appending picture failed\n ";
            return;
        }
    }

    qnode.LEDTask(pap_common::RESETTOPLED, 0);
    // Start stitching process
    pap_common::VisionResult res;
    if(vision_send_functions::sendVisionTask(qnode.getVisionClientRef(), pap_vision::STITCH_PICTURES,  pap_vision::CAMERA_TOP,0,0,0,res,1)){

        padParser.reset();
        dispensed_ids_.clear();
        padFileLoaded_ = false;

        QMessageBox msgBox;
        msgBox.setText("Images were stitched");
        msgBox.exec();

        // Create opencv image out of ros image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(res.mats.front(), sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat outputRGB;
        cvtColor(cv_ptr->image, outputRGB, CV_BGR2RGB);

        // Set background image
        QImage stitchedImage= QImage((uchar*) outputRGB.data, outputRGB.cols, outputRGB.rows, outputRGB.step, QImage::Format_RGB888);
        padParser.deleteBackground();
        padParser.setBackGround(stitchedImage.copy());

        double px_factor_x = res.data3;
        double px_factor_y = res.data4;

        // Search pads on stitched image
        padFinder finder;
        finder.pxRatioPcb_x = px_factor_x;
        finder.pxRatioPcb_y = px_factor_y;
        std::vector<cv::RotatedRect> pads;
        finder.findPads(&(cv_ptr->image),false, cv::Point2f(0,0),pads);

        cv::imshow("Pads found", (cv_ptr->image));
        cv::waitKey(0);

        // Convert opencv rectangle calculations in Padinformations in the robot coordinate system
        for(size_t i = 0; i < pads.size(); i++ ){
            PadInformation pad;
            pad.rect.setX(-pads.at(i).center.y );
            pad.rect.setY(-pads.at(i).center.x );
            pad.rect.setWidth(pads.at(i).size.width);
            pad.rect.setHeight(pads.at(i).size.height);
            pad.dispensed = false;

            if(pads.at(i).angle < 0 ){
                pads.at(i).angle = 180 + std::fabs(pads.at(i).angle);
            }

            pads.at(i).angle *= -1;

            pad.rotation = pads.at(i).angle;
            pad.shapeStr = "rectangle";

            pad.id = i;

            padParser.padInformationArray_.push_back(pad);
            padParser.padInformationArrayPrint_.push_back(pad);
        }

        // Create transform
        const double x_fov = 480 / px_factor_y;
        const double y_fov = 640 / px_factor_x;

        tf::Transform tf;
        tf.setOrigin(tf::Vector3(res.data2 + x_fov / 2.0, res.data1 + y_fov / 2.0, 0.0));
        tf.setRotation(tf::Quaternion(0, 0, 0, 1));

        padParser.setTransformation(tf);
        padParser.rotatePads();

        // Render visualization and set up table

        //padParser.setTable(ui.padTable);
        padFileLoaded_ = true;

        pic_offset_.width = -cv_ptr->image.size().width;
        pic_offset_.height = 0;

        padParser.pixelConversionFactor = px_factor_x;
        padParser.renderImage(&scenePads_, pic_offset_.width, pic_offset_.height, dispensed_ids_);

        qnode.sendPcbImage(padParser.getMarkerList());
    }
}


void pap_gui::MainWindow::on_disp_settings_apply_clicked()
{
    nozzle_diameter_ =  ui.nozzleDispCombo->currentText().toFloat();
    edge_percentage_ = ui.edge_perc_edit->text().toFloat();
    dispenser_velocity_ = ui.dispenser_vel_edit->text().toFloat();
    alpha_ = ui.alpha_text_edit->text().toFloat();
    dispenser_height_offset_ = ui.disp_height_offset_edit->text().toFloat();

    if(ui.planner_select_combo->currentText() == "Line"){
        planner_selection_ = DispenserPlanner::PLANNER_SELECT::LINE_PLANNER;
    }else if(ui.planner_select_combo->currentText() == "Dot"){
        planner_selection_ = DispenserPlanner::PLANNER_SELECT::DOT_PLANNER;
    }

    wait_time_ = ui.wait_time_text_edit->text().toFloat();

    if(ui.align_select_combo->currentText() == "Center"){
        alignment_ = DispenserPlanner::DOT_ALIGN::CENTER;
    }else if(ui.align_select_combo->currentText() == "Left"){
        alignment_ = DispenserPlanner::DOT_ALIGN::LEFT;
    }else if(ui.align_select_combo->currentText() == "Right"){
        alignment_ = DispenserPlanner::DOT_ALIGN::RIGHT;
    }

    padParser.setDispenserInfo(nozzle_diameter_, edge_percentage_);
    redrawPadView();
    //qnode.sendTask(pap_common::PLACER, pap_common::ADJUST_DISPENSER,
    //               nozzle_diameter_, dispenser_velocity_, 0);

}


void pap_gui::MainWindow::on_calibrate_dispenser_button_clicked()
{
    qnode.sendTask(pap_common::PLACER, pap_common::CALIBRATE_DISPENSER, nozzle_diameter_, 0, 0);
}

void pap_gui::MainWindow::on_radioButton_clicked(bool checked)
{
    tip_thresholding_on = checked;
}

bool pap_gui::MainWindow::driveToCoord(const double &x, const double &y, const double &z, const double moving_height){
    processAllCallbacks();

    if(!motor_send_functions::sendMotorControllerAction(qnode.getMotorClientRef(), pap_common::COORD,
                                                        currentPosition.x,
                                                        currentPosition.y,
                                                        45)){
        return false;
    }

    if(!motor_send_functions::sendMotorControllerAction(qnode.getMotorClientRef(), pap_common::COORD,
                                                        x,
                                                        y,
                                                        45)){
        return false;
    }

    if(!motor_send_functions::sendMotorControllerAction(qnode.getMotorClientRef(), pap_common::COORD,
                                                        x,
                                                        y,
                                                        z)){
        return false;
    }

    return true;
}


void pap_gui::MainWindow::on_pushButton_recalibrate_left_tip_clicked()
{
    if(isPressureEnough()){
        ui.tab_manager->setCurrentIndex(4);
        qnode.sendTask(pap_common::PLACER, pap_common::GOTO,
                       currentPosition.x, currentPosition.y, 45.0);
        QMessageBox msgBox;
        msgBox.setWindowTitle("Preparing left tip offset recalibration");
        msgBox.setText(QString("Insert desired nozzles and confirm to start recalibration."));
        msgBox.setStandardButtons(QMessageBox::Yes);
        msgBox.addButton(QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);
        if (msgBox.exec() == QMessageBox::Yes) {
            updateCurrentNozzles();
            qnode.sendTask(pap_common::PLACER, pap_common::RECALIBRATE_LEFT_TIP, leftTipRadius, rightTipRadius, 0.0);
        }
    }
}

void pap_gui::MainWindow::on_pushButton_recalibrate_right_tip_clicked()
{
    if(isPressureEnough()){
        ui.tab_manager->setCurrentIndex(4);
        qnode.sendTask(pap_common::PLACER, pap_common::GOTO,
                       currentPosition.x, currentPosition.y, 45.0);
        QMessageBox msgBox;
        msgBox.setWindowTitle("Preparing right tip offset recalibration");
        msgBox.setText(QString("Insert desired nozzles and confirm to start recalibration."));
        msgBox.setStandardButtons(QMessageBox::Yes);
        msgBox.addButton(QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);
        if (msgBox.exec() == QMessageBox::Yes) {
            updateCurrentNozzles();
            qnode.sendTask(pap_common::PLACER, pap_common::RECALIBRATE_RIGHT_TIP, leftTipRadius, rightTipRadius, 0.0);
        }
    }
}

void pap_gui::MainWindow::on_increaseCamOffsetX_pushButton_clicked()
{
    // Increase offset by 10 µm
    qnode.sendTask(pap_common::PLACER, pap_common::INCR_CAM_PROJECTION_OFFSET_X);

    // Update label
    //double offset =
    //ui.camOffsetX_label->setText();
}

void pap_gui::MainWindow::on_decreaseCamOffsetX_pushButton_clicked()
{
    //
    qnode.sendTask(pap_common::PLACER, pap_common::DECR_CAM_PROJECTION_OFFSET_X);
}


void pap_gui::MainWindow::on_increaseCamOffsetY_pushButton_clicked()
{
    qnode.sendTask(pap_common::PLACER, pap_common::INCR_CAM_PROJECTION_OFFSET_Y);
}

void pap_gui::MainWindow::on_decreaseCamOffsetY_pushButton_clicked()
{
    qnode.sendTask(pap_common::PLACER, pap_common::DECR_CAM_PROJECTION_OFFSET_Y);
}

void pap_gui::MainWindow::on_startCamProjectionCalibration_pushButton_clicked()
{
    qnode.sendTask(pap_common::PLACER, pap_common::START_CAM_PROJECTION_CALIBRATION);

}
