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

//#include "cv.h"
//#include "highgui.h"

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <QStandardItemModel>
#include <QVector>
#include <QString>


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
bool placementProcessRunning = false;
int componentCount = 0;
int boxNumberMax = 50;
int boxNumberMin = 1;
int boxNumberSug = 1;

struct componentEntry {
    string name, package, side, value;
    float posX, posY;
    int rotation;
    int box;
};

QVector<componentEntry> componentVector;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
		QMainWindow(parent), qnode(argc, argv) {
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
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

	// Cameras
	QObject::connect(&qnode, SIGNAL(cameraUpdated(int )), this,
			SLOT(cameraUpdated(int )));

	// Status
	QObject::connect(&qnode, SIGNAL(statusUpdated(int)), this,
			SLOT(statusUpdated(int)));

    // Show Ros status dockwidget
    ui.dock_status->show();

	valve1Active_ = false;


}

MainWindow::~MainWindow() {
}

/********************************************************************"*********
 ** Implementation [Slots]
 *****************************************************************************/

void MainWindow::on_setCompBoxNrButton_clicked() {

    // Check if table empty
    if(!componentTableEmpty) {

    	/* Get current component */
        int currentComponent = ui.tableWidget->currentRow();

        /* If no component selected */
        if(currentComponent == -1) {

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
            if(currentBox == -1) {
            	currentBox = boxNumberSug;
            }

            //int boxNumber = inputDialog->getInt(this, "enter number", "enter number", currentBox, boxNumberMax, boxNumberMin, boxNumberStep, &ok, 0);
            int boxNumber = inputDialog->getInt(this, "enter number", "enter number", currentBox);

            if(boxNumber <= boxNumberMax && boxNumber >= boxNumberMin) {
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

    /* If no table empty */
    } else {
        QMessageBox msgBox;
        msgBox.setText("Component table empty. Please select gerber file first.");
        msgBox.exec();
        msgBox.close();
    }
}

void MainWindow::on_compDeleteButton_clicked() {

	/* Get current component */
    int currentComp = ui.tableWidget->currentRow();

    if(currentComp == -1) {
        QMessageBox msgBox;
        msgBox.setText("No component selected.");
        msgBox.exec();
        msgBox.close();

    } else {
        /* Delete component from vector */
    	componentVector.remove(currentComp);

    	/* Update component table */
    	updateComponentTable();
    }
}


void MainWindow::on_tableWidget_clicked() {

    int currentComp = ui.tableWidget->currentRow();

    // Update component information based on componentVector and currentComponent
    ui.label_compName->setText(componentVector.at(currentComp).name.c_str());
    ui.label_compPackage->setText(componentVector.at(currentComp).package.c_str());
    ui.label_compPos->setText(QString::number(componentVector.at(currentComp).posX) + " / " + QString::number(componentVector.at(currentComp).posY));
    ui.label_compOrient->setText(QString::number(componentVector.at(currentComp).rotation));
    ui.label_compSide->setText(componentVector.at(currentComp).side.c_str());

    if(componentVector.at(currentComp).box == -1) {
    	ui.label_compBox->setText("unknown");
    } else {
    	ui.label_compBox->setText(QString::number(componentVector.at(currentComp).box));
    }
}

void MainWindow::on_clearTableButton_clicked() {

	if(componentTableEmpty){
        QMessageBox msgBox;
        msgBox.setText("No content to delete.");
        msgBox.exec();
        msgBox.close();

	} else {
		while(!componentVector.isEmpty()) {
			componentVector.remove(0);
		}
		updateComponentTable();
	}
}


void MainWindow::on_compOrientButton_clicked() {



}





void MainWindow::on_loadGerberFileButton_clicked() {

    /* Clear component vector and reset component number*/
	componentVector.clear();
	componentCount = 0;

	/* Load gerber file and add new components to vector */
    std::fstream datafile;
    std::string value, package, posX, posY, rotation, side, name;

    datafile.open ("/home/nikolas/catkin_ws/src/PAP2015/pap_gui/src/example.txt", std::fstream::in | std::fstream::out | std::fstream::app);

    /* ok, proceed  */
    if (datafile.is_open()) {

        std::cout << "File opened!" << std::endl;

        string componentString;
        while(getline(datafile, componentString)) {

            /* Filter only component data */
            if(!(componentString.at(0) == (char) 42)){

            	componentEntry newComponent;

                int pos1 = componentString.find('"');
                componentString = componentString.substr(pos1+1, componentString.size()-pos1);

                /* Be careful - neglecting one position!!! */
                pos1 = componentString.find('//');
                newComponent.value = componentString.substr(0,pos1);
                componentString = componentString.substr(pos1+2, componentString.size()-pos1);

                pos1 = componentString.find('"');
                newComponent.package = componentString.substr(0,pos1);
                componentString = componentString.substr(pos1+3, componentString.size()-pos1);

                pos1 = componentString.find('"');
                newComponent.posX = atof((componentString.substr(0,pos1)).c_str());
                componentString = componentString.substr(pos1+3, componentString.size()-pos1);

                pos1 = componentString.find('"');
                newComponent.posY = atof((componentString.substr(0,pos1)).c_str());
                componentString = componentString.substr(pos1+3, componentString.size()-pos1);

                pos1 = componentString.find('"');
                newComponent.rotation = atoi((componentString.substr(0,pos1)).c_str());
                componentString = componentString.substr(pos1+2, componentString.size()-pos1);

                pos1 = componentString.find(',');
                newComponent.side = componentString.substr(0,pos1);
                componentString = componentString.substr(pos1+2, componentString.size()-pos1);

                pos1 = componentString.find('"');
                newComponent.name = componentString.substr(0,pos1);

                // Set box number of component
                newComponent.box = -1;

                //std::cout << newComponent.value << " - " << newComponent.package << " - " << to_string(newComponent.posX) << " - " << to_string(newComponent.posY) << " - " << newComponent.rotation << " - " << newComponent.side << " - " << newComponent.name << std::endl;

                componentVector.append(newComponent);
                componentCount++;

            }


        }

        std::cout << "Number of components: " << componentCount << std::endl;
        std::cout << "Size of vector: " << componentVector.size() << std::endl;

    } else {
        std::cout << "Could not open file!" << std::endl;
    }

    datafile.close();

    if(componentCount != 0) {
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
	    for(int i = 1; i < componentVector.size(); i++) {
	    	vLabels << QString::number(i);
	    }
	    ui.tableWidget->setHorizontalHeaderLabels(hLabels);
	    ui.tableWidget->setVerticalHeaderLabels(vLabels);

	    // Set content
	    for(int i = 0; i < ui.tableWidget->rowCount(); i++) {

	        ui.tableWidget->setItem(i,0, new QTableWidgetItem((componentVector.at(i).name).c_str()));
	        ui.tableWidget->setItem(i,1, new QTableWidgetItem((componentVector.at(i).value).c_str()));
	        ui.tableWidget->setItem(i,2, new QTableWidgetItem((componentVector.at(i).package).c_str()));

	    }

	    // Table settings
	    ui.tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
	    ui.tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
	    //ui.tableWidget->verticalHeader()->setVisible(false);
	    ui.tableWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	    ui.tableWidget->horizontalHeader()->setResizeMode(QHeaderView::Stretch);

	    //ui.tableWidget->setWindowTitle("QTableWidget");
	    ui.tableWidget->show();

	    // Set number of components
	    ui.label_compTotal->setText(QString::number(componentVector.size()));
	    ui.label_compLeft->setText(QString::number(componentVector.size()));
}


void MainWindow::on_startPlacementButton_clicked() {

    if (componentTableEmpty) {
        QMessageBox msgBox;
        msgBox.setText("Component table empty. Please select gerber file first.");
        msgBox.exec();
        msgBox.close();
    }
}


void MainWindow::on_pausePlacementButton_clicked() {

    if (!placementProcessRunning) {
        QMessageBox msgBox;
        msgBox.setText("Component table empty. Please select Gerber file first.");
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

/*****************************************************************************
 ** Implemenation [Slots][manually connected]
 *****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
	ui.view_logging->scrollToBottom();
}

/*****************************************************************************
 ** Implementation [Menu]
 *****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
	QMessageBox::about(this, tr("About ..."),
			tr(
					"<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
 ** Implementation [Configuration]
 *****************************************************************************/

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

void MainWindow::on_xManPos_clicked(bool check) {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
			(float) pap_common::XMOTOR, (float) pap_common::FORWARD, 0.0);
}

void MainWindow::on_xManNeg_clicked(bool check) {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
			(float) pap_common::XMOTOR, (float) pap_common::BACKWARD, 0.0);
}

void MainWindow::on_YManPos_clicked(bool check) {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
			(float) pap_common::YMOTOR, (float) pap_common::FORWARD, 0.0);
}

void MainWindow::on_YManNeg_clicked(bool check) {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
			(float) pap_common::YMOTOR, (float) pap_common::BACKWARD, 0.0);
}

void MainWindow::on_ZManPos_clicked(bool check) {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
			(float) pap_common::ZMOTOR, (float) pap_common::FORWARD, 0.0);
}

void MainWindow::on_ZManNeg_clicked(bool check) {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::MANUAL,
			(float) pap_common::ZMOTOR, (float) pap_common::BACKWARD, 0.0);
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

void MainWindow::on_connectButton_clicked(bool check) {
	qnode.sendTask(pap_common::CONTROLLER, pap_common::CONNECT);
}

void MainWindow::on_valveToggle1_clicked(bool check) {
	if (!valve1Active_) {
		qnode.sendRelaisTask(1, true);
		ui.valveStatus1->setText("On");
		valve1Active_ = true;
	} else {
		qnode.sendRelaisTask(1, false);
		ui.valveStatus1->setText("Off");
		valve1Active_ = false;
	}
}

}
// namespace pap_gui

