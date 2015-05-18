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

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace pap_gui {

using namespace Qt;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

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

}

MainWindow::~MainWindow() {
}

/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/

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

}
// namespace pap_gui

