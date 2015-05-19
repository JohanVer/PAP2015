/**
 * @file /include/pap_gui/main_window.hpp
 *
 * @brief Qt based gui for pap_gui.
 *
 * @date November 2010
 **/
#ifndef pap_gui_MAIN_WINDOW_H
#define pap_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "pap_common/Task.h"


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace pap_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_startHoming_clicked(bool check);
	void on_switchCurrent_clicked(bool check);
	void on_gotoCoord_clicked(bool check);
	void on_xManPos_clicked(bool check);
	void on_xManNeg_clicked(bool check);
	void on_YManPos_clicked(bool check);
	void on_YManNeg_clicked(bool check);
	void on_ZManPos_clicked(bool check);
	void on_ZManNeg_clicked(bool check);
	void on_connectButton_clicked(bool check);
	void on_checkbox_use_environment_stateChanged(int state);
	void cameraUpdated(int index);
	void statusUpdated(int index);
	void on_valve1Button_clicked(bool check);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	QPixmap cameraPicture1;
	QGraphicsScene scene_;
};

}  // namespace pap_gui

#endif // pap_gui_MAIN_WINDOW_H
