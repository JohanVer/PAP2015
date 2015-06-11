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
	void on_xManPos_pressed();
	void on_xManNeg_pressed();
	void on_YManPos_pressed();
	void on_YManNeg_pressed();
	void on_ZManPos_pressed();
	void on_ZManNeg_pressed();
	void on_connectButton_clicked(bool check);
	void on_checkbox_use_environment_stateChanged(int state);
	void cameraUpdated(int index);
	void statusUpdated(int index);
	void on_valveToggle1_clicked(bool check);
	void on_valveToggle2_clicked(bool check);
	void on_valveToggle3_clicked(bool check);
	void on_valveToggle4_clicked(bool check);
	void on_valveToggle5_clicked(bool check);
	void on_valveToggle6_clicked(bool check);
	void on_valveToggle7_clicked(bool check);
	void on_valveToggle8_clicked(bool check);
    void releasexManPos();
    void releaseyManPos();
    void releasezManPos();
    void releasexManNeg();
    void releaseyManNeg();
    void releasezManNeg();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	QPixmap cameraPicture1;
	QGraphicsScene scene_;
	bool valve1Active_;
};

}  // namespace pap_gui

#endif // pap_gui_MAIN_WINDOW_H
