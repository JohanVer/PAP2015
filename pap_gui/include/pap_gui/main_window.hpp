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

#include <QStandardItemModel>


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

    void on_loadGerberFileButton_clicked();
    void on_startPlacementButton_clicked();
    void on_pausePlacementButton_clicked();
    void on_setCompBoxNrButton_clicked();
    void on_compOrientButton_clicked();
    void on_clearTableButton_clicked();
    void on_tableWidget_clicked();
    void on_compDeleteButton_clicked();
    void on_turnLeftTipButton_clicked();
    void on_turnRightTipButton_clicked();
    void updateComponentTable();
    void updateComponentInformation();
    void loadDatabaseContent();
    void updateDatabaseTable();
    void on_placeSingleComponentButton_clicked();
    void on_compPackageButton_clicked();
    void updateSingleComponentInformation();
    void on_scanPCBButton_clicked();
    void on_setCompBoxNrButton_2_clicked();

    //QWizardPage *createIntroPage();

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
	bool valve2Active_;
	bool valve3Active_;
	bool valve4Active_;
	bool valve5Active_;
	bool valve6Active_;
	bool valve7Active_;
	bool valve8Active_;
    QTableView tableView;
};

}  // namespace pap_gui

#endif // pap_gui_MAIN_WINDOW_H
