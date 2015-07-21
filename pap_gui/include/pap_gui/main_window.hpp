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

#include <MyContextMenuTable.hpp>
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "pap_common/Task.h"
#include "../../../pap_common/include/pap_common/vision_message_def.h"
#include "pap_common/PlacerStatus.h"
#include "../../../pap_common/include/pap_common/placer_message_def.h"
#include "ClickGraphicsView.hpp"
#include "MyContextMenuTable.hpp"
#include "GerberPadParser.hpp"
#include "PadView.hpp"
#include <string>
#include <sstream>
#include <iostream>
#include <QPoint>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "cv.h"
#include "highgui.h"
#include <QStandardItemModel>
#include <QPixmap>
#include <QPainter>
#include <QSize>

#include "../../../pap_placer/include/pap_placer/placerNode.hpp"


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
	void placerStatusUpdated(int indicator, int state);
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
    void on_setLEDButton_clicked();
    void on_resetLEDButton_clicked();

    void on_startSinglePlacementButton_clicked();
    //void initializeBoxPositionVector();
    // Vision
    // Buttons
    void on_startChipFinder_Button_clicked();
    void on_startSmallSMDFinder_Button_clicked();
    void on_startTapeFinder_Button_clicked();
    void on_startPadFinder_Button_clicked();
    void on_StartStopVision_Button_clicked();
    void on_startTipFinder_Button_clicked();
    void on_bottomLEDButton_clicked();
    void gotoPad(QPointF padPos);
    // Display Functions
    void displaySMDCoords(float x,float y,float rot, unsigned int cameraSelect);
    void setCamera1Point(QPointF point);
    void setFiducial(QPointF point);
    void setFiducialTable(int number,float xGlobal, float yGlobal);
    void setFiducialPads(int number, float x, float y);
    void initFiducialTable(void);
    void signalPosition(float x,float y);
    void sendGotoFiducial(int indexOfFiducial);
    void on_inputPad_Button_clicked();
    void on_padViewSetSize_button_clicked();
    void on_padViewGenerate_button_clicked();
    void initPadTable(int rows);
    void padPressed(int numberOfFiducial,QPointF padPos);
    void on_calcOrientation_Button_clicked();

    void on_pickupComponentButton_clicked();
    void on_goToComponentButton_clicked();
    void on_goToPCBButton_clicked();
    void on_placeComponentButton_clicked();
    void updatePlacementData();

    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);

    void on_calibrationButton_clicked();
    void on_ScanQRCodeButton_clicked();
    void findQRCode();

    //QWizardPage *createIntroPage();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	bool bottomLayer_;
	bool alreadyFlipped_;

	// Current position
	Offset currentPosition;
	// Structures for holding the camera1 image (top camera)
	QPixmap cameraPicture1, cameraPicture2;
	QGraphicsScene scene_, scene2_;



	// Structures for holding the rendered image of the pcb
	QPixmap renderedPadsPixmap;
	graphicsScene scenePads_;

	// Placement process indiactors
	/*QPixmap indicatorClearPixmap;
	QPixmap indicatorActivePixmap;
	QPixmap indicatorFinishedPixmap;
	QPixmap indicatorErrorPixmap;*/

	// Variables of the valves
	bool valve1Active_;
	bool valve2Active_;
	bool valve3Active_;
	bool valve4Active_;
	bool valve5Active_;
	bool valve6Active_;
	bool valve7Active_;
	bool valve8Active_;
	bool visionStarted_, sizeDefined_,padFileLoaded_;

	// Variables for fiducial marking
    QTableView tableView;
    int fiducialSize_;
    QPointF padPosition_;
    GerberPadParser padParser;
    int id_;

};

}  // namespace pap_gui

#endif // pap_gui_MAIN_WINDOW_H
