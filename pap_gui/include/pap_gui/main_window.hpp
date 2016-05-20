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

#include <pap_gui/MyContextMenuTable.hpp>
#include <ros/package.h>
#include <QtWidgets>
#include <QMainWindow>
#include "ui_main_window.h"
#include "slotselectordialog.h"
#include "versionSelectorDialog.h"
#include "qnode.hpp"
#include "pap_common/Task.h"
#include <pap_common/vision_message_def.h>
#include "pap_common/PlacerStatus.h"
#include <pap_common/placer_message_def.h>
#include "ClickGraphicsView.hpp"
#include "MyContextMenuTable.hpp"
#include "GerberPadParser.hpp"
#include "PadView.hpp"
#include <string>
#include <sstream>
#include <iostream>
#include <QPoint>
#include <QStandardItemModel>
#include <QPixmap>
#include <QPainter>
#include <QSize>
#include "DispenserPlanner.hpp"
#include <tf/transform_broadcaster.h>
#include <algorithm>
#include <pap_common/CommonDataClasses.hpp>
#include "packageDialog.hpp"
#include <motorController/sendfunctions.h>
#include <pap_common/VisionResult.h>
#include <pap_gui/stitchWaypointMaker.h>
#include <pcb_cv/padFinder.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>

class tapeCalibrationValue {
public:
	float x, y, rot;
	int index;

	tapeCalibrationValue(){
		x = 0;
		y = 0;
		rot = 0;
		index = 0;
	}
};

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
	MainWindow(int verions, int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
	void showSelectCompMessage();

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
    void statusUpdated();
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



    // Complete PCB Tab
    void on_loadGerberFileButton_clicked();
    void on_clearTableButton_clicked();

    void on_startSlotWizard_clicked();
    void on_compDeleteButton_clicked();
    void on_compOrientButton_clicked();
    void on_compPackageButton_clicked();

    void on_startPlacementButton_clicked();
    void on_stopPlacementButton_clicked();

    bool emptySlots();

    void on_tableWidget_clicked();

    // Single Component Tab


    void on_turnLeftTipButton_clicked();
    void on_turnRightTipButton_clicked();
    void updateComponentTable();
    void updateComponentInformation();
    void loadDatabaseContent();
    void updateDatabaseTable();
    void on_placeSingleComponentButton_clicked();

    void on_setLEDButton_clicked();
    void setLedFromSelection(int);
    void on_resetLEDButton_clicked();
    void on_ledResetButton_clicked();
    void changeRingLEDBrightness(int brightness);
    void changeBackLEDBrightness(int brightness);
    void changeTopLEDBrightness(int brightness);
    void changeRingColor(int comboValue);
    void on_blinkBackButton_clicked();
    void on_blinkRingButton_clicked();



    // Vision
    // Buttons
    void on_scanQRButton_clicked();
    void on_startChipFinder_Button_clicked();
    void on_startTapeFinder_Button_clicked();
    void on_startPadFinder_Button_clicked();
    void on_StartStopVision_Button_clicked();
    void on_startTipFinder_Button_clicked();
    void on_bottomLEDButton_clicked();
    void on_topLedButton_clicked();

    void gotoPad(QPointF padPos);
    void deletePad(QPointF padPos);

    void on_calibrateTapeButton_clicked(void);
    void calibrateTape(int tapeNumber, float componentWidth,
    		float componentHeight);
    tapeCalibrationValue calculatePosOfTapePart(int numOfTape, int numOfPart);
    // Display Functions
    void displaySMDCoords(float x,float y,float rot, unsigned int cameraSelect);
    void setCamera1Point(QPointF point);
    void setFiducial(QPointF point);
    void setFiducialTable(int number,float xGlobal, float yGlobal);
    void setFiducialPads(int number, float x, float y);
    void initFiducialTable(void);
    void signalPosition(float x,float y);
    void tipToggled(int tipSelect, bool status);
    void sendGotoFiducial(int indexOfFiducial);
    void on_inputPad_Button_clicked();
    void on_padViewSetSize_button_clicked();
    void on_padViewGenerate_button_clicked();
    void redrawPadView();
    void initPadTable(int rows);
    void padPressed(int numberOfFiducial,QPointF padPos);
    void on_calcOrientation_Button_clicked();
    void sendTransforms(double x, double y, double z, double nozzle_1,
    		double nozzle_2);

    //Dispense button

    void on_startDispense_button_clicked();
    void on_stopDispense_button_clicked();
    void on_resetDispense_button_clicked();
    void dispenseSinglePad(QPointF point);

    void on_goToPCBButton_clicked();
    void updatePlacementData(componentEntry &singleComponentIn);

    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);

    void on_calibrationButton_offsets_clicked();
    void on_calibrationButton_ratios_clicked();

    void updatePackageList();
    void updateMissingPackageList();
    void updateMissingPackageTable();

    // "Package Database"-Tab functions
    void on_replaceButton_clicked();
    void on_addPackageButton_clicked();
    void on_editPackageButton_clicked();
    void on_deletePackageButton_clicked();
    void updateCompDimensions();

    int angleToSteps(float angle);

    // Print Buttons
    void on_printButton_offsets_clicked();


    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    void processAllCallbacks();

    void createNewPad(QRectF pad);

private slots:

    void on_take_img_button_clicked();

    void on_pushButton_2_clicked();


    void on_pushButton_clicked();

    void on_pushButton_startQRTop_clicked();

    void on_calibrateSystemButton_clicked();

    void on_calibrateBottomCamButton_clicked();

    void on_calibrateTopCamButton_clicked();

    void on_scanButton_clicked();

    void on_disp_settings_apply_clicked();


    void on_calibrate_dispenser_button_clicked();

    void on_radioButton_clicked(bool checked);

private:

	Ui::MainWindowDesign ui;
	QNode qnode;
	bool bottomLayer_;
	bool alreadyFlipped_;

	// List of all components that need to be placed
	QVector<componentEntry> componentList;
	// List of all types of components/packages used in componentlist
	QVector<string> packageList;
	// List of all unkown/missing packages used in packageList
	QVector<string> missingPackageList;
	// List of all packages in database
	QVector<databaseEntry> databaseVector;

	//DatabaseClass database;
	ComponentPlacerData placementData;
	int tapeCompCounter[20];

	// Current position
	Offset currentPosition;
	// Structures for holding the camera1 image (top camera)
	QPixmap cameraPicture1, cameraPicture2;
    CameraGraphicsScene scene_, scene2_;
	DispenserPlanner dispenserPlanner;

	// Structures for holding the rendered image of the pcb
	QPixmap renderedPadsPixmap;
	graphicsScene scenePads_;

    QPixmap stitched_img_;

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
	float tip1Pos_,tip2Pos_;
	// Variables for fiducial marking
    QTableView tableView;
    int fiducialSize_;
    QPointF padPosition_;
    GerberPadParser padParser;
    int id_;

    bool dispenserPaused;
    int lastDispenserId;

    bool completePlacementRunning;
    bool singlePlacementRunning;
    int componentIndicator;
    bool completeCalibrationRunning;

    std::vector<tapeCalibrationValue> tapeCalibrationValues;

    cv::Size2d pic_offset_;

    // Dispenser settings:

    double edge_percentage_;
    double dispenser_velocity_;
    double nozzle_diameter_;

    bool tip_thresholding_on;
};

}  // namespace pap_gui

#endif // pap_gui_MAIN_WINDOW_H
