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
#include <pap_gui/tapecalibration.h>
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
#include "PlacementPlanner.hpp"
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
#include "DatabaseClass.hpp"

#define MIN_PRESSURE 4

/*****************************************************************************
** Namespace
*****************************************************************************/
namespace pap_gui {


/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
//!
//! \brief The MainWindow class implements all operations relating to the GUI
//!
class MainWindow : public QMainWindow {

Q_OBJECT
public:
    MainWindow(int verions, int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    //!
    //! \brief ReadSettings load up qt program settings at startup
    //!
    void ReadSettings();

    //!
    //! \brief WriteSettings saves qt program settings when closing
    //!
    void WriteSettings();

    //!
    //! \brief closeEvent
    //! \param event
    //!
    void closeEvent(QCloseEvent *event); // Overloaded function

    //!
    //! \brief showNoMasterMessage
    //!
    void showNoMasterMessage();

    //!
    //! \brief showSelectCompMessage
    //!
    void showSelectCompMessage();

public Q_SLOTS:
    ///******************************************
    ///** Auto-connections (connectSlotsByName())
    ///*******************************************/
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

    //!
    //! \brief PCB Placer-Tab Slots
    //!
    void on_loadGerberFileButton_clicked();
    void on_clearTableButton_clicked();
    void on_startSlotWizard_clicked();
    void on_compDeleteButton_clicked();
    void on_compOrientButton_clicked();
    void on_compPackageButton_clicked();
    void on_startPlacementButton_clicked();
    void on_stopPlacementButton_clicked();
    void on_placeSingleComponentButton_clicked();




    void updatePlacementInfo();

    void updateCurrentNozzles();



    void on_tableWidget_clicked();

    void on_turnLeftTipButton_clicked();
    void on_turnRightTipButton_clicked();
    void updateComponentTable();
    void updateComponentInformation();
    void updateDatabaseTable();

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
    void updatePressure(float pressure);



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
    bool startTapePartSelector(int numOfTape);

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

    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);

    bool isPressureEnough();
    bool checkSlotsAndTapeCalibration();
    bool isPCBCalibrated();
    bool isTapeCalibrated();

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

    void transformAllComp(vector<ComponentPlacerData>& allCompData);
    void transformSingleComp(int currentComp, ComponentPlacerData& singleCompData);

    bool driveToCoord(const double &x, const double &y, const double &z, const double moving_height = 45);

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

    void on_pushButton_recalibrate_left_tip_clicked();

    void on_pushButton_recalibrate_right_tip_clicked();

    void on_decreaseCamOffsetX_pushButton_clicked();

    void on_increaseCamOffsetX_pushButton_clicked();

    void on_decreaseCamOffsetY_pushButton_clicked();

    void on_increaseCamOffsetY_pushButton_clicked();

    void on_startCamProjectionCalibration_pushButton_clicked();

private:

    Ui::MainWindowDesign ui;
    QNode qnode;
    std::unique_ptr<pap_gui::TapeCalibrater> tape_calibrater_;
    bool bottomLayer_;
    bool alreadyFlipped_;

    float leftTipRadius, rightTipRadius;
    PlacementPlanner placementPlanner;
    DataIO dataIO;

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
    DispenserPlanner::DispenserPlanner dispenserPlanner;
    DispenserPlanner::DotPlanner dotPlanner;

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
    std::set<size_t> dispensed_ids_;

    bool completePlacementRunning;
    bool singlePlacementRunning;
    int componentIndicator;
    bool completeCalibrationRunning;

    cv::Size2d pic_offset_;

    // Dispenser settings:
    double edge_percentage_;
    double dispenser_velocity_;
    double nozzle_diameter_;
    std::string disp_needle_name_;
    double alpha_;
    enum DispenserPlanner::DOT_ALIGN alignment_;
    enum DispenserPlanner::PLANNER_SELECT planner_selection_;
    double wait_time_;
    double dispenser_height_offset_;

    bool tip_thresholding_on;
    float pressure_;

    bool PCBTransformCalibrated_, tapeCalibrated_;

    std::map<std::string, double> disp_needle_to_od_;
    std::map<std::string, double> disp_needle_to_id_;
};

}  // namespace pap_gui

#endif // pap_gui_MAIN_WINDOW_H


