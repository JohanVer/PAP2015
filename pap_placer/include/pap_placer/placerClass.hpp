#ifndef PLACER_CLASS_H_
#define PLACER_CLASS_H_

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <vector>
#include <cmath>
#include "pap_common/Task.h"
#include "pap_common/Status.h"
#include <pap_common/task_message_def.h>
#include <pap_common/status_message_def.h>
#include <pap_common/vision_message_def.h>
#include <pap_common/CommonDataClasses.hpp>
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <ros/package.h>

using namespace std;


namespace pap_placer{
//!
//! \brief The plane class describes a coordinate form of a plane
//!
class plane{

public:
    plane(){
        a_ = 0;
        b_ = 0;
        c_ = 0;
        d_ = 0;
    }

    plane(double a, double b, double c, double d){
        a_ = a;
        b_ = b;
        c_ = c;
        d_ = d;
    }

    double getHeight(double x, double y){
        return -((a_ * x + b_ * y + d_) / c_);
    }

private:
    double a_;
    double b_;
    double c_;
    double d_;
};
}

//!
//! \brief The PlaceController class provides all calibration, placement and dispensing functionalities
//!
class PlaceController {
public:
    PlaceController();
    ~PlaceController();

    //!
    //! \brief loadParams restores default parameters (px ratios, offsets)
    //!
    void loadParams();

    //!
    //! \brief saveOffsetsToFile stores latest parameter values (px ratios, offsets)
    //!
    bool saveOffsetsToFile();

    //!
    //! \brief updateCameraBottomOffset corrects bottom camera offset
    //! \param update_x offset correction in x
    //! \param update_y offset correction in y
    //!
    void updateCameraBottomOffset(float update_x, float update_y);

    //!
    //! \brief updateTip1Offset corrects relative offset between tip1 and top camera
    //! \param update_x offset correction in x
    //! \param update_y offset correction in y
    //!
    void updateTip1Offset(float update_x, float update_y);

    //!
    //! \brief updateTip2Offset corrects relative offset between tip2 and top camera
    //! \param update_x offset correction in x
    //! \param update_y offset correction in y
    //!
    void updateTip2Offset(float update_x, float update_y);

    //!
    //! \brief updatedispenserTipOffset corrects relative offset between dispenser tip and top camera
    //! \param update_x offset correction in x
    //! \param update_y offset correction in y
    //!
    void updatedispenserTipOffset(float update_x, float update_y);

    //!
    //! \brief updatePlacementData updates component data of desired tip and sets isWaiting flag
    //! \param data is the new placement data of a component
    //! \param usedTip selects tip the new placement data is assigned to
    //!
    void updatePlacementData(ComponentPlacerData& data, TIP usedTip);

    //!
    //! \brief getCameraProjectionOffset Gets offset from top camera to its focus point
    //! \return Offset
    //!
    Offset getCameraProjectionOffset();

    //!
    //! \brief getBottomCamCoordinates returns current bottom cam coordinates
    //!
    Offset getBottomCamCoordinates();

    //!
    //! \brief getTipCoordinates returns current absolute coordinates of used tip
    //! \param usedTip selects left or right tip offset
    //!
    Offset getTipCoordinates(TIP usedTip);

    //!
    //! \brief getDispenserCoordinates returns current dispenser tip coordinates
    //!
    Offset getDispenserCoordinates();

    //!
    //! \brief getBoxCoordinates
    //! \param usedTip selects box of left or right tip component
    //! \return box coordinates
    //!
    Offset getBoxCoordinates(TIP usedTip);

    //!
    //! \brief getCompPickUpCoordinates
    //! \param usedTip selects component pick-up coordinates of left or right tip component
    //! \return absolute pick-up coordinates
    //!
    Offset getCompPickUpCoordinates(TIP usedTip);

    //!
    //! \brief getCompPlaceCoordinates
    //! \param usedTip selects component place coordinates of left or right tip component
    //! \return absolute place coordinates
    //!
    Offset getCompPlaceCoordinates(TIP usedTip);

    //!
    //! \brief getPCBCalibCoordinates
    //! \return absolute PCB coordinates
    //!
    Offset getPCBCalibCoordinates();

    //!
    //! \brief getCompSuckingHeight
    //! \param usedTip selects left or right tip component
    //! \return optimal sucking height for selected component
    //!
    float getCompSuckingHeight(TIP usedTip);

    //!
    //! \brief getCompPlaceHeight
    //! \param usedTip selects left or right tip component
    //! \return optimal placement height for selected component
    //!
    float getCompPlaceHeight(TIP usedTip);

    //!
    //! \brief getBoxNumber
    //! \param usedTip selects left or right tip component
    //! \return box number for selected component
    //!
    int getBoxNumber(TIP usedTip);

    //!
    //! \brief getComponentHeight
    //! \param usedTip selects left or right tip component
    //! \return selected component height
    //!
    float getComponentHeight(TIP usedTip);

    //!
    //! \brief getComponentLenth
    //! \param usedTip selects left or right tip component
    //! \return selected component length
    //!
    float getComponentLenth(TIP usedTip);

    //!
    //! \brief getComponentWidth
    //! \param usedTip selects left or right tip component
    //! \return selected component width
    //!
    float getComponentWidth(TIP usedTip);

    //!
    //! \brief getFinderType
    //! \param usedTip selects left or right tip component
    //! \return corresponding visual finder type for selected component
    //!
    pap_vision::VISION getFinderType(TIP usedTip);

    //!
    //! \brief getDispenserVel
    //! \return curren dispenser velocity
    //!
    double getDispenserVel();

    //!
    //! \brief setDispenserVel
    //! \param vel new dispenser velocity
    //!
    void setDispenserVel(double vel);

    //!
    //! \brief setPickUpCorrectionOffset
    //! \param xDiff pick-up correction in x
    //! \param yDiff pick-up correction in y
    //! \param rotDiff pick-up rotational correction
    //!
    void setPickUpCorrectionOffset(float xDiff, float yDiff, float rotDiff);

    //!
    //! \brief angleToSteps converts a given angle into steps for a stepper motor with 1.8° resolution
    //! \param angle in degrees
    //! \return steps for stepper motor
    //!
    int angleToSteps(float angle);


    // Absolute offsets
    Offset cameraBottomOffset;
    Offset idleCoordinates_;
    Offset SLOT_QR_Offset_, PCB_QR_Offset_;
    Offset TAPE_QR_Offset_, BottomCam_QR_Offset_;
    Offset Checkerboard_top1_Offset_, Checkerboard_top2_Offset_;
    Offset Checkerboard_bottom1_Offset_, Checkerboard_bottom2_Offset_;
    Offset pickUpAreaOffset;
    Offset pcbOriginOffset;
    Offset lastDestination_;
    Offset tipHeightCalibrationOffset_;
    Offset dispenserCalibOffsetA, dispenserCalibOffsetB, dispenserCalibOffsetC;

    // Relative dispenser tip offset
    Offset dispenserTipOffset;

    //
    float leftTipPCBHeight_, rightTipPCBHeight_;
    float leftTipSuckingHeight_, rightTipSuckingHeight_;

    // Height parameters for placement process
    float MovingHeight_, largeBoxHeight_, dispenserHeight_, dispenser_surface_offset_;
    double dispenser_height_offset_;
    pap_placer::plane dispenser_cal_plane_;
    bool plane_calibrated_;

    // Component placement data for both tips
    ComponentPlacerData leftTipComponent, rightTipComponent;

    bool pickRelQR_;
    dispenseInfos dispenseTask;

    // Offsets relative to top camera
    Offset tip2Offset, tip1Offset;


private:

    Offset camera_projection_offset_;

    // Relative correction feedback from vision for pick-up and place
    Offset PickUpCorrection, PlaceCorrection;


    double corr_dispenser_vel_;

};

#endif // PLACER_CLASS_H_
