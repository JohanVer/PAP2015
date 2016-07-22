#ifndef TAPECALIBRATION_H
#define TAPECALIBRATION_H

#include <ros/ros.h>
#include <QDialog>
#include <pap_gui/qnode.hpp>
#include <QPixmap>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <pap_common/CommonDataClasses.hpp>
#include <motorController/sendfunctions.h>
#include <pcb_cv/sendfunctions.h>
#include <pap_placer/offsetTable.hpp>

namespace pap_gui{
    class TapeCalibrater;
}

namespace Ui {
class TapeCalibrationDialog;
}

//!
//! \brief The TapeCalibrationDialog class implements an user
//! interface for displaying and selecting a tape component
//!
class TapeCalibrationDialog : public QDialog
{
    Q_OBJECT
public:
    explicit TapeCalibrationDialog(pap_gui::QNode &ros_node, pap_gui::TapeCalibrater &calibrater, int tape_nr, QWidget *parent = 0);

    //!
    //! \brief getTapePos
    //! \return component position in a tape
    //!
    int getTapePos(){
        return current_tape_pos_;
    }

    ~TapeCalibrationDialog();

public Q_SLOTS:
    void cameraUpdated(int index);

private slots:
    void on_next_button_clicked();
    void on_before_button_clicked();
    void on_buttonBox_2_accepted();
    void on_buttonBox_2_rejected();

private:
    Ui::TapeCalibrationDialog *ui;
    pap_gui::QNode &ros_node_;
    pap_gui::TapeCalibrater &calibrater_;
    QGraphicsScene scene_;
    int current_tape_pos_;
    int current_tape_index_;
};

namespace pap_gui{

//!
//! \brief The tapeCalibrationValue class contains position, rotation and
//! an index of a calibrated tape
//!
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

//!
//! \brief The TapeCalibrater class implements functionalities to
//! get a global offset in the robot coordinate system of a desired
//! component number in a tape
//!
class TapeCalibrater{
public:
    TapeCalibrater(pap_gui::QNode &ros_node);

    //!
    //! \brief reset clears all tape calibration variables
    //!
    void reset();

    //!
    //! \brief calibrateTape computes global offset of the first component
    //! in tape with given tapeNumber
    //! \param tapeNumber of tape to be calibrated
    //! \param componentWidth - width of expected component in this tape
    //! \param componentHeight - height of expected component in this tape
    //! \return true if calibration successfull, otherwise false
    //!
    bool calibrateTape(int tapeNumber, float componentWidth, float componentHeight);

    //!
    //! \brief calculatePosOfTapePart computes global offset of given part and
    //! tape in robot coordinate system
    //! \param numOfTape tape id to be used
    //! \param numOfPart number of part to be used
    //! \param out offset coordinates x and y
    //! \return true if tape and part found, otherwise false
    //!
    bool calculatePosOfTapePart(int numOfTape, int numOfPart, Offset &out);

private:
    pap_gui::QNode &ros_node_;
    std::vector<tapeCalibrationValue> tapeCalibrationValues;
};

}

#endif // TAPECALIBRATION_H
