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

class TapeCalibrationDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TapeCalibrationDialog(pap_gui::QNode &ros_node, pap_gui::TapeCalibrater &calibrater, int tape_nr, QWidget *parent = 0);

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

class TapeCalibrater{
public:
    TapeCalibrater(pap_gui::QNode &ros_node);

    void reset();

    bool calibrateTape(int tapeNumber, float componentWidth, float componentHeight);

    bool calculatePosOfTapePart(int numOfTape, int numOfPart, Offset &out);

private:

    pap_gui::QNode &ros_node_;
    std::vector<tapeCalibrationValue> tapeCalibrationValues;

};

}

#endif // TAPECALIBRATION_H
