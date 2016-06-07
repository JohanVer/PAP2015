#include <pap_gui/tapecalibration.h>
#include "ui_tapecalibration.h"

namespace pap_gui{
TapeCalibrater::TapeCalibrater(pap_gui::QNode &ros_node):
    ros_node_(ros_node){

}

void TapeCalibrater::reset(){
    tapeCalibrationValues.clear();
}

bool TapeCalibrater::calibrateTape(int tapeNumber, float componentWidth,
                                   float componentHeight) {
    // TODO{Johan}: Goto tape with index: tapeNumber

    Offset temp = TapeOffsetTable[tapeNumber];
    temp.x += 108.42;
    temp.y += 261;
    temp.z = 20.2;


    if(!motor_send_functions::sendMotorControllerAction(ros_node_.getMotorClientRef(), pap_common::COORD,
                                                        (ros_node_.getStatus(0)).position,
                                                        (ros_node_.getStatus(1)).position,
                                                        40)){
        return false;
    }

    if(!motor_send_functions::sendMotorControllerAction(ros_node_.getMotorClientRef(), pap_common::COORD,
                                                        temp.x,
                                                        temp.y,
                                                        40)){
        return false;
    }

    if(!motor_send_functions::sendMotorControllerAction(ros_node_.getMotorClientRef(), pap_common::COORD,
                                                        temp.x,
                                                        temp.y,
                                                        temp.z)){
        return false;
    }

    tapeCalibrationValue calibrationVal;
    calibrationVal.index = tapeNumber;

    pap_common::VisionResult res;
    if(!vision_send_functions::sendVisionTask(ros_node_.getVisionClientRef(), pap_vision::START_TAPE_FINDER, pap_vision::CAMERA_TOP, componentWidth, componentHeight, 0, res, 50))
        return false;

    double xTapeCalibration = res.data1;
    double yTapeCalibration = res.data2;

    calibrationVal.x = xTapeCalibration + (ros_node_.getStatus(0)).position;
    calibrationVal.y = yTapeCalibration + (ros_node_.getStatus(1)).position;

    ros::Duration(1).sleep();
    // Search tape dimensions and rotation

    if(!vision_send_functions::sendVisionTask(ros_node_.getVisionClientRef(), pap_vision::START_TAPE_FINDER, pap_vision::CAMERA_TOP, componentWidth, componentHeight, 1, res, 50))
        return false;

    calibrationVal.rot = res.data3;
    if (calibrationVal.rot == -90 || calibrationVal.rot == 90) {
        calibrationVal.rot = 0;
    }

    ROS_INFO(" GUI: Tape Calibration: Got x: %f y: %f rot: %f",
             calibrationVal.x, calibrationVal.y, calibrationVal.rot);
    tapeCalibrationValues.push_back(calibrationVal);

    return true;
}

// EXAMPLE: Get 4th position of component in 1st tape
//tapeCalibrationValue positionOfComponent = calculatePosOfTapePart(1,4);
bool TapeCalibrater::calculatePosOfTapePart(int numOfTape, int numOfPart, Offset &out) {
    tf::Transform rotation_;

    int indexInVector = -1;
    for (size_t i = 0; i < tapeCalibrationValues.size(); i++) {
        if (tapeCalibrationValues[i].index == numOfTape) {
            indexInVector = i;
        }
    }

    if (indexInVector == -1) {
        ROS_ERROR("Tape calibration values not found!");
        return false;
    }

    tf::Point pointToTransform;
    // This point should be transformed
    // Distance between comp. on tape is 2 mm (0402 comp.)
    pointToTransform.setX(0.0);
    pointToTransform.setY(numOfPart * 2.0);
    pointToTransform.setZ(0.0);

    // This rotates the component to the tape orientation
    tf::Quaternion rotQuat;
    rotQuat.setEuler(0.0, 0.0,
                     tapeCalibrationValues[indexInVector].rot * (M_PI / 180.0));
    rotation_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    rotation_.setRotation(rotQuat);

    pointToTransform = rotation_ * pointToTransform;

    out.x = pointToTransform.x() + tapeCalibrationValues[indexInVector].x;
    out.y = pointToTransform.y() + tapeCalibrationValues[indexInVector].y;
    out.rot = tapeCalibrationValues[indexInVector].rot;

    ROS_INFO("GUI: Calculated Pos of part in Tape: x %f y %f rot %f", out.x,
             out.y, out.rot);
    return true;
}

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TapeCalibrationDialog::TapeCalibrationDialog(pap_gui::QNode &ros_node, pap_gui::TapeCalibrater &calibrater, int tape_nr, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TapeCalibrationDialog),
    ros_node_(ros_node),
    calibrater_(calibrater)
{
    ui->setupUi(this);
    ui->tape_view->setScene(&scene_);
    ui->tape_view->show();
    current_tape_index_ = tape_nr;
    current_tape_pos_ = 0;
}

TapeCalibrationDialog::~TapeCalibrationDialog()
{
    delete ui;
}

void TapeCalibrationDialog::cameraUpdated(int index) {
    if (index == 1) {
        scene_.clear();
        QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(*(ros_node_.getCamera1())));
        scene_.addItem(item);

    }
}

void TapeCalibrationDialog::on_next_button_clicked()
{
    current_tape_pos_++;
    Offset pos;
    if(calibrater_.calculatePosOfTapePart(current_tape_index_, current_tape_pos_, pos)){
        if(!motor_send_functions::sendMotorControllerAction(ros_node_.getMotorClientRef(), pap_common::COORD,
                                                            pos.x,
                                                            pos.y,
                                                            (ros_node_.getStatus(2)).position)){
            return;
        }
    }else{
        std::cerr << "Couldn't look up tape pos...\n";
    }
}

void TapeCalibrationDialog::on_before_button_clicked()
{
    if(current_tape_pos_ > 0) current_tape_pos_ = current_tape_pos_ -1;
    Offset pos;
    if(calibrater_.calculatePosOfTapePart(current_tape_index_, current_tape_pos_, pos)){
        if(!motor_send_functions::sendMotorControllerAction(ros_node_.getMotorClientRef(), pap_common::COORD,
                                                            pos.x,
                                                            pos.y,
                                                            (ros_node_.getStatus(2)).position)){
            return;
        }
    }else{
        std::cerr << "Couldn't look up tape pos...\n";
    }
}

void TapeCalibrationDialog::on_buttonBox_2_accepted()
{
 this->close();
}

void TapeCalibrationDialog::on_buttonBox_2_rejected()
{
    current_tape_pos_ = -1;
    this->close();
}
