/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <ros/param.h>
#include <pap_gui/qnode.hpp>
#include <ros/node_handle.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace pap_gui {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv) {
    fakePadPos_ = false;
    pcbWidth_ = 0;
    pcbHeight_ = 0;
}

QNode::~QNode() {
    if (ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc, init_argv, "pap_gui");
    if (!ros::master::check()) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n_;
    if (n_.getParam("/fakePadPos", fakePadPos_)) {
        ROS_INFO("GUI: Param: %d", fakePadPos_);
    }else{
        fakePadPos_ = false;
    }
    image_transport::ImageTransport it_(n_);
    // Add your ros communications here.
    task_publisher = n_.advertise<pap_common::Task>("task", 1000);
    arduino_publisher_ = n_.advertise<pap_common::ArduinoMsg>("arduinoTx",
                                                              1000);
    image_sub_ = it_.subscribe("camera1", 2, &QNode::cameraCallback, this);
    image_sub2_ = it_.subscribe("camera2", 2, &QNode::cameraCallback2, this);
    statusSubsriber_ = n_.subscribe("status", 100, &QNode::statusCallback,
                                    this);
    visionStatusSubsriber_ = n_.subscribe("visionStatus", 100,
                                          &QNode::visionStatusCallback, this);
    placerStatusSubscriber_ = n_.subscribe("placerStatus", 100,
                                           &QNode::placerStatusCallback, this);
    qrCodeScannerSubscriber_ = n_.subscribe("barcode", 100,
                                            &QNode::qrCodeCallback, this);

    dispenser_publisher_ = n_.advertise<pap_common::DispenseTasks>(
                "dispenseTask", 100);
    //imagePub_ = it_.advertise("renderedPcb", 1);
    imagePub_ = n_.advertise<sensor_msgs::PointCloud2>("renderedPcb", 2);
    markerPub_ = n_.advertise<visualization_msgs::MarkerArray>("marker", 2);

    motor_action_client = std::unique_ptr<motor_send_functions::Client>(new motor_send_functions::Client("motor_controller_actions", true));
    vision_action_client = std::unique_ptr<vision_send_functions::Client>(new vision_send_functions::Client("vision_actions", true));

    start();
    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string, std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings, "pap_gui");
    if (!ros::master::check()) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n_;
    image_transport::ImageTransport it_(n_);
    // Add your ros communications here.
    task_publisher = n_.advertise<pap_common::Task>("task", 1000);
    arduino_publisher_ = n_.advertise<pap_common::ArduinoMsg>("arduinoTx",
                                                              1000);
    image_sub_ = it_.subscribe("/camera1", 1, &QNode::cameraCallback, this);
    statusSubsriber_ = n_.subscribe("status", 100, &QNode::statusCallback,
                                    this);
    visionStatusSubsriber_ = n_.subscribe("visionStatus", 100,
                                          &QNode::visionStatusCallback, this);
    start();
    return true;
}

void QNode::cameraCallback(const sensor_msgs::ImageConstPtr& camera_msg) {

    for (size_t i = 0; i < camera_msg->width * camera_msg->height * 3; i++) {
        dataArray[i] = camera_msg->data.at(i);
    }

    if (camera_msg->header.frame_id == "camera1") {
        cameraImage_ = QImage(dataArray, camera_msg->width, camera_msg->height,
                              QImage::Format_RGB888);
        Q_EMIT cameraUpdated(1);
    }
}

void QNode::cameraCallback2(const sensor_msgs::ImageConstPtr& camera_msg) {

    for (size_t i = 0; i < camera_msg->width * camera_msg->height * 3; i++) {
        dataArray2[i] = camera_msg->data.at(i);
    }

    if (camera_msg->header.frame_id == "camera2") {
        cameraImage2_ = QImage(dataArray2, camera_msg->width,
                               camera_msg->height, QImage::Format_RGB888);
        Q_EMIT cameraUpdated(2);
    }

}

void QNode::run() {
    ros::Rate loop_rate(100);
    int count = 0;
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log(const LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(), 1);
    std::stringstream logging_model_msg;
    switch (level) {
    case (Debug): {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case (Info): {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case (Warn): {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case (Error): {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case (Fatal): {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
                          new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::sendTask(pap_common::DESTINATION destination,
                     pap_common::TASK task) {
    pap_common::Task taskMsg;
    taskMsg.destination = destination;
    taskMsg.task = task;
    task_publisher.publish(taskMsg);
}

void QNode::sendTask(pap_common::DESTINATION destination,
                     pap_vision::VISION task) {
    pap_common::Task taskMsg;
    taskMsg.destination = destination;
    taskMsg.task = task;
    task_publisher.publish(taskMsg);
}

void QNode::sendTask(pap_common::DESTINATION destination,
                     pap_vision::VISION task, float x, float y, float z) {
    pap_common::Task taskMsg;
    taskMsg.destination = destination;
    taskMsg.task = task;
    taskMsg.data1 = x;
    taskMsg.data2 = y;
    taskMsg.data3 = z;
    task_publisher.publish(taskMsg);

}

void QNode::sendTask(pap_common::DESTINATION destination, pap_common::TASK task,
                     float x, float y, float z) {
    pap_common::Task taskMsg;
    taskMsg.destination = destination;
    taskMsg.task = task;
    taskMsg.data1 = x;
    taskMsg.data2 = y;
    taskMsg.data3 = z;
    task_publisher.publish(taskMsg);
}

void QNode::placerStatusCallback(
        const pap_common::PlacerStatusConstPtr& statusMsg) {

    if (statusMsg->process == pap_common::GOTO_STATE) {
        if (statusMsg->status == pap_common::PLACER_FINISHED) {
            Q_EMIT positionGotoReached();
        }
    } else if (statusMsg->process == pap_common::INFO) {
        switch (statusMsg->status) {
        case pap_common::DISPENSER_FINISHED:
            Q_EMIT dispenserFinished();
            break;

        case pap_common::RIGHT_TIP_DOWN:
            Q_EMIT tipToggled(1,true);
            break;

        case pap_common::RIGHT_TIP_UP:
            Q_EMIT tipToggled(1,false);
            break;

        case pap_common::LEFT_TIP_DOWN:
            Q_EMIT tipToggled(0,true);
            break;

        case pap_common::LEFT_TIP_UP:
            Q_EMIT tipToggled(0,false);
            break;
        }
    } else {
        int indicator = statusMsg->process;
        int status = statusMsg->status;
        Q_EMIT placerStatusUpdated(indicator, status);
    }
}

void QNode::statusCallback(const pap_common::StatusConstPtr& statusMsg) {

    motorcontrollerStatus[0].energized = statusMsg->energized[0];
    motorcontrollerStatus[1].energized = statusMsg->energized[1];
    motorcontrollerStatus[2].energized = statusMsg->energized[2];

    motorcontrollerStatus[0].error = statusMsg->error[0];
    motorcontrollerStatus[1].error = statusMsg->error[1];
    motorcontrollerStatus[2].error = statusMsg->error[2];

    motorcontrollerStatus[0].positionReached = statusMsg->reached[0];
    motorcontrollerStatus[1].positionReached = statusMsg->reached[1];
    motorcontrollerStatus[2].positionReached = statusMsg->reached[2];

    motorcontrollerStatus[0].position = statusMsg->pos[0];
    motorcontrollerStatus[1].position = statusMsg->pos[1];
    motorcontrollerStatus[2].position = statusMsg->pos[2];

    Q_EMIT statusUpdated();
}

void QNode::sendRelaisTask(int relaisNumber, bool value) {
    pap_common::ArduinoMsg arduinoMsg;
    if (value) {
        arduinoMsg.command = pap_common::SETRELAIS;
        arduinoMsg.data = relaisNumber;
    } else {
        arduinoMsg.command = pap_common::RESETRELAIS;
        arduinoMsg.data = relaisNumber;
    }
    arduino_publisher_.publish(arduinoMsg);
}


void QNode::sendStepperTask(int StepperNumber, int steps) {
    pap_common::ArduinoMsg arduinoMsg;
    if (StepperNumber == 2) {
        arduinoMsg.command = pap_common::RUNSTEPPER1;
        arduinoMsg.data = steps;
        arduino_publisher_.publish(arduinoMsg);
    }
    if (StepperNumber == 1) {
        arduinoMsg.command = pap_common::RUNSTEPPER2;
        arduinoMsg.data = steps;
        arduino_publisher_.publish(arduinoMsg);
    }
}

void QNode::resetStepper() {
    pap_common::ArduinoMsg arduinoMsg;
    arduinoMsg.command = pap_common::RESETSTEPPERS;
    arduino_publisher_.publish(arduinoMsg);
}

void QNode::setLEDTask(int LEDnumber) {
    pap_common::ArduinoMsg arduinoMsg;
    arduinoMsg.command = pap_common::SETLED;
    arduinoMsg.data = LEDnumber;
    arduino_publisher_.publish(arduinoMsg);
}

void QNode::resetLEDTask(int LEDnumber) {
    pap_common::ArduinoMsg arduinoMsg;
    arduinoMsg.command = pap_common::RESETLED;
    arduinoMsg.data = LEDnumber;
    arduino_publisher_.publish(arduinoMsg);
}

void QNode::setBottomLEDTask() {
    pap_common::ArduinoMsg arduinoMsg;
    arduinoMsg.command = pap_common::SETBOTTOMLED;
    arduino_publisher_.publish(arduinoMsg);
}

void QNode::resetBottomLEDTask() {
    pap_common::ArduinoMsg arduinoMsg;
    arduinoMsg.command = pap_common::RESETBOTTOMLED;
    arduino_publisher_.publish(arduinoMsg);
}

void QNode::LEDTask(int task, int data) {
    pap_common::ArduinoMsg arduinoMsg;
    arduinoMsg.command = task;
    arduinoMsg.data = data;
    arduino_publisher_.publish(arduinoMsg);
}

void QNode::sendTask(pap_common::DESTINATION destination, pap_common::TASK task,
                     ComponentPlacerData componentData, TIP usedTip) {
    pap_common::Task taskMsg;
    taskMsg.destination = destination;
    taskMsg.task = task;
    taskMsg.data1 = componentData.destX;
    taskMsg.data2 = componentData.destY;
    taskMsg.data3 = componentData.rotation;
    taskMsg.box = componentData.box;
    taskMsg.length = componentData.length;
    taskMsg.width = componentData.width;
    taskMsg.height = componentData.height;
    taskMsg.velX = componentData.tapeX;
    taskMsg.velY = componentData.tapeY;
    taskMsg.velZ = componentData.tapeRot;
    taskMsg.tip = usedTip;
    task_publisher.publish(taskMsg);
}

void QNode::sendDispenserTask(std::vector<dispenseInfo> dispenseTask, double height_offset) {
    pap_common::DispenseTasks msg;

    for(size_t i = 0; i < dispenseTask.size(); i++){
        dispenseInfo &act = dispenseTask.at(i);
        pap_common::DispenseTask task;
        task.xPos1 = act.xPos;
        task.xPos2 = act.xPos2;
        task.yPos1 = act.yPos;
        task.yPos2 = act.yPos2;
        task.velocity = act.velocity;
        task.waitTime = act.time;
        task.type = act.type;
        msg.lines.push_back(task);
    }

    msg.heightOffset = height_offset;

    dispenser_publisher_.publish(msg);
}

void QNode::visionStatusCallback(
        const pap_common::VisionStatusConstPtr& statusMsg) {
    if (statusMsg->task != pap_vision::START_PAD_FINDER) {
        Q_EMIT smdCoordinates(statusMsg->data1, statusMsg->data2,
                              statusMsg->data3, statusMsg->camera);
    } else {
        Q_EMIT signalPosition(statusMsg->data1, statusMsg->data2);
    }
}

void QNode::qrCodeCallback(const std_msgs::StringConstPtr& qrMsg) {
    ROS_INFO("GUI: QR Code message received");
    //QString qrCode = &qrMsg;
    //Q_EMIT
}

void QNode::sendPcbImage(visualization_msgs::MarkerArray *markerList) {
    markerPub_.publish(*markerList);

}

}  // namespace pap_gui
