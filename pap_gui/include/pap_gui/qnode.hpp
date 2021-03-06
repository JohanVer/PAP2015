/**
 * @file /include/pap_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef pap_gui_QNODE_HPP_
#define pap_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QImage>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Header.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "pap_common/Task.h"
#include "pap_common/Status.h"
#include "pap_common/VisionStatus.h"
#include "pap_common/DispenseTask.h"
#include "pap_common/DispenseTasks.h"
#include "pap_common/ArduinoMsg.h"
#include <pap_common/task_message_def.h>
#include <pap_common/status_message_def.h>
#include <pap_common/arduino_message_def.h>
#include <pap_common/vision_message_def.h>
#include <pap_common/placer_message_def.h>
#include "pap_common/PlacerStatus.h"
#include <pap_common/Pressure.h>

#include <motorController/controller_interface.h>
#include <motorController/sendfunctions.h>
#include <pcb_cv/sendfunctions.h>

#include <pap_placer/placerClass.hpp>

#include "DispenserPlanner.hpp"

#include <vector>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pap_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	QImage* getCamera1() { return &cameraImage_; }
	QImage* getCamera2() { return &cameraImage2_; }
    motor_controller::controllerStatus getStatus(size_t num_controller){ return motorcontrollerStatus[num_controller];}
	void log( const LogLevel &level, const std::string &msg);
	void cameraCallback(const sensor_msgs::Image::ConstPtr& camera_msg);
	void cameraCallback2(const sensor_msgs::ImageConstPtr& camera_msg);

	void sendTask(pap_common::DESTINATION,pap_common::TASK);
	void sendTask(pap_common::DESTINATION,pap_vision::VISION);
	void sendTask(pap_common::DESTINATION destination,pap_common::TASK task, float x, float y, float z );
	void sendTask(pap_common::DESTINATION destination,pap_vision::VISION task,float x, float y, float z);
    void sendTask(pap_common::DESTINATION destination, pap_common::TASK task,ComponentPlacerData componentData, TIP usedTip);
    void sendDispenserTask(std::vector<dispenseInfo> dispenseTask, double height_offset);

	void statusCallback(const pap_common::StatusConstPtr&  statusMsg);
	void placerStatusCallback(const pap_common::PlacerStatusConstPtr&  statusMsg);
	void visionStatusCallback(const pap_common::VisionStatusConstPtr&  statusMsg);
	void qrCodeCallback(const std_msgs::StringConstPtr& qrMsg);
	void sendRelaisTask(int relaisNumber,bool value);
    void sendStepperTask(int StepperNumber, int steps);
	void resetStepper();
	void setLEDTask(int LEDnumber);
	void resetLEDTask(int LEDnumber);
	void sendPcbImage(visualization_msgs::MarkerArray *markerList);
	void setBottomLEDTask();
	void resetBottomLEDTask();
	void LEDTask(int task, int data);
    void pressureCallback(const pap_common::PressureConstPtr& pressure_Msg);

   motor_send_functions::Client& getMotorClientRef(){
       return *motor_action_client;
   }

   vision_send_functions::Client& getVisionClientRef(){
       return *vision_action_client;
   }

Q_SIGNALS:
	void loggingUpdated();
    void statusUpdated();
	void placerStatusUpdated(int indicator, int status);
	void qrCodeUpdated();
	void cameraUpdated(int index);
    void rosShutdown();
    void smdCoordinates(float x,float y,float rot,unsigned int cameraSelect);
    void signalPosition(float x,float y);
    void dispenserFinished();
    void tipToggled(int,bool);
    void positionGotoReached();
    void updatePressure(float pressure);

private:
    std::unique_ptr<motor_send_functions::Client> motor_action_client;
    std::unique_ptr<vision_send_functions::Client> vision_action_client;

	int init_argc;
	char** init_argv;
	ros::Publisher task_publisher, arduino_publisher_, dispenser_publisher_;
	ros::Subscriber statusSubsriber_;
	ros::Subscriber placerStatusSubscriber_;
	ros::Subscriber visionStatusSubsriber_;
	ros::Subscriber qrCodeScannerSubscriber_;
    ros::Subscriber pressure_Subscriber_;
    QStringListModel logging_model;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image_sub2_;
    cv_bridge::CvImagePtr cv_ptr;
    motor_controller::controllerStatus motorcontrollerStatus[3];
    QImage cameraImage_, cameraImage2_;
    ros::Publisher imagePub_;
    ros::Publisher markerPub_;
    uchar dataArray[640*480 * 3];
    uchar dataArray2[640*480 * 3];

public:
    float pcbWidth_,pcbHeight_;
    bool fakePadPos_;
    float pressure_;
};

}  // namespace pap_gui

#endif /* pap_gui_QNODE_HPP_ */
