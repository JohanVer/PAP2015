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
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Header.h"
#include "std_msgs/ByteMultiArray.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv.h"
#include <image_transport/image_transport.h>
#include "pap_common/Task.h"
#include "pap_common/Status.h"
#include "pap_common/VisionStatus.h"
#include "pap_common/ArduinoMsg.h"
#include "../../../pap_common/include/pap_common/task_message_def.h"
#include "../../../pap_common/include/pap_common/status_message_def.h"
#include "../../../pap_common/include/pap_common/arduino_message_def.h"
#include "../../../pap_common/include/pap_common/vision_message_def.h"
#include "../../../pap_placer/include/pap_placer/placerClass.hpp"
#include <vector>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pap_gui {

/*****************************************************************************
** Class
*****************************************************************************/

// TODO: This is redundant
class controllerStatus {
public:
	controllerStatus() {
		error = false;
		energized = false;
		positionReached = false;
		pos = 0;
	}

	float pos;
	bool error;
	bool energized;
	bool positionReached;
private:
};

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
	controllerStatus* getStatus(){ return motorcontrollerStatus;}
	void log( const LogLevel &level, const std::string &msg);
	void cameraCallback(const sensor_msgs::Image::ConstPtr& camera_msg);

	void sendTask(pap_common::DESTINATION,pap_common::TASK);
	void sendTask(pap_common::DESTINATION,pap_vision::VISION);
	void sendTask(pap_common::DESTINATION destination,pap_common::TASK task, float x, float y, float z );
	void sendTask(pap_common::DESTINATION destination,pap_vision::VISION task,float x, float y, float z);
	void sendTask(pap_common::DESTINATION destination, pap_common::TASK task,ComponentPlacerData componentData);

	void statusCallback(const pap_common::StatusConstPtr&  statusMsg);
	void visionStatusCallback(const pap_common::VisionStatusConstPtr&  statusMsg);
	void sendRelaisTask(int relaisNumber,bool value);
	void sendStepperTask(int StepperNumber, int rotationAngle);
	void resetStepper();
	void setLEDTask(int LEDnumber);
	void resetLEDTask(int LEDnumber);

Q_SIGNALS:
	void loggingUpdated();
	void statusUpdated(int index);
	void cameraUpdated(int index);
    void rosShutdown();
    void smdCoordinates(float x,float y,float rot);
    void signalPosition(float x,float y);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher task_publisher, arduino_publisher_;
	ros::Subscriber statusSubsriber_;
	ros::Subscriber visionStatusSubsriber_;
    QStringListModel logging_model;
    image_transport::Subscriber image_sub_;
    cv_bridge::CvImagePtr cv_ptr;
    controllerStatus motorcontrollerStatus[3];
    QImage cameraImage_;

};

}  // namespace pap_gui

#endif /* pap_gui_QNODE_HPP_ */
