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
#include "../include/pap_gui/qnode.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace pap_gui {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

QNode::QNode(int argc, char** argv) :
		init_argc(argc), init_argv(argv) {
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
	image_transport::ImageTransport it_(n_);
	// Add your ros communications here.
	task_publisher = n_.advertise<pap_common::Task>("task", 1000);
	arduino_publisher_ = n_.advertise<pap_common::ArduinoMsg>("arduinoTx",
			1000);
	image_sub_ = it_.subscribe("camera1", 1, &QNode::cameraCallback, this);
	statusSubsriber_ = n_.subscribe("status", 100, &QNode::statusCallback,
			this);
	visionStatusSubsriber_ = n_.subscribe("visionStatus", 100,
			&QNode::visionStatusCallback, this);
	placerStatusSubscriber_ = n_.subscribe("placerStatus", 100,
			&QNode::placerStatusCallback, this);
	qrCodeScannerSubscriber_ = n_.subscribe("barcode", 100,
			&QNode::qrCodeCallback, this);

	imagePub_ = it_.advertise("renderedPcb", 1);

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
	//std::stringstream ss;
	//ss << "Got camera picture, " << camera_msg->header.frame_id.c_str();
	//std::string s = ss.str();
	//this->log(Info, s);

	try {
		cv_ptr = cv_bridge::toCvCopy(camera_msg,
				sensor_msgs::image_encodings::RGB8);
	} catch (cv_bridge::Exception& e) {
		this->log(Error, "Error indexing picture");
		return;
	}

	cameraImage_ = QImage((uchar*) cv_ptr->image.data, cv_ptr->image.cols,
			cv_ptr->image.rows, QImage::Format_RGB888);

	Q_EMIT cameraUpdated(1);
}

void QNode::run() {
	ros::Rate loop_rate(20);
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
	int indicator = statusMsg->process;
	int status = statusMsg->status;
	Q_EMIT placerStatusUpdated(indicator, status);
}

void QNode::statusCallback(const pap_common::StatusConstPtr& statusMsg) {

	int index = statusMsg->data1;
	if (statusMsg->status == pap_common::ENERGIZED) {
		motorcontrollerStatus[index].energized = true;
	}
	if (statusMsg->status == pap_common::NOENERGY) {
		motorcontrollerStatus[index].energized = false;
	}

	if (statusMsg->status == pap_common::POSITIONREACHED) {
		motorcontrollerStatus[index].positionReached = true;
	}

	if (statusMsg->status == pap_common::POSITIONNOTREACHED) {
		motorcontrollerStatus[index].positionReached = false;
	}

	if (statusMsg->status == pap_common::ERROR) {
		motorcontrollerStatus[index].error = true;
	}

	if (statusMsg->status == pap_common::NOERROR) {
		motorcontrollerStatus[index].error = false;
	}

	switch (index) {
	case pap_common::XMOTOR:
		motorcontrollerStatus[index].pos = statusMsg->posX;
		break;
	case pap_common::YMOTOR:
		motorcontrollerStatus[index].pos = statusMsg->posY;
		break;
	case pap_common::ZMOTOR:
		motorcontrollerStatus[index].pos = statusMsg->posZ;
		break;
	}

	Q_EMIT statusUpdated(index);
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

void QNode::sendStepperTask(int StepperNumber, int rotationAngle) {
	pap_common::ArduinoMsg arduinoMsg;
	if (StepperNumber == 1) {
		arduinoMsg.command = pap_common::RUNSTEPPER1;
		arduinoMsg.data = rotationAngle;
		arduino_publisher_.publish(arduinoMsg);
	}
	if (StepperNumber == 2) {
		arduinoMsg.command = pap_common::RUNSTEPPER2;
		arduinoMsg.data = rotationAngle;
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


void QNode::sendTask(pap_common::DESTINATION destination, pap_common::TASK task,
		ComponentPlacerData componentData) {
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
	task_publisher.publish(taskMsg);
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
	ROS_INFO("QR Code message received");
	//QString qrCode = &qrMsg;
	//Q_EMIT
}

void QNode::sendPcbImage(QImage* image) {
	static unsigned int id_counter;
	std_msgs::Header header;
	header.seq = id_counter + 1;
	header.stamp = ros::Time::now();
	header.frame_id = "renderedPCB";

	sensor_msgs::Image out_msg;

	out_msg.header = header; // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever

	out_msg.height = image->height();
	out_msg.width = image->width();
	std::vector<uint8_t> rawDataHolder;

	for (size_t i = 0; i < (image->byteCount()); i++) {
		if(i != 0 && i % 2044 != 00){
		rawDataHolder.push_back((image->bits())[i]);
		}
	}

	ROS_INFO(
			"Image byte size: %d Expected %d Width: %d Height: %d Vector_Size: %d Bytes_Per_Line: %d",
			image->byteCount(), image->width() * image->height() * 3,
			image->width(), image->height(),rawDataHolder.size(),image->bytesPerLine());
	out_msg.step = image->bytesPerLine()-1;
	out_msg.data = rawDataHolder;

	imagePub_.publish(out_msg);
}

}  // namespace pap_gui
