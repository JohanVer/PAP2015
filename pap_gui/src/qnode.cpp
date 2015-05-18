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
	chatter_publisher = n_.advertise<std_msgs::String>("chatter", 1000);
	task_publisher = n_.advertise<pap_common::Task>("task", 1000);
	image_sub_ = it_.subscribe("camera1", 1, &QNode::cameraCallback, this);
	statusSubsriber_ = n_.subscribe("status", 1, &QNode::statusCallback, this);
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
	chatter_publisher = n_.advertise<std_msgs::String>("chatter", 1000);
	task_publisher = n_.advertise<pap_common::Task>("task", 1000);
	image_sub_ = it_.subscribe("/camera1", 1, &QNode::cameraCallback, this);
	statusSubsriber_ = n_.subscribe("status", 1, &QNode::statusCallback, this);
	start();
	return true;
}

void QNode::cameraCallback(const sensor_msgs::ImageConstPtr& camera_msg) {
	std::stringstream ss;
	ss << "Got camera picture, " << camera_msg->header.frame_id.c_str();
	std::string s = ss.str();
	this->log(Info, s);

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
	ros::Rate loop_rate(1);
	int count = 0;
	while (ros::ok()) {

		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info, std::string("I sent: ") + msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
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

void QNode::statusCallback(const pap_common::StatusConstPtr& statusMsg) {
	ROS_INFO("Got status msg");

	int index = statusMsg->data1;
	// TODO : Sort by controller
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

	Q_EMIT statusUpdated(index);
}

}  // namespace pap_gui
