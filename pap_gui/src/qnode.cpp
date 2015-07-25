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
	image_sub_ = it_.subscribe("camera1", 10, &QNode::cameraCallback, this);
	image_sub2_ = it_.subscribe("camera2", 10, &QNode::cameraCallback2, this);
	statusSubsriber_ = n_.subscribe("status", 100, &QNode::statusCallback,
			this);
	visionStatusSubsriber_ = n_.subscribe("visionStatus", 100,
			&QNode::visionStatusCallback, this);
	placerStatusSubscriber_ = n_.subscribe("placerStatus", 100,
			&QNode::placerStatusCallback, this);
	qrCodeScannerSubscriber_ = n_.subscribe("barcode", 100,
			&QNode::qrCodeCallback, this);

	dispenser_publisher_ = n_.advertise<pap_common::DispenseTask>(
			"dispenseTask", 100);
	//imagePub_ = it_.advertise("renderedPcb", 1);
	imagePub_ = n_.advertise<sensor_msgs::PointCloud2>("renderedPcb", 2);
	markerPub_ = n_.advertise<visualization_msgs::MarkerArray>("marker", 2);

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

	uchar dataArray[camera_msg->width * camera_msg->height * 3];

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

	uchar dataArray[camera_msg->width * camera_msg->height * 3];

	for (size_t i = 0; i < camera_msg->width * camera_msg->height * 3; i++) {
		dataArray[i] = camera_msg->data.at(i);
	}

	if (camera_msg->header.frame_id == "camera2") {
		cameraImage2_ = QImage(dataArray, camera_msg->width, camera_msg->height,
				QImage::Format_RGB888);
		Q_EMIT cameraUpdated(2);
	}

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
	if (statusMsg->process == pap_common::INFO) {
		if(statusMsg->status == pap_common::DISPENSER_FINISHED){
			Q_EMIT dispenserFinished();
		}

	} else {
		int indicator = statusMsg->process;
		int status = statusMsg->status;
		Q_EMIT placerStatusUpdated(indicator, status);
	}
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

void QNode::sendDispenserTask(dispenseInfo dispenseTask) {
	pap_common::DispenseTask msg;
	msg.xPos1 = dispenseTask.xPos;
	msg.xPos2 = dispenseTask.xPos2;
	msg.yPos1 = dispenseTask.yPos;
	msg.yPos2 = dispenseTask.yPos2;
	msg.velocity = dispenseTask.velocity;
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
	ROS_INFO("QR Code message received");
	//QString qrCode = &qrMsg;
	//Q_EMIT
}

void QNode::sendPcbImage(QImage* image,
		visualization_msgs::MarkerArray *markerList) {
	static unsigned int id_counter;

	// Pointcloud Approach--------------------------------------------------------------------

	/*
	 std_msgs::Header header;
	 header.seq = id_counter + 1;
	 header.stamp = ros::Time::now();
	 header.frame_id = "renderedPCB";

	 sensor_msgs::PointCloud2 out_msg;

	 out_msg.header = header;
	 //out_msg.height = image->height();
	 //out_msg.width = image->width();
	 //out_msg.point_step = 15;
	 //out_msg.row_step = 15 * image->width();

	 // Set modifiers
	 unsigned int n_points = image->height() * image->width();
	 sensor_msgs::PointCloud2Modifier modifier(out_msg);
	 modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
	 modifier.resize(n_points);

	 // Create rgb data from QImage
	 std::vector<uint8_t> rawDataHolder;
	 for (size_t i = 0; i < (image->byteCount()); i++) {
	 if (i != 0 && i % 2044 != 00) {
	 rawDataHolder.push_back((image->bits())[i]);
	 }
	 }

	 ROS_INFO("Size of RGB Array %d : ",(int)rawDataHolder.size());
	 // Calculate width and height for each point (square)

	 double blockWidth = (pcbWidth_ / (float) image->width()) * 0.001; // In meters
	 double blockHeight = (pcbHeight_ / (float) image->height()) * 0.001; // In meters

	 ROS_INFO("BlockWidth: %f BlockHeight: %f",blockWidth,blockHeight);

	 // Define positions of points in meters
	 std::vector<float> point_data;
	 point_data.resize(image->height()* image->width() *3);


	 for(size_t i = 0; i < image->height(); i++){
	 for(size_t j = 0; j < image->width(); j++){
	 point_data[(i*image->width()*3)+3*j] = j*blockWidth;
	 point_data[(i*image->width()*3)+3*j+1] = i * blockHeight;
	 point_data[(i*image->width()*3)+3*j+2] = 0.1;
	 }
	 }

	 ROS_INFO("Size of Pos Array :  %d",(int)point_data.size());

	 //for(size_t i = 0; i < point_data.size(); i++){
	 //	ROS_INFO("X: %f Y: %f Z: %f", point_data[i*3+0],point_data[i*3+1],point_data[i*3+2]);
	 //}


	 // Define the iterators. When doing so, you define the Field you would like to iterate upon and
	 // the type of you would like returned: it is not necessary the type of the PointField as sometimes
	 // you pack data in another type (e.g. 3 uchar + 1 uchar for RGB are packed in a float)
	 sensor_msgs::PointCloud2Iterator<float> iter_x(out_msg, "x");
	 sensor_msgs::PointCloud2Iterator<float> iter_y(out_msg, "y");
	 sensor_msgs::PointCloud2Iterator<float> iter_z(out_msg, "z");
	 // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), you can create iterators for
	 // those: they will handle data packing for you (in little endian RGB is packed as *,R,G,B in a float
	 // and RGBA as A,R,G,B)
	 sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(out_msg, "r");
	 sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(out_msg, "g");
	 sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(out_msg, "b");

	 // Fill the PointCloud2
	 for (size_t i = 0; i < n_points;
	 ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
	 *iter_x = point_data[3 * i + 0];
	 *iter_y = point_data[3 * i + 1];
	 *iter_z = point_data[3 * i + 2];
	 *iter_r = rawDataHolder[3 * i + 0];
	 *iter_g = rawDataHolder[3 * i + 1];
	 *iter_b = rawDataHolder[3 * i + 2];
	 }

	 imagePub_.publish(out_msg);
	 */

	markerPub_.publish(*markerList);

}

}  // namespace pap_gui
