#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "cv.h"
#include "highgui.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"
#include <vector>
#include <cmath>
#include <image_transport/image_transport.h>
#include "pap_common/Task.h"
#include "pap_common/VisionStatus.h"
#include "../../pap_common/include/pap_common/vision_message_def.h"
#include "../../pap_common/include/pap_common/task_message_def.h"
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace cv;

image_transport::Publisher image_pub_;
image_transport::Publisher image_pub_2;
cv_bridge::CvImage out_msg;

enum VISION_PROCESS {
	CHIP, SMALL_SMD, TAPE, PAD, IDLE, CIRCLE, QRCODE
};

VISION_PROCESS visionState = IDLE;

void parseTask(const pap_common::TaskConstPtr& taskMsg);

int main(int argc, char **argv) {
	ros::init(argc, argv, "image_simulator");
	ros::NodeHandle n;

	image_transport::ImageTransport it_(n);

	image_transport::Publisher qr_image_pub_;

	ros::Subscriber taskSubscriber_ = n.subscribe("task", 10, &parseTask);

	tf::TransformBroadcaster transformBr;
	unsigned int id_counter1 = 0;

	ros::Rate loop_rate(30);
	image_pub_ = it_.advertise("Camera1/image_raw", 10);
	image_pub_2 = it_.advertise("Camera2/image_raw", 10);
	//qr_image_pub_ = it_.advertise("image", 10);

	while (ros::ok()) {
		cv::Mat input;
		switch (visionState) {
		case IDLE:
			input = cv::imread(string(getenv("PAPRESOURCES")) + "images/Chip.png");
			break;

		case CHIP:
			input = cv::imread(string(getenv("PAPRESOURCES")) + "images/Chip.png");
			break;

		case TAPE:
			input = cv::imread(string(getenv("PAPRESOURCES")) + "images/Tape0402.png");
			break;

		case CIRCLE:
			input = cv::imread(string(getenv("PAPRESOURCES")) + "images/Chip.png");
			break;

		case PAD:
			input = cv::imread(string(getenv("PAPRESOURCES")) + "images/Pad.png");
			break;

		case QRCODE:
			input = cv::imread(string(getenv("PAPRESOURCES")) + "images/QRCode.png");
			break;

		}

		cv::Mat outputRGB;
		cvtColor(input, outputRGB, CV_BGR2RGB);
		std_msgs::Header header;
		header.seq = id_counter1;
		header.stamp = ros::Time::now();
		header.frame_id = "camera1";

		out_msg.header = header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
		out_msg.image = outputRGB;
		image_pub_.publish(out_msg.toImageMsg());

		id_counter1++;

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void parseTask(const pap_common::TaskConstPtr& taskMsg) {
	switch (taskMsg->destination) {
	case pap_common::VISION:

		switch (taskMsg->task) {

		case pap_vision::START_CHIP_FINDER:
			visionState = CHIP;
			break;

		case pap_vision::START_SMALL_FINDER:
			visionState = SMALL_SMD;
			break;

		case pap_vision::START_TAPE_FINDER:

			visionState = TAPE;
			break;

		case pap_vision::START__QRCODE_FINDER:
			visionState = QRCODE;
			break;

		case pap_vision::START_PAD_FINDER:
			visionState = PAD;

			break;

		case pap_vision::SEARCH_CIRCLE:
			visionState = CIRCLE;
			break;
		}
		break;
	}
}
