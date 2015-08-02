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
#include "../include/pcb_cv/padFinder.h"
#include <cmath>
#include <image_transport/image_transport.h>
#include "pap_common/Task.h"
#include "pap_common/VisionStatus.h"
#include "../../pap_common/include/pap_common/vision_message_def.h"
#include "../../pap_common/include/pap_common/task_message_def.h"
#include <tf/transform_broadcaster.h>

// Switch for simulation or gathering data from usb camera
//#define SIMULATION

char key;
using namespace std;
using namespace cv;

void parseTask(const pap_common::TaskConstPtr& taskMsg);

enum VISION_PROCESS {
	CHIP, SMALL_SMD, TAPE, PAD, IDLE, CIRCLE, QRCODE
};

VISION_PROCESS visionState = IDLE;
bool visionEnabled, selectPad = false;
ros::Publisher statusPublisher;
padFinder finder;
unsigned int cameraSelect;
cv::Point2f selectPoint;

int main(int argc, char **argv) {
	ros::init(argc, argv, "add_two_ints_server");
	ros::NodeHandle n;

	image_transport::ImageTransport it_(n);
	image_transport::Publisher image_pub_;
	image_transport::Publisher image_pub_2;
	image_transport::Publisher qr_image_pub_;

	ros::Subscriber taskSubscriber_ = n.subscribe("task", 10, &parseTask);
	statusPublisher = n.advertise<pap_common::VisionStatus>("visionStatus",
			1000);
	tf::TransformBroadcaster transformBr;

	ros::Rate loop_rate(15);
	image_pub_ = it_.advertise("camera1", 10);
	image_pub_2 = it_.advertise("camera2", 10);
	qr_image_pub_ = it_.advertise("image", 10);

	//CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
#ifndef SIMULATION
	CvCapture* capture = cvCaptureFromCAM(1);
	CvCapture* capture2 = cvCaptureFromCAM(2);
	//CvCapture* capture3 = cvCaptureFromCAM(3);
#endif

	int id_counter = 0;

	while (ros::ok()) {

#ifndef SIMULATION
		IplImage* frame = cvQueryFrame(capture); //Create image frames from capture
		IplImage* frame2 = cvQueryFrame(capture2);//Create image frames from capture2
		//IplImage* frame3 = cvQueryFrame(capture3);//Create image frames from capture2

		cv::Mat input(frame);
		cv::Mat input2(frame2);
		//cv::Mat input3(frame3);

#else
		cv::Mat input;
		cv::Mat input2;
		//cv::Mat input3;
#endif
		id_counter++;
		cv_bridge::CvImage out_msg;
		cv_bridge::CvImage out_msg2;
		cv_bridge::CvImage out_msg3;
		smdPart smd;
		pap_common::VisionStatus visionMsg;
		cv::Point2f position;
		if (visionEnabled) {
			switch (visionState) {
			case IDLE:
#ifdef SIMULATION
				input =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435326531.png");

				input2 =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435326531.png");
#endif
				break;

			case QRCODE:
				// Publish image
				//qr_image_pub_.publish(out_msg3.toImageMsg());
				break;

			case CHIP:
				// Chip
#ifdef SIMULATION
				input =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435311766.png");
				input2 =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435311766.png");

#endif

				if (cameraSelect == CAMERA_TOP) {
					smd = finder.findChip(&input, cameraSelect);
				} else if (cameraSelect == CAMERA_BOTTOM) {
					smd = finder.findChip(&input2, cameraSelect);
				}

				if (smd.x != 0.0 && smd.y != 0.0) {
					visionMsg.task = pap_vision::START_CHIP_FINDER;
					visionMsg.data1 = smd.y;
					visionMsg.data2 = smd.x;
					visionMsg.data3 = smd.rot;
					visionMsg.camera = cameraSelect;
					ROS_INFO("X %f, Y %f", smd.x, smd.y);
					statusPublisher.publish(visionMsg);
				}

				break;
			case SMALL_SMD:
				// SMD Chip
#ifdef SIMULATION
				input =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435326387.png");
				input2 =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435326387.png");
#endif
				smd = finder.findSmallSMD(&input);
				if (smd.x != 0.0 && smd.y != 0.0) {
					visionMsg.task = pap_vision::START_SMALL_FINDER;
					visionMsg.data1 = smd.x;
					visionMsg.data2 = smd.y;
					visionMsg.data3 = smd.rot;
					visionMsg.camera = 0;
					statusPublisher.publish(visionMsg);
				}
				break;
			case TAPE:
				// SMD Tape
#ifdef SIMULATION
				input =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435327178.png");
				input2 =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435327178.png");
#endif
				smd = finder.findSMDTape(&input);
				if (smd.x != 0.0 && smd.y != 0.0) {
					visionMsg.task = pap_vision::START_TAPE_FINDER;
					visionMsg.data1 = smd.y;
					visionMsg.data2 = smd.x;
					visionMsg.data3 = smd.rot;
					visionMsg.camera = 0;
					statusPublisher.publish(visionMsg);
				}
				break;
			case PAD:
				// Pads
#ifdef SIMULATION
				input =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435326531.png");
				input2 =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435326531.png");
#endif
				position = finder.findPads(&input, selectPad, selectPoint);
				//ROS_INFO("X %f Y  %f", position.x, position.y);
				if (selectPad && position.x != 0.0 && position.y != 0.0) {
					visionMsg.task = pap_vision::START_PAD_FINDER;
					visionMsg.data1 = position.y;
					visionMsg.data2 = position.x;
					visionMsg.data3 = 0.0;
					visionMsg.camera = 0;
					statusPublisher.publish(visionMsg);
				}
				break;

			case CIRCLE:
				smd = finder.findTip(&input2);
				if (smd.x != 0.0 && smd.y != 0.0) {
					visionMsg.task = pap_vision::SEARCH_CIRCLE;
					visionMsg.data1 = -smd.y;
					visionMsg.data2 = smd.x;
					visionMsg.data3 = smd.rot;
					visionMsg.camera = 1;
					statusPublisher.publish(visionMsg);
				}
				break;
				break;

			}
		} else {
#ifdef SIMULATION

			input =
					cv::imread(
							"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435326531.png");

			input2 =
					cv::imread(
							"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435326531.png");

#endif
		}

		// Crosshairs
		Point2f vertices[4];
		vertices[0] = Point2f(input.cols / 2 - 1, 0);
		vertices[1] = Point2f(input.cols / 2 - 1, input.rows - 1);
		vertices[2] = Point2f(input.cols - 1, input.rows / 2 - 1);
		vertices[3] = Point2f(0, input.rows / 2 - 1);

		// Camera 1
		line(input, vertices[1], vertices[0], Scalar(0, 0, 255), 2);
		line(input, vertices[3], vertices[2], Scalar(0, 0, 255), 2);
		circle(input, Point2f(input.cols / 2 - 1, input.rows / 2 - 1), 20,
				CV_RGB(255, 0, 0), 2);

		// Camera 2
		line(input2, vertices[1], vertices[0], Scalar(0, 0, 255), 2);
		line(input2, vertices[3], vertices[2], Scalar(0, 0, 255), 2);
		circle(input2, Point2f(input2.cols / 2 - 1, input2.rows / 2 - 1), 20,
				CV_RGB(255, 0, 0), 2);

		// Convert image to standard msgs format in order to send the image
		// over the ros topics

		// Camera 1

		cv::Mat outputRGB;
		cvtColor(input, outputRGB, CV_BGR2RGB);

		std_msgs::Header header;
		header.seq = id_counter;
		header.stamp = ros::Time::now();
		header.frame_id = "camera1";

		out_msg.header = header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
		out_msg.image = outputRGB;
		image_pub_.publish(out_msg.toImageMsg());

		tf::Transform transform;
		transform.setOrigin(tf::Vector3(0.3, 0.1, 0.05));
		tf::Quaternion rotQuat;
		rotQuat.setEuler(0.0, 0.0, 0.0);
		transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		transform.setRotation(rotQuat);

		transformBr.sendTransform(
				tf::StampedTransform(transform, ros::Time::now(), "/world",
						"/renderedPCB"));

		// Camera 2

		cv::Mat outputRGB2;
		cvtColor(input2, outputRGB2, CV_BGR2RGB);
		std_msgs::Header header2;
		header2.seq = id_counter;
		header2.stamp = ros::Time::now();
		header2.frame_id = "camera2";

		out_msg2.header = header2; // Same timestamp and tf frame as input image
		out_msg2.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
		out_msg2.image = outputRGB2;
		image_pub_2.publish(out_msg2.toImageMsg());

		//cvReleaseCapture(&capture); //Release capture.
		//cvReleaseCapture(&capture2); //Release capture.
		//cvDestroyWindow("Camera_Output"); //Destroy Window
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void parseTask(const pap_common::TaskConstPtr& taskMsg) {
	switch (taskMsg->destination) {
	case pap_common::VISION:

		switch (taskMsg->task) {
		case pap_vision::START_VISION:
			visionEnabled = true;
			visionState = IDLE;
			break;

		case pap_vision::STOP_VISION:
			visionEnabled = false;
			visionState = IDLE;
			break;

		case pap_vision::START_CHIP_FINDER:
			finder.setSize(taskMsg->data1, taskMsg->data2);
			cameraSelect = taskMsg->data3;
			visionState = CHIP;
			break;

		case pap_vision::START_SMALL_FINDER:
			finder.setSize(taskMsg->data1, taskMsg->data2);
			visionState = SMALL_SMD;
			break;

		case pap_vision::START_TAPE_FINDER:
			finder.setSize(taskMsg->data1, taskMsg->data2);
			ROS_INFO("Setting size....");
			visionState = TAPE;
			break;

		case pap_vision::START__QRCODE_FINDER:
			visionState = QRCODE;
			break;

			// This state is for manually selecting of the fiducials
		case pap_vision::START_PAD_FINDER:
			visionState = PAD;
			if (taskMsg->data1 == 1.0) {
				selectPad = true;
				selectPoint.x = taskMsg->data2;
				selectPoint.y = taskMsg->data3;
				ROS_INFO("Pixel %f %f", selectPoint.x, selectPoint.y);
			} else {
				selectPad = false;
				selectPoint.x = 0.0;
				selectPoint.y = 0.0;
			}

			break;

		case pap_vision::SEARCH_CIRCLE:
			finder.setSize(taskMsg->data1, taskMsg->data2);
			visionState = CIRCLE;
			break;
		}

		break;
	}
}
