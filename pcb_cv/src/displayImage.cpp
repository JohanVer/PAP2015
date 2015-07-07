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

char key;
using namespace std;
using namespace cv;

void parseTask(const pap_common::TaskConstPtr& taskMsg);

enum VISION_PROCESS {
	CHIP, SMALL_SMD, TAPE, PAD, IDLE
};

VISION_PROCESS visionState = IDLE;
bool visionEnabled, selectPad = false;
ros::Publisher statusPublisher;
padFinder finder;
cv::Point2f selectPoint;

int main(int argc, char **argv) {
	ros::init(argc, argv, "add_two_ints_server");
	ros::NodeHandle n;
	image_transport::ImageTransport it_(n);
	image_transport::Publisher image_pub_;

	ros::Subscriber taskSubscriber_ = n.subscribe("task", 1, &parseTask);
	statusPublisher = n.advertise<pap_common::VisionStatus>("visionStatus",
			1000);

	ros::Rate loop_rate(25);
	image_pub_ = it_.advertise("camera1", 1);

	//CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	//CvCapture* capture = cvCaptureFromCAM(1);
	int id_counter = 0;

	while (ros::ok()) {
		//IplImage* frame = cvQueryFrame(capture); //Create image frames from capture
		//cv::Mat input(frame);
		cv::Mat input;
		id_counter++;
		cv_bridge::CvImage out_msg;
		smdPart smd;
		pap_common::VisionStatus visionMsg;
		if (visionEnabled) {
			switch (visionState) {
			case IDLE:
				input =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435326531.png");
				break;
			case CHIP:
				// Chip
				input =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435311766.png");
				smd = finder.findChip(&input);
				if (smd.x != 0.0 && smd.y != 0.0) {
					visionMsg.task = pap_vision::START_CHIP_FINDER;
					visionMsg.data1 = smd.x;
					visionMsg.data2 = smd.y;
					visionMsg.data3 = smd.rot;
					statusPublisher.publish(visionMsg);
				}
				break;
			case SMALL_SMD:
				// SMD Chip
				input =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435326387.png");
				smd = finder.findSmallSMD(&input);
				if (smd.x != 0.0 && smd.y != 0.0) {
					visionMsg.task = pap_vision::START_SMALL_FINDER;
					visionMsg.data1 = smd.x;
					visionMsg.data2 = smd.y;
					visionMsg.data3 = smd.rot;
					statusPublisher.publish(visionMsg);
				}
				break;
			case TAPE:
				// SMD Tape
				input =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435327178.png");
				smd = finder.findSMDTape(&input);
				if (smd.x != 0.0 && smd.y != 0.0) {
					visionMsg.task = pap_vision::START_TAPE_FINDER;
					visionMsg.data1 = smd.x;
					visionMsg.data2 = smd.y;
					visionMsg.data3 = smd.rot;
					statusPublisher.publish(visionMsg);
				}
				break;
			case PAD:
				// Pads
				input =
						cv::imread(
								"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435326531.png");
				cv::Point2f position = finder.findPads(&input, selectPad,
						selectPoint);
				ROS_INFO("X %f Y  %f",position.x,position.y);
				if (selectPad && position.x != 0.0 && position.y != 0.0) {
					visionMsg.task = pap_vision::START_PAD_FINDER;
					visionMsg.data1 = position.x;
					visionMsg.data2 = position.y;
					visionMsg.data3 = 0.0;
					statusPublisher.publish(visionMsg);
				}
				break;

			}
		} else {
			input =
					cv::imread(
							"/home/nikolas/Desktop/Webcam_Pictures/Webcam-1435326531.png");
		}

		// Fadenkreuz
		Point2f vertices[4];
		vertices[0] = Point2f(input.cols / 2 - 1, 0);
		vertices[1] = Point2f(input.cols / 2 - 1, input.rows - 1);
		vertices[2] = Point2f(input.cols - 1, input.rows / 2 - 1);
		vertices[3] = Point2f(0, input.rows / 2 - 1);
		line(input, vertices[1], vertices[0], Scalar(0, 0, 255), 3);
		line(input, vertices[3], vertices[2], Scalar(0, 0, 255), 3);
		circle(input, Point2f(input.cols / 2 - 1, input.rows / 2 - 1), 20,
				CV_RGB(255, 0, 0), 3);

		// Convert image to standard msgs format
		cv::Mat outputRGB;
		cvtColor(input, outputRGB, CV_BGR2RGB);
		std_msgs::Header header;
		header.seq = id_counter + 1;
		header.stamp = ros::Time::now();
		header.frame_id = "camera1";

		out_msg.header = header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
		out_msg.image = outputRGB;

		image_pub_.publish(out_msg.toImageMsg());

		//cvReleaseCapture(&capture); //Release capture.
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
		}

		break;
	}
}
