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
char key;
using namespace std;
using namespace cv;

int main(int argc, char **argv) {
	ros::init(argc, argv, "add_two_ints_server");
	ros::NodeHandle n;
	image_transport::ImageTransport it_(n);
	image_transport::Publisher image_pub_;
	ros::Rate loop_rate(25);
	padFinder finder;
	image_pub_ = it_.advertise("camera1", 1);

	//CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	//CvCapture* capture = cvCaptureFromCAM(1);
	int id_counter = 0;

	while (ros::ok()) {
		//IplImage* frame = cvQueryFrame(capture); //Create image frames from capture
		id_counter++;
		cv_bridge::CvImage out_msg;
		//cv::Mat input(frame);

		// Pads
		//cv::Mat input = cv::imread("/home/johan/Desktop/Webcam_Pictures/Webcam-1435326531.png");
		//finder.findPads(&input);

		// Chip
		//cv::Mat input = cv::imread("/home/johan/Desktop/Webcam_Pictures/Webcam-1435311766.png");
		//finder.findChip(&input);

		// SMD Chip
		//cv::Mat input = cv::imread("/home/johan/Desktop/Webcam_Pictures/Webcam-1435326204.png");
		//cv::Mat input = cv::imread("/home/johan/Desktop/Webcam_Pictures/Webcam-1435326387.png");
		//finder.findSmallSMD(&input);

		// SMD Tape
		cv::Mat input = cv::imread("/home/johan/Desktop/Webcam_Pictures/Webcam-1435327178.png");
		finder.findSMDTape(&input);

		// Fadenkreuz
		Point2f vertices[4];
		vertices[0] = Point2f(input.cols/2-1,0);
		vertices[1] = Point2f(input.cols/2-1,input.rows-1);
		vertices[2] = Point2f(input.cols-1,input.rows/2-1);
		vertices[3] = Point2f(0,input.rows/2-1);
		line(input, vertices[1], vertices[0],Scalar(0, 0, 255),3);
		line(input, vertices[3], vertices[2],Scalar(0, 0, 255),3);
		circle(input,Point2f(input.cols/2-1,input.rows/2-1),20,CV_RGB(255,0,0),3);

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
