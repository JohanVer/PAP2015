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

	padFinder finder;
	image_pub_ = it_.advertise("camera1", 1);

	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	int id_counter = 0;

	while (ros::ok()) { //Create infinte loop for live streaming
		IplImage* frame = cvQueryFrame(capture); //Create image frames from capture
		id_counter++;
		cv_bridge::CvImage out_msg;
		cv::Mat input(frame);
		//cv::Mat input = cv::imread("/home/johan/Desktop/2015-03-27-200049.jpg");
		cv::Mat outputRGB;
		//finder.findPads(input);

		cvtColor(input, outputRGB, CV_BGR2RGB);
		std_msgs::Header header;
		header.seq = id_counter + 1;
		header.stamp = ros::Time::now();
		header.frame_id = "camera1";

		out_msg.header = header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
		out_msg.image = outputRGB; // Your cv::Mat

		image_pub_.publish(out_msg.toImageMsg());
		ros::Duration(0.033).sleep();

		//cvReleaseCapture(&capture); //Release capture.
		//cvDestroyWindow("Camera_Output"); //Destroy Window
		//ros::spin();
	}
}
