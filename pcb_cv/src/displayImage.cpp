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
#include "zbar.h"

char key;
using namespace std;
using namespace cv;
using namespace zbar;
void imageCallback1(const sensor_msgs::ImageConstPtr& msg);
void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
void parseTask(const pap_common::TaskConstPtr& taskMsg);

enum VISION_PROCESS {
	CHIP, SMALL_SMD, TAPE, PAD, IDLE, CIRCLE, QRCODE
};

enum VISION_QR_CALIBRATION {
	NO_CAL, TOP_SLOT, TOP_TAPE, TOP_PCB, BOTTOM_CAM
};

VISION_PROCESS visionState = IDLE;
VISION_QR_CALIBRATION qrCalAction = NO_CAL;
bool visionEnabled, selectPad = false;
bool searchTapeRotation = false;
ros::Publisher statusPublisher;
padFinder finder;
unsigned int cameraSelect;
cv::Point2f selectPoint;

CvCapture* capture;
CvCapture* capture2;

unsigned int id_counter1, id_counter2 = 0;
image_transport::Publisher image_pub_;
image_transport::Publisher image_pub_2;

int calibrationIteration = 0;
float pxRatioSlot = 0;
float pxRatioTape = 0;
float pxRatioPcb = 0;
float pxRatioBottom = 0;
ImageScanner scanner;

int main(int argc, char **argv) {
	ros::init(argc, argv, "add_two_ints_server");
	ros::NodeHandle n;

	image_transport::ImageTransport it_(n);

	image_transport::Publisher qr_image_pub_;

	ros::Subscriber taskSubscriber_ = n.subscribe("task", 10, &parseTask);
	statusPublisher = n.advertise<pap_common::VisionStatus>("visionStatus",
			1000);

	image_transport::Subscriber camera1sub = it_.subscribe("Camera1/image_raw",
			2, imageCallback1);
	image_transport::Subscriber camera2sub = it_.subscribe("Camera2/image_raw",
			2, imageCallback2);
	tf::TransformBroadcaster transformBr;

	ros::Rate loop_rate(10);
	image_pub_ = it_.advertise("camera1", 10);
	image_pub_2 = it_.advertise("camera2", 10);
	qr_image_pub_ = it_.advertise("image", 10);

	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

	while (ros::ok()) {
		ros::spinOnce();
		//loop_rate.sleep();
	}
}

void imageCallback1(const sensor_msgs::ImageConstPtr& msg) {

	id_counter1++;
	cv::Mat input;
	cv_bridge::CvImage out_msg;
	smdPart smd;
	pap_common::VisionStatus visionMsg;
	cv::Point2f position;

	try {
		input = cv_bridge::toCvShare(msg, "bgr8")->image;
		//cv::imshow("view", input);
		//cv::waitKey(30);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
				msg->encoding.c_str());
		return;
	}

	if (visionEnabled) {
		switch (visionState) {
		case IDLE:

			break;

		case QRCODE: {
			if (cameraSelect == CAMERA_TOP) {
				cv::Mat gray;
				std::vector<smdPart> tipObjects;
				cv::cvtColor(input, gray, CV_BGR2GRAY);
				cv::Size newSize;
				newSize.width = 640;
				newSize.height = 480;
				cv::resize(gray, gray, newSize);
				cv::resize(input, input, newSize);
				int width = gray.cols;
				int height = gray.rows;
				uchar *raw = (uchar *) gray.data;
				// wrap image data
				Image image(width, height, "Y800", raw, width * height);
				int n = scanner.scan(image);

				for (Image::SymbolIterator symbol = image.symbol_begin();
						symbol != image.symbol_end(); ++symbol) {
					vector<Point> vp;
					// do something useful with results
					cout << "decoded " << symbol->get_type_name()
							<< " symbol \"" << symbol->get_data() << '"' << " "
							<< endl;
					int n = symbol->get_location_size();
					for (int i = 0; i < n; i++) {
						vp.push_back(
								Point(symbol->get_location_x(i),
										symbol->get_location_y(i)));
					}

					std::string calName = symbol->get_data();
					std::string number;
					ROS_INFO("calNAME length: %d", calName.size());
					if (calName.size() == 13) {
						number = calName.substr(calName.size() - 1);
					} else if (calName.size() == 14) {
						number = calName.substr(calName.size() - 2);
					}

					float calNumber = std::atoi(number.c_str());
					ROS_INFO("Cal-Value : %f", calNumber);

					RotatedRect r = minAreaRect(vp);
					Point2f pts[4];
					r.points(pts);
					for (int i = 0; i < 4; i++) {
						line(input, pts[i], pts[(i + 1) % 4], Scalar(255, 0, 0),
								1);
					}

					switch (qrCalAction) {
					case TOP_SLOT:
						calibrationIteration += 1;
						pxRatioSlot += (r.size.width + r.size.height) / 2.0;
						if (calibrationIteration == 20) {
							pxRatioSlot = pxRatioSlot / 20.0;
							pxRatioSlot /= calNumber;
							ROS_INFO("Slot Pixel Ratio : %f", pxRatioSlot);
							finder.setPixelRatioSlot(pxRatioSlot);
							pxRatioSlot = 0;
							calibrationIteration = 0;
							visionMsg.task = pap_vision::START__QRCODE_FINDER;
							statusPublisher.publish(visionMsg);
							qrCalAction = NO_CAL;
						}
						break;

					case TOP_PCB:
						calibrationIteration += 1;
						pxRatioPcb += (r.size.width + r.size.height) / 2.0;
						;
						if (calibrationIteration == 20) {
							pxRatioPcb = pxRatioPcb / 20.0;
							pxRatioPcb /= calNumber;
							ROS_INFO("Pcb Pixel Ratio : %f", pxRatioPcb);
							finder.setPixelRatioPcb(pxRatioPcb);
							pxRatioPcb = 0;
							calibrationIteration = 0;
							visionMsg.task = pap_vision::START__QRCODE_FINDER;
							statusPublisher.publish(visionMsg);
							qrCalAction = NO_CAL;
						}
						break;

					case TOP_TAPE:
						calibrationIteration += 1;
						pxRatioTape += (r.size.width + r.size.height) / 2.0;
						;
						if (calibrationIteration == 20) {
							pxRatioTape = pxRatioTape / 20.0;
							pxRatioTape /= calNumber;
							ROS_INFO("Tape Pixel Ratio : %f", pxRatioTape);
							finder.setPixelRatioTape(pxRatioTape);
							pxRatioTape = 0;
							calibrationIteration = 0;
							visionMsg.task = pap_vision::START__QRCODE_FINDER;
							statusPublisher.publish(visionMsg);
							qrCalAction = NO_CAL;
						}
						break;

					case NO_CAL:
						break;
					}
					cout << "Angle: " << r.angle << endl;
				}
			}
		}
			break;

		case CHIP:
			// Chip

			if (cameraSelect == CAMERA_TOP) {
				smd = finder.findChip(&input, cameraSelect);

				if (smd.x != 0.0 && smd.y != 0.0) {
					visionMsg.task = pap_vision::START_CHIP_FINDER;
					visionMsg.data1 = smd.y;
					visionMsg.data2 = smd.x;
					visionMsg.data3 = smd.rot;
					visionMsg.camera = cameraSelect;
					//ROS_INFO("X %f, Y %f", smd.x, smd.y);
					statusPublisher.publish(visionMsg);
				}
			}

			break;
		case SMALL_SMD:
			// SMD Chip
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
			smd = finder.findSMDTape(&input, searchTapeRotation);
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
		}
	} else {
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

	// Camera 1

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
}

void imageCallback2(const sensor_msgs::ImageConstPtr& msg) {
	cv::Mat input2;
	try {
		input2 = cv_bridge::toCvShare(msg, "bgr8")->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
				msg->encoding.c_str());
		return;
	}
	id_counter2++;
	cv_bridge::CvImage out_msg2;
	smdPart smd;
	pap_common::VisionStatus visionMsg;
	cv::Point2f position;
	if (visionEnabled) {
		switch (visionState) {
		case IDLE:
			break;

		case CHIP:
			// Chip

			if (cameraSelect == CAMERA_BOTTOM) {
				smd = finder.findChip(&input2, cameraSelect);
				if (smd.x != 0.0 && smd.y != 0.0) {
					visionMsg.task = pap_vision::START_CHIP_FINDER;
					visionMsg.data1 = smd.y;
					visionMsg.data2 = smd.x;
					visionMsg.data3 = smd.rot;
					visionMsg.camera = cameraSelect;
					//ROS_INFO("X %f, Y %f", smd.x, smd.y);
					statusPublisher.publish(visionMsg);
				}
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

		case QRCODE:
			if (cameraSelect == CAMERA_BOTTOM) {
				cv::Mat gray;
				std::vector<smdPart> tipObjects;
				cv::cvtColor(input2, gray, CV_BGR2GRAY);
				cv::Size newSize;
				newSize.width = 640;
				newSize.height = 480;
				cv::resize(gray, gray, newSize);
				cv::resize(input2, input2, newSize);
				int width = gray.cols;
				int height = gray.rows;
				uchar *raw = (uchar *) gray.data;
				// wrap image data
				Image image(width, height, "Y800", raw, width * height);
				int n = scanner.scan(image);

				for (Image::SymbolIterator symbol = image.symbol_begin();
						symbol != image.symbol_end(); ++symbol) {
					vector<Point> vp;
					// do something useful with results
					cout << "decoded " << symbol->get_type_name()
							<< " symbol \"" << symbol->get_data() << '"' << " "
							<< endl;
					int n = symbol->get_location_size();
					for (int i = 0; i < n; i++) {
						vp.push_back(
								Point(symbol->get_location_x(i),
										symbol->get_location_y(i)));
					}
					RotatedRect r = minAreaRect(vp);
					Point2f pts[4];
					r.points(pts);
					for (int i = 0; i < 4; i++) {
						line(input2, pts[i], pts[(i + 1) % 4],
								Scalar(255, 0, 0), 1);
					}

					std::string calName = symbol->get_data();
					std::string number;
					ROS_INFO("calNAME length: %d", calName.size());
					if (calName.size() == 13) {
						number = calName.substr(calName.size() - 1);
					} else if (calName.size() == 14) {
						number = calName.substr(calName.size() - 2);
					}

					float calNumber = std::atoi(number.c_str());
					ROS_INFO("Cal-Value : %f", calNumber);

					switch (qrCalAction) {
					case BOTTOM_CAM:
						calibrationIteration += 1;
						pxRatioBottom += (r.size.height + r.size.width) / 2.0;
						if (calibrationIteration == 20) {
							pxRatioBottom = pxRatioBottom / 20.0;
							pxRatioBottom = pxRatioBottom / calNumber;
							ROS_INFO("Bottom Pixel Ratio : %f", pxRatioBottom);
							finder.setPixelRatioBottom(pxRatioBottom);
							pxRatioBottom = 0;
							calibrationIteration = 0;
							visionMsg.task = pap_vision::START__QRCODE_FINDER;
							statusPublisher.publish(visionMsg);
							qrCalAction = NO_CAL;
						}
						break;

					case NO_CAL:
						break;
					}
					cout << "Angle: " << r.angle << endl;
				}
			}
			break;

		}
	} else {
	}

	// Crosshairs
	Point2f vertices[4];
	vertices[0] = Point2f(input2.cols / 2 - 1, 0);
	vertices[1] = Point2f(input2.cols / 2 - 1, input2.rows - 1);
	vertices[2] = Point2f(input2.cols - 1, input2.rows / 2 - 1);
	vertices[3] = Point2f(0, input2.rows / 2 - 1);

	// Camera 2
	line(input2, vertices[1], vertices[0], Scalar(0, 0, 255), 2);
	line(input2, vertices[3], vertices[2], Scalar(0, 0, 255), 2);
	circle(input2, Point2f(input2.cols / 2 - 1, input2.rows / 2 - 1), 20,
			CV_RGB(255, 0, 0), 2);

	// Camera 2

	cv::Mat outputRGB2;
	cvtColor(input2, outputRGB2, CV_BGR2RGB);
	std_msgs::Header header2;
	header2.seq = id_counter2;
	header2.stamp = ros::Time::now();
	header2.frame_id = "camera2";

	out_msg2.header = header2; // Same timestamp and tf frame as input image
	out_msg2.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
	out_msg2.image = outputRGB2;
	image_pub_2.publish(out_msg2.toImageMsg());
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
			finder.setSize(taskMsg->data1 * 1.2, taskMsg->data2 * 1.15);
			if (taskMsg->data3 == 1.0) {
				searchTapeRotation = true;
			} else {
				searchTapeRotation = false;
			}
			visionState = TAPE;
			break;

		case pap_vision::START__QRCODE_FINDER:
			qrCalAction = (VISION_QR_CALIBRATION) taskMsg->data1;
			cameraSelect = taskMsg->data2;
			visionState = QRCODE;
			break;

			// This state is for manually selecting of the fiducials
		case pap_vision::START_PAD_FINDER:
			visionState = PAD;
			if (taskMsg->data1 == 1.0) {
				selectPad = true;
				selectPoint.x = taskMsg->data2;
				selectPoint.y = taskMsg->data3;
				//ROS_INFO("Pixel %f %f", selectPoint.x, selectPoint.y);
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
