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
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"
#include <vector>
#include <cmath>
#include "zbar.h"


// Conversion factors for pixel to mm
// Top-Camera:
#define PIXEL_TO_MM_TOP 32.37
// Bottom-Camera
#define PIXEL_TO_MM_BOTTOM 24.95

#define PIXEL_TO_MM_PCB 33.63

enum CAMERA_SELECT {
	CAMERA_TOP, CAMERA_BOTTOM
};

class smdPart {
public:
	float x;
	float y;
	float rot;
	smdPart(){
		x = 0.0;
		y = 0.0;
		rot = 0.0;
	}
};

class padFinder {
public:
	// öffentlich
	padFinder();                      // der Default-Konstruktor
	~padFinder();                     // der Destruktor

	//std::vector<cv::Point2f > findPads(cv::Mat* input);        // eine Funktion mit einem (Default-) Parameter
	cv::Point2f findPads(cv::Mat* input,bool startSelect,cv::Point2f selectPad);
	smdPart findChip(cv::Mat* input, unsigned int camera_select);
	smdPart findSmallSMD(cv::Mat* input);
	smdPart findSMDTape(cv::Mat* input,bool searchTapeRotation);
	smdPart findTip(cv::Mat* input);

    double getPixelConvVal(cv::Mat &picture, size_t num_averages);
    bool scanCalibrationQRCode(cv::Mat &picture, double &width, double &height, double &truth_len);

	double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
	void setLabel(cv::Mat& im, const std::string label,
			std::vector<cv::Point>& contour);
	void drawRotatedRect(cv::Mat& image, cv::RotatedRect rRect,
			cv::Scalar color);
	bool isBorderTouched(cv::RotatedRect pad);
	bool nearestPart(std::vector<smdPart>* list, smdPart* partDst, int width,
			int height);
	void setSize(float width, float height);

	void setPixelRatioSlot(float value){
		pxRatioSlot = value;
	}
	void setPixelRatioTape(float value){
		pxRatioTape = value;
	}
	void setPixelRatioPcb(float value){
		pxRatioPcb = value;
	}
	void setPixelRatioBottom(float value){
		pxRatioBottom = value;
	}
private:
	// privat
    zbar::ImageScanner scanner;
	int m_eineVariable;
	bool foundVia;
	bool repeat;
	float partWidth_,partHeight_;

	float pxRatioSlot,pxRatioTape,pxRatioPcb,pxRatioBottom;
};
