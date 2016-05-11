#ifndef PADFINDER_H
#define PADFINDER_H

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "sensor_msgs/Image.h"
#include <pap_common/vision_message_def.h>
#include "std_msgs/Header.h"
#include <vector>
#include <cmath>
#include "zbar.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <dlib/svm.h>
typedef dlib::matrix<double, 3, 1> sample_type;

// Linear SVM
typedef dlib::linear_kernel<sample_type> kernel_type_lin;
typedef dlib::decision_function<kernel_type_lin> lin_func_type;
typedef dlib::normalized_function<lin_func_type> lin_funct_type;

// Nonlinear SVM

typedef dlib::radial_basis_kernel<sample_type> kernel_type;

typedef dlib::decision_function<kernel_type> dec_funct_type;
typedef dlib::normalized_function<dec_funct_type> funct_type;

// Probabilistic SVM
typedef dlib::probabilistic_decision_function<kernel_type> probabilistic_funct_type;
typedef dlib::normalized_function<probabilistic_funct_type> pfunct_type;

// Conversion factors for pixel to mm
// Top-Camera:
#define PIXEL_TO_MM_TOP 32.37
// Bottom-Camera
#define PIXEL_TO_MM_BOTTOM 24.95

#define PIXEL_TO_MM_PCB 31.1463

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
    padFinder();
    ~padFinder();

    cv::Mat classifyPixels(const cv::Mat &in);

    cv::Point2f findPads(cv::Mat* input, bool startSelect, cv::Point2f selectPad, std::vector<cv::RotatedRect> &pads);
    bool findChipAvg(std::vector<cv::Mat> *input, enum pap_vision::CAMERA_SELECT camera_select, smdPart &chip);
    bool findChip(cv::Mat* input, unsigned int camera_select, smdPart &part_out);
    bool findSMDTapeAvg(std::vector<cv::Mat> *input, bool searchTapeRotation, smdPart &out);
    bool findSMDTape(cv::Mat &final, bool searchTapeRotation, smdPart &out);
    bool findTipAvg(std::vector<cv::Mat> *input, enum pap_vision::CAMERA_SELECT camera_select, smdPart &tip);
    bool findTip(cv::Mat &final, smdPart &out);

    void appendImage(cv::Mat image, cv::Point3d coord);
    void saveStitchingImages();
    bool  getPixelConvVal(cv::Mat &picture, double &pxRatio);
    bool getPixelConvValAvg(std::vector<cv::Mat> *pictures, double &pxRatio);
    bool scanCalibrationQRCode(cv::Mat &picture, double &width, double &height, double &truth_len);

	double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
	void setLabel(cv::Mat& im, const std::string label,
			std::vector<cv::Point>& contour);
	void drawRotatedRect(cv::Mat& image, cv::RotatedRect rRect,
			cv::Scalar color);
    bool isBorderTouched(cv::RotatedRect pad, cv::Size mat_size);
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
    float pxRatioSlot,pxRatioTape,pxRatioPcb,pxRatioBottom;
private:
    // privat
    lin_funct_type learned_funct_;
    zbar::ImageScanner scanner;
    int m_eineVariable;
    bool foundVia;
    bool repeat;
    float partWidth_,partHeight_;
    std::vector<std::pair<cv::Mat, cv::Point3d> > stitching_data_;

};

#endif // PADFINDER_H
