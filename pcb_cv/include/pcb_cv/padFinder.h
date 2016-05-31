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
#include <ros/package.h>

// SVM definitions that are used in pad-detection

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
#define PIXEL_TO_MM_TOP 33.1339

#define PIXEL_TO_MM_TAPE 31.9609
// Bottom-Camera
#define PIXEL_TO_MM_BOTTOM 40.6267

#define PIXEL_TO_MM_PCB 31.2509

class smdPart {
public:
	float x;
	float y;
	float rot;
    float width;
    float height;
	smdPart(){
		x = 0.0;
		y = 0.0;
		rot = 0.0;
        width = 0.0;
        height = 0.0;
	}
};

//!
//! \brief The padFinder class provides all vision algortihms that are used for the PAP project
//!
class padFinder {
public:
    padFinder();
    ~padFinder();

    //!
    //! \brief loadParams loads parameter from parameter server
    //!
    void loadParams();

    //!
    //! \brief saveOffsetsToFile saves all pixel factors to yaml file
    //! \return true if successfull
    //!
    bool saveOffsetsToFile();

    //!
    //! \brief classifyPixels uses a SVM to classify which pixels belong to pads. This function creates a binary image output.
    //! \param in image to classify
    //! \return classification result as binary image
    //!
    cv::Mat classifyPixels(const cv::Mat &in);

    //!
    //! \brief findPads searches all pads on a pcb on a given image.
    //! \param input input image
    //! \param startSelect if true the function checks if the point "selectPad" lies on any detected pad.
    //!  If this is the case the center position of this pad will be returned.
    //! \param selectPad position for intersect check (see line before)
    //! \param pads list of all detected pads
    //! \return center position of selected pad (see startSelect)
    //!
    cv::Point2f findPads(cv::Mat* input, bool startSelect, cv::Point2f selectPad, std::vector<cv::RotatedRect> &pads);

    //!
    //! \brief findChipAvg searches for a chip. This function takes the average position over all found chips in the input image list.
    //! \param input input image list
    //! \param camera_select selector to choose which camera should be used
    //! \param[out] chip found chip
    //! \return true if successfull
    //!
    bool findChipAvg(std::vector<cv::Mat> *input, enum pap_vision::CAMERA_SELECT camera_select, smdPart &chip);

    //!
    //! \brief findChip searches a chip on given image
    //! \param input input image
    //! \param camera_select selector to choose which camera should be used
    //! \param part_out found chip
    //! \return true if successfull
    //!
    bool findChip(cv::Mat* input, unsigned int camera_select, smdPart &part_out);

    //!
    //! \brief findSMDTapeAvg searches for smd parts in tape. This function calculates the average position over the positions found in the input image list.
    //! \param input input image list
    //! \param searchTapeRotation TODO
    //! \param out found tape part
    //! \return true if successfull
    //!
    bool findSMDTapeAvg(std::vector<cv::Mat> *input, bool searchTapeRotation, smdPart &out);

    //!
    //! \brief findSMDTape searches for smd parts in tape.
    //! \param final input image
    //! \param searchTapeRotation TODO
    //! \param out found smd part position
    //! \return true if successfull
    //!
    bool findSMDTape(cv::Mat &final, bool searchTapeRotation, smdPart &out);

    //!
    //! \brief findTipAvg searches for circles in the given image. This function calculates the average position over the positions found in the input image list.
    //! \param input input image list
    //! \param camera_select selector to choose which camera should be used
    //! \param tip found circle (tip)
    //! \param thresholding TODO
    //! \return true if successfull
    //!
    bool findTipAvg(std::vector<cv::Mat> *input, enum pap_vision::CAMERA_SELECT camera_select, smdPart &tip, bool thresholding);

    //!
    //! \brief findTip searches for circles in the given image.
    //! \param final input image
    //! \param out found circle (tip)
    //! \param thresholding TODO
    //! \return true if successfull
    //!
    bool findTip(cv::Mat &final, smdPart &out, bool thresholding);

    //!
    //! \brief appendImage appends an image / record_coord to stitching_data_
    //! \param image image to append
    //! \param coord corresponding record coordinate
    //!
    void appendImage(cv::Mat image, cv::Point3d coord);

    //!
    //! \brief saveStitchingImages saves all images in stitching_data_ on harddisk
    //!
    void saveStitchingImages();

    //!
    //! \brief getPixelConvVal scans a qr calibration code in order to get the pixel to mm conversion factor
    //! \param picture input image
    //! \param[out] pxRatio calculated pixel to mm factor
    //! \return true if successfull
    //!
    bool  getPixelConvVal(cv::Mat &picture, double &pxRatio);

    //!
    //! \brief getPixelConvValAvg scans a qr calibration code in order to get the pixel to mm conversion factor.
    //!  This functions works on a list of images and returns the average result.
    //! \param pictures input image list
    //! \param pxRatio[out] average factor result
    //! \return true if successfull
    //!
    bool getPixelConvValAvg(std::vector<cv::Mat> *pictures, double &pxRatio);

    //!
    //! \brief scanCalibrationQRCode scans a qr code
    //! \param picture input image
    //! \param width width of qr code
    //! \param height height of qr code
    //! \param truth_len ground truth side length which was encoded in qr code
    //! \return true if successfull
    //!
    bool scanCalibrationQRCode(cv::Mat &picture, double &width, double &height, double &truth_len);

	double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);

    //!
    //! \brief setLabel draws a label on a contour
    //! \param im inout image
    //! \param label label to write
    //! \param contour contour to label
    //!
	void setLabel(cv::Mat& im, const std::string label,
			std::vector<cv::Point>& contour);

    //!
    //! \brief drawRotatedRect draws a rotated rectangle
    //! \param image input/output image
    //! \param rRect rectangle to draw
    //! \param color color of rectangle
    //!
	void drawRotatedRect(cv::Mat& image, cv::RotatedRect rRect,
			cv::Scalar color);

    //!
    //! \brief isBorderTouched checks if rectangle touches the border of an image with a certain size
    //! \param pad rectangle to check
    //! \param mat_size size of the image
    //! \return true if the border is touched
    //!
    bool isBorderTouched(cv::RotatedRect pad, cv::Size mat_size);

    //!
    //! \brief nearestPart gets the nearest part of a list with respect to the center of the image.
    //! \param list list of parts
    //! \param partDst[] nearest part
    //! \param width width of image
    //! \param height height of image
    //! \return true if successfull
    //!
	bool nearestPart(std::vector<smdPart>* list, smdPart* partDst, int width,
			int height);

    //!
    //! \brief setSize sets the size of the part(chip, smd part, tip) to search for
    //! \param width width of part
    //! \param height height of part
    //!
	void setSize(float width, float height);

	void setPixelRatioSlot(float value){
		pxRatioSlot = value;
	}
	void setPixelRatioTape(float value){
		pxRatioTape = value;
	}
	void setPixelRatioPcb(float value){
        pxRatioPcb_x = value;
        pxRatioPcb_y = value;
	}
	void setPixelRatioBottom(float value){
		pxRatioBottom = value;
	}
    double pxRatioSlot,pxRatioTape,pxRatioBottom;
    double pxRatioPcb_x, pxRatioPcb_y;
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
