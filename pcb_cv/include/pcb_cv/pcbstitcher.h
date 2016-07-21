#ifndef PCBSTITCHER_H
#define PCBSTITCHER_H

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/core.hpp>
#include "opencv2/reg/map.hpp"
#include "opencv2/reg/mapaffine.hpp"
#include "opencv2/reg/mapshift.hpp"
#include "opencv2/reg/mapprojec.hpp"
#include "opencv2/reg/mappergradshift.hpp"
#include "opencv2/reg/mappergradeuclid.hpp"
#include "opencv2/reg/mappergradsimilar.hpp"
#include "opencv2/reg/mappergradaffine.hpp"
#include "opencv2/reg/mappergradproj.hpp"
#include "opencv2/reg/mapperpyramid.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>

#include <fstream>

namespace stitcher{

//!
//! \brief The PcbStitcher class provides functionality for stitching images together.
//!
class PcbStitcher
{
public:
    PcbStitcher(double pxFactorInit);

    //!
    //! \brief translateImg translates a image
    //! \param img image to translate
    //! \param offsetx x translation
    //! \param offsety y translation
    //! \return translated image
    //!
    cv::Mat translateImg(cv::Mat &img, int offsetx, int offsety);

    //!
    //! \brief feedImage feeds an image into the storage and aligns it with the previous fed image.
    //! \param image_in input image
    //! \param offset initial guess (offset between input image and previous fed image)
    //!
    void feedImage(cv::Mat image_in, cv::Point2d offset);

    //!
    //! \brief reset resets feeder and blender
    //!
    void reset();

    //!
    //! \brief blendImages blends all fed images together and produces one single image out of it.
    //! \param final blended image
    //!
    void blendImages(cv::Mat &final);

    //!
    //! \brief getApproxPxFactor gets approximated pixel conversion factors
    //! \param px_factor_x[out] pixel conversion factor in x direction
    //! \param px_factor_y[out] pixel conversion factor in y direction
    //!
    void getApproxPxFactor(double &px_factor_x, double &px_factor_y);

    //!
    //! \brief setApproxPxFactor sets initial pixel to mm conversion factor
    //! \param factor pixel to mm conversion factor
    //!
    void setApproxPxFactor(double factor){
        px_conv_factor = factor;
    }

    //!
    //! \brief getImageListSize gets the size of the list of fed images
    //! \return number of fed images
    //!
    size_t getImageListSize(){
        return input_images.size();
    }

    //!
    //! \brief getURCornerCoord gets the position of the upper right corner of the blended image in PAP coordinates
    //! \return coordinate of upper right corner
    //!
    cv::Point2d getURCornerCoord(){
        return ur_corner_coord_;
    }

private:

    //!
    //! \brief showDifference shows difference image between first and second image
    //! \param image1 first image
    //! \param image2 second image
    //! \param title title of image
    //!
    void showDifference(const cv::Mat& image1, const cv::Mat& image2, const char* title);

    //!
    //! \brief align aligns two images
    //! \param img1 first image
    //! \param img2 second image
    //! \param mapshift initial guess of translational shift
    //!
    void align(const cv::Mat& img1, cv::Mat& img2, cv::reg::MapShift &mapshift);

    std::vector<std::pair<cv::Mat,cv::Point2d> > input_images;

    std::vector<cv::Point2d> offsets_;
    double px_conv_factor;
    cv::Point2d scan_pos_;
    size_t i_pic_;
    cv::Size2f size_blended_;
    cv::Point2d ur_corner_coord_;
};

}

#endif // PCBSTITCHER_H
