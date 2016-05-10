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


class PcbStitcher
{
public:
    PcbStitcher();
    cv::Mat translateImg(cv::Mat &img, int offsetx, int offsety);
    void showDifference(const cv::Mat& image1, const cv::Mat& image2, const char* title);
    void align(const cv::Mat& img1, cv::Mat& img2, cv::reg::MapShift &mapshift);
    void blendImages(cv::Mat &final);
    void feedImage(cv::Mat image_in, cv::Point2d offset);
    void reset();

private:
    std::vector<std::pair<cv::Mat,cv::Point2d> > input_images;

    std::vector<cv::Point2d> offsets_;
    double px_conv_factor;
    cv::Point2d scan_pos_;
    size_t i_pic_;
};

}

#endif // PCBSTITCHER_H
