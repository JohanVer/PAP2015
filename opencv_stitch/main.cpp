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
#include "pcbstitcher.h"

#include <fstream>


using namespace std;
using namespace cv;
using namespace cv::reg;


int main(int argc, char *argv[])
{   

    stitcher::PcbStitcher stitcher;

    std::vector<cv::Mat > input_images;

    ifstream file ( "/home/johan/catkin_ws/src/PAP2015/PAP/stitching/stitch_coord.csv" );
    // Get offsets
    std::vector<cv::Point2d> offsets;
    std::string line;
    while (getline(file, line ) &&  file.good() )
    {
        size_t pos = line.find(',');
        double y = std::stof(line.substr(0,pos));
        double x = std::stof(line.substr(pos+1, line.size()));
        offsets.push_back(cv::Point2d(x,y));
    }

    // Get images and combine them with offsets
    for(size_t i = 0; i < 6 ; i++){
        cv::Mat im1 = cv::imread("/home/johan/catkin_ws/src/PAP2015/PAP/stitching/stitch" +std::to_string(i)+ ".jpg");
        im1.convertTo(im1, CV_64FC3);
        input_images.push_back(im1);
    }


    // Stitch
    stitcher.reset();
    for(size_t i = 0; i < input_images.size() ; i++){
        stitcher.feedImage(input_images.at(i), offsets.at(i));
    }


    // Blend
    cv::Mat final;
    stitcher.blendImages(final);

    return 0;
}
