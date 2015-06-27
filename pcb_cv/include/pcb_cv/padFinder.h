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
#include <cmath>

class padFinder
{
  public:                              // öffentlich
    padFinder();                      // der Default-Konstruktor
    ~padFinder();                     // der Destruktor
 
    //std::vector<cv::Point2f > findPads(cv::Mat* input);        // eine Funktion mit einem (Default-) Parameter
    void findPads(cv::Mat* input);
    void findChip(cv::Mat* input);
    void findSmallSMD(cv::Mat* input);
    void findSMDTape(cv::Mat* input);

     double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
    void setLabel(cv::Mat& im, const std::string label,std::vector<cv::Point>& contour);
    void drawRotatedRect(cv::Mat& image, cv::RotatedRect rRect,cv::Scalar color);
    bool isBorderTouched(cv::RotatedRect pad);
  private:                             // privat
    int m_eineVariable;
    bool foundVia;
    bool repeat;
};
