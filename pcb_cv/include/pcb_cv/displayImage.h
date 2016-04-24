#ifndef DISPLAYIMAGE_H
#define DISPLAYIMAGE_H

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
#include <pcb_cv/padFinder.h>
#include <cmath>
#include <image_transport/image_transport.h>
#include "pap_common/Task.h"
#include "pap_common/VisionStatus.h"
#include <pap_common/vision_message_def.h>
#include <pap_common/task_message_def.h>
#include <tf/transform_broadcaster.h>
#include <pap_common/VisionAction.h>
#include <actionlib/server/simple_action_server.h>


namespace pcb_cv{

typedef actionlib::SimpleActionServer<pap_common::VisionAction> ActionServer;

enum VISION_PROCESS {
        CHIP, SMALL_SMD, TAPE, PAD, IDLE, CIRCLE, QRCODE
};

class PcbCvInterface{
public:

    PcbCvInterface();

    void imageCallback1(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
    void parseTask(const pap_common::TaskConstPtr& taskMsg);
    void runVision();
    void createCrosshairs(cv::Mat &input);
    void gatherImages(size_t num_images, std::vector<cv::Mat> &images, enum pap_vision::CAMERA_SELECT camera_sel );

private:

    void execute_action(const pap_common::VisionGoalConstPtr &command);

    ros::NodeHandle nh_;
    ActionServer as_;
    ros::Publisher statusPublisher;
    padFinder finder;
    unsigned int cameraSelect;
    cv::Point2f selectPoint;

    CvCapture* capture;
    CvCapture* capture2;
    image_transport::Publisher image_pub_;
    image_transport::Publisher image_pub_2;

    // Image buffer
    bool gather_bottom_images_, gather_top_images_;
    std::vector<cv::Mat> top_buffer_;
    std::vector<cv::Mat> bottom_buffer_;


    VISION_PROCESS visionState ;
    pap_vision::VISION_QR_CALIBRATION qrCalAction;
    bool visionEnabled, selectPad;
    bool searchTapeRotation;

    unsigned int id_counter1, id_counter2;

    unsigned int img_gather_counter;


    int calibrationIteration;

};

}

#endif // DISPLAYIMAGE_H
