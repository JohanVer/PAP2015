#ifndef DISPLAYIMAGE_H
#define DISPLAYIMAGE_H

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
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
#include <pcb_cv/pcbstitcher.h>


namespace pcb_cv{

typedef actionlib::SimpleActionServer<pap_common::VisionAction> ActionServer;

enum VISION_PROCESS {
        CHIP, SMALL_SMD, TAPE, PAD, IDLE, CIRCLE, QRCODE
};

//!
//! \brief The PcbCvInterface class provides a bridge between ros and all computer vision algorithms used by the PAP software
//!
class PcbCvInterface{
public:

    PcbCvInterface();
    ~PcbCvInterface();

    //!
    //! \brief imageCallback1 callback for top camera. Processes image and sends it to other nodes.
    //! \param msg camera image
    //!
    void imageCallback1(const sensor_msgs::ImageConstPtr& msg);

    //!
    //! \brief imageCallback2 callback for bottom camera. Processes image and sends it to other nodes.
    //! \param msg camera image
    //!
    void imageCallback2(const sensor_msgs::ImageConstPtr& msg);

    //!
    //! \brief parseTask parses incoming task message and starts the corresponding vision algorithm
    //! \param taskMsg task message
    //!
    void parseTask(const pap_common::TaskConstPtr& taskMsg);

    //!
    //! \brief runVision starts vision ros interface
    //!
    void runVision();

    //!
    //! \brief createCrosshairs creates crosshair augmentation on given image
    //! \param input image to augment
    //!
    void createCrosshairs(cv::Mat &input);

    //!
    //! \brief gatherImages gathers a certain number of images from the top or bottom camera.
    //! \param num_images number of images to gather
    //! \param images image storage
    //! \param camera_sel selects which camera should be used
    //!
    void gatherImages(size_t num_images, std::vector<cv::Mat> &images, enum pap_vision::CAMERA_SELECT camera_sel );

private:

    //!
    //! \brief execute_action handler for incomming action requests. Starts corresponding vision algorithm and directly sends feedback.
    //! \param command action message
    //!
    void execute_action(const pap_common::VisionGoalConstPtr &command);

    stitcher::PcbStitcher stitcher_;
    ros::NodeHandle nh_;
    ActionServer as_;
    ros::Publisher statusPublisher;
    padFinder finder;
    unsigned int cameraSelect;
    cv::Point2f selectPoint;

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

    bool tip_thresholding;

};

}

#endif // DISPLAYIMAGE_H
