#include <pcb_cv/displayImage.h>

using namespace std;
using namespace cv;
using namespace zbar;

namespace pcb_cv{

PcbCvInterface::PcbCvInterface() : as_(nh_, "vision_actions", boost::bind(&PcbCvInterface::execute_action, this, _1),false){

    visionState = IDLE;
    qrCalAction = NO_CAL;
    visionEnabled = false;
    selectPad = false;
    searchTapeRotation = false;

    id_counter1 = 0;
    id_counter2 = 0;

    calibrationIteration = 0;
    img_gather_counter = 0;
    gather_top_images_ = false;
    gather_bottom_images_ = false;
}

void PcbCvInterface::gatherImages(size_t num_images, std::vector<cv::Mat> *images, enum CAMERA_SELECT camera_sel ){

    bottom_buffer_.clear();
    top_buffer_.clear();
    img_gather_counter = 0;

    if(camera_sel == CAMERA_TOP){
        gather_top_images_ = true;
    }else{
        gather_bottom_images_ = true;
    }

    // Wait until all images are gathered
    ros::Rate loop_rate(100);
    while(img_gather_counter < num_images){
        ros::spinOnce();
        loop_rate.sleep();
    }

    // All images gathered
    gather_top_images_ = false;
    gather_bottom_images_ = false;

    // Return pointer to storage
    if(camera_sel == CAMERA_TOP){
        images = &top_buffer_;
    }else{
        images = &bottom_buffer_;
    }
}

void PcbCvInterface::execute_action(const pap_common::VisionGoalConstPtr& command)
{
    switch(command->task){

    case pap_vision::START_VISION:
        visionEnabled = true;
        visionState = IDLE;
        as_.setSucceeded();
        break;

    case pap_vision::STOP_VISION:
        visionEnabled = false;
        visionState = IDLE;
        as_.setSucceeded();
        break;

    case pap_vision::START_CHIP_FINDER:{
        finder.setSize(command->data1, command->data2);
        cameraSelect = command->cameraSelect;
        std::vector<cv::Mat> *images;
        gatherImages(command->numAverages, images,(CAMERA_SELECT)cameraSelect);
        smdPart chip;
        if(finder.findChipAvg(images, (CAMERA_SELECT) cameraSelect, chip)){
            pap_common::VisionResult res;
            res.cameraSelect = cameraSelect;
            res.data1 = chip.y;
            res.data2 = chip.x;
            res.data3 = chip.rot;
            as_.setSucceeded(res);
        }else{
            as_.setAborted();
        }
    }
        break;

    case pap_vision::START_TAPE_FINDER:{
        finder.setSize(command->data1, command->data2);
        cameraSelect = command->cameraSelect;
        std::vector<cv::Mat> *images;
        gatherImages(command->numAverages, images,(CAMERA_SELECT) cameraSelect);
        if (command->data3 == 1.0) {
            searchTapeRotation = true;
        } else {
            searchTapeRotation = false;
        }
        smdPart chip;
        if(finder.findSMDTapeAvg(images, searchTapeRotation, chip)){
            pap_common::VisionResult res;
            res.data1 = chip.y;
            res.data2 = chip.x;
            res.data3 = chip.rot;
            res.cameraSelect = 0;
            as_.setSucceeded(res);
        }else{
            as_.setAborted();
        }
    }
        break;

    case pap_vision::START__QRCODE_FINDER:
    {
        cameraSelect = command->cameraSelect;
        std::vector<cv::Mat> *images;
        gatherImages(command->numAverages, images,(CAMERA_SELECT) cameraSelect);

        if ((CAMERA_SELECT) command->cameraSelect == CAMERA_TOP) {
            switch ((VISION_QR_CALIBRATION) command->data1) {
            case TOP_SLOT:{

                double pxRatioSlot;
                if(finder.getPixelConvValAvg(images, pxRatioSlot))
                    finder.setPixelRatioSlot(pxRatioSlot);
                else as_.setAborted();

            }
                break;

            case TOP_PCB:
            {
                double pxRatioPcb;
                if(finder.getPixelConvValAvg(images, pxRatioPcb))
                    finder.setPixelRatioPcb(pxRatioPcb);
                else as_.setAborted();

            }
                break;

            case TOP_TAPE:
            {
                double pxRatioTape;
                if(finder.getPixelConvValAvg(images, pxRatioTape))
                    finder.setPixelRatioTape(pxRatioTape);
                else as_.setAborted();
            }
                break;
            }
        }
        else {
            if((VISION_QR_CALIBRATION) command->data1 == BOTTOM_CAM){
                double pxRatioBottom;
                if(finder.getPixelConvValAvg(images, pxRatioBottom))
                    finder.setPixelRatioBottom(pxRatioBottom);
                else as_.setAborted();
            }
        }

        as_.setSucceeded();
    }
        break;

        // This state is for manually selecting of the fiducials
    case pap_vision::START_PAD_FINDER:
        if (command->data1 == 1.0) {
            selectPad = true;
            selectPoint.x = command->data2;
            selectPoint.y = command->data3;
            //ROS_INFO("Pixel %f %f", selectPoint.x, selectPoint.y);
        } else {
            selectPad = false;
            selectPoint.x = 0.0;
            selectPoint.y = 0.0;
        }
        visionState = PAD;
        as_.setSucceeded();
        break;

    case pap_vision::SEARCH_CIRCLE:{
        finder.setSize(command->data1, command->data2);
        cameraSelect = command->cameraSelect;
        std::vector<cv::Mat> *images;
        gatherImages(command->numAverages, images,(CAMERA_SELECT) cameraSelect);

        smdPart tip;
        if(finder.findTipAvg(images, (CAMERA_SELECT) cameraSelect, tip)){
            pap_common::VisionResult res;
            res.data1 = -tip.y;
            res.data2 = tip.x;
            res.data3 = tip.rot;
            res.cameraSelect = cameraSelect;
            as_.setSucceeded(res);
        }else{
            as_.setAborted();
        }
    }
        break;
    }
}

void PcbCvInterface::imageCallback1(const sensor_msgs::ImageConstPtr& msg) {

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
        ROS_ERROR("displayImage: Could not convert image");
        return;
    }


    if(gather_top_images_){
        top_buffer_.push_back(input);
        img_gather_counter++;
    }

    if (visionEnabled) {
        switch (visionState) {
        case IDLE:

            break;

        case QRCODE: {
            if (cameraSelect == CAMERA_TOP) {
                switch (qrCalAction) {
                case TOP_SLOT:{

                    double pxRatioSlot;
                    if(finder.getPixelConvVal(input, pxRatioSlot))
                        finder.setPixelRatioSlot(pxRatioSlot);

                    visionMsg.task = pap_vision::START__QRCODE_FINDER;
                    statusPublisher.publish(visionMsg);
                    qrCalAction = NO_CAL;
                }
                    break;

                case TOP_PCB:
                {
                    double pxRatioPcb;
                    if(finder.getPixelConvVal(input, pxRatioPcb))
                        finder.setPixelRatioPcb(pxRatioPcb);

                    visionMsg.task = pap_vision::START__QRCODE_FINDER;
                    statusPublisher.publish(visionMsg);
                    qrCalAction = NO_CAL;
                }
                    break;

                case TOP_TAPE:
                {
                    double pxRatioTape;
                    if(finder.getPixelConvVal(input, pxRatioTape))
                        finder.setPixelRatioTape(pxRatioTape);

                    visionMsg.task = pap_vision::START__QRCODE_FINDER;
                    statusPublisher.publish(visionMsg);
                    qrCalAction = NO_CAL;
                }
                    break;

                case NO_CAL:
                    break;
                }
            }
        }
            break;

        case CHIP:
            // Chip

            if (cameraSelect == CAMERA_TOP) {
                if(finder.findChip(&input, cameraSelect, smd)){
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
            if(finder.findSMDTape(input, searchTapeRotation, smd)){
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
    createCrosshairs(input);

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

void PcbCvInterface::imageCallback2(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat input2;
    try {
        input2 = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("displayImage: Could not convert image");
        return;
    }

    if(gather_bottom_images_){
        bottom_buffer_.push_back(input2);
        img_gather_counter++;
    }

    id_counter2++;
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
                if(finder.findChip(&input2, cameraSelect,smd)){
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
            if(finder.findTip(input2, smd)){
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
                if (qrCalAction == BOTTOM_CAM) {

                    double pxRatioBottom;
                    if(finder.getPixelConvVal(input2, pxRatioBottom))
                        finder.setPixelRatioBottom(pxRatioBottom);

                    visionMsg.task = pap_vision::START__QRCODE_FINDER;
                    statusPublisher.publish(visionMsg);
                    qrCalAction = NO_CAL;
                }
            }
            break;
        }
    }

    createCrosshairs(input2);

    // Camera 2
    cv::Mat outputRGB;
    cvtColor(input2, outputRGB, CV_BGR2RGB);
    std_msgs::Header header;
    header.seq = id_counter2;
    header.stamp = ros::Time::now();
    header.frame_id = "camera2";

    cv_bridge::CvImage out_msg;
    out_msg.header = header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::RGB8;
    out_msg.image = outputRGB;
    image_pub_2.publish(out_msg.toImageMsg());
}

void PcbCvInterface::parseTask(const pap_common::TaskConstPtr& taskMsg) {
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
            finder.setSize(taskMsg->data1, taskMsg->data2);
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

void PcbCvInterface::createCrosshairs(cv::Mat &input){
    // Crosshairs
    Point2f vertices[4];
    vertices[0] = Point2f(input.cols / 2 - 1, 0);
    vertices[1] = Point2f(input.cols / 2 - 1, input.rows - 1);
    vertices[2] = Point2f(input.cols - 1, input.rows / 2 - 1);
    vertices[3] = Point2f(0, input.rows / 2 - 1);

    line(input, vertices[1], vertices[0], Scalar(0, 0, 255), 2);
    line(input, vertices[3], vertices[2], Scalar(0, 0, 255), 2);
    circle(input, Point2f(input.cols / 2 - 1, input.rows / 2 - 1), 20,
           CV_RGB(255, 0, 0), 2);
}

void PcbCvInterface::runVision(){


    image_transport::ImageTransport it_(nh_);

    image_transport::Publisher qr_image_pub_;

    ros::Subscriber taskSubscriber_ = nh_.subscribe("task", 10, &PcbCvInterface::parseTask, this);
    statusPublisher = nh_.advertise<pap_common::VisionStatus>("visionStatus",
                                                              1000);

    image_transport::Subscriber camera1sub = it_.subscribe("Camera1/image_raw",
                                                           2, &PcbCvInterface::imageCallback1, this);
    image_transport::Subscriber camera2sub = it_.subscribe("Camera2/image_raw",
                                                           2, &PcbCvInterface::imageCallback2, this);

    ros::Rate loop_rate(100);
    image_pub_ = it_.advertise("camera1", 10);
    image_pub_2 = it_.advertise("camera2", 10);
    qr_image_pub_ = it_.advertise("image", 10);

    as_.start();

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_two_ints_server");
    pcb_cv::PcbCvInterface vision;
    vision.runVision();
    return 0;

}

