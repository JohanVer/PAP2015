#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv_modules.hpp>
#include <opencv.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/core.hpp>
#include <opencv_modules.hpp>
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


// Pad Finder
#define MIN_CONTOUR_AREA 100
#define MIN_PAD_AREA 70
#define DILATE_ITERATIONS 3

// Chip Finder
#define ERODE_ITERATIONS_CHIP_FINDER 0
//#define MIN_CONTOUR_AREA_CHIP 300
#define ERROR_PERCENT_CHIP 50.0

// SMD Finder
#define ERODE_ITERATIONS_SMD_FINDER 1
#define MIN_AREA_SMD_FINDER 300
#define ERROR_PERCENT_SMALLSMD 20.0

// SMD Tape
#define ERODE_ITERATIONS_TAPE_FINDER 1
#define MIN_AREA_TAPE_FINDER 300
#define ERROR_PERCENT_SMDTAPE 30.0

// Tips
#define ERROR_PERCENT_TIP 20.0

static const char* DIFF_IM = "Image difference";
static const char* DIFF_REGPIX_IM = "Image difference: pixel registered";


using namespace std;
using namespace cv;
using namespace cv::reg;
Mat translateImg(Mat &img, int offsetx, int offsety){
    Mat trans_mat = (Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    warpAffine(img,img,trans_mat,img.size()*2);
    return trans_mat;
}

static void showDifference(const Mat& image1, const Mat& image2, const char* title)
{
    Mat img1, img2;
    image1.convertTo(img1, CV_32FC3);
    image2.convertTo(img2, CV_32FC3);
    if(img1.channels() != 1)
        cvtColor(img1, img1, CV_RGB2GRAY);
    if(img2.channels() != 1)
        cvtColor(img2, img2, CV_RGB2GRAY);

    Mat imgDiff;
    img1.copyTo(imgDiff);
    imgDiff -= img2;
    imgDiff /= 2.f;
    imgDiff += 128.f;

    Mat imgSh;
    imgDiff.convertTo(imgSh, CV_8UC3);
    imshow(title, imgSh);
}

static void align(const Mat& img1, Mat& img2, MapShift &mapshift)
{
    // Register
    MapperGradShift mapper;
    MapperPyramid mappPyr(mapper);
    mappPyr.numIterPerScale_ = 15;
    mappPyr.numLev_ = 4;

    cv::Ptr<Map> mapptr = mapshift.inverseMap();
    mappPyr.calculate(img1, img2, mapptr);

    MapShift* mapShift = dynamic_cast<MapShift*>(mapptr.get());
    mapshift = *(mapShift);
}

double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2)
            / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

bool isBorderTouched(cv::RotatedRect pad) {
    cv::Point2f vertices2f[4];
    cv::Point vertices[4];
    pad.points(vertices2f);
    for (int i = 0; i < 4; ++i) {
        vertices[i] = vertices2f[i];

        if (vertices[i].x < 5 || vertices[i].x > 635 || vertices[i].y > 475
                || vertices[i].y < 5) {
            return true;
        }
    }
    return false;
}

void drawRotatedRect(cv::Mat& image, cv::RotatedRect rRect,
                                cv::Scalar color) {

    cv::Point2f vertices2f[4];
    cv::Point vertices[4];
    rRect.points(vertices2f);
    for (int i = 0; i < 4; ++i) {
        vertices[i] = vertices2f[i];
    }
    cv::fillConvexPoly(image, vertices, 4, color);
}

cv::Point2f findPads(cv::Mat* input, bool startSelect,
                                cv::Point2f selectPad) {
    bool foundVia = false;
    cv::Mat gray;
    cv::Point2f outputPosition;
    outputPosition.x = 0.0;
    outputPosition.y = 0.0;
    cv::cvtColor(*input, gray, CV_BGR2GRAY);


    cv::Mat bw;
    cv::threshold(gray, bw, 255, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
    //cv::adaptiveThreshold(gray, bw, 255, ADAPTIVE_THRESH_GAUSSIAN_C,
    //                      CV_THRESH_BINARY, 201, 1.0);


    cv::imshow("bw", bw);
        cv::waitKey(0);
    cv::Mat final = input->clone();
   bool  repeat = false;
    std::vector<std::vector<cv::Point> > contours;
    while (1) {
        //cv::Mat dst = input->clone();
        foundVia = false;
        // Find contours
        contours.clear();
        vector<Vec4i> hierarchy;
        vector<int> circlesIndex;
        cv::findContours(bw.clone(), contours, hierarchy, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);
        std::vector<cv::Point> approx;

        for (int i = 0; i < contours.size(); i++) {
            if (std::fabs(cv::contourArea(contours[i])) > MIN_CONTOUR_AREA) {
                //cv::drawContours(dst, contours, i, CV_RGB(0, 0, 255), 5);
            }

            // Approximate contour with accuracy proportional
            // to the contour perimeter
            cv::approxPolyDP(cv::Mat(contours[i]), approx,
                             cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);
            // Skip small or non-convex objects
            if (std::fabs(cv::contourArea(contours[i])) < 50
                    || !cv::isContourConvex(approx)) {
                continue;
                //ROS_INFO("Zu klein");
            }

            if (approx.size() >= 4 && approx.size() <= 6) {
                // Number of vertices of polygonal curve
                int vtc = approx.size();
                // Get the cosines of all corners
                std::vector<double> cos;
                for (int j = 2; j < vtc + 1; j++)
                    cos.push_back(
                                angle(approx[j % vtc], approx[j - 2],
                            approx[j - 1]));
                // Sort ascending the cosine values
                std::sort(cos.begin(), cos.end());
                // Get the lowest and the highest cosine
                double mincos = cos.front();
                double maxcos = cos.back();
                // Use the degrees obtained above and the number of vertices
                // to determine the shape of the contour
                if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3) {
                    //setLabel(dst, "RECT", contours[i]);

                } else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27) {

                    //setLabel(dst, "PENTA", contours[i]);
                } else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45) {

                    //setLabel(dst, "HEXA", contours[i]);
                }

            } else {
                // Detect and label circles
                double area = cv::contourArea(contours[i]);
                cv::Rect r = cv::boundingRect(contours[i]);
                int radius = r.width / 2;
                if (std::abs(1 - ((double) r.width / r.height)) <= 0.3
                        && std::abs(1 - (area / (CV_PI * std::pow(radius, 2))))
                        <= 0.3) {
                    circlesIndex.push_back(i);
                    //setLabel(dst, "CIR", contours[i]);
                }

            }
        }

        for (int i = 0; i < contours.size(); i++) {
            for (int j = 0; j < circlesIndex.size(); j++) {
                if (hierarchy[i][2] == circlesIndex[j]) {
                    RotatedRect viaPad = minAreaRect(contours[i]);
                    RotatedRect via = minAreaRect(contours[circlesIndex[j]]);
                    via.size.height = via.size.height * 1.6;
                    via.size.width = via.size.width * 1.6;
                    via.angle = viaPad.angle;

                    Point2f vertices[4];
                    Point2f verticesVia[4];
                    viaPad.points(vertices);
                    via.points(verticesVia);
                    for (int k = 0; k < 4; k++) {

                        drawRotatedRect(bw, via, CV_RGB(0, 0, 0));
                    }
                    foundVia = true;
                    break;
                }
            }
        }

        if (foundVia) {
            repeat = true;
            //ROS_INFO("Repeat...");
        } else
            repeat = false;

        if (!repeat) {
            //final = dst;
            break;
        }
    }

    // Draw pads rectangles

    vector<Point2f> padPoints;
    vector<RotatedRect> padRects;

    unsigned int padCounter = 0;
    for (int i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > MIN_PAD_AREA) {

            RotatedRect pad = minAreaRect(contours[i]);
            if (!isBorderTouched(pad)) {
                // Calculate moments of image
                Moments mu;
                mu = moments(contours[i], false);
                //Calculate mass center
                Point2f mc;
                mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
                padPoints.push_back(mc);
                padRects.push_back(pad);

                if (startSelect
                        && cv::pointPolygonTest(contours[i], selectPad, false)
                        == 1.0) {
                    drawRotatedRect(final, pad, CV_RGB(0, 0, 255));

                    // TODO: Change into middle coordinate of the camera
                    outputPosition.x = (mc.x - (input->cols / 2 - 1))
                            / 31;
                    outputPosition.y = ((input->rows / 2 - 1) - mc.y)
                            / 31;

                } else {
                    drawRotatedRect(final, pad, CV_RGB(255, 0, 0));
                }
                padCounter++;
            }
        }
    }

    cv::Mat out;
    cv::cvtColor(bw, out, CV_GRAY2BGR);

    *input = final.clone();
    //cv::imshow("src", *input);
    //	cv::waitKey(0);
    return outputPosition;

    //cv::imshow("bw", bw);
    //cv::imshow("final", final);
    //cv::imshow("grey", gray);
    //ROS_INFO("Found %d pads..", padCounter);
}

void blendImages(std::vector<std::pair<cv::Mat,cv::Point2d> > &images, std::vector<cv::UMat> images_exposure, cv::Mat &final){

    double max_x = 0;
    double max_y = 0;

    double min_y = std::numeric_limits<double>::max();

    cv::Size image_size = images.front().first.size();

    for(size_t i = 0; i < images.size(); i++){
        double x = images.at(i).second.x;
        double y = images.at(i).second.y;
        if(x > max_x){
            max_x = x;
        }

        if(y > max_y){
            max_y = y;
        }

        if(y < min_y){
            min_y = y;
        }
    }

    std::cerr << "Max x: " << max_x << " Max y: " << max_y << std::endl;

    double y_offset = 0;
    if(min_y < 0){
        y_offset = std::fabs(min_y);
    }
    std::cerr << "Y-Offset is: "<< y_offset << std::endl;


    // Init blender
    detail::FeatherBlender blender;
    Rect bounding_box(0,0,max_x + image_size.width , max_y + image_size.height+y_offset);
    std::cerr << "Bounding box: " << bounding_box.width << " , " << bounding_box.height << std::endl;
    blender.prepare(bounding_box);

    //Compensate exposure
/*
    Ptr<detail::ExposureCompensator> gain_compensator = detail::ExposureCompensator::createDefault(detail::ExposureCompensator::GAIN);

    std::vector<cv::Point> corners;
    std::vector<cv::UMat > masks;

    for(size_t i = 0; i < images.size(); i++){

        std::cerr << "Feed image in exposure compensator: " << i << std::endl;

        cv::UMat feed_im2 = images_exposure.at(i);
        cv::Mat feed_im = images.at(i).first;
        cv::UMat mask = cv::UMat::ones(feed_im.rows, feed_im.cols, CV_8U);
        mask.setTo(Scalar::all(255));
        masks.push_back(mask);

        cv::Point px_offset = images.at(i).second;
        px_offset.y = px_offset.y + y_offset;

        corners.push_back(px_offset);


        std::cerr << "Feed pos: " << px_offset.x << " , " << px_offset.y << std::endl;

    }

    std::cerr << "Starting exposure compensating...\n";
    gain_compensator->feed(corners, images_exposure, masks);


    for(size_t i = 0; i < images.size(); i++){

        cv::Mat feed_im = images.at(i).first;
        feed_im.convertTo(feed_im, CV_8UC3);

        cv::Mat mask;
        mask.create(feed_im.size(),CV_8U);
        mask.setTo(Scalar::all(255));

        cv::Point2d px_offset = images.at(i).second;
        px_offset.y = px_offset.y + y_offset;

        gain_compensator->apply(i, px_offset, feed_im, mask );

        images.at(i).first = feed_im;
    }

*/
    //----------------------------------------------------------------------------------------
    // Blending

    for(size_t i = 0; i < images.size(); i++){

        std::cerr << "Feed image in blender: " << i << std::endl;

        cv::Mat feed_im = images.at(i).first;
        feed_im.convertTo(feed_im, CV_16SC3);

        cv::Mat mask = cv::Mat::ones(feed_im.cols, feed_im.rows, CV_8U)*255;

        cv::Point2d px_offset = images.at(i).second;
        std::cerr << "Feed pos: " << px_offset.x << " , " << px_offset.y << std::endl;
        blender.feed(feed_im.clone(), mask, Point(px_offset.x, px_offset.y + y_offset));

    }

    final = cv::Mat(bounding_box.width, bounding_box.height, CV_8UC3);
    cv::Mat dst_mask = cv::Mat::ones(final.cols, final.rows, CV_8U)*255;

    std::cerr << "Mat cols: " << dst_mask.cols << " rows: " << dst_mask.rows << std::endl;
    blender.blend(final,dst_mask );

    final.convertTo(final, (final.type() / 8) * 8);

    imshow("Blended", final);

    waitKey(0);
}

int main(int argc, char *argv[])
{

    double px_conv_factor = 31.025854879;

    std::vector<std::pair<cv::Mat,cv::Point2d> > input_images;
    std::vector<cv::UMat> umats;

    ifstream file ( "/home/johan/catkin_ws/src/PAP2015/PAP/stitching/with_light/stitch_coord.csv" );
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
    for(size_t i = 0; i < 11 ; i++){
        cv::Mat im1 = cv::imread("/home/johan/catkin_ws/src/PAP2015/PAP/stitching/with_light/stitch" +std::to_string(i)+ ".jpg");
        cv::UMat imgu = imread( "/home/johan/catkin_ws/src/PAP2015/PAP/stitching/with_light/stitch" +std::to_string(i)+ ".jpg", IMREAD_COLOR  ).getUMat( ACCESS_READ );
        umats.push_back(imgu);
        im1.convertTo(im1, CV_64FC3);
        cv::Point2d p0(0,0);
        input_images.push_back(std::pair<cv::Mat, cv::Point2d> (im1, p0));
    }

    cv::Point2d scan_pos(0,0);

    for(size_t i = 1; i < input_images.size(); i++){
        cv::Mat &cur_image = input_images.at(i).first;
        cv::Mat &img_before = input_images.at(i-1).first;

        cv::Point2d &cur_point = offsets.at(i);
        cv::Point2d &point_before = offsets.at(i-1);

        cv::Point2d rel_offset = cur_point - point_before;
        rel_offset.y = -rel_offset.y;

        cv::Point2d converted_offset(rel_offset.x * px_conv_factor, rel_offset.y * px_conv_factor);

        std::cerr << "act offset:  " << cur_point << " before: " << point_before << std::endl;
        std::cerr << "converted Offset: " << converted_offset << std::endl;

        // Initial Guess
        Vec<double, 2> shift(converted_offset.x, converted_offset.y);
        MapShift mapTest(shift);

        // TEST----
        cv::Mat test;
        mapTest.warp(cur_image, test);
        //showDifference(img_before, test, DIFF_IM);
        //waitKey(1);
        //------


        align(img_before, cur_image, mapTest);

        cout << endl << "--- Result shift mapper ---" << endl;
        cout << Mat(mapTest.getShift()) << endl;

        cv::Point2d corrected_pos(-mapTest.getShift()[0], -mapTest.getShift()[1]);

        input_images.at(i).second.x = corrected_pos.x;
        input_images.at(i).second.y = corrected_pos.y;

        input_images.at(i).second = input_images.at(i).second + scan_pos;
        scan_pos = input_images.at(i).second;

        std::cerr << "Rel offset: " << rel_offset << std::endl;
        std::cerr << "Corr offset: " << corrected_pos << std::endl;

        double conv_factor_update = 0.0;
        if(std::fabs(rel_offset.x) > std::fabs(rel_offset.y)){

            conv_factor_update = (corrected_pos.x / rel_offset.x);
        }else{
            conv_factor_update = (corrected_pos.y / rel_offset.y);
        }

        px_conv_factor = px_conv_factor/2.0 + conv_factor_update/2.0;
        std::cerr << "Updated px conversion factor: " << px_conv_factor << std::endl;

    }

    cv::Mat final;

    blendImages(input_images, umats, final);

    cv::Point2d dummy;
    findPads(&final,false,dummy);

    imshow("Pads", final);

    waitKey(0);
    return 0;
}
