/*
 * padFinder.cpp
 *
 *  Created on: Mar 29, 2015
 *      Author: johan
 */
#include <pcb_cv/padFinder.h>
using namespace std;
using namespace cv;

// Pad Finder
#define MIN_CONTOUR_AREA 20
#define MIN_PAD_AREA 20
#define DILATE_ITERATIONS 1
#define MIN_VIA_AREA 10

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

padFinder::padFinder() {
    foundVia = false;
    partWidth_ = 0.0;
    partHeight_ = 0.0;
    pxRatioSlot = PIXEL_TO_MM_TOP;
    pxRatioTape = PIXEL_TO_MM_TOP;
    pxRatioPcb = PIXEL_TO_MM_PCB;
    pxRatioBottom = PIXEL_TO_MM_BOTTOM;

    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    dlib::deserialize(string(getenv("PAPRESOURCES")) + "trained_functions/linear_svm_function.dat") >> learned_funct_;
}

cv::Mat padFinder::classifyPixels(const cv::Mat &in){
    cv::Mat out(in.rows, in.cols, CV_8UC1, Scalar(0));

    ros::Time start = ros::Time::now();
    if (in.data)
    {
        uchar* ptr = reinterpret_cast<uchar*>(in.data);
        uchar* ptr_out = reinterpret_cast<uchar*>(out.data);
        for (int i = 0; i < in.cols * in.rows; i++, ptr+=3, ptr_out +=1 )
        {
            sample_type sample;
            sample(0) = *(ptr);
            sample(1) = *(ptr+1);
            sample(2) = *(ptr+2);

            double prob = learned_funct_(sample);

            if(prob >= 0){
                *ptr_out = 0xff;
            }else{
                *ptr_out = 0;
            }
        }
    }

    std::cerr << "SVM took : " << ros::Time::now().toSec() - start.toSec() << " seconds" << std::endl;
    return out;
}

padFinder::~padFinder() {

}

void padFinder::setSize(float width, float height) {
    partWidth_ = width;
    partHeight_ = height;
}

bool padFinder::nearestPart(std::vector<smdPart>* list, smdPart* partDst,
                            int width, int height) {
    int smdId = -1;
    float centerX, centerY, shortestDistance;
    centerX = width / 2 - 1;
    centerY = height / 2 - 1;
    shortestDistance = std::numeric_limits<float>::infinity();

    for (int i = 0; i < list->size(); i++) {
        float distance = sqrt(
                    std::pow(std::fabs(centerX - (*list)[i].x), 2)
                    + std::pow(std::fabs(centerY - (*list)[i].y), 2));
        if (distance < shortestDistance) {
            shortestDistance = distance;
            smdId = i;
        }
    }

    if (smdId != -1) {
        partDst->x = (*list)[smdId].x;
        partDst->y = (*list)[smdId].y;
        partDst->rot = (*list)[smdId].rot;
        return true;
    } else {
        return false;
    }

}

double padFinder::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2)
            / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

bool padFinder::isBorderTouched(cv::RotatedRect pad, cv::Size mat_size) {
    const unsigned char border_gap = 5;
    cv::Point2f vertices2f[4];
    cv::Point vertices[4];
    pad.points(vertices2f);
    for (int i = 0; i < 4; ++i) {
        vertices[i] = vertices2f[i];

        if (vertices[i].x < border_gap || vertices[i].x > mat_size.width - border_gap || vertices[i].y > mat_size.height - border_gap
                || vertices[i].y < border_gap) {
            return true;
        }
    }
    return false;
}

void padFinder::drawRotatedRect(cv::Mat& image, cv::RotatedRect rRect,
                                cv::Scalar color) {

    cv::Point2f vertices2f[4];
    cv::Point vertices[4];
    rRect.points(vertices2f);
    for (int i = 0; i < 4; ++i) {
        vertices[i] = vertices2f[i];
    }
    cv::fillConvexPoly(image, vertices, 4, color);
}

/**
 * Helper function to display text in the center of a contour
 */
void padFinder::setLabel(cv::Mat& im, const std::string label,
                         std::vector<cv::Point>& contour) {
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.4;
    int thickness = 1;
    int baseline = 0;
    cv::Size text = cv::getTextSize(label, fontface, scale, thickness,
                                    &baseline);
    cv::Rect r = cv::boundingRect(contour);
    cv::Point pt(r.x + ((r.width - text.width) / 2),
                 r.y + ((r.height + text.height) / 2));
    cv::rectangle(im, pt + cv::Point(0, baseline),
                  pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255),
                  CV_FILLED);
    cv::putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

bool padFinder::findTipAvg(std::vector<cv::Mat> *input, enum pap_vision::CAMERA_SELECT camera_select, smdPart &tip){
    size_t num_img = input->size();
    size_t num_avg = 0;

    for(size_t i = 0; i < num_img; i++){
        smdPart tip_temp;
        if(findTip(input->at(i),tip_temp)){
            tip.x += tip_temp.x;
            tip.y += tip_temp.y;
            tip.rot += tip_temp.rot;
            num_avg++;
        }
    }

    if(num_avg){
        tip.x /= num_avg;
        tip.y /= num_avg;
        tip.rot /= num_avg;
        return true;
    }else{
        return false;
    }
}

void padFinder::appendImage(cv::Mat image, cv::Point3d coord){
    stitching_data_.push_back(std::pair<cv::Mat, cv::Point3d>(image, coord));
}

void padFinder::saveStitchingImages(){
    static size_t img_c = 0;
    ofstream myfile;
    myfile.open((std::string(std::getenv("PAPRESOURCES")) + "stitching/stitch_coord.csv"));

    for(size_t i = 0; i < stitching_data_.size(); i++){
        cv::imwrite( std::string(std::getenv("PAPRESOURCES")) + "stitching/stitch" + std::to_string(img_c) + ".jpg", stitching_data_.at(i).first);
        cv::Point3d coord = stitching_data_.at(i).second;
        myfile << coord.x << "," << coord.y << "," << coord.z << std::endl;
        img_c++;
    }

    myfile.close();
}

bool padFinder::findTip(cv::Mat &final, smdPart &out) {
    cv::Mat gray;
    std::vector<smdPart> tipObjects;
    cv::cvtColor(final, gray, CV_BGR2GRAY);

    vector<Vec3f> circles;
    std::vector<std::vector<cv::Point> > circlesSorted;

    float radiusMin = ((partWidth_) / 100.0) * (100.0 - ERROR_PERCENT_TIP);
    float radiusMax = ((partWidth_) / 100.0) * (100.0 + ERROR_PERCENT_TIP);

    //cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows / 4, 200,
    //	100, radiusMin * pxRatioBottom,radiusMax * pxRatioBottom);

    cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows / 4, 200,
                     70, 10, 500);

    for (size_t i = 0; i < circles.size(); i++) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        if (radius > radiusMin && radius < radiusMax) {
            // draw the circle center
            //cv::circle( final, center, 3, Scalar(255,0,0), -1, 8, 0 );
            // draw the circle outline
            cv::circle(final, center, radius, Scalar(0, 0, 255), 3, 8, 0);

            smdPart part;
            part.x = center.x;
            part.y = center.y;
            part.rot = radius;
            tipObjects.push_back(part);
        }
    }

    if (circlesSorted.size() > 1) {
        ROS_INFO("More than one tip found! Take nearest one ...");
    }
    // Calculation is done with unit pixel
    if (nearestPart(&tipObjects, &out, final.cols, final.rows)) {
        cv::circle(final, Point2f(out.x, out.y), out.rot,
                   CV_RGB(0, 255, 0), 3);
        cv::circle(final, Point2f(out.x, out.y), 3, Scalar(255, 0, 0),
                   -1, 8, 0);
        out.x = (out.x - (final.cols / 2 - 1)) / pxRatioBottom;
        out.y = ((final.rows / 2 - 1) - out.y) / pxRatioBottom;
        return true;
    }
    else{
        return false;
    }
}

bool padFinder::findChipAvg(std::vector<cv::Mat> *input, enum pap_vision::CAMERA_SELECT camera_select, smdPart &chip){
    size_t num_img = input->size();
    size_t num_avg = 0;

    for(size_t i = 0; i < num_img; i++){
        smdPart chip_temp;
        if(findChip(&(input->at(i)), camera_select,chip_temp)){
            chip.x += chip_temp.x;
            chip.y += chip_temp.y;
            chip.rot += chip_temp.rot;
            num_avg++;
        }
    }

    if(num_avg){
        chip.x /= num_avg;
        chip.y /= num_avg;
        chip.rot /= num_avg;
        return true;
    }else{
        return false;
    }
}

bool padFinder::findChip(cv::Mat* input, unsigned int camera_select, smdPart &part_out) {
    float pxToMM = 0.0;

    if (camera_select == pap_vision::CAMERA_TOP) {
        pxToMM = pxRatioSlot;
    } else if (camera_select == pap_vision::CAMERA_BOTTOM) {
        pxToMM = pxRatioBottom;
    }

    cv::Mat gray;
    cv::Mat final = input->clone();

    //cv::imshow("grey", final);
    //cv::waitKey(0);

    std::vector<smdPart> smdObjects;
    cv::cvtColor(*input, gray, CV_BGR2GRAY);
    cv::threshold(gray, gray, 255, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    //cv::erode(gray, gray, cv::Mat(), cv::Point(-1, -1),
    //ERODE_ITERATIONS_CHIP_FINDER);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point> > contoursSorted;
    cv::findContours(gray.clone(), contours, CV_RETR_TREE,
                     CV_CHAIN_APPROX_NONE);

    //ROS_INFO("Contour Size %d",(int)contours.size());
    for (int i = 0; i < contours.size(); i++) {

        cv::RotatedRect rect = minAreaRect(contours[i]);
        if (isBorderTouched(rect, input->size())) {
            continue;
        }

        if ((rect.size.width / pxToMM
             > ((partWidth_) / 100.0) * (100.0 - ERROR_PERCENT_CHIP)
             && rect.size.width / pxToMM
             < ((partWidth_) / 100.0) * (100.0 + ERROR_PERCENT_CHIP)
             && rect.size.height / pxToMM
             > ((partHeight_) / 100.0) * (100.0 - ERROR_PERCENT_CHIP)
             && rect.size.height / pxToMM
             < ((partHeight_) / 100.0) * (100.0 + ERROR_PERCENT_CHIP))
                || (rect.size.height / pxToMM
                    > ((partWidth_) / 100.0) * (100.0 - ERROR_PERCENT_CHIP)
                    && rect.size.height / pxToMM
                    < ((partWidth_) / 100.0)
                    * (100.0 + ERROR_PERCENT_CHIP)
                    && rect.size.width / pxToMM
                    > ((partHeight_) / 100.0)
                    * (100.0 - ERROR_PERCENT_CHIP)
                    && rect.size.width / pxToMM
                    < ((partHeight_) / 100.0)
                    * (100.0 + ERROR_PERCENT_CHIP))) {
            contoursSorted.push_back(contours[i]);

            //ROS_INFO("Found Width: %f Height: %f", rect.size.width,
            //		rect.size.height);
            smdPart smd;
            smd.x = rect.center.x;
            smd.y = rect.center.y;
            if(smd.rot == -90.0 | smd.rot == 90.0){
                smd.rot = 0.0;
            }
            smd.rot = rect.angle;
            smdObjects.push_back(smd);
            cv::drawContours(final, contours, i, CV_RGB(0, 255, 0), 2);
        }
    }

    if (contoursSorted.size() > 1) {
        ROS_INFO("More than one chip found!");
    }

    *input = final.clone();

    if (nearestPart(&smdObjects, &part_out, input->cols, input->rows)) {
        circle(final, Point2f(part_out.x, part_out.y), 5, CV_RGB(0, 0, 255), 3);
        part_out.x = (part_out.x - (input->cols / 2 - 1)) / pxToMM;
        part_out.y = ((input->rows / 2 - 1) - part_out.y) / pxToMM;
        std::cerr << "Success \n";
        return true;
    }else{
        return false;
    }

    /*
     cv::imshow("grey", gray);
     cv::imshow("input", *input);
     cv::imshow("final", final);
     cv::waitKey(0);*/
}

bool padFinder::getPixelConvValAvg(std::vector<cv::Mat> *pictures, double &pxRatio)
{
    pxRatio = 0.0;
    size_t num_img = pictures->size();
    size_t num_avg = 0;

    for(size_t i = 0; i < num_img; i++){
        double temp_ratio = 0.0;
        if(getPixelConvVal(pictures->at(i), temp_ratio)){
            pxRatio += temp_ratio;
            num_avg++;
        }
    }

    pxRatio /= num_avg;

    if(num_avg) return true;
    else return false;
}

bool  padFinder::getPixelConvVal(cv::Mat &picture, double &pxRatio){

    pxRatio = 0.0;
    double truth_len = 0.0;
    double width = 0.0;
    double height = 0.0;
    if(scanCalibrationQRCode(picture, width, height, truth_len)){
        pxRatio = (width + height) / 2.0;
        pxRatio /= truth_len;
        return true;
    }
    return false;

}


bool padFinder::scanCalibrationQRCode(cv::Mat &picture, double &width, double &height, double &truth_len){
    cv::Mat gray;
    cv::cvtColor(picture, gray, CV_BGR2GRAY);
    cv::Size newSize;
    newSize.width = 640;
    newSize.height = 480;
    cv::resize(gray, gray, newSize);
    cv::resize(picture, picture, newSize);
    int pic_width = gray.cols;
    int pic_height = gray.rows;
    uchar *raw = (uchar *) gray.data;
    // wrap image data
    zbar::Image image(pic_width, pic_height, "Y800", raw, pic_width * pic_height);
    if(!scanner.scan(image)) return false;

    for (zbar::Image::SymbolIterator symbol = image.symbol_begin();
         symbol != image.symbol_end(); ++symbol) {
        vector<Point> vp;

        std::cerr << "decoded " << symbol->get_type_name()
                  << " symbol \"" << symbol->get_data() << '"' << " "
                  << endl;
        int n = symbol->get_location_size();
        for (int i = 0; i < n; i++) {
            vp.push_back(
                        Point(symbol->get_location_x(i),
                              symbol->get_location_y(i)));
        }

        std::string calName = symbol->get_data();
        std::cerr << calName << std::endl;
        std::string number;

        if (calName.size() == 13) {
            number = calName.substr(calName.size() - 1);
        } else if (calName.size() == 14) {
            number = calName.substr(calName.size() - 2);
        }

        float calNumber = std::atoi(number.c_str());
        ROS_INFO("Cal-Value : %f", calNumber);

        RotatedRect r = minAreaRect(vp);
        Point2f pts[4];
        r.points(pts);
        for (int i = 0; i < 4; i++) {
            line(picture, pts[i], pts[(i + 1) % 4], Scalar(255, 0, 0),
                    1);
        }

        // output
        truth_len = calNumber;
        width = r.size.width ;
        height = r.size.height;
        return true;
    }
    return false;
}

bool padFinder::findSMDTapeAvg(std::vector<cv::Mat> *input, bool searchTapeRotation, smdPart &out){
    size_t num_img = input->size();
    size_t num_avg = 0;

    for(size_t i = 0; i < num_img; i++){
        smdPart chip_temp;
        if(findSMDTape((input->at(i)), searchTapeRotation, chip_temp)){
            out.x += chip_temp.x;
            out.y += chip_temp.y;
            out.rot += chip_temp.rot;
            num_avg++;
        }
    }

    if(num_avg){
        out.x /= num_avg;
        out.y /= num_avg;
        out.rot /= num_avg;
        return true;
    }else{
        return false;
    }
}

bool padFinder::findSMDTape(cv::Mat &final, bool searchTapeRotation, smdPart &out) {

    std::vector<smdPart> smdObjects;
    cv::Mat gray;
    cv::cvtColor(final, gray, CV_BGR2GRAY);
    if (searchTapeRotation) {
        cv::threshold(gray, gray, 255, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    } else {
        cv::adaptiveThreshold(gray, gray, 255, ADAPTIVE_THRESH_GAUSSIAN_C,
                              CV_THRESH_BINARY, 801, 2.0);
    }
    cv::erode(gray, gray, cv::Mat(), cv::Point(-1, -1),1);
    //cv::cvtColor(gray, *input, CV_GRAY2BGR);
    cv::imshow("tape thresholded", gray);
    cv::waitKey(0);

    std::vector<std::vector<cv::Point> > contours;
    vector<Vec4i> hierarchy;
    vector<int> circlesIndex;

    cv::findContours(gray, contours, hierarchy, CV_RETR_TREE,
                     CV_CHAIN_APPROX_SIMPLE);



    std::vector<cv::Point> approx;
    for (int i = 0; i < contours.size(); i++) {
        cv::approxPolyDP(cv::Mat(contours[i]), approx,
                         cv::arcLength(cv::Mat(contours[i]), true) * 0.1, true);
        // Skip small or non-convex objects
        if (std::fabs(cv::contourArea(contours[i])) < MIN_AREA_TAPE_FINDER
                || !cv::isContourConvex(approx)) {
            continue;
        }

        if (approx.size() == 3) {
        } else {
            // Detect and label circles
            double area = cv::contourArea(contours[i]);
            cv::Rect r = cv::boundingRect(contours[i]);
            int radius = r.width / 2;
            if (std::abs(1 - ((double) r.width / r.height)) <= 0.3
                    && std::abs(1 - (area / (CV_PI * std::pow(radius, 2))))
                    <= 0.3) {
                setLabel(final, "HOLE", contours[i]);
                circlesIndex.push_back(i);
            }
        }
    }

    for (int i = 0; i < contours.size(); i++) {
        // Sort out circles
        bool found = false;
        for (int j = 0; j < circlesIndex.size(); j++) {
            if (circlesIndex[j] == i) {
                found = true;
            }
        }
        if (!found) {
            cv::RotatedRect rect = minAreaRect(contours[i]);
            float rectConvertedArea = rect.size.width / pxRatioTape
                    * rect.size.height / pxRatioTape;
            if (searchTapeRotation) {
                if (isBorderTouched(rect, final.size()) && rectConvertedArea > 30.0) {
                    smdPart smd;
                    smd.x = rect.center.x;
                    smd.y = rect.center.y;
                    smd.rot = rect.angle;
                    if (rect.size.height < rect.size.width) {
                        smd.rot = std::fabs(smd.rot);
                    } else {
                        smd.rot = -(90.0 + smd.rot);
                    }
                    //ROS_INFO("Angle : %f Area: %f",rect.angle,rectConvertedArea);
                    smdObjects.push_back(smd);
                    drawRotatedRect(final, rect, CV_RGB(0, 0, 255));
                }
            } else {
                if ((!isBorderTouched(rect, final.size())
                     && rect.size.width / pxRatioTape
                     > ((partWidth_) / 100.0)
                     * (100.0 - ERROR_PERCENT_SMDTAPE)
                     && rect.size.width / pxRatioTape
                     < ((partWidth_) / 100.0)
                     * (100.0 + ERROR_PERCENT_SMDTAPE)
                     && rect.size.height / pxRatioTape
                     > ((partHeight_) / 100.0)
                     * (100.0 - ERROR_PERCENT_SMDTAPE)
                     && rect.size.height / pxRatioTape
                     < ((partHeight_) / 100.0)
                     * (100.0 + ERROR_PERCENT_SMDTAPE))
                        || (!isBorderTouched(rect, final.size())
                            && rect.size.height / pxRatioTape
                            > ((partWidth_) / 100.0)
                            * (100.0 - ERROR_PERCENT_SMDTAPE)
                            && rect.size.height / pxRatioTape
                            < ((partWidth_) / 100.0)
                            * (100.0 + ERROR_PERCENT_SMDTAPE)
                            && rect.size.width / pxRatioTape
                            > ((partHeight_) / 100.0)
                            * (100.0 - ERROR_PERCENT_SMDTAPE)
                            && rect.size.width / pxRatioTape
                            < ((partHeight_) / 100.0)
                            * (100.0 + ERROR_PERCENT_SMDTAPE)))
                {

                    // Calculate moments of image
                    Moments mu;
                    mu = moments(contours[i], false);
                    //Calculate mass center
                    Point2f mc;
                    mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

                    cv::approxPolyDP(cv::Mat(contours[i]), approx,
                                     cv::arcLength(cv::Mat(contours[i]), true) * 0.15,
                                     true);

                    if (approx.size() >= 2 && approx.size() <= 6) {

                            smdPart smd;
                            smd.x = mc.x;
                            smd.y = mc.y;
                            smd.rot = rect.angle;
                            if (rect.size.height < rect.size.width) {
                                smd.rot = std::fabs(smd.rot);
                            } else {
                                smd.rot = -(90.0 + smd.rot);
                            }
                            smdObjects.push_back(smd);
                            drawRotatedRect(final, rect, CV_RGB(0, 0, 255));
                            circle(final, Point2f(smd.x, smd.y), 5,
                                   CV_RGB(255, 0, 0), 3);
                    }

                }
            }
        }
    }

    if (!smdObjects.size()) {
        return false;
    }
    if (nearestPart(&smdObjects, &out, final.cols, final.rows)) {
        circle(final, Point2f(out.x, out.y), 5, CV_RGB(0, 0, 255), 3);
        out.x = (out.x - (final.cols / 2 - 1)) / pxRatioTape;
        out.y = ((final.rows / 2 - 1) - out.y) / pxRatioTape;
        return true;
    }else{
        return false;
    }
}

cv::Point2f padFinder::findPads(cv::Mat* input, bool startSelect,
                                cv::Point2f selectPad, std::vector<RotatedRect> &pads) {
    cv::Point2f outputPosition;
    outputPosition.x = 0.0;
    outputPosition.y = 0.0;

    cv::Mat bw;
    bw = classifyPixels(*input);
    //cv::erode(bw, bw, cv::Mat(), cv::Point(-1, -1), DILATE_ITERATIONS);

    //cv::imshow("classified pixel", bw);
    //cv::waitKey(0);

    cv::Mat final = input->clone();
    std::vector<std::vector<cv::Point> > contours;

    vector<Vec4i> hierarchy;
    cv::findContours(bw.clone(), contours, hierarchy, CV_RETR_TREE,
                     CV_CHAIN_APPROX_NONE);

    std::vector<cv::Point> approx;

    vector<Point2f> padPoints;
    vector<RotatedRect> padRects;

    for (int i = 0; i < contours.size(); i++) {

        //cv::drawContours(final, contours, i, CV_RGB(0, 255, 255), 2);

        // Approximate contour with accuracy proportional
        // to the contour perimeter
        cv::approxPolyDP(cv::Mat(contours[i]), approx,
                         cv::arcLength(cv::Mat(contours[i]), true) * 0.15, true);

        // Skip small or non-convex objects
        if (std::fabs(cv::contourArea(contours[i])) < MIN_CONTOUR_AREA){
            continue;
        }

        double area = cv::contourArea(contours[i]);

        if(hierarchy[i][3] != -1){
            continue;
        }

        //if(hierarchy[i][2] != -1 && cv::contourArea(contours.at(hierarchy[i][2])) / area > 0.05){
        if(hierarchy[i][2] != -1 && cv::contourArea(contours.at(hierarchy[i][2])) > MIN_VIA_AREA){
            // Is circle
            //RotatedRect viaPad = minAreaRect(contours[i]);
            //drawRotatedRect(final, viaPad, CV_RGB(0, 255, 0));
            float radius;
            cv::Point2f center;
            cv::minEnclosingCircle(contours.at(hierarchy[i][2]), center, radius);
            cv::circle(final, center, radius * 1.8, CV_RGB(0,255,0), -1);
            continue;
        }

        if (area > MIN_PAD_AREA) {

            RotatedRect pad = minAreaRect(contours[i]);

            if (!isBorderTouched(pad, input->size())) {
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
                            / pxRatioPcb;
                    outputPosition.y = ((input->rows / 2 - 1) - mc.y)
                            / pxRatioPcb;

                } else {
                    drawRotatedRect(final, pad, CV_RGB(255, 0, 0));
                }
            }
        }
    }

    // Convert to pap coordinate system and convert to mm

    for(size_t i = 0; i < padRects.size(); i++){
        RotatedRect rect = padRects.at(i);

        padRects.at(i).center.x = (input->cols - rect.center.x) / pxRatioPcb;
        padRects.at(i).center.y = (rect.center.y) / pxRatioPcb ;
        padRects.at(i).size.width = rect.size.height / pxRatioPcb;
        padRects.at(i).size.height = rect.size.width / pxRatioPcb;
    }

    pads = padRects;

    *input = final.clone();
    //cv::imshow("src", *input);
    //	cv::waitKey(0);
    return outputPosition;

}

