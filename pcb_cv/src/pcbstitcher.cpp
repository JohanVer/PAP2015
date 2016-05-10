#include <pcb_cv/pcbstitcher.h>

using namespace std;
using namespace cv;
using namespace cv::reg;

namespace stitcher{

static const char* DIFF_IM = "Image difference";
static const char* DIFF_REGPIX_IM = "Image difference: pixel registered";


PcbStitcher::PcbStitcher(double pxFactorInit)
{
    px_conv_factor = pxFactorInit;
    i_pic_ = 0;
    scan_pos_ = cv::Point2d(0,0);
    ll_corner_coord_ = cv::Point2d(0,0);
}

Mat PcbStitcher::translateImg(Mat &img, int offsetx, int offsety){
    Mat trans_mat = (Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    warpAffine(img,img,trans_mat,img.size()*2);
    return trans_mat;
}

void PcbStitcher::showDifference(const cv::Mat& image1, const cv::Mat& image2, const char* title)
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

void PcbStitcher::align(const cv::Mat& img1, cv::Mat& img2, cv::reg::MapShift &mapshift)
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

void PcbStitcher::blendImages(cv::Mat &final){

    std::vector<std::pair<cv::Mat,cv::Point2d> > &images = input_images;

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

    std::vector<cv::UMat > images_exposure;
    std::vector<cv::Point> corners;
    std::vector<cv::UMat > masks;

    for(size_t i = 0; i < images.size(); i++){

        std::cerr << "Feed image in exposure compensator: " << i << std::endl;

        cv::Mat feed_im = images.at(i).first;

        // Create UMAT
        cv::Mat temp;
        feed_im.convertTo(temp,CV_8UC3);
        cv::UMat feed_im2 = temp.at(i).getUMat( ACCESS_READ );
        images_exposure.push_back(feed_im2);

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

void PcbStitcher::reset(){
    input_images.clear();
    i_pic_ = 0;
    scan_pos_ = cv::Point2d(0,0);
    offsets_.clear();
}

void PcbStitcher::feedImage(cv::Mat image_in, cv::Point2d offset){
    // This point will be updated later on
    cv::Point2d p0(0,0);

    if(input_images.size() == 0){
        ll_corner_coord_ = offset;
    }

    // Add data to database
    input_images.push_back(std::pair<cv::Mat,cv::Point2d>(image_in, p0));
    offsets_.push_back(offset);

    if(i_pic_>0){
        cv::Mat &cur_image = input_images.at(i_pic_).first;
        cv::Mat &img_before = input_images.at(i_pic_-1).first;

        cv::Point2d &cur_point = offsets_.at(i_pic_);
        cv::Point2d &point_before = offsets_.at(i_pic_-1);

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
        cv::Mat mask = cv::Mat::ones(test.size(), CV_8U) * 255;
        cur_image.copyTo(cur_image, mask);

        showDifference(img_before, test, DIFF_IM);
        waitKey(1);
        //------

        align(img_before, cur_image, mapTest);

        cout << endl << "--- Result shift mapper ---" << endl;
        cout << Mat(mapTest.getShift()) << endl;

        cv::Point2d corrected_pos(-mapTest.getShift()[0], -mapTest.getShift()[1]);

        input_images.at(i_pic_).second.x = corrected_pos.x;
        input_images.at(i_pic_).second.y = corrected_pos.y;

        input_images.at(i_pic_).second = input_images.at(i_pic_).second + scan_pos_;
        scan_pos_ = input_images.at(i_pic_).second;

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
    i_pic_++;
}

}
