/*
 * @file    image_object_detector.cpp
 * @brief テンプレートマッチングによる物体認識
 * @author Toru Nishikawa
 * @date    09  7月 2015
 *
 * @per
 *
 */

//ROS header file
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<ros/package.h>
#include<robocup_perception/Cameralaunch.h>

//OpenCV header file
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#define IMAGENUM 0
#define NOUSETEMPLATE


using namespace cv;
static const std::string OPENCV_WINDOW = "image_window";
Mat tmp_img;
int imagenum = 0;
std::string path;
std::string detect_image;
std::string detect_object;

class ImageConverter{

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    //Mat object_img[IMAGENUM];
    std::vector<Mat> img;




    public:
    ImageConverter() : it_(nh_)
    {
        Imageloader();
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    /*
     * @brief 複数画像の読み込み
     */
    void Imageloader()
    {
        for(imagenum = 0; imagenum < IMAGENUM; imagenum++){
            std::stringstream loadfile;
            std::stringstream number;
            loadfile << "image_" << std::setw(3) << std::setfill('0') <<imagenum <<".jpg";
            number << imagenum;
            path = ros::package::getPath("robocup_perception")+"/template_image/";
            detect_image = loadfile.str();
            detect_object = path + detect_image;
            //object_img[imagenum] = imread(detect_object);
            img.push_back(imread(detect_object,1));
            if(!img[imagenum].empty()){
                std::cerr<<"Success load image "<< detect_image <<std::endl;
            }else{
                std::cerr<<"Faild load image, load miss image is "<< detect_image <<std::endl;
            }
        }
    }


    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cap;
        try{
            cap = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            tmp_img = cap->image;
        }
        catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception : %s", e.what());
            return;
        }

        if(cap->image.empty()){
            std::cerr<<"No capture image"<<std::endl;
        }


#ifdef USETEMPLATE
        for(int i = 0; i < img.size(); i++){
            Mat result_img;
            matchTemplate(tmp_img, img[i], result_img, CV_TM_CCOEFF_NORMED);
            for(int j = 0; j < IMAGENUM; j++){
                cv::Rect roi_rect(0, 0, tmp_img.cols, tmp_img.rows);
                cv::Point max_pt;
                double maxVal;
                minMaxLoc(result_img, NULL, &maxVal, NULL, &max_pt);
                if(maxVal<0.25) break;
                std::cout<<maxVal<<std::endl;
                roi_rect.x = max_pt.x;
                roi_rect.y = max_pt.y;
                cv::rectangle(tmp_img, roi_rect, cv::Scalar(0,255,255),3);
            }
        }
#endif

        imshow(OPENCV_WINDOW, tmp_img);
        waitKey(3);
        image_pub_.publish(cap->toImageMsg());
    }
};

bool camera_call_srv(robocup_perception::Cameralaunch::Request &req,
        robocup_perception::Cameralaunch::Response &res)
{
    bool save_flag = req.flag;
    if(save_flag == true){
#ifdef NOUSETEMPLATE
        Mat roiimage;
        std::string recog_file;
        std::string recog_image;
        std::stringstream save_image;
        save_image << "image" << imagenum;
        recog_file = ros::package::getPath("robocup_perception")+"/object_image/";
        recog_image = recog_file + save_image.str() + ".jpg";
        //cv::rectangle(tmp_img, cv::Point(243, 90), cv::Point(388, 366), cv::Scalar(0,0,255), 3, 4);
        //cvSetImageROI(tmp_img, cv::Rect(243,90,150,280));
        roiimage = tmp_img(cv::Rect(243,90,150,280));
        if(save_flag == true){
            imwrite(recog_image,roiimage);
            res.result = true;
            res.name = save_image.str();

            std::cout<<res.name<<std::endl;
        }else{
            std::cout<<" No data "<<std::endl;
        }
        // cvReleaseImageROI(tmp_img);
#endif
        return true;
    }else{
        return false;
    }

}


int main(int argc ,char **argv)
{
    ros::init(argc, argv, "image_object_detector");
    ros::NodeHandle node;
    ros::ServiceServer camera_srv = node.advertiseService("camera_call",camera_call_srv);
    ImageConverter ic;


    ros::spin();
}
