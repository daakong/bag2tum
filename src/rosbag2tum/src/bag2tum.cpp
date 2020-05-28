/*
 * DaKong@2020.5.21
 * Extract rosbag topics to TUM style png files, including: thermal image & rgb-d flow;
 *
 * TODO: Extract groundtruth, multiple cameras image and imu information;
 *
 * The code is based on
 * bag2tum:
 * king@2018.05.11
 * desp: rosbag to tum style png file
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <chrono>

#include <vector>
#include <assert.h>
#include <sys/stat.h>


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

// king
#include <unistd.h>
#include <iomanip>



#include <Eigen/Eigen>

#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/core/eigen.hpp>

//#include "PCLutils.hpp"

using namespace std;

const double coeff = 1000.0;

const std::string finCalib="/home/da/data/thermal2Kinect.yml";


cv::Mat matK_t, matDist_t, matK_h, matDist_h, matR_t_to_h, matt_t_to_h;


class ImageGrabber
{
public:
    ImageGrabber();
    ~ImageGrabber();
    void run();
private:
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void GrabDT_registerDepth(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const sensor_msgs::ImageConstPtr& msgT );
    void GrabRGBDT(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const sensor_msgs::ImageConstPtr& msgT );
public:
    // ros::NodeHandle node;
private:
    // ros::NodeHandle private_node("~");
    std::string save_folder_;    // where to save tum dataset
    std::string rgb_topic_;      // rgb image topic name
    std::string depth_topic_;    // depth image
    std::ofstream f1_;           // rgb.txt
    std::ofstream f2_;           // depth.txt

    std::string thermal_topic_;  //thermal image
    std::ofstream f3_;      //thermal.txt

};

// Constructor
ImageGrabber::ImageGrabber()
{

}   

// Destructor
ImageGrabber::~ImageGrabber()
{
    //
    f1_.close();
    f2_.close();
    f3_.close();
    ros::shutdown();
}

void ImageGrabber::run()
{
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    // parameters
    private_node.param<std::string>("save_folder", save_folder_, std::string("./image"));
    private_node.param<std::string>("rgb_topic", rgb_topic_, std::string("/kinect2/qhd/image_color_rect"));
    private_node.param<std::string>("depth_topic", depth_topic_, std::string("/kinect2/qhd/image_depth_rect"));

    private_node.param<std::string>("thermal_topic", thermal_topic_, std::string("/optris/thermal_image"));


    // # timestamp filename
    f1_.open(save_folder_+"/rgb.txt");
    f2_.open(save_folder_+"/depth.txt");
    f3_.open(save_folder_+ "/thermal.txt");
    f1_ << "# timestamp filename\n";
    f2_ << "# timestamp filename\n";
    f3_ << "# timestamp filename\n";

    // 
    std::cout << "Subscribe to: " << rgb_topic_ << " & " << depth_topic_ << " & " << thermal_topic_ << std::endl;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node, rgb_topic_, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(node, depth_topic_, 1);
    message_filters::Subscriber<sensor_msgs::Image> thermal_sub(node, thermal_topic_, 1);

//    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol_rgb_d;
//    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol_rgb_t;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync_pol_triple;

    message_filters::Synchronizer<sync_pol_triple> sync(sync_pol_triple(10), rgb_sub,depth_sub,thermal_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBDT,this,_1,_2,_3));

//    message_filters::Synchronizer<sync_pol> sync(sync_pol_rgb_d(10), rgb_sub,depth_sub);
//    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,this,_1,_2));
//
//    message_filters::Synchronizer<sync_pol> sync(sync_pol_rgb_t(10), rgb_sub,thermal_sub);
//    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,this,_1,_2));



    ros::spin();
}


void ImageGrabber::GrabRGBDT(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::ImageConstPtr& msgT)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

//    cv_bridge::CvImageConstPtr cv_ptrT;
//    try
//    {
//        cv_ptrT = cv_bridge::toCvShare(msgT);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }

    cv_bridge::CvImagePtr thermal_cv_ptr ;
    try
    {
        thermal_cv_ptr=cv_bridge::toCvCopy(msgT, sensor_msgs::image_encodings::MONO16);
    }
    catch  (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    assert(thermal_cv_ptr->image.type() == CV_16U);
    assert(thermal_cv_ptr->image.channels() == 1);

    cv::Mat m;
    cv::Mat v;
    cv::meanStdDev(thermal_cv_ptr->image,m,v);
    double maxinm;
    double mininm;
    cv::minMaxIdx(thermal_cv_ptr->image,&maxinm,&mininm);
    //std::cout << m << "\t" << v << "\t" << maxinm  << "\t" << mininm << std::endl;


    // record
    std::stringstream ss;
    ss << std::setiosflags(std::ios::fixed);  //只有在这项设置后，setprecision才是设置小数的位数。
    ss << std::setprecision(6);
    // rgb
    ss << cv_ptrRGB->header.stamp.toSec();
    std::string rgb_name = "rgb/" + ss.str() + ".png";
    f1_ << ss.str() << " " << rgb_name << std::endl;
    // depth
    ss.str("");
    ss << cv_ptrD->header.stamp.toSec();
    std::string depth_name = "depth/" + ss.str() + ".png";
    f2_ << ss.str() << " " << depth_name << std::endl;
    //thermal
    ss.str("");
  //  ss<< cv_ptrT->header.stamp.toSec();
    ss<< thermal_cv_ptr->header.stamp.toSec();  //尝试使用自己建立的ptr
    std::string thermal_name = "thermal/" + ss.str() + ".png";
    f3_ << ss.str() << " "<< thermal_name << std::endl;

    //
    rgb_name = save_folder_ + "/" + rgb_name;
    depth_name = save_folder_ + "/" + depth_name;
    thermal_name = save_folder_ + "/" + thermal_name;
    std::cout << "rgb_name: " << rgb_name << std::endl;
    cv::imwrite(rgb_name, cv_ptrRGB->image);
    cv::imwrite(depth_name, cv_ptrD->image);


    cv::Mat min = m-3.0f*v;
    cv::Mat max = m+3.0f*v;

    double alpha = (255.0f)/(6.0f*v.at<double>(0,0));

    for (int i = 0; i < (thermal_cv_ptr->image).rows; ++i)
    {
        for (int j = 0; j < (thermal_cv_ptr->image).cols; ++j)
        {
            double x = (double)(thermal_cv_ptr->image.at<ushort>(i,j)) - min.at<double>(0,0);
            if (x < 0.0f)
            {
                thermal_cv_ptr->image.at<ushort>(i,j) = 0;
            }
            else
            {
                if (x > max.at<double>(0,0))
                {
                    thermal_cv_ptr->image.at<ushort>(i,j) = 255;
                }
                else
                {
                    thermal_cv_ptr->image.at<ushort>(i,j) =  alpha*x;
                    // printf("%d\n", right_cv_ptr->image.at<ushort>(i,j));
                }
            }
        }
    }
    thermal_cv_ptr->image.convertTo(thermal_cv_ptr->image, CV_8U);

    cv::imwrite( thermal_name , thermal_cv_ptr->image);



    //cv::imwrite(thermal_name, cv_ptrT->image);

}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }




    // record
    std::stringstream ss;
    ss << std::setiosflags(std::ios::fixed);  //只有在这项设置后，setprecision才是设置小数的位数。
    ss << std::setprecision(6);
    // rgb
    ss << cv_ptrRGB->header.stamp.toSec();
    std::string rgb_name = "rgb/" + ss.str() + ".png";
    f1_ << ss.str() << " " << rgb_name << std::endl;
    // depth
    ss.str("");
    ss << cv_ptrD->header.stamp.toSec();
    std::string depth_name = "depth/" + ss.str() + ".png";
    f2_ << ss.str() << " " << depth_name << std::endl;
    //
    rgb_name = save_folder_ + "/" + rgb_name;
    depth_name = save_folder_ + "/" + depth_name;
    std::cout << "rgb_name: " << rgb_name << std::endl;
    cv::imwrite(rgb_name, cv_ptrRGB->image);
    cv::imwrite(depth_name, cv_ptrD->image);

}

void ImageGrabber::GrabDT_registerDepth(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::ImageConstPtr& msgT)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

//    cv_bridge::CvImageConstPtr cv_ptrT;
//    try
//    {
//        cv_ptrT = cv_bridge::toCvShare(msgT);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }

    cv_bridge::CvImagePtr thermal_cv_ptr ;
    try
    {
        thermal_cv_ptr=cv_bridge::toCvCopy(msgT, sensor_msgs::image_encodings::MONO16);
    }
    catch  (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    assert(thermal_cv_ptr->image.type() == CV_16U);
    assert(thermal_cv_ptr->image.channels() == 1);

    cv::Mat m;
    cv::Mat v;
    cv::meanStdDev(thermal_cv_ptr->image,m,v);
    double maxinm;
    double mininm;
    cv::minMaxIdx(thermal_cv_ptr->image,&maxinm,&mininm);
    //std::cout << m << "\t" << v << "\t" << maxinm  << "\t" << mininm << std::endl;


    // record
    std::stringstream ss;
    ss << std::setiosflags(std::ios::fixed);  //只有在这项设置后，setprecision才是设置小数的位数。
    ss << std::setprecision(6);
    // rgb
    ss << cv_ptrRGB->header.stamp.toSec();
    std::string rgb_name = "rgb/" + ss.str() + ".png";
    f1_ << ss.str() << " " << rgb_name << std::endl;
    // depth
    ss.str("");
    ss << cv_ptrD->header.stamp.toSec();
    std::string depth_name = "depth/" + ss.str() + ".png";
    f2_ << ss.str() << " " << depth_name << std::endl;
    //thermal
    ss.str("");
    //  ss<< cv_ptrT->header.stamp.toSec();
    ss<< thermal_cv_ptr->header.stamp.toSec();  //尝试使用自己建立的ptr
    std::string thermal_name = "thermal/" + ss.str() + ".png";
    f3_ << ss.str() << " "<< thermal_name << std::endl;
//////////////////////////////////////////////////////////////////////////////////////////
    cv::Mat min = m-3.0f*v;
    cv::Mat max = m+3.0f*v;

    double alpha = (255.0f)/(6.0f*v.at<double>(0,0));

    for (int i = 0; i < (thermal_cv_ptr->image).rows; ++i)
    {
        for (int j = 0; j < (thermal_cv_ptr->image).cols; ++j)
        {
            double x = (double)(thermal_cv_ptr->image.at<ushort>(i,j)) - min.at<double>(0,0);
            if (x < 0.0f)
            {
                thermal_cv_ptr->image.at<ushort>(i,j) = 0;
            }
            else
            {
                if (x > max.at<double>(0,0))
                {
                    thermal_cv_ptr->image.at<ushort>(i,j) = 255;
                }
                else
                {
                    thermal_cv_ptr->image.at<ushort>(i,j) =  alpha*x;
                    // printf("%d\n", right_cv_ptr->image.at<ushort>(i,j));
                }
            }
        }
    }
    thermal_cv_ptr->image.convertTo(thermal_cv_ptr->image, CV_8U);
    rgb_name = save_folder_ + "/" + rgb_name;
    std::cout << "rgb_name: " << rgb_name << std::endl;
    depth_name = save_folder_ + "/" + depth_name;
    thermal_name = save_folder_ + "/" + thermal_name;
    cv::imwrite(rgb_name, cv_ptrRGB->image);
    cv::imwrite( thermal_name , thermal_cv_ptr->image);

    //////////////////////////////////////////////////////////////////////////////////////////////
    //  host-kinect,2 targe-thermal,1



    //  from host to target
    //  Eigen default is colMajor
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> hostK(matK_h.ptr<double>(), matK_h.rows, matK_h.cols);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> targetK(matK_t.ptr<double>(), matK_t.rows, matK_t.cols);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> eigenR(matR_t_to_h.ptr<double>(), matR_t_to_h.rows, matR_t_to_h.cols);
    Eigen::Vector3d eigent;
    cv::cv2eigen(matt_t_to_h, eigent);

    //  T_t_h, host to targe, 2 to 1
    //  T_h_t, target to host, 1 to 2
    Eigen::Matrix4d T_h_t = Eigen::Matrix4d::Identity();

    T_h_t.block<3, 3>(0, 0) = eigenR;
    T_h_t.block<3, 1>(0, 3) = eigent;
    Eigen::Matrix4d T_t_h = T_h_t.inverse();

    Eigen::Matrix3d hostKinv = hostK.inverse();
    cv::Mat hostDepth ;// = cv::imread(cv_ptrD->image, CV_LOAD_IMAGE_UNCHANGED );
     cv_ptrD->image.copyTo(hostDepth);
    //    cv::Mat undirstHostDepth;
//    cv::undistort(hostDepth, undirstHostDepth, matK_h, matDist_h);
//    undirstHostDepth.copyTo(hostDepth);

     int hTarget=thermal_cv_ptr->image.rows;
     int wTarget=thermal_cv_ptr->image.cols;
     const int wHost=hostDepth.cols;
     const int hHost=hostDepth.rows;

     cv::Mat targetDepth = cv::Mat::zeros(hTarget,wTarget,CV_16UC1);
     for (int i=0; i<hHost; i++){
         for (int j=0; j<wHost; j++){
             if (hostDepth.at<ushort>(i,j)<=0){
                 continue;
             }
             const double depth = static_cast<double>(hostDepth.at<ushort>(i, j)) / coeff;

             std::cout << depth << "depth!" << endl;
             assert(depth > 0 && depth <=256);
             const Eigen::Vector3d pixel(static_cast<double>(j), static_cast<double>(i), 1.0);
             const Eigen::Vector3d pcHost = hostKinv * (pixel * depth);
             Eigen::Vector3d pcTarget = T_t_h.block<3, 3>(0, 0) * pcHost + T_t_h.block<3, 1>(0, 3);
             const double depthTarget = pcTarget(2);
             if (depthTarget <= 0.001) {
                 continue;
             }
             pcTarget /= depthTarget;
             const Eigen::Vector3d pixelTarget = targetK * pcTarget;

             if ((pixelTarget(0) > 0) && (pixelTarget(0) < wTarget) && (pixelTarget(1) > 0) && (pixelTarget(1) < hTarget)) {
                 targetDepth.at<ushort>(pixelTarget(1), pixelTarget(0)) = static_cast<ushort>(depthTarget * coeff);

             }

         }
     }


    //输出原始的depth图像
   // cv::imwrite(depth_name, cv_ptrD->image);
    //std::cout << "Has exported origin depth image."

    //输出投影过的depth图像
     cv::imwrite(depth_name, targetDepth);
     std::cout << "Exported depth image projected to Thermal image.";


    //cv::imwrite(thermal_name, cv_ptrT->image);

}

int main(int argc, char **argv)
{

    cv::FileStorage fs(finCalib,cv::FileStorage::READ);

    fs["K1"] >> matK_t;
    fs["dist1"] >> matDist_t;
    fs["K2"] >> matK_h;
    fs["dist2"] >> matDist_h;
    fs["R1to2"] >> matR_t_to_h;
    fs["t1to2"] >> matt_t_to_h;
    fs.release();


    ros::init(argc, argv, "bag2img_node");
    ros::start();
    // Create bag2img system. 
    ImageGrabber igb;
    igb.run();
    // ros::spin();
    ros::shutdown();
    
    return 0;
}



