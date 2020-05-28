/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/

#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <tuple>
#include "math.h"

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "cv_bridge/cv_bridge.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/Pose.h"
#include <image_transport/image_transport.h>
#include "dso_ros/Flag.h"


#define M_PI       3.14159265358979323846

std::string calib = "";
std::string vignetteFile = "";
std::string gammaFile = "";
std::string right_calib = "";
std::string right_vignetteFile = "";
std::string right_gammaFile = "";
// std::string s = "";
std::string out_root = "./data";
//	left grey, left rgb, right optris, kinect rgb, kinect depth
std::tuple<std::string, std::string, std::string, std::string, std::string> outPaths;

bool useSampleOutput=false;
int flag_state_now = 0;
double offsetOptrisRGB = 0.06;

image_transport::Publisher mImg_pub;

using namespace dso;

void parseArgument(char* arg)
{
	int option;
	char buf[1000];

	if(1==sscanf(arg,"sampleoutput=%d",&option))
	{
		if(option==1)
		{
			useSampleOutput = true;
			printf("USING SAMPLE OUTPUT WRAPPER!\n");
		}
		return;
	}

	if(1==sscanf(arg,"quiet=%d",&option))
	{
		if(option==1)
		{
			setting_debugout_runquiet = true;
			printf("QUIET MODE, I'll shut up!\n");
		}
		return;
	}


	if(1==sscanf(arg,"nolog=%d",&option))
	{
		if(option==1)
		{
			setting_logStuff = false;
			printf("DISABLE LOGGING!\n");
		}
		return;
	}

	if(1==sscanf(arg,"nogui=%d",&option))
	{
		if(option==1)
		{
			disableAllDisplay = true;
			printf("NO GUI!\n");
		}
		return;
	}
	if(1==sscanf(arg,"nomt=%d",&option))
	{
		if(option==1)
		{
			multiThreading = false;
			printf("NO MultiThreading!\n");
		}
		return;
	}
	if(1==sscanf(arg,"calib=%s",buf))
	{
		calib = buf;
		printf("loading calibration from %s!\n", calib.c_str());
		return;
	}
	if(1==sscanf(arg,"vignette=%s",buf))
	{
		vignetteFile = buf;
		printf("loading vignette from %s!\n", vignetteFile.c_str());
		return;
	}

	if(1==sscanf(arg,"gamma=%s",buf))
	{
		gammaFile = buf;
		printf("loading gammaCalib from %s!\n", gammaFile.c_str());
		return;
	}

	if(1==sscanf(arg,"right_calib=%s",buf))
	{
		right_calib = buf;
		printf("loading calibration from %s!\n", right_calib.c_str());
		return;
	}
	if(1==sscanf(arg,"right_vignette=%s",buf))
	{
		right_vignetteFile = buf;
		printf("loading vignette from %s!\n", right_vignetteFile.c_str());
		return;
	}

	if(1==sscanf(arg,"right_gamma=%s",buf))
	{
		right_gammaFile = buf;
		printf("loading gammaCalib from %s!\n", right_gammaFile.c_str());
		return;
	}

	if(1==sscanf(arg,"out_root=%s",buf)) {
		out_root = buf;
		printf("output path %s!\n", out_root.c_str());
		return;
	}
	else
	{
		printf("no out_root path !!\n");
		exit(1);
	}

	printf("could not parse argument \"%s\"!!\n", arg);
}

bool IsFileExist(const std::string& path) {
  if (access(path.c_str(), F_OK) == 0) {
    return true;
  } else {
    return false;
  }
}
void CreateFolder(const std::string& path) {
//   if (IsFileExist(path)) {
//     std::cout << path << "is already exist";
//     return;
//   }
  const std::string path_make = "mkdir -p " + path;
  const int err = system(path_make.c_str());
//   const int err = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (err == -1) {
    std::cout << "can't create " << path;
  }
}


FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
Undistort* rightUndistorter = 0;
int frameID = 0;

void monoVidCb(const sensor_msgs::ImageConstPtr img)
{
	sensor_msgs::Image copiedImg = *img;
	copiedImg.header.stamp = img->header.stamp+ros::Duration(offsetOptrisRGB);
	mImg_pub.publish(copiedImg);
}

void flagCb(const dso_ros::Flag flag)
{
	flag_state_now = flag.flag_state;
	printf("%d\n", flag.flag_state);
}

void llh2xyz(double llh[3],double xyz[3]) //Î³¾­¸ß ×ª µØÐÄµØÇò×ø±êÏµ
{

	double phi = llh[0]*M_PI/180.0f;
	double lambda = llh[1]*M_PI/180.0f;
	double h = llh[2];

	double a = 6378137.0000f;	// earth semimajor axis in meters
	double b = 6356752.3142f;	// earth semiminor axis in meters	
	double e = sqrt (1.0f-(b/a)*(b/a));

	double sinphi = sin(phi);
	double cosphi = cos(phi);
	double coslam = cos(lambda);
	double sinlam = sin(lambda);
	double tan2phi = (tan(phi))*(tan(phi));
	double tmp = 1.0f - e*e;
	double tmpden = sqrt( 1.0f + tmp*tan2phi );

	double x = (a*coslam)/tmpden + h*coslam*cosphi;

	double y = (a*sinlam)/tmpden + h*sinlam*cosphi;

	double tmp2 = sqrt(1.0f - e*e*sinphi*sinphi);
	double z = (a*tmp*sinphi)/tmp2 + h*sinphi;

	xyz[0] = x;
	xyz[1] = y;
	xyz[2] = z;

}

void xyz2enu(double xyz[3],double orgllh[3],double enu[3]) //µØÐÄµØÇò×ø±êÏµ ×ª ¶«±±ÌìµØÀí×ø±êÏµ
{

	double tmpxyz[3];
	double tmporg[3];
	double difxyz[3];
	//double orgllh[3];
	double orgxyz[3];
	double phi,lam,sinphi,cosphi,sinlam,coslam;

	llh2xyz(orgllh,orgxyz);

	int i;
	for(i=0;i<3;i++)
	{
		tmpxyz[i]=xyz[i];
		tmporg[i]=orgxyz[i];
		difxyz[i]=tmpxyz[i]-tmporg[i];
	}

	//xyz2llh(orgxyz,orgllh);

	phi=orgllh[0]*M_PI/180.0f;
	lam=orgllh[1]*M_PI/180.0f;
	sinphi=sin(phi);
	cosphi=cos(phi);
	sinlam=sin(lam);
	coslam=cos(lam);
	double R[3][3]={{-sinlam,coslam,0.0f},{-sinphi*coslam,-sinphi*sinlam,cosphi},{cosphi*coslam,cosphi*sinlam,sinphi}};

	enu[0]=0;
	enu[1]=0;
	enu[2]=0;
	for(i=0;i<3;i++)
	{
		enu[0]=enu[0]+R[0][i]*difxyz[i];
		enu[1]=enu[1]+R[1][i]*difxyz[i];
		enu[2]=enu[2]+R[2][i]*difxyz[i];		
	}
}

void gtCb(const sensor_msgs::NavSatFix nvdata)
{
    FILE *f;
    char buf3[1000];
    snprintf(buf3, 1000, "%s/gt.txt",out_root.c_str());
    f = fopen(buf3, "a");

    double orgllh[3] = { (30.263254f), (120.115654f), 33.196903};
    double llh[3] = {(nvdata.latitude), (nvdata.longitude), nvdata.altitude};
    double xyz[3];
	llh2xyz(llh,xyz);
	double enu[3];
	llh2xyz(llh,xyz);
	xyz2enu(xyz,orgllh,enu);
    fprintf(f, "%lf %lf %lf %lf\n", nvdata.header.stamp.toSec(), enu[0],enu[1],enu[2]);
    fclose(f);
}

void kinectRGBCb(const sensor_msgs::ImageConstPtr img)
{
	//	kinect rgb
	cv_bridge::CvImagePtr kinect_color_cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	assert(kinect_color_cv_ptr->image.type() == CV_8UC3);
	assert(kinect_color_cv_ptr->image.channels() == 3);

	char bufkrgb[1000];
    snprintf(bufkrgb, 1000, "%s/%lf.png", std::get<3>(outPaths).c_str(), kinect_color_cv_ptr->header.stamp.toSec());
    imwrite( bufkrgb, kinect_color_cv_ptr->image );
}

void kinectDepthCb(const sensor_msgs::ImageConstPtr img)
{
	//	kinect depth
	cv_bridge::CvImagePtr kinect_depth_cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_16UC1);
	assert(kinect_depth_cv_ptr->image.type() == CV_16UC1);
	assert(kinect_depth_cv_ptr->image.channels() == 1);

    char bufkd[1000];
    snprintf(bufkd, 1000, "%s/%lf.png", std::get<4>(outPaths).c_str(), kinect_depth_cv_ptr->header.stamp.toSec());
    imwrite( bufkd, kinect_depth_cv_ptr->image);
}

void rgbCb(const sensor_msgs::ImageConstPtr img)
{
	//	color
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	assert(cv_ptr->image.type() == CV_8UC3);
	assert(cv_ptr->image.channels() == 3);

	char bufgrey[1000];
    snprintf(bufgrey, 1000, "%s%lf.png", std::get<0>(outPaths).c_str(), cv_ptr->header.stamp.toSec());
	cv::Mat tis_grey;
	cv::cvtColor(cv_ptr->image, tis_grey, CV_BGR2GRAY);
    imwrite( bufgrey, tis_grey);
	char bufrgb[1000];
    snprintf(bufrgb, 1000, "%s/%lf.png", std::get<1>(outPaths).c_str(), cv_ptr->header.stamp.toSec());
    imwrite( bufrgb, cv_ptr->image );

    FILE *f;
    char buf3[1000];
    snprintf(buf3, 1000, "%s/times.txt",out_root.c_str());
    f = fopen(buf3, "a");
    fprintf(f, "%lf\n", cv_ptr->header.stamp.toSec());
    fclose(f);
}

void optrisCb(const sensor_msgs::ImageConstPtr img)
{
	//	optris
	cv_bridge::CvImagePtr right_cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
	assert(right_cv_ptr->image.type() == CV_16U);
	assert(right_cv_ptr->image.channels() == 1);

	cv::Mat m;
	cv::Mat v;
	cv::meanStdDev(right_cv_ptr->image,m,v);
	double maxinm;
	double mininm;
	cv::minMaxIdx(right_cv_ptr->image,&maxinm,&mininm);
	std::cout << m << "\t" << v << "\t" << maxinm  << "\t" << mininm << std::endl;


	cv::Mat min = m-3.0f*v;
	cv::Mat max = m+3.0f*v;

	double alpha = (255.0f)/(6.0f*v.at<double>(0,0));

	for (int i = 0; i < (right_cv_ptr->image).rows; ++i)
	{
		for (int j = 0; j < (right_cv_ptr->image).cols; ++j)
		{
			double x = (double)(right_cv_ptr->image.at<ushort>(i,j)) - min.at<double>(0,0);
			if (x < 0.0f)
			{
				right_cv_ptr->image.at<ushort>(i,j) = 0;
			}
			else
			{
				if (x > max.at<double>(0,0))
				{
					right_cv_ptr->image.at<ushort>(i,j) = 255;
				}
				else
				{
					right_cv_ptr->image.at<ushort>(i,j) =  alpha*x;
					// printf("%d\n", right_cv_ptr->image.at<ushort>(i,j));
				}
			}
		}
	}
	right_cv_ptr->image.convertTo(right_cv_ptr->image, CV_8U);

    char bufoptris[1000];
    snprintf(bufoptris, 1000, "%s/%lf.png", std::get<2>(outPaths).c_str(), right_cv_ptr->header.stamp.toSec());
    imwrite( bufoptris, right_cv_ptr->image);
}





void chatterCallback(const geometry_msgs::PoseStamped & msg)
{
	double time = msg.header.stamp.toSec();


    FILE *fab;
    char buf4[1000];
    snprintf(buf4, 1000, "%s/groundtruth.txt",out_root.c_str());
    fab = fopen(buf4, "a");

    fprintf(fab, "%lf %lf %lf %lf %lf %lf %lf %lf\n", time,  msg.pose.position.x,  msg.pose.position.y,  msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    fclose(fab);

}

void chatterCallback2(const geometry_msgs::PoseStamped & msg)
{
	double time = msg.header.stamp.toSec();


    FILE *fab;
    char buf4[1000];
    snprintf(buf4, 1000, "%s/groundtruth2.txt",out_root.c_str());
    fab = fopen(buf4, "a");

    fprintf(fab, "%lf %lf %lf %lf %lf %lf %lf %lf\n", time,  msg.pose.position.x,  msg.pose.position.y,  msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    fclose(fab);

}

void chatterCallback3(const geometry_msgs::PoseStamped & msg)
{
	double time = msg.header.stamp.toSec();


    FILE *fab;
    char buf4[1000];
    snprintf(buf4, 1000, "%s/groundtruth3.txt",out_root.c_str());
    fab = fopen(buf4, "a");

    fprintf(fab, "%lf %lf %lf %lf %lf %lf %lf %lf\n", time,  msg.pose.position.x,  msg.pose.position.y,  msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    fclose(fab);

}




int main( int argc, char** argv )
{

	ros::init(argc, argv, "dso_live");

	for(int i=1; i<argc;i++) parseArgument(argv[i]);

	if(out_root.at( out_root.length() - 1 ) != '/') out_root = out_root+"/";

	outPaths = std::make_tuple(out_root + "left/", out_root + "leftRGB/", out_root + "right/",
				out_root + "kinectRGB/", out_root + "kinectDepth/");
	std::cout << "save in " << out_root << std::endl;
	std::cout << "save rgb in " << std::get<0>(outPaths) << std::endl;
	CreateFolder(std::get<0>(outPaths));
	CreateFolder(std::get<1>(outPaths));
	CreateFolder(std::get<2>(outPaths));
	CreateFolder(std::get<3>(outPaths));
	CreateFolder(std::get<4>(outPaths));

	setting_desiredImmatureDensity = 1000;
	setting_desiredPointDensity = 1200;
	setting_minFrames = 5;
	setting_maxFrames = 7;
	setting_maxOptIterations=4;
	setting_minOptIterations=1;
	setting_logStuff = false;
	setting_kfGlobalWeight = 1.3;


	printf("MODE WITH CALIBRATION, but without exposure times!\n");
	setting_photometricCalibration = 2;
	setting_affineOptModeA = 0;
	setting_affineOptModeB = 0;


	setCameraNum(2);
    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

    setGlobalCalib(
            (int)undistorter->getSize()[0],
            (int)undistorter->getSize()[1],
            undistorter->getK().cast<float>());

    rightUndistorter = Undistort::getUndistorterForFile(right_calib, right_gammaFile, right_vignetteFile);


    setGlobalCalibOthers(
            (int)rightUndistorter->getSize()[0],
            (int)rightUndistorter->getSize()[1],
            rightUndistorter->getK().cast<float>(),
            0);


    

    fullSystem = new FullSystem();
    fullSystem->linearizeOperation=false;
    

    if(!disableAllDisplay)
	    fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
	    		 (int)undistorter->getSize()[0],
	    		 (int)undistorter->getSize()[1]));


    if(useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());


    if(undistorter->photometricUndist != 0)
    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

    ros::NodeHandle nh("~");
    std::string imagetopic_L, imagetopic_R, imagetopic_kinectRGB, imagetopic_kinectD;
    nh.getParam("imagetopic_l", imagetopic_L);
    nh.getParam("imagetopic_r", imagetopic_R);
    nh.getParam("imagetopic_krgb", imagetopic_kinectRGB);
    nh.getParam("imagetopic_d", imagetopic_kinectD);
    nh.getParam("offset", offsetOptrisRGB);
	std::cout << "offset between tis and optris: " << offsetOptrisRGB << std::endl;

    ros::Subscriber flagStateSub = nh.subscribe("/optris/flag_state", 1, &flagCb);
    ros::Subscriber gtSub = nh.subscribe("/fix", 1, &gtCb);
    // ros::Subscriber imgSub = nh.subscribe(imagetopic_L, 1, &monoVidCb);
    ros::Subscriber left_sub = nh.subscribe(imagetopic_L, 3, &rgbCb);
    ros::Subscriber right_sub = nh.subscribe(imagetopic_R, 3, &optrisCb);
    ros::Subscriber kinect_rgb_sub = nh.subscribe(imagetopic_kinectRGB, 3, &kinectRGBCb);
    ros::Subscriber kinect_depth_sub = nh.subscribe(imagetopic_kinectD, 3, &kinectDepthCb);
    image_transport::ImageTransport it(nh);
    mImg_pub = it.advertise("/camera/image_raw2", 1);
    // ros::Subscriber imgSub = nh.subscribe("/Robot_1/pose", 10, &chatterCallback);
    // ros::Subscriber imgSub2 = nh.subscribe("/Robot_2/pose", 10, &chatterCallback2);
    // ros::Subscriber imgSub3 = nh.subscribe("/Robot_3/pose", 10, &chatterCallback3);
    // message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/image_raw2", 10, &rgbCb);
    // message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, imagetopic_R, 10, &optrisCb);
    // message_filters::Subscriber<sensor_msgs::Image> kinect_color_sub(nh, imagetopic_kinectRGB, 10);
    // message_filters::Subscriber<sensor_msgs::Image> kinect_depth_sub(nh, imagetopic_kinectD, 10);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    // // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
	// // 														sensor_msgs::Image, sensor_msgs::Image> sync_pol4;

    
    // message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    // // message_filters::Synchronizer<sync_pol4> sync4(sync_pol4(10), left_sub,right_sub, kinect_color_sub, kinect_depth_sub);
    // // sync.setMaxIntervalDuration(ros::Duration(0.005));
    // sync.registerCallback(boost::bind(&vidCb,_1,_2));
    // sync4.registerCallback(boost::bind(&mssensorCb,_1,_2, _3, _4));

    ros::spin();

    fullSystem->printResult("poseresult.txt");

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
        delete ow;
    }

    delete undistorter;
    delete fullSystem;
    delete rightUndistorter;
	return 0;
}

