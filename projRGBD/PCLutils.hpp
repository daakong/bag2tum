/*
 * @Author: chen shenzhou 
 * @Date: 2019-06-19 15:53:47 
 * @Last Modified by: chen shenzhou
 * @Last Modified time: 2019-06-19 20:50:01
 */


/*! \mainpage PCLutils Library
 *
 * PCL utils library for C++.
 *
 * Written by shenzhou chen,
 * ZheJiang University
 *
 * \section requirements Requirements
 * This library requires the PCL, Eigen and OpenCV libraries.
 *
 */

#ifndef PCLUTILS_H
#define PCLUTILS_H

#include <iostream>
#include <fstream>
#include <thread>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <math.h>

#include<algorithm>
#include<chrono>
#include<sstream>
#include<string>
#include <assert.h>

#include <Eigen/Eigen>

#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/core/eigen.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace po = boost::program_options;
using std::cout;
using std::endl;
using std::cerr;
using std::string;
using namespace pcl;
using namespace Eigen;
using namespace cv;

namespace PCLutils {

/**  @brief the function to get fake rgb color
 *
*/
void GroundColorMix(unsigned char &r, unsigned char &g, unsigned char &b, double x, double min, double max);

/**  @brief the function to get fake rgb color
 * @param value input value between 0 and 1
 * @return cv::Scalar
*/
cv::Scalar fakeColor(float value);

/**  @brief the function to read each line of string data in a txt file
 *
 * @param File path of file.
 * @param vNames vector of date
*/
void LoadSingleFile(const std::string &File, std::vector<std::string> &vNames);

/**  @brief the function to scale the intrisic matrix when image resolution changed
 *
 * @param K intrisic matrix.
 * @param wnew new weight.
 * @param hnew new height.
 * @param wold old weight.
 * @param wold old height.
*/
Eigen::Matrix3d scaledK(const Eigen::Matrix3d &K, const int wnew, const int hnew, const int wold, const int hold);

/**  @brief the function to get the pixel position(u,v,depth) corresponding to the lidar point
 *
 * @param pl the 3d point of lidar
 * @param Tcl the transformation from lidar to camera
 * @param K intrisic matrix.
 * @return the pixel position [u, v, depth]
*/
Eigen::Vector3d getImageUVDfromLidar(const Eigen::Vector3d &pl, const Eigen::Matrix4d &Tcl, const Eigen::Matrix3d &K);

/**  @brief the function to get the lidar point from the pixel positon of camera
 *
 * @param uvd the pixel position of image [u, v, depth]
 * @param Tcl the transformation from lidar to camera
 * @param Kinv the inverse of intrisic matrix.
 * @return the 3d lidar point [x, y, z]
*/
Eigen::Vector3d getPointfromImageUVD(const Eigen::Vector3d &uvd, const Eigen::Matrix4d &Tlc, const Eigen::Matrix3d &Kinv);

/**  @brief the function to get the point cloud in the lidar coordinate from the depth image
 *
 * @param cloud output the point cloud in the lidar coordinate
 * @param image the depth image in the color coordinate (short interger Multiplied by 256.0)
 * @param Tcl the transformation from lidar to camera
 * @param Kinv the inverse of intrisic matrix.
*/
void depthMap2PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const cv::Mat &image,
                    const Eigen::Matrix4d &Tlc, const Eigen::Matrix<double, 3, 3> &Kinv);

/**  @brief the function to get the colored point cloud in the lidar coordinate from the masked depth and intensity image
 *
 * @param cloud output the colored point cloud by intensity in the lidar coordinate
 * @param image the depth image in the color coordinate (short interger Multiplied by 256.0)
 * @param imageI the intensity image (short interger Multiplied by 256.0)
 * @param mask the mask image (0 or not)
 * @param Tcl the transformation from lidar to camera
 * @param Kinv the inverse of intrisic matrix.
*/
void DIMapMasked2PointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const cv::Mat &image,
                    const cv::Mat &imageI, const cv::Mat &mask, const Eigen::Matrix4d &Tlc,
                    const Eigen::Matrix<double, 3, 3> &Kinv);

/**  @brief the function to get the colored point cloud in the lidar coordinate from the depth and intensity image
 *
 * @param cloud output the colored point cloud by intensity in the lidar coordinate
 * @param image the depth image in the color coordinate (short interger Multiplied by 256.0)
 * @param imageI the intensity image (short interger Multiplied by 256.0)
 * @param Tcl the transformation from lidar to camera
 * @param Kinv the inverse of intrisic matrix.
*/
void DIMap2PointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const cv::Mat &image,
                    const cv::Mat &imageI, const Eigen::Matrix4d &Tlc,
                    const Eigen::Matrix<double, 3, 3> &Kinv);

/**  @brief the function to get the colored point cloud in the lidar coordinate from the depth and RGB image
 *
 * @param cloud output the colored point cloud by RGB in the lidar coordinate
 * @param imageDepth the depth image in the color coordinate (short interger Multiplied by coeff_depth)
 * @param imageRGB the RGB image
 * @param Tcl the transformation from lidar to camera
 * @param Kinv the inverse of intrisic matrix.
 * @param coeff_depth the metrix of depth (1000.0 in NYU, 256.0 in KITTI).
*/
void DRGBMap2PointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const cv::Mat &imageDepth,
                    const cv::Mat &imageRGB, Eigen::Matrix4d &Tlc,
                    const Eigen::Matrix<double, 3, 3> &Kinv, const double coeff_depth);

/**  @brief the function to estimate the normal of point cloud by knn
 *
 * @param cloud input ptr of the point cloud
 * @param cloud_normals output the normals
 * @param knnRadius input the radius for knn search
*/
void estimateNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, const double knnRadius);


/**  @brief the function to project the point from lidar to RGB image, colored with intensity
 *
 * @param cloud input the point cloud with intensity from lidar
 * @param image_raw the RGB image
 * @param Tcl the transformation from lidar to camera
 * @param K the intrisic matrix.
 * @param bshow imshow or not.
 * @param showname the name for imshow.
 * @return the RGB image superimposed with lidar intensity information
*/
cv::Mat createImageFromLidarIntensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                    const cv::Mat &image_raw, const Eigen::Matrix4d &Tcl,
                                    const Eigen::Matrix3d &K, const bool bshow, const string &showname);

/**  @brief the function to project the point from lidar to RGB image, colored with depth
 *
 * @param cloud input the point cloud with depth from lidar
 * @param image_raw the RGB image
 * @param Tcl the transformation from lidar to camera
 * @param K the intrisic matrix.
 * @param bshow imshow or not.
 * @param showname the name for imshow.
 * @return the RGB image superimposed with lidar depth information
*/
cv::Mat createImageFromLidarRange(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                const cv::Mat &image_raw, const Eigen::Matrix4d &Tcl,
                                const Eigen::Matrix3d &K, const bool bshow, const string &showname);

/**  @brief the function to Count the distribution of 16-bit image
 *
 * @param image input the 16-bit image
*/
void calcuMat16U(const cv::Mat & image);

// class PCLutils
// {
// public:
//     PCLutils();
// };

}   //  namespace PCLutils
#endif // PCLUTILS_H
