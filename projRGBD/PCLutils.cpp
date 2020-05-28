/*
 * @Author: chen shenzhou 
 * @Date: 2019-06-19 15:53:50 
 * @Last Modified by: chen shenzhou
 * @Last Modified time: 2019-06-19 21:08:16
 */
#include "PCLutils.hpp"

namespace PCLutils {

void GroundColorMix(unsigned char &r, unsigned char &g, unsigned char &b, double x, double min, double max)
{
	x = x * 360.0;
	const double posSlope = (max - min) / 60.0;
	const double negSlope = (min - max) / 60.0;
	if (x < 60)
	{
		r = (unsigned char)max;
		g = (unsigned char)(posSlope * x + min);
		b = (unsigned char)min;
		return;
	}
	else if (x < 120)
	{
		r = (unsigned char)(negSlope * x + 2 * max + min);
		g = (unsigned char)max;
		b = (unsigned char)min;
		return;
	}
	else if (x < 180)
	{
		r = (unsigned char)min;
		g = (unsigned char)max;
		b = (unsigned char)(posSlope * x - 2 * max + min);
		return;
	}
	else if (x < 240)
	{
		r = (unsigned char)min;
		g = (unsigned char)(negSlope * x + 4 * max + min);
		b = (unsigned char)max;
		return;
	}
	else if (x < 300)
	{
		r = (unsigned char)(posSlope * x - 4 * max + min);
		g = (unsigned char)min;
		b = (unsigned char)max;
		return;
	}
	else
	{
		r = (unsigned char)max;
		g = (unsigned char)min;
		b = (unsigned char)(negSlope * x + 6 * max);
		return;
	}
}

cv::Scalar fakeColor(float value) {
    const float posSlope = 255 / 60.0;
    const float negSlope = -255 / 60.0;
    value *= 255;
    cv::Vec3f color;
    if (value < 60) {
        color[0] = 255;
        color[1] = posSlope * value + 0;
        color[2] = 0;
    } else if (value < 120) {
        color[0] = negSlope * value + 2 * 255;
        color[1] = 255;
        color[2] = 0;
    } else if (value < 180) {
        color[0] = 0;
        color[1] = 255;
        color[2] = posSlope * value - 2 * 255;
    } else if (value < 240) {
        color[0] = 0;
        color[1] = negSlope * value + 4 * 255;
        color[2] = 255;
    } else if (value < 300) {
        color[0] = posSlope * value - 4 * 255;
        color[1] = 0;
        color[2] = 255;
    } else {
        color[0] = 255;
        color[1] = 0;
        color[2] = negSlope * value + 6 * 255;
    }
    return cv::Scalar(color[0], color[1], color[2]);
}


void LoadSingleFile(const std::string &File,
                        std::vector<std::string> &vNames) {
    std::ifstream fFile(File);
    if (!fFile.is_open()) {
        cerr << "open file " << File << "failed." << endl;
        return;
    }
    std::string line;
    while (getline(fFile, line)) {
        std::string name = line.substr(0, line.find("\n"));
        vNames.push_back(name);
    }
}

Eigen::Matrix3d scaledK(const Eigen::Matrix3d &K, const int wnew, const int hnew, const int wold, const int hold) {
        const double ratio_u = (double)wnew / (double)wold;
        const double ratio_v = (double)hnew / (double)hold;
        const double fu = ratio_u * K(0, 0);
        const double fv = ratio_v * K(1, 1);
        const double cu = ratio_u * K(0, 2);
        const double cv = ratio_v * K(1, 2);
        Eigen::Matrix3d Knew;
        Knew << fu, 0, cu,
                0, fv, cv,
                0, 0, 1;
        return Knew;
}

Eigen::Vector3d getImageUVDfromLidar(const Eigen::Vector3d &pl, const Eigen::Matrix4d &Tcl, const Eigen::Matrix3d &K) {
    Eigen::Vector3d pc = Tcl.block<3, 3>(0, 0) * pl + Tcl.block<3, 1>(0, 3);
    const double depth = pc(2);
    pc = pc / pc(2);
    Eigen::Vector3d pixel = K * pc;
    return Eigen::Vector3d(pixel(0), pixel(1), depth);
}

Eigen::Vector3d getPointfromImageUVD(const Eigen::Vector3d &uvd, const Eigen::Matrix4d &Tlc, const Eigen::Matrix3d &Kinv) {
    const double depth = uvd(2);
    Eigen::Vector3d Kpc(uvd(0) * depth, uvd(1) * depth, depth);
    const Eigen::Vector3d pc = Kinv * Kpc;
    return Tlc.block<3, 3>(0, 0) * pc + Tlc.block<3, 1>(0, 3);
}

void depthMap2PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const cv::Mat &image,
                    const Eigen::Matrix4d &Tlc, const Eigen::Matrix<double, 3, 3> &Kinv) {
    const int w = image.cols;
    const int h = image.rows;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            if (image.at<ushort>(i, j) <=0) {
                continue;
            }
            const double depth = static_cast<double>(image.at<ushort>(i, j)) / 256.0f;
            assert(depth > 0 && depth <=256);
            const Eigen::Vector3d pixel(static_cast<double>(j), static_cast<double>(i), 1.0);
            const Eigen::Vector3d pc = Kinv * pixel * depth;
            const Eigen::Vector3d pl = Tlc.block<3, 3>(0, 0) * pc + Tlc.block<3, 1>(0, 3);
            pcl::PointXYZ point(pl(0), pl(1), pl(2));
            cloud->points.push_back(point);
        }
    }
}

void DIMapMasked2PointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const cv::Mat &image,
                    const cv::Mat &imageI, const cv::Mat &mask, const Eigen::Matrix4d &Tlc,
                    const Eigen::Matrix<double, 3, 3> &Kinv) {
    const int w = image.cols;
    const int h = image.rows;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            if (image.at<ushort>(i, j) <= 0 || mask.at<uchar>(i, j) <= 0) {
                continue;
            }
            const double depth = static_cast<double>(image.at<ushort>(i, j)) / 256.0f;
            const short intensity = static_cast<short>(static_cast<double>(imageI.at<ushort>(i, j)) / 256.0f);
            assert(depth > 0 && depth <=255);
            assert(intensity > 0 && intensity <=255);
            // //  u v 1, u--x轴
            const Eigen::Vector3d uvd(static_cast<double>(j), static_cast<double>(i), depth);
            const Eigen::Vector3d pl = getPointfromImageUVD(uvd, Tlc, Kinv);
            pcl::PointXYZI point;
            point.x = pl(0);    point.y = pl(1);    point.z = pl(2);
            point.intensity = intensity;
            cloud->points.push_back(point);
        }
    }
    cloud->width = 1;cloud->height = cloud->points.size();

}

void DIMap2PointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const cv::Mat &image,
                    const cv::Mat &imageI, const Eigen::Matrix4d &Tlc,
                    const Eigen::Matrix<double, 3, 3> &Kinv) {
    const int w = image.cols;
    const int h = image.rows;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            if (image.at<ushort>(i, j) <= 0) {
                continue;
            }
            const double depth = static_cast<double>(image.at<ushort>(i, j)) / 256.0f;
            const short intensity = static_cast<short>(static_cast<double>(imageI.at<ushort>(i, j)) / 256.0f);
            assert(depth > 0 && depth <=255);
            assert(intensity > 0 && intensity <=255);
            //  u v 1, u--x轴
            const Eigen::Vector3d pixel(static_cast<double>(j), static_cast<double>(i), 1.0);
            const Eigen::Vector3d pc = Kinv * pixel * depth;
            const Eigen::Vector3d pl = Tlc.block<3, 3>(0, 0) * pc + Tlc.block<3, 1>(0, 3);
            // cout << pl.transpose() << endl;
            pcl::PointXYZI point;
            point.x = pl(0);    point.y = pl(1);    point.z = pl(2);
            point.intensity = intensity;
            cloud->points.push_back(point);
        }
    }
    cloud->width = 1;cloud->height = cloud->points.size();

}

void DRGBMap2PointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const cv::Mat &imageDepth,
                    const cv::Mat &imageRGB, Eigen::Matrix4d &Tlc,
                    const Eigen::Matrix<double, 3, 3> &Kinv, const double coeff_depth) {
    int w = imageDepth.cols;
    int h = imageDepth.rows;
    if (w != imageRGB.cols || h != imageRGB.rows) {
        std::cout << "The size of RGB and depth are different!\n";
        return;
    }
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            if (imageDepth.at<ushort>(i, j) <= 0) {
                continue;
            }
            const double depth = static_cast<double>(imageDepth.at<ushort>(i, j)) / coeff_depth;
            if (depth <= 0) {
                continue;
            }
            // u v 1, u--x轴
            const cv::Vec3b& color = imageRGB.at<cv::Vec3b>(i,j);
            const Eigen::Vector3d pixel(static_cast<double>(j), static_cast<double>(i), 1.0);
            const Eigen::Vector3d pc = Kinv * pixel * depth;
            const Eigen::Vector3d pl = Tlc.block<3, 3>(0, 0) * pc + Tlc.block<3, 1>(0, 3);
            // cout << depth << " " << pl.transpose() << endl;
            pcl::PointXYZRGB point;
            point.x = pl(0);    point.y = pl(1);    point.z = pl(2);
            point.r = color[2]; point.g = color[1]; point.b = color[0];
            cloud->points.push_back(point);
        }
    }
    cloud->width = 1;cloud->height = cloud->points.size();

}

void estimateNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, const double knnRadius = 0.03) {
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(knnRadius);
    ne.compute (*cloud_normals);
}

cv::Mat createImageFromLidarRange(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                const cv::Mat &image_raw, const Eigen::Matrix4d &Tcl,
                                const Eigen::Matrix3d &K, const bool bshow, const string &showname) {
    Eigen::Vector4d Pl;
    Eigen::Vector3d Pc_norm;
    Eigen::Vector2d pixel;
    const int W = image_raw.cols;
    const int H = image_raw.rows;
    const ushort init = 0;
    cv::Mat image_gen = cv::Mat(H, W, CV_16UC1, init);
    cv::Mat imageshow = image_raw.clone();
    double x = 0. , y = 0.;
    int cnt = 0;    double sum = 0.0;
    int max = -1, min = 300;

    for (auto point : cloud->points) {
        Pl << point.x, point.y, point.z, 1.0;
        Pc_norm = K * (Tcl * Pl).block<3, 1>(0, 0);
        //  0.9 as MIT paper
        if (Pc_norm(2) <= 0.9) {
            continue;
        }
        double depth = Pc_norm(2);
        Pc_norm /= Pc_norm(2);
        pixel = Pc_norm.block<2, 1>(0, 0);
        x = pixel(0);   y = pixel(1);
        if (x < 0 || x >= W || y < 0 || y >= H) {
            continue;
        }
        //  in image
        cnt++;
        const double range = (point.x * point.x + point.y * point.y + point.z * point.z);
        sum += range;
        max = range > max ? range : max;
        min = range < min ? range : min;
        image_gen.at<ushort>(cv::Point(x, y)) = (ushort)(range * 256.0f);
        // image_gen.at<float>(cv::Point(x, y)) = point.intensity;     //  * 256.;  //  TODO
        if (bshow) {
            // double zc = (double)depth > 1 ? (double)depth : 0;
            const double zc = range;
            if (zc > 0) {
                cv::circle(imageshow, cv::Point(x, y), 1, cv::Scalar(zc, zc, zc));
                // cv::circle(imageshow, cv::Point(x, y), 1,
                //     fakeColor((zc > 130 ? 1.0f :  ((zc - 80.)/ 50.f) )));
                // printf("big value intensity:%f, x:%f, y:%f z:%f\n",
                //     point.intensity, point.x, point.y, point.z);
            }
        }
    }
    const double avg = sum / (double)cnt, std = 0.0;
    printf("intensity in image avg:%f, max:%d, min:%d \n", avg, max, min);
    if (bshow) {
        cv::imshow(showname, imageshow);
        cv::waitKey(3);
    }
    std::cout << "have num points in image: " << cnt << std::endl;

    return image_gen;
}

cv::Mat createImageFromLidarIntensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                    const cv::Mat &image_raw, const Eigen::Matrix4d &Tcl,
                                    const Eigen::Matrix3d &K, const bool bshow, const string &showname) {
    Eigen::Vector4d Pl;
    Eigen::Vector3d Pc_norm;
    Eigen::Vector2d pixel;
    const int W = image_raw.cols;
    const int H = image_raw.rows;
    const ushort init = 0;
    cv::Mat image_gen = cv::Mat(H, W, CV_16UC1, init);
    cv::Mat imageshow = image_raw.clone();
    double x = 0. , y = 0.;
    int cnt = 0;    double sum = 0.0;
    int max = -1, min = 300;

    for (auto point : cloud->points) {
        Pl << point.x, point.y, point.z, 1.0;
        Pc_norm = K * (Tcl * Pl).block<3, 1>(0, 0);
        //  0.9 as MIT paper
        if (Pc_norm(2) <= 0.9) {
            continue;
        }
        double depth = Pc_norm(2);
        Pc_norm /= Pc_norm(2);
        pixel = Pc_norm.block<2, 1>(0, 0);
        x = pixel(0);   y = pixel(1);
        if (x < 0 || x >= W || y < 0 || y >= H) {
            continue;
        }
        //  in image
        cnt++;
        sum+= point.intensity;
        max = point.intensity > max ? point.intensity : max;
        min = point.intensity < min ? point.intensity : min;
        image_gen.at<ushort>(cv::Point(x, y)) = (ushort)(point.intensity * 256.0f);
        // image_gen.at<float>(cv::Point(x, y)) = point.intensity;     //  * 256.;  //  TODO
        if (bshow) {
            const double zc = (double)point.intensity > 0 ? (double)point.intensity : 0;
            // double zc = point.intensity;//(double)point.intensity > 80 ? (double)point.intensity : 0;
            if (zc >= 0) {
                uchar r, g, b;
                GroundColorMix(r, g, b, zc, 0, 255);
                cv::circle(imageshow, cv::Point(x, y), 1, cv::Scalar(r, g, b));
                // cv::circle(imageshow, cv::Point(x, y), 1, cv::Scalar(zc, zc, zc));
                // cv::circle(imageshow, cv::Point(x, y), 1,
                //     fakeColor((zc > 255 ? 1.0f :  ((zc - 0.)/ 255.f) )));
                // printf("big value intensity:%f, x:%f, y:%f z:%f\n",
                //     point.intensity, point.x, point.y, point.z);
            }
        }
    }
    const double avg = sum / (double)cnt, std = 0.0;
    printf("intensity in image avg:%f, max:%d, min:%d \n", avg, max, min);
    if (bshow) {
        cv::imshow(showname, imageshow);
        cv::waitKey(3);
    }
    std::cout << "have num points in image: " << cnt << std::endl;

    return image_gen;
}

void calcuMat16U(const cv::Mat & image) {
    assert(image.channels() == 1 && image.type() == CV_16U);
    const int w = image.cols;
    const int h = image.rows;
    int num = 0;
    double mean = 0.0, std = 0.0, sum = 0.0, min = 1e6, max = -1;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            if (image.at<ushort>(i, j) <= 0) {
                continue;
            }
            const double intensity = static_cast<double>(image.at<ushort>(i, j)) / 256.0f;
            sum += intensity;
            num++;
            max = intensity > max ? intensity : max;
            min = intensity < min ? intensity : min;
        }
    }
    mean = sum / (double)num;
    sum = 0;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            if (image.at<ushort>(i, j) <= 0) {
                continue;
            }
            const double intensity = static_cast<double>(image.at<ushort>(i, j)) / 256.0f;
            sum += pow((intensity - mean), 2);
        }
    }
    std = sum / (double)num;
    printf("image U16 mean:%f, std:%f, max:%f, min:%f \n", mean, std, max, min);
}
// PCLutils::PCLutils()
// {

// }

}   //  namespace PCLutils
