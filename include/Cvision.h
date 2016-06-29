// libraries for the pcl

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

// ros libraries

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// Open CV libraries

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

// common libraries

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <time.h>


class Cvision
{
public:
	Cvision();   // constructor;

	~Cvision();   // deconstructor

	// flag for sync
	bool flag_cloud_;
    bool flag_cvImage_;
    // parameter for process image
    cv::Mat src_gray; 
    cv::Mat result, mask_3, final_mask;	
    int maxCorners;
    /// Parameters for Shi-Tomasi algorithm
    double qualityLevel;
  	double minDistance;    // distanza tra i vari corner (di default era a 10)
  	int blockSize;
  	bool useHarrisDetector;
  	double k;

  	// function for image processing
	int processImage();

private:
	// create new pcl e Mat CV
	pcl::PointCloud<pcl::PointXYZ> my_cloud_;
 	cv::Mat my_cvImage_;
	// node handle 
	ros::NodeHandle nh_;


	//Message Subscriber to pcl and openCv
    ros::Subscriber sub_pcl_;
    
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    
    

    //Message callback, gets executed when a new message is available on topic
    void newCloud(const sensor_msgs::PointCloud2::ConstPtr& message);
    //Message callback to convert image ros into opecv image
    void convertCvmat(const sensor_msgs::ImageConstPtr& msg);
    std::vector<double> comparison_pcl(std::vector<cv::Point2f>);
	std::vector<cv::Point2f> central_mask( int, void* );
	std::vector<cv::Point2f> mask_right( int, void* );
	std::vector<cv::Point2f> mask_left( int, void* );
	bool central_check();
	bool right_check();
	bool left_check();

};