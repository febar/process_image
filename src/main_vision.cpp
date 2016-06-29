 // main_vision

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


#include "Cvision.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Cvision");
	ros::NodeHandle nh_main_;

	Cvision p;
	ros::Rate rate(30);

	while(ros::ok)
	{
		if(p.flag_cloud_ && p.flag_cvImage_ )
		{
			// std::cout << "Fai il mio processo" << std::endl;
			switch (p.processImage())
  			{
    			case 1 : 
     			std::cout << "go ahead" << std::endl;
     			break;
    			case 2 :
    			std::cout << "turn right" << std::endl;
    			break;
    			case 3 :
    			std::cout << "turn left" << std::endl;
    			break;
    			case 4 :
    			std::cout << "STOP" << std::endl;
    			break;
  			}
			p.flag_cloud_ = p.flag_cvImage_ = false;
		}

		// p.comparison_pcl();
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}