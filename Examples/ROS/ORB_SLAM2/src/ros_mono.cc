/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<Eigen/Dense>
#include<Eigen/StdVector>
#include <Eigen/Core>
#include "Converter.h"
#include "opencv2/core/eigen.hpp"

#include<opencv2/core/core.hpp>
////////////////
#include "std_msgs/Float64MultiArray.h"
///////////////
#include"../../../include/System.h"
using namespace std;
//////
Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>  metrix2;
//Eigen::MatrixXd metrix2;
//////
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void GrabMetrix(const std_msgs::Float64MultiArray msg);
    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
////////////////////
    ros::Subscriber sub2 = nodeHandler.subscribe("robot_pose", 1, &ImageGrabber::GrabMetrix,&igb);
//////////////////////////
    ros::Subscriber sub = nodeHandler.subscribe("/kitti/camera_color_left/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    //MXW9.7
    

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
/////
    //mpSLAM->TrackMonocularRos(cv_ptr->image,cv_ptr->header.stamp.toSec());
/////
}
//////
void ImageGrabber::GrabMetrix(const std_msgs::Float64MultiArray msg)
{
    //Eigen::VectorXd metrix=(Eigen::VectorXd) msg.data;
    //Eigen::RowVectorXd metrix;
    //Eigen::RowVectorXd metrix3;
    int a = (int) msg.data.at(0);
    int b = (int) msg.data.at(1);
    int inij;
    metrix2.resize(a,b);
    std::cout<< " a = " << msg.data[0] << std::endl;
    std::cout<< " length = " << std::end(msg.data) - std::begin(msg.data) << std::endl;

    //int num= msg.data.at(0)*msg.data.at(1);
    for(int i=0; i<a;i++)
    {
        if(i==0)
        {
           inij=2;
        }
        else
        {
            inij=0;
        }
        for( int j=inij; j<b;j++)
        {

             metrix2(i,j)=msg.data.at(i);

        }
       
    }
    std::cout<< " b = " << msg.data.at(1) << std::endl;
    //Eigen::MatrixXd metrix2;
    //metrix3=metrix.tail(num-2);
    //metrix2=Eigen::Map<Eigen::MatrixXd>(metrix3.data());
   
    cv::Mat img;
    cv::eigen2cv(metrix2, img);
    mpSLAM->TrackMonocularROS(img);
    std::cout << "ROS part is ok!" <<std::endl;
}
//////