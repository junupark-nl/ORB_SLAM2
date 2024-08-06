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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/Converter.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle nodeHandler):nodeHandler_(nodeHandler), mpSLAM(pSLAM) {
        mPubPose = nodeHandler_.advertise<geometry_msgs::PoseStamped>("/orb_slam2/pose2", 1);
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void Publish(const cv::Mat& Tcw);

    ros::NodeHandle nodeHandler_;
    ORB_SLAM2::System* mpSLAM;
    ros::Publisher mPubPose;
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

    ros::NodeHandle nodeHandler;
    ImageGrabber igb(&SLAM, nodeHandler);

    ros::Subscriber sub = nodeHandler.subscribe("/fisheye_adaptor/image_undistorted", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

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

    Publish(mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec()));
}

void ImageGrabber::Publish(const cv::Mat& Tcw)
{
    // Create a ROS message
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.header.frame_id = "map";

    // Convert the matrix to quaternion
    // cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    // cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    // std::vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

    // poseStamped.pose.position.x = twc.at<float>(0);
    // poseStamped.pose.position.y = twc.at<float>(1);
    // poseStamped.pose.position.z = twc.at<float>(2);

    // poseStamped.pose.orientation.x = q.at(0);
    // poseStamped.pose.orientation.y = q.at(1);
    // poseStamped.pose.orientation.z = q.at(2);
    // poseStamped.pose.orientation.w = q.at(3);



    poseStamped.pose.position.x = 1;
    poseStamped.pose.position.y = 2;
    poseStamped.pose.position.z = 3;

    poseStamped.pose.orientation.x = 0;
    poseStamped.pose.orientation.y = 0;
    poseStamped.pose.orientation.z = 0;
    poseStamped.pose.orientation.w = 1;

    // Publish the data
    mPubPose.publish(poseStamped);
}


