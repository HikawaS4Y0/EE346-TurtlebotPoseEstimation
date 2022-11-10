/** MIT License
Copyright (c) 2017 Sudarshan Raghunathan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 *@copyright Copyright 2017 Sudarshan Raghunathan
 *@file   detect.cpp
 *@author Sudarshan Raghunathan
 *@brief  Ros Nod to subscribe to turtlebot images and perform image processing to detect line
 */
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "ros/console.h"
#include "linedetect.hpp"
#include "line_follower_turtlebot/pos.h"
#include "ros/callback_queue.h"

/**
 *@brief Main function that reads image from the turtlebot and provides direction to move using image processing
 *@param argc is the number of arguments of the main function
 *@param argv is the array of arugments
 *@return 0roslaunch aruco_marker_finder.launch markerId:=701 markerSize:=0.05
*/

ros::Time timestp;
line_follower_turtlebot::pos msg1;
int has_find=0;
int danger=0;

void arucoCallback(const geometry_msgs::PoseStampedPtr& msg){
  timestp=msg->header.stamp;
  auto x1=  msg->pose.position.x;
  auto y1=msg->pose.position.y;
  auto z1=msg->pose.position.z;
  cv::Mat coor1=(cv::Mat_<float>(4,1)<<x1,y1,z1,1);
  cv::Mat q=(cv::Mat_<float>(4,1)<<msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
  float w = q.at<float>(0);
  float x = q.at<float>(1);
  float y = q.at<float>(2);
  float z = q.at<float>(3);
  float xx = x*x;
  float yy = y*y;
  float zz = z*z;
  float xy = x*y;
  float wz = w*z;
  float wy = w*y;
  float xz = x*z;
  float yz = y*z;
  float wx = w*x;
  float ret[4][4];
  ret[0][0] = 1.0f-2*(yy+zz);
  ret[0][1] = 2*(xy-wz);
  ret[0][2] = 2*(wy+xz);
  ret[0][3] = 0.0f;
  ret[1][0] = 2*(xy+wz);
  ret[1][1] = 1.0f-2*(xx+zz);
  ret[1][2] = 2*(yz-wx);
  ret[1][3] = 0.0f;
  ret[2][0] = 2*(xz-wy);
  ret[2][1] = 2*(yz+wx);
  ret[2][2] = 1.0f-2*(xx+yy);
  ret[2][3] = 0.0f;
  ret[3][0] = 0.0f;
  ret[3][1] = 0.0f;
  ret[3][2] = 0.0f;
  ret[3][3] = 1.0f;
  cv::Mat newcoor=cv::Mat(4,4,CV_32FC1,ret).clone()*coor1;
  auto newx=newcoor.at<float>(0);
  auto newy=newcoor.at<float>(1);
  auto newz=newcoor.at<float>(2);
//   ROS_INFO("%f,%f,%f",newx,newy,newz);
  if(newx<-0.036)msg1.direction=11;
  else if(newx>0.01)msg1.direction=12;
  else if(msg->pose.orientation.z>0.07)msg1.direction=13;
  else if(msg->pose.orientation.z<-0.07)msg1.direction=14;
  else if(newz<-0.3)msg1.direction=16;
  else msg1.direction =10;



  
}

  

int main(int argc, char **argv) {
    // Initializing node and object
    ros::init(argc, argv, "detection");
    ros::NodeHandle n;
    LineDetect det;
    // Creating Publisher and subscriber
    ros::Subscriber sub = n.subscribe("/raspicam_node/image/compressed", 1, &LineDetect::imageCallback, &det);
    ros::Subscriber sub_aruco= n.subscribe("/aruco_single/pose", 1,&arucoCallback);

    ros::Publisher dirPub = n.advertise<
    line_follower_turtlebot::pos>("direction", 1);
    

    while (ros::ok()) {
        // ROS_INFO("%f",ros::Time::now().nsec/1e9-timestp.nsec/1e9+ros::Time::now().sec-timestp.sec);
        if (ros::Time::now().nsec/1e9-timestp.nsec/1e9+ros::Time::now().sec-timestp.sec<0.2 or has_find)
        { 
            has_find=1;
            // msg1.direction=(10);
            dirPub.publish(msg1);
        }
        else{
            if (!det.img.empty()) {
            // Perform image processing
            det.img_filt = det.Gauss(det.img);
            msg1.direction = det.colorthresh(det.img_filt);
            // Publish direction message
            dirPub.publish(msg1);
            }
        }
        ros::spinOnce();
        
    }
    // Closing image viewer
    cv::destroyWindow("Turtlebot View");
}


