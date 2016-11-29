/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c)  2013, Örebro University, Sweden
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.

*Authors: Ali Abdul Khaliq on 29/10/2014
*********************************************************************/

#include "lucia_services.h"

//======================================================================================//
//					Main						//
//======================================================================================//
int main(int argc, char** argv)
{

  ros::init(argc, argv, "lucia_services");
  ros::NodeHandle nh_;

  
  nh_.getParam("tf_prefix", param);

  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;

  ros::ServiceServer QRserv       = nh_.advertiseService("getQR", sendQR);
  ros::ServiceServer Goalserv     = nh_.advertiseService("sendGoal", sendGoal);
    ros::ServiceServer Statusserv = nh_.advertiseService("getStatus", sendStatus);
//  ros::ServiceServer Rotateserv   = nh_.advertiseService("rotate", sendRot);
  ros::ServiceServer Locserv      = nh_.advertiseService("getLocation", getLocation);
  ros::Subscriber    amcl_sub     = nh_.subscribe("amcl_pose", 10, amclCallback);
  ros::Subscriber    sub          = nh_.subscribe("move_base/status", 10, goalStatus);
//  ros::Publisher     rotate_pub   = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 100);

  ros::Rate loop_rate(FREQUENCY);


   while (ros::ok())
   {
   
   //if(rotationAfter) //this veriable is handled by sendGoal and rotate services
    //rotate(nh_,rotate_pub);
    
    ros::spinOnce();
    loop_rate.sleep();
   }

 return 0;
}




//===================================================================================
bool sendQR(services::getQR::Request &req, services::getQR::Response &res)
//===================================================================================
 {
  res.qrcode= qrCode;
  return true;
 }
 
 
//========================================================================
bool sendStatus(services::getStatus::Request &req, services::getStatus::Response &res)
//========================================================================
 {
  res.status = statusOfMove;
  return true;
 }

//========================================================================
//bool sendRot(services::rotate::Request &req, services::rotate::Response &res)
//========================================================================
// {
//  rotationAfter = req.rotate;
//    last_yaw=0;
//    curr_yaw=0;
//    init=true;

//  return true;
// }

//===========================================================================================
bool sendGoal(services::sendGoal::Request &req, services::sendGoal::Response &res)
//===========================================================================================
 {
  MoveBaseClient ac("move_base", true);

  statusOfMove = 1;
  rotationAfter = req.rotationAfter;

  while (!ac.waitForServer(ros::Duration(5.0)))
    {
    ROS_INFO("Waiting for the move_base action server to come up");
    }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id  = "map"; 
  goal.target_pose.header.stamp =  ros::Time::now();
  geometry_msgs::Point goalPoint;

  goalPoint.x = req.x;
  goalPoint.y = req.y;
  goal.target_pose.pose.position = goalPoint;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(req.theta);//orientationPoint;

  ac.sendGoal(goal);
    ros::NodeHandle nh_;
      ros::Rate loop_rate(FREQUENCY);
    
  while(!status.status_list.empty() &&
        (int)status.status_list[0].status!= ACTIVE ) //wait for status change
       {
       nh_.subscribe("move_base/status", 100, goalStatus);
       ros::spinOnce();
       loop_rate.sleep();
       }

  res.result = 1;
  return true;
 }

//====================================================================================================
bool getLocation(services::getLocation::Request &req, services::getLocation::Response &res)
//====================================================================================================
 {
  btScalar roll, pitch, yaw;
  btQuaternion q(amcl_pos.pose.pose.orientation.x,
                 amcl_pos.pose.pose.orientation.y,
                 amcl_pos.pose.pose.orientation.z,
                 amcl_pos.pose.pose.orientation.w);

  btMatrix3x3(q).getEulerYPR(yaw, pitch,roll );

  res.id = ROBOT_ID;
  res.x= amcl_pos.pose.pose.position.x;
  res.y= amcl_pos.pose.pose.position.y;
  res.z= amcl_pos.pose.pose.position.z;
  res.theta = yaw;

  return true;
 }

//====================================================================
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
//====================================================================
 {
  amcl_pos.pose.pose.position.x = msg.pose.pose.position.x;
  amcl_pos.pose.pose.position.y = msg.pose.pose.position.y;
  amcl_pos.pose.pose.position.z = msg.pose.pose.position.z;

  amcl_pos.pose.pose.orientation.x= msg.pose.pose.orientation.x;
  amcl_pos.pose.pose.orientation.y= msg.pose.pose.orientation.y;
  amcl_pos.pose.pose.orientation.z= msg.pose.pose.orientation.z;
  amcl_pos.pose.pose.orientation.w= msg.pose.pose.orientation.w;
 }

//==================================================
void goalStatus(const actionlib_msgs::GoalStatusArray& msg)
//==================================================
 {
  status.status_list= msg.status_list; 
 }



//======================================================================================//
//					EOF						//
//======================================================================================//
