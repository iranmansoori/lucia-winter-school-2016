#include <ros/ros.h>
#include <tf/message_filter.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <bullet/LinearMath/btMatrix3x3.h>
#include <services/getQR.h>
#include <services/getStatus.h>
#include <services/sendGoal.h>
#include <services/getLocation.h>
#include <services/getLocation.h>
#include <services/getPanel.h>
#include <services/rotate.h>


#define FREQUENCY	100
#define FAIL		-1
#define ROBOT_ID	 1
#define SUCCEEDED        3
#define ACTIVE           1

  using namespace std;
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//create a reader
 
  geometry_msgs::PoseWithCovarianceStamped amcl_pos;
  actionlib_msgs::GoalStatusArray status;

  bool   init = true;
  double curr_yaw =0;
  double last_yaw =0;
  int    rotationAfter=0;  
  int    statusOfMove = -1;  //statusOfMove < 0 <=> not moving
  int    qrCode = FAIL;
  std::string param;

  bool sendQR(services::getQR::Request &req, services::getQR::Response &res);
  bool sendGoal(services::sendGoal::Request &req, services::sendGoal::Response &res);
  bool getLocation(services::getLocation::Request &req, services::getLocation::Response &res);
  void amclCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void goalStatus(const actionlib_msgs::GoalStatusArray& msg);
 // void rotate(ros::NodeHandle nh_, ros::Publisher rotate_pub);
  
    bool sendStatus(services::getStatus::Request &req, services::getStatus::Response &res);
 // void rotation(ros::NodeHandle nh_, ros::Publisher rotate_pub);
//  bool sendRot(services::rotate::Request &req, services::rotate::Response &res);
  
  

//======================================================================================//
//					EOF						//
//======================================================================================//

