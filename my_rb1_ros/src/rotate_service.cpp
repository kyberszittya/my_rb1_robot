#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <ros/ros.h>

#include <my_rb1_ros/Rotate.h>
#include <nav_msgs/Odometry.h>


struct RobotGoal
{
    double target_angle;
    double current_angle;
    double start_angle;

    RobotGoal(): target_angle(0), current_angle(0) {}
};

constexpr double D_EPS = 0.01;
constexpr double TARGET_ANG_SPEED = 0.2;
constexpr double RATE_CMD_PUB = 10.0;
constexpr unsigned int MAX_ITER = RATE_CMD_PUB * 10;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class MyRb1RosController 
{
private:
  ros::NodeHandle& nh_;
  // Service server to rotate
  ros::ServiceServer rotate_service;
  // Goal state
  RobotGoal goal;
  // Odometry subscribe
  ros::Subscriber sub_odometry;
  // Publish cmd
  ros::Publisher pub_cmd;
public:

  MyRb1RosController(ros::NodeHandle &nh) : nh_(nh) 
  {
    rotate_service = nh_.advertiseService("/rotate_robot", &MyRb1RosController::rotate_robot_service_callback, this);
    sub_odometry = nh_.subscribe("/odom", 1000, &MyRb1RosController::cb_sub_odometry, this);
    pub_cmd = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ROS_INFO("Initialized My RB1 controller");
  }

  inline double diff_deg()
  {
    double d_abs = std::abs(goal.target_angle  - goal.current_angle);
    return d_abs;
  }

  bool goal_reached()
  {    
    return diff_deg() < D_EPS;
  }

  void stop_robot(geometry_msgs::Twist& cmd)
  {
    // Stop robot rotation
    cmd.angular.z = 0.0;
    pub_cmd.publish(cmd);
  }

  bool rotate_robot_service_callback(
    my_rb1_ros::RotateRequest &req, 
    my_rb1_ros::RotateResponse &resp)
  {
    ROS_INFO("Received request: %d", req.degrees);
    // Update goal (convert to radians)
    goal.start_angle = goal.current_angle;    
    goal.target_angle = M_PI * (double)req.degrees/180.0 + goal.start_angle;
    // Update start angle
    geometry_msgs::Twist cmd;
    ros::Rate r(RATE_CMD_PUB);
    double d_ang = goal.target_angle - goal.start_angle;
    double direction = sgn<double>(d_ang);
    unsigned int cmd_cnt = std::abs(d_ang * RATE_CMD_PUB)/TARGET_ANG_SPEED + ((int)RATE_CMD_PUB / 2);
    
    for (unsigned int i=0; i <= cmd_cnt  ; i++)
    {   
        cmd.angular.z = direction*TARGET_ANG_SPEED;
        pub_cmd.publish(cmd);
        ros::spinOnce();
        r.sleep();
        ROS_INFO("%d/%d Start angle: %3.2f, Target angle: %3.2f, Reamining angle: %3.2f",
            i, cmd_cnt, goal.start_angle, goal.target_angle, diff_deg());
        if (goal_reached())
        {
            stop_robot(cmd);
            break;
        }        
    }
    stop_robot(cmd);
    // Check if our robot has reached the goal
    if (!goal_reached())
    {
        resp.result = "UNABLE TO REACH GOAL";
    }
    else
    {
        resp.result = "SUCCESS";
    }
    
    return true;
  }

  void cb_sub_odometry(const nav_msgs::OdometryConstPtr& msg)
  {    
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double psi = std::atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
    goal.current_angle = psi;    
  }
  
};

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "my_rb1_ros_controller");
  ros::NodeHandle nh;
  ROS_INFO("Initializing controller");
  MyRb1RosController controller(nh);
  ROS_INFO("Waiting for requests, spinning...");
  ros::spin();
  ROS_INFO("Exiting application, controller stopped");
  return 0;
}