#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <sstream>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "freeway_msgs/DistanceTimeCalculator.h"
#include "freeway_msgs/FreewayStatus.h"



// void get_goal_cb(const geometry_msgs::PoseStamped &goal_msg) {
//   geometry_msgs::PoseStamped goal;
//   goal.header.stamp = ros::Time::now();
//   goal.header.frame_id = "/map";
//   goal.pose.position.x = goal_msg.pose.position.x;
//   goal.pose.position.y = goal_msg.pose.position.y;
//   goal.pose.orientation.z = goal_msg.pose.orientation.z;
//   goal.pose.orientation.w = goal_msg.pose.orientation.w;
// }

class Distance_TimeCalculator
{
  public:
  
    float total_vel = 0.0;
    int status_flag=0;
    int first_time=1; 
    uint8_t status_info_=0;
    geometry_msgs::Pose current_robot_pose;
    geometry_msgs::Pose prev_current_robot_pose;
    geometry_msgs::PoseStamped final_goal;
    double move_base_GlobalPlanner_plan_Time=0.0;
    float first_global_path_distance=0.0;
    float global_path_percentage=0.0;
    float remaining_time=0.0;
    double msg_global_path_distance=0.0;
    double traveled_distance=0.0;
    float average_total_vel=0.0;
    double remaining_percentage=0.0;
    bool global_path_flag = false;

    Distance_TimeCalculator(ros::NodeHandle *n)
    {
      resume_pub = n->advertise<geometry_msgs::PoseStamped>("freeway/goal", 100);
      feedback_sub = n->subscribe("move_base/feedback", 1000, &Distance_TimeCalculator::get_feedback_cb, this);
      status_sub = n->subscribe("move_base/status", 10, &Distance_TimeCalculator::get_status_cb, this);
      velocity_sub = n->subscribe("cmd_vel", 10, &Distance_TimeCalculator::get_velocity_cb, this);
      getpath_sub = n->subscribe("move_base/GlobalPlanner/plan", 10, &Distance_TimeCalculator::get_globalpath_cb, this);
      resume_sub = n->subscribe("freeway/resume",10, &Distance_TimeCalculator::resume_cb, this);
      goal_sub = n->subscribe("freeway/goal", 10, &Distance_TimeCalculator::goal_cb, this);
    }

void resume_cb(const std_msgs::Empty &resume_msg) {
  ros::Duration tenth(0, 100000000); // 0.1 seconds
  for (int i=0; i<1; i++) {
  resume_pub.publish(final_goal);
  tenth.sleep();
  }
}

void goal_cb(const geometry_msgs::PoseStamped &goal_msg) {
  final_goal = goal_msg;
}

void get_status_cb(const actionlib_msgs::GoalStatusArray &status_msg) {
  // status_flag = status_msg.status_list[0].status;
  int is=0;
  if (!status_msg.status_list.empty()) {
    for(size_t i=0; i<status_msg.status_list.size()-1; i++)
    {
      is++;
    }
      status_info_ = status_msg.status_list[is].status;
  }
  else ROS_INFO("status_list array is empty");
  //ROS_INFO("status flag: %u", status_flag);
}

void get_velocity_cb(const geometry_msgs::Twist &cur_vel) {
  static uint counter_N1 = 1;
  float cur_x_vel = cur_vel.linear.x;
  float cur_y_vel = cur_vel.linear.y;
  float cur_az_vel = cur_vel.angular.z;

  total_vel = sqrt((cur_x_vel*cur_x_vel)+(cur_y_vel*cur_y_vel)+(cur_az_vel*cur_az_vel));
  // ROS_INFO("current velocity : %f", total_vel);
  
  if (status_flag==1) {
  average_total_vel = (float)(average_total_vel*(counter_N1-1)+total_vel)/counter_N1;
  //ROS_INFO("Average velocity : %f", average_total_vel);
  counter_N1++;
  }
  else { counter_N1 = 1; average_total_vel = 0.0; }
  //ROS_INFO("counter number : %u", counter_N1);
}

void get_globalpath_cb(const nav_msgs::Path &globalpath_msgs) {
 move_base_GlobalPlanner_plan_Time = ros::Time::now().toSec();
 static uint counter_N2=1;
 double current_a_x = 0.0;
 double current_b_x = 0.0;
 double current_a_y = 0.0;
 double current_b_y = 0.0;
 double global_path_distance=0.0;
 float pre_global_path_distance=0.0;

  if (status_flag==1){
    if ( !globalpath_msgs.poses.empty()) {
      for (size_t i=0; i < globalpath_msgs.poses.size()-1; i++)
      {
       current_a_x = globalpath_msgs.poses[i].pose.position.x;
       current_b_x = globalpath_msgs.poses[i+1].pose.position.x;
       current_a_y = globalpath_msgs.poses[i].pose.position.y;
       current_b_y = globalpath_msgs.poses[i+1].pose.position.y;
       global_path_distance += hypot((current_b_x - current_a_x), (current_b_y - current_a_y));
       msg_global_path_distance = global_path_distance;
       // ROS_INFO("hypot value : %lf", hypot((current_b_x - current_a_x), (current_b_y - current_a_y)));
       //global_path_distance += sqrt(pow(current_b_x - current_a_x,2) + pow(current_b_y - current_a_y,2));
       if(counter_N2==1) first_global_path_distance = global_path_distance;
       global_path_percentage = ((first_global_path_distance-global_path_distance)/first_global_path_distance)*100;
      }
    }
    remaining_time = global_path_distance/average_total_vel;
    //ROS_INFO("global_path_percentage : %f ", global_path_percentage);
    pre_global_path_distance = global_path_distance;
    // if (pre_global_path_distance - global_path_distance > 0) counter_N2=1;
    counter_N2++;

    global_path_flag = true;
  } 
  else { counter_N2=1; remaining_time = 0.0; global_path_distance=0.0; first_global_path_distance=0.0;}

  //ROS_INFO("remaining_time : %f",remaining_time);
  //ROS_INFO("global_path_distance : %f",msg_global_path_distance);
}

void get_feedback_cb(const move_base_msgs::MoveBaseActionFeedback &feedback_msgs){
  if (global_path_flag) {
  if(first_time==1){
    prev_current_robot_pose.position.x = feedback_msgs.feedback.base_position.pose.position.x;
    prev_current_robot_pose.position.y = feedback_msgs.feedback.base_position.pose.position.y;
    }
  else if (first_time!=1){
    current_robot_pose.position.x = feedback_msgs.feedback.base_position.pose.position.x;
    current_robot_pose.position.y = feedback_msgs.feedback.base_position.pose.position.y;
  }
  traveled_distance += hypot((current_robot_pose.position.x - prev_current_robot_pose.position.x), (prev_current_robot_pose.position.y - current_robot_pose.position.y)); 
  prev_current_robot_pose.position.x = current_robot_pose.position.x;
  prev_current_robot_pose.position.y = current_robot_pose.position.y;
  if(status_flag==1)first_time++;
  else if(status_flag !=1)first_time = 1;

  //ROS_INFO("robot_traveled_distance : %lf",traveled_distance);

  remaining_percentage = traveled_distance/(msg_global_path_distance+traveled_distance)*100;
  }
}
  private:
    ros::Publisher resume_pub;
    ros::Subscriber feedback_sub;
    ros::Subscriber status_sub;
    ros::Subscriber velocity_sub; 
    ros::Subscriber getpath_sub;
    ros::Subscriber resume_sub;
    ros::Subscriber goal_sub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_calculator_oop_node");
  ros::NodeHandle n;
  ros::Publisher distancetimecalculator_pub;
  freeway_msgs::DistanceTimeCalculator distancetimecalculator_msg;

  Distance_TimeCalculator Dt = Distance_TimeCalculator(&n);
  distancetimecalculator_pub = n.advertise<freeway_msgs::DistanceTimeCalculator>("freeway/distancetimecalculator", 100);

  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    // ros::master::V_TopicInfo master_topics;
    // ros::master::getTopics(master_topics);

    // for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
    //   const ros::master::TopicInfo& info = *it;
    //   std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << std::endl;
    // }
    if ((ros::Time::now().toSec() - Dt.move_base_GlobalPlanner_plan_Time) < 2.0) { Dt.status_flag=1; }
    else { Dt.status_flag=0; Dt.remaining_time = 0.0; Dt.msg_global_path_distance=0.0; Dt.first_global_path_distance=0.0; Dt.traveled_distance=0.0; Dt.first_time=0; Dt.remaining_percentage=0.0; Dt.global_path_flag=false;}
    

    distancetimecalculator_msg.distance_remaining = Dt.msg_global_path_distance;
    distancetimecalculator_msg.arrival_time = Dt.remaining_time;
    distancetimecalculator_msg.distance_robot_traveled = Dt.traveled_distance;
    distancetimecalculator_msg.remaining_distance_percentage = Dt.remaining_percentage;
    distancetimecalculator_msg.status_info = Dt.status_info_;
    
    distancetimecalculator_pub.publish(distancetimecalculator_msg);
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
