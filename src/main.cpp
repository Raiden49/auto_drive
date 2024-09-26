/*
 * @Author: Raiden49 
 * @Date: 2024-09-25 11:31:35 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-25 11:33:31
 */
#include "plan.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "auto_drive");
  ros::NodeHandle nh;
  auto plan_ptr = std::make_shared<auto_drive::Plan>(nh);
  
  ROS_INFO("Start auto drive");
  plan_ptr->Loop();
  
  return 0;
}