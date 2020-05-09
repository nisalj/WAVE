#include <ros/ros.h>
#include <wave_mission/mission.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "wave_mission");
  MissionObject mission;
  ROS_INFO("Spinning node");
  return 0;
}
