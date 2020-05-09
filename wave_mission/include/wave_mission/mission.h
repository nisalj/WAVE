#ifndef MISSION_H
#define MISSION_H

#include <dynamic_reconfigure/server.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <wave_mission/WaveMissionConfig.h>
#include <wave_mission/SphericalUtil.hpp>
#include <wave_mission/PolyUtil.hpp>
#include <wave_mission/httplib.h>
#include <wave_mission/rapidjson/document.h>
#include <wave_mission/rapidjson/writer.h>
#include <wave_mission/rapidjson/stringbuffer.h>
using namespace std;

struct Segment
{
  LatLng start;
  LatLng end;
  double bearing;
  double distance; 
  Segment(LatLng start, LatLng end, double bearing, double distance) : start(start), end(end), bearing(bearing), distance(distance) {};
};


class MissionObject
{
public:
  MissionObject();

private:
  vector<Segment> segments;
  ros::NodeHandle node;

  //mission params
  string mission_name;
  double path_tolerance;
  double waypoint_tolerance;
  double cruising_speed;
  double max_speed;
  double lookahead_distance;
  double prediction_time;
  string location_topic; 
  string velocity_topic; 
  string heading_topic; 
  string mission_enable_topic;
  string mission_stats_topic;


  LatLng robotPos; 
  double robotHeading; 
  double robotX; 
  double robotW; 
  double targetBearing; 
  LatLng targetPos;
  LatLng targetWaypoint;  
  bool setMissionPlan; 
  bool missionEnabled; 
  int currSeg; 
  ros::Subscriber location_sub; 
  ros::Subscriber velocity_sub; 
  ros::Subscriber heading_sub;
  ros::Publisher heading_pub;
  ros::Publisher mission_stats_pub;
  bool firstLocation;
  bool firstVelocity;
  bool firstHeading;
  void getPlan(const string &planName);
  void reconfigureCallback(wave_mission::WaveMissionConfig &config, uint32_t level);
  void locationCallback(const sensor_msgs::NavSatFix& fix); 
  void velocityCallback(const geometry_msgs::Twist &vel);
  void headingCallback(const std_msgs::Float64 &yaw); 
  void enableCallback(const std_msgs::Bool& enable); 
  void startSubs();
  void stopSubs(); 
  void doCalcs(); 
  LatLng predictFutureLocation(); 
  LatLng calcOffsetPoint(LatLng perpPoint, Segment line) const;


  string bearing_output_topic;

};

#endif