#ifndef MISSION_H
#define MISSION_H

#include <dynamic_reconfigure/server.h>
#include <wave_mission/WaveMissionConfig.h>
#include <wave_mission/SphericalUtil.hpp> 
#include <vector> 
#include <wave_mission/httplib.h>
#include <wave_mission/rapidjson/document.h>
#include <wave_mission/rapidjson/writer.h>
#include <wave_mission/rapidjson/stringbuffer.h>
using namespace std; 

struct Segment {
  LatLng start; 
  LatLng end; 
  double bearing; 
  Segment(LatLng start, LatLng end, double bearing): start(start), end(end), bearing(bearing) {}
};

class MissionObject {
public: 
MissionObject(); 

private: 

vector<Segment> segments; 
//mission params 
string mission_name; 
bool mission_enabled; 
double path_tolerance; 
double waypoint_tolerance; 
double cruising_speed; 
double max_speed; 

void getPlan(string planName); 
void reconfigureCallback(wave_mission::WaveMissionConfig& config, uint32_t level);
};


#endif