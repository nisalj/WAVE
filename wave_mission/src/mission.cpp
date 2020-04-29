using namespace std; 
#include <wave_mission/mission.h>



MissionObject::MissionObject() {
  ros::NodeHandle node;
  ros::NodeHandle node_priv("~");


  // Get params if specified in launch file or as params on command-line, set
  // defaults
  node_priv.param<string>("mission_name", mission_name, "NULL");
  node_priv.param<bool>("mission_enabled", mission_enabled, false);
  node_priv.param<double>("path_tolerance", path_tolerance, 1.0);
  node_priv.param<double>("waypoint_tolerance", waypoint_tolerance, 1.0);
  node_priv.param<double>("cruising_speed", cruising_speed, 0.5);
  node_priv.param<double>("max_speed", max_speed, 1.0); 

  dynamic_reconfigure::Server<wave_mission::WaveMissionConfig> server;
  dynamic_reconfigure::Server<wave_mission::WaveMissionConfig>::CallbackType f;

  f = boost::bind(&MissionObject::reconfigureCallback, this, _1, _2);
  server.setCallback(f);
  getPlan("pottery"); 
  while (ros::ok())
  {
   // doCalcs();
    ros::spinOnce();

    // Add a small sleep to avoid 100% CPU usage
    ros::Duration(0.001).sleep();
  }

}


void MissionObject::getPlan(string planName) {
  ROS_INFO("Getting mission plan from server");

  const string path = "/plan?planName="+planName; 
	httplib::SSLClient cli("localhost", 5000);
	auto res = cli.Get(path.c_str());
  const char * recievedPlan;  
  	 if (res && res->status == 200) {
      ROS_INFO("Mission plan recieved");
      recievedPlan = res->body.c_str(); 
      rapidjson::Document d; 
      d.Parse(recievedPlan);
      for (int i = 0; i < d.Size(); i++) {
      double startLat = atof(d[i]["startLat"].GetString());
      double startLong = atof(d[i]["startLong"].GetString());
      LatLng start =  LatLng(startLat,startLong); 
      double endLat = atof(d[i]["endLat"].GetString());
      double endLong = atof(d[i]["endLong"].GetString());
      LatLng end = LatLng(endLat, endLong); 
      double bearing = atof(d[i]["bearing"].GetString());
      Segment s = Segment(start, end, bearing); 
      MissionObject::segments.push_back(s);
    //  ROS_INFO("%.10f %.10f %.10f %.10f %.10f", startLat, startLong, endLat, endLong, bearing); 
    } 
    } else {
      ROS_ERROR("Unable to get mission plan");
    }
      
   



   		

}







void MissionObject::reconfigureCallback(wave_mission::WaveMissionConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %s %s %f %f %f %f", 
            config.mission_name.c_str(), config.mission_enabled?"True":"False", 
            config.path_tolerance, 
            config.waypoint_tolerance, config.cruising_speed, config.max_speed );
}
