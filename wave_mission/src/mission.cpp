using namespace std;
#include <wave_mission/mission.h>
#include <wave_mission/MissionState.h>

//converts an angle to (0,360) then [-180,180)
inline double wrapAngle( double angle )
{
    double twoPi = 2.0 * 180;
    double ang =  angle - twoPi * floor( angle / twoPi );
    if (ang >= 180) ang = ang - 360; 
    return ang; 
}

MissionObject::MissionObject() : robotHeading(0), targetBearing(0), robotW(0), robotX(0)
{
  ros::NodeHandle node_priv("~");

  // Get params if specified in launch file or as params on command-line, set
  // defaults
  node_priv.param<string>("mission_name", mission_name, "NULL");
  node_priv.param<double>("path_tolerance", path_tolerance, 1.0);
  node_priv.param<double>("waypoint_tolerance", waypoint_tolerance, 1.0);
  node_priv.param<double>("cruising_speed", cruising_speed, 0.5);
  node_priv.param<double>("max_speed", max_speed, 1.0);
  node_priv.param<double>("look_ahead_distance", lookahead_distance, 5);
  node_priv.param<double> ("prediction_time", prediction_time, 1);
  node_priv.param<string>("location_topic", location_topic, "ublox_gps/fix");
  node_priv.param<string>("velocity_topic", velocity_topic, "mobile_base_controller/sensed_vel");
  node_priv.param<string>("heading_topic", heading_topic, "sensed/yaw");
  node_priv.param<string>("mission_stats_topic", mission_stats_topic, "wave_mission/stats");
  node_priv.param<string>("mission_enable_topic", mission_enable_topic, "wave_mission/enable");
  node_priv.param<string>("bearing_output_topic", bearing_output_topic, "ref/yaw");
  ROS_INFO("%s", bearing_output_topic.c_str());
  firstLocation = true;
  firstHeading = true;
  firstVelocity = true;
  setMissionPlan = false;
  missionEnabled = false; 
  currSeg = 0; 
  dynamic_reconfigure::Server<wave_mission::WaveMissionConfig> server;
  dynamic_reconfigure::Server<wave_mission::WaveMissionConfig>::CallbackType f;
  ros::Subscriber enable_sub = node.subscribe(mission_enable_topic, 1, &MissionObject::enableCallback, this);

  heading_pub = node.advertise<std_msgs::Float64>(bearing_output_topic, 10);
  mission_stats_pub = node.advertise<std_msgs::Float64MultiArray>(mission_stats_topic, 1);

  f = boost::bind(&MissionObject::reconfigureCallback, this, _1, _2);
  server.setCallback(f);


  getPlan("pottery");
  while (ros::ok())
  {
    // doCalcs();
    ros::spinOnce();
    doCalcs(); 

    // Add a small sleep to avoid 100% CPU usage
    ros::Duration(0.001).sleep();
  }
}


void MissionObject::getPlan(const string& planName)
{
  ROS_INFO("Getting mission plan from server");

  const string path = "/plan?planName=" + planName;
  httplib::SSLClient cli("localhost", 5000);
  auto res = cli.Get(path.c_str());
  const char *recievedPlan;
  if (res && res->status == 200)
  {
    ROS_INFO("Mission plan received");
    recievedPlan = res->body.c_str();
    rapidjson::Document d;
    d.Parse(recievedPlan);
    for (int i = 0; i < d.Size(); i++)
    {
      double startLat = atof(d[i]["startLat"].GetString());
      double startLong = atof(d[i]["startLong"].GetString());
      LatLng start = LatLng(startLat, startLong);
      double endLat = atof(d[i]["endLat"].GetString());
      double endLong = atof(d[i]["endLong"].GetString());
      LatLng end = LatLng(endLat, endLong);
      double bearing = atof(d[i]["bearing"].GetString());
      double dist =  SphericalUtil::computeDistanceBetween(start, end); 

      Segment s = Segment(start, end, bearing, dist);
      MissionObject::segments.push_back(s);
      //  ROS_INFO("%.10f %.10f %.10f %.10f %.10f", startLat, startLong, endLat, endLong, bearing);
    }
    setMissionPlan = true; 
    targetWaypoint = segments.front().start; 
  }
  else
  {
    ROS_ERROR("Unable to get mission plan");
  }
}

void MissionObject::enableCallback(const std_msgs::Bool& enable) {
  if(enable.data) {
     ROS_INFO("Starting mission");
     MissionObject::startSubs(); 
     missionEnabled = true; 
  } else {
    ROS_WARN("Stopping mission");
    MissionObject::stopSubs(); 
    missionEnabled = false; 
  }
 }

void MissionObject::locationCallback(const sensor_msgs::NavSatFix& fix) 
{
  if(firstLocation)
      firstLocation = false;

  double lat = fix.latitude; 
  double lng = fix.longitude; 
  robotPos = LatLng(lat,lng);  
}


void MissionObject::velocityCallback(const geometry_msgs::Twist &vel) {
    if(firstVelocity)
        firstVelocity = false;

  robotX = vel.linear.x;
  robotW = vel.angular.z; 
}
//[-180,180]
void MissionObject::headingCallback(const std_msgs::Float64 &yaw) {
    if(firstHeading)
        firstHeading = false;
  robotHeading = yaw.data; 
}


void MissionObject::startSubs() {
 ROS_INFO("Subscribing to updates"); 
 location_sub = node.subscribe(location_topic,1, &MissionObject::locationCallback, this);
 velocity_sub = node.subscribe(velocity_topic,1,&MissionObject::velocityCallback, this);
 heading_sub = node.subscribe(heading_topic,1,&MissionObject::headingCallback, this);
 
}

void MissionObject::stopSubs() {
  ROS_WARN("Removing subscription to updates"); 
  location_sub.shutdown(); 
  velocity_sub.shutdown();
  heading_sub.shutdown(); 
}

double toDegrees(double rad) {
  return  rad * 180.0 / M_PI;
}

static LatLng findPerpPoint(LatLng loc, Segment line) {
  double distToloc = SphericalUtil::computeDistanceBetween(line.start, loc);
  double bearingToLoc = SphericalUtil::computeHeading(line.start, loc);
  if ((line.bearing > 0 && bearingToLoc < 0) || (line.bearing < 0 && bearingToLoc > 0))
    return line.start;
  double ang = abs(line.bearing - bearingToLoc);
  double distanceOnLine = cos(ang * M_PI/ 180.0)*distToloc;
  if (distanceOnLine >= line.distance)
  return line.end; 
  else 
  return SphericalUtil::computeOffset(line.start, line.distance, line.bearing); 

}

LatLng MissionObject::predictFutureLocation() {
  double dist = prediction_time*robotX; 
  double ang = toDegrees(prediction_time*robotW); 
  ang += robotHeading;  
  ang = wrapAngle(ang); 
  LatLng predictedPoint = SphericalUtil::computeOffset(robotPos, dist, ang);
  return predictedPoint; 
}

LatLng MissionObject::calcOffsetPoint(LatLng perpPoint, Segment line) const {
LatLng offset = SphericalUtil::computeOffset(perpPoint, lookahead_distance, line.bearing); 
double totalDist = SphericalUtil::computeDistanceBetween(line.start, offset);
if (totalDist >= line.distance)
return line.end; 
else
return offset; 
}

void MissionObject::doCalcs() {

if(!missionEnabled) {
    ROS_WARN_DELAYED_THROTTLE(5, "Mission disabled");
    return;
}
if(!setMissionPlan) {
     ROS_ERROR_THROTTLE(3, "Mission cannot be enabled without a plan. Set plan parameter first.");
     return; 
}
if(firstLocation) {
    ROS_WARN_THROTTLE(2, "Waiting for first location...");
    return;
}
if(firstHeading) {
    ROS_WARN_THROTTLE(2,"Waiting for first heading...");
    return;
}
if(firstVelocity) {
    ROS_WARN_THROTTLE(2, "Waiting for first velocity...");
    return;
}

Segment &curr = segments[currSeg]; 

double distToWaypoint = SphericalUtil::computeDistanceBetween(robotPos, targetWaypoint);
if (distToWaypoint <= waypoint_tolerance) {
  currSeg++; 
  curr = segments[currSeg];
  targetWaypoint = curr.end; 
}

LatLng futurePoint = predictFutureLocation(); 
LatLng perpPoint = findPerpPoint(futurePoint, curr); 
double perpDist = SphericalUtil::computeDistanceBetween(futurePoint, perpPoint); 
if (perpDist <= path_tolerance) {
targetPos = curr.end; 
targetBearing = SphericalUtil::computeHeading(robotPos, targetPos); 
} else {
targetPos = calcOffsetPoint(perpPoint, curr);
targetBearing = SphericalUtil::computeHeading(robotPos, targetPos); 
}

geographic_msgs::GeoPoint geoPerp, geoTarget, geoFuture;
geoPerp.latitude = perpPoint.lat;
geoPerp.longitude = perpPoint.lng;
geoTarget.latitude = targetPos.lat;
geoTarget.longitude = targetPos.lng;
geoFuture.latitude = futurePoint.lat;
geoFuture.longitude = futurePoint.lng;

wave_mission::MissionState stats_msg;
stats_msg.current_segment = currSeg;
stats_msg.distance_to_waypoint = distToWaypoint;
stats_msg.perp_distance = perpDist;
stats_msg.perp_position = geoPerp;
stats_msg.target_position = geoTarget;
stats_msg.future_position = geoFuture;
stats_msg.target_bearing = targetBearing;
std_msgs::Float64 target_bearing_msg;
target_bearing_msg.data = targetBearing;

heading_pub.publish(target_bearing_msg);
mission_stats_pub.publish(stats_msg);



}


void MissionObject::reconfigureCallback(wave_mission::WaveMissionConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %s %f %f %f %f",
           config.mission_name.c_str(), 
           config.path_tolerance,
           config.waypoint_tolerance, config.cruising_speed, config.max_speed);

  if(mission_name == config.mission_name && config.mission_name != "NULL") {
    mission_name = config.mission_name; 
    getPlan(mission_name); 
  }
  if (config.path_tolerance > 0)
  path_tolerance = config.path_tolerance; 
  if (config.waypoint_tolerance > 0)
  waypoint_tolerance = config.waypoint_tolerance; 
  if (config.cruising_speed >= 0) 
  cruising_speed = config.cruising_speed; 
  if (config.max_speed >= 0)
  max_speed = config.max_speed; 

}
