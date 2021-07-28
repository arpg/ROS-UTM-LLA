#include "utm_lla_converter.h"
using namespace std;

ULConverter::ULConverter(string hemi, int zone, double at, double fla, double k0):tm_(at,fla,k0),zone_(zone), hemi_(hemi){};

struct basic{
  double bx = 0.0;//-594929.9431329881;//east
  double by = 0.0;//-4139043.529676078;//north
  double bz = 0.0;//unit in m
};//origin point

void ULConverter::RegiHandle(ros::NodeHandle &n)
{
  gps_pub_ = n.advertise<sensor_msgs::NavSatFix>("/gps",10);
  xyz_pub_ = n.advertise<geometry_msgs::PoseStamped>("/utm",10);
};

void ULConverter::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs)
{ 
  basic bs;
  sensor_msgs::NavSatFix gps;
  gps.header = msgs->header;

  //position from NDT
  double px = -bs.bx + msgs->pose.position.x ;
  double py = -bs.by + msgs->pose.position.y ;
  double pz = -bs.bz + msgs->pose.position.z;//height not change, just output unit in meters
  ROS_INFO("Get data x: [%f]", px);
  ROS_INFO("Get data y: [%f]", py);
  ROS_INFO("Get data z: [%f]", pz);
  
  if(hemi_ == "North")
    UTMConvert2LLA(NorthH, zone_, px, py, pz);//hemisphere, zone, x, y, height
  else if(hemi_ == "South")
   UTMConvert2LLA(SouthH, zone_, px, py, pz);
  else{
    ROS_ERROR("only North or South Hemisphere. Are you on Mars???");
    ros::shutdown();
  }
    
  gps.latitude  = lla_[0];
  gps.longitude = lla_[1];
  gps.altitude  = lla_[2];
  
  ROS_INFO("Convert to lagitude: [%f]", lla_[0]);
  ROS_INFO("Convert to longitude: [%f]", lla_[1]);
  ROS_INFO("Convert to altitude: [%f]", lla_[2]);
  
  gps_pub_.publish(gps);
  lla_.clear();//don't forget this
}

void ULConverter::GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msgs)
{
  basic bs;
  ROS_INFO("Get data lagitude from GPS: [%f]", msgs->latitude);
  ROS_INFO("Get data longitude from GPS: [%f]", msgs->longitude);
  ROS_INFO("Get data altitude from GPS: [%f]", msgs->altitude);

  if(hemi_ == "North")
    LLAConvert2UTM(NorthH, zone_, msgs->latitude, msgs->longitude, msgs->altitude);
  else if(hemi_ == "South")
    LLAConvert2UTM(SouthH, zone_, msgs->latitude, msgs->longitude, msgs->altitude);
  else{
    ROS_ERROR("only North or South Hemisphere. Are you on Mars?");
    ros::shutdown();
  }

  geometry_msgs::PoseStamped local_pose;

  local_pose.header = msgs->header;
  local_pose.pose.position.x = utm_[0] + bs.bx;
  local_pose.pose.position.y = utm_[1] + bs.by;
  local_pose.pose.position.z = utm_[2] + bs.bz;

  ROS_INFO("Convert to x: [%f]", local_pose.pose.position.x);
  ROS_INFO("Convert to y: [%f]", local_pose.pose.position.y);
  ROS_INFO("Convert to z: [%f]", local_pose.pose.position.z);

  xyz_pub_.publish(local_pose);

  utm_.clear();
}

void ULConverter::UTMConvert2LLA(Hemi hemi,int zone, double east, double north, double height)
{
  
  double latitude;
  double longitude;

  int lon0   = zone*6-183;
  east  -= kE0_;
  north -= (hemi==NorthH) ? kNN_ : kNS_; 

  tm_.Reverse(lon0, east, north, latitude, longitude); 

  lla_.push_back(latitude); lla_.push_back(longitude); lla_.push_back(height);
};

void ULConverter::LLAConvert2UTM(Hemi hemi, int zone, double latitude, double longitude, double height){
  double east,north;

  int lon0  = zone*6-183;

  tm_.Forward(lon0, latitude, longitude, east, north);

  east += kE0_;
  north += (hemi==NorthH) ? kNN_ : kNS_;

  utm_.push_back(east); utm_.push_back(north); utm_.push_back(height);
}

std::vector<double> ULConverter::get_lla()
{
  return lla_;
}
