#include "ros/ros.h"
#include "utm_lla_converter.h"
#include "string"

int main(int argc, char **argv)
{
  ros::init(argc,argv,"convert_ndt2lla");

  ros::NodeHandle n;

  double at      = 6378137.0; //unit in m
  double fla    = 1.0/298.257223563; 
  double k0      = 0.9996;

  int zone = 0;
  std::string hemisphere;
  if(n.getParam("utm_zone",zone)){
    ROS_INFO("UTM coordinate in zone %d ",zone);
  }
  else{
    ROS_ERROR("Error! You must specify the utm zone. Easy to find it at https://mangomap.com/robertyoung/maps/69585/what-utm-zone-am-i-in-#");
    ros::shutdown();
  }

  if(n.getParam("hemisphere",hemisphere)){
    ROS_INFO("North hemisphere or south hemisphere? %s ",hemisphere.c_str());
    if(hemisphere != "North" && hemisphere != "South"){
      ROS_ERROR("hemisphere only can equal to North or South!");
    }
  }
  else{
    ROS_WARN("You need to specify you are in the north or south hemisphere. Default north");
    hemisphere = "North";
  }

  ULConverter con(hemisphere,zone,at,fla,k0);

  con.RegiHandle(n);
 
  ros::Subscriber  sub1 = n.subscribe("/utm_pose",1000,&ULConverter::PoseCallback, &con);
  ros::Subscriber  sub2 = n.subscribe("/gps/fix",1000,&ULConverter::GPSCallback, &con);

  ros::spin();

  return 0;  
}
