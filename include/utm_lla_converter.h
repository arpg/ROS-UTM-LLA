#ifndef GEO_TOLLA_H
#define GEO_TOLLA_H

#include <math.h>
#include <vector>
#include "ros/ros.h"
#include "tf/transform_datatypes.h"//for tf, getYaw
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include <GeographicLib/TransverseMercator.hpp>
//resource from wiki  Universal Transverse Mercator coordinate system. WGS 84
enum Hemi {NorthH, SouthH};

class ULConverter{ 
  public:
    ULConverter(std::string hemi, int zone, double at, double fla, double k0); //function declaration does not need to assign value to others(see source file)  
    void RegiHandle(ros::NodeHandle &n);
    void UTMConvert2LLA(Hemi hemi,int zone, double east, double north, double height);
    void LLAConvert2UTM(Hemi hemi, int zone, double latitude, double longitude, double altitude);
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs);
    void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msgs);
    std::vector<double> get_lla();
  private:
   const double kNN_      = 0;
   const double kNS_      = 10000000.0;
   const double kE0_      = 500000.0;
   const double kPI_      = 3.14159265359;
   const double kDist_    = 1.2;

   std::vector<double> lla_;
   std::vector<double> utm_;
   GeographicLib::TransverseMercator tm_;  //tm need an initial value for constructor:  at(earth radius),fla(inverse flattening), k0. if I define at, fla and k0 as private member and use it here then the compiler will show error. I need assign the value from geo_ToLLA constructor.
   ros::Publisher gps_pub_;
   ros::Publisher xyz_pub_;

   size_t tmp_counter = 0;
   double tmp_x, tmp_y, tmp_z;
   std::vector<double> his_x,his_y,his_z;
   int zone_;
   std::string hemi_;
};

#endif


