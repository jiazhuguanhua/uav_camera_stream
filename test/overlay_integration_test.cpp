// Synthetic telemetry is ONLY published to /overlay_test/* on an isolated ROS master.
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/UInt32.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>

class Check {
  ros::NodeHandle nh_;
  ros::Publisher velocity_,attitude_,battery_,satellites_;
  ros::Subscriber diagnostics_;
  std::map<std::string,std::string> values_;
  std::ofstream log_;
  geometry_msgs::TwistStamped velocity_msg_;
  sensor_msgs::Imu attitude_msg_;
  sensor_msgs::BatteryState battery_msg_;
  std_msgs::UInt32 satellites_msg_;
  void publish() {velocity_.publish(velocity_msg_);attitude_.publish(attitude_msg_);battery_.publish(battery_msg_);satellites_.publish(satellites_msg_);}
  bool wait(const std::function<bool()>& predicate,double seconds,bool send) {
    const auto until=ros::WallTime::now()+ros::WallDuration(seconds);
    while(ros::ok() && ros::WallTime::now()<until){if(send)publish();ros::spinOnce();if(predicate())return true;ros::WallDuration(.05).sleep();}return false;
  }
  void check(bool ok,const std::string& message){log_<<(ok?"PASS ":"FAIL ")<<message<<'\n';log_.flush();if(!ok)throw std::runtime_error(message);}
  bool near(const std::string& key,double expected,double epsilon=.01){return values_.count(key) && std::abs(std::stod(values_[key])-expected)<epsilon;}
  void quaternion(double roll,double pitch,double yaw) {
    constexpr double rad=3.14159265358979323846/180;
    double cr=cos(roll*rad/2),sr=sin(roll*rad/2),cp=cos(pitch*rad/2),sp=sin(pitch*rad/2),cy=cos(yaw*rad/2),sy=sin(yaw*rad/2);
    auto& q=attitude_msg_.orientation;q.x=sr*cp*cy-cr*sp*sy;q.y=cr*sp*cy+sr*cp*sy;q.z=cr*cp*sy-sr*sp*cy;q.w=cr*cp*cy+sr*sp*sy;
  }
 public:
  Check(const std::string& logfile):log_(logfile) {
    velocity_=nh_.advertise<geometry_msgs::TwistStamped>("/overlay_test/velocity",1);
    attitude_=nh_.advertise<sensor_msgs::Imu>("/overlay_test/attitude",1);
    battery_=nh_.advertise<sensor_msgs::BatteryState>("/overlay_test/battery",1);
    satellites_=nh_.advertise<std_msgs::UInt32>("/overlay_test/satellites",1);
    diagnostics_=nh_.subscribe<diagnostic_msgs::DiagnosticArray>("/camera_stream/diagnostics",1,[this](const diagnostic_msgs::DiagnosticArray::ConstPtr& a){if(a->status.empty())return;values_.clear();for(const auto& k:a->status[0].values)values_[k.key]=k.value;});
  }
  void run() {
    check(wait([this]{return values_["telemetry_battery_state"]=="missing";},5,false),"missing topics produce missing, not zero");
    check(wait([this]{return velocity_.getNumSubscribers() && attitude_.getNumSubscribers() && battery_.getNumSubscribers() && satellites_.getNumSubscribers();},5,false),"configured topic names subscribed");
    velocity_msg_.twist.linear.x=3;velocity_msg_.twist.linear.y=4;velocity_msg_.twist.linear.z=-2.5;
    quaternion(30,20,-90);battery_msg_.present=true;battery_msg_.percentage=.73;satellites_msg_.data=17;
    check(wait([this]{return near("telemetry_horizontal_speed_value",5)&&near("telemetry_vertical_speed_value",-2.5)&&near("telemetry_pitch_value",20)&&near("telemetry_yaw_value",-90)&&near("telemetry_roll_value",30)&&near("telemetry_battery_value",73)&&near("telemetry_satellites_value",17);},5,true),"all seven numerical values cross ROS subscription correctly");
    auto text=values_["overlay_text"];log_<<"VALID_OVERLAY\n"<<text<<"\n";
    check(text.find("\n\nHSpeed:5.0m/s\nVSpeed:-2.5m/s\nPitch:20.0deg\nYaw:-90.0deg\nRoll:30.0deg\nBattery:73%\nSat:17")!=std::string::npos,"custom logical lines 4..10 and units/order correct");
    check(wait([this]{return values_.count("overlay_fps") && std::stod(values_["overlay_fps"])>20 && std::stod(values_["overlay_fps"])<36 && std::stod(values_["overlay_udp_mbps"])>0.5;},5,true),"fps and decimal Mbps measured from actual hardware stream");
    // Save a stable all-values screenshot while these known values keep publishing.
    const auto capture_until=ros::WallTime::now()+ros::WallDuration(8);
    while(ros::WallTime::now()<capture_until){publish();ros::spinOnce();ros::WallDuration(.05).sleep();}
    const double invalid=std::numeric_limits<double>::quiet_NaN();
    velocity_msg_.twist.linear.x=invalid;velocity_msg_.twist.linear.z=invalid;
    attitude_msg_.orientation_covariance[0]=-1;battery_msg_.percentage=invalid;satellites_msg_.data=255;
    check(wait([this]{for(const auto* n:{"horizontal_speed","vertical_speed","pitch","yaw","roll","battery","satellites"})if(values_[std::string("telemetry_")+n+"_state"]!="invalid")return false;return true;},5,true),"NaN velocity/battery, unavailable attitude and unknown satellites are invalid");
    check(values_["overlay_text"].find("Battery:N/A")!=std::string::npos,"invalid battery does not show 0 percent");
    check(wait([this]{for(const auto* n:{"horizontal_speed","vertical_speed","pitch","yaw","roll","battery","satellites"})if(values_[std::string("telemetry_")+n+"_state"]!="stale")return false;return true;},6,false),"all seven stale without incoming messages");
    check(values_["overlay_text"].find("Sat:STALE")!=std::string::npos,"stale shown on overlay");
    velocity_msg_.twist.linear.x=0;velocity_msg_.twist.linear.y=0;velocity_msg_.twist.linear.z=0;
    attitude_msg_.orientation_covariance[0]=0;quaternion(0,0,0);battery_msg_.percentage=0;satellites_msg_.data=0;
    check(wait([this]{return near("telemetry_battery_value",0)&&near("telemetry_satellites_value",0)&&near("telemetry_pitch_value",0)&&near("telemetry_yaw_value",0)&&near("telemetry_roll_value",0)&&near("telemetry_horizontal_speed_value",0)&&near("telemetry_vertical_speed_value",0);},5,true),"all seven recover and genuine zero values are accepted");
    log_<<"ALL ROS INTEGRATION CHECKS PASSED\n";
  }
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"overlay_integration_check");
  try {Check test(argc>1?argv[1]:"/tmp/overlay-integration.log");test.run();return 0;}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
