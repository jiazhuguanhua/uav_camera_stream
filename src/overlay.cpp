#include <uav_camera_stream/overlay.hpp>
#include <ros/names.h>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>

namespace uav_camera_stream {
namespace {
double steadyNow() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
void require(bool ok, const std::string& msg) { if (!ok) throw std::runtime_error("overlay config: " + msg); }
bool clean(const std::string& s, size_t max) {
  return s.size() <= max && std::none_of(s.begin(), s.end(), [](unsigned char c) { return c < 32 || c == 127; });
}
void keys(const XmlRpc::XmlRpcValue& v, const std::set<std::string>& allowed) {
  require(v.getType() == XmlRpc::XmlRpcValue::TypeStruct, "expected a mapping");
  for (auto it = v.begin(); it != v.end(); ++it) require(allowed.count(it->first), "unknown key: " + it->first);
}
void read(const XmlRpc::XmlRpcValue& v, const char* key, std::string& out) {
  if (!v.hasMember(key)) return;
  require(v[key].getType() == XmlRpc::XmlRpcValue::TypeString, std::string(key) + " must be a string"); out = static_cast<std::string>(v[key]);
}
void read(const XmlRpc::XmlRpcValue& v, const char* key, bool& out) {
  if (!v.hasMember(key)) return;
  require(v[key].getType() == XmlRpc::XmlRpcValue::TypeBoolean, std::string(key) + " must be boolean"); out = static_cast<bool>(v[key]);
}
void read(const XmlRpc::XmlRpcValue& v, const char* key, int& out) {
  if (!v.hasMember(key)) return;
  require(v[key].getType() == XmlRpc::XmlRpcValue::TypeInt, std::string(key) + " must be integer"); out = static_cast<int>(v[key]);
}
void read(const XmlRpc::XmlRpcValue& v, const char* key, double& out) {
  if (!v.hasMember(key)) return;
  const auto type = v[key].getType();
  require(type == XmlRpc::XmlRpcValue::TypeDouble || type == XmlRpc::XmlRpcValue::TypeInt, std::string(key) + " must be numeric");
  out = type == XmlRpc::XmlRpcValue::TypeInt ? static_cast<int>(v[key]) : static_cast<double>(v[key]);
}
}
const char* metricName(Metric id) {
  static const char* names[] = {"horizontal_speed", "vertical_speed", "pitch", "yaw", "roll", "battery", "satellites"};
  return names[static_cast<size_t>(id)];
}
void Telemetry::set(Metric id, double value, bool valid, double now) {
  values_.at(static_cast<size_t>(id)) = {value, now, true, valid && std::isfinite(value)};
}
void Telemetry::velocity(double x, double y, double z, double now) {
  set(Metric::HorizontalSpeed, std::hypot(x,y), std::isfinite(x) && std::isfinite(y), now);
  set(Metric::VerticalSpeed, z, true, now); // MAVROS local ENU: positive upward, m/s.
}
void Telemetry::attitude(double x, double y, double z, double w, bool available, double now) {
  const double norm = std::hypot(std::hypot(x,y), std::hypot(z,w));
  const bool valid = available && std::isfinite(norm) && norm > 1e-12;
  if (!valid) { for (auto id : {Metric::Roll,Metric::Pitch,Metric::Yaw}) set(id,0,false,now); return; }
  x/=norm; y/=norm; z/=norm; w/=norm;
  constexpr double deg = 180.0 / 3.14159265358979323846;
  set(Metric::Roll, std::atan2(2*(w*x+y*z),1-2*(x*x+y*y))*deg, true, now);
  set(Metric::Pitch, std::asin(std::max(-1.0,std::min(1.0,2*(w*y-z*x))))*deg, true, now);
  set(Metric::Yaw, std::atan2(2*(w*z+x*y),1-2*(y*y+z*z))*deg, true, now);
}
void Telemetry::battery(double fraction, bool present, double now) {
  set(Metric::Battery, fraction*100, present && fraction >= 0 && fraction <= 1, now);
}
void Telemetry::satellites(uint32_t count, double now) { set(Metric::Satellites,count,count < 255,now); }
std::string Telemetry::state(Metric id, double now, double timeout) const {
  const auto& s = get(id);
  if (!s.received) return "missing";
  if (now < s.received_at || now-s.received_at > timeout) return "stale";
  return s.valid ? "ok" : "invalid";
}
Layout::Layout() : fields{
  {Metric::Battery,true,3,"Battery",0,"%"}, {Metric::Satellites,true,3,"Sat",0,""},
  {Metric::HorizontalSpeed,false,4,"HSpeed",1,"m/s"}, {Metric::VerticalSpeed,false,4,"VSpeed",1,"m/s"},
  {Metric::Pitch,false,5,"Pitch",1,"deg"}, {Metric::Yaw,false,5,"Yaw",1,"deg"}, {Metric::Roll,false,5,"Roll",1,"deg"}} {}
Layout Layout::parse(const XmlRpc::XmlRpcValue& c) {
  Layout l;
  keys(c, {"update_hz","telemetry_timeout_sec","time_line","stats_line","stats_enabled","fps_precision","mbps_precision",
           "xpad","ypad","font_desc","missing_text","stale_text","invalid_text","separator","topics","fields"});
#define READ(n) read(c,#n,l.n)
  READ(update_hz); READ(telemetry_timeout_sec); READ(time_line); READ(stats_line); READ(stats_enabled);
  READ(fps_precision); READ(mbps_precision); READ(xpad); READ(ypad); READ(font_desc);
  READ(missing_text); READ(stale_text); READ(invalid_text); READ(separator);
#undef READ
  if (c.hasMember("topics")) {
    const auto& t=c["topics"]; keys(t,{"velocity","attitude","battery","satellites"});
    read(t,"velocity",l.velocity_topic); read(t,"attitude",l.attitude_topic);
    read(t,"battery",l.battery_topic); read(t,"satellites",l.satellites_topic);
  }
  if (c.hasMember("fields")) {
    require(c["fields"].getType()==XmlRpc::XmlRpcValue::TypeArray,"fields must be an array");
    std::vector<Field> parsed;
    for (int i=0;i<c["fields"].size();++i) {
      const auto& item=c["fields"][i]; keys(item,{"name","enabled","line","label","precision","unit"});
      std::string name; read(item,"name",name);
      auto it=std::find_if(l.fields.begin(),l.fields.end(),[&name](const Field& f){return name==metricName(f.id);});
      require(it!=l.fields.end(),"unknown/missing field name: "+name);
      Field f=*it; read(item,"enabled",f.enabled); read(item,"line",f.line); read(item,"label",f.label);
      read(item,"precision",f.precision); read(item,"unit",f.unit); parsed.push_back(f);
    }
    l.fields=parsed;
  }
  l.validate(); return l;
}
void Layout::validate() const {
  require(std::isfinite(update_hz) && update_hz>=1 && update_hz<=20,"update_hz range: 1..20");
  require(std::isfinite(telemetry_timeout_sec) && telemetry_timeout_sec>=0.1 && telemetry_timeout_sec<=300,"telemetry_timeout_sec range: 0.1..300");
  require(time_line>=1 && time_line<=12 && stats_line>=1 && stats_line<=12,"line range: 1..12");
  require(!stats_enabled || time_line!=stats_line,"time_line and stats_line must differ");
  require(fps_precision>=0 && fps_precision<=3 && mbps_precision>=0 && mbps_precision<=3,"precision range: 0..3");
  require(xpad>=0 && xpad<=2048 && ypad>=0 && ypad<=2048,"padding range: 0..2048");
  require(!font_desc.empty() && clean(font_desc,128),"invalid font_desc");
  require(clean(missing_text,16) && !missing_text.empty() && clean(stale_text,16) && !stale_text.empty() &&
          clean(invalid_text,16) && !invalid_text.empty() && clean(separator,16),"invalid placeholders/separator");
  for (const auto& topic : {velocity_topic,attitude_topic,battery_topic,satellites_topic}) {
    std::string error; require(!topic.empty() && topic[0]=='/' && ros::names::validate(topic,error),"topic must be an absolute ROS name: "+topic);
  }
  std::set<Metric> seen;
  for (const auto& f : fields) {
    require(seen.insert(f.id).second,"duplicate field: "+std::string(metricName(f.id)));
    require(f.line>=1 && f.line<=12,"field line range: 1..12");
    require(!f.enabled || (f.line!=time_line && (!stats_enabled || f.line!=stats_line)),"telemetry line conflicts with time/stats line");
    require(f.precision>=0 && f.precision<=3,"field precision range: 0..3");
    require(clean(f.label,32) && !f.label.empty() && clean(f.unit,12),"invalid field label/unit");
  }
}
std::string number(double value, int precision) {
  if (!std::isfinite(value)) return "N/A";
  // Avoid a confusing negative zero, and cap representation of extreme finite input.
  if (std::abs(value)<0.5*std::pow(10.0,-precision)) value=0;
  std::ostringstream s; s.imbue(std::locale::classic());
  if (std::abs(value)>=1e6) s<<std::scientific; else s<<std::fixed;
  s<<std::setprecision(precision)<<value; return s.str();
}
std::string wallTime(time_t now, const std::string& format) {
  tm local{}; if (!localtime_r(&now,&local)) throw std::runtime_error("localtime failed");
  char date[1024]{}, offset[16]{};
  if (!strftime(date,sizeof(date),format.c_str(),&local) || !strftime(offset,sizeof(offset),"%z",&local))
    throw std::runtime_error("time formatting failed");
  const std::string z=offset;
  if (z.size()!=5 || (z[0]!='+' && z[0]!='-') || !std::all_of(z.begin()+1,z.end(),[](char c){return c>='0' && c<='9';}))
    throw std::runtime_error("timezone did not provide a numeric UTC offset");
  std::string compact=z.substr(0,1)+std::to_string(std::stoi(z.substr(1,2)));
  if (z.substr(3,2)!="00") compact+=":"+z.substr(3,2);
  return std::string(date)+compact;
}
std::string render(const Layout& l,const Telemetry& data,time_t wall,double steady,const std::string& format,
                   bool show_time,bool ready,double fps,double mbps) {
  std::array<std::string,12> lines{};
  if(show_time) lines[l.time_line-1]=wallTime(wall,format);
  if(l.stats_enabled) lines[l.stats_line-1]=(ready?number(fps,l.fps_precision):l.missing_text)+"fps-"+
                                       (ready?number(mbps,l.mbps_precision):l.missing_text)+"Mbps";
  for(const auto& f:l.fields) {
    if(!f.enabled)continue;
    auto state=data.state(f.id,steady,l.telemetry_timeout_sec);
    std::string text=state=="ok"?number(data.get(f.id).value,f.precision)+f.unit:
                     state=="missing"?l.missing_text:state=="stale"?l.stale_text:l.invalid_text;
    auto& line=lines[f.line-1]; if(!line.empty())line+=l.separator; line+=f.label+":"+text;
  }
  int end=12; while(end>0 && lines[end-1].empty())--end;
  std::string text; for(int i=0;i<end;++i){if(i)text+='\n';text+=lines[i];} return text;
}
Overlay::Overlay(ros::NodeHandle& p) {
  XmlRpc::XmlRpcValue config;
  if(p.getParam("overlay",config))layout_=Layout::parse(config); else layout_.validate();
  bool velocity=false,attitude=false,battery=false,satellites=false;
  for(const auto& f:layout_.fields)if(f.enabled) {
    velocity|=f.id==Metric::HorizontalSpeed || f.id==Metric::VerticalSpeed;
    attitude|=f.id==Metric::Pitch || f.id==Metric::Yaw || f.id==Metric::Roll;
    battery|=f.id==Metric::Battery; satellites|=f.id==Metric::Satellites;
  }
  ros::NodeHandle nh;
  const auto hints=ros::TransportHints().tcpNoDelay();
  if(velocity) subscribers_.push_back(nh.subscribe<geometry_msgs::TwistStamped>(layout_.velocity_topic,1,
    [this](const geometry_msgs::TwistStamped::ConstPtr& m){telemetry_.velocity(m->twist.linear.x,m->twist.linear.y,m->twist.linear.z,steadyNow());},ros::VoidConstPtr(),hints));
  if(attitude) subscribers_.push_back(nh.subscribe<sensor_msgs::Imu>(layout_.attitude_topic,1,
    [this](const sensor_msgs::Imu::ConstPtr& m){telemetry_.attitude(m->orientation.x,m->orientation.y,m->orientation.z,m->orientation.w,m->orientation_covariance[0]!=-1,steadyNow());},ros::VoidConstPtr(),hints));
  if(battery) subscribers_.push_back(nh.subscribe<sensor_msgs::BatteryState>(layout_.battery_topic,1,
    [this](const sensor_msgs::BatteryState::ConstPtr& m){telemetry_.battery(m->percentage,m->present,steadyNow());},ros::VoidConstPtr(),hints));
  if(satellites) subscribers_.push_back(nh.subscribe<std_msgs::UInt32>(layout_.satellites_topic,1,
    [this](const std_msgs::UInt32::ConstPtr& m){telemetry_.satellites(m->data,steadyNow());},ros::VoidConstPtr(),hints));
}
} // namespace uav_camera_stream
