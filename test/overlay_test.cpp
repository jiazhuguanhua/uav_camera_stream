#include <uav_camera_stream/overlay.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>
#include <limits>
using namespace uav_camera_stream;
constexpr double pi=3.14159265358979323846;
static double nan() {return std::numeric_limits<double>::quiet_NaN();}
static double inf() {return std::numeric_limits<double>::infinity();}
TEST(Velocity, HorizontalMagnitudeAndSignedUp) {
  Telemetry t; t.velocity(3,-4,-2.5,10);
  EXPECT_DOUBLE_EQ(5,t.get(Metric::HorizontalSpeed).value);
  EXPECT_DOUBLE_EQ(-2.5,t.get(Metric::VerticalSpeed).value);
  t.velocity(0,0,0,11); EXPECT_EQ("ok",t.state(Metric::HorizontalSpeed,11,3));
  EXPECT_DOUBLE_EQ(0,t.get(Metric::HorizontalSpeed).value);
}
TEST(Velocity, InvalidComponentsAreIndependent) {
  Telemetry t; t.velocity(nan(),1,2,10);
  EXPECT_EQ("invalid",t.state(Metric::HorizontalSpeed,10,3)); EXPECT_EQ("ok",t.state(Metric::VerticalSpeed,10,3));
  t.velocity(3,4,inf(),11); EXPECT_EQ("ok",t.state(Metric::HorizontalSpeed,11,3)); EXPECT_EQ("invalid",t.state(Metric::VerticalSpeed,11,3));
  t.velocity(1e308,1e308,0,12); EXPECT_TRUE(std::isfinite(t.get(Metric::HorizontalSpeed).value));
}
TEST(Attitude, IdentityAndIndividualAxisSigns) {
  Telemetry t; t.attitude(0,0,0,1,true,1);
  for(auto id:{Metric::Roll,Metric::Pitch,Metric::Yaw}) EXPECT_NEAR(0,t.get(id).value,1e-9);
  t.attitude(std::sin(pi/12),0,0,std::cos(pi/12),true,2); EXPECT_NEAR(30,t.get(Metric::Roll).value,1e-9);
  t.attitude(0,std::sin(-pi/12),0,std::cos(-pi/12),true,3); EXPECT_NEAR(-30,t.get(Metric::Pitch).value,1e-9);
  t.attitude(0,0,std::sin(pi/4),std::cos(pi/4),true,4); EXPECT_NEAR(90,t.get(Metric::Yaw).value,1e-9);
}
TEST(Attitude, CombinedRotationNormalizesQuaternion) {
  double r=30*pi/180,p=20*pi/180,y=-90*pi/180;
  double cr=cos(r/2),sr=sin(r/2),cp=cos(p/2),sp=sin(p/2),cy=cos(y/2),sy=sin(y/2);
  Telemetry t;t.attitude(2*(sr*cp*cy-cr*sp*sy),2*(cr*sp*cy+sr*cp*sy),2*(cr*cp*sy-sr*sp*cy),2*(cr*cp*cy+sr*sp*sy),true,1);
  EXPECT_NEAR(30,t.get(Metric::Roll).value,1e-9);EXPECT_NEAR(20,t.get(Metric::Pitch).value,1e-9);EXPECT_NEAR(-90,t.get(Metric::Yaw).value,1e-9);
  t.attitude(0,sqrt(.5),0,sqrt(.5),true,2); EXPECT_NEAR(90,t.get(Metric::Pitch).value,1e-6);
}
TEST(Attitude, InvalidOrUnavailableDoesNotLookLevel) {
  Telemetry t;
  for(auto q:std::vector<std::array<double,4>>{{0,0,0,0},{nan(),0,0,1},{0,inf(),0,1}}) {
    t.attitude(q[0],q[1],q[2],q[3],true,1);
    for(auto id:{Metric::Roll,Metric::Pitch,Metric::Yaw})EXPECT_EQ("invalid",t.state(id,1,3));
  }
  t.attitude(0,0,0,1,false,1);EXPECT_EQ("invalid",t.state(Metric::Yaw,1,3));
}
TEST(Battery, FractionToPercentAndBoundaries) {
  Telemetry t;
  for(double v:{0.,.73,1.}){t.battery(v,true,1);EXPECT_EQ("ok",t.state(Metric::Battery,1,3));EXPECT_DOUBLE_EQ(v*100,t.get(Metric::Battery).value);}
  for(double v:{-1.,1.01,nan(),inf()}){t.battery(v,true,1);EXPECT_EQ("invalid",t.state(Metric::Battery,1,3));}
  t.battery(.5,false,1);EXPECT_EQ("invalid",t.state(Metric::Battery,1,3));
}
TEST(Satellites, ZeroIsRealAnd255IsUnknown) {
  Telemetry t;for(uint32_t n:{0u,17u,254u}){t.satellites(n,1);EXPECT_EQ("ok",t.state(Metric::Satellites,1,3));EXPECT_EQ(n,t.get(Metric::Satellites).value);}
  for(uint32_t n:{255u,256u,65535u}){t.satellites(n,1);EXPECT_EQ("invalid",t.state(Metric::Satellites,1,3));}
}
TEST(Freshness, MissingStaleRecoveryAndTimeZero) {
  Telemetry t; EXPECT_EQ("missing",t.state(Metric::Battery,10,3));
  t.battery(.5,true,0);EXPECT_EQ("ok",t.state(Metric::Battery,3,3));EXPECT_EQ("stale",t.state(Metric::Battery,3.01,3));
  t.battery(nan(),true,4);EXPECT_EQ("invalid",t.state(Metric::Battery,4,3));EXPECT_EQ("stale",t.state(Metric::Battery,8,3));
  t.battery(.75,true,9);EXPECT_EQ("ok",t.state(Metric::Battery,9,3));EXPECT_EQ("stale",t.state(Metric::Battery,8,3));
}
TEST(Time, CompactOffsetsIncludingFractionalAndDst) {
  const char* old=getenv("TZ");const std::string saved=old?old:""; const bool had=old;
  tm utc{};utc.tm_year=126;utc.tm_mon=9;utc.tm_mday=19;utc.tm_hour=11;utc.tm_min=33;utc.tm_sec=21;const auto now=timegm(&utc);
  for(auto c:std::vector<std::pair<std::string,std::string>>{{"Asia/Shanghai","261019-193321+8"},{"UTC","261019-113321+0"},{"Asia/Kolkata","261019-170321+5:30"},{"Asia/Kathmandu","261019-171821+5:45"},{"America/Los_Angeles","261019-043321-7"}}){setenv("TZ",c.first.c_str(),1);tzset();EXPECT_EQ(c.second,wallTime(now,"%y%m%d-%H%M%S"));}
  if(had)setenv("TZ",saved.c_str(),1);else unsetenv("TZ");tzset();
}
TEST(Render, DefaultOnlyBatteryAndSatAndMeasuredMbps) {
  Layout l;Telemetry t;t.battery(.73,true,10);t.satellites(17,10);
  auto text=render(l,t,0,10,"%y%m%d-%H%M%S",false,true,29,12);
  EXPECT_EQ("\n29fps-12.00Mbps\nBattery:73%  Sat:17",text);
  EXPECT_EQ(std::string::npos,text.find("Pitch"));
  l.mbps_precision=0;EXPECT_NE(std::string::npos,render(l,t,0,10,"",false,true,29,12).find("29fps-12Mbps"));
}
TEST(Render, PlaceholdersLinePositionsAndAllSevenValues) {
  Layout l;l.fields[0].line=6;l.fields[1].line=6;
  for(auto& f:l.fields)f.enabled=true;
  l.validate();Telemetry t;t.velocity(3,4,-2,1);t.attitude(0,0,0,1,true,1);t.battery(.5,true,1);t.satellites(0,1);
  auto text=render(l,t,0,1,"",false,true,29.5,2.012);
  EXPECT_EQ("\n30fps-2.01Mbps\n\nHSpeed:5.0m/s  VSpeed:-2.0m/s\nPitch:0.0deg  Yaw:0.0deg  Roll:0.0deg\nBattery:50%  Sat:0",text);
  t.battery(nan(),true,2);text=render(l,t,0,2,"",false,false,0,0);EXPECT_NE(std::string::npos,text.find("Battery:N/A"));EXPECT_NE(std::string::npos,text.find("--fps---Mbps"));
  text=render(l,t,0,6,"",false,false,0,0);EXPECT_NE(std::string::npos,text.find("Sat:STALE"));
  EXPECT_EQ("0.0",number(-.0001,1));EXPECT_LT(number(1e300,1).size(),20u);
}
TEST(Config, TopicAndLayoutOverrides) {
  XmlRpc::XmlRpcValue c;c["topics"]["battery"]="/test/battery";c["font_desc"]="Sans 18";
  c["fields"].setSize(1);c["fields"][0]["name"]="pitch";c["fields"][0]["enabled"]=true;c["fields"][0]["line"]=7;
  auto l=Layout::parse(c);ASSERT_EQ(1u,l.fields.size());EXPECT_EQ(Metric::Pitch,l.fields[0].id);EXPECT_EQ(7,l.fields[0].line);EXPECT_EQ("/test/battery",l.battery_topic);
}
TEST(Config, RejectsTyposTypesDuplicatesConflictsAndInvalidTopics) {
  XmlRpc::XmlRpcValue c;c["typo"]=1;
  EXPECT_THROW(Layout::parse(c),std::runtime_error);
  c.clear();c["update_hz"]="5";
  EXPECT_THROW(Layout::parse(c),std::runtime_error);
  c.clear();c["topics"]["battery"]="relative/name";
  EXPECT_THROW(Layout::parse(c),std::runtime_error);
  c.clear();c["fields"].setSize(2);for(int i=0;i<2;++i)c["fields"][i]["name"]="battery";
  EXPECT_THROW(Layout::parse(c),std::runtime_error);
  c.clear();c["fields"].setSize(1);c["fields"][0]["name"]="batttery";
  EXPECT_THROW(Layout::parse(c),std::runtime_error);
  c.clear();c["fields"].setSize(1);c["fields"][0]["name"]="battery";c["fields"][0]["line"]=1;
  EXPECT_THROW(Layout::parse(c),std::runtime_error);
  Layout l;l.update_hz=nan();
  EXPECT_THROW(l.validate(),std::runtime_error);l=Layout();l.telemetry_timeout_sec=0;
  EXPECT_THROW(l.validate(),std::runtime_error);
  l=Layout();l.fields[0].precision=4;
  EXPECT_THROW(l.validate(),std::runtime_error);l=Layout();l.fields[0].line=13;
  EXPECT_THROW(l.validate(),std::runtime_error);
  l=Layout();l.fields[0].label="Battery\nInjected";
  EXPECT_THROW(l.validate(),std::runtime_error);
}
int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
