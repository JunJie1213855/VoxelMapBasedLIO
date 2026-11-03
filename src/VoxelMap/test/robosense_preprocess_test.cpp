#include <gtest/gtest.h>
#include "preprocess.h"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace {
struct Return {
  float x=2,y=0,z=1,intensity=42;
  uint16_t ring=0;
  double time=1000;
};
template <typename T> void write(std::vector<uint8_t> &data,size_t offset,T v,bool big) {
  uint8_t bytes[sizeof(T)];
  std::memcpy(bytes,&v,sizeof(T));
  const uint16_t one=1;
  if (big != (*reinterpret_cast<const uint8_t *>(&one)==0)) std::reverse(bytes,bytes+sizeof(T));
  std::memcpy(data.data()+offset,bytes,sizeof(T));
}
sensor_msgs::msg::PointCloud2::SharedPtr cloud(std::vector<Return> points,
    bool byte_intensity=false,bool with_time=true,bool big=false,size_t width=0,size_t padding=0) {
  auto msg=std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp.sec=1000;
  msg->width=width ? width : points.size();
  msg->height=msg->width ? points.size()/msg->width : 0;
  msg->point_step=26;msg->row_step=msg->width*msg->point_step+padding;msg->is_bigendian=big;
  using F=sensor_msgs::msg::PointField;
  const auto field=[&](const char *name,uint32_t offset,uint8_t type){
    F f;f.name=name;f.offset=offset;f.datatype=type;f.count=1;msg->fields.push_back(f);
  };
  field("x",0,F::FLOAT32);field("y",4,F::FLOAT32);field("z",8,F::FLOAT32);
  field("intensity",12,byte_intensity ? F::UINT8 : F::FLOAT32);field("ring",16,F::UINT16);
  if (with_time) field("timestamp",18,F::FLOAT64);
  msg->data.resize(msg->row_step*msg->height);
  for (size_t i=0;i<points.size();++i) {
    const auto &p=points[i];size_t off=(i/msg->width)*msg->row_step+(i%msg->width)*msg->point_step;
    write(msg->data,off,p.x,big);write(msg->data,off+4,p.y,big);write(msg->data,off+8,p.z,big);
    if (byte_intensity) write(msg->data,off+12,uint8_t(p.intensity),big);
    else write(msg->data,off+12,p.intensity,big);
    write(msg->data,off+16,p.ring,big);write(msg->data,off+18,p.time,big);
  }
  return msg;
}
Preprocess preprocessor() {
  Preprocess p;p.lidar_type=ROBOSENSE_AIRY;p.N_SCANS=96;p.blind=.5;p.point_filter_num=1;
  return p;
}
PointCloudXYZI::Ptr process(Preprocess &p,const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
  PointCloudXYZI::Ptr result(new PointCloudXYZI);p.process(msg,result);return result;
}
}

TEST(RoboSenseAiry, All96ChannelsIncludingUpwardRaysSurvive) {
  std::vector<Return> points;
  for (int i=0;i<96;++i) { Return p;p.ring=i;p.z=4;p.time=1000+i*.001;points.push_back(p); }
  auto pre=preprocessor();auto output=process(pre,cloud(points));
  ASSERT_EQ(output->size(),96u);EXPECT_TRUE(pre.given_offset_time);
  EXPECT_NEAR(output->back().curvature,95,1e-3);
  EXPECT_FLOAT_EQ(output->back().z,4);
}
TEST(RoboSenseAiry, EndStampedHeaderAndUnsortedPointsUseActualScanStart) {
  Return a,b,c;a.time=1000.09;b.time=1000;c.time=1000.04;
  auto msg=cloud({a,b,c});msg->header.stamp.nanosec=100000000;
  auto pre=preprocessor();auto output=process(pre,msg);
  ASSERT_EQ(output->size(),3u);EXPECT_DOUBLE_EQ(pre.scan_start_time,1000.);
  EXPECT_NEAR(output->front().curvature,0,1e-4);EXPECT_NEAR(output->back().curvature,90,1e-3);
}
TEST(RoboSenseAiry, TimeOriginIsIndependentOfDecimationAndBlindFiltering) {
  Return a,b,c;a.x=a.y=a.z=0;b.time=1000.01;c.time=1000.02;
  auto pre=preprocessor();pre.point_filter_num=2;auto output=process(pre,cloud({a,b,c}));
  ASSERT_EQ(output->size(),1u);EXPECT_NEAR(output->front().curvature,20,1e-3);
  EXPECT_DOUBLE_EQ(pre.scan_start_time,1000.);
}
TEST(RoboSenseAiry, OrganizedPaddedCloudWithByteIntensityAndBigEndianIsReadCorrectly) {
  Return a,b,c,d;b.time=1000.01;c.time=1000.02;d.time=1000.03;d.ring=95;d.intensity=200;
  auto pre=preprocessor();auto output=process(pre,cloud({a,b,c,d},true,true,true,2,7));
  ASSERT_EQ(output->size(),4u);EXPECT_FLOAT_EQ(output->back().intensity,200);
  EXPECT_NEAR(output->back().curvature,30,1e-3);
}
TEST(RoboSenseAiry, InvalidRingsCoordinatesAndTimestampsAreDiscarded) {
  Return a,b,c,d,e,f;b.ring=96;c.x=std::numeric_limits<float>::quiet_NaN();
  d.time=std::numeric_limits<double>::quiet_NaN();e.time=1005;f.time=1000.05;
  auto pre=preprocessor();auto output=process(pre,cloud({a,b,c,d,e,f}));
  ASSERT_EQ(output->size(),2u);EXPECT_NEAR(output->back().curvature,50,1e-3);
}
TEST(RoboSenseAiry, MissingOrAllZeroTimeUsesZeroOffsetsForLidarOnly) {
  auto pre=preprocessor();Return a,b;b.ring=95;
  auto output=process(pre,cloud({a,b},false,false));ASSERT_EQ(output->size(),2u);
  EXPECT_FALSE(pre.given_offset_time);EXPECT_FLOAT_EQ(output->back().curvature,0);
  a.time=b.time=0;output=process(pre,cloud({a,b}));ASSERT_EQ(output->size(),2u);
  EXPECT_FALSE(pre.given_offset_time);EXPECT_DOUBLE_EQ(pre.scan_start_time,1000.);
}
TEST(RoboSenseAiry, RelativeTimestampInMicrosecondsConvertsToMilliseconds) {
  Return a,b;a.time=0;b.time=90000;
  auto pre=preprocessor();pre.timestamp_unit=2;pre.timestamp_is_absolute=false;
  auto output=process(pre,cloud({a,b}));ASSERT_EQ(output->size(),2u);
  EXPECT_NEAR(output->back().curvature,90,1e-3);EXPECT_DOUBLE_EQ(pre.scan_start_time,1000.);
}
TEST(RoboSenseAiry, RelativeTimeAliasUsesHeaderAsOrigin) {
  Return a,b;a.time=0;b.time=.08;auto msg=cloud({a,b});msg->fields.back().name="time";
  auto pre=preprocessor();auto output=process(pre,msg);ASSERT_EQ(output->size(),2u);
  EXPECT_NEAR(output->back().curvature,80,1e-3);EXPECT_DOUBLE_EQ(pre.scan_start_time,1000.);
}
TEST(RoboSenseAiry, ClockMismatchAndMalformedFieldsFailWithoutStaleCloud) {
  auto pre=preprocessor();Return a,b;b.time=1000.04;
  ASSERT_EQ(process(pre,cloud({a,b}))->size(),2u);
  a.time=std::numeric_limits<double>::quiet_NaN();b.time=a.time;EXPECT_TRUE(process(pre,cloud({a,b}))->empty());
  auto msg=cloud({a,b});msg->data.resize(1);EXPECT_TRUE(process(pre,msg)->empty());
  msg=cloud({a,b});msg->fields[0].offset=100;EXPECT_TRUE(process(pre,msg)->empty());
  msg=cloud({a,b});msg->fields.erase(msg->fields.begin()+4);EXPECT_EQ(process(pre,msg)->size(),2u);EXPECT_FALSE(pre.given_offset_time);
  msg=cloud({a,b});msg->width=msg->height=0;EXPECT_TRUE(process(pre,msg)->empty());
}
TEST(RoboSenseAiry, Recorded96ChannelCloud) {
  const char *path=std::getenv("VOXELMAP_AIRY_TEST_CDR");
  if (!path) GTEST_SKIP() << "Set VOXELMAP_AIRY_TEST_CDR to a recorded PointCloud2 CDR";
  std::ifstream stream(path,std::ios::binary);ASSERT_TRUE(stream.good());
  std::vector<char> data((std::istreambuf_iterator<char>(stream)),std::istreambuf_iterator<char>());
  rclcpp::SerializedMessage serialized(data.size());auto &raw=serialized.get_rcl_serialized_message();
  std::memcpy(raw.buffer,data.data(),data.size());raw.buffer_length=data.size();
  auto msg=std::make_shared<sensor_msgs::msg::PointCloud2>();
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;serializer.deserialize_message(&serialized,msg.get());
  auto pre=preprocessor();auto output=process(pre,msg);
  ASSERT_GT(output->size(),1000u);EXPECT_TRUE(pre.given_offset_time);
  EXPECT_GT(output->back().curvature,90);EXPECT_LT(output->back().curvature,110);
  EXPECT_TRUE(std::is_sorted(output->begin(),output->end(),[](const PointType &a,const PointType &b){return a.curvature<b.curvature;}));
  for (const auto &p:*output) EXPECT_TRUE(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&std::isfinite(p.curvature));
}

TEST(RoboSenseAiry, EpochSecondsAreSubtractedBeforeConvertingToFloatOffsets) {
  Return a,b;a.time=1700000000.;b.time=a.time+.098;
  auto msg=cloud({a,b});msg->header.stamp.sec=1700000000;
  auto pre=preprocessor();auto output=process(pre,msg);
  ASSERT_EQ(output->size(),2u);EXPECT_TRUE(pre.given_offset_time);
  EXPECT_NEAR(output->back().curvature,98.,.001);
}

TEST(RoboSenseAiry, OrganizedDecimationKeepsEveryChannel) {
  std::vector<Return> points;
  for(int col=0;col<6;++col) for(int ring=0;ring<96;++ring) {
    Return point;point.ring=ring;point.intensity=ring;
    point.time=1000.+col*.01+ring*.000001;points.push_back(point);
  }
  auto pre=preprocessor();pre.point_filter_num=3;
  auto result=process(pre,cloud(points,false,true,false,96));
  ASSERT_EQ(result->size(),192u);
  std::vector<int> counts(96,0);
  for(const auto &point:*result) ++counts.at(static_cast<size_t>(point.intensity));
  for(int count:counts) EXPECT_EQ(count,2);
  EXPECT_NEAR(pre.scan_end_time,1000.050095,1e-7);
  EXPECT_LT(result->back().curvature,40);
}
TEST(RoboSenseAiry, UnorganizedDecimationAlsoKeepsEveryChannel) {
  std::vector<Return> points;
  for(int col=0;col<6;++col) for(int ring=0;ring<96;++ring) {
    Return point;point.ring=ring;point.intensity=ring;
    point.time=1000.+col*.01;points.push_back(point);
  }
  auto pre=preprocessor();pre.point_filter_num=3;
  auto result=process(pre,cloud(points));ASSERT_EQ(result->size(),192u);
  std::vector<int> counts(96,0);
  for(const auto &point:*result) ++counts.at(static_cast<size_t>(point.intensity));
  for(int count:counts) EXPECT_EQ(count,2);
}
TEST(RoboSenseAiry, XyziFallbackAnd150MeterRangeMatchReference) {
  Return a,b,c;b.x=151;c.time=1000.09;
  auto msg=cloud({a,b,c});msg->fields.erase(msg->fields.begin()+4);
  auto pre=preprocessor();auto result=process(pre,msg);
  ASSERT_EQ(result->size(),2u);EXPECT_FALSE(pre.given_offset_time);
  EXPECT_DOUBLE_EQ(pre.scan_start_time,1000.);
  EXPECT_FLOAT_EQ(result->back().curvature,0);
}
TEST(RoboSenseAiry, ValidPointClockIsAuthoritativeWhenHeaderClockDiffers) {
  Return a,b;a.time=500;b.time=500.1;
  auto pre=preprocessor();auto result=process(pre,cloud({a,b}));
  ASSERT_EQ(result->size(),2u);EXPECT_DOUBLE_EQ(pre.scan_start_time,500.);
  EXPECT_NEAR(pre.scan_end_time,500.1,1e-9);
}
