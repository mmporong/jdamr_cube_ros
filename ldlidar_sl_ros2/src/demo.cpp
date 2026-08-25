/**
 * @file main.cpp
 * @author LDRobot (contact@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-10
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "ros2_api.h"
#include "ldlidar_driver.h"
#include "scan_utils.hpp"

uint64_t GetTimestamp(void);

void  ToLaserscanMessagePublish(ldlidar::Points2D& src,  double lidar_spin_freq, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub);

void  ToSensorPointCloudMessagePublish(ldlidar::Points2D& src, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr& lidarpub);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // create a ROS2 Node
  auto node = std::make_shared<rclcpp::Node>("ldlidar_published"); 

  std::string product_name;
	std::string laser_scan_topic_name;
  std::string point_cloud_2d_topic_name;
	std::string port_name;
  LaserScanSetting setting;
	setting.frame_id = "base_laser";
  setting.laser_scan_dir = true;
  setting.enable_angle_crop_func = false;
  setting.angle_crop_min = 0.0;
  setting.angle_crop_max = 0.0;
  int serial_baudrate = 0;
  ldlidar::LDType lidartypename = ldlidar::LDType::NO_VER;

  // declare ros2 param
  node->declare_parameter<std::string>("product_name", product_name);
  node->declare_parameter<std::string>("laser_scan_topic_name", laser_scan_topic_name);
  node->declare_parameter<std::string>("point_cloud_2d_topic_name", point_cloud_2d_topic_name);
  node->declare_parameter<std::string>("frame_id", setting.frame_id);
  node->declare_parameter<std::string>("port_name", port_name);
  node->declare_parameter<int>("serial_baudrate", serial_baudrate);
  node->declare_parameter<bool>("laser_scan_dir", setting.laser_scan_dir);
  node->declare_parameter<bool>("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->declare_parameter<double>("angle_crop_min", setting.angle_crop_min);
  node->declare_parameter<double>("angle_crop_max", setting.angle_crop_max);

  // get ros2 param
  node->get_parameter("product_name", product_name);
  node->get_parameter("laser_scan_topic_name", laser_scan_topic_name);
  node->get_parameter("point_cloud_2d_topic_name", point_cloud_2d_topic_name);
  node->get_parameter("frame_id", setting.frame_id);
  node->get_parameter("port_name", port_name);
  node->get_parameter("serial_baudrate", serial_baudrate);
  node->get_parameter("laser_scan_dir", setting.laser_scan_dir);
  node->get_parameter("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->get_parameter("angle_crop_min", setting.angle_crop_min);
  node->get_parameter("angle_crop_max", setting.angle_crop_max);

  ldlidar::LDLidarDriver* lidar_drv = new ldlidar::LDLidarDriver();

  RCLCPP_INFO(node->get_logger(), "LDLiDAR SDK Pack Version is:%s", lidar_drv->GetLidarSdkVersionNumber().c_str());
  RCLCPP_INFO(node->get_logger(), "ROS2 param input:");
  RCLCPP_INFO(node->get_logger(), "<laser_scan_topic_name>: %s", laser_scan_topic_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<point_cloud_2d_topic_name>: %s", point_cloud_2d_topic_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<frame_id>: %s", setting.frame_id.c_str());
  RCLCPP_INFO(node->get_logger(), "<port_name>: %s ", port_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<serial_baudrate>: %d ", serial_baudrate);
  RCLCPP_INFO(node->get_logger(), "<laser_scan_dir>: %s", (setting.laser_scan_dir?"Counterclockwise":"Clockwise"));
  RCLCPP_INFO(node->get_logger(), "<enable_angle_crop_func>: %s", (setting.enable_angle_crop_func?"true":"false"));
  RCLCPP_INFO(node->get_logger(), "<angle_crop_min>: %f", setting.angle_crop_min);
  RCLCPP_INFO(node->get_logger(), "<angle_crop_max>: %f", setting.angle_crop_max);

  if (port_name.empty()) {
    RCLCPP_ERROR(node->get_logger(), "fail, port_name is empty!");
    exit(EXIT_FAILURE);
  }

  lidar_drv->RegisterGetTimestampFunctional(std::bind(&GetTimestamp)); 

  lidar_drv->EnableFilterAlgorithnmProcess(true);
  
  if(!strcmp(product_name.c_str(), "LDLiDAR_LD14")) {
    lidartypename = ldlidar::LDType::LD_14;
  } else if(!strcmp(product_name.c_str(), "LDLiDAR_LD14P")) {
    lidartypename = ldlidar::LDType::LD_14P_4000HZ;  // the measurement frequency of lidar is 4kHz.
  } else {
    RCLCPP_ERROR(node->get_logger(),"Error, input param <product_name> is fail!!");
    exit(EXIT_FAILURE);
  }

  if (lidar_drv->Start(lidartypename, port_name, serial_baudrate)) {
    RCLCPP_INFO(node->get_logger(), "ldlidar node start is success");
  } else {
    RCLCPP_ERROR(node->get_logger(), "ldlidar node start is fail");
    exit(EXIT_FAILURE);
  }

  if (lidar_drv->WaitLidarCommConnect(3500)) {
    RCLCPP_INFO(node->get_logger(), "ldlidar communication is normal.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "ldlidar communication is abnormal.");
    exit(EXIT_FAILURE);
  }

  // create ldlidar data topic and publisher
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_laserscan = 
      node->create_publisher<sensor_msgs::msg::LaserScan>(laser_scan_topic_name, 10);
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr lidar_pub_pointcloud = 
      node->create_publisher<sensor_msgs::msg::PointCloud>(point_cloud_2d_topic_name, 10);

  // Poll well above the 6 Hz spin rate so a completed frame is collected
  // promptly instead of aliasing with the LiDAR period and skipping scans.
  rclcpp::WallRate r(100);

  ldlidar::Points2D laser_scan_points;

  RCLCPP_INFO(node->get_logger(), "start normal, pub lidar data");

  while (rclcpp::ok() && ldlidar::LDLidarDriver::IsOk()) {
  
    switch (lidar_drv->GetLaserScanData(laser_scan_points, 1500)){
      case ldlidar::LidarStatus::NORMAL: {
        double lidar_scan_freq = 0;
        lidar_drv->GetLidarScanFreq(lidar_scan_freq);
        ToLaserscanMessagePublish(laser_scan_points, lidar_scan_freq, setting, node, lidar_pub_laserscan);
        ToSensorPointCloudMessagePublish(laser_scan_points, setting, node, lidar_pub_pointcloud);
        break;
      }
      case ldlidar::LidarStatus::DATA_TIME_OUT: {
        RCLCPP_ERROR(node->get_logger(), "ldlidar point cloud data publish time out, please check your lidar device.");
        lidar_drv->Stop();
        break;
      }
      case ldlidar::LidarStatus::DATA_WAIT: {
        break;
      }
      default:
        break;
    }

    r.sleep();
  }

  lidar_drv->Stop();

  delete lidar_drv;
  lidar_drv = nullptr;

  RCLCPP_INFO(node->get_logger(), "this node of ldlidar_published is end");
  rclcpp::shutdown();

  return 0;
}

uint64_t GetTimestamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

void  ToLaserscanMessagePublish(ldlidar::Points2D& src,  double lidar_spin_freq, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub) {
  if (src.empty()) {
    return;
  }

  const auto publish_time = node->now();
  int beam_size = static_cast<int>(src.size());
  const auto timing = ldlidar_ros2::ComputeScanTiming(
      src.front().stamp, src.back().stamp, src.size(), lidar_spin_freq,
      static_cast<uint64_t>(publish_time.nanoseconds()));
  const double first_angle = ldlidar_ros2::PublishedAngle(
      ANGLE_TO_RADIAN(src.front().angle), setting.laser_scan_dir);
  const auto geometry = ldlidar_ros2::ComputeScanGeometry(
      src.size(), setting.laser_scan_dir, first_angle);

  // Calculate the number of scanning points
  if (lidar_spin_freq > 0) {
    sensor_msgs::msg::LaserScan output;
    output.header.stamp = rclcpp::Time(
        static_cast<int64_t>(timing.start_stamp_ns), RCL_SYSTEM_TIME);
    output.header.frame_id = setting.frame_id;
    output.angle_min = static_cast<float>(geometry.angle_min);
    output.angle_max = static_cast<float>(geometry.angle_max);
    output.range_min = 0.02;
    output.range_max = 12;
    output.angle_increment = static_cast<float>(geometry.angle_increment);
    output.time_increment = static_cast<float>(timing.time_increment);
    output.scan_time = static_cast<float>(timing.scan_time);
    // First fill all the data with Nan
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    for (auto point : src) {
      float range = point.distance / 1000.f;  // distance unit transform to meters
      float intensity = point.intensity;      // laser receive intensity 
      float dir_angle = point.angle;

      if ((point.distance == 0) && (point.intensity == 0)) { // filter is handled to  0, Nan will be assigned variable.
        range = std::numeric_limits<float>::quiet_NaN(); 
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (setting.enable_angle_crop_func) { // Angle crop setting, Mask data within the set angle range
        if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      const float angle = static_cast<float>(ldlidar_ros2::PublishedAngle(
          ANGLE_TO_RADIAN(dir_angle), setting.laser_scan_dir));
      int index = ldlidar_ros2::AngleToIndex(angle, geometry, src.size());
      if (index < beam_size) {
        if (index < 0) {
          RCLCPP_ERROR(node->get_logger(), "error index: %d, beam_size: %d, angle: %f, output.angle_min: %f, output.angle_increment: %f", 
            index, beam_size, angle, output.angle_min, output.angle_increment);
        }

        // If the current content is Nan, it is assigned directly
        if (std::isnan(output.ranges[index])) {
          output.ranges[index] = range;
        } else { // Otherwise, only when the distance is less than the current
                // value, it can be re assigned
          if (range < output.ranges[index]) {
            output.ranges[index] = range;
          }
        }
        output.intensities[index] = intensity;
      }
    }
    lidarpub->publish(output);
  } 
}

void  ToSensorPointCloudMessagePublish(ldlidar::Points2D& src, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr& lidarpub) {
  if (src.empty()) {
    return;
  }

  ldlidar::Points2D dst = src;
  const auto publish_time = node->now();
  const auto timing = ldlidar_ros2::ComputeScanTiming(
      src.front().stamp, src.back().stamp, src.size(), 0.0,
      static_cast<uint64_t>(publish_time.nanoseconds()));

  if (setting.laser_scan_dir) {
    for (auto&point : dst) {
      point.angle = 360.f - point.angle;
      if (point.angle < 0) {
        point.angle += 360.f;
      }
    }
  } 

  int frame_points_num = static_cast<int>(dst.size());

  sensor_msgs::msg::PointCloud output;

  output.header.stamp = rclcpp::Time(
      static_cast<int64_t>(timing.start_stamp_ns), RCL_SYSTEM_TIME);
  output.header.frame_id = setting.frame_id;

  sensor_msgs::msg::ChannelFloat32 defaultchannelval[3];

  defaultchannelval[0].name = std::string("intensity");
  defaultchannelval[0].values.assign(frame_points_num, std::numeric_limits<float>::quiet_NaN());
  // output.channels.assign(1, defaultchannelval);
  output.channels.push_back(defaultchannelval[0]);

  defaultchannelval[1].name = std::string("timeincrement");
  defaultchannelval[1].values.assign(1, static_cast<float>(timing.time_increment));
  output.channels.push_back(defaultchannelval[1]);
  
  defaultchannelval[2].name = std::string("scantime");
  defaultchannelval[2].values.assign(1, static_cast<float>(timing.scan_time));
  output.channels.push_back(defaultchannelval[2]);

  geometry_msgs::msg::Point32 points_xyz_defaultval;
  points_xyz_defaultval.x = std::numeric_limits<float>::quiet_NaN();
  points_xyz_defaultval.y = std::numeric_limits<float>::quiet_NaN();
  points_xyz_defaultval.z = std::numeric_limits<float>::quiet_NaN();
  output.points.assign(frame_points_num, points_xyz_defaultval);

  for (int i = 0; i < frame_points_num; i++) {
    float range = dst[i].distance / 1000.f;  // distance unit transform to meters
    float intensity = dst[i].intensity;      // laser receive intensity 
    float dir_angle = ANGLE_TO_RADIAN(dst[i].angle);
    //  极坐标系转换为笛卡尔直角坐标系
    output.points[i].x = range * cos(dir_angle);
    output.points[i].y = range * sin(dir_angle);
    output.points[i].z = 0.0;
    output.channels[0].values[i] = intensity;
  }
  lidarpub->publish(output);
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
