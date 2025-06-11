/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

 #ifdef _MSC_VER
 #ifndef _USE_MATH_DEFINES
 #define _USE_MATH_DEFINES
 #endif
 #endif
 
 #include "src/CYdLidar.h"
 #include <math.h>
 #include <chrono>
 #include <iostream>
 #include <memory>
 #include "sensor_msgs/msg/point_cloud.hpp"
 #include "rclcpp/clock.hpp"
 #include "rclcpp/rclcpp.hpp"
 #include "rclcpp/time_source.hpp"
 #include "sensor_msgs/msg/laser_scan.hpp"
 #include "std_srvs/srv/empty.hpp"
 #include <vector>
 #include <iostream>
 #include <string>
 #include <signal.h>
 
 #define ROS2Verision "1.0.1"
 
 float normalizeAngle(float angle) {
   // 각도를 -180 ~ 180 범위로 정규화
   while (angle > 180.0f) {
     angle -= 360.0f;
   }
   while (angle < -180.0f) {
     angle += 360.0f;
   }
   return angle;
 }
 
 
 int main(int argc, char *argv[]) {
   rclcpp::init(argc, argv);
 
   auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");
 
   RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());
 
   CYdLidar laser;
   std::string str_optvalue = "/dev/ydlidar";
   node->declare_parameter("port", str_optvalue);
   node->get_parameter("port", str_optvalue);
   ///lidar port
   laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());
 
   ///ignore array
   str_optvalue = "";
   node->declare_parameter("ignore_array", str_optvalue);
   node->get_parameter("ignore_array", str_optvalue);
   laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());
 
   std::string frame_id = "laser_frame";
   node->declare_parameter("frame_id", frame_id);
   node->get_parameter("frame_id", frame_id);
 
   //////////////////////int property/////////////////
   /// lidar baudrate
   int optval = 230400;
   node->declare_parameter("baudrate", optval);
   node->get_parameter("baudrate", optval);
   laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
   /// tof lidar
   optval = TYPE_TRIANGLE;
   node->declare_parameter("lidar_type", optval);
   node->get_parameter("lidar_type", optval);
   laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
   /// device type
   optval = YDLIDAR_TYPE_SERIAL;
   node->declare_parameter("device_type", optval);
   node->get_parameter("device_type", optval);
   laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
   /// sample rate
   optval = 9;
   node->declare_parameter("sample_rate", optval);
   node->get_parameter("sample_rate", optval);
   laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
   /// abnormal count
   optval = 4;
   node->declare_parameter("abnormal_check_count", optval);
   node->get_parameter("abnormal_check_count", optval);
   laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
 
   /// Intenstiy bit count
   optval = 0;
   node->declare_parameter("intensity_bit", optval);
   node->get_parameter("intensity_bit", optval);
   laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));
      
   //////////////////////bool property/////////////////
   /// fixed angle resolution
   bool b_optvalue = false;
   node->declare_parameter("fixed_resolution", b_optvalue);
   node->get_parameter("fixed_resolution", b_optvalue);
   laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
   /// rotate 180
   b_optvalue = true;
   node->declare_parameter("reversion", b_optvalue);
   node->get_parameter("reversion", b_optvalue);
   laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
   /// Counterclockwise
   b_optvalue = true;
   node->declare_parameter("inverted", b_optvalue);
   node->get_parameter("inverted", b_optvalue);
   laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
   b_optvalue = true;
   node->declare_parameter("auto_reconnect", b_optvalue);
   node->get_parameter("auto_reconnect", b_optvalue);
   laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
   /// one-way communication
   b_optvalue = false;
   node->declare_parameter("isSingleChannel", b_optvalue);
   node->get_parameter("isSingleChannel", b_optvalue);
   laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
   /// intensity
   b_optvalue = false;
   node->declare_parameter("intensity", b_optvalue);
   node->get_parameter("intensity", b_optvalue);
   laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
   /// Motor DTR
   b_optvalue = false;
   node->declare_parameter("support_motor_dtr", b_optvalue);
   node->get_parameter("support_motor_dtr", b_optvalue);
   laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
   //是否启用调试
   b_optvalue = false;
   node->declare_parameter("debug", b_optvalue);
   node->get_parameter("debug", b_optvalue);
   laser.setEnableDebug(b_optvalue);
 
   //////////////////////float property/////////////////
   /// unit: °
   float f_optvalue = 180.0f;
   node->declare_parameter("angle_max", f_optvalue);
   node->get_parameter("angle_max", f_optvalue);
   laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
   f_optvalue = -180.0f;
   node->declare_parameter("angle_min", f_optvalue);
   node->get_parameter("angle_min", f_optvalue);
   laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
   /// unit: m
   f_optvalue = 64.f;
   node->declare_parameter("range_max", f_optvalue);
   node->get_parameter("range_max", f_optvalue);
   laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
   f_optvalue = 0.1f;
   node->declare_parameter("range_min", f_optvalue);
   node->get_parameter("range_min", f_optvalue);
   laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
   /// unit: Hz
   f_optvalue = 10.f;
   node->declare_parameter("frequency", f_optvalue);
   node->get_parameter("frequency", f_optvalue);
   laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));
 
   bool invalid_range_is_inf = false;
   node->declare_parameter("invalid_range_is_inf", invalid_range_is_inf);
   node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);
 
 
   bool ret = laser.initialize();
   if (ret) 
   {
     //设置GS工作模式（非GS雷达请无视该代码）
     int i_v = 0;
     node->declare_parameter("m1_mode", i_v);
     node->get_parameter("m1_mode", i_v);
     laser.setWorkMode(i_v, 0x01);
     i_v = 0;
     node->declare_parameter("m2_mode", i_v);
     node->get_parameter("m2_mode", i_v);
     laser.setWorkMode(i_v, 0x02);
     i_v = 1;
     node->declare_parameter("m3_mode", i_v);
     node->get_parameter("m3_mode", i_v);
     laser.setWorkMode(i_v, 0x04);
     //启动扫描
     ret = laser.turnOn();
   } 
   else 
   {
     RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
   }
   
   auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
   auto pc_pub = node->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud", rclcpp::SensorDataQoS());
   
   auto stop_scan_service =
     [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
   const std::shared_ptr<std_srvs::srv::Empty::Request> req,
   std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
   {
     return laser.turnOff();
   };
 
   auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan",stop_scan_service);
 
   auto start_scan_service =
     [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
   const std::shared_ptr<std_srvs::srv::Empty::Request> req,
   std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
   {
     return laser.turnOn();
   };
 
   auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan",start_scan_service);
 
   rclcpp::WallRate loop_rate(20);
 
   while (ret && rclcpp::ok()) {
 
     LaserScan scan;//
 
     // 이 부분은 main 함수 내 while 루프 안의 스캔 처리 부분을 수정
 
     if (laser.doProcessSimple(scan)) {
       auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
       auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud>();
       
       // 기존 헤더 및 설정 코드 유지
       scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
       scan_msg->header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
       scan_msg->header.frame_id = frame_id;
       pc_msg->header = scan_msg->header;
       
       // 로그 추가 - 디버깅용 (선택사항)
       RCLCPP_DEBUG(node->get_logger(), "Angle range: [%.2f, %.2f]", 
                   scan.config.min_angle * 180.0f / M_PI, 
                   scan.config.max_angle * 180.0f / M_PI);
       
       // 각도 범위 확인 및 수정 (타입 불일치 문제 수정)
       // std::max/min 대신 조건문 사용
       if (scan.config.min_angle < -M_PI) {
         scan.config.min_angle = static_cast<float>(-M_PI);  // 타입 캐스팅 추가
       }
       if (scan.config.max_angle > M_PI) {
         scan.config.max_angle = static_cast<float>(M_PI);   // 타입 캐스팅 추가
       }
       
       scan_msg->angle_min = scan.config.min_angle;
       scan_msg->angle_max = scan.config.max_angle;
       scan_msg->angle_increment = scan.config.angle_increment;
       scan_msg->scan_time = scan.config.scan_time;
       scan_msg->time_increment = scan.config.time_increment;
       scan_msg->range_min = scan.config.min_range;
       scan_msg->range_max = scan.config.max_range;
       
       int size = (scan.config.max_angle - scan.config.min_angle) / scan.config.angle_increment + 1;
       scan_msg->ranges.resize(size);
       scan_msg->intensities.resize(size);
       
       // 포인트 클라우드 채널 설정
       pc_msg->channels.resize(2);
       int idx_intensity = 0;
       pc_msg->channels[idx_intensity].name = "intensities";
       int idx_timestamp = 1;
       pc_msg->channels[idx_timestamp].name = "stamps";
       
       // 데이터 포인트 처리 - 각도 정규화 적용
       for(size_t i=0; i < scan.points.size(); i++) {
         // 각도 정규화 (라디안에서 도로 변환, 정규화, 다시 라디안으로 변환)
         float angle_degrees = scan.points[i].angle * 180.0f / static_cast<float>(M_PI);
         float normalized_angle_degrees = normalizeAngle(angle_degrees);
         float normalized_angle_radians = normalized_angle_degrees * static_cast<float>(M_PI) / 180.0f;
         
         // 정규화된 각도가 -180 ~ 180도 범위 내에 있는지 확인
         if (normalized_angle_degrees >= -180.0f && normalized_angle_degrees <= 180.0f) {
           // 원래 각도와 정규화된 각도가 많이 다르면 로그 출력 (디버깅용)
           if (fabs(normalized_angle_degrees - angle_degrees) > 1.0f) {
             RCLCPP_DEBUG(node->get_logger(), 
                         "Normalized angle: %.2f -> %.2f degrees", 
                         angle_degrees, normalized_angle_degrees);
           }
           
           // 정규화된 각도로 인덱스 계산
           int index = std::ceil((normalized_angle_radians - scan.config.min_angle) / scan.config.angle_increment);
           if (index >= 0 && index < size) {
             if (scan.points[i].range >= scan.config.min_range) {
               scan_msg->ranges[index] = scan.points[i].range;
               scan_msg->intensities[index] = scan.points[i].intensity;
             }
           }
           
           // 포인트 클라우드 데이터 추가 (정규화된 각도 사용)
           if (scan.points[i].range >= scan.config.min_range &&
               scan.points[i].range <= scan.config.max_range) {
             geometry_msgs::msg::Point32 point;
             point.x = scan.points[i].range * cos(normalized_angle_radians);
             point.y = scan.points[i].range * sin(normalized_angle_radians);
             point.z = 0.0;
             pc_msg->points.push_back(point);
             pc_msg->channels[idx_intensity].values.push_back(scan.points[i].intensity);
             pc_msg->channels[idx_timestamp].values.push_back(i * scan.config.time_increment);
           }
         } else {
           // 정규화된 후에도 범위를 벗어나는 경우는 일반적으로 발생하지 않아야 함
           RCLCPP_WARN(node->get_logger(), 
                     "Angle still out of range after normalization: %.2f degrees", 
                     normalized_angle_degrees);
         }
       }
       
       // 메시지 발행
       laser_pub->publish(*scan_msg);
       pc_pub->publish(*pc_msg);
     } else {
       RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
     }
     
 
     if(!rclcpp::ok()) {
       break;
     }
     rclcpp::spin_some(node);
     loop_rate.sleep();
   }
 
 
   RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
   laser.turnOff();
   laser.disconnecting();
   rclcpp::shutdown();
 
   return 0;
 }
 