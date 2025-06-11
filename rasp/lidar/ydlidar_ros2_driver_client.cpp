/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node Client
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include <vector>
#include <algorithm>
#include <map>

#define RAD2DEG(x) ((x)*180./M_PI)

// 0 ~ 360 범위로 정규화하는 함수
float normalizeAngleTo360(float angle) {
  // 먼저 -180~180으로 정규화
  while (angle > 180.0f) {
    angle -= 360.0f;
  }
  while (angle < -180.0f) {
    angle += 360.0f;
  }
  // 음수 각도를 360도 범위로 변환
  if (angle < 0) {
    angle += 360.0f;
  }
  return angle;
}

// 유효한 거리 값인지 확인하는 함수
bool isValidDistance(float distance) {
  // 1008 및 1020 등의 특수 값과 0 값을 필터링
  if (distance <= 0.01f || (distance > 900.0f && distance < 1100.0f)) {
    return false;
  }
  return true;
}

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment;
  printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  
  // 각도 범위 출력 (0~360도 범위로 표시)
  float angle_min_deg = normalizeAngleTo360(RAD2DEG(scan->angle_min));
  float angle_max_deg = normalizeAngleTo360(RAD2DEG(scan->angle_max));
  printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", angle_min_deg, angle_max_deg);

  // 각도별 최적 거리 저장 맵 (유효한 값만 저장)
  std::map<float, float> angle_distance_map;
  
  // 각도별 최적 거리 계산 (중복 제거, 유효 값만 선택)
  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    float degree_360 = normalizeAngleTo360(degree);
    float distance = scan->ranges[i];
    
    // 유효한 거리 값만 저장
    if (isValidDistance(distance)) {
      // 이미 해당 각도에 대한 값이 있는지 확인
      auto it = angle_distance_map.find(degree_360);
      if (it == angle_distance_map.end()) {
        // 해당 각도에 대한 첫 번째 유효 값
        angle_distance_map[degree_360] = distance;
      } else {
        // 이미 값이 있으면 더 작은 값을 선택 (더 가까운 물체)
        if (distance < it->second) {
          it->second = distance;
        }
      }
    }
  }
  
  // 정렬된 각도 순서로 유효 데이터만 출력
  std::vector<std::pair<float, float>> sorted_data;
  for (const auto& pair : angle_distance_map) {
    sorted_data.push_back(pair);
  }
  
  std::sort(sorted_data.begin(), sorted_data.end());
  for (const auto& pair : sorted_data) {
    printf("[YDLIDAR INFO]: angle-distance : [%f, %f]\n", pair.first, pair.second);
  }
  
  // 필터링 된 포인트 수 출력 (디버깅용)
  printf("[YDLIDAR INFO]: Filtered points: %d/%d (%.1f%%)\n", 
        (int)sorted_data.size(), count,
        (float)sorted_data.size() / count * 100.0f);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_client");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}