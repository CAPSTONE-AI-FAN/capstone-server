#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fan Car LiDAR System - ROS2 to MQTT Bridge
==========================================
ROS2 LiDAR 데이터를 MQTT로 브리지하는 시스템

기능:
- ROS2 LaserScan → MQTT 브리지
- 서버 호환 토픽 및 메시지 형식
- 실시간 스캔 데이터 변환 및 발행
- 시나리오 기반 더미 데이터 생성
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import json
import time
import math
import signal
import sys
import threading
import paho.mqtt.client as mqtt

class LidarMqttBridge(Node):
    """LiDAR 데이터를 MQTT로 브리지하는 노드"""
    
    def __init__(self, mqtt_broker='192.168.0.8', mqtt_port=1883):
        super().__init__('lidar_mqtt_bridge')
        
        self.get_logger().info("🌉 LiDAR-MQTT 브리지 초기화 중...")
        
        # MQTT 클라이언트 설정
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, 
                                     client_id="ubuntu_pi_lidar_module")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_connected = False
        
        # MQTT 연결 시도
        self.connect_mqtt()
        
        # ROS2 QoS 설정
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # LaserScan 구독자 생성
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos
        )
        
        # 통계 정보
        self.scan_count = 0
        self.last_publish_time = time.time()
        
        self.get_logger().info(f"📡 MQTT 브로커: {mqtt_broker}:{mqtt_port}")
        self.get_logger().info("🔔 /scan 토픽 구독 시작")
        
    def connect_mqtt(self):
        """MQTT 브로커에 연결"""
        try:
            self.get_logger().info(f"📡 MQTT 브로커 연결 시도: {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"❌ MQTT 연결 실패: {e}")
    
    def on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            self.mqtt_connected = True
            self.get_logger().info("✅ MQTT 브로커에 연결됨")
            
            # 상태 발행
            self.publish_device_status("connected")
            
            # 제어 토픽 구독
            self.mqtt_client.subscribe("control/auto_mode")
            self.mqtt_client.subscribe("control/movement")
            self.get_logger().info("🔔 MQTT 제어 토픽 구독 완료")
        else:
            self.get_logger().error(f"❌ MQTT 연결 실패: {rc}")
            self.mqtt_connected = False
    
    def on_mqtt_disconnect(self, client, userdata, rc, properties=None, reason_code=None, reason_string=None):
        self.mqtt_connected = False
        self.get_logger().warning("⚠️ MQTT 연결 끊김")
    
    def laser_callback(self, msg):
        """ROS2 LaserScan 메시지를 MQTT로 변환하여 발행"""
        if not self.mqtt_connected:
            return
        
        self.scan_count += 1
        current_time = time.time()
        
        # 서버 호환 LiDAR 스캔 데이터 준비
        scan_data = []
        nearest_objects = []
        
        for i, range_value in enumerate(msg.ranges):
            if math.isfinite(range_value) and msg.range_min <= range_value <= msg.range_max:
                # 각도 계산 (0~360도 범위)
                angle_rad = msg.angle_min + i * msg.angle_increment
                angle_deg = math.degrees(angle_rad)
                if angle_deg < 0:
                    angle_deg += 360
                
                scan_data.append({
                    "angle": round(angle_deg, 1),
                    "distance": round(range_value, 2)
                })
                
                # 가까운 물체 찾기 (2m 이내)
                if range_value <= 2.0:
                    nearest_objects.append({
                        "angle": round(angle_deg, 1),
                        "distance": round(range_value, 2)
                    })
        
        # 가까운 물체를 거리순으로 정렬
        nearest_objects.sort(key=lambda x: x['distance'])
        
        # 서버 호환 형식으로 LiDAR 데이터 발행
        lidar_message = {
            "device_id": "ubuntu_pi_lidar_module",
            "timestamp": int(current_time * 1000),
            "scan_id": self.scan_count,
            "scan_data": scan_data,
            "nearest_objects": nearest_objects[:10],  # 상위 10개만
            "total_points": len(scan_data),
            "scan_range": {
                "min_angle": round(math.degrees(msg.angle_min), 1),
                "max_angle": round(math.degrees(msg.angle_max), 1),
                "min_distance": round(msg.range_min, 2),
                "max_distance": round(msg.range_max, 2)
            }
        }
        
        # 개별 LiDAR 데이터 포인트 발행 (개별 각도)
        for point in scan_data:
            point_data = {
                "device_id": "ubuntu_pi_lidar_module",
                "angle": point["angle"],
                "distance": point["distance"],
                "timestamp": int(current_time * 1000),
                "scan_id": self.scan_count
            }
            self.mqtt_client.publish("sensor/lidar/data", json.dumps(point_data))
        
        # 통합 스캔 데이터 발행
        self.mqtt_client.publish("sensor/lidar/scan", json.dumps(lidar_message))
        
        # 모바일 앱용 업데이트 발행
        mobile_update = {
            "device_id": "ubuntu_pi_lidar_module",
            "timestamp": int(current_time * 1000),
            "scan_data": scan_data,
            "nearest_objects": nearest_objects[:5]  # 모바일은 상위 5개만
        }
        self.mqtt_client.publish("mobile/lidar/update", json.dumps(mobile_update))
        
        # 5초마다 로그 출력
        if current_time - self.last_publish_time >= 5.0:
            self.get_logger().info(f"📊 스캔 #{self.scan_count}: {len(scan_data)}개 포인트, 가까운 물체: {len(nearest_objects)}개")
            self.last_publish_time = current_time
    
    def publish_device_status(self, status):
        """장치 상태 발행"""
        if not self.mqtt_connected:
            return
        
        status_message = {
            "device_id": "ubuntu_pi_lidar_module",
            "status": status,
            "client_type": "ubuntu_pi",
            "capabilities": ["lidar", "mqtt_bridge"],
            "timestamp": int(time.time() * 1000)
        }
        
        self.mqtt_client.publish("device/status", json.dumps(status_message))
        self.get_logger().info(f"📤 장치 상태 발행: {status}")
    
    def shutdown(self):
        """종료 처리"""
        if self.mqtt_connected:
            self.publish_device_status("disconnected")
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        self.get_logger().info("👋 LiDAR-MQTT 브리지 종료")

class DummyLidarPublisher(Node):
    """더미 LiDAR 데이터 발행기"""
    
    def __init__(self):
        super().__init__('dummy_lidar_publisher')
        
        self.get_logger().info("🚀 더미 LiDAR 발행기 초기화 중...")
        
        # QoS 설정
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # LaserScan 발행자 생성
        self.publisher = self.create_publisher(LaserScan, '/scan', qos)
        
        # 발행 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.publish_scan)
        
        # 테스트 시나리오
        self.scenarios = [
            ("clear", "전방위 안전"),
            ("front_obstacle", "전방 장애물"),
            ("left_obstacle", "좌측 장애물"),
            ("right_obstacle", "우측 장애물"),
            ("narrow_passage", "좁은 통로"),
            ("scattered_obstacles", "산발적 장애물"),
            ("wall_following", "벽면 추적"),
            ("emergency_stop", "긴급 정지")
        ]
        
        self.current_scenario_index = 0
        self.scenario_start_time = time.time()
        self.scenario_duration = 10.0
        
        # LiDAR 파라미터
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.pi / 180.0
        self.range_min = 0.1
        self.range_max = 12.0
        self.scan_time = 0.1
        
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        
        self.get_logger().info(f"📊 스캔 파라미터: {self.num_readings}개 포인트")
        
    def generate_scenario_data(self, scenario_name):
        """시나리오별 데이터 생성"""
        ranges = np.full(self.num_readings, self.range_max)
        
        if scenario_name == "clear":
            ranges = np.random.uniform(3.0, 5.0, self.num_readings)
        elif scenario_name == "front_obstacle":
            ranges = np.random.uniform(3.0, 5.0, self.num_readings)
            front_indices = list(range(0, 31)) + list(range(330, 360))
            for i in front_indices:
                if i < self.num_readings:
                    ranges[i] = np.random.uniform(0.4, 0.6)
        elif scenario_name == "left_obstacle":
            ranges = np.random.uniform(3.0, 5.0, self.num_readings)
            left_indices = range(60, 121)
            for i in left_indices:
                if i < self.num_readings:
                    ranges[i] = np.random.uniform(0.7, 0.9)
        elif scenario_name == "right_obstacle":
            ranges = np.random.uniform(3.0, 5.0, self.num_readings)
            right_indices = range(240, 301)
            for i in right_indices:
                if i < self.num_readings:
                    ranges[i] = np.random.uniform(0.7, 0.9)
        elif scenario_name == "narrow_passage":
            ranges = np.random.uniform(4.0, 5.0, self.num_readings)
            for i in range(60, 121):
                if i < self.num_readings:
                    ranges[i] = np.random.uniform(1.1, 1.3)
            for i in range(240, 301):
                if i < self.num_readings:
                    ranges[i] = np.random.uniform(1.1, 1.3)
        elif scenario_name == "scattered_obstacles":
            ranges = np.random.uniform(3.0, 5.0, self.num_readings)
            obstacle_angles = [45, 135, 225, 315]
            for angle in obstacle_angles:
                for offset in range(-15, 16):
                    idx = angle + offset
                    if 0 <= idx < self.num_readings:
                        ranges[idx] = np.random.uniform(1.4, 1.6)
        elif scenario_name == "wall_following":
            ranges = np.random.uniform(4.0, 5.0, self.num_readings)
            for angle in range(180, 361):
                if angle < self.num_readings:
                    distance = 1.5 + 0.5 * math.sin(math.radians(angle - 180) * 2)
                    ranges[angle] = max(0.5, distance)
        elif scenario_name == "emergency_stop":
            ranges = np.random.uniform(0.3, 0.4, self.num_readings)
        
        # 노이즈 추가
        noise = np.random.normal(0, 0.02, self.num_readings)
        ranges = np.clip(ranges + noise, self.range_min, self.range_max)
        
        return ranges
    
    def publish_scan(self):
        """스캔 데이터 발행"""
        current_time = time.time()
        
        # 시나리오 변경
        if current_time - self.scenario_start_time > self.scenario_duration:
            self.current_scenario_index = (self.current_scenario_index + 1) % len(self.scenarios)
            self.scenario_start_time = current_time
            scenario_name, scenario_desc = self.scenarios[self.current_scenario_index]
            
            print(f"\n📋 시나리오 변경: {scenario_name} → {scenario_desc}")
            self.get_logger().info(f"시나리오 변경: {scenario_name}")
        
        # 데이터 생성
        scenario_name, _ = self.scenarios[self.current_scenario_index]
        ranges = self.generate_scenario_data(scenario_name)
        
        # LaserScan 메시지 생성
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser'
        
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = self.scan_time
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        
        scan_msg.ranges = ranges.tolist()
        scan_msg.intensities = []
        
        # 발행
        self.publisher.publish(scan_msg)

# 전역 변수
lidar_bridge = None

def signal_handler(sig, frame):
    """종료 처리"""
    print("\n🛑 종료 시그널 수신")
    
    global lidar_bridge
    if lidar_bridge:
        lidar_bridge.shutdown()
    
    if 'rclpy' in sys.modules:
        try:
            rclpy.shutdown()
        except:
            pass
    
    print("👋 프로그램 종료")
    sys.exit(0)

def main():
    """메인 함수"""
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print("🚀 Fan Car LiDAR-MQTT 브리지 시작")
    print("=" * 50)
    
    # MQTT 브로커 IP 입력
    try:
        broker_ip = input("📡 MQTT 브로커 IP (기본값: 192.168.0.8): ").strip()
        if not broker_ip:
            broker_ip = "192.168.0.8"
    except:
        broker_ip = "192.168.0.8"
    
    # 모드 선택
    print("\n실행 모드를 선택하세요:")
    print("1. 통합 테스트 (더미 LiDAR + MQTT 브리지)")
    print("2. MQTT 브리지만 (실제 LiDAR → MQTT)")
    print("3. 더미 LiDAR만 (ROS2 발행만)")
    
    try:
        choice = input("\n선택 (1-3): ").strip()
        if choice not in ['1', '2', '3']:
            choice = '1'
    except:
        choice = '1'
    
    global lidar_bridge
    
    try:
        print("\n🔧 ROS2 초기화 중...")
        rclpy.init()
        
        executor = rclpy.executors.MultiThreadedExecutor()
        
        if choice == '1':
            print("\n🔄 통합 테스트 모드")
            dummy_lidar = DummyLidarPublisher()
            executor.add_node(dummy_lidar)
            
            lidar_bridge = LidarMqttBridge(broker_ip)
            executor.add_node(lidar_bridge)
            
        elif choice == '2':
            print("\n🌉 MQTT 브리지 모드")
            lidar_bridge = LidarMqttBridge(broker_ip)
            executor.add_node(lidar_bridge)
            
        elif choice == '3':
            print("\n📡 더미 LiDAR 모드")
            dummy_lidar = DummyLidarPublisher()
            executor.add_node(dummy_lidar)
        
        print("\nCtrl+C로 종료하세요...")
        executor.spin()
    
    except KeyboardInterrupt:
        print("\n⚠️ 키보드 인터럽트")
    except Exception as e:
        print(f"\n❌ 오류: {e}")
    finally:
        signal_handler(None, None)

if __name__ == '__main__':
    main() 