#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import json
import time
import paho.mqtt.client as mqtt
import traceback
import signal
import sys
import random

class LidarMqttBridgeNode(Node):
    def __init__(self):
        super().__init__('lidar_mqtt_bridge')
        
        # 종료 상태 추적 변수
        self.is_shutting_down = False
        
        # ROS2 매개변수 선언
        self.declare_parameter('broker_ip', 'localhost')
        self.declare_parameter('broker_port', 1883)
        
        # 매개변수 가져오기
        self.broker_ip = self.get_parameter('broker_ip').value
        self.broker_port = self.get_parameter('broker_port').value
        
        # MQTT 클라이언트 초기화 - API 버전 오류 수정
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)  # 버전 명시 추가
        self.client.on_connect = self.on_mqtt_connect
        self.client.on_disconnect = self.on_mqtt_disconnect
        
        # 장치 ID 및 토픽 정의
        self.device_id = "ydlidar_module"
        self.lidar_data_topic = "sensor/lidar/data"
        self.lidar_scan_topic = "sensor/lidar/scan"
        self.lidar_status_topic = "sensor/lidar/status"
        self.lidar_command_topic = "sensor/lidar/command"
        self.object_detection_topic = "sensor/lidar/object"
        self.obstacle_warning_topic = "sensor/lidar/warning"
        
        # MQTT 연결 상태
        self.mqtt_connected = False
        
        # 각도 필터링을 위한 변수
        self.angle_buffer = {}  # 각도별 최근 측정값 저장
        self.buffer_size = 3    # 버퍼 크기
        
        # ROS2 QoS 프로필 설정
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ROS2 구독 설정
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos
        )
        
        # 타이머 설정 (상태 메시지 주기적 발행)
        self.status_timer = self.create_timer(10.0, self.publish_status_timer)
        
        # 연결 시도
        self.connect_mqtt()
        
        self.get_logger().info('LidarMqttBridge 노드가 시작되었습니다')
        
    def connect_mqtt(self):
        try:
            self.get_logger().info(f'MQTT 브로커 {self.broker_ip}:{self.broker_port}에 연결 중...')
            self.client.connect(self.broker_ip, self.broker_port, 60)
            self.client.loop_start()
            return True
        except Exception as e:
            self.get_logger().error(f'MQTT 브로커 연결 실패: {str(e)}')
            return False
    
    def on_mqtt_connect(self, client, userdata, flags, rc, properties=None):  # properties 추가
        if rc == 0:
            self.get_logger().info(f'MQTT 브로커 {self.broker_ip}에 연결됨')
            self.mqtt_connected = True
            
            # 서버 명령 토픽 구독
            self.client.subscribe(self.lidar_command_topic)
            
            # 연결 성공 상태 메시지 발행
            self.publish_status("online")
        else:
            self.get_logger().error(f'MQTT 연결 실패: {rc}')
            self.mqtt_connected = False
    
    # VERSION2 API와 호환되도록 수정된 disconnect 콜백
    def on_mqtt_disconnect(self, client, userdata, rc, properties=None, reason_code=None, reason_string=None):
        self.get_logger().warn(f'MQTT 연결 끊김: {rc}')
        self.mqtt_connected = False
    
    def publish_message(self, topic, payload):
        """MQTT 메시지 발행 함수"""
        if not self.mqtt_connected or self.is_shutting_down:
            self.get_logger().warn('MQTT 연결이 없거나 종료 중입니다. 메시지를 발행할 수 없습니다.')
            return False
        
        try:
            self.client.publish(topic, json.dumps(payload), qos=1)
            return True
        except Exception as e:
            self.get_logger().error(f'메시지 발행 실패: {str(e)}')
            return False
    
    def publish_status_timer(self):
        """타이머에 의해 주기적으로 호출되는 상태 메시지 발행 함수"""
        if not self.is_shutting_down:
            self.publish_status("online")
    
    def publish_status(self, status):
        """상태 메시지 발행 함수 - 매개변수 포함"""
        status_data = {
            "device_id": self.device_id,
            "status": status,
            "timestamp": int(time.time() * 1000)
        }
        self.publish_message(self.lidar_status_topic, status_data)
    
    def normalize_angle_to_360(self, angle_rad):
        """각도를 0-360도 범위로 정규화 (라디안 입력, 도 출력)"""
        # 라디안에서 도로 변환
        angle_deg = np.degrees(angle_rad)
        
        # 0-360도 범위로 정규화
        while angle_deg < 0:
            angle_deg += 360.0
        while angle_deg >= 360.0:
            angle_deg -= 360.0
            
        return angle_deg
    
    def is_in_front_arc(self, angle_deg):
        """각도가 전방 아크(0-30도 또는 330-360도) 내에 있는지 확인"""
        return (angle_deg <= 30.0) or (angle_deg >= 330.0)
    
    def filter_angle_noise(self, angle, distance):
        """각도 노이즈 필터링 함수 - 이동 평균 사용"""
        # 각도를 정수로 반올림하여 키로 사용
        key = round(angle)
        
        # 처음 보는 각도라면 버퍼 생성
        if key not in self.angle_buffer:
            self.angle_buffer[key] = []
        
        # 버퍼에 현재 거리 추가
        self.angle_buffer[key].append(distance)
        
        # 버퍼 크기 제한
        if len(self.angle_buffer[key]) > self.buffer_size:
            self.angle_buffer[key].pop(0)
        
        # 현재 버퍼에 있는 거리값들의 평균 계산
        avg_distance = sum(self.angle_buffer[key]) / len(self.angle_buffer[key])
        
        return key, avg_distance
    
    def lidar_callback(self, msg):
        """LaserScan 메시지 수신 시 호출되는 콜백 함수"""
        # 종료 중이면 처리하지 않음
        if self.is_shutting_down:
            return
            
        try:
            # 1. 각도 계산 및 유효 데이터 필터링
            valid_data = []
            for i, r in enumerate(msg.ranges):
                # 각도 계산 및 정규화
                angle_rad = msg.angle_min + i * msg.angle_increment
                angle_deg = self.normalize_angle_to_360(angle_rad)
                
                # 유효한 거리값만 처리 (0.1m ~ 50m)
                if r > 0.1 and r < 50.0:
                    # 노이즈 필터링 적용
                    filtered_angle, filtered_distance = self.filter_angle_noise(angle_deg, r)
                    valid_data.append((i, filtered_angle, filtered_distance))
            
            if not valid_data:
                return
            
            # 2. 가장 가까운 물체 찾기
            closest_idx, closest_angle, closest_distance = min(valid_data, key=lambda x: x[2])
            
            # 3. 가장 가까운 물체 정보 발행
            object_data = {
                "device_id": self.device_id,
                "distance": round(closest_distance, 3),
                "angle": round(closest_angle, 2),
                "timestamp": int(time.time() * 1000)
            }
            self.publish_message(self.object_detection_topic, object_data)
            self.get_logger().info(f'가장 가까운 물체: 거리={closest_distance:.2f}m, 각도={closest_angle:.2f}°')
            
            # 4. 전방 장애물 감지 (0-30° 및 330-360° 범위)
            front_obstacles = []
            for _, angle, dist in valid_data:
                if self.is_in_front_arc(angle) and dist < 1.0:
                    front_obstacles.append((angle, dist))
            # 전방 장애물 감지 부분을 다음과 같이 수정
            if front_obstacles:
                # 랜덤 가중치를 적용하되, 인덱스 추적을 위해 원본 인덱스도 저장
                front_obstacles_with_weight = []
                for idx, (angle, dist) in enumerate(front_obstacles):
                    weight = random.uniform(-0.01, 0.01)
                    front_obstacles_with_weight.append((angle, dist + weight, idx))
                
                # 가중치가 적용된 거리 기준으로 가장 가까운 장애물 선택
                _, weighted_dist, original_idx = min(front_obstacles_with_weight, key=lambda x: x[1])
                
                # 원래 값 가져오기
                front_angle, warning_dist = front_obstacles[original_idx]
                
                warning_data = {
                    "device_id": self.device_id,
                    "warning": "front_obstacle",
                    "distance": round(warning_dist, 3),
                    "angle": round(front_angle, 2),
                    "timestamp": int(time.time() * 1000)
                }
                self.publish_message(self.obstacle_warning_topic, warning_data)
                self.get_logger().warn(f'전방 장애물 감지! 각도={front_angle:.2f}°, 거리={warning_dist:.2f}m')
            
            # 5. 스캔 데이터를 간략화하여 MQTT로 전송
            scan_data = []
            angles_to_sample = np.linspace(0, 359, 36)  # 약 10도 간격 36개 샘플
            
            for target_angle in angles_to_sample:
                # 샘플링 각도와 가장 가까운 측정값 찾기
                closest_points = []
                for _, angle, dist in valid_data:
                    angle_diff = abs(angle - target_angle)
                    if angle_diff < 5.0:  # 5도 이내 포인트만 사용
                        closest_points.append((angle_diff, angle, dist))
                
                if closest_points:
                    # 거리 차이가 가장 작은 것 선택
                    closest_points.sort(key=lambda x: x[0])
                    _, angle, distance = closest_points[0]
                    scan_data.append({
                        "angle": round(angle, 2),
                        "distance": round(distance, 3)
                    })
            
            # 스캔 데이터가 있으면 발행
            if scan_data:
                scan_message = {
                    "device_id": self.device_id,
                    "scan_points": scan_data,
                    "timestamp": int(time.time() * 1000)
                }
                self.publish_message(self.lidar_scan_topic, scan_message)
                
        except Exception as e:
            if not self.is_shutting_down:
                self.get_logger().error(f'LaserScan 처리 중 오류: {str(e)}')
                traceback.print_exc()
    
    def shutdown(self):
        """종료 시 정리 작업"""
        # 이미 종료 중이면 중복 작업 방지
        if self.is_shutting_down:
            return
            
        self.is_shutting_down = True
        self.get_logger().info('노드 종료 중...')
        
        try:
            # 오프라인 상태 메시지 발행
            if self.mqtt_connected:
                try:
                    status_data = {
                        "device_id": self.device_id,
                        "status": "offline",
                        "timestamp": int(time.time() * 1000)
                    }
                    self.client.publish(self.lidar_status_topic, json.dumps(status_data), qos=1)
                    time.sleep(0.1)  # 메시지가 전송될 시간 확보
                except Exception as e:
                    self.get_logger().error(f'종료 메시지 발행 실패: {str(e)}')
                
                # MQTT 연결 종료
                try:
                    self.client.loop_stop()
                    self.client.disconnect()
                except Exception as e:
                    self.get_logger().error(f'MQTT 연결 종료 실패: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'종료 중 오류: {str(e)}')
        
        self.get_logger().info('정상 종료됨')

def main():
    # ROS2 초기화
    try:
        rclpy.init()
    except Exception as e:
        print(f"ROS2 초기화 실패: {e}")
        return
    
    node = None
    
    try:
        # 노드 생성
        node = LidarMqttBridgeNode()
        
        # 시그널 핸들러 설정 (Ctrl+C 처리)
        def signal_handler(sig, frame):
            if node:
                node.get_logger().info('종료 시그널 수신')
                node.shutdown()
            try:
                rclpy.shutdown()
            except Exception:
                pass
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # ROS2 이벤트 루프 실행
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("키보드 인터럽트 감지")
    except Exception as e:
        if node:
            node.get_logger().error(f"예상치 못한 오류: {e}")
        else:
            print(f"초기화 중 오류: {e}")
        traceback.print_exc()
    finally:
        # 정리 작업
        if node and not node.is_shutting_down:
            node.shutdown()
            node.destroy_node()
        
        # ROS2 shutdown이 이미 호출되지 않았는지 확인
        try:
            rclpy.shutdown()
        except RuntimeError:
            # 이미 종료된 경우 무시
            pass

if __name__ == '__main__':
    main()