#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ubuntu 라즈베리파이 테스트 코드
- LIDAR 센서 시뮬레이션
- 초음파 센서 시뮬레이션  
- 바퀴 제어 시뮬레이션
"""

import paho.mqtt.client as mqtt
import json
import time
import random
import threading
from datetime import datetime

class UbuntuPiSimulator:
    def __init__(self, broker_host="192.168.0.8", broker_port=1883):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.client_id = "ubuntu_pi_lidar_module"
        
        # MQTT 클라이언트 초기화
        self.client = mqtt.Client(client_id=self.client_id)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        
        # 시스템 상태
        self.auto_mode = False
        self.is_moving = False
        self.current_speed = 0
        self.current_direction = "stop"
        
        # 센서 데이터 시뮬레이션
        self.lidar_running = False
        self.ultrasonic_running = False
        
        # 토픽 정의
        self.topics = {
            "lidar_data": "sensor/lidar/data",
            "lidar_scan": "sensor/lidar/scan", 
            "ultrasonic_data": "sensor/ultrasonic/data",
            "movement_control": "control/movement",
            "auto_mode": "control/auto_mode",
            "status": "device/status",
            "system_status": "system/status"
        }
        
        print(f"🤖 Ubuntu Pi 시뮬레이터 초기화 완료")
        print(f"📡 브로커: {broker_host}:{broker_port}")
        print(f"🆔 클라이언트 ID: {self.client_id}")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT 브로커 연결 성공!")
            
            # 상태 발행
            self.publish_status("connected")
            
            # 제어 토픽 구독
            client.subscribe(self.topics["movement_control"])
            client.subscribe(self.topics["auto_mode"])
            print(f"🔔 구독 토픽: {self.topics['movement_control']}, {self.topics['auto_mode']}")
            
        else:
            print(f"❌ MQTT 연결 실패, 코드: {rc}")

    def on_disconnect(self, client, userdata, rc):
        print("🔌 MQTT 연결 끊김")
        self.lidar_running = False
        self.ultrasonic_running = False

    def on_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            message = json.loads(msg.payload.decode())
            print(f"📨 메시지 수신: {topic}")
            print(f"📄 내용: {json.dumps(message, indent=2, ensure_ascii=False)}")
            
            if topic == self.topics["movement_control"]:
                self.handle_movement_control(message)
            elif topic == self.topics["auto_mode"]:
                self.handle_auto_mode(message)
                
        except Exception as e:
            print(f"❌ 메시지 처리 오류: {e}")

    def handle_auto_mode(self, message):
        """자율주행 모드 처리"""
        mode = message.get("mode", "")
        
        if mode == "enable_autonomous":
            self.auto_mode = True
            print("🚗 자율주행 모드 활성화!")
            
            # 센서 시뮬레이션 시작
            self.start_sensor_simulation()
            
        elif mode == "disable_autonomous":
            self.auto_mode = False
            print("🛑 자율주행 모드 비활성화!")
            
            # 센서 시뮬레이션 중지
            self.stop_sensor_simulation()
            self.stop_movement()

    def handle_movement_control(self, message):
        """바퀴 움직임 제어 처리"""
        action = message.get("action", "")
        
        if action == "move":
            direction = message.get("direction", "forward")
            speed = message.get("speed", 50)
            self.start_movement(direction, speed)
            
        elif action == "stop":
            self.stop_movement()
            
        elif action == "rotate":
            angle = message.get("angle", 0)
            self.rotate_robot(angle)

    def start_movement(self, direction, speed):
        """바퀴 움직임 시작"""
        self.is_moving = True
        self.current_direction = direction
        self.current_speed = speed
        
        print(f"🚀 이동 시작: {direction}, 속도: {speed}%")
        
        # 이동 상태 발행
        self.publish_movement_status()

    def stop_movement(self):
        """바퀴 움직임 중지"""
        self.is_moving = False
        self.current_direction = "stop"
        self.current_speed = 0
        
        print("⏹️ 이동 중지")
        
        # 이동 상태 발행
        self.publish_movement_status()

    def rotate_robot(self, angle):
        """로봇 회전"""
        print(f"🔄 로봇 회전: {angle}도")
        
        # 회전 시뮬레이션 (일정 시간 후 완료)
        def rotation_complete():
            time.sleep(2)  # 2초 회전 시뮬레이션
            print("✅ 회전 완료")
            self.publish_movement_status()
        
        threading.Thread(target=rotation_complete, daemon=True).start()

    def start_sensor_simulation(self):
        """센서 시뮬레이션 시작"""
        self.lidar_running = True
        self.ultrasonic_running = True
        
        # LIDAR 스레드 시작
        threading.Thread(target=self.lidar_simulation_thread, daemon=True).start()
        
        # 초음파 센서 스레드 시작  
        threading.Thread(target=self.ultrasonic_simulation_thread, daemon=True).start()
        
        print("🎯 센서 시뮬레이션 시작")

    def stop_sensor_simulation(self):
        """센서 시뮬레이션 중지"""
        self.lidar_running = False
        self.ultrasonic_running = False
        print("🎯 센서 시뮬레이션 중지")

    def lidar_simulation_thread(self):
        """LIDAR 센서 시뮬레이션"""
        angle = 0
        
        while self.lidar_running:
            try:
                # 360도 스캔 시뮬레이션
                distance = self.simulate_lidar_distance(angle)
                
                # LIDAR 데이터 발행
                lidar_data = {
                    "device_id": self.client_id,
                    "angle": round(angle, 1),
                    "distance": round(distance, 2),
                    "timestamp": int(time.time() * 1000),
                    "scan_id": int(time.time())
                }
                
                self.client.publish(self.topics["lidar_data"], 
                                  json.dumps(lidar_data))
                
                # 각도 증가
                angle += 2.0  # 2도씩 증가
                if angle >= 360:
                    angle = 0
                    print("🔄 LIDAR 360도 스캔 완료")
                
                time.sleep(0.1)  # 100ms 간격
                
            except Exception as e:
                print(f"❌ LIDAR 시뮬레이션 오류: {e}")
                time.sleep(1)

    def simulate_lidar_distance(self, angle):
        """LIDAR 거리 시뮬레이션 (사람과 벽 구분)"""
        # 벽까지의 기본 거리 (방 크기 시뮬레이션)
        base_distance = 3.0 + random.uniform(-0.5, 0.5)
        
        # 특정 각도에서 사람 시뮬레이션 (45도, 135도, 225도, 315도 근처)
        person_angles = [45, 135, 225, 315]
        
        for person_angle in person_angles:
            angle_diff = abs(angle - person_angle)
            if angle_diff < 10:  # 10도 범위 내
                # 사람까지의 거리 (1.5~2.5m)
                person_distance = 1.5 + random.uniform(0, 1.0)
                
                # 사람 감지 확률 (80%)
                if random.random() < 0.8:
                    return person_distance
        
        return base_distance

    def ultrasonic_simulation_thread(self):
        """초음파 센서 시뮬레이션"""
        sensor_positions = ["front", "back", "left", "right"]
        
        while self.ultrasonic_running:
            try:
                for position in sensor_positions:
                    # 거리 시뮬레이션 (10cm ~ 400cm)
                    distance = random.uniform(10, 400)
                    
                    # 장애물 감지 시뮬레이션
                    obstacle_detected = distance < 50  # 50cm 이내 장애물
                    
                    ultrasonic_data = {
                        "device_id": self.client_id,
                        "sensor_position": position,
                        "distance_cm": round(distance, 1),
                        "obstacle_detected": obstacle_detected,
                        "timestamp": int(time.time() * 1000)
                    }
                    
                    self.client.publish(self.topics["ultrasonic_data"], 
                                      json.dumps(ultrasonic_data))
                
                time.sleep(0.5)  # 500ms 간격
                
            except Exception as e:
                print(f"❌ 초음파 센서 시뮬레이션 오류: {e}")
                time.sleep(1)

    def publish_status(self, status):
        """상태 발행"""
        status_message = {
            "device_id": self.client_id,
            "status": status,
            "client_type": "ubuntu_pi",
            "capabilities": ["lidar", "ultrasonic", "movement"],
            "auto_mode": self.auto_mode,
            "is_moving": self.is_moving,
            "timestamp": int(time.time() * 1000)
        }
        
        self.client.publish(self.topics["status"], json.dumps(status_message))

    def publish_movement_status(self):
        """이동 상태 발행"""
        movement_status = {
            "device_id": self.client_id,
            "is_moving": self.is_moving,
            "direction": self.current_direction,
            "speed": self.current_speed,
            "timestamp": int(time.time() * 1000)
        }
        
        self.client.publish("movement/status", json.dumps(movement_status))

    def connect_and_run(self):
        """연결 및 실행"""
        try:
            print("🔌 MQTT 브로커 연결 시도...")
            self.client.connect(self.broker_host, self.broker_port, 60)
            
            # 상태 모니터링 스레드 시작
            threading.Thread(target=self.status_monitor_thread, daemon=True).start()
            
            # MQTT 루프 시작
            self.client.loop_forever()
            
        except Exception as e:
            print(f"❌ 연결 오류: {e}")

    def status_monitor_thread(self):
        """상태 모니터링 스레드"""
        while True:
            try:
                if self.client.is_connected():
                    self.publish_status("online")
                    
                    # 디버그 정보 출력
                    if self.auto_mode:
                        print(f"🤖 자율주행 모드: ON | 이동: {self.is_moving} | 방향: {self.current_direction}")
                    
                time.sleep(10)  # 10초마다 상태 업데이트
                
            except Exception as e:
                print(f"❌ 상태 모니터링 오류: {e}")
                time.sleep(5)

    def disconnect(self):
        """연결 해제"""
        self.stop_sensor_simulation()
        self.stop_movement()
        self.publish_status("disconnected")
        self.client.disconnect()
        print("👋 Ubuntu Pi 시뮬레이터 종료")

def main():
    print("=" * 50)
    print("🐧 Ubuntu 라즈베리파이 테스트 시뮬레이터")
    print("=" * 50)
    
    # 브로커 IP 입력 (기본값: 192.168.0.8)
    broker_ip = input("📡 MQTT 브로커 IP (기본값: 192.168.0.8): ").strip()
    if not broker_ip:
        broker_ip = "192.168.0.8"
    
    # 시뮬레이터 생성 및 실행
    simulator = UbuntuPiSimulator(broker_host=broker_ip)
    
    try:
        simulator.connect_and_run()
    except KeyboardInterrupt:
        print("\n🛑 사용자 종료 요청")
        simulator.disconnect()

if __name__ == "__main__":
    main() 