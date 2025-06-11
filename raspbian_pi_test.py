#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Raspbian 라즈베리파이 테스트 코드
- 카메라 얼굴인식 시뮬레이션
- 온도센서 시뮬레이션
- 선풍기 회전 제어 시뮬레이션
"""

import paho.mqtt.client as mqtt
import json
import time
import random
import threading
import math
from datetime import datetime

class RaspbianPiSimulator:
    def __init__(self, broker_host="192.168.0.8", broker_port=1883):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.client_id = "raspbian_pi_face_module"
        
        # MQTT 클라이언트 초기화
        self.client = mqtt.Client(client_id=self.client_id)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        
        # 시스템 상태
        self.auto_mode = False
        self.camera_running = False
        self.temp_sensing_running = False
        
        # 선풍기 상태
        self.fan_rotating = False
        self.fan_direction = "center"
        self.fan_speed = 0
        
        # 얼굴 인식 시뮬레이션 데이터
        self.detected_faces = []
        self.face_database = ["person_1", "person_2", "unknown"]
        
        # 온도 시뮬레이션 데이터
        self.temperature_sensors = {
            "sensor_1": {"x": 100, "y": 150, "temp": 36.5},
            "sensor_2": {"x": 500, "y": 150, "temp": 36.8}
        }
        
        # 토픽 정의
        self.topics = {
            "face_data": "sensor/face/thermal",
            "face_detection": "sensor/face/data", 
            "temperature_data": "sensor/temperature/data",
            "fan_control": "control/fan/rotation",
            "fan_direction": "control/direction",
            "auto_mode": "control/auto_mode",
            "status": "device/status"
        }
        
        print(f"📷 Raspbian Pi 시뮬레이터 초기화 완료")
        print(f"📡 브로커: {broker_host}:{broker_port}")
        print(f"🆔 클라이언트 ID: {self.client_id}")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT 브로커 연결 성공!")
            
            # 상태 발행
            self.publish_status("connected")
            
            # 제어 토픽 구독
            client.subscribe(self.topics["fan_control"])
            client.subscribe(self.topics["fan_direction"])
            client.subscribe(self.topics["auto_mode"])
            print(f"🔔 구독 토픽: {self.topics['fan_control']}, {self.topics['fan_direction']}, {self.topics['auto_mode']}")
            
        else:
            print(f"❌ MQTT 연결 실패, 코드: {rc}")

    def on_disconnect(self, client, userdata, rc):
        print("🔌 MQTT 연결 끊김")
        self.camera_running = False
        self.temp_sensing_running = False

    def on_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            message = json.loads(msg.payload.decode())
            print(f"📨 메시지 수신: {topic}")
            print(f"📄 내용: {json.dumps(message, indent=2, ensure_ascii=False)}")
            
            if topic == self.topics["fan_control"]:
                self.handle_fan_control(message)
            elif topic == self.topics["fan_direction"]:
                self.handle_fan_direction(message)
            elif topic == self.topics["auto_mode"]:
                self.handle_auto_mode(message)
                
        except Exception as e:
            print(f"❌ 메시지 처리 오류: {e}")

    def handle_auto_mode(self, message):
        """자율주행 모드 처리"""
        mode = message.get("mode", "")
        
        if mode == "enable_autonomous":
            self.auto_mode = True
            print("🤖 자율주행 모드 활성화!")
            
            # 센서 시뮬레이션 시작
            self.start_sensor_simulation()
            
        elif mode == "disable_autonomous":
            self.auto_mode = False
            print("🛑 자율주행 모드 비활성화!")
            
            # 센서 시뮬레이션 중지
            self.stop_sensor_simulation()
            self.stop_fan()

    def handle_fan_control(self, message):
        """선풍기 제어 처리"""
        action = message.get("action", "")
        
        if action == "start_rotation":
            speed = message.get("speed", 50)
            self.start_fan_rotation(speed)
            
        elif action == "stop_rotation":
            self.stop_fan()
            
        elif action == "set_speed":
            speed = message.get("speed", 50)
            self.set_fan_speed(speed)

    def handle_fan_direction(self, message):
        """선풍기 방향 제어 처리"""
        direction = message.get("direction", "center")
        face_id = message.get("face_id", "unknown")
        
        self.set_fan_direction(direction, face_id)

    def start_fan_rotation(self, speed):
        """선풍기 회전 시작"""
        self.fan_rotating = True
        self.fan_speed = speed
        
        print(f"💨 선풍기 회전 시작: 속도 {speed}%")
        self.publish_fan_status()

    def stop_fan(self):
        """선풍기 정지"""
        self.fan_rotating = False
        self.fan_speed = 0
        self.fan_direction = "center"
        
        print("🛑 선풍기 정지")
        self.publish_fan_status()

    def set_fan_speed(self, speed):
        """선풍기 속도 설정"""
        self.fan_speed = speed
        print(f"💨 선풍기 속도 변경: {speed}%")
        self.publish_fan_status()

    def set_fan_direction(self, direction, face_id="unknown"):
        """선풍기 방향 설정"""
        old_direction = self.fan_direction
        self.fan_direction = direction
        
        print(f"🔄 선풍기 방향 변경: {old_direction} → {direction} (대상: {face_id})")
        
        # 방향 변경 시뮬레이션 (2초 소요)
        def direction_change_complete():
            time.sleep(2)
            print(f"✅ 선풍기 방향 변경 완료: {direction}")
            self.publish_fan_status()
        
        threading.Thread(target=direction_change_complete, daemon=True).start()

    def start_sensor_simulation(self):
        """센서 시뮬레이션 시작"""
        self.camera_running = True
        self.temp_sensing_running = True
        
        # 카메라 스레드 시작
        threading.Thread(target=self.camera_simulation_thread, daemon=True).start()
        
        # 온도센서 스레드 시작
        threading.Thread(target=self.temperature_simulation_thread, daemon=True).start()
        
        print("📷 센서 시뮬레이션 시작")

    def stop_sensor_simulation(self):
        """센서 시뮬레이션 중지"""
        self.camera_running = False
        self.temp_sensing_running = False
        print("📷 센서 시뮬레이션 중지")

    def camera_simulation_thread(self):
        """카메라 얼굴인식 시뮬레이션"""
        while self.camera_running:
            try:
                # 랜덤하게 얼굴 수 결정 (0~3명)
                num_faces = random.choices([0, 1, 2, 3], weights=[10, 40, 35, 15])[0]
                
                faces = []
                
                for i in range(num_faces):
                    # 얼굴 위치 시뮬레이션 (640x480 해상도 기준)
                    x1 = random.randint(50, 400)
                    y1 = random.randint(50, 300)
                    x2 = x1 + random.randint(80, 150)
                    y2 = y1 + random.randint(100, 200)
                    
                    # 얼굴 ID 결정 (80% 확률로 알려진 사람)
                    if random.random() < 0.8:
                        face_id = random.choice(self.face_database[:-1])  # unknown 제외
                    else:
                        face_id = "unknown"
                    
                    # 온도 시뮬레이션 (36.0~38.5도)
                    temperature = round(random.uniform(36.0, 38.5), 1)
                    temp_confidence = "high" if temperature > 37.0 else "medium"
                    
                    face_data = {
                        "face_id": face_id,
                        "face_box": [x1, y1, x2, y2],
                        "confidence": round(random.uniform(0.7, 0.95), 2),
                        "temperature": temperature,
                        "temp_confidence": temp_confidence
                    }
                    
                    faces.append(face_data)
                
                # 얼굴 + 온도 통합 데이터 발행
                face_thermal_data = {
                    "device_id": self.client_id,
                    "faces": faces,
                    "image_size": [640, 480],
                    "timestamp": int(time.time() * 1000)
                }
                
                self.client.publish(self.topics["face_data"], 
                                  json.dumps(face_thermal_data))
                
                # 디버그 출력
                if faces:
                    print(f"👥 감지된 얼굴: {len(faces)}명")
                    for face in faces:
                        print(f"  - {face['face_id']}: {face['temperature']}°C ({face['temp_confidence']})")
                
                time.sleep(1.0)  # 1초 간격
                
            except Exception as e:
                print(f"❌ 카메라 시뮬레이션 오류: {e}")
                time.sleep(1)

    def temperature_simulation_thread(self):
        """온도센서 시뮬레이션"""
        while self.temp_sensing_running:
            try:
                for sensor_id, sensor_info in self.temperature_sensors.items():
                    # 온도 변화 시뮬레이션 (±0.5도 범위)
                    temp_change = random.uniform(-0.5, 0.5)
                    new_temp = sensor_info["temp"] + temp_change
                    
                    # 온도 범위 제한 (35.0~40.0도)
                    new_temp = max(35.0, min(40.0, new_temp))
                    sensor_info["temp"] = round(new_temp, 1)
                    
                    # 온도 데이터 발행
                    temp_data = {
                        "device_id": self.client_id,
                        "sensor_id": sensor_id,
                        "temperature": sensor_info["temp"],
                        "position": {"x": sensor_info["x"], "y": sensor_info["y"]},
                        "timestamp": int(time.time() * 1000)
                    }
                    
                    self.client.publish(self.topics["temperature_data"], 
                                      json.dumps(temp_data))
                
                time.sleep(2.0)  # 2초 간격
                
            except Exception as e:
                print(f"❌ 온도센서 시뮬레이션 오류: {e}")
                time.sleep(1)

    def publish_status(self, status):
        """상태 발행"""
        status_message = {
            "device_id": self.client_id,
            "status": status,
            "client_type": "raspbian_pi",
            "capabilities": ["camera", "face_recognition", "temperature", "fan_control"],
            "auto_mode": self.auto_mode,
            "camera_running": self.camera_running,
            "fan_rotating": self.fan_rotating,
            "timestamp": int(time.time() * 1000)
        }
        
        self.client.publish(self.topics["status"], json.dumps(status_message))

    def publish_fan_status(self):
        """선풍기 상태 발행"""
        fan_status = {
            "device_id": self.client_id,
            "is_rotating": self.fan_rotating,
            "direction": self.fan_direction,
            "speed": self.fan_speed,
            "timestamp": int(time.time() * 1000)
        }
        
        self.client.publish("fan/status", json.dumps(fan_status))

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
                        print(f"🤖 자율주행 모드: ON | 카메라: {self.camera_running} | 선풍기: {self.fan_rotating} ({self.fan_direction})")
                    
                time.sleep(10)  # 10초마다 상태 업데이트
                
            except Exception as e:
                print(f"❌ 상태 모니터링 오류: {e}")
                time.sleep(5)

    def disconnect(self):
        """연결 해제"""
        self.stop_sensor_simulation()
        self.stop_fan()
        self.publish_status("disconnected")
        self.client.disconnect()
        print("👋 Raspbian Pi 시뮬레이터 종료")

def main():
    print("=" * 50)
    print("🍓 Raspbian 라즈베리파이 테스트 시뮬레이터")
    print("=" * 50)
    
    # 브로커 IP 입력 (기본값: 192.168.0.8)
    broker_ip = input("📡 MQTT 브로커 IP (기본값: 192.168.0.8): ").strip()
    if not broker_ip:
        broker_ip = "192.168.0.8"
    
    # 시뮬레이터 생성 및 실행
    simulator = RaspbianPiSimulator(broker_host=broker_ip)
    
    try:
        simulator.connect_and_run()
    except KeyboardInterrupt:
        print("\n🛑 사용자 종료 요청")
        simulator.disconnect()

if __name__ == "__main__":
    main() 