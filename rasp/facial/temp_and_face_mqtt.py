# mqtt.py - MQTT 통신 모듈 (수정 버전)
import paho.mqtt.client as mqtt
import json
import uuid
import threading
import time
import numpy as np
import os
import sys
import argparse
import socket  # 네트워크 연결 테스트용
import subprocess

# MQTT 설정 - 기본값 제공 및 명령행 인수로 설정 가능
DEFAULT_BROKER = "192.168.191.223"
DEFAULT_PORT = 1883
RETRY_INTERVAL = 5  # 연결 실패 시 재시도 간격(초)
MAX_RETRIES = 5     # 최대 재시도 횟수
DEBUG_MODE = False  # 디버그 모드 (상세 로깅)

# 필요하다면 temp_and_face_detect.py 모듈 import (예외 처리 추가)
try:
    import temp_and_face_detect
    TEMP_TEST_AVAILABLE = True
except ImportError:
    print("경고: temp_and_face_detect 모듈을 가져올 수 없습니다. 독립 실행 모드로 작동합니다.")
    TEMP_TEST_AVAILABLE = False

# MQTT 설정
try:
    # 동적 IP 감지 시도
    detected_ip = get_system_ip()
    if detected_ip != "localhost":
        # 네트워크 세그먼트에서 서버 IP 추정 (마지막 옥텟을 223으로 변경)
        ip_parts = detected_ip.split('.')
        if len(ip_parts) == 4:
            estimated_server_ip = f"{ip_parts[0]}.{ip_parts[1]}.{ip_parts[2]}.223"
            MQTT_BROKER = estimated_server_ip
        else:
            MQTT_BROKER = DEFAULT_BROKER
    else:
        MQTT_BROKER = DEFAULT_BROKER
except:
    MQTT_BROKER = DEFAULT_BROKER

MQTT_PORT = DEFAULT_PORT
DEVICE_ID = f"thermal_face_recognition_{uuid.uuid4().hex[:8]}"

# MQTT 토픽 정의
TOPICS = {
    "TEMP": "sensor/temperature",
    "DETECTION": "sensor/detection",
    "STATUS": "device/status",
    "FACE_DETECTION": "sensor/face/data",
    "MULTI_FACE": "sensor/face/grid",
    "THERMAL_MAP": "sensor/face/thermal",
    "GRID_STATUS": "sensor/face/grid",
    "SYSTEM_LOG": "system/log",
    "CONTROL_DIRECTION": "control/direction"
}

# 전역 변수
mqtt_client = None
mqtt_connected = False
mqttThread = None
mqtt_running = False

def convert_numpy_types(obj):
    """JSON 직렬화를 위해 NumPy 타입을 Python 네이티브 타입으로 변환"""
    import numpy as np
    if isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, tuple) and any(isinstance(x, np.integer) for x in obj):
        return tuple(int(x) if isinstance(x, np.integer) else x for x in obj)
    elif isinstance(obj, dict):
        return {k: convert_numpy_types(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy_types(item) for item in obj]
    return obj

# 네트워크 테스트 함수 추가
def test_network_connection(host, port, timeout=2):
    """서버에 대한 기본 네트워크 연결 테스트"""
    try:
        # 간단한 TCP 연결 테스트
        socket.setdefaulttimeout(timeout)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        s.close()
        print(f"네트워크 연결 확인: {host}:{port} 연결 성공")
        return True
    except socket.error as e:
        print(f"네트워크 연결 오류: {host}:{port} - {e}")
        return False

# MQTT 연결 로그 콜백 개선
def on_connect(client, userdata, flags, rc):
    """MQTT 연결 콜백 (개선된 버전)"""
    global mqtt_connected
    
    # MQTT 연결 결과 코드 해석
    connection_codes = {
        0: "성공 - 연결 완료",
        1: "연결 거부: 잘못된 프로토콜 버전",
        2: "연결 거부: 잘못된 클라이언트 ID",
        3: "연결 거부: 서버 사용 불가",
        4: "연결 거부: 잘못된 사용자 이름 또는 비밀번호",
        5: "연결 거부: 권한 없음"
    }
    
    if rc == 0:
        print(f"✅ MQTT 브로커에 성공적으로 연결됨: {MQTT_BROKER}:{MQTT_PORT}")
        print(f"   클라이언트 ID: {client._client_id.decode()}")
        mqtt_connected = True
        
        # 상태 메시지 전송
        try:
            publish_status("online")
        except Exception as e:
            print(f"상태 메시지 발행 오류: {e}")
    else:
        error_msg = connection_codes.get(rc, f"알 수 없는 오류 코드: {rc}")
        print(f"❌ MQTT 연결 실패: {error_msg}")
        mqtt_connected = False

def on_disconnect(client, userdata, rc):
    """MQTT 연결 해제 콜백 (개선된 버전)"""
    global mqtt_connected
    if rc == 0:
        print("MQTT 브로커와 정상적으로 연결 해제됨")
    else:
        print(f"❗ 예기치 않은 MQTT 연결 해제, 코드: {rc}")
    mqtt_connected = False

def on_publish(client, userdata, mid):
    """메시지 발행 콜백"""
    if DEBUG_MODE:
        print(f"메시지 발행 완료: ID={mid}")

def initialize_mqtt():
    """MQTT 클라이언트 초기화 (개선된 버전)"""
    global mqtt_client, mqtt_connected
    
    try:
        print(f"\n=== MQTT 브로커 연결 시도: {MQTT_BROKER}:{MQTT_PORT} ===")
        
        # 네트워크 연결 테스트 추가
        if not test_network_connection(MQTT_BROKER, MQTT_PORT):
            print(f"⚠️ 서버 {MQTT_BROKER}:{MQTT_PORT}에 네트워크 연결할 수 없습니다.")
            print("  네트워크 연결을 확인하거나 다른 브로커를 사용하세요.")
            return False
        
        # 클라이언트 생성 (디버그 로깅 추가)
        client_id = DEVICE_ID
        mqtt_client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311, clean_session=True)
        
        # 디버그 로깅 활성화
        if DEBUG_MODE:
            mqtt_client.enable_logger()
        
        # 콜백 설정
        mqtt_client.on_connect = on_connect
        mqtt_client.on_disconnect = on_disconnect
        mqtt_client.on_publish = on_publish
        
        # Last Will and Testament 설정 (연결 끊김시 자동 메시지)
        will_payload = json.dumps({
            "device_id": DEVICE_ID,
            "status": "offline",
            "timestamp": int(time.time() * 1000)
        })
        mqtt_client.will_set(TOPICS["STATUS"], will_payload, qos=1, retain=True)
        
        print("브로커 연결 시도 중...")
        
        # 동기식 연결 사용 (문제 디버깅 용이)
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        
        # 백그라운드 스레드로 실행
        mqtt_client.loop_start()
        
        # 연결 대기
        retry_count = 0
        max_local_retries = 5
        while not mqtt_connected and retry_count < max_local_retries:
            print(f"MQTT 브로커 연결 대기 중... ({retry_count+1}/{max_local_retries})")
            time.sleep(1)
            retry_count += 1
            
        if not mqtt_connected:
            mqtt_client.loop_stop()
            print("❌ MQTT 연결 시간 초과")
            return False
            
        return True
    except Exception as e:
        print(f"❌ MQTT 초기화 오류: {e}")
        
        # 브로커가 실행 중이 아닌 경우 해결책 제안
        if "Connection refused" in str(e):
            print("\n✋ 가능한 해결책:")
            print("1. 서버에서 Mosquitto가 실행 중인지 확인:")
            print("   (서버) systemctl status mosquitto")
            print("2. 서버 방화벽이 1883 포트를 허용하는지 확인:")
            print("   (서버) sudo ufw status")
            print("   (서버) sudo ufw allow 1883")
            print("3. 서버에서 Mosquitto 설정 확인:")
            print("   (서버) cat /etc/mosquitto/mosquitto.conf")
            print("   설정 파일에 다음 내용이 있어야 합니다:")
            print("     listener 1883")
            print("     allow_anonymous true")
            print("4. 공용 브로커로 테스트:")
            print("   python mqtt.py --broker test.mosquitto.org")
            print("5. 네트워크 연결 테스트:")
            print(f"   ping {MQTT_BROKER}")
            print(f"   nc -zv {MQTT_BROKER} {MQTT_PORT}")
        
        return False

def publish_status(status):
    """장치 상태 발행"""
    if not mqtt_client or not mqtt_connected:
        return False
        
    try:
        payload = {
            "device_id": DEVICE_ID,
            "status": status,
            "client_type": "thermal_face_recognition",
            "connection_type": "tcp",
            "capabilities": ["face_recognition", "thermal_sensing", "temperature_measurement"],
            "timestamp": int(time.time() * 1000)
        }
        
        result = mqtt_client.publish(
            TOPICS["STATUS"], 
            json.dumps(payload), 
            qos=1
        )
        
        if DEBUG_MODE:
            print(f"상태 메시지 발행: {status}, 결과: {result.rc}")
            
        return result.rc == mqtt.MQTT_ERR_SUCCESS
    except Exception as e:
        print(f"상태 발행 오류: {e}")
        return False

def publish_temperature_data(face_id, temperature, confidence, face_box):
    """온도 데이터 발행"""
    if not mqtt_client or not mqtt_connected:
        return False
        
    try:
        payload = {
            "device_id": DEVICE_ID,
            "face_id": face_id,
            "temperature": temperature,
            "confidence": confidence,
            "face_box": face_box,
            "timestamp": int(time.time() * 1000)
        }
        
        result = mqtt_client.publish(
            TOPICS["TEMP"], 
            json.dumps(payload), 
            qos=1
        )
        
        return result.rc == mqtt.MQTT_ERR_SUCCESS
    except Exception as e:
        print(f"온도 데이터 발행 오류: {e}")
        return False

def publish_face_detection(face_data):
    """안면 인식 데이터 발행"""
    if not mqtt_client or not mqtt_connected:
        return False
        
    try:
        # 디버깅: 원본 얼굴 데이터 출력
        print(f"[PUBLISH DEBUG] Original face_data: {face_data}")
        print(f"[PUBLISH DEBUG] face_data type: {type(face_data)}")
        if isinstance(face_data, list) and len(face_data) > 0:
            print(f"[PUBLISH DEBUG] First face: {face_data[0]}")
            if 'face_box' in face_data[0]:
                print(f"[PUBLISH DEBUG] First face_box: {face_data[0]['face_box']}")
                print(f"[PUBLISH DEBUG] face_box type: {type(face_data[0]['face_box'])}")
        
        # 데이터 전처리 - NumPy 타입 변환
        converted_face_data = convert_numpy_types(face_data)
        
        # 디버깅: 변환된 얼굴 데이터 출력
        print(f"[PUBLISH DEBUG] Converted face_data: {converted_face_data}")

        payload = {
            "device_id": DEVICE_ID,
            "faces": converted_face_data,
            "total_faces": len(converted_face_data) if converted_face_data else 0,
            "timestamp": int(time.time() * 1000)
        }
        
        # 디버깅: 최종 페이로드 출력
        print(f"[PUBLISH DEBUG] Final payload: {json.dumps(payload, indent=2)}")
        
        result = mqtt_client.publish(
            TOPICS["FACE_DETECTION"], 
            json.dumps(payload), 
            qos=1
        )
        
        print(f"[PUBLISH DEBUG] Publish result: {result.rc == mqtt.MQTT_ERR_SUCCESS}")
        
        return result.rc == mqtt.MQTT_ERR_SUCCESS
    except Exception as e:
        print(f"안면 인식 데이터 발행 오류: {e}")
        import traceback
        traceback.print_exc()
        return False

def publish_multi_face_grid(grid_data):
    """다중 얼굴 그리드 데이터 발행"""
    if not mqtt_client or not mqtt_connected:
        return False
        
    try:
        # 그리드 데이터를 직렬화 가능하게 변환 - NumPy 타입 처리 추가
        serializable_grid = convert_numpy_types(grid_data)
            
        payload = {
            "device_id": DEVICE_ID,
            "grid_data": serializable_grid,
            "grid_size": [3, 3],  # 3x3 그리드
            "timestamp": int(time.time() * 1000)
        }
        
        result = mqtt_client.publish(
            TOPICS["MULTI_FACE"], 
            json.dumps(payload), 
            qos=1
        )
        
        return result.rc == mqtt.MQTT_ERR_SUCCESS
    except Exception as e:
        print(f"그리드 데이터 발행 오류: {e}")
        return False

def publish_system_log(message, level="info"):
    """시스템 로그 발행"""
    if not mqtt_client or not mqtt_connected:
        return False
        
    try:
        payload = {
            "device_id": DEVICE_ID,
            "level": level,
            "message": message,
            "timestamp": int(time.time() * 1000)
        }
        
        result = mqtt_client.publish(
            TOPICS["SYSTEM_LOG"], 
            json.dumps(payload), 
            qos=0  # 로그는 QoS 0 사용
        )
        
        return result.rc == mqtt.MQTT_ERR_SUCCESS
    except Exception as e:
        print(f"로그 발행 오류: {e}")
        return False

def publish_thermal_map(thermal_data, shape=None):
    """열화상 데이터 발행"""
    if not mqtt_client or not mqtt_connected:
        return False
        
    try:
        # 데이터 전처리 - NumPy 타입 변환
        converted_data = convert_numpy_types(thermal_data)
        
        # 현재 감지된 얼굴 데이터 가져오기
        current_faces = temp_and_face_detect.get_latest_face_data() or []
        
        # 얼굴 데이터 형식 변환 및 검증
        processed_faces = []
        for face in current_faces:
            # 필수 필드 검증
            if not isinstance(face, dict):
                continue
                
            # face_box 검증
            face_box = face.get("face_box")
            if not face_box or not isinstance(face_box, list) or len(face_box) != 4:
                continue
                
            # temperature 검증
            temperature = face.get("temperature")
            if temperature is None or not isinstance(temperature, (int, float)):
                continue
                
            # temp_confidence 검증
            temp_confidence = face.get("temp_confidence")
            if not temp_confidence or not isinstance(temp_confidence, str):
                temp_confidence = "low"
                
            processed_face = {
                "face_id": str(face.get("face_id", "Unknown")),
                "face_box": [float(x) for x in face_box],
                "confidence": float(face.get("confidence", 0.0)),
                "temperature": float(temperature),
                "temp_confidence": temp_confidence,
                "grid_position": [int(x) for x in face.get("grid_position", [0, 0])]
            }
            processed_faces.append(processed_face)
        
        # 열화상 데이터에 얼굴 정보 추가
        payload = {
            "device_id": DEVICE_ID,
            "thermal_data": converted_data,
            "shape": shape or [24, 32],  # 기본 shape
            "faces": processed_faces,  # 처리된 얼굴 정보
            "min_temp": float(np.min(converted_data)),
            "max_temp": float(np.max(converted_data)),
            "timestamp": int(time.time() * 1000)
        }
        
        # 디버그 로깅 추가
        if DEBUG_MODE:
            print(f"발행할 열화상 데이터: {json.dumps(payload, indent=2)}")
            print(f"얼굴 데이터 검증: {len(processed_faces)}개 얼굴 처리됨")
        
        result = mqtt_client.publish(
            TOPICS["THERMAL_MAP"], 
            json.dumps(payload),
            qos=1
        )
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            if DEBUG_MODE:
                print(f"열화상 데이터 발행 성공: {len(converted_data)} 포인트, {len(processed_faces)} 얼굴")
            return True
        else:
            print(f"열화상 데이터 발행 실패: {result.rc}")
            return False
            
    except Exception as e:
        print(f"열화상 데이터 발행 오류: {e}")
        import traceback
        traceback.print_exc()
        return False

def handle_face_thermal_data(faces, frame_shape):
    """얼굴 인식 데이터 처리 및 팬 제어 메시지 발행
    
    Args:
        faces: 감지된 얼굴 데이터 리스트
        frame_shape: 카메라 프레임 크기 (height, width)
    """
    if not faces:
        # 얼굴이 감지되지 않은 경우 중앙 정렬
        mqtt_client.publish(TOPICS["CONTROL_DIRECTION"], "center")
        return

    frame_width = frame_shape[1]
    frame_center_x = frame_width / 2
    center_tolerance = frame_width * 0.1  # 중앙 허용 범위 (10%)

    if len(faces) == 1:
        # 단일 얼굴 처리
        face = faces[0]
        x, y, w, h = face["face_box"]
        face_center_x = x + w/2
        
        # 얼굴이 중앙에 있는지 확인
        if abs(face_center_x - frame_center_x) < center_tolerance:
            direction = "center"
        else:
            direction = "left" if face_center_x < frame_center_x else "right"
            
        print(f"Single face detected: {direction} rotation needed")
        mqtt_client.publish(TOPICS["CONTROL_DIRECTION"], direction)
        
    else:
        # 다중 얼굴 처리 - 온도가 가장 높은 얼굴 선택
        hottest_face = max(
            (face for face in faces if face.get("temperature") is not None),
            key=lambda f: f["temperature"],
            default=None
        )
        
        if hottest_face:
            x, y, w, h = hottest_face["face_box"]
            face_center_x = x + w/2
            
            # 가장 뜨거운 얼굴이 중앙에 있는지 확인
            if abs(face_center_x - frame_center_x) < center_tolerance:
                direction = "center"
            else:
                direction = "left" if face_center_x < frame_center_x else "right"
                
            print(f"Multiple faces detected: Rotating {direction} to center hottest face (temp={hottest_face['temperature']:.1f}°C)")
            mqtt_client.publish(TOPICS["CONTROL_DIRECTION"], direction)
        else:
            # 온도 데이터가 없는 경우 중앙 정렬
            print("No temperature data available, centering")
            mqtt_client.publish(TOPICS["CONTROL_DIRECTION"], "center")

def send_test_face_data():
    """테스트용 가짜 얼굴 데이터 전송"""
    if not mqtt_client or not mqtt_connected:
        return False
    
    # 테스트용 얼굴 데이터 생성
    test_face_data = [
        {
            "face_id": "test_person_1",
            "face_box": [200, 150, 400, 350],  # [x1, y1, x2, y2] 형식
            "confidence": 0.95,
            "temperature": 36.5,
            "temp_confidence": "high"
        }
    ]
    
    print(f"[TEST] Sending test face data: {test_face_data}")
    return publish_face_detection(test_face_data)

# MQTT 스레드 함수 수정
def mqtt_thread_func():
    """MQTT 통신 스레드 함수"""
    global mqtt_running
    
    print("MQTT 통신 스레드 시작")
    
    if not TEMP_TEST_AVAILABLE:
        print("경고: temp_and_face_detect 모듈이 없어 테스트 데이터를 전송합니다.")
        # 테스트 데이터 전송
        test_count = 0
        while mqtt_running:
            if mqtt_connected:
                try:
                    # 10초마다 테스트 얼굴 데이터 전송
                    if test_count % 100 == 0:  # 10초마다 (0.1초 * 100)
                        send_test_face_data()
                    
                    # 하트비트 메시지 (30초마다)
                    if test_count % 300 == 0:  # 30초마다
                        publish_system_log("MQTT 서비스 실행 중 (테스트 모드)")
                    
                    test_count += 1
                except Exception as e:
                    print(f"테스트 메시지 발행 오류: {e}")
            time.sleep(0.1)
        return
    
    # 마지막 전송 시간 추적
    last_face_data_publish = 0
    last_grid_publish = 0
    last_thermal_publish = 0
    
    while mqtt_running:
        try:
            current_time = time.time()
            
            # 얼굴 데이터 전송 (0.5초마다)
            if current_time - last_face_data_publish >= 0.5:
                face_data = temp_and_face_detect.get_latest_face_data()
                if face_data:
                    # 데이터 검증 및 수정
                    validated_face_data = []
                    for i, face in enumerate(face_data):
                        try:
                            # face_box 검증 및 수정
                            if 'face_box' in face:
                                face_box = face['face_box']
                                
                                # OpenCV 형식 (x, y, w, h)를 (x1, y1, x2, y2) 형식으로 변환
                                if isinstance(face_box, (list, tuple)) and len(face_box) == 4:
                                    x, y, w, h = face_box
                                    # 만약 w, h가 너무 크면 (x, y, w, h) 형식이고, 작으면 (x1, y1, x2, y2) 형식일 가능성
                                    if w > 50 and h > 50:  # width, height로 추정
                                        # (x, y, w, h) -> (x1, y1, x2, y2) 변환
                                        face['face_box'] = [x, y, x + w, y + h]
                                        print(f"[VALIDATION] Face {i}: Converted (x,y,w,h) to (x1,y1,x2,y2): {face['face_box']}")
                                    else:
                                        # 이미 (x1, y1, x2, y2) 형식으로 추정
                                        face['face_box'] = list(face_box)
                                        print(f"[VALIDATION] Face {i}: Already in (x1,y1,x2,y2) format: {face['face_box']}")
                                else:
                                    print(f"[VALIDATION] Face {i}: Invalid face_box format, skipping")
                                    continue
                            else:
                                print(f"[VALIDATION] Face {i}: No face_box field, skipping")
                                continue
                            
                            # 필수 필드 기본값 설정
                            if 'face_id' not in face:
                                face['face_id'] = f"unknown_{i}"
                            if 'confidence' not in face:
                                face['confidence'] = 0.8
                            if 'temperature' not in face:
                                face['temperature'] = None
                            if 'temp_confidence' not in face:
                                face['temp_confidence'] = "unknown"
                            
                            validated_face_data.append(face)
                            
                        except Exception as e:
                            print(f"[VALIDATION] Error processing face {i}: {e}")
                            continue
                    
                    # 검증된 데이터가 있으면 발행
                    if validated_face_data:
                        print(f"[VALIDATION] Publishing {len(validated_face_data)} validated faces")
                        publish_face_detection(validated_face_data)
                        
                        # 각 얼굴에 대해 온도 데이터도 발행
                        for face in validated_face_data:
                            if face.get('face_id') != "Unknown" and face.get('temperature') is not None:
                                publish_temperature_data(
                                    face['face_id'],
                                    face['temperature'],
                                    face.get('temp_confidence', 'unknown'),
                                    face['face_box']
                                )
                    else:
                        print(f"[VALIDATION] No valid faces found in data: {face_data}")
                        
                last_face_data_publish = current_time
            
            # 그리드 맵 전송 (1초마다)
            if current_time - last_grid_publish >= 1.0:
                grid_data = temp_and_face_detect.get_latest_grid_data()
                publish_multi_face_grid(grid_data)
                last_grid_publish = current_time
            
            # 열화상 데이터 전송 (10초마다)
            if current_time - last_thermal_publish >= 10.0:
                thermal_data = temp_and_face_detect.get_latest_thermal_data()
                if thermal_data is not None:
                    publish_thermal_map(thermal_data)
                last_thermal_publish = current_time
            
            # CPU 사용량 줄이기 위한 짧은 대기
            time.sleep(0.1)
            
        except Exception as e:
            print(f"MQTT 스레드 오류: {e}")
            time.sleep(1)  # 오류 발생 시 잠시 대기

def start_mqtt_thread():
    """MQTT 통신 스레드 시작"""
    global mqttThread, mqtt_running
    
    if mqttThread is not None and mqttThread.is_alive():
        print("MQTT 스레드가 이미 실행 중입니다.")
        return False
    
    # MQTT 초기화
    if not initialize_mqtt():
        print("MQTT 초기화 실패")
        return False
    
    # 스레드 시작
    mqtt_running = True
    mqttThread = threading.Thread(target=mqtt_thread_func, daemon=True)
    mqttThread.start()
    
    if mqtt_connected:
        publish_system_log("열화상 안면인식 통신 시작")
        print("✅ MQTT 통신 스레드가 시작되었습니다.")
    return True

def stop_mqtt_thread():
    """MQTT 통신 스레드 중지"""
    global mqtt_running, mqttThread, mqtt_client
    
    mqtt_running = False
    
    if mqttThread is not None:
        print("MQTT 스레드 종료 대기...")
        if mqttThread.is_alive():
            mqttThread.join(timeout=2.0)
    
    if mqtt_client and mqtt_connected:
        try:
            publish_status("offline")
        except:
            pass
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
    print("MQTT 연결 종료")

def configure_mqtt(broker=None, port=None, device_id=None):
    """MQTT 설정 함수 (호환성을 위해 추가)"""
    global MQTT_BROKER, MQTT_PORT, DEVICE_ID
    
    if broker:
        MQTT_BROKER = broker
    if port:
        MQTT_PORT = port
    if device_id:
        DEVICE_ID = device_id
    
    print(f"MQTT 설정 완료: {MQTT_BROKER}:{MQTT_PORT}, Device ID: {DEVICE_ID}")
    return True

# 명령행 인수 파싱 (개선된 버전)
def parse_args():
    parser = argparse.ArgumentParser(description='MQTT 통신 모듈')
    parser.add_argument('--broker', type=str, default=DEFAULT_BROKER,
                        help=f'MQTT 브로커 주소 (기본값: {DEFAULT_BROKER})')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT,
                        help=f'MQTT 브로커 포트 (기본값: {DEFAULT_PORT})')
    parser.add_argument('--client-id', type=str, default=None,
                        help='MQTT 클라이언트 ID (기본값: 자동 생성)')
    parser.add_argument('--debug', action='store_true', 
                       help='디버그 모드 활성화 (상세 로깅)')
    parser.add_argument('--auto-fallback', action='store_true',
                       help='연결 실패 시 공용 브로커로 자동 대체')
    return parser.parse_args()

# 테스트를 위한 간단한 상태 발행 함수
def run_simple_test():
    """MQTT 연결과 발행이 작동하는지 테스트"""
    if not mqtt_connected:
        print("MQTT 연결이 설정되지 않았습니다.")
        return False
        
    try:
        # 테스트 메시지
        test_payload = {
            "device_id": DEVICE_ID,
            "test_message": "MQTT 테스트 메시지",
            "timestamp": int(time.time() * 1000)
        }
        
        # 테스트 토픽에 발행
        print("테스트 메시지 발행 중...")
        result = mqtt_client.publish("test/message", json.dumps(test_payload), qos=1)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print("✅ 테스트 메시지가 성공적으로 발행되었습니다.")
            return True
        else:
            print(f"❌ 테스트 메시지 발행 실패, 결과 코드: {result.rc}")
            return False
            
    except Exception as e:
        print(f"❌ 테스트 실행 오류: {e}")
        return False

def decide_direction(face, frame_width, center_tolerance=0.05):
    """
    얼굴 중심이 프레임 중앙에서 얼마나 떨어져 있는지에 따라 방향 결정
    - face: 얼굴 데이터(dict, face_box 포함)
    - frame_width: 카메라 프레임의 가로 픽셀 수
    - center_tolerance: 중앙 허용 오차 비율 (예: 0.05 = 5%)
    """
    x, y, w, h = face["face_box"]
    face_center_x = x + w / 2
    frame_center_x = frame_width / 2
    offset = (face_center_x - frame_center_x) / frame_width  # -0.5 ~ +0.5

    if abs(offset) < center_tolerance:
        return "center"
    elif offset < 0:
        return "left"
    else:
        return "right"

def get_system_ip():
    """시스템의 동적 IP 주소 가져오기"""
    try:
        # ifconfig 명령어 실행
        result = subprocess.run(['ifconfig'], capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            import re
            # 192.168.x.x 또는 10.x.x.x 형식의 IP 주소 찾기
            ip_pattern = r'inet (?:addr:)?(\d+\.\d+\.\d+\.\d+)'
            matches = re.findall(ip_pattern, result.stdout)
            
            for ip in matches:
                # localhost(127.0.0.1)가 아닌 IP 주소 반환
                if not ip.startswith('127.'):
                    print(f"감지된 시스템 IP: {ip}")
                    return ip
        
        # ifconfig 실패 시 대체 명령어 시도
        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            ips = result.stdout.strip().split()
            for ip in ips:
                if not ip.startswith('127.') and '.' in ip:
                    print(f"감지된 시스템 IP (hostname): {ip}")
                    return ip
                    
    except Exception as e:
        print(f"IP 주소 감지 실패: {e}")
    
    # 모든 방법 실패 시 기본값 반환
    print("IP 주소 감지 실패, 기본값 사용: localhost")
    return "localhost"

# 메인 함수 수정
if __name__ == "__main__":
    print("\n===== MQTT 통신 모듈 독립 실행 =====")
    
    # 명령행 인수 파싱
    args = parse_args()
    MQTT_BROKER = args.broker
    MQTT_PORT = args.port
    DEBUG_MODE = args.debug
    auto_fallback = args.auto_fallback
    
    if args.client_id:
        DEVICE_ID = args.client_id
        
    if DEBUG_MODE:
        print("디버그 모드 활성화됨")
    
    print(f"브로커 주소: {MQTT_BROKER}, 포트: {MQTT_PORT}")
    
    # 브로커 목록 (자동 대체 사용 시)
    fallback_brokers = []
    if auto_fallback:
        fallback_brokers = [
            ("test.mosquitto.org", 1883),
            ("broker.hivemq.com", 1883)
        ]
    
    # 연결 재시도 루프
    connected = False
    retry_count = 0
    
    while retry_count < MAX_RETRIES:
        if start_mqtt_thread():
            connected = True
            break
        
        retry_count += 1
        if retry_count < MAX_RETRIES:
            print(f"연결 재시도 중... ({retry_count}/{MAX_RETRIES})")
            time.sleep(RETRY_INTERVAL)
    
    # 기본 브로커 연결 실패, 자동 대체 활성화된 경우
    if not connected and auto_fallback and fallback_brokers:
        print("\n🔄 기본 브로커 연결 실패, 공용 브로커 시도...")
        
        for broker, port in fallback_brokers:
            print(f"\n대체 브로커 시도: {broker}:{port}")
            MQTT_BROKER = broker
            MQTT_PORT = port
            
            if start_mqtt_thread():
                connected = True
                print(f"✅ 대체 브로커 {broker}:{port}에 연결 성공!")
                break
                
            time.sleep(RETRY_INTERVAL)
    
    if not connected:
        print(f"\n❌ 최대 재시도 횟수({MAX_RETRIES})를 초과했습니다.")
        print("\n대체 브로커 옵션:")
        print("  1. public 브로커 사용: --broker test.mosquitto.org")
        print("  2. HiveMQ public 브로커: --broker broker.hivemq.com")
        print("  3. --auto-fallback 옵션 사용: 공용 브로커 자동 시도")
        print("\n서버 확인사항:")
        print("  1. Mosquitto 서비스 실행 여부: systemctl status mosquitto")
        print("  2. 방화벽 설정: ufw status / ufw allow 1883")
        print("  3. 설정 확인: /etc/mosquitto/mosquitto.conf에 'listener 1883'와 'allow_anonymous true' 포함 여부")
        sys.exit(1)
        
    # temp_and_face_detect 모듈이 없는 경우 간단한 테스트 실행
    if not TEMP_TEST_AVAILABLE and connected:
        print("\n⚠️ temp_and_face_detect 모듈을 찾을 수 없어 간단한 MQTT 테스트를 실행합니다...")
        run_simple_test()
    
    try:
        # 메시지 루프
        print("\nMQTT 통신 중... (종료: Ctrl+C)")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nMQTT 모듈 종료 중...")
        stop_mqtt_thread()
        print("MQTT 모듈 종료됨")