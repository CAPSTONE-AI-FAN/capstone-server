import paho.mqtt.client as mqtt
import json
import RPi.GPIO as GPIO
import time
import threading
from datetime import datetime

# GPIO 설정
GPIO.setwarnings(False)

# 스테퍼 모터 핀
IN1 = 12
IN2 = 16
IN3 = 20
IN4 = 21

# DC 모터 핀 (팬)
DC_MOTOR = 13
DC_IN1 = 19 
DC_IN2 = 26

# 스테퍼 모터 시퀀스
seq = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

# GPIO 초기화
GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)
GPIO.setup(DC_MOTOR, GPIO.OUT)
GPIO.setup(DC_IN1, GPIO.OUT)  
GPIO.setup(DC_IN2, GPIO.OUT)

GPIO.output(DC_IN1, GPIO.HIGH)
GPIO.output(DC_IN2, GPIO.LOW)

# PWM 설정
pwm = GPIO.PWM(DC_MOTOR, 1000)
pwm.start(0)

# MQTT 설정
MQTT_BROKER = "192.168.191.223"
MQTT_PORT = 1883
CLIENT_ID = "fan_controller_module"

# 기타 설정
AUTO_TEMP_THRESHOLD = 37.5  # 자동 모드 온도 임계값

# 전역 변수 (main에서 초기화됨)
fan_pwm = None

# 시스템 상태
class FanControlState:
    def __init__(self):
        self.is_rotating = False
        self.rotation_direction = None
        self.auto_mode = True
        self.fan_speed = 0
        self.current_angle = 0
        self.shutdown = False
        
state = FanControlState()

# 온도 센서 관련 함수 제거됨

def control_fan(speed_percent):
    """팬 속도 제어 (0-100%)"""
    global fan_pwm
    
    if speed_percent < 0:
        speed_percent = 0
    elif speed_percent > 100:
        speed_percent = 100
        
    state.fan_speed = speed_percent
    
    # PWM이 초기화되어 있는지 확인
    if fan_pwm:
        fan_pwm.ChangeDutyCycle(speed_percent)
        print(f"Fan speed set to: {speed_percent}%")
    else:
        print("PWM not initialized - cannot control fan")

def move_stepper(direction, steps=10, delay=0.002):
    """스테퍼 모터 회전 (각도 추적 개선 + 로그 최적화)"""
    # 연속 회전 중이 아닐 때만 상세 로그 출력
    if not state.is_rotating:
        print(f"Moving stepper: {direction}, steps: {steps}")
    
    for step_count in range(steps):
        if state.shutdown:
            break
        
        # 방향 로직 수정: 올바른 방향 매핑
        # left는 시계 반대 방향(CCW), right는 시계 방향(CW)
        for step in (seq if direction == 'left' else reversed(seq)):
            if state.shutdown:
                break
            GPIO.output([IN1, IN2, IN3, IN4], step)
            time.sleep(delay)
        
        # 각도 업데이트 (연속 회전 시에도 추적)
        if direction == "left":
            state.current_angle = (state.current_angle - 1) % 360  # 1도씩 감소
        elif direction == "right":
            state.current_angle = (state.current_angle + 1) % 360  # 1도씩 증가
    
    # 스테퍼 모터 정지 (전력 절약)
    GPIO.output([IN1, IN2, IN3, IN4], [0, 0, 0, 0])
    
    # 연속 회전이 아닌 경우에만 완료 로그 출력
    if not state.is_rotating:
        print(f"Stepper moved {direction} {steps} steps (current angle: {state.current_angle}°)")

def continuous_rotation():
    """연속 회전 스레드 함수 (안정화된 연속 회전)"""
    rotation_start_time = time.time()
    total_steps = 0
    max_rotation_time = 10.0  # 최대 10초간 연속 회전
    max_steps_per_session = 100  # 세션당 최대 스텝 수
    
    while state.is_rotating:
        try:
            direction = state.rotation_direction
            if direction in ["left", "right"]:
                # 안전 장치: 너무 오래 회전하거나 너무 많이 회전한 경우 정지
                current_time = time.time()
                if (current_time - rotation_start_time > max_rotation_time or 
                    total_steps > max_steps_per_session):
                    print(f"[SAFETY] Stopping rotation - time: {current_time - rotation_start_time:.1f}s, steps: {total_steps}")
                    break
                
                # 부드러운 연속 회전을 위한 작은 스텝
                move_stepper(direction, steps=2, delay=0.002)  # 2 스텝으로 더 부드럽게
                total_steps += 2
                
                # 적응적 대기 시간 (회전 시간에 따라 조절)
                if total_steps < 20:
                    time.sleep(0.05)  # 초기에는 빠르게
                elif total_steps < 50:
                    time.sleep(0.08)  # 중간에는 보통
                else:
                    time.sleep(0.12)  # 나중에는 천천히
            else:
                print(f"Invalid rotation direction: {direction}")
                break
        except Exception as e:
            print(f"Continuous rotation error: {e}")
            break
    
    print(f"Continuous rotation stopped (total steps: {total_steps}, duration: {time.time() - rotation_start_time:.1f}s)")
    state.is_rotating = False

def start_rotation(direction):
    """회전 시작"""
    if state.is_rotating:
        stop_rotation()
        time.sleep(0.1)
    
    state.is_rotating = True
    state.rotation_direction = direction
    
    # 별도 스레드에서 연속 회전 시작
    rotation_thread = threading.Thread(target=continuous_rotation)
    rotation_thread.daemon = True
    rotation_thread.start()

def stop_rotation():
    """회전 정지"""
    print("Stopping rotation")
    state.is_rotating = False
    state.rotation_direction = None
    time.sleep(0.1)  # 정지 시간 확보

def publish_status(client):
    """상태 정보 발행"""
    status_data = {
        "device_id": CLIENT_ID,
        "status": "connected",
        "timestamp": int(time.time() * 1000),
        "data": {
            "is_rotating": state.is_rotating,
            "rotation_direction": state.rotation_direction,
            "fan_speed": state.fan_speed,
            "auto_mode": state.auto_mode,
            "current_angle": state.current_angle
        }
    }
    
    try:
        client.publish("device/status", json.dumps(status_data), qos=1)
    except Exception as e:
        print(f"Error publishing status: {e}")

def handle_command(client, data):
    """서버 명령 처리"""
    if data.get("target_device") != CLIENT_ID:
        return
        
    action = data.get("action")
    params = data.get("params", {})
    
    print(f"Received command: {action} with params: {params}")
    
    if action == "rotate_fan":
        # 팬 회전 명령 (자동 모드)
        control_fan(70)
        start_rotation("right")
        
    elif action == "stop_rotation":
        # 회전 정지
        stop_rotation()
        
    elif action == "resume_temp_measurement":
        # 온도 측정 재개 (팬 정지)
        control_fan(0)
        stop_rotation()
        
    elif action == "set_rotation":
        # 특정 각도로 회전
        angle = params.get("angle", 0)
        steps = abs(angle) // 2  # 대략적인 스텝 계산
        direction = "right" if angle > 0 else "left"
        move_stepper(direction, steps)
        
    elif action == "set_direction":
        # 방향 설정
        direction = params.get("direction")
        if direction in ["left", "right"]:
            start_rotation(direction)
            
    elif action == "set_mode":
        # 모드 설정
        mode = params.get("mode")
        if mode == "enable_autonomous":
            state.auto_mode = True
        else:
            state.auto_mode = False
            stop_rotation()
            control_fan(0)

def handle_control_rotation(data):
    """회전 제어 명령 처리 (연속 회전 지원)"""
    device_id = data.get("device_id", "")
    
    # 모바일 클라이언트에서 온 명령만 처리
    if not device_id.startswith("mobile_client_"):
        return
    
    # 각도 값 확인
    angle = data.get("angle", 0)
    
    print(f"Manual rotation command from {device_id}: angle {angle}")
    
    if angle == 0:
        # 각도가 0이면 회전 정지
        if state.is_rotating:
            print("Stopping rotation (angle = 0)")
            stop_rotation()
        return
    
    # 각도에 따른 방향 결정
    if angle < 0:
        direction = "left"
        abs_angle = abs(angle)
    else:
        direction = "right"
        abs_angle = angle
    
    # 작은 각도(10도 미만)면 단일 스텝 이동
    if abs_angle < 10:
        steps = max(1, abs_angle // 2)  # 최소 1 스텝
        if state.is_rotating:
            stop_rotation()
            time.sleep(0.1)
        
        move_stepper(direction, steps=steps)
        print(f"Manual single step: {direction}, {steps} steps")
    else:
        # 큰 각도면 연속 회전 시작
        if not state.is_rotating or state.rotation_direction != direction:
            print(f"Starting manual continuous rotation: {direction}")
            start_rotation(direction)
        else:
            print(f"Already rotating {direction} manually")
    
    # 상태 발행
    publish_status(client)

def handle_control_direction(data):
    """방향 제어 명령 처리 (연속 회전 방식으로 개선)"""
    try:
        direction = data.get("direction", "").lower()
        source = data.get("source", "unknown")
        
        if direction not in ["left", "right", "center"]:
            print(f"Invalid direction: {direction}")
            return
        
        print(f"Processing direction command from {source}: {direction}")
        
        # 중복 명령 방지 (서버 명령에 대해서만, 연속 회전 중인 경우 제외)
        current_time = time.time()
        if hasattr(handle_control_direction, 'last_server_command'):
            last_cmd, last_time = handle_control_direction.last_server_command
            if source == "server_controller" and last_cmd == direction and (current_time - last_time) < 2.0:
                # 연속 회전 중이고 같은 방향이면 무시하지 않음
                if not (state.is_rotating and state.rotation_direction == direction):
                    print(f"Ignoring duplicate server command: {direction} (within 2 seconds)")
                    return
        
        # 명령 처리 - 연속 회전 방식
        if direction == "center":
            # 중앙 정렬: 현재 회전을 멈추고 중앙을 향해 단일 이동
            if state.is_rotating:
                print("Stopping rotation for center alignment")
                stop_rotation()
                time.sleep(0.1)  # 잠시 대기
            
            # 중앙 정렬을 위한 단일 스텝 이동
            center_steps = 10
            
            # 현재 각도가 180도 이상이면 왼쪽으로, 미만이면 오른쪽으로
            if state.current_angle >= 180:
                move_stepper("left", steps=center_steps)
                state.current_angle = max(0, state.current_angle - center_steps)
                print(f"Centered with left {center_steps} steps")
            else:
                move_stepper("right", steps=center_steps)  
                state.current_angle = min(360, state.current_angle + center_steps)
                print(f"Centered with right {center_steps} steps")
                
        elif direction in ["left", "right"]:
            # 연속 회전 시작/계속
            if not state.is_rotating or state.rotation_direction != direction:
                # 현재 회전 중이 아니거나 다른 방향으로 회전 중이면 새로 시작
                if state.is_rotating:
                    print(f"Changing rotation direction from {state.rotation_direction} to {direction}")
                    stop_rotation()
                    time.sleep(0.1)
                
                print(f"Starting continuous rotation: {direction}")
                start_rotation(direction)
            else:
                print(f"Already rotating {direction} - continuing")
        
        # 서버 명령 기록 (중복 방지용)
        if source == "server_controller":
            handle_control_direction.last_server_command = (direction, current_time)
        
        # 상태 발행
        publish_status(client)
        
    except Exception as e:
        print(f"Handle control direction error: {e}")
        import traceback
        traceback.print_exc()

def handle_face_thermal_data(data):
    """얼굴 온도 데이터 처리 (연속 회전 방식, 자동 모드에서만)"""
    if not state.auto_mode:
        print("Manual mode - ignoring face thermal data")
        return

    faces = data.get("faces", [])
    print(f"Received face thermal data: {len(faces)} faces detected")

    if not faces:
        print("No faces detected - stopping fan and rotation")
        control_fan(0)
        if state.is_rotating:
            stop_rotation()
        return

    # 가장 뜨거운 얼굴 찾기 및 위치 기반 회전
    hottest_face = None
    max_temp = 0
    
    for face in faces:
        if face.get("temperature") is not None and face["temperature"] > max_temp:
            max_temp = face["temperature"]
            hottest_face = face

    if not hottest_face:
        print("No valid temperature data in faces")
        return

    print(f"Hottest face temperature: {hottest_face['temperature']:.1f}°C")

    # 팬 속도 제어 (온도 임계값)
    if hottest_face["temperature"] >= AUTO_TEMP_THRESHOLD:
        control_fan(70)
        
        # 가장 뜨거운 얼굴의 위치 기반 방향 결정
        if 'face_box' in hottest_face and len(hottest_face['face_box']) == 4:
            x1, y1, x2, y2 = hottest_face['face_box']
            center_x = (x1 + x2) / 2
            frame_width = 640  # 기본 프레임 너비
            normalized_x = center_x / frame_width
            
            # 방향 결정 (화면 3등분)
            if normalized_x < 0.33:
                direction = "left"
            elif normalized_x > 0.66:
                direction = "right"
            else:
                direction = "center"
            
            print(f"Temperature-based tracking: Face at {center_x:.1f} ({normalized_x:.3f}) -> {direction}")
            
            # 연속 회전 방식으로 방향에 따른 회전 실행
            if direction == "center":
                # 중앙에 있으면 회전 정지
                if state.is_rotating:
                    print(f"Hottest face centered - stopping rotation ({hottest_face['temperature']:.1f}°C)")
                    stop_rotation()
                else:
                    print(f"Hottest face already centered - no rotation needed")
            else:
                # 왼쪽이나 오른쪽에 있으면 연속 회전 시작
                if not state.is_rotating or state.rotation_direction != direction:
                    print(f"Starting continuous rotation {direction} toward hottest face ({hottest_face['temperature']:.1f}°C)")
                    start_rotation(direction)
                else:
                    print(f"Already rotating {direction} toward hottest face ({hottest_face['temperature']:.1f}°C)")
        else:
            print("No face position data available for temperature-based tracking")
    else:
        control_fan(0)
        if state.is_rotating:
            stop_rotation()
        print(f"Temperature normal ({hottest_face['temperature']:.1f}°C) - fan and rotation stopped")

def handle_face_detection_data(data):
    """직접 얼굴 인식 데이터 처리 (연속 회전 방식으로 개선 + 안정화)"""
    try:
        # 자동 모드가 아니면 무시
        if not state.auto_mode:
            return

        faces = data.get("faces", [])
        print(f"[DIRECT] Received face detection data: {len(faces)} faces detected")
        
        if not faces:
            print("[DIRECT] No faces detected - stopping rotation")
            # 얼굴이 없으면 회전 정지
            if state.is_rotating:
                stop_rotation()
            return

        # 디버그: 전체 데이터 구조 출력 (조건부)
        if len(faces) <= 2 and hasattr(handle_face_detection_data, 'debug_count'):
            if handle_face_detection_data.debug_count % 10 == 0:  # 10번에 1번만 출력
                print(f"[DEBUG] Full data structure: {json.dumps(data, indent=2)}")
        
        if not hasattr(handle_face_detection_data, 'debug_count'):
            handle_face_detection_data.debug_count = 0
        handle_face_detection_data.debug_count += 1

        # 가장 큰 얼굴 찾기 (더 안정적인 추적을 위해)
        largest_face = None
        largest_area = 0
        
        for i, face in enumerate(faces):
            face_box = face.get("face_box")
            if not face_box or len(face_box) != 4:
                continue
                
            print(f"[DEBUG] Processing face {i}, face_box: {face_box}")
            
            # face_box 형식 확인 및 처리
            x1, y1, x2, y2 = face_box
            
            # face_box가 [x, y, w, h] 형식인지 [x1, y1, x2, y2] 형식인지 판단
            if x2 > 640 or y2 > 720:  # 좌표값이 이미지 크기를 벗어나면 width/height 형식
                # [x, y, w, h] 형식으로 처리
                x, y, w, h = x1, y1, x2, y2
                x2, y2 = x + w, y + h
                print(f"[DEBUG] Detected [x,y,w,h] format: x={x}, y={y}, w={w}, h={h}")
            else:
                # [x1, y1, x2, y2] 형식으로 처리
                print(f"[DEBUG] Detected [x1,y1,x2,y2] format: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
            
            # 면적 계산
            area = (x2 - x1) * (y2 - y1)
            print(f"[DEBUG] Face {i}: box=[{x1},{y1},{x2},{y2}], area={area}")
            
            if area > largest_area and area > 1000:  # 최소 면적 체크
                largest_area = area
                largest_face = face
                print(f"[DEBUG] Face {i} selected as largest (area: {area})")

        if not largest_face:
            print("[DIRECT] No valid faces found")
            return

        # 가장 큰 얼굴의 중심점 계산
        face_box = largest_face.get("face_box")
        x1, y1, x2, y2 = face_box
        
        # face_box 형식 재확인
        if x2 > 640 or y2 > 720:  # [x, y, w, h] 형식
            x, y, w, h = x1, y1, x2, y2
            face_center_x = x + w/2
        else:  # [x1, y1, x2, y2] 형식
            face_center_x = (x1 + x2) / 2
        
        # 프레임 너비 (고정값 사용 - 일반적인 해상도)
        frame_width = 640  # 또는 데이터에서 가져오기
        
        # 정규화된 위치 계산 (0.0 ~ 1.0)
        normalized_position = face_center_x / frame_width
        
        # 방향 결정 - 더 관대한 중앙 범위 설정 (0.35 ~ 0.65)
        center_min = 0.35  # 기존 0.33에서 완화
        center_max = 0.65  # 기존 0.66에서 완화
        
        if center_min <= normalized_position <= center_max:
            direction = "center"
        elif normalized_position < center_min:
            direction = "left"
        else:
            direction = "right"
        
        print(f"[DIRECT] Face center: {face_center_x}, normalized: {normalized_position:.3f}, direction: {direction}")
        
        # 연속 회전 중에는 더 신중하게 처리 (안정화 로직)
        if state.is_rotating:
            # 같은 방향으로 이미 회전 중이면 일정 시간 후에만 재평가
            current_time = time.time()
            if not hasattr(handle_face_detection_data, 'last_direction_time'):
                handle_face_detection_data.last_direction_time = current_time
                handle_face_detection_data.last_detected_direction = direction
            
            time_since_last = current_time - handle_face_detection_data.last_direction_time
            
            # 같은 방향으로 회전 중인데 또 같은 방향이 감지되면 3초 이상 경과했을 때만 계속
            if (state.rotation_direction == direction and time_since_last < 3.0):
                print(f"[DIRECT] Already rotating {direction}, waiting for stabilization ({time_since_last:.1f}s)")
                return
            
            # 방향이 바뀌었거나 충분한 시간이 경과했으면 업데이트
            handle_face_detection_data.last_direction_time = current_time
            handle_face_detection_data.last_detected_direction = direction
        
        # 연속 회전 방식으로 동작
        if direction == "center":
            # 중앙에 있으면 회전 정지
            if state.is_rotating:
                print("[DIRECT] Face centered - stopping rotation")
                stop_rotation()
            else:
                print("[DIRECT] Face already centered - no movement needed")
        else:
            # 왼쪽이나 오른쪽에 있으면 연속 회전 시작
            if not state.is_rotating or state.rotation_direction != direction:
                print(f"[DIRECT] Starting continuous rotation: {direction}")
                start_rotation(direction)
            else:
                print(f"[DIRECT] Already rotating {direction} - continuing")
        
        # 온도 기반 팬 제어
        temperature = largest_face.get("temperature")
        if temperature and temperature >= AUTO_TEMP_THRESHOLD:
            control_fan(70)
        else:
            control_fan(0)
            
    except Exception as e:
        print(f"Handle face detection data error: {e}")
        import traceback
        traceback.print_exc()

def handle_control_auto_mode(data):
    """자동 모드 제어"""
    device_id = data.get("device_id", "")
    
    if not device_id.startswith("mobile_client_"):
        return
        
    mode = data.get("mode")
    
    if mode == "enable_autonomous":
        state.auto_mode = True
        print("Auto mode enabled")
    else:
        state.auto_mode = False
        stop_rotation()
        control_fan(0)
        print("Manual mode enabled")

def on_connect(client, userdata, flags, rc):
    """MQTT 연결 성공 시 콜백 (개선된 버전)"""
    if rc == 0:
        print("Connected to MQTT broker successfully")
        
        # 토픽 구독
        topics_to_subscribe = [
            "device/command",
            "control/rotation", 
            "control/direction",
            "control/auto_mode",
            "sensor/face/thermal",
            "sensor/face/data"
        ]
        
        for topic in topics_to_subscribe:
            result = client.subscribe(topic, qos=1)
            if result[0] == mqtt.MQTT_ERR_SUCCESS:
                print(f"Successfully subscribed to: {topic}")
            else:
                print(f"Failed to subscribe to: {topic}, error: {result[0]}")
        
        # 연결 상태 발행
        publish_status(client)
        
    else:
        error_messages = {
            1: "Connection refused - incorrect protocol version",
            2: "Connection refused - invalid client identifier", 
            3: "Connection refused - server unavailable",
            4: "Connection refused - bad username or password",
            5: "Connection refused - not authorised"
        }
        print(f"Failed to connect to MQTT broker: {error_messages.get(rc, f'Unknown error code: {rc}')}")

def on_message(client, userdata, msg):
    """MQTT 메시지 수신 처리 (개선된 버전)"""
    try:
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        data = json.loads(payload)
        
        # 장치 ID 및 소스 확인
        device_id = data.get("device_id", "unknown")
        
        # source 필드가 없으면 device_id에서 추출
        if "source" not in data:
            if device_id == "server_controller":
                data["source"] = "server_controller"
            elif device_id.startswith("mobile_client_"):
                data["source"] = "mobile_client"
            elif device_id.startswith("thermal_face_"):
                data["source"] = "face_detection"
            else:
                data["source"] = device_id
        
        print(f"Message received - Topic: {topic} - Device: {device_id}")
        
        # 토픽별 처리
        if topic == "device/command":
            handle_command(client, data)
        elif topic == "control/rotation":
            handle_control_rotation(data)
        elif topic == "control/direction":
            # 권한 확인 추가
            source = data.get("source", "unknown")
            if source in ["server_controller", "mobile_client"]:
                handle_control_direction(data)
            else:
                print(f"Ignoring direction command from unauthorized source: {source}")
        elif topic == "sensor/face/thermal":
            handle_face_thermal_data(data)
        elif topic == "sensor/face/data":
            # face detection 데이터는 항상 처리 (source 상관없이)
            handle_face_detection_data(data)
        elif topic == "control/auto_mode":
            handle_control_auto_mode(data)
        else:
            print(f"Unknown topic: {topic}")
            
    except json.JSONDecodeError as e:
        print(f"JSON decode error: {e}")
    except Exception as e:
        print(f"Message processing error: {e}")
        import traceback
        traceback.print_exc()

def test_stepper_directions():
    """스테퍼 모터 방향 테스트 함수"""
    print("\n=== 스테퍼 모터 방향 테스트 시작 ===")
    
    print("1. 오른쪽으로 20스텝 회전 (5초 후 시작)")
    time.sleep(5)
    move_stepper("right", steps=20, delay=0.005)
    
    print("2. 3초 대기...")
    time.sleep(3)
    
    print("3. 왼쪽으로 20스텝 회전")
    move_stepper("left", steps=20, delay=0.005)
    
    print("4. 3초 대기...")
    time.sleep(3)
    
    print("5. 오른쪽으로 10스텝 회전")
    move_stepper("right", steps=10, delay=0.005)
    
    print("=== 스테퍼 모터 방향 테스트 완료 ===\n")
    print("관찰 결과:")
    print("- 첫 번째 회전이 실제로 오른쪽이었다면 방향이 올바릅니다")
    print("- 첫 번째 회전이 실제로 왼쪽이었다면 방향을 다시 바꿔야 합니다")

def main():
    """메인 함수 (개선된 버전)"""
    global client, fan_pwm
    
    # 변수 초기화 (오류 발생 시 정리를 위해)
    client = None
    fan_pwm = None
    
    print("Fan Control Module Starting...")
    
    try:
        # 기존 GPIO 상태 정리 (이전 실행의 잔여물 제거)
        try:
            GPIO.cleanup()
            print("Previous GPIO state cleaned up")
        except:
            pass  # GPIO가 이미 정리되었거나 초기화되지 않은 경우
        
        # GPIO 초기화
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)  # GPIO 경고 무시
        
        # 스테퍼 모터 및 DC 모터 핀 설정
        GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)
        GPIO.setup(DC_MOTOR, GPIO.OUT)
        GPIO.setup(DC_IN1, GPIO.OUT)  
        GPIO.setup(DC_IN2, GPIO.OUT)
        
        # 초기 상태 설정
        GPIO.output(DC_IN1, GPIO.HIGH)
        GPIO.output(DC_IN2, GPIO.LOW)
        
        # PWM 설정 (기존 PWM 객체가 있으면 정리 후 생성)
        try:
            fan_pwm = GPIO.PWM(DC_MOTOR, 1000)  # 1kHz
            fan_pwm.start(0)
            print("PWM initialized successfully")
        except RuntimeError as e:
            if "already exists" in str(e):
                print("PWM object already exists, cleaning up and retrying...")
                GPIO.cleanup()
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                
                # 핀 재설정
                GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)
                GPIO.setup(DC_MOTOR, GPIO.OUT)
                GPIO.setup(DC_IN1, GPIO.OUT)  
                GPIO.setup(DC_IN2, GPIO.OUT)
                GPIO.output(DC_IN1, GPIO.HIGH)
                GPIO.output(DC_IN2, GPIO.LOW)
                
                # PWM 재생성
                fan_pwm = GPIO.PWM(DC_MOTOR, 1000)
                fan_pwm.start(0)
                print("PWM reinitialized successfully after cleanup")
            else:
                raise e
        
        print("GPIO initialized successfully")
        
        # MQTT 클라이언트 설정
        client = mqtt.Client(client_id=CLIENT_ID, protocol=mqtt.MQTTv311, clean_session=True)
        client.on_connect = on_connect
        client.on_message = on_message
        
        # 브로커 연결
        print(f"Connecting to MQTT broker: {MQTT_BROKER}:{MQTT_PORT}")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        
        # 백그라운드에서 MQTT 루프 시작
        client.loop_start()
        
        # 상태 발행 스레드 시작
        def status_loop():
            while not state.shutdown:
                try:
                    if client and client.is_connected():
                        publish_status(client)
                    time.sleep(10)  # 10초마다 상태 발행
                except Exception as e:
                    print(f"Status loop error: {e}")
                    time.sleep(5)
        
        status_thread = threading.Thread(target=status_loop, daemon=True)
        status_thread.start()
        
        print("Fan control module started successfully")
        print("Press Ctrl+C to stop...")
        
        # 메인 루프
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down...")
            state.shutdown = True
            
    except Exception as e:
        print(f"Initialization error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 정리 (모든 변수가 정의되었는지 확인)
        print("Cleaning up...")
        
        # 회전 중지
        try:
            print("Stopping rotation")
            stop_rotation()
            print("Rotation stopped")
        except:
            pass
        
        # 팬 중지
        try:
            control_fan(0)
            print("Fan speed set to: 0%")
        except:
            pass
        
        # PWM 정리
        if fan_pwm:
            try:
                fan_pwm.stop()
                print("PWM stopped")
            except:
                pass
        
        # MQTT 연결 종료
        if client:
            try:
                client.loop_stop()
                client.disconnect()
                print("MQTT disconnected")
            except:
                pass
        
        # GPIO 정리
        try:
            GPIO.cleanup()
            print("GPIO cleaned up")
        except:
            pass
        
        print("Fan control module stopped")

if __name__ == "__main__":
    import sys
    
    # 명령행 인자 확인
    if len(sys.argv) > 1 and sys.argv[1] == "--test-direction":
        print("스테퍼 모터 방향 테스트 모드")
        try:
            test_stepper_directions()
        finally:
            GPIO.cleanup()
    else:
        main()