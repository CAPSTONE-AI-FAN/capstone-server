#!/usr/bin/env python3
"""
IoT 선풍기 제어 시스템 - 라즈베리파이 모듈

이 스크립트는 라즈베리파이를 사용하여 다음 기능을 수행합니다:
1. I2C 온도 센서를 동적으로 감지하고 체온 측정
2. MQTT를 통해 연산 서버와 통신
3. GPIO 핀을 사용하여 선풍기 모터 제어

사용법: python3 fan_controller.py
"""

import time
import json
import sys
import subprocess
import re
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from smbus2 import SMBus

# ===== GPIO 설정 =====
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  # 경고 메시지 비활성화
MOTOR_PIN_1 = 17  # 모터 제어 핀 1
MOTOR_PIN_2 = 18  # 모터 제어 핀 2
GPIO.setup(MOTOR_PIN_1, GPIO.OUT)
GPIO.setup(MOTOR_PIN_2, GPIO.OUT)

# ===== MQTT 설정 =====
MQTT_BROKER = "192.168.0.10"  # 연산 서버의 IP 주소로 변경 필요
MQTT_PORT = 1883
MQTT_TEMP_TOPIC = "fan/temperature"  # 온도 데이터 전송 토픽
MQTT_COMMAND_TOPIC = "fan/command"    # 명령 수신 토픽
MQTT_STATUS_TOPIC = "fan/status"      # 상태 보고 토픽

# ===== 상태 변수 =====
measuring_temperature = True
motor_running = False

# ===== 라즈베리파이 ID (고유 식별자) =====
DEVICE_ID = "fan_control_module"

# ===== I2C 설정 =====
bus = SMBus(1)  # I2C 버스 1 사용

# ===== 함수 정의 =====

def detect_i2c_devices():
    """i2cdetect 명령을 실행하고 발견된 장치 주소 목록을 반환합니다."""
    try:
        # i2cdetect 명령 실행
        result = subprocess.run(['i2cdetect', '-y', '1'], capture_output=True, text=True)
        output = result.stdout
        
        # 정규 표현식을 사용하여 장치 주소 추출
        # 두 자리 16진수만 추출
        addresses = re.findall(r'\s([0-9a-f]{2})\s', output)
        
        # 16진수 주소를 정수로 변환
        devices = [int(addr, 16) for addr in addresses if addr != '--']
        
        # 중복 제거
        devices = list(set(devices))
        
        print(f"발견된 I2C 장치 주소: {[hex(addr) for addr in devices]}")
        return devices
    except Exception as e:
        print(f"I2C 장치 감지 중 오류 발생: {e}")
        # 기본적으로 빈 리스트 반환
        return []

def build_temp_registers_map(devices):
    """발견된 장치를 기반으로 온도 레지스터 매핑을 구성합니다."""
    # 알려진 온도 센서 및 레지스터 매핑 (확장 가능)
    known_sensors = {
        0x5A: 0x07,  # MLX90614 - 물체 온도 레지스터
        0x30: [0x00, 0x01, 0x02, 0x03, 0x04, 0x05],  # 여러 레지스터 시도
        0x38: [0x00, 0x01, 0x02, 0x03, 0x04, 0x05],  # 여러 레지스터 시도
        0x45: [0x00, 0x01, 0x02, 0x03, 0x04, 0x05],  # 여러 레지스터 시도
    }
    
    temp_registers = {}
    
    for addr in devices:
        if addr in known_sensors:
            # 알려진 센서라면 알려진 레지스터 값 사용
            temp_registers[addr] = known_sensors[addr]
        else:
            # 알려지지 않은 센서는 일반적인 레지스터 시도
            # 여러 레지스터를 시도해 볼 수 있도록 리스트 사용
            temp_registers[addr] = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
    
    print("온도 레지스터 매핑:")
    for addr, reg in temp_registers.items():
        print(f"  장치 0x{addr:02x}: 레지스터 {reg}")
    
    return temp_registers

def scan_i2c_bus():
    """I2C 버스를 스캔하고 사용 가능한 장치 주소 목록 반환"""
    devices = []
    for addr in range(0x03, 0x78):  # 유효한 I2C 주소 범위는 0x03~0x77
        try:
            bus.read_byte(addr)
            devices.append(addr)
            print(f"I2C 장치 발견 주소: 0x{addr:02x}")
        except Exception:
            pass
    return devices

def try_read_temperature_mlx(address, register):
    """MLX90614 스타일 온도 센서에서 온도 읽기 시도"""
    try:
        # 온도 데이터 읽기 (2바이트)
        data_low = bus.read_byte_data(address, register)
        data_high = bus.read_byte_data(address, register + 1)
        
        # 두 바이트 결합
        data = (data_high << 8) | data_low
        
        # 섭씨 온도로 변환 (MLX90614 공식)
        temp_c = (data * 0.02) - 273.15
        
        # 기본 유효성 검사 - 온도는 합리적인 범위여야 함
        if -50 <= temp_c <= 150:
            return temp_c
        return None
    except Exception as e:
        return None

def try_read_temperature_direct(address):
    """직접 레지스터에서 값을 읽고 온도로 해석 시도"""
    try:
        # 장치에서 직접 바이트 읽기
        raw_value = bus.read_byte(address)
        
        # 온도 범위 내에 있다면 센서일 가능성이 있음
        if 0 <= raw_value <= 100:  # 0°C ~ 100°C 범위 가정
            return float(raw_value)
        return None
    except Exception:
        return None

def try_various_read_methods(address, registers):
    """다양한 방법으로 온도 읽기 시도"""
    print(f"장치 0x{address:02x}에서 온도 읽기 시도 중...")
    
    # 1. 직접 읽기 시도
    temp = try_read_temperature_direct(address)
    if temp is not None:
        print(f"  직접 읽기 성공: {temp:.2f}°C")
        return temp, "direct", None
    
    # 2. 레지스터가 리스트인 경우 (여러 레지스터 시도)
    if isinstance(registers, list):
        for reg in registers:
            try:
                # 단일 바이트 읽기
                value = bus.read_byte_data(address, reg)
                if 0 <= value <= 100:  # 기본 온도 범위 확인
                    print(f"  레지스터 0x{reg:02x}에서 값 {value} 읽기 성공")
                    return float(value), "single", reg
                
                # MLX 스타일 읽기 시도
                temp = try_read_temperature_mlx(address, reg)
                if temp is not None:
                    print(f"  MLX 스타일 읽기 성공(레지스터 0x{reg:02x}): {temp:.2f}°C")
                    return temp, "mlx", reg
            except Exception as e:
                continue
    
    # 3. 레지스터가 단일 값인 경우
    else:
        try:
            temp = try_read_temperature_mlx(address, registers)
            if temp is not None:
                print(f"  MLX 스타일 읽기 성공(레지스터 0x{registers:02x}): {temp:.2f}°C")
                return temp, "mlx", registers
        except Exception:
            pass
    
    return None, None, None

def discover_temperature_sensor():
    """온도 센서로 사용 가능한 I2C 장치 찾기"""
    # 두 가지 방법으로 장치 스캔
    devices_method1 = detect_i2c_devices()
    devices_method2 = scan_i2c_bus()
    
    # 두 결과 결합 및 중복 제거
    devices = list(set(devices_method1 + devices_method2))
    
    print(f"{len(devices)}개의 I2C 장치 발견")
    
    if not devices:
        print("I2C 장치를 찾을 수 없습니다!")
        return None, None, None
    
    # 레지스터 매핑 구성
    temp_registers = build_temp_registers_map(devices)
    
    # 각 장치가 온도를 읽을 수 있는지 시도
    for addr in devices:
        registers = temp_registers.get(addr, [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07])
        
        temp, method, reg = try_various_read_methods(addr, registers)
        
        if temp is not None:
            print(f"0x{addr:02x} 주소의 장치에서 온도 {temp:.2f}°C 읽기 성공 (방법: {method}, 레지스터: {reg})")
            return addr, method, reg
    
    print("적합한 온도 센서를 찾을 수 없습니다")
    
    # 시뮬레이션 모드 활성화
    print("온도 센서를 찾을 수 없습니다. 시뮬레이션 모드로 전환합니다.")
    return "sim", "sim", None

def measure_temperature_sim():
    """시뮬레이션된 온도 측정"""
    # 36.0 ~ 38.0 범위의 랜덤한 온도 생성
    import random
    temp = 36.0 + (random.random() * 2.0)
    print(f"시뮬레이션된 온도: {temp:.2f}°C")
    return temp

def measure_temperature():
    """발견된 센서 또는 시뮬레이션 방식을 사용하여 온도 측정"""
    global temp_sensor_addr, temp_sensor_method, temp_sensor_reg
    
    if temp_sensor_method == "sim":
        return measure_temperature_sim()
    
    try:
        if temp_sensor_method == "direct":
            # 직접 읽기
            raw_value = bus.read_byte(temp_sensor_addr)
            temp_c = float(raw_value)
            print(f"측정된 온도: {temp_c:.2f}°C")
            return temp_c
            
        elif temp_sensor_method == "single":
            # 단일 바이트 레지스터 읽기
            value = bus.read_byte_data(temp_sensor_addr, temp_sensor_reg)
            temp_c = float(value)
            print(f"측정된 온도: {temp_c:.2f}°C")
            return temp_c
            
        elif temp_sensor_method == "mlx":
            # MLX 스타일 읽기
            data_low = bus.read_byte_data(temp_sensor_addr, temp_sensor_reg)
            data_high = bus.read_byte_data(temp_sensor_addr, temp_sensor_reg + 1)
            data = (data_high << 8) | data_low
            temp_c = (data * 0.02) - 273.15
            print(f"측정된 온도: {temp_c:.2f}°C")
            return temp_c
        
        else:
            print(f"알 수 없는 측정 방법: {temp_sensor_method}")
            return None
            
    except Exception as e:
        print(f"온도 읽기 오류: {e}")
        return None

def rotate_fan():
    """선풍기 회전 시작"""
    print("선풍기 회전 시작")
    GPIO.output(MOTOR_PIN_1, GPIO.HIGH)
    GPIO.output(MOTOR_PIN_2, GPIO.LOW)
    
def stop_fan():
    """선풍기 회전 정지"""
    print("선풍기 회전 정지")
    GPIO.output(MOTOR_PIN_1, GPIO.LOW)
    GPIO.output(MOTOR_PIN_2, GPIO.LOW)

# ===== MQTT 콜백 함수 =====

def on_connect(client, userdata, flags, rc):
    """MQTT 브로커 연결 시 호출"""
    print(f"MQTT 브로커에 연결됨 (결과 코드: {rc})")
    client.subscribe(MQTT_COMMAND_TOPIC)
    
    # 연결 성공 시 상태 메시지 전송
    status_payload = json.dumps({
        "device_id": DEVICE_ID,
        "status": "connected",
        "capabilities": ["temperature_sensing", "fan_control"],
        "sensor_info": {
            "type": temp_sensor_method,
            "address": "simulation" if temp_sensor_addr == "sim" else f"0x{temp_sensor_addr:02x}",
            "register": "N/A" if temp_sensor_reg is None else f"0x{temp_sensor_reg:02x}"
        }
    })
    client.publish(MQTT_STATUS_TOPIC, status_payload)

def on_message(client, userdata, msg):
    """MQTT 메시지 수신 시 호출"""
    global measuring_temperature, motor_running
    
    print(f"메시지 수신: {msg.topic} - {msg.payload.decode()}")
    
    try:
        command = json.loads(msg.payload.decode())
        
        # 이 모듈에 대한 명령인지 확인
        if "target_device" in command and command["target_device"] != DEVICE_ID:
            return  # 다른 장치를 대상으로 하는 명령이면 무시
        
        if "action" in command:
            if command["action"] == "rotate_fan":
                measuring_temperature = False
                motor_running = True
                rotate_fan()
                
                # 상태 업데이트 전송
                status_payload = json.dumps({
                    "device_id": DEVICE_ID,
                    "status": "rotating"
                })
                client.publish(MQTT_STATUS_TOPIC, status_payload)
                
            elif command["action"] == "stop_rotation":
                motor_running = False
                stop_fan()
                
                # 상태 업데이트 전송
                status_payload = json.dumps({
                    "device_id": DEVICE_ID,
                    "status": "stopped"
                })
                client.publish(MQTT_STATUS_TOPIC, status_payload)
                
            elif command["action"] == "resume_temp_measurement":
                measuring_temperature = True
                motor_running = False
                stop_fan()
                
                # 상태 업데이트 전송
                status_payload = json.dumps({
                    "device_id": DEVICE_ID,
                    "status": "measuring"
                })
                client.publish(MQTT_STATUS_TOPIC, status_payload)
                
    except json.JSONDecodeError:
        print("잘못된 JSON 형식의 메시지")

# ===== 메인 로직 =====

# 온도 센서 찾기
print("온도 센서 감지 중...")
temp_sensor_addr, temp_sensor_method, temp_sensor_reg = discover_temperature_sensor()

if temp_sensor_addr is None:
    print("온도 센서를 찾을 수 없고 시뮬레이션도 불가합니다. 시스템을 종료합니다.")
    sys.exit(1)
else:
    if temp_sensor_method == "sim":
        print("시뮬레이션 모드가 활성화되었습니다. 실제 온도 센서가 없습니다.")
    else:
        print(f"주소 0x{temp_sensor_addr:02x}, 방법 {temp_sensor_method}, 레지스터 {temp_sensor_reg}의 온도 센서 사용")

# 테스트 온도 읽기
test_temp = measure_temperature()
if test_temp is not None:
    print(f"테스트 읽기 - 온도: {test_temp:.2f}°C")
else:
    print("온도 읽기 실패. 시스템을 종료합니다.")
    sys.exit(1)

# MQTT 클라이언트 설정
client = mqtt.Client(client_id=DEVICE_ID)
client.on_connect = on_connect
client.on_message = on_message

# MQTT 브로커에 연결
try:
    print(f"MQTT 브로커 {MQTT_BROKER}:{MQTT_PORT}에 연결 시도 중...")
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()
except Exception as e:
    print(f"MQTT 브로커({MQTT_BROKER}:{MQTT_PORT})에 연결할 수 없습니다: {e}")
    print("MQTT 연결 없이 계속 진행합니다. 로컬 모드로 작동합니다.")

# 메인 루프
try:
    print("IoT 선풍기 제어 시스템 시작...")
    while True:
        if measuring_temperature:
            # 체온 측정
            temp = measure_temperature()
            if temp is not None:
                # 측정된 온도 데이터를 서버로 전송
                payload = json.dumps({
                    "device_id": DEVICE_ID,
                    "temperature": temp,
                    "timestamp": time.time()
                })
                try:
                    client.publish(MQTT_TEMP_TOPIC, payload)
                except Exception as e:
                    print(f"MQTT 메시지 전송 실패: {e}")
            
            time.sleep(2)  # 2초마다 온도 측정
                
        elif motor_running:
            # 단순히 모터 회전 상태 유지
            time.sleep(0.5)
        
        else:
            time.sleep(0.5)
            
except KeyboardInterrupt:
    print("프로그램 종료")

finally:
    # 정리
    try:
        client.loop_stop()
        client.disconnect()
    except:
        pass
    GPIO.cleanup()
    print("자원 정리 완료")
