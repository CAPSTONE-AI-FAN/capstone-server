# 🚀 AI FAN 시스템 MQTT 통신 테스트 가이드

## 📋 시스템 구성

이 테스트 환경은 실제 센서 없이 MQTT 메시지 통신을 시뮬레이션합니다.

### 🏗️ 아키텍처

```
Flutter 앱 (Mobile Controller)
    ↕️ MQTT
중앙 서버 (Node.js - MQTT Broker & Controller)
    ↕️ MQTT
Ubuntu Pi (LIDAR + 초음파 + 바퀴)    Raspbian Pi (카메라 + 온도 + 선풍기)
```

### 📱 각 컴포넌트 역할

| 컴포넌트        | 역할                           | 토픽 구독                                                        | 토픽 발행                                                        |
| --------------- | ------------------------------ | ---------------------------------------------------------------- | ---------------------------------------------------------------- |
| **Flutter 앱**  | 자율주행 ON/OFF 제어           | `system/status`, `device/status`                                 | `control/auto_mode`, `control/direction`                         |
| **Ubuntu Pi**   | LIDAR/초음파 센서, 바퀴 제어   | `control/auto_mode`, `control/movement`                          | `sensor/lidar/data`, `sensor/ultrasonic/data`, `movement/status` |
| **Raspbian Pi** | 얼굴인식/온도센서, 선풍기 제어 | `control/auto_mode`, `control/direction`, `control/fan/rotation` | `sensor/face/thermal`, `sensor/temperature/data`, `fan/status`   |
| **중앙 서버**   | 데이터 분석 및 명령 발행       | 모든 센서 데이터                                                 | `control/movement`, `control/fan/rotation`                       |

## 🛠️ 설치 및 설정

### 1. 의존성 설치

```bash
# Python 패키지 설치 (Ubuntu/Raspbian 공통)
./install_requirements.sh

# 또는 수동 설치
pip3 install paho-mqtt numpy opencv-python
```

### 2. Node.js 서버 설정

```bash
cd aifan-server
npm install
```

### 3. Flutter 앱 설정

```bash
cd AI_FAN
flutter pub get
```

## 🚀 테스트 실행 순서

### 1️⃣ MQTT 브로커 서버 시작

```bash
cd aifan-server
node app.js
```

**확인사항:**

- ✅ `MQTT broker is running on port 1883`
- ✅ `MQTT-WS broker is running on port 8883`
- ✅ `API server running on port 3000`

### 2️⃣ Ubuntu Pi 시뮬레이터 실행

#### 방법 A: 기본 MQTT 시뮬레이션

```bash
python3 ubuntu_pi_test.py
```

**입력:** MQTT 브로커 IP (기본값: 192.168.0.8)

#### 방법 B: ROS2-MQTT 브리지 (추천)

```bash
# ROS2 환경 설정 (Ubuntu에서)
source /opt/ros/humble/setup.bash

# 의존성 설치
pip3 install -r requirements_ros2.txt

# ROS2-MQTT 브리지 실행
python3 ubuntu_pi_ros2_mqtt_bridge.py
```

**입력:**

1. MQTT 브로커 IP (기본값: 192.168.0.8)
2. 실행 모드 선택 (1: 통합 테스트, 2: 브리지만, 3: 더미 LiDAR만)

**확인사항:**

- ✅ `✅ MQTT 브로커 연결 성공!` 또는 `✅ MQTT 브로커에 연결됨`
- ✅ 구독 토픽 설정 완료
- ✅ (ROS2 브리지) `📊 스캔 데이터 → MQTT 발행`

### 3️⃣ Raspbian Pi 시뮬레이터 실행

```bash
python3 raspbian_pi_test.py
```

**입력:** MQTT 브로커 IP (기본값: 192.168.0.8)

**확인사항:**

- ✅ `✅ MQTT 브로커 연결 성공!`
- ✅ 구독 토픽 설정 완료

### 4️⃣ Flutter 앱 실행

```bash
cd AI_FAN
flutter run
```

**확인사항:**

- ✅ 앱 실행 후 IP 입력 화면 나타남
- ✅ MQTT 브로커 IP 입력 (예: 192.168.0.8)
- ✅ 연결 상태 표시

## 🧪 테스트 시나리오

### 시나리오 1: 자율주행 모드 활성화

1. **Flutter 앱**에서 "자동 모드로 전환" 버튼 클릭
2. **예상 결과:**
   - 서버 로그: `자율주행 모드 활성화!`
   - Ubuntu Pi: `🚗 자율주행 모드 활성화!` + `🎯 센서 시뮬레이션 시작`
   - Raspbian Pi: `🤖 자율주행 모드 활성화!` + `📷 센서 시뮬레이션 시작`

### 시나리오 2: LIDAR 센서 데이터 흐름

1. 자율주행 모드 활성화 후 대기
2. **Ubuntu Pi 로그 확인:**
   ```
   🔄 LIDAR 360도 스캔 완료
   📨 메시지 수신: sensor/lidar/data
   ```
3. **서버 로그 확인:**
   ```
   LIDAR data accepted from device: ubuntu_pi_lidar_module
   ```

### 시나리오 3: 얼굴 인식 및 온도 기반 선풍기 제어

1. 자율주행 모드 활성화 후 대기
2. **Raspbian Pi 로그 확인:**
   ```
   👥 감지된 얼굴: 2명
     - person_1: 37.8°C (high)
     - person_2: 36.9°C (medium)
   ```
3. **서버 로그 확인:**
   ```
   Processing face thermal data: 2 faces detected
   🔥 고온 감지: 37.8°C
   💨 고온으로 인한 선풍기 가동 명령 발행
   ```
4. **Raspbian Pi에서 선풍기 제어 확인:**
   ```
   📨 메시지 수신: control/fan/rotation
   💨 선풍기 회전 시작: 속도 70%
   ```

### 시나리오 4: 장애물 감지 및 정지

1. 자율주행 모드에서 초음파 센서가 장애물 감지
2. **Ubuntu Pi 로그:**
   ```
   ⚠️ 장애물 감지: front 방향 30.5cm
   ```
3. **서버 로그:**
   ```
   ⚠️ 장애물 감지: front 방향 30.5cm
   🛑 장애물로 인한 정지 명령 발행
   ```
4. **Ubuntu Pi 로그:**
   ```
   📨 메시지 수신: control/movement
   ⏹️ 이동 중지
   ```

### 시나리오 5: Flutter 앱 수동 제어

1. 자동 모드 비활성화
2. 방향 컨트롤러 또는 회전 조이스틱 사용
3. **확인 로그:**
   - Flutter: 메시지 발행
   - 서버: 메시지 수신 및 처리
   - 해당 Pi: 제어 명령 수신

## 📊 모니터링 포인트

### 🔍 서버 로그에서 확인할 내용

```bash
# 연결 상태
✅ MQTT 브로커 연결 성공!
📨 메시지 수신: [토픽명]

# 데이터 처리
LIDAR data accepted from device: ubuntu_pi_lidar_module
Face thermal data accepted from device: raspbian_pi_face_module
🔥 고온 감지: 37.8°C
⚠️ 장애물 감지: front 방향 30.5cm

# 명령 발행
💨 고온으로 인한 선풍기 가동 명령 발행
🛑 장애물로 인한 정지 명령 발행
```

### 📱 Flutter 앱에서 확인할 내용

- 연결 상태: `WebSocket 연결됨` 또는 `연결됨`
- 시스템 상태: `온도 측정 중`, `팬 회전 중` 등
- 메뉴에서 디바이스 연결 상태 확인

### 🐧 Ubuntu Pi에서 확인할 내용

```bash
🤖 자율주행 모드: ON | 이동: false | 방향: stop
🔄 LIDAR 360도 스캔 완료
⚠️ 장애물 감지: front 방향 45.2cm
🚀 이동 시작: forward, 속도: 50%
```

### 🍓 Raspbian Pi에서 확인할 내용

```bash
🤖 자율주행 모드: ON | 카메라: true | 선풍기: true (left)
👥 감지된 얼굴: 1명
💨 선풍기 회전 시작: 속도 70%
🔄 선풍기 방향 변경: center → left (대상: person_1)
```

## 🐛 트러블슈팅

### 연결 문제

```bash
# 1. 브로커 IP 확인
ping 192.168.0.8

# 2. 포트 확인
netstat -an | grep 1883

# 3. 방화벽 설정 확인 (Ubuntu)
sudo ufw status
sudo ufw allow 1883
sudo ufw allow 8883
sudo ufw allow 3000
```

### 메시지가 전달되지 않는 경우

1. **클라이언트 ID 중복 확인:** 각 시뮬레이터가 고유 ID 사용하는지 확인
2. **토픽 이름 일치 확인:** 발행자와 구독자의 토픽명이 정확한지 확인
3. **QoS 설정 확인:** 중요한 메시지는 QoS 1 사용

### Python 패키지 설치 오류

```bash
# 가상환경 사용 권장
python3 -m venv venv
source venv/bin/activate
pip install paho-mqtt

# macOS의 경우
brew install python3
python3 -m pip install --upgrade pip
```

## 📈 성능 최적화 팁

1. **메시지 주기 조정:** 센서 데이터 발행 주기를 조정하여 트래픽 최적화
2. **QoS 설정:** 중요도에 따라 QoS 0, 1, 2 선택
3. **retain 플래그:** 상태 메시지에는 retain=true 설정
4. **클라이언트 정리:** 테스트 종료 시 proper disconnect 수행

## 🎯 테스트 완료 체크리스트

- [ ] 모든 컴포넌트가 MQTT 브로커에 연결됨
- [ ] 자율주행 모드 ON/OFF가 모든 디바이스에 전파됨
- [ ] LIDAR 데이터가 서버로 전송됨
- [ ] 얼굴 인식 + 온도 데이터가 통합되어 전송됨
- [ ] 고온 감지 시 선풍기 제어 명령이 발행됨
- [ ] 장애물 감지 시 정지 명령이 발행됨
- [ ] Flutter 앱에서 수동 제어가 가능함
- [ ] 모든 디바이스 상태가 실시간으로 모니터링됨

성공적으로 완료되면 실제 하드웨어와 연동할 준비가 완료됩니다! 🎉
