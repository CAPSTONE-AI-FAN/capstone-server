#!/bin/bash

echo "=================================================="
echo "🚀 AI FAN 시스템 테스트 환경 설정"
echo "=================================================="

# Python 및 pip 업데이트
echo "📦 Python 및 pip 업데이트 중..."
sudo apt update
python3 -m pip install --upgrade pip

# 필수 패키지 설치
echo "📦 필수 패키지 설치 중..."
pip3 install paho-mqtt
pip3 install numpy
pip3 install opencv-python

# 권한 설정
echo "🔧 실행 권한 설정 중..."
chmod +x ubuntu_pi_test.py
chmod +x raspbian_pi_test.py

echo "✅ 설치 완료!"
echo ""
echo "🎯 테스트 실행 방법:"
echo ""
echo "📋 방법 A: 기본 MQTT 시뮬레이션"
echo "1. AI FAN 서버:"
echo "   cd aifan-server && node app.js"
echo ""
echo "2. Ubuntu Pi 시뮬레이터:"
echo "   python3 ubuntu_pi_test.py"
echo ""
echo "3. Raspbian Pi 시뮬레이터:"
echo "   python3 raspbian_pi_test.py"
echo ""
echo "📋 방법 B: ROS2-MQTT 브리지 (추천)"
echo "1. AI FAN 서버:"
echo "   cd aifan-server && node app.js"
echo ""
echo "2. ROS2 환경 설정 (Ubuntu에서):"
echo "   source /opt/ros/humble/setup.bash"
echo "   pip3 install -r requirements_ros2.txt"
echo ""
echo "3. ROS2-MQTT 브리지:"
echo "   python3 ubuntu_pi_ros2_mqtt_bridge.py"
echo ""
echo "4. Raspbian Pi 시뮬레이터:"
echo "   python3 raspbian_pi_test.py"
echo ""
echo "5. Flutter 앱:"
echo "   cd AI_FAN && flutter run"
echo ""
echo "==================================================" 