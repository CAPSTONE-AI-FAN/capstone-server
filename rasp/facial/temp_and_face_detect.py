import time
import numpy as np
import cv2
import threading
from queue import Queue
from datetime import datetime
import os
from scipy.spatial import distance
import json
import pickle
import sys
import argparse
import subprocess

# 명령행 인자 파싱 - 테스트 모드 확장
parser = argparse.ArgumentParser(description='열화상 얼굴 인식 시스템')
parser.add_argument('--no-window', action='store_true', help='창 없이 실행 (헤드리스 모드)')
parser.add_argument('--test', action='store_true', help='테스트 모드 (카메라만 사용)')
parser.add_argument('--max-faces', type=int, default=10, help='최대 인식 얼굴 수 (테스트 모드용)')
parser.add_argument('--dummy', action='store_true', help='더미 모드 (카메라/센서 없이)')
parser.add_argument('--debug', action='store_true', help='디버그 모드 (GUI로 프로세스 확인)')
parser.add_argument('--thermal-test', action='store_true', help='테스트 모드에서 열화상 센서 사용')
args = parser.parse_args()

# 저장 디렉토리 생성
os.makedirs('fusion_data', exist_ok=True)
os.makedirs('person_data', exist_ok=True)
os.makedirs('face_data', exist_ok=True)

# 열화상과 RGB 이미지를 저장할 큐
thermal_queue = Queue(maxsize=3)
rgb_queue = Queue(maxsize=3)

# 사람 추적을 위한 변수
next_person_id = 0
tracked_persons = {}  # ID -> 데이터 매핑
MAX_DISAPPEARED = 20  # 최대 프레임 사라짐 허용 수

# 얼굴 인식 관련 변수
#known_face_encodings = []  # 등록된 얼굴 특징
#known_face_ids = []        # 얼굴 ID 목록
#FACE_DATABASE_FILE = "face_data/known_faces.pkl"
previous_frame = None
camera_stabilized = True  # 카메라 안정화 상태 플래그

# 글로벌 변수
face_recognition_enabled = True
test_mode = args.test
dummy_mode = args.dummy
debug_mode = args.debug
thermal_test = args.thermal_test
no_window = args.no_window and not args.debug  # 디버그 모드일 때는 항상 창 표시
current_display_image = None
MAX_FACES = args.max_faces  # 최대 인식 얼굴 수

# 데이터 구조: 다른 모듈에서 접근 가능하도록 전역 변수로 유지
latest_face_data = []  # 인식된 얼굴 데이터
latest_grid_data = [[None for _ in range(3)] for _ in range(3)]  # 9분할 그리드 데이터
latest_thermal_data = None  # 최신 열화상 데이터
data_lock = threading.Lock()  # 스레드 안전성을 위한 락

# 통계 변수 (테스트 모드용)
if test_mode:
    total_faces_detected = 0
    known_faces_detected = 0
    max_faces_in_frame = 0
    frames_processed = 0
    face_stats = {}  # 얼굴 ID별 감지 횟수
    start_time = time.time()

def reset_i2c_bus():
    """I2C 버스 리셋 시도"""
    try:
        import subprocess
        # I2C 버스 리셋 명령
        subprocess.run(['i2cdetect', '-y', '1'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(0.5)
        return True
    except Exception as e:
        print(f"I2C 버스 리셋 실패: {e}")
        return False
    
# 9분할 그리드 위치 계산 함수
def calculate_grid_position(face_box, frame_shape):
    """
    얼굴 위치를 3x3 그리드 내 위치로 변환
    
    Args:
        face_box: (x, y, w, h) 얼굴 박스 좌표
        frame_shape: (height, width) 프레임 크기
    
    Returns:
        (row, col): 그리드 내 위치 (0-2, 0-2)
    """
    x, y, w, h = face_box
    height, width = frame_shape[:2]
    
    # 얼굴 중심점 계산
    center_x = x + w/2
    center_y = y + h/2
    
    # 그리드 경계 계산
    grid_width = width / 3
    grid_height = height / 3
    
    # 그리드 위치 계산
    col = int(center_x / grid_width)
    row = int(center_y / grid_height)
    
    # 범위 확인
    col = max(0, min(2, col))
    row = max(0, min(2, row))
    
    return (row, col)

# 그리드 상태 맵 업데이트 함수
def update_grid_map(tracked_persons, frame_shape, thermal_data=None):
    """
    인식된 사람들의 위치를 3x3 그리드에 매핑
    
    Args:
        tracked_persons: 추적된 사람들 정보 딕셔너리
        frame_shape: (height, width) 프레임 크기
        thermal_data: 열화상 데이터 (선택적)
    
    Returns:
        grid_map: 3x3 그리드에 대한 정보 
    """
    # 3x3 빈 그리드 초기화
    grid_map = [[None for _ in range(3)] for _ in range(3)]
    
    # 각 사람에 대해 그리드 위치 계산 및 데이터 추가
    for person_id, person_info in tracked_persons.items():
        face_box = person_info['face_box']
        row, col = calculate_grid_position(face_box, frame_shape)
        
        # 온도 데이터 가져오기
        temp_data = None
        if thermal_data is not None and 'history' in person_info and person_info['history']:
            last_entry = person_info['history'][-1]
            if 'temperature' in last_entry:
                temp_data = last_entry['temperature']
        
        # 그리드에 정보 저장
        grid_map[row][col] = {
            "person_id": person_id,
            "face_id": person_info.get('face_id', 'Unknown'),
            "temperature": temp_data['top_avg_temp'] if temp_data else None,
            "confidence": temp_data['confidence'] if temp_data else "low",
            "position": [row, col]
        }
    
    return grid_map

# 외부 모듈에서 데이터 접근을 위한 함수들
def get_latest_face_data():
    """최신 얼굴 인식 데이터 반환"""
    with data_lock:
        return latest_face_data.copy() if latest_face_data else []

def get_latest_grid_data():
    """최신 그리드 데이터 반환"""
    with data_lock:
        return [row.copy() for row in latest_grid_data] if latest_grid_data else [[None for _ in range(3)] for _ in range(3)]

def get_latest_thermal_data():
    """최신 열화상 데이터 반환"""
    with data_lock:
        return latest_thermal_data.copy() if isinstance(latest_thermal_data, np.ndarray) else latest_thermal_data


def init_face_detectors():
    """DNN 기반 얼굴 검출 모델 로드"""
    print("DNN 얼굴 검출기 초기화...")
    
    # 모델 파일 경로
    model_path = 'face_data/face_detection_model'
    os.makedirs(model_path, exist_ok=True)
    
    # 파일 경로
    prototxt_path = f"{model_path}/deploy.prototxt"
    caffemodel_path = f"{model_path}/res10_300x300_ssd_iter_140000.caffemodel"
    
    # 파일이 없거나, 크기가 1KB 미만이면 재다운로드
    def file_invalid(path):
        return (not os.path.exists(path)) or (os.path.getsize(path) < 1024)
    
    if file_invalid(prototxt_path) or file_invalid(caffemodel_path):
        print("모델 파일 다운로드 중...")
        os.system(f"wget -O {prototxt_path} https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/face_detector/deploy.prototxt")
        os.system(f"wget -O {caffemodel_path} https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/face_detector/res10_300x300_ssd_iter_140000.caffemodel")
    
    # DNN 모델 로드
    face_detector = cv2.dnn.readNetFromCaffe(prototxt_path, caffemodel_path)
    
    # 얼굴 랜드마크 감지기 (눈, 코, 입 등의 위치 - 특징점 추출용)
    lbp_face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    
    return face_detector, lbp_face_cascade

def adjust_coordinates_for_thermal_sensor(face_box, rgb_shape, horizontal_offset_cm=2.5):
    x, y, w, h = face_box
    
    # 얼굴 크기를 기반으로 실제 거리 추정
    # 평균적인 얼굴 너비를 약 16cm로 가정
    pixels_per_cm = w / 16.0
    
    # 수평 오프셋을 픽셀로 변환 (카메라가 왼쪽에 있으므로 오른쪽으로 이동)
    horizontal_offset_pixels = int(horizontal_offset_cm * pixels_per_cm)
    
    # 조정된 X 좌표 계산 (오른쪽으로 이동)
    adjusted_x = min(x + horizontal_offset_pixels, rgb_shape[1] - w)
    
    # 얼굴 위치가 수평 방향으로 오프셋됨
    return [adjusted_x, y, w, h]

def visualize_thermal_mapping(display_image, face_box, rgb_shape, thermal_data, thermal_shape=(24, 32)):
    """오프셋 보정 및 열화상 매핑 시각화 (디버깅용)"""
    debug_img = display_image.copy()
    
    # 원본 얼굴 박스 (녹색)
    x, y, w, h = face_box
    cv2.rectangle(debug_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
    cv2.putText(debug_img, "Camera Face", (x, y-5), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # 오프셋 보정된 박스 (파란색)
    adjusted_box = adjust_coordinates_for_thermal_sensor(face_box, rgb_shape)
    ax, ay, aw, ah = adjusted_box
    cv2.rectangle(debug_img, (ax, ay), (ax+aw, ay+ah), (255, 0, 0), 2)
    cv2.putText(debug_img, "Adjusted (2.5cm)", (ax, ay-5), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    
    if thermal_data is not None:
        # 열화상 영역 표시 (빨간색)
        thermal_box = map_to_thermal(adjusted_box, rgb_shape, thermal_shape)
        tx, ty, tw, th = thermal_box
        
        # 열화상 좌표를 RGB 좌표로 역변환
        screen_tx = int(tx * rgb_shape[1] / thermal_shape[1])
        screen_ty = int(ty * rgb_shape[0] / thermal_shape[0])
        screen_tw = int(tw * rgb_shape[1] / thermal_shape[1])
        screen_th = int(th * rgb_shape[0] / thermal_shape[0])
        
        cv2.rectangle(debug_img, (screen_tx, screen_ty), 
                     (screen_tx+screen_tw, screen_ty+screen_th), (0, 0, 255), 2)
        cv2.putText(debug_img, "Thermal Region", (screen_tx, screen_ty-5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # 온도 정보 추출 및 표시
        temp_data = get_face_temperature(thermal_data, face_box, rgb_shape, thermal_shape)
        if temp_data:
            temp_text = f"Temp: {temp_data['top_avg_temp']:.1f}°C (conf: {temp_data['confidence']})"
            cv2.putText(debug_img, temp_text, (ax, ay+ah+20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    
    # 카메라와 센서 위치 관계 그림 (화면 하단)
    height, width = rgb_shape
    cv2.rectangle(debug_img, (width-300, height-80), (width-10, height-10), (0, 0, 0), -1)
    cv2.putText(debug_img, "Camera", (width-280, height-50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(debug_img, "Thermal", (width-150, height-50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.line(debug_img, (width-200, height-40), (width-180, height-40), (255, 255, 255), 2)
    cv2.putText(debug_img, "2.5cm", (width-210, height-25), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    return debug_img

def convert_numpy_types(obj):
    """Convert NumPy types to Python native types for JSON serialization"""
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

def initialize_mlx90640():
    """MLX90640 열화상 센서 초기화"""
    if dummy_mode:
        print("더미 모드: 더미 열화상 센서 사용")
        return "dummy_thermal"
        
    try:
        import board
        import busio
        import adafruit_mlx90640
        
        i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
        mlx = adafruit_mlx90640.MLX90640(i2c)
        mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ
        return mlx
    except Exception as e:
        print(f"열화상 센서 초기화 실패: {e}")
        print("열화상 없이 계속 진행합니다.")
        return "dummy_thermal"

def initialize_camera_libcamera():
    """libcamera 기반 카메라 초기화 - 극한 성능 최적화"""
    try:
        # 먼저 카메라 사용 가능 여부 확인
        result = subprocess.run(['libcamera-hello', '--version'], 
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if result.returncode != 0:
            print("libcamera-hello 실패, 카메라를 확인하세요.")
            return None
            
        # libcamera 래퍼 클래스 - 극한 최적화 버전
        class LibCameraWrapper:
            def __init__(self):
                self.image_file = "/tmp/libcamera_capture.jpg"
                self.last_capture_time = 0
                self.capture_interval = 0.2  # 5fps로 제한 (극한 최적화)
                
                # 초기 이미지 캡처
                self.capture_image()
                
            def capture_image(self):
                current_time = time.time()
                if current_time - self.last_capture_time < self.capture_interval:
                    return
                    
                try:
                    # 극한 성능 최적화된 캡처 명령
                    subprocess.run([
                        'libcamera-still', 
                        '-n',                    # 미리보기 비활성화
                        '--immediate',           # 즉시 캡처
                        '-o', self.image_file,
                        '-t', '1',               # 최소 캡처 시간
                        '--width', '320',        # 매우 낮은 해상도
                        '--height', '240',       # 4:3 비율
                        '--quality', '50',       # 낮은 품질 (속도 우선)
                        '--nopreview'            # 미리보기 완전 비활성화
                    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=2)
                    
                    self.last_capture_time = current_time
                except Exception as e:
                    print(f"이미지 캡처 실패: {e}")
                
            def read(self):
                # 새 이미지 캡처
                self.capture_image()
                
                # 파일에서 이미지 읽기
                if os.path.exists(self.image_file):
                    try:
                        img = cv2.imread(self.image_file)
                        if img is not None:
                            # 이미 작은 해상도이므로 추가 리사이즈 불필요
                            return True, img
                    except Exception as e:
                        print(f"이미지 읽기 실패: {e}")
                
                return False, None
                
            def release(self):
                if os.path.exists(self.image_file):
                    try:
                        os.remove(self.image_file)
                    except:
                        pass
        
        return LibCameraWrapper()
    except Exception as e:
        print(f"libcamera 초기화 실패: {e}")
        return None

def initialize_camera():
    """카메라 초기화 - 극한 성능 최적화"""
    if dummy_mode:
        print("더미 모드: 더미 카메라 사용")
        return create_dummy_camera()
    
    # libcamera 기반 초기화
    print("libcamera로 카메라 초기화 시도...")
    camera = initialize_camera_libcamera()
    if camera:
        print("libcamera 카메라 초기화 성공")
        return camera

    # OpenCV 직접 초기화 - 극한 최적화
    try:
        print("OpenCV로 기본 카메라 초기화 시도...")
        cap = cv2.VideoCapture(0)
        
        if cap.isOpened():
            # 극한 성능 최적화된 설정
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)   # 매우 낮은 해상도
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # 4:3 비율
            cap.set(cv2.CAP_PROP_FPS, 10)            # 낮은 FPS
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # 버퍼 최소화
            
            # 테스트 프레임 읽기
            ret, frame = cap.read()
            if ret:
                print(f"OpenCV 카메라 초기화 성공 (해상도: {frame.shape[1]}x{frame.shape[0]})")
                return cap
            else:
                cap.release()
    except Exception as e:
        print(f"OpenCV 카메라 초기화 오류: {e}")
    
    # 모든 방법 실패 시 더미 카메라 반환
    print("모든 카메라 초기화 방법 실패, 더미 카메라로 대체합니다.")
    return create_dummy_camera()

def create_dummy_camera():
    """테스트용 더미 카메라 - 극한 성능 최적화"""
    print("더미 카메라 생성")
    
    class DummyCamera:
        def __init__(self):
            # 매우 작은 테스트 이미지 생성
            self.test_image = np.zeros((240, 320, 3), dtype=np.uint8)
            # 간단한 테스트용 상자
            cv2.rectangle(self.test_image, (80, 80), (120, 120), (0, 255, 0), 1)
            cv2.rectangle(self.test_image, (200, 100), (240, 140), (0, 0, 255), 1)
            cv2.putText(self.test_image, "Dummy Fast", (100, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
        def read(self):
            # 노이즈 없이 고정 이미지 반환 (속도 우선)
            return True, self.test_image.copy()
            
        def release(self):
            pass
    
    return DummyCamera()

def dummy_thermal_capture_thread():
    """테스트용 더미 열화상 데이터 생성 스레드"""
    while True:
        # 24x32 크기의 랜덤 열화상 데이터 생성
        thermal_data = np.random.uniform(20, 40, (24, 32))
        
        # 중앙에 더 높은 온도의 "사람" 시뮬레이션
        center_y, center_x = thermal_data.shape[0] // 2, thermal_data.shape[1] // 2
        thermal_data[center_y-3:center_y+3, center_x-4:center_x+4] = np.random.uniform(35, 37, (6, 8))
        
        # 열화상 데이터 큐에 추가
        if not thermal_queue.full():
            thermal_queue.put((thermal_data, time.time()))
        else:
            thermal_queue.get()  # 가장 오래된 항목 제거
            thermal_queue.put((thermal_data, time.time()))
            
        time.sleep(0.25)  # 4Hz에 맞춤

def thermal_capture_thread(mlx):
    """열화상 캡처 스레드"""
    if mlx == "dummy_thermal":
        dummy_thermal_capture_thread()
        return
        
    if mlx is None:
        dummy_thermal_capture_thread()
        return
    
    consecutive_errors = 0
    max_consecutive_errors = 5
        
    while True:
        frame = np.zeros((24*32,))
        try:
            mlx.getFrame(frame)
            thermal_data = np.fliplr(np.reshape(frame, (24, 32)))
            
            # 열화상 데이터 큐에 추가
            if not thermal_queue.full():
                thermal_queue.put((thermal_data, time.time()))
            else:
                thermal_queue.get()  # 가장 오래된 항목 제거
                thermal_queue.put((thermal_data, time.time()))
            
            # 성공적인 읽기 후 에러 카운터 리셋
            consecutive_errors = 0
                
            time.sleep(0.25)  # 4Hz에 맞춤
        except Exception as e:
            consecutive_errors += 1
            print(f"Thermal capture error: {e}")
            
            # 연속 에러가 임계값을 초과하면 더미 모드로 전환
            if consecutive_errors >= max_consecutive_errors:
                print(f"Too many consecutive errors ({consecutive_errors}). Switching to dummy thermal mode.")
                dummy_thermal_capture_thread()
                return
                
            # 에러 후 더 긴 대기 시간
            time.sleep(2)

def rgb_capture_thread(cap):
    """RGB 카메라 캡처 스레드 - 극한 성능 최적화"""
    frame_count = 0
    last_fps_time = time.time()
    
    print("RGB 캡처 스레드 시작 (극한 최적화 모드)")
    
    while True:
        try:
            ret, rgb_image = cap.read()
            if not ret or rgb_image is None:
                print("Camera read error, retrying...")
                time.sleep(0.2)
                continue
            
            # 이미지가 이미 작으므로 추가 처리 불필요
            # RGB 이미지 큐에 추가
            if not rgb_queue.full():
                rgb_queue.put((rgb_image, time.time()))
            else:
                rgb_queue.get()  # 가장 오래된 항목 제거
                rgb_queue.put((rgb_image, time.time()))
            
            # FPS 모니터링 (10초마다)
            frame_count += 1
            current_time = time.time()
            if current_time - last_fps_time >= 10.0:
                fps = frame_count / (current_time - last_fps_time)
                print(f"Camera FPS: {fps:.1f}")
                frame_count = 0
                last_fps_time = current_time
                
            # 더 긴 대기 시간 (5fps 목표)
            time.sleep(0.2)
            
        except Exception as e:
            print(f"RGB capture error: {e}")
            time.sleep(1)

def resize_thermal_to_match_rgb(thermal_data, target_shape):
    """열화상 데이터 크기 조정 및 시각화 개선"""
    min_val = np.min(thermal_data)
    max_val = np.max(thermal_data)
    
    # 0으로 나누기 방지
    if max_val - min_val == 0:
        normalized = np.zeros_like(thermal_data, dtype=np.uint8)
    else:
        normalized = (thermal_data - min_val) / (max_val - min_val) * 255
        normalized = normalized.astype(np.uint8)
    
    # 색상 맵 적용 (JET - 파란색=낮은 온도, 빨간색=높은 온도)
    thermal_colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
    
    # 열화상 이미지 크기 조정 (RGB 크기에 맞춤)
    thermal_resized = cv2.resize(thermal_colored, (target_shape[1], target_shape[0]), 
                             interpolation=cv2.INTER_CUBIC)
    
    return thermal_resized, min_val, max_val

def create_fusion_image(rgb_image, thermal_colored, alpha=0.2):
    """RGB와 열화상 이미지 결합 - alpha 값 낮춤"""
    # 두 이미지가 같은 크기인지 확인
    assert rgb_image.shape[0:2] == thermal_colored.shape[0:2], "Images must be the same size"
    
    # 이미지 합성 (가중치 적용)
    fusion = cv2.addWeighted(rgb_image, 1-alpha, thermal_colored, alpha, 0)
    return fusion

def detect_camera_movement(prev_frame, curr_frame, threshold=0.4):
    """급격한 카메라 움직임 감지"""
    if prev_frame is None or curr_frame is None:
        return False
    
    # 이미지 크기 조정 (연산 속도를 위해)
    prev_small = cv2.resize(prev_frame, (160, 120))
    curr_small = cv2.resize(curr_frame, (160, 120))
    
    # 그레이스케일 변환
    prev_gray = cv2.cvtColor(prev_small, cv2.COLOR_BGR2GRAY)
    curr_gray = cv2.cvtColor(curr_small, cv2.COLOR_BGR2GRAY)
    
    # 이미지 간 차이 계산
    diff = cv2.absdiff(prev_gray, curr_gray)
    
    # 임계값 적용하여 이진화
    _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
    
    # 변화 비율 계산
    change_ratio = np.count_nonzero(thresh) / (thresh.shape[0] * thresh.shape[1])
    
    # 임계값 이상의 변화가 있으면 급격한 움직임으로 판단
    return change_ratio > threshold

def detect_faces_dnn(image, face_detector, conf_threshold=0.6, max_faces=3):
    """DNN 기반 얼굴 검출 - 극한 성능 최적화"""
    # 이미지가 이미 작으므로 추가 리사이즈 불필요
    h, w = image.shape[:2]
    
    # 더 작은 blob 크기로 처리 속도 향상
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (160, 120)), 1.0,
                               (160, 120), (104.0, 177.0, 123.0))
    
    # 얼굴 감지
    face_detector.setInput(blob)
    detections = face_detector.forward()
    
    faces = []
    # 최대 3개 얼굴만 처리 (성능 우선)
    for i in range(0, min(detections.shape[2], 5)):  # 최대 5개만 확인
        confidence = detections[0, 0, i, 2]
        
        # 높은 신뢰도만 처리
        if confidence > conf_threshold:
            # 바운딩 박스 좌표 계산
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")
            
            # 바운딩 박스가 이미지 경계 내에 있는지 확인
            startX = max(0, startX)
            startY = max(0, startY)
            endX = min(w, endX)
            endY = min(h, endY)
            
            # 박스 크기가 적절한지 확인
            box_w = endX - startX
            box_h = endY - startY
            if box_w < 20 or box_h < 20 or box_w > w * 0.8 or box_h > h * 0.8:
                continue
                
            # 유효한 얼굴 영역만 추가
            if startX < endX and startY < endY:
                faces.append({
                    'box': (startX, startY, endX - startX, endY - startY),
                    'confidence': float(confidence)
                })
                
                # 최대 얼굴 수 제한 (성능 우선)
                if len(faces) >= max_faces:
                    break
    
    return faces

def detect_people(rgb_image, face_detector, max_faces=3):
    """다중 얼굴 검출 - 극한 성능 최적화"""
    results = []
    try:
        # DNN 기반 얼굴 감지만 사용 (Haar Cascade 제거로 속도 향상)
        detected_faces = detect_faces_dnn(rgb_image, face_detector, conf_threshold=0.6, max_faces=max_faces)

        for face_info in detected_faces:
            x, y, w, h = face_info['box']
            confidence = face_info['confidence']

            # 너무 작은 얼굴 무시
            if w < 20 or h < 20:
                continue

            # 간단한 전신 영역 계산 (복잡한 계산 제거)
            body_h = int(h * 2.5)
            body_w = int(w * 1.2)
            body_x = max(0, x - int(w * 0.1))
            body_y = y
            if body_y + body_h > rgb_image.shape[0]:
                body_h = rgb_image.shape[0] - body_y
            if body_x + body_w > rgb_image.shape[1]:
                body_w = rgb_image.shape[1] - body_x

            results.append({
                'type': 'face',
                'face_box': (x, y, w, h),
                'body_box': (body_x, body_y, body_w, body_h),
                'confidence': confidence,
                'face_id': None
            })
    except Exception as e:
        print(f"detect_people error: {e}")
    return results

def extract_face_features(image, face_box):
    """얼굴 특징 추출 - LBP 히스토그램 사용"""
    x, y, w, h = face_box
    # 경계 체크
    h_img, w_img = image.shape[:2]
    x = max(0, x)
    y = max(0, y)
    w = min(w, w_img - x)
    h = min(h, h_img - y)
    if w <= 0 or h <= 0:
        return None
    face_roi = image[y:y+h, x:x+w]
    if face_roi.shape[0] < 20 or face_roi.shape[1] < 20:
        return None
    try:
        # 개선된 전처리
        gray = cv2.cvtColor(face_roi, cv2.COLOR_BGR2GRAY)
        # 크기 정규화
        gray = cv2.resize(gray, (64, 64))
        # 히스토그램 평활화
        gray = cv2.equalizeHist(gray)
        
        # 기본 히스토그램 기능
        hist = cv2.calcHist([gray], [0], None, [64], [0, 256])
        hist = cv2.normalize(hist, hist).flatten()
        return hist
    except Exception as e:
        print(f"extract_face_features error: {e}")
        return None
    
def validate_face_detection(face_boxes, image):
    """감지된 얼굴이 실제 얼굴인지 확인 - 완화된 검증"""
    validated_faces = []
    
    for box in face_boxes:
        x, y, w, h = box['box']
        
        # 1. 너무 작은 얼굴이나 큰 얼굴 필터링 (완화된 기준)
        if w < 20 or h < 20 or w > image.shape[1] * 0.8 or h > image.shape[0] * 0.8:
            continue
        
        # 2. 얼굴 영역 추출
        roi = image[y:y+h, x:x+w]
        if roi.size == 0:
            continue
            
        # 3. 다중 피부톤 모델 - 다양한 피부색을 포용하기 위해 여러 색상 공간 사용
        # 3.1 HSV 색상 공간
        try:
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            # HSV 색상 공간에서 피부톤 범위 (더 넓은 범위)
            lower_skin_hsv = np.array([0, 10, 40], dtype=np.uint8)
            upper_skin_hsv = np.array([35, 180, 255], dtype=np.uint8)
            skin_mask_hsv = cv2.inRange(hsv_roi, lower_skin_hsv, upper_skin_hsv)
            
            # 3.2 YCrCb 색상 공간 (다양한 인종의 피부톤에 더 효과적)
            ycrcb_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
            lower_skin_ycrcb = np.array([0, 135, 85], dtype=np.uint8)
            upper_skin_ycrcb = np.array([255, 180, 135], dtype=np.uint8)
            skin_mask_ycrcb = cv2.inRange(ycrcb_roi, lower_skin_ycrcb, upper_skin_ycrcb)
            
            # 두 마스크 결합 (OR 연산)
            skin_mask = cv2.bitwise_or(skin_mask_hsv, skin_mask_ycrcb)
            
            # 피부톤이 전체 영역의 일정 비율 이상인지 확인 (임계값 완화)
            skin_ratio = cv2.countNonZero(skin_mask) / (w * h)
            if skin_ratio < 0.08:  # 최소 8%로 낮춤
                continue
        except:
            # 색상 변환 오류 시 검증 건너뛰기
            pass
            
        # 검증을 통과한 얼굴 추가
        validated_faces.append(box)
    
    return validated_faces

def recognize_face(image, face_box, face_cascade):
    """
    얼굴 인식 대신 위치 기반 ID 생성
    """
    # 얼굴 위치에 기반한 임시 ID 생성
    x, y, w, h = face_box
    position_id = f"Face_{int(x/50)}_{int(y/50)}_{w}_{h}"
    return position_id


def track_people(detected_people, reset_tracking=False):
    """검출된 사람들을 지속적으로 추적"""
    global next_person_id, tracked_persons
    
    # 완전 리셋 모드: 매 프레임마다 모든 ID를 새로 할당
    if reset_tracking:
        tracked_persons = {}
        next_person_id = 0
        
        # 새 ID 할당
        for person in detected_people:
            person['id'] = next_person_id
            person['disappeared'] = 0
            person['history'] = []  # 온도 이력 저장
            tracked_persons[next_person_id] = person
            next_person_id += 1
            
        return tracked_persons
    
    # 등록된 모든 사람의 사라짐 카운터 증가
    for person_id in tracked_persons:
        tracked_persons[person_id]['disappeared'] += 1
    
    # 사람이 감지되지 않은 경우
    if len(detected_people) == 0:
        # 일정 시간 이상 사라진 사람 제거
        tracked_persons = {k: v for k, v in tracked_persons.items() 
                         if v['disappeared'] <= MAX_DISAPPEARED}
        return tracked_persons
    
    # 처음 감지된 경우
    if len(tracked_persons) == 0:
        for person in detected_people:
            person['id'] = next_person_id
            person['disappeared'] = 0
            person['history'] = []  # 온도 이력 저장
            tracked_persons[next_person_id] = person
            next_person_id += 1
    else:
        # 기존 등록자와 새 감지된 사람들 매칭
        tracked_ids = list(tracked_persons.keys())
        
        # 중심점 계산
        centroids_current = []
        for person in detected_people:
            x, y, w, h = person['body_box']
            centroids_current.append((x + w/2, y + h/2))
        
        centroids_tracked = []
        for person_id in tracked_ids:
            x, y, w, h = tracked_persons[person_id]['body_box']
            centroids_tracked.append((x + w/2, y + h/2))
        
        # 중심점 간 거리 계산
        D = distance.cdist(np.array(centroids_tracked), np.array(centroids_current))
        
        # 간단한 그리디 매칭
        matched_rows = set()
        matched_cols = set()
        
        # D 행렬에서 가장 작은 값부터 매칭
        while True:
            if (len(matched_rows) == len(tracked_ids) or 
                len(matched_cols) == len(detected_people) or 
                D.size == 0 or np.min(D) > 200):  # 임계값 이상 거리면 새 사람으로 간주
                break
            
            # 가장 가까운 쌍 찾기
            min_idx = np.argmin(D)
            row = min_idx // D.shape[1]
            col = min_idx % D.shape[1]
            
            if row not in matched_rows and col not in matched_cols:
                person_id = tracked_ids[row]
                
                # 감지된 사람 정보 업데이트
                new_person = detected_people[col]
                tracked_persons[person_id]['face_box'] = new_person['face_box']
                tracked_persons[person_id]['body_box'] = new_person['body_box']
                tracked_persons[person_id]['type'] = new_person['type']
                tracked_persons[person_id]['confidence'] = new_person['confidence']
                tracked_persons[person_id]['disappeared'] = 0
                
                # 얼굴 ID가 설정되면 유지
                if new_person['face_id'] is not None:
                    tracked_persons[person_id]['face_id'] = new_person['face_id']
                
                matched_rows.add(row)
                matched_cols.add(col)
            
            # 해당 쌍은 이미 처리했으므로 큰 값으로 설정
            D[row, col] = float('inf')
        
        # 매칭되지 않은 새 사람 추가
        for col in range(len(detected_people)):
            if col not in matched_cols:
                person = detected_people[col]
                person['id'] = next_person_id
                person['disappeared'] = 0
                person['history'] = []  # 온도 이력 저장
                tracked_persons[next_person_id] = person
                next_person_id += 1
        
        # 오랫동안 감지되지 않은 사람 제거
        tracked_persons = {k: v for k, v in tracked_persons.items() 
                         if v['disappeared'] <= MAX_DISAPPEARED}
    
    return tracked_persons

def map_to_thermal(bounding_box, rgb_shape, thermal_shape):
    """RGB 이미지의 박스 좌표를 열화상 좌표로 변환"""
    x, y, w, h = bounding_box
    
    # RGB 이미지 내 위치를 비율로 변환
    rel_x = x / rgb_shape[1]
    rel_y = y / rgb_shape[0]
    rel_w = w / rgb_shape[1]
    rel_h = h / rgb_shape[0]
    
    # 비율을 열화상 크기에 적용
    thermal_x = int(rel_x * thermal_shape[1])
    thermal_y = int(rel_y * thermal_shape[0])
    thermal_w = max(1, int(rel_w * thermal_shape[1]))
    thermal_h = max(1, int(rel_h * thermal_shape[0]))
    
    return (thermal_x, thermal_y, thermal_w, thermal_h)

def get_face_temperature(thermal_data, face_box, rgb_shape, thermal_shape=(24, 32)):
    """
    조정된 얼굴 영역의 온도 데이터를 추출하고 분석
    """
    if thermal_data is None:
        return None
    
    # 오프셋 보정된 얼굴 좌표 구하기
    adjusted_face_box = adjust_coordinates_for_thermal_sensor(face_box, rgb_shape)
    
    # RGB에서 열화상 좌표로 변환
    thermal_x, thermal_y, thermal_w, thermal_h = map_to_thermal(
        adjusted_face_box, rgb_shape, thermal_shape)
    
    # 열화상 배열 범위 확인 및 조정
    if thermal_x >= thermal_shape[1] or thermal_y >= thermal_shape[0]:
        return None
        
    thermal_x = max(0, thermal_x)
    thermal_y = max(0, thermal_y)
    thermal_w = min(thermal_w, thermal_shape[1] - thermal_x)
    thermal_h = min(thermal_h, thermal_shape[0] - thermal_y)
    
    # 얼굴 영역이 너무 작으면 측정 불가
    if thermal_w < 2 or thermal_h < 2:
        return None
    
    # 열화상 데이터에서 얼굴 영역 추출
    face_temp_region = thermal_data[thermal_y:thermal_y+thermal_h, 
                                  thermal_x:thermal_x+thermal_w]
    
    # 이마 부분 온도 (얼굴 상단 1/3 영역) - 체온 측정에 더 신뢰성 있음
    forehead_region = face_temp_region[:max(1, thermal_h//3), :]
    
    # 온도 데이터 유효성 검사
    if face_temp_region.size == 0 or forehead_region.size == 0:
        return None
    
    try:
        # 온도 통계 계산
        max_temp = float(np.max(face_temp_region))
        avg_temp = float(np.mean(face_temp_region))
        forehead_avg_temp = float(np.mean(forehead_region))
        
        # NaN 체크
        if np.isnan(max_temp) or np.isnan(avg_temp) or np.isnan(forehead_avg_temp):
            return None
        
        # 체온 범위 검증 (정상 범위: 대략 35.0-38.0°C)
        confidence = "low"
        if 35.0 <= forehead_avg_temp <= 38.0:
            confidence = "high"
        elif 30.0 <= forehead_avg_temp < 35.0 or 38.0 < forehead_avg_temp <= 40.0:
            confidence = "medium"
        
        return {
            'max_temp': max_temp,
            'avg_temp': avg_temp,
            'top_avg_temp': forehead_avg_temp,
            'confidence': confidence,
            'thermal_box': [thermal_x, thermal_y, thermal_w, thermal_h]
        }
    except Exception as e:
        print(f"온도 분석 오류: {e}")
        return None

def analyze_person_temperature(thermal_data, thermal_box):
    """감지된 사람의 온도 분석"""
    x, y, w, h = thermal_box
    x_end = min(x + w, thermal_data.shape[1])
    y_end = min(y + h, thermal_data.shape[0])
    x = max(0, x)
    y = max(0, y)
    if x < x_end and y < y_end:
        person_temp = thermal_data[y:y_end, x:x_end]
        if person_temp.size > 0:
            try:
                avg_temp = np.mean(person_temp)
                max_temp = np.max(person_temp)
                min_temp = np.min(person_temp)
                
                # NaN이나 Infinity 체크
                if np.isnan(avg_temp) or np.isinf(avg_temp) or \
                   np.isnan(max_temp) or np.isinf(max_temp) or \
                   np.isnan(min_temp) or np.isinf(min_temp):
                   return None
               
                temp_flat = person_temp.flatten()
                top_temp_threshold = np.percentile(temp_flat, 90)
                top_temp_pixels = temp_flat[temp_flat >= top_temp_threshold]
                top_avg_temp = np.mean(top_temp_pixels) if len(top_temp_pixels) > 0 else max_temp
                
                # NaN이나 Infinity 체크
                if np.isnan(top_avg_temp) or np.isinf(top_avg_temp):
                    top_avg_temp = max_temp
                    
                hist, bins = np.histogram(person_temp, bins=10, range=(min_temp, max_temp))
            
                # 모든 값이 유효한지 확인
                if any(np.isnan(b) or np.isinf(b) for b in bins):
                    return None
                
                return {
                    'avg_temp': float(avg_temp),
                    'max_temp': float(max_temp),
                    'min_temp': float(min_temp),
                    'top_avg_temp': float(top_avg_temp),
                    'histogram': hist.tolist(),
                    'hist_bins': [float(b) for b in bins]
                }
            except Exception as e:
                print(f"analyze_person_temperature error: {e}")
                return None
    return None

def process_frame(rgb_image, thermal_data=None):
    try:
        global frames_processed, total_faces_detected, max_faces_in_frame
        global previous_frame, camera_stabilized, tracked_persons
        global latest_face_data, latest_grid_data, latest_thermal_data
        
        # 이미지 처리를 위한 복사본 생성
        display_image = rgb_image.copy()
        
        # 이전 프레임과 비교하여 급격한 카메라 움직임 감지
        camera_moved = detect_camera_movement(previous_frame, rgb_image)
        
        # 카메라가 움직였다면 추적 정보 초기화
        if camera_moved:
            camera_stabilized = False
            tracked_persons = {}  # 추적 정보 리셋
            print("카메라 움직임 감지: 추적 초기화")
        
        # 이전 프레임 업데이트 (다음 비교를 위해)
        previous_frame = rgb_image.copy()
        
        # 열화상 데이터가 있으면 융합 이미지 생성
        if thermal_data is not None:
            thermal_colored, min_temp, max_temp = resize_thermal_to_match_rgb(
                thermal_data, rgb_image.shape)
                
            # 열화상/RGB 혼합
            display_image = create_fusion_image(rgb_image, thermal_colored, alpha=0.2)
            
            # 열화상 데이터 저장 (전역 변수에)
            with data_lock:
                latest_thermal_data = thermal_data.copy() if isinstance(thermal_data, np.ndarray) else thermal_data
                try:
                    import temp_and_face_mqtt as mqtt
                    if mqtt.mqtt_connected:
                        current_timestamp = time.time()
                        # 클래스 변수가 없으면 생성
                        if not hasattr(process_frame, 'last_thermal_publish'):
                            process_frame.last_thermal_publish = 0
                            
                        # 10초마다 열화상 데이터 발행
                        if current_timestamp - process_frame.last_thermal_publish >= 10.0:
                            processed_thermal = convert_numpy_types(latest_thermal_data)
                            mqtt.publish_thermal_map(processed_thermal)
                            process_frame.last_thermal_publish = current_timestamp
                except Exception as e:
                    if debug_mode:
                        print(f"MQTT 열화상 데이터 발행 오류: {e}")
                        import traceback
                        traceback.print_exc()
        else:
            min_temp = max_temp = 0
        
        # 사람 감지 및 얼굴 인식
        max_faces_limit = MAX_FACES if test_mode else None
        detected_people = detect_people(rgb_image, face_detector, max_faces=max_faces_limit)
        
        # 테스트 모드에서는 통계 업데이트
        if test_mode:
            faces_in_frame = len(detected_people)
            frames_processed += 1
            total_faces_detected += faces_in_frame
            max_faces_in_frame = max(max_faces_in_frame, faces_in_frame)
        
        # 감지된 얼굴이 없으면, 빈 그리드 맵 저장하고 종료
        if len(detected_people) == 0:
            # 카메라 안정화 플래그 설정
            camera_stabilized = True
            
            # 빈 그리드 맵 저장
            with data_lock:
                latest_face_data = []
                latest_grid_data = [[None for _ in range(3)] for _ in range(3)]
            
            # 디버그 모드에서 현재 시간 표시
            if debug_mode:
                display_timestamp = datetime.now().strftime("%H:%M:%S")
                cv2.putText(display_image, f"Time: {display_timestamp} | People: 0", 
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                if thermal_data is not None:
                    cv2.putText(display_image, f"Temp Range: {min_temp:.1f}°C - {max_temp:.1f}°C", 
                              (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            return display_image
        
        # 얼굴이 감지되었다면 카메라가 안정되었다고 표시
        camera_stabilized = True
        
        # 각 감지된 사람에 대해 ID 생성
        for i, person in enumerate(detected_people):
            if person['type'] == 'face':
                # 위치 기반 ID 생성 (카메라와의 거리, 크기 등을 고려)
                x, y, w, h = person['face_box']
                grid_pos = calculate_grid_position(person['face_box'], rgb_image.shape)
                # 고유 ID 생성 (중복 방지를 위해 인덱스 포함)
                person['face_id'] = f"Face_{grid_pos[0]}_{grid_pos[1]}_{i}"
        
        # 사람 추적 - 매 프레임마다 새로 ID 할당 (reset_tracking=True)
        tracked_persons = track_people(detected_people, reset_tracking=True)
        
        # 그리드 상태 계산 및 저장
        grid_map = update_grid_map(tracked_persons, rgb_image.shape, thermal_data)
        with data_lock:
            latest_grid_data = grid_map
        
        # 안면 인식 데이터 준비
        face_detection_data = []
        
        # 현재 시간
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        display_timestamp = datetime.now().strftime("%H:%M:%S")
        
        # 인식된 각 사람에 대해 처리
        for person_id, person_info in tracked_persons.items():
            # 얼굴 및 전신 박스 가져오기
            face_box = person_info['face_box']
            body_box = person_info['body_box']
            face_id = person_info.get('face_id', f"Face_{person_id}")
            
            # 얼굴 표시 (모든 얼굴에 동일한 색상 사용)
            fx, fy, fw, fh = face_box
            color = (0, 255, 0)  # 모든 얼굴에 녹색 사용
            cv2.rectangle(display_image, (fx, fy), (fx+fw, fy+fh), color, 2)
            
            # 온도 분석 (열화상 데이터가 있는 경우)
            temp_data = None
            if thermal_data is not None:
                thermal_face_box = map_to_thermal(face_box, rgb_image.shape, thermal_data.shape)
                temp_data = get_face_temperature(
                                    thermal_data, 
                                    face_box, 
                                    rgb_image.shape[:2],
                                    thermal_data.shape
                                )
            
            # 안면 인식 데이터 추가
            face_data = {
                "person_id": person_id,
                "face_id": face_id,
                "face_box": list(face_box),
                "body_box": list(body_box),
                "confidence": float(person_info['confidence']),
                "temperature": temp_data['top_avg_temp'] if temp_data else None,
                "temp_confidence": temp_data['confidence'] if temp_data else "low",
                "grid_position": calculate_grid_position(face_box, rgb_image.shape),
                "timestamp": int(time.time() * 1000)
            }
            face_detection_data.append(face_data)
            
            # 사람 정보 표시
            if debug_mode:
                try:
                    if temp_data:
                        label = f"Face {person_id}"
                        temp_label = f"{temp_data['top_avg_temp']:.1f}°C ({temp_data['confidence']})"
                        # 온도 이력 저장
                        person_info['history'].append({
                            'timestamp': timestamp,
                            'temperature': temp_data
                        })
                    else:
                        label = f"Face {person_id}"
                        temp_label = ""
                    
                    # 레이블 표시
                    cv2.putText(display_image, label, (fx, fy-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    # 온도 레이블이 있으면 별도로 표시
                    if temp_label:
                        cv2.putText(display_image, temp_label, (fx, fy+fh+20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                except Exception as e:
                    print(f"레이블 표시 오류: {e}")
                    cv2.putText(display_image, "Face Detected", (fx, fy-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # 인식된 얼굴 데이터 전역 변수에 저장
        with data_lock:
            latest_face_data = face_detection_data
            
            try:
                import temp_and_face_mqtt as mqtt
                if mqtt.mqtt_connected:
                    # 데이터 변환 및 발행
                    processed_data = convert_numpy_types(face_detection_data)
                    mqtt.publish_face_detection(processed_data)
            except Exception as e:
                if debug_mode:
                    print(f"MQTT 얼굴 데이터 발행 오류: {e}")
                    import traceback
                    traceback.print_exc()
        
        # 디버그 모드에서 정보 표시
        if debug_mode:
            cv2.putText(display_image, f"Time: {display_timestamp} | People: {len(tracked_persons)}", 
                      (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            if thermal_data is not None:
                cv2.putText(display_image, f"Temp Range: {min_temp:.1f}°C - {max_temp:.1f}°C", 
                          (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 카메라 움직임 상태 표시
            movement_status = "Stable" if camera_stabilized else "Movement Detected"
            cv2.putText(display_image, f"Camera: {movement_status}", (10, 90), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return display_image
    
    except Exception as e:
        print(f"process_frame error: {e}")
        import traceback
        traceback.print_exc()
        return rgb_image

def display_grid(image, grid_map):
    """3x3 그리드를 이미지에 시각화 (디버그 모드용)"""
    if not debug_mode:
        return
        
    h, w = image.shape[:2]
    cell_h, cell_w = h // 3, w // 3
    
    # 그리드 선 그리기
    for i in range(1, 3):
        cv2.line(image, (0, i * cell_h), (w, i * cell_h), (100, 100, 100), 1)
        cv2.line(image, (i * cell_w, 0), (i * cell_w, h), (100, 100, 100), 1)
    
    # 그리드 셀 번호 표시
    for row in range(3):
        for col in range(3):
            cell_center = (col * cell_w + cell_w // 2, row * cell_h + cell_h // 2)
            
            # 사람이 있는 셀 강조
            if grid_map[row][col] is not None:
                person_data = grid_map[row][col]
                # 셀에 반투명 오버레이
                overlay = image.copy()
                cv2.rectangle(overlay, 
                            (col * cell_w, row * cell_h), 
                            ((col+1) * cell_w, (row+1) * cell_h), 
                            (0, 255, 0, 128), -1)
                # 반투명 효과
                cv2.addWeighted(overlay, 0.3, image, 0.7, 0, image)
                
                # 온도 표시
                if person_data["temperature"] is not None:
                    temp_text = f"{person_data['temperature']:.1f}°C"
                    cv2.putText(image, temp_text, 
                              (cell_center[0] - 30, cell_center[1] + 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 그리드 위치 번호
            grid_text = f"({row},{col})"
            cv2.putText(image, grid_text, 
                      (cell_center[0] - 20, cell_center[1] - 30), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

def main():
    # 얼굴 검출기 초기화만 수행 (얼굴 인식 데이터베이스 로드 부분 제거)
    print("얼굴 검출기 초기화 중...")
    global face_detector, face_cascade
    face_detector, face_cascade = init_face_detectors()
    
    # 이전 프레임 변수 초기화
    global previous_frame, camera_stabilized
    previous_frame = None
    camera_stabilized = True
    
    # MLX90640 열화상 센서 초기화
    mlx = None
    if thermal_test or not test_mode:
        print("MLX90640 열화상 센서 초기화 중...")
        mlx = initialize_mlx90640()
    
    # 카메라 초기화
    print("카메라 초기화 중...")
    cap = initialize_camera()
    if cap is None:
        print("카메라 초기화 실패, 프로그램 종료")
        return
    
    # 모든 모드에서 캡처 스레드 시작
    print("캡처 스레드 시작 중...")
    threading.Thread(target=thermal_capture_thread, args=(mlx,), daemon=True).start()
    threading.Thread(target=rgb_capture_thread, args=(cap,), daemon=True).start()
    
    # 스레드 안정화 대기
    time.sleep(1)
    
    # MQTT 통신 모듈 시작 (선택사항)
    try:
        import temp_and_face_mqtt as mqtt
        mqtt.start_mqtt_thread()
        print("MQTT 통신 시작됨")
    except ImportError:
        print("MQTT 모듈을 찾을 수 없습니다. 통신 없이 실행됩니다.")
    except Exception as e:
        print(f"MQTT 통신 시작 오류: {e}")
    
    print("메인 프로세싱 루프 시작...")
    
    # 메인 루프 시작
    try:
        main_loop()
    except KeyboardInterrupt:
        print("사용자에 의해 프로그램 종료")
    except Exception as e:
        print(f"메인 루프 오류: {e}")
        import traceback
        traceback.print_exc()
    
    # 종료 처리
    print("종료 처리 중...")
    if cap:
        cap.release()
        
    try:
        import temp_and_face_mqtt as mqtt
        mqtt.stop_mqtt_thread()
    except:
        pass    
    
    print("프로그램 종료")

def main_loop():
    """메인 처리 루프 - 극한 성능 최적화"""
    global current_display_image
    
    # 타임스탬프 및 관련 변수 초기화
    last_save_time = time.time()
    last_log_time = time.time()
    save_interval = 15  # 15초마다 이미지 저장 (빈도 감소)
    log_interval = 5    # 5초마다 로그 업데이트 (성능 향상)
    
    people_detected_previously = False
    
    # 프레임 스킵 변수 (성능 향상을 위해)
    frame_skip_count = 0
    frame_skip_max = 3  # 3프레임마다 한 번만 처리
    
    # 디버그 모드에서만 GUI 초기화
    if debug_mode:
        cv2.namedWindow("Face Recognition", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Face Recognition", 640, 480)  # 창 크기 더 축소
    
    frame_count = 0
    last_perf_time = time.time()
    
    print("메인 프로세싱 루프 시작 (극한 최적화)...")
    try:
        while True:
            rgb_available = not rgb_queue.empty()
            thermal_available = not thermal_queue.empty()
            
            if rgb_available:
                # RGB 이미지 처리
                rgb_image, rgb_time = rgb_queue.get()
                current_time = time.time()
                
                # 프레임 스킵 적용 (성능 향상)
                frame_skip_count += 1
                if frame_skip_count < frame_skip_max:
                    # 프레임 스킵 - 화면만 업데이트
                    if debug_mode:
                        display_timestamp = datetime.now().strftime("%H:%M:%S")
                        display_image = rgb_image.copy()
                        cv2.putText(display_image, f"Skip Frame - Time: {display_timestamp}", 
                                  (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                        cv2.imshow("Face Recognition", display_image)
                        key = cv2.waitKey(1) & 0xFF
                        if key == 27:  # ESC
                            break
                    continue
                
                # 프레임 스킵 카운터 리셋
                frame_skip_count = 0
                
                # 열화상 데이터 가져오기 (있는 경우에만)
                thermal_data = None
                if thermal_available:
                    thermal_data, thermal_time = thermal_queue.get()
                
                # 프레임 처리 (극한 최적화)
                display_image = process_frame_optimized(rgb_image, thermal_data)
                
                # 성능 모니터링
                frame_count += 1
                if current_time - last_perf_time >= 10.0:
                    processing_fps = frame_count / (current_time - last_perf_time)
                    print(f"Processing FPS: {processing_fps:.1f}")
                    frame_count = 0
                    last_perf_time = current_time
                
                # 현재 인식된 사람 수 확인
                current_people_count = len(tracked_persons)
                
                # 새로운 사람 감지 여부 확인
                new_people_detected = (not people_detected_previously) and (current_people_count > 0)
                people_detected_previously = current_people_count > 0
                
                # 간소화된 로깅 (성능 향상)
                if current_time - last_log_time > log_interval and current_people_count > 0:
                    print(f"현재 감지된 얼굴: {current_people_count}개")
                    last_log_time = current_time
                
                # 저장 로직 (빈도 감소)
                if (new_people_detected or 
                    (current_people_count > 0 and current_time - last_save_time > save_interval)):
                    
                    # 간소화된 저장
                    file_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    image_filename = f"fusion_data/fast_{current_people_count}faces_{file_timestamp}.jpg"
                    cv2.imwrite(image_filename, display_image)
                    print(f"Fast save: {current_people_count} faces")
                    last_save_time = current_time
                
                # 디버그 모드에서만 화면 표시
                if debug_mode:
                    # 성능 정보 표시
                    display_timestamp = datetime.now().strftime("%H:%M:%S")
                    cv2.putText(display_image, f"Fast Mode | Time: {display_timestamp} | Faces: {current_people_count}", 
                              (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    
                    # 이미지 표시
                    cv2.imshow("Face Recognition", display_image)
                    
                    # 키보드 입력 처리
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:  # ESC
                        break
            
            else:
                # 데이터가 준비되지 않았으면 대기
                time.sleep(0.05)  # 적절한 대기 시간
                
                # 디버그 모드에서 키 입력 처리
                if debug_mode:
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:  # ESC
                        break
    
    except KeyboardInterrupt:
        print("사용자에 의해 프로그램 종료")
    except Exception as e:
        print(f"메인 루프 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if debug_mode:
            cv2.destroyAllWindows()

def process_frame_optimized(rgb_image, thermal_data=None):
    """프레임 처리 - 극한 성능 최적화 버전"""
    try:
        global frames_processed, total_faces_detected, max_faces_in_frame
        global previous_frame, camera_stabilized, tracked_persons
        global latest_face_data, latest_grid_data, latest_thermal_data
        
        # 이미지 처리를 위한 복사본 생성 (메모리 최적화)
        display_image = rgb_image
        
        # 카메라 움직임 감지 생략 (성능 향상)
        camera_stabilized = True
        
        # 열화상 데이터가 있으면 간소화된 융합 이미지 생성
        if thermal_data is not None:
            # 간소화된 열화상 처리
            with data_lock:
                latest_thermal_data = thermal_data
        
        # 사람 감지 및 얼굴 인식 (최대 3개 얼굴만)
        detected_people = detect_people(rgb_image, face_detector, max_faces=3)
        
        # 감지된 얼굴이 없으면, 빈 그리드 맵 저장하고 종료
        if len(detected_people) == 0:
            with data_lock:
                latest_face_data = []
                latest_grid_data = [[None for _ in range(3)] for _ in range(3)]
            return display_image
        
        # 카메라가 안정되었다고 표시
        camera_stabilized = True
        
        # 각 감지된 사람에 대해 간소화된 ID 생성
        for i, person in enumerate(detected_people):
            if person['type'] == 'face':
                person['face_id'] = f"Face_{i}"
        
        # 사람 추적 - 매 프레임마다 새로 ID 할당 (간소화)
        tracked_persons = track_people(detected_people, reset_tracking=True)
        
        # 간소화된 그리드 상태 계산
        grid_map = update_grid_map(tracked_persons, rgb_image.shape, thermal_data)
        with data_lock:
            latest_grid_data = grid_map
        
        # 안면 인식 데이터 준비 (간소화)
        face_detection_data = []
        
        # 현재 시간
        timestamp = int(time.time() * 1000)
        
        # 인식된 각 사람에 대해 간소화된 처리
        for person_id, person_info in tracked_persons.items():
            face_box = person_info['face_box']
            face_id = person_info.get('face_id', f"Face_{person_id}")
            
            # 얼굴 표시 (간소화)
            fx, fy, fw, fh = face_box
            color = (0, 255, 0)  # 녹색
            cv2.rectangle(display_image, (fx, fy), (fx+fw, fy+fh), color, 1)
            
            # 간소화된 레이블
            cv2.putText(display_image, f"F{person_id}", (fx, fy-5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # 안면 인식 데이터 추가 (간소화)
            face_data = {
                "person_id": person_id,
                "face_id": face_id,
                "face_box": list(face_box),
                "confidence": float(person_info['confidence']),
                "timestamp": timestamp
            }
            face_detection_data.append(face_data)
        
        # 인식된 얼굴 데이터 전역 변수에 저장
        with data_lock:
            latest_face_data = face_detection_data
            
            # MQTT 발행 (간소화 - 오류 무시)
            try:
                import temp_and_face_mqtt as mqtt
                if mqtt.mqtt_connected:
                    processed_data = convert_numpy_types(face_detection_data)
                    mqtt.publish_face_detection(processed_data)
            except:
                pass  # MQTT 오류 무시
        
        return display_image
    
    except Exception as e:
        print(f"process_frame_optimized error: {e}")
        return rgb_image

if __name__ == "__main__":
    main()


# TODO:
# TOPIC 구성 변경 필요 특히, 온도에 관한 정보는 얼굴 인식 후 그 정보와 포함해서 발행 (특히 다중 인식에 대해서)