import time
import numpy as np
import cv2
import os
import subprocess
import argparse
import pickle

# 명령행 인자 파싱
parser = argparse.ArgumentParser(description='다중 얼굴 인식 테스트')
parser.add_argument('--max-faces', type=int, default=10, help='최대 인식 얼굴 수')
args = parser.parse_args()

# 저장 디렉토리 생성
os.makedirs('face_data', exist_ok=True)

# 글로벌 변수
current_display_image = None
known_face_encodings = []
known_face_ids = []
FACE_DATABASE_FILE = "face_data/known_faces.pkl"
MAX_FACES = args.max_faces

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

def initialize_camera():
    """카메라 초기화 - IMX708에 최적화"""
    try:
        # libcamera 래퍼 클래스
        class LibCameraWrapper:
            def __init__(self):
                self.image_file = "/tmp/libcamera_capture.jpg"
                self.last_capture_time = 0
                self.capture_interval = 0.1  # 최소 캡처 간격 (초)
                
                # 초기 이미지 캡처 (카메라 워밍업)
                self.capture_image()
                
            def capture_image(self):
                current_time = time.time()
                # 너무 빠른 캡처 방지
                if current_time - self.last_capture_time < self.capture_interval:
                    return
                    
                try:
                    # IMX708에 최적화된 캡처 명령 실행
                    subprocess.run([
                        'libcamera-still', 
                        '-n',
                        '--immediate',
                        '-o', self.image_file,
                        '-t', '3',
                        '--width', '2304',   # IMX708 기본 모드 
                        '--height', '1296',  # IMX708 기본 모드
                        '--quality', '90'    # 높은 JPEG 품질
                    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
                    
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
                            # 처리 속도를 위해 이미지 크기 조정
                            img = cv2.resize(img, (1280, 720), interpolation=cv2.INTER_AREA)
                            return True, img
                    except Exception as e:
                        print(f"이미지 읽기 실패: {e}")
                
                return False, None
                
            def release(self):
                # 임시 파일 정리
                if os.path.exists(self.image_file):
                    try:
                        os.remove(self.image_file)
                    except:
                        pass
        
        return LibCameraWrapper()
    except Exception as e:
        print(f"카메라 초기화 실패: {e}")
        return None

def detect_faces_dnn(image, face_detector, conf_threshold=0.6):
    """DNN 기반 얼굴 검출"""
    # 이미지 전처리
    h, w = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 1.0,
                                (300, 300), (104.0, 177.0, 123.0))
    
    # 얼굴 감지
    face_detector.setInput(blob)
    detections = face_detector.forward()
    
    faces = []
    # 감지된 얼굴 처리
    for i in range(0, detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        
        # 신뢰도가 임계값보다 높은 경우
        if confidence > conf_threshold:
            # 바운딩 박스 좌표 계산
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")
            
            # 바운딩 박스가 이미지 경계 내에 있는지 확인
            startX = max(0, startX)
            startY = max(0, startY)
            endX = min(w, endX)
            endY = min(h, endY)
            
            # 유효한 얼굴 영역만 추가
            if startX < endX and startY < endY:
                faces.append({
                    'box': (startX, startY, endX - startX, endY - startY),
                    'confidence': float(confidence)
                })
    
    return faces

def extract_face_features(image, face_box):
    """얼굴 특징 추출"""
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
        gray = cv2.cvtColor(face_roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, (64, 64))
        hist = cv2.calcHist([gray], [0], None, [64], [0, 256])
        hist = cv2.normalize(hist, hist).flatten()
        return hist
    except Exception as e:
        print(f"extract_face_features error: {e}")
        return None

def match_face_features(features, known_features, threshold=0.6):
    """추출된 얼굴 특징과 기존 등록된 얼굴들 간의 유사도 계산"""
    if not known_features or features is None:
        return None
    
    # 각 알려진 얼굴과 유사도 계산
    similarities = []
    for known_feature in known_features:
        # 히스토그램 비교 (상관관계)
        similarity = cv2.compareHist(features, known_feature, cv2.HISTCMP_CORREL)
        similarities.append(similarity)
    
    # 가장 유사한 얼굴 찾기
    best_match_idx = np.argmax(similarities)
    best_match_similarity = similarities[best_match_idx]
    
    # 임계값 이상인 경우에만 매칭으로 판단
    if best_match_similarity >= threshold:
        return best_match_idx
    else:
        return None

def recognize_face(image, face_box):
    """얼굴 인식"""
    global known_face_encodings, known_face_ids
    
    # 얼굴 특징 추출
    face_features = extract_face_features(image, face_box)
    
    if face_features is None:
        return "Unknown"
    
    # 기존 등록된 얼굴과 비교
    if known_face_encodings:
        match_idx = match_face_features(face_features, known_face_encodings)
        
        if match_idx is not None:
            return known_face_ids[match_idx]
    
    return "Unknown"

def register_new_face(image, face_box, face_id=None):
    """새 얼굴 등록"""
    global known_face_encodings, known_face_ids
    
    # 얼굴 특징 추출
    face_features = extract_face_features(image, face_box)
    
    if face_features is None:
        return False, "얼굴 특징을 추출할 수 없습니다"
    
    if face_id is None:
        face_id = f"Person_{len(known_face_encodings) + 1}"
    
    # 이미 등록된 얼굴인지 확인
    if known_face_encodings:
        match_idx = match_face_features(face_features, known_face_encodings, threshold=0.7)
        if match_idx is not None:
            return False, f"이미 등록된 얼굴입니다: {known_face_ids[match_idx]}"
    
    # 얼굴 추가
    known_face_encodings.append(face_features)
    known_face_ids.append(face_id)
    
    # 얼굴 데이터 저장
    save_face_database()
    
    # 얼굴 이미지 저장
    x, y, w, h = face_box
    face_img = image[y:y+h, x:x+w]
    cv2.imwrite(f"face_data/{face_id}.jpg", face_img)
    
    return True, face_id

def save_face_database():
    """얼굴 인식 데이터베이스 저장"""
    data = {
        'encodings': known_face_encodings,
        'ids': known_face_ids
    }
    with open(FACE_DATABASE_FILE, 'wb') as f:
        pickle.dump(data, f)
    print(f"{len(known_face_encodings)}개의 얼굴을 저장했습니다.")

def load_face_database():
    """얼굴 인식 데이터베이스 로드"""
    global known_face_encodings, known_face_ids
    
    if os.path.exists(FACE_DATABASE_FILE):
        with open(FACE_DATABASE_FILE, 'rb') as f:
            data = pickle.load(f)
            known_face_encodings = data['encodings']
            known_face_ids = data['ids']
        print(f"{len(known_face_encodings)}개의 등록된 얼굴을 로드했습니다.")
    else:
        known_face_encodings = []
        known_face_ids = []
        print("얼굴 데이터베이스 초기화")

def mouse_callback(event, x, y, flags, param):
    """마우스 이벤트 처리 콜백 함수"""
    global current_display_image
    
    # 왼쪽 버튼 클릭 - 얼굴 등록
    if event == cv2.EVENT_LBUTTONDOWN:
        if current_display_image is not None:
            # 얼굴 검출
            faces = detect_faces_dnn(current_display_image, face_detector)
            
            # 클릭 위치에 가장 가까운 얼굴 찾기
            min_dist = float('inf')
            closest_face = None
            
            for face_info in faces:
                fx, fy, fw, fh = face_info['box']
                # 얼굴 중심과 클릭 위치 사이의 거리
                center_x, center_y = fx + fw//2, fy + fh//2
                dist = ((center_x - x)**2 + (center_y - y)**2)**0.5
                
                if dist < min_dist:
                    min_dist = dist
                    closest_face = face_info['box']
            
            # 충분히 가까운 얼굴이 있으면 등록
            if min_dist < 100 and closest_face is not None:
                success, face_id = register_new_face(
                    current_display_image, 
                    closest_face,
                    f"Person_{int(time.time())}"
                )
                if success:
                    print(f"얼굴 등록 성공: {face_id}")
                else:
                    print(f"얼굴 등록 실패: {face_id}")

def main():
    # 얼굴 인식 데이터베이스 로드
    print("얼굴 인식 데이터베이스 로드 중...")
    load_face_database()
    
    # 얼굴 검출기 초기화
    print("얼굴 검출기 초기화 중...")
    global face_detector, face_cascade
    face_detector, face_cascade = init_face_detectors()
    
    # 카메라 초기화
    print("카메라 초기화 중...")
    cap = initialize_camera()
    
    if cap is None:
        print("카메라 초기화 실패, 프로그램 종료")
        return
    
    # 창 생성 및 마우스 콜백 설정
    cv2.namedWindow("Multi-Face Recognition Test", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Multi-Face Recognition Test", 1280, 720)
    cv2.setMouseCallback("Multi-Face Recognition Test", mouse_callback)
    
    # 통계 변수
    total_faces_detected = 0
    known_faces_detected = 0
    max_faces_in_frame = 0
    frames_processed = 0
    face_stats = {}  # 얼굴 ID별 감지 횟수
    
    print("다중 얼굴 인식 테스트 시작...")
    start_time = time.time()
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("Camera read error, retrying...")
                time.sleep(1)
                continue
            
            global current_display_image
            current_display_image = frame.copy()
            display_image = frame.copy()
            
            # 얼굴 검출
            faces = detect_faces_dnn(frame, face_detector)
            faces_in_frame = len(faces)
            
            # 통계 업데이트
            frames_processed += 1
            total_faces_detected += faces_in_frame
            max_faces_in_frame = max(max_faces_in_frame, faces_in_frame)
            
            # 제한된 수의 얼굴만 처리
            faces = faces[:MAX_FACES]
            
            # 얼굴 인식 및 표시
            for face_info in faces:
                x, y, w, h = face_info['box']
                confidence = face_info['confidence']
                
                # 얼굴 인식
                face_id = recognize_face(frame, face_info['box'])
                
                # 얼굴 ID별 통계 업데이트
                if face_id != "Unknown":
                    known_faces_detected += 1
                    if face_id in face_stats:
                        face_stats[face_id] += 1
                    else:
                        face_stats[face_id] = 1
                
                # 얼굴 표시
                color = (0, 255, 0) if face_id != "Unknown" else (0, 0, 255)
                cv2.rectangle(display_image, (x, y), (x+w, y+h), color, 2)
                
                # 얼굴 정보 표시
                label = f"{face_id} ({confidence:.2f})"
                cv2.putText(display_image, label, (x, y-10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # 프레임 정보 표시
            elapsed_time = time.time() - start_time
            fps = frames_processed / elapsed_time if elapsed_time > 0 else 0
            
            cv2.putText(display_image, f"FPS: {fps:.1f}", (10, 30), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.putText(display_image, f"Faces in frame: {faces_in_frame}", (10, 60), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.putText(display_image, f"Max faces detected: {max_faces_in_frame}", (10, 90), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.putText(display_image, f"Total registered faces: {len(known_face_ids)}", (10, 120), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 사용법 표시
            cv2.putText(display_image, "Left-click on face to register | ESC: Exit", 
                      (10, display_image.shape[0] - 10), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # 이미지 표시
            cv2.imshow("Multi-Face Recognition Test", display_image)
            
            # 키보드 입력 처리
            key = cv2.waitKey(1) & 0xFF
            
            # ESC 키를 누르면 종료
            if key == 27:  # ESC
                break
                
    except KeyboardInterrupt:
        print("사용자에 의해 프로그램 종료")
    except Exception as e:
        print(f"오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if cap:
            cap.release()
        cv2.destroyAllWindows()
        
        # 최종 통계 출력
        print("\n=== 다중 얼굴 인식 테스트 결과 ===")
        print(f"총 처리 프레임: {frames_processed}")
        print(f"초당 프레임 수: {fps:.1f}")
        print(f"등록된 얼굴 수: {len(known_face_ids)}")
        print(f"최대 동시 감지 얼굴 수: {max_faces_in_frame}")
        print(f"총 감지된 얼굴 수: {total_faces_detected}")
        print(f"식별된 얼굴 수: {known_faces_detected}")
        
        print("\n얼굴 ID별 감지 횟수:")
        for face_id, count in sorted(face_stats.items(), key=lambda x: x[1], reverse=True):
            print(f"  {face_id}: {count}회 감지됨")

if __name__ == "__main__":
    main()