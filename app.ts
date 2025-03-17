import mqtt from "mqtt";
import aedes from "aedes";
import { createServer } from "net";
import { Client } from "aedes";
import { MqttClient } from "mqtt";

// MQTT 브로커 서버 설정
const port: number = 1883;
const aedesInstance = new aedes();
const server = createServer(aedesInstance.handle);

// 온도 분석 설정
const TEMP_THRESHOLD: number = 37.5; // 온도 임계값 (celsius)
const TEMP_CHECK_INTERVAL: number = 5000; // 연속 고온 체크 간격 (ms)
let highTempCount: number = 0; // 연속 고온 측정 횟수
const HIGH_TEMP_THRESHOLD: number = 3; // 연속 고온 임계값

// MQTT 토픽
const TEMP_TOPIC: string = "fan/temperature";
const COMMAND_TOPIC: string = "fan/command";
const STATUS_TOPIC: string = "fan/status";
const OBJECT_DETECTION_TOPIC: string = "detection/object"; // 객체 탐지 모듈에서 메시지 수신 토픽

// 디바이스 ID
const FAN_MODULE_ID: string = "fan_control_module";
const OBJECT_DETECTION_MODULE_ID: string = "object_detection_module";

// 시스템 상태
type SystemState = "measuring" | "rotating" | "detected";
let systemState: SystemState = "measuring";
let lastTemperature: number = 0;
let lastTempReceivedTime: number = 0;

// 디바이스 정보 인터페이스
interface DeviceInfo {
  lastSeen: number;
  status: string;
  capabilities?: string[];
}

// 연결된 장치 목록
let connectedDevices: Map<string, DeviceInfo> = new Map();

// 서버 시작
server.listen(port, function () {
  console.log(`MQTT 브로커 서버가 포트 ${port}에서 시작되었습니다.`);
});

// 클라이언트 연결 이벤트
aedesInstance.on("client", function (client: Client) {
  console.log(`클라이언트 연결됨: ${client.id}`);
});

// 클라이언트 연결 해제 이벤트
aedesInstance.on("clientDisconnect", function (client: Client) {
  console.log(`클라이언트 연결 해제됨: ${client.id}`);

  // 연결 해제된 디바이스 상태 업데이트
  if (connectedDevices.has(client.id)) {
    connectedDevices.delete(client.id);
    console.log(`디바이스 ${client.id} 연결 해제됨`);
  }
});

// MQTT 브로커에 연결
const mqttClient: MqttClient = mqtt.connect(`mqtt://localhost:${port}`);

// MQTT 연결 이벤트
mqttClient.on("connect", function () {
  console.log("MQTT 브로커에 연결됨");

  // 토픽 구독
  mqttClient.subscribe(TEMP_TOPIC, function (err: Error | null) {
    if (!err) {
      console.log(`${TEMP_TOPIC} 토픽 구독 성공`);
    }
  });

  mqttClient.subscribe(STATUS_TOPIC, function (err: Error | null) {
    if (!err) {
      console.log(`${STATUS_TOPIC} 토픽 구독 성공`);
    }
  });

  mqttClient.subscribe(OBJECT_DETECTION_TOPIC, function (err: Error | null) {
    if (!err) {
      console.log(`${OBJECT_DETECTION_TOPIC} 토픽 구독 성공`);
    }
  });
});

// 메시지 데이터 인터페이스
interface TemperatureData {
  device_id: string;
  temperature: number;
  timestamp: number;
}

interface StatusData {
  device_id: string;
  status: string;
  capabilities?: string[];
}

interface ObjectDetectionData {
  device_id: string;
  detected: boolean;
}

// MQTT 메시지 수신 이벤트
mqttClient.on("message", function (topic: string, message: Buffer) {
  console.log(`메시지 수신: ${topic} - ${message.toString()}`);

  try {
    const data = JSON.parse(message.toString());

    // 온도 데이터 처리 (팬 제어 모듈에서)
    if (
      topic === TEMP_TOPIC &&
      data.device_id === FAN_MODULE_ID &&
      data.temperature !== undefined
    ) {
      processTemperatureData(data.temperature);
    }

    // 상태 업데이트 처리
    else if (topic === STATUS_TOPIC) {
      processStatusUpdate(data as StatusData);
    }

    // 객체 탐지 데이터 처리 (객체 탐지 모듈에서)
    else if (
      topic === OBJECT_DETECTION_TOPIC &&
      data.device_id === OBJECT_DETECTION_MODULE_ID
    ) {
      processObjectDetection(data as ObjectDetectionData);
    }
  } catch (error) {
    console.error("메시지 처리 중 오류 발생:", error);
  }
});

// 온도 데이터 처리 함수
function processTemperatureData(temperature: number): void {
  lastTemperature = temperature;
  lastTempReceivedTime = Date.now();

  console.log(`현재 온도: ${temperature}°C`);

  // 시스템이 온도 측정 상태일 때만 분석
  if (systemState === "measuring") {
    // 임계값 이상인 경우
    if (temperature >= TEMP_THRESHOLD) {
      highTempCount++;
      console.log(`고온 감지 (${highTempCount}/${HIGH_TEMP_THRESHOLD})`);

      // 연속 고온 측정 횟수가 임계값 이상인 경우
      if (highTempCount >= HIGH_TEMP_THRESHOLD) {
        console.log("지속적인 고온 감지! 선풍기 회전 명령 전송");

        // 선풍기 회전 명령 전송
        sendCommand(FAN_MODULE_ID, "rotate_fan");

        // 상태 변경
        systemState = "rotating";
        highTempCount = 0;
      }
    } else {
      // 임계값 미만인 경우 카운트 초기화
      if (highTempCount > 0) {
        console.log("온도가 정상 범위로 돌아옴, 카운트 초기화");
        highTempCount = 0;
      }
    }
  }
}

// 상태 업데이트 처리 함수
function processStatusUpdate(data: StatusData): void {
  console.log(`상태 업데이트 수신: ${JSON.stringify(data)}`);

  // 디바이스 연결 정보 업데이트
  if (data.device_id) {
    connectedDevices.set(data.device_id, {
      lastSeen: Date.now(),
      status: data.status,
      capabilities: data.capabilities,
    });
  }
}

// 객체 탐지 데이터 처리 함수
function processObjectDetection(data: ObjectDetectionData): void {
  console.log(`객체 탐지 데이터 수신: ${JSON.stringify(data)}`);

  // 객체 탐지 모듈에서 사람이 감지되었다는 메시지를 받은 경우
  if (data.detected === true && systemState === "rotating") {
    console.log("객체 탐지 모듈에서 사람 감지됨, 선풍기 정지 명령 전송");

    // 선풍기 정지 명령 전송
    sendCommand(FAN_MODULE_ID, "stop_rotation");

    // 일정 시간 후 온도 측정 재개
    setTimeout(() => {
      console.log("온도 측정 재개 명령 전송");
      sendCommand(FAN_MODULE_ID, "resume_temp_measurement");
      systemState = "measuring";
    }, 5000); // 5초 대기

    systemState = "detected"; // 중간 상태로 변경
  }
}

// 명령 전송 함수
function sendCommand(targetDevice: string, action: string): void {
  const command = {
    target_device: targetDevice,
    action: action,
    timestamp: Date.now(),
  };
  mqttClient.publish(COMMAND_TOPIC, JSON.stringify(command));
  console.log(`명령 전송: ${targetDevice} - ${action}`);
}

// 연결 및 상태 모니터링
setInterval(() => {
  const currentTime = Date.now();

  // 연결된 디바이스 상태 확인
  for (const [deviceId, deviceInfo] of connectedDevices.entries()) {
    // 30초 이상 상태 업데이트가 없는 경우
    if (currentTime - deviceInfo.lastSeen > 30000) {
      console.log(
        `경고: 디바이스 ${deviceId}의 상태 업데이트가 30초 이상 없습니다.`
      );
    }
  }

  // 팬 제어 모듈에서 15초 이상 온도 데이터를 받지 못한 경우 (시스템이 온도 측정 상태일 때만)
  if (
    systemState === "measuring" &&
    currentTime - lastTempReceivedTime > 15000 &&
    lastTempReceivedTime !== 0
  ) {
    console.log("연결 문제 감지: 장시간 온도 데이터 수신 없음");
    // 필요시 연결 복구 로직 추가
  }
}, 10000); // 10초마다 체크

// 서버 상태 로깅
console.log("IoT 선풍기 시스템 연산 서버가 시작되었습니다.");
console.log(`- 온도 임계값: ${TEMP_THRESHOLD}°C`);
console.log(`- 고온 지속 임계값: ${HIGH_TEMP_THRESHOLD}회`);
