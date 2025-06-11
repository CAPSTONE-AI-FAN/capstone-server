// server.js - MQTT broker and controller in JavaScript
const mqtt = require("mqtt");
// 서버 측 aedes 초기화 수정
const aedes = require("aedes")({
  id: "aedes-server",
  concurrency: 1000,
  maxClientsIdLength: 256, // 클라이언트 ID 길이 제한 확대
  heartbeatInterval: 60000,
  connectTimeout: 30000,

  // 인증 부분에서 클라이언트 ID를 항상 허용
  authenticate: function (client, username, password, callback) {
    console.log(`인증 시도: ID=${client.id}, 사용자=${username || "aifan"}`);

    // 무조건 인증 허용
    callback(null, true);
  },
});

// 클라이언트 ID 충돌 처리 - 기존 연결 강제 종료하지 않음
aedes.on("clientReady", function (client) {
  console.log(`클라이언트 준비: ${client.id}`);
});

// 클라이언트 검증 로직을 완전 오픈
aedes.validateClient = function (client, callback) {
  console.log(`클라이언트 검증: ${client.id}`);
  callback(null, true);
};
const { createServer } = require("net");
const { createServer: createHttpServer } = require("http");
const websocketStream = require("websocket-stream");
// MQTT broker server configuration
const CONFIG = {
  BROKER: {
    PORT: 1883,
    HOST: "0.0.0.0", // Listen on all interfaces
    WS_PORT: 8883, // WebSocket port for web clients
  },
  TEMPERATURE: {
    THRESHOLD: 37.5,
    CHECK_INTERVAL: 5000,
    HIGH_THRESHOLD_COUNT: 3,
  },
  TOPICS: {
    COMMAND: "device/command",
    STATUS: "device/status",
    CONTROL: {
      PREFIX: "control",
      ROTATION: "control/rotation",
      DIRECTION: "control/direction",
      AUTO_MODE: "control/auto_mode",
      STATUS: "control/status",
    },
    LIDAR: {
      DATA: "sensor/lidar/data",
      SCAN: "sensor/lidar/scan",
      COMMAND: "sensor/lidar/command",
      STATUS: "sensor/lidar/status",
    },
    DETECTION: "sensor/detection",
    SYSTEM: "system/status",
    LOG: "system/log",
    // 얼굴 인식 관련 토픽 추가
    FACE: {
      THERMAL: "sensor/face/thermal", // 얼굴+온도 통합 데이터용 토픽만 유지
      GRID: "sensor/face/grid",
      COMMAND: "sensor/face/command", // 얼굴 인식 모듈 제어 명령
      STATUS: "sensor/face/status", // 얼굴 인식 모듈 상태
      REGISTER: "sensor/face/register", // 새 얼굴 등록 요청/응답
    },
    FAN: {
      COMMAND: "fan/command",
      STATUS: "fan/status",
      ROTATION: "fan/rotation",
      SPEED: "fan/speed",
      MODE: "fan/mode",
    },
  },
  DEVICE_IDS: {
    FAN_MODULE: "fan_control_module",
    SENSOR_MODULE: "sensor_module",
    OBJECT_DETECTION: "object_detection_module",
    LIDAR_MODULE: "ydlidar_module",
    SERVER: "server_controller",
    MOBILE_PREFIX: "mobile_client_",
    // 얼굴 인식 모듈 ID 추가
    FACE_RECOGNITION: "face_recognition_module",
  },
  HEARTBEAT: {
    INTERVAL: 15000,
    TIMEOUT: 30000,
  },
};

// System state management
let systemState = "idle";
let lastTemperature = 0;
let lastTempReceivedTime = 0;
let highTempCount = 0;
let systemStartTime = Date.now();

let statusUpdateTimer = null; // 상태 업데이트 타이머 참조
let shutdownInProgress = false; // 종료 진행 중 플래그

// 에러 처리를 위한 카운터 변수
let errorCount = 0;
let lastErrorTime = 0;

// Connected devices tracking with connection type
let connectedDevices = new Map();

// Websocket client tracking
let websocketClients = new Set();

let lidarData = {
  lastUpdate: 0,
  scans: {}, // 각도를 키로 사용하는 거리 값 맵
  scanBuffer: [], // 최근 스캔 데이터 버퍼 (시간순)
  bufferSize: 180, // 버퍼 크기 (약 180도 범위)
  nearestObjects: [], // 가장 가까운 물체들
};

// Face recognition state variables
let faceRecognitionState = {
  lastUpdate: 0,
  recognizedFaces: [],
  gridMap: [
    [null, null, null],
    [null, null, null],
    [null, null, null],
  ],
  thermalData: null,
  autoRegister: true,
  lastRegisteredFace: null,
  connectedModules: new Set(),
};

// 초기화 함수
function initGlobalVariables() {
  // 전역 변수들이 아직 정의되지 않았다면 정의
  if (typeof shutdownInProgress === "undefined") {
    global.shutdownInProgress = false;
    console.log("Initialized shutdownInProgress flag");
  }

  if (typeof statusUpdateTimer === "undefined") {
    global.statusUpdateTimer = null;
    console.log("Initialized statusUpdateTimer reference");
  }

  if (typeof systemStartTime === "undefined") {
    global.systemStartTime = Date.now();
    console.log("Initialized systemStartTime");
  }

  if (typeof systemState === "undefined") {
    global.systemState = "idle";
    console.log("Initialized systemState");
  }

  if (typeof lastTemperature === "undefined") {
    global.lastTemperature = 0;
    console.log("Initialized lastTemperature");
  }

  // errorCount 변수 초기화
  if (typeof errorCount === "undefined") {
    global.errorCount = 0;
    global.lastErrorTime = 0;
    console.log("Initialized error tracking variables");
  }

  // WebSocket 클라이언트 추적 초기화
  if (typeof websocketClients === "undefined") {
    global.websocketClients = new Set();
    console.log("Initialized WebSocket client tracking");
  }
}

initGlobalVariables();

// Create the TCP server for standard MQTT connections
const mqttServer = createServer(aedes.handle);

// Start the MQTT server (TCP)
mqttServer.listen(CONFIG.BROKER.PORT, CONFIG.BROKER.HOST, function () {
  console.log(`MQTT broker is running on port ${CONFIG.BROKER.PORT}`);
});

// Enhanced WebSocket server setup for mobile clients
const httpServer = createHttpServer();
const wsServer = websocketStream.createServer(
  { server: httpServer },
  function (stream, request) {
    // Add additional WebSocket-specific handling
    const clientIp = request.socket.remoteAddress;
    console.log(`WebSocket connection from ${clientIp}`);

    // Tag this stream as WebSocket for later identification
    stream.isWebSocket = true;
    stream.remoteAddress = clientIp;

    // Let aedes handle the MQTT over WebSocket protocol
    aedes.handle(stream);
  }
);

httpServer.listen(CONFIG.BROKER.WS_PORT, function () {
  console.log(`MQTT-WS broker is running on port ${CONFIG.BROKER.WS_PORT}`);
});

// Global MQTT client reference for server controller
let mqttClient = null;

function isClientReadyToPublish() {
  return (
    mqttClient &&
    mqttClient.connected &&
    !mqttClient.disconnecting &&
    !mqttClient.disconnected &&
    !shutdownInProgress
  );
}

/**
 * 안전한 시스템 로그 발행 함수 (변수 참조 오류 수정)
 */
function publishSystemLog(message, level = "info") {
  // 콘솔에는 항상 출력
  console.log(`[System Log] ${level.toUpperCase()}: ${message}`);

  // 앱이 종료 중인지 확인 (에러를 방지하기 위해 안전한 방법 사용)
  const isShuttingDown =
    typeof global.shutdownInProgress !== "undefined"
      ? global.shutdownInProgress
      : false;

  // 클라이언트를 안전하게 확인
  if (!mqttClient || isShuttingDown) {
    return; // 클라이언트가 없거나 종료 중이면 발행하지 않음
  }

  // 클라이언트 상태 안전하게 확인
  const isConnected =
    typeof mqttClient.connected !== "undefined" ? mqttClient.connected : false;
  const isDisconnecting =
    typeof mqttClient.disconnecting !== "undefined"
      ? mqttClient.disconnecting
      : false;

  if (!isConnected || isDisconnecting) {
    return; // 연결되지 않았거나 연결 종료 중이면 발행하지 않음
  }

  // 로그 메시지 구성
  try {
    // CONFIG 객체를 안전하게 접근
    const deviceId =
      typeof CONFIG !== "undefined" && CONFIG.DEVICE_IDS
        ? CONFIG.DEVICE_IDS.SERVER
        : "server_controller";

    const logTopic =
      typeof CONFIG !== "undefined" && CONFIG.TOPICS
        ? CONFIG.TOPICS.LOG
        : "system/log";

    const logMessage = {
      device_id: deviceId,
      level: level,
      message: message,
      timestamp: Date.now(),
    };

    // 메시지 발행 시도
    mqttClient.publish(logTopic, JSON.stringify(logMessage), { qos: 0 });
  } catch (e) {
    // 발행 중 오류가 발생해도 콘솔에만 기록하고 넘어감
    console.error("Error publishing system log:", e.message);

    // 오류 카운터 증가 (루프 감지)
    if (typeof global.errorCount !== "undefined") {
      global.errorCount++;
      global.lastErrorTime = Date.now();

      // 짧은 시간에 너무 많은 오류가 발생하면 안전 장치 작동
      if (global.errorCount > 10 && Date.now() - global.lastErrorTime < 5000) {
        console.error("Too many publishing errors detected, forcing shutdown");
        process.exit(1);
      }
    }
  }
}

// System state publishing function
function publishSystemState(client, description = "") {
  if (!isClientReadyToPublish()) {
    console.log(
      `[System State] Cannot publish state: ${systemState} (client not ready)`
    );
    return;
  }

  const stateMessage = {
    device_id: CONFIG.DEVICE_IDS.SERVER,
    state: systemState,
    description: description,
    temperature: lastTemperature,
    uptime: Math.floor((Date.now() - systemStartTime) / 1000),
    timestamp: Date.now(),
  };

  try {
    client.publish(CONFIG.TOPICS.SYSTEM, JSON.stringify(stateMessage), {
      qos: 1,
    });
  } catch (e) {
    console.error("Error publishing system state:", e.message);
  }
}

// Enhanced client connection event with WebSocket detection
aedes.on("client", function (client) {
  // Track connection type
  let connectionType = "unknown";
  let clientInfo = `Client connected: ${client.id}`;

  // WebSocket 연결 식별
  if (client.conn && client.conn.isWebSocket) {
    connectionType = "websocket";
    websocketClients.add(client.id);
    clientInfo += ` via WebSocket`;
    if (client.conn.remoteAddress) {
      clientInfo += ` from ${client.conn.remoteAddress}`;
    }
  }
  // TCP 연결 식별
  else if (client.conn && client.conn.remoteAddress) {
    connectionType = "tcp";
    clientInfo += ` via TCP from ${client.conn.remoteAddress}`;
  }

  console.log(clientInfo);
  console.log(`Connection type: ${connectionType}`);

  // Only log after mqttClient is initialized
  if (mqttClient && mqttClient.connected) {
    publishSystemLog(`New connection: ${client.id} (${connectionType})`);
  }
});

// Enhanced client disconnection event with WebSocket detection
aedes.on("clientDisconnect", function (client) {
  // 연결 방식 확인
  let connectionType = websocketClients.has(client.id) ? "websocket" : "tcp";
  console.log(`Client disconnected: ${client.id} (${connectionType})`);

  // WebSocket 클라이언트 추적에서 제거
  if (websocketClients.has(client.id)) {
    websocketClients.delete(client.id);
  }

  if (mqttClient && mqttClient.connected) {
    publishSystemLog(`Client disconnected: ${client.id} (${connectionType})`);
  }

  // Update disconnected device status
  if (connectedDevices.has(client.id)) {
    connectedDevices.delete(client.id);
    console.log(`Device ${client.id} removed from connected devices`);

    // Check if important device disconnected and update system state
    if (client.id === CONFIG.DEVICE_IDS.FAN_MODULE) {
      systemState = "error";
      if (mqttClient && mqttClient.connected) {
        publishSystemState(
          mqttClient,
          "Fan module disconnected, system in error state"
        );
      }
    }
  }
});

// Create controller as local client
setTimeout(() => {
  console.log("Initializing controller client...");

  // Connect to the local MQTT broker with better options
  mqttClient = mqtt.connect(`mqtt://localhost:${CONFIG.BROKER.PORT}`, {
    clientId: CONFIG.DEVICE_IDS.SERVER,
    clean: true,
    reconnectPeriod: 5000,
    connectTimeout: 10000,
    will: {
      topic: CONFIG.TOPICS.SYSTEM,
      payload: JSON.stringify({
        device_id: CONFIG.DEVICE_IDS.SERVER,
        status: "offline",
        timestamp: Date.now(),
      }),
      qos: 1,
      retain: true,
    },
  });

  // Handle connection events with better logging
  mqttClient.on("connect", function () {
    console.log("Controller connected to MQTT broker");
    publishSystemLog("Server controller connected to broker");

    // Subscribe to all relevant topics with wildcards
    const topics = [
      `${CONFIG.TOPICS.TEMP}/#`,
      `${CONFIG.TOPICS.STATUS}/#`,
      `${CONFIG.TOPICS.DETECTION}/#`,
      `${CONFIG.TOPICS.CONTROL.PREFIX}/#`,
      `${CONFIG.TOPICS.LIDAR.DATA}`,
      `${CONFIG.TOPICS.LIDAR.SCAN}`,
      `${CONFIG.TOPICS.LIDAR.STATUS}`,
      // 얼굴 인식 관련 토픽 구독 추가
      `${CONFIG.TOPICS.FACE.THERMAL_DATA}`,
      `${CONFIG.TOPICS.FACE.GRID}`,
      `${CONFIG.TOPICS.FACE.THERMAL}`,
      `${CONFIG.TOPICS.FACE.STATUS}`,
      `${CONFIG.TOPICS.FACE.REGISTER}`,
      "sensor/face/data", // FACE_DETECTION 토픽 추가
      // 새로운 센서 토픽 추가
      "sensor/ultrasonic/data", // 초음파 센서
      "sensor/temperature/data", // 온도 센서
      "movement/status", // 움직임 상태
      "fan/status", // 선풍기 상태
      "control/movement", // 움직임 제어
      "control/fan/rotation", // 선풍기 회전 제어
      "mobile/lidar/update", // 모바일 LiDAR 업데이트
      "device/status", // 장치 상태
      "control/auto_mode", // 자동 모드 제어
      "ping", // Add ping topic for heartbeat
      "ping/request", // Add ping request topic for latency checking
    ];

    topics.forEach((topic) => {
      mqttClient.subscribe(topic, { qos: 1 }, function (err) {
        if (!err) {
          console.log(`Subscribed to ${topic}`);
        } else {
          console.error(`Failed to subscribe to ${topic}: ${err.message}`);
        }
      });
    });

    // Publish server online status as retained message
    publishStatus(mqttClient, "online", true);

    // Publish initial system state
    systemState = "measuring";
    publishSystemState(mqttClient, "System initialized and ready");
  });

  // Enhanced error handling
  mqttClient.on("error", function (error) {
    console.error("MQTT client error:", error);
    publishSystemLog(`MQTT client error: ${error.message}`, "error");
  });

  mqttClient.on("offline", function () {
    console.warn("MQTT client is offline");
    publishSystemLog("Server controller offline", "warning");
  });

  mqttClient.on("reconnect", function () {
    console.log("MQTT client is reconnecting");
    publishSystemLog("Server controller reconnecting", "info");
  });

  // Enhanced message handling with JSON validation and connection type awareness
  mqttClient.on("message", function (topic, message) {
    const messageStr = message.toString();
    console.log(`Message received: ${topic} - ${messageStr}`);

    try {
      // Handle ping topics for connectivity checks
      if (topic === "ping") {
        console.log("Received ping from client");
        return;
      }

      // Handle ping request (used for latency checking)
      if (topic === "ping/request") {
        handlePingRequest(messageStr);
        return;
      }

      // Validate message is proper JSON
      const data = JSON.parse(messageStr);

      // Check for required fields
      if (!data.device_id) {
        console.warn(`Received message without device_id on topic ${topic}`);
        return;
      }

      // Handle connection type info if available
      if (data.connection_type) {
        console.log(
          `Client ${data.device_id} using connection type: ${data.connection_type}`
        );
      }

      // Route to appropriate handler based on topic
      if (topic === CONFIG.TOPICS.FACE.THERMAL) {
        handleFaceThermalData(mqttClient, data); // client 인자 전달
      } else if (topic === CONFIG.TOPICS.STATUS) {
        handleStatusUpdate(data);
      } else if (topic.startsWith(CONFIG.TOPICS.DETECTION)) {
        handleDetectionData(mqttClient, data);
      } else if (topic.startsWith(CONFIG.TOPICS.CONTROL.PREFIX)) {
        handleControlCommand(mqttClient, topic, data);
      } else if (topic === "sensor/face/data") {
        handleFaceDetectionData(mqttClient, data);
      } else if (
        topic === CONFIG.TOPICS.LIDAR.DATA ||
        topic === CONFIG.TOPICS.LIDAR.SCAN
      ) {
        handleLidarData(mqttClient, data);
      } else if (topic === CONFIG.TOPICS.LIDAR.STATUS) {
        handleLidarStatus(data);
      } else if (topic === CONFIG.TOPICS.FACE.GRID) {
        handleFaceGridData(data);
      } else if (topic === CONFIG.TOPICS.FACE.STATUS) {
        handleFaceModuleStatus(mqttClient, data);
      } else if (topic === CONFIG.TOPICS.FACE.REGISTER) {
        handleFaceRegistration(mqttClient, data);
      } else if (topic === "sensor/ultrasonic/data") {
        handleUltrasonicData(mqttClient, data);
      } else if (topic === "sensor/temperature/data") {
        handleTemperatureData(mqttClient, data);
      } else if (topic === "movement/status") {
        handleMovementStatus(data);
      } else if (topic === "fan/status") {
        handleFanStatus(data);
      } else if (topic === "control/movement") {
        handleMovementControl(mqttClient, data);
      } else if (topic === "control/fan/rotation") {
        handleFanControl(mqttClient, data);
      } else if (topic === "mobile/lidar/update") {
        handleMobileLidarUpdate(mqttClient, data);
      } else if (topic === "device/status") {
        handleDeviceStatus(mqttClient, data);
      } else if (topic === "control/auto_mode") {
        handleAutoModeControl(mqttClient, data);
      }
    } catch (error) {
      console.error("Error processing message:", error);
      publishSystemLog(
        `Error processing message on ${topic}: ${error}`,
        "error"
      );
    }
  });

  // Handle ping request (for latency checking)
  function handlePingRequest(messageStr) {
    try {
      const data = JSON.parse(messageStr);

      if (data.device_id && data.response_topic) {
        console.log(
          `Ping request from ${data.device_id}, responding on ${data.response_topic}`
        );

        // Reply with the same timestamp for latency calculation
        mqttClient.publish(data.response_topic, messageStr, { qos: 0 });
      }
    } catch (e) {
      console.error("Error processing ping request:", e);
    }
  }

  // Handle status updates with device type awareness and connection type
  function handleStatusUpdate(data) {
    console.log(`Status update received: ${JSON.stringify(data)}`);

    // 연결 방식 추적
    const connectionType = data.connection_type || "unknown";

    // Update device connection information
    if (data.device_id) {
      connectedDevices.set(data.device_id, {
        lastSeen: Date.now(),
        status: data.status || "unknown",
        type: data.client_type || "unknown",
        connectionType: connectionType,
        capabilities: data.capabilities,
        ip: data.ip,
      });

      // Log new device connection
      if (
        data.status === "connected" &&
        !data.device_id.startsWith(CONFIG.DEVICE_IDS.SERVER)
      ) {
        publishSystemLog(
          `Device ${data.device_id} is now ${data.status} (${connectionType})`
        );
      }
    }
  }

  // Handle detection data with improved validation
  function handleDetectionData(client, data) {
    console.log(
      `Face detection data received from ${data.device_id}: ${
        data.faces?.length || 0
      } faces`
    );

    // 데이터 유효성 검사
    if (!data.faces || !Array.isArray(data.faces)) {
      console.warn("Received face detection data without valid faces array");
      return;
    }

    // 얼굴이 감지된 경우
    if (data.faces.length > 0) {
      // 가장 큰 얼굴 찾기 (face_box의 크기로 판단)
      let largestFace = null;
      let maxArea = 0;

      data.faces.forEach((face, index) => {
        if (face.face_box && face.face_box.length === 4) {
          const [x1, y1, x2, y2] = face.face_box;
          const area = (x2 - x1) * (y2 - y1);
          console.log(
            `Face ${index}: position [${x1}, ${y1}, ${x2}, ${y2}], area: ${area}`
          );
          if (area > maxArea) {
            maxArea = area;
            largestFace = face;
          }
        } else {
          console.warn(
            `Face ${index}: invalid face_box format:`,
            face.face_box
          );
        }
      });

      if (largestFace) {
        // 얼굴의 중심점 계산
        const [x1, y1, x2, y2] = largestFace.face_box;
        const centerX = (x1 + x2) / 2;
        const centerY = (y1 + y2) / 2;

        // 이미지 너비를 기준으로 x 좌표 정규화 (0~1 사이의 값으로 변환)
        const normalizedX = centerX / 640; // 640은 이미지 너비, 필요에 따라 조정

        // 화면을 3등분하여 방향 결정
        let direction;
        if (normalizedX < 0.33) {
          direction = "left";
        } else if (normalizedX > 0.66) {
          direction = "right";
        } else {
          direction = "center";
        }

        // 디버그 로깅 추가
        console.log(`=== FACE TRACKING DEBUG ===`);
        console.log(`Face box: [${x1}, ${y1}, ${x2}, ${y2}]`);
        console.log(
          `Face center: (${centerX.toFixed(2)}, ${centerY.toFixed(2)})`
        );
        console.log(
          `Normalized X: ${normalizedX.toFixed(3)} (${(
            normalizedX * 100
          ).toFixed(1)}%)`
        );
        console.log(
          `Direction zones: left(0-33%), center(33-66%), right(66-100%)`
        );
        console.log(`Calculated direction: ${direction}`);
        console.log(`========================`);

        // 팬 회전 명령 발행 (center 포함)
        const command = {
          device_id: CONFIG.DEVICE_IDS.SERVER,
          action: "set_direction",
          direction: direction,
          face_id: largestFace.face_id || "unknown",
          confidence: largestFace.confidence || 0,
          position: {
            x: centerX,
            y: centerY,
            normalized_x: normalizedX,
          },
          timestamp: Date.now(),
        };

        console.log(
          `Publishing fan rotation command to topic: ${CONFIG.TOPICS.CONTROL.DIRECTION}`
        );
        console.log(`Command payload:`, JSON.stringify(command, null, 2));

        try {
          const result = client.publish(
            CONFIG.TOPICS.CONTROL.DIRECTION,
            JSON.stringify(command),
            { qos: 1 }
          );
          console.log(
            `Command publish result: ${result ? "SUCCESS" : "FAILED"}`
          );
        } catch (error) {
          console.error(`Error publishing command:`, error);
        }

        console.log(
          `✅ Published fan rotation command: ${direction} for face ${
            largestFace.face_id
          } at position (${centerX.toFixed(2)}, ${centerY.toFixed(2)})`
        );
      } else {
        console.warn("No valid face found with proper face_box");
      }
    } else {
      console.log("No faces detected - no rotation command sent");
    }
  }

  // New function to handle control commands from mobile clients
  function handleControlCommand(client, topic, data) {
    // 연결 방식 추적
    const connectionType = data.connection_type || "unknown";
    console.log(
      `Control command received: ${topic} - ${JSON.stringify(
        data
      )} via ${connectionType}`
    );

    // Remove device validation - allow all devices to send control commands
    console.log(`Control command accepted from device: ${data.device_id}`);

    // Forward command to appropriate device based on topic
    if (topic === CONFIG.TOPICS.CONTROL.ROTATION) {
      // Forward rotation command to fan module
      sendCommand(client, CONFIG.DEVICE_IDS.FAN_MODULE, "set_rotation", {
        angle: data.angle,
      });
    } else if (topic === CONFIG.TOPICS.CONTROL.DIRECTION) {
      // Forward direction command to fan module
      sendCommand(client, CONFIG.DEVICE_IDS.FAN_MODULE, "set_direction", {
        direction: data.direction,
      });
    } else if (topic === CONFIG.TOPICS.CONTROL.AUTO_MODE) {
      // Handle auto mode toggle
      if (data.mode === "enable_autonomous") {
        systemState = "measuring";
        publishSystemState(client, "Auto mode enabled, monitoring temperature");
      } else {
        systemState = "idle";
        publishSystemState(client, "Manual control mode active");
      }

      // Forward mode command to fan module
      sendCommand(client, CONFIG.DEVICE_IDS.FAN_MODULE, "set_mode", {
        mode: data.mode,
      });
    }
  }

  // YDLIDAR 데이터 처리 함수
  function handleLidarData(client, data) {
    console.log(`[DEBUG] LIDAR 처리 시작: ${JSON.stringify(data)}`);

    // Remove device ID validation - allow all devices to send LIDAR data
    console.log(`LIDAR data accepted from device: ${data.device_id}`);

    // 데이터 유효성 검사
    if (data.angle === undefined || data.distance === undefined) {
      console.warn("Received lidar data without angle or distance values");
      return;
    }

    // 현재 시간 기록
    const currentTime = Date.now();
    lidarData.lastUpdate = currentTime;

    // 각도를 키로 사용하여 스캔 데이터 저장 (소수점 반올림)
    const angleKey = Math.round(data.angle * 10) / 10; // 소수점 첫째 자리까지
    lidarData.scans[angleKey] = data.distance;

    // 새로운 스캔 데이터를 버퍼에 추가
    lidarData.scanBuffer.push({
      angle: angleKey,
      distance: data.distance,
      timestamp: currentTime,
    });

    // 버퍼 크기 제한
    if (lidarData.scanBuffer.length > lidarData.bufferSize) {
      lidarData.scanBuffer.shift(); // 가장 오래된 데이터 제거
    }

    // 주기적으로 가장 가까운 물체 계산 (모든 데이터를 처리하지 않고 10개마다 처리)
    if (lidarData.scanBuffer.length % 10 === 0) {
      updateNearestObjects();
    }

    // 모바일 클라이언트에 실시간 데이터 전송 (필요한 경우)
    if (lidarData.scanBuffer.length % 5 === 0) {
      // 5개 데이터마다 발행
      publishLidarUpdate(client);
    }

    console.log(
      `[DEBUG] LIDAR 상태: 스캔 수=${
        Object.keys(lidarData.scans).length
      }, 가장 가까운 객체=${lidarData.nearestObjects.length}`
    );
  }

  // 가장 가까운 물체들 업데이트
  function updateNearestObjects() {
    // 유효한 스캔만 필터링 (0이 아닌 거리)
    const validScans = Object.entries(lidarData.scans)
      .filter(([_, distance]) => distance > 0)
      .map(([angle, distance]) => ({
        angle: parseFloat(angle),
        distance,
      }));

    if (validScans.length === 0) return;

    // 거리순 정렬
    validScans.sort((a, b) => a.distance - b.distance);

    // 가장 가까운 10개 물체 저장
    lidarData.nearestObjects = validScans.slice(0, 10);

    console.log(
      `Updated nearest objects. Closest: ${lidarData.nearestObjects[0]?.angle}° at ${lidarData.nearestObjects[0]?.distance}m`
    );
  }

  // 모바일 클라이언트에 LIDAR 데이터 발행
  function publishLidarUpdate(client) {
    if (!isClientReadyToPublish()) {
      return;
    }

    // 모바일 앱에 전송할 데이터 구성
    const updateMessage = {
      device_id: CONFIG.DEVICE_IDS.SERVER,
      timestamp: Date.now(),
      scan_data: lidarData.scanBuffer.slice(-20), // 최근 20개 데이터만 전송
      nearest_objects: lidarData.nearestObjects.slice(0, 5), // 가장 가까운 5개 물체
    };

    // 모바일 앱용 토픽으로 발행
    client.publish("mobile/lidar/update", JSON.stringify(updateMessage), {
      qos: 0,
    });
  }

  // YDLIDAR 상태 처리 함수
  function handleLidarStatus(data) {
    console.log(`LIDAR status update: ${JSON.stringify(data)}`);

    // 상태 업데이트 처리
    if (data.device_id === CONFIG.DEVICE_IDS.LIDAR_MODULE) {
      // 연결된 디바이스 목록 업데이트
      connectedDevices.set(data.device_id, {
        lastSeen: Date.now(),
        status: data.status || "unknown",
        type: "lidar_sensor",
        connectionType: data.connection_type || "unknown",
      });

      // 상태 변경 로그
      if (data.status) {
        publishSystemLog(`LIDAR module is now ${data.status}`);
      }
    }
  }

  // Send command to a device with additional parameters
  function sendCommand(client, targetDevice, action, params = {}) {
    const command = {
      target_device: targetDevice,
      action: action,
      params: params,
      source: CONFIG.DEVICE_IDS.SERVER,
      timestamp: Date.now(),
    };

    client.publish(CONFIG.TOPICS.COMMAND, JSON.stringify(command), { qos: 1 });
    console.log(`Command sent to ${targetDevice}: ${action}`);
  }

  // Publish server status
  function publishStatus(client, status, retain = false) {
    if (!isClientReadyToPublish()) {
      console.log(
        `[Status] Cannot publish status: ${status} (client not ready)`
      );
      return;
    }

    const statusMessage = {
      device_id: CONFIG.DEVICE_IDS.SERVER,
      status: status,
      uptime: Math.floor((Date.now() - systemStartTime) / 1000),
      timestamp: Date.now(),
    };

    try {
      client.publish(CONFIG.TOPICS.STATUS, JSON.stringify(statusMessage), {
        qos: 1,
        retain: retain,
      });
    } catch (e) {
      console.error("Error publishing status:", e.message);
    }
  }

  // Improved status monitoring interval with better device tracking
  statusUpdateTimer = setInterval(() => {
    if (shutdownInProgress) {
      clearInterval(statusUpdateTimer);
      statusUpdateTimer = null;
      return;
    }

    const currentTime = Date.now();

    // Check connected devices
    for (const [deviceId, deviceInfo] of connectedDevices.entries()) {
      // Device hasn't sent an update in too long
      if (currentTime - deviceInfo.lastSeen > CONFIG.HEARTBEAT.TIMEOUT) {
        console.log(
          `Warning: No status update from device ${deviceId} for more than ${
            CONFIG.HEARTBEAT.TIMEOUT / 1000
          } seconds`
        );

        // If critical device is not responsive, mark system in error state
        if (
          deviceId === CONFIG.DEVICE_IDS.FAN_MODULE ||
          deviceId === CONFIG.DEVICE_IDS.SENSOR_MODULE
        ) {
          systemState = "error";
          publishSystemState(
            mqttClient,
            `Lost connection to critical device: ${deviceId}`
          );
        }

        // Remove stale device
        connectedDevices.delete(deviceId);
        publishSystemLog(
          `Device ${deviceId} timed out and was removed`,
          "warning"
        );
      }
    }

    // Check if we're missing temperature data
    if (
      systemState === "measuring" &&
      currentTime - lastTempReceivedTime > CONFIG.HEARTBEAT.TIMEOUT &&
      lastTempReceivedTime !== 0
    ) {
      console.log(
        "Connection issue detected: No temperature data received for an extended period"
      );
      publishSystemLog("No temperature data received", "warning");
    }

    // Publish periodic status update
    //publishStatus(mqttClient, "online");

    // Publish device list for monitoring with connection type info
    const deviceListMessage = {
      device_id: CONFIG.DEVICE_IDS.SERVER,
      connected_devices: Array.from(connectedDevices.entries()).map(
        ([id, info]) => ({
          id,
          type: info.type,
          status: info.status,
          connection_type: info.connectionType || "unknown",
          last_seen: info.lastSeen,
        })
      ),
      websocket_clients: Array.from(websocketClients),
      timestamp: currentTime,
    };

    mqttClient.publish("system/devices", JSON.stringify(deviceListMessage), {
      qos: 0,
    });
  }, 10000);

  // 얼굴 인식 데이터 처리 함수 추가
  function handleFaceDetectionData(client, data) {
    console.log(
      `Face detection data received from ${data.device_id}: ${
        data.faces?.length || 0
      } faces`
    );

    // 데이터 유효성 검사
    if (!data.faces || !Array.isArray(data.faces)) {
      console.warn("Received face detection data without valid faces array");
      return;
    }

    // 얼굴이 감지된 경우
    if (data.faces.length > 0) {
      // 가장 큰 얼굴 찾기 (face_box의 크기로 판단)
      let largestFace = null;
      let maxArea = 0;

      data.faces.forEach((face, index) => {
        if (face.face_box && face.face_box.length === 4) {
          const [x1, y1, x2, y2] = face.face_box;
          const area = (x2 - x1) * (y2 - y1);
          console.log(
            `Face ${index}: position [${x1}, ${y1}, ${x2}, ${y2}], area: ${area}`
          );
          if (area > maxArea) {
            maxArea = area;
            largestFace = face;
          }
        } else {
          console.warn(
            `Face ${index}: invalid face_box format:`,
            face.face_box
          );
        }
      });

      if (largestFace) {
        // 얼굴의 중심점 계산
        const [x1, y1, x2, y2] = largestFace.face_box;
        const centerX = (x1 + x2) / 2;
        const centerY = (y1 + y2) / 2;

        // 이미지 너비를 기준으로 x 좌표 정규화 (0~1 사이의 값으로 변환)
        const normalizedX = centerX / 640; // 640은 이미지 너비, 필요에 따라 조정

        // 화면을 3등분하여 방향 결정
        let direction;
        if (normalizedX < 0.33) {
          direction = "left";
        } else if (normalizedX > 0.66) {
          direction = "right";
        } else {
          direction = "center";
        }

        // 디버그 로깅 추가
        console.log(`=== FACE TRACKING DEBUG ===`);
        console.log(`Face box: [${x1}, ${y1}, ${x2}, ${y2}]`);
        console.log(
          `Face center: (${centerX.toFixed(2)}, ${centerY.toFixed(2)})`
        );
        console.log(
          `Normalized X: ${normalizedX.toFixed(3)} (${(
            normalizedX * 100
          ).toFixed(1)}%)`
        );
        console.log(
          `Direction zones: left(0-33%), center(33-66%), right(66-100%)`
        );
        console.log(`Calculated direction: ${direction}`);
        console.log(`========================`);

        // 팬 회전 명령 발행 (center 포함)
        const command = {
          device_id: CONFIG.DEVICE_IDS.SERVER,
          action: "set_direction",
          direction: direction,
          face_id: largestFace.face_id || "unknown",
          confidence: largestFace.confidence || 0,
          position: {
            x: centerX,
            y: centerY,
            normalized_x: normalizedX,
          },
          timestamp: Date.now(),
        };

        console.log(
          `Publishing fan rotation command to topic: ${CONFIG.TOPICS.CONTROL.DIRECTION}`
        );
        console.log(`Command payload:`, JSON.stringify(command, null, 2));

        try {
          const result = client.publish(
            CONFIG.TOPICS.CONTROL.DIRECTION,
            JSON.stringify(command),
            { qos: 1 }
          );
          console.log(
            `Command publish result: ${result ? "SUCCESS" : "FAILED"}`
          );
        } catch (error) {
          console.error(`Error publishing command:`, error);
        }

        console.log(
          `✅ Published fan rotation command: ${direction} for face ${
            largestFace.face_id
          } at position (${centerX.toFixed(2)}, ${centerY.toFixed(2)})`
        );
      } else {
        console.warn("No valid face found with proper face_box");
      }
    } else {
      console.log("No faces detected - no rotation command sent");
    }
  }

  /**
   * 얼굴 인식 데이터 처리 함수
   */
  function handleFaceThermalData(client, data) {
    // 데이터 유효성 검사
    if (!data) {
      console.warn("Received empty face thermal data");
      return;
    }

    try {
      // 디바이스 ID 검증
      if (!data.device_id) {
        console.warn("Face thermal data missing device_id");
        return;
      }

      // Remove device authorization check - allow all devices
      console.log(`Face thermal data accepted from device: ${data.device_id}`);

      // 얼굴 배열 확인
      if (!data.faces || !Array.isArray(data.faces)) {
        console.warn("Face thermal data missing valid faces array");
        return;
      }

      console.log(
        `Processing face thermal data: ${data.faces.length} faces detected`
      );

      // 얼굴 데이터 업데이트
      faceRecognitionState.lastUpdate = Date.now();
      faceRecognitionState.recognizedFaces = data.faces.map((face) => ({
        ...face,
        lastSeen: Date.now(),
      }));

      // 온도 데이터 처리
      let highTemperatureDetected = false;
      let highestTemperature = 0;

      // 각 얼굴의 온도 확인 및 처리
      data.faces.forEach((face) => {
        if (face.temperature && face.temp_confidence !== "low") {
          // 온도 로깅
          console.log(
            `Face ${face.face_id}: Temperature ${face.temperature}°C (${face.temp_confidence} confidence)`
          );

          // 온도 임계값 확인
          if (face.temperature >= CONFIG.TEMPERATURE.THRESHOLD) {
            highTemperatureDetected = true;
            highestTemperature = Math.max(highestTemperature, face.temperature);
          }

          // 마지막 온도 업데이트
          lastTemperature = face.temperature;
          lastTempReceivedTime = Date.now();
        }
      });

      // 고온 감지 시 처리
      if (highTemperatureDetected && systemState === "measuring") {
        highTempCount++;
        console.log(
          `High temperature detected (${highTempCount}/${CONFIG.TEMPERATURE.HIGH_THRESHOLD_COUNT})`
        );

        // 임계값 도달 시 팬 회전 명령 등 처리
        if (highTempCount >= CONFIG.TEMPERATURE.HIGH_THRESHOLD_COUNT) {
          console.log(
            `Persistent high temperature detected: ${highestTemperature}°C! Sending fan rotation command`
          );

          // 다중 얼굴 감지 시 가장 뜨거운 얼굴 찾기 및 방향 결정
          let hottestFace = null;
          let maxTemp = 0;

          data.faces.forEach((face) => {
            if (face.temperature && face.temperature > maxTemp) {
              maxTemp = face.temperature;
              hottestFace = face;
            }
          });

          if (
            hottestFace &&
            hottestFace.face_box &&
            hottestFace.face_box.length === 4
          ) {
            // 가장 뜨거운 얼굴의 중심점 계산
            const [x1, y1, x2, y2] = hottestFace.face_box;
            const centerX = (x1 + x2) / 2;
            const normalizedX = centerX / 640; // 640은 이미지 너비

            // 화면을 3등분하여 방향 결정
            let direction;
            if (normalizedX < 0.33) {
              direction = "left";
            } else if (normalizedX > 0.66) {
              direction = "right";
            } else {
              direction = "center";
            }

            console.log(`=== TEMPERATURE-BASED FACE TRACKING ===`);
            console.log(`Hottest face temp: ${hottestFace.temperature}°C`);
            console.log(
              `Face center: ${centerX.toFixed(
                2
              )}, normalized: ${normalizedX.toFixed(3)}`
            );
            console.log(`Direction: ${direction}`);
            console.log(`======================================`);

            // 온도 기반 팬 회전 명령 발행
            const command = {
              device_id: CONFIG.DEVICE_IDS.SERVER,
              action: "set_direction",
              direction: direction,
              face_id: hottestFace.face_id || "unknown",
              temperature: hottestFace.temperature,
              reason: "temperature_based_tracking",
              position: {
                x: centerX,
                normalized_x: normalizedX,
              },
              timestamp: Date.now(),
            };

            try {
              const result = client.publish(
                CONFIG.TOPICS.CONTROL.DIRECTION,
                JSON.stringify(command),
                { qos: 1 }
              );
              console.log(
                `✅ Published temperature-based rotation command: ${direction} for hottest face (${hottestFace.temperature}°C)`
              );
            } catch (error) {
              console.error(
                `Error publishing temperature-based command:`,
                error
              );
            }
          } else {
            // 얼굴 위치 정보가 없는 경우 기본 팬 회전
            sendCommand(client, CONFIG.DEVICE_IDS.FAN_MODULE, "rotate_fan");
          }

          // 상태 업데이트
          systemState = "rotating";
          highTempCount = 0;

          // 시스템 상태 발행
          publishSystemState(
            client,
            `High temperature detected (${highestTemperature}°C), activating fan`
          );
        }
      } else if (systemState === "measuring") {
        // 정상 온도면 카운터 리셋
        if (highTempCount > 0) {
          console.log(
            "Temperature returned to normal range, resetting counter"
          );
          highTempCount = 0;
        }
      }

      // 인식된 얼굴이 있는 경우 웹/모바일 클라이언트에 알림
      if (data.faces.length > 0) {
        const recognizedCount = data.faces.filter(
          (face) => face.face_id !== "Unknown"
        ).length;

        // 모바일 클라이언트에 발행
        const updateMessage = {
          device_id: CONFIG.DEVICE_IDS.SERVER,
          timestamp: Date.now(),
          total_faces: data.faces.length,
          recognized_faces: recognizedCount,
          faces: data.faces.map((face) => ({
            face_id: face.face_id || "Unknown",
            position: face.face_box || [],
            temperature: face.temperature,
            confidence: face.temp_confidence || "unknown",
          })),
        };

        // 모바일 앱용 토픽으로 발행
        try {
          client.publish("mobile/face/update", JSON.stringify(updateMessage), {
            qos: 0,
          });
        } catch (e) {
          console.error("Error publishing to mobile/face/update:", e);
        }

        // 시스템 로그에 기록
        if (recognizedCount > 0) {
          publishSystemLog(
            `${recognizedCount} known faces detected with temperature data`
          );
        }
      }
    } catch (e) {
      console.error("Error processing face thermal data:", e);
      publishSystemLog(
        `Error processing face thermal data: ${e.message}`,
        "error"
      );
    }
  }

  /**
   * 얼굴 인식 모듈 상태 처리 함수
   */
  function handleFaceModuleStatus(client, data) {
    console.log(`Face module status update: ${JSON.stringify(data)}`);

    // Remove specific device ID check - allow all face recognition modules
    if (data.device_id) {
      // 연결된 디바이스 목록 업데이트
      connectedDevices.set(data.device_id, {
        lastSeen: Date.now(),
        status: data.status || "unknown",
        type: "face_recognition",
        connectionType: data.connection_type || "unknown",
        capabilities: data.capabilities || ["face_recognition", "thermal"],
      });

      // 상태 변경 로그
      if (
        data.status === "connected" &&
        !faceRecognitionState.connectedModules.has(data.device_id)
      ) {
        faceRecognitionState.connectedModules.add(data.device_id);
        publishSystemLog(
          `Face recognition module connected: ${data.device_id}`
        );

        // 모듈 설정 메시지 전송 (자동 등록 설정 등)
        sendFaceModuleConfig(client);
      } else if (
        data.status === "disconnected" &&
        faceRecognitionState.connectedModules.has(data.device_id)
      ) {
        faceRecognitionState.connectedModules.delete(data.device_id);
        publishSystemLog(
          `Face recognition module disconnected: ${data.device_id}`,
          "warning"
        );
      }

      // 자동 등록 모드 확인 및 업데이트
      if (data.auto_register !== undefined) {
        faceRecognitionState.autoRegister = data.auto_register;
      }
    }
  }

  /**
   * 얼굴 등록 처리 함수
   */
  function handleFaceRegistration(client, data) {
    console.log(`Face registration update: ${JSON.stringify(data)}`);

    // Remove device authorization check - allow all devices to register faces
    if (!data.device_id) {
      console.warn("Face registration data missing device_id");
      return;
    }

    console.log(`Face registration accepted from device: ${data.device_id}`);

    // 등록 결과 처리
    if (data.success) {
      faceRecognitionState.lastRegisteredFace = {
        face_id: data.face_id,
        timestamp: Date.now(),
      };

      // 등록 성공 로그
      publishSystemLog(`New face registered: ${data.face_id}`);

      // 모바일 클라이언트에 알림
      const registrationMessage = {
        device_id: CONFIG.DEVICE_IDS.SERVER,
        event: "face_registered",
        face_id: data.face_id,
        timestamp: Date.now(),
      };

      client.publish(
        "mobile/face/registration",
        JSON.stringify(registrationMessage),
        { qos: 1 }
      );
    } else {
      // 등록 실패 로그
      publishSystemLog(`Face registration failed: ${data.message}`, "warning");
    }
  }

  /**
   * 얼굴 인식 모듈에 설정 전송
   */
  function sendFaceModuleConfig(client) {
    if (!isClientReadyToPublish()) {
      return;
    }

    const configMessage = {
      device_id: CONFIG.DEVICE_IDS.SERVER,
      action: "update_config",
      config: {
        auto_register: faceRecognitionState.autoRegister,
        save_interval: 10, // 10초마다 저장
        debug_mode: false, // 디버그 모드 비활성화
      },
      timestamp: Date.now(),
    };

    client.publish(CONFIG.TOPICS.FACE.COMMAND, JSON.stringify(configMessage), {
      qos: 1,
    });
    console.log("Sent configuration to face recognition module");
  }

  /**
   * 얼굴 인식 모듈에 명령 전송
   */
  function sendFaceModuleCommand(client, action, params = {}) {
    if (!isClientReadyToPublish()) {
      return;
    }

    const command = {
      device_id: CONFIG.DEVICE_IDS.SERVER,
      action: action,
      params: params,
      timestamp: Date.now(),
    };

    client.publish(CONFIG.TOPICS.FACE.COMMAND, JSON.stringify(command), {
      qos: 1,
    });
    console.log(`Command sent to face recognition module: ${action}`);
  }

  // Handle shutdown
  process.on("SIGINT", function () {
    console.log("Shutting down server...");

    // 종료 플래그 설정 (추가)
    shutdownInProgress = true;

    // 이벤트 리스너 제거 (추가)
    mqttClient.removeAllListeners();

    // 상태 메시지 발행 (오류 처리 추가)
    try {
      if (mqttClient && mqttClient.connected && !mqttClient.disconnecting) {
        mqttClient.publish(
          CONFIG.TOPICS.STATUS,
          JSON.stringify({
            device_id: CONFIG.DEVICE_IDS.SERVER,
            status: "offline",
            timestamp: Date.now(),
          }),
          { qos: 1 }
        );
      }
    } catch (e) {
      console.error("Error during shutdown message publishing:", e.message);
    }

    // 즉시 타이머 해제 (추가)
    if (statusUpdateTimer) {
      clearInterval(statusUpdateTimer);
      statusUpdateTimer = null;
    }

    // MQTT 클라이언트 종료 및 연결 해제 핸들러 제거
    setTimeout(() => {
      try {
        // 종료 중 다른 콜백이 실행되지 않도록 콜백 함수 제거
        mqttClient.onDisconnect = null;
        mqttClient.onClose = null;
        mqttClient.onError = null;

        // 정상 종료
        mqttClient.end(true, () => {
          console.log("MQTT client disconnected");
          process.exit(0);
        });
      } catch (e) {
        console.error("Error during client shutdown:", e.message);
        process.exit(1);
      }
    }, 500);
  });

  // MQTT 클라이언트 연결 이벤트 핸들러 개선
  mqttClient.on("error", function (error) {
    console.error("MQTT client error:", error);
    // 서버 종료 중이 아닐 때만 로그 발행
    if (!shutdownInProgress) {
      console.log(`MQTT error: ${error.message}`);
    }
  });
}, 1000); // Wait 1 second for broker to initialize before connecting controller

/**
 * 얼굴 온도 데이터를 팬 모듈로 전달
 */
function forwardFaceThermalDataToFan(client, faceData) {
  if (!isClientReadyToPublish()) {
    return;
  }

  try {
    // 팬 모듈로 전달할 데이터 구성
    const fanMessage = {
      device_id: CONFIG.DEVICE_IDS.SERVER,
      source: "face_recognition",
      timestamp: Date.now(),
      faces: faceData.faces.map((face) => ({
        face_id: face.face_id || "Unknown",
        temperature: face.temperature,
        temp_confidence: face.temp_confidence,
        position: face.face_box,
      })),
    };

    // 팬 모듈이 구독하는 토픽으로 발행
    client.publish(CONFIG.TOPICS.FACE.THERMAL, JSON.stringify(fanMessage), {
      qos: 1,
    });

    console.log(
      `Forwarded face thermal data to fan module: ${faceData.faces.length} faces`
    );
  } catch (e) {
    console.error("Error forwarding face thermal data to fan:", e);
  }
}

/**
 * 팬 모듈 상태 처리
 */
function handleFanModuleStatus(data) {
  console.log(`Fan module status update: ${JSON.stringify(data)}`);

  // 상태 업데이트 처리
  if (data.device_id === CONFIG.DEVICE_IDS.FAN_MODULE) {
    // 연결된 디바이스 목록 업데이트
    connectedDevices.set(data.device_id, {
      lastSeen: Date.now(),
      status: data.status || "unknown",
      type: "fan_control",
      connectionType: data.connection_type || "unknown",
      fanData: data.data || {},
    });

    // 상태 변경 로그
    if (data.status === "connected") {
      publishSystemLog(`Fan control module connected`);
    } else if (data.status === "disconnected") {
      publishSystemLog(`Fan control module disconnected`, "warning");
      // 팬 모듈 연결 해제 시 시스템을 에러 상태로
      systemState = "error";
      publishSystemState(mqttClient, "Fan module disconnected");
    }

    // 팬 회전 상태에 따른 시스템 상태 업데이트
    if (data.data && data.data.is_rotating !== undefined) {
      if (data.data.is_rotating && systemState !== "rotating") {
        systemState = "rotating";
        publishSystemState(
          mqttClient,
          `Fan is rotating (${data.data.rotation_direction})`
        );
      } else if (!data.data.is_rotating && systemState === "rotating") {
        systemState = "measuring";
        publishSystemState(
          mqttClient,
          "Fan rotation stopped, resuming temperature monitoring"
        );
      }
    }
  }
}

/**
 * 모바일 앱용 팬 제어 API 설정
 */
function setupFanControlAPIs(app) {
  // 팬 회전 제어 API
  app.post("/api/fan/rotate", (req, res) => {
    const { direction, speed } = req.body;

    if (!direction || !["left", "right", "stop"].includes(direction)) {
      return res.status(400).json({
        success: false,
        message: "Invalid direction. Use 'left', 'right', or 'stop'",
      });
    }

    if (mqttClient && mqttClient.connected) {
      // 모바일 클라이언트인 것처럼 명령 전송
      const command = {
        device_id: "mobile_client_api",
        direction: direction,
        timestamp: Date.now(),
      };

      mqttClient.publish(
        CONFIG.TOPICS.CONTROL.DIRECTION,
        JSON.stringify(command),
        {
          qos: 1,
        }
      );

      res.json({
        success: true,
        message: `Fan rotation command sent: ${direction}`,
        direction: direction,
      });
    } else {
      res.status(503).json({
        success: false,
        message: "MQTT client not connected",
      });
    }
  });
}

/**
 * 서버를 안전하게 종료하는 함수
 */
function gracefulShutdown() {
  console.log("\n┌─────────────────────────────────────┐");
  console.log("│      Shutting down MQTT server      │");
  console.log("└─────────────────────────────────────┘");

  // 종료 플래그 설정 (안전하게)
  try {
    global.shutdownInProgress = true;
    shutdownInProgress = true;
  } catch (e) {
    console.error("- Could not set shutdown flag:", e.message);
  }

  // 모든 타이머 안전하게 제거
  try {
    // statusUpdateTimer가 전역에 정의되어 있는지 확인
    if (typeof statusUpdateTimer !== "undefined" && statusUpdateTimer) {
      clearInterval(statusUpdateTimer);
      global.statusUpdateTimer = null;
      console.log("- Status update timer cleared");
    } else {
      console.log("- No status update timer to clear");
    }

    // 다른 타이머들이 있다면 여기서 제거
  } catch (e) {
    console.error("- Error clearing timers:", e.message);
  }

  // 마지막 상태 메시지를 발행할 수 있는지 확인
  try {
    if (
      mqttClient &&
      typeof mqttClient.connected !== "undefined" &&
      mqttClient.connected &&
      typeof mqttClient.disconnecting !== "undefined" &&
      !mqttClient.disconnecting
    ) {
      console.log("- Publishing final offline status");

      // 오프라인 상태로 설정 (retained 메시지)
      const finalStatus = {
        device_id:
          typeof CONFIG !== "undefined" && CONFIG.DEVICE_IDS
            ? CONFIG.DEVICE_IDS.SERVER
            : "server_controller",
        status: "offline",
        timestamp: Date.now(),
      };

      const statusTopic =
        typeof CONFIG !== "undefined" && CONFIG.TOPICS
          ? CONFIG.TOPICS.STATUS
          : "device/status";

      mqttClient.publish(statusTopic, JSON.stringify(finalStatus), {
        qos: 1,
        retain: true,
      });
    }
  } catch (e) {
    console.error("- Error publishing shutdown message:", e.message);
  }

  // 모든 이벤트 리스너 제거
  console.log("- Removing all event listeners");
  try {
    if (mqttClient && typeof mqttClient.removeAllListeners === "function") {
      mqttClient.removeAllListeners();
    }
  } catch (e) {
    console.error("- Error removing event listeners:", e.message);
  }

  // 마지막으로 연결 해제 및 종료
  console.log("- Closing MQTT connection");
  setTimeout(() => {
    try {
      if (!mqttClient) {
        console.log("- MQTT client not available, exiting");
        process.exit(0);
        return;
      }

      // 콜백 함수 제거
      mqttClient.onDisconnect = null;
      mqttClient.onClose = null;
      mqttClient.onError = null;

      // 강제 종료
      mqttClient.end(true, () => {
        console.log("- MQTT client successfully disconnected");

        // 프로세스 종료 타이머 설정 (0.5초 후)
        setTimeout(() => {
          console.log("- Exiting process");
          process.exit(0);
        }, 500);
      });

      // 타임아웃 설정 (3초 후 강제 종료)
      setTimeout(() => {
        console.error("- Forced exit due to timeout");
        process.exit(1);
      }, 3000);
    } catch (e) {
      console.error("- Fatal error during shutdown:", e.message);
      process.exit(1);
    }
  }, 1000);
}

// 종료 시그널 핸들러 등록
// 종료 시그널 핸들러 등록
process.on("SIGINT", gracefulShutdown); // Ctrl+C
process.on("SIGTERM", gracefulShutdown); // kill 명령어
process.on("SIGHUP", gracefulShutdown); // 터미널 종료

// 예상치 못한 오류 처리
process.on("uncaughtException", (error) => {
  console.error("Uncaught Exception:", error);
  gracefulShutdown();
});

// Log server startup with more information
console.log(`
┌─────────────────────────────────────┐
│      IoT Fan Control System         │
├─────────────────────────────────────┤
│ MQTT broker: ${CONFIG.BROKER.HOST}:${CONFIG.BROKER.PORT}           │
│ MQTT websockets: ${CONFIG.BROKER.WS_PORT}               │
│ Temperature threshold: ${CONFIG.TEMPERATURE.THRESHOLD}°C       │
│ High temp threshold: ${CONFIG.TEMPERATURE.HIGH_THRESHOLD_COUNT} readings     │
│ Heartbeat interval: ${CONFIG.HEARTBEAT.INTERVAL / 1000}s             │
│ Heartbeat timeout: ${CONFIG.HEARTBEAT.TIMEOUT / 1000}s              │
│ Started: ${new Date().toISOString()}   │
└─────────────────────────────────────┘
`);

const express = require("express");
const cors = require("cors"); // CORS 지원을 위해

// Express 앱 생성
const app = express();
const API_PORT = 3000; // You can change this to any available port

app.use(express.json());
app.use(cors()); // 모든 출처에서의 요청 허용

// Start the Express server
app.listen(API_PORT, () => {
  console.log(`API server running on port ${API_PORT}`);
});

// LIDAR 데이터 API를 수정
app.get("/api/lidar/data", (req, res) => {
  res.json({
    last_update: lidarData.lastUpdate,
    scan_count: Object.keys(lidarData.scans).length,
    nearest_objects: lidarData.nearestObjects,
    updated_at: Date.now(),
    debug: {
      scan_buffer_size: lidarData.scanBuffer.length,
      mqtt_connected: mqttClient ? mqttClient.connected : false,
      system_state: systemState,
      topics: {
        lidar_data: CONFIG.TOPICS.LIDAR.DATA,
        lidar_scan: CONFIG.TOPICS.LIDAR.SCAN,
      },
    },
  });
});

// 전체 스캔 데이터 요청 API
app.get("/api/lidar/scan", (req, res) => {
  // 선택적으로 범위를 제한 (기본: 모든 데이터)
  const start = parseInt(req.query.start) || 0;
  const end = parseInt(req.query.end) || 360;

  // 요청된 각도 범위의 데이터만 필터링
  const filteredData = Object.entries(lidarData.scans)
    .filter(([angle, _]) => {
      const angleValue = parseFloat(angle);
      return angleValue >= start && angleValue <= end;
    })
    .reduce((obj, [angle, distance]) => {
      obj[angle] = distance;
      return obj;
    }, {});

  res.json({
    scan_data: filteredData,
    timestamp: Date.now(),
  });
});

// 얼굴 인식 데이터 API
app.get("/api/face/data", (req, res) => {
  res.json({
    last_update: faceRecognitionState.lastUpdate,
    recognized_faces: faceRecognitionState.recognizedFaces,
    updated_at: Date.now(),
  });
});

// 그리드 매핑 데이터 API
app.get("/api/face/grid", (req, res) => {
  res.json({
    grid_data: faceRecognitionState.gridMap,
    updated_at: Date.now(),
  });
});

// 얼굴 인식 모듈 설정 변경 API
app.post("/api/face/config", (req, res) => {
  const { auto_register } = req.body;

  if (auto_register !== undefined) {
    faceRecognitionState.autoRegister = Boolean(auto_register);

    // 설정 변경사항 모듈에 전송
    if (mqttClient && mqttClient.connected) {
      sendFaceModuleConfig(mqttClient);
    }
  }

  res.json({
    success: true,
    config: {
      auto_register: faceRecognitionState.autoRegister,
    },
  });
});

// 수동 얼굴 등록 API
app.post("/api/face/register", (req, res) => {
  const { face_id } = req.body;

  if (!face_id) {
    return res.status(400).json({
      success: false,
      message: "Face ID is required",
    });
  }

  // 얼굴 등록 명령 전송
  if (mqttClient && mqttClient.connected) {
    sendFaceModuleCommand(mqttClient, "register_face", { face_id });

    res.json({
      success: true,
      message: "Face registration command sent",
      face_id,
    });
  } else {
    res.status(503).json({
      success: false,
      message: "MQTT client not connected",
    });
  }
});

// 팬 모드 설정 API
app.post("/api/fan/mode", (req, res) => {
  const { mode } = req.body;

  if (!mode || !["enable_autonomous", "manual"].includes(mode)) {
    return res.status(400).json({
      success: false,
      message: "Invalid mode. Use 'enable_autonomous' or 'manual'",
    });
  }

  if (mqttClient && mqttClient.connected) {
    const command = {
      device_id: "mobile_client_api",
      mode: mode,
      timestamp: Date.now(),
    };

    mqttClient.publish(
      CONFIG.TOPICS.CONTROL.AUTO_MODE,
      JSON.stringify(command),
      {
        qos: 1,
      }
    );

    res.json({
      success: true,
      message: `Fan mode set to: ${mode}`,
      mode: mode,
    });
  } else {
    res.status(503).json({
      success: false,
      message: "MQTT client not connected",
    });
  }
});

// 팬 상태 조회 API
app.get("/api/fan/status", (req, res) => {
  const fanDevice = connectedDevices.get(CONFIG.DEVICE_IDS.FAN_MODULE);

  if (fanDevice) {
    res.json({
      success: true,
      status: fanDevice.status,
      data: fanDevice.fanData || {},
      last_seen: fanDevice.lastSeen,
      connection_type: fanDevice.connectionType,
    });
  } else {
    res.json({
      success: false,
      message: "Fan module not connected",
      status: "disconnected",
    });
  }
});

function handleFaceGridData(data) {
  console.log(`Face grid data received: ${JSON.stringify(data)}`);

  // 데이터 유효성 검사
  if (!data.device_id || !data.grid_data || !Array.isArray(data.grid_data)) {
    console.warn("Received invalid face grid data");
    return;
  }

  // Remove device authorization check - allow all devices
  console.log(`Face grid data accepted from device: ${data.device_id}`);

  // 그리드 데이터 업데이트
  faceRecognitionState.gridMap = data.grid_data;
  faceRecognitionState.lastUpdate = Date.now();

  // 디버그 로깅
  console.log(
    `Updated face grid map: ${data.grid_data.length}x${
      data.grid_data[0]?.length || 0
    } grid`
  );

  // 모바일 클라이언트에 업데이트 발행
  if (mqttClient && mqttClient.connected) {
    const updateMessage = {
      device_id: CONFIG.DEVICE_IDS.SERVER,
      timestamp: Date.now(),
      grid_data: data.grid_data,
      grid_size: data.grid_size || [3, 3],
    };

    mqttClient.publish("mobile/face/grid", JSON.stringify(updateMessage), {
      qos: 0,
    });
  }
}

// 새로운 핸들러 함수들 추가

// 초음파 센서 데이터 처리
function handleUltrasonicData(client, data) {
  console.log(`Ultrasonic sensor data received: ${JSON.stringify(data)}`);

  if (!data.device_id || typeof data.distance_cm !== "number") {
    console.warn("Invalid ultrasonic sensor data");
    return;
  }

  // 장애물 감지 시 알림
  if (data.obstacle_detected) {
    console.log(
      `⚠️ 장애물 감지: ${data.sensor_position} 방향 ${data.distance_cm}cm`
    );

    // 장애물 감지 시 움직임 중지 명령 발행
    const stopCommand = {
      device_id: CONFIG.DEVICE_IDS.SERVER,
      action: "stop",
      reason: "obstacle_detected",
      obstacle_info: {
        position: data.sensor_position,
        distance: data.distance_cm,
      },
      timestamp: Date.now(),
    };

    client.publish("control/movement", JSON.stringify(stopCommand), { qos: 1 });
    console.log(`🛑 장애물로 인한 정지 명령 발행`);
  }

  // 모바일 앱에 초음파 데이터 전송
  const updateMessage = {
    device_id: CONFIG.DEVICE_IDS.SERVER,
    timestamp: Date.now(),
    ultrasonic_data: data,
  };

  client.publish("mobile/ultrasonic/update", JSON.stringify(updateMessage), {
    qos: 0,
  });
}

// 온도 센서 데이터 처리
function handleTemperatureData(client, data) {
  console.log(`Temperature sensor data received: ${JSON.stringify(data)}`);

  if (!data.device_id || typeof data.temperature !== "number") {
    console.warn("Invalid temperature sensor data");
    return;
  }

  // 고온 감지 시 선풍기 제어
  if (data.temperature > CONFIG.TEMPERATURE.THRESHOLD) {
    console.log(
      `🔥 고온 감지: ${data.temperature}°C (센서: ${data.sensor_id})`
    );

    // 선풍기 회전 명령 발행
    const fanCommand = {
      device_id: CONFIG.DEVICE_IDS.SERVER,
      action: "start_rotation",
      speed: 70, // 고온 시 높은 속도
      reason: "high_temperature",
      temperature: data.temperature,
      sensor_position: data.position,
      timestamp: Date.now(),
    };

    client.publish("control/fan/rotation", JSON.stringify(fanCommand), {
      qos: 1,
    });
    console.log(`💨 고온으로 인한 선풍기 가동 명령 발행`);
  }

  // 온도 데이터를 시스템 상태에 반영
  lastTemperature = data.temperature;
  lastTempReceivedTime = Date.now();

  // 모바일 앱에 온도 데이터 전송
  const updateMessage = {
    device_id: CONFIG.DEVICE_IDS.SERVER,
    timestamp: Date.now(),
    temperature_data: data,
  };

  client.publish("mobile/temperature/update", JSON.stringify(updateMessage), {
    qos: 0,
  });
}

// 움직임 상태 처리
function handleMovementStatus(data) {
  console.log(`Movement status update: ${JSON.stringify(data)}`);

  if (!data.device_id) {
    console.warn("Movement status missing device_id");
    return;
  }

  // 디바이스 상태 업데이트
  if (connectedDevices.has(data.device_id)) {
    const deviceInfo = connectedDevices.get(data.device_id);
    deviceInfo.lastSeen = Date.now();
    deviceInfo.movementData = {
      is_moving: data.is_moving,
      direction: data.direction,
      speed: data.speed,
    };
    connectedDevices.set(data.device_id, deviceInfo);
  }

  publishSystemLog(
    `Movement status: ${data.is_moving ? "moving" : "stopped"} (${
      data.direction
    })`
  );
}

// 선풍기 상태 처리
function handleFanStatus(data) {
  console.log(`Fan status update: ${JSON.stringify(data)}`);

  if (!data.device_id) {
    console.warn("Fan status missing device_id");
    return;
  }

  // 디바이스 상태 업데이트
  if (connectedDevices.has(data.device_id)) {
    const deviceInfo = connectedDevices.get(data.device_id);
    deviceInfo.lastSeen = Date.now();
    deviceInfo.fanData = {
      is_rotating: data.is_rotating,
      direction: data.direction,
      speed: data.speed,
    };
    connectedDevices.set(data.device_id, deviceInfo);
  }

  publishSystemLog(
    `Fan status: ${data.is_rotating ? "rotating" : "stopped"} (${
      data.direction
    })`
  );
}

// 움직임 제어 처리
function handleMovementControl(client, data) {
  console.log(`Movement control command: ${JSON.stringify(data)}`);

  if (!data.device_id || !data.action) {
    console.warn("Invalid movement control command");
    return;
  }

  // LIDAR 데이터와 초음파 센서 데이터를 분석하여 안전한 경로 결정
  // 여기서는 시뮬레이션을 위해 기본적인 처리만 수행

  console.log(
    `Processing movement command: ${data.action} from ${data.device_id}`
  );
  publishSystemLog(`Movement command received: ${data.action}`);
}

// 선풍기 제어 처리
function handleFanControl(client, data) {
  console.log(`Fan control command: ${JSON.stringify(data)}`);

  if (!data.device_id || !data.action) {
    console.warn("Invalid fan control command");
    return;
  }

  console.log(`Processing fan command: ${data.action} from ${data.device_id}`);
  publishSystemLog(`Fan command received: ${data.action}`);
}

// 새로운 핸들러 함수들 추가
function handleMobileLidarUpdate(client, data) {
  console.log(
    `Mobile LiDAR update received from ${data.device_id}: ${
      data.scan_data?.length || 0
    } points`
  );

  // 모바일 앱을 위한 LiDAR 데이터 처리
  if (data.scan_data && Array.isArray(data.scan_data)) {
    // LiDAR 데이터 저장
    lidarData.lastUpdate = Date.now();
    lidarData.nearestObjects = data.nearest_objects || [];

    // 스캔 데이터를 거리별로 정렬
    const sortedData = data.scan_data.sort((a, b) => a.distance - b.distance);

    // 가까운 물체 분석 (2m 이내)
    const closeObjects = sortedData.filter((point) => point.distance <= 2.0);

    if (closeObjects.length > 0) {
      // 장애물 회피 명령 생성
      const closestObject = closeObjects[0];

      let moveCommand = {
        device_id: CONFIG.DEVICE_IDS.SERVER,
        timestamp: Date.now(),
        action: "move",
        direction: "stop",
        speed: 0,
        priority: "high",
        reason: `Obstacle detected at ${closestObject.distance}m @ ${closestObject.angle}°`,
      };

      // 각도에 따른 회피 방향 결정
      if (closestObject.angle >= 315 || closestObject.angle <= 45) {
        // 전방 장애물 - 후진
        moveCommand.direction = "backward";
        moveCommand.speed = 30;
      } else if (closestObject.angle > 45 && closestObject.angle <= 135) {
        // 좌측 장애물 - 우회전
        moveCommand.action = "rotate";
        moveCommand.direction = "right";
        moveCommand.speed = 40;
      } else if (closestObject.angle > 225 && closestObject.angle < 315) {
        // 우측 장애물 - 좌회전
        moveCommand.action = "rotate";
        moveCommand.direction = "left";
        moveCommand.speed = 40;
      }

      // 움직임 명령 발행
      if (isClientReadyToPublish()) {
        client.publish("control/movement", JSON.stringify(moveCommand));
        publishSystemLog(
          `Obstacle avoidance: ${moveCommand.action} ${moveCommand.direction}`,
          "warning"
        );
      }
    }
  }
}

function handleDeviceStatus(client, data) {
  console.log(`Device status received from ${data.device_id}: ${data.status}`);

  // 장치 상태 정보 업데이트
  if (data.device_id) {
    connectedDevices.set(data.device_id, {
      lastSeen: Date.now(),
      status: data.status || "unknown",
      type: data.client_type || "unknown",
      connectionType: data.connection_type || "mqtt",
      capabilities: data.capabilities || [],
      ip: data.ip,
    });

    // 새 장치 연결 알림
    if (data.status === "connected") {
      publishSystemLog(
        `Device connected: ${data.device_id} (${data.client_type})`
      );

      // 연결된 장치 목록 발행
      publishConnectedDevices(client);
    } else if (data.status === "disconnected") {
      publishSystemLog(`Device disconnected: ${data.device_id}`);
      connectedDevices.delete(data.device_id);
      publishConnectedDevices(client);
    }
  }
}

function handleAutoModeControl(client, data) {
  console.log(`Auto mode control received: ${JSON.stringify(data)}`);

  const autoMode = data.auto_mode || false;
  const deviceId = data.device_id || "unknown";

  // 자동 모드 상태 업데이트
  systemState = autoMode ? "auto" : "manual";

  // LiDAR 상태 업데이트
  const lidarStatus = {
    device_id: CONFIG.DEVICE_IDS.SERVER,
    timestamp: Date.now(),
    status: autoMode ? "active" : "standby",
    auto_mode: autoMode,
    safe_distance: 2.0,
  };

  if (isClientReadyToPublish()) {
    client.publish("sensor/lidar/status", JSON.stringify(lidarStatus));
    publishSystemLog(
      `Auto mode ${autoMode ? "enabled" : "disabled"} by ${deviceId}`
    );
  }

  // 시스템 상태 발행
  publishSystemState(client, `Auto mode ${autoMode ? "enabled" : "disabled"}`);
}

function publishConnectedDevices(client) {
  const deviceList = Array.from(connectedDevices.entries()).map(
    ([id, info]) => ({
      device_id: id,
      status: info.status,
      client_type: info.type,
      connection_type: info.connectionType,
      capabilities: info.capabilities,
      last_seen: info.lastSeen,
    })
  );

  const deviceMessage = {
    device_id: CONFIG.DEVICE_IDS.SERVER,
    timestamp: Date.now(),
    connected_devices: deviceList,
    websocket_clients: Array.from(websocketClients).map((id) => ({
      client_id: id,
    })),
  };

  if (isClientReadyToPublish()) {
    client.publish("system/devices", JSON.stringify(deviceMessage));
  }
}
