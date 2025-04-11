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
    TEMP: "sensor/temperature",
    COMMAND: "device/command",
    STATUS: "device/status",
    CONTROL: {
      PREFIX: "control",
      ROTATION: "control/rotation",
      DIRECTION: "control/direction",
      AUTO_MODE: "control/auto_mode",
      STATUS: "control/status",
    },
    DETECTION: "sensor/detection",
    SYSTEM: "system/status",
    LOG: "system/log",
  },
  DEVICE_IDS: {
    FAN_MODULE: "fan_control_module",
    SENSOR_MODULE: "sensor_module",
    OBJECT_DETECTION: "object_detection_module",
    SERVER: "server_controller",
    MOBILE_PREFIX: "mobile_client_",
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
      if (topic.startsWith(CONFIG.TOPICS.TEMP)) {
        handleTemperatureData(mqttClient, data);
      } else if (topic === CONFIG.TOPICS.STATUS) {
        handleStatusUpdate(data);
      } else if (topic.startsWith(CONFIG.TOPICS.DETECTION)) {
        handleDetectionData(mqttClient, data);
      } else if (topic.startsWith(CONFIG.TOPICS.CONTROL.PREFIX)) {
        handleControlCommand(mqttClient, topic, data);
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

  // Handle temperature data with improved logging
  function handleTemperatureData(client, data) {
    if (!data.temperature) {
      console.warn("Received temperature data without temperature value");
      return;
    }

    const temperature = Number(data.temperature);
    if (isNaN(temperature)) {
      console.warn(`Invalid temperature value: ${data.temperature}`);
      return;
    }

    lastTemperature = temperature;
    lastTempReceivedTime = Date.now();

    console.log(`Current temperature: ${temperature}°C`);

    // Only analyze when in measuring state
    if (systemState === "measuring") {
      // High temperature detection
      if (temperature >= CONFIG.TEMPERATURE.THRESHOLD) {
        highTempCount++;
        console.log(
          `High temperature detected (${highTempCount}/${CONFIG.TEMPERATURE.HIGH_THRESHOLD_COUNT})`
        );

        // Check if consecutive threshold reached
        if (highTempCount >= CONFIG.TEMPERATURE.HIGH_THRESHOLD_COUNT) {
          console.log(
            "Persistent high temperature detected! Sending fan rotation command"
          );

          // Send fan rotation command
          sendCommand(client, CONFIG.DEVICE_IDS.FAN_MODULE, "rotate_fan");

          // Update state
          systemState = "rotating";
          highTempCount = 0;

          // Publish system state for all clients
          publishSystemState(
            client,
            "High temperature detected, activating fan"
          );
        }
      } else {
        // Reset counter if temperature returns to normal
        if (highTempCount > 0) {
          console.log(
            "Temperature returned to normal range, resetting counter"
          );
          highTempCount = 0;
        }
      }
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
    console.log(`Detection data received: ${JSON.stringify(data)}`);

    if (data.detected === undefined) {
      console.warn("Received detection data without 'detected' field");
      return;
    }

    // If a person is detected while fan is rotating
    if (data.detected === true && systemState === "rotating") {
      console.log(
        "Person detected by object detection module, sending fan stop command"
      );

      // Send fan stop command
      sendCommand(client, CONFIG.DEVICE_IDS.FAN_MODULE, "stop_rotation");

      // Switch to intermediate state
      systemState = "detected";

      // Publish system state update
      publishSystemState(client, "Person detected, stopping fan for safety");

      // Resume temperature measurement after delay
      setTimeout(() => {
        console.log("Resuming temperature measurement");
        sendCommand(
          client,
          CONFIG.DEVICE_IDS.FAN_MODULE,
          "resume_temp_measurement"
        );
        systemState = "measuring";
        publishSystemState(client, "Resuming temperature measurement");
      }, 5000);
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

    // Validate command source - must be from mobile client
    if (
      !data.device_id ||
      !data.device_id.startsWith(CONFIG.DEVICE_IDS.MOBILE_PREFIX)
    ) {
      console.warn(
        `Control command from unauthorized device: ${data.device_id}`
      );
      return;
    }

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
