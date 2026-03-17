#include "driver/twai.h"
#include "imu_sensor.h"
#include "pid_controller.h"
#include <Arduino.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <stdint.h>

static bool imuReady = false;

static bool autoMode = false;



static PID pitchPid = {1.0f, 0.05f, 0.15f, 30.0f, 40.0f, 0.0f, 0.0f, true};
static PID yawPid   = {0.4f, 0.0f,  0.05f, 30.0f, 40.0f, 0.0f, 0.0f, true};

static unsigned long lastPidMs = 0;

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

#define CAN_RX 16
#define CAN_TX 17
#define CAN_EN 25

const uint32_t CAN_TX_ID        = 0x201;
const uint32_t THRUST_COMMAND = 0x203;

#define SERVO_ID          0x101
#define THRUST_ID         0x102
#define HEARTBEAT_ID      0x103

#define SERVO_TELEMETRY_ID       0x104
#define SERVO_HEARTBEAT_ID       0x105
#define POWER_TELEMETRY_ID       0x106
#define SHUNT_DIAG_ID     0x107
#define REG_DIAG_ID       0x108

const uint32_t SERVO_COMMAND = 0x202;

bool OwnServer = true;

const char *ap_ssid     = "WifiESP-AP";
const char *ap_password = "esp32pass";

//WIFI Network info here
const char *sta_ssid     = "";
const char *sta_password = "";

void wsBroadcast(const char *prefix, const char *msg) {
  char buf[256];
  snprintf(buf, sizeof(buf), "[%s] %s", prefix, msg);
  webSocket.broadcastTXT(buf);
  Serial.println(buf);
}
void wsBroadcast(const String &prefix, const String &msg) {
  wsBroadcast(prefix.c_str(), msg.c_str());
}

bool startCAN();
bool canSendCommand(char cmd, uint32_t tx_id);
void pollCAN();
void parseTelemetryData(twai_message_t &msg);

struct TelemetryState {
  int16_t  servoAngle  = 0;
  uint16_t ch1Pulse    = 1500;
  uint16_t ch2Pulse    = 1500;
  uint8_t  thrustPct   = 0;
  uint16_t ch3Pulse    = 0;
  uint8_t  status      = 0;
  uint32_t aliveTime   = 0;
  bool     servoValid  = false;
  bool     thrustValid = false;
  bool     heartValid  = false;

  uint16_t servo1          = 90;
  uint16_t servo2          = 90;
  uint16_t servo3          = 90;
  uint16_t servo4          = 90;
  uint8_t  servoStatus     = 0;
  uint32_t servoUptime     = 0;
  bool     powerBoardAlive = false;

  uint16_t busMv      = 0;
  int16_t  currentMa  = 0;
  uint16_t powerMw    = 0;
  bool     inaReady   = false;

  int16_t  rawShuntData  = 0;
};
TelemetryState telemetryData;

static uint8_t canSequence = 0;

void startAP() {
  WiFi.mode(WIFI_AP);
  IPAddress local_IP(192, 168, 4, 1), gateway(192, 168, 4, 1),
      subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ap_ssid, ap_password);
}

void startSTA() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.begin(sta_ssid, sta_password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
    delay(500);
  }
}

bool startCAN() {
  pinMode(CAN_EN, OUTPUT);
  digitalWrite(CAN_EN, HIGH);
  delay(20);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    return false;
  }
  if (twai_start() != ESP_OK) {
    return false;
  }
  return true;
}

bool canSendCommand(char command, uint32_t tx_id = CAN_TX_ID) {
  twai_status_info_t status;
  twai_get_status_info(&status);

  if (status.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();
    unsigned long t = millis();
    while (millis() - t < 2000) {
      twai_get_status_info(&status);
      if (status.state == TWAI_STATE_RUNNING)
        break;
      delay(50);
    }
    if (status.state != TWAI_STATE_RUNNING) {
      return false;
    }
  }

  twai_message_t message;
  message.identifier       = tx_id;
  message.flags            = TWAI_MSG_FLAG_NONE;
  message.data_length_code = 2;
  message.data[0]          = ++canSequence;
  message.data[1]          = (uint8_t)command;

  esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(100));
  if (err == ESP_OK) {
    char buf[48];
    snprintf(buf, sizeof(buf), "0x%03X seq=%d cmd=%c", (unsigned)tx_id,
             canSequence, command);
    wsBroadcast("CAN_TX", buf);
  }
  return (err == ESP_OK);
}

void parseTelemetryDataData(twai_message_t &msg) {
  switch (msg.identifier) {

  case SERVO_ID:
    if (msg.data_length_code >= 6) {
      telemetryData.servoAngle = (int16_t)((msg.data[0] << 8) | msg.data[1]);
      telemetryData.ch1Pulse   = (uint16_t)((msg.data[2] << 8) | msg.data[3]);
      telemetryData.ch2Pulse   = (uint16_t)((msg.data[4] << 8) | msg.data[5]);
      telemetryData.servoValid = true;
    }
    break;

  case THRUST_ID:
    if (msg.data_length_code >= 3) {
      telemetryData.thrustPct  = msg.data[0];
      telemetryData.ch3Pulse   = (uint16_t)((msg.data[1] << 8) | msg.data[2]);
      telemetryData.thrustValid = true;
    }
    break;

  case HEARTBEAT_ID:
    if (msg.data_length_code >= 5) {
      telemetryData.status    = msg.data[0];
      telemetryData.aliveTime =
          ((uint32_t)msg.data[1] << 24) | ((uint32_t)msg.data[2] << 16) |
          ((uint32_t)msg.data[3] << 8)  | (uint32_t)msg.data[4];
      telemetryData.heartValid = true;
    }
    break;

  case SERVO_TELEMETRY_ID:
    if (msg.data_length_code == 8) {
      telemetryData.servo1 = (uint16_t)(((msg.data[0] << 8) | msg.data[1]) / 10);
      telemetryData.servo2 = (uint16_t)(((msg.data[2] << 8) | msg.data[3]) / 10);
      telemetryData.servo3 = (uint16_t)(((msg.data[4] << 8) | msg.data[5]) / 10);
      telemetryData.servo4 = (uint16_t)(((msg.data[6] << 8) | msg.data[7]) / 10);
    }
    break;

  case SERVO_HEARTBEAT_ID:
    if (msg.data_length_code >= 5) {
      telemetryData.servoStatus  = msg.data[0];
      telemetryData.servoUptime  =
          ((uint32_t)msg.data[1] << 24) | ((uint32_t)msg.data[2] << 16) |
          ((uint32_t)msg.data[3] << 8)  | (uint32_t)msg.data[4];
      telemetryData.powerBoardAlive = true;
    }
    break;

  case POWER_TELEMETRY_ID:
    if (msg.data_length_code >= 7) {
      telemetryData.busMv     = (uint16_t)((msg.data[0] << 8) | msg.data[1]);
      telemetryData.currentMa = (int16_t)((msg.data[2] << 8) | msg.data[3]);
      telemetryData.powerMw   = (uint16_t)((msg.data[4] << 8) | msg.data[5]);
      telemetryData.inaReady  = (msg.data[6] & 0x01) != 0;
    }
    break;

  case SHUNT_DIAG_ID:
    if (msg.data_length_code >= 2) {
      telemetryData.rawShuntData = (int16_t)((msg.data[0] << 8) | msg.data[1]);
    }
    break;

  case REG_DIAG_ID:
    if (msg.data_length_code >= 8) {
      uint16_t rShunt = (uint16_t)((msg.data[0] << 8) | msg.data[1]);
      uint16_t rBus   = (uint16_t)((msg.data[2] << 8) | msg.data[3]);
      uint16_t rCfg   = (uint16_t)((msg.data[4] << 8) | msg.data[5]);
      uint16_t rCal   = (uint16_t)((msg.data[6] << 8) | msg.data[7]);
      char buf[96];
      snprintf(buf, sizeof(buf),
               "SHUNT_V=0x%04X  BUS_V=0x%04X  CONFIG=0x%04X  CAL=0x%04X",
               rShunt, rBus, rCfg, rCal);
      wsBroadcast("INA226_REGS", buf);
    }
    break;

  case 0x301:
    if (msg.data_length_code >= 4) {
      uint8_t seq    = msg.data[0];
      uint8_t status = msg.data[2];
      uint8_t err2   = msg.data[3];
      char buf[48];
      snprintf(buf, sizeof(buf), "0x301 seq=%d status=%d err=%d", seq, status,
               err2);
      wsBroadcast("CAN_ACK", buf);
    }
    break;

  case 0x302:
    if (msg.data_length_code >= 4) {
      uint8_t seq    = msg.data[0];
      uint8_t status = msg.data[2];
      uint8_t err2   = msg.data[3];
      char buf[48];
      snprintf(buf, sizeof(buf), "0x302 seq=%d status=%d err=%d", seq, status,
               err2);
      wsBroadcast("CAN_ACK", buf);
    }
    break;

  default:
    break;
  }

  if (msg.identifier == POWER_TELEMETRY_ID || msg.identifier == REG_DIAG_ID ||
      msg.identifier == 0x301 || msg.identifier == 0x302) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Rx 0x%03X DLC: %d", msg.identifier,
             msg.data_length_code);
    wsBroadcast("CAN", buf);
  }
}

void pollCAN() {
  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK) {
    parseTelemetryData(message);
  }
}

void addCORSHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.sendHeader("Access-Control-Allow-Private-Network", "true");
}

static char jsonBuffer[1024];

void buildJSON() {
  float imuPitch = 0.0f, imuRoll = 0.0f, imuYaw = 0.0f;
  getIMUOrientation(imuPitch, imuRoll, imuYaw);
  float imuAx = 0.0f, imuAy = 0.0f, imuAz = 0.0f;
  IMU_GetRawAccel(imuAx, imuAy, imuAz);
  float imuGz = 0.0f, imuGx = 0.0f, imuGy = 0.0f;
  getRawGyro(imuGx, imuGy, imuGz);

  snprintf(
      jsonBuffer, sizeof(jsonBuffer),
      "{"
      "\"servo_angle\":%d,"
      "\"ch1_pulse\":%u,\"ch2_pulse\":%u,"
      "\"thrust_pct\":%u,\"ch3_pulse\":%u,"
      "\"stm_status\":%u,\"stm_uptime_ms\":%lu,"
      "\"servo1\":%u,\"servo2\":%u,\"servo3\":%u,\"servo4\":%u,"
      "\"servo_board_status\":%u,\"servo_board_alive\":%s,"
      "\"bus_v\":%.3f,\"current_a\":%.3f,\"power_w\":%.3f,\"ina_ready\":%s,"
      "\"shunt_raw\":%d,"
      "\"imu_pitch\":%.2f,\"imu_roll\":%.2f,\"imu_yaw\":%.2f,"
      "\"imu_temp\":%.1f,\"imu_ready\":%s,"
      "\"imu_ax\":%.3f,\"imu_ay\":%.3f,\"imu_az\":%.3f,"
      "\"imu_gx\":%.3f,\"imu_gy\":%.3f,\"imu_gz\":%.3f,"
      "\"auto_mode\":%s"
      "}",
      telemetryData.servoAngle, telemetryData.ch1Pulse, telemetryData.ch2Pulse,
      telemetryData.thrustPct, telemetryData.ch3Pulse, telemetryData.status,
      (unsigned long)telemetryData.aliveTime, telemetryData.servo1, telemetryData.servo2,
      telemetryData.servo3, telemetryData.servo4, telemetryData.servoStatus,
      telemetryData.powerBoardAlive ? "true" : "false", telemetryData.busMv / 1000.0f,
      telemetryData.currentMa / 1000.0f, telemetryData.powerMw / 1000.0f,
      telemetryData.inaReady ? "true" : "false", telemetryData.rawShuntData,
      imuPitch, imuRoll, imuYaw,
      IMU_GetTemperature(), imuReady ? "true" : "false", imuAx, imuAy, imuAz,
      imuGx, imuGy, imuGz,
      autoMode ? "true" : "false");
}

String makeInfoHTML() {
  String html =
      F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Telemetry</title>"
        "<style>body{font-family:Arial,sans-serif;margin:12px}"
        "table{border-collapse:collapse}td,th{border:1px solid "
        "#ccc;padding:6px 10px}"
        "</style></head><body>");
  html += F("<h2>STM32 Telemetry</h2><table>");
  html += F("<tr><th>Field</th><th>Value</th></tr>");
  html += "<tr><td>Servo Angle</td><td>" + String(telemetryData.servoAngle) +
          " deg</td></tr>";
  html +=
      "<tr><td>CH1 Pulse</td><td>" + String(telemetryData.ch1Pulse) + "</td></tr>";
  html +=
      "<tr><td>CH2 Pulse</td><td>" + String(telemetryData.ch2Pulse) + "</td></tr>";
  html +=
      "<tr><td>Thrust</td><td>" + String(telemetryData.thrustPct) + "%</td></tr>";
  html +=
      "<tr><td>CH3 Pulse</td><td>" + String(telemetryData.ch3Pulse) + "</td></tr>";
  html += "<tr><td>STM Status</td><td>0x" + String(telemetryData.status, HEX) +
          "</td></tr>";
  html += "<tr><td>STM Uptime</td><td>" + String(telemetryData.aliveTime) +
          " ms</td></tr>";
  html += F("</table><p>JSON: <a "
            "href='/info?format=json'>/info?format=json</a></p></body></html>");
  return html;
}

void handleInfo() {
  addCORSHeaders();
  String fmt = server.hasArg("format") ? server.arg("format") : "";
  if (fmt.equalsIgnoreCase("json")) {
    buildJSON();
    server.send(200, "application/json", jsonBuffer);
  } else {
    server.send(200, "text/html; charset=utf-8", makeInfoHTML());
  }
}

void commandHandler() {
  addCORSHeaders();
  if (!server.hasArg("index")) {
    server.send(400, "text/plain", "Bad Request: index parameter missing");
    return;
  }

  String index = server.arg("index");
  char command = 0;
  bool is_thruster_cmd = false;

  if (index == "A")
    command = 'P';
  else if (index == "B")
    command = 'N';
  else if (index == "C")
    command = 'Q';
  else if (index == "D") {
    command = 'D';
    is_thruster_cmd = true;
  } else if (index == "E") {
    command = 'E';
    is_thruster_cmd = true;
  } else if (index == "F") {
    command = 'F';
    is_thruster_cmd = true;
  } else if (index == "P")
    command = 'P';
  else if (index == "Q")
    command = 'Q';
  else if (index == "R")
    command = 'R';
  else if (index == "S")
    command = 'S';
  else if (index == "N")
    command = 'N';
  else if (index == "G") {
    command = 'G';
    is_thruster_cmd = true;
  } else if (index == "H") {
    command = 'H';
    is_thruster_cmd = true;
  } else if (index == "I") {
    command = 'I';
    is_thruster_cmd = true;
  } else if (index == "J") {
    command = 'J';
    is_thruster_cmd = true;
  }
  else {
    server.send(400, "text/plain", "Unknown command: " + index);
    return;
  }

  bool commandStatus =
      canSendCommand(command, is_thruster_cmd ? THRUST_COMMAND : CAN_TX_ID);
  server.send(200, "text/plain",
              commandStatus ? "Command sent: " + index : "TX failed: " + index);
}

//Sanity Test Function
void handleTest() {
  addCORSHeaders();
  bool commandStatus = canSendCommand('B');
  server.send(200, "text/plain",
              commandStatus ? "CAN TX OK (B - Fins Neutral)" : "CAN TX failed");
}


bool sendServoCommand(uint8_t ch, uint8_t angle) {
  if (ch < 1 || ch > 4)
    return false;

  twai_status_info_t status;
  twai_get_status_info(&status);
  if (status.state != TWAI_STATE_RUNNING)
    return false;

  twai_message_t message;
  message.identifier       = SERVO_COMMAND;
  message.flags            = TWAI_MSG_FLAG_NONE;
  message.data_length_code = 3;
  message.data[0]          = ++canSequence;
  message.data[1]          = (uint8_t)(ch - 1);
  message.data[2]          = (uint8_t)angle;

  esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(100));
  if (err == ESP_OK) {
    char buf[48];
    snprintf(buf, sizeof(buf), "0x202 seq=%d cmd=S", canSequence);
    wsBroadcast("CAN_TX", buf);
  }
  return (err == ESP_OK);
}

void handleSetServoAngle() {
  addCORSHeaders();
  if (!server.hasArg("ch") || !server.hasArg("angle")) {
    server.send(400, "text/plain",
                "Bad Request: ch and angle parameters needed");
    return;
  }

  int ch    = server.arg("ch").toInt();
  int angle = server.arg("angle").toInt();

  bool commandStatus = sendServoCommand(ch, angle);
  server.send(200, "text/plain", commandStatus ? "Servo angle set" : "TX failed");
}

void handleMode() {
  addCORSHeaders();
  if (!server.hasArg("val")) {
    server.send(400, "text/plain", "missing val (auto|manual)");
    return;
  }
  String val = server.arg("val");
  if (val == "auto") {
    resetPID(pitchPid);
    resetPID(yawPid);
    lastPidMs = millis();
    autoMode  = true;
    wsBroadcast("MODE", "AUTO — PID active");
  } else {
    autoMode = false;
    wsBroadcast("MODE", "MANUAL");
  }
  server.send(200, "text/plain", autoMode ? "auto" : "manual");
}

void handlePidReset() {
  addCORSHeaders();
  PID *pid = nullptr;
  if (server.hasArg("axis")) {
    String axis = server.arg("axis");
    if (axis == "pitch")    pid = &pitchPid;
    else if (axis == "yaw") pid = &yawPid;
  }
  if (!pid) {
    server.send(400, "text/plain", "axis=pitch|yaw required");
    return;
  }
  if (server.hasArg("kp")) pid->kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) pid->ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) pid->kd = server.arg("kd").toFloat();
  resetPID(*pid);
  char buf[64];
  snprintf(buf, sizeof(buf), "kp=%.3f ki=%.3f kd=%.3f", pid->kp, pid->ki, pid->kd);
  server.send(200, "text/plain", buf);
}

static bool calibrateIMU = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (OwnServer) {
    startAP();
  } else {
    startSTA();
  }

  startCAN();


  uint8_t devicesFound = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("I2C Device found ");
      devicesFound++;
    }
  }
  if (devicesFound == 0) {
    Serial.println("No I2C device found");
  }

  imuReady = initializeIMU();
  if (imuReady) {
    Serial.println("IMU ready");
  } else {
    Serial.println("IMU not found");
  }

  server.on("/test",      handleTest);
  server.on("/servo",     commandHandler);
  server.on("/servo_set", handleSetServoAngle);
  server.on("/info",      handleInfo);
  server.on("/mode",      handleMode);
  server.on("/pid_tune",  handlePidReset);
  server.on("/servo", HTTP_OPTIONS, []() {
    addCORSHeaders();
    server.send(204);
  });
  server.on("/servo_set", HTTP_OPTIONS, []() {
    addCORSHeaders();
    server.send(204);
  });
  server.on("/info", HTTP_OPTIONS, []() {
    addCORSHeaders();
    server.send(204);
  });
  server.on("/test", HTTP_OPTIONS, []() {
    addCORSHeaders();
    server.send(204);
  });
  server.on("/mode", HTTP_OPTIONS, []() {
    addCORSHeaders();
    server.send(204);
  });
  server.on("/pid_tune", HTTP_OPTIONS, []() {
    addCORSHeaders();
    server.send(204);
  });

  server.begin();
  webSocket.begin();
  webSocket.onEvent(
      [](uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
        if (type != WStype_TEXT || length == 0)
          return;
        char buf[64] = {};
        size_t copyLen = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
        memcpy(buf, payload, copyLen);
        if (strstr(buf, "cal_imu")) {
          calibrateIMU = true;
        }
        if (strstr(buf, "mode:auto")) {
          resetPID(pitchPid);
          resetPID(yawPid);
          lastPidMs = millis();
          autoMode  = true;
        }
        if (strstr(buf, "mode:manual")) {
          autoMode = false;
        }
      });
}

void loop() {
  webSocket.loop();
  server.handleClient();
  pollCAN();

  if (calibrateIMU) {
    calibrateIMU = false;
    ZeroIMU();
    webSocket.broadcastTXT("IMU zeroed");
  }

  if (imuReady) {
    updateIMU();
  }

  if (autoMode && imuReady) {
    unsigned long now = millis();
    float dt = (now - lastPidMs) / 1000.0f;
    if (dt >= 0.05f) {
      lastPidMs = now;

      float pitch, roll, yaw;
      getIMUOrientation(pitch, roll, yaw);

      float pitchCorrection = updatePID(pitchPid, 0.0f, pitch, dt);
      uint8_t s3 = (uint8_t)constrain(90.0f + pitchCorrection, 30.0f, 150.0f);
      uint8_t s4 = (uint8_t)constrain(90.0f + pitchCorrection, 30.0f, 150.0f);

      float gx, gy, gz;
      getRawGyro(gx, gy, gz);
      float yawRaw   = gz * (180.0f / PI);
      float yawMeasurement = (yawRaw > -2.0f && yawRaw < 2.0f)
                              ? 0.0f : yawRaw;
      float yawCorrection = updatePID(yawPid, 0.0f, yawMeasurement, dt);
      uint8_t s1 = (uint8_t)constrain(90.0f - yawCorrection, 30.0f, 150.0f);
      uint8_t s2 = (uint8_t)constrain(90.0f + yawCorrection, 30.0f, 150.0f);

      sendServoCommand(1, s1);
      sendServoCommand(2, s2);
      sendServoCommand(3, s3);
      sendServoCommand(4, s4);
    }
  }
}
