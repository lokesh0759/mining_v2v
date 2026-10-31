/*
  =====================================================================
  ESP32 V2V Communication + Collision Warning System (ESP-NOW)
  VEHICLE 1  =  MASTER  (sensors + web console + drives BOTH vehicles)
  =====================================================================
  Compatible with ESP32 Arduino core 3.x (IDF 5.x). Board: ESP32 Dev Module

  HOW IT WORKS
    * Vehicle 1 and Vehicle 2 both join your phone's 2.4 GHz hotspot.
    * You open ONE web page (served by this board) on your phone.
    * MANUAL mode : the single D-pad drives BOTH vehicles together.
    * AUTO mode   : Vehicle 1 runs obstacle avoidance with its front/rear
                    ultrasonic sensors and Vehicle 2 copies every move.
    * Vehicle 1 sends its final drive command (after safety checks) to
      Vehicle 2 every 100 ms over ESP-NOW, so both always do the same
      thing. Vehicle 2 stops by itself if those commands stop arriving.
    * Vehicle 2 sends ACKs back, so the page shows its link + state.

  V1 HARDWARE: ESP32, L298N, 4 motors, GPS (NEO-6M), OLED SSD1306,
               2x HC-SR04 (front + rear), DHT11, MPU6050, buzzer,
               rain sensor module

  LIBRARIES (Library Manager):
    "DHT sensor library" (Adafruit), "Adafruit Unified Sensor",
    "Adafruit MPU6050", "Adafruit SSD1306", "Adafruit GFX Library",
    "TinyGPSPlus" (Mikal Hart)

  WIRING (Vehicle 1)
    DHT11 DATA ............ GPIO 4   (VCC 3.3V, GND)
    US FRONT  TRIG ........ GPIO 27  ECHO -> GPIO 26 (via divider!)
    US REAR   TRIG ........ GPIO 25  ECHO -> GPIO 33 (via divider!)
    GPS  TX -> GPIO 16 (ESP RX2)   GPS RX <- GPIO 17 (ESP TX2)
    OLED + MPU6050 (shared I2C) .. SDA GPIO 21, SCL GPIO 22 (3.3V)
    Rain sensor AO ........ GPIO 34  (VCC 3.3V, GND)
    Buzzer (active) + ..... GPIO 5   (- to GND)
    L298N: ENA GPIO14, IN1 GPIO13, IN2 GPIO18,
           ENB GPIO19, IN3 GPIO23, IN4 GPIO32
           OUT1/OUT2 = both LEFT motors in parallel
           OUT3/OUT4 = both RIGHT motors in parallel

  SETUP ORDER
    1. Same HOTSPOT_SSID / HOTSPOT_PASSWORD in both sketches.
    2. Phone hotspot ON (2.4 GHz).
    3. Flash Vehicle 2 first, note its MAC -> RECEIVER_MAC below.
    4. Flash this sketch, note ITS MAC -> TRANSMITTER_MAC in Vehicle 2,
       then re-flash Vehicle 2.
    5. Open the IP address shown on the OLED / Serial Monitor in your
       phone browser (http://vehicle1.local/ may not work on Android).
  =====================================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// ======================= USER CONFIGURATION =========================
const char *HOTSPOT_SSID     = "v2v";
const char *HOTSPOT_PASSWORD = "1234567890";
const char *MDNS_HOSTNAME    = "vehicle1";

uint8_t RECEIVER_MAC[6] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };  // <-- Vehicle 2's MAC

#define WIFI_CONNECT_TIMEOUT_MS  20000

// ======================= PINS =======================================
#define DHTPIN            4
#define DHTTYPE           DHT11

#define US_FRONT_TRIG    27
#define US_FRONT_ECHO    26
#define US_REAR_TRIG     25
#define US_REAR_ECHO     33
#define US_TIMEOUT_US    30000UL

#define GPS_RX_PIN       16
#define GPS_TX_PIN       17
#define GPS_BAUD         9600

#define I2C_SDA          21
#define I2C_SCL          22
#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT    64
#define OLED_ADDR        0x3C

#define RAIN_PIN         34
#define BUZZER_PIN       5

#define ENA_PIN   14
#define IN1_PIN   13
#define IN2_PIN   18
#define ENB_PIN   19
#define IN3_PIN   23
#define IN4_PIN   32
#define PWM_FREQ       5000
#define PWM_RES_BITS   8

// ======================= TUNING =====================================
#define OBSTACLE_DISTANCE_MM     300
#define BRAKE_ACCEL_DELTA_MS2    4.0
#define SENSOR_INTERVAL_MS       150
#define SENSOR_TX_INTERVAL_MS    300
#define CTRL_TX_INTERVAL_MS      100
#define COMMAND_TIMEOUT_MS       600     // manual: page silent -> stop
#define V2_LINK_TIMEOUT_MS       5000

#define RAIN_DRY_RAW             4095    // analog value when dry
#define RAIN_WET_RAW             1200    // analog value when soaked
#define RAIN_THRESHOLD_PCT       30
#define RAIN_MAX_SPEED           130     // auto-mode speed cap in rain

#define AUTO_PAUSE_MS            300
#define AUTO_REVERSE_MS          700
#define AUTO_TURN_MS             650

// ======================= OBJECTS ====================================
DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);
WebServer server(80);
bool mpuOK = false;
bool oledOK = false;

// ======================= SHARED TYPES ===============================
enum DriveCmd  : uint8_t { CMD_STOP = 0, CMD_FORWARD = 1, CMD_BACKWARD = 2, CMD_LEFT = 3, CMD_RIGHT = 4 };
enum DriveMode : uint8_t { MODE_MANUAL = 0, MODE_AUTO = 1 };
enum AutoState : uint8_t { AS_FORWARD = 0, AS_PAUSE = 1, AS_REVERSE = 2, AS_TURN = 3 };

// V1 -> V2 : sensor data (44 bytes). Must match Vehicle 2 exactly.
typedef struct __attribute__((packed)) v2v_packet_t {
  uint8_t  vehicleId;
  float    temperature;
  float    humidity;
  float    latitude;
  float    longitude;
  bool     gpsValid;
  float    accelX, accelY, accelZ;
  uint16_t distanceFrontMM;
  uint16_t distanceRearMM;
  bool     brakeAlert;
  bool     obstacleFrontAlert;
  bool     obstacleRearAlert;
  bool     emergencyAlert;
  bool     rainDetected;
  uint8_t  rainPercent;
  uint32_t packetId;
} v2v_packet_t;

// V2 -> V1 : acknowledgement + status
typedef struct __attribute__((packed)) v2v_ack_t {
  uint8_t  ackFromVehicle;   // 2
  uint32_t ackedPacketId;
  uint32_t peerPacketsOK;
  uint32_t peerPacketsRejected;
  uint32_t ctrlReceived;
  uint8_t  v2DriveState;
} v2v_ack_t;

// V1 -> V2 : drive command heartbeat (9 bytes)
typedef struct __attribute__((packed)) v2v_ctrl_t {
  uint8_t  ctrlMagic;        // 0xC2
  uint8_t  dir;
  uint8_t  speed;
  uint8_t  mode;
  uint8_t  alertLevel;       // 0 none, 1 info, 2 warning, 3 emergency
  uint32_t cmdId;
} v2v_ctrl_t;
#define CTRL_MAGIC 0xC2

// ======================= STATE ======================================
v2v_packet_t v1Data;
uint32_t packetCounter = 0;
float prevAccelMag = 0;
bool firstReading = true;
unsigned long lastSampleTime = 0, lastSensorTx = 0, lastCtrlTx = 0, lastOledTime = 0, lastDebug = 0;

volatile uint32_t v2AckedPacketId = 0, v2PacketsOK = 0, v2PacketsRejected = 0, v2CtrlRx = 0;
volatile uint8_t  v2DriveStateReported = CMD_STOP;
volatile unsigned long lastAckTime = 0;
volatile bool everAcked = false;

volatile DriveMode driveMode = MODE_MANUAL;
volatile bool autoRun = false;
AutoState autoState = AS_FORWARD;
unsigned long autoStateStart = 0;
bool autoTurnRight = true;

volatile DriveCmd currentCmd = CMD_STOP;   // manual request from page
volatile DriveCmd actualCmd  = CMD_STOP;   // what motors really do (both vehicles)
volatile int requestedSpeed  = 180;
int actualSpeed = 0;
volatile bool blockedForward = false, blockedBackward = false;
volatile unsigned long lastCommandTime = 0;

uint8_t alertLevel = 0;
uint32_t ctrlCounter = 0;
String ipStr = "";

bool v2LinkUp() { return everAcked && (millis() - lastAckTime < V2_LINK_TIMEOUT_MS); }

const char* cmdName(DriveCmd c) {
  switch (c) {
    case CMD_FORWARD:  return "FORWARD";
    case CMD_BACKWARD: return "BACKWARD";
    case CMD_LEFT:     return "LEFT";
    case CMD_RIGHT:    return "RIGHT";
    default:           return "STOPPED";
  }
}

// ======================= ESP-NOW CALLBACKS ==========================
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

void onAckRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(v2v_ack_t)) return;
  v2v_ack_t ack;
  memcpy(&ack, incomingData, sizeof(ack));
  if (ack.ackFromVehicle != 2) return;
  v2AckedPacketId      = ack.ackedPacketId;
  v2PacketsOK          = ack.peerPacketsOK;
  v2PacketsRejected    = ack.peerPacketsRejected;
  v2CtrlRx             = ack.ctrlReceived;
  v2DriveStateReported = ack.v2DriveState;
  lastAckTime          = millis();
  everAcked            = true;
}

// ======================= SENSORS ====================================
uint16_t readUltrasonicMM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, US_TIMEOUT_US);
  if (duration == 0) return 9999;
  long mm = (long)(duration * 0.343 / 2.0);
  if (mm <= 0 || mm > 4500) return 9999;
  return (uint16_t)mm;
}

void readAllSensors() {
  static unsigned long lastDht = 0;
  if (millis() - lastDht >= 2000) {          // DHT11 max rate = once / 2 s
    lastDht = millis();
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t)) v1Data.temperature = t;
    if (!isnan(h)) v1Data.humidity = h;
  }

  if (gps.location.isValid()) {
    v1Data.latitude  = (float)gps.location.lat();
    v1Data.longitude = (float)gps.location.lng();
    v1Data.gpsValid  = true;
  } else {
    v1Data.gpsValid = false;
  }

  if (mpuOK) {
    sensors_event_t a, g, te;
    mpu.getEvent(&a, &g, &te);
    v1Data.accelX = a.acceleration.x;
    v1Data.accelY = a.acceleration.y;
    v1Data.accelZ = a.acceleration.z;
  }

  int raw = analogRead(RAIN_PIN);
  int pct = map(raw, RAIN_DRY_RAW, RAIN_WET_RAW, 0, 100);
  pct = constrain(pct, 0, 100);
  v1Data.rainPercent  = (uint8_t)pct;
  v1Data.rainDetected = (pct >= RAIN_THRESHOLD_PCT);

  v1Data.distanceFrontMM = readUltrasonicMM(US_FRONT_TRIG, US_FRONT_ECHO);
  v1Data.distanceRearMM  = readUltrasonicMM(US_REAR_TRIG, US_REAR_ECHO);

  v1Data.packetId = ++packetCounter;
}

void evaluateSafetyConditions() {
  float mag = sqrt(sq(v1Data.accelX) + sq(v1Data.accelY) + sq(v1Data.accelZ));
  if (firstReading) { prevAccelMag = mag; firstReading = false; }
  v1Data.brakeAlert = (fabs(mag - prevAccelMag) > BRAKE_ACCEL_DELTA_MS2);
  prevAccelMag = mag;

  v1Data.obstacleFrontAlert = (v1Data.distanceFrontMM < OBSTACLE_DISTANCE_MM);
  v1Data.obstacleRearAlert  = (v1Data.distanceRearMM  < OBSTACLE_DISTANCE_MM);
  v1Data.emergencyAlert = v1Data.brakeAlert && (v1Data.obstacleFrontAlert || v1Data.obstacleRearAlert);

  if (v1Data.emergencyAlert) alertLevel = 3;
  else if (v1Data.obstacleFrontAlert || v1Data.obstacleRearAlert) alertLevel = 2;
  else if (v1Data.brakeAlert || v1Data.rainDetected) alertLevel = 1;
  else alertLevel = 0;
}

// ======================= BUZZER =====================================
void driveBuzzer(uint8_t level) {
  unsigned long t = millis();
  bool on = false;
  switch (level) {
    case 3: on = true; break;
    case 2: on = (t % 400) < 200; break;
    case 1: on = (t % 2000) < 100; break;
    default: on = false;
  }
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}

// ======================= MOTORS (4 motors, 2 per side) ==============
void motorsStop() {
  ledcWrite(ENA_PIN, 0); ledcWrite(ENB_PIN, 0);
  digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);
}
void setLeftPair(bool fwd, int speed) {
  digitalWrite(IN1_PIN, fwd ? HIGH : LOW);
  digitalWrite(IN2_PIN, fwd ? LOW  : HIGH);
  ledcWrite(ENA_PIN, speed);
}
void setRightPair(bool fwd, int speed) {
  digitalWrite(IN3_PIN, fwd ? HIGH : LOW);
  digitalWrite(IN4_PIN, fwd ? LOW  : HIGH);
  ledcWrite(ENB_PIN, speed);
}
void applyMotorCommand(DriveCmd cmd, int speed) {
  speed = constrain(speed, 0, 255);
  switch (cmd) {
    case CMD_FORWARD:  setLeftPair(true,  speed); setRightPair(true,  speed); break;
    case CMD_BACKWARD: setLeftPair(false, speed); setRightPair(false, speed); break;
    case CMD_LEFT:     setLeftPair(false, speed); setRightPair(true,  speed); break;
    case CMD_RIGHT:    setLeftPair(true,  speed); setRightPair(false, speed); break;
    default:           motorsStop(); break;
  }
}

// ======================= AUTO MODE (obstacle avoidance) =============
void autoReset() { autoState = AS_FORWARD; autoStateStart = millis(); }

DriveCmd autoStep() {
  unsigned long now = millis();
  switch (autoState) {
    case AS_FORWARD:
      if (v1Data.obstacleFrontAlert) { autoState = AS_PAUSE; autoStateStart = now; return CMD_STOP; }
      return CMD_FORWARD;

    case AS_PAUSE:
      if (now - autoStateStart >= AUTO_PAUSE_MS) {
        autoState = v1Data.obstacleRearAlert ? AS_TURN : AS_REVERSE;
        autoStateStart = now;
        if (autoState == AS_TURN) autoTurnRight = !autoTurnRight;
      }
      return CMD_STOP;

    case AS_REVERSE:
      if (v1Data.obstacleRearAlert || now - autoStateStart >= AUTO_REVERSE_MS) {
        autoState = AS_TURN; autoStateStart = now; autoTurnRight = !autoTurnRight;
        return CMD_STOP;
      }
      return CMD_BACKWARD;

    case AS_TURN:
      if (now - autoStateStart >= AUTO_TURN_MS) {
        autoState = v1Data.obstacleFrontAlert ? AS_PAUSE : AS_FORWARD;
        autoStateStart = now;
        return CMD_STOP;
      }
      return autoTurnRight ? CMD_RIGHT : CMD_LEFT;
  }
  return CMD_STOP;
}

// ======================= DRIVE DECISION (both vehicles) =============
void updateDrive() {
  DriveCmd wanted = CMD_STOP;
  int spd = requestedSpeed;

  if (driveMode == MODE_MANUAL) {
    bool timedOut = (millis() - lastCommandTime > COMMAND_TIMEOUT_MS);
    wanted = timedOut ? CMD_STOP : currentCmd;
  } else {
    if (autoRun) {
      wanted = autoStep();
      if (v1Data.rainDetected && spd > RAIN_MAX_SPEED) spd = RAIN_MAX_SPEED;
    } else {
      autoReset();
    }
  }

  blockedForward = false; blockedBackward = false;
  if (wanted == CMD_FORWARD  && v1Data.obstacleFrontAlert) { wanted = CMD_STOP; blockedForward = true; }
  if (wanted == CMD_BACKWARD && v1Data.obstacleRearAlert)  { wanted = CMD_STOP; blockedBackward = true; }

  actualCmd = wanted;
  actualSpeed = spd;
  applyMotorCommand(wanted, spd);
}

// ======================= ESP-NOW TX =================================
void sendSensorsToVehicle2() {
  esp_now_send(RECEIVER_MAC, (uint8_t *)&v1Data, sizeof(v1Data));
}
void sendCtrlToVehicle2() {
  v2v_ctrl_t c;
  c.ctrlMagic  = CTRL_MAGIC;
  c.dir        = (uint8_t)actualCmd;
  c.speed      = (uint8_t)constrain(actualSpeed, 0, 255);
  c.mode       = (uint8_t)driveMode;
  c.alertLevel = alertLevel;
  c.cmdId      = ++ctrlCounter;
  esp_now_send(RECEIVER_MAC, (uint8_t *)&c, sizeof(c));
}

// ======================= WEB PAGE ===================================
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>V2V Fleet Console</title>
<style>
:root{
  --bg:#e9edf1; --card:#ffffff; --ink:#142030; --mute:#64748b; --line:#d5dde6; --soft:#f3f6f9;
  --v1:#0b6e66; --v2:#4338ca; --ok:#15803d; --warn:#b45309; --bad:#c81e1e;
}
*{box-sizing:border-box; -webkit-tap-highlight-color:transparent;}
body{margin:0; font-family:"Segoe UI",system-ui,Roboto,Arial,sans-serif; background:var(--bg); color:var(--ink);
     padding:0 0 40px; -webkit-user-select:none; user-select:none;}
header{background:#142030; color:#fff; padding:14px 18px; display:flex; flex-wrap:wrap; gap:12px;
       justify-content:space-between; align-items:center;}
header h1{font-size:19px; margin:0; font-weight:650; letter-spacing:.2px;}
header p{margin:2px 0 0; font-size:12.5px; color:#9fb0c6;}
.pills{display:flex; gap:8px; flex-wrap:wrap;}
.pill{display:flex; align-items:center; gap:7px; font-size:12.5px; padding:6px 12px; border-radius:999px;
      background:#1f3047; color:#dbe5f2;}
.dot{width:9px; height:9px; border-radius:50%; background:#ef4444;}
.dot.on{background:#22c55e;}
.wrap{max-width:1180px; margin:0 auto; padding:14px;}
.banner{border-radius:10px; padding:12px 16px; font-weight:650; font-size:14px; background:#dcfce7; color:#14532d;
        border:1px solid #86efac; margin-bottom:14px;}
.banner.warn{background:#fef3c7; color:#78350f; border-color:#fcd34d;}
.banner.bad{background:#fee2e2; color:#7f1d1d; border-color:#fca5a5;}
.layout{display:grid; grid-template-columns:1fr; gap:14px;}
@media(min-width:900px){ .layout{grid-template-columns:400px 1fr;} }
.col{display:flex; flex-direction:column; gap:14px;}
.card{background:var(--card); border:1px solid var(--line); border-radius:12px; padding:16px;}
.card h2{margin:0 0 12px; font-size:15px; font-weight:650; display:flex; justify-content:space-between; align-items:center;}
.chip{font-size:11.5px; font-weight:600; padding:3px 10px; border-radius:999px; background:var(--soft); color:var(--mute); border:1px solid var(--line);}
.chip.v1{color:var(--v1); border-color:var(--v1);} .chip.v2{color:var(--v2); border-color:var(--v2);}
.chip.up{color:var(--ok); border-color:var(--ok);} .chip.down{color:var(--bad); border-color:var(--bad);}

.seg{display:grid; grid-template-columns:1fr 1fr; background:var(--soft); border:1px solid var(--line); border-radius:10px; padding:4px; gap:4px;}
.seg button{border:0; padding:11px; border-radius:7px; font-size:14.5px; font-weight:650; background:transparent; color:var(--mute); cursor:pointer;}
.seg button.active{background:#142030; color:#fff;}
.autobar{display:none; margin-top:12px; padding:12px; background:var(--soft); border:1px solid var(--line); border-radius:10px;}
.autobar.show{display:block;}
.autobar .row{display:flex; justify-content:space-between; align-items:center; gap:10px;}
.autobar .st{font-size:13px; color:var(--mute);} .autobar .st b{color:var(--ink);}
.go{border:0; border-radius:8px; padding:10px 16px; font-weight:650; font-size:14px; color:#fff; background:var(--ok); cursor:pointer;}
.go.running{background:var(--bad);}

.dpad{display:grid; grid-template-columns:repeat(3,76px); grid-template-rows:repeat(3,76px); gap:10px; justify-content:center; margin:18px 0 8px;}
.dpad.off{opacity:.35; pointer-events:none;}
.btn{border:1px solid var(--line); background:var(--soft); border-radius:14px; font-size:26px; color:var(--ink);
     display:flex; align-items:center; justify-content:center; cursor:pointer; box-shadow:0 3px 0 var(--line); touch-action:none;}
.btn.pressed{background:#142030; color:#fff; transform:translateY(3px); box-shadow:none;}
.fwd{grid-column:2; grid-row:1;} .left{grid-column:1; grid-row:2;} .stp{grid-column:2; grid-row:2; font-size:13px; font-weight:700; color:var(--bad);}
.right{grid-column:3; grid-row:2;} .back{grid-column:2; grid-row:3;}
.hint{text-align:center; font-size:12px; color:var(--mute);}
.speed{margin-top:14px;} .speed label{display:flex; justify-content:space-between; font-size:13px; color:var(--mute);}
.speed input{width:100%; accent-color:#142030; margin-top:6px;}
.estop{width:100%; margin-top:14px; padding:13px; border:0; border-radius:10px; background:var(--bad); color:#fff; font-size:15px; font-weight:700; cursor:pointer;}
.estop:active{filter:brightness(.85);}
.states{display:grid; grid-template-columns:1fr 1fr; gap:10px; margin-top:14px;}
.state{border:1px solid var(--line); border-radius:10px; padding:10px; background:var(--soft);}
.state small{color:var(--mute); font-size:12px;} .state b{display:block; font-size:15px; margin-top:2px;}
.state.a b{color:var(--v1);} .state.b b{color:var(--v2);}

.grid{display:grid; grid-template-columns:repeat(auto-fill,minmax(135px,1fr)); gap:10px;}
.cell{border:1px solid var(--line); border-radius:10px; padding:10px 12px; background:var(--soft);}
.cell small{display:block; color:var(--mute); font-size:12px;}
.cell b{display:block; font-size:18px; margin-top:3px; font-weight:650;}
.cell.alert{border-color:var(--bad); background:#fef2f2;} .cell.alert b{color:var(--bad);}
.cell.good{border-color:#86efac;}
.cell.wide{grid-column:span 2;}
.bar{height:6px; background:#dde4ec; border-radius:4px; margin-top:8px; overflow:hidden;}
.bar i{display:block; height:100%; width:0; background:var(--v1); transition:width .3s;}
.cell.alert .bar i{background:var(--bad);}
a{color:var(--v1); font-size:12px; text-decoration:none; font-weight:600;}
.foot{text-align:center; color:var(--mute); font-size:12px; margin-top:14px;}
</style>
</head>
<body>
<header>
  <div><h1>V2V Fleet Console</h1><p>One page, both vehicles. Commands go to Vehicle 1 and are mirrored to Vehicle 2.</p></div>
  <div class="pills">
    <div class="pill"><span class="dot" id="d1"></span><span id="t1">Vehicle 1</span></div>
    <div class="pill"><span class="dot" id="d2"></span><span id="t2">Vehicle 2</span></div>
    <div class="pill" id="modePill">Manual mode</div>
  </div>
</header>

<div class="wrap">
<div class="banner" id="banner">Connecting to Vehicle 1...</div>
<div class="layout">

  <div class="col">
    <div class="card">
      <h2>Drive control <span class="chip" id="modeChip">Manual</span></h2>
      <div class="seg">
        <button id="bManual" class="active">Manual</button>
        <button id="bAuto">Auto</button>
      </div>
      <div class="autobar" id="autobar">
        <div class="row">
          <div class="st">Auto state<br><b id="autoState">Idle</b></div>
          <button class="go" id="bGo">Start auto drive</button>
        </div>
        <div class="hint" style="margin-top:8px;text-align:left">Vehicle 1 avoids obstacles with its front and rear sensors. Vehicle 2 copies each move.</div>
      </div>

      <div class="dpad" id="dpad">
        <button class="btn fwd"  id="bF">&#9650;</button>
        <button class="btn left" id="bL">&#9664;</button>
        <button class="btn stp"  id="bS">STOP</button>
        <button class="btn right" id="bR">&#9654;</button>
        <button class="btn back" id="bB">&#9660;</button>
      </div>
      <div class="hint" id="dpadHint">Hold a button to move both vehicles. Release to stop. Keys: arrows / WASD, space = stop.</div>

      <div class="speed">
        <label><span>Speed (both vehicles)</span><span><b id="spdVal">180</b> / 255</span></label>
        <input type="range" id="spd" min="60" max="255" value="180">
      </div>
      <button class="estop" id="estop">Emergency stop both vehicles</button>

      <div class="states">
        <div class="state a"><small>Vehicle 1 motors</small><b id="m1">STOPPED</b></div>
        <div class="state b"><small>Vehicle 2 motors</small><b id="m2">STOPPED</b></div>
      </div>
    </div>
  </div>

  <div class="col">
    <div class="card">
      <h2>Vehicle 1 sensors <span class="chip v1">Transmitter</span></h2>
      <div class="grid">
        <div class="cell"><small>Temperature</small><b id="vTemp">--</b></div>
        <div class="cell"><small>Humidity</small><b id="vHum">--</b></div>
        <div class="cell" id="cRain"><small>Rain</small><b id="vRain">--</b></div>
        <div class="cell" id="cFront"><small>Front distance</small><b id="vFront">--</b><div class="bar"><i id="bFront"></i></div></div>
        <div class="cell" id="cRear"><small>Rear distance</small><b id="vRear">--</b><div class="bar"><i id="bRear"></i></div></div>
        <div class="cell" id="cBrake"><small>Sudden braking</small><b id="vBrake">--</b></div>
        <div class="cell wide"><small>GPS position</small><b id="vGps" style="font-size:15px">--</b><a id="vMap" href="#" target="_blank"></a></div>
        <div class="cell wide"><small>Acceleration X / Y / Z (m/s&sup2;)</small><b id="vAcc" style="font-size:15px">--</b></div>
        <div class="cell" id="cBuzz"><small>Buzzer</small><b id="vBuzz">--</b></div>
        <div class="cell" id="cBlock"><small>Safety block</small><b id="vBlock">--</b></div>
      </div>
    </div>

    <div class="card">
      <h2>Vehicle 2 link <span class="chip down" id="v2Chip">Down</span></h2>
      <div class="grid">
        <div class="cell"><small>Last packet sent</small><b id="lSent">--</b></div>
        <div class="cell"><small>Confirmed by V2</small><b id="lAck">--</b></div>
        <div class="cell"><small>V2 accepted</small><b id="lOk">--</b></div>
        <div class="cell"><small>V2 rejected</small><b id="lRej">--</b></div>
        <div class="cell"><small>Commands received</small><b id="lCtl">--</b></div>
        <div class="cell"><small>ACK age</small><b id="lAge">--</b></div>
      </div>
    </div>
  </div>
</div>
<div class="foot">Vehicle 2 stops by itself if commands from Vehicle 1 stop arriving for 0.6 s.</div>
</div>

<script>
const $ = id => document.getElementById(id);
const names = ['STOPPED','FORWARD','BACKWARD','LEFT','RIGHT'];
const autoNames = ['Driving forward','Obstacle - pausing','Reversing','Turning away'];
let mode = 0, autoRun = false, activeDir = null, lastOK = 0;

const api = u => fetch(u, {cache:'no-store'}).catch(()=>{});
const sendDir = d => api('/control?dir=' + d + '&speed=' + $('spd').value);

function bindHold(id, dir){
  const b = $(id);
  const start = e => { e.preventDefault(); if(mode!==0) return; activeDir = dir; b.classList.add('pressed'); sendDir(dir); };
  const end   = e => { e.preventDefault(); if(activeDir===dir){ activeDir = null; } b.classList.remove('pressed'); if(mode===0) sendDir('stop'); };
  b.addEventListener('touchstart', start, {passive:false});
  b.addEventListener('touchend', end, {passive:false});
  b.addEventListener('touchcancel', end, {passive:false});
  b.addEventListener('mousedown', start);
  b.addEventListener('mouseup', end);
  b.addEventListener('mouseleave', e => { if(activeDir===dir) end(e); });
}
bindHold('bF','forward'); bindHold('bB','backward'); bindHold('bL','left'); bindHold('bR','right');
$('bS').addEventListener('click', ()=> sendDir('stop'));
setInterval(()=>{ if(activeDir && mode===0) sendDir(activeDir); }, 250);

$('spd').addEventListener('input', e => { $('spdVal').innerText = e.target.value; if(activeDir) sendDir(activeDir); });
$('bManual').onclick = ()=> api('/mode?m=manual');
$('bAuto').onclick   = ()=> api('/mode?m=auto');
$('bGo').onclick     = ()=> api('/auto?run=' + (autoRun ? 0 : 1));
$('estop').onclick   = ()=> { activeDir = null; api('/estop'); };

const keys = {ArrowUp:'bF', w:'bF', ArrowDown:'bB', s:'bB', ArrowLeft:'bL', a:'bL', ArrowRight:'bR', d:'bR'};
const held = {};
document.addEventListener('keydown', e => {
  if(e.key === ' '){ e.preventDefault(); $('estop').click(); return; }
  const id = keys[e.key]; if(id && !held[e.key]){ held[e.key] = true; $(id).dispatchEvent(new Event('mousedown')); }
});
document.addEventListener('keyup', e => {
  const id = keys[e.key]; if(id){ held[e.key] = false; $(id).dispatchEvent(new Event('mouseup')); }
});

function setAlert(id, on){ const el = $(id); el.classList.toggle('alert', !!on); }
function distText(v){ return v >= 9999 ? 'No echo' : v + ' mm'; }
function distBar(v){ return v >= 9999 ? 0 : Math.min(100, v / 10); }

function render(d){
  mode = d.mode; autoRun = d.autoRun;
  $('d1').classList.add('on'); $('t1').innerText = 'Vehicle 1 online';
  $('d2').classList.toggle('on', d.v2Linked); $('t2').innerText = d.v2Linked ? 'Vehicle 2 linked' : 'Vehicle 2 no ACK';
  $('modePill').innerText = mode ? 'Auto mode' : 'Manual mode';
  $('modeChip').innerText = mode ? 'Auto' : 'Manual';
  $('bManual').classList.toggle('active', !mode);
  $('bAuto').classList.toggle('active', !!mode);
  $('autobar').classList.toggle('show', !!mode);
  $('dpad').classList.toggle('off', !!mode);
  $('dpadHint').innerText = mode ? 'Manual buttons are locked in auto mode.' : 'Hold a button to move both vehicles. Release to stop. Keys: arrows / WASD, space = stop.';
  $('bGo').innerText = autoRun ? 'Stop auto drive' : 'Start auto drive';
  $('bGo').classList.toggle('running', autoRun);
  $('autoState').innerText = autoRun ? (autoNames[d.autoState] || '--') : 'Idle';

  $('m1').innerText = d.motorState;
  $('m2').innerText = d.v2Linked ? (names[d.v2DriveState] || '--') : 'NO LINK';

  $('vTemp').innerText = d.temperature.toFixed(1) + ' \u00B0C';
  $('vHum').innerText  = d.humidity.toFixed(0) + ' %';
  $('vRain').innerText = d.rain ? ('Raining (' + d.rainPct + '%)') : ('Dry (' + d.rainPct + '%)');
  $('vFront').innerText = distText(d.distanceFront); $('bFront').style.width = distBar(d.distanceFront) + '%';
  $('vRear').innerText  = distText(d.distanceRear);  $('bRear').style.width  = distBar(d.distanceRear) + '%';
  $('vBrake').innerText = d.brake ? 'Detected' : 'Normal';
  $('vAcc').innerText = d.accelX.toFixed(1) + ' / ' + d.accelY.toFixed(1) + ' / ' + d.accelZ.toFixed(1);
  if(d.gpsValid){
    $('vGps').innerText = d.latitude.toFixed(5) + ', ' + d.longitude.toFixed(5);
    $('vMap').href = 'https://maps.google.com/?q=' + d.latitude + ',' + d.longitude; $('vMap').innerText = 'Open in Maps';
  } else { $('vGps').innerText = 'Searching for fix'; $('vMap').innerText = ''; }
  $('vBuzz').innerText = ['Silent','Notice','Warning','Emergency'][d.alertLevel] || '--';
  $('vBlock').innerText = d.blockedForward ? 'Front blocked' : (d.blockedBackward ? 'Rear blocked' : 'Clear');

  $('lSent').innerText = d.packetId; $('lAck').innerText = d.v2AckedPacketId;
  $('lOk').innerText = d.v2PacketsOK; $('lRej').innerText = d.v2PacketsRejected; $('lCtl').innerText = d.v2CtrlRx;
  $('lAge').innerText = d.v2Linked ? (d.v2AckAgeMs + ' ms') : 'Lost';
  const chip = $('v2Chip'); chip.innerText = d.v2Linked ? 'Up' : 'Down';
  chip.classList.toggle('up', d.v2Linked); chip.classList.toggle('down', !d.v2Linked);

  setAlert('cRain', d.rain); setAlert('cFront', d.obstacleFront); setAlert('cRear', d.obstacleRear);
  setAlert('cBrake', d.brake); setAlert('cBuzz', d.alertLevel >= 2); setAlert('cBlock', d.blockedForward || d.blockedBackward);

  const b = $('banner'); b.className = 'banner';
  if(d.emergency){ b.innerText = 'Emergency: sudden braking with an obstacle close to Vehicle 1'; b.classList.add('bad'); }
  else if(d.blockedForward){ b.innerText = 'Both vehicles held: obstacle in front of Vehicle 1'; b.classList.add('bad'); }
  else if(d.blockedBackward){ b.innerText = 'Both vehicles held: obstacle behind Vehicle 1'; b.classList.add('bad'); }
  else if(!d.v2Linked){ b.innerText = 'Vehicle 2 is not confirming. Check its power and hotspot connection.'; b.classList.add('warn'); }
  else if(d.rain){ b.innerText = 'Rain detected' + (mode ? '. Auto speed is limited.' : '. Drive carefully.'); b.classList.add('warn'); }
  else if(d.brake){ b.innerText = 'Sudden braking detected on Vehicle 1'; b.classList.add('warn'); }
  else if(d.obstacleFront || d.obstacleRear){ b.innerText = 'Obstacle nearby'; b.classList.add('warn'); }
  else { b.innerText = 'All systems normal. Both vehicles linked.'; }
}

async function poll(){
  try{
    const r = await fetch('/data', {cache:'no-store'});
    render(await r.json()); lastOK = Date.now();
  }catch(e){
    $('d1').classList.remove('on'); $('d2').classList.remove('on');
    $('t1').innerText = 'Vehicle 1 offline';
    const b = $('banner'); b.className = 'banner bad'; b.innerText = 'Lost connection to Vehicle 1. Check that your phone is on the same hotspot.';
  }
  setTimeout(poll, 400);
}
poll();
</script>
</body>
</html>
)HTMLPAGE";

// ======================= HTTP HANDLERS ==============================
DriveCmd parseDir(const String &dir) {
  if (dir == "forward")  return CMD_FORWARD;
  if (dir == "backward") return CMD_BACKWARD;
  if (dir == "left")     return CMD_LEFT;
  if (dir == "right")    return CMD_RIGHT;
  return CMD_STOP;
}

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleControl() {
  if (driveMode != MODE_MANUAL) { server.send(200, "text/plain", "AUTO"); return; }
  if (server.hasArg("speed")) requestedSpeed = constrain(server.arg("speed").toInt(), 0, 255);
  currentCmd = parseDir(server.hasArg("dir") ? server.arg("dir") : "stop");
  lastCommandTime = millis();
  server.send(200, "text/plain", "OK");
}

void handleMode() {
  String m = server.hasArg("m") ? server.arg("m") : "manual";
  driveMode = (m == "auto") ? MODE_AUTO : MODE_MANUAL;
  autoRun = false;
  currentCmd = CMD_STOP;
  autoReset();
  server.send(200, "text/plain", "OK");
}

void handleAuto() {
  if (driveMode == MODE_AUTO) {
    autoRun = (server.hasArg("run") && server.arg("run") == "1");
    autoReset();
  }
  server.send(200, "text/plain", "OK");
}

void handleEstop() {
  autoRun = false;
  currentCmd = CMD_STOP;
  lastCommandTime = 0;
  server.send(200, "text/plain", "OK");
}

void handleData() {
  bool v2up = v2LinkUp();
  unsigned long ackAge = everAcked ? (millis() - lastAckTime) : 0;
  String j = "{";
  j += "\"temperature\":" + String(v1Data.temperature, 1) + ",";
  j += "\"humidity\":" + String(v1Data.humidity, 1) + ",";
  j += "\"rain\":" + String(v1Data.rainDetected ? "true" : "false") + ",";
  j += "\"rainPct\":" + String(v1Data.rainPercent) + ",";
  j += "\"gpsValid\":" + String(v1Data.gpsValid ? "true" : "false") + ",";
  j += "\"latitude\":" + String(v1Data.latitude, 6) + ",";
  j += "\"longitude\":" + String(v1Data.longitude, 6) + ",";
  j += "\"accelX\":" + String(v1Data.accelX, 2) + ",";
  j += "\"accelY\":" + String(v1Data.accelY, 2) + ",";
  j += "\"accelZ\":" + String(v1Data.accelZ, 2) + ",";
  j += "\"distanceFront\":" + String(v1Data.distanceFrontMM) + ",";
  j += "\"distanceRear\":" + String(v1Data.distanceRearMM) + ",";
  j += "\"brake\":" + String(v1Data.brakeAlert ? "true" : "false") + ",";
  j += "\"obstacleFront\":" + String(v1Data.obstacleFrontAlert ? "true" : "false") + ",";
  j += "\"obstacleRear\":" + String(v1Data.obstacleRearAlert ? "true" : "false") + ",";
  j += "\"emergency\":" + String(v1Data.emergencyAlert ? "true" : "false") + ",";
  j += "\"blockedForward\":" + String(blockedForward ? "true" : "false") + ",";
  j += "\"blockedBackward\":" + String(blockedBackward ? "true" : "false") + ",";
  j += "\"mode\":" + String((int)driveMode) + ",";
  j += "\"autoRun\":" + String(autoRun ? "true" : "false") + ",";
  j += "\"autoState\":" + String((int)autoState) + ",";
  j += "\"alertLevel\":" + String(alertLevel) + ",";
  j += "\"motorState\":\"" + String(cmdName(actualCmd)) + "\",";
  j += "\"packetId\":" + String(v1Data.packetId) + ",";
  j += "\"v2Linked\":" + String(v2up ? "true" : "false") + ",";
  j += "\"v2AckedPacketId\":" + String(v2AckedPacketId) + ",";
  j += "\"v2PacketsOK\":" + String(v2PacketsOK) + ",";
  j += "\"v2PacketsRejected\":" + String(v2PacketsRejected) + ",";
  j += "\"v2CtrlRx\":" + String(v2CtrlRx) + ",";
  j += "\"v2AckAgeMs\":" + String(ackAge) + ",";
  j += "\"v2DriveState\":" + String(v2DriveStateReported);
  j += "}";
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", j);
}

// ======================= OLED =======================================
void updateOLED() {
  if (!oledOK) return;
  static unsigned long lastToggle = 0;
  static uint8_t page = 0;
  if (millis() - lastToggle > 2500) { lastToggle = millis(); page = (page + 1) % 3; }

  oled.clearDisplay();
  oled.setCursor(0, 0);
  if (page == 0) {
    oled.print("T:"); oled.print(v1Data.temperature, 1); oled.print("C H:"); oled.print(v1Data.humidity, 0); oled.println("%");
    oled.print("Fr:"); oled.print(v1Data.distanceFrontMM < 9999 ? String(v1Data.distanceFrontMM) : "--"); oled.println("mm");
    oled.print("Rr:"); oled.print(v1Data.distanceRearMM  < 9999 ? String(v1Data.distanceRearMM)  : "--"); oled.println("mm");
    oled.print("Rain:"); oled.print(v1Data.rainPercent); oled.println(v1Data.rainDetected ? "% WET" : "% dry");
    if (v1Data.gpsValid) { oled.print(v1Data.latitude, 4); oled.print(","); oled.println(v1Data.longitude, 4); }
    else oled.println("GPS: no fix");
  } else if (page == 1) {
    oled.println("V1 DRIVE STATUS");
    oled.print("Mode: "); oled.println(driveMode == MODE_AUTO ? (autoRun ? "AUTO RUN" : "AUTO idle") : "MANUAL");
    oled.print("V1: "); oled.println(cmdName(actualCmd));
    oled.print("V2: "); oled.println(v2LinkUp() ? cmdName((DriveCmd)v2DriveStateReported) : "LINK DOWN");
    if (v1Data.emergencyAlert)  oled.println("!! EMERGENCY !!");
    else if (blockedForward)    oled.println("FWD BLOCKED");
    else if (blockedBackward)   oled.println("BACK BLOCKED");
    else                        oled.println("Status: OK");
  } else {
    oled.println("OPEN IN BROWSER:");
    oled.println(ipStr);
    oled.println("");
    oled.print("Speed: "); oled.println(actualSpeed);
    oled.print("Pkt#"); oled.println(v1Data.packetId);
  }
  oled.display();
}

void printDebug() {
  Serial.printf("#%lu T:%.1f H:%.0f Fr:%u Rr:%u Rain:%u%% Mode:%s V1:%s V2:%s link:%d alert:%u\n",
    (unsigned long)v1Data.packetId, v1Data.temperature, v1Data.humidity,
    v1Data.distanceFrontMM, v1Data.distanceRearMM, v1Data.rainPercent,
    driveMode == MODE_AUTO ? "AUTO" : "MANUAL", cmdName(actualCmd),
    cmdName((DriveCmd)v2DriveStateReported), v2LinkUp(), alertLevel);
}

// ======================= SETUP ======================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== VEHICLE 1 MASTER BOOTING ===");

  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOK) {
    oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0); oled.println("V2V Fleet Console"); oled.println("Initializing..."); oled.display();
  } else Serial.println("OLED: NOT FOUND");

  dht.begin();
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  mpuOK = mpu.begin();
  if (mpuOK) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050: OK");
  } else Serial.println("MPU6050: NOT FOUND");

  pinMode(US_FRONT_TRIG, OUTPUT); pinMode(US_FRONT_ECHO, INPUT);
  pinMode(US_REAR_TRIG, OUTPUT);  pinMode(US_REAR_ECHO, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(RAIN_PIN, ADC_11db);

  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);
  ledcAttach(ENA_PIN, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(ENB_PIN, PWM_FREQ, PWM_RES_BITS);
  motorsStop();

  // Start with "no obstacle" so nothing is blocked before the first reading.
  v1Data.vehicleId = 1;
  v1Data.distanceFrontMM = 9999;
  v1Data.distanceRearMM  = 9999;

  WiFi.mode(WIFI_STA);
  WiFi.begin(HOTSPOT_SSID, HOTSPOT_PASSWORD);
  Serial.print("Connecting to hotspot \""); Serial.print(HOTSPOT_SSID); Serial.print("\"");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300); Serial.print(".");
    if (millis() - t0 > WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println("\nCould not join hotspot - restarting.");
      if (oledOK) { oled.clearDisplay(); oled.setCursor(0,0); oled.println("Hotspot FAILED"); oled.display(); }
      delay(2000); ESP.restart();
    }
  }
  ipStr = WiFi.localIP().toString();
  Serial.println();
  Serial.print("Connected! Open this in your phone browser: http://"); Serial.println(ipStr);
  Serial.print("Vehicle 1 MAC (put in Vehicle 2's TRANSMITTER_MAC[]): "); Serial.println(WiFi.macAddress());

  if (MDNS.begin(MDNS_HOSTNAME)) { MDNS.addService("http", "tcp", 80); Serial.println("Also: http://vehicle1.local/ (iPhone/PC)"); }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    if (oledOK) { oled.clearDisplay(); oled.setCursor(0,0); oled.println("ESP-NOW FAIL"); oled.display(); }
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onAckRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, RECEIVER_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) Serial.println("Failed to add Vehicle 2 as peer - check RECEIVER_MAC[]");

  server.on("/", handleRoot);
  server.on("/control", handleControl);
  server.on("/mode", handleMode);
  server.on("/auto", handleAuto);
  server.on("/estop", handleEstop);
  server.on("/data", handleData);
  server.begin();

  if (oledOK) {
    oled.clearDisplay(); oled.setCursor(0, 0);
    oled.println("Open in browser:"); oled.println(ipStr); oled.display();
  }
  lastCommandTime = millis();
  delay(3000);
  Serial.println("=== VEHICLE 1 READY ===");
}

// ======================= LOOP =======================================
void loop() {
  server.handleClient();

  while (GPSSerial.available() > 0) gps.encode(GPSSerial.read());

  unsigned long now = millis();

  if (now - lastSampleTime >= SENSOR_INTERVAL_MS) {
    lastSampleTime = now;
    readAllSensors();
    evaluateSafetyConditions();
  }

  updateDrive();
  driveBuzzer(alertLevel);

  if (now - lastCtrlTx >= CTRL_TX_INTERVAL_MS) { lastCtrlTx = now; sendCtrlToVehicle2(); }
  if (now - lastSensorTx >= SENSOR_TX_INTERVAL_MS) { lastSensorTx = now; sendSensorsToVehicle2(); }
  if (now - lastOledTime >= 400) { lastOledTime = now; updateOLED(); }
  if (now - lastDebug >= 1000) { lastDebug = now; printDebug(); }
}
