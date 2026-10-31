/*
  =====================================================================
  ESP32 V2V Communication + Collision Warning System (ESP-NOW)
  VEHICLE 2  =  FOLLOWER  (mirrors every move of Vehicle 1)
  =====================================================================
  Compatible with ESP32 Arduino core 3.x (IDF 5.x). Board: ESP32 Dev Module

  V2 HARDWARE: ESP32, L298N, 4 motors, OLED SSD1306, buzzer

  WHAT THIS BOARD DOES
    * Receives Vehicle 1's drive command (every 100 ms) and executes it,
      so both vehicles always move the same way (manual or auto mode).
    * Receives Vehicle 1's sensor data and shows it on the OLED.
    * Sounds its buzzer when Vehicle 1 reports a notice / warning /
      emergency (obstacle, sudden braking, rain).
    * Sends ACKs back so Vehicle 1's web page shows Vehicle 2's link.
    * Stops its motors if no command arrives for COMMAND_TIMEOUT_MS.
    * Has a small fallback status page at its own IP address.

  LIBRARIES: "Adafruit SSD1306", "Adafruit GFX Library"

  WIRING (Vehicle 2)
    OLED  SDA -> GPIO 21, SCL -> GPIO 22 (VCC 3.3V, GND)
    Buzzer (active) + -> GPIO 5, - -> GND
    L298N: ENA GPIO14, IN1 GPIO13, IN2 GPIO18,
           ENB GPIO19, IN3 GPIO23, IN4 GPIO32
           OUT1/OUT2 = both LEFT motors in parallel
           OUT3/OUT4 = both RIGHT motors in parallel

  SETUP ORDER
    1. Same HOTSPOT_SSID / HOTSPOT_PASSWORD as Vehicle 1.
    2. Phone hotspot ON (2.4 GHz). Flash THIS sketch first.
    3. Note the MAC printed in Serial Monitor -> Vehicle 1's RECEIVER_MAC[].
    4. Flash Vehicle 1, note its MAC -> TRANSMITTER_MAC[] below, re-flash this.
  =====================================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// ======================= USER CONFIGURATION =========================
const char *HOTSPOT_SSID     = "v2v";
const char *HOTSPOT_PASSWORD = "1234567890";
const char *MDNS_HOSTNAME    = "vehicle2";

uint8_t TRANSMITTER_MAC[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };  // <-- Vehicle 1's MAC

#define WIFI_CONNECT_TIMEOUT_MS  20000

#define I2C_SDA 21
#define I2C_SCL 22
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_ADDR      0x3C

#define BUZZER_PIN     5

#define LINK_TIMEOUT_MS      5000   // no sensor packet -> "NO SIGNAL"
#define COMMAND_TIMEOUT_MS   600    // no drive command -> motors stop

#define ENA_PIN   14
#define IN1_PIN   13
#define IN2_PIN   18
#define ENB_PIN   19
#define IN3_PIN   23
#define IN4_PIN   32
#define PWM_FREQ       5000
#define PWM_RES_BITS   8

// ======================= OBJECTS ====================================
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
WebServer server(80);
bool oledOK = false;
String ipStr = "";

// ======================= SHARED TYPES (identical to Vehicle 1) ======
enum DriveCmd : uint8_t { CMD_STOP = 0, CMD_FORWARD = 1, CMD_BACKWARD = 2, CMD_LEFT = 3, CMD_RIGHT = 4 };

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

typedef struct __attribute__((packed)) v2v_ack_t {
  uint8_t  ackFromVehicle;   // 2
  uint32_t ackedPacketId;
  uint32_t peerPacketsOK;
  uint32_t peerPacketsRejected;
  uint32_t ctrlReceived;
  uint8_t  v2DriveState;
} v2v_ack_t;

typedef struct __attribute__((packed)) v2v_ctrl_t {
  uint8_t  ctrlMagic;        // 0xC2
  uint8_t  dir;
  uint8_t  speed;
  uint8_t  mode;
  uint8_t  alertLevel;
  uint32_t cmdId;
} v2v_ctrl_t;
#define CTRL_MAGIC 0xC2

// ======================= STATE ======================================
v2v_packet_t v1Data;
volatile unsigned long lastReceivedAt = 0;
volatile bool everReceived = false;
volatile uint32_t totalPacketsOK = 0, totalPacketsRejected = 0, ctrlReceived = 0;

volatile DriveCmd currentCmd = CMD_STOP;
volatile DriveCmd actualCmd  = CMD_STOP;
volatile int requestedSpeed  = 180;
volatile uint8_t remoteMode  = 0;
volatile uint8_t remoteAlert = 0;
volatile unsigned long lastCtrlTime = 0;
volatile bool everCtrl = false;

volatile bool ackPending = false;
volatile uint32_t pendingAckId = 0;

const char* cmdName(DriveCmd c) {
  switch (c) {
    case CMD_FORWARD:  return "FORWARD";
    case CMD_BACKWARD: return "BACKWARD";
    case CMD_LEFT:     return "LEFT";
    case CMD_RIGHT:    return "RIGHT";
    default:           return "STOPPED";
  }
}

// ======================= MOTORS =====================================
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
bool ctrlTimedOut() { return (millis() - lastCtrlTime > COMMAND_TIMEOUT_MS); }

void updateDrive() {
  DriveCmd wanted = ctrlTimedOut() ? CMD_STOP : currentCmd;
  actualCmd = wanted;
  applyMotorCommand(wanted, requestedSpeed);
}

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

// ======================= ESP-NOW ====================================
void sendAckToVehicle1(uint32_t ackedId) {
  v2v_ack_t ack;
  ack.ackFromVehicle      = 2;
  ack.ackedPacketId       = ackedId;
  ack.peerPacketsOK       = totalPacketsOK;
  ack.peerPacketsRejected = totalPacketsRejected;
  ack.ctrlReceived        = ctrlReceived;
  ack.v2DriveState        = (uint8_t)actualCmd;
  esp_now_send(TRANSMITTER_MAC, (uint8_t *)&ack, sizeof(ack));
}

void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(v2v_packet_t)) {
    v2v_packet_t in;
    memcpy(&in, incomingData, sizeof(in));
    if (in.vehicleId != 1 || in.temperature < -40 || in.temperature > 85 ||
        in.humidity < 0 || in.humidity > 100) { totalPacketsRejected++; return; }
    v1Data = in;
    lastReceivedAt = millis();
    everReceived = true;
    totalPacketsOK++;
    pendingAckId = in.packetId;
    ackPending = true;                  // ACK is sent from loop(), not from the callback

  } else if (len == sizeof(v2v_ctrl_t)) {
    v2v_ctrl_t c;
    memcpy(&c, incomingData, sizeof(c));
    if (c.ctrlMagic != CTRL_MAGIC || c.dir > CMD_RIGHT) { totalPacketsRejected++; return; }
    currentCmd     = (DriveCmd)c.dir;
    requestedSpeed = constrain((int)c.speed, 0, 255);
    remoteMode     = c.mode;
    remoteAlert    = c.alertLevel;
    lastCtrlTime   = millis();
    everCtrl       = true;
    ctrlReceived++;

  } else {
    totalPacketsRejected++;
  }
}

bool linkIsUp() { return everReceived && (millis() - lastReceivedAt < LINK_TIMEOUT_MS); }

// ======================= FALLBACK STATUS PAGE =======================
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Vehicle 2 status</title>
<style>
:root{--ink:#142030;--mute:#64748b;--line:#d5dde6;--ok:#15803d;--bad:#c81e1e;--v2:#4338ca;}
*{box-sizing:border-box;} body{margin:0;font-family:"Segoe UI",system-ui,Arial,sans-serif;background:#e9edf1;color:var(--ink);}
header{background:#142030;color:#fff;padding:14px 18px;} header h1{margin:0;font-size:18px;} header p{margin:3px 0 0;font-size:12.5px;color:#9fb0c6;}
.wrap{max-width:640px;margin:0 auto;padding:14px;}
.banner{padding:12px 16px;border-radius:10px;font-weight:650;font-size:14px;background:#dcfce7;color:#14532d;border:1px solid #86efac;margin-bottom:14px;}
.banner.bad{background:#fee2e2;color:#7f1d1d;border-color:#fca5a5;} .banner.warn{background:#fef3c7;color:#78350f;border-color:#fcd34d;}
.card{background:#fff;border:1px solid var(--line);border-radius:12px;padding:16px;margin-bottom:14px;}
.card h2{margin:0 0 12px;font-size:15px;}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(135px,1fr));gap:10px;}
.cell{border:1px solid var(--line);border-radius:10px;padding:10px 12px;background:#f3f6f9;}
.cell small{display:block;color:var(--mute);font-size:12px;} .cell b{display:block;font-size:18px;margin-top:3px;}
.cell.alert{border-color:var(--bad);background:#fef2f2;} .cell.alert b{color:var(--bad);}
.foot{text-align:center;color:var(--mute);font-size:12px;}
</style></head><body>
<header><h1>Vehicle 2 status</h1><p>Fallback view. Drive both vehicles from the Vehicle 1 page.</p></header>
<div class="wrap">
<div class="banner" id="banner">Waiting for Vehicle 1...</div>
<div class="card"><h2>This vehicle</h2><div class="grid">
  <div class="cell"><small>Motors</small><b id="vMotor" style="color:var(--v2)">--</b></div>
  <div class="cell"><small>Mode (from V1)</small><b id="vMode">--</b></div>
  <div class="cell"><small>Buzzer</small><b id="vBuzz">--</b></div>
  <div class="cell"><small>Accepted / rejected</small><b id="vOk">--</b></div>
  <div class="cell"><small>Commands received</small><b id="vCtl">--</b></div>
</div></div>
<div class="card"><h2>Received from Vehicle 1</h2><div class="grid">
  <div class="cell"><small>Temperature</small><b id="vTemp">--</b></div>
  <div class="cell" id="cRain"><small>Rain</small><b id="vRain">--</b></div>
  <div class="cell" id="cFront"><small>Front distance</small><b id="vFront">--</b></div>
  <div class="cell" id="cRear"><small>Rear distance</small><b id="vRear">--</b></div>
  <div class="cell" id="cBrake"><small>Braking</small><b id="vBrake">--</b></div>
</div></div>
<div class="foot" id="foot"></div>
</div>
<script>
const $=id=>document.getElementById(id);
const names=['STOPPED','FORWARD','BACKWARD','LEFT','RIGHT'], buzz=['Silent','Notice','Warning','Emergency'];
function al(id,on){$(id).classList.toggle('alert',!!on);}
async function poll(){
  try{
    const d=await (await fetch('/data',{cache:'no-store'})).json();
    const up=d.connected&&d.age<5000;
    $('vMotor').innerText=names[d.myState]||'--'; $('vMode').innerText=d.mode?'Auto':'Manual';
    $('vBuzz').innerText=buzz[d.alert]||'--'; $('vOk').innerText=d.packetsOK+' / '+d.packetsRejected; $('vCtl').innerText=d.ctrlRx;
    $('vTemp').innerText=d.temperature.toFixed(1)+' \u00B0C';
    $('vRain').innerText=d.rain?('Raining ('+d.rainPct+'%)'):('Dry ('+d.rainPct+'%)');
    $('vFront').innerText=d.front>=9999?'No echo':d.front+' mm'; $('vRear').innerText=d.rear>=9999?'No echo':d.rear+' mm';
    $('vBrake').innerText=d.brake?'Detected':'Normal';
    al('cRain',d.rain); al('cFront',d.obstacleFront); al('cRear',d.obstacleRear); al('cBrake',d.brake);
    const b=$('banner'); b.className='banner';
    if(!up){b.innerText='No signal from Vehicle 1';b.classList.add('bad');}
    else if(d.emergency){b.innerText='Emergency reported by Vehicle 1';b.classList.add('bad');}
    else if(d.brake||d.rain){b.innerText=d.rain?'Rain reported by Vehicle 1':'Vehicle 1 is braking';b.classList.add('warn');}
    else b.innerText='Linked. All systems normal.';
  }catch(e){const b=$('banner');b.className='banner bad';b.innerText='Status page offline';}
  setTimeout(poll,700);
}
poll();
</script></body></html>
)HTMLPAGE";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleData() {
  unsigned long age = everReceived ? (millis() - lastReceivedAt) : 0;
  String j = "{";
  j += "\"connected\":" + String(everReceived ? "true" : "false") + ",";
  j += "\"age\":" + String(age) + ",";
  j += "\"temperature\":" + String(v1Data.temperature, 1) + ",";
  j += "\"rain\":" + String(v1Data.rainDetected ? "true" : "false") + ",";
  j += "\"rainPct\":" + String(v1Data.rainPercent) + ",";
  j += "\"front\":" + String(v1Data.distanceFrontMM) + ",";
  j += "\"rear\":" + String(v1Data.distanceRearMM) + ",";
  j += "\"brake\":" + String(v1Data.brakeAlert ? "true" : "false") + ",";
  j += "\"obstacleFront\":" + String(v1Data.obstacleFrontAlert ? "true" : "false") + ",";
  j += "\"obstacleRear\":" + String(v1Data.obstacleRearAlert ? "true" : "false") + ",";
  j += "\"emergency\":" + String(v1Data.emergencyAlert ? "true" : "false") + ",";
  j += "\"mode\":" + String((int)remoteMode) + ",";
  j += "\"alert\":" + String((int)(ctrlTimedOut() ? 0 : remoteAlert)) + ",";
  j += "\"packetsOK\":" + String(totalPacketsOK) + ",";
  j += "\"packetsRejected\":" + String(totalPacketsRejected) + ",";
  j += "\"ctrlRx\":" + String(ctrlReceived) + ",";
  j += "\"myState\":" + String((int)actualCmd);
  j += "}";
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", j);
}

// ======================= OLED =======================================
void updateOLED() {
  if (!oledOK) return;
  static unsigned long lastUpdate = 0;
  static uint8_t page = 0;
  if (millis() - lastUpdate < 1000) return;
  lastUpdate = millis();
  page = (page + 1) % 3;

  oled.clearDisplay();
  oled.setCursor(0, 0);

  if (page == 2) {
    oled.println("VEHICLE 2");
    oled.println("Status page:");
    oled.println(ipStr);
    oled.print("Cmds rx: "); oled.println(ctrlReceived);
    oled.display();
    return;
  }

  if (!linkIsUp() || ctrlTimedOut()) {
    oled.println("V1 LINK");
    oled.println(everCtrl ? "NO COMMANDS" : "WAITING...");
    oled.print("Motors: "); oled.println(cmdName(actualCmd));
    oled.display();
    return;
  }

  if (page == 0) {
    oled.print("V1 T:"); oled.print(v1Data.temperature, 1); oled.println("C");
    oled.print("Fr:"); oled.print(v1Data.distanceFrontMM < 9999 ? String(v1Data.distanceFrontMM) : "--"); oled.println("mm");
    oled.print("Rr:"); oled.print(v1Data.distanceRearMM  < 9999 ? String(v1Data.distanceRearMM)  : "--"); oled.println("mm");
    oled.print("Rain:"); oled.print(v1Data.rainPercent); oled.println(v1Data.rainDetected ? "% WET" : "% dry");
  } else {
    oled.println("VEHICLE 2 DRIVE");
    oled.print("Mode: "); oled.println(remoteMode ? "AUTO" : "MANUAL");
    oled.print("Me: "); oled.println(cmdName(actualCmd));
    if (v1Data.emergencyAlert)  oled.println("V1: EMERGENCY");
    else if (v1Data.brakeAlert) oled.println("V1: BRAKING");
    else                        oled.println("V1: OK");
  }
  oled.display();
}

// ======================= SETUP ======================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== VEHICLE 2 FOLLOWER BOOTING ===");

  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOK) {
    oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0); oled.println("Vehicle 2 Boot"); oled.println("Joining hotspot..."); oled.display();
  } else Serial.println("OLED: NOT FOUND");

  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);
  ledcAttach(ENA_PIN, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(ENB_PIN, PWM_FREQ, PWM_RES_BITS);
  motorsStop();

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
  Serial.print("Connected! IP: "); Serial.println(ipStr);
  Serial.print("Vehicle 2 MAC (put in Vehicle 1's RECEIVER_MAC[]): "); Serial.println(WiFi.macAddress());

  if (MDNS.begin(MDNS_HOSTNAME)) MDNS.addService("http", "tcp", 80);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    if (oledOK) { oled.clearDisplay(); oled.setCursor(0,0); oled.println("ESP-NOW FAIL"); oled.display(); }
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, TRANSMITTER_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) Serial.println("Failed to add Vehicle 1 as peer - check TRANSMITTER_MAC[]");

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  lastCtrlTime = millis();
  if (oledOK) { oled.clearDisplay(); oled.setCursor(0,0); oled.println("Status page:"); oled.println(ipStr); oled.display(); }
  delay(2000);
  Serial.println("=== VEHICLE 2 READY ===");
}

// ======================= LOOP =======================================
void loop() {
  server.handleClient();

  if (ackPending) {
    ackPending = false;
    sendAckToVehicle1(pendingAckId);
  }

  updateDrive();
  driveBuzzer(ctrlTimedOut() ? 0 : remoteAlert);
  updateOLED();
}
