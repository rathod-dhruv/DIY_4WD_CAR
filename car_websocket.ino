/*
 * ============================================================
 *  ACEBOTT Car — WebSocket Server (for Unity / Quest / Browser)
 *  ESP32 Core v3.x  |  WiFi + WebSocket (no BLE needed)
 * ============================================================
 *
 *  HOW THIS WORKS:
 *  ─────────────────────────────────────────────────────────
 *  Instead of BLE, the ESP32 runs a WiFi WebSocket SERVER.
 *  Unity (on Quest) connects as a CLIENT over your local WiFi.
 *
 *  Flow:
 *    Quest (Unity)  ←──WebSocket──→  ESP32 (this sketch)
 *         │   sends: "F\n","SL\n"          │  receives: command
 *         │   receives: JSON sensor data   │  sends: {"d":23,...}
 *
 *  WHY WEBSOCKET?
 *  - Two-way communication (unlike HTTP which is one-way)
 *  - Very low latency (~1-5ms on local WiFi vs ~10-30ms BLE)
 *  - Works natively in Unity with NativeWebSocket package
 *  - No pairing needed — just connect to IP address
 *
 *  SETUP:
 *  1. Set WIFI_SSID and WIFI_PASS below
 *  2. Upload to ESP32
 *  3. Open Serial Monitor → note the IP address printed
 *  4. Put that IP in your Unity script
 *
 *  LIBRARY NEEDED:
 *  Install "WebSockets" by Markus Sattler via Library Manager
 *  (Search: "WebSockets" → by Markus Sattler → Install)
 * ============================================================
 */

#include <WiFi.h>
#include <WebSocketsServer.h>  // by Markus Sattler — install from Library Manager

// ════════════════════════════════════════════════════════════════
//  ⚙️  CONFIGURE THESE — your WiFi credentials
// ════════════════════════════════════════════════════════════════
const char* WIFI_SSID = "Home1";   // ← change this
const char* WIFI_PASS = "Welcome1";   // ← change this

// WebSocket port — Unity will connect to ws://[ESP32_IP]:81
// Port 81 is a common choice (port 80 is reserved for HTTP)
const uint16_t WS_PORT = 81;

// ════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS  (ACEBOTT Car-Shield v1.1)
// ════════════════════════════════════════════════════════════════
#define SHCP_PIN   18
#define EN_PIN     16
#define DATA_PIN    5
#define STCP_PIN   17
#define PWM1_PIN   19

#define TRIG_PIN   13   // Ultrasonic trigger
#define ECHO_PIN   14   // Ultrasonic echo

#define SERVO_PIN  25
#define BUZZER_PIN 33
#define LED1_PIN   12
#define LED2_PIN    2
#define IR_PIN      4
#define TRACE_L    39
#define TRACE_M    36
#define TRACE_R    35
#define TEMP_PIN   27

// ════════════════════════════════════════════════════════════════
//  MOTOR BIT PATTERNS (your tested mapping)
// ════════════════════════════════════════════════════════════════
#define FL_FWD  0b00000001
#define FL_BWD  0b00001000
#define FR_FWD  0b00000010
#define FR_BWD  0b00000100
#define RL_FWD  0b00010000
#define RL_BWD  0b00100000
#define RR_FWD  0b01000000
#define RR_BWD  0b10000000

#define DIR_FORWARD      (FL_FWD | FR_FWD | RL_FWD | RR_FWD)
#define DIR_BACKWARD     (FL_BWD | FR_BWD | RL_BWD | RR_BWD)
#define DIR_SPIN_LEFT    (FL_BWD | FR_FWD | RL_BWD | RR_FWD)
#define DIR_SPIN_RIGHT   (FL_FWD | FR_BWD | RL_FWD | RR_BWD)
#define DIR_STRAFE_LEFT  (FL_BWD | FR_FWD | RL_FWD | RR_BWD)
#define DIR_STRAFE_RIGHT (FL_FWD | FR_BWD | RL_BWD | RR_FWD)
#define DIR_STOP         0b00000000

// ════════════════════════════════════════════════════════════════
//  GLOBAL STATE
// ════════════════════════════════════════════════════════════════
// WebSocketsServer listens for incoming connections on WS_PORT
// WEBSOCKETS_SERVER_CLIENT_MAX defaults to 5 (up to 5 clients)
WebSocketsServer wsServer(WS_PORT);

int SPEED = 180;

// Sensor timing — send data every 100ms
unsigned long lastSensorMs = 0;
const unsigned long SENSOR_INTERVAL_MS = 100;

// Track connected clients for broadcasting
// WebSocket client IDs are 0..WEBSOCKETS_SERVER_CLIENT_MAX-1
bool clientConnected = false;

// ════════════════════════════════════════════════════════════════
//  MOTOR FUNCTIONS (identical to BLE version)
// ════════════════════════════════════════════════════════════════
void Move(uint8_t direction, uint8_t speed) {
  digitalWrite(EN_PIN, LOW);
  ledcWrite(PWM1_PIN, speed);
  digitalWrite(STCP_PIN, LOW);
  shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, direction);
  digitalWrite(STCP_PIN, HIGH);
}
void stopCar()     { Move(DIR_STOP, 0); }
void forward()     { Move(DIR_FORWARD, SPEED); }
void backward()    { Move(DIR_BACKWARD, SPEED); }
void spinLeft()    { Move(DIR_SPIN_LEFT, SPEED); }
void spinRight()   { Move(DIR_SPIN_RIGHT, SPEED); }
void strafeLeft()  { Move(DIR_STRAFE_LEFT, SPEED); }
void strafeRight() { Move(DIR_STRAFE_RIGHT, SPEED); }

// ════════════════════════════════════════════════════════════════
//  SERVO
// ════════════════════════════════════════════════════════════════
void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int pulseUs = map(angle, 0, 180, 1000, 2000);
  uint32_t duty = (uint32_t)((pulseUs / 20000.0) * 65535.0);
  ledcWrite(SERVO_PIN, duty);
}

// ════════════════════════════════════════════════════════════════
//  SENSORS
// ════════════════════════════════════════════════════════════════
long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  return dur == 0 ? 999 : dur / 58;
}

float readTempC() {
  int raw = analogRead(TEMP_PIN);
  float v = (raw / 4095.0f) * 3.3f;
  return (v - 0.5f) * 100.0f;  // TMP36 formula
}

String zoneFromDist(long d) {
  if (d >= 999) return "NO_ECHO";
  if (d < 15)   return "DANGER";
  if (d < 30)   return "WARN";
  return "SAFE";
}

// ════════════════════════════════════════════════════════════════
//  ACCESSORIES
// ════════════════════════════════════════════════════════════════
void ledsSet(bool on) {
  digitalWrite(LED1_PIN, on ? HIGH : LOW);
  digitalWrite(LED2_PIN, on ? HIGH : LOW);
}
void honkShort() { tone(BUZZER_PIN, 1200, 120); }

// ════════════════════════════════════════════════════════════════
//  COMMAND HANDLER  (same commands as BLE version)
// ════════════════════════════════════════════════════════════════
// Unity sends exactly the same strings: "F", "B", "SL", "V180", etc.
// So your Unity command strings don't change at all!

void handleCommand(const char* cmd) {
  if (!cmd || !cmd[0]) return;

  // 2-char commands first
  if (strcmp(cmd, "SL") == 0) { strafeLeft();  return; }
  if (strcmp(cmd, "SR") == 0) { strafeRight(); return; }

  char c = cmd[0];
  if (c == 'F' || c == 'f') { forward();   return; }
  if (c == 'B' || c == 'b') { backward();  return; }
  if (c == 'L' || c == 'l') { spinLeft();  return; }
  if (c == 'R' || c == 'r') { spinRight(); return; }
  if (c == 'S' || c == 's') { stopCar();   return; }
  if (c == 'V' || c == 'v') { SPEED = constrain(atoi(cmd+1), 0, 255); return; }
  if (c == 'D' || c == 'd') { ledsSet(atoi(cmd+1) != 0); return; }
  if (c == 'H' || c == 'h') { if (atoi(cmd+1)) honkShort(); return; }
  if (c == 'P' || c == 'p') { setServoAngle(atoi(cmd+1)); return; }
  if (c == 'X' || c == 'x') { Move((uint8_t)constrain(atoi(cmd+1),0,255), SPEED); return; }

  // NEW: joystick command from Unity — "J:x:y" where x,y are -100 to 100
  // e.g. "J:0:100" = full forward, "J:100:0" = strafe right
  // This is more precise than single-key commands for VR joystick input!
  if (c == 'J' || c == 'j') {
    // Parse "J:x:y" format
    // strtok splits string by delimiter ":"
    char buf[32];
    strncpy(buf, cmd, 31); buf[31] = 0;
    char* token = strtok(buf, ":");  // "J"
    token = strtok(NULL, ":");       // x value
    if (!token) return;
    int jx = atoi(token);
    token = strtok(NULL, ":");       // y value
    if (!token) return;
    int jy = atoi(token);

    // Dead zone: ignore small joystick movements (thumb resting)
    if (abs(jx) < 15 && abs(jy) < 15) { stopCar(); return; }

    // Determine dominant axis
    // If more sideways than forward/back = strafe (Mecanum!)
    if (abs(jx) > abs(jy) * 1.5) {
      // Strongly sideways = strafe
      SPEED = map(abs(jx), 15, 100, 80, 255);
      jx > 0 ? strafeRight() : strafeLeft();
    } else if (abs(jy) > abs(jx) * 1.5) {
      // Strongly forward/back
      SPEED = map(abs(jy), 15, 100, 80, 255);
      jy > 0 ? forward() : backward();
    } else {
      // Diagonal = spin turn while moving
      SPEED = map(max(abs(jx),abs(jy)), 15, 100, 80, 255);
      if (jy > 0 && jx > 0) strafeRight();
      else if (jy > 0 && jx < 0) strafeLeft();
      else if (jy < 0 && jx > 0) { SPEED = 150; spinRight(); }
      else { SPEED = 150; spinLeft(); }
    }
    return;
  }

  // NEW: head/look command from Quest head tracking — "HEAD:yaw:pitch"
  // yaw  = left/right head rotation → controls servo pan
  // pitch = up/down (we ignore for now, could tilt a camera later)
  // e.g. "HEAD:45:10" → servo goes to 45°
  if (strncmp(cmd, "HEAD:", 5) == 0) {
    char buf[32];
    strncpy(buf, cmd, 31); buf[31] = 0;
    char* token = strtok(buf, ":");  // "HEAD"
    token = strtok(NULL, ":");       // yaw (-90 to +90)
    if (!token) return;
    int yaw = atoi(token);
    // Map yaw (-90..+90) → servo angle (0..180), center=90
    int angle = map(constrain(yaw, -90, 90), -90, 90, 0, 180);
    setServoAngle(angle);
    return;
  }
}

// ════════════════════════════════════════════════════════════════
//  WEBSOCKET EVENT HANDLER
// ════════════════════════════════════════════════════════════════
// This function is called by the WebSocket library for every event
// num     = client ID (0, 1, 2... up to max clients)
// type    = what happened (connected, disconnected, message, etc)
// payload = the message bytes (if type == text message)
// length  = length of payload

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {

    case WStype_CONNECTED: {
      // A new client connected — get their IP for logging
      // server.remoteIP(num) returns the client's IP address
      IPAddress ip = wsServer.remoteIP(num);
      Serial.printf("[WS] Client #%d connected from %s\n",
                    num, ip.toString().c_str());
      clientConnected = true;

      // Send a welcome message — Unity can use this to confirm connection
      wsServer.sendTXT(num, "{\"event\":\"connected\",\"msg\":\"ACEBOTT_CAR_WS\"}");
      break;
    }

    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%d disconnected\n", num);
      // When last client disconnects, stop the car for safety!
      stopCar();
      break;

    case WStype_TEXT: {
      // Received a text message from Unity
      // payload is raw bytes — cast to char* to use as string
      char* cmd = (char*)payload;

      // Remove trailing newline if present
      if (length > 0 && (cmd[length-1] == '\n' || cmd[length-1] == '\r')) {
        cmd[length-1] = 0;
      }

      Serial.printf("[WS] Command: %s\n", cmd);
      handleCommand(cmd);
      break;
    }

    case WStype_ERROR:
      Serial.printf("[WS] Error on client #%d\n", num);
      break;

    default: break;
  }
}

// ════════════════════════════════════════════════════════════════
//  SEND SENSOR JSON TO ALL CONNECTED CLIENTS
// ════════════════════════════════════════════════════════════════
// broadcastTXT() sends to ALL connected clients simultaneously
// Unity receives this in its OnMessage callback

void sendSensorData() {
  long dist = readDistanceCM();
  int l = digitalRead(TRACE_L);
  int m = digitalRead(TRACE_M);
  int r = digitalRead(TRACE_R);
  int ir = digitalRead(IR_PIN);
  float tC = readTempC();
  String zone = zoneFromDist(dist);
  int obs = (zone == "WARN" || zone == "DANGER") ? 1 : 0;

  char json[160];
  snprintf(json, sizeof(json),
    "{\"d\":%ld,\"l\":%d,\"m\":%d,\"r\":%d,\"ir\":%d,\"t\":%.1f,\"obs\":%d,\"zone\":\"%s\"}",
    dist, l, m, r, ir, tC, obs, zone.c_str());

  // broadcastTXT sends to ALL connected WebSocket clients
  wsServer.broadcastTXT(json);
}

// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ACEBOTT Car WebSocket ===");

  // ── GPIO setup ──────────────────────────────────────────────
  pinMode(SHCP_PIN, OUTPUT); pinMode(EN_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT); pinMode(STCP_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(LED1_PIN, OUTPUT); pinMode(LED2_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(TRACE_L, INPUT); pinMode(TRACE_M, INPUT); pinMode(TRACE_R, INPUT);

  // ── PWM ──────────────────────────────────────────────────────
  ledcAttach(PWM1_PIN, 1000, 8); ledcWrite(PWM1_PIN, 0);
  ledcAttach(SERVO_PIN, 50, 16); setServoAngle(90);

  // ── ADC ──────────────────────────────────────────────────────
  analogReadResolution(12);
  analogSetPinAttenuation(TEMP_PIN, ADC_11db);

  ledsSet(false); stopCar();

  // ── WiFi ─────────────────────────────────────────────────────
  // WiFi.mode(WIFI_STA) = station mode (connect to existing router)
  // Alternative: WIFI_AP = access point (ESP32 becomes a hotspot)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected!");

  // Print IP — THIS IS WHAT YOU PUT IN UNITY!
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("WebSocket URL:    ws://%s:%d\n",
                WiFi.localIP().toString().c_str(), WS_PORT);

  // ── WebSocket server ─────────────────────────────────────────
  // begin() starts listening on WS_PORT
  wsServer.begin();
  // onEvent() registers our callback function
  wsServer.onEvent(onWebSocketEvent);

  Serial.println("WebSocket server started! Waiting for Unity...");
}

// ════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════
void loop() {
  // ── CRITICAL: must call loop() every iteration ───────────────
  // wsServer.loop() processes incoming WebSocket messages and
  // keeps connections alive (sends ping/pong frames).
  // If you block loop() with delay() for too long, clients disconnect!
  wsServer.loop();

  // ── Send sensor data every 100ms ────────────────────────────
  unsigned long now = millis();
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    sendSensorData();
  }

  // NO delay() here — wsServer.loop() needs to run as fast as possible
  // The sensor interval check handles the timing instead
}
