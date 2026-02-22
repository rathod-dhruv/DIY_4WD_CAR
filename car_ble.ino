#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------- Pins (ACEBOTT Car Shield v1.1) ----------------
#define SHCP_PIN   18
#define EN_PIN     16
#define DATA_PIN    5
#define STCP_PIN   17
#define PWM1_PIN   19

#define TRIG_PIN   13
#define ECHO_PIN   14

#define SERVO_PIN  25
#define BUZZER_PIN 33

#define LED1_PIN   12
#define LED2_PIN    2

#define IR_PIN      4

#define TRACE_L    39
#define TRACE_M    36
#define TRACE_R    35

// ---------------- Motor bits (your tested mapping) ----------------
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
#define DIR_STRAFE_LEFT  (FL_BWD | FR_FWD | RL_FWD | RR_BWD) // mecanum
#define DIR_STRAFE_RIGHT (FL_FWD | FR_BWD | RL_BWD | RR_FWD) // mecanum
#define DIR_STOP         0b00000000

// ---------------- BLE ----------------
#define BLE_NAME            "ACEBOTT_CAR_BT"
#define SERVICE_UUID        "0000FFE0-0000-1000-8000-00805F9B34FB"
#define CMD_CHAR_UUID       "0000FFE1-0000-1000-8000-00805F9B34FB"  // RX
#define DATA_CHAR_UUID      "0000FFE2-0000-1000-8000-00805F9B34FB"  // TX notify

BLECharacteristic* pDataChar = nullptr;

// ---------------- Config ----------------
int SPEED = 180;

// obstacle thresholds (cm)
const int DANGER_CM = 15;
const int WARN_CM   = 30;

// sensor update
unsigned long lastSensorMs = 0;
const unsigned long SENSOR_INTERVAL_MS = 100;

// serial logs
unsigned long lastSerialMs = 0;
const unsigned long SERIAL_INTERVAL_MS = 250;
String lastZone = "";

// ---------------- PWM / Servo (LEDC v3) ----------------
static inline uint32_t servoDutyFromAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int pulseUs = map(angle, 0, 180, 1000, 2000);     // 1ms..2ms
  // 50Hz => 20,000us period. 16-bit => 65535 max duty
  return (uint32_t)((pulseUs / 20000.0) * 65535.0);
}

static inline void setServoAngle(int angle) {
  ledcWrite(SERVO_PIN, servoDutyFromAngle(angle));  // v3: write by PIN
}

// ---------------- Motor helpers ----------------
void Move(uint8_t direction, uint8_t speed) {
  digitalWrite(EN_PIN, LOW);         // enable motor driver (active low)
  ledcWrite(PWM1_PIN, speed);        // speed PWM

  digitalWrite(STCP_PIN, LOW);
  shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, direction);
  digitalWrite(STCP_PIN, HIGH);
}

void stopCar()      { Move(DIR_STOP, 0); }
void forward()      { Move(DIR_FORWARD, SPEED); }
void backward()     { Move(DIR_BACKWARD, SPEED); }
void spinLeft()     { Move(DIR_SPIN_LEFT, SPEED); }
void spinRight()    { Move(DIR_SPIN_RIGHT, SPEED); }
void strafeLeft()   { Move(DIR_STRAFE_LEFT, SPEED); }
void strafeRight()  { Move(DIR_STRAFE_RIGHT, SPEED); }

// ---------------- Sensors ----------------
long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  if (dur == 0) return 999;
  return dur / 58;
}

void readTrace(int &l, int &m, int &r) {
  l = digitalRead(TRACE_L);
  m = digitalRead(TRACE_M);
  r = digitalRead(TRACE_R);
}

String zoneFromDist(long d) {
  if (d >= 999) return "NO_ECHO";
  if (d < DANGER_CM) return "DANGER";
  if (d < WARN_CM) return "WARN";
  return "SAFE";
}

int obstaclePresentFromZone(const String& z) {
  return (z == "WARN" || z == "DANGER") ? 1 : 0;
}

// ---------------- Actuators ----------------
void ledsSet(bool on) {
  digitalWrite(LED1_PIN, on ? HIGH : LOW);
  digitalWrite(LED2_PIN, on ? HIGH : LOW);
}

void honkShort() {
  tone(BUZZER_PIN, 1200, 120);
}

// ---------------- BLE send JSON ----------------
void sendSensorDataBLE(long dist, int l, int m, int r, int ir, const String& zone, int obs) {
  if (!pDataChar) return;

  // include zone + obs so HTML can display instantly
  char json[128];
  snprintf(json, sizeof(json),
           "{\"d\":%ld,\"l\":%d,\"m\":%d,\"r\":%d,\"ir\":%d,\"obs\":%d,\"zone\":\"%s\"}",
           dist, l, m, r, ir, obs, zone.c_str());

  pDataChar->setValue((uint8_t*)json, strlen(json));
  pDataChar->notify();
}

// ---------------- Command handling ----------------
void handleCommand(const char* cmd) {
  if (!cmd || !cmd[0]) return;

  // 2-letter commands
  if (strcmp(cmd, "SL") == 0) { strafeLeft();  return; }
  if (strcmp(cmd, "SR") == 0) { strafeRight(); return; }

  // one-letter movement
  char c = cmd[0];
  if (c == 'F' || c == 'f') { forward();  return; }
  if (c == 'B' || c == 'b') { backward(); return; }
  if (c == 'L' || c == 'l') { spinLeft(); return; }
  if (c == 'R' || c == 'r') { spinRight();return; }
  if (c == 'S' || c == 's') { stopCar();  return; }

  // speed V###
  if (c == 'V' || c == 'v') {
    SPEED = constrain(atoi(cmd + 1), 0, 255);
    return;
  }

  // LED D0/D1
  if (c == 'D' || c == 'd') {
    int v = atoi(cmd + 1);
    ledsSet(v != 0);
    return;
  }

  // honk H1 (we just honk short; ignore off)
  if (c == 'H' || c == 'h') {
    int v = atoi(cmd + 1);
    if (v != 0) honkShort();
    return;
  }

  // servo P###
  if (c == 'P' || c == 'p') {
    setServoAngle(atoi(cmd + 1));
    return;
  }

  // raw motor debug X###
  if (c == 'X' || c == 'x') {
    int pattern = constrain(atoi(cmd + 1), 0, 255);
    Move((uint8_t)pattern, SPEED);
    return;
  }
}

class CmdCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String v = pChar->getValue();
    v.trim();
    if (v.length() == 0) return;
    handleCommand(v.c_str());
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("=== ACEBOTT BLE CAR (Obstacle + Trace + IR logs) ===");

  pinMode(SHCP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(STCP_PIN, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(IR_PIN, INPUT);
  pinMode(TRACE_L, INPUT);
  pinMode(TRACE_M, INPUT);
  pinMode(TRACE_R, INPUT);

  // PWM for motor speed
  ledcAttach(PWM1_PIN, 1000, 8);
  ledcWrite(PWM1_PIN, 0);

  // Servo PWM (50Hz, 16-bit)
  ledcAttach(SERVO_PIN, 50, 16);
  setServoAngle(90);

  ledsSet(false);
  stopCar();

  // BLE init
  BLEDevice::init(BLE_NAME);
  BLEServer* server = BLEDevice::createServer();
  BLEService* svc = server->createService(SERVICE_UUID);
  
  BLECharacteristic* cmdChar = svc->createCharacteristic(
    CMD_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  cmdChar->setCallbacks(new CmdCallback());

  pDataChar = svc->createCharacteristic(
    DATA_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pDataChar->addDescriptor(new BLE2902()); // required for notifications

  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising: ACEBOTT_CAR_BT");
}

void loop() {
  const unsigned long now = millis();

  // read + notify BLE
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;

    long dist = readDistanceCM();
    int l,m,r;
    readTrace(l,m,r);
    int ir = digitalRead(IR_PIN);

    String zone = zoneFromDist(dist);
    int obs = obstaclePresentFromZone(zone);

    sendSensorDataBLE(dist, l, m, r, ir, zone, obs);
  }

  // serial debug logs
  if (now - lastSerialMs >= SERIAL_INTERVAL_MS) {
    lastSerialMs = now;

    long dist = readDistanceCM();
    int l,m,r;
    readTrace(l,m,r);
    int ir = digitalRead(IR_PIN);

    String zone = zoneFromDist(dist);
    int obs = obstaclePresentFromZone(zone);

    Serial.print("dist=");
    Serial.print(dist);
    Serial.print("cm  zone=");
    Serial.print(zone);
    Serial.print("  obs=");
    Serial.print(obs);
    Serial.print("  trace=");
    Serial.print(l); Serial.print(m); Serial.print(r);
    Serial.print("  irRaw=");
    Serial.println(ir);

    // only print on zone changes too (nice “event” log)
    if (zone != lastZone) {
      lastZone = zone;
      Serial.print("EVENT: OBSTACLE_ZONE_CHANGED -> ");
      Serial.println(zone);
    }
  }

  delay(5);
}