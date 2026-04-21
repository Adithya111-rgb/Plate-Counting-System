#include <AccelStepper.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* ---------------- Pins ---------------- */
#define DIR_PIN 12
#define STEP_PIN 14
#define SENSOR_BOTTOM 34
#define SENSOR_TOP 32
#define SHARP_PIN 35
#define RELAY_CLEAR 25

/* ---------------- Speeds ---------------- */
#define SPEED_UP    3000
#define SPEED_DOWN -4000  

/* ---------------- Thresholds ---------------- */
#define SHARP_DETECT 1200 
#define SHARP_CLEAR   700 

uint8_t MASTER_MAC[] = {0xC0, 0xCD, 0xD6, 0x85, 0xE9, 0x64};

enum MsgCode {
  MSG_READY       = 10,
  CMD_STACK_CLEAR = 25
};

typedef struct {
  int code;
} debug_msg;

debug_msg incoming, outgoing;

LiquidCrystal_I2C lcd(0x26, 16, 2);
AccelStepper stepper(1, STEP_PIN, DIR_PIN);

enum SState {
  S_HOME,
  S_WAIT,
  S_FULL_STACK_CLEAR,
  S_RELAY_WAIT,
  S_RETURN_TOP
};

volatile SState state = S_HOME;
unsigned long relayTimer = 0;
bool readySent = false;

// Sharp Sensor Variables
bool plate = false;
bool lastPlate = false;
bool moving = false;

int readSharp() {
  long s = 0;
  for (int i = 0; i < 5; i++) s += analogRead(SHARP_PIN);
  return s / 5;
}

void sendReady() {
  outgoing.code = MSG_READY;
  esp_now_send(MASTER_MAC, (uint8_t*)&outgoing, sizeof(outgoing));
}

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&incoming, data, sizeof(incoming));
  if (memcmp(info->src_addr, MASTER_MAC, 6) != 0) return;

  if (incoming.code == CMD_STACK_CLEAR) {
    state = S_FULL_STACK_CLEAR;
    readySent = false;
  }
}

const char* getStateName(SState s) {
  switch (s) {
    case S_HOME:             return "GO TOP         ";
    case S_WAIT:             return "WAITING PLATE  ";
    case S_FULL_STACK_CLEAR: return "FULL CLEAR     ";
    case S_RELAY_WAIT:       return "PLATE EJECTION "; // Updated Name
    case S_RETURN_TOP:       return "RETURN TOP     ";
    default:                 return "UNKNOWN        ";
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_BOTTOM, INPUT);
  pinMode(SENSOR_TOP, INPUT_PULLUP);
  pinMode(SHARP_PIN, INPUT);
  pinMode(RELAY_CLEAR, OUTPUT);
  digitalWrite(RELAY_CLEAR, HIGH);

  Wire.begin(21, 22);
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear(); // Clear display at start

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, MASTER_MAC, 6);
  esp_now_add_peer(&peerInfo);

  stepper.setMaxSpeed(5000);
  Serial.println("STACKER READY");
}

void loop() {
  stepper.runSpeed();

  int sharpVal = readSharp();
  bool topHit = (digitalRead(SENSOR_TOP) == LOW);
  bool bottomHit = (digitalRead(SENSOR_BOTTOM) == LOW);

  lastPlate = plate;
  if (sharpVal > SHARP_DETECT) plate = true;
  else if (sharpVal < SHARP_CLEAR) plate = false;

  switch (state) {
    case S_HOME:
      stepper.setSpeed(SPEED_UP);
      if (topHit) {
        stepper.setSpeed(0);
        state = S_WAIT;
        readySent = false;
      }
      break;

    case S_WAIT:
      if (!readySent) {
        sendReady();
        readySent = true;
      }

      if (plate && !lastPlate && !moving) {
        stepper.setSpeed(SPEED_DOWN);
        moving = true;
      }
      if (!plate && lastPlate && moving) {
        stepper.setSpeed(0);
        moving = false;
      }

      if (bottomHit) {
        stepper.setSpeed(0);
        moving = false;
      }
      break;

    case S_FULL_STACK_CLEAR:
      stepper.setSpeed(SPEED_DOWN);
      if (bottomHit) {
        stepper.setSpeed(0);
        digitalWrite(RELAY_CLEAR, LOW);
        relayTimer = millis();
        state = S_RELAY_WAIT;
      }
      break;

    case S_RELAY_WAIT:
      if (millis() - relayTimer >= 5000) {
        digitalWrite(RELAY_CLEAR, HIGH);
        state = S_RETURN_TOP;
      }
      break;

    case S_RETURN_TOP:
      stepper.setSpeed(SPEED_UP);
      if (topHit) {
        stepper.setSpeed(0);
        state = S_WAIT;
        readySent = false;
        moving = false; 
      }
      break;
  }

  // Updated LCD Logic: Only printing Line 1
  static unsigned long lastLCD = 0;
  if (millis() - lastLCD > 300) {
    lcd.setCursor(0, 0);
    lcd.print(getStateName(state));
    // Line 2 is ignored/left blank
    lastLCD = millis();
  }
}
