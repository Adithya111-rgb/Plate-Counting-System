#include <AccelStepper.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define S_BOTTOM 34
#define S_TOP 32
#define SHARP_PIN 35
#define BTN_START 5
#define DIR_PIN 12
#define STEP_PIN 14

uint8_t MASTER_MAC[] = {0xC0, 0xCD, 0xD6, 0x85, 0xE9, 0x64};

enum MsgCode {
  MSG_READY = 10,
  MSG_EMPTY = 404,
  MSG_GET_STATUS = 50
};

typedef struct { int code; } debug_msg;
debug_msg outgoing;

LiquidCrystal_I2C lcd(0x27, 16, 2);
AccelStepper stepper(1, STEP_PIN, DIR_PIN);

enum FState { F_GO_HOME_AT_BOOT, F_WAIT_START, F_MOVE_UP_TO_PLATE, F_READY_WITH_PLATE, F_EMPTY_REPORT, F_RETURN_DOWN_HOME };
volatile FState state = F_GO_HOME_AT_BOOT;
TaskHandle_t FeederTask;

unsigned long sharpTimer = 0;
bool sharpHighTiming = false, sharpLowTiming = false;
const int SHARP_READY_HIGH = 1200, SHARP_TAKEN_LOW = 700;
const unsigned long SHARP_STABLE_MS = 60;
bool readySent = false, emptySent = false;

void sendMsg(int code) {
  outgoing.code = code;
  esp_now_send(MASTER_MAC, (uint8_t*)&outgoing, sizeof(outgoing));
}

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  debug_msg in;
  memcpy(&in, data, sizeof(in));
  if (in.code == MSG_GET_STATUS) {
    if (state == F_READY_WITH_PLATE) sendMsg(MSG_READY);
    else if (state == F_EMPTY_REPORT) sendMsg(MSG_EMPTY);
  }
}

const char* getStateName(FState s) {
  switch (s) {
    case F_GO_HOME_AT_BOOT: return "BOOT->HOME     ";
    case F_WAIT_START:      return "PRESS START    ";
    case F_MOVE_UP_TO_PLATE:return "INDEXING UP    ";
    case F_READY_WITH_PLATE:return "READY/LOADED   ";
    case F_EMPTY_REPORT:    return "EMPTY REPORT   ";
    case F_RETURN_DOWN_HOME:return "RETURN HOME    ";
    default:                return "UNKNOWN        ";
  }
}

bool sharpHighStable(int value) {
  unsigned long now = millis();
  if (value > SHARP_READY_HIGH) {
    if (!sharpHighTiming) { sharpHighTiming = true; sharpTimer = now; }
    if (now - sharpTimer >= SHARP_STABLE_MS) return true;
  } else sharpHighTiming = false;
  return false;
}

bool sharpLowStable(int value) {
  unsigned long now = millis();
  if (value < SHARP_TAKEN_LOW) {
    if (!sharpLowTiming) { sharpLowTiming = true; sharpTimer = now; }
    if (now - sharpTimer >= SHARP_STABLE_MS) return true;
  } else sharpLowTiming = false;
  return false;
}

void resetSharpTimers() { sharpHighTiming = false; sharpLowTiming = false; }

void FeederCoreLogic(void * pvParameters) {
  for (;;) {
    stepper.runSpeed();
    int sharpVal = analogRead(SHARP_PIN);
    bool bottomHit = (digitalRead(S_BOTTOM) == LOW);
    bool topHit = (digitalRead(S_TOP) == LOW);
    bool startPressed = (digitalRead(BTN_START) == LOW);

    switch (state) {
      case F_GO_HOME_AT_BOOT:
        readySent = false; emptySent = false; resetSharpTimers();
        stepper.setSpeed(-2500);
        if (bottomHit) { stepper.setSpeed(0); state = F_WAIT_START; }
        break;
      case F_WAIT_START:
        stepper.setSpeed(0); readySent = false; emptySent = false; resetSharpTimers();
        if (startPressed) { delay(200); state = F_MOVE_UP_TO_PLATE; }
        break;
      case F_MOVE_UP_TO_PLATE:
        stepper.setSpeed(2200);
        if (sharpHighStable(sharpVal)) { stepper.setSpeed(0); state = F_READY_WITH_PLATE; readySent = false; resetSharpTimers(); }
        else if (topHit) { stepper.setSpeed(0); state = F_EMPTY_REPORT; emptySent = false; resetSharpTimers(); }
        break;
      case F_READY_WITH_PLATE:
        stepper.setSpeed(0);
        if (!readySent) { sendMsg(MSG_READY); readySent = true; }
        if (sharpLowStable(sharpVal)) { state = F_MOVE_UP_TO_PLATE; readySent = false; resetSharpTimers(); }
        break;
      case F_EMPTY_REPORT:
        stepper.setSpeed(0);
        if (!emptySent) { sendMsg(MSG_EMPTY); emptySent = true; }
        state = F_RETURN_DOWN_HOME;
        break;
      case F_RETURN_DOWN_HOME:
        stepper.setSpeed(-2500);
        if (bottomHit) { stepper.setSpeed(0); state = F_WAIT_START; readySent = false; emptySent = false; resetSharpTimers(); }
        break;
    }
    vTaskDelay(1);
  }
}

void setup() {
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(S_BOTTOM, INPUT);
  pinMode(S_TOP, INPUT);
  pinMode(SHARP_PIN, INPUT);
  Wire.begin(21, 22);
  lcd.begin(16, 2); lcd.backlight();
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv); 
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, MASTER_MAC, 6);
  esp_now_add_peer(&p);
  stepper.setMaxSpeed(4000);
  xTaskCreatePinnedToCore(FeederCoreLogic, "FeederTask", 10000, NULL, 3, &FeederTask, 1);
  lcd.clear();
}

void loop() {
  static unsigned long lastLCD = 0;
  if (millis() - lastLCD > 300) {
    lcd.setCursor(0, 0); 
    lcd.print(getStateName(state));
    // Line 2 has been removed (no printing here)
    lastLCD = millis();
  }
}
