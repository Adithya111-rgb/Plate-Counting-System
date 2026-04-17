#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* ---------------- Configuration ---------------- */
int TARGET_COUNT = 5; 
#define DROP_PROX_PIN 32
#define COUNT_PROX_PIN 34
#define RELAY_MACHINE_WAIT 25
#define RELAY_PNP_ARM 27

// Encoder Pins
#define ENC_A 19
#define ENC_B 18
#define ENC_SW 5

uint8_t STACK_MAC[] = {0xC0, 0xCD, 0xD6, 0x83, 0xEC, 0xF0};
uint8_t FEED_MAC[]  = {0xC0, 0xCD, 0xD6, 0x85, 0xF2, 0x44};

/* ---------------- State Management ---------------- */
enum MState {
  M_SETUP, 
  M_WAIT_READY,
  M_RUN_BATCH,
  M_WAIT_STACK_CLEAR
};

MState masterState = M_SETUP; 

/* ---------------- Global Variables ---------------- */
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Manual Encoder Variables
volatile int encoderPos = 5;
volatile bool encoderChanged = false;
unsigned long buttonPressStart = 0;
bool isButtonPressed = false;

/* ---------------- Message Codes ---------------- */
enum MsgCode {
  MSG_READY       = 10,
  MSG_EMPTY       = 404,
  CMD_STACK_CLEAR = 25,
  MSG_GET_STATUS  = 50  // New status request code
};

typedef struct {
  int code;
} debug_msg;

debug_msg incoming, outgoing;

/* ---------------- Arm Pulse State ---------------- */
enum ArmState {
  ARM_IDLE,
  ARM_PULSE_LOW,
  ARM_COOLDOWN
};

ArmState armState = ARM_IDLE;
unsigned long armTimer = 0;
const unsigned long ARM_LOW_MS = 150;
const unsigned long ARM_COOLDOWN_MS = 300;

/* ---------------- Runtime ---------------- */
bool feederReady = false;
bool stackerReady = false;

bool lastDropState = LOW;
bool lastCountState = LOW;

bool countArmed = false;
bool countLatched = false;

int plateCount = 0;

unsigned long lastCountMs = 0;
const unsigned long COUNT_LOCK_MS = 300;

unsigned long countHighStart = 0;
const unsigned long MIN_HIGH_STABLE_MS = 50;

unsigned long batchLockUntil = 0;
const unsigned long BATCH_RESTART_GUARD_MS = 1000;

/* ---------------- Manual Encoder ISR ---------------- */
void IRAM_ATTR readEncoderISR() {
  static uint8_t old_AB = 0;
  static const int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  
  old_AB <<= 2;
  old_AB |= (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  int8_t direction = enc_states[(old_AB & 0x0f)];
  
  if (direction != 0) {
    encoderPos += (direction * 5); 
    if (encoderPos < 5) encoderPos = 5;
    if (encoderPos > 50) encoderPos = 50;
    encoderChanged = true;
  }
}

/* ---------------- Encoder & Button Logic ---------------- */
void handleControls() {
  if (encoderChanged) {
    TARGET_COUNT = encoderPos;
    encoderChanged = false;
  }

  bool btnState = (digitalRead(ENC_SW) == LOW); 

  if (btnState) {
    if (!isButtonPressed) {
      buttonPressStart = millis();
      isButtonPressed = true;
    }
    
    if (millis() - buttonPressStart > 5000) {
      if (masterState != M_SETUP) {
        masterState = M_SETUP;
        lcd.clear();
      }
    }
  } else {
    if (isButtonPressed) {
      unsigned long holdTime = millis() - buttonPressStart;
      if (holdTime > 50 && holdTime < 1000) { 
        if (masterState == M_SETUP) {
          masterState = M_WAIT_READY;
          lcd.clear();
        }
      }
    }
    isButtonPressed = false;
  }
}

/* ---------------- ESP-NOW Receive ---------------- */
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&incoming, data, sizeof(incoming));

  if (memcmp(info->src_addr, FEED_MAC, 6) == 0) {
    if (incoming.code == MSG_READY) {
      feederReady = true;
    } else if (incoming.code == MSG_EMPTY) {
      feederReady = false;
      digitalWrite(RELAY_MACHINE_WAIT, LOW);
      digitalWrite(RELAY_PNP_ARM, HIGH);
      armState = ARM_IDLE;
    }
  }

  if (memcmp(info->src_addr, STACK_MAC, 6) == 0) {
    if (incoming.code == MSG_READY) {
      stackerReady = true;
    }
  }
}

/* ---------------- Helpers ---------------- */
void sendToStacker(int code) {
  outgoing.code = code;
  esp_now_send(STACK_MAC, (uint8_t *)&outgoing, sizeof(outgoing));
}

void stopArmNow() {
  digitalWrite(RELAY_PNP_ARM, HIGH);
  armState = ARM_IDLE;
}

void lockBatchAndClear() {
  digitalWrite(RELAY_MACHINE_WAIT, LOW);
  stopArmNow();

  stackerReady = false;
  batchLockUntil = millis() + BATCH_RESTART_GUARD_MS;

  sendToStacker(CMD_STACK_CLEAR);
  masterState = M_WAIT_STACK_CLEAR;
}

/* ---------------- ARM CONTROL ---------------- */
void updateArmPulse(bool allowPick) {
  unsigned long now = millis();

  if (!allowPick) {
    stopArmNow();
    lastDropState = digitalRead(DROP_PROX_PIN);
    return;
  }

  bool dropState = digitalRead(DROP_PROX_PIN);

  switch (armState) {
    case ARM_IDLE:
      if (dropState == HIGH && lastDropState == LOW) {
        digitalWrite(RELAY_PNP_ARM, LOW);
        armTimer = now;
        armState = ARM_PULSE_LOW;
      }
      break;

    case ARM_PULSE_LOW:
      if (now - armTimer >= ARM_LOW_MS) {
        digitalWrite(RELAY_PNP_ARM, HIGH);
        armTimer = now;
        armState = ARM_COOLDOWN;
      }
      break;

    case ARM_COOLDOWN:
      if (now - armTimer >= ARM_COOLDOWN_MS) {
        armState = ARM_IDLE;
      }
      break;
  }

  lastDropState = dropState;
}

/* ---------------- COUNTING ---------------- */
void updateCounting(bool batchRunning) {
  if (!batchRunning) {
    lastCountState = digitalRead(COUNT_PROX_PIN);
    countArmed = false;
    countLatched = false;
    return;
  }

  bool currentState = digitalRead(COUNT_PROX_PIN);
  unsigned long now = millis();

  if (!countArmed) {
    if (currentState == LOW) {
      countArmed = true;
    }
    lastCountState = currentState;
    return;
  }

  if (currentState == HIGH && lastCountState == LOW) {
    countHighStart = now;
    countLatched = false;
  }

  if (currentState == HIGH && !countLatched) {
    if ((now - countHighStart >= MIN_HIGH_STABLE_MS) &&
        (now - lastCountMs >= COUNT_LOCK_MS)) {

      countLatched = true;
      lastCountMs = now;
      plateCount++;

      if (plateCount >= TARGET_COUNT) {
        lockBatchAndClear();
      }
    }
  }

  lastCountState = currentState;
}

/* ---------------- Setup ---------------- */
void setup() {
  Serial.begin(115200);

  pinMode(RELAY_MACHINE_WAIT, OUTPUT);
  pinMode(RELAY_PNP_ARM, OUTPUT);
  digitalWrite(RELAY_MACHINE_WAIT, LOW);
  digitalWrite(RELAY_PNP_ARM, HIGH);

  pinMode(DROP_PROX_PIN, INPUT);
  pinMode(COUNT_PROX_PIN, INPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), readEncoderISR, CHANGE);

  Wire.begin(21, 22);
  lcd.begin(20, 4);
  lcd.backlight();

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, STACK_MAC, 6);
  esp_now_add_peer(&peerInfo);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, FEED_MAC, 6);
  esp_now_add_peer(&peerInfo);

  // Status Request on Boot
  outgoing.code = MSG_GET_STATUS;
  esp_now_send(STACK_MAC, (uint8_t *)&outgoing, sizeof(outgoing));
  esp_now_send(FEED_MAC, (uint8_t *)&outgoing, sizeof(outgoing));
}

void loop() {
  handleControls();
  unsigned long now = millis();

  switch (masterState) {
    case M_SETUP:
      digitalWrite(RELAY_MACHINE_WAIT, LOW);
      stopArmNow();
      break;

    case M_WAIT_READY:
      digitalWrite(RELAY_MACHINE_WAIT, LOW);
      stopArmNow();
      if (now < batchLockUntil) break;

      if (feederReady && stackerReady) {
        plateCount = 0;
        countArmed = false;
        countLatched = false;
        lastDropState = digitalRead(DROP_PROX_PIN);
        lastCountState = digitalRead(COUNT_PROX_PIN);
        lastCountMs = 0;
        masterState = M_RUN_BATCH;
      }
      break;

    case M_RUN_BATCH: {
      bool machineCanRun = feederReady && stackerReady;
      bool armCanRun = machineCanRun && (plateCount < TARGET_COUNT);
      digitalWrite(RELAY_MACHINE_WAIT, machineCanRun ? HIGH : LOW);
      updateCounting(true);
      updateArmPulse(armCanRun);
      break;
    }

    case M_WAIT_STACK_CLEAR:
      digitalWrite(RELAY_MACHINE_WAIT, LOW);
      stopArmNow();
      if (now < batchLockUntil) break;
      if (stackerReady && feederReady) {
        masterState = M_WAIT_READY;
      }
      break;
  }

  static unsigned long lastLCD = 0;
  if (millis() - lastLCD > 300) {
    if (masterState == M_SETUP) {
      lcd.setCursor(0, 0); lcd.print("--- SETTINGS ---    ");
      lcd.setCursor(0, 1); lcd.print("Set Stack: "); lcd.print(TARGET_COUNT); lcd.print("     ");
      lcd.setCursor(0, 2); lcd.print("Click to Save       ");
      lcd.setCursor(0, 3); lcd.print("HOLD to Reset       ");
    } else {
      lcd.setCursor(0, 0); lcd.print("Count: "); lcd.print(plateCount); lcd.print("/"); lcd.print(TARGET_COUNT); lcd.print("   ");
      lcd.setCursor(0, 1); lcd.print("Feeder: "); lcd.print(feederReady ? "READY   " : "WAIT    ");
      lcd.setCursor(0, 2); lcd.print("Stack:  "); lcd.print(stackerReady ? "READY   " : "WAIT    ");
      lcd.setCursor(0, 3); lcd.print("State: "); lcd.print(masterState == M_WAIT_READY ? "WAIT " : masterState == M_RUN_BATCH ? "RUN  " : "CLEAR");
    }
    lastLCD = millis();
  }
}
