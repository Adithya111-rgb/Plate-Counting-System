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

#define ENC_A 19
#define ENC_B 18
#define ENC_SW 5

uint8_t STACK_MAC[] = {0xC0, 0xCD, 0xD6, 0x83, 0xEC, 0xF0};
uint8_t FEED_MAC[]  = {0xC0, 0xCD, 0xD6, 0x85, 0xF2, 0x44};

/* ---------------- State Management ---------------- */
enum MState { M_SETUP, M_WAIT_READY, M_RUN_BATCH, M_WAIT_STACK_CLEAR };
MState masterState = M_SETUP; 

/* ---------------- Global Variables ---------------- */
LiquidCrystal_I2C lcd(0x27, 20, 4);

volatile int encoderPos = 5;
volatile bool encoderChanged = false;
unsigned long buttonPressStart = 0;
bool isButtonPressed = false;
bool longPressTriggered = false; 

typedef struct { int code; } debug_msg;
debug_msg incoming, outgoing;

enum ArmState { ARM_IDLE, ARM_PULSE_LOW, ARM_COOLDOWN };
ArmState armState = ARM_IDLE;
unsigned long armTimer = 0;
const unsigned long ARM_LOW_MS = 150;
const unsigned long ARM_COOLDOWN_MS = 300;

bool feederReady = false, stackerReady = false;
bool lastDropState = LOW, lastCountState = LOW;
bool countArmed = true; // Start armed

int plateCount = 0;
int completedStacks = 0; 

unsigned long lastCountMs = 0;
const unsigned long COUNT_LOCK_MS = 150; // Optimized for high speed
unsigned long countHighStart = 0;
const unsigned long MIN_HIGH_STABLE_MS = 5; // Ultra-fast detection
unsigned long batchLockUntil = 0;
const unsigned long BATCH_RESTART_GUARD_MS = 1000;

/* ---------------- Encoder ISR ---------------- */
void IRAM_ATTR readEncoderISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 50) {
    if (digitalRead(ENC_A) != digitalRead(ENC_B)) encoderPos += 5;
    else encoderPos -= 5;
    if (encoderPos < 5) encoderPos = 5;
    if (encoderPos > 50) encoderPos = 50;
    encoderChanged = true;
  }
  lastInterruptTime = interruptTime;
}

/* ---------------- Controls ---------------- */
void handleControls() {
  bool btnActive = (digitalRead(ENC_SW) == LOW);
  if (masterState == M_SETUP && encoderChanged) {
      TARGET_COUNT = encoderPos;
      encoderChanged = false;
  }

  if (btnActive) {
    if (!isButtonPressed) {
      buttonPressStart = millis();
      isButtonPressed = true;
      longPressTriggered = false; 
    }
    if (masterState != M_SETUP && !longPressTriggered && (millis() - buttonPressStart > 3000)) {
        masterState = M_SETUP;
        lcd.clear();
        longPressTriggered = true; 
    }
  } else {
    if (isButtonPressed && !longPressTriggered) {
      unsigned long holdTime = millis() - buttonPressStart;
      if (holdTime > 50 && holdTime < 1000 && masterState == M_SETUP) { 
            masterState = M_WAIT_READY;
            plateCount = 0;
            encoderPos = TARGET_COUNT; 
            lcd.clear();
      }
    }
    isButtonPressed = false;
    longPressTriggered = false; 
  }
}

/* ---------------- ESP-NOW ---------------- */
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&incoming, data, sizeof(incoming));
  if (memcmp(info->src_addr, FEED_MAC, 6) == 0) {
    if (incoming.code == 10) feederReady = true;
    else if (incoming.code == 404) feederReady = false;
  }
  if (memcmp(info->src_addr, STACK_MAC, 6) == 0) {
    if (incoming.code == 10) stackerReady = true;
  }
}

void lockBatchAndClear() {
  digitalWrite(RELAY_MACHINE_WAIT, LOW);
  digitalWrite(RELAY_PNP_ARM, HIGH);
  armState = ARM_IDLE;
  completedStacks++; 
  stackerReady = false;
  batchLockUntil = millis() + BATCH_RESTART_GUARD_MS;
  outgoing.code = 25;
  esp_now_send(STACK_MAC, (uint8_t *)&outgoing, sizeof(outgoing));
  masterState = M_WAIT_STACK_CLEAR;
}

void updateArmPulse(bool allowPick) {
  unsigned long now = millis();
  if (!allowPick) {
    digitalWrite(RELAY_PNP_ARM, HIGH);
    armState = ARM_IDLE;
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
      if (now - armTimer >= ARM_COOLDOWN_MS) armState = ARM_IDLE;
      break;
  }
  lastDropState = dropState;
}

void updateCounting(bool batchRunning) {
  if (!batchRunning) return;
  bool currentState = digitalRead(COUNT_PROX_PIN);
  unsigned long now = millis();

  // STEP 1: If sensor is LOW, the space between plates is clear. Ready to count.
  if (currentState == LOW) {
    countArmed = true;
  }

  // STEP 2: If armed and sensor goes HIGH, a plate is detected.
  if (countArmed && currentState == HIGH) {
    // Check timing lockout (has enough time passed since the last valid plate?)
    if (now - lastCountMs >= COUNT_LOCK_MS) {
      
      // Mark when the signal first went high
      if (lastCountState == LOW) {
        countHighStart = now;
      }
      
      // If signal is stable for 5ms, it is a guaranteed plate.
      if (now - countHighStart >= MIN_HIGH_STABLE_MS) {
        plateCount++;
        lastCountMs = now;
        countArmed = false; // Must see LOW again before counting another
        
        if (plateCount >= TARGET_COUNT) {
          lockBatchAndClear();
        }
      }
    }
  }
  lastCountState = currentState;
}

void setup() {
  pinMode(RELAY_MACHINE_WAIT, OUTPUT);
  pinMode(RELAY_PNP_ARM, OUTPUT);
  digitalWrite(RELAY_MACHINE_WAIT, LOW);
  digitalWrite(RELAY_PNP_ARM, HIGH);
  pinMode(DROP_PROX_PIN, INPUT);
  pinMode(COUNT_PROX_PIN, INPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoderISR, FALLING);
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = 0; peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, STACK_MAC, 6); esp_now_add_peer(&peerInfo);
  memcpy(peerInfo.peer_addr, FEED_MAC, 6); esp_now_add_peer(&peerInfo);
}

void loop() {
  handleControls();
  unsigned long now = millis();

  switch (masterState) {
    case M_SETUP:
      digitalWrite(RELAY_MACHINE_WAIT, LOW);
      digitalWrite(RELAY_PNP_ARM, HIGH);
      break;
    case M_WAIT_READY:
      if (now >= batchLockUntil && feederReady && stackerReady) {
        plateCount = 0;
        countArmed = true;
        masterState = M_RUN_BATCH;
      }
      break;
    case M_RUN_BATCH:
      digitalWrite(RELAY_MACHINE_WAIT, (feederReady && stackerReady) ? HIGH : LOW);
      updateCounting(true);
      updateArmPulse(feederReady && stackerReady && (plateCount < TARGET_COUNT));
      break;
    case M_WAIT_STACK_CLEAR:
      if (now >= batchLockUntil && stackerReady && feederReady) masterState = M_WAIT_READY;
      break;
  }

  static unsigned long lastLCD = 0;
  if (millis() - lastLCD > 250) {
    lcd.setCursor(0, 0);
    if (masterState == M_SETUP) {
      lcd.print("--- SETTINGS ---    ");
      lcd.setCursor(0, 1); lcd.print("Set Stack: "); lcd.print(TARGET_COUNT); lcd.print("     "); 
      lcd.setCursor(0, 2); lcd.print("Click: SAVE         ");
      lcd.setCursor(0, 3); lcd.print("Hold: EDIT MODE     ");
    } else {
      lcd.print("C: "); lcd.print(plateCount); lcd.print("/"); lcd.print(TARGET_COUNT);
      lcd.print("  SC: "); lcd.print(completedStacks); lcd.print("    ");
      lcd.setCursor(0, 1); lcd.print("Feeder: "); lcd.print(feederReady ? "READY   " : "WAIT    ");
      lcd.setCursor(0, 2); lcd.print("Stacker: "); lcd.print(stackerReady ? "READY  " : "WAIT   ");
      lcd.setCursor(0, 3); lcd.print("State: "); 
      if(masterState == M_WAIT_READY) lcd.print("WAITING     ");
      else if(masterState == M_RUN_BATCH) lcd.print("RUNNING     ");
      else lcd.print("CLEARING    ");
    }
    lastLCD = millis();
  }
}
