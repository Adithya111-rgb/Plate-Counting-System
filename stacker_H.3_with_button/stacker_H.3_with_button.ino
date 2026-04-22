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
#define BTN_ARM_HOME 23    

/* ---------------- Speed Adjustments ---------------- */
#define SPEED_UP    4000   // Restored to original fast speed
#define SPEED_DOWN -4000   // Reduced significantly to stop jittering during plate drops

/* ---------------- Thresholds ---------------- */
#define SHARP_DETECT 1200 
#define SHARP_CLEAR   700 

uint8_t MASTER_MAC[] = {0xC0, 0xCD, 0xD6, 0x85, 0xE9, 0x64};

typedef struct { int code; } debug_msg;
debug_msg incoming, outgoing;

LiquidCrystal_I2C lcd(0x26, 16, 2);
AccelStepper stepper(1, STEP_PIN, DIR_PIN);

enum SState { S_HOME, S_WAIT, S_FULL_STACK_CLEAR, S_RELAY_WAIT, S_WAIT_FOR_ARM_CONFIRM, S_RETURN_TOP };
volatile SState state = S_HOME;

unsigned long relayTimer = 0;
bool readySent = false;
bool plate = false, lastPlate = false, moving = false;

int readSharp() {
  long s = 0;
  for (int i = 0; i < 5; i++) s += analogRead(SHARP_PIN);
  return s / 5;
}

void sendReady() {
  outgoing.code = 10;
  esp_now_send(MASTER_MAC, (uint8_t*)&outgoing, sizeof(outgoing));
}

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&incoming, data, sizeof(incoming));
  if (incoming.code == 25) { 
    state = S_FULL_STACK_CLEAR; 
    readySent = false; 
  }
}

const char* getStateName(SState s) {
  switch (s) {
    case S_HOME:              return "GO TOP         ";
    case S_WAIT:              return "WAITING PLATE  ";
    case S_FULL_STACK_CLEAR:  return "FULL CLEAR     ";
    case S_RELAY_WAIT:        return "RELAY ON...    "; 
    case S_WAIT_FOR_ARM_CONFIRM: return "WAITING FOR ARM";
    case S_RETURN_TOP:        return "RETURN TOP     ";
    default:                  return "UNKNOWN        ";
  }
}

void setup() {
  pinMode(SENSOR_BOTTOM, INPUT);
  pinMode(SENSOR_TOP, INPUT_PULLUP);
  pinMode(SHARP_PIN, INPUT);
  pinMode(BTN_ARM_HOME, INPUT); // PCB Resistor handles the pull-down
  
  pinMode(RELAY_CLEAR, OUTPUT);
  digitalWrite(RELAY_CLEAR, HIGH); 
  
  Wire.begin(21, 22);
  lcd.begin(16, 2); 
  lcd.backlight();
  
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
  
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, MASTER_MAC, 6);
  esp_now_add_peer(&p);
  
  stepper.setMaxSpeed(5000); 
}

void loop() {
  stepper.runSpeed();
  int sharpVal = readSharp();
  bool topHit = (digitalRead(SENSOR_TOP) == LOW);
  bool bottomHit = (digitalRead(SENSOR_BOTTOM) == LOW);
  
  // Read D23 Arm Status (HIGH = Arm is back)
  bool armIsHome = (digitalRead(BTN_ARM_HOME) == HIGH);

  lastPlate = plate;
  if (sharpVal > SHARP_DETECT) plate = true;
  else if (sharpVal < SHARP_CLEAR) plate = false;

  switch (state) {
    case S_HOME:
      stepper.setSpeed(SPEED_UP);
      if (topHit) { stepper.setSpeed(0); state = S_WAIT; readySent = false; }
      break;

    case S_WAIT:
      if (!readySent) { sendReady(); readySent = true; }
      // Smooth descent when a plate is detected
      if (plate && !lastPlate && !moving) { stepper.setSpeed(SPEED_DOWN); moving = true; }
      if (!plate && lastPlate && moving) { stepper.setSpeed(0); moving = false; }
      if (bottomHit) { stepper.setSpeed(0); moving = false; }
      break;

    case S_FULL_STACK_CLEAR:
      stepper.setSpeed(SPEED_DOWN);
      if (bottomHit) {
        stepper.setSpeed(0);
        digitalWrite(RELAY_CLEAR, LOW); // Trigger Ejection
        relayTimer = millis();
        state = S_RELAY_WAIT;
      }
      break;

    case S_RELAY_WAIT:
      if (millis() - relayTimer >= 5000) {
        digitalWrite(RELAY_CLEAR, HIGH); // Stop Relay
        state = S_WAIT_FOR_ARM_CONFIRM; 
      }
      break;

    case S_WAIT_FOR_ARM_CONFIRM:
      stepper.setSpeed(0); // Safety pause at bottom
      if (armIsHome) { 
        delay(150); 
        state = S_RETURN_TOP; 
      }
      break;

    case S_RETURN_TOP:
      stepper.setSpeed(SPEED_UP); // Fast return to top
      if (topHit) { 
        stepper.setSpeed(0); 
        state = S_WAIT; 
        readySent = false; 
        moving = false; 
      }
      break;
  }

  static unsigned long lastLCD = 0;
  if (millis() - lastLCD > 300) {
    lcd.setCursor(0, 0); 
    lcd.print(getStateName(state));
    lastLCD = millis();
  }
}
