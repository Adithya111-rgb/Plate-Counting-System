/* * PROJECT: PCB-Mounted Pull-Down Test
 * WIRING: COM -> VCC | NO -> D23
 * NOTE: PCB has a physical resistor to GND
 */

#define BTN_PIN 23

void setup() {
  Serial.begin(115200);
  
  // Standard INPUT is fine because the PCB resistor handles the stability
  pinMode(BTN_PIN, INPUT); 
  
  Serial.println("========================================");
  Serial.println("   PCB RESISTOR TEST: PIN D23");
  Serial.println("========================================");
}

void loop() {
  // Logic: 1 (HIGH) = Pressed | 0 (LOW) = Released
  if (digitalRead(BTN_PIN) == HIGH) {
    Serial.println("Button: PRESSED");
  } 
  else {
    Serial.println("Button: RELEASED");
  }

  delay(150); 
}
