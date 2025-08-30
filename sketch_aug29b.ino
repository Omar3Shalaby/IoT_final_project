#include <ESP32Servo.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

// ===== WIFI CONFIGURATION =====
const char* ssid = "LINKDSL-Adam";
const char* password = "dodo1372015";

// ===== MQTT CONFIGURATION =====
const char* mqtt_server = "514a86bd46594cd09a59db9eae156720.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;  // Secure port
const char* mqtt_username = "hivemq.webclient.1756437583110";
const char* mqtt_password = "34vO:1At7@KW>DyjrR#u";
const char* mqtt_client_id = "smartdoor_esp32_001";  // base id: we'll append MAC

// MQTT Topics
const char* topic_sensor_data = "smartdoor/sensors";
const char* topic_door_control = "smartdoor/control";
const char* topic_alerts = "smartdoor/alerts";
const char* topic_status = "smartdoor/status";

// ===== SERVO =====
Servo myservo;
int quarterTurnTime = 230;
int stopPos = 90;
int cw = 180;
int ccw = 0;

// ===== BUZZER =====
const int buzzerPin = 22;

// ===== LCD =====
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===== LDR =====
const int ldrPin = 34;
const int darkThreshold = 1500;   
const int lightThreshold = 2500;
bool isDark = false;

// ===== LED =====
const int ledPin = 21;
const int redLedPin = 2;

// ===== IR SENSOR =====
const int irPin = 35;
unsigned long personDetectedTime = 0;
bool personPresent = false;
unsigned long maxStayTime = 5000;

// ===== FLAME SENSOR =====
const int flamePin = 4;

// ===== KEYPAD =====
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {13, 12, 14, 27};
byte colPins[COLS] = {26, 25, 33, 32};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ===== PASSWORD =====
String password_door = "1234";
String input = "";

// ===== WIFI & MQTT =====
WiFiClientSecure espClient;  // secure client
PubSubClient mqtt_client(espClient);
unsigned long lastSensorUpdate = 0;
const unsigned long sensorUpdateInterval = 2000;

// ===== DOOR STATUS VARIABLES (FIXED) =====
bool doorLocked = true;
bool doorCurrentlyOpen = false;  // Track actual physical door state
unsigned long doorOpenTime = 0;   // When door was opened
const unsigned long doorOpenDuration = 5000; // How long door stays open

// ===== FIRE SENSOR VARIABLES (FIXED) =====
bool fireDetected = false;        // Current fire status
unsigned long lastFireCheck = 0;  // Last time we checked fire sensor
const unsigned long fireCheckInterval = 500; // Check fire every 500ms

// ===== FUNCTION PROTOTYPES =====
void setupWiFi();
void connectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishStatus(String message);
void publishAlert(String type, String message);
void sendSensorData(int ldrValue);
int readLDR();
void handleKeypad();
void handleDoorControl(DynamicJsonDocument& doc);
void handlePasswordChange(DynamicJsonDocument& doc);
void remoteDoorOpen(String user);
void emergencyDoorOpen(String user);
void openDoor();
void flameEmergency();
void nightIntruderAlarm();
unsigned long getEpochTime();
void checkFireSensor();
void checkDoorTimeout();
void showLightStatus(String status);
void wrongPasswordBeep();

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);

  // Servo
  myservo.attach(18);
  myservo.write(stopPos);

  // Buzzer
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // Normal LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Red LED
  pinMode(redLedPin, OUTPUT);
  digitalWrite(redLedPin, LOW);

  // IR + Flame (use pullups for stable readings unless you have explicit pull-downs)
  pinMode(irPin, INPUT_PULLUP);
  pinMode(flamePin, INPUT);

  // LCD
  Wire.begin(23, 19);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Starting...");

  // ADC setup for LDR
  analogSetPinAttenuation(ldrPin, ADC_11db);  // use full range (~0..3.3V)
  analogReadResolution(12); // 12-bit

  // connect WiFi
  setupWiFi();

  // configure NTP for Africa/Cairo (UTC+2) — Egypt currently no DST
  configTime(2 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  // Setup MQTT with SSL
  espClient.setInsecure(); // for testing; use proper CA in production
  mqtt_client.setServer(mqtt_server, mqtt_port);
  mqtt_client.setCallback(mqttCallback);
  mqtt_client.setKeepAlive(60);

  // Connect to MQTT
  connectMQTT();

  // Ready message
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter Password:");

  Serial.println("🚀 Smart Door System Ready!");
  publishStatus("System started and ready");
}

// ========== MAIN LOOP ==========
void loop() {
  // Keep MQTT connection alive
  if (!mqtt_client.connected()) {
    connectMQTT();
  }
  mqtt_client.loop();

  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    setupWiFi();
    return;
  }

  // ----- LDR check -----
  int ldrValue = readLDR();

  if (!isDark && ldrValue > lightThreshold) {
    isDark = true;
    digitalWrite(ledPin, HIGH);
    showLightStatus("Dark");
  }
  else if (isDark && ldrValue < darkThreshold) {
    isDark = false;
    digitalWrite(ledPin, LOW);
    showLightStatus("Light");
  }

  // ----- IR Sensor -----
  int irValue = digitalRead(irPin);
  if (irValue == LOW) {
    if (!personPresent) {
      personPresent = true;
      personDetectedTime = millis();
      publishStatus("Motion detected at door");
    }

    if (!isDark) {
      lcd.setCursor(0,0);
      lcd.print("Someone at Door ");
    } else {
      if (millis() - personDetectedTime > maxStayTime) {
        nightIntruderAlarm();
      }
    }
  } else {
    if (personPresent) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Enter Password:");
    }
    personPresent = false;
  }

  // ----- Flame Sensor (FIXED) -----
  checkFireSensor();
  if (fireDetected) {
    flameEmergency();
  }

  // ----- Keypad input -----
  handleKeypad();

  // ----- Check door timeout (OPTIONAL) -----
  checkDoorTimeout();

  // ----- Send sensor data via MQTT -----
  if (millis() - lastSensorUpdate > sensorUpdateInterval) {
    sendSensorData(ldrValue);
    lastSensorUpdate = millis();
  }
}

// ===== WIFI SETUP =====
void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("📶 Connecting to ");
  Serial.println(ssid);

  lcd.setCursor(0,1);
  lcd.print("WiFi connecting..");

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    lcd.setCursor(0,1);
    lcd.print("WiFi Connected  ");
    delay(1500);
  } else {
    Serial.println("❌ WiFi connection failed!");
    lcd.setCursor(0,1);
    lcd.print("WiFi Failed     ");
    delay(2000);
    ESP.restart();
  }
}

// ===== MQTT SETUP =====
void connectMQTT() {
  int attempts = 0;

  while (!mqtt_client.connected() && attempts < 5) {
    Serial.print("🔗 Connecting to MQTT... Attempt ");
    Serial.println(attempts + 1);

    lcd.setCursor(0,1);
    lcd.print("MQTT connecting.");

    String clientId = String(mqtt_client_id) + "_" + WiFi.macAddress();
    clientId.replace(":", "");

    Serial.print("Client ID: ");
    Serial.println(clientId);

    String willTopic = topic_status;
    String willMessage = "{\"message\":\"ESP32 disconnected unexpectedly\"}";

    bool ok = mqtt_client.connect(clientId.c_str(), mqtt_username, mqtt_password,
                                  willTopic.c_str(), 1, true, willMessage.c_str());

    if (ok) {
      Serial.println(" ✅ Connected!");
      lcd.setCursor(0,1);
      lcd.print("MQTT Connected  ");
      delay(800);

      if (mqtt_client.subscribe(topic_door_control, 1)) {
        Serial.println("📡 Subscribed to door control");
      } else {
        Serial.println("❌ Failed to subscribe to door control");
      }

      publishStatus("ESP32 connected to secure MQTT");
      return;
    } else {
      Serial.print(" ❌ failed, rc=");
      Serial.println(mqtt_client.state());
      lcd.setCursor(0,1);
      lcd.print("MQTT Failed     ");
      attempts++;
      delay(3000);
    }
  }

  if (!mqtt_client.connected()) {
    Serial.println("❌ MQTT connection failed after all attempts");
    lcd.setCursor(0,1);
    lcd.print("MQTT Failed     ");
  }
}

// ===== MQTT CALLBACK =====
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📨 Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (strcmp(topic, topic_door_control) == 0) {
    handleDoorControl(doc);
  }
}

// ===== GET NTP TIMESTAMP =====
unsigned long getEpochTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("⚠ Failed to obtain time, using millis()");
    return millis() / 1000;
  }
  time_t now;
  now = mktime(&timeinfo);
  return (unsigned long) now;
}

// ===== FIRE SENSOR CHECK (NEW FUNCTION) =====
void checkFireSensor() {
  if (millis() - lastFireCheck > fireCheckInterval) {
    int flameDigital = digitalRead(flamePin);
    
    // Most flame sensors output LOW when fire is detected
    // Adjust this logic based on your specific sensor
    bool currentFireState = (flameDigital == LOW);
    
    if (currentFireState != fireDetected) {
      fireDetected = currentFireState;
      if (fireDetected) {
        Serial.println("🔥 FIRE DETECTED!");
      } else {
        Serial.println("🔥 Fire cleared");
      }
    }
    
    lastFireCheck = millis();
  }
}

// ===== DOOR TIMEOUT CHECK (NEW FUNCTION) =====
void checkDoorTimeout() {
  // If door has been open too long, close it automatically
  if (doorCurrentlyOpen && (millis() - doorOpenTime > doorOpenDuration + 1000)) {
    Serial.println("⚠ Door timeout - force closing");
    doorCurrentlyOpen = false;
    doorLocked = true;
    publishStatus("Door closed due to timeout");
  }
}

// ===== SEND SENSOR DATA (FIXED) =====
void sendSensorData(int ldrValue) {
  if (!mqtt_client.connected()) {
    Serial.println("⚠ MQTT not connected, skipping sensor data");
    return;
  }

  DynamicJsonDocument doc(1024);

  doc["device"] = "esp32-door-system";
  doc["timestamp"] = getEpochTime();

  // FIXED: Send actual door state, not just lock status
  doc["door_locked"] = doorLocked;
  doc["door_open"] = doorCurrentlyOpen;  // NEW: Physical door state
  doc["motion_detected"] = personPresent;

  int lightLevel = (ldrValue < 0) ? 0 : ldrValue;
  doc["light_level"] = lightLevel;
  doc["is_dark"] = isDark;

  // FIXED: Use current fire status
  doc["fire_detected"] = fireDetected;

  doc["wifi_rssi"] = WiFi.RSSI();
  doc["uptime"] = millis() / 1000;
  doc["ldr_status"] = (ldrValue < 0) ? "error" : "ok";

  String jsonString;
  serializeJson(doc, jsonString);

  if (mqtt_client.publish(topic_sensor_data, jsonString.c_str(), true)) {
    Serial.println("📤 Sensor data sent - LDR: " + String(lightLevel) + 
                   " Door Open: " + String(doorCurrentlyOpen ? "YES" : "NO") +
                   " Fire: " + String(fireDetected ? "YES" : "NO"));
  } else {
    Serial.println("❌ Failed to send sensor data - MQTT State: " + String(mqtt_client.state()));
  }
}

// ===== PUBLISH ALERT =====
void publishAlert(String type, String message) {
  if (!mqtt_client.connected()) {
    Serial.println("⚠ MQTT not connected, skipping alert");
    return;
  }

  DynamicJsonDocument doc(512);
  doc["device"] = "esp32-door-system";
  doc["type"] = type;
  doc["message"] = message;
  doc["timestamp"] = getEpochTime();

  String jsonString;
  serializeJson(doc, jsonString);

  if (mqtt_client.publish(topic_alerts, jsonString.c_str())) {
    Serial.println("🚨 Alert sent: " + message);
  } else {
    Serial.println("❌ Failed to send alert");
  }
}

// ===== PUBLISH STATUS =====
void publishStatus(String message) {
  if (!mqtt_client.connected()) {
    Serial.println("⚠ MQTT not connected, skipping status");
    return;
  }

  DynamicJsonDocument doc(512);
  doc["device"] = "esp32-door-system";
  doc["message"] = message;
  doc["timestamp"] = getEpochTime();
  doc["uptime"] = millis() / 1000;

  String jsonString;
  serializeJson(doc, jsonString);

  if (mqtt_client.publish(topic_status, jsonString.c_str(), true)) {
    Serial.println("📊 Status: " + message);
  } else {
    Serial.println("❌ Failed to send status");
  }
}

// ===== DOOR CONTROL HANDLER =====
void handleDoorControl(DynamicJsonDocument& doc) {
  String command = "";
  if (doc.containsKey("command")) command = String((const char*) doc["command"]);
  String receivedPassword = "";
  if (doc.containsKey("password")) receivedPassword = String((const char*) doc["password"]);
  String user = "";
  if (doc.containsKey("user")) user = String((const char*) doc["user"]);

  Serial.println("🎮 Door command: " + command);

  if (command == "REMOTE_UNLOCK" || command == "REMOTE_LOCK") {
    if (receivedPassword != password_door) {
      Serial.println("❌ Wrong password for remote command");
      publishAlert("access_denied", "Wrong password for remote command from " + user);
      return;
    }
  }

  if (command == "UNLOCK" || command == "REMOTE_UNLOCK") {
    Serial.println("🔓 Remote unlock command");
    remoteDoorOpen(user);
  }
  else if (command == "LOCK" || command == "REMOTE_LOCK") {
    Serial.println("🔒 Remote lock command");
    doorLocked = true;
    publishStatus("Door locked remotely by " + user);
  }
  else if (command == "EMERGENCY_OPEN") {
    Serial.println("🚨 Emergency open command");
    emergencyDoorOpen(user);
  }
  else if (command == "CHANGE_PASSWORD") {
    Serial.println("🔑 Password change request");
    handlePasswordChange(doc);
  }
  else if (command == "SYSTEM_ARM") {
    Serial.println("🛡 System armed");
    publishStatus("System armed by " + user);
  }
  else if (command == "SYSTEM_DISARM") {
    Serial.println("🔓 System disarmed");
    publishStatus("System disarmed by " + user);
  }
  else if (command == "SYSTEM_RESET") {
    Serial.println("🔄 System reset requested");
    publishStatus("System reset by " + user);
    delay(1000);
    ESP.restart();
  }
}

// ===== PASSWORD CHANGE HANDLER =====
void handlePasswordChange(DynamicJsonDocument& doc) {
  String currentPassword = doc.containsKey("current_password") ? String((const char*) doc["current_password"]) : "";
  String newPassword = doc.containsKey("new_password") ? String((const char*) doc["new_password"]) : "";
  String user = doc.containsKey("user") ? String((const char*) doc["user"]) : "";

  if (currentPassword != password_door) {
    Serial.println("❌ Wrong current password for password change");
    publishAlert("password_change_failed", "Wrong current password from " + user);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Password Change");
    lcd.setCursor(0,1);
    lcd.print("Wrong Current");
    delay(2000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Enter Password:");
    return;
  }

  if (newPassword.length() < 4 || newPassword.length() > 8) {
    Serial.println("❌ New password invalid length");
    publishAlert("password_change_failed", "Invalid new password length from " + user);
    return;
  }

  for (int i = 0; i < newPassword.length(); i++) {
    if (!isDigit(newPassword.charAt(i))) {
      Serial.println("❌ New password must be digits only");
      publishAlert("password_change_failed", "New password must be digits only from " + user);
      return;
    }
  }

  password_door = newPassword;
  Serial.println("✅ Password changed successfully to: " + newPassword);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Password Changed");
  lcd.setCursor(0,1);
  lcd.print("New: " + newPassword);
  delay(2000);

  for (int i=0;i<2;i++) { digitalWrite(buzzerPin, HIGH); delay(150); digitalWrite(buzzerPin, LOW); delay(150); }

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter Password:");

  publishStatus("Password changed successfully by " + user);
}

// ===== REMOTE DOOR OPEN (FIXED) =====
void remoteDoorOpen(String user) {
  Serial.println("🔓 Remote door open for: " + user);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Remote Access");
  lcd.setCursor(0,1);
  lcd.print("Opening...");

  digitalWrite(buzzerPin, HIGH); delay(150); digitalWrite(buzzerPin, LOW);
  
  // FIXED: Update both door states
  myservo.write(cw);
  delay(quarterTurnTime);
  myservo.write(stopPos);

  doorLocked = false;
  doorCurrentlyOpen = true;  // NEW: Track physical state
  doorOpenTime = millis();   // NEW: Track when opened
  
  publishStatus("Door opened remotely by " + user);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Door Open");
  String usr = user;
  if (usr.length() > 8) usr = usr.substring(0,8);
  lcd.setCursor(0,1);
  lcd.print("Remote: " + usr);

  delay(5000);

  // FIXED: Update physical state when closing
  myservo.write(ccw);
  delay(quarterTurnTime);
  myservo.write(stopPos);

  doorLocked = false;        // door is closed but NOT locked
  doorCurrentlyOpen = false;
  publishStatus("Door closed (unlocked)");


  digitalWrite(buzzerPin, HIGH); delay(120); digitalWrite(buzzerPin, LOW);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Door Closed");
  delay(1000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter Password:");
}

// ===== EMERGENCY DOOR OPEN =====
void emergencyDoorOpen(String user) {
  Serial.println("🚨 Emergency door open by: " + user);
  publishAlert("emergency_open", "Emergency door open activated by " + user);
  remoteDoorOpen(user);
}

// ===== KEYPAD DOOR OPEN (FIXED) =====
void openDoor() {
  Serial.println("Correct password → Door OPEN");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Access Granted");
  lcd.setCursor(0,1);
  lcd.print("Door Opening...");

  digitalWrite(buzzerPin, HIGH); delay(150); digitalWrite(buzzerPin, LOW);

  // FIXED: Update both door states
  myservo.write(cw);
  delay(quarterTurnTime);
  myservo.write(stopPos);

  doorLocked = false;
  doorCurrentlyOpen = true;  // NEW: Track physical state
  doorOpenTime = millis();   // NEW: Track when opened
  
  publishStatus("Door opened with keypad password");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Door Open");
  lcd.setCursor(0,1);
  lcd.print("Welcome!");

  delay(5000);

  // FIXED: Update physical state when closing
  myservo.write(ccw);
  delay(quarterTurnTime);
  myservo.write(stopPos);

  doorLocked = true;
  doorCurrentlyOpen = false;  // NEW: Door is now physically closed
  
  publishStatus("Door closed automatically");

  digitalWrite(buzzerPin, HIGH); delay(120); digitalWrite(buzzerPin, LOW);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Door Closed");
  delay(1000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter Password:");
}

// ===== WRONG PASSWORD BEEP =====
void wrongPasswordBeep() {
  publishAlert("wrong_password", "Wrong password attempted on keypad");

  for (int i = 0; i < 3; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(180);
    digitalWrite(buzzerPin, LOW);
    delay(180);
  }
}

// ===== KEYPAD HANDLER =====
void handleKeypad() {
  char key = keypad.getKey();
  if (key) {
    if (key == '#') {
      if (input == password_door) {
        openDoor();
      } else {
        wrongPasswordBeep();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Access Denied");
        lcd.setCursor(0,1);
        lcd.print("Try Again");
        delay(1500);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Enter Password:");
      }
      input = "";
    }
    else if (key == '*') {
      input = "";
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Enter Password:");
    }
    else if (key == 'D') {
      if (input.length() > 0) {
        input.remove(input.length() - 1);
        lcd.setCursor(0,1);
        lcd.print("                ");
        lcd.setCursor(0,1);
        for (int i=0; i<input.length(); i++) lcd.print("*");
      }
    }
    else {
      input += key;
      lcd.setCursor(0,1);
      lcd.print("                ");
      lcd.setCursor(0,1);
      for (int i=0; i<input.length(); i++) lcd.print("*");
    }
  }
}

// ===== NIGHT INTRUDER ALARM =====
void nightIntruderAlarm() {
  publishAlert("night_intrusion", "Night intruder detected - person staying too long");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Intruder Alert!");
  lcd.setCursor(0,1);
  lcd.print("At Night");

  for (int i = 0; i < 20; i++) {
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
    delay(100);
  }

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter Password:");
}

// ===== FLAME EMERGENCY (FIXED) =====
void flameEmergency() {
  publishAlert("fire_detected", "FIRE EMERGENCY - Door opening automatically");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("FIRE DETECTED!");
  lcd.setCursor(0,1);
  lcd.print("Door Opening");

  for (int i = 0; i < 10; i++) {
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(redLedPin, HIGH);
    delay(200);
    digitalWrite(buzzerPin, LOW);
    digitalWrite(redLedPin, LOW);
    delay(200);
  }

  // FIXED: Update both door states
  myservo.write(cw);
  delay(quarterTurnTime);
  myservo.write(stopPos);

  doorLocked = false;
  doorCurrentlyOpen = true;  // NEW: Track physical state
  doorOpenTime = millis();   // NEW: Track when opened
  
  publishStatus("Door opened due to fire emergency");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Door Open");
  lcd.setCursor(0,1);
  lcd.print("Evacuate Now!");

  delay(10000);

  // FIXED: Update physical state when closing
  myservo.write(ccw);
  delay(quarterTurnTime);
  myservo.write(stopPos);

  doorLocked = true;
  doorCurrentlyOpen = false;  // NEW: Door is now physically closed
  
  publishStatus("Door closed after fire emergency");

  digitalWrite(buzzerPin, HIGH); delay(120); digitalWrite(buzzerPin, LOW);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Door Closed");
  delay(1500);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter Password:");
}

// ===== LDR READING =====
int readLDR() {
  long sum = 0;
  int validReadings = 0;

  for (int i = 0; i < 10; i++) {
    int reading = analogRead(ldrPin);
    if (reading >= 0 && reading <= 4095) {
      sum += reading;
      validReadings++;
    }
    delay(2);
  }

  if (validReadings == 0) {
    Serial.println("⚠ LDR: No valid readings!");
    return -1;
  }

  int average = sum / validReadings;
  return average;
}

// ===== SHOW LIGHT STATUS ON LCD =====
void showLightStatus(String status) {
  lcd.setCursor(0, 0);
  lcd.print("Light Status:   ");
  lcd.setCursor(0, 1);
  lcd.print(status + "       ");
}