#include <ESP32Servo.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>  
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ===== WIFI CONFIGURATION =====
const char* ssid = "LINKDSL-Adam";           
const char* password = "dodo1372015";   

// ===== MQTT CONFIGURATION =====
const char* mqtt_server = "514a86bd46594cd09a59db9eae156720.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;  // Secure port
const char* mqtt_username = "hivemq.webclient.1756437583110";
const char* mqtt_password = "34vO:1At7@KW>DyjrR#u";
const char* mqtt_client_id = "smartdoor_esp32_001";  //  Make this unique

// MQTT Topics
const char* topic_sensor_data = "smartdoor/sensors";
const char* topic_door_control = "smartdoor/control";
const char* topic_alerts = "smartdoor/alerts";
const char* topic_status = "smartdoor/status";

// ===== SERVO =====
Servo myservo;
int quarterTurnTime = 177;
int stopPos = 90;
int cw = 180;
int ccw = 0;

// ===== BUZZER =====
const int buzzerPin = 22;

// ===== LCD =====
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===== LDR =====
const int ldrPin = 15;         
const int darkThreshold = 900;
const int lightThreshold = 1100; 
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
WiFiClientSecure espClient;  // Changed to secure client
PubSubClient mqtt_client(espClient);
unsigned long lastSensorUpdate = 0;
const unsigned long sensorUpdateInterval = 2000;
bool doorLocked = true;

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

  // IR + Flame
  pinMode(irPin, INPUT);
  pinMode(flamePin, INPUT);

  // LCD
  Wire.begin(23, 19);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Starting...");

  // Connect to WiFi
  setupWiFi();
  
  // Setup MQTT with SSL
  espClient.setInsecure(); // Skip certificate validation for simplicity
  mqtt_client.setServer(mqtt_server, mqtt_port);
  mqtt_client.setCallback(mqttCallback);
  mqtt_client.setKeepAlive(60);  // Set keepalive to 60 seconds
  mqtt_client.setSocketTimeout(30); // Set socket timeout
  
  // Connect to MQTT
  connectMQTT();
  
  // Ready message
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter Password:");
  
  Serial.println("🚀 Smart Door System Ready!");
  publishStatus("System started and ready");
}

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
  Serial.print("LDR avg: "); Serial.println(ldrValue);

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

  // ----- Flame Sensor -----
  int flameValue = digitalRead(flamePin);
  if (flameValue == LOW) {
    flameEmergency();
  }

  // ----- Keypad input -----
  handleKeypad();
  
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

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) { // Increased timeout
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("✅ WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    lcd.setCursor(0,1);
    lcd.print("WiFi Connected  ");
    delay(2000);
  } else {
    Serial.println("❌ WiFi connection failed!");
    lcd.setCursor(0,1);
    lcd.print("WiFi Failed     ");
    delay(2000);
    // Try again
    ESP.restart();
  }
}

// ===== MQTT SETUP (IMPROVED) =====
void connectMQTT() {
  int attempts = 0;
  
  while (!mqtt_client.connected() && attempts < 5) {
    Serial.print("🔗 Connecting to MQTT... Attempt ");
    Serial.println(attempts + 1);
    
    lcd.setCursor(0,1);
    lcd.print("MQTT connecting.");
    
    // Generate unique client ID with MAC address
    String clientId = "smartdoor_";
    clientId += String(random(0xffff), HEX);
    clientId += "_";
    clientId += WiFi.macAddress().substring(9); // Last part of MAC
    clientId.replace(":", "");
    
    Serial.print("Client ID: ");
    Serial.println(clientId);
    
    // Set last will and testament
    String willTopic = topic_status;
    String willMessage = "{\"message\":\"ESP32 disconnected unexpectedly\",\"timestamp\":" + String(millis()) + "}";
    
    if (mqtt_client.connect(clientId.c_str(), mqtt_username, mqtt_password, 
                           willTopic.c_str(), 1, true, willMessage.c_str())) {
      Serial.println(" ✅ Connected!");
      
      lcd.setCursor(0,1);
      lcd.print("MQTT Connected  ");
      delay(1000);
      
      // Subscribe to door control topic
      if (mqtt_client.subscribe(topic_door_control)) {
        Serial.println("📡 Subscribed to door control");
      } else {
        Serial.println("❌ Failed to subscribe to door control");
      }
      
      // Announce that we're online
      publishStatus("ESP32 connected to secure MQTT");
      
      return; // Success, exit the loop
      
    } else {
      Serial.print(" ❌ failed, rc=");
      Serial.print(mqtt_client.state());
      
      // Print detailed error information
      switch (mqtt_client.state()) {
        case -4:
          Serial.println(" - Connection timeout");
          break;
        case -3:
          Serial.println(" - Connection lost");
          break;
        case -2:
          Serial.println(" - Connect failed");
          break;
        case -1:
          Serial.println(" - Disconnected");
          break;
        case 1:
          Serial.println(" - Bad protocol");
          break;
        case 2:
          Serial.println(" - Bad client ID");
          break;
        case 3:
          Serial.println(" - Unavailable");
          break;
        case 4:
          Serial.println(" - Bad credentials");
          break;
        case 5:
          Serial.println(" - Unauthorized");
          break;
        default:
          Serial.println(" - Unknown error");
      }
      
      lcd.setCursor(0,1);
      lcd.print("MQTT Failed     ");
      attempts++;
      delay(5000);
    }
  }
  
  if (!mqtt_client.connected()) {
    Serial.println("❌ MQTT connection failed after all attempts");
    lcd.setCursor(0,1);
    lcd.print("MQTT Failed     ");
  }
}

// ===== MQTT CALLBACK (Receive Messages) =====
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📨 Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);
  
  // Parse JSON message
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

// ===== SEND SENSOR DATA VIA MQTT (IMPROVED) =====
void sendSensorData(int ldrValue) {
  if (!mqtt_client.connected()) {
    Serial.println("⚠️ MQTT not connected, skipping sensor data");
    return;
  }
  
  DynamicJsonDocument doc(1024);
  
  doc["timestamp"] = millis();
  doc["door_locked"] = doorLocked;
  doc["light_level"] = (ldrValue == -1) ? 0 : ldrValue;
  doc["is_dark"] = isDark;
  doc["motion_detected"] = personPresent;
  doc["fire_detected"] = (digitalRead(flamePin) == LOW);
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["uptime"] = millis() / 1000;
  doc["ldr_status"] = (ldrValue == -1) ? "error" : "ok";
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (mqtt_client.publish(topic_sensor_data, jsonString.c_str(), true)) { // Added retain flag
    Serial.println("📤 Sensor data sent - LDR: " + String(ldrValue));
  } else {
    Serial.println("❌ Failed to send sensor data - MQTT State: " + String(mqtt_client.state()));
  }
}

// ===== PUBLISH ALERT (IMPROVED) =====
void publishAlert(String type, String message) {
  if (!mqtt_client.connected()) {
    Serial.println("⚠️ MQTT not connected, skipping alert");
    return;
  }
  
  DynamicJsonDocument doc(512);
  
  doc["type"] = type;
  doc["message"] = message;
  doc["timestamp"] = millis();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (mqtt_client.publish(topic_alerts, jsonString.c_str())) {
    Serial.println("🚨 Alert sent: " + message);
  } else {
    Serial.println("❌ Failed to send alert");
  }
}

// ===== PUBLISH STATUS (IMPROVED) =====
void publishStatus(String message) {
  if (!mqtt_client.connected()) {
    Serial.println("⚠️ MQTT not connected, skipping status");
    return;
  }
  
  DynamicJsonDocument doc(512);
  
  doc["message"] = message;
  doc["timestamp"] = millis();
  doc["uptime"] = millis() / 1000;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (mqtt_client.publish(topic_status, jsonString.c_str(), true)) { // Added retain flag
    Serial.println("📊 Status: " + message);
  } else {
    Serial.println("❌ Failed to send status");
  }
}

// ... [Rest of the functions remain the same] ...

// ===== HANDLE DOOR CONTROL COMMANDS =====
void handleDoorControl(DynamicJsonDocument& doc) {
  String command = doc["command"];
  String receivedPassword = doc["password"];
  String user = doc["user"];
  
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
    Serial.println("🛡️ System armed");
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

void handlePasswordChange(DynamicJsonDocument& doc) {
  String currentPassword = doc["current_password"];
  String newPassword = doc["new_password"];
  String user = doc["user"];
  
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
  delay(3000);
  
  singleBeep();
  singleBeep();
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter Password:");
  
  publishStatus("Password changed successfully by " + user);
}

void remoteDoorOpen(String user) {
  Serial.println("🔓 Remote door open for: " + user);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Remote Access");
  lcd.setCursor(0,1);
  lcd.print("Opening...");
  
  singleBeep();

  myservo.write(cw);
  delay(quarterTurnTime);
  myservo.write(stopPos);
  
  doorLocked = false;
  publishStatus("Door opened remotely by " + user);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Door Open");
  lcd.setCursor(0,1);
  lcd.print("Remote: " + user.substring(0, 8));

  delay(5000);

  myservo.write(ccw);
  delay(quarterTurnTime);
  myservo.write(stopPos);
  
  doorLocked = true;
  publishStatus("Door closed automatically");

  singleBeep();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Door Closed");
  delay(1500);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter Password:");
}

void emergencyDoorOpen(String user) {
  Serial.println("🚨 Emergency door open by: " + user);
  publishAlert("emergency_open", "Emergency door open activated by " + user);
  remoteDoorOpen(user);
}

void openDoor() {
  Serial.println("Correct password → Door OPEN");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Access Granted");
  lcd.setCursor(0,1);
  lcd.print("Door Opening...");
  
  singleBeep();

  myservo.write(cw);
  delay(quarterTurnTime);
  myservo.write(stopPos);
  
  doorLocked = false;
  publishStatus("Door opened with keypad password");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Door Open");
  lcd.setCursor(0,1);
  lcd.print("Welcome!");

  delay(5000);

  myservo.write(ccw);
  delay(quarterTurnTime);
  myservo.write(stopPos);
  
  doorLocked = true;
  publishStatus("Door closed automatically");

  singleBeep();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Door Closed");
  delay(1500);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter Password:");
}

void wrongPasswordBeep() {
  publishAlert("wrong_password", "Wrong password attempted on keypad");
  
  for (int i = 0; i < 3; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(180);
    digitalWrite(buzzerPin, LOW);
    delay(180);
  }
}

void singleBeep() {
  digitalWrite(buzzerPin, HIGH);
  delay(250);
  digitalWrite(buzzerPin, LOW);
}

void showLightStatus(String status) {
  lcd.setCursor(11, 1);  
  lcd.print("     ");
  lcd.setCursor(11, 1);  
  lcd.print(status);
}

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

  myservo.write(cw);
  delay(quarterTurnTime);
  myservo.write(stopPos);
  
  doorLocked = false;
  publishStatus("Door opened due to fire emergency");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Door Open");
  lcd.setCursor(0,1);
  lcd.print("Evacuate Now!");

  delay(10000);

  myservo.write(ccw);
  delay(quarterTurnTime);
  myservo.write(stopPos);
  
  doorLocked = true;
  publishStatus("Door closed after fire emergency");

  singleBeep();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Door Closed");
  delay(1500);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter Password:");
}

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
    Serial.println("⚠️ LDR: No valid readings!");
    return -1;
  }
  
  int average = sum / validReadings;
  Serial.println("📊 LDR: " + String(validReadings) + " readings, avg: " + String(average));
  return average;
}