#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <EEPROM.h>

// WiFi settings
const char* ssid = "Lan Huong 5G";
const char* password = "cohuong92";

// AP settings
const char* ap_ssid = "ESP8266-E12-AP";
const char* ap_password = "12345678";

// MQTT settings
const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;
const char* mqttTopic_PC2uC = "esp8266/PC2uC";
const char* mqttTopic_uC2PC = "esp8266/uC2PC";

// Pin definitions
const int inputPin = 2;
const int outputPin1 = 0;
const int outputPin2 = 4;
const int buttonPin = 5;

// Global variables
volatile bool motionDetected = false;
volatile bool buttonPressed = false;
volatile bool output1State = true;
volatile bool output2State = false;
volatile bool timerExpired = false;

typedef std::function<void()> TimerCallbackFunction;
Ticker timer;

// WiFi and MQTT clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void motionDetectedISR();
void timer1Callback();
void startTimer(int, TimerCallbackFunction);
void stopTimer();
void mqttCallback(char*, byte*, unsigned int);
void connectToMqtt();
void publishToMqtt(String);

void setup() {
  // Initialize serial port
  Serial.begin(9600);

  // Initialize pins
  pinMode(inputPin, INPUT);
  pinMode(outputPin1, OUTPUT);
  pinMode(outputPin2, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // Set initial pin states
  digitalWrite(outputPin1, output1State);
  digitalWrite(outputPin2, output2State);

  // Initialize WiFi connection
  WiFi.mode(WIFI_AP); // Set WiFi mode to AP
  WiFi.softAP(ap_ssid, ap_password); // Set AP credentials
  Serial.println("AP created with SSID: " + String(ap_ssid) + " and password: " + String(ap_password));

  // Initialize MQTT client
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);

  // Attach interrupt for inputPin
  attachInterrupt(digitalPinToInterrupt(inputPin), motionDetectedISR, RISING);
}

void loop() {
  // Check for MQTT connection
  if (!mqttClient.connected()) {
    //connectTo continue from the previous answer, here is the rest of the modified code:

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    mqttClient.connect("ESP8266Client");
  } else {
    Serial.println("AP mode, IP address: " + WiFi.softAPIP().toString());
    mqttClient.connect("ESP8266Client");
  }
  mqttClient.loop();

  // Handle motion detection
  if (motionDetected) {
    output1State = false;
    digitalWrite(outputPin1, output1State);
    startTimer(1500, timer1Callback);
    motionDetected = false;
    publishToMqtt("motion_detected");
    mqttClient.publish(mqttTopic_uC2PC, "Motion Detected"); 
    // send message to topic esp8266/uC2PC
  }
  // Handle timer expiry
  if (timerExpired) {
    timerExpired = false;
    if (motionDetected) {
      mqttClient.publish(mqttTopic_uC2PC, "Detect People"); 
      // send message to topic esp8266/uC2PC
    } else {
      mqttClient.publish(mqttTopic_uC2PC, "Objects just pass by"); 
      // send message to topic esp8266/uC2PC
    }
  }
  unsigned long lastPublishTime = 0;
  if (!motionDetected && !timerExpired) {
    if (millis() - lastPublishTime > 2000) {
      lastPublishTime = millis();
      mqttClient.publish(mqttTopic_uC2PC, "Not detected"); // gửi thông điệp tới topic esp8266/uC2PC
    }
  }

  // Handle button press
  if (buttonPressed) {
    output2State = !output2State;
    digitalWrite(outputPin2, output2State);
    buttonPressed = false;
    if (output2State) {
      attachInterrupt(digitalPinToInterrupt(inputPin), motionDetectedISR, RISING);
    } else {
      detachInterrupt(digitalPinToInterrupt(inputPin));
    }
    publishToMqtt("button_pressed");
  }
}
}

void motionDetectedISR() {
  motionDetected = true;
}

void timer1Callback() {
  output1State = true;
  digitalWrite(outputPin1, output1State);
  timerExpired = true;
}

void startTimer(int duration, TimerCallbackFunction callback) {
 timer.attach_ms(duration, callback);
}

void stopTimer() {
  timer.detach();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;

  // Convert payload to string
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("MQTT message received: " + message);

  if (message == "turn_off") {
    output2State = false;
    digitalWrite(outputPin2, output2State);
    detachInterrupt(digitalPinToInterrupt(inputPin));
    publishToMqtt("output2_turned_off");
  }
  if (message == "turn_on") {
    output2State = true;
    digitalWrite(outputPin2, output2State);
    detachInterrupt(digitalPinToInterrupt(inputPin));
    publishToMqtt("output2_turned_on");
  }
  // Check if message contains SSID and password
  if (message.startsWith("SSID=") && message.indexOf("&PWD=") != -1) {
    // Extract SSID and password from message
    String ssid = message.substring(5, message.indexOf("&PWD="));
    String password = message.substring(message.indexOf("&PWD=") + 5);

    // Save SSID and password to EEPROM
    EEPROM.begin(512);
    for (unsigned int i = 0; i < ssid.length(); i++) {
      EEPROM.write(i, ssid[i]);
    }
    for (unsigned int i = 0; i < password.length(); i++) {
      EEPROM.write(i + ssid.length() + 1, password[i]);
    }
    EEPROM.commit();
    EEPROM.end();

    Serial.println("SSID and password saved to EEPROM");

    // Restart ESP8266 toapply new WiFi credentials
    ESP.restart();
  }
}

void connectToMqtt() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi connected");
      mqttClient.connect("ESP8266Client");
    } else {
      Serial.println("AP mode, IP address: " + WiFi.softAPIP().toString());
      mqttClient.connect("ESP8266Client");
    }
    delay(5000);
  }
  mqttClient.subscribe(mqttTopic_PC2uC);
}

void publishToMqtt(String message) {
  mqttClient.publish(mqttTopic_PC2uC, message.c_str());
}