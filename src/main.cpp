#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>

// WiFi settings
const char* ssid = "Lan Huong 5G";
const char* password = "cohuong92";

// MQTT settings
const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;
const char* mqttTopic = "esp8266/pirsensor";

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

typedef std::function<void()> TimerCallbackFunction;
Ticker timer;

// WiFi and MQTT clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void motionDetectedISR();
void timer1Callback();
void startTimer(int, TimerCallbackFunction);
void stopTimer();
void mqttCallback(char* , byte* , unsigned int );
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
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  Serial.println("WiFi connected");

  // Initialize MQTT client
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);

  // Attach interrupt for inputPin
  attachInterrupt(digitalPinToInterrupt(inputPin), motionDetectedISR, RISING);
}

void loop() {
  // Check for MQTT connection
  if (!mqttClient.connected()) {
    connectToMqtt();
  }
  mqttClient.loop();

  // Handle motion detection
  if (motionDetected) {
    output1State = false;
    digitalWrite(outputPin1, output1State);
    startTimer(1500, timer1Callback);
    motionDetected = false;
    publishToMqtt("motion_detected");
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

void motionDetectedISR() {
  motionDetected = true;
}

void timer1Callback() {
  output1State = true;
  digitalWrite(outputPin1, output1State);
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
}


void connectToMqtt() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (mqttClient.connect("ESP8266Client")) {
      Serial.println("MQTT client connected");
      mqttClient.subscribe(mqttTopic);
    } else {
      Serial.println("MQTT connection failed");
      delay(5000);
    }
  }
}

void publishToMqtt(String message) {
  mqttClient.publish(mqttTopic, message.c_str());
}