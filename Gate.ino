#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include <base64.h>
#include <Servo.h>

//CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH              4

#define ECHO_PIN          13
#define TRIG_PIN          15
#define SERVO_PIN         12


const char* ssid = "WIFI_SSID";
const char* password = "Password Here";
const char* deviceID = "ESP32CAM01"; // for identification

bool gate = false;

//Your Domain name with URL path or IP address with path
const char* serverName = "http://192.168.0.1/api-endpoint";

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

StaticJsonDocument<200> doc;
Servo myservo;

void configInitCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  } else {
    Serial.println("Camera is working");
  }

  // Drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
}


void setup() {
  Serial.begin(115200);
  pinMode(FLASH, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Config and init the camera
  configInitCamera();
  myservo.attach(SERVO_PIN, 2, 0, 180);
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");
}

void loop() {
  //Operate every timerDelay
  if ((millis() - lastTime) > timerDelay) {
    long distanceCm = checkDistance();
    Serial.print("Distance is : ");
    Serial.println(distanceCm);
    if (distanceCm <= 30 && gate == false) {
      Serial.println("Nearby Object Detected!");
      if (WiFi.status() == WL_CONNECTED) {
        bool sent = httpPOSTRequest(serverName, distanceCm);
        Serial.println(sent);
        if (sent) {
          openGate();
        }
      }
      else {
        Serial.println("WiFi Disconnected");
      }
    } else if (gate == true && distanceCm >= 30) {
      closeGate();
    }
    lastTime = millis();
  }
}

String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;

  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "{}";

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();
  return payload;
}

bool httpPOSTRequest(const char* serverName, float distance) {
  WiFiClient client;
  HTTPClient http;
  DynamicJsonDocument postDoc(32768); // For VGA
  StaticJsonDocument<200> doc;
  camera_fb_t * fb = NULL;

  // Take Picture with Camera
  digitalWrite(FLASH, HIGH);
  delay(200);
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return "";
  }
  delay(200);
  digitalWrite(FLASH, LOW);
  const char* encoded = base64::encode(fb->buf, fb->len).c_str();
  // Free resources
  esp_camera_fb_return(fb);
  fb = NULL;

  Serial.println(encoded);
  postDoc["device"] = deviceID;
  postDoc["distance"] = distance;
  postDoc["image"] = encoded;
  String body = postDoc.as<String>();

  http.begin(client, serverName);
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(body);
  String payload = "{}";

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  Serial.println(payload);
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return false;
  }
  String stats = doc["status"];
  Serial.println(stats);
  if (stats == "OKE") {
    Serial.println("Berhasil");
    return true;
  }
  http.end();
  return false;
}

long checkDistance() {
  long duration, distanceCm;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distanceCm = duration / 28.86 / 2 ;
  return distanceCm;
}

void openGate() {
  Serial.println("Opening Gate");
  myservo.write(90);
  gate = true;
  delay(1000);
}

void closeGate() {
  Serial.println("Closing Gate");
  myservo.write(0);
  gate = false;
  delay(1000);
}
