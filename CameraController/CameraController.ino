#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "fb_gfx.h"
#include "img_converters.h"
#include "soc/rtc_cntl_reg.h" //disable brownout problems
#include "soc/soc.h"          //disable brownout problems
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <base64.h>

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define BUTTON_PIN 14         // Button pin
#define STREAM_DURATION 10000 // Stream duration in milliseconds (10 seconds)

#define WEBSOCKET_URL "ws://192.168.166.75:12345/"

camera_fb_t *fb = NULL;
size_t _jpg_buf_len = 0;
uint8_t *_jpg_buf = NULL;
uint8_t state = 0;

using namespace websockets;
WebsocketsClient client;

unsigned long streamStartTime = 0; // To track when streaming started
bool isStreaming = false;          // Streaming state

///////////////////////////////////INITIALIZE
///FUNCTIONS///////////////////////////////////
esp_err_t init_camera() {
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
  // init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_XGA;
    config.jpeg_quality = 20;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 20;
    config.fb_count = 1;
  }
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return err;
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_brightness(s, 2);
  Serial.println("Cam Success init");
  return ESP_OK;
};

esp_err_t init_wifi(int maxRetries = 20) {
  WiFi.begin("sam_zhang", "aaabbb1234");
  Serial.println("Starting Wifi");

  int retryCount = 0;

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retryCount++;

    if (retryCount >= maxRetries) {
      Serial.println("\nWiFi connection failed. Restarting ESP...");
      return ESP_FAIL;
    }
  }

  Serial.println("\nWiFi connected");
  return ESP_OK;
}

esp_err_t connect_to_websocket() {
  Serial.println("Connecting to websocket");

  bool connected = client.connect(WEBSOCKET_URL);

  while (!connected) {
    delay(500);
    Serial.print(".");
    connected = client.connect(WEBSOCKET_URL);
  }

  Serial.println("Websocket Connected!");
  return ESP_OK;
}

///////////////////////////////////SETUP///////////////////////////////////
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector

  Serial.begin(115200);

  init_camera();

  if (init_wifi() == ESP_FAIL) {
    ESP.restart();
  };

  connect_to_websocket();

  pinMode(BUTTON_PIN, INPUT_PULLUP); // Configure button pin
}

///////////////////////////////////MAIN LOOP////////////////////////////////
void loop() {
  // Keep alive websocket
  if (!client.available()) {
    connect_to_websocket();
  } else {
    client.poll();
  }

  bool buttonIsPressed = digitalRead(BUTTON_PIN) == LOW;

  if (buttonIsPressed && !isStreaming) {
    delay(50);                            // Debounce delay
    if (digitalRead(BUTTON_PIN) == LOW) { // Confirm button press
      isStreaming = true;
      streamStartTime = millis();

      String startCommandBase64 = base64::encode((uint8_t *)"start", 5);
      DynamicJsonDocument startDoc(256);
      startDoc["type"] = "command";
      startDoc["data"] = startCommandBase64;
      String startCommandJson;
      serializeJson(startDoc, startCommandJson);
      client.send(startCommandJson);
    }
  }

  if (isStreaming) {
    if (millis() - streamStartTime < STREAM_DURATION) {
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
        ESP.restart();
      }

      String base64ImageBytes = base64::encode((uint8_t *)fb->buf, fb->len);
      const char *base64ImageUTF8 = base64ImageBytes.c_str();

      DynamicJsonDocument doc(4096);
      doc["type"] = "image";
      doc["data"] = base64ImageUTF8;

      String jsonString;
      serializeJson(doc, jsonString);

      client.send(jsonString);

      esp_camera_fb_return(fb);
    } else {
      isStreaming = false;

      // Send "end" command
      String endCommandBase64 = base64::encode((uint8_t *)"end", 3);
      DynamicJsonDocument endDoc(256);
      endDoc["type"] = "command";
      endDoc["data"] = endCommandBase64;
      String endCommandJson;
      serializeJson(endDoc, endCommandJson);
      client.send(endCommandJson);
    }
  }
}
