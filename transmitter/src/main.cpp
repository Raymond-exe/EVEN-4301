#include <Arduino.h>
#include <ESPNowCam.h>
#include <drivers/CamAIThinker.h>
#include <Crypto.h>
#include <AES.h>
#include <CTR.h>

#define QUALITY 10
#define IV_SIZE 16

CamAIThinker Camera;
ESPNowCam radio;

CTR<AES128> ctr;

// Static 128-bit key (shared with receiver)
const uint8_t key[16] = {
  0xDE, 0xAD, 0xBE, 0xEF,
  0x01, 0x02, 0x03, 0x04,
  0x11, 0x22, 0x33, 0x44,
  0x55, 0x66, 0x77, 0x88
};

void generateIV(uint8_t *iv) {
  for (int i = 0; i < IV_SIZE; i++) {
    iv[i] = esp_random() & 0xFF;
  }
}

void encryptAndSendJPEG(uint8_t* data, size_t len) {
  uint8_t iv[IV_SIZE];
  generateIV(iv);

  ctr.setKey(key, sizeof(key));
  ctr.setIV(iv, IV_SIZE);
  ctr.encrypt(data, data, len);  // Encrypt in-place

  // Allocate buffer for IV + encrypted JPEG
  size_t total_len = IV_SIZE + len;
  uint8_t *payload = (uint8_t*) malloc(total_len);
  if (!payload) return;

  memcpy(payload, iv, IV_SIZE);
  memcpy(payload + IV_SIZE, data, len);

  radio.sendData(payload, total_len);

  free(payload);
}

void processFrame() {
  if (Camera.get()) {
    uint8_t *out_jpg = NULL;
    size_t out_jpg_len = 0;

    frame2jpg(Camera.fb, QUALITY, &out_jpg, &out_jpg_len);
    Serial.print("JPG length: ");
    Serial.println(out_jpg_len);

    encryptAndSendJPEG(out_jpg, out_jpg_len);

    free(out_jpg);
    Camera.free();
  } else {
    Serial.println("[ERROR] Failed to capture image from camera.");
    if (Camera.fb == nullptr) {
      Serial.println("  - Frame buffer is null.");
    } else {
      Serial.printf("  - Frame buffer size: %d x %d\n", Camera.fb->width, Camera.fb->height);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n[INFO] Starting ESP32Cam Transmitter...");

  delay(1000);

  if (psramFound()) {
    size_t psram_size = esp_spiram_get_size() / 1048576;
    Serial.printf("[INFO] PSRAM size: %dMB\n", psram_size);
  } else {
    Serial.println("[WARNING] PSRAM not found. Camera may fail due to insufficient memory.");
  }

  radio.init();

  Camera.config.frame_size = FRAMESIZE_QVGA;
  // Camera.config.jpeg_quality = 10;
  Camera.config.fb_count = 1;

  if (!Camera.begin()) {
    Serial.println("[ERROR] Camera initialization failed.");
    while (true) delay(1000);
  } else {
    Serial.println("[INFO] Camera initialization successful.");
  }

  delay(500);
}

void loop() {
  processFrame();
}