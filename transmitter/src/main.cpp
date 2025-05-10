#include <Arduino.h>
#include <ESPNowCam.h>
#include <drivers/CamAIThinker.h>

#include "esp_system.h"
#include "esp_log.h"
#include "mbedtls/aes.h"

#define QUALITY 16
#define IV_SIZE 16
#define MAX_JPEG_SIZE 5000
#define QUARTER_RES // enable for 1/4 resolution, 160x120

CamAIThinker Camera;
ESPNowCam radio;

// 128-bit AES key
const uint8_t key[16] = {
  0xDE, 0xAD, 0xBE, 0xEF,
  0x01, 0x02, 0x03, 0x04,
  0x11, 0x22, 0x33, 0x44,
  0x55, 0x66, 0x77, 0x88
};

static uint8_t payload[MAX_JPEG_SIZE + IV_SIZE];

void generateIV(uint8_t *iv) {
  for (int i = 0; i < IV_SIZE; i++) {
    iv[i] = esp_random() & 0xFF;
  }
}

bool encryptAES_CTR(const uint8_t *input, uint8_t *output, size_t len, const uint8_t *key, const uint8_t *iv) {
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);

  int ret = mbedtls_aes_setkey_enc(&aes, key, 128);
  if (ret != 0) {
    mbedtls_aes_free(&aes);
    return false;
  }

  // Internal state
  uint8_t stream_block[16];
  size_t nc_off = 0;
  uint8_t iv_copy[16];
  memcpy(iv_copy, iv, 16);

  ret = mbedtls_aes_crypt_ctr(&aes, len, &nc_off, iv_copy, stream_block, input, output);
  mbedtls_aes_free(&aes);

  return (ret == 0);
}

void encryptAndSendJPEG(const uint8_t* data, size_t len) {
  if (len > MAX_JPEG_SIZE) {
    Serial.println("[ERROR] JPEG too large");
    return;
  }

  // Generate IV and place in payload
  generateIV(payload);

  // Encrypt JPEG directly into payload after IV
  if (!encryptAES_CTR(data, payload + IV_SIZE, len, key, payload)) {
    Serial.println("[ERROR] AES encryption failed");
    return;
  }

  radio.sendData(payload, len + IV_SIZE);
}

void processFrame() {
  if (Camera.get()) {
    uint8_t *out_jpg = NULL;
    size_t out_jpg_len = 0;

    frame2jpg(Camera.fb, QUALITY, &out_jpg, &out_jpg_len);
    if (out_jpg && out_jpg_len > 0) {
      encryptAndSendJPEG(out_jpg, out_jpg_len);
    }

    free(out_jpg);
    Camera.free();
  } else {
    Serial.println("[ERROR] Failed to capture image from camera.");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n[INFO] Starting ESP32Cam Transmitter...");

  if (!psramFound()) {
    Serial.println("[WARNING] PSRAM not found. May affect JPEG size limits.");
  }

  radio.init();

  #ifdef QUARTER_RES
    Camera.config.frame_size = FRAMESIZE_QQVGA;
  #else
    Camera.config.frame_size = FRAMESIZE_QVGA;
  #endif
  // Camera.config.jpeg_quality = QUALITY;
  Camera.config.fb_count = 1;

  if (!Camera.begin()) {
    Serial.println("[ERROR] Camera initialization failed.");
    while (true) delay(1000);
  }

  delay(500);
}

void loop() {
  processFrame();
}
