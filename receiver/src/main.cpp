#include <Arduino.h>
#include <ESPNowCam.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <TFT_eSPI.h>
#include <JPEGDecoder.h>

#include "mbedtls/aes.h"

#define IV_SIZE 16

// Static 128-bit key
const uint8_t key[16] = {
  0xDE, 0xAD, 0xBE, 0xEF,
  0x01, 0x02, 0x03, 0x04,
  0x11, 0x22, 0x33, 0x44,
  0x55, 0x66, 0x77, 0x88
};

#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

void onDataReady(uint32_t length);
void renderJPEG(int, int);

uint8_t *frame_buffer;
int32_t width, height;

SPIClass display = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
TFT_eSPI tft = TFT_eSPI();
ESPNowCam radio;

bool decryptAES_CTR(const uint8_t *input, uint8_t *output, size_t len, const uint8_t *key, const uint8_t *iv) {
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);

  int ret = mbedtls_aes_setkey_enc(&aes, key, 128);
  if (ret != 0) {
    mbedtls_aes_free(&aes);
    return false;
  }

  uint8_t stream_block[16];
  size_t nc_off = 0;
  uint8_t iv_copy[16];
  memcpy(iv_copy, iv, 16);

  ret = mbedtls_aes_crypt_ctr(&aes, len, &nc_off, iv_copy, stream_block, input, output);
  mbedtls_aes_free(&aes);
  return (ret == 0);
}

void setup() {
  Serial.begin(115200);

  display.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(display);
  ts.setRotation(1);

  tft.init();
  tft.setRotation(1);
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);

  width = tft.width();
  height = tft.height();

  if (psramFound()) {
    size_t psram_size = esp_spiram_get_size() / 1048576;
    Serial.printf("PSRAM size: %dMb\r\n", psram_size);
  }

  frame_buffer = (uint8_t*) malloc(40000);
  radio.setRecvBuffer(frame_buffer);
  radio.setRecvCallback(onDataReady);

  if (radio.init()) {
    tft.setTextSize(2);
    tft.drawString("Init success", width / 4, height / 2);
    Serial.println("ESPNow Init success");
  }

  delay(1000);
}

void loop() {
  // idle
}

void onDataReady(uint32_t length) {
  Serial.print("Data received: ");
  Serial.println(length);

  if (length <= IV_SIZE) {
    Serial.println("[ERROR] Frame too short to contain IV.");
    return;
  }

  uint8_t* iv = frame_buffer;
  uint8_t* encrypted_data = frame_buffer + IV_SIZE;
  size_t encrypted_len = length - IV_SIZE;

  // Decrypt into-place
  if (!decryptAES_CTR(encrypted_data, encrypted_data, encrypted_len, key, iv)) {
    Serial.println("[ERROR] AES decryption failed.");
    return;
  }

  if (!JpegDec.decodeArray(encrypted_data, encrypted_len)) {
    Serial.println("[ERROR] JPEG decode failed.");
    return;
  }

  renderJPEG(0, 0);
}

void renderJPEG(int xpos, int ypos) {
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  uint32_t min_w = mcu_w < max_x % mcu_w ? mcu_w : max_x % mcu_w;
  uint32_t min_h = mcu_h < max_y % mcu_h ? mcu_h : max_y % mcu_h;

  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  uint16_t *lineBuf = (uint16_t*)malloc(mcu_w * mcu_h * sizeof(uint16_t));

  uint32_t drawTime = millis();
  max_x += xpos;
  max_y += ypos;

  while (JpegDec.read()) {
    pImg = JpegDec.pImage;

    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    if ((mcu_x + win_w) <= tft.width() && (mcu_y + win_h) <= tft.height()) {
      // Byte swap RGB565 values if needed
      for (uint32_t i = 0; i < win_w * win_h; i++) {
        uint16_t color = pImg[i];
        lineBuf[i] = (color << 8) | (color >> 8);  // Swap bytes
      }

      tft.pushImage(mcu_x, mcu_y, win_w, win_h, lineBuf);
    } else if ((mcu_y + win_h) >= tft.height()) {
      JpegDec.abort();
    }
  }

  free(lineBuf);

  drawTime = millis() - drawTime;
  Serial.print("Total render time was    : ");
  Serial.print(drawTime);
  Serial.println(" ms");
  Serial.println("=====================================");
}
