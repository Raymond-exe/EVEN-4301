#include <Arduino.h>
#include <ESPNowCam.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <TFT_eSPI.h>
#include <JPEGDecoder.h>
#include <Crypto.h>
#include <AES.h>
#include <CTR.h>

#define IV_SIZE 16

CTR<AES128> ctr;

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
    tft.drawString("ESPNow Init success", width / 6, height / 2);
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

  ctr.setKey(key, sizeof(key));
  ctr.setIV(iv, IV_SIZE);
  ctr.decrypt(encrypted_data, encrypted_data, encrypted_len);

  if (!JpegDec.decodeArray(encrypted_data, encrypted_len)) {
    Serial.println("[ERROR] JPEG decode failed.");
    return;
  }

  renderJPEG(0, 0);
}

// void renderJPEG(int xpos, int ypos) {
//   uint16_t *pImg;
//   uint16_t mcu_w = JpegDec.MCUWidth;
//   uint16_t mcu_h = JpegDec.MCUHeight;
//   uint32_t max_x = JpegDec.width;
//   uint32_t max_y = JpegDec.height;

//   uint32_t min_w = mcu_w < max_x % mcu_w ? mcu_w : max_x % mcu_w;
//   uint32_t min_h = mcu_h < max_y % mcu_h ? mcu_h : max_y % mcu_h;

//   uint32_t win_w = mcu_w;
//   uint32_t win_h = mcu_h;

//   uint32_t drawTime = millis();

//   while (JpegDec.read()) {
//     pImg = JpegDec.pImage;
//     int mcu_x = JpegDec.MCUx * mcu_w + xpos;
//     int mcu_y = JpegDec.MCUy * mcu_h + ypos;

//     if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
//     else win_w = min_w;
//     if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
//     else win_h = min_h;

//     // Allocate buffer for rotated MCU block
//     uint16_t rotated[win_w * win_h];

//     // Rotate 90° counter-clockwise into buffer
//     for (uint16_t row = 0; row < win_h; row++) {
//       for (uint16_t col = 0; col < win_w; col++) {
//         uint16_t src_idx = row * win_w + col;
//         uint16_t dst_idx = (win_w - 1 - col) * win_h + row;
//         rotated[dst_idx] = pImg[src_idx];
//       }
//     }

//     // Calculate rotated position on screen
//     uint16_t x_rot = tft.height() - (mcu_y + win_h);  // Vertical position
//     uint16_t y_rot = mcu_x;                           // Horizontal position

//     if ((x_rot + win_w <= tft.height()) && (y_rot + win_h <= tft.width())) {
//       tft.setAddrWindow(y_rot, x_rot, y_rot + win_h - 1, x_rot + win_w - 1);
//       tft.pushColors(rotated, win_w * win_h);
//     }
//   }

//   drawTime = millis() - drawTime;
//   Serial.print("Total render time (fast CCW): ");
//   Serial.print(drawTime);
//   Serial.println(" ms");
// }


//working but 300 ms render time

void renderJPEG(int xpos, int ypos) {

  // retrieve infomration about the image
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = mcu_w < max_x % mcu_w ? mcu_w : max_x % mcu_w;
  uint32_t min_h = mcu_h < max_y % mcu_h ? mcu_h : max_y % mcu_h;

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();

  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;

  // read each MCU block until there are no more
  while ( JpegDec.read()) {

    // save a pointer to the image block
    pImg = JpegDec.pImage;

    // calculate where the image block should be drawn on the screen
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    // check if the image block size needs to be changed for the right and bottom edges
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;

    // draw image block if it will fit on the screen
    if ( (mcu_x + win_w) <= width && (mcu_y + win_h) <= height) {
      // open a window onto the screen to paint the pixels into
      //tft.setAddrWindow(mcu_x, mcu_y, mcu_x + win_w - 1, mcu_y + win_h - 1);
      for (uint16_t row = 0; row < win_h; row++) {
        for (uint16_t col = 0; col < win_w; col++) {
          // Read pixel in row-major order
          uint16_t color = pImg[row * win_w + col];

          // Calculate rotated coordinates (90° counter-clockwise)
          uint16_t x_rot = mcu_y + row;
          uint16_t y_rot = width - 1 - (mcu_x + col); // width used because it was landscape

          if (x_rot < height && y_rot < width) {
            tft.drawPixel(y_rot, x_rot, color);
          }
        }
      }
    }

    // stop drawing blocks if the bottom of the screen has been reached
    // the abort function will close the file
    else if ( ( mcu_y + win_h) >= height) JpegDec.abort();

  }

  // calculate how long it took to draw the image
  drawTime = millis() - drawTime; // Calculate the time it took

  // print the results to the serial port
  Serial.print  ("Total render time was    : "); Serial.print(drawTime); Serial.println(" ms");
  Serial.println("=====================================");

}


// void renderJPEG(int xpos, int ypos) {

//   // retrieve infomration about the image
//   uint16_t *pImg;
//   uint16_t mcu_w = JpegDec.MCUWidth;
//   uint16_t mcu_h = JpegDec.MCUHeight;
//   uint32_t max_x = JpegDec.width;
//   uint32_t max_y = JpegDec.height;

//   // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
//   // Typically these MCUs are 16x16 pixel blocks
//   // Determine the width and height of the right and bottom edge image blocks
//   uint32_t min_w = mcu_w < max_x % mcu_w ? mcu_w : max_x % mcu_w;
//   uint32_t min_h = mcu_h < max_y % mcu_h ? mcu_h : max_y % mcu_h;

//   // save the current image block size
//   uint32_t win_w = mcu_w;
//   uint32_t win_h = mcu_h;

//   // record the current time so we can measure how long it takes to draw an image
//   uint32_t drawTime = millis();

//   // save the coordinate of the right and bottom edges to assist image cropping
//   // to the screen size
//   max_x += xpos;
//   max_y += ypos;

//   // read each MCU block until there are no more
//   while ( JpegDec.read()) {

//     // save a pointer to the image block
//     pImg = JpegDec.pImage;

//     // calculate where the image block should be drawn on the screen
//     int mcu_x = JpegDec.MCUx * mcu_w + xpos;
//     int mcu_y = JpegDec.MCUy * mcu_h + ypos;

//     // check if the image block size needs to be changed for the right and bottom edges
//     if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
//     else win_w = min_w;
//     if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
//     else win_h = min_h;

//     // calculate how many pixels must be drawn
//     uint32_t mcu_pixels = win_w * win_h;

//     // draw image block if it will fit on the screen
//     if ( (mcu_x + win_w) <= width && (mcu_y + win_h) <= height) {
//       // open a window onto the screen to paint the pixels into
//       //tft.setAddrWindow(mcu_x, mcu_y, mcu_x + win_w - 1, mcu_y + win_h - 1);
//       tft.setAddrWindow(mcu_x, mcu_y, mcu_x + win_w - 1, mcu_y + win_h - 1);
//       // push all the image block pixels to the screen
//       while (mcu_pixels--) tft.pushColor(*pImg++); // Send to TFT 16 bits at a time
//     }

//     // stop drawing blocks if the bottom of the screen has been reached
//     // the abort function will close the file
//     else if ( ( mcu_y + win_h) >= height) JpegDec.abort();

//   }

//   // calculate how long it took to draw the image
//   drawTime = millis() - drawTime; // Calculate the time it took

//   // print the results to the serial port
//   Serial.print  ("Total render time was    : "); Serial.print(drawTime); Serial.println(" ms");
//   Serial.println("=====================================");

// }