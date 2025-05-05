#include <Arduino.h>
#include <ESPNowCam.h>

// CYD imports
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <TFT_eSPI.h>

#include <JPEGDecoder.h>

// The CYD touch uses some non default
// SPI pins
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

// function declarations
void onDataReady(uint32_t);
void renderJPEG(int, int);

uint8_t *frame_buffer;
int32_t width, height;

SPIClass display = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);

TFT_eSPI tft = TFT_eSPI();

ESPNowCam radio;

void setup() {
  Serial.begin(115200);

  // Start the SPI for the touch screen and init the TS library
  display.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(display);
  ts.setRotation(1);

  // Start the tft display and set it to black
  tft.init();
  tft.setRotation(1); //This is the display in landscape
  tft.startWrite();

  // Clear the screen before writing to it
  tft.fillScreen(TFT_BLACK);

  width = tft.width();
  height = tft.height();

  if(psramFound()){
    size_t psram_size = esp_spiram_get_size() / 1048576;
    Serial.printf("PSRAM size: %dMb\r\n", psram_size);
  }

  // BE CAREFUL WITH IT, IF JPG LEVEL CHANGES, INCREASE IT
  frame_buffer = (uint8_t*) malloc(20000 * sizeof( uint8_t ) ) ;

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
  // loop code
}

void onDataReady(uint32_t length) {
  Serial.print("Data received: ");
  Serial.println(length);
  JpegDec.decodeArray(frame_buffer, length);
  renderJPEG(0, 0);
  // Serial.println(F("==============="));
  // Serial.println(F("JPEG image info"));
  // Serial.println(F("==============="));
  // Serial.print(F(  "Width      :")); Serial.println(JpegDec.width);
  // Serial.print(F(  "Height     :")); Serial.println(JpegDec.height);
  // Serial.print(F(  "Components :")); Serial.println(JpegDec.comps);
  // Serial.print(F(  "MCU / row  :")); Serial.println(JpegDec.MCUSPerRow);
  // Serial.print(F(  "MCU / col  :")); Serial.println(JpegDec.MCUSPerCol);
  // Serial.print(F(  "Scan type  :")); Serial.println(JpegDec.scanType);
  // Serial.print(F(  "MCU width  :")); Serial.println(JpegDec.MCUWidth);
  // Serial.print(F(  "MCU height :")); Serial.println(JpegDec.MCUHeight);
  // Serial.println(F("==============="));
}


//====================================================================================
//   Decode and paint onto the TFT screen
//====================================================================================
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
      tft.setAddrWindow(mcu_x, mcu_y, mcu_x + win_w - 1, mcu_y + win_h - 1);
      // push all the image block pixels to the screen
      while (mcu_pixels--) tft.pushColor(*pImg++); // Send to TFT 16 bits at a time
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
