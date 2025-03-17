#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <algorithm>

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

#define EYE_WIDTH 30
#define EYE_HEIGHT 50
#define EYE_SPACING 80

#define BLINK_FRAMES 20. 
#define FRAME_DELAY 4
#define OPEN_TIME_MIN 1000
#define OPEN_TIME_MAX 4000

#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 135

#define EYE_COLOR ST77XX_WHITE
#define BG_COLOR ST77XX_BLACK

int blinkState = 0;
bool isBlinking = false;
unsigned long lastBlinkTime = 0;
unsigned long openTime = OPEN_TIME_MIN;

int16_t eyeY;
int16_t eyePositions[2];

uint16_t *buffer = NULL;
int bufferWidth = 0;
int bufferHeight = 0;
int bufferX = 0;
int bufferY = 0;

void bufferFillEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry,
                       uint16_t color) {
  x0 = x0 - bufferX;
  y0 = y0 - bufferY;

  for (int16_t y = -ry; y <= ry; y++) {
    if (y0 + y >= 0 && y0 + y < bufferHeight) {
      int16_t x = sqrt(1.0 - (float)(y * y) / (ry * ry)) * rx;

      int16_t startX = max(0, x0 - x);
      int16_t endX = min(bufferWidth - 1, x0 + x);

      for (int16_t i = startX; i <= endX; i++) {
        buffer[i + (y0 + y) * bufferWidth] = color;
      }
    }
  }
}

void clearBuffer() {
  for (int i = 0; i < bufferWidth * bufferHeight; i++) {
    buffer[i] = BG_COLOR;
  }
}

void flushBuffer() {
  tft.startWrite();
  tft.setAddrWindow(bufferX, bufferY, bufferWidth, bufferHeight);

  tft.writePixels(buffer, bufferWidth * bufferHeight);
  tft.endWrite();
}

void setup(void) {
  Serial.begin(115200);

  SPI.setFrequency(80000000);
  setCpuFrequencyMhz(240);

  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  tft.init(DISPLAY_HEIGHT, DISPLAY_WIDTH);
  tft.setRotation(3);

  tft.fillScreen(BG_COLOR);

  int16_t startX = (tft.width() - EYE_SPACING) / 2;
  eyeY = tft.height() / 2;

  for (int i = 0; i < 2; i++) {
    eyePositions[i] = startX + (i * EYE_SPACING);
  }

  bufferX = eyePositions[0] - EYE_WIDTH - 5;
  bufferY = eyeY - EYE_HEIGHT - 5;
  bufferWidth = eyePositions[1] - bufferX + EYE_WIDTH + 5;
  bufferHeight = (EYE_HEIGHT * 2) + 10;

  // Allocate memory for the buffer
  buffer = (uint16_t *)malloc(bufferWidth * bufferHeight * sizeof(uint16_t));
  if (!buffer) {
    Serial.println("Failed to allocate buffer memory!");
    while (1)
      ;
  }

  clearBuffer();
  for (int i = 0; i < 2; i++) {
    bufferFillEllipse(eyePositions[i], eyeY, EYE_WIDTH, EYE_HEIGHT, EYE_COLOR);
  }

  flushBuffer();
  randomSeed(analogRead(0));

  lastBlinkTime = millis();
  openTime = random(OPEN_TIME_MIN, OPEN_TIME_MAX);

  Serial.print("First blink in: ");
  Serial.print(openTime);
  Serial.println(" ms");
}

void loop() {
  unsigned long currentTime = millis();

  if (!isBlinking && (currentTime - lastBlinkTime >= openTime)) {
    Serial.println("Starting blink");
    isBlinking = true;
    blinkState = 0;
  }

  if (isBlinking) {
    float blinkProgress;

    if (blinkState < BLINK_FRAMES / 2) {
      blinkProgress = (float)blinkState / (BLINK_FRAMES / 2);
      blinkProgress = 1.0 - (blinkProgress * blinkProgress); // Linear easing
    } else {
      blinkProgress =
          (float)(blinkState - BLINK_FRAMES / 2) / (BLINK_FRAMES / 2);
      blinkProgress = blinkProgress * (2 - blinkProgress);
    }

    int16_t currentHeight =
        std::max((int16_t)1, (int16_t)(EYE_HEIGHT * blinkProgress));

    clearBuffer();
    for (int i = 0; i < 2; i++) {
      bufferFillEllipse(eyePositions[i], eyeY, EYE_WIDTH, currentHeight,
                        EYE_COLOR);
    }

    flushBuffer();
    blinkState++;

    if (blinkState >= BLINK_FRAMES) {
      Serial.println("Blink complete");
      isBlinking = false;
      lastBlinkTime = millis();

      openTime = random(OPEN_TIME_MIN, OPEN_TIME_MAX);

      Serial.print("Next blink in: ");
      Serial.print(openTime);
      Serial.println(" ms");

      if (random(10) == 0) { // 10% chance of double blink
        Serial.println("Scheduling quick double-blink");
        openTime = 300;
      }

      delay(openTime);
    }

    delay(FRAME_DELAY);
  } else {
    // prevent the cpu going nuts so when we are waiting only every 16ms
    delay(FRAME_DELAY * 4);
  }
}
