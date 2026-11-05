// ESP32 DevKit telemetry display: ST7789 240x240 on VSPI, Mega link on Serial2.
// Wiring: CS=5 DC=21 RST=22 SCK=18 MOSI=23, Mega TX1 (D18) -> GPIO16 (RX2), common GND.

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// BEGIN parseTelemetry
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define TELEMETRY_FIELDS 10
#define TELEMETRY_MAX_LINE 80
#define TELEMETRY_UID_MAX 20

struct Telemetry {
  int ir[5];
  char uid[TELEMETRY_UID_MAX + 1];
  long scans;
  int pwmA;
  int pwmB;
};

static bool parseRange(const char* s, long lo, long hi, long& out) {
  if (*s != '-' && (*s < '0' || *s > '9')) return false;
  char* end;
  errno = 0;
  long v = strtol(s, &end, 10);
  if (*end != '\0' || errno == ERANGE || v < lo || v > hi) return false;
  out = v;
  return true;
}

static bool validUid(const char* s) {
  size_t n = strlen(s);
  if (n == 1 && (s[0] == '-' || s[0] == '!')) return true;
  if (n == 0 || n > TELEMETRY_UID_MAX || n % 2 != 0) return false;
  for (size_t i = 0; i < n; i++) {
    bool hex = (s[i] >= '0' && s[i] <= '9') || (s[i] >= 'A' && s[i] <= 'F');
    if (!hex) return false;
  }
  return true;
}

bool parseTelemetry(const char* line, Telemetry& out) {
  if (strlen(line) >= TELEMETRY_MAX_LINE) return false;
  char buf[TELEMETRY_MAX_LINE];
  strcpy(buf, line);

  char* field[TELEMETRY_FIELDS];
  size_t count = 0;
  char* p = buf;
  for (;;) {
    if (count == TELEMETRY_FIELDS) return false;
    field[count++] = p;
    char* comma = strchr(p, ',');
    if (!comma) break;
    *comma = '\0';
    p = comma + 1;
  }
  if (count != TELEMETRY_FIELDS || strcmp(field[0], "T") != 0) return false;
  if (!validUid(field[6])) return false;

  Telemetry t;
  long v;
  for (int i = 0; i < 5; i++) {
    if (!parseRange(field[1 + i], 0, 1023, v)) return false;
    t.ir[i] = (int)v;
  }
  if (!parseRange(field[7], 0, 2147483647L, v)) return false;
  t.scans = v;
  if (!parseRange(field[8], -255, 255, v)) return false;
  t.pwmA = (int)v;
  if (!parseRange(field[9], -255, 255, v)) return false;
  t.pwmB = (int)v;
  strcpy(t.uid, field[6]);

  out = t;
  return true;
}
// END parseTelemetry

const int TFT_CS = 5;
const int TFT_DC = 21;
const int TFT_RST = 22;
const int SCREEN_W = 240;
const int SCREEN_H = 240;
const int ROW_HEIGHT = 20;
const int ROW_COUNT = 12;
const int COLS = 20;
const unsigned long LINK_TIMEOUT_MS = 1000;

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

char rowCache[ROW_COUNT][COLS + 1];

Telemetry latest;
bool haveData = false;
unsigned long lastValidMs = 0;

void drawRow(int row, const char* text) {
  char padded[COLS + 1];
  snprintf(padded, sizeof(padded), "%-*s", COLS, text);
  if (strcmp(padded, rowCache[row]) == 0) return;
  strcpy(rowCache[row], padded);
  tft.setCursor(0, row * ROW_HEIGHT + 2);
  tft.print(padded);
}

void render() {
  char text[COLS + 1];
  bool linkOk = haveData && millis() - lastValidMs < LINK_TIMEOUT_MS;

  for (int i = 0; i < 5; i++) {
    if (haveData) snprintf(text, sizeof(text), "IR A%d %4d", 3 + i, latest.ir[i]);
    else snprintf(text, sizeof(text), "IR A%d %4s", 3 + i, "--");
    drawRow(i, text);
  }

  if (haveData) snprintf(text, sizeof(text), "RFID scans: %ld", latest.scans);
  else snprintf(text, sizeof(text), "RFID scans: --");
  drawRow(5, text);

  if (!haveData) drawRow(6, "--");
  else if (strcmp(latest.uid, "-") == 0) drawRow(6, "no card yet");
  else if (strcmp(latest.uid, "!") == 0) drawRow(6, "RC522 NOT FOUND");
  else drawRow(6, latest.uid);

  if (haveData) snprintf(text, sizeof(text), "Motor A %4d", latest.pwmA);
  else snprintf(text, sizeof(text), "Motor A %4s", "--");
  drawRow(7, text);

  if (haveData) snprintf(text, sizeof(text), "Motor B %4d", latest.pwmB);
  else snprintf(text, sizeof(text), "Motor B %4s", "--");
  drawRow(8, text);

  drawRow(9, "");
  drawRow(10, linkOk ? "Link OK" : "NO DATA");
  drawRow(11, "");
}

void pollSerial() {
  static char buf[TELEMETRY_MAX_LINE];
  static size_t len = 0;
  static bool discarding = false;

  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!discarding) {
        buf[len] = '\0';
        Telemetry t;
        if (parseTelemetry(buf, t)) {
          latest = t;
          haveData = true;
          lastValidMs = millis();
        }
      }
      discarding = false;
      len = 0;
    } else if (!discarding) {
      if (len < sizeof(buf) - 1) {
        buf[len++] = c;
      } else {
        discarding = true;
        len = 0;
      }
    }
  }
}

void setup() {
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  tft.init(SCREEN_W, SCREEN_H);
  tft.setSPISpeed(20000000);  // jumper wires are unreliable at the default speed
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  for (int i = 0; i < ROW_COUNT; i++) {
    memset(rowCache[i], ' ', COLS);
    rowCache[i][COLS] = '\0';
  }
}

void loop() {
  pollSerial();
  render();
}
