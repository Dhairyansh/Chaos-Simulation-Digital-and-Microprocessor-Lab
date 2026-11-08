#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// ---------------- TFT ----------------
#define TFT_CS 10
#define TFT_DC 8
#define TFT_RST 9
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// ---------------- INPUT ----------------
#define ENC_CLK 2
#define ENC_DT 3
#define ENC_SW 4
#define BTN_MODE 5
#define CHAOS_LED 7

// ---------------- ENCODER ----------------
volatile long encoderValue = 0;
long lastEncoderValue = 0;
int lastEncoded = 0;

// ---------------- PARAM SELECT ----------------
int paramSelect = 0; // 0=sigma, 1=rho, 2=beta

// ---------------- LORENZ 1 (CYAN, controllable) ----------------
float x = 0.1, y = 0.1, z = 0.1;
float sigma = 10, rho = 28, beta = 2.67;

// ---------------- LORENZ 2 (YELLOW, fixed reference) ----------------
float x2 = 0.1, y2 = 0.1, z2 = 0.1;
const float sigma2 = 10, rho2 = 28, beta2 = 2.67;

// ---------------- SHARED ----------------
float dt = 0.0001;

// ---------------- UI ----------------
unsigned long lastUI = 0;

// ================= SETUP =================
void setup() {
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(CHAOS_LED, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_CLK), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), readEncoder, CHANGE);
}

// ================= LOOP =================
void loop() {
  updateEncoder();
  handleButtons();
  lorenzMode();

  if (millis() - lastUI > 100) {
    updateUI();
    lastUI = millis();
  }
}

// ================= ENCODER =================
void readEncoder() {
  int MSB = digitalRead(ENC_CLK);
  int LSB = digitalRead(ENC_DT);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

  lastEncoded = encoded;
}

void updateEncoder() {
  if (encoderValue != lastEncoderValue) {
    long diff = encoderValue - lastEncoderValue;
    adjustParam(diff * 0.1);
    lastEncoderValue = encoderValue;
  }
}

// ================= PARAM CONTROL =================
void adjustParam(float delta) {
  if (paramSelect == 0) sigma += delta;
  if (paramSelect == 1) rho   += delta;
  if (paramSelect == 2) beta  += delta;

  sigma = constrain(sigma, 0, 50);
  rho   = constrain(rho,   0, 100);
  beta  = constrain(beta,  0, 10);

  resetSimulation();
}

// ================= BUTTONS =================
void handleButtons() {
  static unsigned long lastPress = 0;
  if (millis() - lastPress < 200) return;

  // Encoder button: cycle through sigma -> rho -> beta
  if (!digitalRead(ENC_SW)) {
    paramSelect = (paramSelect + 1) % 3;
    lastPress = millis();
  }

  // BTN_MODE: reset cyan attractor params to defaults
  if (!digitalRead(BTN_MODE)) {
    sigma = 10;
    rho   = 28;
    beta  = 2.67;
    resetSimulation();
    lastPress = millis();
  }
}

// ================= RESET =================
void resetSimulation() {
  tft.fillScreen(ILI9341_BLACK);
  x  = 0.1; y  = 0.1; z  = 0.1;
  x2 = 0.1; y2 = 0.1; z2 = 0.1;
}

// ================= UI =================
void updateUI() {
  tft.fillRect(0, 220, 320, 20, ILI9341_DARKGREY);
  tft.setCursor(5, 225);
  tft.setTextSize(1);

  // Cyan attractor params (controllable)
  tft.setTextColor(ILI9341_CYAN);
  tft.print("C:");
  tft.print("s="); tft.print(sigma, 1);
  tft.print(" r="); tft.print(rho, 1);
  tft.print(" b="); tft.print(beta, 1);

  // Yellow attractor params (fixed)
  tft.setTextColor(ILI9341_YELLOW);
  tft.print(" Y:REF");

  // Active param indicator
  tft.setTextColor(ILI9341_WHITE);
  tft.print(" [");
  if (paramSelect == 0) tft.print("s");
  if (paramSelect == 1) tft.print("r");
  if (paramSelect == 2) tft.print("b");
  tft.print("] RST=BTN5");
}

// ================= LORENZ =================
void lorenzMode() {
  // --- Attractor 1 (CYAN, controllable) ---
  float dx = sigma * (y - x);
  float dy = x * (rho - z) - y;
  float dz = x * y - beta * z;

  x += dx * dt;
  y += dy * dt;
  z += dz * dt;

  int px = map(x * 20, -400, 400, 0, 320);
  int py = map(z * 20, 0, 800, 0, 200);

  if (px >= 0 && px < 320 && py >= 0 && py < 220)
    tft.drawPixel(px, py, ILI9341_CYAN);

  // --- Attractor 2 (YELLOW, fixed reference) ---
  float dx2 = sigma2 * (y2 - x2);
  float dy2 = x2 * (rho2 - z2) - y2;
  float dz2 = x2 * y2 - beta2 * z2;

  x2 += dx2 * dt;
  y2 += dy2 * dt;
  z2 += dz2 * dt;

  int px2 = map(x2 * 20, -400, 400, 0, 320);
  int py2 = map(z2 * 20, 0, 800, 0, 200);

  if (px2 >= 0 && px2 < 320 && py2 >= 0 && py2 < 220)
    tft.drawPixel(px2, py2, ILI9341_YELLOW);

  digitalWrite(CHAOS_LED, (rho > 24.74));
}