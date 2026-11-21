#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include "Adafruit_TinyUSB.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

const uint8_t UART_TX_PIN = 0;
const uint8_t UART_RX_PIN = 1;
const uint8_t I2C_SDA_PIN = 4;
const uint8_t I2C_SCL_PIN = 5;

const uint8_t INPUT_PINS[4] = {6, 7, 8, 9};
const uint8_t LED_GREEN_PIN = 20;
const uint8_t LED_YELLOW_PIN = 21;
const uint8_t LED_RED_PIN = 22;
const uint8_t ONBOARD_LED_PIN = 25;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

Adafruit_USBH_Host USBHost;

bool sensorsReady = false;
bool modemMode = false;
bool inetConnected = false;

volatile bool inputsChanged = false;
volatile unsigned long lastInterruptTime = 0;
const unsigned long DEBOUNCE_DELAY = 20;

int brightness = 0;
int fadeAmount = 5;
unsigned long prevFadeMillis = 0;
unsigned long prevSensorMillis = 0;
unsigned long prevModemRetryMillis = 0;
unsigned long bootMillis = 0;

void onInputChangeISR() {
  inputsChanged = true;
  lastInterruptTime = millis();
}

void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    USBHost.task();
    yield();
  }
}
Adafruit_USBH_CDC ModemSerial;

void tuh_cdc_mount_cb(uint8_t idx) {
  Serial1.printf("[USB] CDC mounted idx=%d\r\n", idx);
  if (!ModemSerial.mounted() && idx == 0) {
    ModemSerial.mount(idx);
    Serial1.println("[USB] ModemSerial bound to idx=0 (IF=3)!");
  }
}

void tuh_cdc_umount_cb(uint8_t idx) {
  Serial1.printf("[USB] CDC unmounted idx=%d\r\n", idx);
  if (ModemSerial.mounted()) {
    ModemSerial.umount(idx);
  }
}

String sendAT(const String &cmd, unsigned long timeoutMs) {
  String response = "";
  Serial1.println("[AT >>] " + cmd);
  if (!ModemSerial.mounted()) {
    Serial1.println("[AT <<] ERR: ModemSerial not mounted");
    return response;
  }
  while (ModemSerial.available()) ModemSerial.read();
  ModemSerial.print(cmd + "\r");

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    USBHost.task();
    while (ModemSerial.available()) {
      response += (char)ModemSerial.read();
    }
    if (response.indexOf("\r\nOK\r\n") != -1 || response.indexOf("\r\nERROR\r\n") != -1) {
      break;
    }
    yield();
  }
  Serial1.println("[AT <<] " + response);
  return response;
}

void setup()
{
  Serial1.setTX(UART_TX_PIN);
  Serial1.setRX(UART_RX_PIN);
  Serial1.begin(115200);
  Serial1.println("\r\n=== RP2040 + Quectel EC800E-CN Start ===");

  pinMode(ONBOARD_LED_PIN, OUTPUT);
  analogWriteResolution(8);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  digitalWrite(LED_RED_PIN, HIGH);
  smartDelay(1000);
  digitalWrite(LED_RED_PIN, LOW);

  digitalWrite(LED_YELLOW_PIN, HIGH);
  smartDelay(1000);
  digitalWrite(LED_YELLOW_PIN, LOW);

  digitalWrite(LED_GREEN_PIN, HIGH);
  smartDelay(1000);
  digitalWrite(LED_GREEN_PIN, LOW);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(INPUT_PINS[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(INPUT_PINS[i]), onInputChangeISR, CHANGE);
  }

  USBHost.begin(0);
  ModemSerial.begin(115200);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial1.println("[ERR] OLED SSD1306 not found!");
    while (true) {
      digitalWrite(ONBOARD_LED_PIN, !digitalRead(ONBOARD_LED_PIN));
      delay(100);
    }
  }

  initSensors();
  display.clearDisplay();
  updateSensorsArea();
  updateInputsArea();
  display.display();

  bootMillis = millis();
}
void loop() {
  USBHost.task();
  updateLedFade();
  unsigned long currentMillis = millis();

  if (Serial1.available()) {
    String pcCmd = Serial1.readStringUntil('\n');
    pcCmd.trim();
    if (pcCmd.length() > 0) {
      Serial1.println("[PC -> MDM] " + pcCmd);
sendAT(pcCmd, 1000);
    }
  }

  if (inputsChanged && (currentMillis - lastInterruptTime >= DEBOUNCE_DELAY)) {
    inputsChanged = false;
    updateInputsArea();
    display.display();
  }

  if (!modemMode) {
    if (currentMillis - bootMillis >= 10000) {
      modemMode = true;
      checkAndConnectModem();
      prevModemRetryMillis = millis();
    } else if (currentMillis - prevSensorMillis >= 5000) {
      prevSensorMillis = currentMillis;
      updateSensorsArea();
      display.display();
    }
  } else {
    if (!inetConnected && (currentMillis - prevModemRetryMillis >= 30000)) {
      prevModemRetryMillis = millis();
      checkAndConnectModem();
    }
  }
}
bool initSensors() {
  bool ahtOk = aht.begin(&Wire);
  bool bmpOk = bmp.begin(0x77) || bmp.begin(0x76);
  sensorsReady = (ahtOk && bmpOk);
  Serial1.printf("[I2C] AHT20=%d, BMP280=%d\r\n", ahtOk, bmpOk);
  return sensorsReady;
}

void showSensorNotFound() {
  display.setCursor(0, 4);
  display.println(F("Sensor NOT Found"));
  display.setCursor(0, 16);
  display.println(F("Sensor NOT Found"));
}

void updateSensorsArea() {
  display.fillRect(0, 0, SCREEN_WIDTH, 44, SSD1306_BLACK);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  if (!sensorsReady && !initSensors()) {
    showSensorNotFound();
    return;
  }

  sensors_event_t humidityEvent, tempEvent;
  if (!aht.getEvent(&humidityEvent, &tempEvent)) {
    sensorsReady = false;
    showSensorNotFound();
    return;
  }

  float pressurePa = bmp.readPressure();
  if (isnan(pressurePa) || pressurePa <= 0) {
    sensorsReady = false;
    showSensorNotFound();
    return;
  }

  display.setCursor(0, 4);
  display.print(F("T= "));
  display.print(tempEvent.temperature, 1);
  display.println(F(" C"));

  display.setCursor(0, 16);
  display.print(F("P= "));
  display.print(pressurePa / 133.322, 1);
  display.println(F(" mm"));

  display.setCursor(0, 28);
  display.print(F("P= "));
  display.print(humidityEvent.relative_humidity, 1);
  display.println(F(" %"));
}
void updateInputsArea() {
  display.fillRect(0, 46, SCREEN_WIDTH, 18, SSD1306_BLACK);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.drawFastHLine(0, 44, SCREEN_WIDTH, SSD1306_WHITE);
  display.setCursor(0, 50);
  for (uint8_t i = 0; i < 4; i++) {
    display.print(digitalRead(INPUT_PINS[i]) == HIGH ? '1' : '0');
  }
}

void drawModemStatus(const String &l1, const String &l2, const String &l3) {
  display.fillRect(0, 0, SCREEN_WIDTH, 44, SSD1306_BLACK);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 4);  display.println(l1);
  display.setCursor(0, 16); display.println(l2);
  display.setCursor(0, 28); display.println(l3);
  updateInputsArea();
  display.display();
}

void updateLedFade() {
  unsigned long currentMillis = millis();
  if (currentMillis - prevFadeMillis >= 20) {
    prevFadeMillis = currentMillis;
    analogWrite(ONBOARD_LED_PIN, brightness);
    brightness += fadeAmount;
    if (brightness <= 0 || brightness >= 255) fadeAmount = -fadeAmount;
  }
}

void checkAndConnectModem() {
  String line1 = "AT NO ANSWER";
  String line2 = "RF = --";
  String line3 = "INET NOT";

  String atResp = sendAT("AT", 1000);
  if (atResp.indexOf("OK") == -1) {
    inetConnected = false;
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
    drawModemStatus(line1, line2, line3);
    return;
  }

  sendAT("ATE0", 500);

  String cpinResp = sendAT("AT+CPIN?", 1500);
  if (cpinResp.indexOf("+CPIN: READY") != -1) {
    line1 = "SIM OK";
  } else {
    line1 = "SIM ERROR";
  }

  String csqResp = sendAT("AT+CSQ", 1500);
  int csqPos = csqResp.indexOf("+CSQ:");
  if (csqPos != -1) {
    int commaPos = csqResp.indexOf(',', csqPos);
    if (commaPos != -1) {
      String rssiStr = csqResp.substring(csqPos + 5, commaPos);
      rssiStr.trim();
      line2 = "RF = " + rssiStr;
    }
  }

  drawModemStatus(line1, line2, "INET WAIT...");

  if (line1 == "SIM OK") {
    String statusResp = sendAT("AT+QNETDEVCTL?", 1500);
    if (statusResp.indexOf("+QNETDEVCTL: 1,1,1,1") != -1) {
      inetConnected = true;
    } else {
      String netResp = sendAT("AT+QNETDEVCTL=1,1,1", 5000);
      if (netResp.indexOf("OK") != -1) {
        smartDelay(2000);
        String verifyResp = sendAT("AT+QNETDEVCTL?", 2000);
        String ipResp = sendAT("AT+CGPADDR=1", 1500);
        inetConnected = (verifyResp.indexOf("+QNETDEVCTL: 1,1,1,1") != -1 ||
                        (ipResp.indexOf("+CGPADDR: 1,") != -1 && ipResp.indexOf("0.0.0.0") == -1));
      } else {
        inetConnected = false;
      }
    }
  } else {
    inetConnected = false;
  }

  if (inetConnected) {
    line3 = "INET OK";
    digitalWrite(LED_GREEN_PIN, HIGH);
    digitalWrite(LED_RED_PIN, LOW);
  } else {
    line3 = "INET NOT";
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
  }

  drawModemStatus(line1, line2, line3);
}
