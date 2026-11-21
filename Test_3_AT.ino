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
uint8_t modemDevAddr = 0;
uint8_t epAtOut = 0;
uint8_t epAtIn = 0;
String rxAtBuffer = "";
volatile bool rxBusy = false;
uint8_t rxTempBuf[64];

void onAtRxComplete(tuh_xfer_t* xfer) {
  if (xfer->result == XFER_RESULT_SUCCESS && xfer->actual_len > 0) {
    for (uint32_t i = 0; i < xfer->actual_len; i++) {
      rxAtBuffer += (char)rxTempBuf[i];
    }
  }
  rxBusy = false;
}

void startAtRx() {
  if (modemDevAddr == 0 || epAtIn == 0 || rxBusy) return;
  rxBusy = true;
  tuh_xfer_t xfer = {
    .daddr = modemDevAddr,
    .ep_addr = epAtIn,
    .buflen = sizeof(rxTempBuf),
    .buffer = rxTempBuf,
    .complete_cb = onAtRxComplete,
    .user_data = 0
  };
  if (!tuh_edpt_xfer(&xfer)) rxBusy = false;
}

void parseConfigDesc(uint8_t daddr, tusb_desc_configuration_t const* desc_cfg) {
  uint8_t const* p = (uint8_t const*)desc_cfg;
  uint16_t totalLen = tu_le16toh(desc_cfg->wTotalLength);
  uint16_t offset = 0;
  uint8_t curIf = 0, curClass = 0, curSubClass = 0, curProto = 0;

  while (offset < totalLen) {
    uint8_t len = p[offset];
    uint8_t type = p[offset + 1];
    if (len == 0) break;

    if (type == TUSB_DESC_INTERFACE) {
      tusb_desc_interface_t const* ifDesc = (tusb_desc_interface_t const*)(p + offset);
      curIf = ifDesc->bInterfaceNumber;
      curClass = ifDesc->bInterfaceClass;
      curSubClass = ifDesc->bInterfaceSubClass;
      curProto = ifDesc->bInterfaceProtocol;
      Serial1.printf("[DESC] IF=%d Class=0x%02X Sub=0x%02X Proto=0x%02X EPs=%d\r\n",
                     curIf, curClass, curSubClass, curProto, ifDesc->bNumEndpoints);
    } else if (type == TUSB_DESC_ENDPOINT) {
      tusb_desc_endpoint_t const* epDesc = (tusb_desc_endpoint_t const*)(p + offset);
      Serial1.printf("   [EP] addr=0x%02X attr=0x%02X size=%d\r\n",
                     epDesc->bEndpointAddress, epDesc->bmAttributes.xfer, tu_le16toh(epDesc->wMaxPacketSize));
      // У EC800E (PID 0903): IF=0,1 это RNDIS, IF=2 или IF=3 с 2 Bulk EP — AT-порт
      if (curClass == 0xFF && epDesc->bmAttributes.xfer == TUSB_XFER_BULK && curIf == 4) {
        if (tu_edpt_dir(epDesc->bEndpointAddress) == TUSB_DIR_OUT && epAtOut == 0) {
          tuh_edpt_open(daddr, epDesc);
          epAtOut = epDesc->bEndpointAddress;
        } else if (tu_edpt_dir(epDesc->bEndpointAddress) == TUSB_DIR_IN && epAtIn == 0) {
          tuh_edpt_open(daddr, epDesc);
          epAtIn = epDesc->bEndpointAddress;
        }
      }
    }
    offset += len;
  }
  if (epAtOut && epAtIn) {
    modemDevAddr = daddr;
    Serial1.printf("[USB] AT Bulk EPs opened! OUT=0x%02X IN=0x%02X\r\n", epAtOut, epAtIn);

    tusb_control_request_t request = {
      .bmRequestType_bit = {
        .recipient = TUSB_REQ_RCPT_INTERFACE,
        .type      = TUSB_REQ_TYPE_CLASS,
        .direction = TUSB_DIR_OUT
      },
      .bRequest = 0x22,
      .wValue   = tu_htole16(0x03),
      .wIndex   = tu_htole16(4),
      .wLength  = 0
    };
    tuh_xfer_t ctrlXfer = {
      .daddr       = daddr,
      .ep_addr     = 0,
      .setup       = &request,
      .buffer      = NULL,
      .complete_cb = NULL,
      .user_data   = 0
    };
    uint8_t ctrlRes = tuh_control_xfer_sync(&ctrlXfer);
    Serial1.printf("[USB] DTR/RTS sync res=%d\r\n", ctrlRes);
    startAtRx();
  }


}

void tuh_mount_cb(uint8_t dev_addr) {
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);
  Serial1.printf("[USB] Device connected! addr=%d VID=%04X PID=%04X\r\n", dev_addr, vid, pid);

  uint8_t cfgBuf[512];
  uint8_t res = tuh_descriptor_get_configuration_sync(dev_addr, 0, cfgBuf, sizeof(cfgBuf));
  Serial1.printf("[USB] GetCfgSync res=%d\r\n", res);
  if (res == XFER_RESULT_SUCCESS) {
    parseConfigDesc(dev_addr, (tusb_desc_configuration_t const*)cfgBuf);
  }
}

void tuh_umount_cb(uint8_t dev_addr) {
  Serial1.printf("[USB] Device disconnected! addr=%d\r\n", dev_addr);
  if (modemDevAddr == dev_addr) {
    modemDevAddr = 0; epAtOut = 0; epAtIn = 0; rxBusy = false;
  }
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

String sendAT(const String &cmd, unsigned long timeoutMs) {
  String response = "";
  Serial1.println("[AT >>] " + cmd);
  if (modemDevAddr == 0 || epAtOut == 0 || epAtIn == 0) {
    Serial1.println("[AT <<] ERR: AT Bulk EPs not ready");
    return response;
  }

  rxAtBuffer = "";
  startAtRx();

  String fullCmd = cmd + "\r\n";
  static uint8_t txBuf[128];
  uint16_t txLen = min((size_t)fullCmd.length(), sizeof(txBuf));
  memcpy(txBuf, fullCmd.c_str(), txLen);

  tuh_xfer_t txXfer = {
    .daddr = modemDevAddr,
    .ep_addr = epAtOut,
    .buflen = txLen,
    .buffer = txBuf,
    .complete_cb = NULL,
    .user_data = 0
  };
  tuh_edpt_xfer(&txXfer);

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    USBHost.task();
    startAtRx();
    if (rxAtBuffer.indexOf("\r\nOK\r\n") != -1 || rxAtBuffer.indexOf("\r\nERROR\r\n") != -1) {
      break;
    }
    yield();
  }
  response = rxAtBuffer;
  Serial1.println("[AT <<] " + response);
  return response;
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
