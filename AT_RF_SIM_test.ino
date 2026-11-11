/*
 * Pinout Pico: GP0(TX)->COM12 | GP1(RX) | GP4(SDA)/GP5(SCL)->OLED | GP25->LED | TP2/TP3->EC800E
 * USB Stack: "Adafruit TinyUSB Host (Native)"
 */
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_TinyUSB.h>
#include "host/usbh.h"
#include "host/usbh_pvt.h"

#define MAX_PORTS 6
#define BUF_SZ    64
const char* USSD_CMD = "AT+CUSD=1,\"*100#\",15";

Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_USBH_Host USBHost;

struct QuectelPort { uint8_t daddr, itf, ep_in, ep_out; bool rxPend; String acc; };
QuectelPort qP[MAX_PORTS];
uint8_t qCnt = 0, probeIdx = 0, stepSt = 0; // 0=probe,1=AT,2=COPS=?,3=CPIN,4=COPS?,5=USSD,6=done
volatile int actIdx = -1;
volatile bool cmdBusy = false, oledDirty = true, hasLog = false;
uint8_t rxBuf[MAX_PORTS][BUF_SZ] __attribute__((aligned(4))), txBuf[BUF_SZ] __attribute__((aligned(4)));
String l1="1.AT  : Wait USB", l2="2.Scan: --", l3="3.SIM : --", l4="4.Oper: --", l5="5.Bal : --", logBuf="";
unsigned long cmdMs = 0, probeMs = 0, fadeMs = 0;
int bri = 0, fadeStep = 5;

void logMsg(const String &m) { logBuf = hasLog ? (logBuf + "\r\n" + m) : m; hasLog = true; }

void drawOled() {
  display.clearDisplay(); display.setTextColor(1); display.setTextSize(1);
  display.setCursor(0,0); display.println(F("EC800E-CN Diag")); display.drawFastHLine(0,9,128,1);
  display.setCursor(0,12); display.println(l1); display.setCursor(0,22); display.println(l2);
  display.setCursor(0,32); display.println(l3); display.setCursor(0,42); display.println(l4);
  display.setCursor(0,52); display.println(l5); display.display();
}

String decodeUssd(String s) {
  s.trim();
  if (s.length() < 8 || s.length() % 4 != 0) return s;
  for (unsigned int i = 0; i < s.length(); i++) if (!isxdigit(s.charAt(i))) return s;
  const char* tr[64] = {"A","B","V","G","D","E","Zh","Z","I","Y","K","L","M","N","O","P","R","S","T","U","F","Kh","Ts","Ch","Sh","Sch","","Y","","E","Yu","Ya","a","b","v","g","d","e","zh","z","i","y","k","l","m","n","o","p","r","s","t","u","f","kh","ts","ch","sh","sch","","y","","e","yu","ya"};
  String o = "";
  for (unsigned int i = 0; i + 3 < s.length(); i += 4) {
    uint16_t c = strtol(s.substring(i, i + 4).c_str(), NULL, 16);
    if (c >= 32 && c <= 126) o += (char)c;
    else if (c >= 0x0410 && c <= 0x044F) o += tr[c - 0x0410];
    else o += ' ';
  }
  return o;
}

void readPort(uint8_t i) {
  if (i >= qCnt || !qP[i].daddr || qP[i].rxPend) return;
  qP[i].rxPend = true; usbh_edpt_xfer(qP[i].daddr, qP[i].ep_in, rxBuf[i], BUF_SZ);
}

void sendAt(uint8_t i, const char *c) {
  if (i >= qCnt || !qP[i].daddr) return;
  String s = String(c) + "\r\n"; size_t n = min((size_t)BUF_SZ, s.length());
  memcpy(txBuf, s.c_str(), n); readPort(i);
  usbh_edpt_xfer(qP[i].daddr, qP[i].ep_out, txBuf, n);
  cmdBusy = true; cmdMs = millis(); logMsg("[TX] " + String(c));
}

void onLine(uint8_t i, String ln) {
  ln.trim(); if (!ln.length()) return;
  logMsg("[RX itf=" + String(qP[i].itf) + "] " + ln);
  if (stepSt == 0 && (ln == "OK" || ln.indexOf("AT") >= 0)) {
    actIdx = i; l1 = "1.AT  : OK (itf=" + String(qP[i].itf) + ")"; oledDirty = true; cmdBusy = false; stepSt = 1;
    return;
  }
  if (i != actIdx) return;
  if (stepSt == 1 && ln == "OK") { l1 = "1.AT  : OK"; cmdBusy = false; stepSt = 2; }
  else if (stepSt == 2) {
    if (ln.startsWith("+COPS:")) {
      String nets = ""; int p = 0;
      while ((p = ln.indexOf(",\"", p)) >= 0) {
        int e = ln.indexOf("\"", p + 2);
        if (e > p + 2) { String nm = ln.substring(p + 2, e); if (nm.length() > 3 && !isDigit(nm.charAt(0)) && nets.indexOf(nm) < 0) nets += (nets.length() ? "," : "") + nm; p = e + 1; } else break;
      }
      l2 = "2.Scan: " + (nets.length() ? nets : "Found");
    } else if (ln == "OK" || ln.indexOf("ERR") >= 0) { cmdBusy = false; stepSt = 3; }
  }
  else if (stepSt == 3) {
    if (ln.startsWith("+CPIN:")) { String st = ln.substring(6); st.trim(); l3 = "3.SIM : " + st; stepSt = (st == "READY") ? 4 : 6; }
    else if (ln.indexOf("ERR") >= 0) { l3 = "3.SIM : NO SIM"; l4 = "4.Oper: N/A"; l5 = "5.Bal : N/A"; cmdBusy = false; stepSt = 6; }
    else if (ln == "OK") cmdBusy = false;
  }
  else if (stepSt == 4) {
    if (ln.startsWith("+COPS:")) { int q1 = ln.indexOf('"'), q2 = ln.indexOf('"', q1 + 1); l4 = (q1 >= 0 && q2 > q1) ? ("4.Oper: " + ln.substring(q1 + 1, q2)) : "4.Oper: No reg"; }
    else if (ln == "OK") { cmdBusy = false; stepSt = 5; }
  }
  else if (stepSt == 5) {
    if (ln.startsWith("+CUSD:")) { int q1 = ln.indexOf('"'), q2 = ln.lastIndexOf('"'); l5 = (q1 >= 0 && q2 > q1) ? ("5.Bal : " + decodeUssd(ln.substring(q1 + 1, q2))) : "5.Bal : Err"; cmdBusy = false; stepSt = 6; }
    else if (ln.indexOf("ERR") >= 0) { l5 = "5.Bal : USSD Err"; cmdBusy = false; stepSt = 6; }
  }
  oledDirty = true;
}

bool qInit(void) { return true; }
bool qOpen(uint8_t rh, uint8_t da, const tusb_desc_interface_t *itf, uint16_t len) {
  (void)rh; if (itf->bInterfaceClass != 0xFF || itf->bNumEndpoints < 2 || qCnt >= MAX_PORTS) return false;
  const uint8_t *p = (const uint8_t *)itf + itf->bLength, *e = (const uint8_t *)itf + len;
  uint8_t ei = 0, eo = 0;
  while (p < e && *p && *(p+1) != TUSB_DESC_INTERFACE) {
    if (*(p+1) == TUSB_DESC_ENDPOINT) { const tusb_desc_endpoint_t *ep = (const tusb_desc_endpoint_t *)p; if (ep->bmAttributes.xfer == 2) { if (ep->bEndpointAddress & 0x80) ei = ep->bEndpointAddress; else eo = ep->bEndpointAddress; } }
    p += *p;
  }
  if (!ei || !eo) return false;
  tusb_desc_endpoint_t dI = {7, 5, ei, {2}, BUF_SZ, 0}, dO = {7, 5, eo, {2}, BUF_SZ, 0};
  if (!tuh_edpt_open(da, &dI) || !tuh_edpt_open(da, &dO)) return false;
  qP[qCnt++] = {da, itf->bInterfaceNumber, ei, eo, false, ""}; return true;
}
bool qCfg(uint8_t da, uint8_t itf) {
  for (uint8_t i = 0; i < qCnt; i++) if (qP[i].daddr == da && qP[i].itf == itf) {
    tusb_control_request_t r = {.bmRequestType_bit={1,1,0}, .bRequest=0x22, .wValue=3, .wIndex=itf, .wLength=0};
    tuh_xfer_t x = {.daddr=da, .ep_addr=0, .setup=&r, .buffer=NULL, .complete_cb=NULL, .user_data=0};
    tuh_control_xfer(&x); readPort(i);
  }
  usbh_driver_set_config_complete(da, itf); l1 = "1.AT  : Probing..."; oledDirty = true; return true;
}
bool qXfer(uint8_t da, uint8_t ep, xfer_result_t res, uint32_t bytes) {
  if (ep & 0x80) for (uint8_t i = 0; i < qCnt; i++) if (qP[i].daddr == da && qP[i].ep_in == ep) {
    qP[i].rxPend = false;
    if (res == XFER_RESULT_SUCCESS) for (uint32_t k = 0; k < bytes; k++) { char c = rxBuf[i][k]; if (c == '\n') { onLine(i, qP[i].acc); qP[i].acc = ""; } else if (c != '\r') qP[i].acc += c; }
    readPort(i); break;
  }
  return true;
}
void qClose(uint8_t da) { (void)da; qCnt = 0; actIdx = -1; cmdBusy = false; stepSt = 0; l1="1.AT  : Disconn"; oledDirty = true; }
static const usbh_class_driver_t qDrv = { .name="Q_DRV", .init=qInit, .open=qOpen, .set_config=qCfg, .xfer_cb=qXfer, .close=qClose };
extern "C" const usbh_class_driver_t* usbh_app_driver_get_cb(uint8_t* cnt) { *cnt = 1; return &qDrv; }

void setup() {
  Serial1.setTX(0); Serial1.setRX(1); Serial1.begin(115200); delay(200); Serial1.println(F("\r\n=== EC800E Diag ==="));
  pinMode(25, OUTPUT); analogWriteResolution(8); Wire.setSDA(4); Wire.setSCL(5); Wire.begin();
  if (!display.begin(0x02, 0x3C)) while (true); drawOled();
}
void loop() {
  if (millis() - fadeMs >= 20) { fadeMs = millis(); analogWrite(25, bri); bri += fadeStep; if (bri <= 0 || bri >= 255) fadeStep = -fadeStep; }
  if (hasLog) { String s = logBuf; logBuf = ""; hasLog = false; Serial1.println(s); }
  if (oledDirty) { oledDirty = false; drawOled(); }
}
void setup1() { delay(300); USBHost.begin(0); }
void loop1() {
  USBHost.task(); if (!qCnt) return;
  if (stepSt == 0) { if (millis() - probeMs >= 1500) { probeMs = millis(); sendAt(probeIdx, "AT"); probeIdx = (probeIdx + 1) % qCnt; } return; }
  if (cmdBusy) { if (millis() - cmdMs >= (stepSt == 2 ? 60000 : 15000)) { cmdBusy = false; if (stepSt == 2) { l2="2.Scan: Timeout"; stepSt=3; } else if (stepSt == 5) { l5="5.Bal : Timeout"; stepSt=6; } oledDirty = true; } return; }
  if (stepSt == 1) sendAt(actIdx, "AT");
  else if (stepSt == 2) { l2 = "2.Scan: Wait..."; oledDirty = true; sendAt(actIdx, "AT+COPS=?"); }
  else if (stepSt == 3) sendAt(actIdx, "AT+CPIN?");
  else if (stepSt == 4) sendAt(actIdx, "AT+COPS?");
  else if (stepSt == 5) { l5 = "5.Bal : USSD..."; oledDirty = true; sendAt(actIdx, USSD_CMD); }
}
