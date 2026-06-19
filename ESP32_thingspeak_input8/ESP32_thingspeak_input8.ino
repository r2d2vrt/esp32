//1) Антидребезг (0.3 сек): Состояние каждого входа считается изменившимся, только если оно
//2) стабильно удерживается в новом положении более 300 мс.
//3) Мгновенная реакция: Как только обнаружено стабильное изменение, плата мгновенно включает OUT0,
// ставит статус в очередь на отправку и выводит информацию на экран.
//4) Блокировка на 30 сек: После фиксации изменения включается таймер блокировки на 30 секунд.
// Очередь событий: Если во время этих 30 секунд (пока горит OUT0 и идет отсчет) другие входы
// изменят свое состояние, система зафиксирует это изменение, сформирует новую строку состояния
// и поставит её в очередь (массив). Текущие 30 секунд не прерываются. Как только они истекут,
// плата сразу отправит следующее событие из очереди и запустит новые 30 секунд.
//5) Обычный режим: Если очередь пуста и тревог нет, каждые 300 секунд отправляется текущее базовое состояние.
//6) добавлен AHT20 + BMP280
//7)Обновленный код для ESP32 реализует управление выходами OUT0/OUT1 при изменении состояний входов IN0-IN3 
// с таймером на 30 секунд для OUT1. Реализован механизм повторной отправки данных на ThingSpeak каждые 30 секунд
// при ошибках (RX ERR), а также обновлен интерфейс OLED-дисплея, отображающий температуру, давление, влажность,
// уровни входов и таймер прогресса без названия Wi-Fi сети.
//8) 1) Во время цикла обработки входов на дисплее показывать не время до следующей отправки сообщения, а состояние входов.
//300 секундный цикл дежурного режима начинать каждый раз с нуля после отправки последнего сообщения тревожного цикла,
// а не продолжать прерванный. При изменении IN0 и IN1 устанавливать в единицу на 30 секунд OUT0 плюс устанавливать в единицу 
// на 120 секунд OUT1. При изменении IN1 и IN2 устанавливать в единицу на 120 секунд только OUT1

// ============================================================================
// ESP32 ThingSpeak Monitor - QUEUE, RETRY & OUTPUT LOGIC V2.3
// ============================================================================
// 1) Во время обработки очереди на дисплее показываются СОСТОЯНИЯ ВХОДОВ
// 2) 300с цикл дежурного режима СБРАСЫВАЕТСЯ в 0 после отправки последнего тревожного сообщения
// 3) OUT0/OUT1 логика: 
//    IN0/IN1 changed → OUT0=30с + OUT1=120с
//    IN2/IN3 changed → OUT1=120с
// ============================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <HTTPClient.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
bool isSensorAvailable = false;

#define LED_PIN 2
#define IN0 25
#define IN1 26
#define IN2 27
#define IN3 13
#define OUT0 32
#define OUT1 33

const char* ssid = "Wally";
const char* password = "23051976";
const String serverName = "http://api.thingspeak.com/update";
const String apiKey = "ssc48jt2fzxhxorj";

// === НАСТРОЙКИ ТАЙМЕРОВ ===
const unsigned long DEBOUNCE_DELAY = 300;           // Антидребезг входов
const unsigned long OUT0_PULSE_TIME = 30000;        // OUT0: 30 секунд
const unsigned long OUT1_PULSE_TIME = 120000;       // OUT1: 120 секунд
const unsigned long NORMAL_REPORT_TIME = 300000;    // Обычная отправка: 300 сек
const unsigned long QUEUE_SEND_INTERVAL = 30000;    // Интервал отправки из очереди: 30 сек

const int numInputs = 4;
const int inputPins[numInputs] = {IN0, IN1, IN2, IN3};

// === ПЕРЕМЕННЫЕ ДЛЯ ВХОДОВ ===
int lastSteadyStates[numInputs] = {HIGH, HIGH, HIGH, HIGH};
int lastDebounceStates[numInputs] = {HIGH, HIGH, HIGH, HIGH};
unsigned long lastDebounceTimes[numInputs] = {0, 0, 0, 0};
int previousSteadyStates[numInputs] = {HIGH, HIGH, HIGH, HIGH};

// === ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ===
String globalCurrentStatus = "1111";
unsigned long lastNormalSendTime = 0;

// === ВЫХОДЫ ===
unsigned long out0EndTime = 0;
unsigned long out1EndTime = 0;
bool out0Active = false;
bool out1Active = false;

// === ДАТЧИКИ ===
float sensorTemp = 0.0;
float sensorHum = 0.0;
float sensorPress = 0.0;

// === ОЧЕРЕДЬ FIFO ===
#define QUEUE_MAX_SIZE 20
String msgQueue[QUEUE_MAX_SIZE];
int qHead = 0;
int qCount = 0;
unsigned long lastQueueSendTime = 0;

// ============================================================================
// ФУНКЦИИ ОЧЕРЕДИ
// ============================================================================
void enqueue(String msg) {
  if (qCount < QUEUE_MAX_SIZE) {
    int idx = (qHead + qCount) % QUEUE_MAX_SIZE;
    msgQueue[idx] = msg;
    qCount++;
    Serial.println("✓ Enqueue: " + msg + " | Queue: " + String(qCount));
  } else {
    Serial.println("✗ Queue overflow! Event lost.");
  }
}

void dequeue() {
  if (qCount == 0) return;
  qHead = (qHead + 1) % QUEUE_MAX_SIZE;
  qCount--;
  Serial.println("✓ Dequeue | Remaining: " + String(qCount));
}

String peekQueue() {
  if (qCount == 0) return "";
  return msgQueue[qHead];
}

// ============================================================================
// ДИСПЛЕЙ
// ============================================================================
void showNormalDisplay(float temp, float press, float hum, String inputs, 
                       int progressSec, int maxPeriod) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(0, 0); display.print("T:"); display.print(temp, 1); display.println("C");
  display.setCursor(0, 12); display.print("P:"); display.print(press, 1); display.println("mmHg");
  display.setCursor(0, 24); display.print("H:"); display.print(hum, 0); display.println("%");
  
  display.setTextSize(2);
  display.setCursor(0, 36);
  display.print(inputs);
  
  display.setTextSize(1);
  display.setCursor(90, 42);
  display.print("Q:"); display.println(qCount);
  
  if (maxPeriod > 0) {
    int barWidth = map(progressSec, 0, maxPeriod, 0, SCREEN_WIDTH);
    display.fillRect(0, SCREEN_HEIGHT - 4, barWidth, 4, SSD1306_WHITE);
    display.drawRect(0, SCREEN_HEIGHT - 4, SCREEN_WIDTH, 4, SSD1306_WHITE);
  }
  display.display();
}

// Дисплей во время обработки очереди (по требованию: входы вместо таймера)
void showQueueDisplay(String inputs, int queueSize) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.print("IN:"); display.println(inputs);
  
  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print("Queue: "); display.print(queueSize); display.println(" msg");
  display.println("Sending every 30s...");
  
  display.display();
}

void showStatusMessage(String title, String status) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(title);
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.println(status);
  display.display();
}

// ============================================================================
// ОТПРАВКА НА THINGSPEAK
// ============================================================================
bool sendDataToServer(String payload, bool includeClimate = true) {
  if (WiFi.status() != WL_CONNECTED) return false;
  
  HTTPClient http;
  String url = serverName + "?api_key=" + apiKey + "&field1=" + payload;
  
  if (includeClimate && payload.length() == 4 && isSensorAvailable) {
    url += "&field2=" + String(sensorTemp, 2);
    url += "&field3=" + String(sensorPress, 2);
    url += "&field4=" + String(sensorHum, 2);
  }
  
  Serial.println("→ " + url);
  digitalWrite(LED_PIN, HIGH);
  showStatusMessage("Sending...", "ST=" + payload);
  
  http.begin(url);
  int httpCode = http.GET();
  
  bool success = (httpCode == 200);
  Serial.println(success ? "✓ RX OK" : "✗ RX ERR: " + String(httpCode));
  showStatusMessage(success ? "Server Response" : "Server Error", 
                    success ? "RX OK" : "RX ERR");
  
  digitalWrite(LED_PIN, LOW);
  delay(150);
  http.end();
  return success;
}

// ============================================================================
// ОБНОВЛЕНИЕ ДАТЧИКОВ
// ============================================================================
void updateSensors() {
  if (!isSensorAvailable) return;
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  sensorTemp = bmp.readTemperature();
  sensorHum = humidity.relative_humidity;
  sensorPress = bmp.readPressure() * 0.00750062; // Pa → mmHg
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(OUT0, OUTPUT);
  pinMode(OUT1, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(OUT0, LOW);
  digitalWrite(OUT1, LOW);
  
  for (int i = 0; i < numInputs; i++) {
    pinMode(inputPins[i], INPUT_PULLUP);
    int val = digitalRead(inputPins[i]);
    lastSteadyStates[i] = lastDebounceStates[i] = previousSteadyStates[i] = val;
  }
  globalCurrentStatus = String(digitalRead(IN0)) + String(digitalRead(IN1)) + 
                        String(digitalRead(IN2)) + String(digitalRead(IN3));
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("✗ OLED init failed!");
    while (true);
  }
  display.clearDisplay(); display.display();
  
  if (aht.begin() && bmp.begin(0x77)) {
    isSensorAvailable = true;
    Serial.println("✓ Sensors OK");
  } else {
    isSensorAvailable = false;
    Serial.println("✗ Sensors NOT found!");
  }
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    Serial.print(".");
    showStatusMessage("WiFi...", "Connecting");
    delay(500);
  }
  Serial.println("\n✓ Connected!");
  showStatusMessage("WiFi", "Connected");
  delay(1000);
  
  updateSensors();
  lastNormalSendTime = millis();
  lastQueueSendTime = millis() - QUEUE_SEND_INTERVAL; // Первая отправка мгновенно
  
  sendDataToServer(globalCurrentStatus, true);
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, reconnecting...");
    showStatusMessage("WiFi Error", "Reconnecting...");
    digitalWrite(LED_PIN, HIGH); delay(1000); digitalWrite(LED_PIN, LOW);
    WiFi.reconnect(); delay(2000);
    return;
  }
  
  unsigned long currentMillis = millis();
  bool inputChanged = false;
  
  // Сохраняем предыдущие состояния ДО обработки антидребезга
  for (int i = 0; i < numInputs; i++) previousSteadyStates[i] = lastSteadyStates[i];
  
  // === 1) АНТИДРЕБЕЗГ ВХОДОВ ===
  for (int i = 0; i < numInputs; i++) {
    int reading = digitalRead(inputPins[i]);
    if (reading != lastDebounceStates[i]) {
      lastDebounceTimes[i] = currentMillis;
      lastDebounceStates[i] = reading;
    }
    if ((currentMillis - lastDebounceTimes[i]) >= DEBOUNCE_DELAY) {
      if (reading != lastSteadyStates[i]) {
        lastSteadyStates[i] = reading;
        inputChanged = true;
      }
    }
  }
  
  // === 2) ОБРАБОТКА ИЗМЕНЕНИЙ ВХОДОВ ===
  if (inputChanged) {
    String newStatus = "";
    for (int i = 0; i < numInputs; i++) newStatus += String(lastSteadyStates[i]);
    globalCurrentStatus = newStatus;
    
    bool in0OrIn1Changed = false;
    bool in2OrIn3Changed = false; // Примечание: если имелись в виду IN1 и IN2, замените индексы [2,3] на [1,2]
    
    for (int i = 0; i < numInputs; i++) {
      if (lastSteadyStates[i] != previousSteadyStates[i]) {
        if (i == 0 || i == 1) in0OrIn1Changed = true;
        if (i == 2 || i == 3) in2OrIn3Changed = true;
      }
    }
    
    // Управление выходами (новая логика)
    if (in0OrIn1Changed) {
      out0EndTime = currentMillis + OUT0_PULSE_TIME;
      out0Active = true;
      digitalWrite(OUT0, HIGH);
      
      out1EndTime = (currentMillis + OUT1_PULSE_TIME > out1EndTime) ? currentMillis + OUT1_PULSE_TIME : out1EndTime;
      out1Active = true;
      digitalWrite(OUT1, HIGH);
      Serial.println("✓ OUT0 ON 30s | OUT1 ON 120s (IN0/IN1 changed)");
    }
    
    if (in2OrIn3Changed) {
      out1EndTime = (currentMillis + OUT1_PULSE_TIME > out1EndTime) ? currentMillis + OUT1_PULSE_TIME : out1EndTime;
      out1Active = true;
      digitalWrite(OUT1, HIGH);
      Serial.println("✓ OUT1 ON 120s (IN2/IN3 changed)");
    }
    
    enqueue("1" + globalCurrentStatus);
  }
  
  // === 3) ТАЙМЕРЫ ВЫХОДОВ ===
  if (out0Active && currentMillis >= out0EndTime) {
    out0Active = false; digitalWrite(OUT0, LOW);
    Serial.println("✓ OUT0 OFF (30s elapsed)");
  }
  if (out1Active && currentMillis >= out1EndTime) {
    out1Active = false; digitalWrite(OUT1, LOW);
    Serial.println("✓ OUT1 OFF (120s elapsed)");
  }
  
  // === 4) ОБРАБОТКА ОЧЕРЕДИ (СТРОГО КАЖДЫЕ 30 СЕКУНД) ===
  if (qCount > 0 && (currentMillis - lastQueueSendTime >= QUEUE_SEND_INTERVAL)) {
    String currentMsg = peekQueue();
    bool ok = sendDataToServer(currentMsg, false);
    
    lastQueueSendTime = currentMillis; // Таймер сбрасывается всегда
    
    if (ok) {
      dequeue();
      Serial.println("✓ Queue item sent & removed");
      
      // Доработка 2: Если очередь опустела, сбрасываем 300с цикл дежурного режима в ноль
      if (qCount == 0) {
        lastNormalSendTime = millis();
        Serial.println("✓ Queue empty. 300s normal cycle RESTARTED from 0.");
      }
    } else {
      Serial.println("✗ Queue item failed. Will retry in 30s.");
    }
  }
  
  // === 5) ОБЫЧНЫЙ РЕЖИМ (300 СЕКУНД) - только при пустой очереди ===
  if (qCount == 0) {
    unsigned long elapsed = currentMillis - lastNormalSendTime;
    
    if (elapsed >= NORMAL_REPORT_TIME) {
      updateSensors();
      bool ok = sendDataToServer(globalCurrentStatus, true);
      if (ok) lastNormalSendTime = currentMillis;
      else enqueue(globalCurrentStatus);
    } else {
      int secLeft = (NORMAL_REPORT_TIME - elapsed) / 1000;
      showNormalDisplay(sensorTemp, sensorPress, sensorHum, 
                       globalCurrentStatus, secLeft, NORMAL_REPORT_TIME / 1000);
    }
  } else {
    // Доработка 1: Во время обработки очереди показываем СОСТОЯНИЕ ВХОДОВ, а не таймер
    showQueueDisplay(globalCurrentStatus, qCount);
  }
  
  // === 6) ФОНОВОЕ ОБНОВЛЕНИЕ ДАТЧИКОВ ===
  static unsigned long lastSensorUpdate = 0;
  if (isSensorAvailable && (currentMillis - lastSensorUpdate >= 5000)) {
    updateSensors();
    lastSensorUpdate = currentMillis;
  }
  
  delay(10);
}
