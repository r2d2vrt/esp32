// 1. Подключение WiFi с таймаутом 5 секунд
//  При старте система пытается подключиться к WiFi максимум 5 секунд
//  Если не удалось — выводится «WiFi – NO !!!» и работа продолжается без отправки данных
// 2. Основной цикл (300 секунд)
//  Чтение IN0–IN3 и отправка в поле 1 в формате «xxxx»
//  Датчик 1 (BMP280+AHT20): поле 2 — температура (TOUT=), поле 3 — давление мм рт.ст. (P=), поле 4 — влажность (H=)
//  Датчик 2 (DS18B20): поле 5 — температура (TIN=, справа от TOUT)
//  При отсутствии датчиков — нулевые значения на дисплее и сервере
//  При отсутствии WiFi — данные отображаются, но не отправляются
//  Внизу дисплея — убывающая полоска оставшегося времени
// 3. Цикл ТРЕВОГИ
// IN0:
// Отправка «1xxxx», повтор каждые 30 сек до RX OK
// OUT1 — немедленно
// OUT0 — через 15 секунд
// Через 120 секунд — оба выхода в 0
// Процессы отправки и управления выходами параллельны
// IN1:
// Отправка «1xxxx», повтор каждые 30 сек до RX OK
// OUT0 и OUT1 — немедленно
// Через 120 секунд — оба выхода в 0
// Параллельные процессы
// IN2/IN3:
// Отправка «1xxxx», повтор каждые 30 сек до RX OK
// Выходы НЕ активируются
// Возврат к основному циклу после RX OK
// Множественные изменения:
// Все события в очередь
// Отправка каждые 30 сек с подтверждением
// На дисплее — количество неотправленных сообщений
// Полоска обратного отсчёта до следующей отправки
// 4. Дополнительные возможности
// Автоматическое переподключение WiFi при потере связи
// Антидребезг входов 300 мс
// Асинхронный опрос DS18B20 (не блокирует выполнение)
// Подробный лог в Serial Monitor для отладки
//
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ============================================================================
// КОНФИГУРАЦИЯ УСТРОЙСТВА
// ============================================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Пины входов/выходов
#define LED_PIN 2
#define IN0 25
#define IN1 26
#define IN2 27
#define IN3 13
#define OUT0 32
#define OUT1 33
#define DS18B20_PIN 4

// WiFi и ThingSpeak
const char* ssid = "Wally";
const char* password = "23051976";
const String serverName = "http://api.thingspeak.com/update";
const String apiKey = "ssc48jt2fzxhxorj";

// ============================================================================
// ТАЙМЕРЫ И КОНСТАНТЫ
// ============================================================================
const unsigned long DEBOUNCE_DELAY = 300;          // Антидребезг 300мс
const unsigned long WIFI_CONNECT_TIMEOUT = 5000;   // Таймаут WiFi 5 сек
const unsigned long MAIN_CYCLE_DURATION = 300000;  // Основной цикл 300 сек
const unsigned long ALARM_RETRY_INTERVAL = 30000;  // Повтор тревоги 30 сек
const unsigned long ALARM_OUT_DURATION = 120000;   // Длительность OUT 120 сек
const unsigned long ALARM_IN0_OUT0_DELAY = 15000;  // Задержка OUT0 при IN0 15 сек

// ============================================================================
// ОБЪЕКТЫ И ПЕРЕМЕННЫЕ
// ============================================================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
OneWire oneWire(DS18B20_PIN);
DallasTemperature dsSensor(&oneWire);

// Состояния системы
enum SystemState {
  STATE_MAIN,
  STATE_ALARM_IN0,
  STATE_ALARM_IN1,
  STATE_ALARM_IN23,
  STATE_ALARM_QUEUE
};

SystemState currentState = STATE_MAIN;

// WiFi статус
bool wifiConnected = false;

// Датчики
bool i2cSensorAvailable = false;
bool ds18b20Connected = false;
float sensorTemp = 0.0;    // TOUT (BMP280)
float sensorHum = 0.0;     // H (AHT20)
float sensorPress = 0.0;   // P (BMP280, mmHg)
float sensorTempDS = 0.0;  // TIN (DS18B20)

// Входы
const int numInputs = 4;
const int inputPins[numInputs] = {IN0, IN1, IN2, IN3};
int lastDebounceStates[numInputs];
int lastSteadyStates[numInputs];
int previousSteadyStates[numInputs];
unsigned long lastDebounceTimes[numInputs];
String currentInputStatus = "1111";

// Основной цикл
unsigned long mainCycleStartTime = 0;

// Тревога
unsigned long alarmStartTime = 0;
String alarmMessage = "";
bool alarmMessageDelivered = false;
unsigned long alarmLastSendTime = 0;

// Управление выходами в тревоге
unsigned long alarmOut0ActivateTime = 0;
unsigned long alarmOut1ActivateTime = 0;
unsigned long alarmOutDeactivateTime = 0;
bool alarmOut0Activated = false;
bool alarmOut1Activated = false;

// Очередь событий
#define QUEUE_MAX_SIZE 20
String eventQueue[QUEUE_MAX_SIZE];
int queueHead = 0;
int queueCount = 0;
unsigned long queueLastSendTime = 0;

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("System starting...");
  
  // Инициализация пинов
  pinMode(LED_PIN, OUTPUT);
  pinMode(OUT0, OUTPUT);
  pinMode(OUT1, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(OUT0, LOW);
  digitalWrite(OUT1, LOW);
  
  for (int i = 0; i < numInputs; i++) {
    pinMode(inputPins[i], INPUT_PULLUP);
    int val = digitalRead(inputPins[i]);
    lastDebounceStates[i] = lastSteadyStates[i] = previousSteadyStates[i] = val;
    lastDebounceTimes[i] = 0;
  }
  
  currentInputStatus = String(digitalRead(IN0)) + String(digitalRead(IN1)) +
                       String(digitalRead(IN2)) + String(digitalRead(IN3));
  
  // Инициализация OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED init failed!");
    while (true);
  }
  display.clearDisplay();
  display.display();
  showStatusMessage("System", "Starting...");
  delay(500);
  
  // Инициализация I2C датчиков (BMP280 + AHT20)
  if (aht.begin() && bmp.begin(0x77)) {
    i2cSensorAvailable = true;
    Serial.println("I2C Sensors OK (AHT20+BMP280)");
  } else {
    i2cSensorAvailable = false;
    Serial.println("I2C Sensors NOT found!");
  }
  
  // Инициализация DS18B20
  pinMode(DS18B20_PIN, INPUT_PULLUP);
  dsSensor.begin();
  int dsDevices = dsSensor.getDeviceCount();
  if (dsDevices > 0) {
    ds18b20Connected = true;
    dsSensor.setResolution(9);
    dsSensor.setWaitForConversion(false);
    Serial.println("DS18B20 found on GPIO4");
  } else {
    ds18b20Connected = false;
    sensorTempDS = 0.0;
    Serial.println("DS18B20 NOT found on GPIO4");
  }
  
  // Подключение к WiFi с таймаутом 5 секунд
  showStatusMessage("WiFi", "Connecting...");
  WiFi.begin(ssid, password);
  unsigned long wifiStartTime = millis();
  
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < WIFI_CONNECT_TIMEOUT) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("WiFi Connected!");
    showStatusMessage("WiFi", "Connected!");
    delay(1000);
  } else {
    wifiConnected = false;
    Serial.println("WiFi - NO !!!");
    showStatusMessage("WiFi - NO !!!", "No connection");
    delay(2000);
  }
  
  // Первоначальное чтение датчиков
  updateSensors();
  
  // Отправка начальных данных на сервер
  if (wifiConnected) {
    sendMainData();
  }
  
  // Запуск основного цикла
  mainCycleStartTime = millis();
  currentState = STATE_MAIN;
  Serial.println("Main cycle started");
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long currentMillis = millis();
  
  // Мониторинг входов только в основном цикле
  if (currentState == STATE_MAIN) {
    // Сохранение предыдущих состояний
    for (int i = 0; i < numInputs; i++) {
      previousSteadyStates[i] = lastSteadyStates[i];
    }
    
    // Чтение входов с антидребезгом
    bool inputChanged = readInputs(currentMillis);
    
    // Если вход изменился - переход к тревоге
    if (inputChanged) {
      enterAlarm(currentMillis);
    }
  }
  
  // Обработка текущего состояния
  switch (currentState) {
    case STATE_MAIN:
      handleMainCycle(currentMillis);
      break;
    case STATE_ALARM_IN0:
      handleAlarmIN0(currentMillis);
      break;
    case STATE_ALARM_IN1:
      handleAlarmIN1(currentMillis);
      break;
    case STATE_ALARM_IN23:
      handleAlarmIN23(currentMillis);
      break;
    case STATE_ALARM_QUEUE:
      handleAlarmQueue(currentMillis);
      break;
  }
  
  delay(10);
}

// ============================================================================
// ЧТЕНИЕ ВХОДОВ С АНТИБРЕЗГОМ
// ============================================================================
bool readInputs(unsigned long currentMillis) {
  bool changed = false;
  
  for (int i = 0; i < numInputs; i++) {
    int reading = digitalRead(inputPins[i]);
    
    if (reading != lastDebounceStates[i]) {
      lastDebounceTimes[i] = currentMillis;
      lastDebounceStates[i] = reading;
    }
    
    if ((currentMillis - lastDebounceTimes[i]) >= DEBOUNCE_DELAY) {
      if (reading != lastSteadyStates[i]) {
        lastSteadyStates[i] = reading;
        changed = true;
      }
    }
  }
  
  if (changed) {
    currentInputStatus = "";
    for (int i = 0; i < numInputs; i++) {
      currentInputStatus += String(lastSteadyStates[i]);
    }
    Serial.println("Inputs changed: " + currentInputStatus);
  }
  
  return changed;
}

// ============================================================================
// ПЕРЕХОД К ТРЕВОГЕ
// ============================================================================
void enterAlarm(unsigned long currentMillis) {
  // Определение изменившихся входов
  int changedInputs[numInputs];
  int numChanged = 0;
  
  for (int i = 0; i < numInputs; i++) {
    if (lastSteadyStates[i] != previousSteadyStates[i]) {
      changedInputs[numChanged++] = i;
    }
  }
  
  Serial.println("ALARM: " + String(numChanged) + " input(s) changed");
  
  // Инициализация тревоги
  alarmStartTime = currentMillis;
  alarmMessage = "1" + currentInputStatus;
  alarmMessageDelivered = false;
  alarmLastSendTime = 0; // Немедленная первая отправка
  
  if (numChanged == 1) {
    // Одиночное изменение
    int changedInput = changedInputs[0];
    
    if (changedInput == 0) {
      // IN0: OUT1 сразу, OUT0 через 15 сек, всё выключить через 120 сек
      currentState = STATE_ALARM_IN0;
      alarmOut1ActivateTime = currentMillis;
      alarmOut1Activated = false;
      alarmOut0ActivateTime = currentMillis + ALARM_IN0_OUT0_DELAY;
      alarmOut0Activated = false;
      alarmOutDeactivateTime = currentMillis + ALARM_OUT_DURATION;
      Serial.println("Alarm type: IN0");
      
    } else if (changedInput == 1) {
      // IN1: OUT0 и OUT1 сразу, всё выключить через 120 сек
      currentState = STATE_ALARM_IN1;
      alarmOut0ActivateTime = currentMillis;
      alarmOut0Activated = false;
      alarmOut1ActivateTime = currentMillis;
      alarmOut1Activated = false;
      alarmOutDeactivateTime = currentMillis + ALARM_OUT_DURATION;
      Serial.println("Alarm type: IN1");
      
    } else {
      // IN2 или IN3: без активации выходов
      currentState = STATE_ALARM_IN23;
      Serial.println("Alarm type: IN2/IN3");
    }
    
  } else {
    // Множественное изменение - очередь
    currentState = STATE_ALARM_QUEUE;
    enqueue(alarmMessage);
    queueLastSendTime = 0; // Немедленная первая отправка
    Serial.println("Alarm type: QUEUE (" + String(numChanged) + " inputs)");
  }
}

// ============================================================================
// ОБРАБОТКА ОСНОВНОГО ЦИКЛА
// ============================================================================
void handleMainCycle(unsigned long currentMillis) {
  // Попытка переподключения WiFi если потеряно
  if (!wifiConnected) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      Serial.println("WiFi reconnected!");
    } else if (currentMillis % 10000 < 100) {
      // Попытка переподключения каждые 10 секунд
      WiFi.reconnect();
    }
  }
  
  unsigned long elapsed = currentMillis - mainCycleStartTime;
  
  if (elapsed >= MAIN_CYCLE_DURATION) {
    // Время отправки данных
    updateSensors();
    
    if (wifiConnected) {
      sendMainData();
    } else {
      Serial.println("WiFi not connected - data not sent");
    }
    
    mainCycleStartTime = currentMillis;
  }
  
  // Отображение данных с обратным отсчетом
  int secLeft = (MAIN_CYCLE_DURATION - (currentMillis - mainCycleStartTime)) / 1000;
  if (secLeft < 0) secLeft = 0;
  displayMain(secLeft, MAIN_CYCLE_DURATION / 1000);
}

// ============================================================================
// ОБРАБОТКА ТРЕВОГИ IN0
// ============================================================================
void handleAlarmIN0(unsigned long currentMillis) {
  // Отправка сообщения (параллельный процесс)
  if (!alarmMessageDelivered) {
    if (alarmLastSendTime == 0 || (currentMillis - alarmLastSendTime >= ALARM_RETRY_INTERVAL)) {
      if (wifiConnected || WiFi.status() == WL_CONNECTED) {
        if (!wifiConnected) wifiConnected = true;
        bool ok = sendAlarmMessage(alarmMessage);
        alarmLastSendTime = currentMillis;
        if (ok) {
          alarmMessageDelivered = true;
          Serial.println("Alarm IN0: RX OK");
        } else {
          Serial.println("Alarm IN0: RX ERR - retry in 30s");
        }
      }
    }
  }
  
  // Управление выходами (параллельный процесс)
  // OUT1 - немедленно
  if (!alarmOut1Activated && currentMillis >= alarmOut1ActivateTime) {
    digitalWrite(OUT1, HIGH);
    alarmOut1Activated = true;
    Serial.println("Alarm IN0: OUT1 ON");
  }
  
  // OUT0 - через 15 секунд
  if (!alarmOut0Activated && currentMillis >= alarmOut0ActivateTime) {
    digitalWrite(OUT0, HIGH);
    alarmOut0Activated = true;
    Serial.println("Alarm IN0: OUT0 ON");
  }
  
  // Выключение через 120 секунд
  if (currentMillis >= alarmOutDeactivateTime) {
    digitalWrite(OUT0, LOW);
    digitalWrite(OUT1, LOW);
    Serial.println("Alarm IN0: OUT0, OUT1 OFF");
  }
  
  // Отображение
  unsigned long elapsed = currentMillis - alarmStartTime;
  int secLeft = (ALARM_OUT_DURATION - elapsed) / 1000;
  if (secLeft < 0) secLeft = 0;
  displayAlarmIN0(currentInputStatus, alarmMessageDelivered, secLeft, ALARM_OUT_DURATION / 1000);
  
  // Завершение тревоги (когда истекло 120 сек И сообщение доставлено)
  if (currentMillis >= alarmOutDeactivateTime && alarmMessageDelivered) {
    Serial.println("Alarm IN0: Complete - returning to main cycle");
    currentState = STATE_MAIN;
    mainCycleStartTime = currentMillis;
  }
}

// ============================================================================
// ОБРАБОТКА ТРЕВОГИ IN1
// ============================================================================
void handleAlarmIN1(unsigned long currentMillis) {
  // Отправка сообщения
  if (!alarmMessageDelivered) {
    if (alarmLastSendTime == 0 || (currentMillis - alarmLastSendTime >= ALARM_RETRY_INTERVAL)) {
      if (wifiConnected || WiFi.status() == WL_CONNECTED) {
        if (!wifiConnected) wifiConnected = true;
        bool ok = sendAlarmMessage(alarmMessage);
        alarmLastSendTime = currentMillis;
        if (ok) {
          alarmMessageDelivered = true;
          Serial.println("Alarm IN1: RX OK");
        } else {
          Serial.println("Alarm IN1: RX ERR - retry in 30s");
        }
      }
    }
  }
  
  // Управление выходами
  // OUT0 и OUT1 - немедленно
  if (!alarmOut0Activated && currentMillis >= alarmOut0ActivateTime) {
    digitalWrite(OUT0, HIGH);
    alarmOut0Activated = true;
    Serial.println("Alarm IN1: OUT0 ON");
  }
  
  if (!alarmOut1Activated && currentMillis >= alarmOut1ActivateTime) {
    digitalWrite(OUT1, HIGH);
    alarmOut1Activated = true;
    Serial.println("Alarm IN1: OUT1 ON");
  }
  
  // Выключение через 120 секунд
  if (currentMillis >= alarmOutDeactivateTime) {
    digitalWrite(OUT0, LOW);
    digitalWrite(OUT1, LOW);
    Serial.println("Alarm IN1: OUT0, OUT1 OFF");
  }
  
  // Отображение
  unsigned long elapsed = currentMillis - alarmStartTime;
  int secLeft = (ALARM_OUT_DURATION - elapsed) / 1000;
  if (secLeft < 0) secLeft = 0;
  displayAlarmIN1(currentInputStatus, alarmMessageDelivered, secLeft, ALARM_OUT_DURATION / 1000);
  
  // Завершение тревоги
  if (currentMillis >= alarmOutDeactivateTime && alarmMessageDelivered) {
    Serial.println("Alarm IN1: Complete - returning to main cycle");
    currentState = STATE_MAIN;
    mainCycleStartTime = currentMillis;
  }
}

// ============================================================================
// ОБРАБОТКА ТРЕВОГИ IN2/IN3
// ============================================================================
void handleAlarmIN23(unsigned long currentMillis) {
  // Отправка сообщения
  if (!alarmMessageDelivered) {
    if (alarmLastSendTime == 0 || (currentMillis - alarmLastSendTime >= ALARM_RETRY_INTERVAL)) {
      if (wifiConnected || WiFi.status() == WL_CONNECTED) {
        if (!wifiConnected) wifiConnected = true;
        bool ok = sendAlarmMessage(alarmMessage);
        alarmLastSendTime = currentMillis;
        if (ok) {
          alarmMessageDelivered = true;
          Serial.println("Alarm IN23: RX OK");
        } else {
          Serial.println("Alarm IN23: RX ERR - retry in 30s");
        }
      }
    }
  }
  
  // Выходы НЕ активируются
  
  // Отображение
  unsigned long elapsed = currentMillis - alarmLastSendTime;
  int secLeft = ALARM_RETRY_INTERVAL / 1000 - elapsed / 1000;
  if (secLeft < 0) secLeft = 0;
  displayAlarmIN23(currentInputStatus, alarmMessageDelivered, secLeft, ALARM_RETRY_INTERVAL / 1000);
  
  // Завершение тревоги (когда сообщение доставлено)
  if (alarmMessageDelivered) {
    Serial.println("Alarm IN23: Complete - returning to main cycle");
    currentState = STATE_MAIN;
    mainCycleStartTime = currentMillis;
  }
}

// ============================================================================
// ОБРАБОТКА ТРЕВОГИ - ОЧЕРЕДЬ
// ============================================================================
void handleAlarmQueue(unsigned long currentMillis) {
  // Отправка из очереди
  if (queueCount > 0) {
    if (queueLastSendTime == 0 || (currentMillis - queueLastSendTime >= ALARM_RETRY_INTERVAL)) {
      if (wifiConnected || WiFi.status() == WL_CONNECTED) {
        if (!wifiConnected) wifiConnected = true;
        String msg = peekQueue();
        bool ok = sendAlarmMessage(msg);
        queueLastSendTime = currentMillis;
        if (ok) {
          dequeue();
          Serial.println("Queue: RX OK - Remaining: " + String(queueCount));
        } else {
          Serial.println("Queue: RX ERR - retry in 30s");
        }
      }
    }
  }
  
  // Отображение
  unsigned long elapsed = currentMillis - queueLastSendTime;
  int secLeft = ALARM_RETRY_INTERVAL / 1000 - elapsed / 1000;
  if (secLeft < 0) secLeft = 0;
  displayAlarmQueue(currentInputStatus, queueCount, secLeft, ALARM_RETRY_INTERVAL / 1000);
  
  // Завершение тревоги (когда очередь пуста)
  if (queueCount == 0) {
    Serial.println("Queue: All sent - returning to main cycle");
    currentState = STATE_MAIN;
    mainCycleStartTime = currentMillis;
  }
}

// ============================================================================
// ОБНОВЛЕНИЕ ДАТЧИКОВ
// ============================================================================
void updateSensors() {
  // I2C датчики (BMP280 + AHT20)
  if (i2cSensorAvailable) {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    sensorTemp = bmp.readTemperature();
    sensorHum = humidity.relative_humidity;
    sensorPress = bmp.readPressure() * 0.00750062; // Pa → mmHg
  } else {
    sensorTemp = 0.0;
    sensorHum = 0.0;
    sensorPress = 0.0;
  }
  
  // DS18B20
  if (ds18b20Connected) {
    dsSensor.requestTemperatures();
    unsigned long startWait = millis();
    while (!dsSensor.isConversionComplete() && millis() - startWait < 150) {
      delay(1);
    }
    float t = dsSensor.getTempCByIndex(0);
    if (t > -100.0 && t < 150.0) {
      sensorTempDS = t;
    } else {
      sensorTempDS = 0.0;
      Serial.println("DS18B20 read error");
    }
  } else {
    sensorTempDS = 0.0;
  }
  
  Serial.println("Sensors: TOUT=" + String(sensorTemp, 1) + 
                 " TIN=" + String(sensorTempDS, 1) +
                 " P=" + String(sensorPress, 1) + 
                 " H=" + String(sensorHum, 0));
}

// ============================================================================
// ОТПРАВКА ОСНОВНЫХ ДАННЫХ (все поля)
// ============================================================================
bool sendMainData() {
  if (!wifiConnected) return false;
  
  HTTPClient http;
  String url = serverName + "?api_key=" + apiKey;
  url += "&field1=" + currentInputStatus;
  url += "&field2=" + String(sensorTemp, 2);
  url += "&field3=" + String(sensorPress, 2);
  url += "&field4=" + String(sensorHum, 2);
  url += "&field5=" + String(sensorTempDS, 2);
  
  Serial.println("Sending main data...");
  digitalWrite(LED_PIN, HIGH);
  
  http.begin(url);
  int httpCode = http.GET();
  
  bool success = (httpCode == 200);
  Serial.println(success ? "Main data: RX OK" : "Main data: RX ERR " + String(httpCode));
  
  digitalWrite(LED_PIN, LOW);
  http.end();
  return success;
}

// ============================================================================
// ОТПРАВКА СООБЩЕНИЯ ТРЕВОГИ (только поле 1)
// ============================================================================
bool sendAlarmMessage(String msg) {
  if (!wifiConnected) return false;
  
  HTTPClient http;
  String url = serverName + "?api_key=" + apiKey + "&field1=" + msg;
  
  Serial.println("Sending alarm: " + msg);
  digitalWrite(LED_PIN, HIGH);
  
  http.begin(url);
  int httpCode = http.GET();
  
  bool success = (httpCode == 200);
  Serial.println(success ? "Alarm: RX OK" : "Alarm: RX ERR " + String(httpCode));
  
  digitalWrite(LED_PIN, LOW);
  http.end();
  return success;
}

// ============================================================================
// ФУНКЦИИ ОТОБРАЖЕНИЯ
// ============================================================================
void displayMain(int secLeft, int maxPeriod) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Строка 1: TOUT и TIN
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("TOUT=");
  display.print(sensorTemp, 1);
  display.print(" TIN=");
  display.println(sensorTempDS, 1);
  
  // Строка 2: Давление
  display.setCursor(0, 12);
  display.print("P=");
  display.print(sensorPress, 1);
  display.println(" mmHg");
  
  // Строка 3: Влажность
  display.setCursor(0, 24);
  display.print("H=");
  display.print(sensorHum, 0);
  display.println("%");
  
  // Строка 4: Состояние входов
  display.setTextSize(2);
  display.setCursor(0, 36);
  display.print(currentInputStatus);
  
  // Статус WiFi
  display.setTextSize(1);
  display.setCursor(90, 42);
  if (!wifiConnected) {
    display.println("NO WiFi");
  }
  
  // Полоса обратного отсчета
  if (maxPeriod > 0) {
    int barWidth = map(secLeft, 0, maxPeriod, 0, SCREEN_WIDTH);
    display.fillRect(0, SCREEN_HEIGHT - 4, barWidth, 4, SSD1306_WHITE);
    display.drawRect(0, SCREEN_HEIGHT - 4, SCREEN_WIDTH, 4, SSD1306_WHITE);
  }
  
  display.display();
}

void displayAlarmIN0(String inputs, bool delivered, int secLeft, int maxPeriod) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.print("ALARM IN0");
  
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("IN:");
  display.println(inputs);
  
  display.setCursor(0, 37);
  if (delivered) {
    display.println("RX OK");
  } else {
    display.println("Sending...");
  }
  
  display.setCursor(0, 49);
  display.print("OUT1:");
  display.print(alarmOut1Activated ? "ON " : "OFF");
  display.print(" OUT0:");
  display.println(alarmOut0Activated ? "ON" : "OFF");
  
  // Полоса обратного отсчета
  if (maxPeriod > 0) {
    int barWidth = map(secLeft, 0, maxPeriod, 0, SCREEN_WIDTH);
    display.fillRect(0, SCREEN_HEIGHT - 4, barWidth, 4, SSD1306_WHITE);
    display.drawRect(0, SCREEN_HEIGHT - 4, SCREEN_WIDTH, 4, SSD1306_WHITE);
  }
  
  display.display();
}

void displayAlarmIN1(String inputs, bool delivered, int secLeft, int maxPeriod) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.print("ALARM IN1");
  
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("IN:");
  display.println(inputs);
  
  display.setCursor(0, 37);
  if (delivered) {
    display.println("RX OK");
  } else {
    display.println("Sending...");
  }
  
  display.setCursor(0, 49);
  display.print("OUT0:");
  display.print(alarmOut0Activated ? "ON " : "OFF");
  display.print(" OUT1:");
  display.println(alarmOut1Activated ? "ON" : "OFF");
  
  // Полоса обратного отсчета
  if (maxPeriod > 0) {
    int barWidth = map(secLeft, 0, maxPeriod, 0, SCREEN_WIDTH);
    display.fillRect(0, SCREEN_HEIGHT - 4, barWidth, 4, SSD1306_WHITE);
    display.drawRect(0, SCREEN_HEIGHT - 4, SCREEN_WIDTH, 4, SSD1306_WHITE);
  }
  
  display.display();
}

void displayAlarmIN23(String inputs, bool delivered, int secLeft, int maxPeriod) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.print("ALARM IN2/3");
  
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("IN:");
  display.println(inputs);
  
  display.setCursor(0, 37);
  if (delivered) {
    display.println("RX OK");
  } else {
    display.print("Retry in: ");
    display.print(secLeft);
    display.println("s");
  }
  
  display.setCursor(0, 49);
  display.println("OUT: inactive");
  
  // Полоса обратного отсчета
  if (maxPeriod > 0) {
    int barWidth = map(secLeft, 0, maxPeriod, 0, SCREEN_WIDTH);
    display.fillRect(0, SCREEN_HEIGHT - 4, barWidth, 4, SSD1306_WHITE);
    display.drawRect(0, SCREEN_HEIGHT - 4, SCREEN_WIDTH, 4, SSD1306_WHITE);
  }
  
  display.display();
}

void displayAlarmQueue(String inputs, int queueSize, int secLeft, int maxPeriod) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.print("ALARM QUEUE");
  
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("IN:");
  display.println(inputs);
  
  display.setCursor(0, 37);
  display.print("Unsent: ");
  display.print(queueSize);
  display.println(" msg");
  
  display.setCursor(0, 49);
  display.print("Next in: ");
  display.print(secLeft);
  display.println("s");
  
  // Полоса обратного отсчета
  if (maxPeriod > 0) {
    int barWidth = map(secLeft, 0, maxPeriod, 0, SCREEN_WIDTH);
    display.fillRect(0, SCREEN_HEIGHT - 4, barWidth, 4, SSD1306_WHITE);
    display.drawRect(0, SCREEN_HEIGHT - 4, SCREEN_WIDTH, 4, SSD1306_WHITE);
  }
  
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
// ФУНКЦИИ ОЧЕРЕДИ
// ============================================================================
void enqueue(String msg) {
  if (queueCount < QUEUE_MAX_SIZE) {
    int idx = (queueHead + queueCount) % QUEUE_MAX_SIZE;
    eventQueue[idx] = msg;
    queueCount++;
    Serial.println("Enqueue: " + msg + " | Queue: " + String(queueCount));
  } else {
    Serial.println("Queue overflow!");
  }
}

void dequeue() {
  if (queueCount == 0) return;
  queueHead = (queueHead + 1) % QUEUE_MAX_SIZE;
  queueCount--;
  Serial.println("Dequeue | Remaining: " + String(queueCount));
}

String peekQueue() {
  if (queueCount == 0) return "";
  return eventQueue[queueHead];
}
