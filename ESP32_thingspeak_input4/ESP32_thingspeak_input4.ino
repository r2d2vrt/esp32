//1) Антидребезг (0.3 сек): Состояние каждого входа считается изменившимся, только если оно
//2) стабильно удерживается в новом положении более 300 мс.
//3) Мгновенная реакция: Как только обнаружено стабильное изменение, плата мгновенно включает OUT0,
//ставит статус в очередь на отправку и выводит информацию на экран.
//4) Блокировка на 30 сек: После фиксации изменения включается таймер блокировки на 30 секунд.
//Очередь событий: Если во время этих 30 секунд (пока горит OUT0 и идет отсчет) другие входы
//изменят свое состояние, система зафиксирует это изменение, сформирует новую строку состояния
//и поставит её в очередь (массив). Текущие 30 секунд не прерываются. Как только они истекут,
//плата сразу отправит следующее событие из очереди и запустит новые 30 секунд.
//5) Обычный режим: Если очередь пуста и тревог нет, каждые 300 секунд отправляется текущее базовое состояние.
//6) добавлен AHT20 + BMP280
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Датчики климата
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

// --- Параметры алгоритма ---
const unsigned long DEBOUNCE_DELAY = 300;        // 0.3 секунды фильтр дребезга
const unsigned long ALARM_LOCK_TIME = 30000;     // 30 секунд блокировки отправки
const unsigned long NORMAL_REPORT_TIME = 300000; // 300 секунд обычный период

// --- Переменные для антидребезга ---
const int numInputs = 4;
const int inputPins[numInputs] = {IN0, IN1, IN2, IN3};
int lastSteadyStates[numInputs] = {HIGH, HIGH, HIGH, HIGH}; 
int lastDebounceStates[numInputs] = {HIGH, HIGH, HIGH, HIGH};
unsigned long lastDebounceTimes[numInputs] = {0, 0, 0, 0};

// --- Переменные состояний и таймеров ---
String globalCurrentStatus = "1111"; 
unsigned long lastNormalSendTime = 0;
unsigned long alarmStartTime = 0;
bool isAlarmLocked = false;

// --- Климатические переменные ---
float sensorTemp = 0.0;
float sensorHum = 0.0;
float sensorPress = 0.0;

// --- Очередь сообщений ---
#define QUEUE_MAX_SIZE 10
String alarmQueue[QUEUE_MAX_SIZE];
int queueCount = 0;

// Функция добавления события в очередь
void enqueueAlarm(String status) {
  if (queueCount < QUEUE_MAX_SIZE) {
    alarmQueue[queueCount] = status;
    queueCount++;
    Serial.println("Added to queue: " + status + " (Queue size: " + String(queueCount) + ")");
  } else {
    Serial.println("Queue overflow! Event lost.");
  }
}

// Функция получения события из очереди
String dequeueAlarm() {
  if (queueCount == 0) return "";
  String nextStatus = alarmQueue[0];
  for (int i = 0; i < queueCount - 1; i++) {
    alarmQueue[i] = alarmQueue[i + 1];
  }
  queueCount--;
  return nextStatus;
}

// Чтение физических пинов прямо сейчас (без фильтра)
String readRawInputs() {
  return String(digitalRead(IN0)) + String(digitalRead(IN1)) + String(digitalRead(IN2)) + String(digitalRead(IN3));
}

// Вывод на дисплей (включая строку климата или ошибку датчика)
void showMessage(String title, String status, int progressSeconds = -1, int maxPeriod = 300) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  
  display.setCursor(0, 16);
  display.setTextSize(2); 
  display.println(status);
  
  // Дополнительная строка для отображения климата ниже ST:xxxx
  display.setCursor(0, 36);
  display.setTextSize(1);
  if (!isSensorAvailable) {
    display.println("NO BMP280");
  } else {
    display.print("T:"); display.print(sensorTemp, 1);
    display.print("C H:"); display.print(sensorHum, 0);
    display.print("% P:"); display.print(sensorPress, 0);
  }
  
  if (progressSeconds >= 0 && maxPeriod > 0) {
    int barWidth = map(progressSeconds, 0, maxPeriod, 0, SCREEN_WIDTH);
    if (barWidth > SCREEN_WIDTH) barWidth = SCREEN_WIDTH;
    display.fillRect(0, SCREEN_HEIGHT - 3, barWidth, 3, SSD1306_WHITE);
  }
  display.display();
}

// Отправка данных на ThingSpeak
void sendDataToServer(String payload) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = serverName + "?api_key=" + apiKey + "&field1=" + payload;
    
    // Если это обычный 300-сек отчет (длина строки 4 символа, например "1111") и датчик работает
    if (payload.length() == 4 && isSensorAvailable) {
      url += "&field2=" + String(sensorTemp, 2);
      url += "&field3=" + String(sensorPress, 2);
      url += "&field4=" + String(sensorHum, 2);
    }
    
    Serial.println("Sending request: " + url);
    digitalWrite(LED_PIN, HIGH); 
    showMessage("Sending to Cloud", "ST=" + payload, -1); 

    http.begin(url);
    int httpResponseCode = http.GET();
    
    if (httpResponseCode == 200) {
      Serial.println("HTTP Response: 200 (OK)");
      showMessage("Server Response", "RX OK", -1);
    } else {
      Serial.println("HTTP Error code: " + String(httpResponseCode));
      showMessage("Server Error", "RX ERR", -1);
    }
    digitalWrite(LED_PIN, LOW); 
    delay(500); // Короткая пауза для индикации на экране
    http.end();
  } else {
    Serial.println("WiFi Disconnected. Cannot send data.");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(OUT0, OUTPUT);
  pinMode(OUT1, OUTPUT);
  digitalWrite(OUT0, LOW); 
  digitalWrite(OUT1, LOW);

  for (int i = 0; i < numInputs; i++) {
    pinMode(inputPins[i], INPUT_PULLUP);
    int startVal = digitalRead(inputPins[i]);
    lastSteadyStates[i] = startVal;
    lastDebounceStates[i] = startVal;
  }

  globalCurrentStatus = readRawInputs();

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED init failed"));
    for(;;); 
  }
  display.clearDisplay();
  display.display();

  // Инициализация датчиков AHT20 и BMP280 (0x76 — стандартный адрес модуля)
  if (aht.begin() && bmp.begin(0x77)) { 
    isSensorAvailable = true;
    Serial.println("AHT20+BMP280 detected successfully.");
  } else {
    isSensorAvailable = false;
    Serial.println("AHT20+BMP280 NOT found!");
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    Serial.print(".");
    showMessage("WiFi Connecting...", "...", -1);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }

  Serial.println("\nConnected!");
  showMessage(String(ssid), "connected", -1);
  delay(1500); 
  
  // Первый опрос датчиков перед стартовой отправкой
  if (isSensorAvailable) {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    sensorTemp = bmp.readTemperature();
    sensorHum = humidity.relative_humidity;
    sensorPress = bmp.readPressure() * 0.00750062; // Конвертация Паскалей в мм рт. ст.
  }

  lastNormalSendTime = millis();
  // Первая отправка при запуске (включая климат)
  sendDataToServer(globalCurrentStatus);
}

void loop() {
  // Проверка связи Wi-Fi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Disconnected. Reconnecting...");
    showMessage("WiFi Error", "RECONN", -1);
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(2000);
    return; 
  }

  unsigned long currentMillis = millis();
  bool stateChangedEvent = false;

  // --- 1) ФИЛЬТР ДРЕБЕЗГА КОНТАКТОВ (0.3 СЕКУНДЫ) ---
  for (int i = 0; i < numInputs; i++) {
    int rawReading = digitalRead(inputPins[i]);

    if (rawReading != lastDebounceStates[i]) {
      lastDebounceTimes[i] = currentMillis;
      lastDebounceStates[i] = rawReading;
    }

    if ((currentMillis - lastDebounceTimes[i]) >= DEBOUNCE_DELAY) {
      if (rawReading != lastSteadyStates[i]) {
        lastSteadyStates[i] = rawReading;
        stateChangedEvent = true; 
      }
    }
  }

  // Обновляем строковое представление текущего устойчивого состояния
  if (stateChangedEvent) {
    String newStatus = "";
    for (int i = 0; i < numInputs; i++) {
      newStatus += String(lastSteadyStates[i]);
    }
    globalCurrentStatus = newStatus;

    // Сразу ставим новое тревожное состояние "1xxxx" в очередь
    enqueueAlarm("1" + globalCurrentStatus);
  }

  // --- 2) УПРАВЛЕНИЕ РЕЖИМОМ ТРЕВОГИ И ОЧЕРЕДЬЮ ---
  if (isAlarmLocked) {
    long remainingLockTime = ALARM_LOCK_TIME - (currentMillis - alarmStartTime);
    
    if (remainingLockTime <= 0) {
      isAlarmLocked = false;
      digitalWrite(OUT0, LOW);
      Serial.println("30s Lock ended. OUT0 turned OFF.");
    } else {
      int secondsLeft = (remainingLockTime / 1000) + 1;
      String titleText = "ALARM LOCK (Q:" + String(queueCount) + ")";
      showMessage(titleText, "ST:" + globalCurrentStatus, secondsLeft, 30);
    }
  }

  // Если система не заблокирована таймером и в очереди есть данные — отправляем мгновенно
  if (!isAlarmLocked && queueCount > 0) {
    String nextAlarmMessage = dequeueAlarm();
    
    digitalWrite(OUT0, HIGH); 
    sendDataToServer(nextAlarmMessage); // Отправка без прикрепления климата (длина строки 5 символов)
    
    alarmStartTime = millis();
    isAlarmLocked = true; 
    lastNormalSendTime = millis(); 
  }

  // --- 3) ОБЫЧНЫЙ РЕЖИМ ОТПРАВКИ (ФОРМАТ "xxxx" РАЗ В 300 СЕК) ---
  if (!isAlarmLocked && queueCount == 0) {
    long remainingNormalTime = NORMAL_REPORT_TIME - (currentMillis - lastNormalSendTime);
    
    if (remainingNormalTime <= 0) {
      // Опрос датчиков строго в начале нового цикла ожидания (перед отправкой)
      if (isSensorAvailable) {
        sensors_event_t humidity, temp;
        aht.getEvent(&humidity, &temp);
        sensorTemp = bmp.readTemperature();
        sensorHum = humidity.relative_humidity;
        sensorPress = bmp.readPressure() * 0.00750062; // В мм рт. ст.
      }

      sendDataToServer(globalCurrentStatus);
      lastNormalSendTime = millis();
    } else {
      int normalSecondsLeft = remainingNormalTime / 1000;
      showMessage(String(ssid), "ST:" + globalCurrentStatus, normalSecondsLeft, 300);
    }
  }

  delay(10); 
}
