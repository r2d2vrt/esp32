/*
 * =====================================================================
 * Назначение выводов (Pinout) Raspberry Pi Pico (RP2040):
 * =====================================================================
 * GP0  (UART0 TX) - [Резерв] Передача данных на преобразователь UART-USB
 * GP1  (UART0 RX) - [Резерв] Прием данных от преобразователя UART-USB
 * GP4  (I2C0 SDA) - Шина данных SDA: OLED SSD1306 (0x3C) + AHT20 (0x38) + BMP280 (0x76/0x77)
 * GP5  (I2C0 SCL) - Шина тактирования SCL: OLED SSD1306 + AHT20 + BMP280
 * GP6  (GPIO IN)  - Контактный датчик 1 (прерывание CHANGE, без подтяжки, 0/3.3В)
 * GP7  (GPIO IN)  - Контактный датчик 2 (прерывание CHANGE, без подтяжки, 0/3.3В)
 * GP8  (GPIO IN)  - Контактный датчик 3 (прерывание CHANGE, без подтяжки, 0/3.3В)
 * GP9  (GPIO IN)  - Контактный датчик 4 (прерывание CHANGE, без подтяжки, 0/3.3В)
 * GP25 (GPIO OUT) - Встроенный светодиод на плате (индикатор работы / PWM)
 * =====================================================================
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// Настройки OLED-дисплея
#define SCREEN_WIDTH 128    // Ширина дисплея в пикселях
#define SCREEN_HEIGHT 64    // Высота дисплея в пикселях
#define OLED_RESET -1       // Пин сброса (-1, если общий с Pico)
#define SCREEN_ADDRESS 0x3C // I2C адрес OLED-дисплея

// Пины периферии
const uint8_t I2C_SDA_PIN     = 4;
const uint8_t I2C_SCL_PIN     = 5;
const uint8_t INPUT_PIN_1     = 6;
const uint8_t INPUT_PIN_2     = 7;
const uint8_t INPUT_PIN_3     = 8;
const uint8_t INPUT_PIN_4     = 9;
const uint8_t ONBOARD_LED_PIN = 25;

// Массив пинов датчиков для удобного опроса
const uint8_t INPUT_PINS[4] = {INPUT_PIN_1, INPUT_PIN_2, INPUT_PIN_3, INPUT_PIN_4};

// Создаем объекты дисплея и датчиков
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

// Флаги состояния датчиков I2C
bool sensorsReady = false;

// Переменные для обработки прерываний от входов GP6-GP9
volatile bool inputsChanged = false;
volatile unsigned long lastInterruptTime = 0;
const unsigned long DEBOUNCE_DELAY = 20; // Защита от дребезга контактов (20 мс)

// Переменные для неблокирующего fade светодиода
int brightness = 0;
int fadeAmount = 5;
unsigned long prevFadeMillis = 0;
const unsigned long FADE_INTERVAL = 20;

// Переменные для неблокирующего опроса AHT20+BMP280 (раз в 5 секунд)
unsigned long prevSensorMillis = 0;
const unsigned long SENSOR_INTERVAL = 5000;

// Обработчик прерывания по изменению уровня на любом из входов GP6-GP9
void onInputChangeISR() {
  inputsChanged = true;
  lastInterruptTime = millis();
}

void setup() {
  // 1. Настройка встроенного светодиода
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  analogWriteResolution(8);

  // 2. Настройка входов GP6-GP9 без подтяжки и привязка прерываний по изменению уровня (CHANGE)
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(INPUT_PINS[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(INPUT_PINS[i]), onInputChangeISR, CHANGE);
  }

  // 3. Инициализация шины I2C0 на пинах GP4 и GP5
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  // 4. Инициализация OLED-дисплея
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    while (true) {
      digitalWrite(ONBOARD_LED_PIN, HIGH);
      delay(100);
      digitalWrite(ONBOARD_LED_PIN, LOW);
      delay(100);
    }
  }

  // 5. Инициализация датчиков AHT20 и BMP280
  initSensors();

  // 6. Первичная отрисовка всех данных на дисплее
  display.clearDisplay();
  updateSensorsArea();
  updateInputsArea();
  display.display();
}

void loop() {
  // 1. Плавное дыхание светодиода в фоне
  updateLedFade();

  unsigned long currentMillis = millis();

  // 2. Проверка флага прерывания от входов GP6-GP9 (с учетом времени антидребезга)
  if (inputsChanged && (currentMillis - lastInterruptTime >= DEBOUNCE_DELAY)) {
    inputsChanged = false;
    updateInputsArea();
    display.display();
  }

  // 3. Опрос датчиков температуры, давления и влажности каждые 5 секунд
  if (currentMillis - prevSensorMillis >= SENSOR_INTERVAL) {
    prevSensorMillis = currentMillis;
    updateSensorsArea();
    display.display();
  }
}

// Функция проверки и инициализации обоих чипов на модуле AHT20+BMP280
bool initSensors() {
  bool ahtOk = aht.begin(&Wire);
  bool bmpOk = bmp.begin(0x77) || bmp.begin(0x76);

  sensorsReady = (ahtOk && bmpOk);
  return sensorsReady;
}

// Обновление верхней области экрана (Y = 0..43): T, P, H или "No Sensor"
void updateSensorsArea() {
  // Очищаем только верхнюю зону экрана, не затрагивая строку состояний входов
  display.fillRect(0, 0, SCREEN_WIDTH, 44, SSD1306_BLACK);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  if (!sensorsReady) {
    initSensors();
  }

  if (!sensorsReady) {
    display.setCursor(36, 18);
    display.println(F("No Sensor"));
    return;
  }

  sensors_event_t humidityEvent, tempEvent;
  if (!aht.getEvent(&humidityEvent, &tempEvent)) {
    sensorsReady = false;
    display.setCursor(36, 18);
    display.println(F("No Sensor"));
    return;
  }

  float pressurePa = bmp.readPressure();
  if (isnan(pressurePa) || pressurePa <= 0) {
    sensorsReady = false;
    display.setCursor(36, 18);
    display.println(F("No Sensor"));
    return;
  }

  float tempC      = tempEvent.temperature;
  float pressureMm = pressurePa / 133.322;
  float humPercent = humidityEvent.relative_humidity;

  // Строка 1: Температура
  display.setCursor(0, 4);
  display.print(F("T= "));
  display.print(tempC, 2);
  display.println(F(" C"));

  // Строка 2: Давление
  display.setCursor(0, 16);
  display.print(F("P= "));
  display.print(pressureMm, 2);
  display.println(F(" mm"));

  // Строка 3: Влажность
  display.setCursor(0, 28);
  display.print(F("H= "));
  display.print(humPercent, 2);
  display.println(F(" %"));
}

// Обновление нижней области экрана (Y = 46..63): состояние входов GP6-GP9 в виде 0110
void updateInputsArea() {
  // Очищаем только нижнюю зону экрана
  display.fillRect(0, 46, SCREEN_WIDTH, 18, SSD1306_BLACK);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Тонкая разделительная линия для визуальной аккуратности
  display.drawFastHLine(0, 44, SCREEN_WIDTH, SSD1306_WHITE);

  // Выводим текущие логические уровни GP6, GP7, GP8, GP9 слева направо
  display.setCursor(0, 50);
  for (uint8_t i = 0; i < 4; i++) {
    int state = digitalRead(INPUT_PINS[i]);
    display.print(state == HIGH ? '1' : '0');
  }
}

// Функция плавного изменения яркости светодиода
void updateLedFade() {
  unsigned long currentMillis = millis();

  if (currentMillis - prevFadeMillis >= FADE_INTERVAL) {
    prevFadeMillis = currentMillis;
    analogWrite(ONBOARD_LED_PIN, brightness);
    brightness += fadeAmount;

    if (brightness <= 0 || brightness >= 255) {
      fadeAmount = -fadeAmount;
    }
  }
}
