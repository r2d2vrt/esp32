/*
 * =====================================================================
 * Назначение выводов (Pinout) Raspberry Pi Pico (RP2040):
 * =====================================================================
 * GP0  (UART0 TX) - [Резерв] Передача данных на преобразователь UART-USB
 * GP1  (UART0 RX) - [Резерв] Прием данных от преобразователя UART-USB
 * GP4  (I2C0 SDA) - Линия данных SDA для OLED-дисплея SSD1306 (128x64)
 * GP5  (I2C0 SCL) - Линия тактирования SCL для OLED-дисплея SSD1306 (128x64)
 * GP6  (GPIO IN)  - [Резерв] Контактный датчик 1 (прерывание)
 * GP7  (GPIO IN)  - [Резерв] Контактный датчик 2 (прерывание)
 * GP8  (GPIO IN)  - [Резерв] Контактный датчик 3 (прерывание)
 * GP9  (GPIO IN)  - [Резерв] Контактный датчик 4 (прерывание)
 * GP25 (GPIO OUT) - Встроенный светодиод на плате (индикатор работы / PWM)
 * =====================================================================
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Настройки OLED-дисплея
#define SCREEN_WIDTH 128    // Ширина дисплея в пикселях
#define SCREEN_HEIGHT 64    // Высота дисплея в пикселях
#define OLED_RESET -1       // Пин сброса (-1, если используется общий сброс с Pico)
#define SCREEN_ADDRESS 0x3C // Стандартный I2C адрес для дисплеев 128x64

// Пины периферии (избегаем системного имени PIN_LED)
const uint8_t I2C_SDA_PIN     = 4;
const uint8_t I2C_SCL_PIN     = 5;
const uint8_t ONBOARD_LED_PIN = 25;

// Создаем объект дисплея на шине Wire (I2C0)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Переменные для управления яркостью светодиода
int brightness = 0;
int fadeAmount = 5;
unsigned long previousMillis = 0;
const unsigned long FADE_INTERVAL = 20;

void setup() {
  // 1. Настройка встроенного светодиода
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  analogWriteResolution(8);

  // 2. Явное назначение пинов I2C0 для Raspberry Pi Pico (ядро Earle Philhower)
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  // 3. Инициализация OLED-дисплея
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // Быстрое мигание светодиодом как сигнал ошибки I2C
    while (true) {
      digitalWrite(ONBOARD_LED_PIN, HIGH);
      delay(100);
      digitalWrite(ONBOARD_LED_PIN, LOW);
      delay(100);
    }
  }

  // 4. Очистка буфера и вывод слова "Test"
  display.clearDisplay();
  display.setTextSize(2);              // Размер шрифта (2 = крупный четкий текст)
  display.setTextColor(SSD1306_WHITE); // Белый цвет текста
  display.setCursor(40, 24);           // Позиция X=40, Y=24 (примерно по центру)
  display.println(F("Test"));
  display.display();                   // Отправка буфера на физический экран
}

void loop() {
  // Продолжаем плавное мигание светодиодом в фоне
  updateLedFade();
}

// Функция плавного изменения яркости светодиода
void updateLedFade() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= FADE_INTERVAL) {
    previousMillis = currentMillis;
    analogWrite(ONBOARD_LED_PIN, brightness);
    brightness += fadeAmount;

    if (brightness <= 0 || brightness >= 255) {
      fadeAmount = -fadeAmount;
    }
  }
}
