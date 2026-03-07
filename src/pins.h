#pragma once

#include <Arduino.h>

// =========================
// Текущие рабочие пины
// =========================

// Основной конвейер
constexpr uint8_t PIN_STEP_PUL = 13;        // PUL драйвера DM542

// Каретка сдвига
constexpr uint8_t PIN_SHIFT_DIR = 14;       // DIR сдвига
constexpr uint8_t PIN_SHIFT_PUL = 23;       // PUL сдвига
constexpr uint8_t PIN_SHIFT_SENSOR_Z = 33;  // Геркон, край Z
constexpr uint8_t PIN_SHIFT_SENSOR_C = 25;  // Геркон, край C

// Пневматика и датчики
constexpr uint8_t PIN_FLAG = 27;            // пневмо цилиндр флага
constexpr uint8_t PIN_SENSOR = 26;          // шелевой датчик

// Позиционный конвейер (двух ручейковый)
constexpr uint8_t PIN_POS_PUL = 32;         // PUL драйвера EVA25

// =========================
// Резерв под расширение
// =========================
// Пока это только "бронь", в коде не используется.

// RS485 (рекомендуемый UART2)
constexpr uint8_t PIN_RS485_RX = 16;        // RO -> RX
constexpr uint8_t PIN_RS485_TX = 17;        // DI <- TX
constexpr uint8_t PIN_RS485_DE_RE = 18;     // DE+RE (управление направлением)

// Дополнительный шаговый драйвер (только PUL)
constexpr uint8_t PIN_STEP2_PUL = 19;

// Реле
constexpr uint8_t PIN_RELAY_1 = 21;
constexpr uint8_t PIN_RELAY_2 = 22;

// Дополнительные датчики (план 3..5 шт.)
// GPIO34/35/36/39 только вход, внутренней подтяжки нет.
constexpr uint8_t PIN_SENSOR_EXT_1 = 34;
constexpr uint8_t PIN_SENSOR_EXT_2 = 35;
constexpr uint8_t PIN_SENSOR_EXT_3 = 36;    // VP на плате
constexpr uint8_t PIN_SENSOR_EXT_4 = 39;    // VN на плате
// Примечание: вместе с PIN_SENSOR (GPIO26) это уже 5 датчиков.

// =========================
// Пины, которые лучше не занимать
// =========================
// GPIO6..11  - подключены к встроенной SPI Flash.
// GPIO1, GPIO3 - USB/UART0 прошивка и лог.
// GPIO0,2,4,5,12,15 - strapping-пины (влияют на загрузку ESP32).
