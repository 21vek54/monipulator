#include "config.h"
#include "system_state.h"
#include "sensors.h"
#include "stepper_motor.h"
#include "pneumatics.h"
#include "motion_profiles.h"
#include "calibration.h"
#include "calibration_with_dialog.h"
#include "endurance_test.h"
#include "work_cycle.h"
#include "recovery.h"
#include "grease_removal.h"

// =============================================
// ГЛАВНЫЙ ФАЙЛ СИСТЕМЫ УПРАВЛЕНИЯ МАНИПУЛЯТОРОМ
// Версия: 7 (все вывходы из аварииного режима сделаны)
// =============================================

// Буфер для ввода команд
String inputBuffer = "";

// Флаги для управления состоянием
bool systemInitialized = false;

// =====================================================
// ДОБАВЛЕНО: логика авто-восстановления (как команда V)
// =====================================================
static uint8_t g_autoRecoverAttempts = 0;
static unsigned long g_lastAutoRecoverAttemptMs = 0;

static bool g_autoRecoverPending = false;
static unsigned long g_autoRecoverRunAtMs = 0;
static const __FlashStringHelper* g_autoRecoverReason = F("");

// Проверка: можно ли сейчас запускать recoverWorkCycle()
static bool canAutoRecoverNow() {
    updateAllSensors();

    if (!hasSavedWorkCycleState()) return false;
    if (!isCalibrated) return false;

    // система не должна быть занята
    if (isMoving) return false;
    if (workCycleInProgress) return false;
    if (testInProgress) return false;
    if (greaseRemovalInProgress) return false;
    if (calibrationInProgress) return false;

    // нельзя при ALM или конфликтах датчиков
    if (stepperIsAlarm()) return false;
    if (isAnyConflict()) return false;

    return true;
}

static void scheduleAutoRecover(const __FlashStringHelper* reason, unsigned long delayMs) {
    g_autoRecoverPending = true;
    g_autoRecoverRunAtMs = millis() + delayMs;
    g_autoRecoverReason = reason;

    Serial.println();
    Serial.println(F("=== АВТОВОССТАНОВЛЕНИЕ: запланировано ==="));
    Serial.print(F("Причина: "));
    Serial.println(reason);
    Serial.print(F("Старт через, мс: "));
    Serial.println(delayMs);
}

static void tryRunAutoRecoverNow() {
    if (!g_autoRecoverPending) return;
    if ((long)(millis() - g_autoRecoverRunAtMs) < 0) return; // ещё рано

    // кулдаун + лимит попыток
    if (g_autoRecoverAttempts >= AUTO_RECOVER_MAX_ATTEMPTS) {
        Serial.println(F("Автовосстановление: лимит попыток исчерпан. Используйте V вручную."));
        g_autoRecoverPending = false;
        return;
    }
    if (millis() - g_lastAutoRecoverAttemptMs < AUTO_RECOVER_COOLDOWN_MS) {
        return; // ждём кулдаун
    }

    if (!canAutoRecoverNow()) {
        // условия пока не выполнены — оставляем pending, попробуем позже
        return;
    }

    g_autoRecoverAttempts++;
    g_lastAutoRecoverAttemptMs = millis();

    Serial.println();
    Serial.println(F("=== АВТОВОССТАНОВЛЕНИЕ (как команда V) ==="));
    Serial.print(F("Причина: "));
    Serial.println(g_autoRecoverReason);
    Serial.println(F("Запуск recoverWorkCycle()..."));

    // важный момент: дадим системе сохранить параметры (как в ручной V)
    saveSystemParameters();
    delay(50);

    // запускаем восстановление
    recoverWorkCycle();

    // после запуска считаем задачу выполненной (если восстановление прервётся — оно само сохранит шаг в EEPROM)
    g_autoRecoverPending = false;
}

// =============================================
// ФУНКЦИИ ОБРАБОТКИ КОМАНД
// =============================================

void handleProfileCommand() {
    Serial.println("Введите профиль скорости в формате:");
    Serial.println("{процент1;задержка1:процент2;задержка2:...}");
    Serial.println("Пример: {33.33;1000:33.33;300:33.34;800}");
    Serial.println("Ожидание ввода...");

    // Ждем ввода профиля
    while (!Serial.available()) {
        delay(10);
    }

    String profile = Serial.readStringUntil('\n');
    profile.trim();

    if (profile.length() > 0) {
        Serial.print("Выполнение профиля: ");
        Serial.println(profile);

        // Заменяем запятые на точки для парсинга
        profile.replace(',', '.');

        // Определяем направление
        bool direction;
        if (isLeftLimitPressed()) {
            direction = LOW;  // Вправо
            Serial.println("Направление: вправо");
        } else if (isRightLimitPressed()) {
            direction = HIGH; // Влево
            Serial.println("Направление: влево");
        } else {
            direction = LOW;  // По умолчанию вправо
            Serial.println("Направление: вправо (по умолчанию)");
        }

        // Выполняем движение по профилю (без требования концевика)
        executeSpeedProfile(profile, travelDistance, direction, false);
    }
}

void handleTestCommand() {
    Serial.println("Введите количество циклов для теста (1-1000):");

    // Ждем ввода количества циклов
    unsigned long timeout = millis();
    while (!Serial.available() && (millis() - timeout) < 10000) {
        delay(10);
    }

    if (!Serial.available()) {
        Serial.println("Таймаут ввода. Используйте: T 10 (например)");
        return;
    }

    String input = Serial.readStringUntil('\n');
    input.trim();
    int cycles = input.toInt();

    if (cycles > 0 && cycles <= MAX_TEST_CYCLES) {
        startEnduranceTest(cycles);
    } else {
        Serial.print("Ошибка: количество циклов должно быть от 1 до ");
        Serial.println(MAX_TEST_CYCLES);
    }
}

void processCommand(String command) {
    command.trim();
    if (command.length() == 0) return;

    char cmd = command.charAt(0);

    // Проверяем, не выполняется ли рабочий цикл или тест
    if ((workCycleInProgress || testInProgress || greaseRemovalInProgress) &&
        cmd != 'Z' && cmd != 'z' && cmd != 'S' && cmd != 's' && cmd != 'X' && cmd != 'x') {
        Serial.println("Система занята: выполняется автоматический процесс");
        return;
    }

    // Обработка команд пневматики (строчные и прописные)
    if (pneumoHandleCommand(cmd)) {
        return;
    }

    // Приводим команду к верхнему регистру для остальных команд
    cmd = toupper(cmd);

    switch (cmd) {
        case 'R': // Рабочий цикл
            if (!isMoving && isCalibrated && !testInProgress && !greaseRemovalInProgress) {
                Serial.println("Запуск рабочего цикла...");
                // Перед началом убеждаемся, что предыдущие данные сохранены
                saveSystemParameters();
                delay(50); // Даем время для сохранения
                startWorkCycle();
            } else if (!isCalibrated) {
                Serial.println("Ошибка: сначала выполните калибровку (K)");
            } else if (isMoving) {
                Serial.println("Ошибка: система уже движется");
            } else if (testInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: выполняется тест отказоустойчивости или процедура удаления смазки");
            }
            break;

        case 'V': // Восстановление цикла
            if (!isMoving && isCalibrated && !testInProgress && !workCycleInProgress && !greaseRemovalInProgress) {
                Serial.println("Проверка возможности восстановления...");
                // Сохраняем текущие параметры перед восстановлением
                saveSystemParameters();
                delay(50); // Даем время для сохранения
                recoverWorkCycle();
            } else if (!isCalibrated) {
                Serial.println("Ошибка: сначала выполните калибровку (K)");
            } else if (isMoving || testInProgress || workCycleInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'D': // Движение вправо
            if (!isMoving && isCalibrated && !testInProgress && !workCycleInProgress && !greaseRemovalInProgress) {
                Serial.println("Движение вправо");
                // Сохраняем состояние перед движением
                saveSystemParameters();
                delay(50);
                if (moveRightSafe()) {
                    Serial.println("приехал вправо");
                } else {
                    Serial.println("Ошибка движения вправо");
                }
            } else if (!isCalibrated) {
                Serial.println("Ошибка: сначала выполните калибровку (K)");
            } else if (isMoving || testInProgress || workCycleInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'A': // Движение влево
            if (!isMoving && isCalibrated && !testInProgress && !workCycleInProgress && !greaseRemovalInProgress) {
                Serial.println("Движение влево");
                // Сохраняем состояние перед движением
                saveSystemParameters();
                delay(50);
                if (moveLeftSafe()) {
                    Serial.println("приехал влево");
                } else {
                    Serial.println("Ошибка движения влево");
                }
            } else if (!isCalibrated) {
                Serial.println("Ошибка: сначала выполните калибровку (K)");
            } else if (isMoving || testInProgress || workCycleInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'P': // Движение по произвольному профилю
            if (!isMoving && isCalibrated && !testInProgress && !workCycleInProgress && !greaseRemovalInProgress) {
                Serial.println("Команда: движение по произвольному профилю");
                // Сохраняем состояние перед движением
                saveSystemParameters();
                delay(50);
                handleProfileCommand();
            } else if (!isCalibrated) {
                Serial.println("Ошибка: сначала выполните калибровку (K)");
            } else if (isMoving || testInProgress || workCycleInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'K': // Калибровка с диалогом
            if (!isMoving && !calibrationInProgress && !testInProgress && !workCycleInProgress && !greaseRemovalInProgress) {
                // Сохраняем текущее состояние перед калибровкой
                saveSystemParameters();
                delay(50);
                calibrateDistanceWithDialog();
            } else if (calibrationInProgress) {
                Serial.println("Ошибка: калибровка уже выполняется");
            } else if (isMoving || testInProgress || workCycleInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'T': // Тест отказоустойчивости
            if (!isMoving && !testInProgress && !workCycleInProgress && !greaseRemovalInProgress) {
                // Сохраняем состояние перед тестом
                saveSystemParameters();
                delay(50);
                handleTestCommand();
            } else if (testInProgress) {
                Serial.println("Ошибка: тест уже выполняется");
            } else if (isMoving || workCycleInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'U': // Процедура удаления смазки - ДОБАВЛЕНА ЭТА КОМАНДА
            if (!isMoving && !greaseRemovalInProgress && !testInProgress && !workCycleInProgress) {
                Serial.println("Процедура удаления смазки");
                // Сохраняем состояние перед процедурой
                saveSystemParameters();
                delay(50);
                startGreaseRemoval();
            } else if (greaseRemovalInProgress) {
                Serial.println("Ошибка: процедура удаления смазки уже выполняется");
            } else if (isMoving || testInProgress || workCycleInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'Z': // Экстренная остановка
            Serial.println("Экстренная остановка");
            // Сохраняем состояние перед остановкой
            saveSystemParameters();
            delay(50);
            emergencyStop();
            stopEnduranceTest();
            stopGreaseRemoval(); // ДОБАВЛЕНА ЭТА СТРОКА
            break;

        case 'X': // Аварийный сброс драйвера
            if (!isMoving && !workCycleInProgress && !testInProgress && !greaseRemovalInProgress) {
                Serial.println("Аварийный сброс драйвера через реле...");
                stepperHardReset();
            } else if (isMoving || workCycleInProgress || testInProgress || greaseRemovalInProgress) {
                Serial.println("Ошибка: система занята");
            }
            break;

        case 'G': // Статус датчиков
            printAllSensorsStatus();
            break;

        case 'I': // Информация о системе (статус + EEPROM)
            printSystemInfo();
            break;

        case '0': // Сброс системы
            Serial.println("Сброс состояния системы...");
            resetSystemState();
            break;

        case 'C': // Очистка EEPROM
            Serial.println("Очистка EEPROM...");
            clearSystemState();
            break;

        case '?': // Помощь
            printHelp();
            break;

        default:
            Serial.print("Неизвестная команда: ");
            Serial.println(cmd);
            printHelp();
            break;
    }
}

// =============================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// =============================================

void printWelcomeMessage() {
    Serial.println("\n" + String(50, '='));
    Serial.println("СИСТЕМА УПРАВЛЕНИЯ МАНИПУЛЯТОРОМ v6.1");
    Serial.println("Добавлено: процедура удаления смазки (U)");
    Serial.println("Добавлено: аварийный сброс драйвера (X)");
    Serial.println("ESP32 + DM556 + Пневматика + Реле сброса");
    Serial.println(String(50, '='));
}

void printHelp() {
    Serial.println("\n" + String(40, '='));
    Serial.println("СПРАВКА ПО КОМАНДАМ (регистр не важен)");
    Serial.println(String(40, '='));

    Serial.println("\nИНФОРМАЦИЯ И ДИАГНОСТИКА:");
    Serial.println("  ?        - эта справка");
    Serial.println("  G/g      - статус всех датчиков (герконов)");
    Serial.println("  I/i      - информация о системе + EEPROM");

    Serial.println("\nАВТОМАТИЧЕСКИЕ ПРОЦЕССЫ:");
    Serial.println("  R/r      - рабочий цикл манипулятор");
    Serial.println("  V/v      - восстановление после сбоя (автоматическое)");
    Serial.println("  K/k      - калибровка расстояния (с диалогом)");
    Serial.println("  T/t      - тест отказоустойчивости");
    Serial.println("  U/u      - процедура удаления смазки (после обслуживания)");
    Serial.println("  P/p      - движение по произвольному профилю");

    Serial.println("\nРУЧНОЕ УПРАВЛЕНИЕ:");
    Serial.println("  W/w      - вертикаль ВВЕРХ");
    Serial.println("  S/s      - вертикаль ВНИЗ");
    Serial.println("  A/a      - движение ВЛЕВО");
    Serial.println("  D/d      - движение ВПРАВО");
    Serial.println("  Q/q      - захват ОТКРЫТЬ");
    Serial.println("  E/e      - захват ЗАКРЫТЬ");

    Serial.println("\nУПРАВЛЕНИЕ СИСТЕМОЙ:");
    Serial.println("  0        - сброс состояния системы");
    Serial.println("  C/c      - очистка EEPROM (полный сброс)");
    Serial.println("  Z/z      - экстренная остановка");
    Serial.println("  X/x      - аварийный сброс драйвера (через реле)");

    Serial.println(String(40, '='));
    Serial.println("Готов к работе. Введите команду...");
}

void handleSerialCommands() {
    while (Serial.available() > 0) {
        char incomingChar = Serial.read();

        // Если это символ новой строки - обрабатываем команду
        if (incomingChar == '\n' || incomingChar == '\r') {
            processCommand(inputBuffer);
            inputBuffer = "";
        } else {
            // Добавляем символ в буфер
            inputBuffer += incomingChar;
        }
    }
}

// =============================================
// ОСНОВНЫЕ ФУНКЦИИ ARDUINO
// =============================================

void setup() {
    // Инициализация Serial порта
    Serial.begin(115200);
    delay(1000);

    printWelcomeMessage();

    // Инициализация всех модулей
    Serial.println("\nИнициализация системы...");

    // Сначала инициализируем восстановление
    recoveryInit();
    Serial.println("✓ Система восстановления инициализирована");

    initAllSensors();
    Serial.println("✓ Датчики инициализированы");

    stepperInit();
    Serial.println("✓ Шаговый двигатель инициализирован с аппаратным сбросом");

    pneumoInit();
    Serial.println("✓ Пневматика инициализирована");

    initSystemState();
    Serial.println("✓ Состояние системы инициализировано");

    // Первоначальное обновление датчиков
    updateAllSensors();
    delay(100);
    updateAllSensors();

    // Проверка аварийных состояний
    Serial.println("\nПроверка системы...");

    bool errors = false;

    if (stepperIsAlarm()) {
        Serial.println("⚠️  ВНИМАНИЕ: Драйвер в аварийном состоянии (ALM)");
        Serial.println("  Выполняем автоматический сброс...");
        if (stepperRecoverFromAlarm()) {
            Serial.println("✓ Драйвер успешно восстановлен");
        } else {
            Serial.println("✗ Не удалось восстановить драйвер автоматически");
            Serial.println("  Используйте команду X для аппаратного сброса");
            errors = true;
        }
    } else {
        Serial.println("✓ Драйвер: норма");
    }

    if (isBothLimitsPressed()) {
        Serial.println("⚠️  ВНИМАНИЕ: Оба концевика нажаты одновременно!");
        errors = true;
    } else {
        Serial.println("✓ Концевики: норма");
    }

    if (isGripConflict()) {
        Serial.println("⚠️  ВНИМАНИЕ: Конфликт датчиков захвата!");
        errors = true;
    }

    if (isZConflict()) {
        Serial.println("⚠️  ВНИМАНИЕ: Конфликт датчиков вертикали!");
        errors = true;
    }

    // Определение начального положения
    Serial.print("\nНачальное положение: ");
    if (isLeftLimitPressed()) {
        Serial.println("левый концевик");
    } else if (isRightLimitPressed()) {
        Serial.println("правый концевик");
    } else {
        Serial.println("в середине пути");
    }

    // Проверка восстановления
    Serial.println("\n=== ПРОВЕРКА ВОССТАНОВЛЕНИЯ ===");
    if (hasSavedWorkCycleState()) {
        Serial.println("⚠️  Обнаружено сохраненное состояние рабочего цикла!");

#if AUTO_RECOVER_ON_BOOT
        // На старте — подождём, чтобы всё стабилизировалось
        scheduleAutoRecover(F("старт контроллера"), AUTO_RECOVER_ON_BOOT_DELAY_MS);
#else
        Serial.println("Для восстановления введите команду V");
#endif

    } else {
        Serial.println("✓ Сохраненного состояния нет, готов к новому циклу");
    }

    // Проверка готовности к рабочему циклу
    Serial.print("\nГотовность к рабочему циклу: ");
    if (checkWorkCycleInitialState()) {
        Serial.println("✓ ГОТОВ");
    } else {
        Serial.println("✗ ТРЕБУЕТСЯ НАСТРОЙКА");
    }

    // Проверка калибровки
    if (isCalibrated) {
        Serial.print("✓ Калибровка выполнена: ");
        Serial.print(travelDistance);
        Serial.print(" шагов (");
        Serial.print(travelDistanceMM, 1);
        Serial.println(" мм)");
    } else {
        Serial.println("⚠️  Калибровка не выполнена. Используйте команду K");
    }

    // Рекомендации
    if (errors) {
        Serial.println("\n⚠️  Обнаружены ошибки! Исправьте их перед использованием.");
    } else {
        Serial.println("\n✓ Система готова к работе");
    }

    Serial.println("\n" + String(50, '='));
    Serial.println("Основные команды:");
    Serial.println("R - рабочий цикл, V - восстановление, I - информация");
    Serial.println("K - калибровка, U - удаление смазки, X - сброс драйвера, ? - справка");
    Serial.println(String(50, '='));

    systemInitialized = true;
}

void loop() {
    // Проверка инициализации
    if (!systemInitialized) {
        return;
    }

    // Обновление состояния всех датчиков (каждые 10мс)
    static unsigned long lastSensorUpdate = 0;
    if (millis() - lastSensorUpdate >= 10) {
        updateAllSensors();
        lastSensorUpdate = millis();
    }

    // Автоматическая проверка аварии драйвера (каждые 5 секунд)
    static unsigned long lastAlarmCheck = 0;
    static bool wasAlarm = false;

    if (millis() - lastAlarmCheck >= 5000) {
        bool alarmNow = stepperIsAlarm();

        if (alarmNow) {
            wasAlarm = true;

            Serial.println("\n⚠️  ВНИМАНИЕ: Драйвер в аварийном состоянии!");
            Serial.println("Попытка автоматического восстановления...");

            bool recovered = stepperRecoverFromAlarm();
            if (recovered) {
                Serial.println("✓ Драйвер успешно восстановлен");

#if AUTO_RECOVER_AFTER_ALM
                // Сбрасываем счётчик попыток, чтобы после ALM разрешить авто-V
                g_autoRecoverAttempts = 0;
                g_lastAutoRecoverAttemptMs = 0;

                // Планируем авто-V через 1 секунду (как ты и хотел)
                scheduleAutoRecover(F("выход из ALM (успешное восстановление)"), AUTO_RECOVER_AFTER_ALM_DELAY_MS);
#endif

            } else {
                Serial.println("✗ Не удалось восстановить драйвер автоматически");
                Serial.println("Используйте команду X для аппаратного сброса");
            }
        } else {
            if (wasAlarm) {
                // был ALM, а теперь нет (вариант, если ALM ушёл сам)
                wasAlarm = false;

#if AUTO_RECOVER_AFTER_ALM
                g_autoRecoverAttempts = 0;
                g_lastAutoRecoverAttemptMs = 0;
                scheduleAutoRecover(F("выход из ALM (снялось без recover)"), AUTO_RECOVER_AFTER_ALM_DELAY_MS);
#endif
            }
        }

        lastAlarmCheck = millis();
    }

#if AUTO_RECOVER_IN_LOOP
    // если есть запланированное восстановление — попробуем выполнить
    tryRunAutoRecoverNow();

    // если не запланировано, но EEPROM содержит шаг — можно планировать “мягко”
    // (например, если питание моргнуло и появился сохранённый шаг)
    if (!g_autoRecoverPending && hasSavedWorkCycleState()) {
        // Не спамим — планируем без лишнего вывода/часто не планируем
        scheduleAutoRecover(F("loop: найден сохранённый шаг"), 200); // небольшой стартовый лаг
    }
#endif

    // Обновление теста отказоустойчивости (если выполняется)
    updateEnduranceTest();

    // Обновление процедуры удаления смазки (если выполняется)
    updateGreaseRemoval();

    // Обработка команд с Serial порта
    handleSerialCommands();

    // Небольшая задержка для стабильности
    delay(1);
}
