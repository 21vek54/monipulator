#include <Arduino.h>

constexpr char FW_VERSION[] = "v1.5.2";

constexpr uint8_t PIN_STEP_PUL = 13; // DM542 PUL
constexpr uint8_t PIN_STEP_DIR = 12; // DM542 DIR
constexpr uint8_t PIN_STEP_ENA = 14; // DM542 ENA
constexpr uint8_t PIN_FLAG = 27;     // Пневмоцилиндр флага
constexpr uint8_t PIN_SENSOR = 26;   // E18-D50NK (активный HIGH)

constexpr bool DIR_LEFT_LEVEL = LOW;
constexpr bool DIR_RIGHT_LEVEL = HIGH;
constexpr bool FLAG_UP_LEVEL = HIGH;
constexpr bool FLAG_DOWN_LEVEL = LOW;
constexpr bool ENA_ACTIVE_LEVEL = LOW; // Подтверждено тестом: для вашего DM542 активный LOW
constexpr bool PULSE_ACTIVE_LEVEL = HIGH;

constexpr uint32_t STEP_PULSE_WIDTH_US = 10;
constexpr uint32_t SENSOR_PRINT_INTERVAL_MS = 500;
constexpr uint32_t SENSOR_DEBOUNCE_MS = 80;
constexpr uint32_t MANUAL_MOVE_DELAY_US = 2000; // Фиксированная задержка для команд A/D
constexpr uint32_t PULSES_PER_MM = 5;           // 2000 импульсов = 400 мм => 5 имп/мм
constexpr uint32_t BC_MOVE_DELAY_US = 2000;
constexpr uint32_t BC_CENTER_DISTANCE_MM = 20;
constexpr uint32_t BC_CENTER_STEPS = BC_CENTER_DISTANCE_MM * PULSES_PER_MM;
constexpr uint8_t BC_BATCH_PLATES_DEFAULT = 4;
constexpr uint8_t BC_BATCH_PLATES_MAX = 20;

constexpr uint32_t C3_MOVE_DELAY_US = 2000;
constexpr uint32_t C3_CENTER_STEPS = 20U * PULSES_PER_MM; // 20 мм
constexpr uint32_t C3_FLAG_REOPEN_STEPS = 80U * PULSES_PER_MM; // 80 мм
constexpr uint32_t C3_WAIT_AFTER_FIRST_LEAVE_STEPS = (150U + 35U) * PULSES_PER_MM; // 185 мм
constexpr uint32_t C3_FINAL_AFTER_SECOND_LEAVE_STEPS = (150U - 35U) * PULSES_PER_MM; // 115 мм

struct MotionState {
    bool active = false;
    bool pulseHigh = false;
    uint32_t stepsTotal = 0;
    uint32_t stepsDone = 0;
    uint32_t pulseDelayUs = 1000;
    uint32_t lastPulseStartUs = 0;
};

struct SensorFilterState {
    bool initialized = false;
    bool lastRawPlateDetected = false;
    bool stablePlateDetected = false;
    uint32_t lastRawChangeMs = 0;
};

enum class BCState : uint8_t {
    Idle,
    MovingToSensor,
    CenteringAfterDetect,
    WaitPlateLeave
};

enum class C3State : uint8_t {
    Idle,
    SeekFirstPlate,
    CenterFirstPlate,
    TrackFirstLeaveAndOpen,
    SeekSecondPlate,
    CenterSecondPlate,
    WaitSecondFlagDownTiming,
    MoveAfterSecondFlagDown,
    WaitSecondLeave,
    FinalMoveAfterSecondLeave
};

String g_cmdBuffer;
MotionState g_motion;
SensorFilterState g_sensorFilter;
bool g_sensorStreamEnabled = false;
uint32_t g_lastSensorPrintMs = 0;
uint32_t g_totalStepsCounter = 0;
BCState g_bcState = BCState::Idle;
uint32_t g_bcFlagDownStepCounter = 0;
uint32_t g_bcDetectStepCounter = 0;
bool g_bcBatchActive = false;
uint8_t g_bcBatchIndex = 0;
uint8_t g_bcBatchPlatesTarget = BC_BATCH_PLATES_DEFAULT;
uint32_t g_bcBatchResultsSteps[BC_BATCH_PLATES_MAX] = {};

bool g_c3Active = false;
C3State g_c3State = C3State::Idle;
bool g_c3FirstLeaveCaptured = false;
bool g_c3SilentMode = false;
uint32_t g_c3CenterStartStep = 0;
uint32_t g_c3FirstFlagDownStep = 0;
uint32_t g_c3FirstLeaveStep = 0;
uint32_t g_c3SecondFlagDownStep = 0;
uint32_t g_c3FinalStartStep = 0;
uint32_t g_c3UTSteps = 0;

bool isPlateAtSensorFiltered();

void printHelp()
{
    Serial.println("Команды:");
    Serial.println("  D 200        - вправо, 200 мм (задержка 2000 мкс)");
    Serial.println("  A 200        - влево, 200 мм (задержка 2000 мкс)");
    Serial.println("  BC 2         - тест N тарелок: для каждой +20 мм, замер после опускания флага, в конце таблица и среднее");
    Serial.println("  3            - рабочий цикл: 2 тарелки с UT и таймингами 80/185/115 мм");
    Serial.println("  4            - рабочий цикл как 3, но без вывода сообщений в монитор");
    Serial.println("  W            - флаг вверх");
    Serial.println("  S            - флаг вниз");
    Serial.println("  E            - вкл/выкл поток датчика (каждые 0.5 сек)");
    Serial.println("  H            - помощь");
}

bool parseUnsigned(const String &s, uint32_t &value)
{
    if (s.isEmpty()) {
        return false;
    }

    uint32_t acc = 0;
    for (size_t i = 0; i < s.length(); i++) {
        const char c = s[i];
        if (c < '0' || c > '9') {
            return false;
        }
        const uint32_t digit = static_cast<uint32_t>(c - '0');
        if (acc > (UINT32_MAX - digit) / 10U) {
            return false;
        }
        acc = (acc * 10U) + digit;
    }
    value = acc;
    return true;
}

bool parseDistanceMmArgs(String args, const String &cmd, uint32_t &distanceMm)
{
    args.trim();
    if (args.isEmpty()) {
        return false;
    }

    // Поддержка вариантов вида "A 200" и "A A 200" / "D D 200"
    const int splitPos = args.indexOf(' ');
    if (splitPos > 0) {
        String first = args.substring(0, splitPos);
        first.trim();
        first.toUpperCase();
        if (first == cmd) {
            args = args.substring(splitPos + 1);
            args.trim();
        }
    }

    return parseUnsigned(args, distanceMm);
}

void writePulseInactive()
{
    digitalWrite(PIN_STEP_PUL, PULSE_ACTIVE_LEVEL ? LOW : HIGH);
}

void stopMotion()
{
    writePulseInactive();
    g_motion.active = false;
    g_motion.pulseHigh = false;
}

void startConstantMotion(bool dirLevel, uint32_t steps, uint32_t delayUs)
{
    if (g_motion.active) {
        Serial.println("Ошибка: двигатель уже в движении.");
        return;
    }

    if (steps == 0) {
        Serial.println("Шагов 0: движение не требуется.");
        return;
    }

    digitalWrite(PIN_STEP_ENA, ENA_ACTIVE_LEVEL);
    digitalWrite(PIN_STEP_DIR, dirLevel);

    g_motion.active = true;
    g_motion.pulseHigh = false;
    g_motion.stepsTotal = steps;
    g_motion.stepsDone = 0;
    g_motion.pulseDelayUs = delayUs;
    g_motion.lastPulseStartUs = micros();

    Serial.print("Запуск: шагов=");
    Serial.print(steps);
    Serial.print(", задержка=");
    Serial.print(delayUs);
    Serial.print(" мкс, направление=");
    Serial.println(dirLevel == DIR_LEFT_LEVEL ? "влево" : "вправо");
}

void startBCPlateCycle()
{
    digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);
    digitalWrite(PIN_STEP_ENA, ENA_ACTIVE_LEVEL);
    digitalWrite(PIN_STEP_DIR, DIR_RIGHT_LEVEL);

    g_motion.active = true;
    g_motion.pulseHigh = false;
    g_motion.stepsTotal = UINT32_MAX;
    g_motion.stepsDone = 0;
    g_motion.pulseDelayUs = BC_MOVE_DELAY_US;
    g_motion.lastPulseStartUs = micros();

    g_bcState = BCState::MovingToSensor;
    g_bcFlagDownStepCounter = g_totalStepsCounter;
    g_bcDetectStepCounter = g_totalStepsCounter;

    Serial.print("BC [");
    Serial.print(g_bcBatchIndex + 1U);
    Serial.print("/");
    Serial.print(g_bcBatchPlatesTarget);
    Serial.println("]: флаг поднят, конвейер вправо. Ожидание тарелки на датчике.");
}

void printBCBatchSummary()
{
    float sumMm = 0.0F;
    for (uint8_t i = 0; i < g_bcBatchPlatesTarget; i++) {
        const float mm = static_cast<float>(g_bcBatchResultsSteps[i]) / static_cast<float>(PULSES_PER_MM);
        sumMm += mm;
        Serial.print(i + 1U);
        Serial.print(" | ");
        Serial.print(mm, 1);
        Serial.println(" мм");
    }

    const float avgMm = sumMm / static_cast<float>(g_bcBatchPlatesTarget);
    Serial.print("AVG | ");
    Serial.print(avgMm, 1);
    Serial.println(" мм");
}

void startBCTest(uint8_t platesCount)
{
    if (g_motion.active || g_bcState != BCState::Idle || g_bcBatchActive) {
        Serial.println("BC: ошибка, двигатель уже в движении.");
        return;
    }

    if (g_sensorFilter.stablePlateDetected) {
        Serial.println("BC: тарелка уже на датчике, отведите ее и повторите.");
        return;
    }

    g_bcBatchPlatesTarget = platesCount;
    for (uint8_t i = 0; i < g_bcBatchPlatesTarget; i++) {
        g_bcBatchResultsSteps[i] = 0;
    }
    g_bcBatchActive = true;
    g_bcBatchIndex = 0;

    Serial.print("BC: старт серии на ");
    Serial.print(g_bcBatchPlatesTarget);
    Serial.println(" тарелки.");
    startBCPlateCycle();
}

void startCycle3(bool silentMode)
{
    if (g_motion.active || g_bcBatchActive || g_c3Active) {
        Serial.println("C3: ошибка, двигатель уже в движении.");
        return;
    }

    if (g_sensorFilter.stablePlateDetected) {
        Serial.println("C3: тарелка уже на датчике, отведите ее и повторите.");
        return;
    }

    digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);
    digitalWrite(PIN_STEP_ENA, ENA_ACTIVE_LEVEL);
    digitalWrite(PIN_STEP_DIR, DIR_RIGHT_LEVEL);

    g_motion.active = true;
    g_motion.pulseHigh = false;
    g_motion.stepsTotal = UINT32_MAX;
    g_motion.stepsDone = 0;
    g_motion.pulseDelayUs = C3_MOVE_DELAY_US;
    g_motion.lastPulseStartUs = micros();

    g_c3Active = true;
    g_c3SilentMode = silentMode;
    g_c3State = C3State::SeekFirstPlate;
    g_c3FirstLeaveCaptured = false;
    g_c3CenterStartStep = g_totalStepsCounter;
    g_c3FirstFlagDownStep = g_totalStepsCounter;
    g_c3FirstLeaveStep = g_totalStepsCounter;
    g_c3SecondFlagDownStep = g_totalStepsCounter;
    g_c3FinalStartStep = g_totalStepsCounter;
    g_c3UTSteps = 0;

    if (!g_c3SilentMode) {
        Serial.println("C3: старт рабочего цикла.");
        Serial.println("C3: шаги 1-3: флаги вверх, конвейер вправо, поиск первой тарелки.");
    }
}

void captureC3FirstLeaveIfNeeded()
{
    if (g_c3FirstLeaveCaptured || isPlateAtSensorFiltered()) {
        return;
    }

    g_c3FirstLeaveCaptured = true;
    g_c3FirstLeaveStep = g_totalStepsCounter;
    g_c3UTSteps = g_totalStepsCounter - g_c3FirstFlagDownStep;

    if (!g_c3SilentMode) {
        Serial.print("C3: первая тарелка ушла с датчика, UT=");
        Serial.print(g_c3UTSteps);
        Serial.print(" шагов (");
        Serial.print(static_cast<float>(g_c3UTSteps) / static_cast<float>(PULSES_PER_MM), 1);
        Serial.println(" мм).");
    }
}

void processCycle3()
{
    if (!g_c3Active || g_c3State == C3State::Idle) {
        return;
    }

    if (!g_motion.active) {
        g_c3Active = false;
        g_c3State = C3State::Idle;
        if (!g_c3SilentMode) {
            Serial.println("C3: сценарий прерван (двигатель остановлен вне C3).");
        }
        g_c3SilentMode = false;
        return;
    }

    switch (g_c3State) {
        case C3State::SeekFirstPlate:
            if (isPlateAtSensorFiltered()) {
                g_c3CenterStartStep = g_totalStepsCounter;
                g_c3State = C3State::CenterFirstPlate;
                if (!g_c3SilentMode) {
                    Serial.println("C3: первая тарелка найдена, центрирование +20 мм.");
                }
            }
            return;

        case C3State::CenterFirstPlate:
            if ((uint32_t)(g_totalStepsCounter - g_c3CenterStartStep) >= C3_CENTER_STEPS) {
                digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
                g_c3FirstFlagDownStep = g_totalStepsCounter;
                g_c3FirstLeaveCaptured = false;
                g_c3UTSteps = 0;
                g_c3State = C3State::TrackFirstLeaveAndOpen;
                if (!g_c3SilentMode) {
                    Serial.println("C3: флаги опущены, измеряем UT и ждем +80 мм до подъема флагов.");
                }
            }
            return;

        case C3State::TrackFirstLeaveAndOpen:
            captureC3FirstLeaveIfNeeded();

            if ((uint32_t)(g_totalStepsCounter - g_c3FirstFlagDownStep) >= C3_FLAG_REOPEN_STEPS) {
                digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);
                g_c3State = C3State::SeekSecondPlate;
                if (!g_c3SilentMode) {
                    Serial.println("C3: +80 мм после первого опускания, флаги подняты. Поиск второй тарелки.");
                }
            }
            return;

        case C3State::SeekSecondPlate:
            captureC3FirstLeaveIfNeeded();

            if (isPlateAtSensorFiltered()) {
                g_c3CenterStartStep = g_totalStepsCounter;
                g_c3State = C3State::CenterSecondPlate;
                if (!g_c3SilentMode) {
                    Serial.println("C3: вторая тарелка найдена, центрирование +20 мм.");
                }
            }
            return;

        case C3State::CenterSecondPlate:
            captureC3FirstLeaveIfNeeded();

            if ((uint32_t)(g_totalStepsCounter - g_c3CenterStartStep) >= C3_CENTER_STEPS) {
                g_c3State = C3State::WaitSecondFlagDownTiming;
                if (!g_c3SilentMode) {
                    Serial.println("C3: вторая тарелка центрирована, ожидаем тайминг 185 мм от ухода первой.");
                }
            }
            return;

        case C3State::WaitSecondFlagDownTiming:
            captureC3FirstLeaveIfNeeded();

            if (!g_c3FirstLeaveCaptured) {
                return;
            }

            if ((uint32_t)(g_totalStepsCounter - g_c3FirstLeaveStep) >= C3_WAIT_AFTER_FIRST_LEAVE_STEPS) {
                digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
                g_c3SecondFlagDownStep = g_totalStepsCounter;
                g_c3State = C3State::MoveAfterSecondFlagDown;
                if (!g_c3SilentMode) {
                    Serial.println("C3: выдержан тайминг 185 мм, флаги опущены.");
                }
            }
            return;

        case C3State::MoveAfterSecondFlagDown:
            if ((uint32_t)(g_totalStepsCounter - g_c3SecondFlagDownStep) >= C3_FLAG_REOPEN_STEPS) {
                digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);
                g_c3State = C3State::WaitSecondLeave;
                if (!g_c3SilentMode) {
                    Serial.println("C3: после второго опускания пройдено 80 мм, флаги подняты. Ждем уход второй тарелки.");
                }
            }
            return;

        case C3State::WaitSecondLeave:
            if (!isPlateAtSensorFiltered()) {
                g_c3FinalStartStep = g_totalStepsCounter;
                g_c3State = C3State::FinalMoveAfterSecondLeave;
                if (!g_c3SilentMode) {
                    Serial.println("C3: вторая тарелка ушла с датчика, финальный добег 115 мм.");
                }
            }
            return;

        case C3State::FinalMoveAfterSecondLeave:
            if ((uint32_t)(g_totalStepsCounter - g_c3FinalStartStep) >= C3_FINAL_AFTER_SECOND_LEAVE_STEPS) {
                stopMotion();
                g_c3Active = false;
                g_c3State = C3State::Idle;
                if (!g_c3SilentMode) {
                    Serial.print("C3: цикл завершен. UT=");
                    Serial.print(g_c3UTSteps);
                    Serial.print(" шагов (");
                    Serial.print(static_cast<float>(g_c3UTSteps) / static_cast<float>(PULSES_PER_MM), 1);
                    Serial.println(" мм).");
                }
                g_c3SilentMode = false;
            }
            return;

        case C3State::Idle:
        default:
            return;
    }
}

void processMotion()
{
    if (!g_motion.active) {
        return;
    }

    const uint32_t nowUs = micros();

    if (!g_motion.pulseHigh) {
        if ((uint32_t)(nowUs - g_motion.lastPulseStartUs) >= g_motion.pulseDelayUs) {
            digitalWrite(PIN_STEP_PUL, PULSE_ACTIVE_LEVEL ? HIGH : LOW);
            g_motion.pulseHigh = true;
            g_motion.lastPulseStartUs = nowUs;
        }
        return;
    }

    if ((uint32_t)(nowUs - g_motion.lastPulseStartUs) >= STEP_PULSE_WIDTH_US) {
        writePulseInactive();
        g_motion.pulseHigh = false;
        g_motion.stepsDone++;
        g_totalStepsCounter++;

        if (g_motion.stepsDone >= g_motion.stepsTotal) {
            stopMotion();
            Serial.println("Движение завершено.");
        }
    }
}

void processBCTest()
{
    if (!g_bcBatchActive || g_bcState == BCState::Idle) {
        return;
    }

    if (!g_motion.active) {
        g_bcState = BCState::Idle;
        g_bcBatchActive = false;
        Serial.println("BC: сценарий прерван (двигатель остановлен вне BC).");
        return;
    }

    switch (g_bcState) {
        case BCState::MovingToSensor:
            if (isPlateAtSensorFiltered()) {
                g_bcState = BCState::CenteringAfterDetect;
                g_bcDetectStepCounter = g_totalStepsCounter;
                Serial.println("BC: тарелка найдена, центрирование +20 мм.");
            }
            return;

        case BCState::CenteringAfterDetect:
            if ((uint32_t)(g_totalStepsCounter - g_bcDetectStepCounter) >= BC_CENTER_STEPS) {
                digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
                g_bcFlagDownStepCounter = g_totalStepsCounter;
                g_bcState = BCState::WaitPlateLeave;
                Serial.println("BC: флаг опущен, ждем уход тарелки с датчика.");
            }
            return;

        case BCState::WaitPlateLeave:
            if (!isPlateAtSensorFiltered()) {
                const uint32_t traveledSteps = g_totalStepsCounter - g_bcFlagDownStepCounter;
                const float traveledMm = static_cast<float>(traveledSteps) / static_cast<float>(PULSES_PER_MM);

                stopMotion();
                g_bcBatchResultsSteps[g_bcBatchIndex] = traveledSteps;
                Serial.print("BC: тарелка ");
                Serial.print(g_bcBatchIndex + 1U);
                Serial.print(" завершена. После опускания флага пройдено шагов=");
                Serial.print(traveledSteps);
                Serial.print(", расстояние=");
                Serial.print(traveledMm, 1);
                Serial.println(" мм.");

                g_bcBatchIndex++;
                if (g_bcBatchIndex >= g_bcBatchPlatesTarget) {
                    g_bcState = BCState::Idle;
                    g_bcBatchActive = false;
                    printBCBatchSummary();
                } else {
                    startBCPlateCycle();
                }
            }
            return;

        case BCState::Idle:
        default:
            return;
    }
}

bool readSensorPlateRaw()
{
    return digitalRead(PIN_SENSOR) == HIGH;
}

void updateSensorFilter()
{
    const uint32_t nowMs = millis();
    const bool rawPlateDetected = readSensorPlateRaw();

    if (!g_sensorFilter.initialized) {
        g_sensorFilter.initialized = true;
        g_sensorFilter.lastRawPlateDetected = rawPlateDetected;
        g_sensorFilter.stablePlateDetected = rawPlateDetected;
        g_sensorFilter.lastRawChangeMs = nowMs;
        return;
    }

    if (rawPlateDetected != g_sensorFilter.lastRawPlateDetected) {
        g_sensorFilter.lastRawPlateDetected = rawPlateDetected;
        g_sensorFilter.lastRawChangeMs = nowMs;
    }

    if (rawPlateDetected != g_sensorFilter.stablePlateDetected &&
        (uint32_t)(nowMs - g_sensorFilter.lastRawChangeMs) >= SENSOR_DEBOUNCE_MS) {
        g_sensorFilter.stablePlateDetected = rawPlateDetected;
    }
}

bool isPlateAtSensorFiltered()
{
    return g_sensorFilter.stablePlateDetected;
}

void printSensorState()
{
    const bool isPlateAtSensor = isPlateAtSensorFiltered();
    Serial.print("Датчик E18-D50NK: ");
    Serial.println(isPlateAtSensor ? "тарелка на флаге (HIGH)" : "тарелки нет (LOW)");
}

void processSensorStream()
{
    if (!g_sensorStreamEnabled) {
        return;
    }

    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - g_lastSensorPrintMs) >= SENSOR_PRINT_INTERVAL_MS) {
        g_lastSensorPrintMs = nowMs;
        printSensorState();
    }
}

void handleCommand(String line)
{
    line.trim();
    if (line.isEmpty()) {
        return;
    }

    int splitPos = line.indexOf(' ');
    if (splitPos < 0) {
        splitPos = line.indexOf('\t');
    }

    String cmd = line;
    String args;
    if (splitPos > 0) {
        cmd = line.substring(0, splitPos);
        args = line.substring(splitPos + 1);
        args.trim();
    }
    cmd.toUpperCase();

    if (cmd == "D" || cmd == "A") {
        uint32_t distanceMm = 0;
        if (!parseDistanceMmArgs(args, cmd, distanceMm)) {
            Serial.println("Format error. Example: A 200");
            return;
        }
        if (distanceMm > (UINT32_MAX / PULSES_PER_MM)) {
            Serial.println("Error: distance is too large.");
            return;
        }
        const uint32_t steps = distanceMm * PULSES_PER_MM;
        startConstantMotion(cmd == "D" ? DIR_RIGHT_LEVEL : DIR_LEFT_LEVEL, steps, MANUAL_MOVE_DELAY_US);
        return;
    }

    if (cmd == "3") {
        startCycle3(false);
        return;
    }

    if (cmd == "4") {
        startCycle3(true);
        return;
    }

    if (cmd == "BC") {
        uint32_t platesCount = BC_BATCH_PLATES_DEFAULT;
        if (!args.isEmpty()) {
            if (!parseUnsigned(args, platesCount)) {
                Serial.println("Format error. Example: BC 2");
                return;
            }
        }

        if (platesCount == 0U || platesCount > BC_BATCH_PLATES_MAX) {
            Serial.print("Error: BC supports 1..");
            Serial.print(BC_BATCH_PLATES_MAX);
            Serial.println(" plates.");
            return;
        }

        startBCTest(static_cast<uint8_t>(platesCount));
        return;
    }

    if (cmd == "W") {
        digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);
        Serial.println("Flag: UP.");
        return;
    }

    if (cmd == "S") {
        digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
        Serial.println("Flag: DOWN.");
        return;
    }

    if (cmd == "E") {
        g_sensorStreamEnabled = !g_sensorStreamEnabled;
        g_lastSensorPrintMs = millis();
        Serial.println(g_sensorStreamEnabled ? "Sensor stream ON." : "Sensor stream OFF.");
        return;
    }

    if (cmd == "H") {
        printHelp();
        return;
    }

    Serial.println("Unknown command. Use H for help.");
}

void readSerialCommands()
{
    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            handleCommand(g_cmdBuffer);
            g_cmdBuffer = "";
            continue;
        }

        g_cmdBuffer += ch;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(300);

    pinMode(PIN_STEP_PUL, OUTPUT);
    pinMode(PIN_STEP_DIR, OUTPUT);
    pinMode(PIN_STEP_ENA, OUTPUT);
    pinMode(PIN_FLAG, OUTPUT);
    pinMode(PIN_SENSOR, INPUT_PULLUP);

    writePulseInactive();
    digitalWrite(PIN_STEP_DIR, DIR_LEFT_LEVEL);
    digitalWrite(PIN_STEP_ENA, ENA_ACTIVE_LEVEL);
    digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
    updateSensorFilter();

    Serial.println();
    Serial.println("=== Управление подающим конвейером ===");
    Serial.print("Версия прошивки: ");
    Serial.println(FW_VERSION);
    Serial.println("Плата: ESP32 Dev Module");
    Serial.println("DM542: PUL=13, DIR=12, ENA=14");
    Serial.println("Флаг вверх: GPIO27=HIGH");
    Serial.println("Команды не чувствительны к регистру.");
    printHelp();
}

void loop()
{
    updateSensorFilter();
    readSerialCommands();
    processCycle3();
    processBCTest();
    processMotion();
    processSensorStream();
}
