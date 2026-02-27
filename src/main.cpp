#include <Arduino.h>

#include "wifi_console.h"
#include "mqtt_link.h"

constexpr char FW_VERSION[] = "v1.7";

constexpr uint8_t PIN_STEP_PUL = 13; // DM542 PUL
constexpr uint8_t PIN_FLAG = 27;     // Пневмоцилиндр флага
constexpr uint8_t PIN_SENSOR = 26;   // E18-D50NK (активный HIGH)
constexpr uint8_t PIN_POS_PUL = 32;  // EVA25 PUL

constexpr bool FLAG_UP_LEVEL = HIGH;
constexpr bool FLAG_DOWN_LEVEL = LOW;
constexpr bool PULSE_ACTIVE_LEVEL = HIGH;

constexpr uint32_t STEP_PULSE_WIDTH_US = 10;
constexpr uint32_t POS_RUN_DELAY_US = 1000;
constexpr uint32_t SENSOR_PRINT_INTERVAL_MS = 500;
constexpr uint32_t SENSOR_DEBOUNCE_MS = 80;
constexpr uint32_t MANUAL_MOVE_DELAY_US = 1500; // Фиксированная задержка для команд A/D
constexpr uint32_t MOTION_START_DELAY_MULT_NUM = 2;
constexpr uint32_t MOTION_START_DELAY_MULT_DEN = 1;
constexpr uint32_t MOTION_RAMP_STEPS_DEFAULT = 150;
constexpr uint32_t PULSES_PER_MM = 5;           // 2000 импульсов = 400 мм => 5 имп/мм
constexpr uint32_t C3_COMMAND_DISTANCE_MM = 184U;
constexpr uint32_t C3_COMMAND_STEPS = C3_COMMAND_DISTANCE_MM * PULSES_PER_MM;
constexpr uint32_t C3_COMMAND_DELAY_US = 1200U;

constexpr uint32_t C2_MOVE_DELAY_US = 1500;
constexpr uint32_t C2_CENTER_STEPS = 20U * PULSES_PER_MM; // 20 мм
constexpr uint32_t C2_FLAG_REOPEN_STEPS = 80U * PULSES_PER_MM; // 80 мм
constexpr uint32_t C2_PLATE_DIAMETER_MM = 150U;
constexpr uint32_t C2_TARGET_GAP_MM = 34U;
constexpr uint32_t C2_SENSOR_EFFECTIVE_PLATE_MM = 184U; // Калибровка по факту: при 184 мм тарелки встали вплотную
constexpr uint32_t C2_FIRST_TO_STOP_AFTER_LEAVE_STEPS = 315U * PULSES_PER_MM;
constexpr uint32_t C2_LEAVE_DELTA_TARGET_STEPS = (C2_SENSOR_EFFECTIVE_PLATE_MM + C2_TARGET_GAP_MM) * PULSES_PER_MM; // 218 мм
constexpr uint32_t C2_FINAL_AFTER_SECOND_LEAVE_BASE_STEPS = 408U; // 81.6 мм (зафиксировано по калибровке 2Q)
constexpr uint32_t C2_UT_REFERENCE_STEPS = 248U; // 49.6 мм, калибровочная точка
constexpr int32_t C2_FINAL_UT_SLOPE_NUM = 11; // 1.1 шаг/UT (зафиксировано по калибровке 2Q)
constexpr int32_t C2_FINAL_UT_SLOPE_DEN = 10;
constexpr uint32_t C2_FINAL_AFTER_SECOND_LEAVE_MIN_STEPS = 80U * PULSES_PER_MM; // 80 мм
constexpr uint32_t C2_FINAL_AFTER_SECOND_LEAVE_MAX_STEPS = 130U * PULSES_PER_MM; // 130 мм

struct MotionState {
    bool active = false;
    bool pulseHigh = false;
    uint32_t stepsTotal = 0;
    uint32_t stepsDone = 0;
    uint32_t nominalDelayUs = 1000;
    uint32_t currentDelayUs = 1000;
    uint32_t startDelayUs = 1000;
    uint32_t rampSteps = 1;
    uint32_t lastPulseStartUs = 0;
};

struct PosMotionState {
    bool active = false;
    bool pulseHigh = false;
    uint32_t stepsTotal = 0;
    uint32_t stepsDone = 0;
    uint32_t delayUs = POS_RUN_DELAY_US;
    uint32_t currentDelayUs = POS_RUN_DELAY_US;
    uint32_t startDelayUs = POS_RUN_DELAY_US;
    uint32_t rampSteps = 1;
    uint32_t lastPulseStartUs = 0;
};

struct SensorFilterState {
    bool initialized = false;
    bool lastRawPlateDetected = false;
    bool stablePlateDetected = false;
    uint32_t lastRawChangeMs = 0;
};

enum class C2State : uint8_t {
    Idle,
    SeekFirstPlate,
    CenterFirstPlate,
    TrackFirstLeaveAndOpen,
    SeekSecondPlate,
    CenterSecondPlate,
    WaitSecondReleaseTiming,
    MoveAfterSecondRelease,
    WaitSecondLeave,
    FinalMove
};

String g_cmdBuffer;
MotionState g_motion;
PosMotionState g_posMotion;
SensorFilterState g_sensorFilter;
bool g_sensorStreamEnabled = false;
uint32_t g_lastSensorPrintMs = 0;
uint32_t g_totalStepsCounter = 0;

bool g_c2Active = false;
C2State g_c2State = C2State::Idle;
bool g_c2FirstLeaveCaptured = false;
bool g_c2SecondLeaveCaptured = false;
uint32_t g_c2CenterStartStep = 0;
uint32_t g_c2FirstFlagDownStep = 0;
uint32_t g_c2FirstLeaveStep = 0;
uint32_t g_c2FirstUTSteps = 0;
uint32_t g_c2SecondFlagDownStep = 0;
uint32_t g_c2SecondLeaveStep = 0;
uint32_t g_c2FinalStartStep = 0;
uint32_t g_c2FinalMoveSteps = 0;
uint32_t g_c2CycleStartMs = 0;
int32_t g_c2FinalBaseSteps = static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_BASE_STEPS);
int32_t g_c2FinalSlopeNum = C2_FINAL_UT_SLOPE_NUM;
int32_t g_c2FinalSlopeDen = C2_FINAL_UT_SLOPE_DEN;

bool isPlateAtSensorFiltered();
void stopMotion();
void startCycle2();
void processPositionalMotion();
void printIntegrationStatus();

void printHelp()
{
    Serial.println("Команды:");
    Serial.println("  D 200        - вправо, 200 мм (задержка 1500 мкс)");
    Serial.println("  2            - рабочий ход: 1-я тарелка в упоре, 2-я с зазором 34 мм");
    Serial.println("  3            - позиционный (PUL32): вправо 184 мм, профиль, полка 1200 мкс");
    Serial.println("  W            - флаг вверх");
    Serial.println("  S            - флаг вниз");
    Serial.println("  E            - вкл/выкл поток датчика (каждые 0.5 сек)");
    Serial.println("  IQ           - состояние внешних систем (WiFi + MQTT)");
    Serial.println("  P 300        - позиционный (PUL32): вправо 300 мм, профиль, полка 1000 мкс");
    WifiConsole::printHelp();
    Serial.println("  H            - помощь");
}

void printIntegrationStatus()
{
    Serial.println("=== Integration status ===");
    WifiConsole::printStatus();
    MqttLink::printStatus();
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

uint32_t interpolateDelayUs(uint32_t progressSteps, uint32_t rampSteps, uint32_t startDelayUs, uint32_t nominalDelayUs)
{
    if (rampSteps == 0 || progressSteps >= rampSteps || startDelayUs <= nominalDelayUs) {
        return nominalDelayUs;
    }

    const uint32_t delta = startDelayUs - nominalDelayUs;
    const uint32_t dec = static_cast<uint32_t>((static_cast<uint64_t>(delta) * progressSteps) / rampSteps);
    return startDelayUs - dec;
}

void updateMotionProfileDelay()
{
    if (!g_motion.active) {
        return;
    }

    const uint32_t rampSteps = g_motion.rampSteps == 0 ? 1 : g_motion.rampSteps;

    uint32_t accelProgress = g_motion.stepsDone;
    if (accelProgress > rampSteps) {
        accelProgress = rampSteps;
    }

    uint32_t targetDelayUs = interpolateDelayUs(
        accelProgress, rampSteps, g_motion.startDelayUs, g_motion.nominalDelayUs);

    if (g_motion.stepsTotal != UINT32_MAX) {
        uint32_t remainingSteps = 0;
        if (g_motion.stepsTotal > g_motion.stepsDone) {
            remainingSteps = g_motion.stepsTotal - g_motion.stepsDone;
        }

        uint32_t decelProgress = remainingSteps;
        if (decelProgress > rampSteps) {
            decelProgress = rampSteps;
        }

        const uint32_t decelDelayUs = interpolateDelayUs(
            decelProgress, rampSteps, g_motion.startDelayUs, g_motion.nominalDelayUs);
        if (decelDelayUs > targetDelayUs) {
            targetDelayUs = decelDelayUs;
        }
    }

    if (targetDelayUs < g_motion.nominalDelayUs) {
        targetDelayUs = g_motion.nominalDelayUs;
    }
    g_motion.currentDelayUs = targetDelayUs;
}

void startMotionProfiled(uint32_t stepsTotal, uint32_t nominalDelayUs)
{
    if (nominalDelayUs == 0) {
        nominalDelayUs = 1;
    }

    g_motion.active = true;
    g_motion.pulseHigh = false;
    g_motion.stepsTotal = stepsTotal;
    g_motion.stepsDone = 0;
    g_motion.nominalDelayUs = nominalDelayUs;
    g_motion.currentDelayUs = nominalDelayUs;

    uint32_t startDelayUs = static_cast<uint32_t>(
        (static_cast<uint64_t>(nominalDelayUs) * MOTION_START_DELAY_MULT_NUM) / MOTION_START_DELAY_MULT_DEN);
    if (startDelayUs <= nominalDelayUs) {
        startDelayUs = nominalDelayUs + 1;
    }
    g_motion.startDelayUs = startDelayUs;

    g_motion.rampSteps = MOTION_RAMP_STEPS_DEFAULT;
    if (stepsTotal != UINT32_MAX) {
        uint32_t halfSteps = stepsTotal / 2U;
        if (halfSteps < g_motion.rampSteps) {
            g_motion.rampSteps = halfSteps;
        }
        if (g_motion.rampSteps == 0) {
            g_motion.rampSteps = 1;
        }
    }

    g_motion.lastPulseStartUs = micros();
    updateMotionProfileDelay();
}

void armMotionStopAfterSteps(uint32_t stepsToStop)
{
    if (!g_motion.active) {
        return;
    }

    if (stepsToStop == 0) {
        stopMotion();
        return;
    }

    if (g_motion.stepsDone > (UINT32_MAX - stepsToStop)) {
        g_motion.stepsTotal = UINT32_MAX;
    } else {
        g_motion.stepsTotal = g_motion.stepsDone + stepsToStop;
    }

    g_motion.rampSteps = MOTION_RAMP_STEPS_DEFAULT;
    uint32_t halfSteps = stepsToStop / 2U;
    if (halfSteps < g_motion.rampSteps) {
        g_motion.rampSteps = halfSteps;
    }
    if (g_motion.rampSteps == 0) {
        g_motion.rampSteps = 1;
    }

    updateMotionProfileDelay();
}

void writePulseInactive()
{
    digitalWrite(PIN_STEP_PUL, PULSE_ACTIVE_LEVEL ? LOW : HIGH);
}

void writePosPulseInactive()
{
    digitalWrite(PIN_POS_PUL, PULSE_ACTIVE_LEVEL ? LOW : HIGH);
}

void stopPositionalMotion()
{
    writePosPulseInactive();
    g_posMotion.active = false;
    g_posMotion.pulseHigh = false;
    g_posMotion.stepsTotal = 0;
    g_posMotion.stepsDone = 0;
    Serial.println("P: EVA25 stopped.");
}

void updatePositionalMotionProfileDelay()
{
    if (!g_posMotion.active) {
        return;
    }

    const uint32_t rampSteps = g_posMotion.rampSteps == 0 ? 1 : g_posMotion.rampSteps;

    uint32_t accelProgress = g_posMotion.stepsDone;
    if (accelProgress > rampSteps) {
        accelProgress = rampSteps;
    }

    uint32_t targetDelayUs = interpolateDelayUs(
        accelProgress, rampSteps, g_posMotion.startDelayUs, g_posMotion.delayUs);

    uint32_t remainingSteps = 0;
    if (g_posMotion.stepsTotal > g_posMotion.stepsDone) {
        remainingSteps = g_posMotion.stepsTotal - g_posMotion.stepsDone;
    }

    uint32_t decelProgress = remainingSteps;
    if (decelProgress > rampSteps) {
        decelProgress = rampSteps;
    }

    const uint32_t decelDelayUs = interpolateDelayUs(
        decelProgress, rampSteps, g_posMotion.startDelayUs, g_posMotion.delayUs);
    if (decelDelayUs > targetDelayUs) {
        targetDelayUs = decelDelayUs;
    }

    if (targetDelayUs < g_posMotion.delayUs) {
        targetDelayUs = g_posMotion.delayUs;
    }
    g_posMotion.currentDelayUs = targetDelayUs;
}

void startPositionalProfiledMotion(uint32_t stepsTotal, uint32_t nominalDelayUs)
{
    if (g_posMotion.active) {
        Serial.println("POS: позиционный мотор уже в движении.");
        return;
    }

    if (stepsTotal == 0U) {
        Serial.println("POS: шагов 0, движение не требуется.");
        return;
    }

    if (nominalDelayUs == 0U) {
        nominalDelayUs = 1U;
    }

    writePosPulseInactive();

    g_posMotion.active = true;
    g_posMotion.pulseHigh = false;
    g_posMotion.stepsTotal = stepsTotal;
    g_posMotion.stepsDone = 0;
    g_posMotion.delayUs = nominalDelayUs;
    g_posMotion.currentDelayUs = nominalDelayUs;

    uint32_t startDelayUs = static_cast<uint32_t>(
        (static_cast<uint64_t>(nominalDelayUs) * MOTION_START_DELAY_MULT_NUM) / MOTION_START_DELAY_MULT_DEN);
    if (startDelayUs <= nominalDelayUs) {
        startDelayUs = nominalDelayUs + 1U;
    }
    g_posMotion.startDelayUs = startDelayUs;

    g_posMotion.rampSteps = MOTION_RAMP_STEPS_DEFAULT;
    uint32_t halfSteps = stepsTotal / 2U;
    if (halfSteps < g_posMotion.rampSteps) {
        g_posMotion.rampSteps = halfSteps;
    }
    if (g_posMotion.rampSteps == 0U) {
        g_posMotion.rampSteps = 1U;
    }

    g_posMotion.lastPulseStartUs = micros();
    updatePositionalMotionProfileDelay();

    Serial.print("POS: профилированный ход, шагов=");
    Serial.print(stepsTotal);
    Serial.print(", расстояние=");
    Serial.print(static_cast<float>(stepsTotal) / static_cast<float>(PULSES_PER_MM), 1);
    Serial.print(" мм");
    Serial.print(", полка=");
    Serial.print(nominalDelayUs);
    Serial.println(" мкс.");
}

void processPositionalMotion()
{
    if (!g_posMotion.active) {
        return;
    }

    const uint32_t nowUs = micros();

    if (!g_posMotion.pulseHigh) {
        if ((uint32_t)(nowUs - g_posMotion.lastPulseStartUs) >= g_posMotion.currentDelayUs) {
            digitalWrite(PIN_POS_PUL, PULSE_ACTIVE_LEVEL ? HIGH : LOW);
            g_posMotion.pulseHigh = true;
            g_posMotion.lastPulseStartUs = nowUs;
        }
        return;
    }

    if ((uint32_t)(nowUs - g_posMotion.lastPulseStartUs) >= STEP_PULSE_WIDTH_US) {
        writePosPulseInactive();
        g_posMotion.pulseHigh = false;
        g_posMotion.stepsDone++;
        if (g_posMotion.stepsDone >= g_posMotion.stepsTotal) {
            stopPositionalMotion();
            Serial.println("POS: движение завершено.");
        } else {
            updatePositionalMotionProfileDelay();
        }
    }
}

void stopMotion()
{
    writePulseInactive();
    g_motion.active = false;
    g_motion.pulseHigh = false;
}

void startConstantMotion(uint32_t steps, uint32_t delayUs)
{
    if (g_motion.active) {
        Serial.println("Ошибка: двигатель уже в движении.");
        return;
    }

    if (steps == 0) {
        Serial.println("Шагов 0: движение не требуется.");
        return;
    }

    startMotionProfiled(steps, delayUs);

    Serial.print("Запуск: шагов=");
    Serial.print(steps);
    Serial.print(", задержка=");
    Serial.print(delayUs);
    Serial.println(" мкс, направление=вправо");
}

void startCycle2()
{
    if (g_motion.active || g_c2Active) {
        Serial.println("C2: ошибка, двигатель уже в движении.");
        return;
    }

    const bool plateAlreadyOnSensor = g_sensorFilter.stablePlateDetected;

    digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);
    startMotionProfiled(UINT32_MAX, C2_MOVE_DELAY_US);

    g_c2Active = true;
    g_c2State = plateAlreadyOnSensor ? C2State::CenterFirstPlate : C2State::SeekFirstPlate;
    g_c2FirstLeaveCaptured = false;
    g_c2SecondLeaveCaptured = false;
    g_c2CenterStartStep = g_totalStepsCounter;
    g_c2FirstFlagDownStep = g_totalStepsCounter;
    g_c2FirstLeaveStep = g_totalStepsCounter;
    g_c2FirstUTSteps = 0;
    g_c2SecondFlagDownStep = g_totalStepsCounter;
    g_c2SecondLeaveStep = g_totalStepsCounter;
    g_c2FinalStartStep = g_totalStepsCounter;
    g_c2FinalMoveSteps = 0;
    g_c2CycleStartMs = millis();

    Serial.println("C2: старт. Цель: 1-я тарелка в упоре, 2-я с зазором 34 мм.");
    if (plateAlreadyOnSensor) {
        Serial.println("C2: тарелка уже на датчике, выполняем стартовое центрирование +20 мм (флаг поднят).");
    }
}

void captureC2FirstLeaveIfNeeded()
{
    if (g_c2FirstLeaveCaptured || isPlateAtSensorFiltered()) {
        return;
    }

    g_c2FirstLeaveCaptured = true;
    g_c2FirstLeaveStep = g_totalStepsCounter;
    g_c2FirstUTSteps = g_totalStepsCounter - g_c2FirstFlagDownStep;

    Serial.print("C2: 1-я тарелка ушла с датчика, UT=");
    Serial.print(g_c2FirstUTSteps);
    Serial.print(" шагов (");
    Serial.print(static_cast<float>(g_c2FirstUTSteps) / static_cast<float>(PULSES_PER_MM), 1);
    Serial.println(" мм).");
}

void captureC2SecondLeaveIfNeeded()
{
    if (g_c2SecondLeaveCaptured || isPlateAtSensorFiltered()) {
        return;
    }

    g_c2SecondLeaveCaptured = true;
    g_c2SecondLeaveStep = g_totalStepsCounter;
    Serial.println("C2: 2-я тарелка ушла с датчика.");
}

void scheduleC2FinalMove()
{
    if (g_c2FinalSlopeDen == 0) {
        g_c2FinalSlopeDen = 1;
    }

    const int32_t utDeltaSteps = static_cast<int32_t>(g_c2FirstUTSteps) -
                                 static_cast<int32_t>(C2_UT_REFERENCE_STEPS);
    int32_t correctionNumerator = utDeltaSteps * g_c2FinalSlopeNum;
    if (correctionNumerator >= 0) {
        correctionNumerator += (g_c2FinalSlopeDen / 2);
    } else {
        correctionNumerator -= (g_c2FinalSlopeDen / 2);
    }
    const int32_t utCorrectionSteps = correctionNumerator / g_c2FinalSlopeDen;

    int32_t adaptiveSteps = g_c2FinalBaseSteps + utCorrectionSteps;
    if (adaptiveSteps < static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MIN_STEPS)) {
        adaptiveSteps = static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MIN_STEPS);
    }
    if (adaptiveSteps > static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MAX_STEPS)) {
        adaptiveSteps = static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MAX_STEPS);
    }

    const uint32_t elapsedSinceFirstLeave = g_totalStepsCounter - g_c2FirstLeaveStep;
    uint32_t remainingForFirstToStop = 0;
    if (elapsedSinceFirstLeave < C2_FIRST_TO_STOP_AFTER_LEAVE_STEPS) {
        remainingForFirstToStop = C2_FIRST_TO_STOP_AFTER_LEAVE_STEPS - elapsedSinceFirstLeave;
    }

    g_c2FinalMoveSteps = static_cast<uint32_t>(adaptiveSteps);
    if (remainingForFirstToStop > g_c2FinalMoveSteps) {
        g_c2FinalMoveSteps = remainingForFirstToStop;
    }

    g_c2FinalStartStep = g_totalStepsCounter;
    g_c2State = C2State::FinalMove;
    armMotionStopAfterSteps(g_c2FinalMoveSteps);

    Serial.print("C2: финальный добег ");
    Serial.print(static_cast<float>(g_c2FinalMoveSteps) / static_cast<float>(PULSES_PER_MM), 1);
    Serial.println(" мм.");
}

void processCycle2()
{
    if (!g_c2Active || g_c2State == C2State::Idle) {
        return;
    }

    if (!g_motion.active) {
        if (g_c2State == C2State::FinalMove) {
            g_c2Active = false;
            g_c2State = C2State::Idle;
            Serial.println("C2: цикл завершен.");
            const uint32_t cycleDurationMs = millis() - g_c2CycleStartMs;
            Serial.print("C2: время цикла ");
            Serial.print(cycleDurationMs);
            Serial.print(" мс (");
            Serial.print(static_cast<float>(cycleDurationMs) / 1000.0F, 1);
            Serial.println(" с).");
            return;
        }

        g_c2Active = false;
        g_c2State = C2State::Idle;
        Serial.println("C2: сценарий прерван (двигатель остановлен вне C2).");
        return;
    }

    switch (g_c2State) {
        case C2State::SeekFirstPlate:
            if (isPlateAtSensorFiltered()) {
                g_c2CenterStartStep = g_totalStepsCounter;
                g_c2State = C2State::CenterFirstPlate;
                Serial.println("C2: 1-я тарелка найдена, центрирование +20 мм.");
            }
            return;

        case C2State::CenterFirstPlate:
            if ((uint32_t)(g_totalStepsCounter - g_c2CenterStartStep) >= C2_CENTER_STEPS) {
                digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
                g_c2FirstFlagDownStep = g_totalStepsCounter;
                g_c2FirstLeaveCaptured = false;
                g_c2FirstUTSteps = 0;
                g_c2State = C2State::TrackFirstLeaveAndOpen;
                Serial.println("C2: флаг опущен для 1-й тарелки.");
            }
            return;

        case C2State::TrackFirstLeaveAndOpen:
            captureC2FirstLeaveIfNeeded();

            if ((uint32_t)(g_totalStepsCounter - g_c2FirstFlagDownStep) >= C2_FLAG_REOPEN_STEPS) {
                digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);
                g_c2State = C2State::SeekSecondPlate;
                Serial.println("C2: флаг поднят, поиск 2-й тарелки.");
            }
            return;

        case C2State::SeekSecondPlate:
            captureC2FirstLeaveIfNeeded();

            if (isPlateAtSensorFiltered()) {
                g_c2CenterStartStep = g_totalStepsCounter;
                g_c2State = C2State::CenterSecondPlate;
                Serial.println("C2: 2-я тарелка найдена, центрирование +20 мм.");
            }
            return;

        case C2State::CenterSecondPlate:
            captureC2FirstLeaveIfNeeded();

            if ((uint32_t)(g_totalStepsCounter - g_c2CenterStartStep) >= C2_CENTER_STEPS) {
                g_c2State = C2State::WaitSecondReleaseTiming;
                Serial.println("C2: 2-я тарелка центрирована, расчет момента отпускания.");
            }
            return;

        case C2State::WaitSecondReleaseTiming:
            captureC2FirstLeaveIfNeeded();

            if (!g_c2FirstLeaveCaptured) {
                return;
            }

            {
                uint32_t releaseAfterFirstLeaveSteps = 0;
                if (g_c2FirstUTSteps < C2_LEAVE_DELTA_TARGET_STEPS) {
                    releaseAfterFirstLeaveSteps = C2_LEAVE_DELTA_TARGET_STEPS - g_c2FirstUTSteps;
                }

                if ((uint32_t)(g_totalStepsCounter - g_c2FirstLeaveStep) >= releaseAfterFirstLeaveSteps) {
                    digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
                    g_c2SecondFlagDownStep = g_totalStepsCounter;
                    g_c2SecondLeaveCaptured = false;
                    g_c2State = C2State::MoveAfterSecondRelease;
                    Serial.println("C2: флаг опущен для 2-й тарелки.");
                }
            }
            return;

        case C2State::MoveAfterSecondRelease:
            captureC2SecondLeaveIfNeeded();

            if ((uint32_t)(g_totalStepsCounter - g_c2SecondFlagDownStep) >= C2_FLAG_REOPEN_STEPS) {
                digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);

                if (g_c2SecondLeaveCaptured) {
                    scheduleC2FinalMove();
                } else {
                    g_c2State = C2State::WaitSecondLeave;
                    Serial.println("C2: ждем уход 2-й тарелки с датчика.");
                }
            }
            return;

        case C2State::WaitSecondLeave:
            captureC2SecondLeaveIfNeeded();
            if (g_c2SecondLeaveCaptured) {
                scheduleC2FinalMove();
            }
            return;

        case C2State::FinalMove:
            return;

        case C2State::Idle:
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
        if ((uint32_t)(nowUs - g_motion.lastPulseStartUs) >= g_motion.currentDelayUs) {
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
        } else {
            updateMotionProfileDelay();
        }
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

    if (cmd == "D") {
        uint32_t distanceMm = 0;
        if (!parseDistanceMmArgs(args, cmd, distanceMm)) {
            Serial.println("Format error. Example: D 200");
            return;
        }
        if (distanceMm > (UINT32_MAX / PULSES_PER_MM)) {
            Serial.println("Error: distance is too large.");
            return;
        }
        const uint32_t steps = distanceMm * PULSES_PER_MM;
        startConstantMotion(steps, MANUAL_MOVE_DELAY_US);
        return;
    }

    if (cmd == "P") {
        uint32_t distanceMm = 0;
        if (!parseDistanceMmArgs(args, cmd, distanceMm)) {
            Serial.println("Format error. Example: P 300");
            return;
        }
        if (distanceMm > (UINT32_MAX / PULSES_PER_MM)) {
            Serial.println("Error: distance is too large.");
            return;
        }
        const uint32_t steps = distanceMm * PULSES_PER_MM;
        startPositionalProfiledMotion(steps, POS_RUN_DELAY_US);
        return;
    }

    if (cmd == "3") {
        startPositionalProfiledMotion(C3_COMMAND_STEPS, C3_COMMAND_DELAY_US);
        return;
    }

    if (cmd == "2") {
        startCycle2();
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

    if (cmd == "IQ") {
        printIntegrationStatus();
        return;
    }

    if (WifiConsole::handleCommand(
            cmd, args, g_motion.active || g_c2Active)) {
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

void handleMqttCommand(const String &line)
{
    handleCommand(line);
}

void setup()
{
    Serial.begin(115200);
    delay(300);

    pinMode(PIN_STEP_PUL, OUTPUT);
    pinMode(PIN_FLAG, OUTPUT);
    pinMode(PIN_SENSOR, INPUT_PULLUP);
    pinMode(PIN_POS_PUL, OUTPUT);

    writePulseInactive();
    digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
    writePosPulseInactive();
    updateSensorFilter();
    WifiConsole::begin();
    MqttLink::setCommandHandler(handleMqttCommand);
    MqttLink::begin();

    Serial.println();
    Serial.println("=== Управление подающим конвейером ===");
    Serial.print("Версия прошивки: ");
    Serial.println(FW_VERSION);
    Serial.println("Плата: ESP32 Dev Module");
    Serial.println("DM542: PUL=13");
    Serial.println("DM542 positional: PUL=GPIO32");
    Serial.println("Запущено два конвейера.");
    Serial.println("GPIO 12/14/25/33 не используются.");
    Serial.println("Sensor: GPIO26 (E18-D50NK)");
    Serial.println("Флаг вверх: GPIO27=HIGH");
    Serial.println("WiFi: команды WSCAN/WIFI/WSTAT/WDIS.");
    Serial.println("MQTT: публикация статуса в underwater_conveyor/status.");
    Serial.println("Команды не чувствительны к регистру.");
    printHelp();
}

void loop()
{
    updateSensorFilter();
    readSerialCommands();

    if (!g_posMotion.active) {
        WifiConsole::process();
        MqttLink::process();
    }

    processCycle2();
    processMotion();
    processPositionalMotion();
    processSensorStream();
}
