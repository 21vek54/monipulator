#include <Arduino.h>
#include <Preferences.h>

#include "wifi_console.h"
#include "mqtt_link.h"

constexpr char FW_VERSION[] = "v1.9";

constexpr uint8_t PIN_STEP_PUL = 13; // DM542 PUL
constexpr uint8_t PIN_SHIFT_DIR = 14; // Сдвиг тарелки DIR
constexpr uint8_t PIN_SHIFT_PUL = 12; // Сдвиг тарелки PUL
constexpr uint8_t PIN_SHIFT_SENSOR_Z = 33; // Геркон края Z (HIGH при магните)
constexpr uint8_t PIN_SHIFT_SENSOR_C = 25; // Геркон края C (HIGH при магните)
constexpr uint8_t PIN_FLAG = 27;     // Пневмоцилиндр флага
constexpr uint8_t PIN_SENSOR = 26;   // E18-D50NK (активный HIGH)
constexpr uint8_t PIN_POS_PUL = 32;  // EVA25 PUL

constexpr bool FLAG_UP_LEVEL = HIGH;
constexpr bool FLAG_DOWN_LEVEL = LOW;
constexpr bool PULSE_ACTIVE_LEVEL = HIGH;
constexpr bool SHIFT_DIR_C_LEVEL = HIGH;
constexpr bool SHIFT_DIR_Z_LEVEL = LOW;

constexpr uint32_t STEP_PULSE_WIDTH_US = 10;
constexpr uint32_t POS_RUN_DELAY_US = 1000;
constexpr uint32_t SHIFT_COMMAND_STEPS = 500; // Фиксированный ход C/Z до привязки или без поиска концевика
constexpr uint32_t SHIFT_COMMAND_DELAY_C_US = 300; // Полка скорости команды C
constexpr uint32_t SHIFT_COMMAND_DELAY_Z_US = 50; // Полка скорости команды Z: меньше значение = быстрее возврат
constexpr uint32_t SHIFT_PULSE_WIDTH_US = 120; // Длительность STEP-импульса для привода сдвига
constexpr uint32_t SHIFT_START_DELAY_C_US = 3000; // Стартовая задержка для плавного разгона команды C
constexpr uint32_t SHIFT_START_DELAY_Z_US = 1000; // Стартовая задержка для плавного разгона команды Z
constexpr uint32_t SHIFT_RAMP_STEPS_C = 500; // Длина разгона команды C
constexpr uint32_t SHIFT_RAMP_STEPS_Z = 180; // Длина разгона команды Z
constexpr uint32_t SHIFT_DECEL_STEPS_C = 1000; // Длина торможения только для команды C
constexpr uint32_t SHIFT_SENSOR_DEBOUNCE_MS = 25;
constexpr uint32_t SHIFT_LEAVE_SENSOR_MAX_STEPS = 300;
constexpr uint32_t SHIFT_CAL_SEARCH_MAX_STEPS = 12000;
constexpr uint32_t SHIFT_CAL_MOVE_MARGIN_STEPS = 50; // Запас шагов сверх номинального хода при поиске края
constexpr uint32_t SHIFT_TRAVEL_STEPS_FIXED = 2255; // Фиксированный ход между краями C и Z после старта
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

enum class Program1State : uint8_t {
    Idle,
    StartBufferedPass1,
    WaitFirstC2Done,
    WaitSecondC2Done,
    WaitThirdC2Done,
    WaitFinalPositionalDone,
    WaitBufferFillC2Done
};

enum class Program1MetricKind : uint8_t {
    Cycle2,
    ShiftC,
    Pos3,
    ShiftZ,
    BufferFill
};

struct Program1Metric {
    Program1MetricKind kind = Program1MetricKind::Cycle2;
    uint8_t passIndex = 0;
    uint32_t startMs = 0;
    uint32_t durationMs = 0;
    bool active = false;
    bool finished = false;
};

String g_cmdBuffer;
Preferences g_preferences;
MotionState g_motion;
PosMotionState g_posMotion;
SensorFilterState g_sensorFilter;
SensorFilterState g_shiftSensorZFilter;
SensorFilterState g_shiftSensorCFilter;
bool g_sensorStreamEnabled = false;
uint32_t g_lastSensorPrintMs = 0;
uint32_t g_totalStepsCounter = 0;
bool g_shiftCalibrated = true;
uint32_t g_shiftTravelSteps = SHIFT_TRAVEL_STEPS_FIXED;
Program1State g_program1State = Program1State::Idle;
uint32_t g_program1StartMs = 0;
Program1Metric g_program1Metrics[12] = {};
uint8_t g_program1MetricCount = 0;
int8_t g_program1ActiveC2Metric = -1;
int8_t g_program1ActivePosMetric = -1;
bool g_program1BufferReady = false;
bool g_program1StorageReady = false;

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
bool isShiftSensorZTriggered();
bool isShiftSensorCTriggered();
void stopMotion();
void startCycle2();
void startPositionalProfiledMotion(uint32_t stepsTotal, uint32_t nominalDelayUs);
void startProgram1();
void processPositionalMotion();
void printIntegrationStatus();
void processProgram1();
void updateSensorFilter();
void updateShiftSensorFilters();
void homeShiftToZOnStartup();
void initProgram1Storage();
void setProgram1BufferReady(bool ready);

void printHelp()
{
    Serial.println("Команды:");
    Serial.println("  D 200        - вправо, 200 мм (задержка 1500 мкс)");
    Serial.println("  1            - автоцикл: 2 -> C+(3+Z) -> 2 -> C+(3+Z) -> 2 -> C+Z");
    Serial.println("  2            - рабочий ход: 1-я тарелка в упоре, 2-я с зазором 34 мм");
    Serial.println("  3            - позиционный (PUL32): вправо 184 мм, профиль, полка 1200 мкс");
    Serial.println("  W            - флаг вверх");
    Serial.println("  S            - флаг вниз");
    Serial.println("  E            - вкл/выкл поток датчиков (E18 + 2 геркона, каждые 0.5 сек)");
    Serial.println("  IQ           - состояние внешних систем (WiFi + MQTT)");
    Serial.println("  P 300        - позиционный (PUL32): вправо 300 мм, профиль, полка 1000 мкс");
    Serial.println("  C            - сдвиг тарелки: 500 шагов (DIR14/PUL12), задержка 500 мкс, S-профиль, длинное торможение");
    Serial.println("  Z            - возврат тарелки: 500 шагов (DIR14/PUL12), задержка 300 мкс, плавный ход");
    Serial.println("  CZ           - калибровка хода сдвига по герконам Z=33 и C=25");
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

void updateDebouncedSensorFilter(
    SensorFilterState &state,
    bool rawState,
    uint32_t debounceMs)
{
    const uint32_t nowMs = millis();

    if (!state.initialized) {
        state.initialized = true;
        state.lastRawPlateDetected = rawState;
        state.stablePlateDetected = rawState;
        state.lastRawChangeMs = nowMs;
        return;
    }

    if (rawState != state.lastRawPlateDetected) {
        state.lastRawPlateDetected = rawState;
        state.lastRawChangeMs = nowMs;
    }

    if (rawState != state.stablePlateDetected &&
        (uint32_t)(nowMs - state.lastRawChangeMs) >= debounceMs) {
        state.stablePlateDetected = rawState;
    }
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

void writeShiftPulseInactive()
{
    digitalWrite(PIN_SHIFT_PUL, PULSE_ACTIVE_LEVEL ? LOW : HIGH);
}

void serviceShiftBackground(bool allowPositionalOverlap)
{
    updateSensorFilter();
    updateShiftSensorFilters();
    if (allowPositionalOverlap) {
        processPositionalMotion();
    }
}

void delayShiftUs(uint32_t delayUs, bool allowPositionalOverlap)
{
    const uint32_t startUs = micros();
    while ((uint32_t)(micros() - startUs) < delayUs) {
        serviceShiftBackground(allowPositionalOverlap);
    }
}

uint32_t getShiftNominalDelayUs(bool dirLevel)
{
    return dirLevel == SHIFT_DIR_Z_LEVEL ? SHIFT_COMMAND_DELAY_Z_US : SHIFT_COMMAND_DELAY_C_US;
}

uint32_t getShiftStartDelayUs(bool dirLevel)
{
    return dirLevel == SHIFT_DIR_Z_LEVEL ? SHIFT_START_DELAY_Z_US : SHIFT_START_DELAY_C_US;
}

uint32_t interpolateSmoothDelayUs(
    uint32_t progressSteps,
    uint32_t rampSteps,
    uint32_t startDelayUs,
    uint32_t nominalDelayUs)
{
    if (rampSteps == 0U || progressSteps >= rampSteps || startDelayUs <= nominalDelayUs) {
        return nominalDelayUs;
    }

    const uint32_t delta = startDelayUs - nominalDelayUs;
    constexpr uint32_t kScale = 4096U;

    const uint64_t x = (static_cast<uint64_t>(progressSteps) * kScale) / rampSteps;
    const uint64_t easeScaled =
        ((3ULL * x * x * kScale) - (2ULL * x * x * x)) /
        (static_cast<uint64_t>(kScale) * kScale);

    const uint32_t dec = static_cast<uint32_t>(
        (static_cast<uint64_t>(delta) * easeScaled) / kScale);
    return startDelayUs - dec;
}

uint32_t getShiftStepDelayUs(bool dirLevel, uint32_t stepIndex, uint32_t totalPlannedSteps)
{
    const uint32_t nominalDelayUs = getShiftNominalDelayUs(dirLevel);
    const uint32_t startDelayUs = getShiftStartDelayUs(dirLevel);
    uint32_t rampSteps = (dirLevel == SHIFT_DIR_C_LEVEL) ? SHIFT_RAMP_STEPS_C : SHIFT_RAMP_STEPS_Z;
    uint32_t decelRampSteps = (dirLevel == SHIFT_DIR_C_LEVEL) ? SHIFT_DECEL_STEPS_C : SHIFT_RAMP_STEPS_Z;
    if (totalPlannedSteps != 0U) {
        const uint32_t halfSteps = totalPlannedSteps / 2U;
        if (halfSteps < rampSteps) {
            rampSteps = halfSteps;
        }
        if (halfSteps < decelRampSteps) {
            decelRampSteps = halfSteps;
        }
    }
    if (rampSteps == 0U) {
        rampSteps = 1U;
    }
    if (decelRampSteps == 0U) {
        decelRampSteps = 1U;
    }

    uint32_t accelProgress = stepIndex;
    if (accelProgress > rampSteps) {
        accelProgress = rampSteps;
    }

    const bool useSCurve = (dirLevel == SHIFT_DIR_C_LEVEL);
    uint32_t targetDelayUs = useSCurve
                                 ? interpolateSmoothDelayUs(
                                       accelProgress,
                                       rampSteps,
                                       startDelayUs,
                                       nominalDelayUs)
                                 : interpolateDelayUs(
                                       accelProgress,
                                       rampSteps,
                                       startDelayUs,
                                       nominalDelayUs);

    if (totalPlannedSteps != 0U) {
        uint32_t remainingSteps = 0U;
        if (totalPlannedSteps > stepIndex) {
            remainingSteps = totalPlannedSteps - stepIndex;
        }

        uint32_t decelProgress = remainingSteps;
        if (decelProgress > decelRampSteps) {
            decelProgress = decelRampSteps;
        }

        const uint32_t decelDelayUs = useSCurve
                                          ? interpolateSmoothDelayUs(
                                                decelProgress,
                                                decelRampSteps,
                                                startDelayUs,
                                                nominalDelayUs)
                                          : interpolateDelayUs(
                                                decelProgress,
                                                decelRampSteps,
                                                startDelayUs,
                                                nominalDelayUs);
        if (decelDelayUs > targetDelayUs) {
            targetDelayUs = decelDelayUs;
        }
    }

    if (targetDelayUs < nominalDelayUs) {
        targetDelayUs = nominalDelayUs;
    }

    return targetDelayUs;
}

bool isShiftTargetSensorTriggered(bool dirLevel)
{
    return dirLevel == SHIFT_DIR_Z_LEVEL ? isShiftSensorZTriggered() : isShiftSensorCTriggered();
}

bool isShiftOppositeSensorTriggered(bool dirLevel)
{
    return dirLevel == SHIFT_DIR_Z_LEVEL ? isShiftSensorCTriggered() : isShiftSensorZTriggered();
}

void prepareShiftMove(bool dirLevel)
{
    digitalWrite(PIN_SHIFT_DIR, dirLevel);
    writeShiftPulseInactive();
    delayMicroseconds(200);
}

void pulseShiftStep(bool dirLevel, bool allowPositionalOverlap, uint32_t stepIndex, uint32_t totalPlannedSteps)
{
    const uint32_t shiftDelayUs = getShiftStepDelayUs(dirLevel, stepIndex, totalPlannedSteps);
    digitalWrite(PIN_SHIFT_PUL, PULSE_ACTIVE_LEVEL ? HIGH : LOW);
    delayShiftUs(SHIFT_PULSE_WIDTH_US, allowPositionalOverlap);
    writeShiftPulseInactive();
    delayShiftUs(shiftDelayUs, allowPositionalOverlap);
}

bool shiftLeaveSensor(
    bool dirLevel,
    bool (*sensorFn)(),
    uint32_t maxSteps,
    uint32_t &stepsDone,
    uint32_t &profileStepIndex,
    uint32_t profileTotalSteps,
    bool allowPositionalOverlap)
{
    stepsDone = 0;
    if (!sensorFn()) {
        return true;
    }

    prepareShiftMove(dirLevel);
    for (uint32_t i = 0; i < maxSteps; i++) {
        pulseShiftStep(dirLevel, allowPositionalOverlap, profileStepIndex, profileTotalSteps);
        stepsDone++;
        profileStepIndex++;
        if (!sensorFn()) {
            return true;
        }
    }

    return false;
}

bool shiftFindSensor(
    bool dirLevel,
    bool (*sensorFn)(),
    uint32_t maxSteps,
    uint32_t &stepsDone,
    uint32_t &profileStepIndex,
    uint32_t profileTotalSteps,
    bool allowPositionalOverlap)
{
    stepsDone = 0;
    prepareShiftMove(dirLevel);
    for (uint32_t i = 0; i < maxSteps; i++) {
        pulseShiftStep(dirLevel, allowPositionalOverlap, profileStepIndex, profileTotalSteps);
        stepsDone++;
        profileStepIndex++;
        if (sensorFn()) {
            return true;
        }
    }

    return false;
}

uint32_t getShiftLeaveLimitSteps()
{
    uint32_t leaveLimit = SHIFT_LEAVE_SENSOR_MAX_STEPS;
    if (g_shiftCalibrated) {
        const uint32_t calibratedLimit = g_shiftTravelSteps + SHIFT_CAL_MOVE_MARGIN_STEPS;
        if (calibratedLimit > leaveLimit) {
            leaveLimit = calibratedLimit;
        }
    }
    return leaveLimit;
}

void calibrateShiftTravel()
{
    if (g_motion.active || g_c2Active || g_posMotion.active) {
        Serial.println("SHIFT: отказ, другой двигатель уже в движении.");
        return;
    }

    Serial.println("SHIFT: CZ калибровка старт.");

    const bool zActive = isShiftSensorZTriggered();
    const bool cActive = isShiftSensorCTriggered();
    if (zActive && cActive) {
        g_shiftCalibrated = false;
        Serial.println("SHIFT: ошибка, одновременно сработали Z и C. Проверь герконы.");
        return;
    }

    if (zActive) {
        Serial.println("SHIFT: каретка уже на краю Z.");
    } else {
        if (cActive) {
            uint32_t leaveCSteps = 0;
            uint32_t leaveCProfileStep = 0;
            if (!shiftLeaveSensor(
                    SHIFT_DIR_Z_LEVEL,
                    isShiftSensorCTriggered,
                    getShiftLeaveLimitSteps(),
                    leaveCSteps,
                    leaveCProfileStep,
                    getShiftLeaveLimitSteps(),
                    false)) {
                g_shiftCalibrated = false;
                Serial.println("SHIFT: ошибка, не удалось уйти с края C.");
                return;
            }
            Serial.println("SHIFT: ушли с края C, ищем Z.");
        } else {
            Serial.println("SHIFT: старт между краями, ищем край Z.");
        }

        uint32_t stepsToZ = 0;
        uint32_t stepsToZProfile = 0;
        if (!shiftFindSensor(
                SHIFT_DIR_Z_LEVEL,
                isShiftSensorZTriggered,
                SHIFT_CAL_SEARCH_MAX_STEPS,
                stepsToZ,
                stepsToZProfile,
                SHIFT_CAL_SEARCH_MAX_STEPS,
                false)) {
            g_shiftCalibrated = false;
            Serial.println("SHIFT: ошибка, край Z не найден.");
            return;
        }
    }

    Serial.println("SHIFT: край Z найден.");

    uint32_t leaveZSteps = 0;
    uint32_t leaveZProfileStep = 0;
    if (!shiftLeaveSensor(
            SHIFT_DIR_C_LEVEL,
            isShiftSensorZTriggered,
            getShiftLeaveLimitSteps(),
            leaveZSteps,
            leaveZProfileStep,
            getShiftLeaveLimitSteps(),
            false)) {
        g_shiftCalibrated = false;
        Serial.println("SHIFT: ошибка, не удалось уйти с края Z.");
        return;
    }

    uint32_t stepsToC = 0;
    uint32_t stepsToCProfile = 0;
    if (!shiftFindSensor(
            SHIFT_DIR_C_LEVEL,
            isShiftSensorCTriggered,
            SHIFT_CAL_SEARCH_MAX_STEPS,
            stepsToC,
            stepsToCProfile,
            SHIFT_CAL_SEARCH_MAX_STEPS,
            false)) {
        g_shiftCalibrated = false;
        Serial.println("SHIFT: ошибка, край C не найден.");
        return;
    }

    g_shiftTravelSteps = leaveZSteps + stepsToC;
    if (g_shiftTravelSteps == 0) {
        g_shiftCalibrated = false;
        Serial.println("SHIFT: ошибка, ход получился 0 шагов.");
        return;
    }

    g_shiftCalibrated = true;

    Serial.println("SHIFT: край C найден.");
    Serial.print("SHIFT: калибровка завершена. Ход=");
    Serial.print(g_shiftTravelSteps);
    Serial.println(" шагов.");
    Serial.println("SHIFT: каретка сейчас в крайнем положении C.");
}

bool runShiftMoveInternal(bool dirLevel, const char *name, bool allowPositionalOverlap)
{
    if (g_motion.active || g_c2Active || (g_posMotion.active && !allowPositionalOverlap)) {
        Serial.println("SHIFT: отказ, другой двигатель уже в движении.");
        return false;
    }

    if (!g_shiftCalibrated) {
        const uint32_t shiftNominalDelayUs = getShiftNominalDelayUs(dirLevel);
        Serial.print("SHIFT: калибровки нет, ");
        Serial.print(name);
        Serial.print(" выполнится на фиксированные ");
        Serial.print(SHIFT_COMMAND_STEPS);
        Serial.println(" шагов.");

        Serial.print("SHIFT: старт ");
        Serial.print(name);
        Serial.print(", шагов=");
        Serial.print(SHIFT_COMMAND_STEPS);
        Serial.print(", задержка=");
        Serial.print(shiftNominalDelayUs);
        Serial.println(" мкс.");

        prepareShiftMove(dirLevel);
        uint32_t profileStepIndex = 0;
        for (uint32_t i = 0; i < SHIFT_COMMAND_STEPS; i++) {
            pulseShiftStep(dirLevel, allowPositionalOverlap, profileStepIndex, SHIFT_COMMAND_STEPS);
            profileStepIndex++;
        }

        Serial.print("SHIFT: выполнено ");
        Serial.print(name);
        Serial.print(", шагов=");
        Serial.print(SHIFT_COMMAND_STEPS);
        Serial.print(", задержка=");
        Serial.print(shiftNominalDelayUs);
        Serial.println(" мкс.");
        return true;
    }

    const bool targetActive = isShiftTargetSensorTriggered(dirLevel);
    const bool oppositeActive = isShiftOppositeSensorTriggered(dirLevel);
    if (targetActive && oppositeActive) {
        Serial.println("SHIFT: ошибка, одновременно сработали оба геркона.");
        return false;
    }
    if (targetActive) {
        Serial.print("SHIFT: ");
        Serial.print(name);
        Serial.println(" не требуется, каретка уже в нужном крайнем положении.");
        return true;
    }

    const uint32_t maxSeekSteps = g_shiftTravelSteps + SHIFT_CAL_MOVE_MARGIN_STEPS;
    const uint32_t shiftNominalDelayUs = getShiftNominalDelayUs(dirLevel);
    uint32_t totalSteps = 0;

    Serial.print("SHIFT: старт ");
    Serial.print(name);
    Serial.print(", откалиброванный ход=");
    Serial.print(g_shiftTravelSteps);
    Serial.print(" шагов, задержка=");
    Serial.print(shiftNominalDelayUs);
    Serial.println(" мкс.");

    if (oppositeActive) {
        uint32_t leaveSteps = 0;
        const uint32_t leaveLimitSteps = getShiftLeaveLimitSteps();
        uint32_t profileStepIndex = totalSteps;
        if (!shiftLeaveSensor(dirLevel, dirLevel == SHIFT_DIR_Z_LEVEL ? isShiftSensorCTriggered : isShiftSensorZTriggered,
                leaveLimitSteps, leaveSteps, profileStepIndex, maxSeekSteps, allowPositionalOverlap)) {
            Serial.print("SHIFT: не удалось уйти с противоположного края. Лимит=");
            Serial.print(leaveLimitSteps);
            Serial.println(" шагов.");
            return false;
        }
        totalSteps += leaveSteps;
    }

    uint32_t seekSteps = 0;
    uint32_t profileStepIndex = totalSteps;
    const bool reachedEdge = shiftFindSensor(
        dirLevel,
        dirLevel == SHIFT_DIR_Z_LEVEL ? isShiftSensorZTriggered : isShiftSensorCTriggered,
        maxSeekSteps,
        seekSteps,
        profileStepIndex,
        maxSeekSteps,
        allowPositionalOverlap);
    totalSteps += seekSteps;

    if (reachedEdge) {
        Serial.print("SHIFT: выполнено ");
        Serial.print(name);
        Serial.print(", дошли до края, шагов=");
        Serial.print(totalSteps);
        Serial.println(".");
    } else {
        Serial.print("SHIFT: край не найден, остановка по лимиту. Шагов=");
        Serial.print(totalSteps);
        Serial.print(", лимит=");
        Serial.print(maxSeekSteps);
        Serial.println(".");
    }

    return reachedEdge;
}

void runShiftMove(bool dirLevel, const char *name)
{
    (void)runShiftMoveInternal(dirLevel, name, false);
}

void resetProgram1Metrics()
{
    for (uint8_t i = 0; i < 12; i++) {
        g_program1Metrics[i] = Program1Metric();
    }
    g_program1MetricCount = 0;
    g_program1ActiveC2Metric = -1;
    g_program1ActivePosMetric = -1;
}

void printProgram1MetricLabel(const Program1Metric &metric)
{
    switch (metric.kind) {
        case Program1MetricKind::Cycle2:
            Serial.print("2 #");
            break;
        case Program1MetricKind::ShiftC:
            Serial.print("C #");
            break;
        case Program1MetricKind::Pos3:
            Serial.print("3 #");
            break;
        case Program1MetricKind::ShiftZ:
            Serial.print("Z #");
            break;
        case Program1MetricKind::BufferFill:
            Serial.print("BUF #");
            break;
        default:
            Serial.print("? #");
            break;
    }
    Serial.print(metric.passIndex);
}

int8_t beginProgram1Metric(Program1MetricKind kind, uint8_t passIndex)
{
    if (g_program1MetricCount >= 12) {
        Serial.println("P1: overflow metrics.");
        return -1;
    }

    Program1Metric &metric = g_program1Metrics[g_program1MetricCount];
    metric.kind = kind;
    metric.passIndex = passIndex;
    metric.startMs = millis();
    metric.durationMs = 0;
    metric.active = true;
    metric.finished = false;

    const int8_t metricId = static_cast<int8_t>(g_program1MetricCount);
    g_program1MetricCount++;
    return metricId;
}

void finishProgram1Metric(int8_t metricId)
{
    if (metricId < 0 || metricId >= static_cast<int8_t>(g_program1MetricCount)) {
        return;
    }

    Program1Metric &metric = g_program1Metrics[metricId];
    if (!metric.active || metric.finished) {
        return;
    }

    metric.durationMs = millis() - metric.startMs;
    metric.active = false;
    metric.finished = true;
}

void updateProgram1AsyncMetrics()
{
    if (g_program1ActiveC2Metric >= 0 && !g_c2Active) {
        finishProgram1Metric(g_program1ActiveC2Metric);
        g_program1ActiveC2Metric = -1;
    }

    if (g_program1ActivePosMetric >= 0 && !g_posMotion.active) {
        finishProgram1Metric(g_program1ActivePosMetric);
        g_program1ActivePosMetric = -1;
    }
}

void printProgram1Summary()
{
    uint32_t sum2Ms = 0;
    uint32_t sumCMs = 0;
    uint32_t sum3Ms = 0;
    uint32_t sumZMs = 0;
    uint32_t sumBufMs = 0;

    Serial.println("P1: ===== Summary =====");
    for (uint8_t i = 0; i < g_program1MetricCount; i++) {
        const Program1Metric &metric = g_program1Metrics[i];
        Serial.print("P1: ");
        printProgram1MetricLabel(metric);
        Serial.print(" | ");
        if (!metric.finished) {
            Serial.println("RUNNING");
            continue;
        }

        Serial.print(metric.durationMs);
        Serial.println(" мс");

        switch (metric.kind) {
            case Program1MetricKind::Cycle2:
                sum2Ms += metric.durationMs;
                break;
            case Program1MetricKind::ShiftC:
                sumCMs += metric.durationMs;
                break;
            case Program1MetricKind::Pos3:
                sum3Ms += metric.durationMs;
                break;
            case Program1MetricKind::ShiftZ:
                sumZMs += metric.durationMs;
                break;
            case Program1MetricKind::BufferFill:
                sumBufMs += metric.durationMs;
                break;
            default:
                break;
        }
    }

    const uint32_t totalProgramMs = millis() - g_program1StartMs;
    Serial.print("P1: SUM 2 | ");
    Serial.print(sum2Ms);
    Serial.println(" мс");
    Serial.print("P1: SUM C | ");
    Serial.print(sumCMs);
    Serial.println(" мс");
    Serial.print("P1: SUM 3 | ");
    Serial.print(sum3Ms);
    Serial.println(" мс");
    Serial.print("P1: SUM Z | ");
    Serial.print(sumZMs);
    Serial.println(" мс");
    Serial.print("P1: SUM BUF | ");
    Serial.print(sumBufMs);
    Serial.println(" мс");
    Serial.print("P1: TOTAL | ");
    Serial.print(totalProgramMs);
    Serial.print(" мс (");
    Serial.print(static_cast<float>(totalProgramMs) / 1000.0F, 1);
    Serial.println(" с)");
}

void startProgram1Cycle2Metric(Program1MetricKind kind, uint8_t passIndex)
{
    const int8_t metricId = beginProgram1Metric(kind, passIndex);
    startCycle2();
    if (g_c2Active && metricId >= 0) {
        g_program1ActiveC2Metric = metricId;
    } else {
        finishProgram1Metric(metricId);
    }
}

void startProgram1Cycle2(uint8_t passIndex)
{
    startProgram1Cycle2Metric(Program1MetricKind::Cycle2, passIndex);
}

void startProgram1Pos3(uint8_t passIndex)
{
    if (g_posMotion.active) {
        Serial.print("P1: 3 #");
        Serial.print(passIndex);
        Serial.println(" не запущена, позиционный мотор уже в движении.");
        return;
    }

    const int8_t metricId = beginProgram1Metric(Program1MetricKind::Pos3, passIndex);
    startPositionalProfiledMotion(C3_COMMAND_STEPS, C3_COMMAND_DELAY_US);
    if (g_posMotion.active && metricId >= 0) {
        g_program1ActivePosMetric = metricId;
    } else {
        finishProgram1Metric(metricId);
    }
}

bool runProgram1ShiftStage(bool dirLevel, Program1MetricKind kind, uint8_t passIndex, const char *name)
{
    const int8_t metricId = beginProgram1Metric(kind, passIndex);
    const bool ok = runShiftMoveInternal(dirLevel, name, true);
    finishProgram1Metric(metricId);
    return ok;
}

void initProgram1Storage()
{
    g_program1StorageReady = g_preferences.begin("p1buf", false);
    if (!g_program1StorageReady) {
        Serial.println("P1: EEPROM storage unavailable, buffer flag reset.");
        g_program1BufferReady = false;
        return;
    }

    g_program1BufferReady = g_preferences.getBool("ready", false);
}

void setProgram1BufferReady(bool ready)
{
    g_program1BufferReady = ready;
    if (!g_program1StorageReady) {
        return;
    }
    g_preferences.putBool("ready", ready);
}

void homeShiftToZOnStartup()
{
    if (g_motion.active || g_c2Active || g_posMotion.active) {
        Serial.println("SHIFT: стартовая привязка пропущена, система занята.");
        return;
    }

    g_shiftCalibrated = true;
    g_shiftTravelSteps = SHIFT_TRAVEL_STEPS_FIXED;

    const bool zActive = isShiftSensorZTriggered();
    const bool cActive = isShiftSensorCTriggered();
    if (zActive && cActive) {
        Serial.println("SHIFT: стартовая привязка невозможна, одновременно активны Z и C.");
        return;
    }

    const uint32_t homeLimitSteps = getShiftLeaveLimitSteps();

    if (zActive) {
        Serial.println("SHIFT: стартовая привязка, каретка уже на Z. Отходим и заново ловим Z.");

        uint32_t leaveSteps = 0;
        uint32_t leaveProfileStep = 0;
        if (!shiftLeaveSensor(
                SHIFT_DIR_C_LEVEL,
                isShiftSensorZTriggered,
                homeLimitSteps,
                leaveSteps,
                leaveProfileStep,
                homeLimitSteps,
                false)) {
            Serial.println("SHIFT: ошибка стартовой привязки, не удалось уйти с Z.");
            return;
        }
    } else {
        Serial.println("SHIFT: стартовая привязка, идем к Z.");
    }

    uint32_t seekSteps = 0;
    uint32_t seekProfileStep = 0;
    if (!shiftFindSensor(
            SHIFT_DIR_Z_LEVEL,
            isShiftSensorZTriggered,
            homeLimitSteps,
            seekSteps,
            seekProfileStep,
            homeLimitSteps,
            false)) {
        Serial.println("SHIFT: ошибка стартовой привязки, Z не найден.");
        return;
    }

    Serial.print("SHIFT: стартовая привязка завершена. Z найден, ход=");
    Serial.print(g_shiftTravelSteps);
    Serial.println(" шагов.");
}

void startProgram1()
{
    if (g_program1State != Program1State::Idle) {
        Serial.println("P1: программа 1 уже выполняется.");
        return;
    }

    if (!g_shiftCalibrated) {
        Serial.println("P1: сначала выполни CZ, чтобы откалибровать сдвиг.");
        return;
    }

    if (g_motion.active || g_c2Active || g_posMotion.active) {
        Serial.println("P1: отказ, система уже выполняет движение.");
        return;
    }

    Serial.println("P1: старт программы 1.");
    g_program1StartMs = millis();
    resetProgram1Metrics();

    if (g_program1BufferReady) {
        Serial.println("P1: буфер с 2 тарелками найден, начинаем сразу с C #1.");
        setProgram1BufferReady(false);
        g_program1State = Program1State::StartBufferedPass1;
        return;
    }

    Serial.println("P1: шаг 1/3 -> команда 2.");
    startProgram1Cycle2(1);
    g_program1State = Program1State::WaitFirstC2Done;
}

void processProgram1()
{
    updateProgram1AsyncMetrics();

    if (g_program1State == Program1State::Idle) {
        return;
    }

    if (g_c2Active) {
        return;
    }

    if (g_motion.active) {
        return;
    }

    switch (g_program1State) {
        case Program1State::StartBufferedPass1:
            Serial.println("P1: буферный старт. Выполняем C, затем 3 + Z.");
            if (!runProgram1ShiftStage(SHIFT_DIR_C_LEVEL, Program1MetricKind::ShiftC, 1, "C (сдвиг)")) {
                Serial.println("P1: аварийная остановка на шаге C.");
                g_program1State = Program1State::Idle;
                return;
            }
            startProgram1Pos3(1);
            if (!runProgram1ShiftStage(SHIFT_DIR_Z_LEVEL, Program1MetricKind::ShiftZ, 1, "Z (возврат)")) {
                Serial.println("P1: аварийная остановка на шаге Z.");
                g_program1State = Program1State::Idle;
                return;
            }
            Serial.println("P1: шаг 2/3 -> команда 2.");
            startProgram1Cycle2(2);
            g_program1State = Program1State::WaitSecondC2Done;
            return;

        case Program1State::WaitFirstC2Done:
            Serial.println("P1: 1-й проход 2 завершен. Выполняем C, затем 3 + Z.");
            if (!runProgram1ShiftStage(SHIFT_DIR_C_LEVEL, Program1MetricKind::ShiftC, 1, "C (сдвиг)")) {
                Serial.println("P1: аварийная остановка на шаге C.");
                g_program1State = Program1State::Idle;
                return;
            }
            startProgram1Pos3(1);
            if (!runProgram1ShiftStage(SHIFT_DIR_Z_LEVEL, Program1MetricKind::ShiftZ, 1, "Z (возврат)")) {
                Serial.println("P1: аварийная остановка на шаге Z.");
                g_program1State = Program1State::Idle;
                return;
            }
            Serial.println("P1: шаг 2/3 -> команда 2.");
            startProgram1Cycle2(2);
            g_program1State = Program1State::WaitSecondC2Done;
            return;

        case Program1State::WaitSecondC2Done:
            Serial.println("P1: 2-й проход 2 завершен. Выполняем C, затем 3 + Z.");
            if (!runProgram1ShiftStage(SHIFT_DIR_C_LEVEL, Program1MetricKind::ShiftC, 2, "C (сдвиг)")) {
                Serial.println("P1: аварийная остановка на шаге C.");
                g_program1State = Program1State::Idle;
                return;
            }
            startProgram1Pos3(2);
            if (!runProgram1ShiftStage(SHIFT_DIR_Z_LEVEL, Program1MetricKind::ShiftZ, 2, "Z (возврат)")) {
                Serial.println("P1: аварийная остановка на шаге Z.");
                g_program1State = Program1State::Idle;
                return;
            }
            Serial.println("P1: шаг 3/3 -> команда 2.");
            startProgram1Cycle2(3);
            g_program1State = Program1State::WaitThirdC2Done;
            return;

        case Program1State::WaitThirdC2Done:
            Serial.println("P1: 3-й проход 2 завершен. Финальный C + Z.");
            if (!runProgram1ShiftStage(SHIFT_DIR_C_LEVEL, Program1MetricKind::ShiftC, 3, "C (сдвиг)")) {
                Serial.println("P1: аварийная остановка на шаге C.");
                g_program1State = Program1State::Idle;
                return;
            }
            if (!runProgram1ShiftStage(SHIFT_DIR_Z_LEVEL, Program1MetricKind::ShiftZ, 3, "Z (возврат)")) {
                Serial.println("P1: аварийная остановка на шаге Z.");
                g_program1State = Program1State::Idle;
                return;
            }
            if (g_program1ActivePosMetric >= 0 || g_posMotion.active) {
                Serial.println("P1: ждем завершение последнего этапа 3 перед наполнением буфера.");
                g_program1State = Program1State::WaitFinalPositionalDone;
                return;
            }
            Serial.println("P1: запускаем дополнительный 2 для наполнения буфера.");
            startProgram1Cycle2Metric(Program1MetricKind::BufferFill, 1);
            if (!g_c2Active) {
                Serial.println("P1: ошибка, не удалось запустить буферный 2.");
                g_program1State = Program1State::Idle;
                return;
            }
            g_program1State = Program1State::WaitBufferFillC2Done;
            return;

        case Program1State::WaitFinalPositionalDone:
            if (g_program1ActivePosMetric >= 0 || g_posMotion.active) {
                return;
            }
            Serial.println("P1: запускаем дополнительный 2 для наполнения буфера.");
            startProgram1Cycle2Metric(Program1MetricKind::BufferFill, 1);
            if (!g_c2Active) {
                Serial.println("P1: ошибка, не удалось запустить буферный 2.");
                g_program1State = Program1State::Idle;
                return;
            }
            g_program1State = Program1State::WaitBufferFillC2Done;
            return;

        case Program1State::WaitBufferFillC2Done:
            setProgram1BufferReady(true);
            Serial.println("P1: программа 1 завершена.");
            Serial.println("P1: буфер пополнен, в памяти отмечено: тарелки есть.");
            printProgram1Summary();
            g_program1State = Program1State::Idle;
            return;

        case Program1State::Idle:
        default:
            return;
    }
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

bool readShiftSensorZRaw()
{
    return digitalRead(PIN_SHIFT_SENSOR_Z) == HIGH;
}

bool readShiftSensorCRaw()
{
    return digitalRead(PIN_SHIFT_SENSOR_C) == HIGH;
}

void updateSensorFilter()
{
    updateDebouncedSensorFilter(g_sensorFilter, readSensorPlateRaw(), SENSOR_DEBOUNCE_MS);
}

void updateShiftSensorFilters()
{
    updateDebouncedSensorFilter(
        g_shiftSensorZFilter,
        readShiftSensorZRaw(),
        SHIFT_SENSOR_DEBOUNCE_MS);
    updateDebouncedSensorFilter(
        g_shiftSensorCFilter,
        readShiftSensorCRaw(),
        SHIFT_SENSOR_DEBOUNCE_MS);
}

bool isPlateAtSensorFiltered()
{
    return g_sensorFilter.stablePlateDetected;
}

bool isShiftSensorZTriggered()
{
    updateShiftSensorFilters();
    return g_shiftSensorZFilter.stablePlateDetected;
}

bool isShiftSensorCTriggered()
{
    updateShiftSensorFilters();
    return g_shiftSensorCFilter.stablePlateDetected;
}

void printSensorState()
{
    const bool isPlateAtSensor = isPlateAtSensorFiltered();
    Serial.print("Датчик E18-D50NK: ");
    Serial.println(isPlateAtSensor ? "тарелка на флаге (HIGH)" : "тарелки нет (LOW)");

    Serial.print("Геркон Z GPIO33: ");
    Serial.println(isShiftSensorZTriggered() ? "замкнут (HIGH)" : "разомкнут (LOW)");
    Serial.print("Геркон C GPIO25: ");
    Serial.println(isShiftSensorCTriggered() ? "замкнут (HIGH)" : "разомкнут (LOW)");
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

    if (cmd == "1") {
        startProgram1();
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

    if (cmd == "CZ") {
        calibrateShiftTravel();
        return;
    }

    if (cmd == "C") {
        runShiftMove(SHIFT_DIR_C_LEVEL, "C (сдвиг)");
        return;
    }

    if (cmd == "Z") {
        runShiftMove(SHIFT_DIR_Z_LEVEL, "Z (возврат)");
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
        if (ch == '\r' || ch == '\n') {
            if (!g_cmdBuffer.isEmpty()) {
                handleCommand(g_cmdBuffer);
                g_cmdBuffer = "";
            }
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
    pinMode(PIN_SHIFT_DIR, OUTPUT);
    pinMode(PIN_SHIFT_PUL, OUTPUT);
    pinMode(PIN_SHIFT_SENSOR_Z, INPUT_PULLDOWN);
    pinMode(PIN_SHIFT_SENSOR_C, INPUT_PULLDOWN);
    pinMode(PIN_FLAG, OUTPUT);
    pinMode(PIN_SENSOR, INPUT_PULLUP);
    pinMode(PIN_POS_PUL, OUTPUT);

    writePulseInactive();
    digitalWrite(PIN_SHIFT_DIR, SHIFT_DIR_C_LEVEL);
    writeShiftPulseInactive();
    digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
    writePosPulseInactive();
    initProgram1Storage();
    updateSensorFilter();
    updateShiftSensorFilters();
    homeShiftToZOnStartup();
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
    Serial.println("Запущено два конвейера + сдвиг тарелки.");
    Serial.println("Сдвиг тарелки: DIR=GPIO14, PUL=GPIO12, Z=GPIO33, C=GPIO25.");
    Serial.println("Сдвиг по умолчанию: фиксированный ход 2255 шагов.");
    Serial.print("P1 buffer: ");
    Serial.println(g_program1BufferReady ? "есть 2 тарелки." : "пусто.");
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
    updateShiftSensorFilters();
    readSerialCommands();

    if (!g_posMotion.active) {
        WifiConsole::process();
        MqttLink::process();
    }

    processCycle2();
    processMotion();
    processPositionalMotion();
    processProgram1();
    processSensorStream();
}
