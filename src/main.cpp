#include <Arduino.h>

#include "wifi_console.h"
#include "mqtt_link.h"

constexpr char FW_VERSION[] = "v1.6";

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
constexpr uint32_t MANUAL_MOVE_DELAY_US = 1500; // Фиксированная задержка для команд A/D
constexpr uint32_t MOTION_START_DELAY_MULT_NUM = 2;
constexpr uint32_t MOTION_START_DELAY_MULT_DEN = 1;
constexpr uint32_t MOTION_RAMP_STEPS_DEFAULT = 150;
constexpr uint32_t PULSES_PER_MM = 5;           // 2000 импульсов = 400 мм => 5 имп/мм
constexpr uint32_t BC_MOVE_DELAY_US = 1500;
constexpr uint32_t BC_CENTER_DISTANCE_MM = 20;
constexpr uint32_t BC_CENTER_STEPS = BC_CENTER_DISTANCE_MM * PULSES_PER_MM;
constexpr uint8_t BC_BATCH_PLATES_DEFAULT = 4;
constexpr uint8_t BC_BATCH_PLATES_MAX = 20;

constexpr uint32_t C3_MOVE_DELAY_US = 1500;
constexpr uint32_t C3_CENTER_STEPS = 20U * PULSES_PER_MM; // 20 мм
constexpr uint32_t C3_FLAG_REOPEN_STEPS = 80U * PULSES_PER_MM; // 80 мм
constexpr uint32_t C3_WAIT_AFTER_FIRST_LEAVE_STEPS = (150U + 35U) * PULSES_PER_MM; // 185 мм
constexpr uint32_t C3_FINAL_AFTER_SECOND_LEAVE_STEPS = (150U - 35U) * PULSES_PER_MM; // 115 мм

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
constexpr int32_t C2_TARGET_GAP_TENTHS_MM = static_cast<int32_t>(C2_TARGET_GAP_MM * 10U);
constexpr int32_t C2_CAL_SLOPE_SCALE = 10;
constexpr uint32_t C2_CAL_MIN_UT_DIFF_FOR_SLOPE_STEPS = 20U; // 4 мм
constexpr int32_t C2_CAL_SLOPE_NUM_MIN = -20; // -2.0 шаг/UT
constexpr int32_t C2_CAL_SLOPE_NUM_MAX = 20;  // +2.0 шаг/UT

enum class C2CalStage : uint8_t;

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

struct SensorFilterState {
    bool initialized = false;
    bool lastRawPlateDetected = false;
    bool stablePlateDetected = false;
    uint32_t lastRawChangeMs = 0;
};

struct C2CalibrationState {
    bool active = false;
    C2CalStage stage = static_cast<C2CalStage>(0);
    uint32_t ut35Steps = 0;
    uint32_t final35Steps = 0;
    int32_t gap35TenthsMm = 0;
    uint32_t ut55Steps = 0;
    uint32_t final55Steps = 0;
    int32_t gap55TenthsMm = 0;
};

enum class BCState : uint8_t {
    Idle,
    MovingToSensor,
    CenteringAfterDetect,
    WaitPlateLeave
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

enum class C2CalStage : uint8_t {
    Idle = 0,
    WaitReady35,
    Running35,
    WaitGap35,
    WaitReady55,
    Running55,
    WaitGap55
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
C2CalibrationState g_c2Cal;

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
void stopMotion();
void startCycle2();
bool handleC2CalibrationInput(String line);
void printC2AdaptiveParams();

void printHelp()
{
    Serial.println("Команды:");
    Serial.println("  D 200        - вправо, 200 мм (задержка 1500 мкс)");
    Serial.println("  A 200        - влево, 200 мм (задержка 1500 мкс)");
    Serial.println("  BC 2         - тест N тарелок: для каждой +20 мм, замер после опускания флага, в конце таблица и среднее");
    Serial.println("  2            - рабочий ход: 1-я тарелка в упоре, 2-я с зазором 34 мм");
    Serial.println("  2Q           - опрос-калибровка C2 (35 мм -> ввод зазора -> 55 мм -> ввод зазора)");
    Serial.println("  2P           - показать текущие параметры C2");
    Serial.println("  2R           - сбросить параметры C2 к базовым");
    Serial.println("  3            - рабочий цикл: 2 тарелки с UT и таймингами 80/185/115 мм");
    Serial.println("  4            - рабочий цикл как 3, но без вывода сообщений в монитор");
    Serial.println("  W            - флаг вверх");
    Serial.println("  S            - флаг вниз");
    Serial.println("  E            - вкл/выкл поток датчика (каждые 0.5 сек)");
    WifiConsole::printHelp();
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

void startMotionProfiled(bool dirLevel, uint32_t stepsTotal, uint32_t nominalDelayUs)
{
    if (nominalDelayUs == 0) {
        nominalDelayUs = 1;
    }

    digitalWrite(PIN_STEP_ENA, ENA_ACTIVE_LEVEL);
    digitalWrite(PIN_STEP_DIR, dirLevel);

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

int32_t divRoundNearest(int64_t numerator, int64_t denominator)
{
    if (denominator == 0) {
        return 0;
    }

    const bool negativeResult = (numerator < 0) != (denominator < 0);
    uint64_t absNumerator = numerator < 0 ? static_cast<uint64_t>(-numerator) : static_cast<uint64_t>(numerator);
    uint64_t absDenominator = denominator < 0 ? static_cast<uint64_t>(-denominator) : static_cast<uint64_t>(denominator);
    const uint64_t rounded = (absNumerator + (absDenominator / 2U)) / absDenominator;
    return negativeResult ? -static_cast<int32_t>(rounded) : static_cast<int32_t>(rounded);
}

bool parseMillimetersTenths(String value, int32_t &tenthsMm)
{
    value.trim();
    if (value.isEmpty()) {
        return false;
    }

    bool seenSeparator = false;
    bool hasDigit = false;
    int32_t integerPart = 0;
    int32_t fracFirst = 0;
    int32_t fracSecond = 0;
    uint8_t fracDigits = 0;

    for (size_t i = 0; i < value.length(); i++) {
        const char c = value[i];
        if (c == ' ' || c == '\t') {
            continue;
        }
        if (c == ',' || c == '.') {
            if (seenSeparator) {
                return false;
            }
            seenSeparator = true;
            continue;
        }

        if (c < '0' || c > '9') {
            return false;
        }

        hasDigit = true;
        const int32_t digit = static_cast<int32_t>(c - '0');
        if (!seenSeparator) {
            if (integerPart > (INT32_MAX - digit) / 10) {
                return false;
            }
            integerPart = integerPart * 10 + digit;
        } else {
            if (fracDigits == 0) {
                fracFirst = digit;
            } else if (fracDigits == 1) {
                fracSecond = digit;
            }
            fracDigits++;
        }
    }

    if (!hasDigit) {
        return false;
    }

    int32_t decimalTenths = fracFirst;
    if (fracDigits > 1 && fracSecond >= 5) {
        decimalTenths++;
        if (decimalTenths >= 10) {
            decimalTenths = 0;
            integerPart++;
        }
    }

    tenthsMm = integerPart * 10 + decimalTenths;
    return true;
}

void printC2AdaptiveParams()
{
    Serial.print("C2 params: base=");
    Serial.print(static_cast<float>(g_c2FinalBaseSteps) / static_cast<float>(PULSES_PER_MM), 1);
    Serial.print(" мм, slope=");
    Serial.print(static_cast<float>(g_c2FinalSlopeNum) / static_cast<float>(g_c2FinalSlopeDen), 3);
    Serial.println(" шаг/UT");
}

void resetC2AdaptiveParams()
{
    g_c2FinalBaseSteps = static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_BASE_STEPS);
    g_c2FinalSlopeNum = C2_FINAL_UT_SLOPE_NUM;
    g_c2FinalSlopeDen = C2_FINAL_UT_SLOPE_DEN;
    Serial.println("C2: параметры сброшены к базовым.");
    printC2AdaptiveParams();
}

void startC2CalibrationSurvey()
{
    if (g_motion.active || g_bcBatchActive || g_c2Active || g_c3Active) {
        Serial.println("2Q: нельзя стартовать во время движения.");
        return;
    }

    g_c2Cal.active = true;
    g_c2Cal.stage = C2CalStage::WaitReady35;
    g_c2Cal.ut35Steps = 0;
    g_c2Cal.final35Steps = 0;
    g_c2Cal.gap35TenthsMm = 0;
    g_c2Cal.ut55Steps = 0;
    g_c2Cal.final55Steps = 0;
    g_c2Cal.gap55TenthsMm = 0;

    Serial.println("2Q: этап 1/2. Поставьте тарелки 35 мм и отправьте OK.");
    Serial.println("2Q: для отмены в любой момент отправьте CANCEL.");
}

void applyC2CalibrationFromSurvey()
{
    const int32_t corr35Steps = divRoundNearest(
        static_cast<int64_t>(g_c2Cal.gap35TenthsMm - C2_TARGET_GAP_TENTHS_MM) * static_cast<int64_t>(PULSES_PER_MM), 10);
    const int32_t corr55Steps = divRoundNearest(
        static_cast<int64_t>(g_c2Cal.gap55TenthsMm - C2_TARGET_GAP_TENTHS_MM) * static_cast<int64_t>(PULSES_PER_MM), 10);

    int32_t required35Steps = static_cast<int32_t>(g_c2Cal.final35Steps) + corr35Steps;
    int32_t required55Steps = static_cast<int32_t>(g_c2Cal.final55Steps) + corr55Steps;

    if (required35Steps < static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MIN_STEPS)) {
        required35Steps = static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MIN_STEPS);
    }
    if (required35Steps > static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MAX_STEPS)) {
        required35Steps = static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MAX_STEPS);
    }
    if (required55Steps < static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MIN_STEPS)) {
        required55Steps = static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MIN_STEPS);
    }
    if (required55Steps > static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MAX_STEPS)) {
        required55Steps = static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MAX_STEPS);
    }

    const int32_t ut35Offset = static_cast<int32_t>(g_c2Cal.ut35Steps) - static_cast<int32_t>(C2_UT_REFERENCE_STEPS);
    const int32_t ut55Offset = static_cast<int32_t>(g_c2Cal.ut55Steps) - static_cast<int32_t>(C2_UT_REFERENCE_STEPS);
    const int32_t utDiff = static_cast<int32_t>(g_c2Cal.ut55Steps) - static_cast<int32_t>(g_c2Cal.ut35Steps);

    int32_t slopeNum = 0;
    if (utDiff != 0 &&
        static_cast<uint32_t>(abs(utDiff)) >= C2_CAL_MIN_UT_DIFF_FOR_SLOPE_STEPS) {
        slopeNum = divRoundNearest(
            static_cast<int64_t>(required55Steps - required35Steps) * static_cast<int64_t>(C2_CAL_SLOPE_SCALE),
            static_cast<int64_t>(utDiff));
    }

    if (slopeNum < C2_CAL_SLOPE_NUM_MIN) {
        slopeNum = C2_CAL_SLOPE_NUM_MIN;
    }
    if (slopeNum > C2_CAL_SLOPE_NUM_MAX) {
        slopeNum = C2_CAL_SLOPE_NUM_MAX;
    }

    const int32_t baseFrom35 = required35Steps -
                               divRoundNearest(static_cast<int64_t>(ut35Offset) * static_cast<int64_t>(slopeNum),
                                               static_cast<int64_t>(C2_CAL_SLOPE_SCALE));
    const int32_t baseFrom55 = required55Steps -
                               divRoundNearest(static_cast<int64_t>(ut55Offset) * static_cast<int64_t>(slopeNum),
                                               static_cast<int64_t>(C2_CAL_SLOPE_SCALE));
    int32_t baseSteps = divRoundNearest(static_cast<int64_t>(baseFrom35 + baseFrom55), 2);

    if (baseSteps < static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MIN_STEPS)) {
        baseSteps = static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MIN_STEPS);
    }
    if (baseSteps > static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MAX_STEPS)) {
        baseSteps = static_cast<int32_t>(C2_FINAL_AFTER_SECOND_LEAVE_MAX_STEPS);
    }

    g_c2FinalBaseSteps = baseSteps;
    g_c2FinalSlopeNum = slopeNum;
    g_c2FinalSlopeDen = C2_CAL_SLOPE_SCALE;

    Serial.println("2Q: параметры C2 пересчитаны.");
    Serial.print("2Q: 35 мм -> зазор ");
    Serial.print(static_cast<float>(g_c2Cal.gap35TenthsMm) / 10.0F, 1);
    Serial.print(" мм, 55 мм -> зазор ");
    Serial.print(static_cast<float>(g_c2Cal.gap55TenthsMm) / 10.0F, 1);
    Serial.println(" мм.");
    Serial.print("2Q: UT35=");
    Serial.print(g_c2Cal.ut35Steps);
    Serial.print(", UT55=");
    Serial.print(g_c2Cal.ut55Steps);
    Serial.print(", dUT=");
    Serial.print(utDiff);
    Serial.println(" шагов.");
    printC2AdaptiveParams();
}

bool handleC2CalibrationInput(String line)
{
    if (!g_c2Cal.active) {
        return false;
    }

    line.trim();
    if (line.isEmpty()) {
        return true;
    }

    String token = line;
    token.toUpperCase();

    if (token == "CANCEL" || token == "STOP" || token == "EXIT") {
        g_c2Cal.active = false;
        g_c2Cal.stage = C2CalStage::Idle;
        Serial.println("2Q: опрос отменен.");
        return true;
    }

    if (g_c2Cal.stage == C2CalStage::WaitReady35) {
        if (token == "OK") {
            startCycle2();
            if (g_c2Active) {
                g_c2Cal.stage = C2CalStage::Running35;
                Serial.println("2Q: запущен цикл для 35 мм.");
            }
        } else {
            Serial.println("2Q: поставьте тарелки 35 мм и отправьте OK.");
        }
        return true;
    }

    if (g_c2Cal.stage == C2CalStage::WaitGap35) {
        int32_t gapTenthsMm = 0;
        if (!parseMillimetersTenths(line, gapTenthsMm)) {
            Serial.println("2Q: формат расстояния неверный. Пример: 32.3");
            return true;
        }
        g_c2Cal.gap35TenthsMm = gapTenthsMm;
        g_c2Cal.stage = C2CalStage::WaitReady55;
        Serial.println("2Q: этап 2/2. Поставьте тарелки 55 мм и отправьте OK.");
        return true;
    }

    if (g_c2Cal.stage == C2CalStage::WaitReady55) {
        if (token == "OK") {
            startCycle2();
            if (g_c2Active) {
                g_c2Cal.stage = C2CalStage::Running55;
                Serial.println("2Q: запущен цикл для 55 мм.");
            }
        } else {
            Serial.println("2Q: поставьте тарелки 55 мм и отправьте OK.");
        }
        return true;
    }

    if (g_c2Cal.stage == C2CalStage::WaitGap55) {
        int32_t gapTenthsMm = 0;
        if (!parseMillimetersTenths(line, gapTenthsMm)) {
            Serial.println("2Q: формат расстояния неверный. Пример: 28.9");
            return true;
        }
        g_c2Cal.gap55TenthsMm = gapTenthsMm;
        applyC2CalibrationFromSurvey();
        g_c2Cal.active = false;
        g_c2Cal.stage = C2CalStage::Idle;
        return true;
    }

    if (g_c2Cal.stage == C2CalStage::Running35 || g_c2Cal.stage == C2CalStage::Running55) {
        Serial.println("2Q: цикл в процессе, дождитесь завершения.");
        return true;
    }

    return true;
}

void handleC2CalibrationAfterCycle()
{
    if (!g_c2Cal.active) {
        return;
    }

    if (g_c2Cal.stage == C2CalStage::Running35) {
        g_c2Cal.ut35Steps = g_c2FirstUTSteps;
        g_c2Cal.final35Steps = g_c2FinalMoveSteps;
        g_c2Cal.stage = C2CalStage::WaitGap35;
        Serial.print("2Q: цикл 35 мм завершен. Введите измеренный зазор (мм), например ");
        Serial.println("32.3");
        return;
    }

    if (g_c2Cal.stage == C2CalStage::Running55) {
        g_c2Cal.ut55Steps = g_c2FirstUTSteps;
        g_c2Cal.final55Steps = g_c2FinalMoveSteps;
        g_c2Cal.stage = C2CalStage::WaitGap55;
        Serial.print("2Q: цикл 55 мм завершен. Введите измеренный зазор (мм), например ");
        Serial.println("28.9");
    }
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

    startMotionProfiled(dirLevel, steps, delayUs);

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
    startMotionProfiled(DIR_RIGHT_LEVEL, UINT32_MAX, BC_MOVE_DELAY_US);

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

void startCycle2()
{
    if (g_motion.active || g_bcBatchActive || g_c3Active || g_c2Active) {
        Serial.println("C2: ошибка, двигатель уже в движении.");
        return;
    }

    const bool plateAlreadyOnSensor = g_sensorFilter.stablePlateDetected;

    digitalWrite(PIN_FLAG, FLAG_UP_LEVEL);
    startMotionProfiled(DIR_RIGHT_LEVEL, UINT32_MAX, C2_MOVE_DELAY_US);

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
            handleC2CalibrationAfterCycle();
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
    startMotionProfiled(DIR_RIGHT_LEVEL, UINT32_MAX, C3_MOVE_DELAY_US);

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
        if (g_c3State == C3State::FinalMoveAfterSecondLeave) {
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
            return;
        }

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
                armMotionStopAfterSteps(C3_FINAL_AFTER_SECOND_LEAVE_STEPS);
                if (!g_c3SilentMode) {
                    Serial.println("C3: вторая тарелка ушла с датчика, финальный добег 115 мм.");
                }
            }
            return;

        case C3State::FinalMoveAfterSecondLeave:
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

    if (handleC2CalibrationInput(line)) {
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

    if (cmd == "2") {
        startCycle2();
        return;
    }

    if (cmd == "2Q") {
        startC2CalibrationSurvey();
        return;
    }

    if (cmd == "2P") {
        printC2AdaptiveParams();
        return;
    }

    if (cmd == "2R") {
        resetC2AdaptiveParams();
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

    if (WifiConsole::handleCommand(
            cmd, args, g_motion.active || g_bcBatchActive || g_c2Active || g_c3Active)) {
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
    pinMode(PIN_STEP_DIR, OUTPUT);
    pinMode(PIN_STEP_ENA, OUTPUT);
    pinMode(PIN_FLAG, OUTPUT);
    pinMode(PIN_SENSOR, INPUT_PULLUP);

    writePulseInactive();
    digitalWrite(PIN_STEP_DIR, DIR_LEFT_LEVEL);
    digitalWrite(PIN_STEP_ENA, ENA_ACTIVE_LEVEL);
    digitalWrite(PIN_FLAG, FLAG_DOWN_LEVEL);
    updateSensorFilter();
    WifiConsole::begin();
    MqttLink::setCommandHandler(handleMqttCommand);
    MqttLink::begin();

    Serial.println();
    Serial.println("=== Управление подающим конвейером ===");
    Serial.print("Версия прошивки: ");
    Serial.println(FW_VERSION);
    Serial.println("Плата: ESP32 Dev Module");
    Serial.println("DM542: PUL=13, DIR=12, ENA=14");
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
    WifiConsole::process();
    MqttLink::process();
    processCycle2();
    processCycle3();
    processBCTest();
    processMotion();
    processSensorStream();
}
