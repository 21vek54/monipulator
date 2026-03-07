#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

#include "pins.h"

namespace {

constexpr char FW_VERSION[] = "v0.5-master-modbus";
constexpr char DEVICE_NAME[] = "Master controller";

constexpr uint32_t SERIAL_BAUD = 115200;
// Most VFDs default to 9600 for Modbus RTU until reconfigured.
constexpr uint32_t RS485_BAUD = 9600;
constexpr uint32_t TX_SETTLE_US = 150;
constexpr size_t RS485_BUFFER_MAX_LEN = 96;
constexpr bool RS485_TEXT_SNIFFER_ENABLED = false;

constexpr uint32_t MODBUS_TIMEOUT_MS = 300;
constexpr uint16_t MODBUS_MAX_READ_REGS = 32;
constexpr uint8_t MODBUS_FUNC_READ_HOLDING = 0x03;
constexpr uint8_t MODBUS_FUNC_WRITE_SINGLE = 0x06;

// VFD (Vemper/VR) register map from manual pages 86-89.
constexpr uint16_t VFD_REG_CMD = 0x2000;
constexpr uint16_t VFD_REG_STATUS = 0x3000;
constexpr uint16_t VFD_REG_FREQ_SET = 0x1000;   // communication setpoint, -10000..10000 == -100..100%
constexpr uint16_t VFD_REG_FREQ_RUN = 0x1002;   // running frequency (readback)
constexpr uint16_t VFD_REG_FAULT = 0x8000;      // fault code
constexpr uint16_t VFD_REG_PANEL_FREQ_SET = 0xF00B; // P0-11, panel setpoint (used by this drive config)
constexpr uint16_t VFD_REG_FREQ_DECIMALS = 0xF014;  // P0-20, decimal digits for frequency values

constexpr uint16_t VFD_CMD_FWD = 0x0001;
constexpr uint16_t VFD_CMD_REV = 0x0002;
constexpr uint16_t VFD_CMD_STOP_DEC = 0x0006;
constexpr uint16_t VFD_CMD_RESET = 0x0007;

constexpr int32_t VFD_FREQ_RAW_MIN = -10000; // -100.00%
constexpr int32_t VFD_FREQ_RAW_MAX = 10000;  // 100.00%
constexpr float VFD_FREQ_SCALE = 100.0F;     // Hz -> 0.01 Hz units (for percent conversion helper)
constexpr float VFD_RUN_FREQ_SCALE = 10.0F;  // 0x1002 units are 0.1 Hz
constexpr float VFD_COMM_SCALE = 10000.0F;   // 10000 == 100.00% (per VR manual)
constexpr float VFD_CYCLE30_DEFAULT_MIN_HZ = 10.0F;
constexpr float VFD_CYCLE30_DEFAULT_MAX_HZ = 50.0F;
constexpr uint32_t VFD_CYCLE30_TOTAL_MS = 30000;
constexpr uint32_t VFD_CYCLE30_STEP_MS = 1000;
constexpr uint8_t MODBUS_RETRY_COUNT = 3;

String g_rs485RxBuffer;
String g_serialCmdBuffer;
uint32_t g_rsTxCount = 0;
uint32_t g_rsRxCount = 0;

uint8_t g_modbusSlaveId = 1;
uint32_t g_mbReqCount = 0;
uint32_t g_mbOkCount = 0;
uint32_t g_mbErrCount = 0;
uint8_t g_mbLastException = 0;
float g_vfdBaseHz = 50.0F; // P0-14 equivalent used for Hz<->percent conversion.

enum class MbResult : uint8_t {
    Ok,
    ArgError,
    Timeout,
    ProtocolError,
    CrcError,
    Exception
};

void rs485SetReceiveMode()
{
    digitalWrite(PIN_RS485_DE_RE, LOW); // RE=0, DE=0
}

void rs485SetTransmitMode()
{
    digitalWrite(PIN_RS485_DE_RE, HIGH); // RE=1, DE=1
}

void rs485DrainRx()
{
    while (Serial2.available() > 0) {
        (void)Serial2.read();
    }
}

bool rs485ReadExact(uint8_t *dst, size_t len, uint32_t timeoutMs)
{
    size_t readBytes = 0;
    const uint32_t startMs = millis();

    while (readBytes < len) {
        const int c = Serial2.read();
        if (c >= 0) {
            dst[readBytes++] = static_cast<uint8_t>(c);
            continue;
        }

        if ((uint32_t)(millis() - startMs) >= timeoutMs) {
            return false;
        }
        delay(1);
    }

    return true;
}

uint16_t modbusCrc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
            } else {
                crc = static_cast<uint16_t>(crc >> 1);
            }
        }
    }
    return crc;
}

void appendCrc(uint8_t *frame, size_t payloadLen)
{
    const uint16_t crc = modbusCrc16(frame, payloadLen);
    frame[payloadLen] = static_cast<uint8_t>(crc & 0xFF);
    frame[payloadLen + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
}

bool checkFrameCrc(const uint8_t *frame, size_t frameLen)
{
    if (frameLen < 4) {
        return false;
    }

    const uint16_t calc = modbusCrc16(frame, frameLen - 2);
    const uint16_t got = static_cast<uint16_t>(frame[frameLen - 2]) |
                         (static_cast<uint16_t>(frame[frameLen - 1]) << 8);
    return calc == got;
}

void rs485SendBytes(const uint8_t *data, size_t len)
{
    rs485SetTransmitMode();
    Serial2.write(data, len);
    Serial2.flush();
    delayMicroseconds(TX_SETTLE_US);
    rs485SetReceiveMode();
}

MbResult modbusReadHolding(uint16_t reg, uint16_t count, uint16_t *outRegs)
{
    if (count == 0 || count > MODBUS_MAX_READ_REGS || outRegs == nullptr) {
        return MbResult::ArgError;
    }

    uint8_t req[8] = {
        g_modbusSlaveId,
        MODBUS_FUNC_READ_HOLDING,
        static_cast<uint8_t>((reg >> 8) & 0xFF),
        static_cast<uint8_t>(reg & 0xFF),
        static_cast<uint8_t>((count >> 8) & 0xFF),
        static_cast<uint8_t>(count & 0xFF),
        0,
        0};
    appendCrc(req, 6);

    rs485DrainRx();
    rs485SendBytes(req, sizeof(req));
    g_mbReqCount++;

    uint8_t hdr[3] = {};
    if (!rs485ReadExact(hdr, sizeof(hdr), MODBUS_TIMEOUT_MS)) {
        return MbResult::Timeout;
    }

    if (hdr[0] != g_modbusSlaveId) {
        return MbResult::ProtocolError;
    }

    if (hdr[1] == static_cast<uint8_t>(MODBUS_FUNC_READ_HOLDING | 0x80)) {
        uint8_t tail[2] = {};
        if (!rs485ReadExact(tail, sizeof(tail), MODBUS_TIMEOUT_MS)) {
            return MbResult::Timeout;
        }

        uint8_t frame[5] = {hdr[0], hdr[1], hdr[2], tail[0], tail[1]};
        if (!checkFrameCrc(frame, sizeof(frame))) {
            return MbResult::CrcError;
        }

        g_mbLastException = hdr[2];
        return MbResult::Exception;
    }

    if (hdr[1] != MODBUS_FUNC_READ_HOLDING) {
        return MbResult::ProtocolError;
    }

    const uint8_t byteCount = hdr[2];
    if (byteCount != static_cast<uint8_t>(count * 2U)) {
        return MbResult::ProtocolError;
    }

    uint8_t tail[(MODBUS_MAX_READ_REGS * 2U) + 2U] = {};
    const size_t tailLen = static_cast<size_t>(byteCount) + 2U;
    if (!rs485ReadExact(tail, tailLen, MODBUS_TIMEOUT_MS)) {
        return MbResult::Timeout;
    }

    uint8_t frame[3 + (MODBUS_MAX_READ_REGS * 2U) + 2U] = {};
    frame[0] = hdr[0];
    frame[1] = hdr[1];
    frame[2] = hdr[2];
    memcpy(&frame[3], tail, tailLen);

    if (!checkFrameCrc(frame, 3U + tailLen)) {
        return MbResult::CrcError;
    }

    for (uint16_t i = 0; i < count; i++) {
        const uint8_t hi = tail[i * 2U];
        const uint8_t lo = tail[i * 2U + 1U];
        outRegs[i] = (static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo);
    }

    return MbResult::Ok;
}

MbResult modbusWriteSingle(uint16_t reg, uint16_t value)
{
    uint8_t req[8] = {
        g_modbusSlaveId,
        MODBUS_FUNC_WRITE_SINGLE,
        static_cast<uint8_t>((reg >> 8) & 0xFF),
        static_cast<uint8_t>(reg & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF),
        0,
        0};
    appendCrc(req, 6);

    rs485DrainRx();
    rs485SendBytes(req, sizeof(req));
    g_mbReqCount++;

    uint8_t resp[8] = {};
    if (!rs485ReadExact(resp, sizeof(resp), MODBUS_TIMEOUT_MS)) {
        return MbResult::Timeout;
    }

    if (resp[0] != g_modbusSlaveId) {
        return MbResult::ProtocolError;
    }

    if (resp[1] == static_cast<uint8_t>(MODBUS_FUNC_WRITE_SINGLE | 0x80)) {
        g_mbLastException = resp[2];
        if (!checkFrameCrc(resp, 5)) {
            return MbResult::CrcError;
        }
        return MbResult::Exception;
    }

    if (resp[1] != MODBUS_FUNC_WRITE_SINGLE) {
        return MbResult::ProtocolError;
    }

    if (!checkFrameCrc(resp, sizeof(resp))) {
        return MbResult::CrcError;
    }

    if (memcmp(req, resp, sizeof(req) - 2U) != 0) {
        return MbResult::ProtocolError;
    }

    return MbResult::Ok;
}

MbResult modbusWriteSingleRetry(uint16_t reg, uint16_t value, uint8_t attempts)
{
    if (attempts == 0) {
        attempts = 1;
    }

    MbResult last = MbResult::ArgError;
    for (uint8_t i = 0; i < attempts; i++) {
        last = modbusWriteSingle(reg, value);
        if (last == MbResult::Ok || last == MbResult::Exception || last == MbResult::ArgError) {
            return last;
        }
        delay(20);
    }
    return last;
}

MbResult modbusReadHoldingRetry(uint16_t reg, uint16_t count, uint16_t *outRegs, uint8_t attempts)
{
    if (attempts == 0) {
        attempts = 1;
    }

    MbResult last = MbResult::ArgError;
    for (uint8_t i = 0; i < attempts; i++) {
        last = modbusReadHolding(reg, count, outRegs);
        if (last == MbResult::Ok || last == MbResult::Exception || last == MbResult::ArgError) {
            return last;
        }
        delay(20);
    }
    return last;
}

bool freqHzToRawU16(float hz, uint16_t &rawU16, int32_t &rawSigned)
{
    if (g_vfdBaseHz < 0.01F) {
        return false;
    }

    rawSigned = static_cast<int32_t>(lroundf((hz / g_vfdBaseHz) * VFD_COMM_SCALE));
    if (rawSigned < VFD_FREQ_RAW_MIN || rawSigned > VFD_FREQ_RAW_MAX) {
        return false;
    }
    rawU16 = static_cast<uint16_t>(static_cast<int16_t>(rawSigned));
    return true;
}

MbResult readVfdFrequencyScale(float &scaleOut)
{
    uint16_t dec = 0;
    const MbResult r = modbusReadHoldingRetry(VFD_REG_FREQ_DECIMALS, 1, &dec, MODBUS_RETRY_COUNT);
    if (r != MbResult::Ok) {
        return r;
    }

    if (dec == 1) {
        scaleOut = 10.0F;
    } else if (dec == 2) {
        scaleOut = 100.0F;
    } else {
        // Safe fallback for uncommon settings.
        scaleOut = 10.0F;
    }
    return MbResult::Ok;
}

bool hzToPanelRawU16(float hz, float scale, uint16_t &rawOut)
{
    if (hz < 0.0F || hz > 400.0F || scale <= 0.0F) {
        return false;
    }

    const int32_t raw = static_cast<int32_t>(lroundf(hz * scale));
    if (raw < 0 || raw > 0xFFFF) {
        return false;
    }

    rawOut = static_cast<uint16_t>(raw);
    return true;
}

void printMbResult(MbResult result)
{
    switch (result) {
        case MbResult::Ok:
            g_mbOkCount++;
            Serial.println("MODBUS: OK");
            break;
        case MbResult::ArgError:
            g_mbErrCount++;
            Serial.println("MODBUS: arg error.");
            break;
        case MbResult::Timeout:
            g_mbErrCount++;
            Serial.println("MODBUS: timeout.");
            break;
        case MbResult::ProtocolError:
            g_mbErrCount++;
            Serial.println("MODBUS: protocol error.");
            break;
        case MbResult::CrcError:
            g_mbErrCount++;
            Serial.println("MODBUS: CRC error.");
            break;
        case MbResult::Exception:
            g_mbErrCount++;
            Serial.print("MODBUS: exception 0x");
            Serial.println(g_mbLastException, HEX);
            break;
        default:
            g_mbErrCount++;
            Serial.println("MODBUS: unknown error.");
            break;
    }
}

bool parseU16(String token, uint16_t &value)
{
    token.trim();
    if (token.isEmpty()) {
        return false;
    }

    char *end = nullptr;
    const unsigned long v = strtoul(token.c_str(), &end, 0);
    if (end == token.c_str() || *end != '\0' || v > 0xFFFFUL) {
        return false;
    }

    value = static_cast<uint16_t>(v);
    return true;
}

bool parseI32(String token, int32_t &value)
{
    token.trim();
    if (token.isEmpty()) {
        return false;
    }

    char *end = nullptr;
    const long v = strtol(token.c_str(), &end, 0);
    if (end == token.c_str() || *end != '\0') {
        return false;
    }

    value = static_cast<int32_t>(v);
    return true;
}

bool parseF32(String token, float &value)
{
    token.trim();
    if (token.isEmpty()) {
        return false;
    }

    token.replace(',', '.');
    char *end = nullptr;
    const float v = strtof(token.c_str(), &end);
    if (end == token.c_str() || *end != '\0') {
        return false;
    }

    value = v;
    return true;
}

String nextToken(String &s)
{
    s.trim();
    if (s.isEmpty()) {
        return "";
    }

    const int split = s.indexOf(' ');
    if (split < 0) {
        String t = s;
        s = "";
        return t;
    }

    String t = s.substring(0, split);
    s = s.substring(split + 1);
    s.trim();
    return t;
}

bool mapToFlagCommand(String input, String &mapped)
{
    input.trim();
    input.toUpperCase();

    if (input == "W" || input == "UP" || input == "FLAG UP" || input == "1" || input == "ON") {
        mapped = "FLAG UP";
        return true;
    }

    if (input == "S" || input == "DOWN" || input == "FLAG DOWN" || input == "0" || input == "OFF") {
        mapped = "FLAG DOWN";
        return true;
    }

    return false;
}

void rs485SendTextLine(const String &line)
{
    String cmd = line;
    cmd.trim();
    if (cmd.isEmpty()) {
        return;
    }

    rs485SetTransmitMode();
    Serial2.print(cmd);
    Serial2.print('\n');
    Serial2.flush();
    delayMicroseconds(TX_SETTLE_US);
    rs485SetReceiveMode();

    g_rsTxCount++;
    Serial.print("RS485 TX: ");
    Serial.println(cmd);
}

void printHelp()
{
    Serial.println("Commands:");
    Serial.println("  UP / W            - send FLAG UP as text (legacy)");
    Serial.println("  DOWN / S          - send FLAG DOWN as text (legacy)");
    Serial.println("  RS <text>         - send raw text line to RS485");
    Serial.println("  MBID <id>         - set Modbus slave id (1..247)");
    Serial.println("  MBR <reg> [cnt]   - read holding regs (func 03)");
    Serial.println("  MBW <reg> <val>   - write single reg (func 06)");
    Serial.println("  VFD <...>         - VFD quick commands (VFD HELP)");
    Serial.println("  STATE             - print RS + Modbus counters");
    Serial.println("  H                 - help");
}

void printState()
{
    Serial.print("RS485 state: tx=");
    Serial.print(g_rsTxCount);
    Serial.print(", rx=");
    Serial.println(g_rsRxCount);

    Serial.print("MODBUS state: id=");
    Serial.print(g_modbusSlaveId);
    Serial.print(", req=");
    Serial.print(g_mbReqCount);
    Serial.print(", ok=");
    Serial.print(g_mbOkCount);
    Serial.print(", err=");
    Serial.println(g_mbErrCount);
}

void handleCommandMbId(String args)
{
    String tok = nextToken(args);
    uint16_t id = 0;
    if (!parseU16(tok, id) || id == 0 || id > 247) {
        Serial.println("Usage: MBID <1..247>");
        return;
    }

    g_modbusSlaveId = static_cast<uint8_t>(id);
    Serial.print("MODBUS: slave id set to ");
    Serial.println(g_modbusSlaveId);
}

void handleCommandMbRead(String args)
{
    String regTok = nextToken(args);
    String cntTok = nextToken(args);

    uint16_t reg = 0;
    if (!parseU16(regTok, reg)) {
        Serial.println("Usage: MBR <reg> [count]");
        return;
    }

    uint16_t count = 1;
    if (!cntTok.isEmpty() && !parseU16(cntTok, count)) {
        Serial.println("Usage: MBR <reg> [count]");
        return;
    }

    if (count == 0 || count > MODBUS_MAX_READ_REGS) {
        Serial.print("Count must be 1..");
        Serial.println(MODBUS_MAX_READ_REGS);
        return;
    }

    uint16_t regs[MODBUS_MAX_READ_REGS] = {};
    const MbResult result = modbusReadHolding(reg, count, regs);
    printMbResult(result);
    if (result != MbResult::Ok) {
        return;
    }

    for (uint16_t i = 0; i < count; i++) {
        Serial.print("R[");
        Serial.print(reg + i);
        Serial.print("] = ");
        Serial.print(regs[i]);
        Serial.print(" (0x");
        Serial.print(regs[i], HEX);
        Serial.println(")");
    }
}

void handleCommandMbWrite(String args)
{
    String regTok = nextToken(args);
    String valTok = nextToken(args);

    uint16_t reg = 0;
    uint16_t value = 0;
    if (!parseU16(regTok, reg) || !parseU16(valTok, value)) {
        Serial.println("Usage: MBW <reg> <value>");
        return;
    }

    const MbResult result = modbusWriteSingle(reg, value);
    printMbResult(result);
}

void printVfdUsage()
{
    Serial.println("VFD commands:");
    Serial.println("  VFD FWD                 - run forward (0x2000=1)");
    Serial.println("  VFD REV                 - run reverse (0x2000=2)");
    Serial.println("  VFD STOP                - decel stop (0x2000=6)");
    Serial.println("  VFD RESET               - fault reset (0x2000=7)");
    Serial.println("  VFD STAT                - read status (0x3000)");
    Serial.println("  VFD FAULT               - read fault code (0x8000)");
    Serial.println("  VFD FREQ <hz>           - set setpoint in Hz (via P0-11)");
    Serial.println("  VFD FREQRAW <value>     - set raw percent -10000..10000 (legacy 0x1000)");
    Serial.println("  VFD MAXHZ [hz]          - set/get base Hz used for VFD FREQ");
    Serial.println("  VFD RUNFREQ             - read run freq raw (0x1002)");
    Serial.println("  VFD CYCLE30 [min] [max] - 30s ramp + actual Hz monitor + STOP");
}

void printVfdStatusWord(uint16_t statusWord)
{
    Serial.print("VFD STATUS: ");
    switch (statusWord) {
        case 0x0001:
            Serial.println("FORWARD");
            break;
        case 0x0002:
            Serial.println("REVERSE");
            break;
        case 0x0003:
            Serial.println("STOP");
            break;
        default:
            Serial.print("UNKNOWN (");
            Serial.print(statusWord);
            Serial.println(")");
            break;
    }
}

void handleCommandVfd(String args)
{
    String sub = nextToken(args);
    sub.toUpperCase();

    if (sub.isEmpty() || sub == "H" || sub == "HELP") {
        printVfdUsage();
        return;
    }

    if (sub == "FWD" || sub == "RUN") {
        printMbResult(modbusWriteSingleRetry(VFD_REG_CMD, VFD_CMD_FWD, MODBUS_RETRY_COUNT));
        return;
    }

    if (sub == "REV") {
        printMbResult(modbusWriteSingleRetry(VFD_REG_CMD, VFD_CMD_REV, MODBUS_RETRY_COUNT));
        return;
    }

    if (sub == "STOP") {
        printMbResult(modbusWriteSingleRetry(VFD_REG_CMD, VFD_CMD_STOP_DEC, MODBUS_RETRY_COUNT));
        return;
    }

    if (sub == "RESET") {
        printMbResult(modbusWriteSingleRetry(VFD_REG_CMD, VFD_CMD_RESET, MODBUS_RETRY_COUNT));
        return;
    }

    if (sub == "STAT" || sub == "STATUS") {
        uint16_t reg = 0;
        const MbResult r = modbusReadHoldingRetry(VFD_REG_STATUS, 1, &reg, MODBUS_RETRY_COUNT);
        printMbResult(r);
        if (r == MbResult::Ok) {
            printVfdStatusWord(reg);
        }
        return;
    }

    if (sub == "FAULT") {
        uint16_t reg = 0;
        const MbResult r = modbusReadHoldingRetry(VFD_REG_FAULT, 1, &reg, MODBUS_RETRY_COUNT);
        printMbResult(r);
        if (r == MbResult::Ok) {
            Serial.print("VFD FAULT: ");
            Serial.print(reg);
            Serial.print(" (0x");
            Serial.print(reg, HEX);
            Serial.println(")");
        }
        return;
    }

    if (sub == "FREQRAW") {
        String valueTok = nextToken(args);
        int32_t signedRaw = 0;
        if (!parseI32(valueTok, signedRaw)) {
            Serial.println("Usage: VFD FREQRAW <-10000..10000>");
            return;
        }

        if (signedRaw < VFD_FREQ_RAW_MIN || signedRaw > VFD_FREQ_RAW_MAX) {
            Serial.print("VFD FREQRAW out of range: ");
            Serial.print(VFD_FREQ_RAW_MIN);
            Serial.print("..");
            Serial.println(VFD_FREQ_RAW_MAX);
            return;
        }

        const uint16_t valueRawU16 = static_cast<uint16_t>(static_cast<int16_t>(signedRaw));
        printMbResult(modbusWriteSingleRetry(VFD_REG_FREQ_SET, valueRawU16, MODBUS_RETRY_COUNT));
        Serial.print("VFD setpoint raw: ");
        Serial.print(signedRaw);
        Serial.print(" (~");
        Serial.print((static_cast<float>(signedRaw) / VFD_COMM_SCALE) * 100.0F, 2);
        Serial.println("%)");
        return;
    }

    if (sub == "MAXHZ") {
        String hzTok = nextToken(args);
        if (hzTok.isEmpty()) {
            Serial.print("VFD MAXHZ: ");
            Serial.println(g_vfdBaseHz, 2);
            return;
        }

        float hz = 0.0F;
        if (!parseF32(hzTok, hz) || hz <= 0.0F || hz > 400.0F) {
            Serial.println("Usage: VFD MAXHZ <0.01..400>");
            return;
        }

        g_vfdBaseHz = hz;
        Serial.print("VFD MAXHZ set to ");
        Serial.println(g_vfdBaseHz, 2);
        return;
    }

    if (sub == "FREQ") {
        String hzTok = nextToken(args);
        float hz = 0.0F;
        if (!parseF32(hzTok, hz)) {
            Serial.println("Usage: VFD FREQ <hz>");
            return;
        }

        float freqScale = 10.0F;
        const MbResult sr = readVfdFrequencyScale(freqScale);
        if (sr != MbResult::Ok) {
            printMbResult(sr);
            return;
        }

        uint16_t rawU16 = 0;
        if (!hzToPanelRawU16(hz, freqScale, rawU16)) {
            Serial.println("VFD FREQ out of range.");
            return;
        }

        const MbResult r = modbusWriteSingleRetry(VFD_REG_PANEL_FREQ_SET, rawU16, MODBUS_RETRY_COUNT);
        printMbResult(r);
        if (r == MbResult::Ok) {
            Serial.print("VFD setpoint Hz: ");
            Serial.print(hz, 2);
            Serial.print(" (P0-11 raw=");
            Serial.print(rawU16);
            Serial.println(")");
        }
        return;
    }

    if (sub == "RUNFREQ") {
        uint16_t reg = 0;
        const MbResult r = modbusReadHoldingRetry(VFD_REG_FREQ_RUN, 1, &reg, MODBUS_RETRY_COUNT);
        printMbResult(r);
        if (r == MbResult::Ok) {
            const int16_t signedRaw = static_cast<int16_t>(reg);
            Serial.print("VFD RUN FREQ RAW: ");
            Serial.print(signedRaw);
            Serial.print(" (~");
            Serial.print(static_cast<float>(signedRaw) / VFD_RUN_FREQ_SCALE, 2);
            Serial.println(" Hz)");
        }
        return;
    }

    if (sub == "CYCLE30") {
        String minTok = nextToken(args);
        String maxTok = nextToken(args);

        float minHz = VFD_CYCLE30_DEFAULT_MIN_HZ;
        float maxHz = VFD_CYCLE30_DEFAULT_MAX_HZ;

        if (!minTok.isEmpty() && !parseF32(minTok, minHz)) {
            Serial.println("Usage: VFD CYCLE30 [minHz] [maxHz]");
            return;
        }
        if (!maxTok.isEmpty() && !parseF32(maxTok, maxHz)) {
            Serial.println("Usage: VFD CYCLE30 [minHz] [maxHz]");
            return;
        }
        if (maxHz <= minHz) {
            Serial.println("VFD CYCLE30: maxHz must be > minHz.");
            return;
        }

        uint16_t rawMinU16 = 0;
        uint16_t rawMaxU16 = 0;
        float freqScale = 10.0F;
        const MbResult sr = readVfdFrequencyScale(freqScale);
        if (sr != MbResult::Ok) {
            printMbResult(sr);
            return;
        }

        if (!hzToPanelRawU16(minHz, freqScale, rawMinU16) ||
            !hzToPanelRawU16(maxHz, freqScale, rawMaxU16)) {
            Serial.println("VFD CYCLE30: frequency out of range.");
            return;
        }

        Serial.print("VFD CYCLE30 start: ");
        Serial.print(minHz, 2);
        Serial.print(" -> ");
        Serial.print(maxHz, 2);
        Serial.println(" -> STOP");

        MbResult r = modbusWriteSingleRetry(VFD_REG_PANEL_FREQ_SET, rawMinU16, MODBUS_RETRY_COUNT);
        printMbResult(r);
        if (r != MbResult::Ok) {
            Serial.println("VFD CYCLE30 aborted at pre-set.");
            return;
        }

        r = modbusWriteSingleRetry(VFD_REG_CMD, VFD_CMD_FWD, MODBUS_RETRY_COUNT);
        printMbResult(r);
        if (r != MbResult::Ok) {
            Serial.println("VFD CYCLE30 aborted at start command.");
            return;
        }

        const uint32_t halfMs = VFD_CYCLE30_TOTAL_MS / 2U;
        const uint32_t steps = VFD_CYCLE30_TOTAL_MS / VFD_CYCLE30_STEP_MS;
        const uint32_t cycleStartMs = millis();

        for (uint32_t step = 0; step <= steps; step++) {
            const uint32_t elapsed = step * VFD_CYCLE30_STEP_MS;
            while (static_cast<uint32_t>(millis() - cycleStartMs) < elapsed) {
                delay(1);
            }

            float phase = 0.0F;
            float targetHz = minHz;

            if (elapsed <= halfMs) {
                phase = static_cast<float>(elapsed) / static_cast<float>(halfMs);
                targetHz = minHz + (maxHz - minHz) * phase;
            } else {
                phase = static_cast<float>(elapsed - halfMs) / static_cast<float>(halfMs);
                targetHz = maxHz - (maxHz - minHz) * phase;
            }

            uint16_t targetRawU16 = 0;
            if (!hzToPanelRawU16(targetHz, freqScale, targetRawU16)) {
                Serial.println("VFD CYCLE30: internal target out of range, stopping.");
                break;
            }

            r = modbusWriteSingleRetry(VFD_REG_PANEL_FREQ_SET, targetRawU16, MODBUS_RETRY_COUNT);
            if (r != MbResult::Ok) {
                printMbResult(r);
                Serial.println("VFD CYCLE30 write failed, stopping.");
                break;
            }
            g_mbOkCount++;

            uint16_t runRawU16 = 0;
            const MbResult runR = modbusReadHoldingRetry(VFD_REG_FREQ_RUN, 1, &runRawU16, MODBUS_RETRY_COUNT);

            Serial.print("CYCLE30 t=");
            Serial.print(elapsed / 1000U);
            Serial.print("s target=");
            Serial.print(targetHz, 2);
            Serial.print("Hz actual=");

            if (runR == MbResult::Ok) {
                g_mbOkCount++;
                const int16_t runRaw = static_cast<int16_t>(runRawU16);
                Serial.print(static_cast<float>(runRaw) / VFD_RUN_FREQ_SCALE, 2);
                Serial.println("Hz");
            } else {
                Serial.println("n/a");
                printMbResult(runR);
            }
        }

        r = modbusWriteSingleRetry(VFD_REG_CMD, VFD_CMD_STOP_DEC, MODBUS_RETRY_COUNT);
        printMbResult(r);
        Serial.println("VFD CYCLE30 finished.");
        return;
    }

    Serial.println("Unknown VFD subcommand. Use: VFD HELP");
}

void handleConsoleLine(String line)
{
    line.trim();
    if (line.isEmpty()) {
        return;
    }

    String cmd = line;
    String args = "";
    const int sp = cmd.indexOf(' ');
    if (sp > 0) {
        args = cmd.substring(sp + 1);
        cmd = cmd.substring(0, sp);
    } else if (sp == 0) {
        cmd = "";
    }

    cmd.trim();
    cmd.toUpperCase();
    args.trim();

    if (cmd == "H" || cmd == "HELP") {
        printHelp();
        return;
    }

    if (cmd == "STATE") {
        printState();
        return;
    }

    if (cmd == "MBID") {
        handleCommandMbId(args);
        return;
    }

    if (cmd == "MBR") {
        handleCommandMbRead(args);
        return;
    }

    if (cmd == "MBW") {
        handleCommandMbWrite(args);
        return;
    }

    if (cmd == "VFD") {
        handleCommandVfd(args);
        return;
    }

    if (cmd == "RS") {
        rs485SendTextLine(args);
        return;
    }

    String mapped;
    if (mapToFlagCommand(cmd + (args.isEmpty() ? "" : String(" ") + args), mapped)) {
        rs485SendTextLine(mapped);
        return;
    }

    Serial.println("Unknown command. Use H.");
}

void processConsole()
{
    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\r' || ch == '\n') {
            if (!g_serialCmdBuffer.isEmpty()) {
                handleConsoleLine(g_serialCmdBuffer);
                g_serialCmdBuffer = "";
            }
            continue;
        }

        g_serialCmdBuffer += ch;
    }
}

void processRs485RxText()
{
    while (Serial2.available() > 0) {
        const char ch = static_cast<char>(Serial2.read());
        if (ch == '\r' || ch == '\n') {
            if (!g_rs485RxBuffer.isEmpty()) {
                g_rsRxCount++;
                Serial.print("RS485 RX: ");
                Serial.println(g_rs485RxBuffer);
                g_rs485RxBuffer = "";
            }
            continue;
        }

        g_rs485RxBuffer += ch;
        if (g_rs485RxBuffer.length() > RS485_BUFFER_MAX_LEN) {
            g_rs485RxBuffer = "";
            Serial.println("RS485 RX overflow, buffer cleared.");
        }
    }
}

void printBanner()
{
    Serial.println();
    Serial.print("=== ");
    Serial.print(DEVICE_NAME);
    Serial.println(" ===");
    Serial.print("FW: ");
    Serial.println(FW_VERSION);
    Serial.println("Role: RS485 + Modbus RTU master.");
    Serial.print("RS485: RX=");
    Serial.print(PIN_RS485_RX);
    Serial.print(", TX=");
    Serial.print(PIN_RS485_TX);
    Serial.print(", DE/RE=");
    Serial.println(PIN_RS485_DE_RE);
    Serial.print("MODBUS default slave id: ");
    Serial.println(g_modbusSlaveId);
    printHelp();
}

} // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(300);

    pinMode(PIN_RS485_DE_RE, OUTPUT);
    rs485SetReceiveMode();
    Serial2.begin(RS485_BAUD, SERIAL_8E1, PIN_RS485_RX, PIN_RS485_TX);

    printBanner();
}

void loop()
{
    processConsole();
    if (RS485_TEXT_SNIFFER_ENABLED) {
        processRs485RxText();
    }
}
