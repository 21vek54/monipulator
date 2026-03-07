#include <Arduino.h>

#include "pins.h"

namespace {

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t RS485_BAUD = 115200;
constexpr uint32_t AUTO_PING_INTERVAL_MS = 1000;
constexpr uint32_t TX_SETTLE_US = 150;
constexpr size_t MAX_LINE_LEN = 96;

String g_usbLine;
String g_rsLine;
uint32_t g_pingSeq = 0;
uint32_t g_lastPingMs = 0;
uint32_t g_sentCount = 0;
uint32_t g_recvCount = 0;
bool g_autoPing = true;

void rsSetRx()
{
    digitalWrite(PIN_RS485_DE_RE, LOW); // RE=0, DE=0
}

void rsSetTx()
{
    digitalWrite(PIN_RS485_DE_RE, HIGH); // RE=1, DE=1
}

void rsSendLine(const String &line)
{
    String msg = line;
    msg.trim();
    if (msg.isEmpty()) {
        return;
    }

    rsSetTx();
    Serial2.print(msg);
    Serial2.print('\n');
    Serial2.flush();
    delayMicroseconds(TX_SETTLE_US);
    rsSetRx();

    g_sentCount++;
    Serial.print("TX> ");
    Serial.println(msg);
}

void sendPing()
{
    g_pingSeq++;
    rsSendLine(String("PING ") + String(g_pingSeq));
}

void printHelp()
{
    Serial.println("RS485 master test commands:");
    Serial.println("  PING         - send one ping");
    Serial.println("  AUTO ON      - enable auto ping every 1s");
    Serial.println("  AUTO OFF     - disable auto ping");
    Serial.println("  MSG <text>   - send custom line");
    Serial.println("  STATE        - print counters");
    Serial.println("  H            - help");
}

void printState()
{
    Serial.print("State: sent=");
    Serial.print(g_sentCount);
    Serial.print(", recv=");
    Serial.print(g_recvCount);
    Serial.print(", auto=");
    Serial.println(g_autoPing ? "ON" : "OFF");
}

void handleUsbLine(String line)
{
    line.trim();
    if (line.isEmpty()) {
        return;
    }

    String upper = line;
    upper.toUpperCase();

    if (upper == "PING") {
        sendPing();
        return;
    }

    if (upper == "AUTO ON") {
        g_autoPing = true;
        Serial.println("Auto ping ON.");
        return;
    }

    if (upper == "AUTO OFF") {
        g_autoPing = false;
        Serial.println("Auto ping OFF.");
        return;
    }

    if (upper == "STATE") {
        printState();
        return;
    }

    if (upper == "H" || upper == "HELP") {
        printHelp();
        return;
    }

    if (upper.startsWith("MSG ")) {
        rsSendLine(line.substring(4));
        return;
    }

    Serial.println("Unknown command. Use H.");
}

void processUsb()
{
    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\r' || ch == '\n') {
            if (!g_usbLine.isEmpty()) {
                handleUsbLine(g_usbLine);
                g_usbLine = "";
            }
            continue;
        }

        g_usbLine += ch;
    }
}

void handleRsLine(const String &line)
{
    g_recvCount++;
    Serial.print("RX< ");
    Serial.println(line);
}

void processRs()
{
    while (Serial2.available() > 0) {
        const char ch = static_cast<char>(Serial2.read());
        if (ch == '\r' || ch == '\n') {
            if (!g_rsLine.isEmpty()) {
                handleRsLine(g_rsLine);
                g_rsLine = "";
            }
            continue;
        }

        g_rsLine += ch;
        if (g_rsLine.length() > MAX_LINE_LEN) {
            g_rsLine = "";
            Serial.println("RX overflow, line cleared.");
        }
    }
}

} // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(300);

    pinMode(PIN_RS485_DE_RE, OUTPUT);
    rsSetRx();
    Serial2.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

    Serial.println();
    Serial.println("=== RS485 TEST: MASTER ===");
    Serial.print("UART2 RX=");
    Serial.print(PIN_RS485_RX);
    Serial.print(" TX=");
    Serial.print(PIN_RS485_TX);
    Serial.print(" DE/RE=");
    Serial.println(PIN_RS485_DE_RE);
    printHelp();
}

void loop()
{
    processUsb();
    processRs();

    if (g_autoPing && (uint32_t)(millis() - g_lastPingMs) >= AUTO_PING_INTERVAL_MS) {
        g_lastPingMs = millis();
        sendPing();
    }
}
