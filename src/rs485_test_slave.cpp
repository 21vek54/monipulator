#include <Arduino.h>

#include "pins.h"

namespace {

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t RS485_BAUD = 115200;
constexpr uint32_t TX_SETTLE_US = 150;
constexpr size_t MAX_LINE_LEN = 96;

String g_usbLine;
String g_rsLine;
uint32_t g_sentCount = 0;
uint32_t g_recvCount = 0;
bool g_replyEnabled = true;

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

void printHelp()
{
    Serial.println("RS485 slave test commands:");
    Serial.println("  REPLY ON     - enable auto reply");
    Serial.println("  REPLY OFF    - disable auto reply");
    Serial.println("  SEND <text>  - send custom line");
    Serial.println("  STATE        - print counters");
    Serial.println("  H            - help");
}

void printState()
{
    Serial.print("State: sent=");
    Serial.print(g_sentCount);
    Serial.print(", recv=");
    Serial.print(g_recvCount);
    Serial.print(", reply=");
    Serial.println(g_replyEnabled ? "ON" : "OFF");
}

void handleUsbLine(String line)
{
    line.trim();
    if (line.isEmpty()) {
        return;
    }

    String upper = line;
    upper.toUpperCase();

    if (upper == "REPLY ON") {
        g_replyEnabled = true;
        Serial.println("Reply ON.");
        return;
    }

    if (upper == "REPLY OFF") {
        g_replyEnabled = false;
        Serial.println("Reply OFF.");
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

    if (upper.startsWith("SEND ")) {
        rsSendLine(line.substring(5));
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

void handleRsLine(String line)
{
    g_recvCount++;
    line.trim();
    Serial.print("RX< ");
    Serial.println(line);

    if (!g_replyEnabled || line.isEmpty()) {
        return;
    }

    if (line.startsWith("PING ")) {
        rsSendLine(String("PONG ") + line.substring(5));
        return;
    }

    rsSendLine(String("ACK ") + line);
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
    Serial.println("=== RS485 TEST: SLAVE ===");
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
}
