#include "wifi_console.h"

#include <WiFi.h>

namespace WifiConsole {

constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
constexpr char WIFI_DEFAULT_SSID[] = "21VEK";
constexpr char WIFI_DEFAULT_PASS[] = "vek2121vek";

struct WiFiState {
    bool connectPending = false;
    uint32_t connectStartedMs = 0;
    String targetSsid;
};

WiFiState g_state;

const char *authModeToText(wifi_auth_mode_t mode)
{
    switch (mode) {
        case WIFI_AUTH_OPEN:
            return "OPEN";
        case WIFI_AUTH_WEP:
            return "WEP";
        case WIFI_AUTH_WPA_PSK:
            return "WPA";
        case WIFI_AUTH_WPA2_PSK:
            return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK:
            return "WPA/WPA2";
        case WIFI_AUTH_WPA2_ENTERPRISE:
            return "WPA2-ENT";
        case WIFI_AUTH_WPA3_PSK:
            return "WPA3";
        case WIFI_AUTH_WPA2_WPA3_PSK:
            return "WPA2/WPA3";
        case WIFI_AUTH_WAPI_PSK:
            return "WAPI";
        default:
            return "UNKNOWN";
    }
}

bool parseArgs(String args, String &ssid, String &password)
{
    args.trim();
    if (args.isEmpty()) {
        return false;
    }

    const int splitPos = args.indexOf(' ');
    if (splitPos < 0) {
        ssid = args;
        password = "";
        ssid.trim();
        return !ssid.isEmpty();
    }

    ssid = args.substring(0, splitPos);
    password = args.substring(splitPos + 1);
    ssid.trim();
    password.trim();
    return !ssid.isEmpty();
}

void printStatus()
{
    const wl_status_t status = WiFi.status();
    Serial.print("WiFi status: ");
    Serial.println(status == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");

    if (status == WL_CONNECTED) {
        Serial.print("  SSID: ");
        Serial.println(WiFi.SSID());
        Serial.print("  IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("  RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    }
}

void disconnect()
{
    g_state.connectPending = false;
    WiFi.disconnect(true, false);
    Serial.println("WiFi: disconnected.");
}

void startConnect(const String &ssid, const String &password, bool machineBusy)
{
    if (ssid.isEmpty()) {
        Serial.println("WiFi: SSID is empty.");
        return;
    }

    if (machineBusy) {
        Serial.println("WiFi: команда недоступна во время движения/цикла.");
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid.c_str(), password.c_str());

    g_state.connectPending = true;
    g_state.connectStartedMs = millis();
    g_state.targetSsid = ssid;

    Serial.print("WiFi: connecting to ");
    Serial.print(ssid);
    Serial.println(" ...");
}

void scanNetworks(bool machineBusy)
{
    if (machineBusy) {
        Serial.println("WiFi scan: недоступно во время движения/цикла.");
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    Serial.println("WiFi scan: scanning...");
    const int count = WiFi.scanNetworks(false, true);
    if (count < 0) {
        Serial.println("WiFi scan: failed.");
        return;
    }

    if (count == 0) {
        Serial.println("WiFi scan: no networks found.");
        return;
    }

    Serial.print("WiFi scan: found ");
    Serial.print(count);
    Serial.println(" network(s):");

    for (int i = 0; i < count; i++) {
        Serial.print("  ");
        Serial.print(i + 1);
        Serial.print(". SSID='");
        Serial.print(WiFi.SSID(i));
        Serial.print("' RSSI=");
        Serial.print(WiFi.RSSI(i));
        Serial.print(" dBm CH=");
        Serial.print(WiFi.channel(i));
        Serial.print(" ENC=");
        Serial.println(authModeToText(WiFi.encryptionType(i)));
    }

    WiFi.scanDelete();
}

void begin()
{
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    startConnect(String(WIFI_DEFAULT_SSID), String(WIFI_DEFAULT_PASS), false);
}

void process()
{
    if (!g_state.connectPending) {
        return;
    }

    if (WiFi.status() == WL_CONNECTED) {
        g_state.connectPending = false;
        Serial.print("WiFi: connected to ");
        Serial.println(WiFi.SSID());
        Serial.print("WiFi: IP ");
        Serial.println(WiFi.localIP());
        return;
    }

    if ((uint32_t)(millis() - g_state.connectStartedMs) >= WIFI_CONNECT_TIMEOUT_MS) {
        g_state.connectPending = false;
        Serial.print("WiFi: connect timeout for ");
        Serial.println(g_state.targetSsid);
        Serial.println("WiFi: check password/signal and retry.");
    }
}

void printHelp()
{
    Serial.println("  WSCAN        - скан WiFi-сетей вокруг");
    Serial.println("  WIFI ssid [pass] - подключить ESP32 к WiFi");
    Serial.println("  WSTAT        - статус WiFi (SSID/IP/RSSI)");
    Serial.println("  WDIS         - отключить WiFi");
}

bool handleCommand(const String &cmd, const String &args, bool machineBusy)
{
    if (cmd == "WSCAN") {
        scanNetworks(machineBusy);
        return true;
    }

    if (cmd == "WIFI") {
        String ssid;
        String password;
        if (!parseArgs(args, ssid, password)) {
            Serial.println("Format error. Example: WIFI MySSID MyPassword");
            return true;
        }
        startConnect(ssid, password, machineBusy);
        return true;
    }

    if (cmd == "WSTAT") {
        printStatus();
        return true;
    }

    if (cmd == "WDIS") {
        disconnect();
        return true;
    }

    return false;
}

} // namespace WifiConsole
