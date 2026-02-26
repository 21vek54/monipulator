#include "mqtt_link.h"

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

namespace MqttLink {

constexpr char MQTT_BROKER_HOST[] = "192.168.1.136";
constexpr uint16_t MQTT_BROKER_PORT = 1883;
constexpr char MQTT_TOPIC_STATUS[] = "underwater_conveyor/status";
constexpr char MQTT_TOPIC_FLAG_COMMAND[] = "underwater_conveyor/cmd/flag";
constexpr uint32_t MQTT_RECONNECT_INTERVAL_MS = 5000;
constexpr uint32_t MQTT_HEARTBEAT_INTERVAL_MS = 30000;

WiFiClient g_wifiClient;
PubSubClient g_mqtt(g_wifiClient);
uint32_t g_lastReconnectAttemptMs = 0;
uint32_t g_lastHeartbeatMs = 0;
String g_clientId;
CommandHandler g_commandHandler = nullptr;

void setCommandHandler(CommandHandler handler)
{
    g_commandHandler = handler;
}

void processFlagCommand(const String &payload)
{
    String normalized = payload;
    normalized.trim();
    normalized.toUpperCase();

    String mapped;
    if (normalized == "UP" || normalized == "W" || normalized == "FLAG_UP") {
        mapped = "W";
    } else if (normalized == "DOWN" || normalized == "S" || normalized == "FLAG_DOWN") {
        mapped = "S";
    } else {
        Serial.print("MQTT: unknown flag cmd '");
        Serial.print(payload);
        Serial.println("'.");
        return;
    }

    Serial.print("MQTT: flag cmd ");
    Serial.println(normalized);

    if (g_commandHandler != nullptr) {
        g_commandHandler(mapped);
    }
}

void onMqttMessage(char *topic, uint8_t *payload, unsigned int length)
{
    String message;
    message.reserve(length);
    for (unsigned int i = 0; i < length; i++) {
        message += static_cast<char>(payload[i]);
    }

    if (String(topic) == MQTT_TOPIC_FLAG_COMMAND) {
        processFlagCommand(message);
    }
}

String buildClientId()
{
    uint64_t chipId = ESP.getEfuseMac();
    uint32_t low = static_cast<uint32_t>(chipId & 0xFFFFFFFFULL);
    char suffix[9];
    snprintf(suffix, sizeof(suffix), "%08lX", static_cast<unsigned long>(low));
    return String("underwater_conveyor_") + suffix;
}

void publishOnline()
{
    g_mqtt.publish(MQTT_TOPIC_STATUS, "online", true);
}

bool connectIfNeeded()
{
    if (g_mqtt.connected()) {
        return true;
    }

    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }

    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - g_lastReconnectAttemptMs) < MQTT_RECONNECT_INTERVAL_MS) {
        return false;
    }
    g_lastReconnectAttemptMs = nowMs;

    if (g_mqtt.connect(g_clientId.c_str(), nullptr, nullptr, MQTT_TOPIC_STATUS, 1, true, "offline")) {
        Serial.println("MQTT: connected.");
        g_mqtt.subscribe(MQTT_TOPIC_FLAG_COMMAND, 1);
        Serial.print("MQTT: subscribed ");
        Serial.println(MQTT_TOPIC_FLAG_COMMAND);
        publishOnline();
        g_lastHeartbeatMs = nowMs;
        return true;
    }

    Serial.print("MQTT: connect failed, state=");
    Serial.println(g_mqtt.state());
    return false;
}

void begin()
{
    g_clientId = buildClientId();
    g_mqtt.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    g_mqtt.setCallback(onMqttMessage);
    Serial.print("MQTT: broker ");
    Serial.print(MQTT_BROKER_HOST);
    Serial.print(":");
    Serial.println(MQTT_BROKER_PORT);
}

void process()
{
    if (!connectIfNeeded()) {
        return;
    }

    g_mqtt.loop();

    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - g_lastHeartbeatMs) >= MQTT_HEARTBEAT_INTERVAL_MS) {
        publishOnline();
        g_lastHeartbeatMs = nowMs;
    }
}

} // namespace MqttLink
