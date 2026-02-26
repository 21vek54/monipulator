# Changelog

## v1.6

- Added initial MQTT integration for the conveyor controller:
  - Wi-Fi auto-connect support for ESP32.
  - MQTT status publish to `underwater_conveyor/status`.
  - MQTT command subscribe for flag control via `underwater_conveyor/cmd/flag`.
- Added dashboard groundwork and control flow for manual flag up/down commands.
- Added groundwork in the codebase for production cycle logic around command `2`.

