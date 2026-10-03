#pragma once

// BALLY S6000 / UNO R4 Minima V6.5.14 RAM TX + CALIBRATED 12-BIT KEYPAD configuration.
// This build is deliberately fail-closed for value-changing SAS commands.
#define V6_BUILD_ID "S6000-R4-MINIMA-V6.5.14-RAM-AUTO-CLEAR"

#define V6_ENABLE_ADDRESS_AUTO_SCAN 0
#define V6_ADDRESS_SCAN_COOLDOWN_MS 30000UL

// UNO R4 Minima has no onboard WiFi. Keep WiFi diagnostics disabled in this build.
#define V6_ENABLE_WIFI_DIAGNOSTICS 0
#define V6_WIFI_RETRY_MS 20000UL
#define V6_WIFI_SSID ""
#define V6_WIFI_PASS ""

// Protocol simulator hooks may exercise RAM transaction/decoder logic, but do not send value.
#define V6_ENABLE_PROTOCOL_SIMULATOR 1

// Final money mode:
 // - RFID -> S6000 deposit: proven Legacy Bonus 0x8A + LP1A verification.
 // - S6000 -> RFID cashout: full AFT machine->host + LP1A verification.
 // - Split cashout: full AFT, then restore hopper remainder with proven 0x8A.
 // LP71 ticket value commands, A8 and LP94 remain disabled.
#define V6_ENABLE_LEGACY_BONUS_8A 1
#define V6_ENABLE_TICKET_VALUE_COMMANDS 0
#define V6_ENABLE_AFT_VALUE_COMMANDS 1
#define V6_AFT_CASHOUT_TEST_MAX_CREDITS 1UL
#define V6_ENABLE_AFT_FULL_CASHOUT_TEST 1
#define V6_ENABLE_HANDPAY_RESET_COMMANDS 0

// V6.5.14 policy: transaction/recovery state is RAM-only; profile EEPROM remains enabled.
