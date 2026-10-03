/*
  BALLY S6000 + Arduino UNO R4 Minima + MAX232
  V6.5.14 - RAM TX AUTO-CLEAR + CALIBRATED 12-BIT ANALOG KEYPAD

  Target:
    Arduino UNO R4 Minima (Renesas RA4M1)
    Arduino Renesas Boards core: 1.6.0
    WiFi: disabled in this Minima build (UNO R4 Minima has no onboard WiFi).

  Hardware preserved from the original project:
    D0  <- MAX232 R1OUT  (S6000 TX -> MAX232 -> UNO RX)
    D1  -> MAX232 T1IN   (UNO TX -> MAX232 -> S6000 RX)
    D4  RGB BLUE relay output  (LOW = ON)
    D5  RGB RED relay output   (LOW = ON)
    D7  RGB GREEN relay output (LOW = ON)
    D9  MFRC522 RST
    D10 MFRC522 SS
    D11 MOSI
    D12 MISO
    D13 SCK
    A0  analog keypad
    A4/A5 I2C LCD

  IMPORTANT V6.0 CHANGES
  ----------------------
  1) Uses the proven patched Serial1 TRUE 9-bit UART path at 19200 9N1.
  2) Correctly separates SAS Type R / Type S / Type M framing.
       Type R: Address(wakeup) + Command, NO host-request CRC.
       Type S: Address + Command + data + CRC.
       Type M: Type S with 2-byte BCD game number.
  3) First byte is sent with the SAS wakeup 9th bit = 1; remaining bytes = 0.
  4) Keeps the supplied SASTEST 2 ms wakeup delay between address and command.
     This remains inside the SAS <=5 ms inter-byte requirement.
  5) Arduino/RFID amount entry is 0..99999. The old >100 reset is removed.
  6) V6.3 enables Legacy Bonus 0x8A only for RFID deposit transactions with ACK + exact LP1A post-meter verification.
  7) V6.0 auto-detects the SAS profile and remains diagnostic/interrogation only for value movement.
  8) A 20/20 LP1A stability pass automatically saves the verified profile to EEPROM.
  9) On the next boot, the cached profile is loaded immediately, but LP54 verifies the
     connected S6000 serial/version before the cache is trusted.
 10) Startup retries several times. If the S6000 is still booting/offline, the Arduino
     continues retrying every few seconds forever. Link heartbeat also detects later
     S6000 reboot/disconnect and returns to AUTO RETRY.
 11) Two alternating EEPROM profile slots + CRC32 are used so a power loss during a profile save does
     not destroy the last known-good profile.
 12) V6 includes a non-blocking General Poll event queue/state engine.
 13) Priority exceptions are serviced without waiting for the normal 200 ms poll slot:
       51 -> LP1B handpay information
       57 -> LP57 pending system-validation cashout information
       67 -> LP70 ticket validation data + LP71 FF status interrogation
       68 -> LP71 FF final/current ticket status interrogation
       3D -> LP3D standard ticket info OR LP4D FF enhanced/system look-ahead
       69 -> AFT-complete observation
       6F -> game-lock observation
 14) LP1F is read on every live connection to learn the SAS accounting denomination.
 15) RFID/keypad amount # performs preflight, then (when enabled) a guarded Legacy Bonus 0x8A deposit:
       PRE LP1A -> RAM PREPARED transaction -> 0x8A -> ACK observation ->
       POST LP1A exact delta verification -> same-card RFID balance commit.
     Any ambiguous failure leaves RFID unchanged and marks RECOVERY REQUIRED.
 16) Final V6.5 money mode enables only the proven value paths used here:
     Legacy Bonus 0x8A for deposits/restores and AFT LP72 machine->host full cashout.
     LP71 ticket-value authorization, A8 and LP94 remain disabled.
 17) V6.5.14 keeps transaction/recovery state in RAM only. Clear no-movement failures automatically; power reset forgets in-flight state.
 18) V6 adds RFID atomic-commit primitives: SAS/meter verification must complete before
     a card balance can be committed, and the same card UID must still be present.
 19) V6 adds LP72 FE/FF AFT interrogation state tracking. Value-changing LP72 initiate
     messages remain intentionally disabled in this validation build.
 20) V6 adds Ticket test-state tracking using 67->70->71(FF)->68 and exact SAS status
     decoding. Value-authorizing LP71 transfer codes are not transmitted by this build.
 21) V6 adds a Handpay-94 eligibility gate based on LP1B Reset ID/capability, but does
     not transmit LP94/A8, so S6000 upper-jackpot/credit-limit logic is not bypassed.
 22) V6 reads selected LP2F meters before/after ticket/AFT events for independent proof.
 23) V6 stores up to eight verified machine/address profiles and can scan SAS 1..127.
 24) Minima build keeps WiFi diagnostics compiled out; USB Serial remains the service console.
 25) V6 boot protocol self-test, trace ring, recovery interlocks and fail-closed health
     status are included.

  HIDDEN TEST MODE (NO CARD ONLY)
  -------------------------------
    Hold # for 2 seconds = enter TEST MODE
    Hold # for 2 seconds again = exit TEST MODE
    Presenting an RFID card also exits TEST MODE immediately.

    Inside TEST MODE only:
      1 = AUTO profile/status summary
      2 = LP1A current credits
      3 = cached A4/74 limit summary
      4 = re-probe LPA0 enabled features
      5 = LP74 AFT status (only when applicable)
      6 = LP1A 20-cycle communication stability test
      7 = reconnect/verify fixed SAS address 1
      8 = LP54 version/serial
      9 = V6 event/RAM transaction/handpay-gate detail
      0 = LP1F machine info / accounting denomination
      * = communication counters / last event
      # = clear last event

  CARD KEYS
  ---------
    0..9 = amount entry, max 99999
    * short once  = arm DEPOSIT confirmation (NO money moves)
    * short again = execute RFID -> S6000 deposit
    # short once  = arm CASHOUT confirmation (NO money moves)
    # short again = execute desired S6000 -> RFID split cashout
    hold 0 for 1 sec = CLEAR amount + cancel confirmation (NO money moves)
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <stddef.h>

#include "BALLY_S6000_UNO_R4_V6_CONFIG.h"
#if V6_ENABLE_WIFI_DIAGNOSTICS
#include <WiFiS3.h>
#endif


// ============================================================================
// HARDWARE
// ============================================================================
#define RFID_RST_PIN       9
#define RFID_SS_PIN       10
#define LED_BLUE           4
#define LED_RED            5
#define LED_GREEN          7
#define ANALOG_KEYPAD_PIN A0

LiquidCrystal_I2C lcd(0x27, 16, 2);
MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);
MFRC522::MIFARE_Key keyA;

// ============================================================================
// RFID / USER VALUES
// ============================================================================
static const byte CARD_BLOCK = 4;
static const uint32_t DEFAULT_BALANCE = 1000UL;
static const uint32_t MAX_INPUT_AMOUNT = 99999UL;

struct CardBlockData {
  uint32_t bal;
  uint32_t nbal;
  char tag[4];
  uint8_t crc8;
  uint8_t pad[3];
};

static_assert(sizeof(CardBlockData) == 16, "CardBlockData must be 16 bytes");

bool cardActive = false;
uint32_t cardBalance = 0;
uint32_t inputValue = 0;
uint32_t lastCardCheckMs = 0;

// V6.5.9 money-key safety: first * or # only arms a confirmation.
// No value-changing SAS command is sent until the SAME key is pressed again.
enum class MoneyConfirmMode : uint8_t { NONE = 0, DEPOSIT = 1, CASHOUT = 2 };
MoneyConfirmMode moneyConfirmMode = MoneyConfirmMode::NONE;
uint32_t moneyConfirmAmount = 0;
uint32_t moneyConfirmUntilMs = 0;
static const uint32_t MONEY_CONFIRM_TIMEOUT_MS = 5000UL;

// V6.5.9 operator diagnostics gate.
// Diagnostics are hidden during normal no-card operation and are entered only
// by holding the analog-keypad # key for 2 seconds with NO RFID card active.
bool keypadTestMode = false;

// V6.5.9: continuous USB Serial keypad ADC monitor.
// In TEST MODE this is automatically active and prints the actual A0 median
// even when no key can be decoded (important for diagnosing key 5).
bool keypadSerialAdcLive = false;
uint32_t keypadSerialAdcLastMs = 0;
static const uint16_t KEYPAD_SERIAL_ADC_PERIOD_MS = 100;

// V6.5.14 optional ADC calibration mode.  This mode is entered ONLY from
// USB Serial with the command "adctest", so it does not depend on any keypad
// decoding table.  While active, A0 is switched to 12-bit resolution and the
// normal keypad money/test handling is suspended.  A single line is printed
// only once per physical press; no-key samples are silent.
bool keypadAdcCalibrationMode = false;
bool keypadAdcRawStream = false;
uint16_t keypadAdcCalBaseline = 4095;
uint16_t keypadAdcCalStableLevel = 4095;
uint16_t keypadAdcCalCandidateLevel = 4095;
bool keypadAdcCalCandidate = false;
uint32_t keypadAdcCalCandidateSinceMs = 0;
uint32_t keypadAdcRawLastMs = 0;
// Calibration deliberately does NOT know any key ADC table.  It observes only
// stable 12-bit voltage levels.  This avoids the circular failure where an
// unknown key must first be decoded before its ADC value can be displayed.
static const uint8_t KEYPAD_ADC_CAL_SAMPLES = 33;
static const uint16_t KEYPAD_ADC_CAL_CHANGE_COUNTS = 3;
static const uint16_t KEYPAD_ADC_CAL_CANDIDATE_BAND = 4;
static const uint16_t KEYPAD_ADC_CAL_RELEASE_MARGIN = 7;
static const uint16_t KEYPAD_ADC_CAL_STABLE_MS = 55;
static const uint16_t KEYPAD_ADC_RAW_PERIOD_MS = 100;

uint8_t activeCardUid[10] = {};
uint8_t activeCardUidLen = 0;

// ============================================================================
// SAS CONSTANTS
// ============================================================================
uint8_t sasAddress = 0x01;
static const uint32_t SAS_BAUD = 19200UL;
static const uint32_t SAS_POLL_INTERVAL_MS = 50UL;
static const uint32_t SAS_FIRST_BYTE_TIMEOUT_US = 1500000UL;
static const uint32_t SAS_PACKET_GAP_US = 20000UL;
static const uint32_t SAS_WAKEUP_DELAY_US = 2000UL;

// ============================================================================
// PROVEN TRUE 9-BIT SERIAL1 TRANSPORT FOR UNO R4 MINIMA D0/D1
// Baseline: UNO_R4_MINIMA_SAS_9BIT_REAL_COMM_TEST.ino
// ============================================================================
class Sas9Uart {
public:
  bool begin(uint32_t baudrate) {
    Serial1.begin(baudrate, SERIAL_9N1);
    Serial1.setTimeout(200);
    delay(100);
    clearRx();
    return true;
  }

  bool writeWords(uint16_t *words, size_t count) {
    if (words == nullptr || count == 0) return false;
    const size_t bytes = count * sizeof(uint16_t);
    const size_t written = Serial1.write((uint8_t *)words, bytes);
    return written == bytes;
  }

  bool readWord(uint16_t &word) {
    if (Serial1.available() <= 0) return false;
    const int r = Serial1.read();
    if (r < 0) return false;
    word = (uint16_t)((uint8_t)r);
    return true;
  }

  void clearRx() {
    // Exact working REAL_COMM_TEST behavior: drain all bytes currently queued.
    while (Serial1.available() > 0) Serial1.read();
  }

  uint32_t errorCount() const { return 0; }
  uint32_t overflowCount() const { return 0; }
};

Sas9Uart sasUart;

// ============================================================================
// SAS STATS / STATE
// ============================================================================
uint32_t gpSent = 0;
uint32_t gpResponses = 0;
uint32_t gpEvents = 0;
uint32_t lpOk = 0;
uint32_t lpTimeout = 0;
uint32_t lpCrcFail = 0;
uint32_t lpFormatFail = 0;
uint8_t lastEvent = 0xFF;
uint32_t lastPollTxMs = 0;

bool a0Known = false;
bool a0AftSupported = false;
bool a0TicketRedemption = false;
bool a0FortyMsSupported = false;
uint8_t a0ValidationStyle = 0;

// ============================================================================
// AUTO COMPATIBILITY PROFILE
// S6000 Option 17 exposes SAS 5.01 or SAS 6.01. The profile code also
// recognizes intermediate SAS revisions so this host core can be reused on
// compatible machines without hard-coding one protocol version.
// ============================================================================
enum class SasProfile : uint8_t {
  UNKNOWN = 0,
  SAS_PRE_500,
  SAS_500_501,
  SAS_502,
  SAS_510,
  SAS_600,
  SAS_601_PLUS
};

struct SasCapabilities {
  bool versionKnown = false;
  uint16_t versionCode = 0;       // 501 => SAS 5.01, 601 => SAS 6.01
  String versionText;
  String machineSerial;
  SasProfile profile = SasProfile::UNKNOWN;

  // Features expected from protocol revision (not the same as enabled).
  bool ticketProtocolByVersion = false;     // 70/71 introduced by SAS 5.00 family
  bool systemValidationByVersion = false;   // 5.01 extension
  bool remoteHandpayByVersion = false;      // 94 in SAS 5.00 family
  bool multiDenomByVersion = false;         // 5.02 additions
  bool aftProtocolByVersion = false;        // AFT added in SAS 5.10

  // Runtime probes from this exact gaming machine.
  bool a0Responded = false;
  bool ticketRedemptionEnabled = false;
  bool validationExtensions = false;
  uint8_t validationStyle = 0;
  bool aftEnabled = false;
  bool fortyMsSupported = false;
  bool a4Responded = false;
  uint64_t cashOutLimitCredits = 0;
  bool lp74Responded = false;
  uint64_t aftTransferLimitCents = 0;

  // LP74 AFT state used by the guarded machine->RFID cashout path.
  uint8_t aftAssetNumber[4] = {0, 0, 0, 0};
  uint8_t aftGameLockStatus = 0xFF;
  uint8_t aftAvailableTransfers = 0;
  uint8_t aftHostCashoutStatus = 0;
  uint8_t aftStatusFlags = 0;
  uint8_t aftMaxBufferIndex = 0;
  uint64_t aftCashableMinorUnits = 0;
  uint64_t aftRestrictedMinorUnits = 0;
  uint64_t aftNonrestrictedMinorUnits = 0;
};



SasCapabilities sasCaps;

// ============================================================================
// V5 EVENT / TRANSACTION OBSERVER STATE
// ============================================================================
enum class V5RuntimeState : uint8_t {
  OFFLINE = 0,
  NORMAL,
  HANDPAY_PENDING,
  SYSTEM_VALIDATION_PENDING,
  TICKET_IN_ESCROW,
  TICKET_STATUS,
  TICKET_PRINTED,
  AFT_COMPLETE_SEEN,
  GAME_LOCKED,
  ERROR_STATE
};

struct MachineInfoSnapshot {
  bool valid = false;
  uint8_t denominationCode = 0;
  uint32_t accountingDenomMinorUnits = 0;
  uint8_t maxBetCredits = 0;
  String gameId;
  String additionalId;
  String paytableId;
};

struct HandpaySnapshot {
  bool valid = false;
  uint8_t progressiveGroup = 0;
  uint8_t level = 0;
  uint64_t amount = 0;
  uint64_t partialPay = 0;
  uint8_t resetId = 0;
};

struct PendingCashoutSnapshot {
  bool valid = false;
  uint8_t cashoutType = 0x80;
  uint64_t amountMinorUnits = 0;
};

struct TicketInSnapshot {
  bool valid = false;
  uint8_t ticketStatus = 0xFF;
  uint64_t amountMinorUnits = 0;
  uint8_t parsingCode = 0xFF;
  uint8_t validationLen = 0;
  uint8_t validationData[32] = {};
  uint8_t machineStatus = 0xFF;
  uint64_t machineTransferMinorUnits = 0;
};

struct PrintedTicketSnapshot {
  bool valid = false;
  bool enhanced = false;
  uint8_t validationType = 0;
  uint8_t bufferIndex = 0;
  uint64_t amountMinorUnits = 0;
  uint16_t ticketNumber = 0xFFFF;
};

V5RuntimeState v5State = V5RuntimeState::OFFLINE;
MachineInfoSnapshot machineInfo;
HandpaySnapshot lastHandpay;
PendingCashoutSnapshot pendingCashout;
TicketInSnapshot ticketIn;
PrintedTicketSnapshot printedTicket;

static const uint8_t V5_EVENT_QUEUE_SIZE = 16;
uint8_t v5EventQueue[V5_EVENT_QUEUE_SIZE] = {};
uint8_t v5EventHead = 0;
uint8_t v5EventTail = 0;
uint8_t v5EventCount = 0;
uint32_t v5EventDropped = 0;
uint8_t v5LastQueuedEvent = 0xFF;
uint32_t v5LastQueuedEventMs = 0;
uint32_t v5LastEventProcessedMs = 0;
uint32_t v5EventScreenUntilMs = 0;

static const bool CARD_UNIT_EQUALS_SAS_ACCOUNTING_CREDIT = true;

// ============================================================================
// EEPROM VERIFIED PROFILE CACHE
// UNO R4 WiFi exposes 8 KB of data flash through EEPROM.h.
// Two slots are used as a tiny journal. We only save after a clean 20/20
// communication stability pass, so normal polling does NOT wear the EEPROM.
// ============================================================================
static const uint32_t EEPROM_PROFILE_MAGIC = 0x53365234UL; // ASCII-ish "S6R4"
static const uint16_t EEPROM_PROFILE_SCHEMA = 0x0401;
static const int EEPROM_SLOT0_ADDR = 0;
static const int EEPROM_SLOT1_ADDR = 256;
static const uint16_t STABILITY_TEST_TOTAL = 20;
static const uint16_t STABILITY_PASS_REQUIRED = 20;

// flags stored in EEPROM
static const uint32_t PF_VERSION_KNOWN        = (1UL << 0);
static const uint32_t PF_TICKET_BY_VERSION    = (1UL << 1);
static const uint32_t PF_SYSVAL_BY_VERSION    = (1UL << 2);
static const uint32_t PF_REMOTE94_BY_VERSION  = (1UL << 3);
static const uint32_t PF_MULTIDENOM_BY_VERSION= (1UL << 4);
static const uint32_t PF_AFT_BY_VERSION       = (1UL << 5);
static const uint32_t PF_A0_RESPONDED         = (1UL << 6);
static const uint32_t PF_TICKET_ENABLED       = (1UL << 7);
static const uint32_t PF_VALIDATION_EXT       = (1UL << 8);
static const uint32_t PF_AFT_ENABLED          = (1UL << 9);
static const uint32_t PF_40MS_SUPPORTED       = (1UL << 10);
static const uint32_t PF_A4_RESPONDED         = (1UL << 11);
static const uint32_t PF_74_RESPONDED         = (1UL << 12);
static const uint32_t PF_STABILITY_PASSED     = (1UL << 13);

struct StoredSasProfile {
  uint32_t magic;
  uint16_t schema;
  uint16_t recordSize;
  uint32_t generation;

  uint16_t versionCode;
  uint8_t profile;
  uint8_t validationStyle;

  uint16_t stabilityGood;
  uint16_t stabilityTotal;
  uint32_t flags;

  uint64_t cashOutLimitCredits;
  uint64_t aftTransferLimitCents;

  char versionText[8];
  char machineSerial[24];

  uint32_t crc32;
};

static_assert(sizeof(StoredSasProfile) < 256, "EEPROM profile record too large");


StoredSasProfile savedProfile = {};
bool savedProfileValid = false;
int savedProfileSlot = -1;

// ============================================================================
// V6 EEPROM PROFILE MAP + RAM-ONLY TRANSACTION STATE
// UNO R4 WiFi EEPROM emulation uses 8 KiB data flash in the target core.
// 0..511      : legacy V4/V5 dual verified-profile cache
// 512..1535   : eight V6 machine/address profiles, 128-byte stride
// 2048..8191  : unused/reserved by V6.5.9 transaction logic; never read/written for transactions
// remaining   : reserved; runtime transactions are RAM-only
// ============================================================================
static const uint32_t V6_PROFILE_MAGIC = 0x56365052UL; // V6PR
static const uint16_t V6_PROFILE_SCHEMA = 0x0600;
static const int V6_PROFILE_DB_BASE = 512;
static const int V6_PROFILE_DB_SLOTS = 8;
static const int V6_PROFILE_DB_STRIDE = 128;

// RAM transaction record tags. These are NOT EEPROM addresses/formats in V6.5.9.
static const uint32_t V6_JOURNAL_MAGIC = 0x56364A52UL; // historical tag, RAM only
static const uint16_t V6_JOURNAL_SCHEMA = 0x0600;
static const int V6_RAM_TX_RECORD_MAX = 160;
static const int V6_EEPROM_BYTES = 8192; // profile storage capacity only

#pragma pack(push, 1)
struct V6ProfileRecord {
  uint32_t magic;
  uint16_t schema;
  uint16_t recordSize;
  uint32_t generation;
  uint8_t address;
  uint8_t profile;
  uint8_t validationStyle;
  uint8_t denominationCode;
  uint16_t versionCode;
  uint16_t stabilityGood;
  uint16_t stabilityTotal;
  uint32_t flags;
  uint32_t accountingDenomMinorUnits;
  uint64_t cashOutLimitCredits;
  uint64_t aftTransferLimitMinorUnits;
  char versionText[8];
  char machineSerial[24];
  uint32_t crc32;
};

struct V6MeterSnapshotStored {
  uint32_t currentCredits;
  uint32_t ticketInCredits;
  uint32_t ticketOutCredits;
  uint32_t transferToGameCredits;
  uint32_t transferToHostCredits;
};

enum class V6JournalType : uint8_t {
  NONE = 0,
  TICKET_IN = 1,
  TICKET_OUT = 2,
  AFT = 3,
  HANDPAY = 4,
  RFID_ATOMIC = 5,
  RFID_CASHOUT = 6,
  RFID_SPLIT_CASHOUT = 7
};

enum class V6JournalState : uint8_t {
  EMPTY = 0,
  OBSERVED = 1,
  PREPARED = 2,
  PENDING = 3,
  SAS_VERIFIED = 4,
  METERS_VERIFIED = 5,
  RFID_COMMITTED = 6,
  DONE = 7,
  FAILED = 8,
  RECOVERY_REQUIRED = 9
};

struct V6JournalRecord {
  uint32_t magic;
  uint16_t schema;
  uint16_t recordSize;
  uint32_t generation;
  uint8_t type;
  uint8_t state;
  uint8_t address;
  uint8_t sasStatus;
  uint8_t cardUidLen;
  uint8_t cardUid[10];
  uint32_t amountCredits;
  uint64_t amountMinorUnits;
  uint32_t balanceBefore;
  uint32_t balanceAfter;
  V6MeterSnapshotStored beforeMeters;
  V6MeterSnapshotStored afterMeters;
  char transactionId[16];
  char machineSerial[16];
  uint32_t crc32;
};
#pragma pack(pop)

static_assert(sizeof(V6ProfileRecord) <= V6_PROFILE_DB_STRIDE, "V6 profile too large");
static_assert(sizeof(V6JournalRecord) <= V6_RAM_TX_RECORD_MAX, "V6 RAM transaction record too large");
static_assert(V6_PROFILE_DB_BASE + V6_PROFILE_DB_SLOTS * V6_PROFILE_DB_STRIDE <= V6_EEPROM_BYTES,
              "V6 profile EEPROM map exceeds UNO R4 EEPROM space");

struct V6MeterSnapshot {
  bool valid = false;
  uint64_t currentCredits = 0;
  uint64_t ticketInCredits = 0;
  uint64_t ticketOutCredits = 0;
  uint64_t transferToGameCredits = 0;
  uint64_t transferToHostCredits = 0;
};

struct V6AftSnapshot {
  bool valid = false;
  uint8_t bufferPosition = 0;
  uint8_t transferStatus = 0xFF;
  uint8_t receiptStatus = 0xFF;
  uint8_t transferType = 0xFF;
  uint64_t cashableMinorUnits = 0;
  uint64_t restrictedMinorUnits = 0;
  uint64_t nonrestrictedMinorUnits = 0;
};

V6ProfileRecord v6ProfileDb[V6_PROFILE_DB_SLOTS] = {};
bool v6ProfileDbValid[V6_PROFILE_DB_SLOTS] = {};
int v6MostRecentProfileSlot = -1;

// Runtime transaction record only. Never persisted to EEPROM in V6.5.9.
V6JournalRecord v6LatestJournal = {};
bool v6LatestJournalValid = false;
int v6LatestJournalSlot = -1; // retained only for source compatibility; always -1
bool v6RecoveryRequired = false;
uint32_t v6RuntimeTxGeneration = 0;
uint32_t v6AftTxSequence = 0;
uint32_t v6NextRecoveryMs = 0;

V6MeterSnapshot v6BeforeMeters;
V6MeterSnapshot v6AfterMeters;
uint8_t v6BeforeMeterContext = 0;
V6AftSnapshot v6LastAft;

bool v6BootSelfTestPassed = false;
bool v6LastMeterVerifyPassed = false;
uint32_t v6LastAddressScanMs = 0;
uint32_t v6LastWebAttemptMs = 0;
uint64_t v6LastCurrentCredits = 0;

struct V6TraceEntry {
  uint32_t ms;
  uint8_t kind;
  uint8_t code;
  uint16_t detail;
};
static const uint8_t V6_TRACE_SIZE = 24;
V6TraceEntry v6Trace[V6_TRACE_SIZE] = {};
uint8_t v6TraceHead = 0;

#if V6_ENABLE_WIFI_DIAGNOSTICS
WiFiServer v6WebServer(80);
bool v6WebStarted = false;
#endif

// ============================================================================
// BOOT / LINK RETRY STATE
// Arduino may power before or after the S6000. Therefore startup never assumes
// the machine is ready. After the fast startup attempts, retry continues forever.
// ============================================================================
static const uint8_t BOOT_FAST_RETRIES = 3;
static const uint16_t BOOT_FAST_RETRY_DELAY_MS = 500;
static const uint32_t BACKGROUND_RETRY_MS = 5000UL;
static const uint32_t LINK_HEARTBEAT_MS = 15000UL;
static const uint8_t LINK_HEARTBEAT_FAIL_LIMIT = 3;

bool sasOnline = false;
bool cachedIdentityMatched = false;
uint8_t reconnectAttempt = 0;
uint8_t heartbeatFailCount = 0;
uint32_t nextReconnectMs = 0;
uint32_t nextHeartbeatMs = 0;

// ============================================================================
// UI
// ============================================================================
void setColor(bool blue, bool red, bool green) {
  digitalWrite(LED_BLUE,  blue  ? LOW : HIGH);
  digitalWrite(LED_RED,   red   ? LOW : HIGH);
  digitalWrite(LED_GREEN, green ? LOW : HIGH);
}

void lcdLine(uint8_t row, const String &text) {
  lcd.setCursor(0, row);
  String s = text;
  if (s.length() > 16) s.remove(16);
  lcd.print(s);
  for (uint8_t i = s.length(); i < 16; i++) lcd.print(' ');
}

void showTestModeMenu() {
  lcd.clear();
  lcdLine(0, "TEST HOLD# EXIT");
  lcdLine(1, "0-9 *CNT #CLR");
  setColor(false, true, true);
}

void showIdle() {
  lcd.clear();

  // Normal operators must not see or accidentally invoke diagnostics.
  // TEST MODE is only reachable with no card by holding # for 2 seconds.
  if (keypadTestMode && !cardActive) {
    showTestModeMenu();
    return;
  }

  if (!sasOnline) {
    String cache = savedProfileValid ? String(savedProfile.versionText) : String("NONE");
    lcdLine(0, String("S6000 WAIT C:") + cache);
    lcdLine(1, "AUTO RETRY...");
    setColor(false, true, false);
    return;
  }

  switch (v5State) {
    case V5RuntimeState::HANDPAY_PENDING:
      lcdLine(0, "HANDPAY PENDING");
      lcdLine(1, "9=DETAIL");
      setColor(false, true, false);
      return;
    case V5RuntimeState::SYSTEM_VALIDATION_PENDING:
      lcdLine(0, "VALIDATION REQ");
      lcdLine(1, "9=DETAIL");
      setColor(false, true, true);
      return;
    case V5RuntimeState::TICKET_IN_ESCROW:
    case V5RuntimeState::TICKET_STATUS:
      lcdLine(0, "TICKET IN");
      lcdLine(1, "9=STATUS");
      setColor(false, false, true);
      return;
    case V5RuntimeState::TICKET_PRINTED:
      lcdLine(0, "TICKET PRINTED");
      lcdLine(1, "9=DETAIL");
      setColor(false, false, true);
      return;
    case V5RuntimeState::GAME_LOCKED:
      lcdLine(0, "GAME LOCKED");
      lcdLine(1, "9=DETAIL");
      setColor(false, true, false);
      return;
    default:
      break;
  }

  String v = sasCaps.versionKnown ? sasCaps.versionText : String("?");
  lcdLine(0, String("SAS ") + v + " ONLINE");
  lcdLine(1, "CARD / HOLD#TEST");
  setColor(true, false, false);
}

void showCard() {
  lcd.clear();
  lcdLine(0, String("BAL:") + cardBalance + " H0=CLR");
  lcdLine(1, String("A:") + inputValue + " *IN #OUT");
  setColor(false, false, true);
}

void showAmount() {
  lcdLine(1, String("A:") + inputValue + " *IN #OUT");
}

void cancelMoneyConfirm(bool clearAmount = false) {
  moneyConfirmMode = MoneyConfirmMode::NONE;
  moneyConfirmAmount = 0;
  moneyConfirmUntilMs = 0;
  if (clearAmount) inputValue = 0;
}

void showMoneyConfirm(MoneyConfirmMode mode, uint32_t amount) {
  lcd.clear();
  if (mode == MoneyConfirmMode::DEPOSIT) {
    lcdLine(0, String("IN:") + amount + " CONFIRM");
    lcdLine(1, "*=OK H0=CLR");
  } else if (mode == MoneyConfirmMode::CASHOUT) {
    lcdLine(0, String("OUT:") + amount + " CONFIRM");
    lcdLine(1, "#=OK H0=CLR");
  } else {
    showCard();
  }
}

void armMoneyConfirm(MoneyConfirmMode mode) {
  moneyConfirmMode = mode;
  moneyConfirmAmount = inputValue;
  moneyConfirmUntilMs = millis() + MONEY_CONFIRM_TIMEOUT_MS;
  showMoneyConfirm(mode, inputValue);
}

void showMessage(const String &line1, const String &line2, uint16_t holdMs = 1200) {
  lcd.clear();
  lcdLine(0, line1);
  lcdLine(1, line2);
  if (holdMs) delay(holdMs);
}

// ============================================================================
// ANALOG KEYPAD
// ============================================================================
struct AnalogKey {
  uint16_t center;
  int8_t key;
};

// V6.5.14: REAL 12-bit A0 values measured on the user's actual keypad.
// No extrapolated 10-bit values are used. Centers are the stable medians from
// the calibration log. Key codes: 11=#, 12=*.
const AnalogKey keyMap[] = {
  {1081, 1 }, {1817, 2 }, {3109, 3 }, {2677, 4 },
  {4095, 5 }, {3385, 6 }, {2356, 7 }, {2224, 8 },
  {1598, 9 }, {2000, 0 }, { 931, 11}, {1738, 12}
};

static const uint8_t KEY_COUNT = sizeof(keyMap) / sizeof(keyMap[0]);
// The nearest pair is *=1738 and 2=1817 (79 counts apart). The 7-sample
// median + 45 ms stability filter is retained; nearest-center decoding gives
// a midpoint at about 1777/1778. A 70-count acquisition window accepts the
// observed jitter while still rejecting the large no-key gap.
static const uint16_t KEY_MAX_DIFF = 70;
// This ladder is LOW when released: measured no-key median is about 13..18.
// The lowest real key is # at about 931, so <=400 is a generous release zone.
static const uint16_t KEYPAD_RELEASE_ADC = 400;
static const uint8_t KEY_MEDIAN_SAMPLES = 7;
static const uint16_t KEY_STABLE_MS = 45;
static const uint16_t KEY_LONG_AMOUNT_CLEAR_MS = 1000;
static const uint16_t KEY_LONG_TEST_MODE_MS = 2000;
static const uint16_t KEY_LONG_TX_SUPPRESS_MS = 800;

// Keep Arduino .ino auto-prototype generation safe: do not use a user-defined
// type in function signatures here.  Arduino may insert prototypes above local
// struct declarations, which caused V6.5.9 to fail with
// the V6.5.9 Arduino auto-prototype type-order compile failure.
void readKeypadAdcStats(uint16_t &minValue, uint16_t &medianValue, uint16_t &maxValue) {
  uint16_t s[KEY_MEDIAN_SAMPLES];
  for (uint8_t i = 0; i < KEY_MEDIAN_SAMPLES; i++) {
    s[i] = (uint16_t)analogRead(ANALOG_KEYPAD_PIN);
  }

  for (uint8_t i = 1; i < KEY_MEDIAN_SAMPLES; i++) {
    const uint16_t v = s[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && s[j] > v) {
      s[j + 1] = s[j];
      j--;
    }
    s[j + 1] = v;
  }

  minValue = s[0];
  medianValue = s[KEY_MEDIAN_SAMPLES / 2];
  maxValue = s[KEY_MEDIAN_SAMPLES - 1];
}

uint16_t readKeypadMedianAdc() {
  uint16_t minValue = 0, medianValue = 0, maxValue = 0;
  readKeypadAdcStats(minValue, medianValue, maxValue);
  return medianValue;
}

int8_t decodeAnalogKey(uint16_t adc) {
  // Actual hardware is near zero when no key is pressed.
  if (adc <= KEYPAD_RELEASE_ADC) return -1;

  int32_t bestDiff = 0x7FFFFFFF;
  int8_t bestKey = -1;

  for (uint8_t i = 0; i < KEY_COUNT; i++) {
    int32_t diff = (int32_t)adc - (int32_t)keyMap[i].center;
    if (diff < 0) diff = -diff;

    if (diff < bestDiff) {
      bestDiff = diff;
      bestKey = keyMap[i].key;
    }
  }

  if ((uint32_t)bestDiff > KEY_MAX_DIFF) return -1;
  return bestKey;
}

// Return values:
//   0..9  digit press (0 is delivered on release so long-0 can be CLEAR)
//   11    # short press/release
//   12    * short press/release
//   13    hold 0 >= 1 sec => CLEAR amount + cancel confirmation
//   14    hold # >= 2 sec => TEST MODE toggle (loop accepts only with no card)
//
// Safety rules:
// - # and * are release-gated. Holding a transaction key never executes money.
// - # held >=2 s becomes TEST toggle, never a short cashout key.
// - * held >=0.8 s is suppressed (no transaction, no CLEAR).
// - 0 short is a normal digit; 0 held >=1 s clears the entered amount.
int8_t readKeyEvent() {
  static int8_t candidate = -1;
  static uint32_t candidateSince = 0;
  static int8_t activeKey = -1;
  static uint32_t activeSince = 0;

  const uint32_t now = millis();
  const uint16_t adc = readKeypadMedianAdc();
  const int8_t raw = decodeAnalogKey(adc);

  if (activeKey < 0) {
    if (raw < 0) {
      candidate = -1;
      candidateSince = 0;
      return -1;
    }

    if (raw != candidate) {
      candidate = raw;
      candidateSince = now;
      return -1;
    }

    if ((uint32_t)(now - candidateSince) < KEY_STABLE_MS) return -1;

    activeKey = raw;
    activeSince = now;
    candidate = -1;
    candidateSince = 0;

    // 1..9 can be delivered immediately.  0 must wait for release so a
    // sustained zero can be distinguished from the CLEAR gesture.
    if (activeKey >= 1 && activeKey <= 9) return activeKey;
    return -1;
  }

  // A held key produces no repeated events.  Wait for a physical release.
  if (raw >= 0) return -1;

  const int8_t releasedKey = activeKey;
  const uint32_t heldMs = (uint32_t)(now - activeSince);
  activeKey = -1;
  activeSince = 0;

  if (releasedKey == 0) {
    if (heldMs >= KEY_LONG_AMOUNT_CLEAR_MS) return 13;
    return 0;
  }

  if (releasedKey == 11) { // #
    if (heldMs >= KEY_LONG_TEST_MODE_MS) return 14;
    if (heldMs >= KEY_LONG_TX_SUPPRESS_MS) return -1;
    return 11;
  }

  if (releasedKey == 12) { // *
    if (heldMs >= KEY_LONG_TX_SUPPRESS_MS) return -1;
    return 12;
  }

  return -1;
}

void printKeypadAdcLine(uint16_t minValue, uint16_t medianValue, uint16_t maxValue) {
  const int8_t key = decodeAnalogKey(medianValue);

  // Requested simple format: "4 - 300", "6 - 700".
  // When the ladder value is not currently mapped to a key, still print the
  // ADC so a missing key (especially 5) can be calibrated from real hardware.
  if (key == 11) Serial.print('#');
  else if (key == 12) Serial.print('*');
  else if (key >= 0 && key <= 9) Serial.print(key);
  else if (medianValue <= KEYPAD_RELEASE_ADC) Serial.print("UP");
  else Serial.print('?');

  Serial.print(" - ");
  Serial.print(medianValue);

  // Min/max makes resistor/ADC jitter visible without changing the simple
  // key-value prefix. Example: 4 - 665 [662..668]
  Serial.print(" [");
  Serial.print(minValue);
  Serial.print("..");
  Serial.print(maxValue);
  Serial.println("]");
}

void printKeypadAdcDiagnostic() {
  uint16_t minValue = 0, medianValue = 0, maxValue = 0;
  readKeypadAdcStats(minValue, medianValue, maxValue);
  printKeypadAdcLine(minValue, medianValue, maxValue);
}

void serviceKeypadSerialAdcMonitor() {
  if (!(keypadTestMode || keypadSerialAdcLive)) return;
  const uint32_t now = millis();
  if ((uint32_t)(now - keypadSerialAdcLastMs) < KEYPAD_SERIAL_ADC_PERIOD_MS) return;
  keypadSerialAdcLastMs = now;

  uint16_t minValue = 0, medianValue = 0, maxValue = 0;
  readKeypadAdcStats(minValue, medianValue, maxValue);
  printKeypadAdcLine(minValue, medianValue, maxValue);
}


void readKeypadAdcCalStats12(uint16_t &minValue, uint16_t &medianValue, uint16_t &maxValue) {
  uint16_t s[KEYPAD_ADC_CAL_SAMPLES];
  for (uint8_t i = 0; i < KEYPAD_ADC_CAL_SAMPLES; i++) {
    s[i] = (uint16_t)analogRead(ANALOG_KEYPAD_PIN);
  }
  for (uint8_t i = 1; i < KEYPAD_ADC_CAL_SAMPLES; i++) {
    const uint16_t v = s[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && s[j] > v) {
      s[j + 1] = s[j];
      j--;
    }
    s[j + 1] = v;
  }
  minValue = s[0];
  medianValue = s[KEYPAD_ADC_CAL_SAMPLES / 2];
  maxValue = s[KEYPAD_ADC_CAL_SAMPLES - 1];
}

uint16_t calibrateKeypadAdcNoKeyBaseline12() {
  // Caller must ensure no key is held.  Take multiple 7-sample medians and
  // select the median of those medians to reject occasional ADC spikes.
  static const uint8_t N = 21;
  uint16_t v[N];
  for (uint8_t i = 0; i < N; i++) {
    uint16_t mn = 0, md = 0, mx = 0;
    readKeypadAdcCalStats12(mn, md, mx);
    v[i] = md;
    delay(3);
  }
  for (uint8_t i = 1; i < N; i++) {
    const uint16_t x = v[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && v[j] > x) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = x;
  }
  return v[N / 2];
}

void enterKeypadAdcCalibrationMode() {
  keypadTestMode = false;
  keypadSerialAdcLive = false;
  keypadAdcCalibrationMode = true;
  keypadAdcRawStream = false;
  keypadAdcCalCandidate = false;
  keypadAdcCalCandidateSinceMs = 0;
  keypadAdcRawLastMs = 0;

  analogReadResolution(12);
  delay(40);
  keypadAdcCalBaseline = calibrateKeypadAdcNoKeyBaseline12();
  keypadAdcCalStableLevel = keypadAdcCalBaseline;
  keypadAdcCalCandidateLevel = keypadAdcCalBaseline;

  Serial.println();
  Serial.println("=== 12-BIT KEYPAD ADC STABLE-CHANGE TEST ===");
  Serial.print("NO-KEY BASELINE = "); Serial.println(keypadAdcCalBaseline);
  Serial.println("No key table is used. Stable ADC level changes are measured directly.");
  Serial.println("Press ONE key, hold briefly, release, then press the next key.");
  Serial.println("A stable pressed level prints once; release is silent.");
  Serial.println("Format: ADC=xxxx [min..max] BASE=xxxx DROP=n");
  Serial.println("Type adcraw for unconditional 100ms raw/median stream.");
  Serial.println("Type adcexit to return to normal 12-bit keypad operation.");
  showMessage("ADC TEST 12BIT", "SERIAL STABLE", 350);
}

void exitKeypadAdcCalibrationMode() {
  keypadAdcCalibrationMode = false;
  keypadAdcRawStream = false;
  keypadAdcCalCandidate = false;
  keypadAdcCalCandidateSinceMs = 0;
  keypadAdcRawLastMs = 0;
  analogReadResolution(12);
  Serial.println("ADC TEST: EXIT -> normal 12-bit keypad operation.");
  if (cardActive) showCard(); else showIdle();
}

void setKeypadAdcRawStream(bool enable) {
  if (!keypadAdcCalibrationMode) enterKeypadAdcCalibrationMode();
  keypadAdcRawStream = enable;
  keypadAdcRawLastMs = 0;
  Serial.print("ADC RAW STREAM: ");
  Serial.println(enable ? "ON (100ms, unconditional)" : "OFF (stable-change mode)");
}

void serviceKeypadAdcCalibrationMode() {
  if (!keypadAdcCalibrationMode) return;

  uint16_t minValue = 0, medianValue = 0, maxValue = 0;
  readKeypadAdcCalStats12(minValue, medianValue, maxValue);
  const uint32_t now = millis();

  if (keypadAdcRawStream) {
    if ((uint32_t)(now - keypadAdcRawLastMs) < KEYPAD_ADC_RAW_PERIOD_MS) return;
    keypadAdcRawLastMs = now;
    Serial.print("RAW ADC="); Serial.print(medianValue);
    Serial.print(" ["); Serial.print(minValue); Serial.print(".."); Serial.print(maxValue); Serial.print("]");
    Serial.print(" BASE="); Serial.print(keypadAdcCalBaseline);
    Serial.print(" RISE=");
    Serial.println(medianValue >= keypadAdcCalBaseline ? medianValue - keypadAdcCalBaseline : 0);
    return;
  }

  const uint16_t diffFromStable = (medianValue >= keypadAdcCalStableLevel)
      ? (medianValue - keypadAdcCalStableLevel)
      : (keypadAdcCalStableLevel - medianValue);

  // No meaningful level change: cancel any pending candidate.  This path is
  // intentionally independent of the normal keypad decode table.
  if (diffFromStable < KEYPAD_ADC_CAL_CHANGE_COUNTS) {
    keypadAdcCalCandidate = false;
    keypadAdcCalCandidateSinceMs = 0;
    return;
  }

  const uint16_t diffFromCandidate = (medianValue >= keypadAdcCalCandidateLevel)
      ? (medianValue - keypadAdcCalCandidateLevel)
      : (keypadAdcCalCandidateLevel - medianValue);

  if (!keypadAdcCalCandidate || diffFromCandidate > KEYPAD_ADC_CAL_CANDIDATE_BAND) {
    keypadAdcCalCandidate = true;
    keypadAdcCalCandidateLevel = medianValue;
    keypadAdcCalCandidateSinceMs = now;
    return;
  }

  if ((uint32_t)(now - keypadAdcCalCandidateSinceMs) < KEYPAD_ADC_CAL_STABLE_MS) return;

  // Accept the new stable voltage plateau.  Near-baseline transitions are a
  // release and stay silent.  Any stable level below baseline is a key-like
  // plateau and is printed once, even if no key mapping exists yet.
  keypadAdcCalStableLevel = medianValue;
  keypadAdcCalCandidate = false;
  keypadAdcCalCandidateSinceMs = 0;

  if (medianValue <= KEYPAD_RELEASE_ADC) {
    // Release / no-key plateau on the real ladder is near zero.
    keypadAdcCalBaseline = medianValue;
    return;
  }

  Serial.print("ADC="); Serial.print(medianValue);
  Serial.print(" ["); Serial.print(minValue); Serial.print(".."); Serial.print(maxValue); Serial.print("]");
  Serial.print(" BASE="); Serial.print(keypadAdcCalBaseline);
  Serial.print(" RISE=");
  Serial.println(medianValue >= keypadAdcCalBaseline ? medianValue - keypadAdcCalBaseline : 0);
}

// ============================================================================
// RFID
// ============================================================================
uint8_t cardCrc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U)
                          : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

void setupRFID() {
  SPI.begin();
  mfrc522.PCD_Init();
  for (byte i = 0; i < 6; i++) keyA.keyByte[i] = 0xFF;
}

bool authenticateBlock(byte block) {
  MFRC522::StatusCode st = mfrc522.PCD_Authenticate(
    MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &keyA, &(mfrc522.uid)
  );
  return st == MFRC522::STATUS_OK;
}

bool readBalanceFromCard(uint32_t &balanceOut) {
  if (!authenticateBlock(CARD_BLOCK)) return false;
  byte buf[18];
  byte size = sizeof(buf);
  if (mfrc522.MIFARE_Read(CARD_BLOCK, buf, &size) != MFRC522::STATUS_OK) return false;

  CardBlockData bb;
  memcpy(&bb, buf, sizeof(bb));
  if (memcmp(bb.tag, "BAL1", 4) != 0) return false;
  if (bb.nbal != ~bb.bal) return false;
  if (bb.crc8 != cardCrc8(reinterpret_cast<uint8_t *>(&bb), 12)) return false;
  balanceOut = bb.bal;
  return true;
}

bool writeBalanceToCard(uint32_t balance) {
  if (!authenticateBlock(CARD_BLOCK)) return false;
  CardBlockData bb = {};
  bb.bal = balance;
  bb.nbal = ~balance;
  memcpy(bb.tag, "BAL1", 4);
  bb.crc8 = cardCrc8(reinterpret_cast<uint8_t *>(&bb), 12);
  return mfrc522.MIFARE_Write(CARD_BLOCK, reinterpret_cast<byte *>(&bb), 16)
         == MFRC522::STATUS_OK;
}

bool detectNewCard() {
  if (!mfrc522.PICC_IsNewCardPresent()) return false;
  if (!mfrc522.PICC_ReadCardSerial()) return false;

  uint32_t bal = 0;
  if (!readBalanceFromCard(bal)) {
    bal = DEFAULT_BALANCE;
    if (!writeBalanceToCard(bal)) return false;
  }

  cardBalance = bal;
  inputValue = 0;
  cancelMoneyConfirm(false);
  cardActive = true;
  activeCardUidLen = mfrc522.uid.size > sizeof(activeCardUid) ? sizeof(activeCardUid) : mfrc522.uid.size;
  for (uint8_t i = 0; i < activeCardUidLen; i++) activeCardUid[i] = mfrc522.uid.uidByte[i];
  return true;
}

bool checkCardStillReadable() {
  uint32_t dummy = 0;
  return readBalanceFromCard(dummy);
}

void closeCardSession() {
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  cardActive = false;
  cardBalance = 0;
  inputValue = 0;
  cancelMoneyConfirm(false);
  activeCardUidLen = 0;
  memset(activeCardUid, 0, sizeof(activeCardUid));
}

// ============================================================================
// SAS CRC16
// ============================================================================
uint16_t sasCrc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    const uint8_t c = data[i];
    uint16_t q = (uint16_t)((crc ^ c) & 0x0FU);
    crc = (uint16_t)((crc >> 4) ^ (q * 0x1081U));
    q = (uint16_t)((crc ^ (c >> 4)) & 0x0FU);
    crc = (uint16_t)((crc >> 4) ^ (q * 0x1081U));
  }
  return crc;
}

bool sasCheckResponseCrc(const uint8_t *data, size_t len) {
  if (len < 4) return false;
  const uint16_t crc = sasCrc16(data, len - 2);
  return data[len - 2] == (uint8_t)(crc & 0xFFU) &&
         data[len - 1] == (uint8_t)(crc >> 8);
}

// ============================================================================
// SAS RAW I/O
// ============================================================================
void waitForNormalPollSlot() {
  while ((uint32_t)(millis() - lastPollTxMs) < SAS_POLL_INTERVAL_MS) delay(1);
}

bool sasTransmitMessage(const uint8_t *bytes, size_t len) {
  if (bytes == nullptr || len == 0 || len > 64) return false;

  // EXACT REAL_COMM_TEST framing: all 9-bit words are sent in ONE bulk write.
  sasUart.clearRx();

  alignas(4) uint16_t words[64];
  for (size_t i = 0; i < len; i++) {
    words[i] = (uint16_t)bytes[i];
    if (i == 0) words[i] |= 0xFF00U; // address/wakeup bit ON
    else        words[i] |= 0xFC00U; // following bytes wakeup bit OFF
  }

  if (!sasUart.writeWords(words, len)) return false;
  lastPollTxMs = millis();
  return true;
}

size_t sasReceivePacket(uint8_t *rx, uint16_t *rx9, size_t maxLen) {
  if (rx == nullptr || maxLen == 0) return 0;

  const uint32_t startUs = micros();
  uint32_t lastByteUs = startUs;
  bool started = false;
  size_t count = 0;

  while ((uint32_t)(micros() - startUs) < SAS_FIRST_BYTE_TIMEOUT_US) {
    uint16_t word = 0;
    if (sasUart.readWord(word)) {
      const uint8_t b = (uint8_t)(word & 0xFFU);

      // Ignore GP/noise until the machine address starts a long-poll response.
      if (!started) {
        if (b != sasAddress) continue;
        started = true;
      }

      if (count < maxLen) {
        rx[count] = b;
        if (rx9) rx9[count] = word;
        count++;
      } else {
        return count;
      }

      lastByteUs = micros();

      // Full SAS long-poll frames terminate in CRC. Return immediately once valid.
      if (count >= 4 && sasCheckResponseCrc(rx, count)) return count;
    } else {
      const uint32_t now = micros();
      if (started && (uint32_t)(now - lastByteUs) >= SAS_PACKET_GAP_US) {
        // Handles one-byte address ACK as well as non-CRC short responses.
        return count;
      }
      delayMicroseconds(50);
    }
  }

  return count;
}

void dumpPacket(const char *tag, const uint8_t *data, const uint16_t *raw9, size_t len) {
  Serial.print(tag);
  Serial.print(" len=");
  Serial.print(len);
  Serial.print(" : ");
  for (size_t i = 0; i < len; i++) {
    if (raw9 && (raw9[i] & 0x0100U)) Serial.print('^');
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

size_t sasTypeR(uint8_t command, uint8_t *rx, uint16_t *rx9, size_t rxMax) {
  waitForNormalPollSlot();
  const uint8_t tx[2] = { sasAddress, command };
  if (!sasTransmitMessage(tx, sizeof(tx))) {
    lpFormatFail++;
    return 0;
  }

  const size_t got = sasReceivePacket(rx, rx9, rxMax);
  if (got == 0) {
    lpTimeout++;
    return 0;
  }
  dumpPacket("RX", rx, rx9, got);

  if (got >= 4 && !sasCheckResponseCrc(rx, got)) {
    lpCrcFail++;
    return got;
  }
  if (got >= 2 && rx[0] == sasAddress && rx[1] == command) lpOk++;
  else lpFormatFail++;
  return got;
}

size_t sasTypeS(uint8_t command, const uint8_t *payload, uint8_t payloadLen,
                uint8_t *rx, uint16_t *rx9, size_t rxMax) {
  if ((size_t)payloadLen + 4U > 64U) return 0;
  waitForNormalPollSlot();

  uint8_t tx[64];
  size_t n = 0;
  tx[n++] = sasAddress;
  tx[n++] = command;
  for (uint8_t i = 0; i < payloadLen; i++) tx[n++] = payload[i];
  const uint16_t crc = sasCrc16(tx, n);
  tx[n++] = (uint8_t)(crc & 0xFFU);
  tx[n++] = (uint8_t)(crc >> 8);

  Serial.print("TX cmd=0x");
  if (command < 0x10) Serial.print('0');
  Serial.print(command, HEX);
  Serial.print(" bytes=");
  Serial.println(n);

  if (!sasTransmitMessage(tx, n)) {
    lpFormatFail++;
    return 0;
  }

  const size_t got = sasReceivePacket(rx, rx9, rxMax);
  if (got == 0) {
    lpTimeout++;
    return 0;
  }
  dumpPacket("RX", rx, rx9, got);

  if (got == 1) {
    if (rx[0] == sasAddress) lpOk++;
    else lpFormatFail++;
    return got;
  }
  if (got >= 4 && !sasCheckResponseCrc(rx, got)) {
    lpCrcFail++;
    return got;
  }
  if (got >= 2 && rx[0] == sasAddress && rx[1] == command) lpOk++;
  else lpFormatFail++;
  return got;
}

// ============================================================================
// V5 PRIORITY LONG POLLS
// Priority exceptions such as 57/67 should be serviced promptly. These helpers
// skip waitForNormalPollSlot(), while keeping the same 9-bit/CRC validation.
// ============================================================================
size_t sasTypeRPriority(uint8_t command,
                        uint8_t *rx,
                        uint16_t *rx9,
                        size_t rxMax) {
  const uint8_t tx[2] = { sasAddress, command };

  Serial.print("TX PRIORITY R cmd=0x");
  if (command < 0x10) Serial.print('0');
  Serial.println(command, HEX);

  if (!sasTransmitMessage(tx, sizeof(tx))) {
    lpFormatFail++;
    return 0;
  }

  const size_t got = sasReceivePacket(rx, rx9, rxMax);
  if (got == 0) {
    lpTimeout++;
    return 0;
  }

  dumpPacket("RX PRIORITY", rx, rx9, got);

  if (got >= 4 && !sasCheckResponseCrc(rx, got)) {
    lpCrcFail++;
    return got;
  }

  if (got >= 2 && rx[0] == sasAddress && rx[1] == command) lpOk++;
  else lpFormatFail++;

  return got;
}

size_t sasTypeSPriority(uint8_t command,
                        const uint8_t *payload,
                        uint8_t payloadLen,
                        uint8_t *rx,
                        uint16_t *rx9,
                        size_t rxMax) {
  if ((size_t)payloadLen + 4U > 64U) return 0;

  uint8_t tx[64];
  size_t n = 0;
  tx[n++] = sasAddress;
  tx[n++] = command;
  for (uint8_t i = 0; i < payloadLen; i++) tx[n++] = payload[i];

  const uint16_t crc = sasCrc16(tx, n);
  tx[n++] = (uint8_t)(crc & 0xFFU);
  tx[n++] = (uint8_t)(crc >> 8);

  Serial.print("TX PRIORITY S cmd=0x");
  if (command < 0x10) Serial.print('0');
  Serial.print(command, HEX);
  Serial.print(" bytes=");
  Serial.println(n);

  if (!sasTransmitMessage(tx, n)) {
    lpFormatFail++;
    return 0;
  }

  const size_t got = sasReceivePacket(rx, rx9, rxMax);
  if (got == 0) {
    lpTimeout++;
    return 0;
  }

  dumpPacket("RX PRIORITY", rx, rx9, got);

  if (got == 1) {
    if (rx[0] == sasAddress) lpOk++;
    else lpFormatFail++;
    return got;
  }

  if (got >= 4 && !sasCheckResponseCrc(rx, got)) {
    lpCrcFail++;
    return got;
  }

  if (got >= 2 && rx[0] == sasAddress && rx[1] == command) lpOk++;
  else lpFormatFail++;

  return got;
}

// ============================================================================
// V5 GENERAL POLL EVENT QUEUE
// The GP receive path MUST remain short. It only records the event here.
// Long polls and LCD work are done after the GP function returns.
// ============================================================================
bool queueV5Event(uint8_t ev) {
  const uint32_t now = millis();

  // Suppress electrical duplicates that arrive almost immediately.
  if (ev == v5LastQueuedEvent &&
      (uint32_t)(now - v5LastQueuedEventMs) < 150UL) {
    return true;
  }

  v5LastQueuedEvent = ev;
  v5LastQueuedEventMs = now;

  if (v5EventCount >= V5_EVENT_QUEUE_SIZE) {
    v5EventDropped++;
    return false;
  }

  v5EventQueue[v5EventTail] = ev;
  v5EventTail = (uint8_t)((v5EventTail + 1U) % V5_EVENT_QUEUE_SIZE);
  v5EventCount++;
  return true;
}

bool popV5Event(uint8_t &ev) {
  if (v5EventCount == 0) return false;

  ev = v5EventQueue[v5EventHead];
  v5EventHead = (uint8_t)((v5EventHead + 1U) % V5_EVENT_QUEUE_SIZE);
  v5EventCount--;
  return true;
}

void clearV5EventQueue() {
  v5EventHead = 0;
  v5EventTail = 0;
  v5EventCount = 0;
}

void handleGeneralPollEvent(uint8_t ev) {
  if (ev == 0x00) return;

  lastEvent = ev;
  gpEvents++;

  Serial.print("GP EVENT 0x");
  if (ev < 0x10) Serial.print('0');
  Serial.print(ev, HEX);
  Serial.print(" QUEUE=");
  Serial.println(v5EventCount);

  if (!queueV5Event(ev)) {
    Serial.println("WARNING: V5 event queue full; event dropped.");
    v5State = V5RuntimeState::ERROR_STATE;
    setColor(false, true, false);
    return;
  }

  // Color can change immediately; LCD and long-poll work are deferred.
  switch (ev) {
    case 0x3D:
    case 0x67:
      setColor(false, false, true);
      break;

    case 0x51:
    case 0x6F:
      setColor(false, true, false);
      break;

    case 0x57:
      setColor(false, true, true);
      break;

    case 0x52:
    case 0x68:
    case 0x69:
      setColor(true, true, false);
      break;

    default:
      break;
  }
}

void sendGeneralPoll() {
  if ((uint32_t)(millis() - lastPollTxMs) < SAS_POLL_INTERVAL_MS) return;

  uint16_t gp1[] = {0xFF80};
  uint16_t gp2[] = {0xFF81};

  sasUart.clearRx();

  // EXACT REAL_COMM_TEST timing.
  if (!sasUart.writeWords(gp1, 1)) return;
  delay(20);
  if (!sasUart.writeWords(gp2, 1)) return;
  delay(10);

  lastPollTxMs = millis();
  gpSent++;

  bool sawAny = false;
  uint16_t r = 0;

  // Consume everything currently queued, exactly like the working test.
  while (sasUart.readWord(r)) {
    sawAny = true;
    const uint8_t b = (uint8_t)(r & 0xFFU);

    // These are normal poll/noise/status bytes seen by the proven test.
    if (b == 0x00 || b == 0x01 || b == 0x1F ||
        b == 0x80 || b == 0x81 || b == 0x7C) {
      continue;
    }

    handleGeneralPollEvent(b);
  }

  if (sawAny) gpResponses++;
}

void warmGeneralPoll(uint32_t durationMs, bool verbose = false) {
  const uint32_t startMs = millis();
  uint32_t cycles = 0;

  // Force each cycle immediately; sendGeneralPoll() itself updates lastPollTxMs.
  while ((uint32_t)(millis() - startMs) < durationMs) {
    lastPollTxMs = 0;
    sendGeneralPoll();
    cycles++;
    delay(20);
  }

  if (verbose) {
    Serial.print("GP WARMUP cycles=");
    Serial.println(cycles);
  }
}

// ============================================================================
// BCD HELPERS
// ============================================================================
bool bcdToU64(const uint8_t *p, uint8_t len, uint64_t &value) {
  value = 0;
  for (uint8_t i = 0; i < len; i++) {
    const uint8_t hi = (uint8_t)((p[i] >> 4) & 0x0F);
    const uint8_t lo = (uint8_t)(p[i] & 0x0F);
    if (hi > 9 || lo > 9) return false;
    value = value * 100ULL + (uint64_t)(hi * 10 + lo);
  }
  return true;
}

String u64ToString(uint64_t value) {
  // Manual decimal conversion for reliable uint64_t output on UNO R4.
  if (value == 0) return String("0");

  char buf[21];
  buf[20] = '\0';
  int8_t pos = 20;

  while (value > 0 && pos > 0) {
    const uint8_t digit = (uint8_t)(value % 10ULL);
    value /= 10ULL;
    buf[--pos] = (char)('0' + digit);
  }

  return String(&buf[pos]);
}


bool u32ToBcd4(uint32_t value, uint8_t out[4]) {
  if (out == nullptr || value > 99999999UL) return false;
  for (int8_t i = 3; i >= 0; i--) {
    const uint8_t pair = (uint8_t)(value % 100UL);
    value /= 100UL;
    out[i] = (uint8_t)(((pair / 10U) << 4) | (pair % 10U));
  }
  return true;
}


bool u64ToBcd5(uint64_t value, uint8_t out[5]) {
  if (out == nullptr || value > 9999999999ULL) return false;
  for (int8_t i = 4; i >= 0; i--) {
    const uint8_t pair = (uint8_t)(value % 100ULL);
    value /= 100ULL;
    out[i] = (uint8_t)(((pair / 10U) << 4) | (pair % 10U));
  }
  return true;
}

void v6MakeAftTransactionId(char out[7]) {
  // V6.5.9 deliberately does NOT use EEPROM for transaction IDs.
  // Mix a RAM sequence, current timing, card UID and current machine CREDIT.
  // Previous/current AFT cycle is FF-synchronized before a new value command,
  // so this only needs to avoid accidental ID reuse; it is not a security token.
  uint32_t x = ++v6AftTxSequence;
  x ^= (uint32_t)micros();
  x ^= ((uint32_t)millis() << 7) | ((uint32_t)millis() >> 25);
  x ^= (uint32_t)v6LastCurrentCredits * 2654435761UL;
  for (uint8_t i = 0; i < activeCardUidLen; i++) {
    x = (x * 33UL) ^ activeCardUid[i];
  }
  x ^= (v6RuntimeTxGeneration * 2246822519UL);
  x &= 0x00FFFFFFUL;
  if (x == 0) x = 1;
  static const char hex[] = "0123456789ABCDEF";
  for (int8_t i = 5; i >= 0; i--) {
    out[i] = hex[x & 0x0FU];
    x >>= 4;
  }
  out[6] = '\0';
}

uint32_t buildProfileFlags();

// ============================================================================
// V6 VALIDATION CORE: CRC-protected profile DB + RAM transaction, meters, AFT observer
// ============================================================================
uint32_t v6Crc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      const uint32_t mask = (uint32_t)-(int32_t)(crc & 1UL);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

void v6CopyFixed(char *dst, size_t dstSize, const String &src) {
  if (!dst || dstSize == 0) return;
  size_t n = src.length();
  if (n >= dstSize) n = dstSize - 1;
  for (size_t i = 0; i < n; i++) dst[i] = src[i];
  dst[n] = '\0';
  for (size_t i = n + 1; i < dstSize; i++) dst[i] = '\0';
}

String v6FixedString(const char *p, size_t n) {
  String s;
  for (size_t i = 0; i < n && p[i] != '\0'; i++) s += p[i];
  return s;
}

void v6TraceAdd(uint8_t kind, uint8_t code, uint16_t detail = 0) {
  v6Trace[v6TraceHead] = { millis(), kind, code, detail };
  v6TraceHead = (uint8_t)((v6TraceHead + 1U) % V6_TRACE_SIZE);
}

bool v6ProfileValid(const V6ProfileRecord &r) {
  if (r.magic != V6_PROFILE_MAGIC || r.schema != V6_PROFILE_SCHEMA ||
      r.recordSize != sizeof(V6ProfileRecord) || r.address < 1 || r.address > 127) return false;
  const uint32_t c = v6Crc32(reinterpret_cast<const uint8_t*>(&r), offsetof(V6ProfileRecord, crc32));
  return c == r.crc32;
}

void v6LoadProfileDb() {
  uint32_t bestGen = 0;
  v6MostRecentProfileSlot = -1;
  for (int i = 0; i < V6_PROFILE_DB_SLOTS; i++) {
    EEPROM.get(V6_PROFILE_DB_BASE + i * V6_PROFILE_DB_STRIDE, v6ProfileDb[i]);
    v6ProfileDbValid[i] = v6ProfileValid(v6ProfileDb[i]);
    if (v6ProfileDbValid[i] && v6ProfileDb[i].generation >= bestGen) {
      bestGen = v6ProfileDb[i].generation;
      v6MostRecentProfileSlot = i;
    }
  }
  if (v6MostRecentProfileSlot >= 0) {
    sasAddress = v6ProfileDb[v6MostRecentProfileSlot].address;
    Serial.print("V6 PROFILE DB: preferred SAS address="); Serial.println(sasAddress);
  } else {
    Serial.println("V6 PROFILE DB: no valid records; starting address=1");
  }
}

bool v6SaveCurrentProfileDb(uint16_t stabilityGood, uint16_t stabilityTotal) {
  if (!sasCaps.versionKnown || sasCaps.machineSerial.length() == 0 || sasAddress < 1 || sasAddress > 127) return false;

  uint32_t maxGen = 0;
  int target = -1;
  int oldest = -1;
  uint32_t oldestGen = 0xFFFFFFFFUL;
  for (int i = 0; i < V6_PROFILE_DB_SLOTS; i++) {
    if (v6ProfileDbValid[i]) {
      if (v6ProfileDb[i].generation > maxGen) maxGen = v6ProfileDb[i].generation;
      if (v6FixedString(v6ProfileDb[i].machineSerial, sizeof(v6ProfileDb[i].machineSerial)) == sasCaps.machineSerial) target = i;
      if (v6ProfileDb[i].generation < oldestGen) { oldestGen = v6ProfileDb[i].generation; oldest = i; }
    } else if (target < 0) {
      target = i;
    }
  }
  if (target < 0) target = oldest >= 0 ? oldest : 0;

  V6ProfileRecord r = {};
  r.magic = V6_PROFILE_MAGIC;
  r.schema = V6_PROFILE_SCHEMA;
  r.recordSize = sizeof(V6ProfileRecord);
  r.generation = maxGen + 1UL;
  r.address = sasAddress;
  r.profile = (uint8_t)sasCaps.profile;
  r.validationStyle = sasCaps.validationStyle;
  r.denominationCode = machineInfo.valid ? machineInfo.denominationCode : 0;
  r.versionCode = sasCaps.versionCode;
  r.stabilityGood = stabilityGood;
  r.stabilityTotal = stabilityTotal;
  r.flags = buildProfileFlags();
  r.accountingDenomMinorUnits = machineInfo.valid ? machineInfo.accountingDenomMinorUnits : 0;
  r.cashOutLimitCredits = sasCaps.cashOutLimitCredits;
  r.aftTransferLimitMinorUnits = sasCaps.aftTransferLimitCents;
  v6CopyFixed(r.versionText, sizeof(r.versionText), sasCaps.versionText);
  v6CopyFixed(r.machineSerial, sizeof(r.machineSerial), sasCaps.machineSerial);
  r.crc32 = v6Crc32(reinterpret_cast<const uint8_t*>(&r), offsetof(V6ProfileRecord, crc32));

  EEPROM.put(V6_PROFILE_DB_BASE + target * V6_PROFILE_DB_STRIDE, r);
  V6ProfileRecord check = {};
  EEPROM.get(V6_PROFILE_DB_BASE + target * V6_PROFILE_DB_STRIDE, check);
  if (!v6ProfileValid(check) || check.generation != r.generation) return false;
  v6ProfileDb[target] = check;
  v6ProfileDbValid[target] = true;
  v6MostRecentProfileSlot = target;
  Serial.print("V6 PROFILE DB SAVED slot="); Serial.print(target);
  Serial.print(" addr="); Serial.print(check.address);
  Serial.print(" serial="); Serial.println(v6FixedString(check.machineSerial, sizeof(check.machineSerial)));
  return true;
}

bool v6JournalValid(const V6JournalRecord &r) {
  if (r.magic != V6_JOURNAL_MAGIC || r.schema != V6_JOURNAL_SCHEMA ||
      r.recordSize != sizeof(V6JournalRecord)) return false;
  return r.crc32 == v6Crc32(reinterpret_cast<const uint8_t*>(&r), offsetof(V6JournalRecord, crc32));
}

bool v6JournalTerminal(uint8_t st) {
  return st == (uint8_t)V6JournalState::RFID_COMMITTED ||
         st == (uint8_t)V6JournalState::DONE ||
         st == (uint8_t)V6JournalState::FAILED;
}

void v6LoadJournal() {
  // V6.5.14 power-loss policy:
  //   - DO NOT read any legacy transaction journal from EEPROM.
  //   - A reset/power cycle forgets the in-flight transaction.
  //   - On the next value command we interrogate current S6000/AFT state first.
  //   - Never blind-resend a value command from an old transaction.
  memset(&v6LatestJournal, 0, sizeof(v6LatestJournal));
  v6LatestJournalValid = false;
  v6LatestJournalSlot = -1;
  v6RecoveryRequired = false;
  v6RuntimeTxGeneration = 0;
  v6AftTxSequence = 0;
  Serial.println("V6 TX STATE: RAM ONLY; legacy EEPROM transaction journal ignored.");
  Serial.println("POWER RESET POLICY: forget in-flight transaction; no old value command is resent.");
}

bool v6WriteJournal(V6JournalRecord &r) {
  // Despite the historical function name, this is RAM-only in V6.5.14.
  // There is intentionally NO EEPROM API access here.
  const uint32_t nextGen = ++v6RuntimeTxGeneration;
  r.magic = V6_JOURNAL_MAGIC;
  r.schema = V6_JOURNAL_SCHEMA;
  r.recordSize = sizeof(V6JournalRecord);
  r.generation = nextGen;
  r.address = sasAddress;
  v6CopyFixed(r.machineSerial, sizeof(r.machineSerial), sasCaps.machineSerial);
  r.crc32 = v6Crc32(reinterpret_cast<const uint8_t*>(&r), offsetof(V6JournalRecord, crc32));

  v6LatestJournal = r;
  v6LatestJournalValid = true;
  v6LatestJournalSlot = -1;
  v6RecoveryRequired = !v6JournalTerminal(r.state);
  v6TraceAdd(2, r.type, r.state);
  return true;
}

// V6.5.14: clear ONLY the volatile transaction state.
// This never touches EEPROM/profile data and never sends a SAS value command.
void v6ClearRuntimeTransaction(const char *reason) {
  if (reason && *reason) {
    Serial.print("V6 RAM TX CLEAR: ");
    Serial.println(reason);
  }
  memset(&v6LatestJournal, 0, sizeof(v6LatestJournal));
  v6LatestJournalValid = false;
  v6LatestJournalSlot = -1;
  v6RecoveryRequired = false;
  v6BeforeMeters = V6MeterSnapshot();
  v6AfterMeters = V6MeterSnapshot();
  v6LastMeterVerifyPassed = false;
}

V6MeterSnapshotStored v6StoreMeters(const V6MeterSnapshot &m) {
  V6MeterSnapshotStored x = {};
  x.currentCredits = (uint32_t)m.currentCredits;
  x.ticketInCredits = (uint32_t)m.ticketInCredits;
  x.ticketOutCredits = (uint32_t)m.ticketOutCredits;
  x.transferToGameCredits = (uint32_t)m.transferToGameCredits;
  x.transferToHostCredits = (uint32_t)m.transferToHostCredits;
  return x;
}

V6MeterSnapshot v6RestoreMeters(const V6MeterSnapshotStored &x) {
  V6MeterSnapshot m;
  m.valid = true;
  m.currentCredits = x.currentCredits;
  m.ticketInCredits = x.ticketInCredits;
  m.ticketOutCredits = x.ticketOutCredits;
  m.transferToGameCredits = x.transferToGameCredits;
  m.transferToHostCredits = x.transferToHostCredits;
  return m;
}

bool v6ObservedJournalWriteBlocked(bool latestValid, uint8_t latestState) {
  return latestValid && !v6JournalTerminal(latestState);
}

bool v6JournalIsActiveNonTerminal() {
  return v6ObservedJournalWriteBlocked(v6LatestJournalValid,
                                       v6LatestJournal.state);
}

bool v6ObserverOnlyAftJournal(const V6JournalRecord &r) {
  // AFT journals created only by GP event observation contain no RFID/card/value
  // ownership information.  Old V6.5.2 could accidentally let such a journal
  // replace an active RFID transaction journal.  This signature lets recovery
  // safely identify that old observer-only record without guessing about money.
  if (r.type != (uint8_t)V6JournalType::AFT) return false;
  if (r.amountCredits != 0 || r.amountMinorUnits != 0) return false;
  if (r.balanceBefore != 0 || r.balanceAfter != 0) return false;
  if (r.cardUidLen != 0) return false;
  if (r.transactionId[0] != '\0') return false;
  return true;
}

bool v6BeginObservedJournal(V6JournalType type, uint64_t amountMinorUnits = 0) {
  // CRITICAL JOURNAL OWNERSHIP RULE:
  // An asynchronous GP observation must NEVER replace any unfinished journal.
  // In V6.5.2, AFT request events 66/6A/6B could create a generic AFT journal
  // while an RFID_CASHOUT / RFID_SPLIT_CASHOUT was still active.  The generic
  // record then became the newest EEPROM journal and recovery looped forever.
  if (v6JournalIsActiveNonTerminal()) {
    if (v6LatestJournal.type == (uint8_t)type) {
      // Duplicate observation of the same active event: keep the existing owner.
      return true;
    }
    Serial.print("V6 JOURNAL GUARD: skip observed type=");
    Serial.print((uint8_t)type);
    Serial.print("; active type=");
    Serial.print(v6LatestJournal.type);
    Serial.print(" state=");
    Serial.println(v6LatestJournal.state);
    return false;
  }

  V6JournalRecord r = {};
  r.type = (uint8_t)type;
  r.state = (uint8_t)V6JournalState::OBSERVED;
  r.sasStatus = 0xFF;
  r.amountMinorUnits = amountMinorUnits;
  r.beforeMeters = v6StoreMeters(v6BeforeMeters);
  return v6WriteJournal(r);
}

bool v6UpdateJournal(V6JournalState state, uint8_t sasStatus = 0xFF) {
  if (!v6LatestJournalValid) return false;
  V6JournalRecord r = v6LatestJournal;
  r.state = (uint8_t)state;
  r.sasStatus = sasStatus;
  r.afterMeters = v6StoreMeters(v6AfterMeters);
  return v6WriteJournal(r);
}

bool v6ReadSelectedMeters2F(V6MeterSnapshot &out, bool priority = false, bool displayResult = false) {
  // length=7: game number 0000 (2 bytes) + five one-byte meter codes.
  // 000C current credits, 0015 ticket in, 0016 ticket out,
  // 0017 transfers to gaming machine, 0018 transfers to host.
  const uint8_t payload[8] = {0x07, 0x00, 0x00, 0x0C, 0x15, 0x16, 0x17, 0x18};
  uint8_t rx[64];
  uint16_t rx9[64];
  const size_t n = priority
    ? sasTypeSPriority(0x2F, payload, sizeof(payload), rx, rx9, sizeof(rx))
    : sasTypeS(0x2F, payload, sizeof(payload), rx, rx9, sizeof(rx));

  out = V6MeterSnapshot();
  if (n < 10 || rx[0] != sasAddress || rx[1] != 0x2F || !sasCheckResponseCrc(rx, n)) {
    if (displayResult) showMessage("LP2F FAIL", String("RX=") + n, 1000);
    return false;
  }
  const size_t dataEnd = 3U + (size_t)rx[2];
  if (n < dataEnd + 2U || rx[3] != 0x00 || rx[4] != 0x00) return false;

  bool have0C=false, have15=false, have16=false, have17=false, have18=false;
  size_t i = 5;
  while (i + 5 <= dataEnd) {
    const uint8_t code = rx[i++];
    uint64_t value = 0;
    if (!bcdToU64(&rx[i], 4, value)) return false;
    i += 4;
    switch (code) {
      case 0x0C: out.currentCredits = value; have0C=true; break;
      case 0x15: out.ticketInCredits = value; have15=true; break;
      case 0x16: out.ticketOutCredits = value; have16=true; break;
      case 0x17: out.transferToGameCredits = value; have17=true; break;
      case 0x18: out.transferToHostCredits = value; have18=true; break;
      default: break;
    }
  }
  out.valid = have0C && have15 && have16 && have17 && have18;
  if (out.valid) {
    Serial.print("LP2F CR="); Serial.print(u64ToString(out.currentCredits));
    Serial.print(" TIN="); Serial.print(u64ToString(out.ticketInCredits));
    Serial.print(" TOUT="); Serial.print(u64ToString(out.ticketOutCredits));
    Serial.print(" XIN="); Serial.print(u64ToString(out.transferToGameCredits));
    Serial.print(" XOUT="); Serial.println(u64ToString(out.transferToHostCredits));
  }
  return out.valid;
}

bool v6MeterAdvanced(uint64_t before, uint64_t after) {
  if (after > before) return true;
  // LP2F meters used here are 4 BCD bytes (8 decimal digits). Treat a near-top
  // -> near-zero transition as a normal rollover instead of a failure.
  return before >= 99000000ULL && after <= 1000000ULL;
}

bool v6VerifyMeterDelta(V6JournalType type, const V6MeterSnapshot &before,
                        const V6MeterSnapshot &after) {
  if (!before.valid || !after.valid) return false;
  switch (type) {
    case V6JournalType::TICKET_IN:
      return v6MeterAdvanced(before.ticketInCredits, after.ticketInCredits);
    case V6JournalType::TICKET_OUT:
      return v6MeterAdvanced(before.ticketOutCredits, after.ticketOutCredits);
    case V6JournalType::RFID_CASHOUT:
      return v6MeterAdvanced(before.transferToHostCredits, after.transferToHostCredits);
    case V6JournalType::AFT:
      if (v6LastAft.valid) {
        switch (v6LastAft.transferType) {
          case 0x00: case 0x10: case 0x11: case 0x20: case 0x40: case 0x60:
            return v6MeterAdvanced(before.transferToGameCredits, after.transferToGameCredits) ||
                   (v6LastAft.transferType == 0x20 || v6LastAft.transferType == 0x60) &&
                   v6MeterAdvanced(before.ticketOutCredits, after.ticketOutCredits);
          case 0x80: case 0x90:
            return v6MeterAdvanced(before.transferToHostCredits, after.transferToHostCredits);
          default:
            break;
        }
      }
      return v6MeterAdvanced(before.transferToGameCredits, after.transferToGameCredits) ||
             v6MeterAdvanced(before.transferToHostCredits, after.transferToHostCredits);
    default:
      return true;
  }
}

const char *v6AftStatusShort(uint8_t st) {
  switch (st) {
    case 0x00: return "FULL OK"; case 0x01: return "PARTIAL OK"; case 0x40: return "PENDING";
    case 0x80: return "CANCELLED"; case 0x81: return "TXID DUP"; case 0x82: return "BAD FUNC";
    case 0x83: return "BAD AMOUNT"; case 0x84: return "XFER LIMIT"; case 0x85: return "DENOM ERR";
    case 0x86: return "NO PARTIAL"; case 0x87: return "BUSY/TILT"; case 0x88: return "NOT REG";
    case 0x89: return "REG KEY"; case 0x8A: return "NO POS ID"; case 0x8B: return "NO WON CR";
    case 0x8C: return "NO DENOM"; case 0x8D: return "EXPIRATION"; case 0x8E: return "NO TKT DEV";
    case 0x8F: return "POOL CONFLICT"; case 0x90: return "NO RECEIPT"; case 0x91: return "RECEIPT DATA";
    case 0x92: return "RECEIPT N/A"; case 0x93: return "ASSET ID"; case 0x94: return "NOT LOCKED";
    case 0x95: return "TXID INVALID"; case 0x9F: return "UNEXPECTED"; case 0xC0: return "INCOMPATIBLE";
    case 0xC1: return "UNSUPPORTED"; case 0xFF: return "NO INFO"; default: return "AFT STATUS";
  }
}

const char *v6AftStatusKo(uint8_t st) {
  switch (st) {
    case 0x00: return "전체 이체 성공"; case 0x01: return "부분 이체 성공"; case 0x40: return "이체 처리 중";
    case 0x80: return "호스트 취소"; case 0x81: return "Transaction ID 중복"; case 0x82: return "지원하지 않는 이체 기능";
    case 0x83: return "금액/만료 형식 오류"; case 0x84: return "기계 이체 한도 초과"; case 0x85: return "기계 denomination 배수 오류";
    case 0x87: return "문열림/tilt/비활성 등 현재 이체 불가"; case 0x88: return "등록 필요";
    case 0x93: return "Asset ID 0 또는 불일치"; case 0x94: return "잠금 필요 조건인데 잠기지 않음";
    case 0xC0: return "현재 거래와 호환되지 않는 명령"; case 0xC1: return "지원하지 않는 transfer code";
    case 0xFF: return "이체 정보 없음"; default: return "AFT 상태 코드 확인";
  }
}

const char *v6TicketStatusKo(uint8_t st) {
  switch (st) {
    case 0x00: return "현금성 티켓 상환 성공"; case 0x01: return "제한 프로모션 티켓 성공";
    case 0x02: return "비제한 프로모션 티켓 성공"; case 0x20: return "Long Poll 71 대기";
    case 0x40: return "티켓 상환 처리 중"; case 0x80: return "호스트가 티켓 거부";
    case 0x81: return "Validation Number 불일치"; case 0x82: return "지원하지 않는 기능";
    case 0x83: return "금액 BCD 오류"; case 0x84: return "기계 Credit Limit 초과";
    case 0x85: return "Denomination 배수 오류"; case 0x86: return "티켓 금액 불일치";
    case 0x87: return "현재 티켓 이체 불가"; case 0x88: return "티켓 처리 시간 초과";
    case 0x89: return "SAS 링크 다운"; case 0x8A: return "Ticket Redemption 비활성";
    case 0x8B: return "Validator 오류"; case 0xC0: return "현재 티켓 사이클과 호환되지 않음";
    case 0xFF: return "Validation 정보 없음"; default: return "티켓 상태 코드 확인";
  }
}

bool v6InterrogateAft72(uint8_t transferCode, bool priority = true, bool displayResult = false) {
  if (transferCode != 0xFE && transferCode != 0xFF) return false;
  const uint8_t payload[3] = {0x02, transferCode, 0x00};
  uint8_t rx[96]; uint16_t rx9[96];
  const size_t n = priority
    ? sasTypeSPriority(0x72, payload, sizeof(payload), rx, rx9, sizeof(rx))
    : sasTypeS(0x72, payload, sizeof(payload), rx, rx9, sizeof(rx));

  v6LastAft = V6AftSnapshot();
  if (n < 7 || rx[0] != sasAddress || rx[1] != 0x72 || !sasCheckResponseCrc(rx, n)) {
    if (displayResult) showMessage("LP72 INT FAIL", String("RX=") + n, 1000);
    return false;
  }
  const uint8_t len = rx[2];
  const size_t expected = 3U + len + 2U;
  if (n < expected || len < 2) return false;

  v6LastAft.valid = true;
  v6LastAft.bufferPosition = rx[3];
  v6LastAft.transferStatus = rx[4];
  if (len >= 4) {
    v6LastAft.receiptStatus = rx[5];
    v6LastAft.transferType = rx[6];
  }
  if (len >= 19 && n >= 24) {
    bcdToU64(&rx[7], 5, v6LastAft.cashableMinorUnits);
    bcdToU64(&rx[12], 5, v6LastAft.restrictedMinorUnits);
    bcdToU64(&rx[17], 5, v6LastAft.nonrestrictedMinorUnits);
  }

  Serial.print("AFT72 "); Serial.print(transferCode, HEX);
  Serial.print(" status=0x"); if (v6LastAft.transferStatus < 0x10) Serial.print('0');
  Serial.print(v6LastAft.transferStatus, HEX); Serial.print(" "); Serial.print(v6AftStatusShort(v6LastAft.transferStatus));
  Serial.print(" / "); Serial.println(v6AftStatusKo(v6LastAft.transferStatus));
  v6TraceAdd(3, v6LastAft.transferStatus, v6LastAft.bufferPosition);
  if (displayResult) showMessage(String("72:") + String(v6LastAft.transferStatus, HEX), v6AftStatusShort(v6LastAft.transferStatus), 1800);
  return true;
}

bool v6Handpay94GateEligible(String &reason) {
  if (!sasOnline) { reason = "SAS OFFLINE"; return false; }
  if (!lastHandpay.valid || v5State != V5RuntimeState::HANDPAY_PENDING) { reason = "NO HANDPAY"; return false; }
  if (!sasCaps.remoteHandpayByVersion) { reason = "94 NOT IN SAS"; return false; }
  if (lastHandpay.resetId != 0x01) { reason = "RESET ID != 1"; return false; }
  if (!v6BootSelfTestPassed) { reason = "SELFTEST FAIL"; return false; }
  reason = "ELIGIBLE/NO SEND";
  return true;
}

bool v6SameActiveCardUid(const V6JournalRecord &r) {
  if (!cardActive || r.cardUidLen == 0 || r.cardUidLen != activeCardUidLen) return false;
  for (uint8_t i = 0; i < r.cardUidLen; i++) if (r.cardUid[i] != activeCardUid[i]) return false;
  return true;
}

bool v6PrepareAtomicCardJournal(uint32_t newBalance, uint32_t amountCredits, uint64_t amountMinorUnits) {
  if (!cardActive || activeCardUidLen == 0 || newBalance > cardBalance) return false;
  if (v6RecoveryRequired) return false;
  V6JournalRecord r = {};
  r.type = (uint8_t)V6JournalType::RFID_ATOMIC;
  r.state = (uint8_t)V6JournalState::PREPARED;
  r.cardUidLen = activeCardUidLen;
  for (uint8_t i = 0; i < activeCardUidLen; i++) r.cardUid[i] = activeCardUid[i];
  r.amountCredits = amountCredits;
  r.amountMinorUnits = amountMinorUnits;
  r.balanceBefore = cardBalance;
  r.balanceAfter = newBalance;
  r.beforeMeters = v6StoreMeters(v6BeforeMeters);
  return v6WriteJournal(r);
}

bool v6AtomicCommitCardBalance(uint32_t newBalance) {
  // This primitive is intentionally callable only after a journal reaches a verified
  // SAS/meter state. No V6 machine-observer path fabricates that success.
  if (!v6LatestJournalValid || !v6SameActiveCardUid(v6LatestJournal)) return false;
  if (v6LatestJournal.state != (uint8_t)V6JournalState::SAS_VERIFIED &&
      v6LatestJournal.state != (uint8_t)V6JournalState::METERS_VERIFIED) return false;
  if (!writeBalanceToCard(newBalance)) return false;
  uint32_t check = 0;
  if (!readBalanceFromCard(check) || check != newBalance) return false;
  cardBalance = check;
  V6JournalRecord r = v6LatestJournal;
  r.balanceAfter = newBalance;
  r.state = (uint8_t)V6JournalState::RFID_COMMITTED;
  return v6WriteJournal(r);
}

bool v6BootProtocolSelfTest() {
  static const uint8_t crcVec[] = {0x01, 0x74, 0xFF, 0x00, 0x00, 0x00};
  const bool crcOK = sasCrc16(crcVec, sizeof(crcVec)) == 0xF91A;
  static const uint8_t bcdVec[] = {0x00, 0x00, 0x12, 0x34};
  uint64_t bcdValue = 0;
  const bool bcdOK = bcdToU64(bcdVec, sizeof(bcdVec), bcdValue) && bcdValue == 1234ULL;
  const bool profileMapOK =
    (V6_PROFILE_DB_BASE + V6_PROFILE_DB_SLOTS * V6_PROFILE_DB_STRIDE <= V6_EEPROM_BYTES) &&
    (EEPROM_SLOT1_ADDR + 256 <= V6_EEPROM_BYTES);
  v6BootSelfTestPassed = crcOK && bcdOK && profileMapOK;
  Serial.print("V6 BOOT SELFTEST CRC="); Serial.print(crcOK ? "PASS" : "FAIL");
  Serial.print(" BCD="); Serial.print(bcdOK ? "PASS" : "FAIL");
  Serial.print(" PROFILE_EEPROM="); Serial.print(profileMapOK ? "PASS" : "FAIL");
  Serial.println(" TX_EEPROM=DISABLED");
  return v6BootSelfTestPassed;
}

bool v6HealthReady() {
  return v6BootSelfTestPassed && sasOnline &&
         savedProfileValid && savedProfile.stabilityGood >= STABILITY_PASS_REQUIRED &&
         sasUart.errorCount() == 0 && sasUart.overflowCount() == 0 &&
         lpCrcFail == 0 && !v6RecoveryRequired;
}


// ============================================================================
// V5 UNIT / STATUS HELPERS
// ============================================================================
uint32_t sasDenominationToMinorUnits(uint8_t code) {
  // SAS 6.x Table C-4. Values are minor currency units where integral.
  switch (code) {
    case 0x01: return 1;
    case 0x02: return 5;
    case 0x03: return 10;
    case 0x04: return 25;
    case 0x05: return 50;
    case 0x06: return 100;
    case 0x07: return 500;
    case 0x08: return 1000;
    case 0x09: return 2000;
    case 0x0A: return 10000;
    case 0x0B: return 20;
    case 0x0C: return 200;
    case 0x0D: return 250;
    case 0x0E: return 2500;
    case 0x0F: return 5000;
    case 0x10: return 20000;
    case 0x11: return 25000;
    case 0x12: return 50000;
    case 0x13: return 100000;
    case 0x14: return 200000;
    case 0x15: return 250000;
    case 0x16: return 500000;
    case 0x17: return 2;
    case 0x18: return 3;
    case 0x19: return 15;
    case 0x1A: return 40;
    default:   return 0;
  }
}

const char *v5StateName(V5RuntimeState state) {
  switch (state) {
    case V5RuntimeState::OFFLINE:                   return "OFFLINE";
    case V5RuntimeState::NORMAL:                    return "NORMAL";
    case V5RuntimeState::HANDPAY_PENDING:           return "HANDPAY";
    case V5RuntimeState::SYSTEM_VALIDATION_PENDING: return "SYS VALID";
    case V5RuntimeState::TICKET_IN_ESCROW:          return "TKT ESCROW";
    case V5RuntimeState::TICKET_STATUS:             return "TKT STATUS";
    case V5RuntimeState::TICKET_PRINTED:            return "TKT PRINT";
    case V5RuntimeState::AFT_COMPLETE_SEEN:         return "AFT DONE";
    case V5RuntimeState::GAME_LOCKED:               return "LOCKED";
    default:                                        return "ERROR";
  }
}

const char *ticketMachineStatusName(uint8_t status) {
  switch (status) {
    case 0x00: return "CASH REDEEMED";
    case 0x01: return "REST REDEEMED";
    case 0x02: return "PROMO REDEEMED";
    case 0x20: return "WAIT LP71";
    case 0x40: return "PENDING";
    case 0x80: return "HOST REJECT";
    case 0x81: return "VAL MISMATCH";
    case 0x82: return "BAD FUNCTION";
    case 0x83: return "BAD AMOUNT";
    case 0x84: return "CREDIT LIMIT";
    case 0x85: return "DENOM MISMATCH";
    case 0x86: return "AMT MISMATCH";
    case 0x87: return "CANNOT ACCEPT";
    case 0x88: return "TIMEOUT";
    case 0x89: return "LINK DOWN";
    case 0x8A: return "TKT DISABLED";
    case 0x8B: return "VALIDATOR FAIL";
    case 0xC0: return "INCOMPATIBLE";
    case 0xFF: return "NO VAL INFO";
    default:   return "OTHER STATUS";
  }
}

void showV5EventScreen(const String &line1,
                       const String &line2,
                       uint16_t holdMs = 2500) {
  lcd.clear();
  lcdLine(0, line1);
  lcdLine(1, line2);
  v5EventScreenUntilMs = millis() + (uint32_t)holdMs;
}

void serviceV5EventScreen() {
  if (v5EventScreenUntilMs == 0) return;

  if ((int32_t)(millis() - v5EventScreenUntilMs) >= 0) {
    v5EventScreenUntilMs = 0;
    if (cardActive) showCard();
    else showIdle();
  }
}

String bytesToPrintableAscii(const uint8_t *p, size_t len) {
  String out;
  for (size_t i = 0; i < len; i++) {
    if (p[i] >= 32 && p[i] <= 126) out += (char)p[i];
  }
  return out;
}


// ============================================================================
// SAS VERSION / PROFILE HELPERS
// ============================================================================
uint16_t parseSasVersionCode(const String &text) {
  char digits[5] = {0, 0, 0, 0, 0};
  uint8_t n = 0;
  for (size_t i = 0; i < text.length() && n < 4; i++) {
    const char c = text[i];
    if (c >= '0' && c <= '9') digits[n++] = c;
  }
  if (n < 3) return 0;
  return (uint16_t)((digits[0] - '0') * 100 +
                    (digits[1] - '0') * 10 +
                    (digits[2] - '0'));
}

const char *sasProfileName(SasProfile p) {
  switch (p) {
    case SasProfile::SAS_PRE_500:  return "PRE5";
    case SasProfile::SAS_500_501: return "5.00/5.01";
    case SasProfile::SAS_502:     return "5.02";
    case SasProfile::SAS_510:     return "5.10";
    case SasProfile::SAS_600:     return "6.00";
    case SasProfile::SAS_601_PLUS:return "6.01+";
    default:                       return "UNKNOWN";
  }
}

void classifySasVersion() {
  const uint16_t v = sasCaps.versionCode;
  sasCaps.ticketProtocolByVersion = false;
  sasCaps.systemValidationByVersion = false;
  sasCaps.remoteHandpayByVersion = false;
  sasCaps.multiDenomByVersion = false;
  sasCaps.aftProtocolByVersion = false;

  if (!sasCaps.versionKnown || v == 0) {
    sasCaps.profile = SasProfile::UNKNOWN;
    return;
  }

  if (v < 500) sasCaps.profile = SasProfile::SAS_PRE_500;
  else if (v <= 501) sasCaps.profile = SasProfile::SAS_500_501;
  else if (v == 502) sasCaps.profile = SasProfile::SAS_502;
  else if (v < 600) sasCaps.profile = SasProfile::SAS_510;
  else if (v == 600) sasCaps.profile = SasProfile::SAS_600;
  else sasCaps.profile = SasProfile::SAS_601_PLUS;

  if (v >= 500) {
    sasCaps.ticketProtocolByVersion = true;
    sasCaps.remoteHandpayByVersion = true;
  }
  if (v >= 501) sasCaps.systemValidationByVersion = true;
  if (v >= 502) sasCaps.multiDenomByVersion = true;
  if (v >= 510) sasCaps.aftProtocolByVersion = true;
}

String validationName(uint8_t style) {
  switch (style & 0x03U) {
    case 0: return "STD/NONE";
    case 1: return "SYSTEM";
    case 2: return "SECURE";
    default:return "RESERVED";
  }
}

// ============================================================================
// EEPROM / CRC32 HELPERS
// ============================================================================
uint32_t profileCrc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;

  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      const uint32_t mask = (uint32_t)-(int32_t)(crc & 1UL);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }

  return ~crc;
}

void copyStringToFixed(char *dst, size_t dstSize, const String &srcText) {
  if (dstSize == 0) return;

  size_t n = srcText.length();
  if (n >= dstSize) n = dstSize - 1;

  for (size_t i = 0; i < n; i++) dst[i] = srcText[i];
  dst[n] = '\0';

  for (size_t i = n + 1; i < dstSize; i++) dst[i] = '\0';
}

String fixedToString(const char *src, size_t maxLen) {
  String out;
  for (size_t i = 0; i < maxLen; i++) {
    const char c = src[i];
    if (c == '\0') break;
    out += c;
  }
  return out;
}

bool storedProfileIsValid(const StoredSasProfile &p) {
  if (p.magic != EEPROM_PROFILE_MAGIC) return false;
  if (p.schema != EEPROM_PROFILE_SCHEMA) return false;
  if (p.recordSize != sizeof(StoredSasProfile)) return false;

  const uint32_t expected = profileCrc32(
    reinterpret_cast<const uint8_t*>(&p),
    offsetof(StoredSasProfile, crc32)
  );

  return p.crc32 == expected;
}

bool readStoredProfileSlot(int address, StoredSasProfile &out) {
  EEPROM.get(address, out);
  return storedProfileIsValid(out);
}

uint32_t buildProfileFlags() {
  uint32_t f = 0;
  if (sasCaps.versionKnown)             f |= PF_VERSION_KNOWN;
  if (sasCaps.ticketProtocolByVersion)  f |= PF_TICKET_BY_VERSION;
  if (sasCaps.systemValidationByVersion)f |= PF_SYSVAL_BY_VERSION;
  if (sasCaps.remoteHandpayByVersion)   f |= PF_REMOTE94_BY_VERSION;
  if (sasCaps.multiDenomByVersion)      f |= PF_MULTIDENOM_BY_VERSION;
  if (sasCaps.aftProtocolByVersion)     f |= PF_AFT_BY_VERSION;
  if (sasCaps.a0Responded)              f |= PF_A0_RESPONDED;
  if (sasCaps.ticketRedemptionEnabled)  f |= PF_TICKET_ENABLED;
  if (sasCaps.validationExtensions)     f |= PF_VALIDATION_EXT;
  if (sasCaps.aftEnabled)               f |= PF_AFT_ENABLED;
  if (sasCaps.fortyMsSupported)         f |= PF_40MS_SUPPORTED;
  if (sasCaps.a4Responded)              f |= PF_A4_RESPONDED;
  if (sasCaps.lp74Responded)            f |= PF_74_RESPONDED;
  f |= PF_STABILITY_PASSED;
  return f;
}

StoredSasProfile buildStoredProfile(uint32_t generation,
                                    uint16_t stabilityGood,
                                    uint16_t stabilityTotal) {
  StoredSasProfile p = {};
  p.magic = EEPROM_PROFILE_MAGIC;
  p.schema = EEPROM_PROFILE_SCHEMA;
  p.recordSize = sizeof(StoredSasProfile);
  p.generation = generation;

  p.versionCode = sasCaps.versionCode;
  p.profile = (uint8_t)sasCaps.profile;
  p.validationStyle = sasCaps.validationStyle;
  p.stabilityGood = stabilityGood;
  p.stabilityTotal = stabilityTotal;
  p.flags = buildProfileFlags();

  p.cashOutLimitCredits = sasCaps.cashOutLimitCredits;
  p.aftTransferLimitCents = sasCaps.aftTransferLimitCents;

  copyStringToFixed(p.versionText, sizeof(p.versionText), sasCaps.versionText);
  copyStringToFixed(p.machineSerial, sizeof(p.machineSerial), sasCaps.machineSerial);

  p.crc32 = profileCrc32(
    reinterpret_cast<const uint8_t*>(&p),
    offsetof(StoredSasProfile, crc32)
  );

  return p;
}

bool storedProfilesSameConfiguration(const StoredSasProfile &a,
                                     const StoredSasProfile &b) {
  if (a.versionCode != b.versionCode) return false;
  if (a.profile != b.profile) return false;
  if (a.validationStyle != b.validationStyle) return false;

  // Generation is intentionally excluded.
  const uint32_t maskWithoutStability = ~PF_STABILITY_PASSED;
  if ((a.flags & maskWithoutStability) != (b.flags & maskWithoutStability)) return false;

  if (a.cashOutLimitCredits != b.cashOutLimitCredits) return false;
  if (a.aftTransferLimitCents != b.aftTransferLimitCents) return false;

  if (fixedToString(a.versionText, sizeof(a.versionText)) !=
      fixedToString(b.versionText, sizeof(b.versionText))) return false;

  if (fixedToString(a.machineSerial, sizeof(a.machineSerial)) !=
      fixedToString(b.machineSerial, sizeof(b.machineSerial))) return false;

  return true;
}

void applyStoredProfileToRuntime(const StoredSasProfile &p) {
  sasCaps = SasCapabilities();

  sasCaps.versionKnown = (p.flags & PF_VERSION_KNOWN) != 0;
  sasCaps.versionCode = p.versionCode;
  sasCaps.versionText = fixedToString(p.versionText, sizeof(p.versionText));
  sasCaps.machineSerial = fixedToString(p.machineSerial, sizeof(p.machineSerial));
  sasCaps.profile = (SasProfile)p.profile;

  sasCaps.ticketProtocolByVersion = (p.flags & PF_TICKET_BY_VERSION) != 0;
  sasCaps.systemValidationByVersion = (p.flags & PF_SYSVAL_BY_VERSION) != 0;
  sasCaps.remoteHandpayByVersion = (p.flags & PF_REMOTE94_BY_VERSION) != 0;
  sasCaps.multiDenomByVersion = (p.flags & PF_MULTIDENOM_BY_VERSION) != 0;
  sasCaps.aftProtocolByVersion = (p.flags & PF_AFT_BY_VERSION) != 0;

  sasCaps.a0Responded = (p.flags & PF_A0_RESPONDED) != 0;
  sasCaps.ticketRedemptionEnabled = (p.flags & PF_TICKET_ENABLED) != 0;
  sasCaps.validationExtensions = (p.flags & PF_VALIDATION_EXT) != 0;
  sasCaps.validationStyle = p.validationStyle;
  sasCaps.aftEnabled = (p.flags & PF_AFT_ENABLED) != 0;
  sasCaps.fortyMsSupported = (p.flags & PF_40MS_SUPPORTED) != 0;
  sasCaps.a4Responded = (p.flags & PF_A4_RESPONDED) != 0;
  sasCaps.cashOutLimitCredits = p.cashOutLimitCredits;
  sasCaps.lp74Responded = (p.flags & PF_74_RESPONDED) != 0;
  sasCaps.aftTransferLimitCents = p.aftTransferLimitCents;

  a0Known = sasCaps.a0Responded;
  a0AftSupported = sasCaps.aftEnabled;
  a0TicketRedemption = sasCaps.ticketRedemptionEnabled;
  a0FortyMsSupported = sasCaps.fortyMsSupported;
  a0ValidationStyle = sasCaps.validationStyle;
}

bool loadProfileFromEeprom() {
  StoredSasProfile a = {};
  StoredSasProfile b = {};

  const bool aOK = readStoredProfileSlot(EEPROM_SLOT0_ADDR, a);
  const bool bOK = readStoredProfileSlot(EEPROM_SLOT1_ADDR, b);

  if (!aOK && !bOK) {
    savedProfileValid = false;
    savedProfileSlot = -1;
    Serial.println("EEPROM: no valid verified SAS profile.");
    return false;
  }

  if (aOK && bOK) {
    if (a.generation >= b.generation) {
      savedProfile = a;
      savedProfileSlot = 0;
    } else {
      savedProfile = b;
      savedProfileSlot = 1;
    }
  } else if (aOK) {
    savedProfile = a;
    savedProfileSlot = 0;
  } else {
    savedProfile = b;
    savedProfileSlot = 1;
  }

  savedProfileValid = true;
  applyStoredProfileToRuntime(savedProfile);

  Serial.println();
  Serial.println("--- EEPROM VERIFIED PROFILE LOADED ---");
  Serial.print("Generation : "); Serial.println(savedProfile.generation);
  Serial.print("SAS        : "); Serial.println(fixedToString(savedProfile.versionText, sizeof(savedProfile.versionText)));
  Serial.print("Serial     : "); Serial.println(fixedToString(savedProfile.machineSerial, sizeof(savedProfile.machineSerial)));
  Serial.print("Stability  : "); Serial.print(savedProfile.stabilityGood);
  Serial.print("/"); Serial.println(savedProfile.stabilityTotal);
  Serial.println("--------------------------------------");

  return true;
}

bool saveVerifiedProfileToEeprom(uint16_t stabilityGood,
                                 uint16_t stabilityTotal) {
  if (!sasCaps.versionKnown || sasCaps.machineSerial.length() == 0) {
    Serial.println("EEPROM SAVE SKIPPED: machine identity is not known.");
    return false;
  }

  uint32_t nextGeneration = savedProfileValid ? (savedProfile.generation + 1UL) : 1UL;
  StoredSasProfile next = buildStoredProfile(
    nextGeneration,
    stabilityGood,
    stabilityTotal
  );

  // Avoid unnecessary flash writes if nothing except generation changed.
  if (savedProfileValid &&
      storedProfilesSameConfiguration(savedProfile, next) &&
      savedProfile.stabilityGood == stabilityGood &&
      savedProfile.stabilityTotal == stabilityTotal) {
    Serial.println("EEPROM: verified profile unchanged; write skipped.");
    return true;
  }

  const int targetSlot = (savedProfileSlot == 0) ? 1 : 0;
  const int targetAddress = (targetSlot == 0) ? EEPROM_SLOT0_ADDR : EEPROM_SLOT1_ADDR;

  EEPROM.put(targetAddress, next);

  StoredSasProfile verify = {};
  EEPROM.get(targetAddress, verify);

  if (!storedProfileIsValid(verify) ||
      verify.generation != next.generation) {
    Serial.println("EEPROM ERROR: profile write verification failed.");
    return false;
  }

  savedProfile = verify;
  savedProfileValid = true;
  savedProfileSlot = targetSlot;

  Serial.println();
  Serial.println("*** EEPROM VERIFIED PROFILE SAVED ***");
  Serial.print("Slot       : "); Serial.println(targetSlot);
  Serial.print("Generation : "); Serial.println(savedProfile.generation);
  Serial.print("SAS        : "); Serial.println(fixedToString(savedProfile.versionText, sizeof(savedProfile.versionText)));
  Serial.print("Serial     : "); Serial.println(fixedToString(savedProfile.machineSerial, sizeof(savedProfile.machineSerial)));
  Serial.print("Stability  : "); Serial.print(stabilityGood);
  Serial.print("/"); Serial.println(stabilityTotal);
  Serial.println("*************************************");

  return true;
}

bool cachedIdentityMatchesFresh(uint16_t freshVersionCode,
                                const String &freshSerial) {
  if (!savedProfileValid) return false;

  const String cachedSerial = fixedToString(
    savedProfile.machineSerial,
    sizeof(savedProfile.machineSerial)
  );

  return savedProfile.versionCode == freshVersionCode &&
         cachedSerial == freshSerial;
}

void printAutoProfileSummary() {
  Serial.println();
  Serial.println("--- SAS AUTO PROFILE ---");
  Serial.print("Version         : "); Serial.println(sasCaps.versionKnown ? sasCaps.versionText : "UNKNOWN");
  Serial.print("Version code    : "); Serial.println(sasCaps.versionCode);
  Serial.print("Profile         : "); Serial.println(sasProfileName(sasCaps.profile));
  Serial.print("Machine serial  : "); Serial.println(sasCaps.machineSerial);
  Serial.print("SAS address     : "); Serial.println(sasAddress);
  Serial.print("Ticket by ver   : "); Serial.println(sasCaps.ticketProtocolByVersion ? "YES" : "NO/UNKNOWN");
  Serial.print("System val by ver: "); Serial.println(sasCaps.systemValidationByVersion ? "YES" : "NO/UNKNOWN");
  Serial.print("Remote 94 by ver: "); Serial.println(sasCaps.remoteHandpayByVersion ? "YES" : "NO/UNKNOWN");
  Serial.print("AFT by ver      : "); Serial.println(sasCaps.aftProtocolByVersion ? "YES" : "NO");
  Serial.print("A0 responded    : "); Serial.println(sasCaps.a0Responded ? "YES" : "NO");
  if (sasCaps.a0Responded) {
    Serial.print("Ticket enabled  : "); Serial.println(sasCaps.ticketRedemptionEnabled ? "YES" : "NO");
    Serial.print("Validation ext  : "); Serial.println(sasCaps.validationExtensions ? "YES" : "NO");
    Serial.print("Validation style: "); Serial.println(validationName(sasCaps.validationStyle));
    Serial.print("AFT enabled     : "); Serial.println(sasCaps.aftEnabled ? "YES" : "NO");
    Serial.print("40ms indicated  : "); Serial.println(sasCaps.fortyMsSupported ? "YES" : "NO");
  }
  if (sasCaps.a4Responded) {
    Serial.print("Cashout limit   : "); Serial.print(u64ToString(sasCaps.cashOutLimitCredits)); Serial.println(" credits");
  }
  if (sasCaps.lp74Responded) {
    Serial.print("AFT xfer limit  : "); Serial.print(u64ToString(sasCaps.aftTransferLimitCents)); Serial.println(" minor units");
  }
  if (machineInfo.valid) {
    Serial.print("Accounting denom: 0x");
    if (machineInfo.denominationCode < 0x10) Serial.print('0');
    Serial.print(machineInfo.denominationCode, HEX);
    Serial.print(" = ");
    Serial.print(machineInfo.accountingDenomMinorUnits);
    Serial.println(" minor units / credit");
  }
  Serial.println("------------------------");
}

// ============================================================================
// READ-ONLY / INTERROGATION DIAGNOSTICS
// ============================================================================
bool readVersion54(bool displayResult = true) {
  uint8_t rx[64];
  uint16_t rx9[64];
  const size_t n = sasTypeR(0x54, rx, rx9, 64);

  if (n < 8 || rx[0] != sasAddress || rx[1] != 0x54 || !sasCheckResponseCrc(rx, n)) {
    sasCaps.versionKnown = false;
    sasCaps.versionCode = 0;
    sasCaps.profile = SasProfile::UNKNOWN;
    if (displayResult) showMessage("LP54 FAIL", String("RX=") + n);
    return false;
  }

  const uint8_t dataLen = rx[2];
  const size_t expected = 3U + dataLen + 2U;
  if (dataLen < 3 || n < expected) {
    if (displayResult) showMessage("LP54 LENGTH", String("LEN=") + dataLen);
    return false;
  }

  String ver;
  for (uint8_t i = 3; i < 6; i++) {
    if (rx[i] >= 32 && rx[i] <= 126) ver += (char)rx[i];
  }

  String serial;
  for (size_t i = 6; i < 3U + dataLen && i < n - 2; i++) {
    if (rx[i] >= 32 && rx[i] <= 126) serial += (char)rx[i];
  }

  sasCaps.versionText = ver;
  sasCaps.machineSerial = serial;
  sasCaps.versionCode = parseSasVersionCode(ver);
  sasCaps.versionKnown = sasCaps.versionCode != 0;
  classifySasVersion();

  Serial.print("SAS version: "); Serial.println(ver);
  Serial.print("Machine serial: "); Serial.println(serial);
  Serial.print("AUTO profile: "); Serial.println(sasProfileName(sasCaps.profile));

  if (displayResult) {
    showMessage(String("SAS:") + ver, String("P:") + sasProfileName(sasCaps.profile), 1700);
  }
  return true;
}

void diagnostic54() {
  readVersion54(true);
}

bool readCurrentCredits1A(uint64_t &creditsOut, bool displayResult = true) {
  uint8_t rx[16];
  uint16_t rx9[16];
  const size_t n = sasTypeR(0x1A, rx, rx9, 16);

  if (n < 8 || rx[0] != sasAddress || rx[1] != 0x1A || !sasCheckResponseCrc(rx, n)) {
    if (displayResult) showMessage("LP1A FAIL", String("RX=") + n);
    return false;
  }

  uint64_t credits = 0;
  if (!bcdToU64(&rx[2], 4, credits)) {
    if (displayResult) showMessage("LP1A BCD ERR", "CHECK RAW USB");
    return false;
  }

  creditsOut = credits;
  v6LastCurrentCredits = credits;
  Serial.print("Current credits: "); Serial.println(u64ToString(credits));
  if (displayResult) showMessage("CURRENT CREDIT", u64ToString(credits));
  return true;
}

void diagnostic1A(bool displayResult = true) {
  uint64_t credits = 0;
  readCurrentCredits1A(credits, displayResult);
}


// ============================================================================
// V5 MACHINE INFO / DENOMINATION
// LP1F is read-only and gives the SAS accounting denomination used by "credits".
// ============================================================================
bool readMachineInfo1F(bool displayResult = true, bool priority = false) {
  uint8_t rx[40];
  uint16_t rx9[40];

  const size_t n = priority
    ? sasTypeRPriority(0x1F, rx, rx9, sizeof(rx))
    : sasTypeR(0x1F, rx, rx9, sizeof(rx));

  // Address + command + 20 data bytes + CRC = 24 bytes.
  if (n < 24 || rx[0] != sasAddress || rx[1] != 0x1F ||
      !sasCheckResponseCrc(rx, n)) {
    machineInfo.valid = false;
    if (displayResult) showMessage("LP1F FAIL", String("RX=") + n);
    return false;
  }

  machineInfo.valid = true;
  machineInfo.gameId = bytesToPrintableAscii(&rx[2], 2);
  machineInfo.additionalId = bytesToPrintableAscii(&rx[4], 3);
  machineInfo.denominationCode = rx[7];
  machineInfo.accountingDenomMinorUnits =
    sasDenominationToMinorUnits(machineInfo.denominationCode);
  machineInfo.maxBetCredits = rx[8];
  machineInfo.paytableId = bytesToPrintableAscii(&rx[12], 6);

  Serial.println("--- LP1F MACHINE INFO ---");
  Serial.print("Game ID            : "); Serial.println(machineInfo.gameId);
  Serial.print("Additional ID      : "); Serial.println(machineInfo.additionalId);
  Serial.print("Denomination code  : 0x");
  if (machineInfo.denominationCode < 0x10) Serial.print('0');
  Serial.println(machineInfo.denominationCode, HEX);
  Serial.print("Accounting denom MU: ");
  Serial.println(machineInfo.accountingDenomMinorUnits);
  Serial.print("Max bet credits    : "); Serial.println(machineInfo.maxBetCredits);
  Serial.print("Paytable           : "); Serial.println(machineInfo.paytableId);

  if (displayResult) {
    String d = "D:0x";
    if (machineInfo.denominationCode < 0x10) d += "0";
    d += String(machineInfo.denominationCode, HEX);

    String mu = machineInfo.accountingDenomMinorUnits > 0
      ? String(" MU:") + machineInfo.accountingDenomMinorUnits
      : String(" MU:?");

    showMessage(d + mu,
                String("MAXBET:") + machineInfo.maxBetCredits,
                1800);
  }

  return true;
}

void diagnostic1F() {
  readMachineInfo1F(true, false);
}

// ============================================================================
// V5 HANDPAY OBSERVER: exception 51 -> priority LP1B
// ============================================================================
bool readHandpay1BPriority() {
  uint8_t rx[32];
  uint16_t rx9[32];
  const size_t n = sasTypeRPriority(0x1B, rx, rx9, sizeof(rx));

  if (n < 24 || rx[0] != sasAddress || rx[1] != 0x1B ||
      !sasCheckResponseCrc(rx, n)) {
    lastHandpay.valid = false;
    showV5EventScreen("HANDPAY 1B FAIL", String("RX=") + n, 1800);
    return false;
  }

  uint64_t amount = 0;
  uint64_t partial = 0;
  if (!bcdToU64(&rx[4], 5, amount) ||
      !bcdToU64(&rx[9], 2, partial)) {
    lastHandpay.valid = false;
    showV5EventScreen("HANDPAY BCD ERR", "CHECK USB RAW", 1800);
    return false;
  }

  lastHandpay.valid = true;
  lastHandpay.progressiveGroup = rx[2];
  lastHandpay.level = rx[3];
  lastHandpay.amount = amount;
  lastHandpay.partialPay = partial;
  lastHandpay.resetId = rx[11];

  Serial.println("--- HANDPAY LP1B ---");
  Serial.print("Progressive group: 0x"); Serial.println(lastHandpay.progressiveGroup, HEX);
  Serial.print("Level            : 0x"); Serial.println(lastHandpay.level, HEX);
  Serial.print("Amount raw units : "); Serial.println(u64ToString(lastHandpay.amount));
  Serial.print("Partial pay      : "); Serial.println(u64ToString(lastHandpay.partialPay));
  Serial.print("Reset ID         : "); Serial.println(lastHandpay.resetId);
  Serial.println("V5 NOTE: handpay amount unit depends on whether a progressive is included.");

  showV5EventScreen(
    String("HANDPAY ") + u64ToString(amount),
    String("RST:") + lastHandpay.resetId + " P:" + u64ToString(partial),
    3000
  );

  return true;
}

// ============================================================================
// V5 SYSTEM VALIDATION OBSERVER: exception 57 -> priority LP57
// V5 DOES NOT send a valid LP58 validation number.
// ============================================================================
bool readPendingCashout57Priority() {
  uint8_t rx[20];
  uint16_t rx9[20];
  const size_t n = sasTypeRPriority(0x57, rx, rx9, sizeof(rx));

  if (n < 10 || rx[0] != sasAddress || rx[1] != 0x57 ||
      !sasCheckResponseCrc(rx, n)) {
    pendingCashout.valid = false;
    showV5EventScreen("LP57 FAIL", String("RX=") + n, 1300);
    return false;
  }

  uint64_t amount = 0;
  if (!bcdToU64(&rx[3], 5, amount)) {
    pendingCashout.valid = false;
    showV5EventScreen("LP57 BCD ERR", "CHECK USB RAW", 1300);
    return false;
  }

  pendingCashout.valid = true;
  pendingCashout.cashoutType = rx[2];
  pendingCashout.amountMinorUnits = amount;

  Serial.println("--- SYSTEM VALIDATION LP57 ---");
  Serial.print("Cashout type : 0x"); Serial.println(pendingCashout.cashoutType, HEX);
  Serial.print("Amount MU    : "); Serial.println(u64ToString(amount));
  Serial.println("V5: observation only; no LP58 validation number is generated.");

  showV5EventScreen(
    "VALIDATION REQ",
    String("MU:") + u64ToString(amount),
    1800
  );

  return true;
}

// ============================================================================
// V5 TICKET-IN OBSERVER
// 67 -> LP70 -> LP71 with transfer code FF (status interrogation ONLY).
// ============================================================================
bool readTicket70Priority() {
  uint8_t rx[64];
  uint16_t rx9[64];
  const size_t n = sasTypeRPriority(0x70, rx, rx9, sizeof(rx));

  ticketIn = TicketInSnapshot();

  if (n < 6 || rx[0] != sasAddress || rx[1] != 0x70 ||
      !sasCheckResponseCrc(rx, n)) {
    showV5EventScreen("LP70 FAIL", String("RX=") + n, 1500);
    return false;
  }

  const uint8_t messageLen = rx[2];
  const size_t expected = 3U + (size_t)messageLen + 2U;
  if (n < expected || messageLen < 1) {
    showV5EventScreen("LP70 LENGTH ERR", String("L=") + messageLen, 1500);
    return false;
  }

  ticketIn.valid = true;
  ticketIn.ticketStatus = rx[3];

  Serial.println("--- TICKET LP70 ---");
  Serial.print("Ticket status: 0x"); Serial.println(ticketIn.ticketStatus, HEX);

  if (ticketIn.ticketStatus == 0xFF) {
    showV5EventScreen("TICKET 70", "NO ESCROW", 1500);
    return true;
  }

  if (messageLen < 7 || n < 12) {
    ticketIn.valid = false;
    showV5EventScreen("LP70 DATA SHORT", String("L=") + messageLen, 1500);
    return false;
  }

  uint64_t amount = 0;
  if (!bcdToU64(&rx[4], 5, amount)) {
    ticketIn.valid = false;
    showV5EventScreen("LP70 BCD ERR", "CHECK USB RAW", 1500);
    return false;
  }

  ticketIn.amountMinorUnits = amount;
  ticketIn.parsingCode = rx[9];

  const uint8_t availableValidation =
    (messageLen > 7U) ? (uint8_t)(messageLen - 7U) : 0U;
  ticketIn.validationLen =
    availableValidation > sizeof(ticketIn.validationData)
      ? sizeof(ticketIn.validationData)
      : availableValidation;

  for (uint8_t i = 0; i < ticketIn.validationLen; i++) {
    ticketIn.validationData[i] = rx[10 + i];
  }

  Serial.print("Ticket amount MU : "); Serial.println(u64ToString(ticketIn.amountMinorUnits));
  Serial.print("Parsing code     : 0x"); Serial.println(ticketIn.parsingCode, HEX);
  Serial.print("Validation bytes : "); Serial.println(ticketIn.validationLen);
  Serial.println("NOTE: LP70 amount may be zero when the validator cannot read an encoded amount.");

  showV5EventScreen(
    "TICKET IN 70 OK",
    String("MU:") + u64ToString(ticketIn.amountMinorUnits),
    1600
  );

  return true;
}

bool interrogateTicket71Priority(bool displayResult = true) {
  // Length=1, Transfer Code=FF. No amount/parsing/validation data is supplied.
  const uint8_t payload[2] = {0x01, 0xFF};
  uint8_t rx[64];
  uint16_t rx9[64];

  const size_t n = sasTypeSPriority(
    0x71, payload, sizeof(payload), rx, rx9, sizeof(rx)
  );

  if (n < 6 || rx[0] != sasAddress || rx[1] != 0x71 ||
      !sasCheckResponseCrc(rx, n)) {
    if (displayResult) showV5EventScreen("LP71 FF FAIL", String("RX=") + n, 1500);
    return false;
  }

  const uint8_t messageLen = rx[2];
  const size_t expected = 3U + (size_t)messageLen + 2U;
  if (n < expected || messageLen < 1) {
    if (displayResult) showV5EventScreen("LP71 LENGTH ERR", String("L=") + messageLen, 1500);
    return false;
  }

  ticketIn.machineStatus = rx[3];
  ticketIn.machineTransferMinorUnits = 0;

  if (messageLen >= 7U && n >= 12U) {
    uint64_t amt = 0;
    if (bcdToU64(&rx[4], 5, amt)) {
      ticketIn.machineTransferMinorUnits = amt;
    }
  }

  Serial.println("--- TICKET LP71 FF STATUS ---");
  Serial.print("Machine status : 0x");
  if (ticketIn.machineStatus < 0x10) Serial.print('0');
  Serial.print(ticketIn.machineStatus, HEX);
  Serial.print(" = ");
  Serial.println(ticketMachineStatusName(ticketIn.machineStatus));
  Serial.print("Transfer MU    : ");
  Serial.println(u64ToString(ticketIn.machineTransferMinorUnits));

  if (displayResult) {
    String code = "71:";
    if (ticketIn.machineStatus < 0x10) code += "0";
    code += String(ticketIn.machineStatus, HEX);

    showV5EventScreen(
      code + " " + ticketMachineStatusName(ticketIn.machineStatus),
      String("MU:") + u64ToString(ticketIn.machineTransferMinorUnits),
      2600
    );
  }

  return true;
}

// ============================================================================
// V5 TICKET-OUT OBSERVER
// Standard validation -> LP3D.
// Secure enhanced/system validation -> LP4D function FF (look-ahead only).
// ============================================================================
bool readStandardPrintedTicket3DPriority() {
  uint8_t rx[24];
  uint16_t rx9[24];
  const size_t n = sasTypeRPriority(0x3D, rx, rx9, sizeof(rx));

  printedTicket = PrintedTicketSnapshot();

  if (n < 13 || rx[0] != sasAddress || rx[1] != 0x3D ||
      !sasCheckResponseCrc(rx, n)) {
    showV5EventScreen("LP3D FAIL", String("RX=") + n, 1500);
    return false;
  }

  uint64_t amount = 0;
  if (!bcdToU64(&rx[6], 5, amount)) {
    showV5EventScreen("LP3D BCD ERR", "CHECK USB RAW", 1500);
    return false;
  }

  printedTicket.valid = true;
  printedTicket.enhanced = false;
  printedTicket.amountMinorUnits = amount;

  Serial.println("--- STANDARD PRINTED TICKET LP3D ---");
  Serial.print("Ticket amount MU: "); Serial.println(u64ToString(amount));

  showV5EventScreen(
    "TICKET PRINTED",
    String("MU:") + u64ToString(amount),
    2600
  );

  return true;
}

bool lookAheadEnhancedValidation4DPriority() {
  const uint8_t payload[1] = {0xFF}; // look ahead; do not mark record read
  uint8_t rx[48];
  uint16_t rx9[48];

  const size_t n = sasTypeSPriority(
    0x4D, payload, sizeof(payload), rx, rx9, sizeof(rx)
  );

  printedTicket = PrintedTicketSnapshot();

  if (n < 35 || rx[0] != sasAddress || rx[1] != 0x4D ||
      !sasCheckResponseCrc(rx, n)) {
    showV5EventScreen("LP4D FF FAIL", String("RX=") + n, 1500);
    return false;
  }

  uint64_t amount = 0;
  if (!bcdToU64(&rx[19], 5, amount)) {
    showV5EventScreen("LP4D BCD ERR", "CHECK USB RAW", 1500);
    return false;
  }

  printedTicket.valid = true;
  printedTicket.enhanced = true;
  printedTicket.validationType = rx[2];
  printedTicket.bufferIndex = rx[3];
  printedTicket.amountMinorUnits = amount;
  printedTicket.ticketNumber = (uint16_t)((uint16_t)rx[24] << 8) | rx[25];

  Serial.println("--- ENHANCED/SYSTEM VALIDATION LP4D FF ---");
  Serial.print("Validation type : 0x"); Serial.println(printedTicket.validationType, HEX);
  Serial.print("Buffer index    : "); Serial.println(printedTicket.bufferIndex);
  Serial.print("Amount MU       : "); Serial.println(u64ToString(amount));
  Serial.print("Ticket number   : "); Serial.println(printedTicket.ticketNumber);
  Serial.println("LP4D FF is look-ahead only; it does not mark the record read.");

  showV5EventScreen(
    "TICKET PRINT 4D",
    String("MU:") + u64ToString(amount),
    2600
  );

  return true;
}

// ============================================================================
// V5 EVENT STATE ENGINE
// ============================================================================
void serviceV5EventEngine() {
  if (!sasOnline || v5EventCount == 0) return;

  uint8_t ev = 0;
  if (!popV5Event(ev)) return;

  v5LastEventProcessedMs = millis();
  v6TraceAdd(1, ev, v5EventCount);

  switch (ev) {
    case 0x51:
      v5State = V5RuntimeState::HANDPAY_PENDING;
      v6BeforeMeters = V6MeterSnapshot();
      v6ReadSelectedMeters2F(v6BeforeMeters, true, false);
      v6BeginObservedJournal(V6JournalType::HANDPAY, 0);
      readHandpay1BPriority();
      break;

    case 0x52:
      lastHandpay.valid = false;
      v5State = V5RuntimeState::NORMAL;
      if (v6LatestJournalValid && v6LatestJournal.type == (uint8_t)V6JournalType::HANDPAY)
        v6UpdateJournal(V6JournalState::DONE, 0x00);
      showV5EventScreen("HANDPAY RESET", "EVENT 52", 1800);
      break;

    case 0x57:
      v5State = V5RuntimeState::SYSTEM_VALIDATION_PENDING;
      v6BeforeMeters = V6MeterSnapshot();
      v6ReadSelectedMeters2F(v6BeforeMeters, true, false);
      v6BeforeMeterContext = 0x57;
      if (readPendingCashout57Priority())
        v6BeginObservedJournal(V6JournalType::TICKET_OUT, pendingCashout.amountMinorUnits);
      break;

    case 0x67:
      v5State = V5RuntimeState::TICKET_IN_ESCROW;
      v6BeforeMeters = V6MeterSnapshot();
      v6ReadSelectedMeters2F(v6BeforeMeters, true, false);
      v6BeforeMeterContext = 0x67;
      if (readTicket70Priority()) {
        v6BeginObservedJournal(V6JournalType::TICKET_IN, ticketIn.amountMinorUnits);
        v5State = V5RuntimeState::TICKET_STATUS;
        interrogateTicket71Priority(true);
        if (v6LatestJournalValid) v6UpdateJournal(V6JournalState::PENDING, ticketIn.machineStatus);
        Serial.print("TICKET STATUS KO: "); Serial.println(v6TicketStatusKo(ticketIn.machineStatus));
      }
      break;

    case 0x68:
      v5State = V5RuntimeState::TICKET_STATUS;
      if (interrogateTicket71Priority(true)) {
        Serial.print("TICKET FINAL KO: "); Serial.println(v6TicketStatusKo(ticketIn.machineStatus));
        v6AfterMeters = V6MeterSnapshot();
        v6ReadSelectedMeters2F(v6AfterMeters, true, false);
        const bool sasOK = ticketIn.machineStatus <= 0x02;
        v6LastMeterVerifyPassed = sasOK && v6VerifyMeterDelta(V6JournalType::TICKET_IN, v6BeforeMeters, v6AfterMeters);
        if (v6LatestJournalValid && v6LatestJournal.type == (uint8_t)V6JournalType::TICKET_IN) {
          if (sasOK) v6UpdateJournal(v6LastMeterVerifyPassed ? V6JournalState::DONE : V6JournalState::SAS_VERIFIED, ticketIn.machineStatus);
          else v6UpdateJournal(V6JournalState::FAILED, ticketIn.machineStatus);
        }
      }
      break;

    case 0x3D:
      v5State = V5RuntimeState::TICKET_PRINTED;
      if (sasCaps.a0Responded && sasCaps.validationStyle != 0) {
        if (!lookAheadEnhancedValidation4DPriority()) readStandardPrintedTicket3DPriority();
      } else {
        readStandardPrintedTicket3DPriority();
      }
      v6AfterMeters = V6MeterSnapshot();
      v6ReadSelectedMeters2F(v6AfterMeters, true, false);
      if (v6LatestJournalValid && v6LatestJournal.type == (uint8_t)V6JournalType::TICKET_OUT) {
        v6LastMeterVerifyPassed = v6VerifyMeterDelta(V6JournalType::TICKET_OUT, v6BeforeMeters, v6AfterMeters);
        v6UpdateJournal(v6LastMeterVerifyPassed ? V6JournalState::DONE : V6JournalState::METERS_VERIFIED, 0x00);
      }
      break;

    case 0x66:
    case 0x6A:
    case 0x6B:
      // AFT request events are observations, not ownership of a host money
      // transaction.  Never create a competing generic AFT RAM transaction here.
      // The real RFID cashout/split paths already own their RAM transaction state
      // BEFORE LP72 is sent.  V6.5.2 incorrectly created a second AFT transaction record
      // here and could hide the real RFID journal.
      v6BeforeMeters = V6MeterSnapshot();
      v6ReadSelectedMeters2F(v6BeforeMeters, true, false);
      v6BeforeMeterContext = ev;
      if (v6JournalIsActiveNonTerminal()) {
        Serial.print("AFT EVENT 0x"); Serial.print(ev, HEX);
        Serial.print(" observed; active RAM transaction retained type=");
        Serial.print(v6LatestJournal.type);
        Serial.print(" state="); Serial.println(v6LatestJournal.state);
      } else {
        Serial.print("AFT EVENT 0x"); Serial.print(ev, HEX);
        Serial.println(" observed; no competing observer RAM transaction created.");
      }
      showV5EventScreen("AFT REQUEST", String("EV:") + String(ev, HEX), 1200);
      break;

    case 0x69:
      v5State = V5RuntimeState::AFT_COMPLETE_SEEN;
      if (v6InterrogateAft72(0xFF, true, true)) {
        v6AfterMeters = V6MeterSnapshot();
        v6ReadSelectedMeters2F(v6AfterMeters, true, false);
        const bool sasOK = v6LastAft.transferStatus == 0x00 || v6LastAft.transferStatus == 0x01;
        v6LastMeterVerifyPassed = sasOK && v6VerifyMeterDelta(V6JournalType::AFT, v6BeforeMeters, v6AfterMeters);
        if (v6LatestJournalValid && v6LatestJournal.type == (uint8_t)V6JournalType::AFT) {
          if (sasOK) v6UpdateJournal(v6LastMeterVerifyPassed ? V6JournalState::DONE : V6JournalState::SAS_VERIFIED, v6LastAft.transferStatus);
          else if (v6LastAft.transferStatus == 0x40) v6UpdateJournal(V6JournalState::PENDING, 0x40);
          else v6UpdateJournal(V6JournalState::FAILED, v6LastAft.transferStatus);
        }
      } else {
        showV5EventScreen("AFT COMPLETE", "72 FF FAIL", 1600);
      }
      break;

    case 0x6F:
      v5State = V5RuntimeState::GAME_LOCKED;
      showV5EventScreen("GAME LOCKED", "EVENT 6F", 2200);
      break;

    default: {
      String text = "EV 0x";
      if (ev < 0x10) text += '0';
      text += String(ev, HEX);
      showV5EventScreen("SAS EVENT", text, 900);
      break;
    }
  }
}

void showV5EventState() {
  Serial.println("--- V5 EVENT STATE ---");
  Serial.print("State         : "); Serial.println(v5StateName(v5State));
  Serial.print("Queue         : "); Serial.println(v5EventCount);
  Serial.print("Dropped       : "); Serial.println(v5EventDropped);
  Serial.print("Last GP event : 0x"); Serial.println(lastEvent, HEX);

  if (lastHandpay.valid) {
    Serial.print("Handpay amount: "); Serial.println(u64ToString(lastHandpay.amount));
    Serial.print("Handpay reset : "); Serial.println(lastHandpay.resetId);
    String gateReason;
    const bool gate = v6Handpay94GateEligible(gateReason);
    Serial.print("LP94 gate     : "); Serial.print(gate ? "ELIGIBLE" : "BLOCKED"); Serial.print(" / "); Serial.println(gateReason);
    Serial.println("LP94/A8 transmit is intentionally absent in V6 validation build.");
  }

  if (pendingCashout.valid) {
    Serial.print("Pending cashout MU: ");
    Serial.println(u64ToString(pendingCashout.amountMinorUnits));
  }

  if (ticketIn.valid) {
    Serial.print("Ticket status 70: 0x"); Serial.println(ticketIn.ticketStatus, HEX);
    Serial.print("Ticket 71 status: 0x"); Serial.print(ticketIn.machineStatus, HEX);
    Serial.print(" "); Serial.println(ticketMachineStatusName(ticketIn.machineStatus));
  }

  if (printedTicket.valid) {
    Serial.print("Printed ticket MU: ");
    Serial.println(u64ToString(printedTicket.amountMinorUnits));
  }

  showMessage(
    String("STATE:") + v5StateName(v5State),
    String("Q:") + v5EventCount + " DROP:" + v5EventDropped,
    1800
  );
}

struct CardDepositPreflightResult {
  bool ok = false;
  String reason = "BLOCK";
  uint64_t requestMinorUnits = 0;
};

CardDepositPreflightResult evaluateCardDepositPreflight(bool printReport = true) {
  CardDepositPreflightResult r;

  if (!cardActive) {
    r.reason = "NO CARD";
  } else if (!sasOnline) {
    r.reason = "SAS OFFLINE";
  } else if (!v6BootSelfTestPassed) {
    r.reason = "SELFTEST FAIL";
  } else if (v6RecoveryRequired) {
    r.reason = "RECOVERY REQUIRED";
  } else if (inputValue == 0 || inputValue > cardBalance) {
    r.reason = "INVALID AMOUNT";
  } else if (!machineInfo.valid || machineInfo.accountingDenomMinorUnits == 0) {
    r.reason = "DENOM UNKNOWN";
  } else if (sasCaps.a4Responded &&
             (uint64_t)inputValue > sasCaps.cashOutLimitCredits) {
    r.reason = "OVER A4 LIMIT";
  } else {
    r.requestMinorUnits =
      (uint64_t)inputValue *
      (uint64_t)machineInfo.accountingDenomMinorUnits;

    if (sasCaps.lp74Responded &&
        r.requestMinorUnits > sasCaps.aftTransferLimitCents) {
      r.reason = "OVER AFT LIMIT";
    } else {
      r.ok = true;
      r.reason = "READY";
    }
  }

  if (printReport) {
    Serial.println("--- CARD AMOUNT PREFLIGHT ---");
    Serial.print("Card balance      : "); Serial.println(cardBalance);
    Serial.print("Requested credits : "); Serial.println(inputValue);

    Serial.print("A4 limit credits  : ");
    if (sasCaps.a4Responded) Serial.println(u64ToString(sasCaps.cashOutLimitCredits));
    else Serial.println("UNKNOWN");

    Serial.print("Accounting denom  : ");
    if (machineInfo.valid) {
      Serial.print("0x"); Serial.print(machineInfo.denominationCode, HEX);
      Serial.print(" -> "); Serial.print(machineInfo.accountingDenomMinorUnits);
      Serial.println(" minor units/credit");
    } else {
      Serial.println("UNKNOWN");
    }

    Serial.print("Request MU        : ");
    if (r.requestMinorUnits > 0) Serial.println(u64ToString(r.requestMinorUnits));
    else Serial.println("UNKNOWN");

    Serial.print("AFT limit MU      : ");
    if (sasCaps.lp74Responded) Serial.println(u64ToString(sasCaps.aftTransferLimitCents));
    else Serial.println("N/A/UNKNOWN");

    Serial.print("Result            : "); Serial.println(r.ok ? "READY" : r.reason);
  }

  return r;
}

void safeCardAmountPreflight() {
  const CardDepositPreflightResult r = evaluateCardDepositPreflight(true);
  if (!r.ok) {
    showMessage("PREFLIGHT BLOCK", r.reason, 1500);
    return;
  }

  showMessage(
    String("READY CR:") + inputValue,
    String("MU:") + u64ToString(r.requestMinorUnits),
    1500
  );
}

bool sendLegacyBonus8A(uint32_t credits, uint8_t &ackByte) {
#if V6_ENABLE_LEGACY_BONUS_8A
  uint8_t bcd[4];
  if (!u32ToBcd4(credits, bcd)) return false;

  // ArduinoTITO LegacyBonus:
  // [amount BCD1..BCD4][type=0x00], Type-S CRC added by sasTypeS().
  const uint8_t payload[5] = {
    bcd[0], bcd[1], bcd[2], bcd[3], 0x00
  };

  uint8_t rx[16] = {};
  uint16_t rx9[16] = {};

  Serial.println("--- LEGACY BONUS 0x8A ---");
  Serial.print("Credits BCD       : ");
  for (uint8_t i = 0; i < 4; i++) {
    if (bcd[i] < 0x10) Serial.print('0');
    Serial.print(bcd[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println("Type              : 0x00");

  const size_t n = sasTypeS(0x8A, payload, sizeof(payload), rx, rx9, sizeof(rx));
  if (n >= 1 && rx[0] == sasAddress) {
    ackByte = rx[0];
    Serial.println("8A ACK            : PASS (address 01)");
    return true;
  }

  ackByte = 0xFF;
  Serial.println("8A ACK            : NOT RECEIVED");
  return false;
#else
  (void)credits;
  ackByte = 0xFF;
  Serial.println("8A DISABLED IN CONFIG");
  return false;
#endif
}

bool waitForExactCreditMeter(uint64_t expectedCredits,
                             uint64_t &observedCredits,
                             uint32_t timeoutMs = 5000UL) {
  const uint32_t start = millis();
  uint8_t attempts = 0;

  while ((uint32_t)(millis() - start) < timeoutMs) {
    attempts++;

    uint64_t nowCredits = 0;
    if (readCurrentCredits1A(nowCredits, false)) {
      observedCredits = nowCredits;
      Serial.print("POST LP1A attempt "); Serial.print(attempts);
      Serial.print(" observed="); Serial.print(u64ToString(nowCredits));
      Serial.print(" expected="); Serial.println(u64ToString(expectedCredits));

      if (nowCredits == expectedCredits) return true;
      if (nowCredits > expectedCredits) return false;
    }

    warmGeneralPoll(120UL, false);
    delay(80);
  }

  return false;
}

// V6.5.14: prove a no-movement result without resending any value command.
// Three independent LP1A reads must all equal target.
bool v6ConfirmStableCreditExact(uint64_t target, uint8_t samples) {
  if (samples < 2) samples = 2;
  for (uint8_t i = 0; i < samples; ++i) {
    warmGeneralPoll(120UL, false);
    delay(80);
    uint64_t now = 0;
    if (!readCurrentCredits1A(now, false) || now != target) return false;
  }
  return true;
}

bool v6SyncAftCycleBeforeValueCommand(const char *context,
                                      uint32_t timeoutMs,
                                      bool showLcd);
bool v6AftTransferRejected(uint8_t status);

bool executeCardDepositTransaction() {
#if !V6_ENABLE_LEGACY_BONUS_8A
  showMessage("8A DISABLED", "CHECK CONFIG", 1200);
  return false;
#else
  const CardDepositPreflightResult pf = evaluateCardDepositPreflight(true);
  if (!pf.ok) {
    showMessage("DEPOSIT BLOCKED", pf.reason, 1500);
    return false;
  }

  // A previous AFT may have completed but still require an FF acknowledgement.
  // Do this before 0x8A so deposit-after-cashout is repeatable.
  if (sasCaps.aftProtocolByVersion || sasCaps.lp74Responded) {
    if (!v6SyncAftCycleBeforeValueCommand("DEPOSIT", 2500UL, true)) {
      Serial.println("DEPOSIT BLOCK: AFT cycle is not closed/ready.");
      return false;
    }
  }

  // Re-read the card immediately before the transaction.
  uint32_t freshBalance = 0;
  if (!readBalanceFromCard(freshBalance) || freshBalance != cardBalance) {
    Serial.println("DEPOSIT BLOCK: RFID balance/read changed before transaction.");
    showMessage("CARD READ FAIL", "NO CREDIT SEND", 1500);
    return false;
  }

  const uint32_t amountCredits = inputValue;
  const uint32_t newBalance = cardBalance - amountCredits;

  uint64_t beforeCredits = 0;
  if (!readCurrentCredits1A(beforeCredits, false)) {
    Serial.println("DEPOSIT BLOCK: PRE LP1A failed.");
    showMessage("PRE LP1A FAIL", "NO CREDIT SEND", 1500);
    return false;
  }

  if (beforeCredits > 99999999ULL ||
      (uint64_t)amountCredits > (99999999ULL - beforeCredits)) {
    Serial.println("DEPOSIT BLOCK: expected credit meter would exceed 8-digit LP1A range.");
    showMessage("CREDIT RANGE", "NO CREDIT SEND", 1500);
    return false;
  }

  const uint64_t expectedCredits = beforeCredits + (uint64_t)amountCredits;

  Serial.println();
  Serial.println("=== RFID -> S6000 ATOMIC DEPOSIT ===");
  Serial.print("PRE credits       : "); Serial.println(u64ToString(beforeCredits));
  Serial.print("Add credits       : "); Serial.println(amountCredits);
  Serial.print("Expected POST     : "); Serial.println(u64ToString(expectedCredits));
  Serial.print("RFID before       : "); Serial.println(cardBalance);
  Serial.print("RFID after        : "); Serial.println(newBalance);

  // Best-effort LP2F evidence. LP1A exact delta is the mandatory proof.
  v6BeforeMeters = V6MeterSnapshot();
  if (!v6ReadSelectedMeters2F(v6BeforeMeters, true, false)) {
    v6BeforeMeters = V6MeterSnapshot();
    v6BeforeMeters.valid = true;
    v6BeforeMeters.currentCredits = beforeCredits;
  } else {
    v6BeforeMeters.currentCredits = beforeCredits;
  }

  if (!v6PrepareAtomicCardJournal(newBalance, amountCredits, pf.requestMinorUnits)) {
    Serial.println("DEPOSIT BLOCK: could not initialize RAM transaction.");
    showMessage("RAM TX FAIL", "NO CREDIT SEND", 1500);
    return false;
  }

  showMessage(String("SEND CR:") + amountCredits, "8A + VERIFY", 700);

  uint8_t ackByte = 0xFF;
  const bool ackOK = sendLegacyBonus8A(amountCredits, ackByte);

  // Never decide by ACK alone. The machine credit meter is authoritative.
  uint64_t afterCredits = beforeCredits;
  const bool exactMeterOK =
    waitForExactCreditMeter(expectedCredits, afterCredits, 5000UL);

  v6AfterMeters = V6MeterSnapshot();
  v6AfterMeters.valid = true;
  v6AfterMeters.currentCredits = afterCredits;

  Serial.print("8A ACK result      : "); Serial.println(ackOK ? "PASS" : "NO/UNKNOWN");
  Serial.print("POST LP1A          : "); Serial.println(u64ToString(afterCredits));
  Serial.print("Exact delta        : "); Serial.println(exactMeterOK ? "PASS" : "FAIL/AMBIGUOUS");

  if (!exactMeterOK) {
    // V6.5.14: distinguish a definite NO-MOVEMENT failure from a truly ambiguous result.
    // Never resend 0x8A here. If LP1A is still exactly PRE and remains there for
    // three independent reads, the requested value was not applied. Clear RAM state
    // automatically so the next transaction is not blocked.
    if (afterCredits == beforeCredits && v6ConfirmStableCreditExact(beforeCredits, 3)) {
      v6UpdateJournal(V6JournalState::FAILED, ackOK ? ackByte : 0x00);
      Serial.println("DEPOSIT FAILED: LP1A unchanged at PRE; no value movement proved.");
      Serial.println("RFID unchanged. Auto-clearing volatile transaction state.");
      v6ClearRuntimeTransaction("DEPOSIT NO MOVEMENT");
      inputValue = 0;
      cancelMoneyConfirm(false);
      showMessage("DEPOSIT FAILED", "NO MOVE / RESET", 1500);
      return false;
    }

    // PRE < NOW < EXPECT, NOW > EXPECT, or unstable reads are genuinely ambiguous.
    // Keep only volatile recovery state; a power reset still forgets it.
    v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED, ackOK ? ackByte : 0xFF);
    Serial.println("DEPOSIT AMBIGUOUS: RFID unchanged; RAM recovery retained (no resend).");
    showMessage("VERIFY AMBIG", "RFID UNCHANGED", 1800);
    return false;
  }

  // Exact LP1A delta independently proves that the requested credits arrived.
  if (!v6UpdateJournal(V6JournalState::SAS_VERIFIED, ackOK ? ackByte : 0x01)) {
    Serial.println("DEPOSIT ERROR: could not update RAM SAS_VERIFIED state.");
    showMessage("JOURNAL VERIFY", "RFID UNCHANGED", 1800);
    return false;
  }

  if (!cardActive || !v6SameActiveCardUid(v6LatestJournal)) {
    Serial.println("DEPOSIT CREDITED but card is not the same active UID.");
    Serial.println("RFID NOT CHANGED; RAM transaction retained until success or reset.");
    showMessage("CREDIT OK", "RETAG SAME CARD", 1800);
    return false;
  }

  if (!v6AtomicCommitCardBalance(newBalance)) {
    Serial.println("DEPOSIT CREDITED but RFID commit failed.");
    Serial.println("Do NOT resend 8A. Retag the same card for recovery.");
    showMessage("CARD WRITE FAIL", "DO NOT RESEND", 1800);
    return false;
  }

  // RFID_COMMITTED is already terminal, but write DONE as the final transaction record.
  v6UpdateJournal(V6JournalState::DONE, ackOK ? ackByte : 0x01);

  Serial.println("*** DEPOSIT SUCCESS ***");
  Serial.print("S6000 credits     : "); Serial.println(u64ToString(afterCredits));
  Serial.print("RFID new balance  : "); Serial.println(cardBalance);
  Serial.println("=== TRANSACTION DONE ===");
  Serial.println();

  inputValue = 0;
  showMessage("DEPOSIT SUCCESS", String("BAL:") + cardBalance, 1800);
  return true;
#endif
}



// Explicit declaration required before the new V6.4 AFT helpers.
// Keep the default argument here; the later definition has no default.
bool probeAftStatus74(bool displayResult = true);

bool v6AftAssetIsNonZero() {
  return sasCaps.aftAssetNumber[0] != 0 ||
         sasCaps.aftAssetNumber[1] != 0 ||
         sasCaps.aftAssetNumber[2] != 0 ||
         sasCaps.aftAssetNumber[3] != 0;
}

bool v6PrepareRfidCashoutJournal(uint32_t newBalance,
                                 uint32_t amountCredits,
                                 uint64_t amountMinorUnits,
                                 const char *transactionId) {
  if (!cardActive || activeCardUidLen == 0) return false;
  if (v6RecoveryRequired) return false;
  if (newBalance < cardBalance) return false; // cashout must add to card

  V6JournalRecord r = {};
  r.type = (uint8_t)V6JournalType::RFID_CASHOUT;
  r.state = (uint8_t)V6JournalState::PREPARED;
  r.sasStatus = 0xFF;
  r.cardUidLen = activeCardUidLen;
  for (uint8_t i = 0; i < activeCardUidLen; i++) r.cardUid[i] = activeCardUid[i];
  r.amountCredits = amountCredits;
  r.amountMinorUnits = amountMinorUnits;
  r.balanceBefore = cardBalance;
  r.balanceAfter = newBalance;
  r.beforeMeters = v6StoreMeters(v6BeforeMeters);
  if (transactionId) {
    for (uint8_t i = 0; i < sizeof(r.transactionId) - 1 && transactionId[i]; i++)
      r.transactionId[i] = transactionId[i];
  }
  return v6WriteJournal(r);
}

bool v6ParseAft72Response(const uint8_t *rx, size_t n, bool displayResult = false) {
  v6LastAft = V6AftSnapshot();

  if (n < 7 || rx == nullptr || rx[0] != sasAddress || rx[1] != 0x72 ||
      !sasCheckResponseCrc(rx, n)) {
    if (displayResult) showMessage("AFT72 RX FAIL", String("RX=") + n, 1200);
    return false;
  }

  const uint8_t len = rx[2];
  const size_t expected = 3U + (size_t)len + 2U;
  if (n < expected || len < 2) return false;

  v6LastAft.valid = true;
  v6LastAft.bufferPosition = rx[3];
  v6LastAft.transferStatus = rx[4];

  if (len >= 4) {
    v6LastAft.receiptStatus = rx[5];
    v6LastAft.transferType = rx[6];
  }

  if (len >= 19 && n >= 24) {
    if (!bcdToU64(&rx[7], 5, v6LastAft.cashableMinorUnits)) return false;
    if (!bcdToU64(&rx[12], 5, v6LastAft.restrictedMinorUnits)) return false;
    if (!bcdToU64(&rx[17], 5, v6LastAft.nonrestrictedMinorUnits)) return false;
  }

  Serial.print("AFT72 status=0x");
  if (v6LastAft.transferStatus < 0x10) Serial.print('0');
  Serial.print(v6LastAft.transferStatus, HEX);
  Serial.print(" ");
  Serial.print(v6AftStatusShort(v6LastAft.transferStatus));
  Serial.print(" type=0x");
  if (v6LastAft.transferType < 0x10) Serial.print('0');
  Serial.println(v6LastAft.transferType, HEX);

  if (displayResult) {
    showMessage(
      String("AFT72:") + String(v6LastAft.transferStatus, HEX),
      v6AftStatusShort(v6LastAft.transferStatus),
      1600
    );
  }
  return true;
}


bool v6AftReceiptTerminal(uint8_t status) {
  return status == 0x00 || status == 0xFF;
}

bool v6AftTransferRejected(uint8_t status) {
  return status != 0x00 &&
         status != 0x01 &&
         status != 0x40 &&
         status != 0xFF;
}

bool v6AftCycleSnapshotFinal() {
  if (!v6LastAft.valid) return false;
  if (v6LastAft.transferStatus == 0x40) return false;
  // C0/C1 are errors for the interrogation itself/current cycle context,
  // not an acknowledgement that a previous transfer cycle is closed.
  if (v6LastAft.transferStatus == 0xC0 ||
      v6LastAft.transferStatus == 0xC1) return false;
  if (!v6AftReceiptTerminal(v6LastAft.receiptStatus)) return false;
  return true;
}

// SAS 6.01/6.02: FE is read-only and does NOT acknowledge completion.
// FF retrieves/acknowledges the current/final transfer status. Before any new
// value-changing operation we use FF to close a stale completed AFT cycle and
// refuse to proceed while a transfer or receipt is still pending.
bool v6SyncAftCycleBeforeValueCommand(const char *context,
                                      uint32_t timeoutMs = 2500UL,
                                      bool showLcd = true) {
  if (!sasOnline) return false;

  const uint32_t start = millis();
  uint8_t tries = 0;

  while ((uint32_t)(millis() - start) < timeoutMs) {
    tries++;
    warmGeneralPoll(120UL, false);

    if (v6InterrogateAft72(0xFF, true, false)) {
      Serial.print("AFT CYCLE SYNC [");
      Serial.print(context ? context : "?");
      Serial.print("] try=");
      Serial.print(tries);
      Serial.print(" status=0x");
      if (v6LastAft.transferStatus < 0x10) Serial.print('0');
      Serial.print(v6LastAft.transferStatus, HEX);
      Serial.print(" receipt=0x");
      if (v6LastAft.receiptStatus < 0x10) Serial.print('0');
      Serial.println(v6LastAft.receiptStatus, HEX);

      if (v6AftCycleSnapshotFinal()) {
        Serial.println("AFT CYCLE READY: previous/current cycle is terminal and acknowledged.");
        return true;
      }
    }

    delay(120);
  }

  Serial.print("AFT CYCLE NOT READY [");
  Serial.print(context ? context : "?");
  Serial.println("]: pending/unacknowledged or no FF response.");
  if (showLcd) showMessage("AFT CYCLE WAIT", "TRY AGAIN LATER", 1600);
  return false;
}

// After an initiate, FF is used until BOTH transfer and receipt states are
// terminal. This is interrogation only; it never resends the money command.
bool v6WaitAftFinalStatus(uint32_t timeoutMs = 6000UL) {
  const uint32_t start = millis();
  uint8_t tries = 0;

  while ((uint32_t)(millis() - start) < timeoutMs) {
    tries++;
    warmGeneralPoll(150UL, false);
    delay(80);

    if (v6InterrogateAft72(0xFF, true, false)) {
      Serial.print("AFT FINAL FF try=");
      Serial.print(tries);
      Serial.print(" status=0x");
      if (v6LastAft.transferStatus < 0x10) Serial.print('0');
      Serial.print(v6LastAft.transferStatus, HEX);
      Serial.print(" receipt=0x");
      if (v6LastAft.receiptStatus < 0x10) Serial.print('0');
      Serial.println(v6LastAft.receiptStatus, HEX);

      if (v6AftCycleSnapshotFinal()) return true;
    }
  }

  Serial.println("AFT72 final/receipt status timeout; NEVER AUTO RESEND.");
  return false;
}


// A clear terminal rejection (81/82/87/C0/etc.) with an unchanged LP1A meter
// proves that this attempt moved no value. Such a transaction is FAILED,
// terminal, and MUST NOT poison all later deposit/cashout operations.
// If LP1A changed or cannot be read, keep fail-closed recovery instead.
bool v6AbortOnClearAftReject(uint64_t beforeCredits, const char *context) {
  if (!v6LastAft.valid || !v6AftTransferRejected(v6LastAft.transferStatus))
    return false;

  warmGeneralPoll(180UL, false);

  uint64_t nowCredits = beforeCredits;
  const bool readOK = readCurrentCredits1A(nowCredits, false);

  Serial.print("AFT CLEAR REJECT [");
  Serial.print(context ? context : "?");
  Serial.print("] status=0x");
  if (v6LastAft.transferStatus < 0x10) Serial.print('0');
  Serial.print(v6LastAft.transferStatus, HEX);
  Serial.print(" PRE=");
  Serial.print(u64ToString(beforeCredits));
  Serial.print(" NOW=");
  Serial.println(readOK ? u64ToString(nowCredits) : String("READ FAIL"));

  v6AfterMeters = V6MeterSnapshot();
  v6AfterMeters.valid = readOK;
  v6AfterMeters.currentCredits = nowCredits;

  if (readOK && nowCredits == beforeCredits) {
    v6UpdateJournal(V6JournalState::FAILED, v6LastAft.transferStatus);
    Serial.println("AFT REJECT CONFIRMED: no credit moved; volatile transaction will be cleared.");
    v6ClearRuntimeTransaction("AFT REJECTED / NO MOVEMENT");
    inputValue = 0;
    cancelMoneyConfirm(false);
    showMessage("AFT REJECTED",
                String("ST:") + String(v6LastAft.transferStatus, HEX),
                1600);
  } else {
    v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED,
                    v6LastAft.transferStatus);
    Serial.println("AFT REJECT AMBIGUOUS: credit changed/unreadable; recovery retained.");
    showMessage("AFT AMBIGUOUS", "CHECK JOURNAL", 1800);
  }

  return true;
}

bool v6InitiateAftCashoutToHost(uint64_t cashableMinorUnits,
                                const char transactionId[7]) {
#if !V6_ENABLE_AFT_VALUE_COMMANDS
  (void)cashableMinorUnits;
  (void)transactionId;
  Serial.println("AFT VALUE COMMANDS DISABLED");
  return false;
#else
  uint8_t cashableBcd[5];
  if (!u64ToBcd5(cashableMinorUnits, cashableBcd)) return false;
  if (!v6AftAssetIsNonZero()) return false;

  // SAS 6.02 Table 8.3a. Six-char transaction ID keeps total tx at 64 bytes.
  // Transfer code 00 = exact/full requested transfer only.
  // Transfer type 80 = in-house amount from gaming machine to host.
  // Transfer flags 00 = do NOT request machine cashout of the remainder.
  const uint8_t txIdLen = 6;
  const uint8_t dataLen = (uint8_t)(53U + txIdLen); // bytes after length, before CRC
  uint8_t payload[60];
  uint8_t p = 0;

  payload[p++] = dataLen;
  payload[p++] = 0x00; // transfer code: exact/full requested amount only
  payload[p++] = 0x00; // transaction index: current
  payload[p++] = 0x80; // machine -> host, in-house

  for (uint8_t i = 0; i < 5; i++) payload[p++] = cashableBcd[i];
  for (uint8_t i = 0; i < 5; i++) payload[p++] = 0x00; // restricted
  for (uint8_t i = 0; i < 5; i++) payload[p++] = 0x00; // nonrestricted

  payload[p++] = 0x00; // transfer flags: remainder stays on CREDIT

  for (uint8_t i = 0; i < 4; i++) payload[p++] = sasCaps.aftAssetNumber[i];

  // In-house transfer: registration key all zeros.
  for (uint8_t i = 0; i < 20; i++) payload[p++] = 0x00;

  payload[p++] = txIdLen;
  for (uint8_t i = 0; i < txIdLen; i++) payload[p++] = (uint8_t)transactionId[i];

  for (uint8_t i = 0; i < 4; i++) payload[p++] = 0x00; // expiration
  payload[p++] = 0x00; payload[p++] = 0x00;             // pool ID
  payload[p++] = 0x00;                                  // receipt data length
  payload[p++] = 0x00; payload[p++] = 0x00;             // lock timeout

  if (p != sizeof(payload) || payload[0] != 59) {
    Serial.print("AFT72 INTERNAL LENGTH ERROR p=");
    Serial.println(p);
    return false;
  }

  Serial.println("--- AFT72 MACHINE -> RFID TEST ---");
  Serial.print("Transfer type      : 0x80");
  Serial.println(" (machine -> host in-house)");
  Serial.print("Transfer code      : 0x00");
  Serial.println(" (exact amount only)");
  Serial.print("Cashable MU        : ");
  Serial.println(u64ToString(cashableMinorUnits));
  Serial.print("Transaction ID     : ");
  Serial.println(transactionId);
  Serial.println("Transfer flags     : 0x00 (leave remainder on CREDIT)");

  uint8_t rx[96] = {};
  uint16_t rx9[96] = {};

  warmGeneralPoll(300UL, false);
  const size_t n = sasTypeS(0x72, payload, sizeof(payload), rx, rx9, sizeof(rx));

  if (n == 0) {
    Serial.println("AFT72 initiate: NO RESPONSE (AMBIGUOUS - NEVER AUTO RESEND)");
    v6LastAft = V6AftSnapshot();
    return false;
  }

  return v6ParseAft72Response(rx, n, true);
#endif
}

bool v6WaitForExactCreditDecrease(uint64_t expectedCredits,
                                  uint64_t &observedCredits,
                                  uint32_t timeoutMs = 6000UL) {
  const uint32_t start = millis();
  uint8_t attempts = 0;

  while ((uint32_t)(millis() - start) < timeoutMs) {
    attempts++;
    warmGeneralPoll(150UL, false);

    uint64_t nowCredits = 0;
    if (readCurrentCredits1A(nowCredits, false)) {
      observedCredits = nowCredits;
      Serial.print("POST LP1A attempt ");
      Serial.print(attempts);
      Serial.print(" observed=");
      Serial.print(u64ToString(nowCredits));
      Serial.print(" expected=");
      Serial.println(u64ToString(expectedCredits));

      if (nowCredits == expectedCredits) return true;

      // If it fell below expected, another payout/play/cashout changed state.
      if (nowCredits < expectedCredits) return false;
    }
    delay(100);
  }

  return false;
}

void v6PrintAftCashoutCheck() {
  Serial.println();
  Serial.println("=== AFT SPLIT CASHOUT PREFLIGHT ===");

  const bool p74 = probeAftStatus74(false);
  uint64_t credits = 0;
  const bool p1a = readCurrentCredits1A(credits, false);

  Serial.print("LP74             : ");
  Serial.println(p74 ? "PASS" : "FAIL");
  Serial.print("LP1A credits     : ");
  Serial.println(p1a ? u64ToString(credits) : String("FAIL"));

  if (p74) {
    Serial.print("Asset            : ");
    for (uint8_t i = 0; i < 4; i++) {
      if (sasCaps.aftAssetNumber[i] < 0x10) Serial.print('0');
      Serial.print(sasCaps.aftAssetNumber[i], HEX);
    }
    Serial.println();

    Serial.print("Available        : 0x");
    Serial.println(sasCaps.aftAvailableTransfers, HEX);
    Serial.print("AFT status       : 0x");
    Serial.println(sasCaps.aftStatusFlags, HEX);
    Serial.print("Cashable MU      : ");
    Serial.println(u64ToString(sasCaps.aftCashableMinorUnits));
    Serial.print("Transfer limit MU: ");
    Serial.println(u64ToString(sasCaps.aftTransferLimitCents));

    Serial.print("From machine OK  : ");
    Serial.println((sasCaps.aftAvailableTransfers & 0x02U) ? "YES" : "NO");
    Serial.print("Partial host OK  : ");
    Serial.println((sasCaps.aftStatusFlags & 0x02U) ? "YES" : "NO");
    Serial.print("In-house enabled : ");
    Serial.println((sasCaps.aftStatusFlags & 0x10U) ? "YES" : "NO");
    Serial.print("Any AFT enabled  : ");
    Serial.println((sasCaps.aftStatusFlags & 0x80U) ? "YES" : "NO");
  }

  if (!v6InterrogateAft72(0xFE, false, false)) {
    Serial.println("AFT FE            : no current transfer / FE unsupported");
  }

  Serial.println("==================================");
  Serial.println();
}

bool executeAftCashoutTest1Credit() {
#if !V6_ENABLE_AFT_VALUE_COMMANDS
  Serial.println("AFT cashout test disabled in config.");
  return false;
#else
  const uint32_t amountCredits = V6_AFT_CASHOUT_TEST_MAX_CREDITS;

  Serial.println();
  Serial.println("=== REAL AFT CASHOUT TEST: 1 CREDIT ===");
  Serial.println("This build can transfer ONLY 1 credit to the active RFID card.");

  if (!cardActive) {
    Serial.println("BLOCK: RFID card not active.");
    showMessage("AFT OUT BLOCK", "TAG RFID CARD", 1500);
    return false;
  }
  if (!sasOnline || !v6BootSelfTestPassed || !v6HealthReady()) {
    Serial.println("BLOCK: SAS/health not ready.");
    showMessage("AFT OUT BLOCK", "HEALTH NOT READY", 1500);
    return false;
  }
  if (v6RecoveryRequired) {
    Serial.println("BLOCK: unfinished RAM transaction exists.");
    showMessage("AFT OUT BLOCK", "RECOVERY REQ", 1500);
    return false;
  }
  if (!machineInfo.valid || machineInfo.accountingDenomMinorUnits == 0) {
    Serial.println("BLOCK: accounting denomination unknown.");
    return false;
  }

  uint32_t freshBalance = 0;
  if (!readBalanceFromCard(freshBalance) || freshBalance != cardBalance) {
    Serial.println("BLOCK: RFID balance changed/read failed.");
    return false;
  }
  if (cardBalance > 0xFFFFFFFFUL - amountCredits) {
    Serial.println("BLOCK: RFID balance overflow.");
    return false;
  }

  if (!probeAftStatus74(false)) {
    Serial.println("BLOCK: LP74 AFT status unavailable.");
    showMessage("AFT OUT BLOCK", "LP74 FAIL", 1500);
    return false;
  }

  if (!v6AftAssetIsNonZero()) {
    Serial.println("BLOCK: LP74 asset number is zero.");
    showMessage("AFT OUT BLOCK", "ASSET=0", 1500);
    return false;
  }
  if ((sasCaps.aftAvailableTransfers & 0x02U) == 0) {
    Serial.println("BLOCK: LP74 says transfer FROM gaming machine not currently available.");
    showMessage("AFT OUT BLOCK", "FROM GM=N", 1500);
    return false;
  }
  if ((sasCaps.aftStatusFlags & 0x10U) == 0 ||
      (sasCaps.aftStatusFlags & 0x80U) == 0) {
    Serial.println("BLOCK: in-house/AFT not enabled in LP74 status.");
    showMessage("AFT OUT BLOCK", "AFT DISABLED", 1500);
    return false;
  }

  if (!v6SyncAftCycleBeforeValueCommand("AFT-1", 3000UL, true)) {
    Serial.println("BLOCK: previous/current AFT cycle not ready.");
    showMessage("AFT OUT BLOCK", "CYCLE NOT READY", 1500);
    return false;
  }

  uint64_t beforeCredits = 0;
  if (!readCurrentCredits1A(beforeCredits, false) ||
      beforeCredits < amountCredits) {
    Serial.println("BLOCK: not enough machine credits.");
    showMessage("AFT OUT BLOCK", "NO CREDIT", 1500);
    return false;
  }

  // Splitting requires "less than full available amount" support.
  if (beforeCredits > amountCredits &&
      (sasCaps.aftStatusFlags & 0x02U) == 0) {
    Serial.println("BLOCK: machine does not allow transfer-to-host less than full available amount.");
    showMessage("AFT OUT BLOCK", "NO PARTIAL HOST", 1800);
    return false;
  }

  const uint64_t amountMinor =
    (uint64_t)amountCredits * (uint64_t)machineInfo.accountingDenomMinorUnits;

  if (sasCaps.aftCashableMinorUnits < amountMinor) {
    Serial.print("BLOCK: LP74 cashable MU=");
    Serial.print(u64ToString(sasCaps.aftCashableMinorUnits));
    Serial.print(" requested=");
    Serial.println(u64ToString(amountMinor));
    showMessage("AFT OUT BLOCK", "CASHABLE LOW", 1600);
    return false;
  }

  if (sasCaps.aftTransferLimitCents != 0 &&
      amountMinor > sasCaps.aftTransferLimitCents) {
    Serial.println("BLOCK: amount exceeds LP74 transfer limit.");
    return false;
  }

  const uint64_t expectedCredits = beforeCredits - amountCredits;
  const uint32_t newBalance = cardBalance + amountCredits;

  v6BeforeMeters = V6MeterSnapshot();
  if (!v6ReadSelectedMeters2F(v6BeforeMeters, true, false)) {
    // Exact LP1A remains authoritative; store at least the pre-credit.
    v6BeforeMeters = V6MeterSnapshot();
    v6BeforeMeters.valid = true;
    v6BeforeMeters.currentCredits = beforeCredits;
  } else {
    v6BeforeMeters.currentCredits = beforeCredits;
  }

  char txid[7];
  v6MakeAftTransactionId(txid);

  Serial.print("PRE machine credits: "); Serial.println(u64ToString(beforeCredits));
  Serial.print("RFID before        : "); Serial.println(cardBalance);
  Serial.print("Move to RFID       : "); Serial.println(amountCredits);
  Serial.print("Amount MU          : "); Serial.println(u64ToString(amountMinor));
  Serial.print("Expected machine   : "); Serial.println(u64ToString(expectedCredits));
  Serial.print("Expected RFID      : "); Serial.println(newBalance);
  Serial.print("Hopper remainder   : "); Serial.println(u64ToString(expectedCredits));

  if (!v6PrepareRfidCashoutJournal(newBalance, amountCredits, amountMinor, txid)) {
    Serial.println("BLOCK: could not initialize PREPARED RFID_CASHOUT RAM transaction.");
    showMessage("AFT OUT BLOCK", "RAM TX FAIL", 1600);
    return false;
  }

  // From this point onward, NEVER blindly initiate LP72 a second time.
  const bool rxParsed = v6InitiateAftCashoutToHost(amountMinor, txid);

  if (rxParsed && v6LastAft.valid) {
    v6UpdateJournal(
      v6LastAft.transferStatus == 0x40 ? V6JournalState::PENDING
                                      : V6JournalState::PREPARED,
      v6LastAft.transferStatus
    );
  } else {
    v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED, 0xFF);
  }

  if (rxParsed && v6LastAft.valid && v6LastAft.transferStatus == 0x40) {
    v6WaitAftFinalStatus(6000UL);
  }

  if (v6LastAft.valid) {
    Serial.print("Final AFT status   : 0x");
    if (v6LastAft.transferStatus < 0x10) Serial.print('0');
    Serial.print(v6LastAft.transferStatus, HEX);
    Serial.print(" ");
    Serial.println(v6AftStatusShort(v6LastAft.transferStatus));
  } else {
    Serial.println("Final AFT status   : UNKNOWN");
  }

  if (v6AbortOnClearAftReject(beforeCredits, "AFT-1")) {
    return false;
  }

  uint64_t afterCredits = beforeCredits;
  const bool exactDecrease =
    v6WaitForExactCreditDecrease(expectedCredits, afterCredits, 6000UL);

  v6AfterMeters = V6MeterSnapshot();
  const bool metersRead = v6ReadSelectedMeters2F(v6AfterMeters, true, false);
  if (!metersRead) {
    v6AfterMeters = V6MeterSnapshot();
    v6AfterMeters.valid = true;
    v6AfterMeters.currentCredits = afterCredits;
  } else {
    v6AfterMeters.currentCredits = afterCredits;
  }

  const bool hostMeterAdvanced =
    metersRead && v6BeforeMeters.valid &&
    v6MeterAdvanced(v6BeforeMeters.transferToHostCredits,
                    v6AfterMeters.transferToHostCredits);

  Serial.print("POST machine credits: "); Serial.println(u64ToString(afterCredits));
  Serial.print("Exact -1 proof       : "); Serial.println(exactDecrease ? "PASS" : "FAIL/AMBIGUOUS");
  Serial.print("LP2F XOUT advanced   : "); Serial.println(hostMeterAdvanced ? "YES" : "NO/UNKNOWN");

  if (!exactDecrease) {
    v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED,
                    v6LastAft.valid ? v6LastAft.transferStatus : 0xFF);
    Serial.println("*** AFT CASHOUT NOT COMMITTED ***");
    Serial.println("RFID unchanged. DO NOT RESEND. RAM transaction state retained.");
    showMessage("AFT VERIFY FAIL", "RFID UNCHANGED", 2000);
    return false;
  }

  // Exact machine-credit decrease is the independent proof that value left the machine.
  if (!v6UpdateJournal(V6JournalState::SAS_VERIFIED,
                       v6LastAft.valid ? v6LastAft.transferStatus : 0x00)) {
    Serial.println("AFT credited out but SAS_VERIFIED RAM transaction update failed.");
    showMessage("AFT OUT CREDITED", "JOURNAL ERROR", 2000);
    return false;
  }

  if (!cardActive || !v6SameActiveCardUid(v6LatestJournal)) {
    Serial.println("AFT succeeded but the same RFID card is not active.");
    Serial.println("Retag the SAME card. DO NOT RESEND LP72.");
    showMessage("AFT OUT CREDITED", "RETAG SAME CARD", 2000);
    return false;
  }

  if (!v6AtomicCommitCardBalance(newBalance)) {
    Serial.println("AFT succeeded but RFID write/verify failed.");
    Serial.println("DO NOT RESEND LP72. RAM transaction state retained.");
    showMessage("CARD WRITE FAIL", "DO NOT RESEND", 2000);
    return false;
  }

  v6UpdateJournal(V6JournalState::DONE,
                  v6LastAft.valid ? v6LastAft.transferStatus : 0x00);

  Serial.println();
  Serial.println("*** AFT RFID CASHOUT SUCCESS ***");
  Serial.print("RFID new balance    : "); Serial.println(cardBalance);
  Serial.print("Machine remaining   : "); Serial.println(u64ToString(afterCredits));
  Serial.println("Remainder was NOT auto-cashed-out by V6.");
  Serial.println("Press the S6000 physical CASH OUT button only if you want the remainder from the hopper.");
  Serial.println();

  showMessage(
    String("CARD +") + amountCredits + " OK",
    String("HOPPER LEFT:") + u64ToString(afterCredits),
    2500
  );
  return true;
#endif
}



bool executeAftCashoutAllCredits() {
#if !V6_ENABLE_AFT_VALUE_COMMANDS || !V6_ENABLE_AFT_FULL_CASHOUT_TEST
  Serial.println("AFT full cashout test disabled in config.");
  return false;
#else
  Serial.println();
  Serial.println("=== REAL AFT FULL CASHOUT -> RFID ===");
  Serial.println("Moves ALL currently cashable machine credits to the active RFID card.");
  Serial.println("Success requires exact LP1A transition to ZERO before RFID is credited.");

  if (!cardActive) {
    Serial.println("BLOCK: RFID card not active.");
    showMessage("AFT ALL BLOCK", "TAG RFID CARD", 1500);
    return false;
  }
  if (!sasOnline || !v6BootSelfTestPassed || !v6HealthReady()) {
    Serial.println("BLOCK: SAS/health not ready.");
    showMessage("AFT ALL BLOCK", "HEALTH NOT READY", 1500);
    return false;
  }
  if (v6RecoveryRequired) {
    Serial.println("BLOCK: unfinished RAM transaction exists.");
    showMessage("AFT ALL BLOCK", "RECOVERY REQ", 1500);
    return false;
  }
  if (!machineInfo.valid || machineInfo.accountingDenomMinorUnits == 0) {
    Serial.println("BLOCK: accounting denomination unknown.");
    showMessage("AFT ALL BLOCK", "DENOM UNKNOWN", 1500);
    return false;
  }

  uint32_t freshBalance = 0;
  if (!readBalanceFromCard(freshBalance) || freshBalance != cardBalance) {
    Serial.println("BLOCK: RFID balance changed/read failed.");
    showMessage("AFT ALL BLOCK", "CARD READ FAIL", 1500);
    return false;
  }

  if (!probeAftStatus74(false)) {
    Serial.println("BLOCK: LP74 AFT status unavailable.");
    showMessage("AFT ALL BLOCK", "LP74 FAIL", 1500);
    return false;
  }
  if (!v6AftAssetIsNonZero()) {
    Serial.println("BLOCK: LP74 asset number is zero.");
    showMessage("AFT ALL BLOCK", "ASSET=0", 1500);
    return false;
  }
  if ((sasCaps.aftAvailableTransfers & 0x02U) == 0) {
    Serial.println("BLOCK: transfer FROM gaming machine not currently available.");
    showMessage("AFT ALL BLOCK", "FROM GM=N", 1500);
    return false;
  }
  if ((sasCaps.aftStatusFlags & 0x10U) == 0 ||
      (sasCaps.aftStatusFlags & 0x80U) == 0) {
    Serial.println("BLOCK: in-house/AFT not enabled in LP74 status.");
    showMessage("AFT ALL BLOCK", "AFT DISABLED", 1500);
    return false;
  }

  if (!v6SyncAftCycleBeforeValueCommand("AFT-ALL", 3000UL, true)) {
    Serial.println("BLOCK: previous/current AFT cycle not ready for a new transfer.");
    showMessage("AFT ALL BLOCK", "CYCLE NOT READY", 1600);
    return false;
  }

  uint64_t beforeCredits = 0;
  if (!readCurrentCredits1A(beforeCredits, false) || beforeCredits == 0) {
    Serial.println("BLOCK: machine has no readable credits.");
    showMessage("AFT ALL BLOCK", "CREDIT=0", 1500);
    return false;
  }
  if (beforeCredits > 0xFFFFFFFFULL) {
    Serial.println("BLOCK: machine credit count exceeds RFID/journal uint32 range.");
    showMessage("AFT ALL BLOCK", "CREDIT RANGE", 1500);
    return false;
  }

  const uint32_t amountCredits = (uint32_t)beforeCredits;
  if (cardBalance > (0xFFFFFFFFUL - amountCredits)) {
    Serial.println("BLOCK: RFID balance would overflow.");
    showMessage("AFT ALL BLOCK", "CARD OVERFLOW", 1500);
    return false;
  }

  const uint64_t amountMinor =
    beforeCredits * (uint64_t)machineInfo.accountingDenomMinorUnits;

  if (sasCaps.aftCashableMinorUnits != amountMinor) {
    Serial.println("BLOCK: LP1A credits are not exactly equal to LP74 cashable amount.");
    Serial.print("LP1A x denom MU    : "); Serial.println(u64ToString(amountMinor));
    Serial.print("LP74 cashable MU   : "); Serial.println(u64ToString(sasCaps.aftCashableMinorUnits));
    showMessage("AFT ALL BLOCK", "CASHABLE MISMATCH", 2000);
    return false;
  }

  if (sasCaps.aftTransferLimitCents != 0 &&
      amountMinor > sasCaps.aftTransferLimitCents) {
    Serial.println("BLOCK: full amount exceeds LP74 AFT transfer limit.");
    Serial.print("Requested MU       : "); Serial.println(u64ToString(amountMinor));
    Serial.print("Transfer limit MU  : "); Serial.println(u64ToString(sasCaps.aftTransferLimitCents));
    showMessage("AFT ALL BLOCK", "OVER AFT LIMIT", 1800);
    return false;
  }

  const uint64_t expectedCredits = 0;
  const uint32_t newBalance = cardBalance + amountCredits;

  v6BeforeMeters = V6MeterSnapshot();
  if (!v6ReadSelectedMeters2F(v6BeforeMeters, true, false)) {
    v6BeforeMeters = V6MeterSnapshot();
    v6BeforeMeters.valid = true;
    v6BeforeMeters.currentCredits = beforeCredits;
  } else {
    v6BeforeMeters.currentCredits = beforeCredits;
  }

  char txid[7];
  v6MakeAftTransactionId(txid);

  Serial.println("--- FULL CASHOUT PLAN ---");
  Serial.print("PRE machine credits: "); Serial.println(u64ToString(beforeCredits));
  Serial.print("Accounting denom   : "); Serial.println(machineInfo.accountingDenomMinorUnits);
  Serial.print("Full cashable MU   : "); Serial.println(u64ToString(amountMinor));
  Serial.print("RFID before        : "); Serial.println(cardBalance);
  Serial.print("RFID after target  : "); Serial.println(newBalance);
  Serial.print("Expected machine   : 0");
  Serial.println();
  Serial.print("Transaction ID     : "); Serial.println(txid);

  if (!v6PrepareRfidCashoutJournal(
        newBalance, amountCredits, amountMinor, txid)) {
    Serial.println("BLOCK: could not initialize PREPARED RFID_CASHOUT RAM transaction.");
    showMessage("AFT ALL BLOCK", "RAM TX FAIL", 1600);
    return false;
  }

  showMessage("AFT FULL OUT", String("CR:") + amountCredits, 700);

  // Exactly one LP72 initiate. Never blind-retry after this point.
  const bool rxParsed = v6InitiateAftCashoutToHost(amountMinor, txid);

  if (rxParsed && v6LastAft.valid) {
    v6UpdateJournal(
      v6LastAft.transferStatus == 0x40
        ? V6JournalState::PENDING
        : V6JournalState::PREPARED,
      v6LastAft.transferStatus
    );
  } else {
    v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED, 0xFF);
  }

  if (rxParsed && v6LastAft.valid &&
      v6LastAft.transferStatus == 0x40) {
    v6WaitAftFinalStatus(8000UL);
  }

  if (v6LastAft.valid) {
    Serial.print("Final AFT status   : 0x");
    if (v6LastAft.transferStatus < 0x10) Serial.print('0');
    Serial.print(v6LastAft.transferStatus, HEX);
    Serial.print(" ");
    Serial.println(v6AftStatusShort(v6LastAft.transferStatus));
  } else {
    Serial.println("Final AFT status   : UNKNOWN");
  }

  if (v6AbortOnClearAftReject(beforeCredits, "AFT-ALL")) {
    return false;
  }

  uint64_t afterCredits = beforeCredits;
  const bool exactZero =
    v6WaitForExactCreditDecrease(expectedCredits, afterCredits, 8000UL);

  v6AfterMeters = V6MeterSnapshot();
  const bool metersRead =
    v6ReadSelectedMeters2F(v6AfterMeters, true, false);

  if (!metersRead) {
    v6AfterMeters = V6MeterSnapshot();
    v6AfterMeters.valid = true;
    v6AfterMeters.currentCredits = afterCredits;
  } else {
    v6AfterMeters.currentCredits = afterCredits;
  }

  const bool hostMeterAdvanced =
    metersRead && v6BeforeMeters.valid &&
    v6MeterAdvanced(
      v6BeforeMeters.transferToHostCredits,
      v6AfterMeters.transferToHostCredits
    );

  Serial.println("--- FULL CASHOUT VERIFY ---");
  Serial.print("POST machine credits: "); Serial.println(u64ToString(afterCredits));
  Serial.print("Exact ZERO proof     : "); Serial.println(exactZero ? "PASS" : "FAIL/AMBIGUOUS");
  Serial.print("LP2F XOUT advanced   : "); Serial.println(hostMeterAdvanced ? "YES" : "NO/UNKNOWN");

  if (!exactZero) {
    v6UpdateJournal(
      V6JournalState::RECOVERY_REQUIRED,
      v6LastAft.valid ? v6LastAft.transferStatus : 0xFF
    );
    Serial.println("*** AFT FULL CASHOUT NOT COMMITTED TO RFID ***");
    Serial.println("RFID unchanged. DO NOT SEND aftoutall AGAIN.");
    Serial.println("RAM transaction state retained.");
    showMessage("AFT VERIFY FAIL", "RFID UNCHANGED", 2200);
    return false;
  }

  if (!v6UpdateJournal(
        V6JournalState::SAS_VERIFIED,
        v6LastAft.valid ? v6LastAft.transferStatus : 0x00)) {
    Serial.println("Machine reached zero but RAM transaction update failed.");
    Serial.println("DO NOT RESEND AFT. RFID remains unchanged pending recovery.");
    showMessage("MACHINE ZERO", "JOURNAL ERROR", 2200);
    return false;
  }

  if (!cardActive || !v6SameActiveCardUid(v6LatestJournal)) {
    Serial.println("Machine cashout succeeded but SAME RFID card is not active.");
    Serial.println("Retag the SAME card. DO NOT RESEND LP72.");
    showMessage("MACHINE ZERO", "RETAG SAME CARD", 2200);
    return false;
  }

  if (!v6AtomicCommitCardBalance(newBalance)) {
    Serial.println("Machine cashout succeeded but RFID write/verify failed.");
    Serial.println("DO NOT RESEND LP72. RAM transaction state retained.");
    showMessage("CARD WRITE FAIL", "DO NOT RESEND", 2200);
    return false;
  }

  v6UpdateJournal(
    V6JournalState::DONE,
    v6LastAft.valid ? v6LastAft.transferStatus : 0x00
  );

  Serial.println();
  Serial.println("*** AFT FULL RFID CASHOUT SUCCESS ***");
  Serial.print("Machine credits     : "); Serial.println(u64ToString(afterCredits));
  Serial.print("RFID added credits  : "); Serial.println(amountCredits);
  Serial.print("RFID new balance    : "); Serial.println(cardBalance);
  Serial.println("RAM transaction : DONE");
  Serial.println();

  showMessage("AFT FULL SUCCESS", String("CARD:") + cardBalance, 2500);
  return true;
#endif
}



bool v6PrepareSplitCashoutJournal(uint32_t totalMachineCredits,
                                  uint32_t rfidCredits,
                                  uint64_t totalMinorUnits,
                                  const char *transactionId) {
  if (!cardActive || activeCardUidLen == 0) return false;
  if (v6RecoveryRequired) return false;
  if (rfidCredits == 0 || rfidCredits > totalMachineCredits) return false;
  if (cardBalance > 0xFFFFFFFFUL - rfidCredits) return false;

  V6JournalRecord r = {};
  r.type = (uint8_t)V6JournalType::RFID_SPLIT_CASHOUT;
  r.state = (uint8_t)V6JournalState::PREPARED;
  r.sasStatus = 0xFF;
  r.cardUidLen = activeCardUidLen;
  for (uint8_t i = 0; i < activeCardUidLen; i++) r.cardUid[i] = activeCardUid[i];

  // Store total machine credits moved by the full AFT. Requested RFID amount
  // is balanceAfter-balanceBefore; hopper remainder = total - RFID amount.
  r.amountCredits = totalMachineCredits;
  r.amountMinorUnits = totalMinorUnits;
  r.balanceBefore = cardBalance;
  r.balanceAfter = cardBalance + rfidCredits;
  r.beforeMeters = v6StoreMeters(v6BeforeMeters);

  if (transactionId) {
    for (uint8_t i = 0; i < sizeof(r.transactionId) - 1 && transactionId[i]; i++)
      r.transactionId[i] = transactionId[i];
  }

  return v6WriteJournal(r);
}

// V6.5.9 in-session automatic split reconciliation.
// No blind resend is ever used. Recovery is based on the original machine total,
// a stable LP1A settlement snapshot, the final AFT interrogation state, and the
// SAME RFID card. Hidden stage bytes live after transactionId's NUL terminator,
// so the 6-byte SAS transaction ID sent to the S6000 is unchanged and old
// V6.5.x EEPROM records remain binary-compatible.
static const char V6_SPLIT_STAGE_TAG = 'S';
static const char V6_SPLIT_STAGE_AFT = 'A';
static const char V6_SPLIT_STAGE_RESTORE = 'R';
static const char V6_SPLIT_STAGE_LEGACY = 'L';

char v6SplitRecoveryStage(const V6JournalRecord &r) {
  if (r.transactionId[7] != V6_SPLIT_STAGE_TAG) return 0;
  const char st = r.transactionId[8];
  if (st == V6_SPLIT_STAGE_AFT || st == V6_SPLIT_STAGE_RESTORE ||
      st == V6_SPLIT_STAGE_LEGACY) return st;
  return 0;
}

void v6SetSplitRecoveryStage(V6JournalRecord &r, char stage) {
  r.transactionId[7] = V6_SPLIT_STAGE_TAG;
  r.transactionId[8] = stage;
  r.transactionId[9] = '\0';
}

bool v6PersistSplitRecovery(char stage, uint8_t sasStatus,
                            uint32_t settledMachineCredits) {
  if (!v6LatestJournalValid ||
      v6LatestJournal.type != (uint8_t)V6JournalType::RFID_SPLIT_CASHOUT)
    return false;
  if (settledMachineCredits > v6LatestJournal.amountCredits) return false;

  V6JournalRecord r = v6LatestJournal;
  r.state = (uint8_t)V6JournalState::RECOVERY_REQUIRED;
  r.sasStatus = sasStatus;
  r.afterMeters.currentCredits = settledMachineCredits;
  v6SetSplitRecoveryStage(r, stage);
  return v6WriteJournal(r);
}

bool v6SplitReconcileTarget(uint32_t totalCredits,
                            uint32_t settledMachineCredits,
                            uint32_t &finalRfidCredits) {
  if (totalCredits == 0 || settledMachineCredits > totalCredits) return false;
  finalRfidCredits = totalCredits - settledMachineCredits;
  return true;
}

// Read LP1A repeatedly and only accept a value that is unchanged for the whole
// window. This is used to adopt/move a recovery settlement snapshot; it never
// transmits a value-changing SAS command.
bool v6ReadStableCurrentCredits(uint32_t totalCredits,
                                uint32_t &stableCredits,
                                uint32_t windowMs = 1800UL) {
  const uint32_t start = millis();
  bool have = false;
  uint64_t candidate = 0;
  uint8_t samples = 0;

  while ((uint32_t)(millis() - start) < windowMs) {
    uint64_t nowCredits = 0;
    if (!readCurrentCredits1A(nowCredits, false) || nowCredits > totalCredits) {
      return false;
    }
    if (!have) {
      candidate = nowCredits;
      have = true;
    } else if (nowCredits != candidate) {
      // Value is still moving; do not guess. A later automatic recovery pass
      // will sample again.
      return false;
    }
    samples++;
    warmGeneralPoll(120UL, false);
    delay(120);
  }

  if (!have || samples < 4 || candidate > 0xFFFFFFFFULL) return false;
  stableCredits = (uint32_t)candidate;
  return true;
}

bool v6AutoReconcileSplitToCard(uint32_t totalCredits,
                                uint32_t plannedRfidCredits,
                                uint32_t settledMachineCredits,
                                const char *context) {
  uint32_t finalRfidCredits = 0;
  if (!v6SplitReconcileTarget(totalCredits, settledMachineCredits, finalRfidCredits)) {
    Serial.println("AUTO RECONCILE: invalid total/machine snapshot; automatic retry pending.");
    return false;
  }

  if (!cardActive || !v6SameActiveCardUid(v6LatestJournal)) {
    Serial.println("AUTO RECONCILE: waiting for SAME RFID card; no AFT/0x8A resend.");
    showMessage("AUTO RECOVERY", "RETAG SAME CARD", 900);
    return false;
  }

  // Re-read live machine credit before the card commit. The card is committed
  // only while the machine still equals the settlement snapshot. If it moved,
  // recovery remains automatic but waits rather than mixing later game play.
  uint64_t liveMachine = 0;
  if (!readCurrentCredits1A(liveMachine, false) ||
      liveMachine != (uint64_t)settledMachineCredits) {
    Serial.print("AUTO RECONCILE: machine moved after settlement snapshot. expected=");
    Serial.print(settledMachineCredits); Serial.print(" live=");
    Serial.println(u64ToString(liveMachine));
    return false;
  }

  uint32_t liveBalance = 0;
  if (!readBalanceFromCard(liveBalance)) {
    Serial.println("AUTO RECONCILE: RFID read failed; retry will be automatic.");
    return false;
  }

  const uint32_t base = v6LatestJournal.balanceBefore;
  if (finalRfidCredits > 0xFFFFFFFFUL - base) {
    Serial.println("AUTO RECONCILE: RFID target overflow; automatic recovery stopped safely.");
    return false;
  }
  const uint32_t target = base + finalRfidCredits;

  // Accept a card value anywhere inside this transaction's provable interval.
  // This also heals a power-loss case where the card write partially completed
  // before the terminal journal write. Values outside the interval are never
  // overwritten automatically.
  if (liveBalance < base || liveBalance > target) {
    Serial.print("AUTO RECONCILE: card outside safe transaction interval. live=");
    Serial.print(liveBalance); Serial.print(" before="); Serial.print(base);
    Serial.print(" target="); Serial.println(target);
    return false;
  }

  Serial.println();
  Serial.println("=== SPLIT AUTO RECONCILE ===");
  Serial.print("Context             : "); Serial.println(context ? context : "?");
  Serial.print("Original total      : "); Serial.println(totalCredits);
  Serial.print("Planned RFID        : "); Serial.println(plannedRfidCredits);
  Serial.print("Settled machine     : "); Serial.println(settledMachineCredits);
  Serial.print("Actual RFID delta   : "); Serial.println(finalRfidCredits);
  Serial.print("RFID before         : "); Serial.println(base);
  Serial.print("RFID live           : "); Serial.println(liveBalance);
  Serial.print("RFID final balance  : "); Serial.println(target);
  if (plannedRfidCredits != finalRfidCredits) {
    Serial.println("AUTO ADJUST: actual moved value wins; exact total is preserved.");
  }

  V6JournalRecord r = v6LatestJournal;
  r.balanceAfter = target;
  r.afterMeters.currentCredits = settledMachineCredits;
  r.state = (uint8_t)V6JournalState::METERS_VERIFIED;
  if (!v6WriteJournal(r)) {
    Serial.println("AUTO RECONCILE: RAM transaction target update failed; RFID unchanged.");
    return false;
  }

  cardBalance = liveBalance;
  if (liveBalance != target) {
    if (!v6AtomicCommitCardBalance(target)) {
      Serial.println("AUTO RECONCILE: RFID commit failed; automatic retry will continue.");
      return false;
    }
  } else {
    cardBalance = liveBalance;
  }

  v6UpdateJournal(V6JournalState::DONE, v6LatestJournal.sasStatus);
  inputValue = 0;
  cancelMoneyConfirm(false);
  Serial.println("*** SPLIT AUTO RECOVERY DONE ***");
  Serial.print("Machine settled     : "); Serial.println(settledMachineCredits);
  Serial.print("RFID credited total : "); Serial.println(finalRfidCredits);
  Serial.println("No AFT/0x8A resend was used by reconciliation.");
  showMessage("AUTO RECOVER OK", String("CARD+") + finalRfidCredits, 1400);
  return true;
}

// Automatically resolves state=9 split RAM transactions. V6.5.9 transactions carry an
// A/R stage marker and a settlement snapshot. Old V6.5.0..6.5.4 journals do not;
// for those, a stable live LP1A value is adopted once (L marker), then the same
// conservation rule is applied. If AFT is still pending, it waits automatically.
bool v6AutoResolveSplitRecovery(uint32_t totalCredits,
                                uint32_t plannedRfidCredits,
                                uint32_t currentCredits) {
  if (!v6LatestJournalValid ||
      v6LatestJournal.type != (uint8_t)V6JournalType::RFID_SPLIT_CASHOUT ||
      v6LatestJournal.state != (uint8_t)V6JournalState::RECOVERY_REQUIRED)
    return false;

  char stage = v6SplitRecoveryStage(v6LatestJournal);
  uint32_t settled = v6LatestJournal.afterMeters.currentCredits;

  if (stage == 0) {
    // Legacy journal: first make sure there is no still-pending AFT. FF is
    // interrogation/acknowledgement only; it does not initiate money movement.
    const bool ffOK = v6InterrogateAft72(0xFF, true, false);
    if (ffOK && v6LastAft.valid &&
        (v6LastAft.transferStatus == 0x40 || v6LastAft.receiptStatus == 0x40)) {
      Serial.println("SPLIT AUTO RECOVERY: AFT still pending; waiting automatically.");
      return false;
    }

    uint32_t stable = 0;
    if (!v6ReadStableCurrentCredits(totalCredits, stable)) {
      Serial.println("SPLIT AUTO RECOVERY: CREDIT not stable yet; retry automatic.");
      return false;
    }
    settled = stable;
    if (!v6PersistSplitRecovery(V6_SPLIT_STAGE_LEGACY,
                                v6LatestJournal.sasStatus, settled)) {
      Serial.println("SPLIT AUTO RECOVERY: cannot store RAM settlement snapshot.");
      return false;
    }
    stage = V6_SPLIT_STAGE_LEGACY;
    Serial.print("SPLIT AUTO RECOVERY: legacy stable snapshot adopted=");
    Serial.println(settled);
  } else {
    // AFT may finish after the first zero-verification timeout. If an A-stage
    // snapshot subsequently falls, adopt the new stable lower value. A restore
    // may complete late; if an R-stage snapshot rises, adopt the stable higher
    // value up to the planned hopper remainder.
    const uint32_t hopperTarget = totalCredits - plannedRfidCredits;
    if (currentCredits != settled) {
      bool allowedDirection = false;
      if (stage == V6_SPLIT_STAGE_AFT && currentCredits < settled)
        allowedDirection = true;
      if (stage == V6_SPLIT_STAGE_RESTORE &&
          currentCredits > settled && currentCredits <= hopperTarget)
        allowedDirection = true;

      if (allowedDirection) {
        uint32_t stable = 0;
        if (!v6ReadStableCurrentCredits(totalCredits, stable)) return false;
        if ((stage == V6_SPLIT_STAGE_AFT && stable <= settled) ||
            (stage == V6_SPLIT_STAGE_RESTORE && stable >= settled &&
             stable <= hopperTarget)) {
          settled = stable;
          if (!v6PersistSplitRecovery(stage, v6LatestJournal.sasStatus, settled))
            return false;
          Serial.print("SPLIT AUTO RECOVERY: settlement advanced to stable=");
          Serial.println(settled);
        }
      } else {
        Serial.println("SPLIT AUTO RECOVERY: machine CREDIT changed outside safe direction; waiting.");
        return false;
      }
    }
  }

  return v6AutoReconcileSplitToCard(totalCredits, plannedRfidCredits,
                                    settled, "AUTO-STATE9");
}

bool executeSplitCashoutToRfid(uint32_t requestedRfidCredits) {
  Serial.println();
  Serial.println("=== FINAL SPLIT CASHOUT: MACHINE -> RFID + HOPPER ===");
  Serial.println("FULL AFT out -> exact zero proof -> restore hopper remainder with 0x8A -> RFID commit.");
  Serial.println("Physical S6000 CASH OUT pays the remaining machine credits from the hopper.");

  if (!cardActive) {
    Serial.println("BLOCK: RFID card not active.");
    showMessage("SPLIT BLOCK", "TAG RFID CARD", 1500);
    return false;
  }
  if (requestedRfidCredits == 0) {
    Serial.println("BLOCK: enter RFID cashout amount first.");
    showMessage("ENTER AMOUNT", "# = RFID OUT", 1300);
    return false;
  }
  if (!sasOnline || !v6BootSelfTestPassed || !v6HealthReady()) {
    Serial.println("BLOCK: SAS/health not ready.");
    showMessage("SPLIT BLOCK", "HEALTH NOT READY", 1500);
    return false;
  }
  if (v6RecoveryRequired) {
    Serial.println("BLOCK: unfinished RAM transaction exists.");
    showMessage("SPLIT BLOCK", "RECOVERY REQ", 1500);
    return false;
  }
  if (!machineInfo.valid || machineInfo.accountingDenomMinorUnits == 0) {
    Serial.println("BLOCK: accounting denomination unknown.");
    showMessage("SPLIT BLOCK", "DENOM UNKNOWN", 1500);
    return false;
  }

  uint32_t freshBalance = 0;
  if (!readBalanceFromCard(freshBalance) || freshBalance != cardBalance) {
    Serial.println("BLOCK: RFID read/balance changed.");
    showMessage("SPLIT BLOCK", "CARD READ FAIL", 1500);
    return false;
  }

  if (!probeAftStatus74(false)) {
    Serial.println("BLOCK: LP74 AFT status unavailable.");
    showMessage("SPLIT BLOCK", "LP74 FAIL", 1500);
    return false;
  }
  if (!v6AftAssetIsNonZero()) {
    Serial.println("BLOCK: AFT asset number is zero.");
    showMessage("SPLIT BLOCK", "ASSET=0", 1500);
    return false;
  }
  if ((sasCaps.aftAvailableTransfers & 0x02U) == 0) {
    Serial.println("BLOCK: transfer FROM gaming machine unavailable.");
    showMessage("SPLIT BLOCK", "FROM GM=N", 1500);
    return false;
  }
  if ((sasCaps.aftStatusFlags & 0x10U) == 0 ||
      (sasCaps.aftStatusFlags & 0x80U) == 0) {
    Serial.println("BLOCK: in-house/AFT not enabled.");
    showMessage("SPLIT BLOCK", "AFT DISABLED", 1500);
    return false;
  }

  // Close/acknowledge any previous completed transfer with FF, and refuse
  // to start while the current transfer/receipt is still pending.
  if (!v6SyncAftCycleBeforeValueCommand("SPLIT", 3000UL, true)) {
    Serial.println("BLOCK: previous/current AFT cycle not ready.");
    showMessage("SPLIT BLOCK", "CYCLE NOT READY", 1500);
    return false;
  }

  uint64_t beforeCredits64 = 0;
  if (!readCurrentCredits1A(beforeCredits64, false) ||
      beforeCredits64 == 0 ||
      beforeCredits64 > 0xFFFFFFFFULL) {
    Serial.println("BLOCK: machine CREDIT unavailable/out of range.");
    showMessage("SPLIT BLOCK", "CREDIT ERROR", 1500);
    return false;
  }

  const uint32_t totalCredits = (uint32_t)beforeCredits64;
  if (requestedRfidCredits > totalCredits) {
    Serial.print("BLOCK: requested RFID=");
    Serial.print(requestedRfidCredits);
    Serial.print(" > machine CREDIT=");
    Serial.println(totalCredits);
    showMessage("SPLIT BLOCK", "AMT > CREDIT", 1500);
    return false;
  }
  if (cardBalance > 0xFFFFFFFFUL - requestedRfidCredits) {
    Serial.println("BLOCK: RFID balance overflow.");
    showMessage("SPLIT BLOCK", "CARD OVERFLOW", 1500);
    return false;
  }

  const uint32_t hopperCredits = totalCredits - requestedRfidCredits;
  const uint64_t totalMinor =
    (uint64_t)totalCredits *
    (uint64_t)machineInfo.accountingDenomMinorUnits;

  // This real S6000 reports Partial host OK=NO. The final split algorithm
  // therefore moves the FULL cashable balance first. Refuse mixed balances.
  if (sasCaps.aftCashableMinorUnits != totalMinor) {
    Serial.println("BLOCK: displayed CREDIT != LP74 cashable amount.");
    Serial.print("LP1A x denom MU   : "); Serial.println(u64ToString(totalMinor));
    Serial.print("LP74 cashable MU  : "); Serial.println(u64ToString(sasCaps.aftCashableMinorUnits));
    showMessage("SPLIT BLOCK", "CASHABLE MISMATCH", 1800);
    return false;
  }
  if (sasCaps.aftTransferLimitCents != 0 &&
      totalMinor > sasCaps.aftTransferLimitCents) {
    Serial.println("BLOCK: full AFT exceeds transfer limit.");
    showMessage("SPLIT BLOCK", "OVER AFT LIMIT", 1800);
    return false;
  }

  v6BeforeMeters = V6MeterSnapshot();
  if (!v6ReadSelectedMeters2F(v6BeforeMeters, true, false)) {
    v6BeforeMeters = V6MeterSnapshot();
    v6BeforeMeters.valid = true;
    v6BeforeMeters.currentCredits = totalCredits;
  } else {
    v6BeforeMeters.currentCredits = totalCredits;
  }

  char txid[7];
  v6MakeAftTransactionId(txid);
  const uint32_t targetCardBalance = cardBalance + requestedRfidCredits;

  Serial.println("--- SPLIT PLAN ---");
  Serial.print("Machine CREDIT     : "); Serial.println(totalCredits);
  Serial.print("RFID requested     : "); Serial.println(requestedRfidCredits);
  Serial.print("Hopper remainder   : "); Serial.println(hopperCredits);
  Serial.print("Full AFT MU        : "); Serial.println(u64ToString(totalMinor));
  Serial.print("RFID before        : "); Serial.println(cardBalance);
  Serial.print("RFID target        : "); Serial.println(targetCardBalance);
  Serial.print("Transaction ID     : "); Serial.println(txid);

  if (!v6PrepareSplitCashoutJournal(
        totalCredits, requestedRfidCredits, totalMinor, txid)) {
    Serial.println("BLOCK: RAM PREPARED transaction init failed.");
    showMessage("SPLIT BLOCK", "RAM TX FAIL", 1600);
    return false;
  }

  // ------------------------------------------------------------------
  // STAGE 1: FULL AFT machine -> host. One initiate only.
  // ------------------------------------------------------------------
  Serial.println("--- STAGE 1: FULL AFT OUT ---");
  const bool aftRx = v6InitiateAftCashoutToHost(totalMinor, txid);

  if (aftRx && v6LastAft.valid) {
    v6UpdateJournal(
      v6LastAft.transferStatus == 0x40
        ? V6JournalState::PENDING
        : V6JournalState::PREPARED,
      v6LastAft.transferStatus
    );
  } else {
    v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED, 0xFF);
  }

  if (aftRx && v6LastAft.valid &&
      v6LastAft.transferStatus == 0x40) {
    v6WaitAftFinalStatus(8000UL);
  }

  if (v6AbortOnClearAftReject((uint64_t)totalCredits, "SPLIT-AFT")) {
    return false;
  }

  uint64_t afterAftCredits = totalCredits;
  const bool machineZero =
    v6WaitForExactCreditDecrease(0, afterAftCredits, 8000UL);

  Serial.print("After AFT CREDIT    : "); Serial.println(u64ToString(afterAftCredits));
  Serial.print("Full AFT zero proof : "); Serial.println(machineZero ? "PASS" : "FAIL/AMBIGUOUS");

  if (!machineZero) {
    const uint32_t settled =
      afterAftCredits <= (uint64_t)totalCredits ? (uint32_t)afterAftCredits
                                                : totalCredits;
    v6AfterMeters = V6MeterSnapshot();
    v6AfterMeters.valid = true;
    v6AfterMeters.currentCredits = settled;
    v6PersistSplitRecovery(
      V6_SPLIT_STAGE_AFT,
      v6LastAft.valid ? v6LastAft.transferStatus : 0xFF,
      settled
    );
    Serial.println("*** SPLIT AFT DID NOT REACH ZERO ***");
    Serial.println("AUTO RECOVERY will reconcile the actual moved value; LP72 will not be resent.");
    showMessage("AUTO RECOVERY", "AFT SETTLE", 1200);
    Serial.println("AUTO RECOVERY: background verification will settle this journal automatically.");
    return false;
  }

  v6AfterMeters = V6MeterSnapshot();
  v6AfterMeters.valid = true;
  v6AfterMeters.currentCredits = 0;
  if (!v6UpdateJournal(
        V6JournalState::SAS_VERIFIED,
        v6LastAft.valid ? v6LastAft.transferStatus : 0x00)) {
    Serial.println("Machine is ZERO but journal SAS_VERIFIED write failed.");
    Serial.println("DO NOT retry AFT.");
    showMessage("MACHINE ZERO", "JOURNAL ERROR", 2200);
    return false;
  }

  // ------------------------------------------------------------------
  // STAGE 2: return only the hopper remainder with proven 0x8A.
  // Before 0x8A, explicitly FF-acknowledge the AFT completion cycle.
  // We set PENDING + sasStatus 0x8A BEFORE transmission. If power fails
  // after that point recovery will never blind-resend the bonus.
  // ------------------------------------------------------------------
  if (hopperCredits > 0) {
    if (!v6SyncAftCycleBeforeValueCommand("SPLIT-RESTORE", 3500UL, true)) {
      Serial.println("Machine is ZERO but AFT final cycle is not acknowledged yet.");
      Serial.println("0x8A remainder restore NOT sent. Recovery will retry interrogation only.");
      showMessage("AFT ACK WAIT", "NO 8A SENT", 1800);
      return false;
    }

    Serial.println("--- STAGE 2: RESTORE HOPPER CREDIT WITH 0x8A ---");
    Serial.print("Restore credits     : "); Serial.println(hopperCredits);

    if (!v6UpdateJournal(V6JournalState::PENDING, 0x8A)) {
      Serial.println("Cannot persist 0x8A PENDING. 0x8A NOT SENT.");
      showMessage("RESTORE BLOCK", "RAM TX FAIL", 1800);
      return false;
    }

    uint8_t ack = 0xFF;
    const bool ackOK = sendLegacyBonus8A(hopperCredits, ack);

    uint64_t restoredCredits = 0xFFFFFFFFFFFFFFFFULL;
    const bool restoreExact =
      waitForExactCreditMeter((uint64_t)hopperCredits,
                              restoredCredits,
                              6000UL);

    Serial.print("0x8A ACK             : "); Serial.println(ackOK ? "PASS" : "NO/UNKNOWN");
    Serial.print("Machine restored    : "); Serial.println(u64ToString(restoredCredits));
    Serial.print("Restore exact proof : "); Serial.println(restoreExact ? "PASS" : "FAIL/AMBIGUOUS");

    if (!restoreExact) {
      const bool haveSnapshot = restoredCredits != 0xFFFFFFFFFFFFFFFFULL &&
                                restoredCredits <= (uint64_t)totalCredits;
      const uint32_t settled = haveSnapshot ? (uint32_t)restoredCredits : 0U;
      v6AfterMeters = V6MeterSnapshot();
      v6AfterMeters.valid = haveSnapshot;
      v6AfterMeters.currentCredits = settled;

      if (haveSnapshot) {
        v6PersistSplitRecovery(V6_SPLIT_STAGE_RESTORE,
                               ackOK ? ack : 0x8A, settled);
        Serial.println("*** SPLIT RESTORE NOT EXACT ***");
        Serial.println("AUTO RECOVERY will settle actual machine value; 0x8A will not be resent.");
        showMessage("AUTO RECOVERY", "RESTORE SETTLE", 1200);
        Serial.println("AUTO RECOVERY: background verification will settle this journal automatically.");
      } else {
        // No trustworthy LP1A snapshot. Leave this as an untagged legacy-style
        // state=9 record; the background service will adopt a stable LP1A value.
        v6AfterMeters = V6MeterSnapshot();
        v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED, ackOK ? ack : 0x8A);
        Serial.println("AUTO RECOVERY: waiting for stable LP1A settlement snapshot.");
        showMessage("AUTO RECOVERY", "WAIT STABLE CR", 1200);
      }
      return false;
    }

    v6AfterMeters = V6MeterSnapshot();
    v6AfterMeters.valid = true;
    v6AfterMeters.currentCredits = restoredCredits;

    // METERS_VERIFIED means the final machine balance (hopper remainder)
    // has been independently proven.
    if (!v6UpdateJournal(
          V6JournalState::METERS_VERIFIED,
          ackOK ? ack : 0x01)) {
      Serial.println("Restore verified but RAM transaction update failed.");
      Serial.println("DO NOT resend 0x8A.");
      showMessage("RESTORE OK", "JOURNAL ERROR", 2200);
      return false;
    }
  }

  // ------------------------------------------------------------------
  // STAGE 3: credit only requested amount to the SAME RFID card.
  // hopperCredits==0 remains in SAS_VERIFIED, also accepted by the atomic
  // card commit primitive.
  // ------------------------------------------------------------------
  Serial.println("--- STAGE 3: RFID COMMIT ---");

  if (!cardActive || !v6SameActiveCardUid(v6LatestJournal)) {
    Serial.println("Machine split is ready but SAME RFID card is not active.");
    Serial.println("Retag SAME card. No AFT/0x8A resend.");
    showMessage("SPLIT READY", "RETAG SAME CARD", 2200);
    return false;
  }

  if (!v6AtomicCommitCardBalance(targetCardBalance)) {
    Serial.println("Machine split succeeded but RFID write/verify failed.");
    Serial.println("DO NOT resend AFT or 0x8A. Retag SAME card for recovery.");
    showMessage("CARD WRITE FAIL", "DO NOT RESEND", 2200);
    return false;
  }

  v6UpdateJournal(
    V6JournalState::DONE,
    v6LatestJournal.sasStatus
  );

  inputValue = 0;

  Serial.println();
  Serial.println("*** FINAL SPLIT CASHOUT SUCCESS ***");
  Serial.print("RFID added          : "); Serial.println(requestedRfidCredits);
  Serial.print("RFID new balance    : "); Serial.println(cardBalance);
  Serial.print("Machine/Hopper left : "); Serial.println(hopperCredits);
  Serial.println("Press S6000 physical CASH OUT to pay the remaining machine credits.");
  Serial.println("Hopper requires machine Option 61 = 0001 (Hopper Only) or 0003 (Printer+Hopper).");
  Serial.println("RAM transaction : DONE");
  Serial.println();

  if (hopperCredits > 0) {
    showMessage(
      String("CARD +") + requestedRfidCredits,
      String("HOPPER:") + hopperCredits,
      2600
    );
  } else {
    showMessage(
      String("CARD +") + requestedRfidCredits,
      "MACHINE CREDIT 0",
      2600
    );
  }

  return true;
}


bool readCashOutLimitA4(bool displayResult = true) {
  uint8_t rx[20];
  uint16_t rx9[20];
  const uint8_t game0000[2] = {0x00, 0x00};
  const size_t n = sasTypeS(0xA4, game0000, 2, rx, rx9, 20);

  if (n < 8 || rx[0] != sasAddress || rx[1] != 0xA4 || !sasCheckResponseCrc(rx, n)) {
    sasCaps.a4Responded = false;
    if (displayResult) showMessage("LPA4 FAIL", String("RX=") + n);
    return false;
  }

  uint64_t limit = 0;
  if (!bcdToU64(&rx[4], 2, limit)) {
    sasCaps.a4Responded = false;
    if (displayResult) showMessage("LPA4 BCD ERR", "CHECK RAW USB");
    return false;
  }

  sasCaps.a4Responded = true;
  sasCaps.cashOutLimitCredits = limit;
  Serial.print("Cash-out limit (credits): "); Serial.println(u64ToString(limit));
  if (displayResult) showMessage("CASHOUT LIMIT", u64ToString(limit));
  return true;
}

void diagnosticA4() {
  readCashOutLimitA4(true);
}

bool probeEnabledFeaturesA0(bool displayResult = true) {
  uint8_t rx[24];
  uint16_t rx9[24];
  const uint8_t game0000[2] = {0x00, 0x00};
  const size_t n = sasTypeS(0xA0, game0000, 2, rx, rx9, 24);

  if (n < 12 || rx[0] != sasAddress || rx[1] != 0xA0 || !sasCheckResponseCrc(rx, n)) {
    a0Known = false;
    sasCaps.a0Responded = false;
    if (displayResult) showMessage("LPA0 FAIL", String("RX=") + n);
    return false;
  }

  const uint8_t f1 = rx[4];
  const uint8_t f2 = rx[5];
  const uint8_t f3 = rx[6];

  a0Known = true;
  a0TicketRedemption = (f1 & 0x80U) != 0;
  a0ValidationStyle = (uint8_t)((f1 >> 5) & 0x03U);
  a0AftSupported = (f2 & 0x40U) != 0;
  a0FortyMsSupported = (f3 & 0x01U) != 0;

  sasCaps.a0Responded = true;
  sasCaps.ticketRedemptionEnabled = a0TicketRedemption;
  sasCaps.validationExtensions = (f1 & 0x10U) != 0;
  sasCaps.validationStyle = a0ValidationStyle;
  sasCaps.aftEnabled = a0AftSupported;
  sasCaps.fortyMsSupported = a0FortyMsSupported;

  Serial.print("A0 Features1=0x"); Serial.println(f1, HEX);
  Serial.print("A0 Features2=0x"); Serial.println(f2, HEX);
  Serial.print("A0 Features3=0x"); Serial.println(f3, HEX);
  Serial.print("Ticket redemption: "); Serial.println(a0TicketRedemption ? "YES" : "NO");
  Serial.print("AFT supported: "); Serial.println(a0AftSupported ? "YES" : "NO");
  Serial.print("40ms supported: "); Serial.println(a0FortyMsSupported ? "YES" : "NO");

  if (displayResult) {
    showMessage(
      String("TKT:") + (a0TicketRedemption ? "Y" : "N") + " AFT:" + (a0AftSupported ? "Y" : "N"),
      String("V:") + validationName(a0ValidationStyle) + " 40:" + (a0FortyMsSupported ? "Y" : "N"),
      1800
    );
  }
  return true;
}

void diagnosticA0() {
  probeEnabledFeaturesA0(true);
}

bool probeAftStatus74(bool displayResult) {
  // Do not send AFT 74 to a protocol revision that predates AFT unless A0
  // explicitly told us AFT is enabled.
  if (sasCaps.versionKnown && !sasCaps.aftProtocolByVersion && !sasCaps.aftEnabled) {
    sasCaps.lp74Responded = false;
    if (displayResult) showMessage("AFT NOT IN VER", sasCaps.versionText, 1200);
    return false;
  }
  if (sasCaps.a0Responded && !sasCaps.aftEnabled) {
    sasCaps.lp74Responded = false;
    if (displayResult) showMessage("AFT DISABLED", "A0 F2.6 = 0", 1200);
    return false;
  }

  const uint8_t payload[4] = {0xFF, 0x00, 0x00, 0x00};
  uint8_t rx[56];
  uint16_t rx9[56];
  const size_t n = sasTypeS(0x74, payload, sizeof(payload), rx, rx9, 56);

  if (n < 40 || rx[0] != sasAddress || rx[1] != 0x74 || !sasCheckResponseCrc(rx, n)) {
    sasCaps.lp74Responded = false;
    if (displayResult) showMessage("LP74 FAIL", String("RX=") + n);
    return false;
  }

  const uint8_t gameLock = rx[7];
  const uint8_t available = rx[8];
  const uint8_t hostCashout = rx[9];
  const uint8_t aftStatus = rx[10];

  uint64_t cashable = 0, restricted = 0, nonrestricted = 0, transferLimit = 0;
  const bool b1 = bcdToU64(&rx[12], 5, cashable);
  const bool b2 = bcdToU64(&rx[17], 5, restricted);
  const bool b3 = bcdToU64(&rx[22], 5, nonrestricted);
  const bool b4 = bcdToU64(&rx[27], 5, transferLimit);

  Serial.print("AFT game lock status: 0x"); Serial.println(gameLock, HEX);
  Serial.print("AFT available transfers: 0x"); Serial.println(available, HEX);
  Serial.print("AFT host cashout: 0x"); Serial.println(hostCashout, HEX);
  Serial.print("AFT status: 0x"); Serial.println(aftStatus, HEX);
  if (b1) { Serial.print("AFT cashable cents: "); Serial.println(u64ToString(cashable)); }
  if (b2) { Serial.print("AFT restricted cents: "); Serial.println(u64ToString(restricted)); }
  if (b3) { Serial.print("AFT nonrestricted cents: "); Serial.println(u64ToString(nonrestricted)); }
  if (b4) { Serial.print("AFT transfer limit cents: "); Serial.println(u64ToString(transferLimit)); }

  if (!b4) {
    sasCaps.lp74Responded = false;
    if (displayResult) showMessage("LP74 BCD ERR", "CHECK RAW USB");
    return false;
  }

  sasCaps.lp74Responded = true;
  sasCaps.aftTransferLimitCents = transferLimit;
  for (uint8_t i = 0; i < 4; i++) sasCaps.aftAssetNumber[i] = rx[3 + i];
  sasCaps.aftGameLockStatus = gameLock;
  sasCaps.aftAvailableTransfers = available;
  sasCaps.aftHostCashoutStatus = hostCashout;
  sasCaps.aftStatusFlags = aftStatus;
  sasCaps.aftMaxBufferIndex = rx[11];
  sasCaps.aftCashableMinorUnits = b1 ? cashable : 0;
  sasCaps.aftRestrictedMinorUnits = b2 ? restricted : 0;
  sasCaps.aftNonrestrictedMinorUnits = b3 ? nonrestricted : 0;

  Serial.print("AFT asset number   : ");
  for (uint8_t i = 0; i < 4; i++) {
    if (sasCaps.aftAssetNumber[i] < 0x10) Serial.print('0');
    Serial.print(sasCaps.aftAssetNumber[i], HEX);
  }
  Serial.println();

  if (displayResult) {
    showMessage(
      String("AFT LIM:") + u64ToString(transferLimit),
      String("TOGM:") + ((available & 0x01U) ? "Y" : "N") + " AFT:" + ((aftStatus & 0x80U) ? "Y" : "N"),
      1800
    );
  }
  return true;
}

void diagnostic74() {
  probeAftStatus74(true);
}

void showAutoStatus() {
  String v = sasCaps.versionKnown ? sasCaps.versionText : String("?");
  String p = sasProfileName(sasCaps.profile);
  showMessage(String("SAS:") + v + " " + p,
              String("T:") + (sasCaps.ticketRedemptionEnabled ? "Y" : "N") +
              " A:" + (sasCaps.aftEnabled ? "Y" : "N") +
              " R:" + (sasCaps.remoteHandpayByVersion ? "Y" : "N"), 1700);
}

void showLimitsSummary() {
  String l1 = sasCaps.a4Responded ? String("CO:") + u64ToString(sasCaps.cashOutLimitCredits)
                                  : String("CO:?");
  String l2 = sasCaps.lp74Responded ? String("AFTc:") + u64ToString(sasCaps.aftTransferLimitCents)
                                    : String("AFTc:N/A");
  showMessage(l1, l2, 1800);
}

bool autoDetectSasProfile(bool displaySteps = true) {
  // Save EEPROM identity before the fresh LP54 overwrites runtime sasCaps.
  const bool hadCache = savedProfileValid;
  const uint16_t cachedVersionCode = hadCache ? savedProfile.versionCode : 0;
  const String cachedSerial = hadCache
    ? fixedToString(savedProfile.machineSerial, sizeof(savedProfile.machineSerial))
    : String("");

  sasCaps = SasCapabilities();
  a0Known = false;
  a0AftSupported = false;
  a0TicketRedemption = false;
  a0FortyMsSupported = false;
  a0ValidationStyle = 0;

  if (displaySteps) {
    lcdLine(0, "SEARCH S6000");
    lcdLine(1, "LP54...");
  }

  bool verOK = readVersion54(false);
  if (!verOK) {
    sasOnline = false;
    cachedIdentityMatched = false;
    return false;
  }

  const String freshVersionText = sasCaps.versionText;
  const String freshSerial = sasCaps.machineSerial;
  const uint16_t freshVersionCode = sasCaps.versionCode;
  const SasProfile freshProfile = sasCaps.profile;

  cachedIdentityMatched =
    hadCache &&
    cachedVersionCode == freshVersionCode &&
    cachedSerial == freshSerial;

  if (cachedIdentityMatched) {
    // Restore last verified capabilities immediately.
    applyStoredProfileToRuntime(savedProfile);

    // Preserve the fresh identity returned by LP54.
    sasCaps.versionKnown = true;
    sasCaps.versionText = freshVersionText;
    sasCaps.machineSerial = freshSerial;
    sasCaps.versionCode = freshVersionCode;
    sasCaps.profile = freshProfile;

    Serial.println("EEPROM CACHE: identity MATCH. Using last verified profile while refreshing live capabilities.");
  } else if (hadCache) {
    Serial.println("EEPROM CACHE: identity MISMATCH. Old cache will NOT be trusted for this machine.");
    Serial.print("Cached serial: "); Serial.println(cachedSerial);
    Serial.print("Fresh serial : "); Serial.println(freshSerial);
  }

  if (displaySteps) {
    lcdLine(0, String("SAS:") + sasCaps.versionText);
    lcdLine(1, cachedIdentityMatched ? "CACHE MATCH" : "PROBE 1F...");
  }

  // Learn the base accounting denomination before interpreting credits.
  machineInfo.valid = false;
  readMachineInfo1F(false, false);

  if (displaySteps) lcdLine(1, "PROBE A0...");

  // Always refresh live features. The cache makes startup deterministic,
  // but the connected machine remains the source of truth.
  probeEnabledFeaturesA0(false);

  if (displaySteps) lcdLine(1, "PROBE A4...");
  readCashOutLimitA4(false);

  if ((sasCaps.aftProtocolByVersion || sasCaps.aftEnabled) &&
      (!sasCaps.a0Responded || sasCaps.aftEnabled)) {
    if (displaySteps) lcdLine(1, "PROBE AFT74...");
    probeAftStatus74(false);
  }

  // A simple read-only LP1A is the final "machine is alive now" check.
  uint64_t credits = 0;
  const bool creditsOK = readCurrentCredits1A(credits, false);

  sasOnline = creditsOK;
  heartbeatFailCount = 0;

  printAutoProfileSummary();

  if (displaySteps) {
    if (sasOnline) {
      String line1 = String("SAS:") + sasCaps.versionText +
                     (cachedIdentityMatched ? " CACHE" : " LIVE");
      String line2 = String("T:") + (sasCaps.ticketRedemptionEnabled ? "Y" : "N") +
                     " A:" + (sasCaps.aftEnabled ? "Y" : "N") +
                     " CR:" + u64ToString(credits);
      showMessage(line1, line2, 1400);
    } else {
      showMessage("S6000 RESP PART", "1A RETRY...", 600);
    }
  }

  return sasOnline;
}


// ============================================================================
// V6 SAS ADDRESS AUTO-SCAN / JOURNAL RECOVERY
// ============================================================================
bool v6ProbeAddress54(uint8_t candidate) {
  const uint8_t old = sasAddress;
  sasAddress = candidate;
  uint8_t rx[64]; uint16_t rx9[64];
  const size_t n = sasTypeRPriority(0x54, rx, rx9, sizeof(rx));
  const bool ok = n >= 8 && rx[0] == candidate && rx[1] == 0x54 && sasCheckResponseCrc(rx, n);
  if (!ok) sasAddress = old;
  return ok;
}

bool v6ScanSasAddress(bool displaySteps = true) {
#if V6_ENABLE_ADDRESS_AUTO_SCAN
  const uint8_t original = sasAddress;
  if (displaySteps) { lcdLine(0, "SAS ADDR SCAN"); lcdLine(1, "1..127"); }
  Serial.println("V6 SAS ADDRESS SCAN 1..127");
  for (uint16_t a = 1; a <= 127; a++) {
    if (displaySteps && (a == 1 || (a % 8) == 0)) lcdLine(1, String("ADDR ") + a);
    if (v6ProbeAddress54((uint8_t)a)) {
      sasAddress = (uint8_t)a;
      Serial.print("V6 ADDRESS FOUND: "); Serial.println(sasAddress);
      v6TraceAdd(4, sasAddress, 1);
      return true;
    }
  }
  sasAddress = original;
  Serial.println("V6 ADDRESS SCAN: no S6000 response.");
  return false;
#else
  (void)displaySteps;
  return false;
#endif
}

void v6ServiceRecovery() {
  if (!sasOnline || !v6RecoveryRequired || !v6LatestJournalValid || v5EventCount > 0) return;
  const uint32_t now = millis();
  if ((int32_t)(now - v6NextRecoveryMs) < 0) return;
  v6NextRecoveryMs = now + 5000UL;

  Serial.println("V6 RAM RECOVERY: interrogating unfinished in-memory transaction; never blind-resending a value command.");
  const V6JournalType type = (V6JournalType)v6LatestJournal.type;
  if (type == V6JournalType::RFID_SPLIT_CASHOUT) {
    const uint32_t totalCredits = v6LatestJournal.amountCredits;

    if (v6LatestJournal.balanceAfter < v6LatestJournal.balanceBefore) {
      Serial.println("SPLIT RECOVERY: invalid card balance journal.");
      return;
    }

    const uint32_t rfidCredits =
      v6LatestJournal.balanceAfter - v6LatestJournal.balanceBefore;

    if (rfidCredits == 0 || rfidCredits > totalCredits) {
      Serial.println("SPLIT RECOVERY: invalid split amounts.");
      return;
    }

    const uint32_t hopperCredits = totalCredits - rfidCredits;
    uint64_t current = 0;
    if (!readCurrentCredits1A(current, false)) {
      Serial.println("SPLIT RECOVERY: LP1A unavailable. No value command resent.");
      return;
    }

    const V6JournalState st = (V6JournalState)v6LatestJournal.state;

    Serial.print("SPLIT RECOVERY state="); Serial.print((uint8_t)st);
    Serial.print(" total="); Serial.print(totalCredits);
    Serial.print(" card="); Serial.print(rfidCredits);
    Serial.print(" hopper="); Serial.print(hopperCredits);
    Serial.print(" machineNow="); Serial.println(u64ToString(current));

    if (st == V6JournalState::PREPARED) {
      // PREPARED may mean power failed before/after LP72, OR an older build
      // stored an immediate clear rejection here. Only the latter is safe to
      // terminally clear when the machine CREDIT is still exactly unchanged.
      if (v6AftTransferRejected(v6LatestJournal.sasStatus) &&
          current == (uint64_t)totalCredits) {
        v6AfterMeters = V6MeterSnapshot();
        v6AfterMeters.valid = true;
        v6AfterMeters.currentCredits = current;
        v6UpdateJournal(V6JournalState::FAILED, v6LatestJournal.sasStatus);
        Serial.println("SPLIT RECOVERY: clear AFT rejection + unchanged CREDIT -> FAILED.");
        v6ClearRuntimeTransaction("SPLIT REJECTED / NO MOVEMENT");
        inputValue = 0;
        cancelMoneyConfirm(false);
      } else {
        Serial.println("SPLIT RECOVERY: PREPARED is ambiguous; no LP72 resend.");
      }
      return;
    }

    if (st == V6JournalState::PENDING && v6LatestJournal.sasStatus != 0x8A) {
      // AFT was at least observed/pending. FF interrogates AND acknowledges a
      // final status; it never re-initiates the value transfer.
      const bool ffOK = v6InterrogateAft72(0xFF, true, false);
      if (current == 0) {
        v6AfterMeters = V6MeterSnapshot();
        v6AfterMeters.valid = true;
        v6AfterMeters.currentCredits = 0;
        if (!v6UpdateJournal(V6JournalState::SAS_VERIFIED,
                             v6LastAft.valid ? v6LastAft.transferStatus
                                             : v6LatestJournal.sasStatus)) {
          return;
        }
      } else if (ffOK && v6LastAft.valid &&
                 v6AftTransferRejected(v6LastAft.transferStatus) &&
                 current == (uint64_t)totalCredits) {
        v6AfterMeters = V6MeterSnapshot();
        v6AfterMeters.valid = true;
        v6AfterMeters.currentCredits = current;
        v6UpdateJournal(V6JournalState::FAILED, v6LastAft.transferStatus);
        Serial.println("SPLIT RECOVERY: pending resolved as clear rejection; unchanged CREDIT -> FAILED.");
        v6ClearRuntimeTransaction("SPLIT PENDING REJECT / NO MOVEMENT");
        inputValue = 0;
        cancelMoneyConfirm(false);
        return;
      } else {
        Serial.println("SPLIT RECOVERY: AFT pending/ambiguous; no resend.");
        return;
      }
    }

    if (st == V6JournalState::RECOVERY_REQUIRED) {
      // Clear AFT rejection with unchanged CREDIT means no money moved.
      if (v6AftTransferRejected(v6LatestJournal.sasStatus) &&
          current == (uint64_t)totalCredits) {
        v6AfterMeters = V6MeterSnapshot();
        v6AfterMeters.valid = true;
        v6AfterMeters.currentCredits = current;
        v6UpdateJournal(V6JournalState::FAILED, v6LatestJournal.sasStatus);
        Serial.println("SPLIT RECOVERY: clear rejection + unchanged CREDIT -> FAILED; RAM auto-clear.");
        v6ClearRuntimeTransaction("SPLIT RECOVERY REJECT / NO MOVEMENT");
        inputValue = 0;
        cancelMoneyConfirm(false);
        return;
      }

      // Otherwise settle automatically from a stage-marked snapshot, or adopt
      // a stable LP1A snapshot for legacy V6.5.x state=9 journals.
      v6AutoResolveSplitRecovery(totalCredits, rfidCredits, (uint32_t)current);
      return;
    }

    // Refresh because the branch above may have advanced PENDING -> SAS_VERIFIED.
    V6JournalState nowState = (V6JournalState)v6LatestJournal.state;

    if (nowState == V6JournalState::SAS_VERIFIED && hopperCredits > 0) {
      // This state proves the full AFT is complete and, by construction,
      // RESTORE-PENDING has not yet been persisted. It is safe to start the
      // restore exactly once after first writing the PENDING/0x8A marker.
      if (!cardActive || !v6SameActiveCardUid(v6LatestJournal)) {
        Serial.println("SPLIT RECOVERY: AFT verified. Retag SAME RFID before hopper restore.");
        return;
      }

      if (!v6SyncAftCycleBeforeValueCommand("SPLIT-REC", 3000UL, false)) {
        Serial.println("SPLIT RECOVERY: final AFT cycle not acknowledged yet; 0x8A not sent.");
        return;
      }

      if (!v6UpdateJournal(V6JournalState::PENDING, 0x8A)) {
        Serial.println("SPLIT RECOVERY: cannot persist restore-pending marker.");
        return;
      }

      uint8_t ack = 0xFF;
      const bool ackOK = sendLegacyBonus8A(hopperCredits, ack);

      uint64_t restored = 0;
      const bool exact =
        waitForExactCreditMeter((uint64_t)hopperCredits, restored, 6000UL);

      if (!exact) {
        v6AfterMeters = V6MeterSnapshot();
        v6AfterMeters.valid = true;
        v6AfterMeters.currentCredits = restored;
        v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED,
                        ackOK ? ack : 0x8A);
        Serial.println("SPLIT RECOVERY: restore ambiguous. No 0x8A resend.");
        return;
      }

      v6AfterMeters = V6MeterSnapshot();
      v6AfterMeters.valid = true;
      v6AfterMeters.currentCredits = restored;
      if (!v6UpdateJournal(V6JournalState::METERS_VERIFIED,
                           ackOK ? ack : 0x01)) {
        return;
      }
    }

    nowState = (V6JournalState)v6LatestJournal.state;

    if (nowState == V6JournalState::PENDING &&
        v6LatestJournal.sasStatus == 0x8A) {
      // 0x8A may already have been sent. NEVER resend from this state.
      if (hopperCredits > 0 && current == hopperCredits) {
        v6AfterMeters = V6MeterSnapshot();
        v6AfterMeters.valid = true;
        v6AfterMeters.currentCredits = current;
        if (!v6UpdateJournal(V6JournalState::METERS_VERIFIED, 0x8A)) return;
      } else {
        Serial.println("SPLIT RECOVERY: 0x8A PENDING ambiguous; no resend.");
        return;
      }
    }

    nowState = (V6JournalState)v6LatestJournal.state;

    if (nowState == V6JournalState::SAS_VERIFIED && hopperCredits == 0) {
      // Full cashout to card: machine zero is already the final machine state.
    } else if (nowState != V6JournalState::METERS_VERIFIED &&
               nowState != V6JournalState::RFID_COMMITTED) {
      return;
    }

    if (!cardActive || !v6SameActiveCardUid(v6LatestJournal)) {
      Serial.println("SPLIT RECOVERY: machine side verified; retag SAME RFID card.");
      return;
    }

    uint32_t liveBalance = 0;
    if (!readBalanceFromCard(liveBalance)) {
      Serial.println("SPLIT RECOVERY: RFID read failed.");
      return;
    }

    if (liveBalance == v6LatestJournal.balanceAfter) {
      cardBalance = liveBalance;
      v6UpdateJournal(V6JournalState::DONE, v6LatestJournal.sasStatus);
      Serial.println("SPLIT RECOVERY: RFID already committed; DONE.");
      return;
    }

    if (liveBalance != v6LatestJournal.balanceBefore) {
      Serial.println("SPLIT RECOVERY: RFID balance differs from journal; manual review.");
      return;
    }

    cardBalance = liveBalance;
    if (v6AtomicCommitCardBalance(v6LatestJournal.balanceAfter)) {
      v6UpdateJournal(V6JournalState::DONE, v6LatestJournal.sasStatus);
      Serial.println("SPLIT RECOVERY: SAME RFID committed; DONE.");
    }
  } else if (type == V6JournalType::RFID_CASHOUT) {
    // NEVER re-initiate 0x72 during recovery.
    const V6MeterSnapshot before = v6RestoreMeters(v6LatestJournal.beforeMeters);
    if (before.currentCredits < v6LatestJournal.amountCredits) {
      Serial.println("RFID CASHOUT RECOVERY: invalid persisted PRE meter; manual review.");
      return;
    }

    const uint64_t expected =
      before.currentCredits - (uint64_t)v6LatestJournal.amountCredits;

    uint64_t current = 0;
    if (!readCurrentCredits1A(current, false)) {
      Serial.println("RFID CASHOUT RECOVERY: LP1A unavailable; no resend.");
      return;
    }

    // FF is interrogation only here and also acknowledges final completion.
    // It must never become another initiate request.
    v6InterrogateAft72(0xFF, true, false);

    Serial.print("RFID CASHOUT RECOVERY PRE=");
    Serial.print(u64ToString(before.currentCredits));
    Serial.print(" EXPECT=");
    Serial.print(u64ToString(expected));
    Serial.print(" NOW=");
    Serial.println(u64ToString(current));

    if (current == expected) {
      v6AfterMeters = V6MeterSnapshot();
      v6AfterMeters.valid = true;
      v6AfterMeters.currentCredits = current;

      if (v6LatestJournal.state != (uint8_t)V6JournalState::SAS_VERIFIED &&
          v6LatestJournal.state != (uint8_t)V6JournalState::METERS_VERIFIED &&
          v6LatestJournal.state != (uint8_t)V6JournalState::RFID_COMMITTED) {
        v6UpdateJournal(V6JournalState::SAS_VERIFIED,
                        v6LastAft.valid ? v6LastAft.transferStatus : 0x00);
      }

      if (cardActive && v6SameActiveCardUid(v6LatestJournal)) {
        uint32_t liveBalance = 0;
        if (readBalanceFromCard(liveBalance)) {
          if (liveBalance == v6LatestJournal.balanceAfter) {
            cardBalance = liveBalance;
            v6UpdateJournal(V6JournalState::DONE, 0x00);
            Serial.println("RFID CASHOUT RECOVERY: card already credited; DONE.");
          } else if (liveBalance == v6LatestJournal.balanceBefore) {
            cardBalance = liveBalance;
            if (v6AtomicCommitCardBalance(v6LatestJournal.balanceAfter)) {
              v6UpdateJournal(V6JournalState::DONE, 0x00);
              Serial.println("RFID CASHOUT RECOVERY: same card credited; DONE.");
            }
          } else {
            Serial.println("RFID CASHOUT RECOVERY: card balance does not match journal; manual review.");
          }
        }
      } else {
        Serial.println("RFID CASHOUT RECOVERY: machine decrease verified; retag SAME RFID card.");
      }
    } else if (current == before.currentCredits &&
               v6AftTransferRejected(v6LatestJournal.sasStatus)) {
      v6AfterMeters = V6MeterSnapshot();
      v6AfterMeters.valid = true;
      v6AfterMeters.currentCredits = current;
      v6UpdateJournal(V6JournalState::FAILED, v6LatestJournal.sasStatus);
      Serial.println("RFID CASHOUT RECOVERY: clear rejection + unchanged CREDIT -> FAILED.");
      v6ClearRuntimeTransaction("RFID CASHOUT REJECT / NO MOVEMENT");
      inputValue = 0;
      cancelMoneyConfirm(false);
    } else if (current == before.currentCredits &&
               v6LastAft.valid &&
               v6AftTransferRejected(v6LastAft.transferStatus)) {
      v6AfterMeters = V6MeterSnapshot();
      v6AfterMeters.valid = true;
      v6AfterMeters.currentCredits = current;
      v6UpdateJournal(V6JournalState::FAILED, v6LastAft.transferStatus);
      Serial.println("RFID CASHOUT RECOVERY: FF confirmed rejection + unchanged CREDIT -> FAILED.");
      v6ClearRuntimeTransaction("RFID CASHOUT FF REJECT / NO MOVEMENT");
      inputValue = 0;
      cancelMoneyConfirm(false);
    } else {
      Serial.println("RFID CASHOUT RECOVERY: current credit not exact expected value.");
      Serial.println("No LP72 resend. Manual review required.");
      v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED,
                      v6LastAft.valid ? v6LastAft.transferStatus : 0xFF);
    }
  } else if (type == V6JournalType::AFT) {
    if (v6InterrogateAft72(0xFF, true, false)) {
      if (v6LastAft.transferStatus == 0x00 || v6LastAft.transferStatus == 0x01) {
        // V6.5.2 migration fix:
        // A generic observer-only AFT journal contains no card UID, amount, balance
        // target or transaction id.  If FF reports a terminal SUCCESS, this record
        // has no remaining value-side commit to perform.  Do NOT demand a meter delta:
        // its PRE snapshot may have been captured after the real transfer event,
        // which is exactly what caused the endless FF -> LP2F recovery loop.
        if (v6ObserverOnlyAftJournal(v6LatestJournal)) {
          v6AfterMeters = V6MeterSnapshot();
          v6ReadSelectedMeters2F(v6AfterMeters, true, false); // diagnostic only
          v6LastMeterVerifyPassed = true;
          v6UpdateJournal(V6JournalState::DONE, v6LastAft.transferStatus);
          Serial.println("AFT OBSERVER RECOVERY: FF success; observer-only journal DONE/cleared.");
          return;
        }

        // Non-observer AFT journal: keep the stricter historical proof rule.
        const V6MeterSnapshot before = v6RestoreMeters(v6LatestJournal.beforeMeters);
        v6AfterMeters = V6MeterSnapshot();
        const bool metersRead = v6ReadSelectedMeters2F(v6AfterMeters, true, false);
        const bool metersOK = metersRead && v6VerifyMeterDelta(V6JournalType::AFT, before, v6AfterMeters);
        v6LastMeterVerifyPassed = metersOK;
        if (metersOK) {
          v6UpdateJournal(V6JournalState::METERS_VERIFIED, v6LastAft.transferStatus);
          v6UpdateJournal(V6JournalState::DONE, v6LastAft.transferStatus);
        } else {
          v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED, v6LastAft.transferStatus);
        }
      } else if (v6LastAft.transferStatus == 0x40) {
        v6UpdateJournal(V6JournalState::PENDING, 0x40);
      } else if (v6LastAft.transferStatus != 0xFF) {
        v6UpdateJournal(V6JournalState::FAILED, v6LastAft.transferStatus);
      }
    }
  } else if (type == V6JournalType::TICKET_IN) {
    if (interrogateTicket71Priority(false)) {
      if (ticketIn.machineStatus <= 0x02) {
        const V6MeterSnapshot before = v6RestoreMeters(v6LatestJournal.beforeMeters);
        v6AfterMeters = V6MeterSnapshot();
        const bool metersRead = v6ReadSelectedMeters2F(v6AfterMeters, true, false);
        const bool metersOK = metersRead && v6VerifyMeterDelta(V6JournalType::TICKET_IN, before, v6AfterMeters);
        v6LastMeterVerifyPassed = metersOK;
        if (metersOK) {
          v6UpdateJournal(V6JournalState::METERS_VERIFIED, ticketIn.machineStatus);
          v6UpdateJournal(V6JournalState::DONE, ticketIn.machineStatus);
        } else {
          v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED, ticketIn.machineStatus);
        }
      } else if (ticketIn.machineStatus == 0x40 || ticketIn.machineStatus == 0x20) {
        v6UpdateJournal(V6JournalState::PENDING, ticketIn.machineStatus);
      } else if (ticketIn.machineStatus != 0xFF) {
        v6UpdateJournal(V6JournalState::FAILED, ticketIn.machineStatus);
      }
    }
  } else if (type == V6JournalType::RFID_ATOMIC) {
    // NEVER resend 0x8A during recovery.
    const V6MeterSnapshot before = v6RestoreMeters(v6LatestJournal.beforeMeters);
    const uint64_t expected =
      before.currentCredits + (uint64_t)v6LatestJournal.amountCredits;

    uint64_t current = 0;
    if (!readCurrentCredits1A(current, false)) {
      Serial.println("RFID RECOVERY: LP1A unavailable; keeping recovery required.");
      return;
    }

    Serial.print("RFID RECOVERY PRE="); Serial.print(u64ToString(before.currentCredits));
    Serial.print(" EXPECT="); Serial.print(u64ToString(expected));
    Serial.print(" NOW="); Serial.println(u64ToString(current));

    if (current == expected) {
      v6AfterMeters = V6MeterSnapshot();
      v6AfterMeters.valid = true;
      v6AfterMeters.currentCredits = current;

      if (v6LatestJournal.state != (uint8_t)V6JournalState::SAS_VERIFIED &&
          v6LatestJournal.state != (uint8_t)V6JournalState::METERS_VERIFIED) {
        v6UpdateJournal(V6JournalState::SAS_VERIFIED, 0x01);
      }

      if (cardActive && v6SameActiveCardUid(v6LatestJournal)) {
        uint32_t liveBalance = 0;
        if (readBalanceFromCard(liveBalance)) {
          if (liveBalance == v6LatestJournal.balanceAfter) {
            cardBalance = liveBalance;
            v6UpdateJournal(V6JournalState::DONE, 0x01);
            Serial.println("RFID RECOVERY: card already has committed balance; DONE.");
          } else if (liveBalance == v6LatestJournal.balanceBefore) {
            cardBalance = liveBalance;
            if (v6AtomicCommitCardBalance(v6LatestJournal.balanceAfter)) {
              v6UpdateJournal(V6JournalState::DONE, 0x01);
              Serial.println("RFID RECOVERY: same card committed; DONE.");
            }
          } else {
            Serial.println("RFID RECOVERY: card balance differs from both journal states; manual review.");
          }
        }
      } else {
        Serial.println("RFID RECOVERY: credit verified; retag the SAME card to commit balance.");
      }
    } else if (current == before.currentCredits) {
      // V6.5.14: PRE == NOW is a definite no-movement result once it remains stable.
      // This is exactly the PRE=0 EXPECT=1 NOW=0 loop seen on the real machine.
      // Do not resend 0x8A; simply terminate and clear the volatile transaction.
      if (v6ConfirmStableCreditExact(before.currentCredits, 3)) {
        v6UpdateJournal(V6JournalState::FAILED, 0x00);
        Serial.println("RFID RECOVERY: PRE == NOW and stable; value did not move.");
        Serial.println("RFID RECOVERY: auto-clear RAM transaction; next transaction enabled.");
        v6ClearRuntimeTransaction("RFID DEPOSIT NO MOVEMENT");
        inputValue = 0;
        cancelMoneyConfirm(false);
      } else {
        v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED, 0xFF);
        Serial.println("RFID RECOVERY: PRE value not stable; keeping volatile recovery (no resend).");
      }
    } else {
      // NOW differs from both PRE and EXPECT: this can reflect partial/unrelated machine
      // activity and cannot be settled safely from LP1A alone. Never resend a value command.
      v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED, 0xFF);
      Serial.println("RFID RECOVERY: NOW differs from PRE/EXPECT; ambiguous, no resend.");
    }
  } else if (type == V6JournalType::HANDPAY) {
    // A failed LP1B read is not proof that a handpay was cleared. Keep the
    // record recoverable until the machine reports a definitive reset event/state.
    if (readHandpay1BPriority()) v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED, 0x51);
    else                         v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED, 0xFF);
  } else {
    // Ticket-out records are never re-issued.
    v6AfterMeters = V6MeterSnapshot();
    if (v6ReadSelectedMeters2F(v6AfterMeters, true, false))
      v6UpdateJournal(V6JournalState::RECOVERY_REQUIRED, 0xFF);
  }
}

// ============================================================================
// STARTUP / RECONNECT / HEARTBEAT
// ============================================================================
bool tryConnectOnce(bool displaySteps = true) {
  // Re-establish proven GP activity before LP54/profile interrogation.
  warmGeneralPoll(600UL, false);
  bool ok = autoDetectSasProfile(displaySteps);

  if (!ok && V6_ENABLE_ADDRESS_AUTO_SCAN) {
    const uint32_t now = millis();
    if (v6LastAddressScanMs == 0 || (uint32_t)(now - v6LastAddressScanMs) >= V6_ADDRESS_SCAN_COOLDOWN_MS) {
      v6LastAddressScanMs = now;
      if (v6ScanSasAddress(displaySteps)) ok = autoDetectSasProfile(displaySteps);
    }
  }

  if (ok) {
    sasOnline = true;
    v5State = V5RuntimeState::NORMAL;
    clearV5EventQueue();
    reconnectAttempt = 0;
    heartbeatFailCount = 0;
    nextHeartbeatMs = millis() + LINK_HEARTBEAT_MS;

    Serial.println("SAS LINK: ONLINE");
    return true;
  }

  sasOnline = false;
  v5State = V5RuntimeState::OFFLINE;
  return false;
}

bool startupConnectWithRetry() {
  for (uint8_t attempt = 1; attempt <= BOOT_FAST_RETRIES; attempt++) {
    reconnectAttempt = attempt;

    lcdLine(0, "WAIT FOR S6000");
    lcdLine(1, String("TRY ") + attempt + "/" + BOOT_FAST_RETRIES);

    Serial.print("BOOT SAS attempt ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(BOOT_FAST_RETRIES);

    if (tryConnectOnce(false)) {
      String srcText = cachedIdentityMatched ? String("EEPROM+LIVE") : String("LIVE DETECT");
      showMessage(String("SAS ") + sasCaps.versionText + " ONLINE",
                  srcText,
                  1000);
      return true;
    }

    delay(BOOT_FAST_RETRY_DELAY_MS);
  }

  sasOnline = false;
  v5State = V5RuntimeState::OFFLINE;
  nextReconnectMs = millis() + BACKGROUND_RETRY_MS;

  Serial.println("BOOT SAS: S6000 not ready. Switching to endless background retry.");
  showMessage("S6000 NOT READY", "AUTO RETRY 5SEC", 1000);
  return false;
}

void serviceBackgroundReconnect() {
  if (sasOnline) return;

  const uint32_t now = millis();
  if ((int32_t)(now - nextReconnectMs) < 0) return;

  nextReconnectMs = now + BACKGROUND_RETRY_MS;
  reconnectAttempt++;

  lcdLine(0, "S6000 OFFLINE");
  lcdLine(1, String("RETRY ") + reconnectAttempt);

  Serial.print("BACKGROUND SAS retry #");
  Serial.println(reconnectAttempt);

  if (tryConnectOnce(false)) {
    String srcText = cachedIdentityMatched ? String("CACHE VERIFIED") : String("NEW PROFILE");
    showMessage(String("SAS ") + sasCaps.versionText + " ONLINE",
                srcText,
                1000);
    showIdle();
  }
}

void serviceLinkHeartbeat() {
  if (!sasOnline) return;

  // Do not inject a heartbeat while a time-critical priority event is queued
  // or while ticket/system-validation handling is active.
  if (v5EventCount > 0 ||
      v5State == V5RuntimeState::TICKET_IN_ESCROW ||
      v5State == V5RuntimeState::TICKET_STATUS ||
      v5State == V5RuntimeState::SYSTEM_VALIDATION_PENDING) {
    return;
  }

  const uint32_t now = millis();
  if ((int32_t)(now - nextHeartbeatMs) < 0) return;

  nextHeartbeatMs = now + LINK_HEARTBEAT_MS;

  uint64_t credits = 0;
  const bool ok = readCurrentCredits1A(credits, false);

  if (ok) {
    heartbeatFailCount = 0;
    return;
  }

  heartbeatFailCount++;
  Serial.print("SAS heartbeat failure ");
  Serial.print(heartbeatFailCount);
  Serial.print("/");
  Serial.println(LINK_HEARTBEAT_FAIL_LIMIT);

  if (heartbeatFailCount >= LINK_HEARTBEAT_FAIL_LIMIT) {
    Serial.println("SAS LINK: marked OFFLINE. Re-entering automatic discovery.");
    sasOnline = false;
    v5State = V5RuntimeState::OFFLINE;
    clearV5EventQueue();
    cachedIdentityMatched = false;
    heartbeatFailCount = 0;
    nextReconnectMs = millis() + 500UL;
    showMessage("SAS LINK LOST", "AUTO RECONNECT", 700);
    showIdle();
  }
}

void stabilityTest1A() {
  showMessage("LP1A TEST", "GP+1A 20 CYC", 300);

  const uint32_t okStart = lpOk;
  const uint32_t crcStart = lpCrcFail;
  const uint32_t timeoutStart = lpTimeout;
  const uint32_t fmtStart = lpFormatFail;

  Serial.println();
  Serial.println("=== LP1A 20-CYCLE STABILITY TEST V6.5.9 ===");
  Serial.println("Pattern: GP warmup -> LP1A -> GP recovery, repeated 20 times.");

  // Verified on the real S6000: this cadence produced 20/20 PASS,
  // CRC=0, timeout=0, format=0.
  warmGeneralPoll(1000UL, true);

  uint8_t cycleGood = 0;

  for (uint8_t i = 0; i < STABILITY_TEST_TOTAL; i++) {
    warmGeneralPoll(250UL, false);

    uint64_t credits = 0;
    const bool ok = readCurrentCredits1A(credits, false);

    if (ok) cycleGood++;

    Serial.print("1A test ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(STABILITY_TEST_TOTAL);
    Serial.print(" = ");
    Serial.print(ok ? "PASS" : "FAIL");

    if (ok) {
      Serial.print(" CR=");
      Serial.print(u64ToString(credits));
    }
    Serial.println();

    lcdLine(0, String("1A TEST ") + (i + 1) + "/20");
    lcdLine(1, String("OK:") + cycleGood + " TO:" + (lpTimeout - timeoutStart));

    warmGeneralPoll(150UL, false);
  }

  const uint32_t good = lpOk - okStart;
  const uint32_t crcBad = lpCrcFail - crcStart;
  const uint32_t toBad = lpTimeout - timeoutStart;
  const uint32_t fmtBad = lpFormatFail - fmtStart;

  Serial.println("--- LP1A STABILITY RESULT ---");
  Serial.print("Cycle PASS     : "); Serial.print(cycleGood); Serial.print("/"); Serial.println(STABILITY_TEST_TOTAL);
  Serial.print("LP OK delta    : "); Serial.println(good);
  Serial.print("CRC fail delta : "); Serial.println(crcBad);
  Serial.print("Timeout delta  : "); Serial.println(toBad);
  Serial.print("Format delta   : "); Serial.println(fmtBad);

  showMessage(
    String("1A OK:") + cycleGood + "/20",
    String("C:") + crcBad + " T:" + toBad + " F:" + fmtBad,
    2200
  );

  const bool perfectPass =
    cycleGood == STABILITY_TEST_TOTAL &&
    good >= STABILITY_PASS_REQUIRED &&
    crcBad == 0 &&
    toBad == 0 &&
    fmtBad == 0 &&
    sasOnline &&
    sasCaps.versionKnown;

  if (perfectPass) {
    const bool saved = saveVerifiedProfileToEeprom(
      (uint16_t)cycleGood,
      STABILITY_TEST_TOTAL
    );
    const bool dbSaved =
      saved &&
      v6SaveCurrentProfileDb((uint16_t)cycleGood, STABILITY_TEST_TOTAL);

    if (saved && dbSaved) {
      Serial.println("*** 20/20 PASS: VERIFIED PROFILE SAVED ***");
      showMessage("20/20 PASS", "EEPROM+DB SAVED", 1500);
    } else {
      Serial.println("*** 20/20 PASS but EEPROM/DB save failed ***");
      showMessage("20/20 PASS", "EEPROM SAVE ERR", 1500);
    }
  } else {
    Serial.println("*** STABILITY TEST FAIL: profile NOT marked verified ***");
    showMessage(
      String("TEST FAIL ") + cycleGood + "/20",
      String("C:") + crcBad + " T:" + toBad + " F:" + fmtBad,
      1600
    );
  }

  // Leave the SAS link in a healthy polling state.
  warmGeneralPoll(500UL, false);
}

void showCounters() {
  lcd.clear();
  lcdLine(0, String("GP:") + gpResponses + " EV:" + gpEvents);
  lcdLine(1, String("LP:") + lpOk + " TO:" + lpTimeout);

  Serial.println("--- SAS COUNTERS ---");
  Serial.print("GP sent       : "); Serial.println(gpSent);
  Serial.print("GP responses  : "); Serial.println(gpResponses);
  Serial.print("GP events     : "); Serial.println(gpEvents);
  Serial.print("LP OK         : "); Serial.println(lpOk);
  Serial.print("LP timeout    : "); Serial.println(lpTimeout);
  Serial.print("LP CRC fail   : "); Serial.println(lpCrcFail);
  Serial.print("LP format fail: "); Serial.println(lpFormatFail);
  Serial.print("UART errors   : "); Serial.println(sasUart.errorCount());
  Serial.print("RX overflow   : "); Serial.println(sasUart.overflowCount());
  Serial.print("Last event    : 0x"); Serial.println(lastEvent, HEX);
  Serial.print("V6 event queue: "); Serial.println(v5EventCount);
  Serial.print("V6 evt dropped: "); Serial.println(v5EventDropped);
  Serial.print("V6 state      : "); Serial.println(v5StateName(v5State));
  Serial.print("SAS address   : "); Serial.println(sasAddress);
  Serial.print("Boot selftest : "); Serial.println(v6BootSelfTestPassed ? "PASS" : "FAIL");
  Serial.print("Health ready  : "); Serial.println(v6HealthReady() ? "YES" : "NO");
  Serial.print("Recovery req  : "); Serial.println(v6RecoveryRequired ? "YES" : "NO");
  if (v6LastAft.valid) { Serial.print("Last AFT 72   : 0x"); Serial.print(v6LastAft.transferStatus, HEX); Serial.print(" "); Serial.println(v6AftStatusKo(v6LastAft.transferStatus)); }
  if (ticketIn.valid) { Serial.print("Last TKT 71   : 0x"); Serial.print(ticketIn.machineStatus, HEX); Serial.print(" "); Serial.println(v6TicketStatusKo(ticketIn.machineStatus)); }

  delay(1600);
}



bool bootRealCommTest();

// ============================================================================
// V6 USB SERIAL DIAGNOSTIC CONSOLE (READ-ONLY / OBSERVER COMMANDS)
// Commands: help, status, meters, journal, profiles, scan, realtest, aftfe, selftest, simtest, jtest, jclear, 94gate
// ============================================================================
String v6UsbCommandLine;

void v6PrintJournal() {
  Serial.println("--- V6 RAM TRANSACTION ---");
  if (!v6LatestJournalValid) { Serial.println("EMPTY (nothing persisted across reset)"); return; }
  Serial.print("Generation : "); Serial.println(v6LatestJournal.generation);
  Serial.print("Type       : "); Serial.println(v6LatestJournal.type);
  Serial.print("State      : "); Serial.println(v6LatestJournal.state);
  Serial.print("SAS addr   : "); Serial.println(v6LatestJournal.address);
  Serial.print("SAS status : 0x"); Serial.println(v6LatestJournal.sasStatus, HEX);
  Serial.print("Amount CR  : "); Serial.println(v6LatestJournal.amountCredits);
  Serial.print("Amount MU  : "); Serial.println(u64ToString(v6LatestJournal.amountMinorUnits));
  Serial.print("Recovery   : "); Serial.println(v6RecoveryRequired ? "YES" : "NO");
}

void v6PrintProfiles() {
  Serial.println("--- V6 PROFILE DB ---");
  for (int i = 0; i < V6_PROFILE_DB_SLOTS; i++) {
    if (!v6ProfileDbValid[i]) continue;
    Serial.print('#'); Serial.print(i);
    Serial.print(" gen="); Serial.print(v6ProfileDb[i].generation);
    Serial.print(" addr="); Serial.print(v6ProfileDb[i].address);
    Serial.print(" SAS="); Serial.print(v6FixedString(v6ProfileDb[i].versionText, sizeof(v6ProfileDb[i].versionText)));
    Serial.print(" serial="); Serial.print(v6FixedString(v6ProfileDb[i].machineSerial, sizeof(v6ProfileDb[i].machineSerial)));
    Serial.print(" pass="); Serial.print(v6ProfileDb[i].stabilityGood); Serial.print('/'); Serial.println(v6ProfileDb[i].stabilityTotal);
  }
}

bool v6RunProtocolSimulator() {
#if V6_ENABLE_PROTOCOL_SIMULATOR
  uint8_t pass = 0, total = 0;
  auto t = [&](bool ok, const char *name) {
    total++; if (ok) pass++;
    Serial.print(ok ? "PASS  " : "FAIL  "); Serial.println(name);
  };
  static const uint8_t p72[] = {0x01,0x72,0x02,0xFE,0x00};
  static const uint8_t p71[] = {0x01,0x71,0x01,0xFF};
  static const uint8_t p2f[] = {0x01,0x2F,0x07,0x00,0x00,0x0C,0x15,0x16,0x17,0x18};
  t(sasCrc16(p72, sizeof(p72)) == 0x3BD7, "LP72 FE CRC vector");
  t(sasCrc16(p71, sizeof(p71)) == 0xD01F, "LP71 FF CRC vector");
  t(sasCrc16(p2f, sizeof(p2f)) == 0x1CE2, "LP2F meter CRC vector");
  t(v6MeterAdvanced(1000, 1001), "meter increment");
  t(!v6MeterAdvanced(1000, 1000), "unchanged meter rejected");
  t(v6MeterAdvanced(99999998ULL, 3ULL), "8-digit meter rollover");
  t(String(v6TicketStatusKo(0x84)).indexOf("Credit Limit") >= 0, "71:84 decoder");
  t(String(v6AftStatusKo(0x84)).indexOf("한도") >= 0, "72:84 decoder");
  t(v6AftReceiptTerminal(0xFF), "AFT receipt FF terminal");
  t(!v6AftReceiptTerminal(0x40), "AFT receipt 40 pending");
  t(v6AftTransferRejected(0x87), "AFT 87 clear rejection");
  t(v6AftTransferRejected(0x81), "AFT 81 duplicate ID rejection");
  t(!v6AftTransferRejected(0x40), "AFT pending is not rejection");
  t(!v6AftTransferRejected(0x00), "AFT success is not rejection");
  t(v6ObservedJournalWriteBlocked(true, (uint8_t)V6JournalState::PREPARED),
    "active PREPARED blocks observer overwrite");
  t(v6ObservedJournalWriteBlocked(true, (uint8_t)V6JournalState::RECOVERY_REQUIRED),
    "active RECOVERY blocks observer overwrite");
  t(!v6ObservedJournalWriteBlocked(true, (uint8_t)V6JournalState::DONE),
    "terminal DONE allows next observer journal");
  V6JournalRecord obsAft = {};
  obsAft.type = (uint8_t)V6JournalType::AFT;
  t(v6ObserverOnlyAftJournal(obsAft), "observer-only AFT journal signature");
  obsAft.amountCredits = 1;
  t(!v6ObserverOnlyAftJournal(obsAft), "value AFT is not observer-only");
  uint32_t autoCard = 0;
  t(v6SplitReconcileTarget(254, 250, autoCard) && autoCard == 4,
    "split auto reconcile 254-250 => card 4");
  t(v6SplitReconcileTarget(100, 30, autoCard) && autoCard == 70,
    "split exact plan 100-30 => card 70");
  t(v6SplitReconcileTarget(100, 100, autoCard) && autoCard == 0,
    "split unchanged machine => card 0");
  t(!v6SplitReconcileTarget(100, 101, autoCard),
    "split invalid machine>total rejected");
  V6JournalRecord splitRec = {};
  splitRec.type = (uint8_t)V6JournalType::RFID_SPLIT_CASHOUT;
  splitRec.state = (uint8_t)V6JournalState::RECOVERY_REQUIRED;
  splitRec.amountCredits = 254;
  splitRec.afterMeters.currentCredits = 250;
  t(v6SplitRecoveryStage(splitRec) == 0,
    "legacy split journal has no stage marker");
  v6SetSplitRecoveryStage(splitRec, V6_SPLIT_STAGE_AFT);
  t(v6SplitRecoveryStage(splitRec) == V6_SPLIT_STAGE_AFT,
    "AFT recovery stage marker roundtrip");
  v6SetSplitRecoveryStage(splitRec, V6_SPLIT_STAGE_RESTORE);
  t(v6SplitRecoveryStage(splitRec) == V6_SPLIT_STAGE_RESTORE,
    "restore recovery stage marker roundtrip");
  v6SetSplitRecoveryStage(splitRec, V6_SPLIT_STAGE_LEGACY);
  t(v6SplitRecoveryStage(splitRec) == V6_SPLIT_STAGE_LEGACY,
    "legacy adopted stage marker roundtrip");
  t(decodeAnalogKey(1000) == 5,
    "key 5 nominal ADC decodes as 5");
  t(decodeAnalogKey(1021) == 5,
    "key 5 high-edge ADC 1021 still decodes as 5");
  t(decodeAnalogKey(1022) < 0,
    "released keypad threshold 1022 is not key 5");
  t(decodeAnalogKey(500) == 0,
    "key 0 nominal ADC decodes as 0");
  t(decodeAnalogKey(235) == 11,
    "single # nominal ADC decodes as #");
  t(decodeAnalogKey(430) == 12,
    "single * nominal ADC decodes as *");
  Serial.print("V6 SIM RESULT "); Serial.print(pass); Serial.print('/'); Serial.println(total);
  return pass == total;
#else
  Serial.println("V6 protocol simulator disabled in config.");
  return false;
#endif
}

bool v6CreateRecoveryJournalTest() {
#if V6_ENABLE_PROTOCOL_SIMULATOR
  if (v6RecoveryRequired) {
    Serial.println("JTEST BLOCKED: an unfinished real/test journal already exists.");
    return false;
  }
  V6JournalRecord r = {};
  r.type = (uint8_t)V6JournalType::RFID_ATOMIC;
  r.state = (uint8_t)V6JournalState::PREPARED;
  r.sasStatus = 0xFE; // local test marker only
  r.balanceBefore = 12345;
  r.balanceAfter = 12345; // no value delta and no card write
  r.beforeMeters = v6StoreMeters(v6BeforeMeters);
  if (!v6WriteJournal(r)) return false;
  Serial.println("JTEST CREATED: synthetic PREPARED RAM transaction, no SAS value command, no RFID write.");
  Serial.println("While powered: Recovery must be active. After reboot: it must be forgotten/CLEAR.");
  return true;
#else
  return false;
#endif
}

bool v6ClearRecoveryJournalTest() {
  if (!v6LatestJournalValid || v6LatestJournal.type != (uint8_t)V6JournalType::RFID_ATOMIC ||
      v6LatestJournal.sasStatus != 0xFE) {
    Serial.println("JCLEAR BLOCKED: latest journal is not the synthetic JTEST record.");
    return false;
  }
  return v6UpdateJournal(V6JournalState::FAILED, 0xFE);
}

void v6PrintUsage() {
  Serial.println();
  Serial.println("=== V6.5.9 RAM TX + SAFE UI QUICK USAGE ===");
  Serial.println("[NO RFID CARD / NORMAL]");
  Serial.println("  hold # for 2 sec = ENTER TEST MODE");
  Serial.println("  normal no-card keys do NOT execute diagnostics");
  Serial.println();
  Serial.println("[TEST MODE - NO RFID CARD ONLY]");
  Serial.println("  hold # for 2 sec = EXIT to normal screen");
  Serial.println("  1 = Auto status");
  Serial.println("  2 = LP1A current credits");
  Serial.println("  3 = A4/AFT limits summary");
  Serial.println("  4 = A0 feature information");
  Serial.println("  5 = LP74 AFT status");
  Serial.println("  6 = LP1A 20-cycle GP-aware stability test");
  Serial.println("  7 = reconnect at fixed SAS address 1");
  Serial.println("  8 = LP54 SAS version / machine serial");
  Serial.println("  9 = event state");
  Serial.println("  0 = LP1F denomination / game info");
  Serial.println("  * = communication counters");
  Serial.println("  # = clear observer event display only");
  Serial.println();
  Serial.println("[RFID CARD ACTIVE]");
  Serial.println("  0..9 = enter credit amount");
  Serial.println("  * once  = ARM deposit confirmation; NO money moves");
  Serial.println("  * again = EXECUTE RFID -> S6000 deposit");
  Serial.println("  # once  = ARM cashout confirmation; NO money moves");
  Serial.println("  # again = EXECUTE entered amount -> RFID; remainder stays for hopper");
  Serial.println("  hold 0 for 1 sec = CLEAR amount + CANCEL confirmation");
  Serial.println();
  Serial.println("Example: machine CREDIT=100, enter 70 then # then # again");
  Serial.println("  full AFT 100 -> machine 0");
  Serial.println("  proven 0x8A restores 30 -> machine CREDIT 30");
  Serial.println("  RFID gets +70 only after both machine steps verify");
  Serial.println("  then press physical S6000 CASH OUT for hopper 30");
  Serial.println();
  Serial.println("HOPPER MACHINE OPTION:");
  Serial.println("  Option 61=0001 Hopper Only, or 0003 Printer+Hopper.");
  Serial.println("  Option 61=0002 is Printer Only and will not hopper-pay.");
  Serial.println();
  Serial.println("[USB SERIAL MONITOR: 115200 + Newline]");
  Serial.println("  usage     = this screen");
  Serial.println("  help      = command list");
  Serial.println("  status    = compact system status");
  Serial.println("  keyadc    = one-shot A0: key - median [min..max]");
  Serial.println("  keylive   = toggle continuous 100ms A0 Serial monitor");
  Serial.println("  realtest  = proven GP 2s + 01 1A raw link test");
  Serial.println("  meters    = selected LP2F meters");
  Serial.println("  profiles  = stored machine profiles");
  Serial.println("  journal   = current RAM transaction state (not EEPROM)");
  Serial.println("  aftcheck  = READ-ONLY AFT readiness");
  Serial.println("  aftcycle  = FF-sync/ack current AFT cycle; NO money command");
  Serial.println("  aftoutall = REAL FULL AFT all credits -> RFID");
  Serial.println("  split N   = REAL FINAL split: N credits to RFID, remainder for hopper");
  Serial.println("  selftest / simtest / jtest / jclear / 94gate = diagnostics");
  Serial.println("================================");
  Serial.println();
}

void v6PrintCompactStatus() {
  Serial.println("--- V6 STATUS ---");
  Serial.print("Build      : "); Serial.println(V6_BUILD_ID);
  Serial.print("SAS        : "); Serial.println(sasOnline ? "ONLINE" : "OFFLINE");
  Serial.print("Address    : "); Serial.println(sasAddress);
  Serial.print("Version    : "); Serial.println(sasCaps.versionText);
  Serial.print("Serial     : "); Serial.println(sasCaps.machineSerial);
  Serial.print("Credits    : "); Serial.println(u64ToString(v6LastCurrentCredits));
  Serial.print("State      : "); Serial.println(v5StateName(v5State));
  Serial.print("Selftest   : "); Serial.println(v6BootSelfTestPassed ? "PASS" : "FAIL");
  Serial.print("Health     : "); Serial.println(v6HealthReady() ? "READY" : "NOT READY");
  Serial.print("Recovery   : "); Serial.println(v6RecoveryRequired ? "RAM ACTIVE" : "CLEAR");
  Serial.print("KeypadMode : "); Serial.println(keypadAdcCalibrationMode ? "ADC12-SERIAL" : (keypadTestMode ? "TEST" : "NORMAL"));
  if (v6LastAft.valid) {
    Serial.print("Last AFT   : 0x");
    if (v6LastAft.transferStatus < 0x10) Serial.print('0');
    Serial.print(v6LastAft.transferStatus, HEX);
    Serial.print(" receipt=0x");
    if (v6LastAft.receiptStatus < 0x10) Serial.print('0');
    Serial.println(v6LastAft.receiptStatus, HEX);
  }
  Serial.println("Active SAS : 2-PRESS *IN/#OUT / HOLD0 CLEAR / HOLD# TEST / RAM-TX");
}

void v6ExecuteUsbCommand(String cmd) {
  cmd.trim(); cmd.toLowerCase();
  if (cmd.length() == 0) return;
  if (cmd == "help") {
    Serial.println("usage help status adctest adcraw adcstable adcexit keyadc keylive meters journal profiles scan realtest aftfe aftcheck aftcycle aftoutall split N selftest simtest jtest jclear 94gate");
  } else if (cmd == "usage") {
    v6PrintUsage();
  } else if (cmd == "status") {
    v6PrintCompactStatus();
  } else if (cmd == "adctest") {
    enterKeypadAdcCalibrationMode();
  } else if (cmd == "adcraw") {
    setKeypadAdcRawStream(true);
  } else if (cmd == "adcstable") {
    setKeypadAdcRawStream(false);
  } else if (cmd == "adcexit") {
    exitKeypadAdcCalibrationMode();
  } else if (cmd == "keyadc") {
    printKeypadAdcDiagnostic();
  } else if (cmd == "keylive") {
    if (keypadAdcCalibrationMode) { Serial.println("KEYPAD ADC LIVE: blocked during adctest; use adcexit first."); return; }
    keypadSerialAdcLive = !keypadSerialAdcLive;
    keypadSerialAdcLastMs = 0;
    Serial.print("KEYPAD ADC LIVE: ");
    Serial.println(keypadSerialAdcLive ? "ON" : "OFF");
    Serial.println("Format: key - median [min..max]; unknown=?; released=UP");
  } else if (cmd == "meters") {
    V6MeterSnapshot m; v6ReadSelectedMeters2F(m, false, true);
  } else if (cmd == "journal") {
    v6PrintJournal();
  } else if (cmd == "profiles") {
    v6PrintProfiles();
  } else if (cmd == "scan") {
    Serial.println("Address scan disabled: proven S6000 address is fixed at 1.");
    sasAddress = 0x01;
    tryConnectOnce(true);
  } else if (cmd == "realtest") {
    Serial.println("--- REAL_COMM_TEST INSIDE V6.5.9 ---");
    const bool ok = bootRealCommTest();
    Serial.print("REALTEST RESULT: ");
    Serial.println(ok ? "PASS" : "FAIL");
  } else if (cmd == "aftfe") {
    if (!v6InterrogateAft72(0xFE, false, true)) Serial.println("AFT FE interrogation failed/not supported.");
  } else if (cmd == "aftcheck") {
    v6PrintAftCashoutCheck();
  } else if (cmd == "aftcycle") {
    const bool ok = v6SyncAftCycleBeforeValueCommand("USB-CHECK", 3500UL, false);
    Serial.print("AFT CYCLE RESULT: ");
    Serial.println(ok ? "READY/ACKNOWLEDGED" : "NOT READY");
  } else if (cmd == "aftout1") {
    executeAftCashoutTest1Credit();
  } else if (cmd == "aftoutall") {
    executeAftCashoutAllCredits();
  } else if (cmd.startsWith("split ")) {
    const uint32_t amount = (uint32_t)cmd.substring(6).toInt();
    executeSplitCashoutToRfid(amount);
  } else if (cmd == "selftest") {
    v6BootProtocolSelfTest();
  } else if (cmd == "simtest") {
    v6RunProtocolSimulator();
  } else if (cmd == "jtest") {
    v6CreateRecoveryJournalTest();
  } else if (cmd == "jclear") {
    if (v6ClearRecoveryJournalTest()) Serial.println("JTEST RAM transaction cleared as FAILED/terminal.");
  } else if (cmd == "94gate") {
    String reason; const bool ok = v6Handpay94GateEligible(reason);
    Serial.print("LP94 eligibility: "); Serial.print(ok ? "ELIGIBLE" : "BLOCKED"); Serial.print(" / "); Serial.println(reason);
    Serial.println("No LP94/A8 command is transmitted by this build.");
  } else {
    Serial.println("Unknown command. Type: help");
  }
}

void v6ServiceUsbConsole() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      v6ExecuteUsbCommand(v6UsbCommandLine);
      v6UsbCommandLine = "";
    } else if (v6UsbCommandLine.length() < 48) {
      v6UsbCommandLine += c;
    }
  }
}

// ============================================================================
// V6 READ-ONLY WIFI DIAGNOSTICS (NO COMMAND/TRIGGER ENDPOINTS)
// ============================================================================
#if V6_ENABLE_WIFI_DIAGNOSTICS
void v6ServiceWiFiConnection() {
  if (strlen(V6_WIFI_SSID) == 0) return;
  if (WiFi.status() == WL_CONNECTED) {
    if (!v6WebStarted) { v6WebServer.begin(); v6WebStarted = true; Serial.println("V6 WEB: server started on port 80"); }
    return;
  }
  const uint32_t now = millis();
  if ((uint32_t)(now - v6LastWebAttemptMs) < V6_WIFI_RETRY_MS) return;
  v6LastWebAttemptMs = now;
  v6WebStarted = false;
  Serial.print("V6 WIFI: connecting to "); Serial.println(V6_WIFI_SSID);
  WiFi.begin(V6_WIFI_SSID, V6_WIFI_PASS);
}

String v6StatusJson() {
  String j = "{";
  j += "\"build\":\"" V6_BUILD_ID "\",";
  j += "\"sas_online\":" + String(sasOnline ? "true" : "false") + ",";
  j += "\"sas_address\":" + String(sasAddress) + ",";
  j += "\"sas_version\":\"" + sasCaps.versionText + "\",";
  j += "\"machine_serial\":\"" + sasCaps.machineSerial + "\",";
  j += "\"current_state\":\"" + String(v5StateName(v5State)) + "\",";
  j += "\"current_credits\":" + u64ToString(v6LastCurrentCredits) + ",";
  j += "\"cashout_limit_credits\":" + u64ToString(sasCaps.cashOutLimitCredits) + ",";
  j += "\"aft_limit_minor\":" + u64ToString(sasCaps.aftTransferLimitCents) + ",";
  j += "\"gp_events\":" + String(gpEvents) + ",";
  j += "\"lp_ok\":" + String(lpOk) + ",";
  j += "\"lp_timeout\":" + String(lpTimeout) + ",";
  j += "\"lp_crc_fail\":" + String(lpCrcFail) + ",";
  j += "\"uart_errors\":" + String(sasUart.errorCount()) + ",";
  j += "\"recovery_required\":" + String(v6RecoveryRequired ? "true" : "false") + ",";
  j += "\"health_ready\":" + String(v6HealthReady() ? "true" : "false") + ",";
  j += "\"active_value_commands\":false";
  j += "}";
  return j;
}

void v6ServiceWeb() {
  if (!v6WebStarted || WiFi.status() != WL_CONNECTED) return;
  WiFiClient client = v6WebServer.available();
  if (!client) return;
  String firstLine;
  uint32_t start = millis();
  while (client.connected() && (uint32_t)(millis() - start) < 120UL) {
    if (!client.available()) continue;
    firstLine = client.readStringUntil('\n');
    break;
  }
  const bool json = firstLine.indexOf("GET /status.json") >= 0;
  client.println("HTTP/1.1 200 OK");
  client.println(json ? "Content-Type: application/json; charset=utf-8" : "Content-Type: text/html; charset=utf-8");
  client.println("Connection: close");
  client.println("Cache-Control: no-store");
  client.println();
  if (json) {
    client.print(v6StatusJson());
  } else {
    client.println("<!doctype html><meta charset='utf-8'><meta name='viewport' content='width=device-width'>");
    client.println("<title>S6000 V6 Diagnostics</title><style>body{font-family:monospace;max-width:900px;margin:30px auto;padding:0 12px}b{display:inline-block;width:230px}</style>");
    client.println("<h2>Bally S6000 / UNO R4 V6 - READ ONLY</h2>");
    client.print("<p><b>Build</b>"); client.print(V6_BUILD_ID); client.println("</p>");
    client.print("<p><b>SAS</b>"); client.print(sasOnline ? "ONLINE" : "OFFLINE"); client.print(" addr="); client.print(sasAddress); client.println("</p>");
    client.print("<p><b>Version / Serial</b>"); client.print(sasCaps.versionText); client.print(" / "); client.print(sasCaps.machineSerial); client.println("</p>");
    client.print("<p><b>State</b>"); client.print(v5StateName(v5State)); client.println("</p>");
    client.print("<p><b>Current credits</b>"); client.print(u64ToString(v6LastCurrentCredits)); client.println("</p>");
    client.print("<p><b>A4 cashout limit</b>"); client.print(u64ToString(sasCaps.cashOutLimitCredits)); client.println(" credits</p>");
    client.print("<p><b>AFT transfer limit</b>"); client.print(u64ToString(sasCaps.aftTransferLimitCents)); client.println(" minor units</p>");
    client.print("<p><b>Last Ticket 71</b>0x"); client.print(ticketIn.machineStatus, HEX); client.print(" - "); client.print(v6TicketStatusKo(ticketIn.machineStatus)); client.println("</p>");
    client.print("<p><b>Last AFT 72</b>0x"); client.print(v6LastAft.transferStatus, HEX); client.print(" - "); client.print(v6AftStatusKo(v6LastAft.transferStatus)); client.println("</p>");
    client.print("<p><b>RAM transaction recovery</b>"); client.print(v6RecoveryRequired ? "REQUIRED" : "CLEAR"); client.println("</p>");
    client.print("<p><b>Health</b>"); client.print(v6HealthReady() ? "READY" : "NOT READY"); client.println("</p>");
    client.println("<p><b>Active value commands</b>DISABLED IN THIS BUILD</p>");
    client.println("<p><a href='/status.json'>status.json</a></p>");
  }
  delay(1);
  client.stop();
}
#else
void v6ServiceWiFiConnection() {}
void v6ServiceWeb() {}
#endif

bool bootRealCommTest() {
  Serial.println();
  Serial.println("BOOT REAL_COMM_TEST baseline");
  Serial.println("Warming General Poll for 2 seconds...");

  warmGeneralPoll(2000UL, true);
  sasUart.clearRx();

  uint8_t cmd[2] = {sasAddress, 0x1A};
  uint16_t words[2];
  words[0] = (uint16_t)(0xFF00U | cmd[0]);
  words[1] = (uint16_t)(0xFC00U | cmd[1]);

  Serial.println("TX -> SAS Credit Meter request: 01 1A");
  Serial1.write((uint8_t *)words, sizeof(words));

  const uint32_t startMs = millis();
  bool sawAddr = false;
  bool sawHeader = false;
  int count = 0;

  Serial.print("RAW RX -> ");
  while ((uint32_t)(millis() - startMs) < 1500UL) {
    if (Serial1.available() > 0) {
      const int r = Serial1.read();
      if (r < 0) continue;
      const uint8_t b = (uint8_t)r;

      if (b < 0x10) Serial.print('0');
      Serial.print(b, HEX);
      Serial.print(' ');
      count++;

      if (!sawAddr) {
        if (b == sasAddress) sawAddr = true;
      } else {
        if (b == 0x1A) sawHeader = true;
        sawAddr = (b == sasAddress);
      }
    }
  }
  Serial.println();

  if (sawHeader) {
    Serial.println("*** PASS: Found SAS response header 01 1A ***");
    Serial.println("*** 9-bit TX/RX communication is working. ***");
    return true;
  }

  Serial.println("*** FAIL: Did not find response header 01 1A ***");
  if (count == 0) Serial.println("No RX at all.");
  else Serial.println("RX existed, but 01 1A was not found.");
  return false;
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
void setup() {
  // EXACT beginning of the working REAL_COMM_TEST.
  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("====================================================");
  Serial.println(" BALLY S6000 UNO R4 MINIMA V6.5.14 RAM AUTO-CLEAR");
  Serial.println("====================================================");

  if (!sasUart.begin(SAS_BAUD)) {
    Serial.println("ERROR: Serial1 19200 9N1 initialization failed.");
    while (true) delay(1000);
  }

  Serial.println("Serial1 = 19200 9N1 / SAS address = 01");
  const bool rawLinkPass = bootRealCommTest();
  Serial.print("BOOT RAW LINK = ");
  Serial.println(rawLinkPass ? "PASS" : "FAIL");

  // Known-good legacy peripheral order. Do NOT call Wire.begin() separately.
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("S6000 V6.5.14");
  lcd.setCursor(0, 1);
  lcd.print(rawLinkPass ? "9BIT LINK PASS" : "9BIT LINK FAIL");
  Serial.println("LCD INIT DONE: 0x27 16x2");

  setupRFID();
  // V6.5.14: normal keypad operation uses the measured 12-bit calibration.
  analogReadResolution(12);

  Serial.println("KEYPAD ADC: 12-bit calibrated from real hardware values");
  Serial.println("KEYS: #=931 1=1081 9=1598 *=1738 2=1817 0=2000 8=2224 7=2356 4=2677 3=3109 6=3385 5=4095 UP~15");
  Serial.println("LCD/RFID init complete (known-good legacy order)");

  // V6 storage / validation framework starts only after the proven live-link test.
  const bool cacheOK = loadProfileFromEeprom();
  v6LoadProfileDb();
  v6LoadJournal();

  if (cacheOK) {
    lcdLine(0, String("CACHE SAS ") + fixedToString(savedProfile.versionText, sizeof(savedProfile.versionText)));
    lcdLine(1, "VERIFY MACHINE");
  } else {
    lcdLine(0, "NO SAVED PROFILE");
    lcdLine(1, "LIVE DETECT");
  }
  delay(300);

  Serial.println();
  Serial.println("=== BALLY S6000 UNO R4 MINIMA V6.5.14 RAM TX AUTO-CLEAR + 12-BIT KEYPAD ===");
  Serial.println("Transport: exact REAL_COMM_TEST Serial1 19200 9N1 baseline");
  Serial.println("General Poll: FF80 ->20ms -> FF81 ->10ms");
  Serial.println("Type-R/S: one bulk Serial1.write((uint8_t*)words, bytes)");
  Serial.println("FINAL MONEY MODE: 2-press confirm; * deposit 0x8A; # split full AFT + 0x8A restore.");
  Serial.println("TRANSACTION STORAGE: RAM ONLY. Transaction/recovery EEPROM writes = 0.");
  Serial.println("RESET POLICY: forget in-flight TX; interrogate current S6000/AFT before any new value command.");
  Serial.println("KEYPAD TEST MODE: no card + hold # 2 sec; hold # 2 sec again to exit.");
  Serial.println("WIFI: DISABLED - UNO R4 MINIMA has no onboard WiFi.");
  Serial.println("HOPPER NOTE: machine Option 61 must be 0001 or 0003; keep Option 90=0000.");
  v6BootProtocolSelfTest();

  lastPollTxMs = 0;

  // If the raw 01/1A proof passed, normal V6 profile discovery should now use
  // the same proven transport. If it failed, retries still keep GP alive.
  startupConnectWithRetry();

  // V6.5.9 reset policy: do NOT recover an old transaction. Refresh only the
  // machine's current observable state so the next user action starts fresh.
  if (sasOnline) {
    uint64_t bootCredits = 0;
    if (readCurrentCredits1A(bootCredits, false)) {
      v6LastCurrentCredits = bootCredits;
      Serial.print("BOOT CURRENT CREDIT: "); Serial.println(u64ToString(bootCredits));
    } else {
      Serial.println("BOOT CURRENT CREDIT: unavailable; background link retry remains active.");
    }
    if (probeAftStatus74(false)) {
      Serial.print("BOOT AFT STATUS: 0x"); Serial.println(sasCaps.aftStatusFlags, HEX);
    } else {
      Serial.println("BOOT AFT STATUS: LP74 unavailable; next transaction will re-check before value command.");
    }
  }
  showIdle();
}
void loop() {
  // Keep the exact proven General Poll alive in BOTH online and offline states.
  sendGeneralPoll();

  serviceBackgroundReconnect();
  v6ServiceWiFiConnection();
  v6ServiceWeb();
  v6ServiceUsbConsole();

  // Dedicated Serial-entered ADC calibration mode.  Keep SAS polling/USB
  // alive, but suspend all normal keypad transaction/test handling.
  if (keypadAdcCalibrationMode) {
    serviceKeypadAdcCalibrationMode();
    return;
  }

  if (sasOnline) {
    serviceV5EventEngine();
    v6ServiceRecovery();
    serviceLinkHeartbeat();
  }

  serviceV5EventScreen();

  if (cardActive && moneyConfirmMode != MoneyConfirmMode::NONE &&
      (int32_t)(millis() - moneyConfirmUntilMs) >= 0) {
    cancelMoneyConfirm(false);
    showMessage("CONFIRM TIMEOUT", "NO MONEY MOVED", 450);
    showCard();
  }

  if (!cardActive) {
    if (detectNewCard()) {
      keypadTestMode = false;
      lastCardCheckMs = millis();
      Serial.println("KEYPAD TEST MODE: auto-exit because RFID card became active.");
      showCard();
    }
  } else if ((uint32_t)(millis() - lastCardCheckMs) >= 700UL) {
    lastCardCheckMs = millis();
    if (!checkCardStillReadable()) {
      closeCardSession();
      showIdle();
    }
  }

  // V6.5.9 TEST MODE continuously exposes the REAL A0 ladder value on USB Serial.
  // This runs even when decodeAnalogKey() returns NONE/UP, so a dead key can be calibrated.
  serviceKeypadSerialAdcMonitor();

  const int8_t key = readKeyEvent();
  if (key < 0) return;

  // Dedicated operator diagnostics gate.  A long # is honored ONLY with no
  // RFID card active.  It never arms or executes a money transaction.
  if (key == 14) {
    if (!cardActive) {
      keypadTestMode = !keypadTestMode;
      lastEvent = 0xFF;
      v5EventScreenUntilMs = 0;
      if (keypadTestMode) {
        Serial.println("KEYPAD TEST MODE: ENTER (# held 2 sec, no card).");
        Serial.println("ADC LIVE: key - median [min..max], every 100ms. Unknown key prints ? - ADC.");
        showMessage("TEST MODE", "0-9 *CNT #CLR", 350);
        showTestModeMenu();
      } else {
        Serial.println("KEYPAD TEST MODE: EXIT -> NORMAL CARD SCREEN.");
        showMessage("TEST MODE EXIT", "CARD READY", 300);
        showIdle();
      }
    } else {
      // Holding # with a card present is never treated as a cashout key.
      showMessage("TEST ONLY NO CARD", "HOLD 0=CLEAR", 500);
      showCard();
    }
    return;
  }

  if (!cardActive) {
    // Outside TEST MODE, no-card keypad presses are intentionally inert.
    // This prevents accidental diagnostics during normal customer operation.
    if (!keypadTestMode) {
      showIdle();
      return;
    }

    setColor(false, true, true);

    // Keys that require an active S6000 link are blocked while offline.
    if (!sasOnline && key != 1 && key != 7 && key != 9 && key != 12 && key != 11) {
      showMessage("S6000 OFFLINE", "WAIT AUTO RETRY", 700);
      showIdle();
      return;
    }

    switch (key) {
      case 1:
        if (sasOnline) showAutoStatus();
        else if (savedProfileValid) {
          showMessage(
            String("CACHE SAS ") + fixedToString(savedProfile.versionText, sizeof(savedProfile.versionText)),
            String("PASS ") + savedProfile.stabilityGood + "/" + savedProfile.stabilityTotal,
            1300
          );
        } else {
          showMessage("NO SAS / CACHE", "AUTO RETRY", 900);
        }
        break;

      case 2: diagnostic1A(); break;
      case 3: showLimitsSummary(); break;
      case 4: diagnosticA0(); break;
      case 5: diagnostic74(); break;
      case 6: stabilityTest1A(); break;

      case 7:
        sasAddress = 0x01;
        if (tryConnectOnce(true)) {
          showMessage("ADDR:1 OK", sasCaps.versionText, 900);
        } else {
          nextReconnectMs = millis() + 500UL;
          showMessage("ADDR1 DETECT FAIL", "AUTO RETRY", 800);
        }
        break;

      case 8: diagnostic54(); break;
      case 9: showV5EventState(); break;
      case 0: diagnostic1F(); break;
      case 12: showCounters(); break; // *

      case 11: // #
        lastEvent = 0xFF;
        clearV5EventQueue();

        // Display/observer clear only. No machine reset command is sent.
        if (v5State != V5RuntimeState::HANDPAY_PENDING &&
            v5State != V5RuntimeState::SYSTEM_VALIDATION_PENDING) {
          v5State = sasOnline ? V5RuntimeState::NORMAL
                              : V5RuntimeState::OFFLINE;
        }

        showMessage("EVENT CLEARED", "MONITOR READY", 600);
        break;

      default: break;
    }

    showIdle();
    return;
  }

  if (key >= 0 && key <= 9) {
    // Any digit cancels a pending confirmation before editing the amount.
    // No money has moved at the confirmation stage.
    cancelMoneyConfirm(false);
    const uint32_t next = inputValue * 10UL + (uint8_t)key;
    if (next <= MAX_INPUT_AMOUNT) {
      inputValue = next;
      showAmount();
      setColor(false, true, true);
    } else {
      showMessage("MAX INPUT", "99999", 700);
      showCard();
    }
  } else if (key == 13) { // hold 0 >=1 sec = CLEAR, NEVER a transaction
    cancelMoneyConfirm(true);
    showMessage("AMOUNT CLEARED", "NO MONEY MOVED", 500);
    showCard();
  } else if (key == 12) { // * short: arm/confirm RFID -> S6000 deposit
    if (inputValue == 0) {
      cancelMoneyConfirm(false);
      showMessage("ENTER AMOUNT", "HOLD 0=CLEAR", 650);
      showCard();
    } else if (moneyConfirmMode == MoneyConfirmMode::DEPOSIT &&
               moneyConfirmAmount == inputValue &&
               (int32_t)(moneyConfirmUntilMs - millis()) > 0) {
      cancelMoneyConfirm(false);
      executeCardDepositTransaction();
      delay(200);
      showCard();
    } else {
      // FIRST short press only arms confirmation. Zero SAS value commands here.
      armMoneyConfirm(MoneyConfirmMode::DEPOSIT);
    }
  } else if (key == 11) { // # short: arm/confirm desired split cashout
    if (inputValue == 0) {
      cancelMoneyConfirm(false);
      showMessage("ENTER AMOUNT", "HOLD 0=CLEAR", 650);
      showCard();
    } else if (moneyConfirmMode == MoneyConfirmMode::CASHOUT &&
               moneyConfirmAmount == inputValue &&
               (int32_t)(moneyConfirmUntilMs - millis()) > 0) {
      cancelMoneyConfirm(false);
      executeSplitCashoutToRfid(inputValue);
      delay(200);
      showCard();
    } else {
      // FIRST short press only arms confirmation. Zero SAS value commands here.
      armMoneyConfirm(MoneyConfirmMode::CASHOUT);
    }
  }
}
