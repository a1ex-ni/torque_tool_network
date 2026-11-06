// Tool unit: tare, MAD filtering, target cutoff and relays run locally;
// commands and status go over ESP-NOW. Boots IDLE (no auto-tare).

#include <Arduino.h>
#include "HX711.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_system.h>
#include <math.h>
#include <string.h>
#include <climits>
#include <algorithm>

// Unique per tool
const char TOOL_ID[] = "TOOL_01";

const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

// Active-LOW relays
const int RELAY_1_PIN = 1;
const int RELAY_2_PIN = 0;
// -1 = no direction relay: set reverse on the wrench by hand
const int REVERSE_RELAY_PIN = -1;

HX711 scale;

// From datasheets: 300 kN / (2^23 * 0.69936 / 3.90625). Matches dashboard.py
const double N_PER_COUNT = 0.19975;

const int MAX_WINDOW_SIZE = 1000;
const int WINDOW_SIZE = 10;  // ~1 s

// MAXIMUM lags least, so overshoots least
enum TargetDetection { DETECT_AVERAGE, DETECT_MINIMUM, DETECT_MAXIMUM };
const TargetDetection TARGET_DETECTION = DETECT_MAXIMUM;
// MAD needs 3 readings to reject a spike
const int MIN_DETECT_READINGS = 3;
long window[MAX_WINDOW_SIZE];
long scratch[MAX_WINDOW_SIZE];
const double MIN_SCALED_MAD = 2.0;   // floor: MAD of 0 would pass spikes
int windowWriteIndex = 0;
long totalReadingsTaken = 0;

const int TARE_WINDOW_SIZE = 10;
const long MAX_TARE_ERROR = 1500;    // counts
const double MAX_TARE_MAD = 500;     // counts
const unsigned long TARE_TIMEOUT = 20000;

// Matches max_over_target_pct in bolt_profiles.json
const float MAX_OVER_TARGET = 1.25;
const int MAX_FORCE_READINGS = 3;
const unsigned long MAX_RUN_MS = 180000;

const unsigned long LOOSEN_TIMEOUT_MS = 5000;    // short to avoid the wrench's overload cut-out
const unsigned long DIRECTION_SETTLE_MS = 500;

// Pulsing disabled, see loop()
const unsigned long PULSE_ON_MS = 250;
const unsigned long PULSE_OFF_MS = 3000;

const unsigned long STATUS_INTERVAL_MS = 250;
const unsigned long DISCOVERY_INTERVAL_MS = 1000;

const unsigned long DISPLAY_AVG_INTERVAL_MS = 1000;
const int MIN_DISPLAY_SAMPLES = 5;
long displayWindow[MAX_WINDOW_SIZE];
int displayReadingsTaken = 0;
long filteredAvgCounts = 0;
unsigned long lastDisplayAvgTime = 0;

enum ToolState {
  IDLE,
  TARE_FAILED,
  READY,
  MONITORING,
  TARGET_REACHED,
  STOPPED,
  LOOSENING
};

ToolState toolState = IDLE;
bool tareValid = false;          // STOP keeps it, RESET clears it
float targetN = 0;
long targetCounts = 0;
long maxCounts = 0;
int overMaxReadings = 0;
unsigned long runStartTime = 0;
unsigned long loosenStartTime = 0;
bool loosenTared = false;
ToolState stateAfterLoosen = IDLE;
long latestReading = 0;
long filteredMeasuredCounts = 0;
bool relaysOn = false;
bool reverseOn = false;
unsigned long phaseStartTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastDiscoveryTime = 0;

enum MessageType : uint8_t {
  MSG_DISCOVERY = 1,
  MSG_STATUS = 2,
  MSG_COMMAND = 3,
  MSG_ACK = 4,
  MSG_NACK = 5,
  MSG_DONE = 6     // TARE/LOOSEN finished
};

enum CommandType : uint8_t {
  CMD_SET_TARGET = 1,
  CMD_STOP = 2,
  CMD_RESET = 3,
  CMD_TARE = 4,
  CMD_LOOSEN = 5
};

// Must match WirelessPacket in tool_network_controller.ino byte for byte
struct __attribute__((packed)) WirelessPacket {
  uint8_t messageType;
  uint8_t command;
  char toolId[16];
  float value;
  int32_t rawCounts;
  int32_t filteredCounts;      // stop value compared with the target
  int32_t filteredAvgCounts;   // repeats filteredCounts (layout kept)
  uint8_t state;
  uint8_t relayOn;
};

const uint8_t BROADCAST_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// WiFi callback only queues; loop() runs commands (tare blocks)
const int COMMAND_QUEUE_SIZE = 8;
WirelessPacket commandQueue[COMMAND_QUEUE_SIZE];
volatile int commandQueueHead = 0;
volatile int commandQueueTail = 0;
portMUX_TYPE commandQueueMux = portMUX_INITIALIZER_UNLOCKED;

double calculateMedianWindow(long *data, long *sortedBuffer, int n) {
  memcpy(sortedBuffer, data, n * sizeof(long));
  if (n % 2 == 0) {
    std::nth_element(sortedBuffer, sortedBuffer + n / 2, sortedBuffer + n);
    long upperMid = sortedBuffer[n / 2];
    long lowerMid = *std::max_element(sortedBuffer, sortedBuffer + n / 2);
    return (lowerMid + upperMid) / 2.0;
  }

  std::nth_element(sortedBuffer, sortedBuffer + n / 2, sortedBuffer + n);
  return sortedBuffer[n / 2];
}

double calculateMADWindow(long *data, long *scratchBuf, int n, double median) {
  for (int i = 0; i < n; i++)
    scratchBuf[i] = labs((long)round(data[i] - median));

  std::nth_element(scratchBuf, scratchBuf + n / 2, scratchBuf + n);
  double mad = scratchBuf[n / 2];

  if (n % 2 == 0) {
    long lower = *std::max_element(scratchBuf, scratchBuf + n / 2);
    mad = (mad + lower) / 2.0;
  }

  return mad;
}

// Mean / max / min of readings passing the MAD filter
double filteredAvgOfWindow(long *data, int n, double threshold = 3.5) {
  double median = calculateMedianWindow(data, scratch, n);
  double mad = calculateMADWindow(data, scratch, n, median);
  double scaledMAD = std::max(mad * 1.4826, MIN_SCALED_MAD);

  double sum = 0;
  int count = 0;

  for (int i = 0; i < n; i++) {
    double modifiedZ = fabs(data[i] - median) / scaledMAD;
    if (modifiedZ <= threshold) {
      sum += data[i];
      count++;
    }
  }

  return count > 0 ? sum / count : median;
}

long filteredMaxOfWindow(long *data, int n, double threshold = 3.5) {
  double median = calculateMedianWindow(data, scratch, n);
  double mad = calculateMADWindow(data, scratch, n, median);
  double scaledMAD = std::max(mad * 1.4826, MIN_SCALED_MAD);

  long maxVal = LONG_MIN;
  bool found = false;

  for (int i = 0; i < n; i++) {
    double modifiedZ = fabs(data[i] - median) / scaledMAD;
    if (modifiedZ <= threshold && (!found || data[i] > maxVal)) {
      maxVal = data[i];
      found = true;
    }
  }

  return found ? maxVal : (long)round(median);
}

long filteredMinOfWindow(long *data, int n, double threshold = 3.5) {
  double median = calculateMedianWindow(data, scratch, n);
  double mad = calculateMADWindow(data, scratch, n, median);
  double scaledMAD = std::max(mad * 1.4826, MIN_SCALED_MAD);

  long minVal = LONG_MAX;
  bool found = false;

  for (int i = 0; i < n; i++) {
    double modifiedZ = fabs(data[i] - median) / scaledMAD;
    if (modifiedZ <= threshold && (!found || data[i] < minVal)) {
      minVal = data[i];
      found = true;
    }
  }

  return found ? minVal : (long)round(median);
}

long detectionValue(long *data, int n) {
  switch (TARGET_DETECTION) {
    case DETECT_MINIMUM: return filteredMinOfWindow(data, n);
    case DETECT_MAXIMUM: return filteredMaxOfWindow(data, n);
    default: return (long)round(filteredAvgOfWindow(data, n));
  }
}

// Only call with the drive relays off
void setReverse(bool reverse) {
  if (REVERSE_RELAY_PIN < 0) return;
  digitalWrite(REVERSE_RELAY_PIN, reverse ? LOW : HIGH);
  reverseOn = reverse;
}

void relaysOFF() {
  digitalWrite(RELAY_2_PIN, HIGH);
  delay(200);
  digitalWrite(RELAY_1_PIN, HIGH);
  relaysOn = false;
  if (reverseOn) {
    delay(DIRECTION_SETTLE_MS);
    setReverse(false);
  }
}

void relaysON() {
  if (reverseOn) relaysOFF();
  digitalWrite(RELAY_2_PIN, LOW);
  delay(100);
  digitalWrite(RELAY_1_PIN, LOW);
  relaysOn = true;
}

void relaysReverseON() {
  if (relaysOn && !reverseOn) {
    relaysOFF();
    delay(DIRECTION_SETTLE_MS);
  }
  setReverse(true);
  delay(100);
  digitalWrite(RELAY_2_PIN, LOW);
  delay(100);
  digitalWrite(RELAY_1_PIN, LOW);
  relaysOn = true;
}

void sendPacket(uint8_t messageType);

// Keep the tool online while loop() is blocked
const unsigned long KEEPALIVE_INTERVAL_MS = 250;
void keepAlive() {
  unsigned long now = millis();
  if (now - lastDiscoveryTime >= KEEPALIVE_INTERVAL_MS) {
    lastDiscoveryTime = now;
    sendPacket(MSG_DISCOVERY);
  }
}

// Tare until readings are near zero and stable, or timeout
bool tareUntilStable(int windowSize, long maxError, double maxScaledMAD, unsigned long timeoutMs) {
  if (windowSize <= 0 || windowSize > MAX_WINDOW_SIZE) {
    windowSize = MAX_WINDOW_SIZE;
  }

  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    keepAlive();
    scale.tare();
    keepAlive();

    for (int i = 0; i < windowSize; i++) {
      while (!scale.is_ready()) {
        if (millis() - start >= timeoutMs) return false;
        keepAlive();
      }
      window[i] = scale.get_value(1);
    }

    double avg = filteredAvgOfWindow(window, windowSize);
    double median = calculateMedianWindow(window, scratch, windowSize);
    double scaledMAD =
        calculateMADWindow(window, scratch, windowSize, median) * 1.4826;

    Serial.print("Tare check - Filtered mean: ");
    Serial.print(avg, 1);
    Serial.print(" counts (");
    Serial.print(avg * N_PER_COUNT, 2);
    Serial.print(" N), Median: ");
    Serial.print(median, 1);
    Serial.print(" counts (");
    Serial.print(median * N_PER_COUNT, 2);
    Serial.print(" N), Scaled MAD: ");
    Serial.print(scaledMAD, 1);
    Serial.print(" counts (");
    Serial.print(scaledMAD * N_PER_COUNT, 2);
    Serial.println(" N)");

    if (fabs(avg) <= maxError && scaledMAD <= maxScaledMAD)
      return true;
  }

  return false;
}

const char *stateName(ToolState state) {
  switch (state) {
    case IDLE: return "IDLE";
    case TARE_FAILED: return "TARE_FAILED";
    case READY: return "READY";
    case MONITORING: return "MONITORING";
    case TARGET_REACHED: return "TARGET_REACHED";
    case STOPPED: return "STOPPED";
    case LOOSENING: return "LOOSENING";
    default: return "UNKNOWN";
  }
}

WirelessPacket buildPacket(uint8_t messageType) {
  WirelessPacket packet = {};
  packet.messageType = messageType;
  strncpy(packet.toolId, TOOL_ID, sizeof(packet.toolId) - 1);
  packet.value = targetN;
  packet.rawCounts = latestReading;
  packet.filteredCounts = filteredMeasuredCounts;
  packet.filteredAvgCounts = filteredMeasuredCounts;
  packet.state = (uint8_t)toolState;
  packet.relayOn = relaysOn;
  return packet;
}

unsigned long sendFailures = 0;
unsigned long sendsAttempted = 0;

void broadcast(const WirelessPacket &packet) {
  sendsAttempted++;
  if (esp_now_send(BROADCAST_MAC, (const uint8_t *)&packet, sizeof(packet)) != ESP_OK)
    sendFailures++;
}

void reportSendFailures() {
  static unsigned long lastReport = 0;
  if (millis() - lastReport < 1000) return;
  lastReport = millis();
  if (sendFailures > 0) {
    Serial.print("WARN: ");
    Serial.print(sendFailures);
    Serial.print(" of ");
    Serial.print(sendsAttempted);
    Serial.println(" ESP-NOW sends failed in the last second");
  }
  sendFailures = 0;
  sendsAttempted = 0;
}

void sendPacket(uint8_t messageType) {
  WirelessPacket packet = buildPacket(messageType);
  broadcast(packet);
}

void sendStatus() {
  sendPacket(MSG_STATUS);
}

// value = target, or the refused one
void sendResponse(uint8_t messageType, uint8_t command, float value) {
  WirelessPacket packet = buildPacket(messageType);
  packet.command = command;
  packet.value = value;
  broadcast(packet);
}

void resetMeasurementWindow() {
  windowWriteIndex = 0;
  totalReadingsTaken = 0;
  latestReading = 0;
  filteredMeasuredCounts = 0;
}

void resetDisplayWindow() {
  displayReadingsTaken = 0;
  lastDisplayAvgTime = millis();
  filteredAvgCounts = 0;
}

void handleCommand(const WirelessPacket &packet) {
  // RESET avoids ESP.restart(): it left the board silent
  if (packet.command == CMD_RESET || packet.command == CMD_STOP) {
    relaysOFF();
    targetN = 0;
    targetCounts = 0;
    toolState = IDLE;
    if (packet.command == CMD_RESET) {
      tareValid = false;
      resetMeasurementWindow();
      resetDisplayWindow();
      Serial.println("Remote reset: relays off, target and tare cleared, IDLE. Send TARE before use.");
    } else {
      Serial.println(tareValid
          ? "Stopped: relays off, target cleared, IDLE. Tare and readings kept, ready to tighten."
          : "Stopped: relays off, target cleared, IDLE. Send TARE before use.");
    }
    sendResponse(MSG_ACK, packet.command, targetN);
    sendStatus();
    return;
  }

  if (packet.command == CMD_TARE) {
    // ACK first: the controller then allows for the silent tare
    sendResponse(MSG_ACK, CMD_TARE, targetN);
    relaysOFF();
    Serial.println("Remote tare requested.");

    if (tareUntilStable(TARE_WINDOW_SIZE, MAX_TARE_ERROR, MAX_TARE_MAD, TARE_TIMEOUT)) {
      tareValid = true;
      toolState = READY;
      resetMeasurementWindow();
      resetDisplayWindow();
      Serial.println("Remote tare successful.");
    } else {
      tareValid = false;
      toolState = TARE_FAILED;
      resetMeasurementWindow();
      resetDisplayWindow();
      Serial.println("Remote tare failed.");
    }

    sendResponse(MSG_DONE, CMD_TARE, targetN);
    sendStatus();
    return;
  }

  if (packet.command == CMD_LOOSEN) {
    if (toolState != LOOSENING) {
      loosenTared = tareValid;
      stateAfterLoosen = loosenTared ? READY : toolState;
    }

    relaysReverseON();
    loosenStartTime = millis();
    toolState = LOOSENING;
    if (REVERSE_RELAY_PIN < 0) Serial.println("No direction relay: make sure the wrench is set to REVERSE by hand.");
    Serial.print("Loosening (reverse) for up to ");
    Serial.print(LOOSEN_TIMEOUT_MS / 1000);
    Serial.println(" s.");

    sendResponse(MSG_ACK, CMD_LOOSEN, targetN);
    sendStatus();
    return;
  }

  if (packet.command == CMD_SET_TARGET) {
    if (!(packet.value > 0)) {   // also catches NaN
      Serial.println("Target refused: must be above 0 N.");
      sendResponse(MSG_NACK, CMD_SET_TARGET, packet.value);
      return;
    }
    if (!tareValid) {
      Serial.print("Target refused, tool not tared (state ");
      Serial.print(stateName(toolState));
      Serial.println("). Send TARE first.");
      sendResponse(MSG_NACK, CMD_SET_TARGET, packet.value);
      return;
    }

    targetN = packet.value;
    targetCounts = (long)round(targetN / N_PER_COUNT);
    maxCounts = (long)round(targetN * MAX_OVER_TARGET / N_PER_COUNT);
    overMaxReadings = 0;

    // Already at target: leave relays off
    if (toolState != MONITORING && toolState != LOOSENING &&
        totalReadingsTaken >= MIN_DETECT_READINGS && filteredMeasuredCounts >= targetCounts) {
      relaysOFF();
      toolState = TARGET_REACHED;
      Serial.print("Target ");
      Serial.print(targetN, 4);
      Serial.print(" N already reached (");
      Serial.print(filteredMeasuredCounts * N_PER_COUNT, 2);
      Serial.println(" N filtered). Relays stay off.");
      sendResponse(MSG_ACK, CMD_SET_TARGET, targetN);
      sendStatus();
      return;
    }

    resetMeasurementWindow();

    relaysON();
    phaseStartTime = millis();
    runStartTime = phaseStartTime;
    toolState = MONITORING;

    Serial.print("New target: ");
    Serial.print(targetN, 4);
    Serial.print(" N (");
    Serial.print(targetCounts);
    Serial.println(" counts)");

    sendResponse(MSG_ACK, CMD_SET_TARGET, targetN);
    sendStatus();
  }
}

void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(WirelessPacket)) return;

  WirelessPacket packet;
  memcpy(&packet, data, sizeof(packet));

  if (packet.messageType != MSG_COMMAND) return;

  bool addressedToMe = strncmp(packet.toolId, TOOL_ID, sizeof(packet.toolId)) == 0;
  bool addressedToAll = strncmp(packet.toolId, "ALL", sizeof(packet.toolId)) == 0;

  if (!addressedToMe && !addressedToAll) return;

  portENTER_CRITICAL(&commandQueueMux);
  int next = (commandQueueHead + 1) % COMMAND_QUEUE_SIZE;
  if (next != commandQueueTail) {
    commandQueue[commandQueueHead] = packet;
    commandQueueHead = next;
  }
  portEXIT_CRITICAL(&commandQueueMux);
}

bool popCommand(WirelessPacket &out) {
  bool found = false;
  portENTER_CRITICAL(&commandQueueMux);
  if (commandQueueTail != commandQueueHead) {
    out = commandQueue[commandQueueTail];
    commandQueueTail = (commandQueueTail + 1) % COMMAND_QUEUE_SIZE;
    found = true;
  }
  portEXIT_CRITICAL(&commandQueueMux);
  return found;
}

// Serial debug: typed commands go through handleCommand(). HELP lists them.

void printSerialHelp() {
  Serial.println("--- SERIAL DEBUG COMMANDS ---");
  Serial.println("TARE            - trigger a tare, same as remote TARE");
  Serial.println("STOP            - relays off, back to IDLE (tare kept), same as remote STOP");
  Serial.println("RESET           - like STOP but also clears the tare (needs TARE), same as remote RESET");
  Serial.println("TARGET,<N>      - start monitoring toward N newtons, same as remote TARGET");
  Serial.println("                  (TARGET,TOOL_01,<N> also works; any upper/lower case)");
  Serial.println("LOOSEN          - run in reverse, stops by itself after 5 s (STOP ends it early)");
  Serial.println("STATUS          - print current state/readings locally");
  Serial.println("RAW             - 5 s of raw HX711 readings + sample rate");
  Serial.println("HELP            - show this list");
}

// RAW: 5 s of absolute and tared HX711 readings, then a summary
void runRawDiagnostic() {
  if (toolState == MONITORING || toolState == LOOSENING) {
    Serial.println("ERROR,RAW_NOT_ALLOWED_WHILE_RUNNING - send STOP first");
    return;
  }
  relaysOFF();
  Serial.println("RAW,ms,absolute_counts,tared_counts,tared_N");
  const unsigned long durationMs = 5000;
  unsigned long start = millis();
  long n = 0, minV = LONG_MAX, maxV = LONG_MIN;
  double sum = 0;
  while (millis() - start < durationMs) {
    if (!scale.is_ready()) { keepAlive(); continue; }
    long absolute = scale.read();
    long tared = absolute - scale.get_offset();
    Serial.print("RAW,");
    Serial.print(millis() - start);
    Serial.print(",");
    Serial.print(absolute);
    Serial.print(",");
    Serial.print(tared);
    Serial.print(",");
    Serial.println(tared * N_PER_COUNT, 2);
    n++;
    sum += tared;
    minV = std::min(minV, tared);
    maxV = std::max(maxV, tared);
  }
  Serial.print("RAW_SUMMARY,samples=");
  Serial.print(n);
  Serial.print(",rate_hz=");
  Serial.print(n * 1000.0 / durationMs, 1);
  if (n > 0) {
    Serial.print(",mean_tared=");
    Serial.print(sum / n, 1);
    Serial.print(",min=");
    Serial.print(minV);
    Serial.print(",max=");
    Serial.print(maxV);
    Serial.print(",spread=");
    Serial.print(maxV - minV);
    Serial.print(",mean_N=");
    Serial.print(sum / n * N_PER_COUNT, 2);
    Serial.print(",min_N=");
    Serial.print(minV * N_PER_COUNT, 2);
    Serial.print(",max_N=");
    Serial.print(maxV * N_PER_COUNT, 2);
    Serial.print(",spread_N=");
    Serial.print((maxV - minV) * N_PER_COUNT, 2);
  }
  Serial.println();
}

void printLocalStatus() {
  Serial.println("--- STATUS ---");
  Serial.print("State: ");
  Serial.println(stateName(toolState));
  Serial.print("Target: ");
  Serial.print(targetN, 4);
  Serial.print(" N (");
  Serial.print(targetCounts);
  Serial.println(" counts)");
  Serial.print("Latest reading: ");
  Serial.print(latestReading);
  Serial.print(" counts (");
  Serial.print(latestReading * N_PER_COUNT, 4);
  Serial.println(" N)");
  Serial.print("Filtered value (target detection): ");
  Serial.print(filteredMeasuredCounts);
  Serial.print(" counts (");
  Serial.print(filteredMeasuredCounts * N_PER_COUNT, 4);
  Serial.println(" N)");
  Serial.print("Filtered avg (live display, ~1s): ");
  Serial.print(filteredAvgCounts);
  Serial.print(" counts (");
  Serial.print(filteredAvgCounts * N_PER_COUNT, 4);
  Serial.println(" N)");
  Serial.print("Relays: ");
  Serial.println(relaysOn ? "ON" : "OFF");
}

bool readSerialLine(String &out) {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      out = buf;
      out.trim();
      buf = "";
      return true;
    } else if (c != '\r') {
      buf += c;
    }
  }
  return false;
}

void handleSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  String upperLine = line;
  upperLine.toUpperCase();

  if (upperLine == "HELP") {
    printSerialHelp();
    return;
  }

  if (upperLine == "STATUS") {
    printLocalStatus();
    return;
  }

  if (upperLine == "RAW") {
    runRawDiagnostic();
    return;
  }

  WirelessPacket packet = {};
  packet.messageType = MSG_COMMAND;
  strncpy(packet.toolId, TOOL_ID, sizeof(packet.toolId) - 1);

  if (upperLine == "TARE") {
    Serial.println("[SERIAL] Simulating TARE command...");
    packet.command = CMD_TARE;
    handleCommand(packet);
    return;
  }

  if (upperLine == "STOP") {
    Serial.println("[SERIAL] Simulating STOP command...");
    packet.command = CMD_STOP;
    handleCommand(packet);
    return;
  }

  if (upperLine == "LOOSEN") {
    Serial.println("[SERIAL] Simulating LOOSEN command...");
    packet.command = CMD_LOOSEN;
    handleCommand(packet);
    return;
  }

  if (upperLine == "RESET") {
    Serial.println("[SERIAL] Simulating RESET command...");
    packet.command = CMD_RESET;
    handleCommand(packet);
    return;
  }

  if (upperLine.startsWith("TARGET")) {
    int first = line.indexOf(',');
    int last = line.lastIndexOf(',');
    if (first < 0) {
      Serial.println("ERROR,BAD_TARGET_COMMAND - usage: TARGET,<N> or TARGET,<tool>,<N>");
      return;
    }
    if (last != first) {
      String id = line.substring(first + 1, last);
      id.trim();
      id.toUpperCase();
      if (id != TOOL_ID) {
        Serial.print("ERROR,WRONG_TOOL - this tool is ");
        Serial.println(TOOL_ID);
        return;
      }
    }
    String number = line.substring(last + 1);
    number.trim();
    float value = number.toFloat();
    if (value <= 0) {
      Serial.println("ERROR,BAD_TARGET_VALUE - target must be a number above 0, e.g. TARGET,150");
      return;
    }
    Serial.print("[SERIAL] Simulating TARGET command, value=");
    Serial.println(value, 4);
    packet.command = CMD_SET_TARGET;
    packet.value = value;
    handleCommand(packet);
    return;
  }

  Serial.println("ERROR,UNKNOWN_COMMAND - type HELP for a list");
}

const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "POWER ON";
    case ESP_RST_SW: return "SOFTWARE RESTART";
    case ESP_RST_PANIC: return "PANIC (crash)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "WATCHDOG (code stuck)";
    case ESP_RST_BROWNOUT: return "BROWNOUT (supply voltage dipped)";
    case ESP_RST_EXT: return "RESET PIN / BUTTON";
    default: return "OTHER";
  }
}

bool setupWireless() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) return false;

  esp_now_register_recv_cb(onDataReceived);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (!esp_now_is_peer_exist(BROADCAST_MAC) &&
      esp_now_add_peer(&peer) != ESP_OK)
    return false;

  Serial.print("Tool ID: ");
  Serial.println(TOOL_ID);
  Serial.print("WiFi MAC: ");
  Serial.println(WiFi.macAddress());

  return true;
}

void setup() {
  // Relays off before anything else
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  digitalWrite(RELAY_1_PIN, HIGH);
  digitalWrite(RELAY_2_PIN, HIGH);
  relaysOn = false;
  if (REVERSE_RELAY_PIN >= 0) {
    pinMode(REVERSE_RELAY_PIN, OUTPUT);
    digitalWrite(REVERSE_RELAY_PIN, HIGH);
  }
  reverseOn = false;

  Serial.begin(115200);

  // Don't block forever: no PC on battery
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 2000)
    delay(10);

  Serial.println("--- TARGET PRELOAD MONITOR (ESP-NOW) ---");
  Serial.print("Reset reason: ");
  Serial.println(resetReasonName(esp_reset_reason()));

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  if (!setupWireless()) {
    Serial.println("ESP-NOW initialization failed.");
    while (true) delay(1000);
  }

  while (!scale.is_ready()) {
    Serial.println("HX711 not found. Check your wiring connections!");
    keepAlive();
    delay(500);
  }

  toolState = IDLE;
  Serial.println("Idle. Waiting for TARE command from controller before use.");

  sendStatus();

  printSerialHelp();
}

void loop() {
  unsigned long now = millis();
  reportSendFailures();

  if (now - lastDiscoveryTime >= DISCOVERY_INTERVAL_MS) {
    lastDiscoveryTime = now;
    sendPacket(MSG_DISCOVERY);
  }

  if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
    lastStatusTime = now;
    sendStatus();
  }

  WirelessPacket queued;
  while (popCommand(queued))
    handleCommand(queued);

  String serialLine;
  if (readSerialLine(serialLine)) {
    handleSerialCommand(serialLine);
  }

  now = millis();

  // One HX711 read per loop: a second read would starve
  bool haveSample = false;
  long sample = 0;
  bool tared = tareValid;
  if (tared && scale.is_ready()) {
    // Absolute value: compression can read negative
    sample = labs((long)scale.get_value(1));
    haveSample = true;
    if (toolState != MONITORING) latestReading = sample;
    if (displayReadingsTaken < MAX_WINDOW_SIZE)
      displayWindow[displayReadingsTaken++] = sample;
  }

  if (now - lastDisplayAvgTime >= DISPLAY_AVG_INTERVAL_MS &&
      displayReadingsTaken >= MIN_DISPLAY_SAMPLES) {
    lastDisplayAvgTime = now;
    filteredAvgCounts =
        (long)round(filteredAvgOfWindow(displayWindow, displayReadingsTaken));
    displayReadingsTaken = 0;
  }

  if (toolState == LOOSENING) {
    if (haveSample && loosenTared) {
      latestReading = sample;
      window[windowWriteIndex] = latestReading;
      windowWriteIndex = (windowWriteIndex + 1) % WINDOW_SIZE;
      if (totalReadingsTaken < WINDOW_SIZE) totalReadingsTaken++;
      if (totalReadingsTaken >= MIN_DETECT_READINGS)
        filteredMeasuredCounts = detectionValue(window, std::min((int)totalReadingsTaken, WINDOW_SIZE));
    }
    if (now - loosenStartTime >= LOOSEN_TIMEOUT_MS) {
      relaysOFF();
      toolState = stateAfterLoosen;
      sendResponse(MSG_DONE, CMD_LOOSEN, targetN);
      sendStatus();
      Serial.print("Loosen finished: ");
      Serial.print(LOOSEN_TIMEOUT_MS / 1000);
      Serial.print(" s timeout, relays off, state ");
      Serial.println(stateName(toolState));
    }
    delay(1);
    return;
  }

  // Keep the value live so the dashboard sees bolts relax
  if (toolState == TARGET_REACHED || toolState == READY || toolState == STOPPED || toolState == IDLE) {
    if (haveSample) {
      latestReading = sample;
      window[windowWriteIndex] = latestReading;
      windowWriteIndex = (windowWriteIndex + 1) % WINDOW_SIZE;
      if (totalReadingsTaken < WINDOW_SIZE) totalReadingsTaken++;
      if (totalReadingsTaken >= MIN_DETECT_READINGS)
        filteredMeasuredCounts = detectionValue(window, std::min((int)totalReadingsTaken, WINDOW_SIZE));
    }
    delay(1);
    return;
  }

  if (toolState != MONITORING) {
    delay(1);
    return;
  }

  if (now - runStartTime >= MAX_RUN_MS) {
    relaysOFF();
    toolState = STOPPED;
    sendStatus();
    Serial.println("SAFETY STOP: run timed out, relays off.");
    return;
  }

  // Relay pulsing (disabled)
  // if (relaysOn && (now - phaseStartTime >= PULSE_ON_MS)) {
  //   relaysOFF();
  //   phaseStartTime = now;
  //   Serial.println(">>> BOTH RELAYS OFF");
  // } else if (!relaysOn && (now - phaseStartTime >= PULSE_OFF_MS)) {
  //   relaysON();
  //   phaseStartTime = now;
  //   Serial.println(">>> BOTH RELAYS ON");
  // }

  if (!haveSample) return;

  latestReading = sample;
  Serial.println(latestReading);
  window[windowWriteIndex] = latestReading;
  windowWriteIndex = (windowWriteIndex + 1) % WINDOW_SIZE;
  totalReadingsTaken++;

  // Don't wait for a full window: stop as soon as possible
  if (totalReadingsTaken < MIN_DETECT_READINGS) return;
  int filled = totalReadingsTaken < WINDOW_SIZE ? (int)totalReadingsTaken : WINDOW_SIZE;

  filteredMeasuredCounts = detectionValue(window, filled);

  if (totalReadingsTaken % 10 == 0) {
    Serial.print(TARGET_DETECTION == DETECT_AVERAGE ? "Average filtered value (last "
                 : TARGET_DETECTION == DETECT_MINIMUM ? "Lowest filtered value (last "
                 : "Highest filtered value (last ");
    Serial.print(WINDOW_SIZE);
    Serial.print(" readings): ");
    Serial.print(filteredMeasuredCounts);
    Serial.print(" counts (");
    Serial.print(filteredMeasuredCounts * N_PER_COUNT, 3);
    Serial.println(" N)");
  }

  // Over-force on the filtered value so one spike can't stop a run
  overMaxReadings = filteredMeasuredCounts > maxCounts ? overMaxReadings + 1 : 0;
  if (overMaxReadings >= MAX_FORCE_READINGS) {
    relaysOFF();
    toolState = STOPPED;
    sendStatus();
    Serial.print("SAFETY STOP: over max force, ");
    Serial.print(filteredMeasuredCounts * N_PER_COUNT, 2);
    Serial.println(" N (filtered). Relays off.");
    return;
  }

  if (filteredMeasuredCounts >= targetCounts) {
    relaysOFF();
    toolState = TARGET_REACHED;
    sendStatus();

    Serial.println();
    Serial.println("=========================================");
    Serial.println("TARGET REACHED");
    Serial.println("BOTH RELAYS OFF");
    Serial.print("Target: ");
    Serial.print(targetN, 4);
    Serial.println(" N");
    Serial.print("Achieved: ");
    Serial.print(filteredMeasuredCounts * N_PER_COUNT, 4);
    Serial.println(" N");
    Serial.print("Count: ");
    Serial.print(filteredMeasuredCounts);
    Serial.print(" counts (");
    Serial.print(filteredMeasuredCounts * N_PER_COUNT, 2);
    Serial.println(" N)");
    Serial.println("=========================================");
  }
}
