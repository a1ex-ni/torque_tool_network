// ESP-NOW <-> USB Serial gateway for the tool units.
// The receive callback only queues; loop() owns Serial and the tool table.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <string.h>

const int MAX_TOOLS = 20;
const unsigned long TOOL_OFFLINE_MS = 5000;
// Extra offline allowance while a tool tares (silent)
const unsigned long TOOL_TARE_TIMEOUT_MS = 60000;
const int RX_QUEUE_SIZE = 32;
const unsigned long RESPONSE_TIMEOUT_MS = 3000;  // then ERROR,NO_RESPONSE

enum MessageType : uint8_t {
  MSG_DISCOVERY = 1,
  MSG_STATUS = 2,
  MSG_COMMAND = 3,
  MSG_ACCEPTED = 4,
  MSG_NACCEPTED = 5,
  MSG_DONE = 6     // long command (TARE/LOOSEN) finished
};

enum CommandType : uint8_t {
  CMD_SET_TARGET = 1,
  CMD_STOP = 2,
  CMD_RESET = 3,
  CMD_TARE = 4,
  CMD_LOOSEN = 5
};

// Must match WirelessPacket in target_monitor_wifi.ino byte for byte
struct __attribute__((packed)) WirelessPacket {
  uint8_t messageType;
  uint8_t command;
  char toolId[16];
  float value;
  int32_t rawCounts;
  int32_t filteredCounts;      // stop value compared with the target
  int32_t filteredAvgCounts;
  uint8_t state;
  uint8_t relayOn;
};

struct ToolPeer {
  bool used;
  bool online;
  char toolId[16];
  uint8_t mac[6];
  unsigned long lastSeen;
  WirelessPacket lastStatus;
  uint8_t pendingCommand;
  unsigned long pendingSince;
  bool taring;
};

ToolPeer tools[MAX_TOOLS];

struct RxPacket {
  uint8_t mac[6];
  WirelessPacket packet;
};

RxPacket rxQueue[RX_QUEUE_SIZE];
volatile int rxHead = 0;
volatile int rxTail = 0;
volatile uint32_t rxDropped = 0;
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

// Matches enum ToolState in target_monitor_wifi.ino
const char *stateName(uint8_t state) {
  switch (state) {
    case 0: return "IDLE";
    case 1: return "TARE_FAILED";
    case 2: return "READY";
    case 3: return "MONITORING";
    case 4: return "TARGET_REACHED";
    case 5: return "STOPPED";
    case 6: return "LOOSENING";
    default: return "UNKNOWN";
  }
}

const char *commandName(uint8_t command) {
  switch (command) {
    case CMD_SET_TARGET: return "TARGET";
    case CMD_STOP: return "STOP";
    case CMD_RESET: return "RESET";
    case CMD_TARE: return "TARE";
    case CMD_LOOSEN: return "LOOSEN";
    default: return "UNKNOWN";
  }
}

const char *responseName(uint8_t messageType) {
  switch (messageType) {
    case MSG_ACCEPTED: return "COMMAND ACCEPTED";
    case MSG_NACCEPTED: return "COMMAND REJECTED";
    case MSG_DONE: return "DONE";
    default: return "UNKNOWN";
  }
}

void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    if (i) Serial.print(":");
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
  }
}

int findTool(const char *toolId) {
  for (int i = 0; i < MAX_TOOLS; i++)
    if (tools[i].used && strncmp(tools[i].toolId, toolId, sizeof(tools[i].toolId)) == 0)
      return i;

  return -1;
}

int registerTool(const char *toolId, const uint8_t *mac) {
  int index = findTool(toolId);

  if (index < 0) {
    for (int i = 0; i < MAX_TOOLS; i++) {
      if (!tools[i].used) {
        index = i;
        tools[i].used = true;
        strncpy(tools[i].toolId, toolId, sizeof(tools[i].toolId) - 1);
        memcpy(tools[i].mac, mac, 6);

        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, mac, 6);
        peer.channel = 0;
        peer.encrypt = false;

        if (!esp_now_is_peer_exist(mac))
          esp_now_add_peer(&peer);

        Serial.print("EVENT,DISCOVERED,");
        Serial.print(toolId);
        Serial.print(",");
        printMac(mac);
        Serial.println();

        break;
      }
    }
  } else if (memcmp(tools[index].mac, mac, 6) != 0) {
    if (esp_now_is_peer_exist(tools[index].mac))
      esp_now_del_peer(tools[index].mac);

    memcpy(tools[index].mac, mac, 6);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
  }

  return index;
}

// STATUS,<id>,<state>,<target_N>,<raw_counts>,<stop_value_counts>,<relay_on>
void printStatus(const WirelessPacket &packet) {
  Serial.print("STATUS,");
  Serial.print(packet.toolId);
  Serial.print(",");
  Serial.print(stateName(packet.state));
  Serial.print(",");
  Serial.print(packet.value, 4);
  Serial.print(",");
  Serial.print(packet.rawCounts);
  Serial.print(",");
  Serial.print(packet.filteredCounts);
  Serial.print(",");
  Serial.println(packet.relayOn ? 1 : 0);
}

// COMMAND ACCEPTED | COMMAND REJECTED | DONE,<id>,<command>,<state>,<target_N>
void printResponse(const WirelessPacket &packet) {
  Serial.print(responseName(packet.messageType));
  Serial.print(",");
  Serial.print(packet.toolId);
  Serial.print(",");
  Serial.print(commandName(packet.command));
  Serial.print(",");
  Serial.print(stateName(packet.state));
  Serial.print(",");
  Serial.println(packet.value, 4);
}

// WiFi task: queue only
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(WirelessPacket)) return;

  portENTER_CRITICAL(&rxMux);
  int next = (rxHead + 1) % RX_QUEUE_SIZE;
  if (next != rxTail) {
    memcpy(rxQueue[rxHead].mac, info->src_addr, 6);
    memcpy(&rxQueue[rxHead].packet, data, sizeof(WirelessPacket));
    rxHead = next;
  } else {
    rxDropped++;
  }
  portEXIT_CRITICAL(&rxMux);
}

bool popRx(RxPacket &out) {
  bool found = false;
  portENTER_CRITICAL(&rxMux);
  if (rxTail != rxHead) {
    out = rxQueue[rxTail];
    rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
    found = true;
  }
  portEXIT_CRITICAL(&rxMux);
  return found;
}

void handlePacket(const RxPacket &rx) {
  WirelessPacket packet = rx.packet;

  if (packet.messageType < MSG_DISCOVERY || packet.messageType > MSG_DONE ||
      packet.messageType == MSG_COMMAND)
    return;

  packet.toolId[sizeof(packet.toolId) - 1] = '\0';

  char toolId[sizeof(packet.toolId) + 1];
  memcpy(toolId, packet.toolId, sizeof(packet.toolId));
  toolId[sizeof(packet.toolId)] = '\0';

  int index = registerTool(toolId, rx.mac);
  if (index < 0) {
    Serial.println("ERROR,TOOL_TABLE_FULL");
    return;
  }

  tools[index].lastSeen = millis();
  if (!tools[index].online) {
    tools[index].online = true;
    Serial.print("EVENT,ONLINE,");
    Serial.println(tools[index].toolId);
  }

  if (packet.messageType == MSG_STATUS) {
    tools[index].taring = false;
    tools[index].lastStatus = packet;
    printStatus(packet);
  } else if (packet.messageType != MSG_DISCOVERY) {
    if (packet.command == tools[index].pendingCommand)
      tools[index].pendingCommand = 0;
    if (packet.command == CMD_TARE) {
      if (packet.messageType == MSG_ACCEPTED) tools[index].taring = true;
      if (packet.messageType == MSG_DONE) tools[index].taring = false;
    }
    printResponse(packet);
  }
}

void checkResponses() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_TOOLS; i++) {
    if (tools[i].used && tools[i].pendingCommand &&
        now - tools[i].pendingSince > RESPONSE_TIMEOUT_MS) {
      Serial.print("ERROR,NO_RESPONSE,");
      Serial.print(tools[i].toolId);
      Serial.print(",");
      Serial.println(commandName(tools[i].pendingCommand));
      tools[i].pendingCommand = 0;
    }
  }
}

void checkOffline() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_TOOLS; i++) {
    if (!tools[i].used || !tools[i].online) continue;
    unsigned long limit = TOOL_OFFLINE_MS;
    if (tools[i].taring) limit += TOOL_TARE_TIMEOUT_MS;
    if (now - tools[i].lastSeen > limit) {
      tools[i].online = false;
      Serial.print("EVENT,OFFLINE,");
      Serial.println(tools[i].toolId);
    }
  }
}

bool sendCommand(const char *toolId, uint8_t command, float value = 0) {
  int index = findTool(toolId);

  if (index < 0) {
    Serial.print("ERROR,UNKNOWN_TOOL,");
    Serial.println(toolId);
    return false;
  }

  WirelessPacket packet = {};
  packet.messageType = MSG_COMMAND;
  packet.command = command;
  strncpy(packet.toolId, toolId, sizeof(packet.toolId) - 1);
  packet.value = value;

  esp_err_t result =
      esp_now_send(tools[index].mac, (uint8_t *)&packet, sizeof(packet));

  if (result != ESP_OK) {
    Serial.print("ERROR,SEND_FAILED,");
    Serial.println(toolId);
    return false;
  }

  tools[index].pendingCommand = command;
  tools[index].pendingSince = millis();

  // SENT,<id>,<command>[,<target_N>]
  Serial.print("SENT,");
  Serial.print(toolId);
  Serial.print(",");
  Serial.print(commandName(command));
  if (command == CMD_SET_TARGET) {
    Serial.print(",");
    Serial.print(value, 4);
  }
  Serial.println();
  return true;
}

void sendCommandToAll(uint8_t command, float value = 0) {
  bool any = false;
  for (int i = 0; i < MAX_TOOLS; i++) {
    if (tools[i].used) {
      sendCommand(tools[i].toolId, command, value);
      any = true;
    }
  }
  if (!any)
    Serial.println("ERROR,NO_TOOLS");
}

bool parseTarget(const String &text, float &targetN) {
  targetN = text.toFloat();
  if (targetN <= 0) {
    Serial.println("ERROR,BAD_TARGET_VALUE");
    return false;
  }
  return true;
}

void listTools() {
  unsigned long now = millis();

  for (int i = 0; i < MAX_TOOLS; i++) {
    if (!tools[i].used) continue;

    Serial.print("TOOL,");
    Serial.print(tools[i].toolId);
    Serial.print(",");
    Serial.print(tools[i].online ? "ONLINE" : "OFFLINE");
    Serial.print(",");
    printMac(tools[i].mac);
    Serial.println();
  }
}

void handleSerialCommand(String line) {
  line.trim();
  if (!line.length()) return;
  line.toUpperCase();

  if (line == "LIST") {
    listTools();
    return;
  }

  if (line == "STOP_ALL") {
    sendCommandToAll(CMD_STOP);
    return;
  }

  if (line == "RESET_ALL") {
    sendCommandToAll(CMD_RESET);
    return;
  }

  if (line == "TARE_ALL") {
    sendCommandToAll(CMD_TARE);
    return;
  }

  if (line == "LOOSEN_ALL") {
    sendCommandToAll(CMD_LOOSEN);
    return;
  }

  int comma = line.indexOf(',');
  if (comma < 0) {
    Serial.println("ERROR,BAD_COMMAND");
    return;
  }

  String command = line.substring(0, comma);
  String rest = line.substring(comma + 1);

  if (command == "TARGET_ALL") {
    float targetN;
    if (parseTarget(rest, targetN))
      sendCommandToAll(CMD_SET_TARGET, targetN);
    return;
  }

  if (command == "STOP") {
    sendCommand(rest.c_str(), CMD_STOP);
    return;
  }

  if (command == "RESET") {
    sendCommand(rest.c_str(), CMD_RESET);
    return;
  }

  if (command == "TARE") {
    sendCommand(rest.c_str(), CMD_TARE);
    return;
  }

  if (command == "LOOSEN") {
    sendCommand(rest.c_str(), CMD_LOOSEN);
    return;
  }

  if (command == "TARGET") {
    int secondComma = rest.indexOf(',');
    if (secondComma < 0) {
      Serial.println("ERROR,BAD_TARGET_COMMAND");
      return;
    }

    String toolId = rest.substring(0, secondComma);
    float targetN;
    if (parseTarget(rest.substring(secondComma + 1), targetN))
      sendCommand(toolId.c_str(), CMD_SET_TARGET, targetN);
    return;
  }

  Serial.println("ERROR,UNKNOWN_COMMAND");
}

void setup() {
  Serial.begin(115200);

  while (!Serial)
    delay(100);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR,ESP_NOW_INIT_FAILED");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onDataReceived);

  Serial.println("EVENT,CONTROLLER_READY");
  Serial.print("EVENT,CONTROLLER_MAC,");
  Serial.println(WiFi.macAddress());
  Serial.println("EVENT,PROTOCOL,LIST | TARGET,<tool>,<N> | STOP,<tool> | TARE,<tool> | RESET,<tool> | LOOSEN,<tool> | TARGET_ALL,<N> | STOP_ALL | TARE_ALL | RESET_ALL | LOOSEN_ALL");
  Serial.println("EVENT,REPLIES,SENT,<tool>,<cmd>[,<N>] | ACCEPTED/REJECTED/DONE,<tool>,<cmd>,<state>,<N> | ERROR,NO_RESPONSE,<tool>,<cmd>");
}

bool readSerialLine(String &out) {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      out = buf;
      buf = "";
      return true;
    }
    if (c != '\r' && buf.length() < 128) buf += c;
  }
  return false;
}

void loop() {
  RxPacket rx;
  while (popRx(rx))
    handlePacket(rx);

  static uint32_t reportedDrops = 0;
  if (rxDropped != reportedDrops) {
    reportedDrops = rxDropped;
    Serial.print("ERROR,RX_QUEUE_FULL,");
    Serial.println(reportedDrops);
  }

  checkOffline();
  checkResponses();

  String line;
  if (readSerialLine(line))
    handleSerialCommand(line);

  delay(1);
}
