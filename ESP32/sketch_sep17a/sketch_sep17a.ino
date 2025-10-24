#include <WiFi.h>
#include <PubSubClient.h>

// ---------- MACROS FIRST (so later code can use them) ----------
#define DBG Serial
#define STM Serial2     // UART2 bridge to STM32 (RX2=GPIO16, TX2=GPIO17 on most ESP32 dev boards)

// ---------- AIO / WiFi settings ----------
#define WIFI_SSID "your-ssid"
#define WIFI_PASS "your-pass"


const char* AIO_HOST  = "io.adafruit.com";
const uint16_t AIO_PORT = 1883;
#define AIO_USER  "your-aio-username"
#define AIO_KEY   "your-aio-key"
const char* DEVICE_ID = "stm32f4-01";

// ---------- MQTT client ----------
WiFiClient net;
PubSubClient mqtt(net);

// ---------- Feed topics ----------
const char* FEED_CFG_HEIGHT = "suryasurya/feeds/stm32f4-01.cfg-dot-height-cm";
const char* FEED_CFG_MASS   = "suryasurya/feeds/stm32f4-01.cfg-dot-mass-kg";
const char* FEED_CFG_STRIDE = "suryasurya/feeds/stm32f4-01.cfg-dot-stride-m";
const char* FEED_CMD        = "suryasurya/feeds/stm32f4-01.cmd";


// ---------- Prototypes ----------
void wifiConnect();
void mqttConnect();
static void sendCfgToStm32(const char* key, const char* val);

// ---------- Send CFG line to STM32 ----------
static void sendCfgToStm32(const char* key, const char* val) {
  DBG.printf("[CFG->STM] %s=%s\n", key, val);
  STM.print("CFG ");
  STM.print(key);
  STM.print("=");
  STM.print(val);
  STM.print("\n");
}
static void sendCmdToStm32(const char* word) {
  DBG.printf("[CMD->STM] %s\n", word);
  STM.print("CMD ");
  STM.print(word);
  STM.print("\n");
}


void setup() {
  DBG.begin(115200);
  delay(100);
  DBG.println("\n[BOOT]");

  // STM32 UART (RX2=GPIO16, TX2=GPIO17)
  STM.begin(115200, SERIAL_8N1, 16, 17);

  // WiFi + MQTT
  WiFi.mode(WIFI_STA);
  mqtt.setServer(AIO_HOST, AIO_PORT);

  // MQTT callback: route cfg topics -> STM32
  mqtt.setCallback([](char* topic, byte* payload, unsigned int length) {
    char msg[64];
    unsigned int n = (length < sizeof(msg)-1) ? length : sizeof(msg)-1;
    memcpy(msg, payload, n);
    msg[n] = '\0';

    DBG.printf("[SUB] %s <- %s\n", topic, msg);

    if (strcmp(topic, FEED_CFG_HEIGHT) == 0) { sendCfgToStm32("height_cm", msg); return; }
    if (strcmp(topic, FEED_CFG_MASS)   == 0) { sendCfgToStm32("mass_kg",  msg); return; }
    if (strcmp(topic, FEED_CFG_STRIDE) == 0) { sendCfgToStm32("stride_m", msg); return; }

    if (strcmp(topic, FEED_CMD) == 0) {
  // payload is expected to be: start | stop | snapshot | snap | summary
  char cmd[24];
  unsigned int n = (length < sizeof(cmd)-1) ? length : sizeof(cmd)-1;
  memcpy(cmd, payload, n);
  cmd[n] = '\0';

  // normalize to lowercase (optional)
  for (char* q = cmd; *q; ++q) { if (*q >= 'A' && *q <= 'Z') *q = *q - 'A' + 'a'; }

  if (strcmp(cmd,"start")==0 || strcmp(cmd,"stop")==0 ||
      strcmp(cmd,"snapshot")==0 || strcmp(cmd,"snap")==0 ||
      strcmp(cmd,"summary")==0) {
    sendCmdToStm32(cmd);
  } else {
    DBG.printf("[CMD] unknown: %s\n", cmd);
  }
  return;
}

  });

  wifiConnect();
  mqttConnect();
  DBG.println("[READY] Waiting for PUB lines from STM32...");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) wifiConnect();
  if (!mqtt.connected()) mqttConnect();
  mqtt.loop();

  static char lineBuf[256];
  static size_t lineLen = 0;

  while (STM.available()) {
    char c = (char)STM.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        // echo what we got from STM32 for visibility
        DBG.printf("[STM] %s\n", lineBuf);
        handleUartLine(lineBuf);
        lineLen = 0;
      }
    } else {
      if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = c;
      } else {
        // overflow -> reset buffer
        lineLen = 0;
      }
    }
  }
}


// ------- WiFi & MQTT connect helpers -------
void wifiConnect() {
  DBG.printf("[WiFi] Connecting to %s...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); DBG.print(".");
    if (millis() - t0 > 20000) { DBG.println("\n[WiFi] retry"); WiFi.disconnect(true); WiFi.begin(WIFI_SSID, WIFI_PASS); t0 = millis(); }
  }
  DBG.printf("\n[WiFi] OK %s\n", WiFi.localIP().toString().c_str());
}

void mqttConnect() {
  while (!mqtt.connected()) {
    DBG.printf("[MQTT] Connecting to %s...\n", AIO_HOST);
    if (mqtt.connect(DEVICE_ID, AIO_USER, AIO_KEY)) {
      DBG.println("[MQTT] Connected");
      mqtt.subscribe(FEED_CFG_HEIGHT);
      mqtt.subscribe(FEED_CFG_MASS);
      mqtt.subscribe(FEED_CFG_STRIDE);
      DBG.println("[MQTT] Subscribed to cfg feeds");
      mqtt.subscribe(FEED_CMD);
DBG.println("[MQTT] Subscribed to cmd feed");

    } else {
      DBG.printf("[MQTT] Fail rc=%d, retry in 2s\n", mqtt.state());
      delay(2000);
    }
  }
}

// ---- publish helper (retries if connection dropped) ----
static bool mqttPublish(const char* topic, const char* payload) {
  bool ok = mqtt.publish(topic, payload);
  if (!ok) {
    DBG.println("[MQTT] publish failed, reconnecting...");
    mqttConnect();
    ok = mqtt.publish(topic, payload);
  }
  return ok;
}

// ---- handle one complete line from STM32: "PUB <topic> <payload>" ----
static void handleUartLine(const char* line) {
  if (strncmp(line, "PUB ", 4) != 0) {
    DBG.printf("[STM?] %s\n", line);   // not a PUB; just print for debug
    return;
  }
  const char* p = line + 4;

  // extract topic (up to first space)
  static char topic[200];
  size_t tlen = 0;
  while (*p && *p != ' ' && tlen < sizeof(topic)-1) topic[tlen++] = *p++;
  topic[tlen] = '\0';

  // skip spaces to payload
  while (*p == ' ') p++;
  const char* payload = p;
  if (*payload == '\0') { DBG.println("[PARSE] empty payload"); return; }

  DBG.printf("[PUB] %s  <-  %s\n", topic, payload);
  mqttPublish(topic, payload);
}

