/*
 * Solar weather station for FireBeetle 2 ESP32-C5 + GY-BME280-3.3.
 *
 * Current test behavior:
 *   1. Stay awake with WiFi and the dashboard online.
 *   2. Measure the BME280 every 2 seconds.
 *   3. Prefer 5 GHz WiFi for RSSI presence detection.
 *   4. Append readings to LittleFS as CSV history.
 *
 * For solar deployment, set LOW_POWER_SLEEP to true and change
 * MEASUREMENT_INTERVAL_SECONDS back to 30.
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Adafruit_BME280.h>
#include "esp_wifi.h"

// ===================== User configuration =====================
const char *WIFI_SSID = "DEIN_WLAN_NAME";
const char *WIFI_PASSWORD = "DEIN_WLAN_PASSWORT";

static const bool LOW_POWER_SLEEP = false;
static const bool FORCE_5GHZ_WIFI = true;

static const uint32_t MEASUREMENT_INTERVAL_SECONDS = 2;
static const uint32_t WEB_WINDOW_MS = 6000;
static const uint32_t WEB_WINDOW_WHEN_CLIENT_MS = 120000;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;

static const uint16_t MAX_HISTORY_RECORDS = 2880;  // 96 min at 2 s interval
static const uint16_t COMPACT_MARGIN_RECORDS = 80;

static const uint8_t I2C_SDA_PIN = 9;
static const uint8_t I2C_SCL_PIN = 10;
static const uint8_t SENSOR_POWER_PIN = 0;  // FireBeetle 3V3_C control
static const uint8_t BATTERY_ADC_PIN = 1;   // FireBeetle battery detect pin
static const uint8_t STATUS_LED_PIN = 15;   // FireBeetle onboard LED: 15 / D13

static const uint8_t PRESENCE_WINDOW = 30;
static const float PRESENCE_FACTOR = 1.30f;
static const uint8_t PRESENCE_SAMPLES_PER_WAKE = 5;
static const uint16_t PRESENCE_SAMPLE_INTERVAL_MS = 1000;

static const char *HISTORY_PATH = "/weather.csv";

// Adjust this after measuring VBAT with a multimeter.
static const float BATTERY_DIVIDER_FACTOR = 2.0f;

// ===================== Runtime state =====================
WebServer server(80);
Adafruit_BME280 bme;

struct WeatherRecord {
  time_t epoch;
  float temperature;
  float humidity;
  float pressure;
  float battery;
  int rssi;
  float rssiStdDev;
  bool presence;
  bool calibrated;
  uint8_t channel;
  bool wifiConnected;
};

WeatherRecord latest = {};
uint32_t webDeadlineMs = 0;
uint32_t bootStartedMs = 0;

RTC_DATA_ATTR int8_t rssiBuffer[PRESENCE_WINDOW];
RTC_DATA_ATTR uint8_t rssiIndex = 0;
RTC_DATA_ATTR uint8_t rssiSamples = 0;
RTC_DATA_ATTR bool presenceCalibrated = false;
RTC_DATA_ATTR float baselineRssiStdDev = 0.0f;
RTC_DATA_ATTR uint16_t historyRecordsCached = 0;
RTC_DATA_ATTR uint32_t bootCounter = 0;

// ===================== Helpers =====================
void beginSerial() {
  Serial.begin(115200);
  const uint32_t started = millis();
  while (!Serial && millis() - started < 5000) {
    delay(10);
  }
  delay(300);
  Serial.println();
  Serial.println("========================================");
  Serial.println("FireBeetle 2 ESP32-C5 weather station");
  Serial.println("Serial OK at 115200 baud");
  Serial.println("========================================");
  Serial.flush();
}

float calculateStdDev(const int8_t *buffer, uint8_t len) {
  if (len < 2) return 0.0f;

  float mean = 0.0f;
  for (uint8_t i = 0; i < len; i++) mean += buffer[i];
  mean /= len;

  float variance = 0.0f;
  for (uint8_t i = 0; i < len; i++) {
    const float diff = buffer[i] - mean;
    variance += diff * diff;
  }
  return sqrtf(variance / len);
}

void sensorPower(bool enabled) {
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, enabled ? HIGH : LOW);
  if (!enabled) {
    pinMode(I2C_SDA_PIN, INPUT);
    pinMode(I2C_SCL_PIN, INPUT);
  }
}

void configure5GHzPreference() {
  if (!FORCE_5GHZ_WIFI) return;

#if defined(WIFI_BAND_MODE_5G_ONLY)
  const esp_err_t err = esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY);
  if (err != ESP_OK) {
    Serial.printf("5 GHz band mode failed: %s\n", esp_err_to_name(err));
  } else {
    Serial.println("WiFi band: 5 GHz only");
  }
#else
  Serial.println("5 GHz band API not available in this Arduino core");
#endif
}

bool connectWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(LOW_POWER_SLEEP);
  configure5GHzPreference();

  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection failed");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.printf("RSSI: %d dBm, channel: %u%s\n",
                WiFi.RSSI(),
                WiFi.channel(),
                WiFi.channel() >= 36 ? " (5 GHz)" : " (2.4 GHz)");
  return true;
}

void syncTimeBriefly() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  const uint32_t started = millis();
  while (time(nullptr) < 1700000000 && millis() - started < 1500) {
    delay(100);
  }
}

float readBatteryVoltage() {
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
  const uint16_t raw = analogRead(BATTERY_ADC_PIN);
  const float pinVoltage = (raw / 4095.0f) * 3.3f;
  return pinVoltage * BATTERY_DIVIDER_FACTOR;
}

bool readBme280(WeatherRecord &record) {
  sensorPower(true);
  delay(20);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  bool ok = bme.begin(0x76, &Wire);
  if (!ok) ok = bme.begin(0x77, &Wire);

  if (!ok) {
    Serial.println("BME280 not found at 0x76 or 0x77");
    sensorPower(false);
    return false;
  }

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::FILTER_OFF);
  bme.takeForcedMeasurement();

  record.temperature = bme.readTemperature();
  record.humidity = bme.readHumidity();
  record.pressure = bme.readPressure() / 100.0f;
  record.battery = readBatteryVoltage();

  sensorPower(false);
  return true;
}

void pushRssiSample(int8_t rssi) {
  rssiBuffer[rssiIndex] = rssi;
  rssiIndex = (rssiIndex + 1) % PRESENCE_WINDOW;
  if (rssiSamples < PRESENCE_WINDOW) rssiSamples++;

  if (!presenceCalibrated && rssiSamples == PRESENCE_WINDOW) {
    baselineRssiStdDev = calculateStdDev(rssiBuffer, PRESENCE_WINDOW);
    presenceCalibrated = true;
    Serial.printf("Presence baseline calibrated: %.2f dBm\n", baselineRssiStdDev);
  }
}

void updatePresence(WeatherRecord &record) {
  if (WiFi.status() != WL_CONNECTED) return;

  const uint8_t samples = LOW_POWER_SLEEP ? PRESENCE_SAMPLES_PER_WAKE : 1;
  for (uint8_t i = 0; i < samples; i++) {
    const int8_t rssi = WiFi.RSSI();
    pushRssiSample(rssi);
    record.rssi = rssi;
    if (i + 1 < samples) delay(PRESENCE_SAMPLE_INTERVAL_MS);
  }

  record.rssiStdDev = calculateStdDev(rssiBuffer, rssiSamples);
  record.calibrated = presenceCalibrated;
  record.presence = presenceCalibrated && baselineRssiStdDev > 0.0f &&
                    record.rssiStdDev > baselineRssiStdDev * PRESENCE_FACTOR;
}

uint16_t countHistoryRecords() {
  File file = LittleFS.open(HISTORY_PATH, "r");
  if (!file) return 0;

  uint16_t count = 0;
  while (file.available()) {
    if (file.read() == '\n') count++;
  }
  file.close();
  return count;
}

void appendHistory(const WeatherRecord &record) {
  File file = LittleFS.open(HISTORY_PATH, "a");
  if (!file) {
    Serial.println("Could not open history file for append");
    return;
  }

  file.printf("%ld,%.2f,%.2f,%.2f,%.2f,%d,%.3f,%u,%u,%u\n",
              static_cast<long>(record.epoch),
              record.temperature,
              record.humidity,
              record.pressure,
              record.battery,
              record.rssi,
              record.rssiStdDev,
              record.presence ? 1 : 0,
              record.calibrated ? 1 : 0,
              record.channel);
  file.close();
  historyRecordsCached++;
}

void compactHistoryIfNeeded() {
  if (historyRecordsCached <= MAX_HISTORY_RECORDS + COMPACT_MARGIN_RECORDS) return;

  File in = LittleFS.open(HISTORY_PATH, "r");
  File out = LittleFS.open("/weather.tmp", "w");
  if (!in || !out) {
    if (in) in.close();
    if (out) out.close();
    return;
  }

  const uint16_t skip = historyRecordsCached - MAX_HISTORY_RECORDS;
  uint16_t line = 0;
  while (in.available()) {
    const String row = in.readStringUntil('\n');
    if (line++ >= skip && row.length() > 0) out.println(row);
  }
  in.close();
  out.close();

  LittleFS.remove(HISTORY_PATH);
  LittleFS.rename("/weather.tmp", HISTORY_PATH);
  historyRecordsCached = MAX_HISTORY_RECORDS;
  Serial.println("History compacted");
}

String latestJson() {
  String json;
  json.reserve(256);
  json += F("{\"epoch\":");
  json += static_cast<long>(latest.epoch);
  json += F(",\"temperature\":");
  json += String(latest.temperature, 2);
  json += F(",\"humidity\":");
  json += String(latest.humidity, 2);
  json += F(",\"pressure\":");
  json += String(latest.pressure, 2);
  json += F(",\"battery\":");
  json += String(latest.battery, 2);
  json += F(",\"rssi\":");
  json += latest.rssi;
  json += F(",\"rssiStdDev\":");
  json += String(latest.rssiStdDev, 3);
  json += F(",\"presence\":");
  json += (latest.presence ? F("true") : F("false"));
  json += F(",\"calibrated\":");
  json += (latest.calibrated ? F("true") : F("false"));
  json += F(",\"channel\":");
  json += latest.channel;
  json += F(",\"wifiConnected\":");
  json += (latest.wifiConnected ? F("true") : F("false"));
  json += F(",\"baselineStdDev\":");
  json += String(baselineRssiStdDev, 3);
  json += F(",\"historyRecords\":");
  json += historyRecordsCached;
  json += F("}");
  return json;
}

void streamHistoryJson() {
  File file = LittleFS.open(HISTORY_PATH, "r");
  if (!file) {
    server.send(200, "application/json", "[]");
    return;
  }

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent("[");
  bool first = true;
  while (file.available()) {
    const String row = file.readStringUntil('\n');
    if (row.length() < 10) continue;

    long epoch;
    float t, h, p, b, stddev;
    int rssi;
    unsigned int presence, calibrated, channel;
    const int parsed = sscanf(row.c_str(), "%ld,%f,%f,%f,%f,%d,%f,%u,%u,%u",
                              &epoch, &t, &h, &p, &b, &rssi, &stddev,
                              &presence, &calibrated, &channel);
    if (parsed != 10) continue;

    if (!first) server.sendContent(",");
    first = false;
    String item;
    item.reserve(128);
    item += F("{\"e\":");
    item += epoch;
    item += F(",\"t\":");
    item += String(t, 2);
    item += F(",\"h\":");
    item += String(h, 2);
    item += F(",\"p\":");
    item += String(p, 2);
    item += F(",\"b\":");
    item += String(b, 2);
    item += F(",\"r\":");
    item += rssi;
    item += F(",\"s\":");
    item += String(stddev, 3);
    item += F(",\"m\":");
    item += (presence ? F("1") : F("0"));
    item += F(",\"c\":");
    item += (calibrated ? F("1") : F("0"));
    item += F(",\"ch\":");
    item += channel;
    item += '}';
    server.sendContent(item);
  }
  file.close();
  server.sendContent("]");
  server.sendContent("");
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Solar Wetterstation</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js"></script>
<style>
:root{color-scheme:dark;--bg:#101419;--panel:#171e25;--line:#2b3642;--text:#e8edf2;--muted:#8d9aa7;--green:#58d68d;--red:#ff6b6b;--blue:#6cb6ff;--yellow:#ffd166}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,sans-serif}
header{padding:18px 18px 10px;border-bottom:1px solid var(--line)}h1{margin:0 0 6px;font-size:1.4rem;font-weight:650}.sub{color:var(--muted);font-size:.9rem}
main{padding:16px;max-width:1200px;margin:auto}.metrics{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin-bottom:16px}
.metric{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:12px;min-height:86px}.metric span{display:block;color:var(--muted);font-size:.78rem}.metric strong{display:block;font-size:1.45rem;margin-top:8px}
.presence strong{color:var(--green)}.presence.off strong{color:var(--muted)}.chart{height:260px;background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:12px;margin-bottom:12px}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:0 0 12px}.pill{border:1px solid var(--line);border-radius:999px;padding:6px 10px;color:var(--muted);font-size:.84rem}
button{border:1px solid var(--line);background:#1e2933;color:var(--text);border-radius:6px;padding:8px 10px;cursor:pointer}button:hover{background:#263442}
</style>
</head>
<body>
<header><h1>Solar Wetterstation</h1><div class="sub">FireBeetle 2 ESP32-C5 · BME280 · 5 GHz RSSI-Präsenzerkennung</div></header>
<main>
<div class="metrics">
<div class="metric"><span>Temperatur</span><strong id="temp">--</strong></div>
<div class="metric"><span>Luftfeuchte</span><strong id="hum">--</strong></div>
<div class="metric"><span>Luftdruck</span><strong id="press">--</strong></div>
<div class="metric"><span>Batterie</span><strong id="bat">--</strong></div>
<div class="metric"><span>RSSI</span><strong id="rssi">--</strong></div>
<div class="metric presence off" id="presenceBox"><span>Präsenz</span><strong id="presence">--</strong></div>
</div>
<div class="row">
<span class="pill" id="status">Lade Daten...</span>
<span class="pill" id="samples">-- Messpunkte</span>
<span class="pill" id="channel">--</span>
<button onclick="loadData()">Aktualisieren</button>
</div>
<div class="chart"><canvas id="weatherChart"></canvas></div>
<div class="chart"><canvas id="presenceChart"></canvas></div>
</main>
<script>
let weatherChart,presenceChart;
const fmtTime=e=>e>1700000000?new Date(e*1000).toLocaleTimeString([], {hour:'2-digit',minute:'2-digit'}):'Boot '+e;
function makeChart(id,datasets,yTitle){
  return new Chart(document.getElementById(id),{type:'line',data:{labels:[],datasets},options:{responsive:true,maintainAspectRatio:false,animation:false,interaction:{mode:'index',intersect:false},plugins:{legend:{labels:{color:'#cbd5df'}}},scales:{x:{ticks:{color:'#8d9aa7',maxTicksLimit:8},grid:{color:'#26313c'}},y:{title:{display:true,text:yTitle,color:'#8d9aa7'},ticks:{color:'#8d9aa7'},grid:{color:'#26313c'}}}}});
}
async function loadData(){
  const [latest,history]=await Promise.all([fetch('/api/latest').then(r=>r.json()),fetch('/api/history').then(r=>r.json())]);
  temp.textContent=latest.temperature.toFixed(1)+' °C'; hum.textContent=latest.humidity.toFixed(0)+' %'; press.textContent=latest.pressure.toFixed(0)+' hPa';
  bat.textContent=latest.battery.toFixed(2)+' V'; rssi.textContent=latest.rssi+' dBm';
  presence.textContent=latest.calibrated?(latest.presence?'Erkannt':'Nein'):'Kalibriert...';
  presenceBox.classList.toggle('off',!latest.presence);
  status.textContent=latest.wifiConnected?'WLAN verbunden':'WLAN offline';
  samples.textContent=history.length+' Messpunkte';
  channel.textContent='Kanal '+latest.channel+(latest.channel>=36?' · 5 GHz':' · 2.4 GHz');
  const labels=history.map(x=>fmtTime(x.e));
  if(!weatherChart){
    weatherChart=makeChart('weatherChart',[
      {label:'°C',data:[],borderColor:'#ff6b6b',pointRadius:0,yAxisID:'y'},
      {label:'% rF',data:[],borderColor:'#58d68d',pointRadius:0,yAxisID:'y'},
      {label:'hPa / 10',data:[],borderColor:'#6cb6ff',pointRadius:0,yAxisID:'y'}
    ],'Wetter');
    presenceChart=makeChart('presenceChart',[
      {label:'RSSI dBm',data:[],borderColor:'#ffd166',pointRadius:0},
      {label:'StdDev',data:[],borderColor:'#c792ea',pointRadius:0},
      {label:'Präsenz',data:[],borderColor:'#58d68d',stepped:true,pointRadius:0}
    ],'RSSI / Präsenz');
  }
  weatherChart.data.labels=labels;
  weatherChart.data.datasets[0].data=history.map(x=>x.t);
  weatherChart.data.datasets[1].data=history.map(x=>x.h);
  weatherChart.data.datasets[2].data=history.map(x=>x.p/10);
  presenceChart.data.labels=labels;
  presenceChart.data.datasets[0].data=history.map(x=>x.r);
  presenceChart.data.datasets[1].data=history.map(x=>x.s);
  presenceChart.data.datasets[2].data=history.map(x=>x.m?0:-100);
  weatherChart.update(); presenceChart.update();
}
loadData(); setInterval(loadData,2000);
</script>
</body>
</html>
)HTML";

void handleRoot() {
  webDeadlineMs = millis() + WEB_WINDOW_WHEN_CLIENT_MS;
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleLatest() {
  webDeadlineMs = millis() + WEB_WINDOW_WHEN_CLIENT_MS;
  server.send(200, "application/json", latestJson());
}

void handleHistory() {
  webDeadlineMs = millis() + WEB_WINDOW_WHEN_CLIENT_MS;
  streamHistoryJson();
}

void startWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/latest", HTTP_GET, handleLatest);
  server.on("/api/history", HTTP_GET, handleHistory);
  server.begin();
  webDeadlineMs = millis() + WEB_WINDOW_MS;
  Serial.println("Webserver started");
  Serial.print("Open dashboard: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
}

void enterSleep() {
  server.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  sensorPower(false);

  const uint32_t awakeSeconds = (millis() - bootStartedMs + 999) / 1000;
  const uint32_t sleepSeconds =
      awakeSeconds >= MEASUREMENT_INTERVAL_SECONDS ? 1 : MEASUREMENT_INTERVAL_SECONDS - awakeSeconds;

  Serial.printf("Deep sleep for %lu s\n", static_cast<unsigned long>(sleepSeconds));
  Serial.flush();
  esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(sleepSeconds) * 1000000ULL);
  esp_deep_sleep_start();
}

void setup() {
  bootStartedMs = millis();
  bootCounter++;
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  beginSerial();
  Serial.printf("\nSolar Wetterstation boot #%lu\n", static_cast<unsigned long>(bootCounter));
  Serial.println("If you can read this, the uploaded sketch is running.");

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  } else if (historyRecordsCached == 0) {
    historyRecordsCached = countHistoryRecords();
  }

  latest = {};
  latest.epoch = bootCounter;
  const bool bmeOk = readBme280(latest);
  latest.wifiConnected = connectWiFi();

  if (latest.wifiConnected) {
    syncTimeBriefly();
    const time_t now = time(nullptr);
    if (now > 1700000000) latest.epoch = now;
    latest.channel = WiFi.channel();
    latest.rssi = WiFi.RSSI();
    updatePresence(latest);
  }

  if (bmeOk) {
    appendHistory(latest);
    compactHistoryIfNeeded();
  }

  Serial.printf("T %.2f C, H %.2f %%, P %.2f hPa, VBAT %.2f V, RSSI %d dBm, StdDev %.2f, presence %s\n",
                latest.temperature,
                latest.humidity,
                latest.pressure,
                latest.battery,
                latest.rssi,
                latest.rssiStdDev,
                latest.presence ? "yes" : "no");

  if (latest.wifiConnected) {
    startWebServer();
  } else if (LOW_POWER_SLEEP) {
    enterSleep();
  }
}

void loop() {
  if (latest.wifiConnected) {
    server.handleClient();
  }

  if (!LOW_POWER_SLEEP) {
    static uint32_t lastMeasurementMs = 0;
    static uint32_t lastReconnectMs = 0;

    if (!latest.wifiConnected && millis() - lastReconnectMs >= 10000) {
      lastReconnectMs = millis();
      latest.wifiConnected = connectWiFi();
      if (latest.wifiConnected) {
        syncTimeBriefly();
        startWebServer();
      }
    }

    if (millis() - lastMeasurementMs >= MEASUREMENT_INTERVAL_SECONDS * 1000UL) {
      lastMeasurementMs = millis();
      const bool bmeOk = readBme280(latest);
      const time_t now = time(nullptr);
      latest.epoch = now > 1700000000 ? now : bootCounter;

      if (latest.wifiConnected) {
        latest.channel = WiFi.channel();
        latest.rssi = WiFi.RSSI();
        if (WiFi.status() != WL_CONNECTED) {
          latest.wifiConnected = false;
        } else {
          updatePresence(latest);
        }
      }

      if (bmeOk) {
        appendHistory(latest);
        compactHistoryIfNeeded();
      }

      Serial.printf("T %.2f C, H %.2f %%, P %.2f hPa, VBAT %.2f V, RSSI %d dBm, StdDev %.2f, presence %s, IP %s\n",
                    latest.temperature,
                    latest.humidity,
                    latest.pressure,
                    latest.battery,
                    latest.rssi,
                    latest.rssiStdDev,
                    latest.presence ? "yes" : "no",
                    latest.wifiConnected ? WiFi.localIP().toString().c_str() : "offline");
      digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    }
    delay(2);
    return;
  }

  if (millis() > webDeadlineMs) {
    enterSleep();
  }
  delay(2);
}
