/*
 * NodeMCU V3 (ESP8266MOD) Sensor Dashboard
 * FIX: AJAX Polling statt SSE (robuster auf ESP8266)
 * Entfernt: Hinweis-Box
 * 
 * Sensoren:
 *   - AM2302 (DHT22): Raumtemperatur & Luftfeuchtigkeit (GPIO 14 / D5)
 *   - ADC (A0): Analoger Sensor
 *   - WiFi RSSI: Signalstärke + Präsenzerkennung
 */

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DHT.h>
#include <Arduino_JSON.h>

// ============== KONFIGURATION ==============
const char* ssid = "DEIN_WLAN_NAME";
const char* password = "DEIN_WLAN_PASSWORT";

#define DHTPIN 14       // GPIO 14 = D5 auf NodeMCU
#define DHTTYPE DHT22
#define ADC_PIN A0

#define SENSOR_INTERVAL 2000
#define PRESENCE_WINDOW 30
#define PRESENCE_FACTOR 1.3

// ============== GLOBALE VARIABELN ==============
AsyncWebServer server(80);
DHT dht(DHTPIN, DHTTYPE);

int8_t rssiBuffer[PRESENCE_WINDOW];
int rssiIndex = 0;
bool bufferFull = false;
float baselineStdDev = 0;
bool isCalibrated = false;
bool presenceDetected = false;

unsigned long lastSensorRead = 0;

float roomTemp = 0, humidity = 0, adcVoltage = 0;
int adcValue = 0;
int8_t rssi = 0;

// ============== HTML DASHBOARD (AJAX POLLING) ==============
const char index_html[] PROGMEM = 
"<!DOCTYPE html>"
"<html lang='de'>"
"<head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
"<title>NodeMCU V3 Sensor Dashboard</title>"
"<script src='https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js'></script>"
"<style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{font-family:'Segoe UI',sans-serif;background:linear-gradient(135deg,#1a1a2e,#16213e);color:#e0e0e0;min-height:100vh;padding:20px}"
"h1{text-align:center;margin-bottom:10px;color:#00d4ff;font-size:2rem}"
".subtitle{text-align:center;color:#888;margin-bottom:25px;font-size:.9rem}"
".status-bar{display:flex;justify-content:center;gap:20px;margin-bottom:25px;flex-wrap:wrap}"
".status-item{background:rgba(255,255,255,.05);padding:10px 20px;border-radius:20px;font-size:.85rem;border:1px solid rgba(255,255,255,.1)}"
".status-item .label{color:#888}.status-item .value{color:#00d4ff;font-weight:bold}"
"#presenceIndicator{display:inline-block;width:12px;height:12px;border-radius:50%;background:#333;margin-left:8px;transition:all .3s}"
"#presenceIndicator.active{background:#0f8;box-shadow:0 0 12px #0f8}"
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:20px;max-width:1400px;margin:0 auto}"
".card{background:rgba(255,255,255,.03);border-radius:16px;padding:20px;border:1px solid rgba(255,255,255,.08)}"
".card h3{font-size:.9rem;color:#aaa;margin-bottom:15px}"
".current-value{font-size:2rem;font-weight:bold;color:#fff;margin-bottom:10px}"
".current-value .unit{font-size:1rem;color:#888;font-weight:normal}"
"canvas{max-height:200px}"
".footer{text-align:center;margin-top:30px;color:#555;font-size:.8rem}"
"@media(max-width:600px){h1{font-size:1.4rem}.grid{grid-template-columns:1fr}}"
"</style>"
"</head>"
"<body>"
"<h1>NodeMCU V3 Sensor Dashboard</h1>"
"<p class='subtitle'>ESP8266MOD | Echtzeit-Umgebungsmonitoring & WLAN-Prasenzerkennung</p>"
"<div class='status-bar'>"
"<div class='status-item'><span class='label'>Verbindung:</span><span class='value' id='connStatus'>Lade...</span></div>"
"<div class='status-item'><span class='label'>RSSI:</span><span class='value' id='rssiDisplay'>-- dBm</span></div>"
"<div class='status-item'><span class='label'>Prasenz:</span><span class='value' id='presenceDisplay'>Kalibrierung...</span><span id='presenceIndicator'></span></div>"
"</div>"
"<div class='grid'>"
"<div class='card'><h3>Raumtemperatur (AM2302)</h3><div class='current-value' id='valRoomTemp'>--<span class='unit'> C</span></div><canvas id='chartRoomTemp'></canvas></div>"
"<div class='card'><h3>Luftfeuchtigkeit (AM2302)</h3><div class='current-value' id='valHumidity'>--<span class='unit'> %</span></div><canvas id='chartHumidity'></canvas></div>"
"<div class='card'><h3>Analoger Sensor (ADC A0)</h3><div class='current-value' id='valAdc'>--<span class='unit'> /1023</span></div><canvas id='chartAdc'></canvas></div>"
"<div class='card'><h3>ADC Spannung (A0)</h3><div class='current-value' id='valVoltage'>--<span class='unit'> V</span></div><canvas id='chartVoltage'></canvas></div>"
"<div class='card'><h3>RSSI & Prasenz</h3><div class='current-value' id='valRssi'>--<span class='unit'> dBm</span></div><canvas id='chartRssi'></canvas></div>"
"<div class='card'><h3>WiFi Signalqualitat</h3><div class='current-value' id='valQuality'>--<span class='unit'> %</span></div><canvas id='chartQuality'></canvas></div>"
"</div>"
"<div class='footer'>NodeMCU V3 (ESP8266MOD) Web Science Projekt | AJAX Polling | Chart.js</div>"
"<script>"
"const chartConfig=(l,c,mn,mx)=>({type:'line',data:{labels:[],datasets:[{label:l,data:[],borderColor:c,backgroundColor:c+'20',borderWidth:2,pointRadius:0,pointHoverRadius:4,tension:.4,fill:true}]},options:{responsive:true,maintainAspectRatio:false,animation:false,interaction:{intersect:false,mode:'index'},plugins:{legend:{display:false}},scales:{x:{display:false},y:{min:mn,max:mx,grid:{color:'rgba(255,255,255,.05)'},ticks:{color:'#888',font:{size:10}}}}}});"
"const charts={"
"roomTemp:new Chart(document.getElementById('chartRoomTemp'),chartConfig('C','#ff6b6b',10,40)),"
"humidity:new Chart(document.getElementById('chartHumidity'),chartConfig('%','#4ecdc4',0,100)),"
"adc:new Chart(document.getElementById('chartAdc'),chartConfig('Wert','#a8e6cf',0,1023)),"
"voltage:new Chart(document.getElementById('chartVoltage'),chartConfig('V','#ffe66d',0,1.1)),"
"rssi:new Chart(document.getElementById('chartRssi'),chartConfig('dBm','#ff8b94',-90,-30)),"
"quality:new Chart(document.getElementById('chartQuality'),chartConfig('%','#c7ceea',0,100))"
"};"
"const MAX_POINTS=50;"
"function updateChart(ch,v){"
"const t=new Date();"
"const tm=t.getHours().toString().padStart(2,'0')+':'+t.getMinutes().toString().padStart(2,'0')+':'+t.getSeconds().toString().padStart(2,'0');"
"ch.data.labels.push(tm);ch.data.datasets[0].data.push(v);"
"if(ch.data.labels.length>MAX_POINTS){ch.data.labels.shift();ch.data.datasets[0].data.shift();}"
"ch.update('none');"
"}"
"function rssiToQuality(r){if(r>=-50)return 100;if(r<=-100)return 0;return 2*(r+100);}"
"const cs=document.getElementById('connStatus');"
"const pi=document.getElementById('presenceIndicator');"
"const pd=document.getElementById('presenceDisplay');"
"const rd=document.getElementById('rssiDisplay');"
"let lastDataTime=0;"
"function updateData(){"
"fetch('/api/data').then(r=>r.json()).then(d=>{"
"cs.textContent='Verbunden';cs.style.color='#0f8';lastDataTime=Date.now();"
"document.getElementById('valRoomTemp').innerHTML=d.roomTemp.toFixed(1)+'<span class=\"unit\"> C</span>';"
"document.getElementById('valHumidity').innerHTML=d.humidity.toFixed(1)+'<span class=\"unit\"> %</span>';"
"document.getElementById('valAdc').innerHTML=d.adc+'<span class=\"unit\"> /1023</span>';"
"document.getElementById('valVoltage').innerHTML=d.voltage.toFixed(3)+'<span class=\"unit\"> V</span>';"
"document.getElementById('valRssi').innerHTML=d.rssi+'<span class=\"unit\"> dBm</span>';"
"document.getElementById('valQuality').innerHTML=Math.round(rssiToQuality(d.rssi))+'<span class=\"unit\"> %</span>';"
"rd.textContent=d.rssi+' dBm';"
"if(d.calibrated){"
"if(d.presence){pd.textContent='ERKANNT';pd.style.color='#0f8';pi.classList.add('active');}"
"else{pd.textContent='Keine Bewegung';pd.style.color='#888';pi.classList.remove('active');}"
"}else{pd.textContent='Kalibrierung... ('+d.calProgress+'%)';pd.style.color='#ffe66d';}"
"updateChart(charts.roomTemp,d.roomTemp);updateChart(charts.humidity,d.humidity);"
"updateChart(charts.adc,d.adc);updateChart(charts.voltage,d.voltage);"
"updateChart(charts.rssi,d.rssi);updateChart(charts.quality,rssiToQuality(d.rssi));"
"}).catch(e=>{"
"cs.textContent='Fehler';cs.style.color='#f66';"
"console.error('Fetch Fehler:',e);"
"});"
"}"
"setInterval(updateData,2000);"
"updateData();"
"setInterval(function(){"
"if(Date.now()-lastDataTime>5000){cs.textContent='Getrennt';cs.style.color='#f66';}"
"},3000);"
"</script>"
"</body>"
"</html>";

// ============== FUNKTIONEN ==============

float calculateStdDev(int8_t buffer[], int len) {
    if (len < 2) return 0;
    float mean = 0;
    for (int i = 0; i < len; i++) mean += buffer[i];
    mean /= len;
    float variance = 0;
    for (int i = 0; i < len; i++) variance += pow(buffer[i] - mean, 2);
    return sqrt(variance / len);
}

void updatePresence() {
    rssi = WiFi.RSSI();
    rssiBuffer[rssiIndex] = rssi;
    rssiIndex = (rssiIndex + 1) % PRESENCE_WINDOW;
    if (rssiIndex == 0) bufferFull = true;
    int samples = bufferFull ? PRESENCE_WINDOW : rssiIndex;
    if (!isCalibrated) {
        if (bufferFull) {
            baselineStdDev = calculateStdDev(rssiBuffer, PRESENCE_WINDOW);
            isCalibrated = true;
            Serial.println("[Prasenz] Kalibriert. Baseline: " + String(baselineStdDev));
        }
        return;
    }
    float currentStdDev = calculateStdDev(rssiBuffer, samples);
    presenceDetected = (currentStdDev > baselineStdDev * PRESENCE_FACTOR);
}

String buildJSON() {
    JSONVar doc;
    doc["roomTemp"] = roomTemp;
    doc["humidity"] = humidity;
    doc["adc"] = adcValue;
    doc["voltage"] = adcVoltage;
    doc["rssi"] = rssi;
    doc["presence"] = presenceDetected;
    doc["calibrated"] = isCalibrated;
    doc["calProgress"] = bufferFull ? 100 : (rssiIndex * 100 / PRESENCE_WINDOW);
    return JSON.stringify(doc);
}

void readSensors() {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
        humidity = h;
        roomTemp = t;
    }
    adcValue = analogRead(ADC_PIN);
    adcVoltage = adcValue * (3.3 / 1023.0);
    updatePresence();
}

// ============== SETUP ==============
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== NodeMCU V3 Sensor Dashboard ===");
    Serial.println("(ESP8266MOD) - AJAX Polling Version");

    dht.begin();

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.print("WLAN-Verbindung");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nIP: " + WiFi.localIP().toString());

    // CORS Header fuer AJAX
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

    // Web-Root
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", index_html);
    });

    // API-Endpunkt fuer AJAX
    server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "application/json", buildJSON());
    });

    server.begin();
    Serial.println("Webserver gestartet");
    Serial.println("Kalibrierung: Raum verlassen (60 Sek.)");
}

// ============== LOOP ==============
void loop() {
    if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
        lastSensorRead = millis();

        readSensors();

        Serial.print("T:" + String(roomTemp, 1) + " H:" + String(humidity, 1) + " ");
        Serial.print("ADC:" + String(adcValue) + " V:" + String(adcVoltage, 3) + " ");
        Serial.print("RSSI:" + String(rssi) + " ");
        Serial.println("Prasenz:" + String(presenceDetected ? "JA" : "NEIN"));
    }

    if (WiFi.status() != WL_CONNECTED) {
        WiFi.reconnect();
    }
}
 String(roomTemp, 1) + " H:" + String(humidity, 1) + " ");
        Serial.print("ADC:" + String(adcValue) + " V:" + String(adcVoltage, 3) + " ");
        Serial.print("RSSI:" + String(rssi) + " ");
        Serial.println("Prasenz:" + String(presenceDetected ? "JA" : "NEIN"));
    }

    if (WiFi.status() != WL_CONNECTED) {
        WiFi.reconnect();
    }
}
