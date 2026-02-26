#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <PCF8574.h>
#include <RTClib.h>

// Library Firebase
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// =================== 1. KONFIGURASI WIFI & FIREBASE ====================
#include <EEPROM.h>

#define MAX_WIFI 5
#define SSID_LEN 32
#define PASS_LEN 32
#define EEPROM_MAGIC 0xCAFE

struct WifiCred {
  char ssid[SSID_LEN];
  char pass[PASS_LEN];
};

struct EEPROMData {
  uint16_t magic;
  WifiCred wifiList[MAX_WIFI];
  uint16_t checksum;
};

WifiCred wifiList[MAX_WIFI];

// --- FIREBASE RTDB ---
#define FIREBASE_HOST "https://monitoring-tomat-8c584-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "tMWgotr8VHt0nuwG9hQWpdexx9LCpuKXz47n5kPr"

// --- FIREBASE FIRESTORE (Untuk Kalibrasi & Config) ---
#define PROJECT_ID "monitoring-tomat-8c584"
#define API_KEY    "AIzaSyBWMHCMOY7_o94NHQIx7LJKtkJsn7hluGw"
#define FIRESTORE_CALIB_URL \
"https://firestore.googleapis.com/v1/projects/" PROJECT_ID \
"/databases/(default)/documents/Kalibrasi/SensorOffset?key=" API_KEY
#define FIRESTORE_CONFIG_URL \
"https://firestore.googleapis.com/v1/projects/" PROJECT_ID \
"/databases/(default)/documents/Config/MainConfig?key=" API_KEY
#define FIRESTORE_LOGS_URL \
"https://firestore.googleapis.com/v1/projects/" PROJECT_ID \
"/databases/(default)/documents/logs/"

// =================== 2. PIN HARDWARE & KONSTANTA ===================
#define DHTPIN  D4
#define DHTTYPE DHT22
#define SOIL_PIN A0
#define FLOW_PIN D5

// Ultrasonic Air (T1)
#define TRIG_AIR D3
#define ECHO_AIR D7

// Ultrasonic Pestisida (T2)
#define TRIG_PEST D0
#define ECHO_PEST D6

// PCF8574 (Relay Active LOW)
#define RELAY_WATER_PIN 0
#define RELAY_PEST_PIN  1
PCF8574 pcf(0x20);

// Ultrasonic Calibration
const float FULL_LEVEL  = 3.0;    // Penuh jika <= 3 cm
const float EMPTY_THRESHOLD = 44.0; // Habis jika >= 44 cm
const float OFFSET_AIR   = -0.8;
const float OFFSET_PEST = -1.6;

// =================== 3. OBJEK & VARIABEL GLOBAL ===================
DHT dht(DHTPIN, DHTTYPE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
RTC_DS3231 rtc;

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// WiFi State Management
bool systemOnline = false;
bool wifiScanned = false;
unsigned long lastWiFiCheck = 0;
unsigned long lastReconnectAttempt = 0;
unsigned long reconnectDelay = 10000;

// Data Sensor
float suhuUdara       = 0.0;
float kelembapanUdara = 0.0;
int kelembapanTanah   = 0;
float litersWaterToday = 0.0;
float litersPestToday  = 0.0;
String tangkiAirStatus  = "Error";
String tangkiPestStatus = "Error";
int umurTanamanDays  = 0;

// Data Kontrol RTDB
String systemCondition = "Manual";
String targetWater     = "OFF";
String targetPestisida = "OFF";

// Kalibrasi dari Firestore
float dhtTempOffsetF   = 0;
float hygroTempOffsetF = 0;
float dhtHumOffsetF    = 0;
float hygroHumOffsetF  = 0;
int soilMeterOffset   = 0;
int soilOffset        = 0;
bool isCalibrated     = false;
bool calibSystemReady = false;
bool calibDataLoaded = false;

// Config dari Firestore
int setPointSuhuMin = 20;
int setPointSuhuMax = 27;
int setPointHumidityMin = 60;
int setPointHumidityMax = 80;
int setPointSoilMin = 5;
int setPointSoilMax = 8;
String tanggalDitanam = "";

// Flow Meter
volatile unsigned long pulseCount;
float flowRate = 0.0;

// =================== RELAY STATE MACHINE ===================
// Desain: hanya SATU relay boleh ACTIVE sekaligus.
// Jika relay lain sedang aktif, request masuk ke PENDING.
// Saat relay selesai, processRelayQueue() otomatis jalankan yang PENDING.

enum RelayState : uint8_t {
  RS_IDLE    = 0,  // Mati
  RS_ACTIVE  = 1,  // Sedang menyala & memompa
  RS_PENDING = 2   // Menunggu giliran (relay lain sedang aktif)
};

enum RelaySource : uint8_t {
  SRC_NONE   = 0,
  SRC_MANUAL = 1,
  SRC_AUTO   = 2
};

struct RelayCtrl {
  RelayState    state;
  RelaySource   source;
  float         targetVolumeMl;   // 0 = tidak ada batas (Manual)
  float         volumeMl;         // Volume terpompa sesi ini
  unsigned long startMs;
  unsigned long timeoutMs;
  // Antrian
  bool          hasPending;
  RelaySource   pendingSrc;
  float         pendingVolumeMl;
  // Flow watchdog
  unsigned long lastPulseMs;
  bool          watchdogTriggered;
  // Hardware
  uint8_t       hwPin;
};

#define RELAY_WATER  0
#define RELAY_PEST   1
#define RELAY_COUNT  2

RelayCtrl relays[RELAY_COUNT];

const float          TARGET_WATER_ML   = 250.0;
const float          TARGET_PEST_ML    = 200.0;
const unsigned long  PUMP_TIMEOUT_MS   = 300000UL; // 5 menit
const unsigned long  FLOW_WATCHDOG_MS  = 5000UL;   // 5 detik tanpa pulse = error

// Variabel untuk deteksi edge perintah Manual (cegah request berulang tiap 1 detik)
String prevWaterCmd = "OFF";
String prevPestCmd  = "OFF";

// Pestisida Tracking
int pestTundaDays = 0;
String lastPestDate = "";
bool pestScheduledToday = false;

// Timers
unsigned long lastRTDBRead = 0;
unsigned long lastRTDBWrite = 0;
unsigned long lastFlowCalc = 0;
unsigned long lastFirestoreRead = 0;
unsigned long lastLogUpload = 0;
unsigned long lastDayCheck = 0;
unsigned long lastCalibRetry = 0;
unsigned long lastCalibRead = 0;

// =================== FORWARD DECLARATIONS ===================
void readFirestoreCalibOffset();
void readFirestoreConfig();
void uploadFirestoreLogs(String timeSlot);
void sendNotification(String message);
void processRelayQueue();
void stopRelay(int idx, const char* reason);
void requestRelay(int idx, RelaySource src, float targetMl);    // TAMBAHAN
void activateRelay(int idx, RelaySource src, float targetMl);   // TAMBAHAN
void applyManualCommands(const String& waterCmd, const String& pestCmd); // TAMBAHAN

// Schedule guards
int lastWateringHour = -1;
int lastPesticideHour = -1;

// Previous state untuk deteksi perubahan hari
int previousDay = 0;

// OLED Display State
uint8_t currentScreen = 0;
unsigned long lastScreenChange = 0;
const unsigned long SCREEN_INTERVAL = 5000; // 5 detik per layar

// =================== 4. ISR FLOW METER ===================
void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

// =================== 5. INISIALISASI RELAY STATE MACHINE ===================
void initRelays() {
  relays[RELAY_WATER] = {RS_IDLE, SRC_NONE, 0, 0, 0, PUMP_TIMEOUT_MS,
                         false, SRC_NONE, 0, 0, false, RELAY_WATER_PIN};
  relays[RELAY_PEST]  = {RS_IDLE, SRC_NONE, 0, 0, 0, PUMP_TIMEOUT_MS,
                         false, SRC_NONE, 0, 0, false, RELAY_PEST_PIN};
}

// =================== 6. EEPROM UTILITIES ===================

uint16_t calculateChecksum(EEPROMData* data) {
  uint16_t sum = 0;
  uint8_t* ptr = (uint8_t*)data;
  for (size_t i = 0; i < sizeof(EEPROMData) - 2; i++) {
    sum += ptr[i];
  }
  return sum;
}

void setWifiList() {
  strcpy(wifiList[0].ssid, "Seven");
  strcpy(wifiList[0].pass, "Satu72002");

  strcpy(wifiList[1].ssid, "VANZA");
  strcpy(wifiList[1].pass, "sehatselalu");

  strcpy(wifiList[2].ssid, "Telkomakses");
  strcpy(wifiList[2].pass, "");

  strcpy(wifiList[3].ssid, "User");
  strcpy(wifiList[3].pass, "User12345");

  strcpy(wifiList[4].ssid, "TECNO SPARK 30C");
  strcpy(wifiList[4].pass, "masvanza");
}

void saveWifiEEPROM() {
  EEPROMData data;
  data.magic = EEPROM_MAGIC;
  memcpy(data.wifiList, wifiList, sizeof(wifiList));
  data.checksum = calculateChecksum(&data);
  
  EEPROM.begin(sizeof(EEPROMData));
  EEPROM.put(0, data);
  EEPROM.commit();
  EEPROM.end();
  
  Serial.println("WiFi credentials saved to EEPROM");
}

void loadWifiEEPROM() {
  EEPROMData data;
  
  EEPROM.begin(sizeof(EEPROMData));
  EEPROM.get(0, data);
  EEPROM.end();
  
  if (data.magic == EEPROM_MAGIC && data.checksum == calculateChecksum(&data)) {
    memcpy(wifiList, data.wifiList, sizeof(wifiList));
    Serial.println("✓ WiFi credentials loaded from EEPROM");
  } else {
    Serial.println("⚠ EEPROM invalid, using defaults");
    setWifiList();
    saveWifiEEPROM();
  }
}

// =================== 7. WIFI UTILITIES (NON-BLOCKING) ===================

bool connectWiFi() {
  Serial.println("\n=== WiFi Connection ===");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(200);

  // Scan networks dulu
  Serial.println("Scanning networks...");
  int n = WiFi.scanNetworks(); // BLOCKING scan
  
  if (n <= 0) {
    Serial.println("No networks found");
    return false;
  }

  Serial.printf("Found %d networks\n", n);

  // Loop cari jaringan yang ada di list
  for (int i = 0; i < n; i++) {
    String found = WiFi.SSID(i);

    for (int j = 0; j < MAX_WIFI; j++) {
      if (found == wifiList[j].ssid && strlen(wifiList[j].ssid) > 0) {
        Serial.printf("→ Trying: %s\n", wifiList[j].ssid);
        
        WiFi.begin(wifiList[j].ssid, wifiList[j].pass);
        
        // Blocking wait with timeout
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 40) {
          delay(250);
          Serial.print(".");
          attempts++;
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("✓ WiFi Connected!");
          Serial.printf("  IP: %s\n", WiFi.localIP().toString().c_str());
          Serial.printf("  RSSI: %d dBm\n", WiFi.RSSI());
          WiFi.scanDelete();
          return true;
        } else {
          Serial.println("✗ Connection failed");
        }
      }
    }
  }

  WiFi.scanDelete();
  Serial.println("✗ No suitable network found");
  return false;
}

void checkWiFiConnection() {
  unsigned long now = millis();
  
  if (now - lastWiFiCheck < 5000) return;
  lastWiFiCheck = now;
  
  if (WiFi.status() == WL_CONNECTED) {
    if (!systemOnline) {
      Serial.println("✓ WiFi Reconnected!");
      systemOnline = true;
      reconnectDelay = 10000;
      
      if (!Firebase.ready()) {
        Serial.println("Reinitializing Firebase...");
        config.database_url = FIREBASE_HOST;
        config.signer.tokens.legacy_token = FIREBASE_AUTH;
        fbdo.setResponseSize(4096);
        Firebase.begin(&config, &auth);
        Firebase.reconnectWiFi(true);
        delay(2000);
        
        if (Firebase.ready()) {
          Serial.println("✓ Firebase Ready!");
        }
      }
      
      Serial.println("Force loading calibration data...");
      readFirestoreCalibOffset();
      readFirestoreConfig();
      

      if (calibDataLoaded) {
        calibSystemReady = true;
        Serial.println("✓ Calibration system ready after reconnect!");
      }
      
      lastFirestoreRead = millis();
    }
    return;
  }
  
  if (systemOnline) {
    Serial.println("✗ WiFi Lost!");
    systemOnline = false;
  }
  
  if (now - lastReconnectAttempt >= reconnectDelay) {
    lastReconnectAttempt = now;
    Serial.println("Attempting reconnect...");
    
    WiFi.disconnect();
    delay(100);
    
    if (connectWiFi()) {
      systemOnline = true;
      reconnectDelay = 10000;
      
      config.database_url = FIREBASE_HOST;
      config.signer.tokens.legacy_token = FIREBASE_AUTH;
      fbdo.setResponseSize(4096);
      Firebase.begin(&config, &auth);
      Firebase.reconnectWiFi(true);
      delay(2000);
      
      Serial.println("✓ Reconnection successful!");
    } else {
      reconnectDelay = min(reconnectDelay * 2, 120000UL);
      Serial.printf("  Next attempt in: %lu seconds\n", reconnectDelay/1000);
    }
  }
}

// =================== 8. FUNGSIONALITAS SENSOR ===================

int readSoilRaw() {
  long sum = 0;
  int samples = 10;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(SOIL_PIN);
    delay(10);
  }
  int adc = sum / samples;
  int level = map(adc, 1023, 0, 1, 10);
  return constrain(level, 1, 10);
}

int soilFinalValue(int soilRaw) {
  int delta = soilMeterOffset - soilOffset;
  int result = constrain(soilRaw + delta, 1, 10);
  return result;
}

void getSoilMoisture() {
  int soilRaw = readSoilRaw();
  kelembapanTanah = isCalibrated ? soilFinalValue(soilRaw) : soilRaw;
}

void getDHT22() {
  float tRaw = dht.readTemperature();
  float hRaw = dht.readHumidity();

  if (isnan(tRaw) || isnan(hRaw)) {
    Serial.println("⚠ DHT read failed! Using last value.");
    return;
  }

  if (isCalibrated) {
    float tempDelta = hygroTempOffsetF - dhtTempOffsetF;
    float humDelta  = hygroHumOffsetF - dhtHumOffsetF;
    
    suhuUdara = tRaw + tempDelta;
    kelembapanUdara = constrain(hRaw + humDelta, 0, 100);
  } else {
    suhuUdara = tRaw;
    kelembapanUdara = hRaw;
  }
}

float readUltrasonic(int trigPin, int echoPin, float offset) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

if (duration == 0) {
  return -1;
}

float distance = duration * 0.0343 / 2.0;


  if (distance == 0 || distance > 400) return -1;
  return distance + offset;
}

String getTankStatus(float distance) {
  if (distance < 0) return "Error";
  if (distance <= FULL_LEVEL) return "Penuh";
  if (distance >= EMPTY_THRESHOLD) return "Habis";
  return "Normal";
}

void getTankLevel() {
  float jarakAir = readUltrasonic(TRIG_AIR, ECHO_AIR, OFFSET_AIR);
  tangkiAirStatus = getTankStatus(jarakAir);

  float jarakPest = readUltrasonic(TRIG_PEST, ECHO_PEST, OFFSET_PEST);
  tangkiPestStatus = getTankStatus(jarakPest);
}

void calculateFlow() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastFlowCalc >= 1000) {
    unsigned long actualInterval = currentMillis - lastFlowCalc;
    lastFlowCalc = currentMillis;

    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    float flowRateHz = count;
    flowRate = flowRateHz / 7.5;

    float volumeAdded = (flowRate / 60.0) * (actualInterval / 1000.0) * 1000.0; // dalam mL

    // Kreditkan volume ke relay yang benar-benar ACTIVE secara fisik
    for (int idx = 0; idx < RELAY_COUNT; idx++) {
      RelayCtrl& r = relays[idx];
      if (r.state != RS_ACTIVE) continue;

      r.volumeMl += volumeAdded;

      // Ada aliran -> reset flow watchdog
      if (count > 0) {
        r.lastPulseMs       = currentMillis;
        r.watchdogTriggered = false;
      }
    }

    // Akumulasi volume harian (dari relay yang aktif)
    if (relays[RELAY_WATER].state == RS_ACTIVE) litersWaterToday += volumeAdded / 1000.0;
    if (relays[RELAY_PEST].state  == RS_ACTIVE) litersPestToday  += volumeAdded / 1000.0;
  }
}

void readAllSensors() {
  getDHT22();
  getSoilMoisture();
  getTankLevel();
}

// =================== 9. FIRESTORE UTILITIES ===================

void readFirestoreCalibOffset() {
  if (!systemOnline) {
    Serial.println("⚠ OFFLINE - Skipping Firestore Calib");
    return;
  }
  
  Serial.println("\n=== READING FIRESTORE CALIB ===");

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, FIRESTORE_CALIB_URL);
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.GET();
  
  if (httpCode == 200) {
    String payload = http.getString();
    JsonDocument doc;
    if (deserializeJson(doc, payload) != DeserializationError::Ok) {
      Serial.println("✗ JSON Parse Error");
      calibDataLoaded = false;
      calibSystemReady = false;
      http.end(); 
      return;
    }
    
    JsonObject fields = doc["fields"];
    
    // Baca semua offset
    dhtTempOffsetF   = fields["dht_temp_offset"]["doubleValue"] | 0.0;
    hygroTempOffsetF = fields["hygro_temp_offset"]["doubleValue"] | 0.0;
    dhtHumOffsetF    = fields["dht_hum_offset"]["doubleValue"] | 0.0;
    hygroHumOffsetF  = fields["hygro_hum_offset"]["doubleValue"] | 0.0;
    soilMeterOffset  = (int)round(fields["soil_meter_offset"]["doubleValue"] | 0.0);
    soilOffset       = (int)round(fields["soil_offset"]["doubleValue"] | 0.0);

    calibDataLoaded = true;
    
    // ============ INI BAGIAN PENTING! ============
    // Baca flag dari Firestore
    bool firestoreCalibFlag = fields["isCalibrated"]["booleanValue"] | false;
    
    // DETEKSI OTOMATIS: Jika ada data offset yang tidak 0, anggap sudah kalibrasi
    bool hasCalibrationData = (
      abs(dhtTempOffsetF) > 0.01 || 
      abs(hygroTempOffsetF) > 0.01 || 
      abs(dhtHumOffsetF) > 0.01 || 
      abs(hygroHumOffsetF) > 0.01 || 
      soilMeterOffset != 0 || 
      soilOffset != 0
    );
    
    // Gunakan flag Firestore ATAU deteksi otomatis
    isCalibrated = firestoreCalibFlag || hasCalibrationData;
    // ============================================

    calibSystemReady = true;

    Serial.printf("✓ Calib Data Loaded:\n");
    Serial.printf("  dhtTemp: %.2f | hygroTemp: %.2f\n", dhtTempOffsetF, hygroTempOffsetF);
    Serial.printf("  dhtHum: %.2f | hygroHum: %.2f\n", dhtHumOffsetF, hygroHumOffsetF);
    Serial.printf("  soilMeter: %d | soilOffset: %d\n", soilMeterOffset, soilOffset);
    Serial.printf("  calibDataLoaded: TRUE\n");
    Serial.printf("  calibSystemReady: TRUE\n");
    Serial.printf("  Firestore isCalibrated flag: %s\n", firestoreCalibFlag ? "TRUE" : "FALSE");
    Serial.printf("  Auto-detected calibration: %s\n", hasCalibrationData ? "TRUE" : "FALSE");
    Serial.printf("  FINAL isCalibrated: %s\n", isCalibrated ? "TRUE (Using calibrated data)" : "FALSE (Using RAW data)");
    
  } else {
    Serial.printf("✗ Firestore Error: %d\n", httpCode);
  }
  http.end();
}

void readFirestoreConfig() {
  if (!systemOnline) return;
  
  Serial.println("\n=== READING FIRESTORE CONFIG ===");

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, FIRESTORE_CONFIG_URL);
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.GET();
  
  if (httpCode == 200) {
    String payload = http.getString();
    JsonDocument doc;
    if (deserializeJson(doc, payload) != DeserializationError::Ok) {
      Serial.println("✗ JSON Parse Error");
      http.end(); 
      return;
    }
    
    JsonObject fields = doc["fields"];
    
    setPointSuhuMin = fields["SetPoint_SuhuMin"]["integerValue"] | 20;
    setPointSuhuMax = fields["SetPoint_SuhuMax"]["integerValue"] | 27;
    setPointHumidityMin = fields["SetPoint_KelembapanUdaraMin"]["integerValue"] | 60;
    setPointHumidityMax = fields["SetPoint_KelembapanUdaraMax"]["integerValue"] | 80;
    setPointSoilMin = fields["SetPoint_KelembapanTanahMin"]["integerValue"] | 5;
    setPointSoilMax = fields["SetPoint_KelembapanTanahMax"]["integerValue"] | 8;
    
    if (fields["TanggalDitanam"]["stringValue"].is<const char*>()) {
      tanggalDitanam = fields["TanggalDitanam"]["stringValue"].as<String>();
    }

    Serial.println("✓ Config loaded from Firestore");
  } else {
    Serial.printf("✗ Firestore Config Error: %d\n", httpCode);
  }
  http.end();
}

void uploadFirestoreLogs(String timeSlot) {
  if (!systemOnline) {
    Serial.printf("⚠ Offline - Logs tidak diupload [%s]\n", timeSlot.c_str());
    return;
  }
  
  Serial.printf("\n=== UPLOADING LOGS [%s] ===\n", timeSlot.c_str());

  DateTime now = rtc.now();
  char dateStr[16];
  sprintf(dateStr, "%04d-%02d-%02d", now.year(), now.month(), now.day());
  
  String url = String(FIRESTORE_LOGS_URL) + dateStr + "/data/" + timeSlot + "?key=" + API_KEY;
  
  JsonDocument doc;
  JsonObject fields = doc.createNestedObject("fields");
  
  char timestampStr[32];
  sprintf(timestampStr, "%04d-%02d-%02d %02d:%02d:%02d", 
    now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  
  fields["timestamp"]["stringValue"] = timestampStr;
  fields["suhuUdara"]["doubleValue"] = round(suhuUdara * 10) / 10;
  fields["kelembapanUdara"]["integerValue"] = (int)round(kelembapanUdara);
  fields["kelembapanTanah"]["integerValue"] = kelembapanTanah;
  fields["TotalLitersWaterToday"]["doubleValue"] = round(litersWaterToday * 100) / 100;
  fields["TotalLitersPestisidaToday"]["doubleValue"] = round(litersPestToday * 100) / 100;
  fields["UmurTanamanDays"]["integerValue"] = umurTanamanDays;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.PATCH(jsonString);
  
  if (httpCode == 200) {
    Serial.println("✓ Logs uploaded successfully");
  } else {
    Serial.printf("✗ Logs upload failed: %d\n", httpCode);
  }
  http.end();
}

// =================== 10. FIREBASE RTDB UTILITIES ===================

void readRTDBControl() {
  if (!systemOnline || !Firebase.ready()) return;

  String prevCondition = systemCondition;

  if (Firebase.RTDB.getString(&fbdo, "/Control/Condition")) {
    systemCondition = fbdo.stringData();
  }
  
  String waterCmd = prevWaterCmd;
  String pestCmd  = prevPestCmd;

  if (Firebase.RTDB.getString(&fbdo, "/Control/Water_State")) {
    waterCmd = fbdo.stringData();
    if (systemCondition == "Manual") targetWater = waterCmd; // sinkron UI
  }
  
  if (Firebase.RTDB.getString(&fbdo, "/Control/Pestisida_State")) {
    pestCmd = fbdo.stringData();
    if (systemCondition == "Manual") targetPestisida = pestCmd; // sinkron UI
  }

  // Proses perintah Manual via state machine (deteksi edge di dalam fungsi)
  if (systemCondition == "Manual") {
    applyManualCommands(waterCmd, pestCmd);
  }

  // Mode berubah Manual -> Auto: matikan relay yang sumbernya Manual
  if (prevCondition == "Manual" && systemCondition == "Auto") {
    if (relays[RELAY_WATER].state != RS_IDLE && relays[RELAY_WATER].source == SRC_MANUAL)
      stopRelay(RELAY_WATER, "mode->Auto");
    if (relays[RELAY_PEST].state != RS_IDLE && relays[RELAY_PEST].source == SRC_MANUAL)
      stopRelay(RELAY_PEST, "mode->Auto");
    prevWaterCmd = "OFF";
    prevPestCmd  = "OFF";
  }
}

void writeRTDBStatus() {
  if (!systemOnline || !Firebase.ready()) {
    Serial.println("⚠ Offline - Status tidak dikirim");
    return;
  }

  FirebaseJson json;
  
  DateTime now = rtc.now();
  char timestampStr[32];
  sprintf(timestampStr, "%04d-%02d-%02d %02d:%02d:%02d", 
    now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  
  json.set("timestamp", timestampStr);
  json.set("suhuUdara", round(suhuUdara * 10) / 10);
  json.set("kelembapanUdara", (int)round(kelembapanUdara));
  json.set("kelembapanTanah", kelembapanTanah);
  json.set("tangkiAir", tangkiAirStatus);
  json.set("tangkiPestisida", tangkiPestStatus);
  json.set("TotalLitersWaterToday", round(litersWaterToday * 100) / 100);
  json.set("TotalLitersPestisidaToday", round(litersPestToday * 100) / 100);
  json.set("UmurTanamanDays", umurTanamanDays);

  if (Firebase.RTDB.setJSON(&fbdo, "/Status", &json)) {
    Serial.println("✓ RTDB Status Sent!");
  } else {
    Serial.printf("✗ RTDB Write Error: %s\n", fbdo.errorReason().c_str());
  }
}

void sendNotification(String message) {
  if (!systemOnline || !Firebase.ready()) {
    Serial.printf("⚠ OFFLINE Notif: %s\n", message.c_str());
    return;
  }
  
  if (Firebase.RTDB.setString(&fbdo, "/Notifications/latest", message)) {
    Serial.printf("📢 Notif: %s\n", message.c_str());
  } else {
    Serial.printf("✗ Notif Error: %s\n", fbdo.errorReason().c_str());
  }
}

// =================== 11. LOGIKA PENYIRAMAN OTOMATIS ===================

void calculateUmurTanaman() {
  if (tanggalDitanam == "" || tanggalDitanam.length() < 10) return;
  
  int year = tanggalDitanam.substring(0, 4).toInt();
  int month = tanggalDitanam.substring(5, 7).toInt();
  int day = tanggalDitanam.substring(8, 10).toInt();
  
  DateTime planted(year, month, day, 0, 0, 0);
  DateTime now = rtc.now();
  
  TimeSpan diff = now - planted;
  umurTanamanDays = diff.days();
}

bool isPesticideScheduledDay() {
  if (umurTanamanDays < 14) return false;
  
  DateTime now = rtc.now();
  int dayOfWeek = now.dayOfTheWeek();
  
  return (dayOfWeek == 3 || dayOfWeek == 6); // Rabu & Sabtu
}

bool isConditionSafeForPesticide() {
  bool tempOK = (suhuUdara >= setPointSuhuMin && suhuUdara <= setPointSuhuMax);
  bool humOK = (kelembapanUdara >= setPointHumidityMin && kelembapanUdara <= setPointHumidityMax);
  
  return (tempOK && humOK);
}

void checkScheduledWatering() {
  if (systemCondition != "Auto") return;
  
  DateTime now = rtc.now();
  int hour = now.hour();
  int minute = now.minute();
  
  bool isWateringTime = (
    (hour == 7 && minute == 0) ||
    (hour == 12 && minute == 0) ||
    (hour == 17 && minute == 0)
  );
  
  // Trigger awal: jadwal cocok dan belum dijalankan jam ini
  if (isWateringTime && hour != lastWateringHour) {
    lastWateringHour = hour;

    if (kelembapanTanah >= setPointSoilMax) {
      sendNotification("Penyiraman dibatalkan - Tanah sudah lembap");
      return;
    }
    if (tangkiAirStatus == "Habis" || tangkiAirStatus == "Error") {
      sendNotification("Penyiraman gagal - Tangki air habis");
      return;
    }

    // requestRelay() menangani mutual exclusion & antrian secara otomatis
    requestRelay(RELAY_WATER, SRC_AUTO, TARGET_WATER_ML);
  }
  // Catatan: monitoring selesai/timeout kini ditangani updateRelayStateMachine()
}

void checkScheduledPesticide() {
  if (systemCondition != "Auto") return;
  if (umurTanamanDays < 14) return;
  
  DateTime now = rtc.now();
  int hour = now.hour();
  int minute = now.minute();
  
  bool isPesticideTime = (hour == 17 && minute == 0);
  
  char dateStr[16];
  sprintf(dateStr, "%04d-%02d-%02d", now.year(), now.month(), now.day());
  String todayDate = String(dateStr);
  
  if (lastPestDate != todayDate) {
    pestScheduledToday = false;
    lastPestDate = todayDate;
  }
  
  if (isPesticideTime && isPesticideScheduledDay() && !pestScheduledToday && 
      hour != lastPesticideHour) {
    
    lastPesticideHour = hour;
    
    if (tangkiPestStatus == "Habis" || tangkiPestStatus == "Error") {
      sendNotification("Penyemprotan gagal - Tangki pestisida habis");
      pestTundaDays++;
      return;
    }
    
    if (!isConditionSafeForPesticide()) {
      pestTundaDays++;
      sendNotification("Penyemprotan ditunda - Kondisi tidak ideal");
      
      if (pestTundaDays >= 2) {
        sendNotification("Peringatan: Pestisida tertunda " + String(pestTundaDays) + " hari - Cari waktu aman");
      }
      return;
    }
    
    pestScheduledToday = true;
    pestTundaDays = 0;

    // requestRelay() menangani mutual exclusion & antrian secara otomatis
    // Jika relay air sedang aktif, pest akan masuk antrian (PENDING)
    requestRelay(RELAY_PEST, SRC_AUTO, TARGET_PEST_ML);
  }
  // Catatan: monitoring selesai/timeout kini ditangani updateRelayStateMachine()
}

// =================== 12. RELAY STATE MACHINE ===================

void writeRelayHW(int idx, bool on) {
  // Relay active LOW: ON = LOW, OFF = HIGH
  pcf.write(relays[idx].hwPin, on ? LOW : HIGH);
}

void activateRelay(int idx, RelaySource src, float targetMl) {
  RelayCtrl& r    = relays[idx];
  r.state         = RS_ACTIVE;
  r.source        = src;
  r.targetVolumeMl= targetMl;
  r.volumeMl      = 0.0;
  r.startMs       = millis();
  r.lastPulseMs   = millis();
  r.watchdogTriggered = false;
  writeRelayHW(idx, true);

  const char* name   = (idx == RELAY_WATER) ? "Air" : "Pestisida";
  const char* srcStr = (src == SRC_MANUAL)  ? "Manual" : "Auto";
  Serial.printf("✓ Relay %s ACTIVE [%s] target=%.0fml\n", name, srcStr, targetMl);
  sendNotification(String(name) + " dimulai [" + srcStr + "]" +
    (targetMl > 0 ? " target " + String((int)targetMl) + "ml" : ""));
}

void stopRelay(int idx, const char* reason) {
  RelayCtrl& r = relays[idx];
  if (r.state == RS_IDLE) return;

  writeRelayHW(idx, false);

  const char* name = (idx == RELAY_WATER) ? "Air" : "Pestisida";
  Serial.printf("■ Relay %s STOP [%s] volume=%.0fml\n", name, reason, r.volumeMl);
  sendNotification(String(name) + " selesai [" + reason + "] " +
    String((int)r.volumeMl) + "ml");

  r.state  = RS_IDLE;
  r.source = SRC_NONE;

  // Setelah relay berhenti, cek apakah ada yang antri
  processRelayQueue();
}

void requestRelay(int idx, RelaySource src, float targetMl) {
  RelayCtrl& r     = relays[idx];
  int        other = (idx == RELAY_WATER) ? RELAY_PEST : RELAY_WATER;

  // Relay ini sudah aktif: abaikan
  if (r.state == RS_ACTIVE) return;

  // Relay LAIN sedang aktif: masuk antrian
  if (relays[other].state == RS_ACTIVE) {
    const char* name = (idx == RELAY_WATER) ? "Air" : "Pestisida";
    Serial.printf("⏳ Relay %s PENDING\n", name);
    r.state           = RS_PENDING;
    r.hasPending      = true;
    r.pendingSrc      = src;
    r.pendingVolumeMl = targetMl;
    sendNotification(String(name) + " antri — relay lain sedang aktif");
    return;
  }

  activateRelay(idx, src, targetMl);
}

void cancelPending(int idx) {
  RelayCtrl& r = relays[idx];
  if (r.state != RS_PENDING) return;
  r.state      = RS_IDLE;
  r.hasPending = false;
  const char* name = (idx == RELAY_WATER) ? "Air" : "Pestisida";
  Serial.printf("✕ Pending %s dibatalkan\n", name);
}

void processRelayQueue() {
  // Prioritas: air dulu, lalu pestisida
  for (int idx = 0; idx < RELAY_COUNT; idx++) {
    RelayCtrl& r = relays[idx];
    if (r.hasPending && r.state == RS_PENDING) {
      r.hasPending = false;
      activateRelay(idx, r.pendingSrc, r.pendingVolumeMl);
      return;
    }
  }
}

// Dipanggil tiap loop: cek target/timeout/watchdog
void updateRelayStateMachine() {
  unsigned long now = millis();

  for (int idx = 0; idx < RELAY_COUNT; idx++) {
    RelayCtrl& r = relays[idx];
    if (r.state != RS_ACTIVE) continue;

    // Flow watchdog: relay ON tapi tidak ada pulse > 5 detik
    if ((now - r.lastPulseMs) > FLOW_WATCHDOG_MS && !r.watchdogTriggered) {
      r.watchdogTriggered = true;
      const char* name = (idx == RELAY_WATER) ? "Air" : "Pestisida";
      Serial.printf("⚠ Flow watchdog! Relay %s ON tapi tidak ada aliran\n", name);
      sendNotification(String("⚠ ERROR: Relay ") + name +
                       " ON tapi aliran tidak terdeteksi!");
    }

    // Target volume tercapai
    if (r.targetVolumeMl > 0 && r.volumeMl >= r.targetVolumeMl) {
      stopRelay(idx, "target tercapai");
      continue;
    }

    // Kondisi stop Auto tambahan (tanah sudah cukup lembap)
    if (r.source == SRC_AUTO && idx == RELAY_WATER &&
        kelembapanTanah >= setPointSoilMax) {
      stopRelay(idx, "tanah cukup lembap");
      continue;
    }

    // Timeout paksa
    if ((now - r.startMs) > r.timeoutMs) {
      stopRelay(idx, "timeout");
      continue;
    }
  }
}

// Tangani perintah Manual dari RTDB dengan deteksi edge (OFF->ON / ON->OFF)
// Dipanggil dari readRTDBControl() saat mode Manual
void applyManualCommands(const String& waterCmd, const String& pestCmd) {
  if (waterCmd != prevWaterCmd) {
    prevWaterCmd = waterCmd;
    if (waterCmd == "ON") {
      if (tangkiAirStatus == "Habis" || tangkiAirStatus == "Error")
        sendNotification("Manual Air gagal - tangki habis/error");
      else
        requestRelay(RELAY_WATER, SRC_MANUAL, 0.0); // 0 = tanpa batas volume
    } else {
      if (relays[RELAY_WATER].state == RS_ACTIVE)  stopRelay(RELAY_WATER, "Manual OFF");
      if (relays[RELAY_WATER].state == RS_PENDING) cancelPending(RELAY_WATER);
    }
  }

  if (pestCmd != prevPestCmd) {
    prevPestCmd = pestCmd;
    if (pestCmd == "ON") {
      if (tangkiPestStatus == "Habis" || tangkiPestStatus == "Error")
        sendNotification("Manual Pestisida gagal - tangki habis/error");
      else
        requestRelay(RELAY_PEST, SRC_MANUAL, 0.0);
    } else {
      if (relays[RELAY_PEST].state == RS_ACTIVE)  stopRelay(RELAY_PEST, "Manual OFF");
      if (relays[RELAY_PEST].state == RS_PENDING) cancelPending(RELAY_PEST);
    }
  }
}

// Fungsi lama applyRelay() diganti — stub ini ditinggal agar tidak ada build error
// jika ada pemanggilan tersisa; semua logika sudah pindah ke state machine di atas.
void applyRelay() {
  // Tidak diperlukan lagi — relay dikontrol oleh requestRelay() / stopRelay()
  // Fungsi ini sengaja dikosongkan agar loop() tidak perlu diubah.
}

// =================== 13. OLED DISPLAY (3 TAMPILAN BERGANTIAN) ===================
void drawOLED() {
  oled.clearBuffer();
  oled.setFontMode(1);
  oled.setBitmapMode(1);
  
  char buf[32];
  DateTime rtcNow = rtc.now();
  
  // Tampilkan loading HANYA jika belum terkoneksi WiFi atau data kalibrasi belum dimuat
  if (!systemOnline || !calibDataLoaded) {
    oled.setFont(u8g2_font_4x6_tr);
    
    oled.drawStr(14, 11, "LOADING CALIBRATION...");
    oled.drawStr(7, 30, "Connect Firestore...");
    oled.drawStr(7, 45, "WiFi:");
    oled.drawStr(35, 45, systemOnline ? "OK" : "OFFLINE");
    
    oled.sendBuffer();
    return;
  }
  
  // Jika data kalibrasi = 0 (belum dikalibrasi), tampilkan mode RAW dengan data REAL-TIME
  if (!isCalibrated) {
    oled.setFont(u8g2_font_4x6_tr);
    oled.drawStr(14, 11, "KALIBRASI MODE (RAW DATA)");
    oled.drawStr(7, 23, "Suhu");
    oled.drawStr(7, 33, "Kelembapan Udara");
    oled.drawStr(7, 43, "Kelembapan Tanah");
    oled.drawStr(77, 24, ":");
    oled.drawStr(77, 33, ":");
    oled.drawStr(77, 43, ":");
    
    // Tampilkan data sensor REAL-TIME (bukan statis!)
    sprintf(buf, "%.0f C", suhuUdara);
    oled.drawStr(84, 24, buf);
    
    sprintf(buf, "%.0f%%", kelembapanUdara);
    oled.drawStr(84, 33, buf);
    
    sprintf(buf, "%d/10", kelembapanTanah);
    oled.drawStr(84, 43, buf);
    
    oled.drawStr(12, 57, "Pastikan Kalibrasi Di APPS");
    oled.sendBuffer();
    return;
  }
  
  // Jika sudah dikalibrasi, tampilkan 3 screen bergantian
  unsigned long now = millis();
  if (now - lastScreenChange >= SCREEN_INTERVAL) {
    lastScreenChange = now;
    currentScreen = (currentScreen + 1) % 3; 
  }
  
  switch (currentScreen) {
    case 0: // Tampilan 1: Data Monitoring
      oled.setFont(u8g2_font_helvB08_tr);
      oled.drawStr(24, 12, "Data Monitoring");
      
      oled.setFont(u8g2_font_5x8_tr);
      oled.drawStr(3, 22, "Suhu Udara");
      oled.drawStr(3, 31, "Kelembapan Udara");
      oled.drawStr(3, 40, "Kelembapan tanah");
      
      oled.setFont(u8g2_font_4x6_tr);
      oled.drawStr(86, 22, ":");
      oled.drawStr(86, 31, ":");
      oled.drawStr(86, 40, ":");
      
      oled.setFont(u8g2_font_5x8_tr);
      oled.drawStr(2, 49, "Tangki Air");
      oled.setFont(u8g2_font_4x6_tr);
      oled.drawStr(86, 49, ":");
      
      oled.setFont(u8g2_font_5x8_tr);
      oled.drawStr(2, 58, "Tangki Pestisida");
      oled.setFont(u8g2_font_4x6_tr);
      oled.drawStr(86, 58, ":");
      
      // Data values
      oled.setFont(u8g2_font_5x8_tr);
      sprintf(buf, "%.1fC", suhuUdara);
      oled.drawStr(93, 22, buf);
      
      sprintf(buf, "%.0f%%", kelembapanUdara);
      oled.drawStr(93, 31, buf);
      
      sprintf(buf, "%d/10", kelembapanTanah);
      oled.drawStr(93, 40, buf);
      
      oled.drawStr(93, 49, tangkiAirStatus.c_str());
      oled.drawStr(93, 58, tangkiPestStatus.c_str());
      break;
      
    case 1: // Tampilan 2: Info Tanaman & Mode
      oled.setFont(u8g2_font_helvB08_tr);
      oled.drawStr(24, 12, "Data Monitoring");
      
      oled.setFont(u8g2_font_5x8_tr);
      oled.drawStr(3, 24, "Tanggal Ditanam");
      oled.drawStr(5, 35, "Umur Tanaman");
      oled.drawStr(80, 24, ":");
      oled.drawStr(80, 36, ":");
      
      // Format tanggal ditanam (YYYY-MM-DD -> DD/MM/YY)
      if (tanggalDitanam.length() >= 10) {
        String day = tanggalDitanam.substring(8, 10);
        String month = tanggalDitanam.substring(5, 7);
        String year = tanggalDitanam.substring(2, 4);
        sprintf(buf, "%s/%s/%s", day.c_str(), month.c_str(), year.c_str());
      } else {
        strcpy(buf, "--/--/--");
      }
      oled.drawStr(85, 24, buf);
      
      // Umur tanaman dalam hari (jika < 30) atau bulan
      if (umurTanamanDays < 30) {
        sprintf(buf, "%d Hari", umurTanamanDays);
      } else {
        int bulan = umurTanamanDays / 30;
        sprintf(buf, "%d Bulan", bulan);
      }
      oled.drawStr(86, 36, buf);
      
      oled.drawStr(5, 46, "Mode Alat");
      oled.drawStr(80, 47, ":");
      oled.drawStr(86, 47, systemCondition.c_str());
      
      // Timestamp
      sprintf(buf, "%02d/%02d/%02d | %02d:%02d:%02d", 
        rtcNow.day(), rtcNow.month(), rtcNow.year() % 100,
        rtcNow.hour(), rtcNow.minute(), rtcNow.second());
      oled.drawStr(6, 59, buf);
      break;
      
    case 2: // Tampilan 3: Status Relay
      oled.setFont(u8g2_font_helvB08_tr);
      oled.drawStr(28, 12, "Status Relay");
      
      oled.setFont(u8g2_font_5x8_tr);
      oled.drawStr(3, 28, "Relay Air");
      oled.drawStr(3, 40, "Relay Pestisida");
      oled.drawStr(80, 28, ":");
      oled.drawStr(80, 40, ":");
      
      // Tampilkan ON / WAIT / OFF berdasarkan state machine
      oled.drawStr(86, 28,
        relays[RELAY_WATER].state == RS_ACTIVE  ? "ON" :
        relays[RELAY_WATER].state == RS_PENDING ? "WAIT" : "OFF");
      oled.drawStr(86, 40,
        relays[RELAY_PEST].state == RS_ACTIVE   ? "ON" :
        relays[RELAY_PEST].state == RS_PENDING  ? "WAIT" : "OFF");
      
      sprintf(buf, "Air: %.0fml | Pst: %.0fml", 
        litersWaterToday * 1000, 
        litersPestToday * 1000);
      oled.setFont(u8g2_font_4x6_tr);
      oled.drawStr(8, 57, buf);
      break;
  }
  
  oled.sendBuffer();
}

// =================== 14. RESET HARIAN ===================
void checkDailyReset() {
  DateTime now = rtc.now();
  
  if (now.day() != previousDay) {
    previousDay = now.day();
    
    litersWaterToday = 0.0;
    litersPestToday = 0.0;

    lastWateringHour = -1;
    lastPesticideHour = -1;

    calculateUmurTanaman();

    Serial.println("\n=== DAILY RESET ===");
    Serial.printf("New Day: %02d/%02d/%04d\n", now.day(), now.month(), now.year());
    Serial.printf("Plant Age: %d days\n", umurTanamanDays);

    sendNotification("Sistem reset harian - Hari ke-" + String(umurTanamanDays));
  }
}

// =================== 15. SCHEDULED LOGS UPLOAD ===================
void checkScheduledLogs() {
  DateTime now = rtc.now();
  int hour = now.hour();
  int minute = now.minute();
  
  bool isLogTime = (
    (hour == 0 && minute == 0) ||
    (hour == 7 && minute == 0) ||
    (hour == 12 && minute == 0) ||
    (hour == 17 && minute == 0) ||
    (hour == 20 && minute == 0)
  );
  
  static int lastLogHour = -1;
  
  if (isLogTime && hour != lastLogHour) {
    lastLogHour = hour;
    
    char timeSlot[8];
    sprintf(timeSlot, "%02d-00", hour);

    uploadFirestoreLogs(timeSlot);
  }
}

// =================== 16. MONITORING KONDISI KRITIS ===================
void checkCriticalConditions() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 60000) return;
  lastCheck = millis();
  
  if (suhuUdara < setPointSuhuMin - 3 || suhuUdara > setPointSuhuMax + 3) {
    sendNotification("⚠️ Suhu kritis: " + String(suhuUdara, 1) + "°C");
  }
  
  if (kelembapanUdara < setPointHumidityMin - 10) {
    sendNotification("⚠️ Kelembapan udara rendah: " + String((int)kelembapanUdara) + "%");
  }
  
  static String lastAirStatus = "";
  if (tangkiAirStatus == "Habis" && lastAirStatus != "Habis") {
    sendNotification("🚨 Tangki air habis - Segera isi ulang!");
  }
  lastAirStatus = tangkiAirStatus;
  
  static String lastPestStatus = "";
  if (tangkiPestStatus == "Habis" && lastPestStatus != "Habis") {
    sendNotification("🚨 Tangki pestisida habis - Segera isi ulang!");
  }
  lastPestStatus = tangkiPestStatus;
  
  if (kelembapanTanah < setPointSoilMin - 2) {
    sendNotification("🌱 Tanah terlalu kering: " + String(kelembapanTanah) + "/10");
  }
}

// =================== 17. SETUP ===================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  dht.begin();
  pcf.begin();
  pcf.write8(0xFF);

  // Inisialisasi state machine relay
  initRelays();
  
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseCounter, FALLING);
  
  pinMode(TRIG_AIR, OUTPUT);
  pinMode(ECHO_AIR, INPUT);
  pinMode(TRIG_PEST, OUTPUT);
  pinMode(ECHO_PEST, INPUT);
  
  // RTC Init
  if (!rtc.begin()) {
    Serial.println("⚠ RTC not found!");
  } else {
    if (rtc.lostPower()) {
      Serial.println("RTC lost power, setting to compile time");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    DateTime now = rtc.now();
    previousDay = now.day();
    Serial.printf("RTC Time: %02d/%02d/%04d %02d:%02d:%02d\n", 
      now.day(), now.month(), now.year(), 
      now.hour(), now.minute(), now.second());
  }
  
  // OLED Init
  oled.begin();
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 32, "Connecting WiFi...");
  oled.sendBuffer();
  
  // Load WiFi credentials
  loadWifiEEPROM();
  
  // WiFi Connection - BLOCKING SAMPAI BERHASIL ATAU TIMEOUT
  Serial.println("\n=== Starting WiFi Connection ===");
  bool wifiConnected = connectWiFi();
  
  if (wifiConnected) {
    systemOnline = true;
    
    oled.clearBuffer();
    oled.drawStr(0, 24, "WiFi Connected!");
    oled.drawStr(0, 40, "Init Firebase...");
    oled.sendBuffer();
    
    // FIREBASE INIT - dengan extra delay untuk stabilitas
    Serial.println("\n=== Initializing Firebase ===");
    config.database_url = FIREBASE_HOST;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;
    fbdo.setResponseSize(4096);
    
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    
    // PENTING: Beri waktu Firebase untuk init
    Serial.println("Waiting for Firebase to initialize...");
    delay(3000);
    
    // Verifikasi Firebase ready
    int attempts = 0;
    while (!Firebase.ready() && attempts < 10) {
      Serial.print(".");
      delay(500);
      attempts++;
    }
    Serial.println();
    
    if (Firebase.ready()) {
      Serial.println("✓ Firebase Connected!");
      
      oled.clearBuffer();
      oled.drawStr(0, 32, "Firebase Ready!");
      oled.sendBuffer();
      delay(1000);
      
      // Load configs
      oled.clearBuffer();
      oled.drawStr(0, 32, "Loading Config...");
      oled.sendBuffer();
      
      readFirestoreCalibOffset();
      readFirestoreConfig();

      if (calibDataLoaded) {
        calibSystemReady = true;
        Serial.println("✓ Calibration system ready!");
      }
      
      sendNotification("✅ Sistem online - Umur: " + String(umurTanamanDays) + " hari");
      
    } else {
      Serial.println("⚠ Firebase initialization timeout!");
      oled.clearBuffer();
      oled.drawStr(0, 24, "Firebase TIMEOUT");
      oled.drawStr(0, 40, "Running limited");
      oled.sendBuffer();
      delay(2000);
    }
    
  } else {
    systemOnline = false;
    Serial.println("✗ WiFi Connection Failed - Running OFFLINE");
    
    oled.clearBuffer();
    oled.drawStr(0, 24, "WiFi FAILED");
    oled.drawStr(0, 40, "Running OFFLINE");
    oled.sendBuffer();
    delay(3000);
  }
  
  // Hitung umur tanaman & baca sensor
  calculateUmurTanaman();
  readAllSensors();
  
  Serial.println("\n✓ SETUP COMPLETE");
  Serial.printf("Plant Age: %d days\n", umurTanamanDays);
  Serial.printf("Calibration: %s\n", isCalibrated ? "Active" : "Raw Mode");
  Serial.printf("WiFi Status: %s\n", systemOnline ? "ONLINE" : "OFFLINE");
  Serial.printf("Firebase Status: %s\n", Firebase.ready() ? "READY" : "NOT READY");
  
  delay(2000);

}

// =================== 18. LOOP ===================
void loop() {
  // Check WiFi & Firebase connection status
  checkWiFiConnection();

  // Retry loading calibration jika belum berhasil (fallback saat startup)
  if (systemOnline && !calibDataLoaded) {
    if (millis() - lastCalibRetry >= 3000) {
      lastCalibRetry = millis();
      readFirestoreCalibOffset();
    }
  }
  
  // Flow meter selalu jalan
  calculateFlow();

  // Update state machine relay (cek watchdog, target volume, timeout)
  updateRelayStateMachine();
  
  // Core functions (1 detik interval)
  static unsigned long lastCoreUpdate = 0;
  if (millis() - lastCoreUpdate >= 1000) {
    lastCoreUpdate = millis();
    
    readAllSensors();
    checkDailyReset();

    if (systemCondition == "Auto") {
      checkScheduledWatering();
      checkScheduledPesticide();
    }

    applyRelay();
    drawOLED();
  }
  
  // Firebase operations - HANYA jika WiFi connect DAN Firebase ready
  if (systemOnline && WiFi.status() == WL_CONNECTED && Firebase.ready()) {
    
    // Read Control (1 detik)
    if (millis() - lastRTDBRead >= 1000) {
      lastRTDBRead = millis();
      readRTDBControl();
    }
    
    // Write Status (1 menit)
    if (millis() - lastRTDBWrite >= 60000) {
      lastRTDBWrite = millis();
      writeRTDBStatus();
    }

    // Baca KALIBRASI setiap 1 menit (responsif untuk perubahan)
    if (millis() - lastCalibRead >= 100000) {
      lastCalibRead = millis();
      readFirestoreCalibOffset();
    }

    // Baca CONFIG setiap 5 menit (jarang berubah)
    if (millis() - lastFirestoreRead >= 300000) {
      lastFirestoreRead = millis();
      readFirestoreConfig();
    }

    // Scheduled tasks (1 detik check)
    static unsigned long lastScheduleCheck = 0;
    if (millis() - lastScheduleCheck >= 1000) {
      lastScheduleCheck = millis();
      checkScheduledLogs();
      checkCriticalConditions();
    }
    
  } else {
    // Offline mode - skip Firebase operations
    static unsigned long lastOfflineLog = 0;
    if (millis() - lastOfflineLog >= 30000) {
      lastOfflineLog = millis();
      Serial.println("⚠ Running in OFFLINE mode");
      Serial.printf("  WiFi: %s | Firebase: %s\n", 
        WiFi.status() == WL_CONNECTED ? "OK" : "FAIL",
        Firebase.ready() ? "OK" : "FAIL");
    }
  }
  
  delay(100);
}