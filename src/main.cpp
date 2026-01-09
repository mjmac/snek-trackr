/*
 * Snek-Trackr: Ball Python Enclosure Monitor
 *
 * Monitors two Govee H5075 hygrometers via BLE and pushes data to Blynk.
 * Designed for Arduino Nano ESP32.
 *
 * Copy src/config.h to src/config_local.h and fill in your values.
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>

// Config must be before BlynkSimpleEsp32.h include
#if __has_include("config_local.h")
  #include "config_local.h"
#else
  #error "Missing config_local.h - copy config.h to config_local.h and fill in your values"
#endif

#include <BlynkSimpleEsp32.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

const char* wifi_ssid = WIFI_SSID;
const char* wifi_pass = WIFI_PASS;
const char* govee_hot_mac = GOVEE_HOT_SIDE_MAC;
const char* govee_cool_mac = GOVEE_COOL_SIDE_MAC;

// Blynk Virtual Pins
#define VPIN_HOT_TEMP     V0
#define VPIN_HOT_HUMIDITY V1
#define VPIN_COOL_TEMP    V2
#define VPIN_COOL_HUMIDITY V3
#define VPIN_HOT_BATTERY  V4
#define VPIN_COOL_BATTERY V5

// How often to push data to Blynk (milliseconds)
#define BLYNK_PUSH_INTERVAL_MS 60000  // 1 minute

// BLE scan duration (seconds)
#define BLE_SCAN_DURATION_SEC 5


// =============================================================================
// GLOBALS
// =============================================================================

struct SensorData {
  float temperature;  // Celsius internally
  float humidity;     // Percentage
  uint8_t battery;    // Percentage
  bool valid;
  unsigned long lastUpdate;
};

SensorData hotSide = {0, 0, 0, false, 0};
SensorData coolSide = {0, 0, 0, false, 0};

NimBLEScan* pBLEScan;
unsigned long lastBlynkPush = 0;
bool initialPushDone = false;

// =============================================================================
// GOVEE H5075 PACKET DECODING
// =============================================================================

bool decodeGoveeH5075(const uint8_t* data, size_t length, float* temp, float* humidity, uint8_t* battery) {
  if (length < 7) {
    return false;
  }

  int32_t encodedValue = ((int32_t)data[3] << 16) | ((int32_t)data[4] << 8) | data[5];

  // Handle negative temperatures
  bool negative = false;
  if (encodedValue & 0x800000) {
    negative = true;
    encodedValue = encodedValue ^ 0xFFFFFF;
  }

  // Decode: value = (temp_c * 10000) + (humidity * 10)
  *humidity = (float)(encodedValue % 1000) / 10.0;
  *temp = (float)(encodedValue / 1000) / 10.0;

  if (negative) {
    *temp = -*temp;
  }

  // Battery is at byte 6
  *battery = data[6];

  // Sanity check
  if (*temp < -40 || *temp > 60 || *humidity < 0 || *humidity > 100 || *battery > 100) {
    return false;
  }

  return true;
}

// =============================================================================
// BLE SCANNING
// =============================================================================

class GoveeAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    std::string addr = advertisedDevice->getAddress().toString();

    bool isHotSide = (addr == govee_hot_mac);
    bool isCoolSide = (addr == govee_cool_mac);

    if (!isHotSide && !isCoolSide) {
      return;
    }

    if (!advertisedDevice->haveManufacturerData()) {
      return;
    }

    std::string mfgData = advertisedDevice->getManufacturerData();
    float temp, humidity;
    uint8_t battery;

    if (decodeGoveeH5075((const uint8_t*)mfgData.data(), mfgData.length(), &temp, &humidity, &battery)) {
      SensorData* sensor = isHotSide ? &hotSide : &coolSide;
      sensor->temperature = temp;
      sensor->humidity = humidity;
      sensor->battery = battery;
      sensor->valid = true;
      sensor->lastUpdate = millis();

      Serial.printf("[%s] Temp: %.1f°C (%.1f°F), Humidity: %.1f%%, Battery: %d%%\n",
                    isHotSide ? "HOT " : "COOL",
                    temp,
                    temp * 9.0 / 5.0 + 32.0,
                    humidity,
                    battery);
    }
  }
};

void scanForGoveeDevices() {
  Serial.println("Starting BLE scan...");

  NimBLEScanResults results = pBLEScan->start(BLE_SCAN_DURATION_SEC, false);

  Serial.printf("Scan complete. Devices found: %d\n", results.getCount());
  pBLEScan->clearResults();
}

// =============================================================================
// BLYNK
// =============================================================================

void pushToBlynk() {
  if (!Blynk.connected()) {
    Serial.println("Blynk not connected, skipping push");
    return;
  }

  if (hotSide.valid) {
    Blynk.virtualWrite(VPIN_HOT_TEMP, hotSide.temperature);
    Blynk.virtualWrite(VPIN_HOT_HUMIDITY, hotSide.humidity);
    Blynk.virtualWrite(VPIN_HOT_BATTERY, hotSide.battery);
    Serial.printf("Pushed hot side: %.1f°C, %.1f%%, %d%% battery\n",
                  hotSide.temperature, hotSide.humidity, hotSide.battery);
  }

  if (coolSide.valid) {
    Blynk.virtualWrite(VPIN_COOL_TEMP, coolSide.temperature);
    Blynk.virtualWrite(VPIN_COOL_HUMIDITY, coolSide.humidity);
    Blynk.virtualWrite(VPIN_COOL_BATTERY, coolSide.battery);
    Serial.printf("Pushed cool side: %.1f°C, %.1f%%, %d%% battery\n",
                  coolSide.temperature, coolSide.humidity, coolSide.battery);
  }
}

// =============================================================================
// SETUP & LOOP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n=================================");
  Serial.println("    Snek-Trackr Starting Up");
  Serial.println("=================================\n");

  // Initialize BLE
  Serial.println("Initializing BLE...");
  NimBLEDevice::init("");
  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new GoveeAdvertisedDeviceCallbacks(), true);
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  // Connect to WiFi
  Serial.printf("Connecting to WiFi: %s", wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
  Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());

  // Initialize Blynk
  Serial.println("Connecting to Blynk...");
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();

  Serial.println("\nSetup complete! Starting monitoring...\n");
}

void loop() {
  Blynk.run();

  // Scan for Govee devices
  scanForGoveeDevices();

  unsigned long now = millis();

  // Push immediately once we have data from both sensors
  if (!initialPushDone && hotSide.valid && coolSide.valid) {
    Serial.println("Initial data received, pushing immediately...");
    pushToBlynk();
    lastBlynkPush = now;
    initialPushDone = true;
  }
  // Then push on regular interval
  else if (now - lastBlynkPush >= BLYNK_PUSH_INTERVAL_MS) {
    pushToBlynk();
    lastBlynkPush = now;
  }

  // Small delay between scan cycles
  delay(1000);
}
