#define CODE_VERSION 0.2f
#define ONE_WIRE_BUS D2
#define ARDUINOJSON_ENABLE_COMMENTS 1
#define DEVICE_DISCONNECTED -127

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClientSecureBearSSL.h>
#include <ESP8266HTTPClient.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>


ESP8266WiFiMulti wifiMulti;
HTTPClient httpClient;
BearSSL::WiFiClientSecure wifiClient;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
ModbusMaster node;

StaticJsonDocument<1200> configDoc;

IPAddress ip;
IPAddress gateway;
IPAddress subnet;
IPAddress primaryDNS;
IPAddress secondaryDNS;

const char *hostname;
const char *nodeType;
const char *updateUrl = NULL;
const char *influxUrl = NULL;

#define dataSize 2048
char data[dataSize];

unsigned long setupMillis = 0;
unsigned long readConfigFromSpiffsLatencyMillis;
unsigned long initialTimeSyncLatencyMillis;

boolean monitorSolar;

boolean loopStarted;


uint16_t pack_as_int(uint8_t v0, uint8_t v1) {
  uint16_t t = (v0 << 8) | v1;
  return t;
}

uint32_t upper_int(uint16_t v) {
  return (v >> 8) & 0xFF;
}

uint32_t lower_int(uint16_t v) {
  return v & 0xFF;
}

void customTimeSync(const char *tzInfo, const char* ntpServer1) {
  configTzTime(tzInfo, ntpServer1);

  // Wait till time is synced
  int i = 0;
  while (time(nullptr) < 1000000000ul && i < 100) {
    delay(5);
    i++;
  }

  // Fallback on getting the time from the solar charge controller if
  // monitoring solar is enabled and we failed to get the time from NTP
  if (time(nullptr) < 1000000000ul && monitorSolar) {
    uint8_t result = node.readHoldingRegisters(0x9013, 0x3);
    if (result == node.ku8MBSuccess) {

      uint16_t min_and_sec = node.getResponseBuffer(0x0);
      uint16_t day_and_hour = node.getResponseBuffer(0x1);
      uint16_t year_and_month = node.getResponseBuffer(0x2);

      struct tm tm;

      tm.tm_year = (upper_int(year_and_month) + 2000) - 1900;
      tm.tm_mon = lower_int(year_and_month) - 1;
      tm.tm_mday = upper_int(day_and_hour);
      tm.tm_hour = lower_int(day_and_hour);
      tm.tm_min = upper_int(min_and_sec);
      tm.tm_sec = lower_int(min_and_sec);

      // Serial.printf("Got date from charge controller: %d-%d-%dT%d:%d:%dZ\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

      time_t t = mktime(&tm);
      struct timeval now = { .tv_sec = t };
      settimeofday(&now, NULL);
    }
  }
}

void logAtLevel(char* message, char* levelName, uint8_t levelCode) {
  int pointLogSize = 256 + strlen(message);
  char pointLog[pointLogSize];
  int pointLogPos = 0;

  pointLogPos += snprintf(&pointLog[pointLogPos], pointLogSize - pointLogPos, "syslog,location=%s", hostname);
  pointLogPos += snprintf(&pointLog[pointLogPos], pointLogSize - pointLogPos, ",facility=%s", "console");
  pointLogPos += snprintf(&pointLog[pointLogPos], pointLogSize - pointLogPos, ",host=%s", hostname);
  pointLogPos += snprintf(&pointLog[pointLogPos], pointLogSize - pointLogPos, ",hostname=%s", hostname);
  pointLogPos += snprintf(&pointLog[pointLogPos], pointLogSize - pointLogPos, ",severity=%s ", levelName);

  pointLogPos += snprintf(&pointLog[pointLogPos], pointLogSize - pointLogPos, "facility_code=%di", 14);
  pointLogPos += snprintf(&pointLog[pointLogPos], pointLogSize - pointLogPos, ",severity_code=%di", levelCode);
  pointLogPos += snprintf(&pointLog[pointLogPos], pointLogSize - pointLogPos, ",message=\"%s\"", message);

  writePointToInflux(pointLog, pointLogPos);
}

void logInfo(char* message) {
  logAtLevel(message, "info", 6);
}

void logWarning(char* message) {
  logAtLevel(message, "warning", 4);
}

void writePointToInflux(char *point, int pointLen) {
  if (!influxUrl) {
    return;
  }

  if(!httpClient.begin(wifiClient, influxUrl)) {
    Serial.printf("[E] Begin failed\n");
    return;
  }

  httpClient.addHeader(F("Content-Type"), F("text/plain"));

  int statusCode = httpClient.POST((uint8_t*)point, pointLen);

  if (statusCode != 204) {
      Serial.printf("[D] Response: %d '%s'\n", statusCode, httpClient.getString().c_str());
  }

  httpClient.end();
}

void setup() {
  Serial.begin(115200);

  setupMillis = millis();

  bool success = SPIFFS.begin();

  if (!success) {
    Serial.println("Error mounting the file system");
    delay(1000);
    return;
  }

  unsigned long configReadStartMillis = millis();

  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Config file open failed");
    delay(1000);
    return;
  }

  deserializeJson(configDoc, configFile);
  configFile.close();

  readConfigFromSpiffsLatencyMillis = millis() - configReadStartMillis;

  hostname = configDoc["hostname"];
  nodeType = configDoc["nodeType"];

  if (configDoc.containsKey("ip") &&
      configDoc.containsKey("gateway") &&
      configDoc.containsKey("subnet") &&
      configDoc.containsKey("primaryDNS") &&
      configDoc.containsKey("secondaryDNS")) {

    ip.fromString(configDoc["ip"].as<char*>());
    gateway.fromString(configDoc["gateway"].as<char*>());
    subnet.fromString(configDoc["subnet"].as<char*>());
    primaryDNS.fromString(configDoc["primaryDNS"].as<char*>());
    secondaryDNS.fromString(configDoc["secondaryDNS"].as<char*>());

    WiFi.config(ip, gateway, subnet, primaryDNS, secondaryDNS);
  }

  WiFi.mode(WIFI_STA);

  WiFi.hostname(hostname);
  ArduinoOTA.setHostname(hostname);
  WiFi.persistent(true);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);

  int8_t wifiStatus = WiFi.waitForConnectResult(10 * 1000);

  JsonArray stages = configDoc["stages"].as<JsonArray>();

  if (wifiStatus != WL_CONNECTED) {
    for(JsonVariant stage : stages) {
      wifiMulti.addAP(
        stage["ssid"].as<char*>(),
        stage["pass"].as<char*>()
      );
    }

    int i = 0;
    while (wifiMulti.run() != WL_CONNECTED && i < 100) {
      delay(100);
      i++;
    }

  }

  ArduinoOTA.begin();

  monitorSolar = configDoc["monitorSolar"];

  if (monitorSolar) {
    node.begin(1, Serial);
  }

  unsigned long beforeInitialTimeSyncMillis = millis();

  customTimeSync(configDoc["tzInfo"], configDoc["ntpServer0"]);

  initialTimeSyncLatencyMillis = millis() - beforeInitialTimeSyncMillis;

  JsonVariant activeStage;
  for(JsonVariant stage : stages) {
    if (WiFi.SSID() == stage["ssid"].as<char*>()) {
      activeStage = stage;
      break;
    }
  }

  if (activeStage) {
    updateUrl = activeStage["updateUrl"];
    influxUrl = activeStage["influxUrl"];

    File caCertFile = SPIFFS.open(activeStage["influxCaCertFile"].as<char*>(), "r");
    if (!caCertFile) {
      Serial.printf("File '%s' open failed\n", activeStage["influxCaCertFile"].as<char*>());
      delay(1000);
      return;
    }

    size_t caCertSize = caCertFile.size();
    uint8_t caCertText[caCertSize];
    caCertFile.read(caCertText, caCertSize);
    caCertFile.close();

    BearSSL::X509List *cert = new BearSSL::X509List(caCertText, caCertSize);
    wifiClient.setTrustAnchors(cert);

    wifiClient.setCiphersLessSecure();
  }

  loopStarted = false;
}

void loop() {
  if (loopStarted) {
    return;
  }
  loopStarted = true;

  uint8_t result;
  char warnText[256];

  unsigned long startLoopMillis = millis();

  int dataPos = snprintf(data, dataSize, "%s,location=%s ", nodeType, hostname);

  dataPos += snprintf(&data[dataPos], dataSize - dataPos, "rssi=%di", WiFi.RSSI());
  dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",codeVersion=%f", CODE_VERSION);
  dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",setupLatency=%di", millis() - setupMillis);
  dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",freeHeap=%di", ESP.getFreeHeap());
  dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",heapFragmentation=%di", ESP.getHeapFragmentation());
  dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",maxFreeBlockSize=%di", ESP.getMaxFreeBlockSize());

  dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",readConfigFromSpiffsLatency=%di", readConfigFromSpiffsLatencyMillis);
  dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",initialTimeSyncLatency=%di", initialTimeSyncLatencyMillis);

  JsonArray sensorsConfig = configDoc["sensors"].as<JsonArray>();

  if (sensorsConfig.size() > 0) {
    unsigned long beforeRequestTemperatures = millis();
    sensors.requestTemperatures();

    boolean sleptWaitingForSensor = false;
    int sensorsFound = 0;

    for(JsonVariant sensor : sensorsConfig) {
      uint8_t sensorId[8];

      unsigned long A = millis();

      sscanf(sensor["id"].as<char*>(), "0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx", &sensorId[0], &sensorId[1], &sensorId[2], &sensorId[3], &sensorId[4], &sensorId[5], &sensorId[6], &sensorId[7]);

      unsigned long B = millis();

      float temperature = sensors.getTempC(sensorId);

      if (!sleptWaitingForSensor && temperature == 85.0) {
        // https://github.com/PaulStoffregen/OneWire/issues/58#issuecomment-574392596
        delay(1000);
        sleptWaitingForSensor = true;
        temperature = sensors.getTempC(sensorId);
      }

      unsigned long C = millis();

      if (temperature == DEVICE_DISCONNECTED || temperature == 85.0) {
        snprintf(warnText, 256, "Sensor %s (0x%hhX, 0x%hhX, 0x%hhX, 0x%hhX, 0x%hhX, 0x%hhX, 0x%hhX, 0x%hhX) returned invalid code %f.",
            sensor["name"].as<char*>(), sensorId[0], sensorId[1], sensorId[2], sensorId[3], sensorId[4], sensorId[5], sensorId[6], sensorId[7], temperature);
        logWarning(warnText);
        continue;
      }

      sensorsFound += 1;
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",%s=%f", sensor["name"].as<char*>(), temperature);
    }

    dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",NumSensorsExpected=%di", sensorsConfig.size());
    dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",NumSensorsFound=%di", sensorsFound);

    dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",sensorsLatency=%di", millis() - beforeRequestTemperatures);
  }

  time_t tnow = time(nullptr);
  struct tm *tm = localtime(&tnow);

  if (monitorSolar) {
    if (0 /* TODO: compare controller time with NTP time and only update when drifted */) {
      unsigned long timeSyncStartWriteMillis = millis();

      node.setTransmitBuffer(0, pack_as_int(tm->tm_min, tm->tm_sec));
      node.setTransmitBuffer(1, pack_as_int(tm->tm_mday, tm->tm_hour));
      node.setTransmitBuffer(2, pack_as_int((tm->tm_year + 1900) - 2000, tm->tm_mon + 1));

      result = node.writeMultipleRegisters(0x9013, 3);
      if (result == node.ku8MBSuccess) {
        logInfo("Successfully set charge controller date/time.");
      } else {
        snprintf(warnText, 256, "0x%02X - Failed to set charge controller date/time...", result);
        logWarning(warnText);
      }

      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",WriteCtrlrTimeLatencyMillis=%di", millis() - timeSyncStartWriteMillis);
    }

    result = node.readInputRegisters(0x3008, 0x1);
    if (result == node.ku8MBSuccess) {
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_MODE=%di", node.getResponseBuffer(0x00));
    } else {
      snprintf(warnText, 256, "0x%02X - Could not read 0x3008 from device...", result);
      logWarning(warnText);
    }

    float chargingEquipmentOutputVoltage;

    // Read 16 registers starting at 0x3100)
    result = node.readInputRegisters(0x3100, 0x13);
    if (result == node.ku8MBSuccess) {
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_INPUT_VOLTAGE=%f", node.getResponseBuffer(0x00)/100.0f);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_INPUT_CURRENT=%f", node.getResponseBuffer(0x01)/100.0f);

      uint32_t chargingEquipmentInputPower = node.getResponseBuffer(0x03);
      chargingEquipmentInputPower = (chargingEquipmentInputPower << 16) | node.getResponseBuffer(0x02);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_INPUT_POWER=%f", chargingEquipmentInputPower/100.0f);

      chargingEquipmentOutputVoltage = node.getResponseBuffer(0x04)/100.0f;

      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_OUTPUT_VOLTAGE=%f", chargingEquipmentOutputVoltage);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_OUTPUT_CURRENT=%f", node.getResponseBuffer(0x05)/100.0f);

      uint32_t chargingEquipmentOutputPower = node.getResponseBuffer(0x07);
      chargingEquipmentOutputPower = (chargingEquipmentOutputPower << 16) | node.getResponseBuffer(0x06);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_OUTPUT_POWER=%f", chargingEquipmentOutputPower/100.0f);

      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",DISCHARGING_EQUIPMENT_OUTPUT_VOLTAGE=%f", node.getResponseBuffer(0x0C)/100.0f);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",DISCHARGING_EQUIPMENT_OUTPUT_CURRENT=%f", node.getResponseBuffer(0x0D)/100.0f);

      uint32_t dischargingEquipmentOutputPower = node.getResponseBuffer(0x0F);
      dischargingEquipmentOutputPower = (dischargingEquipmentOutputPower << 16) | node.getResponseBuffer(0x0E);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",DISCHARGING_EQUIPMENT_OUTPUT_POWER=%f", dischargingEquipmentOutputPower/100.0f);

      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",BATTERY_TEMPERATURE=%f", node.getResponseBuffer(0x10)/100.0f);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",TEMPERATURE_INSIDE_EQUIPMENT=%f", node.getResponseBuffer(0x11)/100.0f);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",POWER_COMPONENTS_TEMPERATURE=%f", node.getResponseBuffer(0x12)/100.0f);

    } else {
      snprintf(warnText, 256, "0x%02X - Could not read 0x3100 - 0x3112 from device...", result);
      logWarning(warnText);
    }

    result = node.readInputRegisters(0x311A, 0x2);
    if (result == node.ku8MBSuccess) {
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",BATTERY_SOC=%f", node.getResponseBuffer(0x00)/100.0f);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",REMOTE_BATTERY_TEMPERATURE=%f", node.getResponseBuffer(0x01)/100.0f);
    } else {
      snprintf(warnText, 256, "0x%02X - Could not read 0x311A - 0x311B from device...", result);
      logWarning(warnText);
    }

    result = node.readInputRegisters(0x3200, 0x2);
    if (result == node.ku8MBSuccess) {
      uint16_t batteryStatus = node.getResponseBuffer(0x00);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",BATTERY_STATUS.WrongRatedVoltageId=%di", (batteryStatus >> 15) & 0x1);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",BATTERY_STATUS.BatteryResistanceAbnormal=%di", (batteryStatus >> 8) & 0x1);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",BATTERY_STATUS.BatteryThermalStatus=%di", (batteryStatus >> 4) & 0x7);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",BATTERY_STATUS.BatteryVoltageStatus=%di", batteryStatus & 0x7);

      uint16_t chargingEquipmentStatus = node.getResponseBuffer(0x01);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_STATUS.InputVoltStatus=%di", (chargingEquipmentStatus >> 14));
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_STATUS.ChargingMosfetShort=%di", (chargingEquipmentStatus >> 13) & 0x1);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_STATUS.ChargingOrAntiReverseMosfetShort=%di", (chargingEquipmentStatus >> 12) & 0x1);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_STATUS.AntiReverseMosfetShort=%di", (chargingEquipmentStatus >> 11) & 0x1);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_STATUS.InputOverCurrent=%di", (chargingEquipmentStatus >> 10) & 0x1);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_STATUS.LoadOverCurrent=%di", (chargingEquipmentStatus >> 9) & 0x1);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_STATUS.LoadShort=%di", (chargingEquipmentStatus >> 8) & 0x1);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_STATUS.LoadMosfetShort=%di", (chargingEquipmentStatus >> 7) & 0x1);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_STATUS.PvInputShort=%di", (chargingEquipmentStatus >> 4) & 0x1);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_STATUS.ChargingStatus=%di", (chargingEquipmentStatus >> 2) & 0x3);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_STATUS.Status=%di", (chargingEquipmentStatus >> 1) & 0x1);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CHARGING_EQUIPMENT_STATUS.State=%di", chargingEquipmentStatus & 0x1);
    } else {
      snprintf(warnText, 256, "0x%02X - Could not read 0x3200 - 0x3201 from device...", result);
      logWarning(warnText);
    }

    result = node.readInputRegisters(0x3300, 0x0E);
    if (result == node.ku8MBSuccess) {
      uint32_t consumedEnergyToday = node.getResponseBuffer(0x05);
      consumedEnergyToday = (consumedEnergyToday << 16) | node.getResponseBuffer(0x04);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",CONSUMED_ENERGY_TODAY=%f", consumedEnergyToday/100.0f);

      uint32_t generatedEnergyToday = node.getResponseBuffer(0x0D);
      generatedEnergyToday = (generatedEnergyToday << 16) | node.getResponseBuffer(0x0C);
      dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",GENERATED_ENERGY_TODAY=%f", generatedEnergyToday/100.0f);
    } else {
      snprintf(warnText, 256, "0x%02X - Could not read 0x3300 - 0x330E from device...", result);
      logWarning(warnText);
    }

    float lowPowerModeThreshold = configDoc["lowPowerModeThreshold"];
    if (lowPowerModeThreshold > 0) {
      result = node.readCoils(0x2, 0x1);

      if (result == node.ku8MBSuccess) {
        bool currentLoadStatus = node.getResponseBuffer(0x00) > 0;

        time_t tnow = time(nullptr);
        struct tm *tm = localtime(&tnow);

        bool desiredLoadStatus = (chargingEquipmentOutputVoltage >= lowPowerModeThreshold) || (tm->tm_hour % 2 == 0 && tm->tm_min < 15);

        bool shouldUpdateState = currentLoadStatus != desiredLoadStatus;

        if (shouldUpdateState) {
          node.writeSingleCoil(0x2, desiredLoadStatus ? 1 : 0);
        }
      }
    }

  }

  // Only check for updates on minutes which are multiples of ten
  if (updateUrl && tm->tm_min % 10 == 0) {
    unsigned long checkUpdatesStartMillis = millis();
    t_httpUpdate_return ret = ESPhttpUpdate.update(wifiClient, updateUrl, CODE_VERSION);
    switch(ret) {
      case HTTP_UPDATE_FAILED:
          //Serial.printf("[update] Could not update from '%s'\n", updateUrl);
          break;
      case HTTP_UPDATE_NO_UPDATES:
          //Serial.println("[update] Server indicated no update is required.");
          break;
      case HTTP_UPDATE_OK:
          Serial.println("[update] Update ok."); // may not called we reboot the ESP
          break;
    }
    dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",checkUpdatesLatency=%di", millis() - checkUpdatesStartMillis);
  }

  unsigned long writeToInfluxStartMillis = millis();

  writePointToInflux(data, dataPos);

  if (tm->tm_min % 12 == 0) {
    dataPos = snprintf(data, dataSize, "%s,location=%s ", nodeType, hostname);

    dataPos += snprintf(&data[dataPos], dataSize - dataPos, "writeInfluxLatency=%di", millis() - writeToInfluxStartMillis);
    dataPos += snprintf(&data[dataPos], dataSize - dataPos, ",totalLatency=%di", millis() - setupMillis);

    writePointToInflux(data, dataPos);
  }

  // TODO: Consider sleeping until the start of the next minute instead of this strategy
  unsigned long sleepMicros = (60 * 1000 * 1000) - ((millis() - setupMillis) * 1000);

  ESP.deepSleep(sleepMicros);
}
