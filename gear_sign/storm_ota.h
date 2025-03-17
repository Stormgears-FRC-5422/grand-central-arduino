#include <WiFi.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>

// Static IP configuration
bool ota_connected = false;

const char *ssid = "stormnet";
const char *password = "steveRobot";
IPAddress local_ip(192, 168, 200, 232);  // Set the static IP here
const char *hostname = "gearsign";

IPAddress gateway(192, 168, 200, 1);     // Set the gateway IP (usually your router)
IPAddress subnet(255, 255, 255, 0);    // Set the subnet mask

void storm_ota_setup() {
  WiFi.mode(WIFI_STA);
  // Set Static IP
  WiFi.config(local_ip, gateway, subnet);  // Apply static IP configuration
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult(15000) != WL_CONNECTED) {
    Serial.println("Connection Failed! moving on...");
    return;
    // delay(5000);
    // ESP.restart();
  }

  // Port defaults to 3232
  ArduinoOTA.setPort(3232);
  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(hostname);
  // No authentication by default
  // ArduinoOTA.setPassword("admin");
  // Password can be set with its md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }
      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Hostname: ");
  Serial.print(hostname);

  ota_connected = true;
}

void storm_ota_loop() {
  if (ota_connected) {
    ArduinoOTA.handle();
  }
}
