/*
  Johannes Huber
  https://github.com/joehubi/growbox
*/

/*
  Libray
*/
// ESP8266
#include <ESP8266WiFi.h>        // https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/ESP8266WiFi.h
#include <ESPAsyncTCP.h>        // https://github.com/me-no-dev/ESPAsyncTCP
#include <ESPAsyncWebServer.h>  // https://github.com/me-no-dev/ESPAsyncWebServer
#include <FS.h>                 // https://github.com/esp8266/Arduino/blob/master/cores/esp8266/FS.h

#include <Wire.h>               // https://www.arduino.cc/en/Reference/Wire

#include <SoftwareSerial.h>     //https://www.arduino.cc/en/Reference/SoftwareSerial
#include <ArduinoJson.h>        //https://github.com/bblanchon/ArduinoJson

float TS01_sim = 0.0;
int FS01_sim = 0;

SoftwareSerial nodemcu(D6, D5);

// Replace with your network credentials
const char* ssid = "";
const char* password = "";

String dummy_state = "OFF";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

/*
 * Funktionen
 */

String getTS01() {
  Serial.println(TS01_sim);
  return String(TS01_sim);
}
  
String getFS01() {
  Serial.println(FS01_sim);
  return String(FS01_sim);
}

// Multiplex processor for all variables 
String processor(const String& var){
  Serial.println("\n");
  Serial.println("processor: " + var + "\n");
  if(var == "STATE"){
    Serial.print(dummy_state);
    return dummy_state;
  }
  else if (var == "TS01"){
    return getTS01();
  }
  else if (var == "FS01"){
    return getFS01();
  }
}

 /*
  * Setup
  */
void setup(){

  // Init SC
  Serial.begin(115200);
  nodemcu.begin(4800);
 
  //??????
  //while (!Serial) continue;
  //??????

  // Initialize SPIFFS
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  /*
  WLAN
  */
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());


  /*
  Web Server GUI
  */
  // Load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });
  // Called when <IP>/ browsed (Refresh button)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  // Called when <IP>/on browsed
  server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request){  
    dummy_state = "ON";
    request->send(SPIFFS, "/index.html", String(), false, processor);
  }); 
  // Called when <IP>/off browsed
  server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request){  
    dummy_state = "OFF";
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Start server
  server.begin();
}

 /*
  * Loop
  */
  
void loop(){

  StaticJsonDocument<300> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, nodemcu);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Fetch values.
  //
  // Most of the time, you can rely on the implicit casts.
  // In other case, you can do doc["time"].as<long>();
  const char* growbox = doc["growbox"];
  long time = doc["time"];

  TS01_sim = doc["data"][0];
  FS01_sim = doc["data"][1];

  // Print values.
  Serial.println(growbox);
  Serial.println(time);
  Serial.println(TS01_sim, 1);
  Serial.println(FS01_sim, 0);
  
  delay(1000);
  
}
