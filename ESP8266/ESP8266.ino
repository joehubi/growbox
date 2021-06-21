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

  float TS01_sim = 23.1;
  int FS01_sim = 53;
  
// Replace with your network credentials
const char* ssid = "Pumuckel";
const char* password = "Stiller_83";

String dummy_state = "OFF";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

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
 
void setup(){

  // Serial port for debugging purposes
  Serial.begin(115200);

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
 
void loop(){


  /*
  Simulation
  */
  if (FS01_sim > 56) {
    FS01_sim = 52; 
    }
    else {
      FS01_sim = FS01_sim + 1;
    }

  if (TS01_sim > 25.0) {
    TS01_sim = 23.2;
    } 
    else {
      TS01_sim = TS01_sim + 0.1;
    }
  
  delay(500);
}
