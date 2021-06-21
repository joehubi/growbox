/*
  Johannes Huber
  https://github.com/joehubi/growbox
*/

// Import required libraries
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <Wire.h>

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

// Replaces placeholder with dummy state value
String processor(const String& var){
  Serial.println(var);
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
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });



  /*
  Datenkommunikation
  */
  // Route to set ON
  server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request){  
    dummy_state = "ON";
    request->send(SPIFFS, "/index.html", String(), false, processor);
  }); 
  // Route to set OFF
  server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request){  
    dummy_state = "OFF";
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  server.on("/TS01", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", getTS01().c_str());
  });
  server.on("/FS01", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", getFS01().c_str());
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
