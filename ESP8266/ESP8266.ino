/*
  Johannes Huber
  https://github.com/joehubi/growbox
*/

// #################################################################### Libray

#include <ESP8266WiFi.h>        // https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/ESP8266WiFi.h
#include <ESPAsyncTCP.h>        // https://github.com/me-no-dev/ESPAsyncTCP
#include <ESPAsyncWebServer.h>  // https://github.com/me-no-dev/ESPAsyncWebServer
#include <FS.h>                 // https://github.com/esp8266/Arduino/blob/master/cores/esp8266/FS.h

#include <Wire.h>               // https://www.arduino.cc/en/Reference/Wire

#include <SoftwareSerial.h>     //https://www.arduino.cc/en/Reference/SoftwareSerial
#include <ArduinoJson.h>        //https://github.com/bblanchon/ArduinoJson


// #################################################################### Variables

word debug_level = 0;

float TS01 = 0.0;
int FS01 = 0;

int _sends = 0;
int _reads = 0;

unsigned long millisec;           // time-ms
unsigned long cycle_read_pre = 0;
const unsigned long cycle_read_period = 1000;  
unsigned long cycle_send_pre = 0;
const unsigned long cycle_send_period = 10000;  

SoftwareSerial nodemcu(D6, D5);           // Serial conncetion to NodeMCU

unsigned long arduino_time_ms = 0;

const char* ssid = "Pumuckel";            // wifi network
const char* password = "Stiller_83";      // wifi network

int dummy_state = 0;                      // 0=OFF, 1=ON
String dummy_state_str = "...";

int dummy_state_ctl = 0;                  // 0=Manual ON, 1=Manual OFF, 2=AUTOMATIC
String dummy_state_ctl_str = "...";

// Timer for timed loops
int cycle_1500ms = 1000;      
unsigned long cycle_1500ms_dt;

int cycle_500ms = 500;      
unsigned long cycle_500ms_dt;

AsyncWebServer server(80);                // Create AsyncWebServer object on port 80









// #################################################################### Functions

// Get-functions for collecting the data from the arduino and display it on the webserver
String gettime() {
  return String(arduino_time_ms);
}
String getsends() {
  return String(_sends);
}
String getreads() {
  return String(_reads);
}
String getTS01() {
  return String(TS01);
}
String getFS01() {
  return String(FS01);
}

// Multiplex processor for all data variables 
String processor(const String& var){

  if(var == "dummy_state_str"){
    return dummy_state_str;
  }
  if(var == "dummy_state_ctl_str"){
    return dummy_state_ctl_str;
  }
  else if (var == "TS01"){
    return getTS01();
  }
  else if (var == "FS01"){
    return getFS01();
  }
  else if (var == "arduino_time_ms"){
    return gettime();
  }
  else if (var == "_reads"){
    return getreads();
  }
  else if (var == "_sends"){
    return getsends();
  }
}



// #################################################################### Setup

void setup(){

  nodemcu.setTimeout(20);
  nodemcu.begin(1200);    // set low baud rate due to json communication

  Serial.begin(9600);   // for serial output in window / debugging
   
  // Initialize SPIFFS (for web server)
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

// ######################## Wifi

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi ...");
  }
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());


// ######################## Webserver GUI

  // Load style.css file
  
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Called when <IP>/ browsed (Refresh button)
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });


  // Dummy State Button (Called when <IP>/b1_off browsed)
  
  server.on("/b1_off", HTTP_GET, [](AsyncWebServerRequest *request){  
    dummy_state_ctl = 0;
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  server.on("/b1_on", HTTP_GET, [](AsyncWebServerRequest *request){  
    dummy_state_ctl = 1;
    request->send(SPIFFS, "/index.html", String(), false, processor);
  }); 
  server.on("/b1_auto", HTTP_GET, [](AsyncWebServerRequest *request){  
    request->send(SPIFFS, "/index.html", String(), false, processor);
    dummy_state_ctl = 2;
  });


  // Start server
  server.begin();
}


// #################################################################### Loop

void loop(){

  millisec = millis();

// ############################ READ

  if ((millisec - cycle_read_pre > cycle_read_period)) {
    cycle_read_pre = millisec; 
  
    StaticJsonDocument<300> rxdoc;
    DeserializationError error = deserializeJson(rxdoc, nodemcu);

    if (error) {
      if (debug_level == 2) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());       
      }
    }
      else {
        if (rxdoc.isNull()) {       
        } else {
          arduino_time_ms = rxdoc["time"];              // extract time from arduino   
          TS01 = rxdoc["data"][0];                      // extract data
          FS01 = rxdoc["data"][1];
          dummy_state = rxdoc["state"][0];        
          _reads++;

          if (debug_level == 1) {
            Serial.print("###READ### rx-buffer: "+String(rxdoc.size())+","+String(sizeof(rxdoc))+","+String(rxdoc.memoryUsage()));            // Debugging      
            Serial.print("\n");             
            serializeJson(rxdoc, Serial);     
            Serial.print("\n");   
          }         
      }
    }
  }
  
// ############################ SEND 

  if ((millisec - cycle_send_pre > cycle_send_period)) {
    cycle_send_pre = millisec;
    
    StaticJsonDocument<200> txdoc;
  
    txdoc["growbox"] = "to_Arduino";
    JsonArray state = txdoc.createNestedArray("switches");      // Add state array
    state.add(dummy_state_ctl);                                           // Put data in array
  
    //Send data to Arduino
    serializeJson(txdoc, nodemcu);

    _sends++;

    if (debug_level == 1) {  
      Serial.print("###SEND### tx-buffer: "+String(txdoc.size())+","+String(sizeof(txdoc))+","+String(txdoc.memoryUsage()));  
      Serial.print("\n");             
      serializeJson(txdoc, Serial);                               
      Serial.print("\n");
    }
  }

// ############################ Buttons 

  switch (dummy_state_ctl) {
    case 0:
      dummy_state_ctl_str = "Manual OFF";
      break;
    case 1:
      dummy_state_ctl_str = "Manual ON";
      break;
    case 2:
      dummy_state_ctl_str = "AUTOMATIC";
      break;  
    default:
      break;
  }

// ############################ States from Arduino 

  switch (dummy_state) {
    case 0:
      dummy_state_str = "OFF";
      break;
    case 1:
      dummy_state_str = "ON";
      break;
    default:
      break;
  }
    
  delay(1);
}
  
