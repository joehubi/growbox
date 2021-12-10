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

word debug_level = 2;

float dutycycle = 0.0;
int dutycycle_int = 0;

float TS01 = 0.0;
float TS02 = 0.0;
float TS03 = 0.0;
int TS01_int = 0;
int TS02_int = 0;
int TS03_int = 0;

int _sends = 0;
int _reads = 0;

int word_hour = 0;
int word_minute = 0;

unsigned long millisec;           // time-ms
unsigned long cycle_read_pre = 0;
const unsigned long cycle_read_period = 456;  
unsigned long cycle_send_pre = 0;
const unsigned long cycle_send_period = 5000;  

const int mcutimeout = 300;
const int mcubaudrate = 1200;
const int txbuffer = 150;
const int rxbuffer = 500;
SoftwareSerial nodemcu(D6, D5);           // Serial conncetion to NodeMCU

unsigned long arduino_time_ms = 0;

const char* ssid = "Pumuckel";            // wifi network
const char* password = "Stiller_83";      // wifi network

byte dummy_state = 0;                      // 0=OFF, 1=ON
String dummy_state_str = "...";
byte dummy_state_ctl = 2;                  // 0=Manual ON, 1=Manual OFF, 2=AUTOMATIC
String dummy_state_ctl_str = "...";

byte pipevent_state = 0;                      // 0=OFF, 1=ON
String pipevent_state_str = "...";
byte pipevent_state_ctl = 2;                  // 0=Manual ON, 1=Manual OFF, 2=AUTOMATIC
String pipevent_state_ctl_str = "...";

byte led_state = 0;                      // 0=OFF, 1=ON
String led_state_str = "...";
byte led_state_ctl = 2;                  // 0=Manual ON, 1=Manual OFF, 2=AUTOMATIC
String led_state_ctl_str = "...";

byte ventilator_state = 0;                      // 0=OFF, 1=ON
String ventilator_state_str = "...";
byte ventilator_state_ctl = 2;                  // 0=Manual ON, 1=Manual OFF, 2=AUTOMATIC
String ventilator_state_ctl_str = "...";

// Timer for timed loops
int cycle_1500ms = 1000;      
unsigned long cycle_1500ms_dt;

int cycle_500ms = 500;      
unsigned long cycle_500ms_dt;

AsyncWebServer server(80);                // Create AsyncWebServer object on port 80






// #################################################################### Functions

String state_ctl_txt(int ctl){
  String state_ctl_txt = "";
  switch (ctl) {
    case 0:
      state_ctl_txt = "Manual OFF";
      break;
    case 1:
      state_ctl_txt = "Manual ON";
      break;
    case 2:
      state_ctl_txt = "AUTOMATIC";
      break;
  }
  return state_ctl_txt;
}

String state_txt(int state){
  String state_txt = "";
  switch (state) {
    case 0:
      state_txt = "OFF";
      break;
    case 1:
      state_txt = "ON";
      break;
  }
  return state_txt;
}

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
String getTS02() {
  return String(TS02);
}
String getTS03() {
  return String(TS03);
}
String getdutycycle() {
  return String(dutycycle);
}
String getwordhour() {
  return String(word_hour);
}
String getwordminute() {
  return String(word_minute);
}

// Multiplex processor for all data variables 
String processor(const String& var){

  if(var == "dummy_state_str"){
    return dummy_state_str;
  }
  if(var == "dummy_state_ctl_str"){
    return dummy_state_ctl_str;
  }
  if(var == "pipevent_state_str"){
    return pipevent_state_str;
  }
  if(var == "pipevent_state_ctl_str"){
    return pipevent_state_ctl_str;
  }
  if(var == "led_state_str"){
    return led_state_str;
  }
  if(var == "led_state_ctl_str"){
    return led_state_ctl_str;
  }
  if(var == "ventilator_state_str"){
    return ventilator_state_str;
  }
  if(var == "ventilator_state_ctl_str"){
    return ventilator_state_ctl_str;
  }      
  else if (var == "TS01"){
    return getTS01();
  }
  else if (var == "TS02"){
    return getTS02();
  }
  else if (var == "TS03"){
    return getTS03();
  }
  else if (var == "dutycycle"){
    return getdutycycle();
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
  else if (var == "word_hour"){
    return getwordhour();
  }
  else if (var == "word_minute"){
    return getwordminute();
  }
}


// #################################################################### Setup

void setup(){

  nodemcu.setTimeout(mcutimeout);
  nodemcu.begin(mcubaudrate);    // set low baud rate due to json communication

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

  // Pipe ventilator state Button
  
  server.on("/b2_off", HTTP_GET, [](AsyncWebServerRequest *request){  
    pipevent_state_ctl = 0;
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  server.on("/b2_on", HTTP_GET, [](AsyncWebServerRequest *request){  
    pipevent_state_ctl = 1;
    request->send(SPIFFS, "/index.html", String(), false, processor);
  }); 
  server.on("/b2_auto", HTTP_GET, [](AsyncWebServerRequest *request){  
    request->send(SPIFFS, "/index.html", String(), false, processor);
    pipevent_state_ctl = 2;
  });

  // LED state Button
  
  server.on("/b3_off", HTTP_GET, [](AsyncWebServerRequest *request){  
    led_state_ctl = 0;
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  server.on("/b3_on", HTTP_GET, [](AsyncWebServerRequest *request){  
    led_state_ctl = 1;
    request->send(SPIFFS, "/index.html", String(), false, processor);
  }); 
  server.on("/b3_auto", HTTP_GET, [](AsyncWebServerRequest *request){  
    request->send(SPIFFS, "/index.html", String(), false, processor);
    led_state_ctl = 2;
  });

  // ventilator state Button
  
  server.on("/b4_off", HTTP_GET, [](AsyncWebServerRequest *request){  
    ventilator_state_ctl = 0;
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  server.on("/b4_on", HTTP_GET, [](AsyncWebServerRequest *request){  
    ventilator_state_ctl = 1;
    request->send(SPIFFS, "/index.html", String(), false, processor);
  }); 
  server.on("/b4_auto", HTTP_GET, [](AsyncWebServerRequest *request){  
    request->send(SPIFFS, "/index.html", String(), false, processor);
    ventilator_state_ctl = 2;
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
  
    StaticJsonDocument<rxbuffer> rxdoc;
    DeserializationError error = deserializeJson(rxdoc, nodemcu);

    if (error) {
      if (debug_level >= 2) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());       
      }
    }
      else {
        if (rxdoc.isNull() || rxdoc.memoryUsage() < 284) {       
        } else {
          arduino_time_ms = rxdoc["time"];              // extract time from arduino   
          TS01_int = rxdoc["data"][0];                      // extract data
          TS02_int = rxdoc["data"][1];
          TS03_int = rxdoc["data"][2];
          dutycycle_int = rxdoc["data"][3];
          
          TS01 = float(TS01_int)/10;
          TS02 = float(TS02_int)/10;
          TS03 = float(TS03_int)/10;      
          dutycycle = float(dutycycle_int)/10; 
             
          dummy_state =       rxdoc["state"][0];        
          pipevent_state =    rxdoc["state"][1];
          led_state =         rxdoc["state"][2];
          ventilator_state =  rxdoc["state"][3];

          word_hour =         rxdoc["rtc_time"][0];
          word_minute =       rxdoc["rtc_time"][1];
         
          _reads++;

          if (debug_level >= 1) {           
            serializeJson(rxdoc, Serial);     
            Serial.print("\n");   
          }         
          if (debug_level >= 2) {
            Serial.print("###READ### rx-buffer: "+String(rxdoc.size())+","+String(sizeof(rxdoc))+","+String(rxdoc.memoryUsage()));            // Debugging      
            Serial.print("\n");               
          }    
      }
    }
    
// ############################ Buttons 

    dummy_state_ctl_str       = state_ctl_txt(dummy_state_ctl);
    pipevent_state_ctl_str    = state_ctl_txt(pipevent_state_ctl);
    led_state_ctl_str         = state_ctl_txt(led_state_ctl);
    ventilator_state_ctl_str  = state_ctl_txt(ventilator_state_ctl);

// ############################ States from Arduino 

    dummy_state_str       = state_txt(dummy_state);
    pipevent_state_str    = state_txt(pipevent_state);
    led_state_str         = state_txt(led_state);
    ventilator_state_str  = state_txt(ventilator_state);
       
  }
  
// ############################ SEND 

  if ((millisec - cycle_send_pre > cycle_send_period)) {
    cycle_send_pre = millisec;
    
    StaticJsonDocument<txbuffer> txdoc;
  
    txdoc["growbox"] = "to_Arduino";
    JsonArray state = txdoc.createNestedArray("switches");      // Add state array
    state.add(dummy_state_ctl);                                           // Put data in array
    state.add(pipevent_state_ctl);
    state.add(led_state_ctl);
    state.add(ventilator_state_ctl);
     
    //Send data to Arduino
    serializeJson(txdoc, nodemcu);

    _sends++;

    if (debug_level >= 1) {            
      serializeJson(txdoc, Serial);                               
      Serial.print("\n");
    }
    if (debug_level >= 2) {  
      Serial.print("###SEND### tx-buffer: "+String(txdoc.size())+","+String(sizeof(txdoc))+","+String(txdoc.memoryUsage()));  
      Serial.print("\n");             
    }
  }
   
  delay(1);
}
  
