/*
  Johannes Huber
  https://github.com/joehubi/growbox
  16.05.24
*/

// #################################################################### Libray

#include <ESP8266WiFi.h>        // https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/ESP8266WiFi.h
#include <ESPAsyncTCP.h>        // https://github.com/me-no-dev/ESPAsyncTCP
#include <ESPAsyncWebServer.h>  // https://github.com/me-no-dev/ESPAsyncWebServer
#include <FS.h>                 // https://github.com/esp8266/Arduino/blob/master/cores/esp8266/FS.h

#include <Wire.h>               // https://www.arduino.cc/en/Reference/Wire

#define SLAVE_ADDRESS 9
#define TIMEOUT_MS 5000         // Timeout in Millisekunden

// #################################################################### Variables

byte debug_level = 0;

float dutycycle = 0.0;
int dutycycle_int = 0;

float TS01 = 0.0;
float TS02 = 0.0;
float TS03 = 0.0;
int TS01_int = 0;
int TS02_int = 0;
int TS03_int = 0;

int debocap_percentage = 0; // 0..100 % Bodenfeuchte

byte word_hour = 0;
byte word_minute = 0;

unsigned long millisec;           // time-ms
unsigned long cycle_read_pre = 0;
const unsigned long cycle_read_period = 5000;  // in ms
unsigned long cycle_send_pre = 0;
const unsigned long cycle_send_period = 2000;  // in ms

//int port = 8888;  //Port number
//WiFiServer server(port);

const char* ssid = "Pumuckel";            // wifi network
const char* password = "Stiller_83";      // wifi network
AsyncWebServer server(80);                // Create AsyncWebServer object on port 8888

byte heater_state = 0;                     // 0=OFF, 1=ON
String heater_state_str = "...";
byte heater_state_ctl = 0;                 // 0=Manual ON, 1=Manual OFF, 2=AUTOMATIC
String heater_state_ctl_str = "...";

byte pipevent_state = 0;                   // 0=OFF, 1=ON
String pipevent_state_str = "...";
byte pipevent_state_ctl = 2;               // 0=OFF, 1=ON, 2=Intervall
String pipevent_state_ctl_str = "...";
byte pipevent_dutycycle_ctl = 15;          // Duty cycle control (%)

byte led_state = 0;                      // 0=OFF, 1=ON
String led_state_str = "...";
byte led_state_ctl = 2;                  // 0=Manual ON, 1=Manual OFF, 2=AUTOMATIC
String led_state_ctl_str = "...";

byte ventilator_state = 0;                      // 0=OFF, 1=ON
String ventilator_state_str = "...";
byte ventilator_state_ctl = 2;                  // 0=Manual ON, 1=Manual OFF, 2=AUTOMATIC
String ventilator_state_ctl_str = "...";

const byte data_bytes_to_slave = 5;
const byte data_bytes_request_from_slave = 17;

byte data_to_slave[data_bytes_to_slave]; // Array zur Speicherung der Werte
byte data_request[data_bytes_request_from_slave]; // Array zur Speicherung der empfangenen Daten

byte msg_req_feedback = 0;

// #################################################################### Functions

String state_ctl_txt(int ctl){
  String _txt = "";
  switch (ctl) {
    case 0:
      _txt = "Manual OFF";
      break;
    case 1:
      _txt = "Manual ON";
      break;
    case 2:
      _txt = "AUTOMATIC";
      break;
  }
  return _txt;
}

String state_ctl_led_txt(int ctl){
  String _txt = "";
  switch (ctl) {
    case 0:
      _txt = "Manual OFF";
      break;
    case 1:
      _txt = "Manual ON";
      break;
    case 2:
      _txt = "18/6 (4-22)";
      break;
    case 3:
      _txt = "12/12 (8-20)";
      break;
    case 4:
      _txt = "ARRAY";
      break;      
  }
  return _txt;
}

String state_txt(int state){
  String _txt = "";
  switch (state) {
    case 0:
      _txt = "OFF";
      break;
    case 1:
      _txt = "ON";
      break;
  }
  return _txt;
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
String getdebocap_percentage() {
  return String(debocap_percentage);
}
String getwordhour() {
  return String(word_hour);
}
String getwordminute() {
  return String(word_minute);
}
String getmsg_req_feedback() {
  return String(msg_req_feedback);
}

// Multiplex processor for all data variables 
String processor(const String& var){

  if(var == "heater_state_str"){
    return heater_state_str;
  }
  if(var == "heater_state_ctl_str"){
    return heater_state_ctl_str;
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
  else if (var == "debocap_percentage"){
    return getdebocap_percentage();
  }       
  else if (var == "word_hour"){
    return getwordhour();
  }
  else if (var == "word_minute"){
    return getwordminute();
  }
  else if (var == "msg_req_feedback"){
    return getmsg_req_feedback();
  }
  return "not_valid";
}


// #################################################################### Setup

void setup(){

  Wire.begin(D5, D6);   // Initialisierung des I2C - D5 (SDA) und D6 (SCL) für I2C
  Serial.begin(9600);   // for serial output in window / debugging
   
  // Initialize SPIFFS (for web server)
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

// ######################## Wifi

  //WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi ...");
  }
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

// ######################## Webserver GUI

  Serial.println("Start programme");
  
  // Load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Called when <IP>/ browsed (Refresh button)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });


  // Heater State Button (Called when <IP>/b1_off browsed)
  
  server.on("/b1_off", HTTP_GET, [](AsyncWebServerRequest *request){  
    heater_state_ctl = 0;
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  server.on("/b1_on", HTTP_GET, [](AsyncWebServerRequest *request){  
    heater_state_ctl = 1;
    request->send(SPIFFS, "/index.html", String(), false, processor);
  }); 
  server.on("/b1_auto", HTTP_GET, [](AsyncWebServerRequest *request){  
    request->send(SPIFFS, "/index.html", String(), false, processor);
    heater_state_ctl = 2;
  });

  // Pipe ventilator state Button
  
  server.on("/b21_off", HTTP_GET, [](AsyncWebServerRequest *request){  
    pipevent_state_ctl = 0;
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  server.on("/b21_on", HTTP_GET, [](AsyncWebServerRequest *request){  
    pipevent_state_ctl = 1;
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  server.on("/b21_on_intervall", HTTP_GET, [](AsyncWebServerRequest *request){  
    pipevent_state_ctl = 2;
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
    
  // Pipe ventilator duty cycle Button
  server.on("/b22_15", HTTP_GET, [](AsyncWebServerRequest *request){  
    request->send(SPIFFS, "/index.html", String(), false, processor);
    pipevent_dutycycle_ctl = 15;
  });  
  server.on("/b22_30", HTTP_GET, [](AsyncWebServerRequest *request){  
    request->send(SPIFFS, "/index.html", String(), false, processor);
    pipevent_dutycycle_ctl = 30;
  });  
  server.on("/b22_50", HTTP_GET, [](AsyncWebServerRequest *request){  
    request->send(SPIFFS, "/index.html", String(), false, processor);
    pipevent_dutycycle_ctl = 50;
  });  
  server.on("/b22_80", HTTP_GET, [](AsyncWebServerRequest *request){  
    request->send(SPIFFS, "/index.html", String(), false, processor);
    pipevent_dutycycle_ctl = 80;
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
  server.on("/b3_1806", HTTP_GET, [](AsyncWebServerRequest *request){  
    request->send(SPIFFS, "/index.html", String(), false, processor);
    led_state_ctl = 2;
  });
  server.on("/b3_1212", HTTP_GET, [](AsyncWebServerRequest *request){  
    request->send(SPIFFS, "/index.html", String(), false, processor);
    led_state_ctl = 3;
  });
    server.on("/b3_array", HTTP_GET, [](AsyncWebServerRequest *request){  
    request->send(SPIFFS, "/index.html", String(), false, processor);
    led_state_ctl = 4;
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
  Serial.println("Server started");
  
}


// #################################################################### Loop

void loop(){

  millisec = millis();

// ############################ READ

  if ((millisec - cycle_read_pre > cycle_read_period)) {
    cycle_read_pre = millisec; 

    // Sende eine Anfrage an den Slave
    Serial.println("Request data on I2C from SLAVE");

    unsigned long startTime = millis(); // Startzeit messen
      
    Wire.requestFrom(SLAVE_ADDRESS, data_bytes_request_from_slave); // Fordere X Bytes vom Slave an

    while(Wire.available() < data_bytes_request_from_slave) {
      // Überprüfe, ob ein Timeout aufgetreten ist
      if (millis() - startTime >= TIMEOUT_MS) {
        Serial.println("Timeout beim Warten auf Antwort vom Slave!");
        break;
      }
      // Warte, bis alle Bytes empfangen wurden
      delay(1);
    }

    // Werte in Array schreiben
    int i = 0;
    while(Wire.available()) {
      // Empfange die Daten vom Slave und schreibe sie in das Array
      data_request[i] = Wire.read();
      Serial.print("Value ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(data_request[i]);
      i++;
    }

    // Byte in Integer umwandeln
    TS01_int =      (data_request[1] << 8) | data_request[0];  // Low-Byte und High-Byte zusammenfügen
    TS02_int =      (data_request[3] << 8) | data_request[2];
    TS03_int =      (data_request[5] << 8) | data_request[4];
    dutycycle_int = (data_request[7] << 8) | data_request[6];
    debocap_percentage = (data_request[15] << 8) | data_request[14];
    
    // Integer in Float umwandeln
    TS01 = float(TS01_int)/10;
    TS02 = float(TS02_int)/10;
    TS03 = float(TS03_int)/10;  
    dutycycle = float(dutycycle_int)/10; 

    // States zuweisen
    heater_state =      data_request[8];      
    pipevent_state =    data_request[9];
    led_state =         data_request[10];
    ventilator_state =  data_request[11];
    word_hour =         data_request[12];
    word_minute =       data_request[13];
    // Feedback counter
    msg_req_feedback =  data_request[16];
    
    // ############################ Buttons 

    heater_state_ctl_str      = state_ctl_txt(heater_state_ctl);
    led_state_ctl_str         = state_ctl_led_txt(led_state_ctl);
    ventilator_state_ctl_str  = state_ctl_txt(ventilator_state_ctl);
    pipevent_state_ctl_str    = state_ctl_txt(pipevent_state_ctl);
    
    // ############################ States from Arduino 

    heater_state_str       = state_txt(heater_state);
    pipevent_state_str    = state_txt(pipevent_state);
    led_state_str         = state_txt(led_state);
    ventilator_state_str  = state_txt(ventilator_state);
            
  }
  
// ############################ SEND 

  if ((millisec - cycle_send_pre > cycle_send_period)) {
    cycle_send_pre = millisec;

    Serial.println("Send data on I2C to SLAVE (start)");
  
    // Bytes in das Datenarray schreiben
    data_to_slave[0] = heater_state_ctl;
    data_to_slave[1] = pipevent_state_ctl;
    data_to_slave[2] = led_state_ctl;
    data_to_slave[3] = ventilator_state_ctl;
    data_to_slave[4] = pipevent_dutycycle_ctl;
    
    // Daten an den Bus senden
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(data_to_slave, sizeof(data_to_slave));
    Wire.endTransmission();

    Serial.println("Send data on I2C to SLAVE (end)");
  }
   
  delay(1);
}
  
