/*
  Johannes Huber
  https://github.com/joehubi/growbox
  10.09.24
*/

// ######################## Libray
  #include <ESP8266WiFi.h>        // https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/ESP8266WiFi.h
  #include <ESPAsyncTCP.h>        // https://github.com/me-no-dev/ESPAsyncTCP
  #include <ESPAsyncWebServer.h>  // https://github.com/me-no-dev/ESPAsyncWebServer
  #include <FS.h>                 // https://github.com/esp8266/Arduino/blob/master/cores/esp8266/FS.h
  #include <SoftwareSerial.h>

// ######################## Variables
  // ######################## Debugging
    #define PRINT_VARIABLE(var) Serial.print(#var " = "); Serial.println(var);

  // ################### SoftwareSerial
    #define rxPin D5
    #define txPin D6
    SoftwareSerial SoftwareSerial_ESP8266(rxPin, txPin);
    int msg_req_feedback = 0;
    // ################### READ
      char incoming_char_array[50]; // max. number of signs in data string
      int read_ctn = 0;         
    // ################### SEND
      char send_char_array[50];
      int snd_ctn = 5;     
  // ######################## pipe ventilator
    float dutycycle = 0.0;
    int dutycycle_int = 0;

    int pipevent_state = 0;                   // 0=OFF, 1=ON
    String pipevent_state_str = "...";
    int pipevent_state_ctl = 2;               // 0=OFF, 1=ON, 2=Intervall
    String pipevent_state_ctl_str = "...";
    int pipevent_dutycycle_ctl = 15;          // Duty cycle control (%)

    int pipevent_minute_ON     = 4;
    int pipevent_minute_OFF    = 2;

  // ######################## Sensors
    float TS01 = 0.0;
    float TS02 = 0.0;
    float TS03 = 0.0;
    int TS01_int = 0;
    int TS02_int = 0;
    int TS03_int = 0;
            
    int FS01_LF = 0;   // Luftfeuchte Sensor 1 - 0..100%
    int FS02_LF = 0;   // Luftfeuchte Sensor 2 - 0..100%

    int debocap_percentage = 0; // 0..100 % Bodenfeuchte

  // ######################## RTC Time
    int word_hour = 0;
    int word_minute = 0;

  // ######################## Timers
    const bool debug_timers = true;
    unsigned long millisec;           // time-ms
    unsigned long cycle_read_pre = 0;
    const unsigned long cycle_read_period = 1500;  // in ms
    unsigned long cycle_send_pre = 0;
    const unsigned long cycle_send_period = 2000;  // in ms

  // ######################## WiFi
    //int port = 8888;  //Port number
    //WiFiServer server(port);

    const char* ssid = "Pumuckel";            // wifi network
    const char* password = "Stiller_83";      // wifi network
    AsyncWebServer server(80);                // Create AsyncWebServer object on port 8888

  // ######################## Heater
    int heater_state = 0;                     // 0=OFF, 1=ON
    String heater_state_str = "...";
    int heater_state_ctl = 1;                 // 0=Manual ON, 1=Manual OFF, 2=AUTOMATIC
    String heater_state_ctl_str = "...";
  // ######################## LED
    int led_state = 0;                      // 0=OFF, 1=ON
    String led_state_str = "...";
    int led_state_ctl = 2;                  // 0=Manual ON, 1=Manual OFF, 2=AUTOMATIC
    String led_state_ctl_str = "...";
  // ######################## Ventilator
    int ventilator_state = 0;                      // 0=OFF, 1=ON
    String ventilator_state_str = "...";
    int ventilator_state_ctl = 2;                  // 0=Manual ON, 1=Manual OFF, 2=AUTOMATIC
    String ventilator_state_ctl_str = "...";

// ######################## Functions
  // ######################## String function Switches/States
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

  // ######################## String function LED
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

  // ######################## String function ON/OFF
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

  // ######################## Build strings for variables
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
    String getpipevent_minute_ON() {
      return String(pipevent_minute_ON);
    }
    String getpipevent_minute_OFF() {
      return String(pipevent_minute_OFF);
    }
    String getdebocap_percentage() {
      return String(debocap_percentage);
    }
    String getFS01_LF() {
      return String(FS01_LF);
    }
    String getFS02_LF() {
      return String(FS02_LF);
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

  // ######################## Multiplex processor for all data variables 
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
      else if (var == "pipevent_minute_ON"){
        return getpipevent_minute_ON();
      }
      else if (var == "pipevent_minute_OFF"){
        return getpipevent_minute_OFF();
      }    
      else if (var == "debocap_percentage"){
        return getdebocap_percentage();
      }
      else if (var == "FS01_LF"){
        return getFS01_LF();
      } 
      else if (var == "FS02_LF"){
        return getFS02_LF();
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



void setup(){

  // Setup
    // ######################## SoftwareSerial
      pinMode(rxPin, INPUT);
      pinMode(txPin, OUTPUT);
      SoftwareSerial_ESP8266.begin(9600);

    // ######################## Debug Monitor
      Serial.begin(9600);   // for serial output in window / debugging
      while (!Serial) {
        ; // wait for serial port to connect
      }

    // ######################## Initialize SPIFFS (for web server)
      if(!SPIFFS.begin()){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
      }

    // ######################## WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi ...");
      }
      Serial.println(WiFi.localIP()); // Print ESP32 Local IP Address

    // ######################## Webserver GUI
        Serial.println("Start server setup");
      // ######################## Load style.css file
        server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
          request->send(SPIFFS, "/style.css", "text/css");
        });

      // ######################## Called when <IP>/ browsed (Refresh button)
        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
          request->send(SPIFFS, "/index.html", String(), false, processor);
        });
    
      // ######################## Heater State Button (Called when <IP>/b1_off browsed)
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

      // ######################## Pipe ventilator state Button
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
        
      // ######################## Pipe ventilator duty cycle Button
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
              
      // ######################## LED state Button
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
      
      // ######################## ventilator state Button
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
      
      // ######################## Pipe ventilator ON-Zeit Button
        server.on("/b5_2", HTTP_GET, [](AsyncWebServerRequest *request){  
          request->send(SPIFFS, "/index.html", String(), false, processor);
          pipevent_minute_ON = 2;
        });  
        server.on("/b5_4", HTTP_GET, [](AsyncWebServerRequest *request){  
          request->send(SPIFFS, "/index.html", String(), false, processor);
          pipevent_minute_ON = 4;
        });  
        server.on("/b5_6", HTTP_GET, [](AsyncWebServerRequest *request){  
          request->send(SPIFFS, "/index.html", String(), false, processor);
          pipevent_minute_ON = 6;
        });  
        server.on("/b5_8", HTTP_GET, [](AsyncWebServerRequest *request){  
          request->send(SPIFFS, "/index.html", String(), false, processor);
          pipevent_minute_ON = 8;
        });  

      // ######################## Pipe ventilator OFF-Zeit Button
        server.on("/b6_0", HTTP_GET, [](AsyncWebServerRequest *request){  
          request->send(SPIFFS, "/index.html", String(), false, processor);
          pipevent_minute_OFF = 0;
        });  
        server.on("/b6_2", HTTP_GET, [](AsyncWebServerRequest *request){  
          request->send(SPIFFS, "/index.html", String(), false, processor);
          pipevent_minute_OFF = 2;
        });  
        server.on("/b6_4", HTTP_GET, [](AsyncWebServerRequest *request){  
          request->send(SPIFFS, "/index.html", String(), false, processor);
          pipevent_minute_OFF = 4;
        });  
        server.on("/b6_6", HTTP_GET, [](AsyncWebServerRequest *request){  
          request->send(SPIFFS, "/index.html", String(), false, processor);
          pipevent_minute_OFF = 6;
        }); 

      // ################### Start server
        server.begin();
        Serial.println("Server started");
        
}

void loop(){

  millisec = millis();

  // ############################ READ
    if ((millisec - cycle_read_pre > cycle_read_period)) {
      cycle_read_pre = millisec; 

      if (debug_timers == true) {
        Serial.println("Start READ cycle");
      }

      // ################### READ
        if (SoftwareSerial_ESP8266.available()) {
          delay(50);  // short delay that all received data is present at the Rx
          String _incoming_string = SoftwareSerial_ESP8266.readStringUntil('\n'); // read Rx until \n
          _incoming_string.toCharArray(incoming_char_array, 50); // copy data to char-Array (for sscanf)
          PRINT_VARIABLE(incoming_char_array);  //debugging

          // decode data from char-array
          sscanf(incoming_char_array, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", &TS01_int, &TS02_int, &TS03_int, &dutycycle_int, &heater_state, &pipevent_state, &led_state, &ventilator_state, &word_hour, &word_minute, &debocap_percentage, &msg_req_feedback, &FS01_LF, &FS02_LF);
          PRINT_VARIABLE(read_ctn);
        }    

      // Integer in Float umwandeln
        TS01 = float(TS01_int)/10;
        TS02 = float(TS02_int)/10;
        TS03 = float(TS03_int)/10;  
        dutycycle = float(dutycycle_int)/10; 

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

      if (debug_timers == true) {
        Serial.println("Start SEND cycle");
      }

    // ################### SEND
      Serial.println("Send data to ArduinoUNO (SoftwareSerial)");
      sprintf(send_char_array, "%u,%u,%u,%u,%u,%u,%u", heater_state_ctl, pipevent_state_ctl, led_state_ctl, ventilator_state_ctl, pipevent_dutycycle_ctl, pipevent_minute_ON, pipevent_minute_OFF);
      SoftwareSerial_ESP8266.println(send_char_array);
      PRINT_VARIABLE(send_char_array);
      
    }
   
  delay(1);
}
  
