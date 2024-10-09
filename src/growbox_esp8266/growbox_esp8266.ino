/*
  Johannes Huber
  https://github.com/joehubi/growbox
  09.10.24
*/

// ################### DEFINITION
  // ################### Libray
    #include <ESPAsyncTCP.h>        // https://github.com/me-no-dev/ESPAsyncTCP
    #include <ESPAsyncWebServer.h>  // https://github.com/me-no-dev/ESPAsyncWebServer
    #include <dataexchange.h>       // https://github.com/joehubi/dataexchange.git
  // ################### Variables
    // ################### Debugging
      #define PRINT_VARIABLE(var) Serial.print(#var " = "); Serial.println(var);

    // ################### Dataexchange
      #define rxPin D5
      #define txPin D6
      dataexchange DATAX_ARDU_ESP(rxPin, txPin);
      const bool debug_softwareserial = true;   
      int msg_req_feedback = 0;
      // ################### READ
        char received_data[50]; // max. number of signs in data string
      // ################### SEND
        char data_to_send[50];
        int snd_active = 0;   // > 5 = SEND aktivieren 
        const int snd_counts_to_active = 5; 
    // ################### pipe ventilator dutycycle
      float dutycycle = 0.0;
      int dutycycle_int = 0;
      int pipevent_dutycycle_ctl = 50;          // Duty cycle control (%)

      int pipevent_minute_ON     = 4;
      int pipevent_minute_OFF    = 2;

    // ################### Sensors
      float TS01 = 0.0;
      float TS02 = 0.0;
      float TS03 = 0.0;
      int TS01_int = 0;
      int TS02_int = 0;
      int TS03_int = 0;
              
      int FS01_LF = 0;   // Luftfeuchte Sensor 1 - 0..100%
      int FS02_LF = 0;   // Luftfeuchte Sensor 2 - 0..100%

      int debocap_percentage = 0; // 0..100 % Bodenfeuchte

    // ################### RTC Time
      int word_hour = 0;
      int word_minute = 0;

    // ################### Timers
      const bool debug_timers = true;
      unsigned long millisec;           // time-ms

      const unsigned long cycle_50ms = 50;  // in ms
      unsigned long cycle_50ms_dt = 0;

      const unsigned long cycle_1000ms = 1000;  // in ms
      unsigned long cycle_1000ms_dt = 0;

    // ################### WiFi
      const char* ssid = "Pumuckel";            // wifi network
      const char* password = "Stiller_83";      // wifi network
      AsyncWebServer server(80);                // Create AsyncWebServer object on port ...
    // ################### Actuators
      struct state_ctl {  // class for actuators to control and visualize states
        int state             = 0;         // 0=OFF, 1=ON, etc.
        String state_str      = "...";     // displays state on GUI/HTML 
        int state_ctl         = 1;         // 0=Manual ON, 1=Manual OFF, etc.
        String state_ctl_str  = "...";     // for button text
      };
      state_ctl heater;     // create instance for heater
      state_ctl led; 
      state_ctl ventilator; 
      state_ctl pipevent;
    // ################### state setpoints
      const int OFF   = 0;
      const int ON    = 1;
      const int AUTO  = 2;
      const int LED_186   = 2;
      const int LED_1212  = 3;
      const int LED_ARRAY = 4;
  // ################### Functions
    // ######################## String function Switches/States
      String state_ctl_txt(int ctl){
        String _txt = "";
        switch (ctl) {
          case OFF:
            _txt = "Manual OFF";
            break;
          case ON:
            _txt = "Manual ON";
            break;
          case AUTO:
            _txt = "AUTOMATIC";
            break;
        }
        return _txt;
      }

    // ######################## String function LED
      String state_ctl_led_txt(int ctl){
        String _txt = "";
        switch (ctl) {
          case OFF:
            _txt = "Manual OFF";
            break;
          case ON:
            _txt = "Manual ON";
            break;
          case LED_186:
            _txt = "18/6 (4-22)";
            break;
          case LED_1212:
            _txt = "12/12 (8-20)";
            break;
          case LED_ARRAY:
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
          return heater.state_str;
        }
        if(var == "heater_state_ctl_str"){
          return heater.state_ctl_str;
        }
        if(var == "pipevent_state_str"){
          return pipevent.state_str;
        }
        if(var == "pipevent_state_ctl_str"){
          return pipevent.state_ctl_str;
        }
        if(var == "led_state_str"){
          return led.state_str;
        }
        if(var == "led_state_ctl_str"){
          return led.state_ctl_str;
        }
        if(var == "ventilator_state_str"){
          return ventilator.state_str;
        }
        if(var == "ventilator_state_ctl_str"){
          return ventilator.state_ctl_str;
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
    // ######################## Server ON
      void handleButton(const char* url, int& state_var, int new_state) {
          server.on(url, HTTP_GET, [&, new_state](AsyncWebServerRequest *request){  
              state_var = new_state;
              request->send(SPIFFS, "/index.html", String(), false, processor);
          });
      }


// ################### SETUP
  void setup(){
    // ################### Initial values
      heater.state_ctl        = 0;    // OFF
      pipevent.state_ctl      = 1;    // ON    
      led.state_ctl           = 2;    // 18/6
      ventilator.state_ctl    = 1;    // ON
      pipevent_dutycycle_ctl  = 50;   // 50 %
    // ################### Debug Monitor
      Serial.begin(9600);   // for serial output in window / debugging
      while (!Serial) {
        ; // wait for serial port to connect
      }
      delay(1000);
    // ################### Dataexchange
      pinMode(rxPin, INPUT);
      pinMode(txPin, OUTPUT);
      DATAX_ARDU_ESP.begin(9600);
      delay(1000);
    // ################### Initialize SPIFFS (for web server)
      if(!SPIFFS.begin()){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
      }

    // ################### WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi ...");
      }
      Serial.println(WiFi.localIP()); // Print ESP32 Local IP Address

    // ################### Webserver GUI
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
          handleButton("/b1_off", heater.state_ctl, OFF);
          handleButton("/b1_on", heater.state_ctl, ON);
          handleButton("/b1_auto", heater.state_ctl, AUTO);

          // server.on("/b1_off", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   heater.state_ctl = 0;
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          // });
          // server.on("/b1_on", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   heater.state_ctl = 1;
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          // }); 
          // server.on("/b1_auto", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   heater.state_ctl = 2;
          // });

        // ######################## Pipe ventilator state Button
          handleButton("/b21_off", pipevent.state_ctl, OFF);
          handleButton("/b21_on", pipevent.state_ctl, ON);
          handleButton("/b21_on_intervall", pipevent.state_ctl, AUTO);

          // server.on("/b21_off", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   pipevent.state_ctl = 0;
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          // });
          // server.on("/b21_on", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   pipevent.state_ctl = 1;
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          // });
          // server.on("/b21_on_intervall", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   pipevent.state_ctl = 2;
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          // });
          
        // ######################## Pipe ventilator duty cycle Button
          handleButton("/b22_15", pipevent_dutycycle_ctl, 15);
          handleButton("/b22_30", pipevent_dutycycle_ctl, 30);
          handleButton("/b22_50", pipevent_dutycycle_ctl, 50);
          handleButton("/b22_80", pipevent_dutycycle_ctl, 80);

          // server.on("/b22_15", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   pipevent_dutycycle_ctl = 15;
          // });  
          // server.on("/b22_30", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   pipevent_dutycycle_ctl = 30;
          // });  
          // server.on("/b22_50", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   pipevent_dutycycle_ctl = 50;
          // });  
          // server.on("/b22_80", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   pipevent_dutycycle_ctl = 80;
          // });  
                
        // ######################## LED state Button
          handleButton("/b3_off", led.state_ctl, OFF);
          handleButton("/b3_on", led.state_ctl, ON);
          handleButton("/b3_1806", led.state_ctl, LED_186);
          handleButton("/b3_1212", led.state_ctl, LED_1212);
          handleButton("/b3_array", led.state_ctl, LED_ARRAY);

          // server.on("/b3_off", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   led.state_ctl = 0;
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          // });
          // server.on("/b3_on", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   led.state_ctl = 1;
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          // }); 
          // server.on("/b3_1806", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   led.state_ctl = 2;
          // });
          // server.on("/b3_1212", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   led.state_ctl = 3;
          // });
          //   server.on("/b3_array", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   led.state_ctl = 4;
          // });
        
        // ######################## ventilator state Button
          handleButton("/b4_off", ventilator.state_ctl, OFF);
          handleButton("/b4_on", ventilator.state_ctl, ON);
          handleButton("/b4_auto", ventilator.state_ctl, AUTO);

          // server.on("/b4_off", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   ventilator.state_ctl = 0;
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          // });
          // server.on("/b4_on", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   ventilator.state_ctl = 1;
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          // }); 
          // server.on("/b4_auto", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   ventilator.state_ctl = 2;
          // });
        
        // ######################## Pipe ventilator ON-Zeit Button
          handleButton("/b5_2", pipevent_minute_ON, 2);
          handleButton("/b5_4", pipevent_minute_ON, 4);
          handleButton("/b5_6", pipevent_minute_ON, 6);
          handleButton("/b5_8", pipevent_minute_ON, 8);

          // server.on("/b5_2", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   pipevent_minute_ON = 2;
          // });  
          // server.on("/b5_4", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   pipevent_minute_ON = 4;
          // });  
          // server.on("/b5_6", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   pipevent_minute_ON = 6;
          // });  
          // server.on("/b5_8", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   pipevent_minute_ON = 8;
          // });  

        // ######################## Pipe ventilator OFF-Zeit Button
          handleButton("/b6_0", pipevent_minute_OFF, 0);
          handleButton("/b6_2", pipevent_minute_OFF, 2);
          handleButton("/b6_4", pipevent_minute_OFF, 4);
          handleButton("/b6_6", pipevent_minute_OFF, 6);

          // server.on("/b6_0", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   pipevent_minute_OFF = 0;
          // });  
          // server.on("/b6_2", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   pipevent_minute_OFF = 2;
          // });  
          // server.on("/b6_4", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   pipevent_minute_OFF = 4;
          // });  
          // server.on("/b6_6", HTTP_GET, [](AsyncWebServerRequest *request){  
          //   request->send(SPIFFS, "/index.html", String(), false, processor);
          //   pipevent_minute_OFF = 6;
          // }); 

        // ################### Start server
          server.begin();
          Serial.println("Server started");
          
  }

// ################### LOOP
  void loop(){
    millisec = millis();
    // ################### 50 ms
      if ((millisec - cycle_50ms_dt > cycle_50ms)) {
        cycle_50ms_dt = millisec; 

        if (debug_timers == true) {
          Serial.println("<Start> 50 ms");
        }

        // ################### dataexchange READ
        if (DATAX_ARDU_ESP.rxData(received_data, 50)) {  // check if data is available and get data
          if (debug_softwareserial == true) {
            PRINT_VARIABLE(received_data);  // debugging
          }
        }

        sscanf(received_data, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", &TS01_int, &TS02_int, &TS03_int, &dutycycle_int, &heater.state, &pipevent.state, &led.state, &ventilator.state, &word_hour, &word_minute, &debocap_percentage, &msg_req_feedback, &FS01_LF, &FS02_LF);
        
      }
    
    // ################### 1000 ms 
      if ((millisec - cycle_1000ms_dt > cycle_1000ms)) {
        cycle_1000ms_dt = millisec;

        if (debug_timers == true) {
          Serial.println("<Start> 1000 ms");
        }

        // ################### Integer in Float umwandeln
          TS01 = float(TS01_int)/10;
          TS02 = float(TS02_int)/10;
          TS03 = float(TS03_int)/10;  
          dutycycle = float(dutycycle_int)/10; 

        // ################### Buttons 
          heater.state_ctl_str      = state_ctl_txt(heater.state_ctl);
          led.state_ctl_str         = state_ctl_led_txt(led.state_ctl);
          ventilator.state_ctl_str  = state_ctl_txt(ventilator.state_ctl);
          pipevent.state_ctl_str    = state_ctl_txt(pipevent.state_ctl);
          
        // ################### States from Arduino 
          heater.state_str      = state_txt(heater.state);
          pipevent.state_str    = state_txt(pipevent.state);
          led.state_str         = state_txt(led.state);
          ventilator.state_str  = state_txt(ventilator.state);   

        // ################### SEND
          if (snd_active < snd_counts_to_active) {
            snd_active++;
          }
        // ################### dataexchange SEND
          if (snd_active >= snd_counts_to_active) {
            sprintf(data_to_send, "%u,%u,%u,%u,%u,%u,%u", heater.state_ctl, pipevent.state_ctl, led.state_ctl, ventilator.state_ctl, pipevent_dutycycle_ctl, pipevent_minute_ON, pipevent_minute_OFF);
            DATAX_ARDU_ESP.txData(data_to_send);
            if (debug_softwareserial == true) {
              PRINT_VARIABLE(data_to_send); // debugging
            }
          }
      }
    
  }
  
