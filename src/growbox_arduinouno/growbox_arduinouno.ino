/*
  Johannes Huber
  https://github.com/joehubi/growbox
  15.09.2024
*/

// ################### DEFINITION
  // ################### Library

    // Lib's integrated in Arduino-Standard folder  
    #include <SoftwareSerial.h>
    #include <RTClib.h>               // https://github.com/StephanFink/RTClib/
    #include <DallasTemperature.h>    // https://github.com/milesburton/Arduino-Temperature-Control-Library.git
    #include <DHT.h>                  // https://github.com/Khuuxuanngoc/DHT-sensor-library.git

  // ################### Macros
    #define PRINT_VARIABLE(var) Serial.print(#var " = "); Serial.println(var);
  // ################### Variables
    // ################### General debugging Variables
      const bool debug_array   = false;    // Debugging Array-Management

      // Interupt-Input for testing the Interupt-Output
      // const byte interruptPin = 2;      // Debugging
      // volatile byte state = LOW;        // Debugging

    // ################### Sensor's
      // ################### DHT22 (temp & humidity)
        #define DHTPIN1 6          // Connected on Pin D6 on Arduino UNO    
        #define DHTPIN2 7          // Connected on Pin D7 on Arduino UNO      
        #define DHTTYPE DHT22      // Es handelt sich um den DHT22 Sensor

        DHT dht1(DHTPIN1, DHTTYPE);               // Der Sensor wird mit „dth1“ angesprochen
        DHT dht2(DHTPIN2, DHTTYPE);               // Der Sensor wird mit „dth2“ angesprochen

        class DHT22_th {
          public:
            int temp = 0;
            int hum = 0;
            static bool debug;
        };

        bool DHT22_th::debug = false;   // init static bool for all instances
        DHT22_th dht22_1;               // build instance of class
        DHT22_th dht22_2;
  
      // ################### SHELLY DS18B20 (Temperature, OneWire)
        #define ONE_WIRE_BUS 8                    // Data wire on Pin D8 at Arduino UNO
        OneWire oneWire(ONE_WIRE_BUS);            // Setup a OneWire instance to communicate with any OneWire devices
        DallasTemperature tempsensors(&oneWire);  // Pass OneWire reference to Dallas Temperature

        class DS18B20 {
        public:
            float t_float = 0.0;
            int t_int = 0;  

            /**
            * @brief Static debug variable that applies to all instances.
            * If `true`, debug outputs are activated.
            */
            static bool debug;    // Static debug variable that applies to all instances
  
            /**
            * @brief Static method for setting the debug variable.
            * 
            * @param debugvalue Value `true` to activate debugging or `false` to deactivate it.
            */
            static void setDebug(bool debugvalue) { // Static method for setting the debug variable
                debug = debugvalue;
            }
        };

        bool DS18B20::debug = false; // Initialization of the static variable

        DS18B20 DS18B20_temp1;    // build instance of class
        DS18B20 DS18B20_temp2;
        DS18B20 DS18B20_temp3; 

      // ################### Soil Humuditiy sensor
        
        class DEBOCAP {
          public:
            bool debug;
            int pin;
            int p100;   // Integer value according to 100 % humidity
            int p0;     // Integer value according to 0 % humidity
            int soil_humidity_p = 0;  // Value for soil humidity 0..100 %
        };
        
        DEBOCAP debo;

    // ################### Electrical Socket's with timing

      struct state_ctl {
        int state = 0;           // 0 = OFF, 1 = ON, etc.
        int state_ctl = 0;       // 0 = OFF, 1 = ON, etc.
      };

      const int _OFF = 0;
      const int _ON = 1;
      const int AUTO = 2;

      const int LED_186 = 2;      // 18/6 Belichtungszyklus (04:00 Uhr bis 22:00 Uhr)
      const int LED_1212 = 3;     // 12/12 Belichtungszyklus (08:00 Uhr bis 20:00 Uhr)
      const int LED_ARRAY = 4;    // Entsprechend Array 

      // ################### Pipe ventilator (socket switch + PWM signal)
        int pipevent_minute = 0;
        int pipevent_minute_pre = 0; 
        int pipevent_minute_change = 0;
        int pipevent_minute_ON     = 2;
        int pipevent_minute_OFF    = 5;
        int pipevent_minute_count  = 0;

        state_ctl pipevent;

        int pipevent_dutycycle_ctl = 50;  // Duty cycle control (%)

        int const pipevent_pin = PD3; 

        // ############################ Interupt-Output interrupt for pipe ventilator
          double duty_cycle   = 0.0;     // 0-100%. This duty cycle is present on pin 9 and inverted on Pin 10
          int duty_cycle_int  = 0;    
          // int icr_const     = 8000;    // ONLY TIMER1 -> results in 1 Hz PWM cycle (debugging)
          // int icr_const       = 7;     // ONLY TIMER1 -> results in 1,12 kHz PWM cycle 

        // ############################ Control of the pipe ventilator (NOT USED)
          // const float P_ = 2;           // P-Anteil
          // const float i_ = 0.01;        // I-Anteil
          // const float T_set = 25.0;     // Setpoint
          // float T_act = 0.0;      // Actual value of sensor
          // float T_err = 0.0;      // Error (actual value - setpoint)
          // float T_i_prev = 0.0;   // buffer for integrator

      // ################### Heater (socket switch)
        int const heater_pin = PD2; 
        state_ctl heater;

      // ################### ventilator (socket switch + timing)
        int const ventilator_pin = PD5;  
        state_ctl ventilator;

        int vent_minute      = 0;
        int vent_minute_pre  = 0; 

        const byte vent_min_array[31]={
          0 ,2 ,4 ,6 ,8 ,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60
          };  // minutes of an hour
        const byte vent_state_array[31]={
          1 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0
          };    // on/off time of ventilator can be programmed here (0=off, 1=on within one minute)

      // ################### LED (socket switch + timing)
        int const led_pin = PD4; 
        state_ctl led;

        int led_minute = 0;
        int led_minute_pre = 0; 

        const word led_hourm_array[96]={
          0   , 15  , 30  , 45  , 60  , 75  , 90  , 105 , 120 , 135 , 150 , 165 , 180 , 195 , 210 , 225 , 240 , 255 , 270 , 285 ,
          300 , 315 , 330 , 345 , 360 , 375 , 390 , 405 , 420 , 435 , 450 , 465 , 480 , 495 , 510 , 525 , 540 , 555 , 570 ,
          585 , 600 , 615 , 630 , 645 , 660 , 675 , 690 , 705 , 720 , 735 , 750 , 765 , 780 , 795 , 810 , 825 , 840 , 855 ,
          870 , 885 , 900 , 915 , 930 , 945 , 960 , 975 , 990 , 1005, 1020, 1035, 1050, 1065, 1080, 1095, 1110, 1125, 1140,
          1155, 1170, 1185, 1200, 1215, 1230, 1245, 1260, 1275, 1290, 1305, 1320, 1335, 1350, 1365, 1380, 1395, 1410, 1425
          };  // minutes of a 24 hour day
        const word led_state_array[96]={
          0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   ,
          0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 1   , 1   , 1   , 1   , 1   ,
          1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   ,
          1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   , 1   ,
          1   , 1   , 1   , 1   , 1   , 1   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0
          };    // on/off time of LED can be programmed here (0=off, 1=on within one day)
          
    // ################### Software Serial
      #define rxPin 12
      #define txPin 13
      SoftwareSerial SoftwareSerial_Arduino(rxPin, txPin);
      const bool debug_softwareserial = false;    
      // ################### READ
        const int incoming_char_size = 50; 
        char incoming_char_array[incoming_char_size]; // max. number of sign's in one incoming string
      // ################### SEND
        char send_char_array[50]; // Puffer für die Datenübertragung (max. Länge der Nachricht festlegen)
        int snd_ctn = 0;
    // ################### RTC
      /*  Die RTC muss über die Pin's A4 und A5 verbunden werden (siehe Arduino UNO Pinout und Dokumentation https://github.com/StephanFink/RTClib/, UNO is ATmega328).
          Die Kommunikation ist I2C. 
          A4 = SDA (serial data) = D18
          A5 = SCL (serial clock) = D19 
      */

      RTC_DS3231 RTC; 
      const bool syncOnFirstStart = false;  // true, falls die Zeitinformationen der RTC mit dem PC synchronisiert werden sollen (default = FALSE)

      const bool RTC_use         = true;      // 0 = don't use RTC (no init will be done), 1 = use RTC
      const bool RTC_adjust      = false;     // adjust time on RTC chip

      String timestamp  = "dd.mm.yyyy HH:MM:ss";
      int _minute      = 0;
      int _hour        = 0;
      int _second      = 0;
      int _hourminute  = 0;

      struct RTC_s {
          int hour;
          int minute;
          int hourminute;
      };

    // ################### Timer for timed loops
      const bool debug_timers = false;
      const bool debug_second_timer = false;

      class Cycle {
        public:
            int time = 1000;        // default 1000 ms
            unsigned long dt = 0;   // for timer calculation

            // default
            Cycle() {}
            // constructor with parameters
            Cycle(int time_in_ms) {
                time = time_in_ms;
            }
            // set time value
            void set_timer(int time_in_ms) {
                time = time_in_ms;
            }
            // // get actual time value
            // int get_time() const {
            //     return time;
            // }
      };

      Cycle _500ms(500);    // build instance for 500 ms cycle-timer
      Cycle _1000ms(1000);  
      Cycle _1500ms(1500);  
      Cycle _2000ms(2000);  

      unsigned long millisec;           // arduino time-ms

  // ################### Functions
  // ################### RTC
    RTC_s getRTC() {
        DateTime now = RTC.now();
        RTC_s outputs;
        outputs.hour = now.hour();
        outputs.minute = now.minute();
        outputs.hourminute = outputs.minute + 60 * outputs.hour;  // Berechne hourminute (Minuten des Tages)
        return outputs;
    }
  // ################### FreeMemory  
    // extern int __heap_start, *__brkval;
    // int freeMemory() {
    //     int v;
    //     return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
    // }

// ################### SETUP
  void setup(void){
    // ################### Initial values (if ESP8266 is not available)
      // In case of no ESP8266 (Node MCU) connection set initial values
      // These values are immediately overwritten by the ESP8266
      heater.state_ctl        = 0;    // OFF
      pipevent.state_ctl      = 1;    // ON    
      led.state_ctl           = 2;    // 18/6
      ventilator.state_ctl    = 1;    // ON
      pipevent_dutycycle_ctl  = 50;   // 50 %

    // ################### Debugging  
      Serial.begin(9600);             // for Debugging/print  
      while (!Serial) {
        ; // wait for serial port to connect
      }
      delay(1000);
    // ################### SoftwareSerial
      pinMode(rxPin, INPUT);
      pinMode(txPin, OUTPUT);
      SoftwareSerial_Arduino.begin(9600);
    // ################### Digital-Output
      pinMode(heater_pin, OUTPUT);
      pinMode(pipevent_pin, OUTPUT);
      pinMode(led_pin, OUTPUT);
      pinMode(ventilator_pin, OUTPUT);

      digitalWrite(heater_pin, LOW);
      digitalWrite(pipevent_pin, LOW);
      digitalWrite(led_pin, LOW);
      digitalWrite(ventilator_pin, LOW);
    // ################### DEBOCAP
      debo.debug  = false;
      debo.pin    = A0;
      debo.p100   = 258;  // tested for water (3,3 V with short connection)
      debo.p0     = 635;  // tested for dry air (3,3 V with short connection)
      // debo.p0     = 429;  // tested for normal soil (3,3 V with short connection)
      // debo.p0     = 255;  // tested for wet soil (3,3 V with short connection)

    // ################### DHT temp.
      tempsensors.begin();    // Temperatursensoren starten         
      dht1.begin();           // Feuchtigkeitssensor DHT22 (1) starten
      dht2.begin();           // Feuchtigkeitssensor DHT22 (2) starten
    
    // ################### DS18B20 temp.
      // DS18B20::setDebug(true);
    // ################### RTC
      if (RTC_use == true) {
        if (RTC_adjust == true) {
          // Hier kann (einmalig oder bei Wechsel der Sommer- bzw. Winterzeit) die Zeit für die RTC initialisiert werden
          RTC.adjust(DateTime(2024, 8, 23, 18, 23, 0)); // Adjust time YYYY,MM,DD,hh,mm,ss    
        }
        if (! RTC.begin()) {
          Serial.println("Error: RTC can not be initialized");
        }
        else {
          Serial.println("RTC initialized");
          RTC_s rtc_values = getRTC();    // get once at startup
          _minute     = rtc_values.minute;
          _hour       = rtc_values.hour;
          _hourminute = rtc_values.hourminute;
        }
      }
      else {
        Serial.println("RTC time set manually");
        _minute     = 0;     // time can be set here manually
        _hour       = 12;   // time can be set here manually
        _hourminute = _minute + 60*_hour; // calc hourm (minutes of the day)       
      }  

      delay(50);
      Serial.println("RTC-time: HH:MM - " + String(_hour) + ":" + String(_minute) +" , Hour-Minute - " + String(_hourminute));

    // ################### Interrupt-Output (PWM pins)
      // ATmega328P
      // Control register setup according to ATMEGA
      // https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
      // OC2A = PB3 (Timer2)
      // OC2B = PD3 (Timer2)
      // OC1A = PB1 (Timer1)
      // OC1B = PB2 (Timer1)

      // ################### Using Timer1
        // !!! Timer 1 had an issue with parallel SoftwareSerial (used same timers internally) !!!
        // This sets it so OCA1 and OCB1, connected to pin 9 and pin 10, to be inverted
        // in comparison to one another. Also sets Waveform generator mode to 10.
        // TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM11);  // pin setup
        // TCCR1B = (1 << WGM13) | (1 << CS12) | (0 << CS11) | (1 >> CS10);  // prescale to 1024  
        // DDRB = (1 << DDB2) | (1 << DDB1);   // Configure DDR2 and DDR1 to be output mode, for pin D9 (PD1) and pin D10 (PB2).
        // ICR1 = icr_const;   // Multiplikator für prescaler
        // OCR1A = (duty_cycle/100)*icr_const; // Send to interrupt-output
        // OCR1B = (duty_cycle/100)*icr_const; // Send to interrupt-output

      // ################### Using Timer2
        // Set Timer2 for PWM on Pin D11 (OC2A, PB3)
        TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);  // Fast PWM, clear OC2A on compare match
        TCCR2B = (1 << CS22) | (0 << CS21) | (0 << CS20);      // Prescaler 64
        DDRB |= (1 << DDB3);                    // Set Pin 11 (PB3) as output
        OCR2A = (duty_cycle * 255) / 100;       // duty_cycle für Timer2 anpassen, 0 bis 255

      //pinMode(LED_BUILTIN, OUTPUT);                                           // Interrupt-Input for testing
      //pinMode(interruptPin, INPUT_PULLUP);                                    // Interrupt-Input for testing
      //attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);    // Interrupt-Input for testing
      //pinMode(pot1, INPUT);                                                   // Debugging/Simulation
      //pinMode(pot2, INPUT);                                                   // Debugging/Simulation
  }
// ################### LOOP
  void loop(void){
    millisec = millis();  // get time from arduino-clock (time since arduino is running in ms)
    // ################### 500 ms
      if (millisec - _500ms.dt >= _500ms.time) {
        _500ms.dt = millisec;

        if (debug_timers == true) {
          Serial.println("<Start> 500 ms");
        }

        // ################### READ SoftwareSerial
          if (SoftwareSerial_Arduino.available()) {
            delay(50);  // short delay, to be sure all data is present at the Rx
            String _incoming_string = SoftwareSerial_Arduino.readStringUntil('\n'); // read Rx
            _incoming_string.toCharArray(incoming_char_array, incoming_char_size);     // copy data to char-Array (for sscanf)
            if (debug_softwareserial == true) {
              int _lenght = _incoming_string.length();  // lenght for debugging
              PRINT_VARIABLE(_lenght);                  // lenght for debugging
              PRINT_VARIABLE(incoming_char_array);
            }
            // decode data from char-array
            byte _number_of_items = sscanf(incoming_char_array,"%u,%u,%u,%u,%u,%u,%u", \
            &heater.state_ctl, &pipevent.state_ctl, &led.state_ctl, &ventilator.state_ctl, \
            &pipevent_dutycycle_ctl, &pipevent_minute_ON, &pipevent_minute_OFF);
            if (debug_softwareserial == true) {
              PRINT_VARIABLE(_number_of_items);   // check number of items (here 11)
            }
          }
      }

    // ################### 1000 ms

      if (millisec - _1000ms.dt >= _1000ms.time) {
        _1000ms.dt = millisec;

        if (debug_timers == true) {
          Serial.println("<Start> 1000 ms");
        }

        // ################### Calculate times with second-timer
          _second++;
        
          if (_second >= 60) {
            _second = 0;
            _minute++;
            if (_minute >= 60) {
              _minute = 0;
              _hour++;
              if (_hour >= 24) {
                _hour = 0;
                
                if (RTC_use == true) {
                  RTC_s rtc_values  = getRTC(); // Aktuelle Uhrzeit einmal am Tag von der RTC holen
                  _minute           = rtc_values.minute;
                  _hour             = rtc_values.hour;
                  _hourminute       = rtc_values.hourminute;
                }
              }
            }
            _hourminute = _minute + 60*_hour; // calc hourm (minutes of the day)
          }
          
          if (debug_second_timer == true) {
            Serial.println("Zeiten"); 
            Serial.println("Sekunde: "+String(_second));
            Serial.println("Minute: "+String(_minute));
            Serial.println("Stunde: "+String(_hour));
            Serial.println("Stundenminute: "+String(_hourminute));
          }

        // ################### Trigger für Statuswechsel (LED / Ventilator) updaten)
          vent_minute     = _minute;
          pipevent_minute = _minute;
          led_minute      = _minute;
        
        // ################### pipe ventilator
          // ################### control
            switch (pipevent.state_ctl) {
              case _OFF:
                pipevent.state = 0;
                break;
              case _ON:
                pipevent.state = 1;
                break;
              case AUTO:
                // At change of minute value
                // Pipe ventilator should rest every few minutes to decrease heat-cost for growbox
                if (pipevent_minute != pipevent_minute_pre) {
                  pipevent_minute_pre = pipevent_minute;
            
                  pipevent_minute_count++;    // add counter
                  //Serial.println("pipevent_minute_count: "+String(pipevent_minute_count));
                        
                  if (pipevent_minute_count >= pipevent_minute_change) {
                    //Serial.println("Send pipeventilator to rest to reduce heating cost");
                    //Serial.println("pipevent_minute_change: "+String(pipevent_minute_change));
                    
                    // reset counter
                    pipevent_minute_count = 0;
            
                    // Invertiere den Wert von ...
                    if (pipevent.state == 0) {
                      pipevent.state = 1;
                      pipevent_minute_change = pipevent_minute_ON; // Set pipeventilator ON for X min
                      //Serial.println("pipeventilator ON for (min) - "+String(pipevent_minute_ON));
                    }
                    else {
                      pipevent.state = 0;
                      pipevent_minute_change = pipevent_minute_OFF; // Set pipeventilator ON for X min
                      //Serial.println("pipeventilator OFF for (min) - "+String(pipevent_minute_OFF));
                    }
                  }
                  
                }
                break;
                        
              default:
              break;
            }

          // ################### dutycycle
            switch (pipevent_dutycycle_ctl) {
              case 15:
                duty_cycle = 15.0;
                break;
              case 30:
                duty_cycle = 30.0;
                break;
              case 50:
                duty_cycle = 50.0;
                break;
              case 80:
                duty_cycle = 80.0;
                break;
                                            
              default:
                duty_cycle = 0.0;
              break;
            }

          // ################### interrupt output  
            OCR2A = (duty_cycle * 255) / 100; // duty_cycle für Timer2 anpassen, 0 bis 255

          // ################### Pipe ventilator controller (NOT USED AT THE MOMENT)
            // T_act = TS01;   
            // T_err = (T_act - T_set);                  // calc error
            // T_i_prev = T_i_prev + T_err;              // integrate
            // duty_cycle = P_ * T_err + i_ * T_i_prev;  // calc output (P-Anteil + I-Anteil)
        // ################### FLOAT -> INT (for sending to ESP8266)
          DS18B20_temp1.t_int = DS18B20_temp1.t_float*10;
          DS18B20_temp2.t_int = DS18B20_temp2.t_float*10;
          DS18B20_temp3.t_int = DS18B20_temp3.t_float*10;
          duty_cycle_int = duty_cycle*10;
        // ################### to interface (SoftwareSerial)      
          snd_ctn++;
          sprintf(send_char_array, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", \
          DS18B20_temp1.t_int, DS18B20_temp2.t_int, DS18B20_temp3.t_int, \
          duty_cycle_int, heater.state, pipevent.state, led.state, ventilator.state, \
          _hour,_minute, debo.soil_humidity_p, snd_ctn, dht22_1.hum, dht22_2.hum);
          SoftwareSerial_Arduino.println(send_char_array);
          if (debug_softwareserial == true) {
            PRINT_VARIABLE(send_char_array);
          }
      }

    // ################### 1500 ms

      if (millisec - _1500ms.dt >= _1500ms.time) {
        _1500ms.dt = millisec;
        
        if (debug_timers == true) {
          Serial.println("<Start> 1500 ms"); 
        }

        // ################### calculate control/states
          // ################### heater
            switch (heater.state_ctl) {
              case _OFF:
                heater.state = 0;
                break;
              case _ON:
                heater.state = 1;
                break;
              case AUTO:
                // Add code for automation here 
                break;  
              default:
                break;
            }

          // ################### ventilator
            switch (ventilator.state_ctl) {
              case _OFF: // manual OFF
                ventilator.state = 0;
                break;
              case _ON: // manual ON
                ventilator.state = 1;
                break;
              case AUTO: // automatic   
              // At change of minute value
              if (vent_minute != vent_minute_pre) {
                vent_minute_pre = vent_minute;

                if (debug_array == true) {
                //Serial.println("check ventilator state");
                }

                // Change ventilator state (on/off) according to actual minutes and state-array
                for (word i = 0; (i < sizeof(vent_min_array) / sizeof(vent_min_array[0]) && i < 1000) ; i++){
                  if (_minute == vent_min_array[i]) {
                    ventilator.state = vent_state_array[i];
                    if (debug_array == true) {
                      //Serial.println("change ventilator state (" + String(ventilator_state) + ") at minute: " + String(_minute));
                    }
                    i = 1000; //exit for-loop
                  }
                  else if (_minute < vent_min_array[i]) {
                    ventilator.state = vent_state_array[i-1];
                    if (debug_array == true) {
                      //Serial.println("change ventilator state (" + String(ventilator_state) + ") at minute: " + String(_minute));
                    }
                    i = 1000; //exit for-loop       
                  }
                  // next loop cycle
                  }  
                break;
              }
            }
          
          // ################### LED
          switch (led.state_ctl) {
            case _OFF: // manual OFF
              led.state = 0;
              break;
            case _ON: // manual ON
              led.state = 1;
              break;
            case LED_186: // 18/6 Belichtungszyklus (04:00 Uhr bis 22:00 Uhr)
              if ((_hourminute >= 240) && (_hourminute <= 1320)) {
                led.state = 1;
              }
              else {
                led.state = 0;
              }
              break;
            case LED_1212: // 12/12 Belichtungszyklus (08:00 Uhr bis 20:00 Uhr)
              if ((_hourminute >= 480) && (_hourminute <= 1200)) {
                led.state = 1;
              }
              else {
                led.state = 0;
              }
              break;
            case LED_ARRAY: // Entsprechend Array   
              // At change of minute value
              if (led_minute != led_minute_pre) {
                led_minute_pre = led_minute;

                if (debug_array == true) {
                //Serial.println("check LED state");
                }

                // Change led state (on/off) according to actual hour-minutes and state-array
                for (word i = 0; (i < sizeof(led_hourm_array) / sizeof(led_hourm_array[0]) && i < 1000) ; i++){
                  if (_hourminute == led_hourm_array[i]) {
                    led.state = led_state_array[i];
                    if (debug_array == true) {
                      //Serial.println("change LED state (" + String(led_state) + ") at hour minute: " + String(_hourminute));
                    }
                    i = 1000; //exit for-loop
                  }
                  else if (_hourminute < led_hourm_array[i]) {
                    led.state = led_state_array[i-1];
                    if (debug_array == true) {
                      //Serial.println("change LED state at (" + String(led_state) + ") hour minute: " + String(_hourminute));
                    }
                    i = 1000; //exit for-loop       
                  }
                  // next loop cycle
                  }  
                break;
                }    
          }


        // ################### write hardware outputs
          digitalWrite(heater_pin,      heater.state);
          digitalWrite(pipevent_pin,    pipevent.state);
          digitalWrite(led_pin,         led.state);
          digitalWrite(ventilator_pin,  ventilator.state);  

        // ################### checking free dynamic memory
          // Serial.print("Freier Speicher: ");  
          // Serial.println(freeMemory());
      }

    // ################### 2000 ms
      if ((millisec - _2000ms.dt > _2000ms.time)) {
        _2000ms.dt = millisec;
      
        if (debug_timers == true) {
          Serial.println("<START> 2000 ms");
        }

        // ############################################   READ temperature sensors
          tempsensors.requestTemperatures();
          DS18B20_temp1.t_float = tempsensors.getTempCByIndex(0);
          DS18B20_temp2.t_float = tempsensors.getTempCByIndex(1);
          DS18B20_temp3.t_float = tempsensors.getTempCByIndex(2);

          if (DS18B20::debug == true) {
            Serial.println("Temperaturen [°C] der DS18B20-Sensoren");
            PRINT_VARIABLE(DS18B20_temp1.t_float);    
            PRINT_VARIABLE(DS18B20_temp2.t_float);    
            PRINT_VARIABLE(DS18B20_temp3.t_float);    
          }

        // ############################################   READ soil moisture sensor
          int debocap_value = analogRead(debo.pin);
        
          // Konvertiere den analogen Wert in eine Bodenfeuchte-Prozentzahl
          debo.soil_humidity_p = map(debocap_value, debo.p100, debo.p0, 100, 0);

          if (debo.debug == true) {
            Serial.println("Feuchtigkeit [%] und Spannung [V] des DEBOCAP"); 
            PRINT_VARIABLE(debo.soil_humidity_p);
            PRINT_VARIABLE(debocap_value);
          }

        // ############################################   READ humidity sensor
          dht22_1.hum = dht1.readHumidity();        
          dht22_1.temp  = dht1.readTemperature(); 
              
          dht22_1.hum = dht2.readHumidity();        
          dht22_2.temp  = dht2.readTemperature();  

          if (DHT22_th::debug == true) {
            Serial.println("Relative Luftfeuchtigkeit [%] und Temperatur [°C] des DHT22");
            PRINT_VARIABLE(dht22_1.hum);
            PRINT_VARIABLE(dht22_1.temp);
            PRINT_VARIABLE(dht22_2.hum);
            PRINT_VARIABLE(dht22_2.temp);  
          }        

      }
    
  }
