/*
  Johannes Huber
  https://github.com/joehubi/growbox
  06.09.2024
*/

// ############################ Library

  // Lib's integrated in Arduino-Standard folder  
  #include <Wire.h>                 // https://github.com/codebendercc/arduino-library-files.git
  #include <RTClib.h>               // https://github.com/StephanFink/RTClib/
  #include <OneWire.h>              // https://www.pjrc.com/teensy/td_libs_OneWire.html
  #include <DallasTemperature.h>    // https://github.com/milesburton/Arduino-Temperature-Control-Library.git
  #include <DHT.h>                  // https://github.com/Khuuxuanngoc/DHT-sensor-library.git

// ############################ Variables
  // ############################ General debugging Variables
    #define PRINT_VARIABLE(var) Serial.print(#var " = "); Serial.println(var);
    const bool debug_array   = false;    // Debugging Array-Management

    // Interupt-Input for testing the Interupt-Output
    // const byte interruptPin = 2;      // Debugging
    // volatile byte state = LOW;        // Debugging

  // ############################ Sensor's
    // ############################ DHT22 (temp & humidity)
      #define DHTPIN1 6          // Connected on Pin D6 on Arduino UNO    
      #define DHTPIN2 7          // Connected on Pin D7 on Arduino UNO      
      #define DHTTYPE DHT22      // Es handelt sich um den DHT22 Sensor

      const bool debug_DHT22 = false;

      DHT dht1(DHTPIN1, DHTTYPE);               // Der Sensor wird mit „dth1“ angesprochen
      DHT dht2(DHTPIN2, DHTTYPE);               // Der Sensor wird mit „dth2“ angesprochen

      int FS01_T_int = 0;         
      int FS02_T_int = 0;         
      int FS01_LF_int = 0;         
      int FS02_LF_int = 0;   

    // ############################ SHELLY DS18B20 (Temperature, OneWire)
      #define ONE_WIRE_BUS 8                    // Data wire on Pin D8 at Arduino UNO
      OneWire oneWire(ONE_WIRE_BUS);            // Setup a OneWire instance to communicate with any OneWire devices
      DallasTemperature tempsensors(&oneWire);  // Pass OneWire reference to Dallas Temperature

      const bool debug_temp_DS18B20 = false;

      float TS01 = 0.0;  
      float TS02 = 0.0; 
      float TS03 = 0.0;    
      int TS01_int = 0;         
      int TS02_int = 0;         
      int TS03_int = 0;   

    // ############################ Soil Humuditiy sensor
      const int debocap_pin = A0;     // Data wire on Pin A0 or D14 at Arduino UNO
      int debocap_percentage = 0;
        
      const bool debug_debocap = false;

      const int debocap_dry_air = 635;        // 3,3 V und lange Leitung
      //const int debocap_normal_soil = 429;  // 3,3 V und kurze Leitung
      //const int debocap_wet_soil = 255;     // 3,3 V und kurze Leitung
      const int debocap_water = 258;          // 3,3 V und lange Leitung

  // ############################ Electrical Socket's with timing
    // ############################ Pipe ventilator (socket switch + PWM signal)
      byte pipevent_minute = 0;
      byte pipevent_minute_pre = 0; 
      byte pipevent_minute_change = 0;
      byte pipevent_minute_ON     = 2;
      byte pipevent_minute_OFF    = 5;
      byte pipevent_minute_count  = 0;

      byte pipevent_state = 0;           // 0 = OFF, 1 = ON
      byte pipevent_state_ctl = 0;       // 0 = OFF, 1 = ON, 2=AUTOMATIC
      byte pipevent_dutycycle_ctl = 50;  // Duty cycle control (%)

      int const pipevent_pin = PD3; 

      // ############################ Interupt-Output interrupt for pipe ventilator
        double duty_cycle   = 0.0;     // 0-100%. This duty cycle is present on pin 9 and inverted on Pin 10
        int duty_cycle_int  = 0;    
        //int icr_const     = 8000;    // -> results in 1 Hz PWM cycle (debugging)
        int icr_const       = 7;       // -> results in 1,12 kHz PWM cycle 

      // ############################ Control of the pipe ventilator (NOT USED)
        // const float P_ = 2;           // P-Anteil
        // const float i_ = 0.01;        // I-Anteil
        // const float T_set = 25.0;     // Setpoint
        // float T_act = 0.0;      // Actual value of sensor
        // float T_err = 0.0;      // Error (actual value - setpoint)
        // float T_i_prev = 0.0;   // buffer for integrator

    // ############################ Heater (socket switch)
      int const heater_pin = PD2; 

      byte heater_state = 0;            // 0 = OFF, 1 = ON
      byte heater_state_ctl = 0;        // 0 = OFF, 1 = ON, 2=AUTOMATIC

    // ############################ ventilator (socket switch + timing)
      int const ventilator_pin = PD5;  

      byte ventilator_state = 0;      
      byte ventilator_state_ctl = 0;      // 0 = OFF, 1 = ON, 2=AUTOMATIC

      byte vent_minute      = 0;
      byte vent_minute_pre  = 0; 

      const byte vent_min_array[31]={
        0 ,2 ,4 ,6 ,8 ,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60
        };  // minutes of an hour
      const byte vent_state_array[31]={
        1 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0
        };    // on/off time of ventilator can be programmed here (0=off, 1=on within one minute)

    // ############################ LED (socket switch + timing)
      int const led_pin = PD4; 

      byte led_state = 0;      
      byte led_state_ctl = 0;             // 0 = OFF, 1 = ON, 2 = 18/6 Belichtungszyklus (04:00 Uhr bis 22:00 Uhr), 
                                          // 3 = 12/12 Belichtungszyklus (08:00 Uhr bis 20:00 Uhr), 4 = Entsprechend Array 
      byte led_minute = 0;
      byte led_minute_pre = 0; 

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
        

    
  // ############################ Communication Arduino Uno <-> ESP8266
    // MASTER = ESP8266, SLAVE = ARDUINO
    #define SLAVE_ADDRESS 9           // MASTER = ESP8266, SLAVE = ARDUINO
    
    // ############################ MASTER -> SLAVE
      const bool debug_master2slave     = true;
      const byte data_bytes_from_master = 7;
      byte data_from_master[data_bytes_from_master]; // Array zur Speicherung der empfangenen Daten
      byte msg_req_cnt = 0;

    // ############################ SLAVE -> MASTER
      const bool debug_slave2master = true;

      byte send0 = 0;
      byte send1 = 0;
      byte send2 = 0;
      byte send3 = 0;
      byte send4 = 0;
      byte send5 = 0;
      byte send6 = 0;
      byte send7 = 0;
      byte send8 = 0;
      byte send9 = 0;
      byte send10 = 0;
      byte send11 = 0;
      byte send12 = 0;
      byte send13 = 0;
      byte send14 = 0;
      byte send15 = 0;
      byte send16 = 0;
      byte send17 = 0;
      byte send18 = 0;

  // ############################ RTC
    // Die RTC muss über die Pin's A4 und A5 verbunden werden (siehe Arduino UNO Pinout und Dokumentation https://github.com/StephanFink/RTClib/, UNO is ATmega328).
    // Die Kommunikation ist I2C. 
    // A4 = SDA (serial data) = D18
    // A5 = SCL (serial clock) = D19 
    RTC_DS3231 RTC; 
    const bool syncOnFirstStart = false;  // true, falls die Zeitinformationen der RTC mit dem PC synchronisiert werden sollen (default = FALSE)

    const bool RTC_init     = false;

    String timestamp  = "dd.mm.yyyy HH:MM:ss";
    byte _minute      = 0;
    byte _hour        = 0;
    byte _second      = 0;
    word _hourminute  = 0;

    struct RTC_s {
        byte hour;
        byte minute;
        byte hourminute;
    };

  // ############################ Timer for timed loops
    const bool debug_timers = false;

    const int cycle_1000ms          = 1000;      
    unsigned long cycle_1000ms_dt   = 0;

    const int cycle_1500ms          = 1500;
    unsigned long cycle_1500ms_dt   = 0;

    unsigned long timer_read_pre    = 0;
    const unsigned long timer_read_period = 2000;  

    unsigned long millisec;           // arduino time-ms

// ############################ Functions

  RTC_s getRTC() {
      DateTime now = RTC.now();
      RTC_s outputs;
      outputs.hour = now.hour();
      outputs.minute = now.minute();
      outputs.hourminute = outputs.minute + 60 * outputs.hour;  // Berechne hourminute (Minuten des Tages)
      return outputs;
  }

// ############################ Setup
void setup(void){
// ############################ Communication Arduino Uno <-> ESP8266 (Node MCU)
  Wire.begin(SLAVE_ADDRESS);      // Initialize Interface
  Wire.onReceive(receiveEvent);   // Initialize EVENT
  Wire.onRequest(requestEvent);   // Initialize EVENT

// ############################ Debugging Schnittstelle  
  Serial.begin(9600);             // for Debugging/print  
  delay(500);
  
// ############################ Digital-Output

  pinMode(heater_pin, OUTPUT);
  pinMode(pipevent_pin, OUTPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(ventilator_pin, OUTPUT);

  digitalWrite(heater_pin, LOW);
  digitalWrite(pipevent_pin, LOW);
  digitalWrite(led_pin, LOW);
  digitalWrite(ventilator_pin, LOW);

  delay(500);

// ############################ Temperature sensors

  tempsensors.begin();    // Temperatursensoren starten         
  dht1.begin();           // Feuchtigkeitssensor DHT22 (1) starten
  dht2.begin();           // Feuchtigkeitssensor DHT22 (2) starten
      
  delay(500);
  
// ############################ RTC

  if (RTC_init == true) {
    // Hier kann (einmalig oder bei Wechsel der Sommer- bzw. Winterzeit) die Zeit für die RTC initialisiert werden
    RTC.adjust(DateTime(2024, 8, 23, 18, 23, 0)); // Adjust time YYYY,MM,DD,hh,mm,ss    
  }
  
  if (! RTC.begin()) {
    Serial.println("Error: RTC can not be initialized");
    //while (1);
  }
  else {
    Serial.println("RTC initialized");    
  }

  delay(1000);

  // Uhrzeit einmalig bei Start von der RTC holen
  RTC_s rtc_values = getRTC();
  _minute     = rtc_values.minute;
  _hour       = rtc_values.hour;
  _hourminute = rtc_values.hourminute;
  
  Serial.println("RTC-time: HH:MM - " + String(_hour) + ":" + String(_minute) +" , Hour-Minute - " + String(_hourminute));


// ############################ Interrupt-Output (PWM pins)

  /*
    PWM_frequency = 16 Mhz / (2 * 1024 * icr_const)
    Control register setup according to ATMEGA
    https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
    This sets it so OCA1 and OCB1, connected to pin 9 and pin 10, to be inverted
    in comparison to one another. Also sets Waveform generator mode to 10.
  */
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM11);  // pin setup
  TCCR1B = (1 << WGM13) | (1 << CS12) | (0 << CS11) | (1 >> CS10);        // prescale 1024

  ICR1 = icr_const; 
  DDRB = (1 << DDB2) | (1 << DDB1);  // Configure DDR2 and DDR1 to be output mode, for pin 9 and pin 10.
  OCR1A = (duty_cycle/100)*icr_const;
  OCR1B = (duty_cycle/100)*icr_const;

  //pinMode(LED_BUILTIN, OUTPUT);         // Interrupt-Input for testing
  //pinMode(interruptPin, INPUT_PULLUP);  // Interrupt-Input for testing
  //attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);  // Interrupt-Input for testing

  //pinMode(pot1, INPUT);   // Debugging/Simulation
  //pinMode(pot2, INPUT);   // Debugging/Simulation

  //In case of no ESP8266 (Node MCU) connection set initial values
  //These values are immediately overwritten by the ESP8266

// ############################ Initial values (if ESP8266 is not available)

  heater_state_ctl      = 0;    // OFF
  pipevent_state_ctl    = 1;    // ON    
  led_state_ctl         = 3;    // 12/12
  ventilator_state_ctl  = 1;    // ON
  pipevent_dutycycle_ctl = 50;  // 50 %

}

// #############################################################  Loop

void loop(void){
  
millisec = millis();      // get time from arduino-clock (time since arduino is running in ms)

// ############################ 1000 ms

  if (millisec - cycle_1000ms_dt >= cycle_1000ms) {
    cycle_1000ms_dt = millisec;

    if (debug_timers == true) {
      Serial.println("Start 1000 ms");
      Serial.println("millisec: "+String(millisec));
    }

    // ############################################ Calculate times with second-timer
      _second++;
    
      if (_second >= 60) {
        _second = 0;
        _minute++;
        if (_minute >= 60) {
          _minute = 0;
          _hour++;
          if (_hour >= 24) {
            _hour = 0;
            
            // Altuelle Uhrzeit einmal am Tag von der RTC holen
            RTC_s rtc_values  = getRTC();
            _minute           = rtc_values.minute;
            _hour             = rtc_values.hour;
            _hourminute       = rtc_values.hourminute;
          }
        }
        _hourminute = _minute + 60*_hour; // calc hourm (minutes of the day)
      }
      
      if (debug_timers == true) {
        Serial.println("Zeiten"); 
        Serial.println("Sekunde: "+String(_second));
        Serial.println("Minute: "+String(_minute));
        Serial.println("Stunde: "+String(_hour));
        Serial.println("Stundenminute: "+String(_hourminute));
      }

    // ############################################ Trigger für Statuswechsel (LED / Ventilator) updaten)
      vent_minute     = _minute;
      pipevent_minute = _minute;
      led_minute      = _minute;
    
    // ############################################   pipe ventilator

      // ############################################   Pipe ventilator controller (NOT USED)
      // T_act = TS01;   
      // T_err = (T_act - T_set);                  // calc error
      // T_i_prev = T_i_prev + T_err;              // integrate
      // duty_cycle = P_ * T_err + i_ * T_i_prev;  // calc output (P-Anteil + I-Anteil)

      switch (pipevent_state_ctl) {
        case 0:
          pipevent_state = 0;
          break;
        case 1:
          pipevent_state = 1;
          break;
        case 2:
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
              if (pipevent_state == 0) {
                pipevent_state = 1;
                pipevent_minute_change = pipevent_minute_ON; // Set pipeventilator ON for X min
                //Serial.println("pipeventilator ON for (min) - "+String(pipevent_minute_ON));
              }
              else {
                pipevent_state = 0;
                pipevent_minute_change = pipevent_minute_OFF; // Set pipeventilator ON for X min
                //Serial.println("pipeventilator OFF for (min) - "+String(pipevent_minute_OFF));
              }
            }
            
          }
          break;
                  
        default:
        break;
      }

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

      OCR1A = (duty_cycle/100)*icr_const;     // Send to interrupt-output
      OCR1B = (duty_cycle/100)*icr_const;     // Send to interrupt-output
    
    if (debug_timers == true) {
      Serial.println("End 1000 ms");
    }
    
  }
// ############################ 1500 ms

  if (millisec - cycle_1500ms_dt >= cycle_1500ms) {
    cycle_1500ms_dt = millisec;
    
    if (debug_timers == true) {
      Serial.println("Start 1500 ms"); 
    }

    // ############################################   heater
      switch (heater_state_ctl) {
        case 0:
          heater_state = 0;
          break;
        case 1:
          heater_state = 1;
          break;
        case 2:
          // Add code for automation here 
          break;  
        default:
          break;
      }

    // ############################################   ventilator
      switch (ventilator_state_ctl) {
        case 0: // manual OFF
          ventilator_state = 0;
          break;
        case 1: // manual ON
          ventilator_state = 1;
          break;
        case 2: // automatic   
        // At change of minute value
        if (vent_minute != vent_minute_pre) {
          vent_minute_pre = vent_minute;

          if (debug_array == true) {
          //Serial.println("check ventilator state");
          }

          // Change ventilator state (on/off) according to actual minutes and state-array
          for (word i = 0; (i < sizeof(vent_min_array) / sizeof(vent_min_array[0]) && i < 1000) ; i++){
            if (_minute == vent_min_array[i]) {
              ventilator_state = vent_state_array[i];
              if (debug_array == true) {
                //Serial.println("change ventilator state (" + String(ventilator_state) + ") at minute: " + String(_minute));
              }
              i = 1000; //exit for-loop
            }
            else if (_minute < vent_min_array[i]) {
              ventilator_state = vent_state_array[i-1];
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
    
    // ############################################   LED
      switch (led_state_ctl) {
        case 0: // manual OFF
          led_state = 0;
          break;
        case 1: // manual ON
          led_state = 1;
          break;
        case 2: // 18/6 Belichtungszyklus (04:00 Uhr bis 22:00 Uhr)
          if ((_hourminute >= 240) && (_hourminute <= 1320)) {
            led_state = 1;
          }
          else {
            led_state = 0;
          }
          break;
        case 3: // 12/12 Belichtungszyklus (08:00 Uhr bis 20:00 Uhr)
          if ((_hourminute >= 480) && (_hourminute <= 1200)) {
            led_state = 1;
          }
          else {
            led_state = 0;
          }
          break;
        case 4: // Entsprechend Array   
          // At change of minute value
          if (led_minute != led_minute_pre) {
            led_minute_pre = led_minute;

            if (debug_array == true) {
            //Serial.println("check LED state");
            }

            // Change led state (on/off) according to actual hour-minutes and state-array
            for (word i = 0; (i < sizeof(led_hourm_array) / sizeof(led_hourm_array[0]) && i < 1000) ; i++){
              if (_hourminute == led_hourm_array[i]) {
                led_state = led_state_array[i];
                if (debug_array == true) {
                  //Serial.println("change LED state (" + String(led_state) + ") at hour minute: " + String(_hourminute));
                }
                i = 1000; //exit for-loop
              }
              else if (_hourminute < led_hourm_array[i]) {
                led_state = led_state_array[i-1];
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

    // ############################################   Write hardware outputs
      digitalWrite(heater_pin,       heater_state);
      digitalWrite(pipevent_pin,    pipevent_state);
      digitalWrite(led_pin,         led_state);
      digitalWrite(ventilator_pin,  ventilator_state);  

    if (debug_timers == true) {
      Serial.println("End 1500 ms");
    }
  }




// ############################ 2000 ms (read sensors)

  if ((millisec - timer_read_pre > timer_read_period)) {
    timer_read_pre = millisec;
  
    if (debug_timers == true) {
      Serial.println("Read sensors Start 2000 ms (read sensors)");
    }

    // ############################################   READ temperature sensors
      tempsensors.requestTemperatures();

      // Serial.println("Temperature is: " + String(tempsensors.getTempCByIndex(0)) + "°C");

      TS01 = tempsensors.getTempCByIndex(0);
      TS02 = tempsensors.getTempCByIndex(1);
      TS03 = tempsensors.getTempCByIndex(2);

      if (debug_temp_DS18B20 == true) {
        Serial.println("Temperaturen [°C] der DS18B20-Sensoren");
        PRINT_VARIABLE(TS01);    
        PRINT_VARIABLE(TS02);    
        PRINT_VARIABLE(TS03);    
      }

    // ############################################   READ soil moisture sensor
      int debocap_value = analogRead(debocap_pin);
    
      // Konvertiere den analogen Wert in eine Bodenfeuchte-Prozentzahl
      debocap_percentage = map(debocap_value, debocap_water, debocap_dry_air, 100, 0);

      if (debug_debocap == true) {
        Serial.println("Feuchtigkeit [%] und Spannung [V] des DEBOCAP"); 
        PRINT_VARIABLE(debocap_percentage);
        PRINT_VARIABLE(debocap_value);
      }

    // ############################################   READ humidity sensor

      FS01_LF_int = dht1.readHumidity();        
      FS01_T_int  = dht1.readTemperature(); 
          
      FS02_LF_int = dht2.readHumidity();        
      FS02_T_int  = dht2.readTemperature();  

      if (debug_DHT22 == true) {
        Serial.println("Relative Luftfeuchtigkeit [%] und Temperatur [°C] des DHT22");
        PRINT_VARIABLE(FS01_LF_int);
        PRINT_VARIABLE(FS01_T_int);
        PRINT_VARIABLE(FS02_LF_int);
        PRINT_VARIABLE(FS02_T_int);  
      }        

    if (debug_timers == true) {
      Serial.println("End 2000 ms (read sensors)");
    }

  }
  
  //  ##############################################################  End of loop 

}

// ############################################ MASTER -> SLAVE
void receiveEvent(int bytes) {
  
  if (debug_master2slave == true) {
    Serial.println("Master2Slave: Data Event started");
  }
  
  // Stelle sicher, dass die richtige Anzahl an Bytes empfangen wurde
  if (bytes == data_bytes_from_master) {

    // Lese die empfangenen Daten in das Array
    for (int i = 0; i < bytes; i++) {
      data_from_master[i] = Wire.read();
      if (debug_master2slave == true) {
        Serial.println("Read: Data - "+String(i));
      }
    }

    heater_state_ctl =        data_from_master[0];   
    pipevent_state_ctl =      data_from_master[1];
    led_state_ctl =           data_from_master[2];
    ventilator_state_ctl =    data_from_master[3];
    pipevent_dutycycle_ctl =  data_from_master[4];
    pipevent_minute_ON =      data_from_master[5];
    pipevent_minute_OFF =     data_from_master[6];
        
    if (debug_master2slave == true) {    
      PRINT_VARIABLE(heater_state_ctl);
      PRINT_VARIABLE(pipevent_state_ctl);
      PRINT_VARIABLE(led_state_ctl);
      PRINT_VARIABLE(ventilator_state_ctl);
      PRINT_VARIABLE(pipevent_dutycycle_ctl);
      PRINT_VARIABLE(pipevent_minute_ON);
      PRINT_VARIABLE(pipevent_minute_OFF);
    }
  }
  
  // Setze alle Werte im Array auf 0
  for (int i = 0; i < data_bytes_from_master; i++) {
    data_from_master[i] = 0;
  }

  if (debug_master2slave == true) {
    Serial.println("Master2Slave: End");
  }
}

// ############################################ Data REQUEST from MASTER
void requestEvent() {

  msg_req_cnt++;
  if (debug_slave2master == true) {
    Serial.println("Slave2Master: Message Request Counter - "+String(msg_req_cnt));
    PRINT_VARIABLE(msg_req_cnt);
  }

  // Float in Integer umwandeln
  TS01_int = TS01*10;
  TS02_int = TS02*10;
  TS03_int = TS03*10;
  duty_cycle_int = duty_cycle*10;
        
  // Integer in Byte umwandeln für die Übertragung per I2C
  send0 = lowByte(TS01_int); 
  send1 = highByte(TS01_int);
  send2 = lowByte(TS02_int); 
  send3 = highByte(TS02_int);
  send4 = lowByte(TS03_int); 
  send5 = highByte(TS03_int);
  send6 = lowByte(duty_cycle_int); 
  send7 = highByte(duty_cycle_int);
  send8 =  heater_state;
  send9 =  pipevent_state;
  send10 = led_state;
  send11 = ventilator_state;
  send12 = _hour;
  send13 = _minute;
  send14 = lowByte(debocap_percentage);
  send15 = highByte(debocap_percentage);
  send16 = msg_req_cnt;
  send17 = FS01_LF_int;
  send18 = FS02_LF_int;

  if (debug_slave2master == true) {
    PRINT_VARIABLE(send0);
    PRINT_VARIABLE(send1);
    PRINT_VARIABLE(send2);
    PRINT_VARIABLE(send3);
    PRINT_VARIABLE(send4);
    PRINT_VARIABLE(send5);
    PRINT_VARIABLE(send6);
    PRINT_VARIABLE(send7);
    PRINT_VARIABLE(send8);
    PRINT_VARIABLE(send9);
    PRINT_VARIABLE(send10);
    PRINT_VARIABLE(send11);
    PRINT_VARIABLE(send12);
    PRINT_VARIABLE(send13);
    PRINT_VARIABLE(send14);
    PRINT_VARIABLE(send15);
    PRINT_VARIABLE(send16);
    PRINT_VARIABLE(send17);
    PRINT_VARIABLE(send18);
  }

  // Sende Daten an den Master
  Wire.write(send0);
  Wire.write(send1);
  Wire.write(send2);
  Wire.write(send3);
  Wire.write(send4);
  Wire.write(send5);
  Wire.write(send6);
  Wire.write(send7);
  Wire.write(send8);
  Wire.write(send9);
  Wire.write(send10);
  Wire.write(send11);
  Wire.write(send12);
  Wire.write(send13);
  Wire.write(send14);
  Wire.write(send15);
  Wire.write(send16);
  Wire.write(send17);
  Wire.write(send18);

  if (debug_slave2master == true) {
    Serial.println("Slave2Master: End");
  }
}
