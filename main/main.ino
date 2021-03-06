/*
  Johannes Huber
  https://github.com/joehubi/growbox
*/

// #############################################################  Library

#include <Wire.h>             // Die Datums- und Zeit-Funktionen der DS3231 RTC werden �ber das I2C aufgerufen.
#include "RTClib.h"           // https://github.com/StephanFink/RTClib/archive/master.zip
#include <SoftwareSerial.h>   //https://www.arduino.cc/en/Reference/SoftwareSerial
#include <ArduinoJson.h>      //https://github.com/bblanchon/ArduinoJson
#include <OneWire.h>
#include <DallasTemperature.h>

// #############################################################  Debugging Variables

word debug_level = 2;

// Interupt-Input for testing the Interupt-Output
//const byte interruptPin = 2;      // Debugging
//volatile byte state = LOW;        // Debugging

// #############################################################  Temperature sensors

#define ONE_WIRE_BUS 8                    // Data wire is plugged into pin 8 on the Arduino
OneWire oneWire(ONE_WIRE_BUS);            // Setup a OneWire instance to communicate with any OneWire devices
DallasTemperature tempsensors(&oneWire);  // Pass OneWire reference to Dallas Temperature

float TS01 = 0.0;  
float TS02 = 0.0; 
float TS03 = 0.0;    
int TS01_int = 0;         
int TS02_int = 0;         
int TS03_int = 0;         

int TS_max = 40;
int TS_error = 0;

// #############################################################  Actuators

byte dummy_state = 0;      // Simulation
byte dummy_state_ctl = 2;  // Init to Automatic Mode
int const dummy_pin = PD2; 

byte pipevent_state = 0;      
byte pipevent_state_ctl = 2; 
int const pipevent_pin = PD3; 
 
byte led_state = 0;      
byte led_state_ctl = 2; 
int const led_pin = PD4; 
 
byte ventilator_state = 0;      
byte ventilator_state_ctl = 2; 
int const ventilator_pin = PD5;  

// #############################################################  Variables

const int mcutimeout = 300;
const int mcubaudrate = 1200;
const int txbuffer = 150;
const int rxbuffer = 220;
int rx = 6;                       // Receive pin for serial conncetion
int tx = 7;                       // Send pin for serial conncetion
SoftwareSerial nodemcu(rx,tx);    //Initialise serial connection (SC) to NodeMCU

RTC_DS3231 rtc;

bool syncOnFirstStart = false; // true, falls die Zeitinformationen der RTC mit dem PC synchronisiert werden sollen.
                              // sollte standardm��ig auf false stehen
String timestamp = "dd.mm.yyyy HH:MM:ss";
word word_minute = 0;
word word_minute_pre = 0; 
word word_hour = 0;
word word_hourm = 0;

word vent_min_array[31]={
  0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60
  };  // minutes of an hour
word vent_state_array[31]={
  0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0
  };    // on/off time of ventilator can be programmed here (0=off, 1=on within one minute)

word led_hourm_array[96]={
  0,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,270,285,300,315,
  330,345,360,375,390,405,420,435,450,465,480,495,510,525,540,555,570,585,600,615,
  630,645,660,675,690,705,720,735,750,765,780,795,810,825,840,855,870,885,900,915,
  930,945,960,975,990,1005,1020,1035,1050,1065,1080,1095,1110,1125,1140,1155,1170,
  1185,1200,1215,1230,1245,1260,1275,1290,1305,1320,1335,1350,1365,1380,1395,1410,1425
  };  // minutes of a 24 hour day
word led_state_array[96]={
  0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,
  0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,
  0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,
  0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,
  0,1,0,1,0,1,0,1
  };    // on/off time of LED can be programmed here (0=off, 1=on within one day)
  
//String day_ = "";
//String month_ = "";
//String hour_ = "";
//String minute_ = "";
//String second_ = "";

// Interupt-Output interrupt for pipe ventilator
double duty_cycle = 0.0;     // This duty cycle is present on pin 9 and inverted on Pin 10
int duty_cycle_int = 0;    
const double duty_cycle_min = 15.0;     
const double duty_cycle_max = 100.0;     
//int icr_const = 8000;       // -> results in 1 Hz PWM cycle (debugging)
int icr_const = 7;            // -> results in 1,12 kHz PWM cycle 

// Control of the pipe ventilator
float P_ = 2;           // P-Anteil
float i_ = 0.01;        // I-Anteil
float T_set = 25.0;     // Setpoint
float T_act = 0.0;      // Actual value of sensor
float T_err = 0.0;      // Error (actual value - setpoint)
float T_i_prev = 0.0;   // buffer for integrator

// Timer for timed loops
const int cycle_1000ms = 1000;      
unsigned long cycle_1000ms_dt;
const int cycle_1500ms = 1500;
unsigned long cycle_1500ms_dt;
const int cycle_5000ms = 5000;
unsigned long cycle_5000ms_dt;
const int cycle_debug = 3000;
unsigned long cycle_debug_dt;

unsigned long timer_read_pre = 0;
const unsigned long timer_read_period = 678;  
unsigned long timer_send_pre = 0;
const unsigned long timer_send_period = 5000;  

unsigned long millisec;           // arduino time-ms

// #############################################################  Functions

/*
// switch LED
void blink() {
  state = !state;
}
*/
 
/* // Fügt eine "0" bei Dezimal-Werten < 10 ein. z.B. "9" -> "09"
String add_null(String input) {
    //Serial.println("Input:"+input+"    Input lenght:"+String(input.length()));
    if (input.length() == 1) {
      input = "0"+input;
    }
    return input;
}
*/

// #############################################################      Setup

void setup(void){

  Serial.begin(9600);   // Debugging/print  
  delay(500);
  
// ############################  Digital-Output

  pinMode(dummy_pin, OUTPUT);
  pinMode(pipevent_pin, OUTPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(ventilator_pin, OUTPUT);

  digitalWrite(dummy_pin, LOW);
  digitalWrite(pipevent_pin, LOW);
  digitalWrite(led_pin, LOW);
  digitalWrite(ventilator_pin, LOW);

// ############################  NodeMCU

  nodemcu.setTimeout(mcutimeout);
  nodemcu.begin(mcubaudrate);
  pinMode(tx, OUTPUT);    // Serial interface to NodeMCU
  pinMode(rx, INPUT);     // Serial interface to NodeMCU
  delay(500);

// ############################  Temperature sensors

  tempsensors.begin();
  delay(500);
  
// ############################  RTC

  if (! rtc.begin()) {
    Serial.println("RTC can not be initialized");
    while (1);
  }

  //rtc.adjust(DateTime(2021, 11, 12, 14, 13, 0)); // Adjust time YYYY,MM,DD,hh,mm,ss


// ############################  Interrupt-Output (PWM pins)

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

//  pinMode(pot1, INPUT);   // Debugging/Simulation
//  pinMode(pot2, INPUT);   // Debugging/Simulation

}


// #############################################################      Loop

void loop(void){

millisec = millis();      // get time from arduino-clock (time since arduino is running in ms)

// ############################################   1500 ms

  if (millisec - cycle_1500ms_dt > cycle_1500ms) {
    cycle_1500ms_dt = millisec;

// ############################################   dummy switch

    switch (dummy_state_ctl) {
      case 0:
        dummy_state = 0;
        break;
      case 1:
        dummy_state = 1;
        break;
      case 2:
        // Add code for automation here 
//        if (dummy_state == 1) {
//          dummy_state = 0;
//         }
//        else {
//          dummy_state = 1;
//        }
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
    if (word_minute != word_minute_pre) {
      word_minute_pre = word_minute;

      // Change ventilator state (on/off) according to actual minutes and state-array
      for (word i = 0; (i < sizeof(vent_min_array) / sizeof(vent_min_array[0]) && i < 1000) ; i++){
        if (word_minute == vent_min_array[i]) {
          ventilator_state = vent_state_array[i];
          if (debug_level >= 1) {
            Serial.println("minute:"+String(word_minute));
          }
          i = 1000; //exit for-loop
        }
        else if (word_minute < vent_min_array[i]) {
          ventilator_state = vent_state_array[i-1];
          if (debug_level >= 1) {
            Serial.println("minute:"+String(word_minute));
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
    case 2: // automatic   
      // At change of minute value
      if (word_minute != word_minute_pre) {
        word_minute_pre = word_minute;
        
      // Change led state (on/off) according to actual hour-minutes and state-array
      for (word i = 0; (i < sizeof(led_hourm_array) / sizeof(led_hourm_array[0]) && i < 1000) ; i++){
        if (word_hourm == led_hourm_array[i]) {
          led_state = led_state_array[i];
          if (debug_level >= 1) {
            Serial.println("hour minute:"+String(word_hourm));
          }
          i = 1000; //exit for-loop
        }
        else if (word_hourm < led_hourm_array[i]) {
          led_state = led_state_array[i-1];
          if (debug_level >= 1) {
            Serial.println("hour minute:"+String(word_hourm));
          }
          i = 1000; //exit for-loop       
        }
        // next loop cycle
        }  
      break;
      }                
  }

 // ############################################   Write hardware outputs
  
  if (TS_error == 1) {
                   
    // Error
    dummy_state       = 0;
    pipevent_state    = 0;
    led_state         = 0;
    ventilator_state  = 0;

    digitalWrite(dummy_pin,         0);
    digitalWrite(pipevent_pin,      0);
    digitalWrite(led_pin,           0);
    digitalWrite(ventilator_pin,    0); 
  }
    // No error
  else {
    digitalWrite(dummy_pin,       dummy_state);
    digitalWrite(pipevent_pin,    pipevent_state);
    digitalWrite(led_pin,         led_state);
    digitalWrite(ventilator_pin,  ventilator_state);  
  }
  
  // ###### End 1500 ms
  }

// ############################################   SEND
  
// Send all relevant data to the webserver to view it in a browser
    
  if ((millisec - timer_send_pre > timer_send_period)) {
    timer_send_pre = millisec;
    
    TS01_int = TS01*10;
    TS02_int = TS02*10;
    TS03_int = TS03*10;
    duty_cycle_int = duty_cycle*10;
        
    StaticJsonDocument<txbuffer> txdoc;

    txdoc["growbox"] = "to NodeMCU";
    txdoc["time"] = millisec;      // send arduino time to NodeMCU

    JsonArray data = txdoc.createNestedArray("data");       // Add data array
    data.add(TS01_int);     // Add data ...
    data.add(TS02_int);
    data.add(TS03_int);
    data.add(duty_cycle_int);
    JsonArray state = txdoc.createNestedArray("state");         // Add state array
    state.add(dummy_state); // Add data ...  
    state.add(pipevent_state); 
    state.add(led_state);  
    state.add(ventilator_state);  
    JsonArray rtc_time = txdoc.createNestedArray("rtc_time");         // Add time array
    rtc_time.add(word_hour);  
    rtc_time.add(word_minute);  

    //Send data to NodeMCU
    serializeJson(txdoc, nodemcu);

    if (debug_level >= 1) {
      serializeJson(txdoc, Serial);                                
      Serial.print("\n");     
    }
    if (debug_level >= 2) {
      Serial.print("###SEND### tx-buffer: "+String(txdoc.size())+","+String(sizeof(txdoc))+","+String(txdoc.memoryUsage()));
      Serial.print("\n");     
    }
  }

// ############################################   1000 ms

  if (millisec - cycle_1000ms_dt > cycle_1000ms) {
    cycle_1000ms_dt = millisec;
    
// ############################################   Pipe ventilator controller
    T_act = TS01;   
    T_err = (T_act - T_set);                  // calc error
    T_i_prev = T_i_prev + T_err;              // integrate
    duty_cycle = P_ * T_err + i_ * T_i_prev;  // calc output (P-Anteil + I-Anteil)

// ############################################   Pipe ventilator output


    // ############################################   pipe ventilator

    switch (pipevent_state_ctl) {
      case 0:
        pipevent_state = 0;
        duty_cycle = 0.0;
        break;
      case 1:
        pipevent_state = 1;
        duty_cycle = 80.0;
        break;
      case 2:
        pipevent_state = 2; 
        
        // min. Output
        if (duty_cycle < duty_cycle_min) {
          duty_cycle = duty_cycle_min;
        }
        // max. Ouput
        if (duty_cycle > duty_cycle_max) {
          duty_cycle = duty_cycle_max;
        }       
        break;  
        
        default:
        break;
    }

    OCR1A = (duty_cycle/100)*icr_const;     // Send to interrupt-output
    OCR1B = (duty_cycle/100)*icr_const;     // Send to interrupt-output

  }

// ############################################   5000 ms 
  
  if (millisec - cycle_5000ms_dt > cycle_5000ms) {
    cycle_5000ms_dt = millisec;
    
// ############################################   Real time clock 

    DateTime now = rtc.now();

    word_hour = now.hour();
    word_minute = now.minute();
    word_hourm = now.minute() + 60*(now.hour());   // calc hourm (minutes of the day)

//    Serial.print("###TIME### "+String(word_hour)+":"+String(word_minute));
//    Serial.print("\n");
      
//    day_ = String(now.day());    
//    month_ = String(now.month());
//    hour_ = String(now.hour());
//    minute_ = String(now.minute());
//    second_ = String(now.second());
//       
//    day_ = add_null(day_);
//    month_ = add_null(month_);
//    // year (not neccesary) 
//    hour_ = add_null(hour_);
//    minute_ = add_null(minute_);
//    second_ = add_null(second_);
//     
//    timestamp = day_+"."
//                +month_+"."
//                +String(now.year())
//                +" "
//                +hour_+":"
//                +minute_+":"
//                +second_;
//    Serial.println(timestamp);
  }


// ############################################   READ

// Reads the switches from the NodeMCU (webserver) via Tx/Rx

  if ((millisec - timer_read_pre > timer_read_period)) {
  timer_read_pre = millisec;

  StaticJsonDocument<rxbuffer> rxdoc;
  DeserializationError error = deserializeJson(rxdoc, nodemcu);
 
  if (error) {
    if (debug_level >= 2) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      }    
    }
    else {
      if (rxdoc.isNull() || rxdoc.memoryUsage() < 76) {       
        } else {
          
          dummy_state_ctl =       rxdoc["switches"][0];    // Get data from array
          pipevent_state_ctl =    rxdoc["switches"][1];
          led_state_ctl =         rxdoc["switches"][2];
          ventilator_state_ctl =  rxdoc["switches"][3];
                    
          if (debug_level >= 1) {
            serializeJson(rxdoc, Serial);                                
            Serial.print("\n");
          }         
          if (debug_level >= 2) {
            Serial.print("###READ### rx-buffer: "+String(rxdoc.size())+","+String(sizeof(rxdoc))+","+String(rxdoc.memoryUsage()));  
            Serial.print("\n");
          }        
        }
      }

  // ############################################   temperature sensors

  tempsensors.requestTemperatures();
  //Serial.println("Temperature is: " + String(tempsensors.getTempCByIndex(0)) + "°C");
  TS01 = tempsensors.getTempCByIndex(0);
  TS02 = tempsensors.getTempCByIndex(1);
  TS03 = tempsensors.getTempCByIndex(2);

    // Maximum temperature ?
    if (TS01 > TS_max || TS02 > TS_max || TS01 < -10 || TS02 < -10) {
      TS_error = 1; 
      
      Serial.println("Temperature to high"); 
      Serial.println("TS_max:"+String(TS_max)); 
      Serial.println("TS01:"+String(TS01));
      Serial.println("TS02:"+String(TS02));
      Serial.println("TS03:"+String(TS03));
      Serial.print("\n"); 
    }
  }
  
//  ##############################################################  Debug 

  if (millisec - cycle_debug_dt > cycle_debug) {
    cycle_debug_dt = millisec;

    if (debug_level >= 1) {

    }  
      
    if (debug_level >= 2) {
      Serial.println("duty_cycle:"+String(duty_cycle)); 
    }  

    if (debug_level >= 3) {                    
    } 
  }


//  ##############################################################  End of loop 

   // digitalWrite(LED_BUILTIN, state);     // Debuggin / switch LED for interrupt testing   
   
}
