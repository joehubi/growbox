/*
  Johannes Huber
  https://github.com/joehubi/growbox
  29.04.2024
*/

// #############################################################  Library

#include <Wire.h>             // Die Datums- und Zeit-Funktionen der DS3231 RTC werden �ber das I2C aufgerufen.
#include "RTClib.h"           // https://github.com/StephanFink/RTClib/archive/master.zip
#include <OneWire.h>
#include <DallasTemperature.h>

#define SLAVE_ADDRESS 9
// MASTER = ESP8266
// SLAVE = ARDUINO

// #############################################################  Debugging Variables

const bool debug_misc = false;
const bool debug_auto = false;

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

const int TS_max = 50;
int TS_error = 0;

// #############################################################  Actuators

byte dummy_state = 0;      // Simulation
byte dummy_state_ctl = 2;  // Init to Automatic Mode
int const dummy_pin = PD2; 

byte pipevent_state = 0;   // 0 = OFF, 1 = ON 
byte pipevent_state_ctl = 2; 
int const pipevent_pin = PD3; 
 
byte led_state = 0;      
byte led_state_ctl = 2; 
int const led_pin = PD4; 
 
byte ventilator_state = 0;      
byte ventilator_state_ctl = 2; 
int const ventilator_pin = PD5;  

// #############################################################  Variables

const byte data_bytes_from_master = 4;
byte data_from_master[data_bytes_from_master]; // Array zur Speicherung der empfangenen Daten

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
  
RTC_DS3231 rtc;

const bool syncOnFirstStart = false;  // true, falls die Zeitinformationen der RTC mit dem PC synchronisiert werden sollen.
                                      // sollte standardm��ig auf false stehen
String timestamp = "dd.mm.yyyy HH:MM:ss";


byte vent_minute = 0;
byte vent_minute_pre = 0; 

byte led_minute = 0;
byte led_minute_pre = 0; 

byte _minute = 0;
byte _hour = 0;
word word_hourm = 0;

const byte vent_min_array[31]={
  0 ,2 ,4 ,6 ,8 ,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60
  };  // minutes of an hour
const byte vent_state_array[31]={
  1 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0 ,1 ,1 ,1 ,0 ,0
  };    // on/off time of ventilator can be programmed here (0=off, 1=on within one minute)

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
  
// Interupt-Output interrupt for pipe ventilator
double duty_cycle = 0.0;     // This duty cycle is present on pin 9 and inverted on Pin 10
int duty_cycle_int = 0;    
const double duty_cycle_min = 15.0;     
const double duty_cycle_max = 100.0;     
//int icr_const = 8000;       // -> results in 1 Hz PWM cycle (debugging)
int icr_const = 7;            // -> results in 1,12 kHz PWM cycle 

// Control of the pipe ventilator
const float P_ = 2;           // P-Anteil
const float i_ = 0.01;        // I-Anteil
const float T_set = 25.0;     // Setpoint
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
const unsigned long timer_read_period = 2000;  
//unsigned long timer_send_pre = 0;
//const unsigned long timer_send_period = 5000;  

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

  Wire.begin(SLAVE_ADDRESS); // I2C default nur auf SDA und SCL Pin's des Arduino
  Wire.onReceive(receiveEvent); // MASTER schickt Daten an SLAVE
  Wire.onRequest(requestEvent); // MASTER fordert Daten an SLAVE
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

  //nodemcu.setTimeout(mcutimeout);
  //nodemcu.begin(mcubaudrate);
  //pinMode(tx, OUTPUT);    // Serial interface to NodeMCU
  //pinMode(rx, INPUT);     // Serial interface to NodeMCU
  delay(500);

// ############################  Temperature sensors

  tempsensors.begin();
  delay(500);
  
// ############################  RTC

  if (! rtc.begin()) {
    Serial.println("RTC can not be initialized");
    while (1);
  }

  // Hier kann (einmalig oder bei Wechsel der Sommer- bzw. Winterzeit) die Zeit für die RTC initialisiert werden
  // rtc.adjust(DateTime(2023, 8, 23, 18, 23, 0)); // Adjust time YYYY,MM,DD,hh,mm,ss


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
    if (vent_minute != vent_minute_pre) {
      vent_minute_pre = vent_minute;

      if (debug_auto == true) {
      Serial.println("check ventilator state");
      }

      // Change ventilator state (on/off) according to actual minutes and state-array
      for (word i = 0; (i < sizeof(vent_min_array) / sizeof(vent_min_array[0]) && i < 1000) ; i++){
        if (_minute == vent_min_array[i]) {
          ventilator_state = vent_state_array[i];
          if (debug_auto == true) {
            Serial.println("change ventilator state (" + String(ventilator_state) + ") at minute: " + String(_minute));
          }
          i = 1000; //exit for-loop
        }
        else if (_minute < vent_min_array[i]) {
          ventilator_state = vent_state_array[i-1];
          if (debug_auto == true) {
            Serial.println("change ventilator state (" + String(ventilator_state) + ") at minute: " + String(_minute));
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
      if ((word_hourm >= 240) && (word_hourm <= 1320)) {
        led_state = 1;
      }
      else {
        led_state = 0;
      }
      break;
    case 3: // 12/12 Belichtungszyklus (08:00 Uhr bis 20:00 Uhr)
      if ((word_hourm >= 480) && (word_hourm <= 1200)) {
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

        if (debug_auto == true) {
        Serial.println("check LED state");
        }

        // Change led state (on/off) according to actual hour-minutes and state-array
        for (word i = 0; (i < sizeof(led_hourm_array) / sizeof(led_hourm_array[0]) && i < 1000) ; i++){
          if (word_hourm == led_hourm_array[i]) {
            led_state = led_state_array[i];
            if (debug_auto == true) {
              Serial.println("change LED state (" + String(led_state) + ") at hour minute: " + String(word_hourm));
            }
            i = 1000; //exit for-loop
          }
          else if (word_hourm < led_hourm_array[i]) {
            led_state = led_state_array[i-1];
            if (debug_auto == true) {
              Serial.println("change LED state at (" + String(led_state) + ") hour minute: " + String(word_hourm));
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
        pipevent_state = 1; 
        
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

    _hour = now.hour();
    _minute = now.minute();
    word_hourm = now.minute() + 60*(now.hour());   // calc hourm (minutes of the day)

    // Trigger für Statuswechsel (LED / Ventilator updaten)
    vent_minute = _minute;
    led_minute = _minute;

  }


// ############################################   READ

  if ((millisec - timer_read_pre > timer_read_period)) {
  timer_read_pre = millisec;

  // ############################################   READ temperature sensors

  tempsensors.requestTemperatures();
  //Serial.println("Temperature is: " + String(tempsensors.getTempCByIndex(0)) + "°C");
  TS01 = tempsensors.getTempCByIndex(0);
  TS02 = tempsensors.getTempCByIndex(1);
  TS03 = tempsensors.getTempCByIndex(2);

    // Maximum temperature ?
    if (TS01 > TS_max || TS02 > TS_max) {
      
      //TS_error = 1; 
      
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

    if (debug_misc == true) {
      Serial.println("duty_cycle:"+String(duty_cycle)); 
    }  

  }

  // digitalWrite(LED_BUILTIN, state);     // Debuggin / switch LED for interrupt testing   

//  ##############################################################  End of loop 

}


//  ##############################################################  Data RECEIVE from MASTER

void receiveEvent(int bytes) {
  //Serial.println("Data RECEIVE from MASTER");

  // Stelle sicher, dass die richtige Anzahl an Bytes empfangen wurde
  if (bytes == data_bytes_from_master) {

    // Lese die empfangenen Daten in das Array
    for (int i = 0; i < bytes; i++) {
      data_from_master[i] = Wire.read();
    }

    dummy_state_ctl =       data_from_master[0];   
    pipevent_state_ctl =    data_from_master[1];
    led_state_ctl =         data_from_master[2];
    ventilator_state_ctl =  data_from_master[3];
    
    // Zeige die Werte im Serial Monitor an
    //Serial.print("Received control: ");
    //Serial.print(dummy_state_ctl);
    //Serial.print(", ");
    //Serial.print(pipevent_state_ctl);
    //Serial.print(", ");
    //Serial.print(led_state_ctl);
    //Serial.print(", ");
    //Serial.println(ventilator_state_ctl);
    
  }
  
  // Setze alle Werte im Array auf 0
  for (int i = 0; i < data_bytes_from_master; i++) {
    data_from_master[i] = 0;
  }
  
}

//  ##############################################################  Data REQUEST from MASTER

void requestEvent() {
  //Serial.println("Data REQUEST from MASTER");

  // 4 x Integer (je 2 Bytes)
  // 6 x Byte 
  // -> Insgesamt 14 Bytes

  // Float in Integer umwandeln
  TS01_int = TS01*10;
  TS02_int = TS02*10;
  TS03_int = TS03*10;
  duty_cycle_int = duty_cycle*10;
    
  // Integer in Byte umwandeln für die Übertragung per I2C
  send1 = lowByte(TS01_int); 
  send2 = highByte(TS01_int);
  send3 = lowByte(TS02_int); 
  send4 = highByte(TS02_int);
  send5 = lowByte(TS03_int); 
  send6 = highByte(TS03_int);
  send7 = lowByte(duty_cycle_int); 
  send8 = highByte(duty_cycle_int);
  send9 =   dummy_state;
  send10 =  pipevent_state;
  send11 =  led_state;
  send12 =  ventilator_state;
  send13 =  _hour;
  send14 =  _minute;

  //Serial.print("Send data to MASTER:");
  //Serial.print(send13);
  //Serial.print(", ");
  //Serial.println(send14);
   
  // Sende Daten an den Master
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

}
