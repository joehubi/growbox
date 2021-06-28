/*
  Johannes Huber
  https://github.com/joehubi/growbox
*/

/*
  Library
*/
#include <SoftwareSerial.h>   //https://www.arduino.cc/en/Reference/SoftwareSerial
#include <ArduinoJson.h>      //https://github.com/bblanchon/ArduinoJson


// OUTPUT interrupt
double duty_cycle = 80.0; // This duty cycle is present on pin 9 and inverted on Pin 10
int icr_const = 8000;

// Control of the pipe ventilator
    float P_ = 2;
    float i_ = 0.01;
    float T_set = 30.0;
    float T_act = 0.0;
    float T_err = 0.0;
    float T_i_prev = 0.0;


// INPUT testing
const byte interruptPin = 2;
volatile byte state = LOW;

// timer
int cycle_1000ms = 1000;
int cycle_3000ms = 3000;
unsigned long cycle_1000ms_dt;
unsigned long cycle_3000ms_dt;
unsigned long millisec;

// Pins for Serial conncetion
int rx = 5;
int tx = 6;

int const pot1 = A0;
int const pot2 = A1;

// Simulation
float TS01_sim = 0.0;
int FS01_sim = 0;

//Initialise serial connection (SC) to NodeMCU
SoftwareSerial nodemcu(rx, tx);

void setup(){

/*
  PWM pins
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

  /*
   * INPUT interrupt for testing
   */
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);

  
  // Potis
  pinMode(pot1, INPUT);
  pinMode(pot2, INPUT);

  // Serial interface
  pinMode(tx, OUTPUT);
  pinMode(rx, INPUT);
  Serial.begin(115200);
  nodemcu.begin(4800);

  delay(1000);
}
 
void loop(){

  millisec = millis();

  if (millisec - cycle_3000ms_dt > cycle_3000ms) {
    cycle_3000ms_dt = millisec; // save timer

    //Serial.print("Loop1");
    //Serial.print("\n");    
    
    /*
    Simulation
    */
  
    TS01_sim = analogRead(pot1)/10.1;
    FS01_sim = analogRead(pot2)/10;
  
    Serial.print("\n");
    Serial.print("TS01:");
    Serial.print(TS01_sim);
    Serial.print("\n");
    Serial.print("FS01:");
    Serial.print(FS01_sim);
    Serial.print("\n");
               
    /*
    Serial Send to NodeMCU via JSON
    */
    StaticJsonDocument<200> doc;
    doc["growbox"] = "webserver";
    doc["time"] = millisec;
  
    // Add data array
    JsonArray data = doc.createNestedArray("data");
    data.add(TS01_sim);
    data.add(FS01_sim);
  
    // Generate the minified JSON and send it to the Serial port.
    serializeJson(doc, Serial);   // debugging
    serializeJson(doc, nodemcu);  // to NodeMCU


  }

  if (millisec - cycle_1000ms_dt > cycle_1000ms) {
    cycle_1000ms_dt = millisec; // save timer

    /*
    Simulation
    */
    //Serial.print("Loop2");
    //Serial.print("\n");
//
//switch (var) {
//  case label1:
//    // Statement(s)
//    break;
//  case label2:
//    // Statement(s)
//    break;
//  default:
//    // Statement(s)
//    break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
//}

    // Control of the pipe ventilator
    T_act = analogRead(pot2)/10;
    T_err = (T_act - T_set);
 
    T_i_prev = T_i_prev + T_err;
    Serial.print("T_i_prev:");
    Serial.print(T_i_prev);
    Serial.print("\n");   

    duty_cycle = P_ * T_err + i_ * T_i_prev;

    // min. Output 
    if (duty_cycle < 10.0) {
      duty_cycle = 10.0;
    }
    // max. Ouput
    if (duty_cycle > 100.0) {
      duty_cycle = 100.0;
    }
    
    OCR1A = (duty_cycle/100)*icr_const;
    OCR1B = (duty_cycle/100)*icr_const;

    Serial.print("\n");
    Serial.print("duty cycle:");
    Serial.print(duty_cycle);
    Serial.print("\n");

  }

  // switch LED for interrupt testing
  digitalWrite(LED_BUILTIN, state);
    
  delay(1);
}

/*
 * Funktionen
 * 
 */
// switch LED
void blink() {
  state = !state;
}
