/*
  Johannes Huber
  https://github.com/joehubi/growbox
*/

/*
  Library
*/
#include <SoftwareSerial.h>   //https://www.arduino.cc/en/Reference/SoftwareSerial
#include <ArduinoJson.h>      //https://github.com/bblanchon/ArduinoJson

// Pins for Serial conncetion
int rx = 5;
int tx = 6;

// Simulation
float TS01_sim = 25.0;
int FS01_sim = 51;

//Initialise serial conncetion (SC) to NodeMCU
SoftwareSerial nodemcu(rx, tx);

void setup(){

  pinMode(tx, OUTPUT);
  pinMode(rx, INPUT);

  Serial.begin(115200);
  nodemcu.begin(4800);

  delay(1000);
}
 
void loop(){

  /*
  Simulation
  */
  if (FS01_sim > 56) {
    FS01_sim = 52; 
    }
    else {
      FS01_sim = FS01_sim + 1;
    }

  if (TS01_sim > 25.0) {
    TS01_sim = 23.2;
    } 
    else {
      TS01_sim = TS01_sim + 0.1;
    }

  /*
  Serial Send to NodeMCU via JSON
  */
  StaticJsonDocument<200> doc;
  doc["growbox"] = "webserver";
  doc["time"] = 1351824120;

  // Add data array
  JsonArray data = doc.createNestedArray("data");
  data.add(TS01_sim);
  data.add(FS01_sim);

  // Generate the minified JSON and send it to the Serial port.
  serializeJson(doc, Serial);
  serializeJson(doc, nodemcu);


  delay(2000);
}
