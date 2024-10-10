
// ################### DEFINITION
  // ################### Library
    #include <DHT.h>                  // https://github.com/Khuuxuanngoc/DHT-sensor-library.git

  // ################### DHT22 (temp & humidity)
    #define DHTPIN1 6          // Connected on Pin D6 on Arduino UNO    
    #define DHTPIN2 7          // Connected on Pin D7 on Arduino UNO      
    #define DHTTYPE DHT22      // Es handelt sich um den DHT22 Sensor

    DHT dht1(DHTPIN1, DHTTYPE);               // Der Sensor wird mit „dth1“ angesprochen
    DHT dht2(DHTPIN2, DHTTYPE);               // Der Sensor wird mit „dth2“ angesprochen

    class DHT22_th {
      public:
        int   temp  = 0;
        int   hum   = 0;
        bool  use   = true;
    };

    DHT22_th dht22_1;                 // build instance of class
    DHT22_th dht22_2;

void setup() {

    // ################### Debugging Monitor  
    Serial.begin(9600);             // for Debugging/print  
    while (!Serial) {
      ; // wait for serial port to connect
    }
    delay(1000);
    Serial.println("Serial Monitor initialized");

  // ################### DHT22
    if (dht22_1.use == true) {
      dht1.begin();           // Feuchtigkeitssensor DHT22 (1) starten
      Serial.println("DHT22_1 initialized");
    }
    if (dht22_2.use == true) {
      dht2.begin();           // Feuchtigkeitssensor DHT22 (2) starten
      Serial.println("DHT22_2 initialized");
    }
}

void loop() {
  // ############################################   READ humidity sensor
    if (dht22_1.use == true) {
      dht22_1.hum   = dht1.readHumidity();        
      dht22_1.temp  = dht1.readTemperature(); 
    }
    if (dht22_2.use == true) {
      dht22_2.hum   = dht2.readHumidity();        
      dht22_2.temp  = dht2.readTemperature();  
    }

    Serial.println("DHT22_1: Feuchtigkeit = " + String(dht22_1.hum) + " %");
    Serial.println("DHT22_1: Temperatur = " + String(dht22_1.temp) + " °C");

    Serial.println("DHT22_2: Feuchtigkeit = " + String(dht22_2.hum) + " %"); 
    Serial.println("DHT22_2: Temperatur = " + String(dht22_2.temp) + " °C");  

    delay(1000);
}
