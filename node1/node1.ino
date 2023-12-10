#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_HTU21DF.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiManager.h>
#include <WiFiClient.h> 
#include <WiFi.h>

#define LIGHT_SENSOR_PIN 34 //LDR sensor pin
#define LED_WIFI 4

#define TRIGPIN    5  
#define ECHOPIN    18 

#define SOUND_SPEED 0.034
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP280 bmp;
char ssid[40] ="";  // type your wifi name
char pass[40] ="";  // type your wifi password


//Set Water Level Distance in CM
int emptyTankDistance = 120 ;  //Distance when tank is empty
int fullTankDistance =  35 ;  //Distance when tank is full

//Set trigger value in percentage
int Low_triggerPer =   10 ;  //alarm will start when water level drop below triggerPer
int High_triggerPer =  95 ;  //alarm will start when water level exceed upper triggerPer

// long duration;
float distance;
int   waterLevelPer;


bool timeouted = false;
void connectWiFi(){
  WiFiManager wm;
  wm.resetSettings();
  wm.setConfigPortalTimeout(120); // set connection timeout to 2 minutes
  bool res = wm.autoConnect("AutoconnectAP",""); // password protected aphttp://192.168.4.1/

  if (res) {
    digitalWrite(LED_WIFI, HIGH);
    Serial.println("Connected to WiFi");
    delay(1000);
    return; // exit the function on successful connection
  } 
  else {
    Serial.println("Failed to connect to WiFi");
    timeouted = true;
    delay(2);
    return; // exit the function on failure
  }
}


double calcLightIntensity(int AnVal, int Resistor){
  float lux = (AnVal / 4095.0) * 100.0;
  return lux;
}



void measureDistance(){
  // Set the trigger pin LOW for 2uS
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
 
  // Set the trigger pin HIGH for 20us to send pulse
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(20);
 
  // Return the trigger pin to LOW
  digitalWrite(TRIGPIN, LOW);
 
  int distance = pulseIn(ECHOPIN,HIGH,26000);
  distance = distance/58;
  Serial.print("Distance: ");
  Serial.println(distance);  
  
  waterLevelPer = map((int)distance ,emptyTankDistance, fullTankDistance, 0, 100);
  
  Serial.println(waterLevelPer);
  
  // Delay before repeating measurement
  delay(100);
}

void setup() {
  Serial.begin(115200);

  //Set pinmodes for sensor connections
  pinMode(ECHOPIN, INPUT_PULLUP);
  pinMode(TRIGPIN, OUTPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(LED_WIFI,OUTPUT);

  digitalWrite(LED_WIFI, LOW);

  //Wire.begin();
  Wire.begin();
  unsigned status;
  status = bmp.begin(0x76);
  if (!status) {
    // connection error or device address wrong!
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    
    while (1) // stay here
      delay(1000);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  
}

void loop() {

  if (WiFi.status() != WL_CONNECTED && timeouted == false) {
    connectWiFi();
  }

  
  
  float temp_bmp280 = bmp.readTemperature();    // get temperature in degree Celsius
  float pres = bmp.readPressure()/100;       // get pressure in Pa

  int analogVal = analogRead(LIGHT_SENSOR_PIN); //read values from D15
  int Resistance= 10; //I am using a 10k ohms resistor
  int luxval=calcLightIntensity(analogVal, Resistance );
  
  delay(1000);  // wait a second

  Serial.print("Temperature1 = ");
  Serial.print(temp_bmp280);
  Serial.println("*C");
  

  Serial.print("Pressure = ");
  Serial.print(pres);
  Serial.println("hPa");
 
  Serial.print("Light Intensity:");
  Serial.print(luxval);
  Serial.println(" lux");

  measureDistance();
    
}

