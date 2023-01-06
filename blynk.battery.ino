#define BLYNK_TEMPLATE_ID "TMPL_zYMQixC"
#define BLYNK_DEVICE_NAME "battery management"
#define BLYNK_AUTH_TOKEN "_R5in8SKLf9so5wcopKkJQGXtyRhaeRX"
#define BLYNK_FIRMWARE_VERSION "0.1.0"
#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "username";
char pass[] = "password";

// Uncomment your board, or configure a custom board in Settings.h
//#define USE_SPARKFUN_BLYNK_BOARD
#define USE_NODE_MCU_BOARD
//#define USE_WITTY_CLOUD_BOARD
//#define USE_WEMOS_D1_MINI

#include<ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#include <DHT.h>
       // Reset pin # (or -1 if sharing Arduino reset pin)

#define DHTTYPE DHT11    // DHT 22
#define DHTPIN D2 //DHT22 Pin D4(GPIO 2)


DHT dht(DHTPIN, DHTTYPE);

float voltage;
int bat_percentage;
int analogInPin  = A0;    // Analog input pin
int sensorValue;
float calibration = 0.40; // Check Battery voltage using multimeter & add/subtract the value



void setup()
{
  Serial.begin(115200);
  delay(100);
  Blynk.begin(auth,ssid,pass);
  dht.begin();
  delay(100);

}

void loop() {
  Blynk.run();
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  sensorValue = analogRead(analogInPin);
  voltage = (((sensorValue * 3.3) / 1024) * 2 - calibration); //multiply by two as voltage divider network is 100K & 100K Resistor

  bat_percentage = mapfloat(voltage, 2.8, 4.2, 0, 100); //2.8V as Battery Cut off Voltage & 4.2V as Maximum Voltage

  if (bat_percentage >= 100)
  {
    bat_percentage = 100;
  }
  if (bat_percentage <= 0)
  {
    bat_percentage = 1;
  }
  
      //send data to blynk
    Blynk.virtualWrite(V1, t); //for Temperature
    Blynk.virtualWrite(V3, voltage);  // for battery voltage
    Blynk.virtualWrite(V4, bat_percentage);  // for battery percentage
  
    //Print data on serial monitor
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" *C");
  Serial.print("Analog Value = ");
  Serial.println(sensorValue);
  Serial.print("Output Voltage = ");
  Serial.println(voltage);
  Serial.print("Battery Percentage = ");
  Serial.println(bat_percentage);

  Serial.println();
  Serial.println("****");
  Serial.println();
  delay(1000);
  if (bat_percentage <=30)
  {
    Serial.println("Battery level below 30%, Charge battery on time");
    Blynk.logEvent("battery_low", "Battery is getting low.... Plugin to charge") ;
    delay(500);
  }
  if (bat_percentage >= 95) 
  { 
    Serial.println("battery percent is more than 95%, Turn off the switch on time"); 
    Blynk.logEvent("battery_high", "Battery is getting full charge... Turn off on time"); 
    delay(500); 
  } 
  if (t > 25) 
  {
    Serial.println("Temperature of the battery is getting higher than 40 degree celsius"); 
    Blynk.logEvent("Temp_high", "Temperature of the battery is getting higher than 40 degree celsius"); 
    delay(500); 
  }
  
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x * out_max)/4.2;
}
