/*
  ESP8266 BlinkWithoutDelay by Simon Peter
  Blink the blue LED on the ESP-01 module
  Based on the Arduino Blink without Delay example
  This example code is in the public domain

  The blue LED on the ESP-01 module is connected to GPIO1
  (which is also the TXD pin; so we cannot use Serial.print() at the same time)

  Note that this sketch uses LED_BUILTIN to find the pin with the internal LED
*/

#include <Wire.h>
#include "Adafruit_ADS1015.h"
#include "ESP8266WiFi.h"
#include "PubSubClient.h"
#include <ESP8266WiFiMulti.h>
/*********************************************************************************************\
 * Define ADC I2C SCL/D1   SDA/D2
\*********************************************************************************************/
char ADC_1_solar_voltage[50];
char ADC_2_reserve[50];
char ADC_3_mousture_sensor[50];
char ADC_0_battery_volate[50];
char pump_sensor[50];
const int sleepTimeS = 10;
/*********************************************************************************************\
 * Define limit
\*********************************************************************************************/
int limit_solar     = 4500;
int limit_moisture  = 4000;

String pump_str;
String adc0_str;
String adc1_str;
String adc2_str;
String adc3_str;
Adafruit_ADS1115 ads1115(0x48); // construct an ads1115 at address 0x49
bool debugging_ADC=true;

/*********************************************************************************************\
 * Define Relay
\*********************************************************************************************/
int ledState = LOW;//for testing
int RelayState=LOW;
int relay_pin = 2; // D4 Node MCU

/*********************************************************************************************\
 * Define Wifi
\*********************************************************************************************/
ESP8266WiFiMulti wifiMulti;






const char* ssid = "Pink1";
const char* password = "abcdefghiJklM8684";
//alternate Wifi
const char* ssid1 = "Pink1";
const char* password1 = "abcdefghiJklM8684";

const char* mqtt_server = "192.168.0.70";
const char* mqtt_user ="username";
const char* mqtt_password ="password";



WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

/*********************************************************************************************\
 * Define Delays
\*********************************************************************************************/
unsigned long previousMillis = 0;
const long interval = 1000;//Contoler Loop time

unsigned long previousMillis_wifi = 0;
const long interval_wifi = 1000*6*5;//default 1000*60*5;//WIFI interval min 60000*5 = 5min

unsigned long previousMillisKeep_alive = 0;
const long interval_Keep_alive = 100;

int delay_wifi=600; //in s
/*********************************************************************************************\
 * I2C Scanner for Scanning I2C Bus
\*********************************************************************************************/
/*void I2C_SCANNER(){
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan}
}
*/

/*********************************************************************************************\
 * MQTT SETUP
\*********************************************************************************************/


void setup() {
  Wire.begin();
  Serial.begin(115200);

    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED,LOW);
  /*********************************************************************************************\
   * PUMP SETUP
  \*********************************************************************************************/


  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, HIGH);//PUMP on
  delay(1000);  Serial.print("SETUP_PUMP_on_Start_Setup");
  digitalWrite(relay_pin, LOW);//PUMP off
  delay(1000);  Serial.print("SETUP_PUMP_off");
  digitalWrite(relay_pin, HIGH);//PUMP on
  delay(1000);  Serial.print("SETUP_PUMP_on");
  digitalWrite(relay_pin, LOW);//PUMP off
  delay(1000);  Serial.print("SETUP_PUMP_off_Setup_done");


  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
  //I2C_SCANNER();
  Serial.println("ADC ON");
  ads1115.begin();
  //Wifi SETUP


  setup_wifi();
//wifiMulti.addAP("...franken.freifunk.net", "");
  wifiMulti.addAP("Pink", "abcdefghiJklM8684");
  wifiMulti.addAP("Pink", "abcdefghiJklM8684");
  wifiMulti.addAP("MrPink", "abcdefghiJklM8684");

  Serial.println("Connecting Wifi...");
      if(wifiMulti.run() == WL_CONNECTED) {
          Serial.println("");
          Serial.println("WiFi connected");
          Serial.println("IP address: ");
          Serial.println(WiFi.localIP());
      }


  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);




}
void setup_wifi() {


    /*  wifiMulti.addAP("Pink1", "your_password_for_AP_1");
      wifiMulti.addAP("Pink2", "your_password_for_AP_2");
      wifiMulti.addAP("MrPink", "abcdefghiJklM8684");

      Serial.println("Connecting Wifi...");
          if(wifiMulti.run() == WL_CONNECTED) {
              Serial.println("");
              Serial.println("WiFi connected");
              Serial.println("IP address: ");
              Serial.println(WiFi.localIP());
          }



  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.println(ssid);
while(WiFi.status() != WL_CONNECTED){
for (int i=0;((WiFi.status() == WL_CONNECTED)||(i<=2));i++){
  Serial.print(".");
  delay(500);
}
  /*
  while (WiFi.status() != WL_CONNECTED) {
          Serial.print("Connecting to ");
            Serial.println(ssid);
              for (int i=0;i<=10;i++)
              {
                WiFi.begin(ssid, password);
                delay(500);
                Serial.print(".");
              }

            Serial.print("Connecting to ");
            Serial.println(ssid1);
            WiFi.disconnect();
            delay(500);
              for (int i=0;i<=25;i++)
              {
                WiFi.begin(ssid1, password1);
                delay(500);
                Serial.print(".");
                //Serial.print(ssid1);
                //Serial.print(password1);
              }
//

  if((WiFi.status() != WL_CONNECTED)){
    Serial.print("Connecting to ");
    Serial.println(ssid1);
    WiFi.disconnect();

    delay(500);
    WiFi.begin(ssid1, password1);
    for(int i=0;((WiFi.status()==WL_CONNECTED)||(i<=20));i++){
      Serial.print(".");
        Serial.print("x");
        Serial.print(i);
        Serial.print("wifi");
        Serial.print(WiFi.status());
      delay(500);
  }

  }
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  */
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  /* Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, HIGH);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, LOW);  // Turn the LED off by making the voltage HIGH
  }
  */

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "Garden is Online");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop(){
  int16_t adc0, adc1, adc2, adc3;
  adc0 = ads1115.readADC_SingleEnded(0);
  adc1 = ads1115.readADC_SingleEnded(1);
  adc2 = ads1115.readADC_SingleEnded(2);
  adc3 = ads1115.readADC_SingleEnded(3);
float adc0_cal=adc0/2670.476;

if(debugging_ADC){
  Serial.print("AIN0: "); Serial.print(adc0_cal);//battery
  Serial.print("  AIN1: "); Serial.print(adc1);//solar
  Serial.print("  AIN2: "); Serial.print(adc2);//reserve
  Serial.print("  AIN3: "); Serial.print(adc3);//mousiture
  Serial.print(" ");
}
unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;


    unsigned long currentMillis_wifi = millis();
      if (currentMillis_wifi - previousMillis_wifi >= interval_wifi) {
        previousMillis_wifi = currentMillis_wifi;
        Serial.print("Wifi_on");
        relay_control (adc1,adc3,limit_moisture,limit_solar);


        WiFi.forceSleepWake();
        delay(3000);
        for(int i=0;i<2;i++) {
          relay_control (adc1,adc3,limit_moisture,limit_solar);

        WiFi.forceSleepWake();
        delay(3000);
        //WIFI
        if(wifiMulti.run() != WL_CONNECTED) {
               Serial.println("WiFi not connected!");
               delay(1000);
           }


        if (!client.connected()&&wifiMulti.run()==WL_CONNECTED) {
          relay_control (adc1,adc3,limit_moisture,limit_solar);
            reconnect();
            delay(2000);
            Serial.print("reconnect");
          }
           client.loop();
          delay(2000);
      /*  Serial.print("Control_on");

          Serial.print("Wifi_Control_loop:");
          Serial.print(i);
          if ((adc3<=limit_moisture)&&(adc1>=limit_solar)) {
          RelayState = HIGH;
          Serial.print("    RELAY_HIGH");}  // Note that this switches the LED *on*
          else {
          RelayState = LOW;
          Serial.print("RELAY_LOW");}
          digitalWrite(relay_pin, RelayState);
          Serial.println(" ");
          //const char* mqtt_msg = (const char*) adc3;
          /*const char* ADC_1_solar_voltage=(const char*) adc1;
          const char* ADC_2_reserve=(const char*) adc2;
          const char* ADC_3_mousture_sensor=(const char*) adc3;
          const char* ADC_0_battery_volate=(const char*) adc0;;
          */

          /*//MQTT msg
          Serial.print("Publish message: ");
          // Serial.print(msg);
          client.publish("outTopic_MG", mqtt_msg);
          */


          /*String adc_msg2=(String)adc2;
          String  adc_msg3=(String)adc3;
          String  adc_msg0=(String)adc0;
          */


              Serial.print("Publish message: ");
              Serial.println(ADC_1_solar_voltage);
              //Serial.print (adc_msg1);

              adc0_str = String(adc0_cal); //converting ftemp (the float variable above) to a string
              adc0_str.toCharArray(ADC_0_battery_volate, adc0_str.length() + 1); //packaging up the data to publish to mqtt whoa...

              adc1_str = String(adc1); //converting ftemp (the float variable above) to a string
              adc1_str.toCharArray(ADC_1_solar_voltage, adc1_str.length() + 1); //packaging up the data to publish to mqtt whoa...
          /*
              adc2_str = String(adc2); //converting ftemp (the float variable above) to a string
              adc2_str.toCharArray(ADC_2_reserve; adc2_str.length() + 1); //packaging up the data to publish to mqtt whoa...
          */
              adc3_str = String(adc3); //converting ftemp (the float variable above) to a string
              adc3_str.toCharArray(ADC_3_mousture_sensor, adc3_str.length() + 1); //packaging up the data to publish to mqtt whoa...

              pump_str = String(RelayState); //converting ftemp (the float variable above) to a string
              pump_str.toCharArray(pump_sensor, pump_str.length() + 1);

              client.publish("solar",ADC_1_solar_voltage);
              client.publish("moisture",ADC_3_mousture_sensor);
              client.publish("battery_volt",ADC_0_battery_volate);
              client.publish("pump",pump_sensor);

        }
}

   /* if (RelayState == LOW)
      RelayState = HIGH;  // Note that this switches the LED *off*
    else
      RelayState = LOW;   // Note that this switches the LED *on*
    digitalWrite(RELAY, RelayState);
   */
relay_control (adc1,adc3,limit_moisture,limit_solar);

/*
Serial.print("Control_on");
//if ((adc3<=12000)&&(adc1>=10000))
//if ((adc3<=limit_moisture)&&(adc1>=limit_solar)){
if (adc3<=limit_moisture){
RelayState = HIGH;
Serial.print("RELAY_HIGH");}  // Note that this switches the LED *on*
else {
RelayState = LOW;
Serial.print("RELAY_LOW");}
digitalWrite(relay_pin, RelayState);
Serial.println(" ");
//const char* mqtt_msg = (const char*) adc3;
/*const char* ADC_1_solar_voltage=(const char*) adc1;
const char* ADC_2_reserve=(const char*) adc2;
const char* ADC_3_mousture_sensor=(const char*) adc3;
const char* ADC_0_battery_volate=(const char*) adc0;;
*/

/*//MQTT msg
Serial.print("Publish message: ");
// Serial.print(msg);
client.publish("outTopic_MG", mqtt_msg);
*/


/*String adc_msg2=(String)adc2;
String  adc_msg3=(String)adc3;
String  adc_msg0=(String)adc0;
*/

/*
    Serial.print("Publish message: ");
    Serial.println(ADC_1_solar_voltage);
    //Serial.print (adc_msg1);

    adc0_str = String(adc0_cal); //converting ftemp (the float variable above) to a string
    adc0_str.toCharArray(ADC_0_battery_volate, adc0_str.length() + 1); //packaging up the data to publish to mqtt whoa...

    adc1_str = String(adc1); //converting ftemp (the float variable above) to a string
    adc1_str.toCharArray(ADC_1_solar_voltage, adc1_str.length() + 1); //packaging up the data to publish to mqtt whoa...
/*
    adc2_str = String(adc2); //converting ftemp (the float variable above) to a string
     adc2_str.toCharArray(ADC_2_reserve; adc2_str.length() + 1); //packaging up the data to publish to mqtt whoa...

    adc3_str = String(adc3); //converting ftemp (the float variable above) to a string
    adc3_str.toCharArray(ADC_3_mousture_sensor, adc3_str.length() + 1); //packaging up the data to publish to mqtt whoa...

    pump_str = String(RelayState); //converting ftemp (the float variable above) to a string
    pump_str.toCharArray(pump_sensor, pump_str.length() + 1);

    client.publish("solar",ADC_1_solar_voltage);
    client.publish("moisture",ADC_3_mousture_sensor);
    client.publish("battery_volt",ADC_0_battery_volate);
    client.publish("pump",pump_sensor);
*/

}
WiFi.forceSleepBegin();
Serial.println("Wifi_off");
delay(1000);
// Sleep
//  Serial.println("ESP8266 in sleep mode");
//  ESP.deepSleep(sleepTimeS * 1000000);
}

void relay_control(int adc1,int adc3,int limit_moisture,int limit_solar){
                  Serial.print("Control_ON:");
                  //if ((adc3<=12000)&&(adc1>=10000))
                  //if ((adc3<=limit_moisture)&&(adc1>=limit_solar)){


                  if ((adc3<=limit_moisture)&&(adc1>=limit_solar)){
                  RelayState = HIGH;
                  Serial.print("RELAY_HIGH");
                  digitalWrite(relay_pin, RelayState);}  // Note that this switches the LED *on*
                          else {
                          RelayState =LOW ;
                          Serial.print("RELAY_LOW");
                          digitalWrite(relay_pin, RelayState);
                        }
                  Serial.println(" ");

}
