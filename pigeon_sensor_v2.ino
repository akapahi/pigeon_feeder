
#include <WiFi.h>
#include "HX711.h"
#include <TinyStepper.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "WiFiClientSecure.h"
#include <WiFiMulti.h>

// Define Arduino Pin Outputs
#define IN1 27
#define IN2 14
#define IN3 12
#define IN4 13
#define HALFSTEPS 4096  // Number of half-steps for a full rotation

// Define the pins for the HX711 communication
const uint8_t DATA_PIN = 25;  // Can use any pins!
const uint8_t CLOCK_PIN = 26; // Can use any pins!
HX711 scale;

TinyStepper stepper(HALFSTEPS, IN1, IN2, IN3, IN4);

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "kapahiak"
#define AIO_KEY         "21ccec319da06ead06986564351ff9891009b843"

#define WLAN_SSID "Internet"
#define WLAN_PASS "connected"

WiFiClient client;
WiFiMulti wifiMulti;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup feeds 
Adafruit_MQTT_Publish pigeon = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pigeonsensors.p1");

unsigned long motorLockTime = 300000;
unsigned long motorTimer = 0;
bool motorLock = false;

void setup() {
  Serial.begin(115200);
  stepper.Enable();

  // Initialize the HX711
  scale.begin(DATA_PIN, CLOCK_PIN);

  // wait for serial port to connect. Needed for native USB port only
  while (!Serial) {
    delay(10);
  }
 

  scaleSetup();

  connectWifiMulti();
   
  motorTimer = millis();

} 

void loop() {

   while( Serial.available() > 0){
      int angle = Serial.parseInt();
      Serial.print(angle);
      stepper.Move(angle);
      delay(2000);
  }

  if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }

  Serial.print("\t| average:\t");
  float weight = scale.get_units(5);
  Serial.println(weight, 2);

  if(weight > 100){
    if(!motorLock){
      stepper.Move(-90);
      motorLock = true;
    }
    else{
      Serial.println(F("motor locked for: "));
      Serial.print((motorTimer+motorLockTime-millis())/1000);
      Serial.println(F(" seconds"));

    }

 if (! pigeon.publish(weight)){
       Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
    }

  }

  if(millis() - motorTimer > motorLockTime){
    motorLock = false;
    motorTimer = millis();
  }

  delay(2000);
}

void scaleSetup(){
  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale.read());      // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));   // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
            // by the SCALE parameter (not set yet)
            
  scale.set_scale(-706.356);
  //scale.set_scale(-471.497);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();               // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale.read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
            // by the SCALE parameter set with set_scale

  Serial.println("Readings:");
}

void connectWifi(){
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  delay(2000);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  connect();
}

void connectWifiMulti(){
  wifiMulti.addAP("Internet","connected");
  wifiMulti.addAP("Param06","^H*8gde!$Ju%");
  wifiMulti.addAP("Param03","^H*8gde!$Ju%");
  wifiMulti.addAP("Param07","^H*8gde!$Ju%");
  wifiMulti.addAP("Param014","^H*8gde!$Ju%");
  wifiMulti.addAP("Param011","^H*8gde!$Ju%");
  wifiMulti.addAP("Param008","^H*8gde!$Ju%");

  Serial.println("Connecting Wifi multi...");

  if (wifiMulti.run() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
  connect();
}

// connect to adafruit io via MQTT
void connect() {

  if (wifiMulti.run() != WL_CONNECTED) {
    connectWifiMulti();
  }else{

    Serial.print(F("Connecting to Adafruit IO... "));

    int8_t ret;

    while ((ret = mqtt.connect()) != 0) {

      switch (ret) {
        case 1: Serial.println(F("Wrong protocol")); break;
        case 2: Serial.println(F("ID rejected")); break;
        case 3: Serial.println(F("Server unavail")); break;
        case 4: Serial.println(F("Bad user/pass")); break;
        case 5: Serial.println(F("Not authed")); break;
        case 6: Serial.println(F("Failed to subscribe")); break;
        default: Serial.println(F("Connection failed")); break;
      }

      if(ret >= 0)
        mqtt.disconnect();

      Serial.println(F("Retrying connection..."));
      delay(5000);

    }

    Serial.println(F("Adafruit IO Connected!"));
  }
}