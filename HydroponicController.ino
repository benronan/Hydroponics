
#include <WifiApServer.h>
#include <OTAService.h>
#include <ESP8266mDNS.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BME280.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>


// Adafruit IO variables
#define AIO_USERNAME  "ENTER_USERNAME"
#define AIO_KEY       "ENTER_KEY_HERE"
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish waterTemperatureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hydroponic-water-temp");
Adafruit_MQTT_Publish ambientTemperatureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hydroponic-ambient-temp");
Adafruit_MQTT_Publish ambientHumidityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hydroponic-ambient-humidity");
long sendInterval = 60*1000;
long lastSendTime = -sendInterval;

//global variables
String BSSID = "SSID_YOU_WANT_FOR_WEMOS";
String OTA_PASSWORD = "PASSWORD_FOR_OTA_FLASHINIG";

//array of wifi networks to try to connect to when starting up
String Networks[][2] = {
	{"WIFI_NETWORK1", "WIFI_NETWORK1_PASSWORD"},
	{"WIFI_NETWORK2","WIFI_NETWORK2_PASSWORD"}
};

WifiApServer *server;
OneWire oneWire(2);
DallasTemperature sensors(&oneWire);
DeviceAddress waterSensorAddress;
TBME280 weather;

void setup() {
  Serial.begin(115200);  
  pinMode(BUILTIN_LED,OUTPUT);
  
  InitServer();
  
  Serial.println("Starting OTA Service");
  OTAService.begin(8266,BSSID,OTA_PASSWORD);
  Serial.println("OTA Service running");
  
  //setup mdns responder
  Serial.println("Starting mDNS");
  if (!MDNS.begin(BSSID.c_str())) {
    Serial.println("Error setting up MDNS responder!");
  } else {
	  //mdns to let devices know we have a http server
	  //MDNS.addService("8266", "tcp", 80);
    MDNS.addServiceTxt("8266", "http", "info", "/");
  }
  
  digitalWrite(BUILTIN_LED,LOW);
  InitSensors();
}

void loop() {
  server->HandleClient();
  yield();
  OTAService.handle();
  yield();
  MQTT_connect();
  if (mqtt.connected() && (millis() - lastSendTime > sendInterval)) {
    SendDataToCloud();
    lastSendTime = millis();
  }
  yield();
}

void InitSensors() {
  Serial.print("Initializing sensors");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) 
	Serial.println("ON");
  else 
	  Serial.println("OFF");
  
  if (!sensors.getAddress(waterSensorAddress, 0)) 
	  Serial.println("Unable to find address for Device 0"); 
  
  Serial.print("Device 0 Address: ");  
  for (uint8_t i = 0; i < 8; i++) {
    if (waterSensorAddress[i] < 16) Serial.print("0");
    Serial.print(waterSensorAddress[i], HEX);
  }
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(waterSensorAddress, 9);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(waterSensorAddress), DEC); 
  Serial.println();
  
  sensors.begin();

  Serial.println("Initializing BME280");
  weather.Initialize();
  Serial.println("Initializing sensors complete");
}

void InitServer() {
  Serial.println("InitServer()");
  server =  new WifiApServer();
  Serial.println("registering paths");
  server->RegisterPath("/",HandleRequest);

  // connect to network
  Serial.println("connecting to network");
  int i=0;
  int iMax = sizeof Networks / sizeof Networks[0];
  bool connected = false;
  String ssid,psk;
  while(!connected) {
    ssid= Networks[i][0];
    psk= Networks[i][1];
    Serial.println("connecting to \"" + ssid + "\" with psk \"" + psk + "\"");
    connected = server->Connect(ssid,psk,BSSID);
    if(!connected) {
      Serial.println("Failed to connect to " + ssid);
      if(++i >= iMax) {
        Serial.println("Restarting");
        ESP.restart();
      }
    }
  }
  Serial.println("Connected to \"" + ssid + "\" with psk \"" + psk + "\"");
  Serial.println("IPAddress: " + ipToString(WiFi.localIP()));
  Serial.println("InitServer() complete");
}

void HandleRequest() {
  sensors.requestTemperatures();
  float waterTemperature = DallasTemperature::toFahrenheit(sensors.getTempC(waterSensorAddress));
  weather.Read();
  double at = weather.Temperature();
  double ap = weather.Pressure();
  double ah = weather.Humidity();
  
  String response = "{";
  response += "\"WaterTemperature\":\""+String(waterTemperature)+"\"";
  response += ",\"AmbientReadings\":";
  response += "[";
  response += "{\"Temperature\":\""+String(at)+"\"},";
  response += "{\"Humidity\":\""+String(ah)+"\"},";
  response += "{\"Pressure\":\""+String(ap)+"\"}";
  response += "]";
  response += "}";
  server->Send(200, "application/json", response);
  Serial.println();
  Serial.println();
  Serial.println(response);
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
          break;
       }
  }
  Serial.println("MQTT Connected!");
}

void SendDataToCloud(){
  Serial.println("SendDataToCloud");
  sensors.requestTemperatures();
  float waterTemperature = DallasTemperature::toFahrenheit(sensors.getTempC(waterSensorAddress));
  weather.Read();
  double at = weather.Temperature();
  double ah = weather.Humidity();
  
  if (!waterTemperatureFeed.publish(waterTemperature)) {
    Serial.println("waterTemperatureFeed failed");
  }
  if (!ambientTemperatureFeed.publish(at)) {
    Serial.println("ambientTemperatureFeed failed");
  }
  if (!ambientHumidityFeed.publish(ah)) {
    Serial.println("ambientHumidityFeed failed");
  }
  Serial.println("SendDataToCloud complete");
}

