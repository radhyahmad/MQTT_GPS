#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>

struct gps_data
{
  double lat = 0.0F;
  double lng = 0.0F;
};

struct sensor_data
{
  float temperature = 0.0F;
  float humidity = 0.0F;
  float pressure = 0.0F;
};

void initialize_sensor();
void initialize_wifi();
boolean reconnect();
void update_data();

const char ssid[] = "ceritechIoT";
const char password[] = "ceritechkopi";

WiFiClient net;
PubSubClient mqttClient(net);

const char mqtt_broker[] = "w7b0b774.en.emqx.cloud";
const char publish_topic[] = "v1/devices/me/telemetry";

#define CLIENT_ID "mqttx_c2035c2a"
#define USERNAME "ceri_12345"
#define PASSWORD "CeriTech12345"

long lastReconnectAttempt = 0;

TinyGPSPlus gps;
Adafruit_BME280 bme;

static gps_data dataGps;
static sensor_data dataSensor;
static char payload[256];
StaticJsonDocument<256>doc;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial1.begin(9600);
  initialize_sensor();
  initialize_wifi();
  mqttClient.setServer(mqtt_broker, 12143); 
  lastReconnectAttempt = 0;
}

void loop() {
  if (WiFi.status() != WL_CONNECTED)
  {
    initialize_wifi();
  }

  if (!mqttClient.connected())
  {
    long now = millis();

    if (now - lastReconnectAttempt > 5000)
    {
      lastReconnectAttempt = now;

      if (reconnect())
      {
        lastReconnectAttempt = 0;
      }
      
    }
    
  } else
  {
    mqttClient.loop();
  }

  update_data();
}

void initialize_sensor()
{
  Wire.begin();
  unsigned status;

  status = bme.begin();

  if (!status)
  {
    Serial.println("Could not find a valid BME280.");
    while(1) delay(10);
  }
  
}

void initialize_wifi(){
  delay(100);
  WiFi.disconnect();
  Serial.println();
  Serial.print("Firmware version: ");
  Serial.println(WiFi.firmwareVersion());
  Serial.print("Connecting WiFi to: ");
  Serial.println(ssid);

  while(WiFi.status() != WL_CONNECTED)
  {
    WiFi.begin(ssid, password);
    Serial.print("Attempting WiFi connection .....");
    
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Serial.println("\nWiFi RSSI: ");
      Serial.println(WiFi.RSSI());
      digitalWrite(LED_BUILTIN, HIGH);
    }

    else
    {
      Serial.print("Failed to connect to WiFi");
      Serial.println(", Try again in 5 seconds");
      digitalWrite(LED_BUILTIN, LOW);
      delay(5000);
    }
    
  }
  
}

boolean reconnect(){
  Serial.println("Attempting to connect MQTT");

  if (mqttClient.connect(CLIENT_ID, USERNAME, PASSWORD))
  {
    Serial.println("Connected to MQTT broker");
  }

  return mqttClient.connected();
}

void update_data(){

  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure()/100.0F;

  dataSensor.temperature = t;
  dataSensor.humidity = h;
  dataSensor.pressure = p;
  
  while(Serial1.available() > 0){

    if(gps.encode(Serial1.read())){
  
      if(gps.location.isValid())
      {
        double latitude  = gps.location.lat();
        double longitude = gps.location.lng();

        dataGps.lat = latitude;
        dataGps.lng = longitude;

        doc["Latitute"] = dataGps.lat;
        doc["Longitude"] = dataGps.lng;
        doc["Temperature"] = dataSensor.temperature;
        doc["Humidity"] = dataSensor.humidity;
        doc["Pressure"] = dataSensor.pressure;

        serializeJsonPretty(doc, payload);
        mqttClient.publish(publish_topic, payload);
        Serial.println(payload);
        //delay(5000);
      }

      else
      {
        Serial.println(F("INVALID "));
      }
    }
  }
  
  if(millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected: check wiring:");
    while(true);
  }
}
