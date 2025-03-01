#define BLYNK_TEMPLATE_ID "TMPL6hrUOrJb6"
#define BLYNK_TEMPLATE_NAME "Weather Station"
#define BLYNK_AUTH_TOKEN "CRpMYJIueVfoO3Dj9YRsES8y33fkRDUw"

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <BlynkSimpleEsp32.h>

// Replace the next variables with your SSID/Password combination
const char* ssid = "Yesyesyes";
const char* password = "AWEWEEWOO";

// Add your MQTT Broker IP address
const char* mqtt_server = "192.168.83.111";
const int mqtt_port = 1883;
const char *mqtt_user = "NULL";
const char *mqtt_password = "NULL";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

Adafruit_BME280 bme; // I2C
float temperature = 0;
float humidity = 0;

// Blynk Authentication Token
char auth[] = BLYNK_AUTH_TOKEN;

// LED Pin (using on-board LED connected to GPIO 2)
const int ledPin = 2;

void setup() {
  Serial.begin(115200);

  Blynk.begin(auth, ssid, password);
  
    if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.println("Connecting to WiFi");
  }

  Serial.println("WiFi connected");

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  delay(2000);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Connected to MQTT");
      delay(3000);
    }
  }
}

void loop() {
  Blynk.run();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  temperature = bme.readTemperature();

  char tempString[8];
  dtostrf(temperature, 1, 2, tempString);
  Serial.print("Temperature: ");
  Serial.println(tempString);
  client.publish("esp32/temperature", tempString);
  Blynk.virtualWrite(V1, temperature);

  humidity = bme.readHumidity();
  char humString[8];
  dtostrf(humidity, 1, 2, humString);
  Serial.print("Humidity: ");
  Serial.println(humString);
  client.publish("esp32/humidity", humString);
  Blynk.virtualWrite(V2, humidity);

  float pressure = bme.readPressure() / 100.0F;
  char pressureString[8];
  dtostrf(pressure, 1, 2, pressureString);
  Serial.print("Pressure: ");
  Serial.println(pressureString);
  client.publish("esp32/pressure", pressureString);
  Blynk.virtualWrite(V3, pressure);

  float seaLevelPressure = 1013.25;
  float altitude = bme.readAltitude(seaLevelPressure);
  char altitudeString[8];
  dtostrf(altitude, 1, 2, altitudeString);
  Serial.print("Altitude: ");
  Serial.println(altitudeString);
  client.publish("esp32/altitude", altitudeString);
  Blynk.virtualWrite(V4, altitude);

  if (temperature < 10 || temperature > 35) {
    // Send Blynk notification
    Blynk.logEvent("warning_temp", String("Temperature: ") + temperature);
  }
    if (humidity < 20 || humidity > 85) {
    // Send Blynk notification
    Blynk.logEvent("warning_hum", String("Humidity: ") + humidity);
  }
    if (pressure < 1010 || pressure > 1025) {
    // Send Blynk notification
    Blynk.logEvent("warning_pres", String("Pressure: ") + pressure);
  }
  // Blink the on-board LED after successfully publishing data
  digitalWrite(ledPin, HIGH);
  delay(500); // Delay for half a second
  digitalWrite(ledPin, LOW);
}

void callback(char *topic, byte *payload, unsigned int length) {
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      delay(3000);
    }
  }
}