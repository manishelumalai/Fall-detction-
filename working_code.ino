#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// WiFi credentials
const char* ssid = "realme 9 Pro 5G";
const char* password = "manish32";

// HiveMQ Cloud Broker details
const char* mqttServer = "6045e79d076a4ac5970c933c07e298f2.s1.eu.hivemq.cloud";
const int mqttPort = 8883;
const char* mqttUser = "Manish";
const char* mqttPassword = "Manish@004";

// MQTT topics
const char* mqttTopic = "mpu/acceleration";
const char* mqttPublishTopic = "mpu/falldetect";

// CA certificate
static const char* ca_cert  = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFBTCCAu2gAwIBAgIQS6hSk/eaL6JzBkuoBI110DANBgkqhkiG9w0BAQsFADBP
MQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJuZXQgU2VjdXJpdHkgUmVzZWFy
Y2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBYMTAeFw0yNDAzMTMwMDAwMDBa
Fw0yNzAzMTIyMzU5NTlaMDMxCzAJBgNVBAYTAlVTMRYwFAYDVQQKEw1MZXQncyBF
bmNyeXB0MQwwCgYDVQQDEwNSMTAwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEK
AoIBAQDPV+XmxFQS7bRH/sknWHZGUCiMHT6I3wWd1bUYKb3dtVq/+vbOo76vACFL
YlpaPAEvxVgD9on/jhFD68G14BQHlo9vH9fnuoE5CXVlt8KvGFs3Jijno/QHK20a
/6tYvJWuQP/py1fEtVt/eA0YYbwX51TGu0mRzW4Y0YCF7qZlNrx06rxQTOr8IfM4
FpOUurDTazgGzRYSespSdcitdrLCnF2YRVxvYXvGLe48E1KGAdlX5jgc3421H5KR
mudKHMxFqHJV8LDmowfs/acbZp4/SItxhHFYyTr6717yW0QrPHTnj7JHwQdqzZq3
DZb3EoEmUVQK7GH29/Xi8orIlQ2NAgMBAAGjgfgwgfUwDgYDVR0PAQH/BAQDAgGG
MB0GA1UdJQQWMBQGCCsGAQUFBwMCBggrBgEFBQcDATASBgNVHRMBAf8ECDAGAQH/
AgEAMB0GA1UdDgQWBBS7vMNHpeS8qcbDpHIMEI2iNeHI6DAfBgNVHSMEGDAWgBR5
tFnme7bl5AFzgAiIyBpY9umbbjAyBggrBgEFBQcBAQQmMCQwIgYIKwYBBQUHMAKG
Fmh0dHA6Ly94MS5pLmxlbmNyLm9yZy8wEwYDVR0gBAwwCjAIBgZngQwBAgEwJwYD
VR0fBCAwHjAcoBqgGIYWaHR0cDovL3gxLmMubGVuY3Iub3JnLzANBgkqhkiG9w0B
AQsFAAOCAgEAkrHnQTfreZ2B5s3iJeE6IOmQRJWjgVzPw139vaBw1bGWKCIL0vIo
zwzn1OZDjCQiHcFCktEJr59L9MhwTyAWsVrdAfYf+B9haxQnsHKNY67u4s5Lzzfd
u6PUzeetUK29v+PsPmI2cJkxp+iN3epi4hKu9ZzUPSwMqtCceb7qPVxEbpYxY1p9
1n5PJKBLBX9eb9LU6l8zSxPWV7bK3lG4XaMJgnT9x3ies7msFtpKK5bDtotij/l0
GaKeA97pb5uwD9KgWvaFXMIEt8jVTjLEvwRdvCn294GPDF08U8lAkIv7tghluaQh
1QnlE4SEN4LOECj8dsIGJXpGUk3aU3KkJz9icKy+aUgA+2cP21uh6NcDIS3XyfaZ
QjmDQ993ChII8SXWupQZVBiIpcWO4RqZk3lr7Bz5MUCwzDIA359e57SSq5CCkY0N
4B6Vulk7LktfwrdGNVI5BsC9qqxSwSKgRJeZ9wygIaehbHFHFhcBaMDKpiZlBHyz
rsnnlFXCb5s8HKn5LsUgGvB24L7sGNZP2CX7dhHov+YhD+jozLW2p9W4959Bz2Ei
RmqDtmiXLnzqTpXbI+suyCsohKRg6Un0RC47+cpiVwHiXZAW+cn8eiNIjqbVgXLx
KPpdzvvtTnOPlC7SQZSYmdunr3Bf9b77AiC/ZidstK36dRILKz7OA54=
-----END CERTIFICATE-----
)EOF";

// Create WiFiClientSecure for SSL
WiFiClientSecure espClient;
PubSubClient client(espClient);

Adafruit_MPU6050 mpu;

// Button pin and LED for fall detection
const int buttonPin = 14;
const int ledPin = 13;
bool fallDetected = false;
unsigned long fallTime = 0;
unsigned long fallTimeout = 30000; // 30 seconds
int B

// Callback function for MQTT messages
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  // Initialize button and LED
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Set CA certificate
  espClient.setCACert(ca_cert);

  // Set MQTT server and credentials
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  // Connect to the MQTT Broker
  if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
    Serial.println("Connected to HiveMQ Cloud broker");
    client.subscribe(mqttTopic);
  } else {
    Serial.print("Failed to connect, state: ");
    Serial.println(client.state());
  }

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set up motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
}

void loop() {
  // Ensure MQTT connection remains alive
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Get sensor data
  if (mpu.getMotionInterruptStatus()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Publish acceleration data to MQTT
    String payload = "AccelX: " + String(a.acceleration.x) + ", AccelY: " + String(a.acceleration.y) + ", AccelZ: " + String(a.acceleration.z);
    client.publish(mqttPublishTopic, payload.c_str());

    Serial.println(payload);

    // Detect fall (based on acceleration)
    if (a.acceleration.x > 15 || a.acceleration.y > 15 || a.acceleration.z > 15) {
      Serial.println("Fall detected!");
      fallDetected = true;
      fallTime = millis();
      digitalWrite(ledPin, HIGH); // Turn on LED
    }
  }

  // Check if fall has been detected and if button is pressed within 30 seconds
  if (fallDetected) {
    if (digitalRead(buttonPin) == LOW) {
      Serial.println("Button pressed. Continuing data transmission.");
      digitalWrite(ledPin, LOW); // Turn off LED
      fallDetected = false;
    } else if (millis() - fallTime > fallTimeout) {
      Serial.println("No response. Sending SMS alert via Twilio.");
      sendSmsAlert(); // Call the function to send an alert
      fallDetected = false;
    }
  }
}

// Function to reconnect to MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("connected");
      client.subscribe(mqttTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Function to send SMS alert via Twilio
void sendSmsAlert() {
  // Implement Twilio API call to send SMS alert
  // You can use the WiFiClientSecure to connect to Twilio API
  Serial.println("SMS alert sent via Twilio.");
}
