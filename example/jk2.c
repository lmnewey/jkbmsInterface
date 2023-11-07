#include <WiFi.h>
#include <PubSubClient.h>
#include "jkbmsInterface.h"


const char* ssid = "yourSSID";
const char* password = "yourPASSWORD";


// Update these with values suitable for your network.
const char* mqtt_server = "broker.hivemq.com";

WiFiClient espClient;
PubSubClient client(espClient);
JKBMS jkbms;

void setup_mqtt() {
  client.setServer(mqtt_server, 1883);
  // Optionally set a callback function for incoming messages
  client.setCallback(mqtt_callback);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // Handle incoming messages
}


void setup_wifi() {
  delay(10);
  // Connect to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
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

void setup() {
  Serial.begin(115200);
  setup_wifi();
  setup_mqtt();
  reconnect();

    // Initialize the JKBMS interface
  jkbms.start();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Publish messages
  client.publish("outTopic", "the payload");

   if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  // Request data from JKBMS
  jkbms.Request_JK_Battery_485_Status_Frame();

  // Assuming there's a small delay for data to be ready
  delay(1000); // Adjust the delay as needed

  // Publish each value to its own MQTT topic
  char message[50];

  // Example for publishing remaining capacity
  snprintf(message, 50, "%.2f", jkbms.capacity_remaining_derived_sensor_);
  mqttClient.publish("jkbms/remaining_capacity", message);

  // Repeat for other data points, e.g., total voltage, current, etc.
  snprintf(message, 50, "%.2f", jkbms.total_voltage);
  mqttClient.publish("jkbms/total_voltage", message);

  snprintf(message, 50, "%.2f", jkbms.current);
  mqttClient.publish("jkbms/current", message);

    // ... Add more data points as needed

  // Ensure you don't publish too frequently to avoid flooding the MQTT broker
  delay(5000); // Adjust the delay as needed
}