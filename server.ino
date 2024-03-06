#include <ESP8266WiFi.h>
#include <ArduinoJson.h>

const char* ssid = "PAYLOAD";
const char* password = "12345678";

WiFiServer server(23);

void setup() {
  Serial.begin(115200);
  pinMode(9, OUTPUT);
  
  WiFi.softAP(ssid, password);
  Serial.println("AP Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
  Serial.println("Server started");
}

void loop() {
  //Making sure killswitch is OFF by default
  digitalWrite(9, LOW);

  // Check for a client connection
  WiFiClient client = server.available();
  if (!client) {
    return; // If no client is connected, do nothing
  }

  Serial.println("Client connected.");
  while (client.connected()) { // Loop as long as the client is connected
    if (client.available()) {
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, client);

      if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        continue; // Skip to the next iteration of the loop
      }

      String type = doc["type"]; // Get the message type
      Serial.print("Received Type: ");
      Serial.println(type);

      if (type == "kill_switch") {
        int value = doc["value"]; // Get the kill switch value
        digitalWrite(9, value == 1 ? LOW : HIGH); // Handle the kill switch
        Serial.print("Kill Switch Value: ");
        Serial.println(value);
      } else if (type == "pid") {
        float p = doc["p"]; // Get P value
        float i = doc["i"]; // Get I value
        float d = doc["d"]; // Get D value
        // Handle PID values here
        Serial.print("PID Values - P: ");
        Serial.print(p);
        Serial.print(", I: ");
        Serial.print(i);
        Serial.print(", D: ");
        Serial.println(d);

        // Transmit PID values over TX
        String pidMessage = String(p) + ":" + String(i) + ":" + String(d) + "\n";
        Serial.print(pidMessage); // This line sends the PID string over TX
      }

      // Clear the buffer to ensure no stale data is processed
      while(client.available()) client.read();
    }
  }
  Serial.println("Client disconnected.");
}
