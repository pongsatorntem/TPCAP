typedef unsigned char U1;
typedef unsigned int U2;
typedef unsigned long U4;
typedef bool BL;

#define DEBUG_SERIAL Serial

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SPI.h>
#include <MCP23S17.h>  // MCP23S17 library
#include <ArduinoJson.h>

MCP inputchip1(0, D2);  // Instantiate an object called "inputchip" on an MCP23S17 device at address 1
MCP inputchip2(0, D1);  // Instantiate an object called "outputchip" on an MCP23S17 device at address 2
// Wi-Fi and Server Configuration
const char* ssid = "TOP";
const char* password = "tatoptatop";
const char* serverIP = "172.20.10.3";  // Replace with your server's IP
const int serverPort = 4499;  // The port your server is running on
WiFiClient client;  // WiFi client for HTTP communication
HTTPClient http;
String serverPath = "/flowrack/item_counter";
// Define the DIP switch pins (for selecting HP)
// const int DIP_SWITCH_PINS[4] = {29, 30, 31, 32};  // 4-channel DIP switch

// System Config
U2 JUDGE_TIME_THRESHOLD = 500;  // ms for detecting box movement

int lastKnownTotal[28] = {0};  // Array to store last known totals for sensors

// Timing variables
U4 lastReconnectAttempt = 0;
U4 lastTime = 0;
U4 timerDelay = 100;
U4 lastTime_2 = 0;
U4 timerDelay_2 = 500;
U4 lastActiveTime[28] = {0};  // Track the last time each sensor detected a box
U4 brokenSensorTimeout = 300000;  // Time threshold to detect if a sensor has stopped working (5 min)

bool isSensorBroken[28] = {false};  // Track if a sensor is marked as broken

// Sensor states and box counts
BL prevSensorStates[28] = {0};  // Store previous state of each sensor (16 sensors)
BL boxDetected[28] = {0};  // Track if a box was detected by each sensor
// int boxCounts[6][28] = {0};  // 6 HP and 16 sensors per HP COUNT

int currentHP = 1;  // Variable to store the current HP number

void setup() {
  Serial.begin(115200);  // Start serial communication
  SPI.begin();  // Start SPI communication
  
  inputchip1.begin();
  inputchip1.pinMode(0xFFFF);  // Set all MCP pins as inputs
  inputchip1.pullupMode(0xFFFF);  // Enable pull-up resistors
  inputchip1.inputInvert(0xFFFF);  // Invert the input logic

  inputchip2.begin();
  inputchip2.pinMode(0xFFFF);  // Set all MCP pins as inputs
  inputchip2.pullupMode(0xFFFF);  // Enable pull-up resistors
  inputchip2.inputInvert(0xFFFF);  // Invert the input logic

  // Wi-Fi connection setup
  WiFi.begin(ssid, password);  // Connect to Wi-Fi
  WiFi.setAutoReconnect(true);  // Enable automatic reconnection
  WiFi.persistent(true);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
  }
  
  Serial.println("Connected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // currentHP = readHPFromDipSwitch();  // Read HP from DIP switch at setup
  Serial.print("Current HP identified: ");
  Serial.println(currentHP);

  connectToWiFi();  // Ensure connection to Wi-Fi
  
  //fetchTotalsFromServer();     // GET TOTAL FROM SERVER

}

void loop() {
  int input_channel_1 = inputchip1.digitalRead();  // Read sensors from MCP1
  int input_channel_2 = inputchip2.digitalRead();  // Read sensors from MCP2

  // Call function to read sensors and count boxes
  for (int i = 0; i < 16; i++) {
    BL currentState = (i < 8) ? ((input_channel_1 >> i) & 0x01) : ((input_channel_2 >> (i - 8)) & 0x01);
    handleSensor(i, currentState, millis());
  }
  // Check Wi-Fi connection status and reconnect if necessary
  if (WiFi.status() != WL_CONNECTED) {
    U4 currentMillis = millis();
    if (currentMillis - lastReconnectAttempt >= 5000) {  // Attempt to reconnect every 5 seconds
      lastReconnectAttempt = currentMillis;
      Serial.println("Wi-Fi disconnected. Attempting to reconnect...");
      connectToWiFi();
    }
  } else {
    Serial.print("Connected to Wi-Fi with IP: ");
    Serial.println(WiFi.localIP());
  }

  delay(100);  // Add delay for readability and processing
}

/*void fetchTotalsFromServer() {
  String url = String("http://") + serverIP + ":" + String(serverPort) + "/get_totals";
  if (http.begin(client, url)) {
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.print("Received total values from server: ");
      Serial.println(response);

      // Parse JSON response to get the total counts
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, response);

      for (int i = 0; i < 28; i++) {
        lastKnownTotal[i] = doc["sensor"][i];  // Assume server response has total values for each sensor
        boxCounts[currentHP - 1][i] = lastKnownTotal[i];  // Update local count to match server
      }
    } else {
      Serial.print("Error in fetching totals: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  }
}
*/        // GET DATA FROM SERVER

// Function to send HTTP POST request to the server
void POST_https(String serverPath, String HP, String flowrack, int state, int slot) {
  String request_path = serverPath + "?hp=" + HP + "&flowrack=" + flowrack + "&state=" + String(state) + "&slot=" + String(slot);

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.setTimeout(1000);  // Set a timeout of 1000ms for the request
    http.begin(client, request_path.c_str());

    // Send an HTTP POST request
    int httpResponseCode = http.POST("");  // Empty payload

    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();  // Get the response payload
      Serial.println("Response from server: " + payload);
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }

    // Free resources
    http.end();
  } else {
    Serial.println("WiFi Disconnected. Data will be stored locally.");
  }
}


// Function to connect to Wi-Fi
void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 10) {
    delay(1000);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to Wi-Fi. Retrying...");
  }
}

// Function to read the current HP from the DIP switch
// int readHPFromDipSwitch() {
//   int hp = 0;
//   for (int i = 0; i < 4; i++) {
//     int pinValue = (inputchip2.digitalRead() >> (12 + i)) & 0x01;
//     if (digitalRead(DIP_SWITCH_PINS[i]) == LOW) {
//       hp |= (1 << i);
//     }
//   }
//   return (hp % 6) + 1;  // Ensure HP value is between 1 and 6
// }

// Function to read sensor states and count boxes
void readAndCountSensors(int input_channel_1, int input_channel_2) {
  U4 current_time = millis();  // Get current time for time-based calculations

  BL sensor_flags[28] = {
    (input_channel_1 >> 0) & 0x01, 
    (input_channel_1 >> 1) & 0x01, 
    (input_channel_1 >> 2) & 0x01,
    (input_channel_1 >> 3) & 0x01, 
    (input_channel_1 >> 4) & 0x01, 
    (input_channel_1 >> 5) & 0x01,
    (input_channel_1 >> 6) & 0x01, 
    (input_channel_1 >> 7) & 0x01, 
    (input_channel_1 >> 8) & 0x01,
    (input_channel_1 >> 9) & 0x01, 
    (input_channel_1 >> 10) & 0x01, 
    (input_channel_1 >> 11) & 0x01,
    (input_channel_1 >> 12) & 0x01, 
    (input_channel_1 >> 13) & 0x01, 
    (input_channel_1 >> 14) & 0x01,
    (input_channel_1 >> 15) & 0x01, 
    (input_channel_2 >> 0) & 0x01, 
    (input_channel_2 >> 1) & 0x01,
    (input_channel_2 >> 2) & 0x01, 
    (input_channel_2 >> 3) & 0x01, 
    (input_channel_2 >> 4) & 0x01,
    (input_channel_2 >> 5) & 0x01, 
    (input_channel_2 >> 6) & 0x01, 
    (input_channel_2 >> 7) & 0x01,
    (input_channel_2 >> 8) & 0x01, 
    (input_channel_2 >> 9) & 0x01, 
    (input_channel_2 >> 10) & 0x01,
    (input_channel_2 >> 11) & 0x01
  };

  for (int i = 0; i < 28; i++) {
    handleSensor(i, sensor_flags[i], current_time);
  }

  // Store counts periodically or based on some condition
  if ((millis() - lastTime_2) > timerDelay_2) {
    printCounts();  // Print stored box counts for current HP
    lastTime_2 = millis();
  }
}

// Handle sensor logic (detect boxes and count)
void handleSensor(int sensorIndex, BL currentSensorState, U4 current_time) {
  String HP = String(currentHP);  // HP 1-6 (you can set this dynamically or use a dip switch)
  String flowrack = (sensorIndex < 4) ? "binning" : "domestic";  // First 4 sensors are BIN, next 4 are DOM
  int slot = (sensorIndex < 4) ? sensorIndex + 1 : (sensorIndex - 3);  // Slots 1-4 for BIN, 1-4 for DOM
  int state = 1;  // 1 for receiving

  if (currentSensorState == 1 && prevSensorStates[sensorIndex] == 0) {
    // Box detected (rising edge)
    boxDetected[sensorIndex] = true;
    lastActiveTime[sensorIndex] = current_time;  // Update the last active time
    isSensorBroken[sensorIndex] = false;  // Reset the broken flag if the sensor is active
    Serial.print("Sensor ");
    Serial.print(sensorIndex + 1);
    Serial.println(" detected a box.");
  } else if (currentSensorState == 0 && prevSensorStates[sensorIndex] == 1 && boxDetected[sensorIndex]) {
    // Box passed (falling edge)
    // boxCounts[currentHP - 1][sensorIndex]++;  // Increment count for current sensor at current HP COUNT
    boxDetected[sensorIndex] = false;
    lastActiveTime[sensorIndex] = current_time;  // Update the last active time
    Serial.print("Sensor ");
    Serial.print(sensorIndex + 1);
    Serial.print(" counted a box. ");
    // Serial.println(boxCounts[currentHP - 1][sensorIndex]); // COUNT
  }
  // Send updated count to server
    
    // Serial.print("String(currentHP), "domestic", boxCounts[currentHP - 1][sensorIndex], sensorIndex + 1 ");
    POST_https(serverPath, "demo", flowrack, state, slot);  // POST the data to server
  }
  // Check if the sensor has stopped working for too long
  if (current_time - lastActiveTime[sensorIndex] > brokenSensorTimeout && !isSensorBroken[sensorIndex]) {
    isSensorBroken[sensorIndex] = true;  // Mark the sensor as broken
    Serial.print("Warning: Sensor ");
    Serial.print(sensorIndex + 1);
    Serial.println(" is broken or disconnected.");
  }

  // Update previous state for next loop
  prevSensorStates[sensorIndex] = currentSensorState;
}

// Function to print box counts for the current HP
// void printCounts() {
//   Serial.print("HP ");
//   Serial.print(currentHP);
//   Serial.println(" Box Counts:");
//   for (int i = 0; i < 28; i++) {
//     Serial.print("Sensor ");
//     Serial.print(i + 1);
//     Serial.print(": ");
//     // Serial.println(boxCounts[currentHP - 1][i]); //COUNT
//   }
}
