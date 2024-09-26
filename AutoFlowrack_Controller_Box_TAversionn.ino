
typedef unsigned char U1;
typedef unsigned int U2;
typedef unsigned long U4;
typedef bool BL;

#define DEBUG_SERIAL Serial

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SPI.h>              // We use this library, so it must be called here.
#include <MCP23S17.h>         // Here is the new class to make using the MCP23S17 easy.
#include <ArduinoJson.h>

MCP inputchip1(0, D2);             // Instantiate an object called "inputchip" on an MCP23S17 device at address 1
                    // and slave-select on Arduino pin 10
MCP inputchip2(0, D1);            // Instantiate an object called "outputchip" on an MCP23S17 device at address 2
                    // and slave-select on Arduino pin 10

//char* ssid = "Wireless-PC";
//char* password = "";
const char* ssid = "TOP";
const char* password = "tatoptatop";
const char* serverIP = "172.20.10.3";  // Server IP address
const char* serverPort = "4499";           // Server port
// const char* serverBaseURL = "http://" + String(serverIP) + ":" + String(serverPort);
// const char* serverGetURL = serverBaseURL + String(getFlowrackPath);
// const char* serverPostURL = serverBaseURL + String(postFlowrackPath);
const char* serverGetURL = "http://172.20.10.3:4499/station/initial-flowrack-name?station_code=1"; 
const char* serverPostURL = "http://172.20.10.3:4499/flowrack/item_counter";

// System configuration
U2 JUDGE_TIME_THRESHOLD = 500;  // Time threshold for box movement detection (ms)
U4 brokenSensorTimeout = 300000; // Timeout for detecting sensor failures

// Sensor tracking and flag variables
U4 lastActiveTime[28] = {0};
bool isSensorBroken[28] = {false};
BL prevSensorStates[28] = {0};
BL boxDetected[28] = {0};
int lastKnownTotal[28] = {0};  // Array to store the last known totals for sensors
int currentHP = 1;

const int sensorsPerFlowrack = 4;  // Each flowrack type will use 4 sensors
int totalFlowrackTypes = 0;  // Will be dynamically updated from server data
BL sensorFlags[28] = {0};  // Holds the sensor flags for all 28 sensors
BL finalSensorFlags[28] = {0};  // Holds final sensor flags for each sensor
BL prevSensorFlags[28] = {0};  // Holds previous sensor flags for debouncing
U4 sensorTimes[28] = {0};


U4 lastTime = 0;
U4 timerDelay = 100;
U4 lastTime_2 = 0;
U4 timerDelay_2 = 500;

/*********** Define Pin_name and Pin_no ***********/
// Digital Input
//U1 Binning_Button = 14; //GPIO14 -> D5


// Digital Output
//U1 Binning_Lamp = 5;  //GPIO5 -> D1


//Input Status
U4 BIN_slot_1_time = 0;
U4 BIN_slot_2_time = 0;
U4 BIN_slot_3_time = 0;
U4 BIN_slot_4_time = 0;
U4 DOM_slot_1_time = 0;
U4 DOM_slot_2_time = 0;
U4 DOM_slot_3_time = 0;
U4 DOM_slot_4_time = 0;

BL BIN_slot_1_flag = 0;
BL final_BIN_slot_1_flag = 0;
BL prev_BIN_slot_1_flag = 0;
BL BIN_slot_2_flag = 0;
BL final_BIN_slot_2_flag = 0;
BL prev_BIN_slot_2_flag = 0;
BL BIN_slot_3_flag = 0;
BL final_BIN_slot_3_flag = 0;
BL prev_BIN_slot_3_flag = 0;
BL BIN_slot_4_flag = 0;
BL final_BIN_slot_4_flag = 0;
BL prev_BIN_slot_4_flag = 0;
BL DOM_slot_1_flag = 0;
BL final_DOM_slot_1_flag = 0;
BL prev_DOM_slot_1_flag = 0;
BL DOM_slot_2_flag = 0;
BL final_DOM_slot_2_flag = 0;
BL prev_DOM_slot_2_flag = 0;
BL DOM_slot_3_flag = 0;
BL final_DOM_slot_3_flag = 0;
BL prev_DOM_slot_3_flag = 0;
BL DOM_slot_4_flag = 0;
BL final_DOM_slot_4_flag = 0;
BL prev_DOM_slot_4_flag = 0;




WiFiClient client;

int test_counter = 0;

const int interruptInterval = 2000;  // Interval in milliseconds
volatile bool interruptFlag = false;

// Function Declarations
void Connect_Self_Network();
void POST_https(String serverPath, String HP, String flowrack, int state, int slot);
int readHPFromDipSwitch();
void checkForBrokenSensors(U4 current_time);
String getFlowrackDataFromServer();
void parseFlowrackData(String jsonData);
void assignSensorsToFlowrack(String flowrackType, int flowrackIndex, int maxSensorsToUse);


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  DEBUG_SERIAL.begin(115200);

  /*********** MCP23S17 2 channels Config ***********/
  inputchip1.begin();
  
  //outputchip.begin();
  inputchip1.pinMode(0xFFFF);     // Use word-write mode to set all of the pins on inputchip to be inputs
  inputchip1.pullupMode(0xFFFF);  // Use word-write mode to Turn on the internal pull-up resistors.
  inputchip1.inputInvert(0xFFFF); // Use word-write mode to invert the inputs so that logic 0 is read as HIGH

  inputchip2.begin();
  inputchip2.pinMode(0xFFFF);     // Use word-write mode to set all of the pins on inputchip to be inputs
  inputchip2.pullupMode(0xFFFF);  // Use word-write mode to Turn on the internal pull-up resistors.
  inputchip2.inputInvert(0xFFFF); // Use word-write mode to invert the inputs so that logic 0 is read as HIGH
  

  /*********** Input Config ***********/
  //pinMode(Binning_Button, INPUT_PULLUP);


  /*********** Output Config ***********/
  //pinMode(Binning_Lamp, OUTPUT);


  //digitalWrite(Binning_Lamp,HIGH);

//  Binning_flag = !digitalRead(Binning_Button);
//  if(Binning_flag == false)
//  {
//    ssid = "AMR_TDEM";
//    password = "1234@abcd";
//    Connect_Self_Network();
//  }
//  else
//  {
//    ssid = "Wireless-PC";
//    password = "";
//    Connect_TPCAP_Network();
//  }

  //ssid = "AMR_TDEM";
  //password = "1234@abcd";
  
  // Read DIP switch to identify which H/P is being used
  currentHP = readHPFromDipSwitch();
  Serial.print("Current H/P: ");
  Serial.println(currentHP);

  Connect_Self_Network();
  
  String jsonData = getFlowrackDataFromServer(String(currentHP));  
  parseFlowrackData(jsonData);  
//  // Set up timer interrupt every 2 seconds
//  noInterrupts();  // Disable interrupts during configuration
//  timer1_attachInterrupt(timerISR);  // Attach the ISR function
//  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);  // Set timer frequency and type
//  timer1_write(80000000 / 16 * 2);  // Set the timer period for 2 seconds
//  interrupts();  // Enable interrupts after configuration
//
//  Serial.println("Timer interrupt every 100 ms is set up.");
  
}

// void timerISR() {
//   //interruptFlag = true;  // Set the flag indicating the interrupt occurred
//   //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
//   Serial.println("Timer interrupt");
// }



void loop() {
    // Read input channels from MCP23S17
    int input_channel_1 = inputchip1.digitalRead();  // Read the first set of 16 inputs
    int input_channel_2 = inputchip2.digitalRead();  // Read the second set of 16 inputs

    Serial.println("***********************************************************************************************************");

    int hp = readHPFromDipSwitch();
    Serial.print("Debounced HP value: ");
    Serial.println(hp);

    // Print all sensors connected to ESP8266 (MCP23S17 chips)
    Serial.println("All Sensors Connected to ESP8266:");
    for (int sensorIndex = 0; sensorIndex < 32; sensorIndex++) {
        int detect = 0;
        if (sensorIndex < 16) {
            detect = (input_channel_1 >> sensorIndex) & 0x01;  // From first MCP
        } else {
            detect = (input_channel_2 >> (sensorIndex - 16)) & 0x01;  // From second MCP
        }

        // Print status of each sensor (connected to MCP23S17)
        Serial.print("Sensor ");
        Serial.print(sensorIndex + 1);  // 1-based index
        Serial.print(": detection ");
        Serial.println(detect);  // Detection (1 for detected, 0 for not detected)
    }

    // Fetch data from the server
    Serial.println("Fetching data from server...");
    String serverResponse = getFlowrackDataFromServer(String(currentHP));
    if (serverResponse.isEmpty()) {
        Serial.println("Error: Could not retrieve data from the server.");
    } else {
        Serial.println("Server Response: ");
        Serial.println(serverResponse);

        // Parse the server response (assuming this updates the 'flowrackData' structure)
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, serverResponse);

        if (error) {
            Serial.print("Failed to parse JSON: ");
            Serial.println(error.c_str());
        } else {
            JsonArray flowrackArray = doc["data"].as<JsonArray>();

            U4 current_time = millis();
            if ((millis() - lastTime) > timerDelay) {
                lastTime = millis();

                Serial.println("Photo Sensor Status (Used by Server):");

                int sensorIndex = 0;  // Start indexing the sensors globally

                // Iterate over each flowrack from the server
                for (int i = 0; i < flowrackArray.size(); i++) {
                    JsonObject flowrack = flowrackArray[i];  // Get the flowrack object
                    String flowrackName = flowrack["name"]; // Get the flowrack name
                    JsonArray slotStyle = flowrack["slot_style"]; // Get the slot style array

                    Serial.print("Checking sensors for flowrack: ");
                    Serial.println(flowrackName);

                    // Iterate through each row in the slot style (rows of the flowrack)
                    for (int row = 0; row < slotStyle.size(); row++) {
                        JsonArray rowArray = slotStyle[row];  // Get each row in the slot style

                        // Calculate row-based sensor start for the current flowrack
                        for (int col = 0; col < rowArray.size(); col++) {
                            int sensorState = rowArray[col];  // 1 means sensor working, 0 means not working

                            // Read the sensor state from MCP based on sensorIndex
                            int detect = 0;
                            if (sensorIndex < 16) {
                                detect = (input_channel_1 >> sensorIndex) & 0x01;  // From first MCP
                            } else if (sensorIndex < 32) {
                                detect = (input_channel_2 >> (sensorIndex - 16)) & 0x01;  // From second MCP
                            }

                            // Only process and print for working sensors (sensorState == 1)
                            if (sensorState == 1) {
                                String slot = "[" + String(row) + "," + String(col) + "]";  // Slot format
                                Serial.print("Photo ");
                                Serial.print(sensorIndex + 1);  // Photo/sensor number (1-based index)
                                Serial.print(", ");
                                Serial.print(flowrackName);     // Flowrack type
                                Serial.print(", state 1, detect ");
                                Serial.print(detect);  // Detection (1 for detected, 0 for not detected)
                                Serial.print(", slot ");
                                Serial.println(slot);  // Slot format [x, y]
                            }

                            sensorIndex++;  // Move to the next sensor
                        }
                    }
                }

                // Handle the sensor timing and state change logic (if needed)
                checkForBrokenSensors(current_time);
            }
        }
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected, attempting to reconnect...");
        Connect_Self_Network();
    }

    delay(1000);  // Delay for readability
}





void POST_https(String serverURL, String HP, String flowrack, int state, int slot) {    
    String requestPath = serverURL + "?hp=" + HP + "&flowrack=" + flowrack + "&state=" + state + "&slot=" + slot;

    // Debug: Print the full request path
    Serial.println("Sending HTTP POST to: " + requestPath);

    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.setTimeout(1000);
        client.setTimeout(1000);

        // Initiate HTTP request
        http.begin(client, requestPath.c_str());

        // Send POST request (empty payload for now)
        int httpResponseCode = http.POST("");

        // Debug: Check and print the response
        if (httpResponseCode > 0) {
            Serial.print("HTTP Response code: ");
            Serial.println(httpResponseCode);
            String payload = http.getString();
            Serial.println("Response Payload: " + payload);
        } else {
            // Debug: Print the error code and details
            Serial.print("Error on HTTP request: ");
            Serial.println(httpResponseCode);
            Serial.println("Check if the server path or WiFi connection is correct.");
        }
        http.end();  // Free resources
    } else {
        Serial.println("WiFi Disconnected, cannot send HTTP request.");
    }
}



void Connect_TPCAP_Network()
{
  /*********** Wifi Config ***********/
  const char* ssid = "Wireless-PC";
  const char* password = "";

  // Set your Static IP address
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  delay(1000);
  
  //WiFi.begin(ssid, password);
  WiFi.begin(ssid);
  DEBUG_SERIAL.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(300);
    DEBUG_SERIAL.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  DEBUG_SERIAL.println("");
  DEBUG_SERIAL.print("Connected to WiFi network with IP Address: ");
  DEBUG_SERIAL.println(WiFi.localIP());
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  digitalWrite(LED_BUILTIN, LOW);
}

void Connect_Self_Network() 
{
  /*********** Wifi Config ***********/
  ssid = "TOP";
  password = "tatoptatop";

  Serial.println();
  Serial.print("ESP Board MAC Address: ");
  Serial.println(WiFi.macAddress());
  IPAddress local_IP(172,20,10,4);
  IPAddress gateway(172,20,10,1);
  IPAddress subnet(255,255,255,240);

  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  } 
  else{
    Serial.println("STA Completed to configure");
  }
  
  WiFi.begin(ssid, password);
  //WiFi.begin(ssid);
  DEBUG_SERIAL.println("Connecting");
  
  // Wait until the device connects to WiFi
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    DEBUG_SERIAL.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    attempts++;
  }
  
  // Check the status and print a more detailed message
  if (WiFi.status() == WL_CONNECTED) {
    DEBUG_SERIAL.println("");
    DEBUG_SERIAL.print("Connected to WiFi network with IP Address: ");
    DEBUG_SERIAL.println(WiFi.localIP());
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    digitalWrite(LED_BUILTIN, HIGH);
  } 
  else {
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println("Failed to connect to WiFi. Please check SSID or password.");
    
    // Additional error messages
    if (WiFi.status() == WL_NO_SSID_AVAIL) {
      DEBUG_SERIAL.println("Wrong SSID or SSID not available.");
    } else if (WiFi.status() == WL_CONNECT_FAILED) {
      DEBUG_SERIAL.println("Connection failed. Check the password.");
    } else if (WiFi.status() == WL_DISCONNECTED) {
      DEBUG_SERIAL.println("WiFi disconnected.");
    }
  }
}

const U4 DEBOUNCE_DELAY = 50;  // 50ms debounce delay
U4 lastDebounceTime = 0;
int lastStableHP = 0;

int readHPFromDipSwitch() {
    int pin30 = inputchip2.digitalRead(5); // GPA5 -> Pin 30
    int pin31 = inputchip2.digitalRead(6); // GPA6 -> Pin 31
    int pin32 = inputchip2.digitalRead(7); // GPA7 -> Pin 32

    int hp = (pin32 << 2) | (pin31 << 1) | pin30; // Binary to decimal conversion
    // hp = 1; // example 
    Serial.print("DIP Switch Channel: ");  
    Serial.print("Pin30: "); Serial.print(pin30);  
    Serial.print(" Pin31: "); Serial.print(pin31);  
    Serial.print(" Pin32: "); Serial.println(pin32);

    Serial.print("HP Value (from DIP Switch): ");
    Serial.println(hp);
    return hp;  // Return HP value
}


// Function to check if sensors are broken
void checkForBrokenSensors(U4 current_time) {
    for (int i = 0; i < 28; i++) {
        if ((current_time - lastActiveTime[i] > brokenSensorTimeout) && !isSensorBroken[i]) {
            isSensorBroken[i] = true;
            Serial.print("Warning: Sensor ");
            Serial.print(i + 1);
            Serial.println(" is broken or disconnected.");
        }
    }
}


String getFlowrackDataFromServer(String HP) {
    HTTPClient http;
    String payload = "";

    if (WiFi.status() == WL_CONNECTED) {
        http.begin(client, serverGetURL);
        int httpResponseCode = http.GET();
        if (httpResponseCode > 0) {
            payload = http.getString();
            Serial.println("Server Response: " + payload);
        } else {
            Serial.print("Error on HTTP request: ");
            Serial.println(httpResponseCode);
        }
        http.end();
    } else {
        Serial.println("WiFi not connected");
    }
    return payload;
}

void parseFlowrackData(String jsonData) {
    if (jsonData.isEmpty()) {
        Serial.println("Empty server response, cannot parse JSON.");
        return;
    }

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
        Serial.println("JSON deserialization failed:");
        Serial.println(error.c_str());  // Print the error message
        return;
    }

    // If JSON is valid, continue with flowrack data parsing
    serializeJsonPretty(doc, Serial);  // Debug: print parsed JSON

    if (doc.containsKey("data")) {  // Ensure the key exists
        JsonArray flowracks = doc["data"];
        int totalFlowrackTypes = flowracks.size();  // Number of flowrack types
        Serial.print("Total Flowrack Types: ");
        Serial.println(totalFlowrackTypes);

        // Iterate through the flowrack types and assign sensors
        for (int i = 0; i < totalFlowrackTypes; i++) {
            String flowrackType = flowracks[i];
            Serial.println("Flowrack Type: " + flowrackType);  // Debug print
            assignSensorsToFlowrack(flowrackType, i, totalFlowrackTypes);
        }
    } else {
        Serial.println("Data key is missing in JSON response");
    }
}




void assignSensorsToFlowrack(String flowrackType, int flowrackIndex, int totalFlowrackTypes) {
    // Each flowrack type will use 4 sensors
    int sensorsPerType = 4;  // Fixed to 4 sensors per flowrack type
    int sensorStart = flowrackIndex * sensorsPerType;
    int sensorEnd = sensorStart + sensorsPerType;

    // Limit the number of sensors based on total number of flowrack types (max 28 sensors)
    if (sensorEnd > 28) {
        sensorEnd = 28; // Limit the total number of sensors to 28
    }

    // Assign sensors dynamically to flowrack types
    for (int i = sensorStart; i < sensorEnd; i++) {
        Serial.print("Assigning sensor ");
        Serial.print(i + 1);
        Serial.print(" to flowrack type: ");
        Serial.println(flowrackType);
    }
}



