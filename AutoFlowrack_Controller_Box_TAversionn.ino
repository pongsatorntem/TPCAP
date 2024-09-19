
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
  // put your main code here, to run repeatedly:

  int input_channel_1,input_channel_2;                        // declare an integer to hold the value temporarily.
  input_channel_1 = inputchip1.digitalRead();  // read the input chip in word-mode, storing the result in "value"
  input_channel_2 = inputchip2.digitalRead();  // read the input chip in word-mode, storing the result in "value"
  // Serial.print(input_channel_1,BIN);
  // Serial.print("  ");
  // Serial.println(input_channel_2,BIN);

  int hp = readHPFromDipSwitch();
  Serial.print("Debounced HP value: ");
  Serial.println(hp);

  //Read PhotoSensor Loop: 100ms
  U4 current_time = millis();
  if ((millis() - lastTime) > timerDelay) 
  {

    // Serial.print(input_channel_1,BIN);
    // Serial.print("  ");
    // Serial.println(input_channel_2,BIN);

//  Serial.print(BIN_slot_1_flag);
//  Serial.print(BIN_slot_2_flag);
//  Serial.print(BIN_slot_3_flag);
//  Serial.print(BIN_slot_4_flag);
//  Serial.print(DOM_slot_1_flag);
//  Serial.print(DOM_slot_2_flag);
//  Serial.print(DOM_slot_3_flag);
//  Serial.println(DOM_slot_4_flag);

    
    lastTime = millis();
  }



  /*****Input Define*****/
  /*
  RB0 = BIN_slot_1
  RB1 = BIN_slot_2
  RB2 = BIN_slot_3
  RB3 = BIN_slot_4
  RB4 = DOM_slot_1
  RB5 = DOM_slot_2
  RB6 = DOM_slot_3
  RB7 = DOM_slot_4
  */


  BIN_slot_1_flag = (input_channel_1>>9)&0x01; //MCP1-GPB1
  BIN_slot_2_flag = (input_channel_1>>8)&0x01; //MCP1-GPB0
  BIN_slot_3_flag = 1-(input_channel_2>>8)&0x01; //MCP2-GPB0
  BIN_slot_4_flag = (input_channel_1>>10)&0x01; //MCP1-GPB2
  DOM_slot_1_flag = 1-(input_channel_2>>14)&0x01; //MCP2-GPB6
  DOM_slot_2_flag = (input_channel_1>>12)&0x01; //MCP1-GPB4
  DOM_slot_3_flag = (input_channel_1)&0x01; //MCP1-GPA0
  DOM_slot_4_flag = 1-(input_channel_2>>13)&0x01; //MCP2-GPB5

  //  Serial.print(BIN_slot_1_flag);
  //  Serial.print(BIN_slot_2_flag);
  //  Serial.print(BIN_slot_3_flag);
  //  Serial.print(BIN_slot_4_flag);
  //  Serial.print(DOM_slot_1_flag);
  //  Serial.print(DOM_slot_2_flag);
  //  Serial.print(DOM_slot_3_flag);
  //  Serial.println(DOM_slot_4_flag);

  // Serial.println(input_channel_1,BIN);
  // Serial.print("  ");
  // Serial.println(input_channel_2,BIN);


  // Cannot use input from channel_1
  // DOM_slot_2_flag = (input_channel_2>>8)&0x01;

  /************ Check Basket Detected *************/
  if(BIN_slot_1_flag==1 && prev_BIN_slot_1_flag==0)
  {
    BIN_slot_1_time = current_time;
  }
  if(BIN_slot_2_flag==1 && prev_BIN_slot_2_flag==0)
  {
    BIN_slot_2_time = current_time;
  }
  if(BIN_slot_3_flag==1 && prev_BIN_slot_3_flag==0)
  {
    BIN_slot_3_time = current_time;
  }
  if(BIN_slot_4_flag==1 && prev_BIN_slot_4_flag==0)
  {
    BIN_slot_4_time = current_time;
  }
  // if(DOM_slot_1_flag==1 && prev_DOM_slot_1_flag==0)
  // {
  //   DOM_slot_1_time = current_time;
  // }
  // Reverse logic due to sensor cannot adjust 
  if(DOM_slot_1_flag==1 && prev_DOM_slot_1_flag==0)
  {
    DOM_slot_1_time = current_time;
  }
  if(DOM_slot_2_flag==1 && prev_DOM_slot_2_flag==0)
  {
    DOM_slot_2_time = current_time;
  }
  if(DOM_slot_3_flag==1 && prev_DOM_slot_3_flag==0)
  {
    DOM_slot_3_time = current_time;
  }
  if(DOM_slot_4_flag==1 && prev_DOM_slot_4_flag==0)
  {
    DOM_slot_4_time = current_time;
  }

  /***** Check Basket Un-Detected and Confirm Basket Passed *****/
  if(BIN_slot_1_flag==0 && prev_BIN_slot_1_flag==1)
  {
    if(current_time-BIN_slot_1_time>JUDGE_TIME_THRESHOLD)
    {
      final_BIN_slot_1_flag = 1;
    }
  }
  if(BIN_slot_2_flag==0 && prev_BIN_slot_2_flag==1)
  {
    if(current_time-BIN_slot_2_time>JUDGE_TIME_THRESHOLD)
    {
      final_BIN_slot_2_flag = 1;
    }
  }
  if(BIN_slot_3_flag==0 && prev_BIN_slot_3_flag==1)
  {
    if(current_time-BIN_slot_3_time>JUDGE_TIME_THRESHOLD)
    {
      final_BIN_slot_3_flag = 1;
    }
  }
  if(BIN_slot_4_flag==0 && prev_BIN_slot_4_flag==1)
  {
    if(current_time-BIN_slot_4_time>JUDGE_TIME_THRESHOLD)
    {
      final_BIN_slot_4_flag = 1;
    }
  }
  // if(DOM_slot_1_flag==0 && prev_DOM_slot_1_flag==1)
  // {
  //   if(current_time-DOM_slot_1_time>JUDGE_TIME_THRESHOLD)
  //   {
  //     final_DOM_slot_1_flag = 1;
  //   }
  // }

  // Reverse logic due to sensor cannot adjust
  if(DOM_slot_1_flag==0 && prev_DOM_slot_1_flag==1)
  {
    if(current_time-DOM_slot_1_time>JUDGE_TIME_THRESHOLD)
    {
      final_DOM_slot_1_flag = 1;
    }
  }
  if(DOM_slot_2_flag==0 && prev_DOM_slot_2_flag==1)
  {
    if(current_time-DOM_slot_2_time>JUDGE_TIME_THRESHOLD)
    {
      final_DOM_slot_2_flag = 1;
    }
  }
  if(DOM_slot_3_flag==0 && prev_DOM_slot_3_flag==1)
  {
    if(current_time-DOM_slot_3_time>JUDGE_TIME_THRESHOLD)
    {
      final_DOM_slot_3_flag = 1;
    }
  }
  if(DOM_slot_4_flag==0 && prev_DOM_slot_4_flag==1)
  {
    if(current_time-DOM_slot_4_time>JUDGE_TIME_THRESHOLD)
    {
      final_DOM_slot_4_flag = 1;
    }
  }
  
  // Check for broken sensors
  checkForBrokenSensors(current_time);
  getFlowrackDataFromServer(String(currentHP));



  if (((millis() - lastTime_2) > timerDelay_2))
  {
    //http://localhost:4499/flowrack/item_counter?hp=demo&flowrack=domestic&state=1&slot=1
    String HP = "demo";
    String flowrack = "binning";
    int state = 1;
    int slot = 1;
    String serverPath = "http://172.20.10.3:4499/flowrack/item_counter";
    //String url = serverPath + "?hp=" + HP +"&flowrack=" + flowrack +"&state=" + state+"&slot=" + slot ;
    //POST_https(url);

    /*test_counter++;
    if(test_counter<=3)
    {
      state = 1;
    }
    else
    {
      state = 2;
    }
    if(test_counter==6)test_counter = 0;
    
    POST_https(serverPath,HP,flowrack,state,slot);*/

    /************ Send to Server *************/
    //POST_https(serverPath,HP,flowrack,state,slot);
    if(final_BIN_slot_1_flag==1)
    {
      POST_https(serverPath,HP,"binning",2,1);
      final_BIN_slot_1_flag = 0;
    }
    if(final_BIN_slot_2_flag==1)
    {
      POST_https(serverPath,HP,"binning",2,2);
      final_BIN_slot_2_flag = 0;
    }
    if(final_BIN_slot_3_flag==1)
    {
      POST_https(serverPath,HP,"binning",2,3);
      final_BIN_slot_3_flag = 0;
    }
    if(final_BIN_slot_4_flag==1)
    {
      POST_https(serverPath,HP,"binning",2,4);
      final_BIN_slot_4_flag = 0;
    }
    if(final_DOM_slot_1_flag==1)
    {
      POST_https(serverPath,HP,"domestic",1,1);
      final_DOM_slot_1_flag = 0;
    }
    if(final_DOM_slot_2_flag==1)
    {
      POST_https(serverPath,HP,"domestic",1,2);
      final_DOM_slot_2_flag = 0;
    }
    if(final_DOM_slot_3_flag==1)
    {
      POST_https(serverPath,HP,"domestic",1,3);
      final_DOM_slot_3_flag = 0;
    }
    if(final_DOM_slot_4_flag==1)
    {
      POST_https(serverPath,HP,"domestic",1,4);
      final_DOM_slot_4_flag = 0;
    }
    

    lastTime_2 = millis();
  }

  // Update previous sensor states
  prev_BIN_slot_1_flag = BIN_slot_1_flag;
  prev_BIN_slot_2_flag = BIN_slot_2_flag;
  prev_BIN_slot_3_flag = BIN_slot_3_flag;
  prev_BIN_slot_4_flag = BIN_slot_4_flag;
  prev_DOM_slot_1_flag = DOM_slot_1_flag;
  prev_DOM_slot_2_flag = DOM_slot_2_flag;
  prev_DOM_slot_3_flag = DOM_slot_3_flag;
  prev_DOM_slot_4_flag = DOM_slot_4_flag;

  Serial.println("Triggered Sensors: ");
  if(BIN_slot_1_flag == 1) { Serial.println("BIN Slot 1 Triggered"); }
  if(BIN_slot_2_flag == 1) { Serial.println("BIN Slot 2 Triggered"); }
  if(BIN_slot_3_flag == 1) { Serial.println("BIN Slot 3 Triggered"); }
  if(BIN_slot_4_flag == 1) { Serial.println("BIN Slot 4 Triggered"); }
  if(DOM_slot_1_flag == 1) { Serial.println("DOM Slot 1 Triggered"); }
  if(DOM_slot_2_flag == 1) { Serial.println("DOM Slot 2 Triggered"); }
  if(DOM_slot_3_flag == 1) { Serial.println("DOM Slot 3 Triggered"); }
  if(DOM_slot_4_flag == 1) { Serial.println("DOM Slot 4 Triggered"); }

  delay(1000);
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
    hp = 1; // example
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
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
        Serial.println("JSON deserialization failed");
        Serial.println(error.c_str());  // Print the error message
        return;
    }

    // Debug: Print the entire JSON object to see its structure
    serializeJsonPretty(doc, Serial);
    
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

    // Assign sensors for each flowrack type
    for (int i = sensorStart; i < sensorEnd; i++) {
        Serial.print("Assigning sensor ");
        Serial.print(i + 1);  // Sensor numbers start at 1, so i+1
        Serial.print(" to flowrack type: ");
        Serial.println(flowrackType);

        // Here you can also map to coordinates if needed, like:
        // Map coordinates based on sensor position in the set
        // int coordX = (i % 2);  // Alternates between 0 and 1 for X coordinate
        // int coordY = (i / 2);  // Alternates for Y coordinate
        // Serial.print("Coordinates: [");
        // Serial.print(coordX);
        // Serial.print(", ");
        // Serial.print(coordY);
        // Serial.println("]");

        // Assuming "HP" is stored in a global or retrieved variable
        int state = 1; // Example state, you can set this based on your logic
        int slot = i + 1; // Sensor slot is i+1

        // Send HTTP POST request to assign sensor to flowrack type
        POST_https(serverPostURL, String(currentHP), flowrackType, state, slot);
    }
}





