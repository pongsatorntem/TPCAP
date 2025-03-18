
typedef unsigned char U1;
typedef unsigned int U2;
typedef unsigned long U4;
typedef unsigned long long U64;
typedef bool BL;

#define DEBUG_SERIAL Serial

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SPI.h>      // We use this library, so it must be called here.
#include <MCP23S17.h> // Here is the new class to make using the MCP23S17 easy.
#include <ArduinoJson.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncHTTPRequest_Generic.h>
#include <ArduinoOTA.h> // Include the ArduinoOTA library

// Define SPI pins specifically for ESP8266
// #define SCK_PIN D5   // Serial Clock
// #define MISO_PIN D7  // Master In Slave Out
// #define MOSI_PIN D6  // Master Out Slave In

MCP inputchip1(0, D2); // Instantiate an object called "inputchip" on an MCP23S17 device at address 1
                       // and slave-select on Arduino pin 10
MCP inputchip2(0, D1); // Instantiate an object called "outputchip" on an MCP23S17 device at address 2
                       // and slave-select on Arduino pin 10

// const char *ssid = "sugareyez_2.4G";
// const char *password = "987656789";
const char *ssid = "AMR_TDEM_2.4G";
const char *password = "1234@abcd";
const char *serverIP = "192.168.1.13"; // Server IP address
byte flowrackIp[4] = {192, 168, 1, 203};  // สุ่มเลขไม่ให้ซ้ำ
byte gateway[4] = {192, 168, 1, 2};
byte subnet[4] = {255, 255, 255, 0};
const char *serverPort = "4499"; // Server port

// Time control
U64 initialTimestamp = 0; // Timestamp received from the server
U64 millisAtSync = 0;     // millis() value when timestamp was received
U64 updatedTimestamp = 0;

// Use String objects for URL construction
String serverBaseURL = "http://" + String(serverIP) + ":" + String(serverPort);
String getFlowrackPath = "/station/initial-flowrack-name";
String itemCounterPath = "/station/item_counter";

// Construct the full URLs
String serverGetURL = serverBaseURL + getFlowrackPath;
String serverPostURL = serverBaseURL + itemCounterPath;

int currentHP = 1;

// System configuration
U2 JUDGE_TIME_THRESHOLD = 500;   // Time threshold for box movement detection (ms)
U4 brokenSensorTimeout = 300000; // Timeout for detecting sensor failures

// Sensor tracking and flag variables
U4 lastActiveTime[28] = {0};
bool isSensorBroken[28] = {false};
int prevSensorStates[28] = {0};
BL boxDetected[28] = {0};
int lastKnownTotal[28] = {0}; // Array to store the last known totals for sensors

const int sensorsPerFlowrack = 4; // Each flowrack type will use 4 sensors
int totalFlowrackTypes = 0;       // Will be dynamically updated from server data
BL sensorFlags[28] = {0};         // Holds the sensor flags for all 28 sensors
BL finalSensorFlags[28] = {0};    // Holds final sensor flags for each sensor
BL prevSensorFlags[28] = {0};     // Holds previous sensor flags for debouncing
U4 sensorTimes[28] = {0};

U4 lastTime = 0;
U4 lastNetTime = 0;
U4 timerDelay = 200;
U4 timerNetDelay = 8000;
U4 lastTime_2 = 0;
U4 timerDelay_2 = 500;

U1 PHOTO_ON = 1;
U1 PHOTO_OFF = 0;

U2 SENSOR_HOLD_CHECKED = 4000;
U2 SENSOR_HOLD_CNT_UP_CHECKED = 2000;
U2 SENSOR_CHANGE_CHECK_DELAY = 100;

enum SlotType
{
  SLOT_EMPTY_TYPE,
  SLOT_DOM_TYPE,
  SLOT_BIN_TYPE, // 0
  SLOT_EXPORT_TYPE,
  SLOT_UNDEFINED // 1
};

enum SlotTransferType
{
  SLOT_UNDEFINED_TYPE,
  SLOT_RECEIVE_TYPE,
  SLOT_SEND_TYPE,
};

enum SlotRequestState
{
  NO_REQUEST,
  REQUESTED,
  REQUEST_HOLD
};

enum RequestCounterState
{
  COUNT_UP = 1,
  COUNT_DOWN = 2,
  HOLD = 3,
  NO_HOLD = 4,
  ITEM_IN = 5
};

/*********** Define Pin_name and Pin_no ***********/
// Digital Input
// U1 Binning_Button = 14; //GPIO14 -> D5

// Digital Output
// U1 Binning_Lamp = 5;  //GPIO5 -> D1

// Input Status
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

const int interruptInterval = 2000; // Interval in milliseconds
volatile bool interruptFlag = false;

const int MAX_SLOTS = 4;
const int MAX_FLOWRACKS = 10;

// Function Declarations
void connectNetwork();
int readHPFromDipSwitch();
void handlePhotoState(U2 input_channel_1, U2 input_channel_2, U64 currentTimestamp);
SlotType getSlotType(String slotTypeName);
String getFlowrackDataFromServer(String HP);
SlotTransferType getSlotTransferType(String slotTransferTypeName);
void asyncPostHTTP(String flowrack, RequestCounterState state, int row, int col);
void onRequestPostHTTPComplete(void *arg, AsyncHTTPRequest *req, int readyState);
void setupOTA();

// Slot class
class Slot
{
private:
  int row;
  int col;
  U64 currentTimestamp;
  int currentSensorState;
  U64 previousTimestamp;
  int previousSensorState;
  int isActive;
  int readIndex;
  String flowrackName;
  SlotType slotType;
  SlotTransferType slotTransferType;
  SlotRequestState requestState;

public:
  // Constructor to initialize the slot with its row and column
  Slot(int row, int col, String flowrackName, int isActive, SlotType type, SlotTransferType transferType, int sensorReadIndex)
  {
    this->row = row;
    this->col = col;
    this->flowrackName = flowrackName;
    this->isActive = isActive;
    this->currentSensorState = 0;
    this->previousSensorState = 0;
    this->currentTimestamp = initialTimestamp;
    this->previousTimestamp = initialTimestamp;
    this->slotType = type;
    this->slotTransferType = transferType;
    this->requestState = NO_REQUEST;
    this->readIndex = sensorReadIndex;
  }

  U64 getCurrentTimestamp()
  {
    return this->currentTimestamp;
  }

  U64 getPreviousTimestamp()
  {
    return this->previousTimestamp;
  }

  int getReadIndex()
  {
    return this->readIndex;
  }

  void setPhotoState(int photoState, U64 timeStamp)
  {
    this->previousSensorState = this->currentSensorState;
    this->previousTimestamp = this->currentTimestamp;
    this->currentSensorState = photoState;
    this->currentTimestamp = timeStamp;
  }
  void setRequestState(SlotRequestState state)
  {
    this->requestState = state;
  }

  SlotRequestState getRequestState()
  {
    return this->requestState;
  }

  int getPreviousSensorState()
  {
    return this->previousSensorState;
  }

  int getCurrentSensorState()
  {
    return this->currentSensorState;
  }

  int getRow()
  {
    return this->row;
  }

  int getCol()
  {
    return this->col;
  }

  bool isSensorStateChanged(int photoState, U64 currentTimeStamp)
  {
    if (currentTimeStamp - this->currentTimestamp > SENSOR_CHANGE_CHECK_DELAY)
    {
      return (this->currentSensorState != photoState);
    }
    else
    {
      return 0;
    }
  }

  bool shouldSendHoldFlg(U64 currentTimestamp)
  {
    if (currentTimestamp - this->currentTimestamp > SENSOR_HOLD_CHECKED)
    {
      if (this->requestState == NO_REQUEST)
      {
        return 1;
      }
      else
      {
        return 0;
      }
    }
    else
    {
      return 0;
    }
  }

  bool shouldSendCountUpFlg()
  {
    if (this->requestState == NO_REQUEST)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }

  bool shouldSendReleaseHoldFlg(U64 currentTimestamp)
  {
    DEBUG_SERIAL.print("📌 Checking shouldSendReleaseHoldFlg | PHOTO STATE: ");
    DEBUG_SERIAL.println(this->currentSensorState);
    DEBUG_SERIAL.print("🕒 Time Diff: ");
    DEBUG_SERIAL.println(currentTimestamp - this->currentTimestamp);
    DEBUG_SERIAL.print("📌 Request State: ");
    DEBUG_SERIAL.println(this->requestState);

    if (this->currentSensorState == PHOTO_OFF)
    {
        DEBUG_SERIAL.println("✅ PHOTO_OFF detected! Sending NO_HOLD.");
        return true;
    }
    if (this->requestState == REQUEST_HOLD &&
        (currentTimestamp - this->currentTimestamp > SENSOR_HOLD_CHECKED))
    {
        DEBUG_SERIAL.println("✅ Hold timeout reached! Sending NO_HOLD.");
        return true;
    }

    DEBUG_SERIAL.println("❌ NO_HOLD condition not met yet.");
    return false;
  }


  bool shouldSendCountUpOnHoldSendSlot(U64 currentTimestamp)
  {
    if (currentTimestamp - this->currentTimestamp > SENSOR_HOLD_CNT_UP_CHECKED)
    {
      if (this->requestState == NO_REQUEST)
      {
        return 1;
      }
      else
      {
        return 0;
      }
    }
    else
    {
      return 0;
    }
  }
  String getFlowrackName()
  {
    return this->flowrackName;
  }

  SlotType getSlotType()
  {
    return this->slotType;
  }

  SlotTransferType getSlotTransferType()
  {
    return this->slotTransferType;
  }
};

// Flowrack class to manage 2D slots

Slot *slotlist[32];
U1 allSlotCount = 0;

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  // SPI.pins(SCK_PIN, MISO_PIN, MOSI_PIN, D2);
  // SPI.begin();
  // Initialize SPI with custom pins

  DEBUG_SERIAL.begin(9600);

  /*********** MCP23S17 2 channels Config ***********/
  inputchip1.begin();

  // outputchip.begin();
  inputchip1.pinMode(0xFFFF);     // Use word-write mode to set all of the pins on inputchip to be inputs
  inputchip1.pullupMode(0xFFFF);  // Use word-write mode to Turn on the internal pull-up resistors.
  inputchip1.inputInvert(0xFFFF); // Use word-write mode to invert the inputs so that logic 0 is read as HIGH

  inputchip2.begin();
  inputchip2.pinMode(0xFFFF);     // Use word-write mode to set all of the pins on inputchip to be inputs
  inputchip2.pullupMode(0xFFFF);  // Use word-write mode to Turn on the internal pull-up resistors.
  inputchip2.inputInvert(0xFFFF); // Use word-write mode to invert the inputs so that logic 0 is read as HIGH

  // Read DIP switch to identify which H/P is being used
  currentHP = readHPFromDipSwitch();  // HP4 0010 Exp 0000
  connectNetwork();
  delay(4000);

  String jsonData = getFlowrackDataFromServer(String(currentHP));

  // Parse the server response (assuming this updates the 'flowrackData' structure)
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, jsonData);

  if (!error)
  {

    JsonArray flowrackArray = doc["data"].as<JsonArray>();
    initialTimestamp = doc["timestamp"];
    millisAtSync = millis();

    // Loop through JSON array and create Flowrack objects
    for (JsonObject obj : flowrackArray)
    {
      String flowrackName = obj["name"].as<String>();
      JsonArray slotIsActive = obj["slot_style"];
      JsonArray slotType = obj["slot_type"];
      JsonArray slotTransferType = obj["slot_transfer_type"];
      JsonArray slotReadIndex = obj["slot_read_index"];

      int rows = slotIsActive.size();
      int cols = slotIsActive[0].size();

      for (int row = 0; row < rows; row++)
      {
        for (int col = 0; col < cols; col++)
        {
          int isActive = slotIsActive[row][col];
          int readIndex = slotReadIndex[row][col];
          String type = slotType[row][col];
          String transferType = slotTransferType[row][col];
          SlotType slotType = getSlotType(type);
          SlotTransferType slotTransferType = getSlotTransferType(transferType);
          slotlist[allSlotCount] = new Slot(row, col, flowrackName, isActive, slotType, slotTransferType, readIndex);
          allSlotCount++;
        }
      }
    }
  }
  else
  {
    DEBUG_SERIAL.println("Failed to parse JSON data.");
  }

  setupOTA();
}

void loop()
{
  ArduinoOTA.handle();
  U64 current_time = millis();
  if ((current_time - lastTime) > timerDelay)
  {

    // Read input channels from MCP23S17
    U2 input_channel_1 = ~inputchip1.digitalRead() & 0xffff; // Read the first set of 16 inputs (MCP1)
    U2 input_channel_2 = ~inputchip2.digitalRead() & 0xffff; // Read the second set of 16 inputs (MCP2)
    U64 updatedTimestamp = initialTimestamp + current_time;
    lastTime = current_time;

    handlePhotoState(input_channel_1, input_channel_2, updatedTimestamp);
  }

  // Serial.println(WiFi.status());

  if (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(LED_BUILTIN, 0);
    connectNetwork(); // Attempt to reconnect if WiFi is disconnected
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void handlePhotoState(U2 input_channel_1, U2 input_channel_2, U64 currentTimestamp)
{
  U4 inputMergeChannel = (input_channel_2 << 16) | input_channel_1;

  for (U1 cnt = 0; cnt < allSlotCount; cnt++)
  {
    Slot *SlotObj = slotlist[cnt];
    int rIndex = SlotObj->getReadIndex();
    U1 photoState = (inputMergeChannel >> (rIndex - 1)) & 0x01;
    // SlotType type = SlotObj->getSlotType();
    SlotTransferType transferType = SlotObj->getSlotTransferType();
    String flowrackName = SlotObj->getFlowrackName();
    int row = SlotObj->getRow();
    int col = SlotObj->getCol();

    if (SlotObj->isSensorStateChanged(photoState, currentTimestamp))
    {
      // Update photo state
      SlotObj->setPhotoState(photoState, currentTimestamp);

      // Photo changed from OFF -> ON
      if (photoState == PHOTO_ON)
      {
        DEBUG_SERIAL.print("Index: ");
        DEBUG_SERIAL.print(rIndex);
        DEBUG_SERIAL.println(" changed to: on");

        if (transferType == SLOT_RECEIVE_TYPE)
        {
          // Send item in flag to server
          asyncPostHTTP(flowrackName, ITEM_IN, row, col);
        }
      }
      else
      {
        // Photo changed from ON -> OFF
        if (transferType == SLOT_SEND_TYPE)
        {
          if (SlotObj->shouldSendCountUpFlg())
          { // Send count up to server
            asyncPostHTTP(flowrackName, COUNT_UP, row, col);
          }
          else
          {
            // Reset flag to NO Request
            SlotObj->setRequestState(NO_REQUEST);
          }
        }
        else if (transferType == SLOT_RECEIVE_TYPE)
        {
          if (photoState == PHOTO_OFF) // กล่องออกไปแล้ว
          {
              if (SlotObj->shouldSendReleaseHoldFlg(currentTimestamp))
              {
                  DEBUG_SERIAL.println("✅ Sending NO_HOLD to clear FULL state.");
                  asyncPostHTTP(flowrackName, NO_HOLD, row, col);
                  SlotObj->setRequestState(NO_REQUEST);
              }
          }
        }
      }
    }
    else
    {
      if (photoState == PHOTO_ON)
      {
        if (transferType == SLOT_RECEIVE_TYPE)
        {
          if (SlotObj->shouldSendHoldFlg(currentTimestamp))
          {
            // Send hold to server
            asyncPostHTTP(flowrackName, HOLD, row, col);
            SlotObj->setRequestState(REQUEST_HOLD);
          }
        }
        else if (transferType == SLOT_SEND_TYPE)
        {
          if (SlotObj->shouldSendCountUpOnHoldSendSlot(currentTimestamp))
          {
            asyncPostHTTP(flowrackName, COUNT_UP, row, col);
            SlotObj->setRequestState(REQUEST_HOLD);
          }
        }
      }
    }
  }
}

void connectNetwork()
{
  /*********** Wifi Config ***********/
  // Set your Static IP address
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print("ESP Board MAC Address:  ");
  DEBUG_SERIAL.println(WiFi.macAddress());
  IPAddress local_IP(192, 168, 1, 115);
  IPAddress gateway(192, 168, 1, 2);

  IPAddress subnet(255, 255, 255, 0);

  delay(1000);
  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet))
  {
    DEBUG_SERIAL.println("STA Failed to configure");
  }
  else
  {
    DEBUG_SERIAL.println("STA Completed to configure");
  }

  WiFi.begin(ssid, password);
  // WiFi.begin(ssid);
  DEBUG_SERIAL.println("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
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

int readHPFromDipSwitch()
{
  U2 mcp_value = inputchip2.digitalRead(); // GPA5 -> Pin 30
  U1 dip_value = mcp_value >> 4 & 0x0f;

  return dip_value; // Return HP value
}

String getFlowrackDataFromServer(String HP)
{
  HTTPClient http;
  String payload = "";
  String query = "?station_code=" + HP;

  String initFlowrackPath = String(serverGetURL) + query;
  if (WiFi.status() == WL_CONNECTED)
  {
    DEBUG_SERIAL.println("Requesting path: " + initFlowrackPath);
    http.begin(client, initFlowrackPath);
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0)
    {
      payload = http.getString();
      DEBUG_SERIAL.println("Server Response: " + payload);
    }
    else
    {
      DEBUG_SERIAL.print("Error on HTTP request: ");
      DEBUG_SERIAL.println(httpResponseCode);
    }
    http.end();
  }
  else
  {
    DEBUG_SERIAL.println("WiFi not connected");
  }
  return payload;
}

void asyncPostHTTP(String flowrack, RequestCounterState state, int row, int col)
{
  AsyncHTTPRequest *request = new AsyncHTTPRequest(); // Dynamically allocated

  String jsonPayload = "{\"station_code\":\"" + String(currentHP) +
                       "\",\"flowrack_name\":\"" + flowrack +
                       "\",\"state\":" + state +
                       ",\"row\":" + row +
                       ",\"col\":" + col + "}";

  DEBUG_SERIAL.println("Sending HTTP POST to: " + jsonPayload);

  if (WiFi.status() == WL_CONNECTED)
  {
    DEBUG_SERIAL.println("Requesting path: " + serverPostURL);
    if (request->readyState() == readyStateUnsent || request->readyState() == readyStateDone)
    {
      // Attach a lambda to clean up after the request completes
      request->onReadyStateChange([request](void *arg, AsyncHTTPRequest *req, int readyState)
                                  {
                if (readyState == readyStateDone)
                {
                    // DEBUG_SERIAL.printf("Request completed: %s\n", req->responseText().c_str());
                    delete request; // Clean up
                } });

      request->open("POST", serverPostURL.c_str());
      request->setReqHeader("Content-Type", "application/json");
      request->send(jsonPayload);
    }
    else
    {
      delete request; // Clean up if the request cannot be sent
    }
  }
  else
  {
    delete request; // Clean up if WiFi is not connected
  }
}

void onRequestPostHTTPComplete(void *arg, AsyncHTTPRequest *req, int readyState)
{
  if (readyState == readyStateDone)
  {
    DEBUG_SERIAL.printf("Request 1 completed: %s\n", req->responseText().c_str());
  }
}

SlotType getSlotType(String slotTypeName)
{
  SlotType type = SLOT_UNDEFINED;
  if (slotTypeName == "Binning")
  {
    type = SLOT_BIN_TYPE;
  }
  else if (slotTypeName == "Empty")
  {
    type = SLOT_EMPTY_TYPE;
  }
  else if (slotTypeName == "Domestic")
  {
    type = SLOT_DOM_TYPE;
  }
  else if (slotTypeName == "Export")
  {
    type = SLOT_EXPORT_TYPE;
  }
  else
  {
    type = SLOT_UNDEFINED;
  }

  return type;
}

SlotTransferType getSlotTransferType(String slotTransferTypeName)
{
  SlotTransferType type = SLOT_UNDEFINED_TYPE;
  if (slotTransferTypeName == "receive")

  {
    type = SLOT_RECEIVE_TYPE;
  }
  else
  {
    type = SLOT_SEND_TYPE;
  }

  return type;
}

void setupOTA()
{
  ArduinoOTA.onStart([]()
                     {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        DEBUG_SERIAL.println("Start updating " + type); });
  ArduinoOTA.onEnd([]()
                   { DEBUG_SERIAL.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { DEBUG_SERIAL.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
        DEBUG_SERIAL.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            DEBUG_SERIAL.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            DEBUG_SERIAL.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            DEBUG_SERIAL.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            DEBUG_SERIAL.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            DEBUG_SERIAL.println("End Failed");
        } });
  ArduinoOTA.begin();
}