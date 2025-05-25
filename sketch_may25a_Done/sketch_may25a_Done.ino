
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
const char *serverIP = "192.168.1.13";   // Server IP address
byte flowrackIp[4] = {192, 168, 1, 99}; // สุ่มเลขไม่ให้ซ้ำ /จภ
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
U4 requestCounterId = 1;

U1 PHOTO_ON = 1;
U1 PHOTO_OFF = 0;

U2 SENSOR_HOLD_CHECKED = 4000;
U2 SENSOR_HOLD_CNT_UP_CHECKED = 2000;
U2 SENSOR_CHANGE_CHECK_DELAY = 100;

enum SlotType : uint8_t
{
  SLOT_EMPTY_TYPE,
  SLOT_DOM_TYPE,
  SLOT_BIN_TYPE, // 0
  SLOT_EXPORT_TYPE,
  SLOT_UNDEFINED // 1
};

enum SlotTransferType : uint8_t
{
  SLOT_UNDEFINED_TYPE,
  SLOT_RECEIVE_TYPE,
  SLOT_SEND_TYPE,
};

enum SlotRequestState : uint8_t
{
  NO_REQUEST,
  REQUESTED,
  REQUEST_HOLD
};

enum RequestCounterState : uint8_t
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

// Slot class
class Slot
{
private:
  U1 row;
  U1 col;
  U64 currentTimestamp;
  U1 currentSensorState;
  U64 previousTimestamp;
  U1 previousSensorState;
  U1 isActive;
  U1 readIndex;
  char flowrackName[15];
  SlotType slotType;
  SlotTransferType slotTransferType;
  SlotRequestState requestState;

public:
  // Add default constructor
  Slot() : row(0), col(0), isActive(0), slotType(SLOT_UNDEFINED), slotTransferType(SLOT_UNDEFINED_TYPE), requestState(NO_REQUEST), readIndex(0)
  {
    flowrackName[0] = '\0'; // Initialize empty string
    currentTimestamp = 0;
    previousTimestamp = 0;
    currentSensorState = 0;
    previousSensorState = 0;
  }

  // Constructor to initialize the slot with its row and column
  Slot(U1 row, U1 col, char flowrackName[], U1 isActive, SlotType type, SlotTransferType transferType, U1 sensorReadIndex)
  {
    this->row = row;
    this->col = col;
    strcpy(this->flowrackName, flowrackName);
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

  U1 getReadIndex()
  {
    return this->readIndex;
  }

  void setPhotoState(U1 photoState, U64 timeStamp)
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

  U1 getCurrentSensorState()
  {
    return this->currentSensorState;
  }

  U1 getRow()
  {
    return this->row;
  }

  U1 getCol()
  {
    return this->col;
  }

  bool isSensorStateChanged(U1 photoState, U64 currentTimeStamp)
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

  bool shouldSendReleaseHoldFlg()
  {
    if (this->requestState == REQUEST_HOLD)
    {
      return 1;
    }
    else
    {
      return 0;
    }
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
  char *getFlowrackName()
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

Slot slotlist[32];
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
  currentHP = readHPFromDipSwitch();
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
      char flowrackName[15];
      strcpy(flowrackName, obj["name"].as<String>().c_str());
      JsonArray slotIsActive = obj["slot_style"];
      JsonArray slotType = obj["slot_type"];
      JsonArray slotTransferType = obj["slot_transfer_type"];
      JsonArray slotReadIndex = obj["slot_read_index"];

      U1 rows = slotIsActive.size();
      U1 cols = slotIsActive[0].size();

      for (int row = 0; row < rows; row++)
      {
        for (int col = 0; col < cols; col++)
        {
          U1 isActive = slotIsActive[row][col];
          U1 readIndex = slotReadIndex[row][col];
          String type = slotType[row][col];
          String transferType = slotTransferType[row][col];
          SlotType slotType = getSlotType(type);
          SlotTransferType slotTransferType = getSlotTransferType(transferType);
          slotlist[allSlotCount] = Slot(row, col, flowrackName, isActive, slotType, slotTransferType, readIndex);
          allSlotCount++;
        }
      }
    }
  }
  else
  {
    DEBUG_SERIAL.println("Failed to parse JSON data.");
  }
}

void loop()
{
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
    Slot &SlotObj = slotlist[cnt];
    U1 rIndex = SlotObj.getReadIndex();
    U1 photoState = (inputMergeChannel >> (rIndex - 1)) & 0x01;
    // SlotType type = SlotObj.getSlotType();
    SlotTransferType transferType = SlotObj.getSlotTransferType();
    char flowrackName[15];
    strcpy(flowrackName, SlotObj.getFlowrackName());
    U1 row = SlotObj.getRow();
    U1 col = SlotObj.getCol();

    if (SlotObj.isSensorStateChanged(photoState, currentTimestamp))
    {
      // Update photo state
      SlotObj.setPhotoState(photoState, currentTimestamp);
      DEBUG_SERIAL.printf("📌 [setPhotoState] Sensor [%s %d,%d] changed to %s\n",
                          flowrackName, row, col,
                          (photoState == PHOTO_ON ? "ON" : "OFF"));

      // Photo changed from OFF -> ON
      if (photoState == PHOTO_ON)
      {
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
          if (SlotObj.shouldSendCountUpFlg())
          { // Send count up to server
            asyncPostHTTP(flowrackName, COUNT_UP, row, col);
          }
          else
          {
            // Reset flag to NO Request
            SlotObj.setRequestState(NO_REQUEST);
          }
        }
        else if (transferType == SLOT_RECEIVE_TYPE)
        {
          // Release hold flg
          if (SlotObj.shouldSendReleaseHoldFlg())
          {
            SlotObj.setRequestState(NO_REQUEST);
            // Send count up to server
            asyncPostHTTP(flowrackName, NO_HOLD, row, col);
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
          if (SlotObj.shouldSendHoldFlg(currentTimestamp))
          {
            SlotObj.setRequestState(REQUEST_HOLD);
            asyncPostHTTP(flowrackName, HOLD, row, col);
          }
        }
        else if (transferType == SLOT_SEND_TYPE)
        {
          if (SlotObj.shouldSendCountUpOnHoldSendSlot(currentTimestamp))
          {
            SlotObj.setRequestState(REQUEST_HOLD);
            asyncPostHTTP(flowrackName, COUNT_UP, row, col);
          }
        }
      }
      else
      {
        if (transferType == SLOT_RECEIVE_TYPE)
        {
          if (SlotObj.shouldSendReleaseHoldFlg())
          {
            SlotObj.setRequestState(NO_REQUEST);
            // Send count up to server
            asyncPostHTTP(flowrackName, NO_HOLD, row, col);
          }
        }
      }
    }
  }
}

void connectNetwork()
{
  DEBUG_SERIAL.println("\n===== connectNetwork =====");
  DEBUG_SERIAL.print("ESP MAC: ");
  DEBUG_SERIAL.println(WiFi.macAddress());

  IPAddress local_IP(flowrackIp[0], flowrackIp[1], flowrackIp[2], flowrackIp[3]);
  IPAddress gateway(192, 168, 1, 2);
  IPAddress subnet(255, 255, 255, 0);

  if (!WiFi.config(local_IP, gateway, subnet))
  {
    DEBUG_SERIAL.println("❌ STA config failed");
  }
  else
  {
    DEBUG_SERIAL.println("✅ STA config success");
  }

  int freeBefore = ESP.getFreeHeap();
  DEBUG_SERIAL.printf("📊 Heap before scan: %d\n", freeBefore);

  int bestRSSI = -1000;
  int bestIndex = -1;
  String preferredSSID = "AMR_TDEM_2.4G";

  if (freeBefore > 25000)
  {
    DEBUG_SERIAL.println("📶 Scanning for WiFi networks...");
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++)
    {
      String ssidFound = WiFi.SSID(i);
      int rssi = WiFi.RSSI(i);
      if (ssidFound == preferredSSID && rssi > bestRSSI)
      {
        bestRSSI = rssi;
        bestIndex = i;
      }
    }

    int freeAfterScan = ESP.getFreeHeap();
    DEBUG_SERIAL.printf("📉 Heap after scan: %d\n", freeAfterScan);

    // 🟡 ใส่ตรงนี้!
    WiFi.scanDelete(); // ✅ ล้างข้อมูล scan ออก → คืน heap
    DEBUG_SERIAL.println("🧹 scanDelete() called.");

    DEBUG_SERIAL.println("🕒 Waiting 5 seconds for heap to recover...");
    delay(5000); // 🧘 wait to allow internal WiFi scan cleanup
    yield();     // ✅ เพิ่มหลัง delay เพื่อ clear watchdog

    int heapRecovered = ESP.getFreeHeap();
    DEBUG_SERIAL.printf("📈 Heap after delay: %d\n", heapRecovered);
  }
  else
  {
    DEBUG_SERIAL.println("⚠️ Heap too low, skipping WiFi scan");
  }

  if (bestIndex != -1)
  {
    uint8_t *bssid = WiFi.BSSID(bestIndex);
    DEBUG_SERIAL.printf("👉 Connecting to best '%s' RSSI=%d\n", preferredSSID.c_str(), bestRSSI);
    WiFi.begin(preferredSSID.c_str(), password, WiFi.channel(bestIndex), bssid);
  }
  else
  {
    DEBUG_SERIAL.println("⚠️ No preferred SSID found. Connecting using default.");
    WiFi.begin(ssid, password);
  }

  int retry = 0;
  DEBUG_SERIAL.print("⏳ Connecting");
  while (WiFi.status() != WL_CONNECTED && retry < 20)
  {
    delay(500);
    DEBUG_SERIAL.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    retry++;
  }

  DEBUG_SERIAL.println("\n✅ WiFi Connected!");
  DEBUG_SERIAL.print("IP Address: ");
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

void asyncPostHTTP(char flowrack[], RequestCounterState state, U1 row, U1 col)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    DEBUG_SERIAL.println("WiFi not connected");
    return;
  }

  auto *request = new AsyncHTTPRequest();
  if (!request)
  {
    DEBUG_SERIAL.println("Failed to allocate request");
    return;
  }

  String jsonPayload = "{\"station_code\":\"" + String(currentHP) +
                       "\",\"flowrack_name\":\"" + flowrack +
                       "\",\"state\":" + state +
                       ",\"row\":" + row +
                       ",\"col\":" + col +
                       ",\"running_num\":" + String(requestCounterId++) + "}";

  DEBUG_SERIAL.println("Sending HTTP POST to: " + jsonPayload);

  if (request->readyState() == readyStateUnsent || request->readyState() == readyStateDone)
  {
    request->setTimeout(5000);
    // Attach a lambda to clean up after the request completes
    request->onReadyStateChange([request](void *arg, AsyncHTTPRequest *req, int readyState)
                                {
    if (readyState == readyStateDone)
    {
        DEBUG_SERIAL.println("Request done, clean up request");
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
