
typedef unsigned char U1;
typedef unsigned int U2;
typedef unsigned long U4;
typedef unsigned long long U64;
typedef bool BL;




#define DEBUG  // << เปิด debug ได้/ปิด debug ได้ด้วยการ comment บรรทัดนี้




#ifdef DEBUG
  #define DEBUG_SERIAL Serial
#else
  #define DEBUG_SERIAL if(false) Serial
#endif

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
#include <vector>
#include <map>
// #include <WebSerial.h>
#include <stdlib.h>  // สำหรับ random()


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
const char *ssid = "AMR_TDEM";
const char *password = "1234@abcd";
const char *serverIP = "192.168.1.13"; // Server IP address
byte flowrackIp[4] = {192, 168, 1, 92};  // สุ่มเลขไม่ให้ซ้ำ /จภ
byte gateway[4] = {192, 168, 1, 2};
byte subnet[4] = {255, 255, 255, 0};
const char *serverPort = "4499"; // Server port

const int MAX_ACTIVE_REQUESTS = 6;  // ✅ เปลี่ยนจาก 4 เป็น 6


// Time control
U64 initialTimestamp = 0; // Timestamp received from the server
U64 millisAtSync = 0;     // millis() value when timestamp was received
U64 updatedTimestamp = 0;

// MCP check
U64 lastMcpCheckTime = 0;
U64 mcpResetInterval = 300000; // รีเซ็ต MCP ทุก 5 นาที
bool shouldRetryAll = false;




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
U4 lastFullCheckTime = 0;   // เก็บเวลาล่าสุดที่เช็ค FULL flag
U4 fullCheckDelay = 500;    // ตั้งค่าตรวจสอบ FULL ทุก 500ms

// Sensor tracking and flag variables
U4 lastActiveTime[28] = {0};
bool isSensorBroken[28] = {false};
int prevSensorStates[28] = {0};
BL boxDetected[28] = {0};
int lastKnownTotal[28] = {0}; // Array to store the last known totals for sensors
int activeRequests = 0;


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

U2 SENSOR_HOLD_CHECKED = 10000; // 10 sec
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



const int interruptInterval = 2000; // Interval in milliseconds
volatile bool interruptFlag = false;

const int MAX_SLOTS = 4;
const int MAX_FLOWRACKS = 10;

// Function Declarations
void connectNetwork_safe();
int readHPFromDipSwitch();
void handlePhotoState(U2 input_channel_1, U2 input_channel_2, U64 currentTimestamp);
SlotType getSlotType(String slotTypeName);
String getFlowrackDataFromServer(String HP);
SlotTransferType getSlotTransferType(String slotTransferTypeName);
void asyncPostHTTP(String flowrack, RequestCounterState state, int row, int col, bool allowRetry = true);
void onRequestPostHTTPComplete(void *arg, AsyncHTTPRequest *req, int readyState);
void setupOTA();
void retryFailedRequests();  // ✅ เพิ่มตรงนี้


// === Global flags to replace std::map ===
bool alreadySentHold[33] = {0};     // index 1-32
bool alreadySentNoHold[33] = {0};   // index 1-32

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
  U64 lastPostTime = 0;  // สำหรับ pacing per slot
  U64 lastItemInTime = 0;  // ✅ เวลาที่ ITEM_IN ถูกส่งล่าสุด
  
  
  // std::map<int, bool> alreadySentHold;   // ✅ แยก flag ตาม `sensorReadIndex`
  // std::map<int, bool> alreadySentNoHold; // ✅ แยก flag ตาม `sensorReadIndex`
  
  int test_counter = 0;
  int activeRequests = 0;  // 🔁 ตัวนับ requests ที่ยังไม่จบ

public:

  // ✅ ใช้ enum ที่ถูกต้อง
  RequestCounterState lastRoundRobinState;

  bool isSameAsLastRoundRobin(RequestCounterState state) {
    return lastRoundRobinState == state;
  }
  void markRoundRobinState(RequestCounterState state) {
    lastRoundRobinState = state;
  }


  U64 getLastPostTime() {
    return lastPostTime;
  }

  
  int getPhotoState() {
    return this->currentSensorState;
  }

  bool isPhotoOn() {
    return this->currentSensorState == PHOTO_ON;
  }

  bool canPostNow() {
    return (millis() - lastPostTime > 150);  // เว้นอย่างน้อย 150ms
  }

  void updateLastPostTime() {
    lastPostTime = millis();
  }

  // ✅ Getter & Setter สำหรับ `alreadySentHold`
  bool getAlreadySentHold(int sensorIndex) {
    return (sensorIndex >= 1 && sensorIndex <= 32) ? alreadySentHold[sensorIndex] : false;
  }
  void setAlreadySentHold(int sensorIndex, bool value) {
    if (sensorIndex >= 1 && sensorIndex <= 32) alreadySentHold[sensorIndex] = value;
  }

  // ✅ Getter & Setter สำหรับ `alreadySentNoHold`
  bool getAlreadySentNoHold(int sensorIndex) {
    return (sensorIndex >= 1 && sensorIndex <= 32) ? alreadySentNoHold[sensorIndex] : false;
  }
  void setAlreadySentNoHold(int sensorIndex, bool value) {
    if (sensorIndex >= 1 && sensorIndex <= 32) alreadySentNoHold[sensorIndex] = value;
  }

  bool isSendingCountUp = false;
  bool getIsSendingCountUp() {
    return this->isSendingCountUp;
  }

  bool isSendingHold = false;
  bool isSendingNoHold = false;

  bool getIsSendingHold() {
    return this->isSendingHold;
  }
  void setIsSendingHold(bool val) {
    this->isSendingHold = val;
  }

  bool getIsSendingNoHold() {
    return this->isSendingNoHold;
  }
  void setIsSendingNoHold(bool val) {
    this->isSendingNoHold = val;
  }


  void setIsSendingCountUp(bool val) {
    this->isSendingCountUp = val;
  }

  bool isSendingPost = false;
  bool getIsSendingPost() { return this->isSendingPost; }
  void setIsSendingPost(bool val) { this->isSendingPost = val; }

  U4 lastSentTimestamp = 0;  // เพิ่มใน Slot class

  bool canSendRequest(U64 now) {
    return (now - lastSentTimestamp > 2000); // หน่วง 2 วิ
  }

  void markSent(U64 now) {
    lastSentTimestamp = now;
  }
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

    // 📌 ถ้า MCP23S17 ไม่ตอบสนอง → ข้ามการอัปเดตค่า
    if (inputchip1.digitalRead() == 0xFFFF && inputchip2.digitalRead() == 0xFFFF) 
    {
        DEBUG_SERIAL.println("⚠ ERROR: MCP23S17 not detected! Skipping setPhotoState...");
        return;
    }


    // 🧪 Log เวลาก่อนเซนเซอร์เปลี่ยนจาก ON → OFF
    if (this->currentSensorState == PHOTO_ON && photoState == PHOTO_OFF) {
        U64 duration = timeStamp - this->currentTimestamp;
        DEBUG_SERIAL.printf("🧪 Sensor was ON for %.2f sec before turning OFF (%s [%d,%d])\n",
                            duration / 1000.0, this->flowrackName.c_str(), this->row, this->col);
    }

    this->previousSensorState = this->currentSensorState;
    this->previousTimestamp = this->currentTimestamp;
    this->currentSensorState = photoState;
    this->currentTimestamp = timeStamp; // ✅ อัปเดตค่าก่อนคำนวณ Time Diff

    DEBUG_SERIAL.print("📌 [setPhotoState] Updated currentTimestamp: ");
    DEBUG_SERIAL.println(this->currentTimestamp);
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
      if (photoState == this->currentSensorState) return false;

      if (currentTimeStamp - this->currentTimestamp > SENSOR_CHANGE_CHECK_DELAY)
      {
          // 💡 ป้องกันสลับเร็วเกิน
          if (abs(photoState - this->currentSensorState) == 1) {
              if (currentTimeStamp - this->previousTimestamp < 100 || currentTimeStamp - this->currentTimestamp < 100) {
                  DEBUG_SERIAL.println("⏱ Debounce: flip too fast (dual check)");
                  return false;
              }
          }
          return true;
      }
      return false;
  }

  void markItemInTime(U64 now) {
    lastItemInTime = now;
  }

  bool shouldSendHoldFlg(U64 currentTimestamp)
  {
      int sensorIndex = this->getReadIndex();

      if (this->currentSensorState != PHOTO_ON)
          return false;

      if (this->requestState == REQUEST_HOLD)
          return false;

      if (this->getAlreadySentHold(sensorIndex))
          return false;

      // 🕒 คำนวณเวลาที่กล่องอยู่
      if (currentTimestamp - this->previousTimestamp > SENSOR_HOLD_CHECKED)
      {
          // ✅ แสดง log แค่ครั้งเดียวทุก 2 วิ
          static std::map<int, U64> lastHoldLogTime;
          if (lastHoldLogTime[sensorIndex] == 0 || millis() - lastHoldLogTime[sensorIndex] > 2000)
          {
              DEBUG_SERIAL.printf("✅ HOLD condition met! [sensorIndex=%d, time=%.1fs]\n", sensorIndex,
                                  (currentTimestamp - this->previousTimestamp) / 1000.0);
              lastHoldLogTime[sensorIndex] = millis();
          }

          this->setAlreadySentHold(sensorIndex, true);
          this->setAlreadySentNoHold(sensorIndex, false);
          return true;
      }

      return false;
  }




  bool shouldSendCountUpFlg(U64 now)
  {
    if (this->previousTimestamp == this->currentTimestamp) {
        // 💡 ยังไม่เคยมีการเปลี่ยนสถานะหลังบูต → ไม่อนุญาต COUNT_UP
        return false;
    }
    return (this->previousSensorState == PHOTO_OFF && this->currentSensorState == PHOTO_ON);
  }




  



  bool shouldSendReleaseHoldFlg(U64 currentTimestamp)
  {
    int sensorIndex = this->readIndex;

    // ✅ ป้องกันหลุด NO_HOLD
    if (this->getAlreadySentNoHold(sensorIndex))
        return false;  

    // ✅ ไม่ต้องเช็ค requestState == REQUEST_HOLD เพราะบางทีมันเปลี่ยนก่อนส่ง NO_HOLD
    if (this->currentSensorState == PHOTO_OFF) 
    {
        DEBUG_SERIAL.println("✅ Slot is free! Sending NO_HOLD.");
        this->setAlreadySentNoHold(sensorIndex, true);  // ✅ ตั้ง flag NO_HOLD
        this->setAlreadySentHold(sensorIndex, false);   // ✅ รีเซ็ต flag HOLD
        this->setRequestState(NO_REQUEST);
        return true;
    }

    return false;
  }







  bool shouldSendCountUpOnHoldSendSlot(U64 currentTimestamp)
  {
    if (currentTimestamp - this->previousTimestamp > SENSOR_HOLD_CNT_UP_CHECKED)
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





Slot* slotlist[32];  // ✅ เหมือนเดิม
U1 allSlotCount = 0; // ✅ เหมือนเดิม


// void checkFullRealTime()
// {
//     static U64 lastCheck = 0;
//     U64 currentMillis = millis();

//     if (currentMillis - lastCheck < fullCheckDelay) return;
//     lastCheck = currentMillis;

//     for (U1 cnt = 0; cnt < allSlotCount; cnt++)
//     {
//         Slot *SlotObj = slotlist[cnt];
//         int sensorIndex = SlotObj->getReadIndex();

//         if (SlotObj->getSlotTransferType() == SLOT_RECEIVE_TYPE)
//         {
//             String flowrackName = SlotObj->getFlowrackName();
//             int row = SlotObj->getRow();
//             int col = SlotObj->getCol();

//             if (SlotObj->shouldSendHoldFlg(currentMillis))
//             {
//                 DEBUG_SERIAL.println("🚨 FULL DETECTED! Sending HOLD.");
//                 asyncPostHTTP(flowrackName, HOLD, row, col);
//                 SlotObj->setRequestState(REQUEST_HOLD);
//                 SlotObj->setAlreadySentHold(sensorIndex, true);
//                 SlotObj->setAlreadySentNoHold(sensorIndex, false);
//             }
            
//             // ✅ บังคับให้เช็คทุกช่อง
//             else if (SlotObj->shouldSendReleaseHoldFlg(currentMillis))
//             {
//                 DEBUG_SERIAL.println("✅ Slot is free! Sending NO_HOLD.");
//                 asyncPostHTTP(flowrackName, NO_HOLD, row, col);
//                 SlotObj->setAlreadySentNoHold(sensorIndex, true);
//                 SlotObj->setAlreadySentHold(sensorIndex, false);
//             }
//         }
//     }
// }





void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  // SPI.pins(SCK_PIN, MISO_PIN, MOSI_PIN, D2);
  // SPI.begin();
  // Initialize SPI with custom pins

  DEBUG_SERIAL.begin(9600);
  Serial.println();
  Serial.printf("Boot reason: %d\n", ESP.getResetInfoPtr()->reason);


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
  connectNetwork_safe();
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
          Slot* newSlot = new Slot(row, col, flowrackName, isActive, slotType, slotTransferType, readIndex);
          slotlist[allSlotCount] = newSlot;
          allSlotCount++;
        }
      }
    }
    DEBUG_SERIAL.printf("✅ Total slots: %d\n", allSlotCount);
    DEBUG_SERIAL.printf("📊 Heap after creating slots: %d bytes\n", ESP.getFreeHeap());

  }
  else
  {
    DEBUG_SERIAL.println("Failed to parse JSON data.");
  }

}

struct RetryPacket {
  String payload;
  int retryCount;
  U64 lastTriedTime;

  RetryPacket(String p, int c = 0, U64 t = 0) : payload(p), retryCount(c), lastTriedTime(t) {}
};

std::vector<RetryPacket> failedRequests;  // ✅ ใช้ struct ใหม่แทน
int httpErrorMinus4Count = 0;
const int MAX_HTTP_MINUS4_BEFORE_REBOOT = 6;



bool allowRetryDrain = false;



void loop()
{
  ESP.wdtFeed();  // ✅ ป้อน watchdog ทันทีที่เข้ามาใน loop

  ArduinoOTA.handle();
  U64 current_time = millis();
  if ((current_time - lastTime) > timerDelay)
  {
    U2 input_channel_1 = ~inputchip1.digitalRead() & 0xffff;
    U2 input_channel_2 = ~inputchip2.digitalRead() & 0xffff;
    U64 updatedTimestamp = initialTimestamp + current_time;
    lastTime = current_time;

    handlePhotoState(input_channel_1, input_channel_2, updatedTimestamp);
    ESP.wdtFeed();  // ✅ feed หลังอ่าน sensor
  }

  retryFailedRequests();  // ✅ เรียก retry queue
  ESP.wdtFeed();  // ✅ feed หลัง retry

  // ✅ MCP Reset ทุก 5 นาที
  if (millis() - lastMcpCheckTime > mcpResetInterval)
  {
    lastMcpCheckTime = millis();
    inputchip1.begin();
    inputchip2.begin();
    DEBUG_SERIAL.println("🔁 Re-init MCP23S17");
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    DEBUG_SERIAL.println("🚨 WiFi Lost! Please check network.");
    digitalWrite(LED_BUILTIN, 0);
    WiFi.disconnect();
    delay(1000);
    WiFi.forceSleepWake();
    delay(200);
    connectNetwork_safe();
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }

  if (millis() % 10000 < 100) {
    DEBUG_SERIAL.print("📊 Free Heap: ");
    DEBUG_SERIAL.println(ESP.getFreeHeap());
  }

  static U64 lastHeapCheck = 0;
  static int lastHeap = 0;
  static U64 lastWiFiDebug = 0;
  if (millis() - lastWiFiDebug > 10000) {
    lastWiFiDebug = millis();
    DEBUG_SERIAL.println("📊 ===== System Summary =====");
    DEBUG_SERIAL.print("📶 WiFi.status(): ");
    DEBUG_SERIAL.println(WiFi.status());
    DEBUG_SERIAL.print("📡 Local IP: ");
    DEBUG_SERIAL.println(WiFi.localIP());
    DEBUG_SERIAL.print("📉 Free Heap: ");
    DEBUG_SERIAL.println(ESP.getFreeHeap());
    DEBUG_SERIAL.print("📦 Retry Queue Size: ");
    DEBUG_SERIAL.println(failedRequests.size());
    DEBUG_SERIAL.print("📊 Active HTTP Requests: ");
    DEBUG_SERIAL.println(activeRequests);
    DEBUG_SERIAL.println("=============================");
  }

  if (millis() - lastHeapCheck > 30000) {
    int currentHeap = ESP.getFreeHeap();
    if (currentHeap < 2000 && abs(currentHeap - lastHeap) < 100) {
      DEBUG_SERIAL.println("❌ Heap stuck or low. Rebooting...");
      ESP.restart();
    }
    lastHeapCheck = millis();
    lastHeap = currentHeap;
  }

  // ✅ ตรวจ activeRequests ค้างนานเกิน 15 วินาที พร้อมล้าง flags
  static U64 lastHttpStateChange = 0;
  static int lastKnownActiveRequests = 0;

  if (activeRequests > 0) {
    if (activeRequests != lastKnownActiveRequests) {
      lastKnownActiveRequests = activeRequests;
      lastHttpStateChange = millis(); // รีเซ็ตเมื่อมีการเปลี่ยนแปลง
    } else if (millis() - lastHttpStateChange > 15000) {
      DEBUG_SERIAL.println("🧨 activeRequests stuck >15s. Forcing clear...");
      activeRequests = 0;
      lastKnownActiveRequests = 0;
      lastHttpStateChange = millis();

      // ✅ เคลียร์ flags ของทุก slot
      for (U1 i = 0; i < allSlotCount; i++) {
        Slot* slot = slotlist[i];

        if (slot->getIsSendingCountUp() || slot->getIsSendingHold() || slot->getIsSendingNoHold()) {
          DEBUG_SERIAL.printf("🧹 Cleared stuck flags: %s [%d,%d] → C:%d H:%d NH:%d\n",
            slot->getFlowrackName().c_str(),
            slot->getRow(), slot->getCol(),
            slot->getIsSendingCountUp(), slot->getIsSendingHold(), slot->getIsSendingNoHold());
        }

        slot->setIsSendingCountUp(false);
        slot->setIsSendingHold(false);
        slot->setIsSendingNoHold(false);
      }

      DEBUG_SERIAL.println("✅ All slot sending flags cleared.");
    }
  } else {
    lastHttpStateChange = millis();  // รีเซ็ตเมื่อไม่มี active request
  }

  yield(); // ป้องกัน watchdog reset
}







U64 lastFullCheck = 0;
U64 lastRoundRobinTime = 0;
U4 roundRobinIndex = 0;

void handlePhotoState(U2 input_channel_1, U2 input_channel_2, U64 currentTimestamp) {
  // ✅ Check MCP valid
  U64 mcpStart = millis();
  while ((input_channel_1 == 0xFFFF && input_channel_2 == 0xFFFF) && millis() - mcpStart < 100) {
    input_channel_1 = ~inputchip1.digitalRead() & 0xffff;
    input_channel_2 = ~inputchip2.digitalRead() & 0xffff;
    yield();
  }

  if (input_channel_1 == 0xFFFF && input_channel_2 == 0xFFFF) {
    DEBUG_SERIAL.println("⚠️ MCP still unresponsive after retries. Skipping...");
    return;
  }

  DEBUG_SERIAL.printf("📟 input_channel_1: %04X input_channel_2: %04X\n", input_channel_1, input_channel_2);


  U4 inputMergeChannel = (input_channel_2 << 16) | input_channel_1;

  static std::map<int, int> rapidOnCount;
  static std::map<int, U64> lastRapidTime;
  static std::map<int, U64> lastStuckLogTime;
  static std::map<int, U64> lastCountUpRetryTime;

  bool allowRoundRobin = (millis() - lastRoundRobinTime >= 30000);


  // ================= Loop through slots =================
  for (U1 cnt = 0; cnt < allSlotCount; cnt++) {
    Slot *slot = slotlist[cnt];
    int rIndex = slot->getReadIndex();
    if (rIndex <= 0 || rIndex > 32) continue;

    U1 photoState = (inputMergeChannel >> (rIndex - 1)) & 0x01;
    SlotTransferType transferType = slot->getSlotTransferType();
    String flowrackName = slot->getFlowrackName();
    int row = slot->getRow();
    int col = slot->getCol();

    // ✅ เพิ่ม log sensor แต่ละช่อง
    DEBUG_SERIAL.printf("📌 Sensor[%s %d,%d] rIndex=%d | raw=%d | prev=%d\n",
                        flowrackName.c_str(), row, col, rIndex,
                        photoState, slot->getCurrentSensorState());

    // ✅ Rapid ON protection (SEND_TYPE)
    if (photoState == PHOTO_OFF) {
      rapidOnCount[rIndex] = 0;
    }

    if (transferType == SLOT_SEND_TYPE && photoState == PHOTO_ON) {
      U64 now = millis();
      if (now - lastRapidTime[rIndex] < 1000) {
        rapidOnCount[rIndex]++;
        if (rapidOnCount[rIndex] > 3 && now - slot->getCurrentTimestamp() < 5000) {
          DEBUG_SERIAL.printf("🚫 Rapid ON at rIndex %d. Skipping COUNT_UP.\n", rIndex);
          continue;
        }
      } else {
        rapidOnCount[rIndex] = 1;
        lastRapidTime[rIndex] = now;
      }
    }

    // ✅ Retry COUNT_UP if stuck
    if (slot->getIsSendingCountUp() && millis() - slot->getLastPostTime() > 5000) {
      if (millis() - lastCountUpRetryTime[rIndex] > 60000) {
        if (millis() - lastStuckLogTime[rIndex] > 5000) {
          DEBUG_SERIAL.printf("⚠️ ON stuck >60s → retry at rIndex %d\n", rIndex);
          lastStuckLogTime[rIndex] = millis();
        }
        slot->setIsSendingCountUp(false);
        lastCountUpRetryTime[rIndex] = millis();
      }
    }

    // ✅ Sensor changed
    if (slot->isSensorStateChanged(photoState, currentTimestamp)) {


      DEBUG_SERIAL.printf("🧐 Sensor CHANGED [%s %d,%d] rIndex=%d | %d → %d\n",
                          flowrackName.c_str(), row, col, rIndex,
                          slot->getCurrentSensorState(), photoState);

      

      // [ข้อ 2] เพิ่ม debug แจ้งเตือนถ้า flip เร็วผิดปกติ
      U64 tsDiff = currentTimestamp - slot->getPreviousTimestamp();
      if (tsDiff < 150 && slot->getPreviousSensorState() != photoState) {
        DEBUG_SERIAL.printf("⚡ Glitch alert: sensor [%s %d,%d] flip too fast (%.1f ms)\n",
                            flowrackName.c_str(), row, col, tsDiff * 1.0);
      }
      // หรือจะต่อด้วยการ skip
      if (tsDiff < 150 && slot->getPreviousSensorState() != photoState) {
        DEBUG_SERIAL.printf("⏱ Glitch ignored on [%s %d,%d] → skip toggle\n",
                            flowrackName.c_str(), row, col);
        continue;
      }

      slot->setPhotoState(photoState, currentTimestamp);
      DEBUG_SERIAL.printf("📌 [setPhotoState] Sensor [%s %d,%d] changed to %s\n",
                          flowrackName.c_str(), row, col,
                          (photoState == PHOTO_ON ? "ON" : "OFF"));

      if (photoState == PHOTO_ON) {
        if (transferType == SLOT_SEND_TYPE && slot->shouldSendCountUpFlg(currentTimestamp)) {
          if (!slot->getIsSendingCountUp() && slot->canPostNow() && activeRequests < 2) {
            slot->setIsSendingCountUp(true);
            slot->updateLastPostTime();
            DEBUG_SERIAL.printf("🔼 [SEND_TYPE] COUNT_UP: %s [%d,%d]\n", flowrackName.c_str(), row, col);
            asyncPostHTTP(flowrackName, COUNT_UP, row, col);
          }
        }


        if (tsDiff < 300) {
          DEBUG_SERIAL.printf("🧪 [False ON Filter] Sensor [%s %d,%d] ON < 300ms → Ignore (%.1f ms)\n",
              flowrackName.c_str(), row, col, tsDiff * 1.0);
          continue; // ✅ ข้าม false ON
        }


        if (transferType == SLOT_RECEIVE_TYPE && slot->canPostNow() && !slot->getIsSendingPost() && activeRequests < MAX_ACTIVE_REQUESTS - 1  &&
          (currentTimestamp - slot->getPreviousTimestamp()) > 200) {
          slot->updateLastPostTime();
          slot->markItemInTime(currentTimestamp);  // ✅ บันทึกเวลา ITEM_IN

          // ✅ เพิ่ม debug ตรงนี้
          DEBUG_SERIAL.printf("🚨 [ITEM_IN Debug] sensorIndex=%d | photoState=%d | prev=%d | ts_diff=%llu\n",
            slot->getReadIndex(),
            photoState,
            slot->getPreviousSensorState(),
            tsDiff

          );


          DEBUG_SERIAL.printf("📥 [RECEIVE_TYPE] ITEM_IN: %s [%d,%d]\n", flowrackName.c_str(), row, col);
          slot->setIsSendingPost(true);  // ✅ ป้องกัน POST ซ้อน
          asyncPostHTTP(flowrackName, ITEM_IN, row, col);
          delay(20);
          yield();
        }

      } else {
        // ✅ Sensor OFF → NO_HOLD
        if (transferType == SLOT_RECEIVE_TYPE && slot->shouldSendReleaseHoldFlg(currentTimestamp)) {
          if (!slot->getIsSendingNoHold() && slot->canPostNow() && activeRequests < 3) {
            slot->setIsSendingNoHold(true);
            slot->setIsSendingHold(false);
            slot->updateLastPostTime();
            DEBUG_SERIAL.printf("✅ Slot is free! Sending NO_HOLD.\n");
            asyncPostHTTP(flowrackName, NO_HOLD, row, col);
          }
        }
      }
    }

    // ✅ Check for HOLD
    if (transferType == SLOT_RECEIVE_TYPE && slot->shouldSendHoldFlg(currentTimestamp)) {
      if (!slot->getIsSendingHold() && slot->canPostNow() && !slot->getIsSendingPost() && activeRequests < 3) {
        slot->setIsSendingHold(true);
        slot->setIsSendingNoHold(false);
        slot->updateLastPostTime();
        DEBUG_SERIAL.printf("✅ HOLD condition met! Sending FULL signal.\n");
        slot->setIsSendingPost(true);

        asyncPostHTTP(flowrackName, HOLD, row, col);
      }
    }

    yield();
  }





  // ✅ RoundRobin FULL/NO_HOLD (1 slot per 5s)
  if (allowRoundRobin && allSlotCount > 0) {
    uint8_t roundRobinAttempt = 0;
    for (int loop = 0; loop < allSlotCount; loop++) {
      int idx = roundRobinIndex % allSlotCount;
      Slot *slot = slotlist[idx];
      roundRobinIndex = (roundRobinIndex + 1) % allSlotCount;

      if (slot == nullptr || slot->getSlotTransferType() != SLOT_RECEIVE_TYPE) continue;

      int rIndex = slot->getReadIndex();
      U1 photoState = (inputMergeChannel >> (rIndex - 1)) & 0x01;
      String flowrackName = slot->getFlowrackName();
      int row = slot->getRow();
      int col = slot->getCol();

      DEBUG_SERIAL.printf("📊 [RoundRobin] Checking %s [%d,%d] = %s\n",
                          flowrackName.c_str(), row, col,
                          (photoState == PHOTO_ON ? "Full" : "NoFull"));


        // 🧪 เพิ่ม log ค้าง ON นานเท่าไหร่
      if (photoState == PHOTO_ON) {
        U64 timeHeld = currentTimestamp - slot->getPreviousTimestamp();
        DEBUG_SERIAL.printf("🧪 [Hold Debug] %s [%d,%d] ON for %.1f sec\n", flowrackName.c_str(), row, col, timeHeld / 1000.0);
      }

      if (slot->canPostNow() && activeRequests < 4) {
        RequestCounterState state = (photoState == PHOTO_ON) ? HOLD : NO_HOLD;
        const char* statusText = (state == HOLD) ? "FULL" : "NoFull";



        // ✅ เพิ่ม filter ว่า state ไม่ควรซ้ำกับที่ส่งล่าสุด
        if (!slot->isSameAsLastRoundRobin(state)) {
          DEBUG_SERIAL.printf("📤 [RoundRobin] Sending %s: %s [%d,%d]\n", statusText, flowrackName.c_str(), row, col);
          asyncPostHTTP(flowrackName, state, row, col, false);
          slot->updateLastPostTime();
          slot->markRoundRobinState(state);  // 🧠 จำ state ล่าสุด
          lastRoundRobinTime = millis();
          break;
        } else {
          DEBUG_SERIAL.printf("🔁 [RoundRobin] Already sent same %s recently → Skip: %s [%d,%d]\n", statusText, flowrackName.c_str(), row, col);
        }
      } else {
        DEBUG_SERIAL.printf("⏳ [RoundRobin] Skipped due to pacing or activeRequests: %s [%d,%d]\n", flowrackName.c_str(), row, col);
        roundRobinAttempt++;
      }

      if (roundRobinAttempt >= allSlotCount) {
        DEBUG_SERIAL.println("⚠️ [RoundRobin] All slots skipped due to pacing or activeRequests.");
        lastRoundRobinTime = millis();  // ✅ force reset รอบใหม่
        break;
      }

      yield();
    }
  }

  // ✅ Heap monitor (every 10s)
  static U4 lastHeapCheckTime = 0;
  if (millis() - lastHeapCheckTime > 10000) {
    lastHeapCheckTime = millis();
    DEBUG_SERIAL.printf("🧠 [HEAP_MONITOR] Free Heap: %d | Retry Queue: %d | activeRequests: %d\n",
                        ESP.getFreeHeap(), failedRequests.size(), activeRequests);
    if (ESP.getFreeHeap() < 25000) {
      DEBUG_SERIAL.printf("⚠️ [HEAP LOW] Free Heap critically low: %d\n", ESP.getFreeHeap());
    }
  }

  yield();  // Ensure WDT does not trigger
}










void resetAlreadySentFlags(int sensorIndex)
{
    for (U1 cnt = 0; cnt < allSlotCount; cnt++)
    {
        Slot *SlotObj = slotlist[cnt];

        if (SlotObj->getReadIndex() == sensorIndex)
        {
            SlotObj->setAlreadySentHold(sensorIndex, false);   // ✅ รีเซ็ต flag เมื่อเซ็นเซอร์เป็น PHOTO_OFF
            SlotObj->setAlreadySentNoHold(sensorIndex, false); 
        }
    }
}







void connectNetwork_safe() {
  DEBUG_SERIAL.println("\n===== connectNetwork_safe() =====");
  DEBUG_SERIAL.print("ESP MAC: ");
  DEBUG_SERIAL.println(WiFi.macAddress());

  IPAddress local_IP(flowrackIp[0], flowrackIp[1], flowrackIp[2], flowrackIp[3]);
  IPAddress gateway(192, 168, 1, 2);
  IPAddress subnet(255, 255, 255, 0);

  if (!WiFi.config(local_IP, gateway, subnet)) {
    DEBUG_SERIAL.println("❌ STA config failed");
  } else {
    DEBUG_SERIAL.println("✅ STA config success");
  }

  int freeBefore = ESP.getFreeHeap();
  DEBUG_SERIAL.printf("📊 Heap before scan: %d\n", freeBefore);

  int bestRSSI = -1000;
  int bestIndex = -1;
  String preferredSSID = "AMR_TDEM_2.4G";

  if (freeBefore > 25000) {
    DEBUG_SERIAL.println("📶 Scanning for WiFi networks...");
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++) {
      String ssidFound = WiFi.SSID(i);
      int rssi = WiFi.RSSI(i);
      if (ssidFound == preferredSSID && rssi > bestRSSI) {
        bestRSSI = rssi;
        bestIndex = i;
      }
    }

    int freeAfterScan = ESP.getFreeHeap();
    DEBUG_SERIAL.printf("📉 Heap after scan: %d\n", freeAfterScan);

      // 🟡 ใส่ตรงนี้!
    WiFi.scanDelete();  // ✅ ล้างข้อมูล scan ออก → คืน heap
    DEBUG_SERIAL.println("🧹 scanDelete() called.");

    DEBUG_SERIAL.println("🕒 Waiting 10 seconds for heap to recover...");
    delay(10000);  // 🧘 wait to allow internal WiFi scan cleanup
    yield();  // ✅ เพิ่มหลัง delay เพื่อ clear watchdog

    int heapRecovered = ESP.getFreeHeap();
    DEBUG_SERIAL.printf("📈 Heap after delay: %d\n", heapRecovered);
  } else {
    DEBUG_SERIAL.println("⚠️ Heap too low, skipping WiFi scan");
  }

  if (bestIndex != -1) {
    uint8_t* bssid = WiFi.BSSID(bestIndex);
    DEBUG_SERIAL.printf("👉 Connecting to best '%s' RSSI=%d\n", preferredSSID.c_str(), bestRSSI);
    WiFi.begin(preferredSSID.c_str(), password, WiFi.channel(bestIndex), bssid);
  } else {
    DEBUG_SERIAL.println("⚠️ No preferred SSID found. Connecting using default.");
    WiFi.begin(ssid, password);
  }

  int retry = 0;
  DEBUG_SERIAL.print("⏳ Connecting");
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
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

  // ✅ เริ่ม OTA หลัง WiFi สำเร็จแล้ว
  setupOTA();  
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




U4 requestCounterId = 1;  // เริ่มต้นที่ 1 ทุกครั้งที่บูต

// ✅ เพิ่ม activeRequests limit เป็น 6 ทั่วทั้งระบบ

// ---------------------- CONFIG ----------------------

void asyncPostHTTP(String flowrack, RequestCounterState state, int row, int col, bool allowRetry)
{
  DEBUG_SERIAL.printf("📤 asyncPostHTTP: %s [%d,%d]\n", flowrack.c_str(), row, col);

  if (inputchip1.digitalRead() == 0xFFFF && inputchip2.digitalRead() == 0xFFFF) {
    DEBUG_SERIAL.println("⚠️ MCP23S17 not detected! Skipping HTTP request...");
    return;
  }

  if (ESP.getFreeHeap() < 12000) {
    DEBUG_SERIAL.printf("⚠️ Heap too low (%d) before creating payload. Skipping.\n", ESP.getFreeHeap());
    return;
  }

  // ⛑️ สร้าง JSON ด้วย buffer (ใช้ RAM ต่ำกว่า String concat)
  char buffer[180];
  snprintf(buffer, sizeof(buffer),
           "{\"station_code\":\"%d\",\"flowrack_name\":\"%s\",\"state\":%d,\"row\":%d,\"col\":%d,\"running_num\":%lu}",
           currentHP, flowrack.c_str(), state, row, col, requestCounterId++);

  // String jsonPayload = String(buffer);  // สร้าง String เดียว ไม่ต่อหลายครั้ง
  String jsonPayload = String(buffer); 

  if (WiFi.status() != WL_CONNECTED) {
    DEBUG_SERIAL.println("🚨 WiFi disconnected, adding to retry queue.");
    failedRequests.push_back(RetryPacket(jsonPayload, 0));
    if (failedRequests.size() > 100) {
      DEBUG_SERIAL.println("⚠️ Retry queue overflow. Clearing.");
      failedRequests.clear();
    }
    return;
  }

  if (activeRequests >= MAX_ACTIVE_REQUESTS) {
    DEBUG_SERIAL.printf("🚦 Too many requests (%d), deferring.\n", activeRequests);
    failedRequests.push_back(RetryPacket(jsonPayload, 0));
    return;
  }

  delay(10 + random(20));  // 🧘 ลด burst
  yield();

  AsyncHTTPRequest *request = new AsyncHTTPRequest();
  if (!request) {
    DEBUG_SERIAL.println("❌ Failed to allocate request object.");
    failedRequests.push_back(RetryPacket(jsonPayload, 0));
    return;
  }

  request->setTimeout(5000);
  activeRequests++;
  DEBUG_SERIAL.printf("🌐 Posting: %s\n", jsonPayload.c_str());


  // 🔁 ทำสำเนาตัวแปรไว้ก่อนเข้า lambda (ป้องกัน dangling reference)
  String flowrack_copy = flowrack;
  int row_copy = row;
  int col_copy = col;

  request->onReadyStateChange([request, jsonPayload, flowrack, row, col, allowRetry](void *arg, AsyncHTTPRequest *req, int readyState) {
    if (readyState != readyStateDone) return;
    activeRequests--;
    int code = req->responseHTTPcode();
    DEBUG_SERIAL.printf("✅ HTTP Done [%d] | Active left: %d\n", code, activeRequests);

    if (code == -4) {
      httpErrorMinus4Count++;
      if (httpErrorMinus4Count >= MAX_HTTP_MINUS4_BEFORE_REBOOT) {
        DEBUG_SERIAL.println("💣 Too many -4 errors. Rebooting...");
        delay(500);
        ESP.restart();
      }
    } else {
      httpErrorMinus4Count = 0;
    }

    // Clear slot state
    for (U1 i = 0; i < allSlotCount; i++) {
      Slot *slot = slotlist[i];
      if (slot->getFlowrackName() == flowrack && slot->getRow() == row && slot->getCol() == col) {
        slot->setIsSendingCountUp(false);
        slot->setIsSendingHold(false);
        slot->setIsSendingNoHold(false);
        slot->setIsSendingPost(false);  // ✅ เพิ่มบรรทัดนี้

        break;
      }
    }

    if (code != 200) {
      if ((code == -4 || code == -1 || code == 0) && allowRetry) {
        failedRequests.push_back(RetryPacket(jsonPayload, 0));
      }
    }

    delete request;
  });

  request->open("POST", serverPostURL.c_str());
  request->setReqHeader("Content-Type", "application/json");
  delay(30 + random(50));
  yield();

  if (!request->send(buffer)) {
    DEBUG_SERIAL.println("❌ send() failed");
    activeRequests--;
    failedRequests.push_back(RetryPacket(jsonPayload, 0));
    delete request;
  }
}



// ======================= [REVISED] retryFailedRequests() =======================
void retryFailedRequests() {
  static U64 lastDrainTime = 0;

  if (!allowRetryDrain || failedRequests.empty()) return;
  if (activeRequests >= MAX_ACTIVE_REQUESTS) return;
  if (millis() - lastDrainTime < 1000) return;

  RetryPacket packet = failedRequests.front();
  failedRequests.erase(failedRequests.begin());
  lastDrainTime = millis();

  if (packet.retryCount >= 3) {
    DEBUG_SERIAL.println("\xE2\x9D\x8C Max retry exceeded. Dropping packet.");
    return;
  }

  DEBUG_SERIAL.printf("♻️ Retrying: %s\n", packet.payload.substring(0, 80).c_str());

  AsyncHTTPRequest *request = new AsyncHTTPRequest();
  request->setTimeout(5000);
  activeRequests++;

  request->onReadyStateChange([request, packet](void *arg, AsyncHTTPRequest *req, int readyState) mutable {
    if (readyState != readyStateDone) return;
    int code = req->responseHTTPcode();
    if (activeRequests > 0) activeRequests--;

    if (code == 200) {
      DEBUG_SERIAL.println("✅ Retry success.");
    } else if (code == -4 || code == -1 || code == 0) {
      failedRequests.push_back({packet.payload, packet.retryCount + 1});
      DEBUG_SERIAL.println("⚠️ Retry failed. Requeued.");
    }

    delete request;
  });

  if (request->open("POST", serverPostURL.c_str())) {
    request->setReqHeader("Content-Type", "application/json");
    delay(20 + random(30));
    yield();
    request->send(packet.payload);
  } else {
    failedRequests.push_back({packet.payload, packet.retryCount + 1});
    activeRequests--;
    delete request;
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
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    DEBUG_SERIAL.print(F("Start updating "));
    DEBUG_SERIAL.println(type);
  });

  ArduinoOTA.onEnd([]() {
    DEBUG_SERIAL.println(F("\nEnd"));
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG_SERIAL.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG_SERIAL.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) DEBUG_SERIAL.println(F("Auth Failed"));
    else if (error == OTA_BEGIN_ERROR) DEBUG_SERIAL.println(F("Begin Failed"));
    else if (error == OTA_CONNECT_ERROR) DEBUG_SERIAL.println(F("Connect Failed"));
    else if (error == OTA_RECEIVE_ERROR) DEBUG_SERIAL.println(F("Receive Failed"));
    else if (error == OTA_END_ERROR) DEBUG_SERIAL.println(F("End Failed"));
  });

  ArduinoOTA.begin();
}
