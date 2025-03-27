
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
byte flowrackIp[4] = {192, 168, 1, 204};  // สุ่มเลขไม่ให้ซ้ำ
byte gateway[4] = {192, 168, 1, 2};
byte subnet[4] = {255, 255, 255, 0};
const char *serverPort = "4499"; // Server port

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
int activeRequests = 0;  // 🔁 ตัวนับ requests ที่ยังไม่จบ

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
  
  std::map<int, bool> alreadySentHold;   // ✅ แยก flag ตาม `sensorReadIndex`
  std::map<int, bool> alreadySentNoHold; // ✅ แยก flag ตาม `sensorReadIndex`

public:
  
  // ✅ Getter & Setter สำหรับ `alreadySentHold`
  bool getAlreadySentHold(int sensorIndex) {
        return alreadySentHold.count(sensorIndex) ? alreadySentHold[sensorIndex] : false;
    }
  void setAlreadySentHold(int sensorIndex, bool value) {
      alreadySentHold[sensorIndex] = value;
  }

  // ✅ Getter & Setter สำหรับ `alreadySentNoHold`
  bool getAlreadySentNoHold(int sensorIndex) {
        return alreadySentNoHold.count(sensorIndex) ? alreadySentNoHold[sensorIndex] : false;
    }
  void setAlreadySentNoHold(int sensorIndex, bool value) {
      alreadySentNoHold[sensorIndex] = value;
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
    int sensorIndex = this->readIndex;

    if (this->currentSensorState != PHOTO_ON)
    {
        return false;
    }

    if (this->requestState == REQUEST_HOLD)
    {
        return false;
    }

    if (this->getAlreadySentHold(sensorIndex))  
    {
        return false;
    }    

    if (currentTimestamp - this->previousTimestamp > SENSOR_HOLD_CHECKED) // ใช้ currentTimestamp และต้องอัปเดต previousTimestamp
    {
        DEBUG_SERIAL.println("✅ HOLD condition met! Sending FULL signal.");
        this->setAlreadySentHold(sensorIndex, true);
        this->setAlreadySentNoHold(sensorIndex, false); // ✅ รีเซ็ต NO_HOLD
        // this->previousTimestamp = currentTimestamp; // ✅ ป้องกันการส่งซ้ำ
        return true;
    }

    return false;
  }



  bool shouldSendCountUpFlg()
  {
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

Slot *slotlist[32];
U1 allSlotCount = 0;


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

struct RetryPacket {
  String payload;
  int retryCount;

  RetryPacket(String p, int c = 0) : payload(p), retryCount(c) {}
};

std::vector<RetryPacket> failedRequests;  // ✅ ใช้ struct ใหม่แทน

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


  //checkFullRealTime();

  retryFailedRequests();  // ✅ เรียก retry queue ที่นี่!


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
    connectNetwork();
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
    DEBUG_SERIAL.print("📶 WiFi.status(): ");
    DEBUG_SERIAL.println(WiFi.status());
    DEBUG_SERIAL.print("📡 Local IP: ");
    DEBUG_SERIAL.println(WiFi.localIP());
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
  yield(); // ป้องกัน watchdog reset
}


void handlePhotoState(U2 input_channel_1, U2 input_channel_2, U64 currentTimestamp)
{
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

    U4 inputMergeChannel = (input_channel_2 << 16) | input_channel_1;

    for (U1 cnt = 0; cnt < allSlotCount; cnt++)
    {
        Slot *SlotObj = slotlist[cnt];
        int rIndex = SlotObj->getReadIndex();
        if (rIndex <= 0 || rIndex > 32) {
            DEBUG_SERIAL.print("⚠ Invalid sensor index: ");
            DEBUG_SERIAL.println(rIndex);
            continue;
        }
        U1 photoState = (inputMergeChannel >> (rIndex - 1)) & 0x01;
        SlotTransferType transferType = SlotObj->getSlotTransferType();
        String flowrackName = SlotObj->getFlowrackName();
        int row = SlotObj->getRow();
        int col = SlotObj->getCol();

        // ตรวจจับการเปลี่ยนแปลงของ sensor
        if (SlotObj->isSensorStateChanged(photoState, currentTimestamp))
        {
            SlotObj->setPhotoState(photoState, currentTimestamp);
            DEBUG_SERIAL.printf("📌 [setPhotoState] Updated sensor [%s %d,%d] to %s\n", 
                flowrackName.c_str(), row, col, (photoState == PHOTO_ON ? "ON" : "OFF"));

            if (photoState == PHOTO_ON)
            {
                DEBUG_SERIAL.print("Index: ");
                DEBUG_SERIAL.print(rIndex);
                DEBUG_SERIAL.println(" changed to: on");

                // ✅ ส่ง COUNT_UP เฉพาะเมื่อจาก OFF → ON เท่านั้น
                if (transferType == SLOT_SEND_TYPE && SlotObj->shouldSendCountUpFlg())
                {
                    if (!SlotObj->getIsSendingCountUp()) {
                        SlotObj->setIsSendingCountUp(true); // กันส่งซ้ำ
                        DEBUG_SERIAL.println("🔼 Sending COUNT_UP!");
                        asyncPostHTTP(flowrackName, COUNT_UP, row, col);
                        delay(50);
                        yield(); // ✅ ใส่ delay สั้น ๆ ให้ TCP buffer ระบาย
                    }
                    else {
                    DEBUG_SERIAL.printf("⏸ [SKIP] COUNT_UP already sending → %s [%d,%d]\n", flowrackName.c_str(), row, col);
                  }
                }

                // ส่ง ITEM_IN เฉพาะฝั่ง RECEIVE
                if (transferType == SLOT_RECEIVE_TYPE)
                {
                    DEBUG_SERIAL.printf("📥 [RECV] ITEM_IN → %s [%d,%d]\n", flowrackName.c_str(), row, col);
                    asyncPostHTTP(flowrackName, ITEM_IN, row, col);
                }
            }
            else  // PHOTO_OFF
            {
                // ตรวจ NO_HOLD เมื่อกล่องถูกยกออก
                if (transferType == SLOT_RECEIVE_TYPE && SlotObj->shouldSendReleaseHoldFlg(currentTimestamp))
                {
                    if (!SlotObj->getIsSendingNoHold()) {
                      SlotObj->setIsSendingNoHold(true);   // ✅ ตั้ง flag
                      SlotObj->setIsSendingHold(false);    // ✅ clear HOLD
                      DEBUG_SERIAL.println("✅ Sending NO_HOLD to clear FULL state.");
                      asyncPostHTTP(flowrackName, NO_HOLD, row, col);
                    } else {
                      DEBUG_SERIAL.println("⏸ NO_HOLD already in progress, skipping...");
                    }
                }
            }
        }

        // ✅ ตรวจเงื่อนไขส่ง FULL signal (HOLD) แบบ realtime
        if (transferType == SLOT_RECEIVE_TYPE && SlotObj->shouldSendHoldFlg(currentTimestamp))
        {
            if (!SlotObj->getIsSendingHold()) {
              SlotObj->setIsSendingHold(true);  // ✅ ตั้ง flag
              SlotObj->setIsSendingNoHold(false); // ✅ clear NO_HOLD
              DEBUG_SERIAL.println("🚨 FULL DETECTED! Sending HOLD.");
              asyncPostHTTP(flowrackName, HOLD, row, col);
            } else {
              DEBUG_SERIAL.println("⏸ HOLD already in progress, skipping...");
            }
        }
        yield();
    }
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
  int retry = 0;

  DEBUG_SERIAL.println("Connecting");
  while (WiFi.status() != WL_CONNECTED && retry < 20)
  {
    delay(500);
    DEBUG_SERIAL.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    retry++;

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
  DEBUG_SERIAL.print("📤 asyncPostHTTP called for: ");
  DEBUG_SERIAL.print(flowrack); DEBUG_SERIAL.print(" ["); DEBUG_SERIAL.print(row); DEBUG_SERIAL.print(","); DEBUG_SERIAL.print(col); DEBUG_SERIAL.println("]");

  // 📌 ตรวจสอบ MCP23S17 ก่อนส่งข้อมูล
  if (inputchip1.digitalRead() == 0xFFFF && inputchip2.digitalRead() == 0xFFFF) 
  {
      DEBUG_SERIAL.println("⚠ ERROR: MCP23S17 not detected! Skipping HTTP request...");
      return;
  }

  // 📌 ตรวจสอบสถานะ WiFi ก่อนส่ง
  if (WiFi.status() != WL_CONNECTED)
  {
      DEBUG_SERIAL.println("🚨 ERROR: WiFi Disconnected! Skipping HTTP request.");
      DEBUG_SERIAL.print("📉 Heap on WiFi error: ");
      DEBUG_SERIAL.println(ESP.getFreeHeap());

      String jsonPayload = "{\"station_code\":\"" + String(currentHP) +
                             "\",\"flowrack_name\":\"" + flowrack +
                             "\",\"state\":" + state +
                             ",\"row\":" + row +
                             ",\"col\":" + col + "}";
      failedRequests.push_back({jsonPayload, 0});
      if (failedRequests.size() > 100) {
        DEBUG_SERIAL.println("⚠ Too many failed requests! Clearing queue.");
        failedRequests.clear();
      }
      return;
  }

  String jsonPayload = "{\"station_code\":\"" + String(currentHP) +
                       "\",\"flowrack_name\":\"" + flowrack +
                       "\",\"state\":" + state +
                       ",\"row\":" + row +
                       ",\"col\":" + col + "}";

  DEBUG_SERIAL.println("Sending HTTP POST to: " + jsonPayload);
  DEBUG_SERIAL.println("🌐 Requesting path: " + serverPostURL);
  DEBUG_SERIAL.print("📉 Heap before request: ");
  DEBUG_SERIAL.println(ESP.getFreeHeap());


  if (activeRequests >= 3) {  // 💡 จำกัดพร้อมกันไม่เกิน 3
    DEBUG_SERIAL.println("🚦 Too many active HTTP requests! Skipping this one.");
    DEBUG_SERIAL.print(activeRequests);
    DEBUG_SERIAL.println("). Skipping this one.");
    return;
  }

  AsyncHTTPRequest *request = new AsyncHTTPRequest();
  if (request->readyState() == readyStateUnsent || request->readyState() == readyStateDone)
  {
    request->setTimeout(5000); // ตั้ง timeout 5 วินาที

    activeRequests++;  // 📌 เพิ่มตัวนับก่อนส่ง
    DEBUG_SERIAL.println("📊 Active HTTP Requests: " + String(activeRequests));


    request->onReadyStateChange([request, jsonPayload, flowrack, row, col](void *arg, AsyncHTTPRequest *req, int readyState) {
      if (readyState != readyStateDone) {
        DEBUG_SERIAL.print("⏳ HTTP state: ");
        DEBUG_SERIAL.println(readyState);

      }

      if (req->responseHTTPcode() == -4) {
          DEBUG_SERIAL.println("❗ CONNECTION FAILED → Server refused connection or timed out.");
      }

      if (readyState == readyStateDone) {

        activeRequests--;  // 📌 ลดตัวนับเมื่อเสร็จ
        DEBUG_SERIAL.println("✅ HTTP Request finished. Active left: " + String(activeRequests));

        int code = req->responseHTTPcode();
        String res = req->responseText();

        DEBUG_SERIAL.println("📥 Server responded with HTTP " + String(code));
        DEBUG_SERIAL.print("📥 Response: ");
        DEBUG_SERIAL.println(res.length() > 50 ? res.substring(0, 50) + "..." : res);

        DEBUG_SERIAL.print("📉 Heap after response: ");
        DEBUG_SERIAL.println(ESP.getFreeHeap());

        for (U1 i = 0; i < allSlotCount; i++) {
          Slot* slot = slotlist[i];
          if (slot->getFlowrackName() == flowrack && slot->getRow() == row && slot->getCol() == col) {
            slot->setIsSendingCountUp(false);
            slot->setIsSendingHold(false);     // ✅ clear HOLD flag
            slot->setIsSendingNoHold(false);   // ✅ clear NO_HOLD flag 
            break;
          }
        }

        if (code != 200) {
          DEBUG_SERIAL.println("⚠ ERROR: Server did not return 200 OK! Adding to retry queue.");
          failedRequests.push_back({jsonPayload, 0});
        }

        delete request;
      }
    });

    request->open("POST", serverPostURL.c_str());
    request->setReqHeader("Content-Type", "application/json");
    delay(30); // 💡 หน่วงเบา ๆ ให้ TCP stack ระบาย connection เก่า
    yield();  // 🧠 ปล่อย CPU ให้ background ทำงาน
    bool success = request->send(jsonPayload);
    DEBUG_SERIAL.print("💡 Flowrack: "); DEBUG_SERIAL.print(flowrack);
    DEBUG_SERIAL.print(" Row: "); DEBUG_SERIAL.print(row);
    DEBUG_SERIAL.print(" Col: "); DEBUG_SERIAL.print(col);

    if (!success) {
        DEBUG_SERIAL.println("❌ send() failed! Possibly too many concurrent requests.");
        DEBUG_SERIAL.println("🧠 Free heap on fail: " + String(ESP.getFreeHeap()));
        failedRequests.push_back({jsonPayload, 0});

        for (U1 i = 0; i < allSlotCount; i++) {
          Slot* slot = slotlist[i];
          if (slot->getFlowrackName() == flowrack && slot->getRow() == row && slot->getCol() == col) {
            slot->setIsSendingCountUp(false);
            break;
          }
        }
        delete request;
    }
  }
  else
  {
    DEBUG_SERIAL.println("⚠ ERROR: HTTP request not in correct state!");
    DEBUG_SERIAL.print("📉 Heap at bad state: ");
    DEBUG_SERIAL.println(ESP.getFreeHeap());
    failedRequests.push_back({jsonPayload, 0});
    

    // ✅ ล้าง flag ของ Slot
    for (U1 i = 0; i < allSlotCount; i++) {
      Slot* slot = slotlist[i];
      if (slot->getFlowrackName() == flowrack && slot->getRow() == row && slot->getCol() == col) {
        slot->setIsSendingCountUp(false);
        break;
      }
    }

    delete request;
  }
}



void retryFailedRequests() {
  static U64 lastRetryTime = 0;
  U64 currentMillis = millis();

  if (failedRequests.empty()) return;
  if (currentMillis - lastRetryTime < 3000) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (activeRequests >= 3) {
    DEBUG_SERIAL.println("🚦 Skip retry: too many activeRequests.");
    return;
  }

  lastRetryTime = currentMillis;

  RetryPacket packet = failedRequests.front();
  failedRequests.erase(failedRequests.begin());

  // ✅ แยกข้อมูลจาก JSON string
  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, packet.payload);
  String flowrack = "unknown";
  int row = -1, col = -1;

  if (!error) {
    flowrack = doc["flowrack_name"] | "unknown";
    row = doc["row"] | -1;
    col = doc["col"] | -1;
  }

  // ✅ Drop ถ้าเกิน 3 รอบ
  if (packet.retryCount >= 3) {
    DEBUG_SERIAL.print("❌ Max retry reached for: ");
    DEBUG_SERIAL.print(flowrack);
    DEBUG_SERIAL.print(" [");
    DEBUG_SERIAL.print(row);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(col);
    DEBUG_SERIAL.println("]. Dropping request.");
    return;
  }

  DEBUG_SERIAL.print("♻ Retrying: ");
  DEBUG_SERIAL.print(flowrack);
  DEBUG_SERIAL.print(" [");
  DEBUG_SERIAL.print(row);
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(col);
  DEBUG_SERIAL.print("] Attempt: ");
  DEBUG_SERIAL.println(packet.retryCount + 1);
  DEBUG_SERIAL.print("📉 Heap before retry: ");
  DEBUG_SERIAL.println(ESP.getFreeHeap());

  AsyncHTTPRequest* request = new AsyncHTTPRequest();
  request->setTimeout(5000);
  activeRequests++;
  DEBUG_SERIAL.println("📊 Active HTTP Requests (retry): " + String(activeRequests));

  request->onReadyStateChange([request, packet, flowrack, row, col](void *arg, AsyncHTTPRequest *req, int readyState) mutable {
    if (readyState != readyStateDone) return;

    int code = req->responseHTTPcode();
    DEBUG_SERIAL.print("📡 Retry HTTP Code: ");
    DEBUG_SERIAL.println(code);
    DEBUG_SERIAL.print("📉 Heap after retry: ");
    DEBUG_SERIAL.println(ESP.getFreeHeap());

    if (code == 200) {
      DEBUG_SERIAL.print("📥 Retry response: ");
      DEBUG_SERIAL.println(req->responseText().substring(0, 100));
    }
    else if (code >= 500 || code == -1 || code == -4) {
      DEBUG_SERIAL.println("⚠ Retry failed (server/network). Re-adding to queue.");
      failedRequests.push_back({packet.payload, packet.retryCount + 1});
    }
    else if (code >= 400 && code < 500) {
      DEBUG_SERIAL.print("🚫 Server rejected retry [");
      DEBUG_SERIAL.print(flowrack);
      DEBUG_SERIAL.print("] with HTTP ");
      DEBUG_SERIAL.println(code);
      // Don't re-add
    }

    DEBUG_SERIAL.println("✅ Retry done. Active left: " + String(activeRequests - 1));
    activeRequests--;
    delete request;
  });

  if (!request->open("POST", serverPostURL.c_str())) {
    DEBUG_SERIAL.println("⚠ Failed to open retry request.");
    failedRequests.push_back({packet.payload, packet.retryCount + 1});
    activeRequests--;
    delete request;
    return;
  }

  request->setReqHeader("Content-Type", "application/json");
  if (!request->send(packet.payload)) {
    DEBUG_SERIAL.println("❌ Failed to send retry HTTP request.");
    failedRequests.push_back({packet.payload, packet.retryCount + 1});
    activeRequests--;
    delete request;
    return;
  }

  yield();
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
                        { DEBUG_SERIAL.printf("Progress: %u%%\r", (progress / (total / 150))); });
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