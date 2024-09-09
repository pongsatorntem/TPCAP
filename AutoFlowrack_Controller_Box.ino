
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

MCP inputchip1(0, 4);             // Instantiate an object called "inputchip" on an MCP23S17 device at address 1
                    // and slave-select on Arduino pin 10
MCP inputchip2(0, 5);            // Instantiate an object called "outputchip" on an MCP23S17 device at address 2
                    // and slave-select on Arduino pin 10

//char* ssid = "Wireless-PC";
//char* password = "";
char* ssid = "AMR_TDEM";
char* password = "1234@abcd";

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
BL prev_DOM_slot_1_flag = 1;
BL DOM_slot_2_flag = 0;
BL final_DOM_slot_2_flag = 0;
BL prev_DOM_slot_2_flag = 0;
BL DOM_slot_3_flag = 0;
BL final_DOM_slot_3_flag = 0;
BL prev_DOM_slot_3_flag = 0;
BL DOM_slot_4_flag = 0;
BL final_DOM_slot_4_flag = 0;
BL prev_DOM_slot_4_flag = 0;


// System Config
U2 JUDGE_TIME_THRESHOLD = 500;  //ms

WiFiClient client;

U4 lastTime = 0;
U4 timerDelay = 100;
U4 lastTime_2 = 0;
U4 timerDelay_2 = 500;

int test_counter = 0;

const int interruptInterval = 2000;  // Interval in milliseconds
volatile bool interruptFlag = false;

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
  Connect_Self_Network();

//  // Set up timer interrupt every 2 seconds
//  noInterrupts();  // Disable interrupts during configuration
//  timer1_attachInterrupt(timerISR);  // Attach the ISR function
//  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);  // Set timer frequency and type
//  timer1_write(80000000 / 16 * 2);  // Set the timer period for 2 seconds
//  interrupts();  // Enable interrupts after configuration
//
//  Serial.println("Timer interrupt every 100 ms is set up.");
  
}

void timerISR() {
  //interruptFlag = true;  // Set the flag indicating the interrupt occurred
  //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  Serial.println("Timer interrupt");
}

void loop() {
  // put your main code here, to run repeatedly:

  int input_channel_1,input_channel_2;                        // declare an integer to hold the value temporarily.
  input_channel_1 = inputchip1.digitalRead();  // read the input chip in word-mode, storing the result in "value"
  input_channel_2 = inputchip2.digitalRead();  // read the input chip in word-mode, storing the result in "value"
  //Serial.print(input_channel_1,BIN);
  //Serial.print("  ");
  //Serial.println(input_channel_2,BIN);

  
  //Read PhotoSensor Loop: 100ms
  U4 current_time = millis();
  if ((millis() - lastTime) > timerDelay) 
  {
    //Serial.print(input_channel_1,BIN);
    //Serial.print("  ");
    //Serial.println(input_channel_2,BIN);

//    Serial.print(BIN_slot_1_flag);
//    Serial.print(BIN_slot_2_flag);
//    Serial.print(BIN_slot_3_flag);
//    Serial.print(BIN_slot_4_flag);
//    Serial.print(DOM_slot_1_flag);
//    Serial.print(DOM_slot_2_flag);
//    Serial.print(DOM_slot_3_flag);
//    Serial.println(DOM_slot_4_flag);

    
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
  BIN_slot_1_flag = (input_channel_1>>8)&0x01;
  BIN_slot_2_flag = (input_channel_1>>9)&0x01;
  BIN_slot_3_flag = (input_channel_1>>10)&0x01;
  BIN_slot_4_flag = (input_channel_2>>13)&0x01;
  DOM_slot_1_flag = (input_channel_1>>12)&0x01;
  // DOM_slot_2_flag = (input_channel_1>>13)&0x01;
  DOM_slot_3_flag = (input_channel_1>>14)&0x01;
  DOM_slot_4_flag = (input_channel_2>>14)&0x01;


  // Cannot use input from channel_1
  DOM_slot_2_flag = (input_channel_2>>8)&0x01;

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
  if(DOM_slot_1_flag==0 && prev_DOM_slot_1_flag==1)
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
  if(DOM_slot_1_flag==1 && prev_DOM_slot_1_flag==0)
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
  


  if (((millis() - lastTime_2) > timerDelay_2))
  {
    //http://localhost:4499/flowrack/item_counter?hp=demo&flowrack=domestic&state=1&slot=1
    String HP = "demo";
    String flowrack = "binning";
    int state = 1;
    int slot = 1;
    String serverPath = "http://192.168.1.100:4499/flowrack/item_counter";
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

  prev_BIN_slot_1_flag = BIN_slot_1_flag;
  prev_BIN_slot_2_flag = BIN_slot_2_flag;
  prev_BIN_slot_3_flag = BIN_slot_3_flag;
  prev_BIN_slot_4_flag = BIN_slot_4_flag;
  prev_DOM_slot_1_flag = DOM_slot_1_flag;
  prev_DOM_slot_2_flag = DOM_slot_2_flag;
  prev_DOM_slot_3_flag = DOM_slot_3_flag;
  prev_DOM_slot_4_flag = DOM_slot_4_flag;
}

void POST_https(String serverPath,String HP,String flowrack, int state, int slot)
{
    //String url = serverPath + "?hp=" + HP +"&flowrack=" + flowrack +"&state=" + state+"&slot=" + slot ;
    String request_path = serverPath + "?hp=" + HP +"&flowrack=" + flowrack +"&state=" + state+"&slot=" + slot ;
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
      //WiFiClient client;
      HTTPClient http;
      http.setTimeout(1000);
      client.setTimeout(1000);
      //client.setInsecure();
      
      // Your Domain name with URL path or IP address with path
      http.begin(client, request_path.c_str());
      //http.begin(request_path.c_str());
      
      // Send HTTP GET request
      int httpResponseCode = http.POST("");
      
      if (httpResponseCode>0) {
        DEBUG_SERIAL.print("HTTP Response code: ");
        DEBUG_SERIAL.println(httpResponseCode);
        String payload = http.getString();
        DEBUG_SERIAL.println(payload);
      }
      else {
        DEBUG_SERIAL.print("Error code: ");
        DEBUG_SERIAL.println(httpResponseCode);
      }
      // Free resources
      http.end();
    }
    else {
      DEBUG_SERIAL.println("WiFi Disconnected");
    }
}

void Connect_TPCAP_Network()
{
  /*********** Wifi Config ***********/
  ssid = "Wireless-PC";
  password = "";

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
  //ssid = "AMR_TDEM_2.4G";
  //password = "1234@abcd";
  //ssid = "PrivateNetwork_2.4G";
  //password = "zaazap01";
  ssid = "AMR_TDEM_AUTOFEED";
  password = "1234@abcd";

  // Set your Static IP address
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  IPAddress local_IP(192,168,1,70);
  IPAddress gateway(192,168,1,1);
  IPAddress subnet(255,255,255,0);

  delay(1000);
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
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    DEBUG_SERIAL.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    //digitalWrite(Binning_Lamp,!digitalRead(LED_BUILTIN));
  }
  DEBUG_SERIAL.println("");
  DEBUG_SERIAL.print("Connected to WiFi network with IP Address: ");
  DEBUG_SERIAL.println(WiFi.localIP());
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  digitalWrite(LED_BUILTIN, HIGH);
}
