#include <Arduino.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Load Wi-Fi networking
#include <WiFi.h>
#include <esp_wifi.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <WebSerial.h>
#include <time.h>

static AsyncWebServer server(80);

// Load FastLED
#include <FastLED.h>

// Webserver and file system
#define SPIFFS LittleFS
#include <LittleFS.h>
#include <FS.h>
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson?utm_source=platformio&utm_medium=piohome

// my wifi secrets
#include "secrets.h"

// esp32 native TWAI / CAN library
#include "driver/twai.h"
TaskHandle_t canbus_task_handle = NULL; // task handle for canbus task

// my canbus stuff
#include "canbus_msg.h"
#include "canbus_flags.h"

#define CAN_SELF_MSG 0

// Interval:
#define TRANSMIT_RATE_MS 4000
#define POLLING_RATE_MS 1000

// time stuff
#define NTP_SERVER     "us.pool.ntp.org"
#define UTC_OFFSET     0
#define UTC_OFFSET_DST 0

// organize nodes 
struct remoteNode {
  uint8_t   nodeID[4]; // four byte node identifier 
  uint16_t  nodeType; // first introduction type
  uint16_t  subModules[4]; // introductions for up to four sub modules 
  uint8_t   moduleCnt; // sub module count
  uint32_t  lastSeen; // unix timestamp 
};

struct remoteNode nodeList[8]; // list of remote nodes

static bool driver_installed = false;

unsigned long previousMillis = 0;  // will store last time a message was send

static const char *TAG = "can_control";

volatile uint8_t nodeSwitchState[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // switch state
volatile uint8_t nodeSwitchMode[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // switch mode

volatile uint8_t testState[3] = {1, 0, 2}; // test state
volatile uint8_t testPtr = 0; // test pointer
volatile uint8_t testRestart = true; // set flag to true to restart test message squence

volatile uint16_t introMsg[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // intro messages
volatile uint8_t  introMsgPtr = 0; // intro message pointer
volatile uint8_t  introMsgData[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // intro message data
volatile uint8_t  introMsgCnt = 0; // intro message count

#ifdef M5STACK
const char* AP_SSID  = "m5stack-atom";
const char* hostname = "m5stack-atom";
#define CAN_MY_TYPE BOX_SW_4RELAY // 4 relay switch box
const uint8_t* myNodeFeatureMask = FEATURE_BOX_SW_4RELAY; // node feature mask
const uint8_t mySwitchCount = 4;
const uint8_t otherNodeID[] = {0x25, 0x97, 0x51, 0x1C}; // M5PICO node id

#elif M5PICO2
const char* AP_SSID  = "m5pico2";
const char* hostname = "m5pico2";
#define CAN_MY_TYPE BOX_SW_4GANG // 4 switch box
const uint8_t* myNodeFeatureMask = FEATURE_BOX_SW_4GANG; // node feature mask
const uint8_t mySwitchCount = 4;
const uint8_t otherNodeID[] = {0x25, 0x97, 0x51, 0x1C}; // M5PICO node id

#else
const char* AP_SSID  = "cancontrol-test";
const char* hostname = "cancontrol-test";
#define CAN_MY_TYPE DISP_LCD // LCD display
const uint8_t* myNodeFeatureMask = FEATURE_DISP_LCD; // node feature mask
const uint8_t otherNodeID[] = {0x25, 0x97, 0x51, 0x1C}; // M5STACK node id
#endif

const char* ssid     = SECRET_SSID;
const char* password = SECRET_PSK;

int period = 1000;
int8_t ipCnt = 0;

unsigned long time_now = 0;

CRGB leds[ARGB_LEDS];

unsigned long ota_progress_millis = 0;

static volatile bool wifi_connected = false;
static volatile uint8_t myNodeID[] = {0, 0, 0, 0}; // node ID

// Function that gets current epoch time
unsigned long getEpoch() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

// print timestamp to webserial console
static void printEpoch() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return;
  }
  time(&now);
  WebSerial.printf("%lu\n", now);
}

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    /* Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]); */
    myNodeID[0] = baseMac[2];
    myNodeID[1] = baseMac[3];
    myNodeID[2] = baseMac[4];
    myNodeID[3] = baseMac[5];
    Serial.printf("Node ID: %02x:%02x:%02x:%02x\n", myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3]);
  } else {
    Serial.println("Failed to set NODE ID");
  }
}

void wifiOnConnect(){
  Serial.println("STA Connected");
  Serial.print("STA SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("STA IPv4: ");
  Serial.println(WiFi.localIP());
}

//when wifi disconnects
void wifiOnDisconnect(){
  Serial.println("STA disconnected, reconnecting...");
  delay(1000);
  WiFi.begin(ssid, password);
}

void WiFiEvent(WiFiEvent_t event){
 
}

static void send_message(const uint16_t msgID, const uint8_t *msgData, const uint8_t dlc) {
  twai_message_t message;
  // static uint8_t dataBytes[] = {0, 0, 0, 0, 0, 0, 0, 0}; // initialize dataBytes array with 8 bytes of 0

  leds[0] = CRGB::Blue;
  FastLED.show();

  // Format message
  message.identifier = msgID;       // set message ID
  message.extd = 0;                 // 0 = standard frame, 1 = extended frame
  message.rtr = 0;                  // 0 = data frame, 1 = remote frame
  message.self = CAN_SELF_MSG;      // 0 = normal transmission, 1 = self reception request 
  message.dlc_non_comp = 0;         // non-compliant DLC (0-8 bytes)  
  message.data_length_code = dlc;   // data length code (0-8 bytes)
  memcpy(message.data, (const uint8_t*) msgData, dlc);  // copy data to message data field 
  
  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(3000)) == ESP_OK) {
    // ESP_LOGI(TAG, "Message queued for transmission\n");
    WebSerial.printf("TX: MSG: %03x WITH %u DATA", msgID, dlc);

    // printf("Message queued for transmission\n");
    // WebSerial.printf("TX: MSG: %03x Data: ", msgID);
    // for (int i = 0; i < dlc; i++) {
    //   WebSerial.printf("%02x ", message.data[i]);
    // }
    // WebSerial.printf("\n");
  } else {
    leds[0] = CRGB::Red;
    FastLED.show();
    // ESP_LOGE(TAG, "Failed to queue message for transmission, initiating recovery");
    WebSerial.printf("ERR: Failed to queue message for transmission, resetting controller\n");
    twai_initiate_recovery();
    twai_stop();
    WebSerial.printf("WARN: twai Stopped\n");
    vTaskDelay(500);
    twai_start();
    WebSerial.printf("WARN: twai Started\n");
    // ESP_LOGI(TAG, "twai restarted\n");
    // wifiOnConnect();
    vTaskDelay(500);
    leds[0] = CRGB::Black;
    FastLED.show();
  }
  leds[0] = CRGB::Black;
  FastLED.show();
  // vTaskDelay(100);
}

static void rxDisplayMode(uint8_t *data, uint8_t displayMode) {
  static uint8_t rxdisplayID = data[4]; // display id
  WebSerial.printf("RX: Display: %d Mode: %d\n", rxdisplayID, displayMode);

  switch (displayMode) {
    case 0: // display off
      break;
    case 1: // display on
      break;
    case 2: // clear display
      break;
    case 3: // flash display
      break;
    default:
      WebSerial.println("Invalid display mode");
      break;
  }
}

static void rxSwMomDur(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint16_t swDuration = (data[5] << 8) | data[6]; // duration in ms
}

static void rxSwBlinkDelay(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint16_t swBlinkDelay = (data[5] << 8) | data[6]; // delay in ms 
}

static void rxSwStrobePat(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint8_t swStrobePat = data[5]; // strobe pattern
}

static void rxPWMDuty(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint16_t PWMDuty = (data[5] << 8) | data[6]; // pwm duty cycle
}

static void rxPWMFreq(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint16_t PWMFreq = (data[5] << 8) | data[6]; // pwm frequency 
}

static void txSwitchState(uint8_t *nodeID, uint8_t switchID, uint8_t swState) {
  if (nodeID == NULL) {
    // WebSerial.println("Invalid node ID");
    return;
  }

  static uint8_t txDLC = 5;
  static uint8_t dataBytes[] = {nodeID[0], nodeID[1], nodeID[2], nodeID[3], switchID}; // set node id and switch ID
  
  WebSerial.printf("TX: To %02x:%02x:%02x:%02x Switch %d State %d\n", nodeID[0],nodeID[1],nodeID[2],nodeID[3], switchID, swState);

  switch (swState) {
    case 0: // switch off
      send_message(SW_SET_OFF, dataBytes, txDLC);

      break;
    case 1: // switch on
      send_message(SW_SET_ON, dataBytes, txDLC);
      break;
    case 2: // momentary press
      send_message(SW_MOM_PRESS, dataBytes, txDLC);
      break;
    default: // unsupported state
      // WebSerial.println("Invalid switch state for transmission");
      break;
  }
}

static void rxSwitchState(uint8_t *data, uint8_t swState) {
  static uint8_t switchID = data[4]; // switch ID 
  // static uint8_t unitID[] = {data[0], data[1], data[2], data[3]}; // unit ID
  static uint8_t dataBytes[] = {myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3], switchID}; // send my own node ID, along with the switch number

  // WebSerial.printf("RX: Set Switch %d State %d\n", switchID, swState);
  nodeSwitchState[switchID] = swState; // update switch state
  

  switch (swState) {
    case 0: // switch off
      // send_message(DATA_OUTPUT_SWITCH_OFF, dataBytes, sizeof(dataBytes));
      break;
    case 1: // switch on
      // send_message(DATA_OUTPUT_SWITCH_ON, dataBytes, sizeof(dataBytes));
      break;
    case 2: // momentary press
      send_message(DATA_OUTPUT_SWITCH_MOM_PUSH , dataBytes, sizeof(dataBytes));
      // send_message(DATA_OUTPUT_SWITCH_ON, dataBytes, sizeof(dataBytes));
      // send_message(DATA_OUTPUT_SWITCH_OFF, dataBytes, sizeof(dataBytes));
      break;
    default:
      WebSerial.println("Invalid switch state");
      break;
  }
}

static void txSwitchMode(uint8_t *data, uint8_t switchID, uint8_t switchMode) {
  static uint8_t txDLC = 6;
  static uint8_t dataBytes[] = {data[0], data[1], data[2], data[3], switchID, switchMode}; // set node id switch ID
  WebSerial.printf("TX: To %02x:%02x:%02x:%02x Switch %d Mode %d\n",data[0], data[1], data[2], data[3], switchID, switchMode);
  send_message(SW_SET_MODE, dataBytes, sizeof(dataBytes)); // send message to set switch mode
}

static void rxSwitchMode(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint8_t switchMode = data[5]; // switch mode

  static uint8_t dataBytes[] = {myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3], switchID, switchMode}; // send my own node ID, along with the switch number

  WebSerial.printf("RX: Set Switch %d State %d\n", switchID, switchMode);
  // send_message(DATA_OUTPUT_SWITCH_MODE, dataBytes, sizeof(dataBytes));    
  nodeSwitchMode[switchID] = switchMode; // update switch mode


  switch (switchMode) {
    case 0: // solid state (on/off)
      break;  
    break;
    case 1: // one-shot momentary
      break;
    case 2: // blinking
      break;
    case 3: // strobing
      break;
    case 4: // pwm
      break;
    default:
      WebSerial.println("Invalid switch mode");
      break;
  }
}

// send an introduction message to the bus
// static void txIntroduction(const uint8_t* txNodeID, const uint8_t* txNodeFeatureMask, const uint_8t txMsgData, const uint16_t txmsgID, const uint8_t ptr) {
static void txIntroduction(uint8_t* txNodeID, uint8_t* txNodeFeatureMask, uint8_t txMsgData, uint16_t txmsgID, uint8_t ptr) {
     WebSerial.printf("TX: INTRO PTR %d\n", ptr);    

    if (txmsgID < 1) {
      // WebSerial.printf("RX: INTRO ACK NULL\n");
      return; // exit function if message ID is null
    }

    if (ptr == 0) {
      uint8_t dataBytes[6] = { txNodeID[0], txNodeID[1], txNodeID[2], txNodeID[3], 
                               txNodeFeatureMask[0], txNodeFeatureMask[1] }; 

      send_message(txmsgID, dataBytes, 6);
    } else {
      uint8_t dataBytes[5] = { txNodeID[0], txNodeID[1], txNodeID[2], txNodeID[3], txMsgData }; 

      send_message(txmsgID, dataBytes, 5);
    }
}

static void nodeCheckStatus() {
  if (FLAG_SEND_INTRODUCTION) {
    // send introduction message to all nodes
    txIntroduction((uint8_t*)myNodeID, (uint8_t*)myNodeFeatureMask, introMsgData[introMsgPtr], introMsg[introMsgPtr], introMsgPtr);
    WebSerial.printf("TX: INTRO MSG %d OF %d\n", introMsgPtr, introMsgCnt);

    if (introMsgPtr >= introMsgCnt) {
      FLAG_SEND_INTRODUCTION = false; // clear flag to send introduction message
      FLAG_BEGIN_NORMAL_OPER = true; // set flag to begin normal operation 
    }
  }

  if (!FLAG_BEGIN_NORMAL_OPER) {
    return; // normal operation not started, exit function
  }

  for (uint8_t switchID = 0; switchID < mySwitchCount; switchID++) {
    uint8_t swState = nodeSwitchState[switchID]; // get switch state
    uint8_t swMode = nodeSwitchMode[switchID]; // get switch mode
    uint8_t stateData[] = {myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3], switchID}; // send my own node ID, along with the switch number
    uint8_t modeData[] = {myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3], switchID, swMode}; // send my own node ID, along with the switch number
      
    // send_message(DATA_OUTPUT_SWITCH_MODE, modeData, sizeof(modeData));  
    // WebSerial.printf("TX: DATA: Switch %d State %d Mode %d\n", switchID, swState, swMode);

    switch (swState) {
      case 0: // switch off
        send_message(DATA_OUTPUT_SWITCH_OFF, stateData, sizeof(stateData));
        break;
      case 1: // switch on
        send_message(DATA_OUTPUT_SWITCH_ON, stateData, sizeof(stateData));
        break;
      case 2: // momentary press
        // send_message(DATA_OUTPUT_SWITCH_ON, dataBytes, sizeof(dataBytes));
        // send_message(DATA_OUTPUT_SWITCH_OFF, dataBytes, sizeof(dataBytes));
        break;
      default:
        break;
    }
  }
 
}

static void handle_rx_message(twai_message_t &message) {
  bool msgFlag = false;
  bool haveRXID = false; 
  int msgIDComp;
  uint8_t rxNodeID[4] = {0, 0, 0, 0}; // node ID

  leds[0] = CRGB::Orange;
  FastLED.show();
  
  // WebSerial.printf("RX: MSG: %03x DATA: %u\n", message.identifier, message.data_length_code);

  // check if message contains enough data to have node id
  if (message.data_length_code >= 3) { 
    memcpy((void *)rxNodeID, (const void *)message.data, 4); // copy node id from message
    msgIDComp = memcmp((const void *)rxNodeID, (const void *)myNodeID, 4);
    haveRXID = true; // set flag to true if message contains node id

    if (msgIDComp == 0) { // message is for us
      msgFlag = true; // message is for us, set flag to true
    }
  }


  // if ((!msgFlag) && (message.identifier <= 0x17F)) { // switch control message but not for us
  //   return; // exit function
  // } 

  if (message.data_length_code > 0) { // message contains data, check if it is for us
    if (msgFlag) {
      WebSerial.printf("RX: MATCH MSG: %03x DATA: %u\n", message.identifier, message.data_length_code);
    } else {
      WebSerial.printf("RX: NO MATCH MSG: %03x DATA: u%\n", message.identifier, message.data_length_code);
    }
  } else {
    if (msgFlag) {
      WebSerial.printf("RX: MATCH MSG: %03x NO DATA\n", message.identifier);
    } else {
      WebSerial.printf("RX: NO MATCH MSG: %03x NO DATA\n", message.identifier);
    } 
  }


  switch (message.identifier) {
    case MSG_NORM_OPER: // normal operation message
      WebSerial.printf("RX: Normal Operation Message\n");
      FLAG_BEGIN_NORMAL_OPER = true; // set flag to begin normal operation
      // introMsgPtr = introMsgPtr + 1; // increment intro message pointer 4th step
      break;
    case MSG_HALT_OPER: // halt operation message
      WebSerial.printf("RX: Halt Operation Message\n");
      // introMsgPtr = 0; // reset intro message pointer
      FLAG_BEGIN_NORMAL_OPER = false; // clear flag to halt normal operation
      break;
    case SW_SET_OFF:            // set output switch off
      rxSwitchState(message.data, 0);
      break;
    case SW_SET_ON:             // set output switch on
      rxSwitchState(message.data, 1);
      break;
    case SW_MOM_PRESS:          // set output momentary
      rxSwitchState(message.data, 2);
      break;
    case SW_SET_MODE:           // setup output switch modes
      rxSwitchMode(message.data);
      break;
    case SW_SET_PWM_DUTY:          // set output switch pwm duty
      rxPWMDuty(message.data);  
      break;
    case SW_SET_PWM_FREQ:          // set output switch pwm frequency
      rxPWMFreq(message.data);
      break;
    case SW_SET_MOM_DUR:          // set output switch momentary duration
      rxSwMomDur(message.data);
      break;
    case SW_SET_BLINK_DELAY:          // set output switch blink delay
      rxSwBlinkDelay(message.data);
      break;
    case SW_SET_STROBE_PAT:          // set output switch strobe pattern
      rxSwStrobePat(message.data);
      break;
    case SET_DISPLAY_OFF:          // set display off
      rxDisplayMode(message.data, 0); 
      break;
    case SET_DISPLAY_ON:          // set display on
      rxDisplayMode(message.data, 1); 
      break;    
    case SET_DISPLAY_CLEAR:          // clear display
      rxDisplayMode(message.data, 2); 
      break;
    case SET_DISPLAY_FLASH:          // flash display
      rxDisplayMode(message.data, 3); 
      break;
    case DATA_OUTPUT_SWITCH_OFF:          
      // txSwitchState((uint8_t *)otherNodeID, 2, 1);
      break;
    case DATA_OUTPUT_SWITCH_ON:
      // txSwitchState((uint8_t *)otherNodeID, 2, 2);
      break;      
    case DATA_OUTPUT_SWITCH_MOM_PUSH:
      // txSwitchState((uint8_t *)otherNodeID, 2, 0);
      // FLAG_BEGIN_NORMAL_OPER = false; // clear flag to halt normal operation
      // vTaskDelay(10);
      // txSendHalt((uint8_t *)otherNodeID); // send halt message to other node
      // introMsgPtr = introMsgPtr + 1;
      break;
    case REQ_SWITCHBOX: // request for box introduction, kicks off the introduction sequence
      if (haveRXID) { // check if REQ message contains node id
        WebSerial.printf("RX: REQ BOX Responding to %02x:%02x:%02x:%02x\n", rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3]);
        introMsgPtr = 0; // reset intro message pointer
        FLAG_SEND_INTRODUCTION = true; // set flag to send introduction message
      }
      break;
    case ACK_SWITCHBOX:
      if (msgFlag) { // message was sent to our ID
        if (introMsgPtr < introMsgCnt) {
          WebSerial.printf("RX: INTRO ACK %d\n", introMsgPtr);  
          FLAG_SEND_INTRODUCTION = true; // keep sending introductions until all messages have been acknowledged
          introMsgPtr = introMsgPtr + 1; // increment intro message pointer 1st step
        }
      }
      break;
    
    default:
 
      break;
  }

  leds[0] = CRGB::Black;
  FastLED.show();

} // end of handle_rx_message

void TaskTWAI(void *pvParameters) {
  // give some time at boot the cpu setup other parameters
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  leds[0] = CRGB::Red;
  FastLED.show();

  // hardware acceptance filter
  // filter 0x100:0x17f and 0x410:0x47f
  // filter also contains DB1 of the node ID  
  const uint16_t filterF1 = (0x100 << 5) | (myNodeID[0] >> 4);            
  const uint16_t filterF2 = (0x400 << 5) | (myNodeID[0] & 0x0F);  
  // const uint32_t maskF1F2 = (uint32_t) 0xF00FF00F;
  const uint32_t maskF1F2 = (uint32_t) 0xFF00FF0;

  // const uint16_t filterF1 = (0x110 << 5);            // 0x110:0x17f are switch control codse, 
  // const uint16_t filterF2 = (0x410 << 5);            // 0x410:0x47f are sub-module request codes,
  // const uint32_t maskF1F2 = (uint32_t) 0xF000F000;

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);  // TWAI_MODE_NO_ACK , TWAI_MODE_LISTEN_ONLY , TWAI_MODE_NORMAL
  g_config.rx_queue_len = 20; // RX queue length
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();  //Look in the api-reference for other speed sets.
  // 3. Configure the acceptance filter
  // Hardware filter will accept IDs 0x100 through 0x17F
  twai_filter_config_t f_config = {
    .acceptance_code = (((uint32_t) filterF1<<16) | ((uint32_t) filterF2)), // (0x100 << 21)
    .acceptance_mask = maskF1F2,
    .single_filter = false          // Confirm single filter mode
  };

  // twai_filter_config_t f_config = {
  //   .acceptance_code = (uint32_t)0x20000000, // (0x100 << 21)
  //   .acceptance_mask = (uint32_t)0xF0000000,
  //   .single_filter = true          // Confirm single filter mode
  // };

  
  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    WebSerial.println("Driver installed");
  } else {
    WebSerial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    WebSerial.println("Driver started");
  } else {
    WebSerial.println("Failed to start driver");
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    WebSerial.println("CAN Alerts reconfigured");
  } else {
    WebSerial.println("Failed to reconfigure alerts");
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;


  for (;;) {
    if (!driver_installed) {
      // Driver not installed
      vTaskDelay(1000);
      return;
    }
    // Check if alert happened
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    // Handle alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      WebSerial.println("Alert: TWAI controller has become error passive.");
      leds[0] = CRGB::Red;
      FastLED.show();
    }

    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      WebSerial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      WebSerial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
      leds[0] = CRGB::Red;
      FastLED.show();
    }

    if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
      WebSerial.println("Alert: The Transmission failed.");
      WebSerial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
      WebSerial.printf("TX error: %d\t", twaistatus.tx_error_counter);
      WebSerial.printf("TX failed: %d\n", twaistatus.tx_failed_count);
      leds[0] = CRGB::Red;
      FastLED.show();
    }

    if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
      leds[0] = CRGB::Green;
      FastLED.show();
      // Serial.println("Alert: The Transmission was successful.");
      // Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
    }

    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      WebSerial.println("Alert: The RX queue is full causing a received frame to be lost.");
      WebSerial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
      WebSerial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
      WebSerial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
    }

    // Check if message is received
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {

      // One or more messages received. Handle all.
      twai_message_t message;
      while (twai_receive(&message, 0) == ESP_OK) {
        handle_rx_message(message);
      }
    }
    // Send message
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
      leds[0] = CRGB::Blue;
      FastLED.show();
      previousMillis = currentMillis;

      // WebSerial.printf("Acc code %x Acc mask %x\n", f_config.acceptance_code, f_config.acceptance_mask);
      WebSerial.printf(".\n");

      nodeCheckStatus();
    }
    vTaskDelay(10);

  }
}

void printWifi() {
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);

  if (d == "C0"){
    vTaskSuspend(canbus_task_handle); // suspend canbus task
    // digitalWrite(LED, HIGH);
  }
  if (d == "C1"){
    vTaskResume(canbus_task_handle); // resume canbus task
    // digitalWrite(LED, HIGH);
  }
  if (d == "W"){
    printWifi();
    // digitalWrite(LED, HIGH);
  }

  if (d=="LIST"){
    // dumpNodeList();
    // digitalWrite(LED, LOW);
  }
  if (d == "ON"){
    FLAG_BEGIN_NORMAL_OPER = true; // flag to start normal operation
  }
  if (d=="OFF"){
    FLAG_BEGIN_NORMAL_OPER = false; // clear flag for normal operation
  }
}

void setup() {

  #ifdef M5STACK
  introMsgCnt = 2; // number of intro messages
  introMsgPtr = 0; // start at zero
  introMsg[0] = (uint16_t) BOX_SW_4RELAY; // intro message for 4 relay switch box
  introMsg[1] = (uint16_t) OUT_MECH_RELAY; // intro message for mechanical relay
  
  introMsgData[0] = 0; // send feature mask
  introMsgData[1] = 4; // four relays  
  #elif M5PICO2
  introMsgCnt = 3; // number of intro messages
  introMsgPtr = 0; // start at zero
  introMsg[0] = (uint16_t) BOX_SW_4GANG; // intro message for 4 relay switch box
  introMsg[1] = (uint16_t) OUT_HIGH_CURRENT_SW; // intro message for high current switch
  introMsg[2] = (uint16_t) OUT_LOW_CURRENT_SW; // intro message for low current switch

  introMsgData[0] = 0x00; // send feature mask
  introMsgData[1] = 2; // two high current switches
  introMsgData[2] = 2; // two low current switches
  #endif


  delay(5000);

  // Timer0_Cfg = timerBegin(0, 80, true);
  // timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  // timerAlarmWrite(Timer0_Cfg, 100000, true);
  // timerAlarmEnable(Timer0_Cfg);

  xTaskCreatePinnedToCore(
    TaskTWAI,     // Task function.
    "Task TWAI",  // name of task.
    3172,         // Stack size of task
    NULL,         // parameter of the task
    1,            // priority of the task
    &canbus_task_handle,           // Task handle to keep track of created task
    tskNO_AFFINITY); // allow task to run on either core, automatic depends the load of each core
  // );  

  // xTaskCreate(
  //   TaskFLED,     // Task function.
  //   "Task FLED",  // name of task.
  //   2048,         // Stack size of task
  //   NULL,         // parameter of the task
  //   1,            // priority of the task
  //   NULL    // Task handle to keep track of created task
  // );              // pin task to core 0
  //tskNO_AFFINITY); // pin task to core is automatic depends the load of each core

  FastLED.addLeds<SK6812, ARGB_PIN, GRB>(leds, ARGB_LEDS);
  leds[0] = CRGB::Black;
  FastLED.show();


  Serial.begin(115200); // alternate serial port
  Serial.setDebugOutput(true);
 
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.softAP(AP_SSID);
  WiFi.begin(ssid, password);

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
    vTaskSuspend(canbus_task_handle); // suspend canbus task
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf(".");
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();

  Serial.println("AP Started");
  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP IPv4: ");
  Serial.println(WiFi.softAPIP());

  // Make it possible to access webserver at http://myEsp32.local
  if (!MDNS.begin(hostname)) {
    Serial.println("Error setting up mDNS responder!");
  } else {
    Serial.printf("Access at http://%s.local\n", hostname);
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is AsyncWebServer.");
  });

  server.begin();
  Serial.println("HTTP server started");

  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);
  WebSerial.onMessage(recvMsg);
  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
  printWifi();

  configTime(UTC_OFFSET, UTC_OFFSET_DST, NTP_SERVER);

  FLAG_SEND_INTRODUCTION = true; // set flag to send introduction message
}

void loop() {
  ArduinoOTA.handle();
  // NOP;
}