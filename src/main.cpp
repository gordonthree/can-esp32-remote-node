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

// my canbus stuff
#include "canbus_msg.h"
#include "canbus_flags.h"

#define CAN_SELF_MSG 0

// Interval:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000

static bool driver_installed = false;

unsigned long previousMillis = 0;  // will store last time a message was send

static const char *TAG = "can_control";

volatile uint8_t nodeSwitchState[8];
volatile uint8_t nodeSwitchMode[8];


#ifdef M5PICO
const char* AP_SSID  = "m5stamp-pico";
const char* hostname = "m5stamp-pico";
#define CAN_MY_TYPE IFACE_TOUCHSCREEN_TYPE_A
const uint8_t* myNodeFeatureMask = FEATURE_IFACE_TOUCHSCREEN_TYPE_A; // node feature mask
const uint16_t myNodeIntro = REQ_INTERFACES; // intro request for my node type
const uint8_t otherNodeID[] = {0xFA, 0x61, 0x5D, 0xDC}; // M5STACK node id
#elif M5STACK
const char* AP_SSID  = "m5stack-atom";
const char* hostname = "m5stack-atom";
#define CAN_MY_TYPE BOX_SW_4RELAY // 4 relay switch box
const uint8_t* myNodeFeatureMask = FEATURE_BOX_SW_4RELAY; // node feature mask
const uint16_t myNodeIntro = REQ_BOXES; // intro request for my node type
const uint8_t otherNodeID[] = {0x25, 0x97, 0x51, 0x1C}; // M5PICO node id
#else
const char* AP_SSID  = "candisplay";
const char* hostname = "candisplay";
#define CAN_MY_TYPE DISP_LCD // LCD display
const uint8_t* myNodeFeatureMask = FEATURE_DISP_LCD; // node feature mask
const uint16_t myNodeIntro = REQ_DISPLAYS; // intro request for my node type
const uint8_t otherNodeID[] = {0xFA, 0x61, 0x5D, 0xDC}; // M5STACK node id
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

void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
  if (d == "ON"){
    // digitalWrite(LED, HIGH);
  }
  if (d=="OFF"){
    // digitalWrite(LED, LOW);
  }
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
   /*  switch(event) {

        case SYSTEM_EVENT_AP_START:
            //can set ap hostname here
            WiFi.softAPsetHostname(AP_SSID);
            //enable ap ipv6 here
            WiFi.softAPenableIpV6();
            break;

        case SYSTEM_EVENT_STA_START:
            //set sta hostname here
            WiFi.setHostname(AP_SSID);
            break;

        case SYSTEM_EVENT_STA_CONNECTED:
            //enable sta ipv6 here
            WiFi.enableIpV6();
            break;

        case SYSTEM_EVENT_AP_STA_GOT_IP6:
            //both interfaces get the same event
            Serial.print("STA IPv6: ");
            Serial.println(WiFi.localIPv6());
            Serial.print("AP IPv6: ");
            Serial.println(WiFi.softAPIPv6());
            break;

        case SYSTEM_EVENT_STA_GOT_IP:
            wifiOnConnect();
            wifi_connected = true;
            break;

        case SYSTEM_EVENT_STA_DISCONNECTED:
            wifi_connected = false;
            wifiOnDisconnect();
            break;

        default:
            break;
    } */
}

static void send_message( uint16_t msgid, uint8_t *data, uint8_t dlc) {
  static twai_message_t message;
  static uint8_t dataBytes[] = {0, 0, 0, 0, 0, 0, 0, 0}; // initialize dataBytes array with 8 bytes of 0

  leds[0] = CRGB::Blue;
  FastLED.show();
  // Format message
  message.identifier = msgid;       // set message ID
  message.extd = 0;                 // 0 = standard frame, 1 = extended frame
  message.rtr = 0;                  // 0 = data frame, 1 = remote frame
  message.self = CAN_SELF_MSG;      // 0 = normal transmission, 1 = self reception request 
  message.dlc_non_comp = 0;         // non-compliant DLC (0-8 bytes)  
  message.data_length_code = dlc;   // data length code (0-8 bytes)
  memcpy(message.data, data, dlc);  // copy data to message data field 
  
  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(3000)) == ESP_OK) {
    // ESP_LOGI(TAG, "Message queued for transmission\n");
    // printf("Message queued for transmission\n");
  } else {
    leds[0] = CRGB::Red;
    FastLED.show();
    // ESP_LOGE(TAG, "Failed to queue message for transmission, initiating recovery");
    WebSerial.printf("ERR: Failed to queue message for transmission, resetting controller\n");
    twai_initiate_recovery();
    twai_stop();
    WebSerial.printf("WARN: twai Stoped\n");
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
  WebSerial.printf("Display: %d Mode: %d\n", rxdisplayID, displayMode);

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

static void txSwitchState(uint8_t *data, uint8_t switchID, uint8_t swState) {
  static uint8_t txDLC = 5;
  static uint8_t dataBytes[] = {data[0], data[1], data[2], data[3], switchID}; // set node id and switch ID
  
  WebSerial.printf("TX: To %02x:%02x:%02x:%02x Switch %d State %d\n", data[0],data[1],data[2],data[3], switchID, swState);

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
      WebSerial.println("Invalid switch state for transmission");
      break;
  }
}

static void rxSwitchState(uint8_t *data, uint8_t swState) {
  static uint8_t switchID = data[4]; // switch ID 
  // static uint8_t unitID[] = {data[0], data[1], data[2], data[3]}; // unit ID
  static uint8_t dataBytes[] = {myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3], switchID}; // send my own node ID, along with the switch number

  WebSerial.printf("RX: Set Switch %d State %d\n", switchID, swState);
  nodeSwitchState[switchID] = swState; // update switch state
  

  switch (swState) {
    case 0: // switch off
      send_message(DATA_OUTPUT_SWITCH_OFF, dataBytes, sizeof(dataBytes));
      break;
    case 1: // switch on
      send_message(DATA_OUTPUT_SWITCH_ON, dataBytes, sizeof(dataBytes));
      break;
    case 2: // momentary press
      send_message(DATA_OUTPUT_SWITCH_ON, dataBytes, sizeof(dataBytes));
      send_message(DATA_OUTPUT_SWITCH_OFF, dataBytes, sizeof(dataBytes));
      break;
    default:
      WebSerial.println("Invalid switch state");
      break;
  }
}

static void txSwitchMode(uint8_t *data, uint8_t txSwitchID, uint8_t swMode) {
  static uint8_t txDLC = 6;
  static uint8_t dataBytes[] = {data[0], data[1], data[2], data[3], txSwitchID, swMode}; // set node id switch ID
  WebSerial.printf("TX: To %02x:%02x:%02x:%02x Switch %d Mode %d\n",data[0], data[1], data[2], data[3], txSwitchID, swMode);
  send_message(SW_SET_MODE, dataBytes, txDLC);
}

static void rxSwitchMode(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint8_t switchMode = data[5]; // switch mode

  static uint8_t dataBytes[] = {myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3], switchID, switchMode}; // send my own node ID, along with the switch number

  WebSerial.printf("RX: Set Switch %d State %d\n", switchID, switchMode);
  send_message(DATA_OUTPUT_SWITCH_MODE, dataBytes, sizeof(dataBytes));    
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

static void txIntroduction() {
  // send 32-bit node id and 16-bit feature mask
  static uint8_t dataBytes[6] = { myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3], 
                                  myNodeFeatureMask[0], myNodeFeatureMask[1] }; 

  send_message(CAN_MY_TYPE, dataBytes, sizeof(dataBytes));
}

static void txIntroack(uint8_t* txNodeID) {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55}; // data bytes
  send_message(ACK_INTRODUCTION, txNodeID, 4);
}

static void nodeCheckStatus() {

}

static void handle_rx_message(twai_message_t &message) {
  // static twai_message_t altmessage;
  static bool msgFlag = false;
  leds[0] = CRGB::Orange;
  FastLED.show();


  if (message.data_length_code > 0) { // message contains data, check if it is for us
    static uint8_t rxUnitID[4] = {message.data[0], message.data[1], message.data[2], message.data[3]};
    static int comp = memcmp((const void *)rxUnitID, (const void *)myNodeID, 4);

  /*   if (comp == 0) {
      msgFlag = true; // message is for us
      leds[0] = CRGB::Green;
      FastLED.show();
      WebSerial.printf("Node Match MSG ID: 0x%x Data:", message.identifier);
    } else {
      msgFlag = false; // message is not for us
    
      WebSerial.printf("No Match MSG ID: 0x%x Data:", message.identifier);
    }
 */
    WebSerial.printf("RX: MSG 0x%x Data:", message.identifier);
    for (int i = 0; i < message.data_length_code; i++) {
      WebSerial.printf(" %d = %02x", i, message.data[i]);
    }
    WebSerial.println("");
  } else {
    msgFlag = true; // general broadcast message is valid
    WebSerial.printf("RX: MSG 0x%x NO DATA\n", message.identifier);
  }

  /*   
  if (msgFlag == false) {
    return; // message is not for us, exit function
  }
 */
  // if (!msgFlag) {
  //   WebSerial.println("Message does not match our ID.");
  // }

  switch (message.identifier) {
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
    case REQ_INTERFACES: // request for interface introduction     
      WebSerial.println("RX: IFACE intro req, responding");
      FLAG_SEND_INTRODUCTION = true; // set flag to send introduction message
      txIntroduction(); // send our introduction message
      break;
    case REQ_BOXES: // request for box introduction
      WebSerial.println("RX: BOX intro req, responding");
      FLAG_SEND_INTRODUCTION = true; // set flag to send introduction message
      txIntroduction(); // send our introduction message
      break;
    case ACK_INTRODUCTION:
      WebSerial.println("RX: Intro ACK, clearing flag");    
      FLAG_SEND_INTRODUCTION = false; // stop sending introduction messages
      break;
    
    default:
      if ((message.identifier & MASK_24BIT) == (INTRO_INTERFACE)) { // received an interface introduction
        WebSerial.printf("RX: IFACE intro from %02x:%02x:%02x:%02x\n", message.data[0], message.data[1], message.data[2], message.data[3]);
        txIntroack((uint8_t*) otherNodeID);
      } else
      if ((message.identifier & MASK_24BIT) == (INTRO_BOX)) { // received an interface introduction
        WebSerial.printf("RX: BOX intro from %02x:%02x:%02x:%02x\n", message.data[0], message.data[1], message.data[2], message.data[3]);
        txIntroack((uint8_t*) otherNodeID);
        txSwitchState((uint8_t *)otherNodeID, 7, 1); 
        txSwitchState((uint8_t *)otherNodeID, 7, 0); 
        txSwitchState((uint8_t *)otherNodeID, 7, 2);   
      }
      break;
  }

  leds[0] = CRGB::Black;
  FastLED.show();

} // end of handle_rx_message

void TaskTWAI(void *pvParameters) {
  // give some time at boot the cpu setup other parameters
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NO_ACK);  // TWAI_MODE_NO_ACK , TWAI_MODE_LISTEN_ONLY , TWAI_MODE_NORMAL
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

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
      leds[0] = CRGB::Red;
      FastLED.show();
      WebSerial.println("Alert: The RX queue is full causing a received frame to be lost.");
      WebSerial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
      WebSerial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
      WebSerial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
    }

    // Check if message is received
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
      // leds[0] = CRGB::Yellow;
      // FastLED.show();
      // Serial.println("Testing line");
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
      #ifdef M5PICO
      send_message(REQ_BOXES, NULL, 0); // send interface introduction request
      #elif M5STACK
      // send_message(REQ_INTERFACES, NULL, 0); // send interface introduction request
      #endif
    }
    vTaskDelay(10);

    nodeCheckStatus();
  }
}

void printWifi() {
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  delay(5000);

  // Timer0_Cfg = timerBegin(0, 80, true);
  // timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  // timerAlarmWrite(Timer0_Cfg, 100000, true);
  // timerAlarmEnable(Timer0_Cfg);

  xTaskCreate(
    TaskTWAI,     // Task function.
    "Task TWAI",  // name of task.
    3172,         // Stack size of task
    NULL,         // parameter of the task
    1,            // priority of the task
    NULL          // Task handle to keep track of created task
  );              // pin task to core 0
  //tskNO_AFFINITY); // pin task to core is automatic depends the load of each core

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

  Serial.begin(115200);
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
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
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
}

void loop() {
  ArduinoOTA.handle();
  // NOP;
}