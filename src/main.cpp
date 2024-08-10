// #define CANFILTER

#include <Arduino.h>
#include <myWifi.h>
#include <myPreferences.h>
#include <driver/twai.h>
#include "AsyncUDP.h"
#include "Adafruit_INA219.h"
#include "LittleFS.h"
#include <ArduinoJSON.h>
// put function declarations here:

#define SDA_0 42
#define SCL_0 41
#define CAN1_RX_PIN 2
#define CAN1_TX_PIN 1
#define CAN2_RX_PIN 2
#define CAN2_TX_PIN 1
#define keyPowerPin 15
#define batPowerPin 14

#pragma region GlobalClasses
  MyPrefrences myPrefs;
  MyWifi myWifi;
#pragma endregion

#pragma region GlobalStructs
  struct ProgramVars_t{
    uint8_t wifiConnected;
    uint8_t AOGconnected;
    uint8_t can1connected;
    uint8_t can2connected;
    byte ina219Connected;
    uint32_t udpTimer;
    uint32_t can1Timer;
    uint32_t can2Timer;
    byte version1;
    byte version2;
    byte version3;
  } progVars;

#pragma endregion

class UDPMethods{
  private:
    
    uint32_t udpTimeout = 1000;
    
    int heartbeatTimePrevious=0;
    int heartbeatTimeTrip=1000;
    int flowDataTimePrevious=0;
    int flowDataTimeTrip=1000;
  public:
    AsyncUDP udp;
    
    UDPMethods(){
    }
    
    void begin(){
      udp.listen(8888);
      Serial.println("UDP Listening");
      udp.onPacket([](AsyncUDPPacket packet) {
        if (packet.data()[0]==0x80 & packet.data()[1]==0x81){
          progVars.udpTimer = millis();
            
          }
      });
    }

    void udpCheck(){    // 📌  udpCheck 📝 🗑️
      if (millis() - progVars.udpTimer < udpTimeout){
        progVars.AOGconnected == 1;
      } else
      {
        progVars.AOGconnected == 2;
      }
      
    }

    void sendHeartbeat(){
      // TODO: add heartbeat
      // if (esp_timer_get_time()-heartbeatTimePrevious > heartbeatTimeTrip){
      //   heartbeatTimePrevious = esp_timer_get_time();
      //   hbData->aogByte1 = 0x80;
      //   hbData->aogByte2 = 0x81;
      //   hbData->sourceAddress = 51;
      //   hbData->PGN = HEARBEAT_PGN;
      //   hbData->length = 10;
      //   hbData->machineID = 2;
      //   hbData->udpState = programStates.udpConnected;
      //   int cksum=0;
      //   for (int i=2;i<=sizeof(heartbeatStruct_t)-1;i++)
      //   {
      //     cksum += heartbeat.bytes[i];
      //     // Serial.print(heartbeat.bytes[i]);
      //     // Serial.print(" ");
      //   }
      //   udp.writeTo(heartbeat.bytes,sizeof(heartbeatStruct_t),IPAddress(myWifi.address_1,myWifi.address_2,myWifi.address_3,255),9999);
        
      }
 
};
UDPMethods udpMethods = UDPMethods();

class CanHandler{
  private:
    
    byte TX_PIN = 14;
    byte RX_PIN = 13;

  public:
    
    byte udpBuffer[512];
    uint16_t cnt = 0;
    #pragma region UDPgrainFlow
      struct __attribute__ ((packed)) GrainFlow_t{
        uint8_t aogByte1 : 8;
        uint8_t aogByte2 : 8;
        uint8_t srcAddrs : 8;
        uint8_t PGN : 8;
        uint8_t length : 8;
        uint8_t myPGN : 8;
        uint16_t massFlow : 16;
        uint16_t moisture : 16;
        uint8_t checksum : 8;
      };

      union GrainFlow_u{
        GrainFlow_t grainFlow_t;
        uint8_t bytes[sizeof(GrainFlow_t)];
      } grainFlow;
    #pragma endregion
    #pragma region CANID
      union
      {
        struct
        {
          uint8_t byte0;
          uint8_t byte1;
          uint8_t byte2;
          uint8_t byte3;
        };
        uint32_t longint;
      } canID;
    #pragma endregion
    #pragma region GrainFlowMsg
      struct __attribute__ ((packed)) CanGrainFlow_t{
        uint16_t optocode : 16;
        uint16_t grainFlow : 16;
        uint16_t moisture : 16;
      };

      union CanGrainFlow_u{
        CanGrainFlow_t canGrainFlow_t;
        uint8_t bytes[sizeof(CanGrainFlow_t)];
      } canGrainFlow;
    #pragma endregion
    uint32_t scaleID = 0xCCBFF90;
    uint8_t scaleCANdata[8];
    CanHandler(){}


    byte startCAN(byte _RX_PIN, byte _TX_PIN){
      TX_PIN = _TX_PIN;
      RX_PIN = _RX_PIN;
      Serial.println("StartingCan");
      twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NO_ACK);  // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
      twai_timing_config_t t_config  = TWAI_TIMING_CONFIG_250KBITS();
      #ifndef CANFILTER
        twai_filter_config_t f_config  = TWAI_FILTER_CONFIG_ACCEPT_ALL();
      #endif
      
      #ifdef CANFILTER
        twai_filter_config_t f_config = {
          .acceptance_code = (0x18EEFF01<<3),    //Bit shift our target ID to the 29 MSBits
          .acceptance_mask = 0x7,    //Mask out our don't care bits (i.e., the 3 LSBits)
          .single_filter = true
        };
        twai_filter_config_t f_config;
        f_config.acceptance_code =   0x10EFFFD3<<3;
        f_config.acceptance_mask = ~(0x10111111<<3);
        f_config.single_filter = true;
       #endif
      twai_driver_install(&g_config, &t_config, &f_config);
      
      if (twai_start() == ESP_OK) {
        printf("Driver started\n");
        
      } else {
        printf("Failed to start driver\n");
        return 2;
      }
      
      twai_status_info_t status;
      twai_get_status_info(&status);
      Serial.print("TWAI state ");
      Serial.println(status.state);
      return 1;
    }
    
        
    void handle_tx_message(twai_message_t message)
    {
      esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(100));
      if (result == ESP_OK){
      }
      else {
        Serial.printf("\n%s: Failed to queue the message for transmission.\n", esp_err_to_name(result));
      }
    }

    void transmit_normal_message(uint32_t identifier, uint8_t data[], uint8_t data_length_code = TWAI_FRAME_MAX_DLC)
      {
        // configure message to transmit
        twai_message_t message = {
          .flags = TWAI_MSG_FLAG_EXTD,
          .identifier = identifier,
          .data_length_code = data_length_code,
        };

        memcpy(message.data, data, data_length_code);

        //Transmit messages using self reception request
        handle_tx_message(message);
      }

    
    void checkCAN(){
      
    }

    void canRecieve(){
      twai_message_t message;
      if (twai_receive(&message, pdMS_TO_TICKS(1)) == ESP_OK) {
          canID.longint = message.identifier;
          if (canID.byte1 == 0xEE && canID.byte2 == 0xFF){
            for (int i=0;i<message.data_length_code; i++){
              canGrainFlow.bytes[i] = message.data[i];
            }
            if (canGrainFlow.canGrainFlow_t.optocode == 2383){
              grainFlow.grainFlow_t.massFlow = canGrainFlow.canGrainFlow_t.grainFlow;
              grainFlow.grainFlow_t.moisture = canGrainFlow.canGrainFlow_t.moisture;
              udpMethods.udp.writeTo(grainFlow.bytes,sizeof(GrainFlow_t),IPAddress(myPrefs.ips[0],myPrefs.ips[1],myPrefs.ips[2],255),9999);
             
            }
            
            
          }
         

          #pragma region UDPcanbuffer
            udpBuffer[cnt * 12 + 0] = canID.byte0;
            udpBuffer[cnt * 12 + 1] = canID.byte1;
            udpBuffer[cnt * 12 + 2] = canID.byte2;
            udpBuffer[cnt * 12 + 3] = canID.byte3;
            for (int i=0;i<message.data_length_code; i++){
              udpBuffer[cnt*12+4+i] = message.data[i];
            }
            cnt++;

            if (cnt == 40){
              Serial.print(millis());
              Serial.print(" ");
              Serial.println("dataLoadFull ");
              cnt = 0;
              for (int i = 0; i < sizeof(udpBuffer); i++){
                udpBuffer[i] = 0;
              }
            }
          #pragma endregion
        }
      }

    void sendCANbuffer(){
      uint8_t outgoingData[512];
      outgoingData[0] = 0x80;
      outgoingData[1] = 0x81;
      outgoingData[2] = 0;
      outgoingData[3] = 149;
      outgoingData[4] = 512;
      outgoingData[5] = 2;
      for (int i = 6; i < 512; i++){
        outgoingData[i] = udpBuffer[i-6];
      }
      udpMethods.udp.writeTo(outgoingData,512,IPAddress(myPrefs.ips[0],myPrefs.ips[1],myPrefs.ips[2],255),9999);
    }
    
};
CanHandler canHandler1 = CanHandler();
CanHandler canHandler2 = CanHandler();

class BusMonitor{
  private:

  public:
    float current_mA = 0;
    float busvoltage = 0;

    Adafruit_INA219 ina219;
    BusMonitor(){}

    byte begin(){
      if (! ina219.begin()) {
        Serial.println("Failed to find INA219 chip");
        return 2;
      }
      return 1;
    }
    
    void sampleBus(){
      busvoltage = ina219.getBusVoltage_V();
      current_mA = ina219.getCurrent_mA();
    }
    
};
BusMonitor busMonitor = BusMonitor();

class KeyPowerMonitor{
  private:
    byte keyPin;
    byte batPin;
  public:
    
    KeyPowerMonitor(){}

    void begin(byte _keyPin, byte _batPin){
      keyPin = _keyPin;
      batPin = _batPin;
      pinMode(keyPin, INPUT);
      pinMode(batPin, OUTPUT);
      

    }
    void turnOnBattery(){
      digitalWrite(batPin, HIGH);
    }

    void turnOffBattery(){
      
      digitalWrite(batPin, LOW);
    }

    void sampleInput(){
      analogReadMilliVolts(keyPin);
    }
};
KeyPowerMonitor keyMonitor = KeyPowerMonitor();

class Configuration{
  private:

  public:
    Configuration(){}

    bool begin() {
      if (!LittleFS.begin(true)) {
        Serial.println("An error has occurred while mounting LittleFS");
        return false;
      }
      Serial.println("LittleFS mounted successfully");

      File file = LittleFS.open("/config.json", "r");
      if (!file) {
        Serial.println("Failed to open file for reading");
        return false;
      }

      String jsonString;
      while (file.available()) {
          jsonString += char(file.read());
      }
      file.close();
      
      // Print the JSON string to verify
      Serial.println(jsonString);

      const size_t capacity = JSON_OBJECT_SIZE(10) + 200;
      DynamicJsonDocument doc(capacity);

      // Parse the JSON string
      DeserializationError error = deserializeJson(doc, jsonString);
      if (error) {
          Serial.print("Failed to parse JSON: ");
          Serial.println(error.c_str());
          return false;
      }
      // Access the JSON data
      const char* value = doc["key"];
      Serial.println(value);
      return true;
    }
};
Configuration config = Configuration();



void setup(){
  Serial.begin(115200);
  config.begin();
  myPrefs.begin();
  keyMonitor.begin(keyPowerPin, batPowerPin);
  progVars.wifiConnected = myWifi.connect(myPrefs.getIPAddr());
  udpMethods.begin();
  progVars.can1connected = canHandler1.startCAN(13, 14);
  progVars.can2connected = canHandler2.startCAN(11, 12);
  progVars.ina219Connected = busMonitor.begin();
  
}

void loop(){
  if (progVars.can1connected == 1)
  {
    canHandler1.canRecieve();
    canHandler1.checkCAN();
  }
  if (progVars.can2connected == 1)
  {
    canHandler2.canRecieve();
    canHandler2.checkCAN();
  }
  if (progVars.ina219Connected == 1){
    busMonitor.sampleBus();
  }
  
  
  udpMethods.udpCheck();

}