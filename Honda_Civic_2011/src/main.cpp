// Honda Civic 1.8 Version without Brightness Control, RPM vs. Coolant Temp.
#include "BluetoothSerial.h"
#include "ELMduino.h"
#include "Wire.h"
#include "math.h"
#include <BH1750.h>
// stuff for removing bonding
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include"esp_gap_bt_api.h"
#include "esp_err.h"

#define GR_CORR 0.3

BluetoothSerial SerialBT;
#define ELM_PORT   SerialBT
#define DEBUG_PORT Serial

#define RES_PWM 10
#define LED_MAX 1024


/*GPIOs*/

//LEFT ID = 2
#define LEFT_RGB 2
#define RGB_LEFT_RED 32
#define RGB_LEFT_BLUE 33
#define RGB_LEFT_GREEN 18

#define RIGHT_RGB 1
#define RGB_RIGHT_RED 16
#define RGB_RIGHT_GREEN 17
#define RGB_RIGHT_BLUE 4

#define WARNING_LED 25
#define STATUS_LED 26
#define OIL_LED 27
#define DPF_LED 5

//bonding removal stuff
#define REMOVE_BONDED_DEVICES 1   // <- Set to 0 to view all bonded devices addresses, set to 1 to remove
#define PAIR_MAX_DEVICES 20
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];

ELM327 myELM327;

BH1750 lightMeter;

struct RGBColor
{
  int R;
  int G;
  int B;
};

RGBColor HeatScale[255];

uint32_t MY_ENGINE_RPM = 0;
uint32_t MY_ENGINE_OIL_TEMP = 0;
uint32_t MY_ENGINE_COOLANT_TEMP = 0;
uint32_t MY_REGEN_COUNT = 0;
uint32_t MY_REGEN_STATE = 0;


float currentBrightnessTarget = 0.01;
int cyclecounter = 0;

void setSingleLEDValue(uint8_t ID, uint16_t value, float brightness);
void setRGBLEDColor(uint8_t ID, uint16_t R, uint16_t G, uint16_t B, float brightness);
float getBackgroundLightLevel();

void removeAllBonded();
bool initBluetooth();
char *bda2str(const uint8_t* bda, char *str, size_t size);
uint8_t errorCount = 0;
uint8_t queryFlag = 0;
uint8_t loopCount = 0;

void setup()
{

  //create heatscale
  int j=0;
  for(j=0;j<256;j++)
  {
    if(j>=0 && j<=40)
    {
      HeatScale[j].B = 1000;
      HeatScale[j].R = 0;
      HeatScale[j].G = 0; 
    }
    if(j>40 && j<=80)
    {
      HeatScale[j].B = 1000;
      HeatScale[j].R = (j-40)*25;
      HeatScale[j].G = (j-40)*25; 
    }
    if(j>80 && j<=110)
    {
      HeatScale[j].B = 990-(j-80)*33;
      HeatScale[j].R = 1000;
      HeatScale[j].G = 1000; 
    }
    if(j>110 && j<=140)
    {
      HeatScale[j].B = 0;
      HeatScale[j].R = 1000;
      HeatScale[j].G = 1000-(j-110)*33; 
    }
    if(j>140 && j<255)
    {
      HeatScale[j].B = 0;
      HeatScale[j].R = 1000;
      HeatScale[j].G = 0; 
    }
  }


  //-------------------SETUP LEDs---------------------
  ledcSetup(0, 1000, RES_PWM);
  ledcSetup(1, 1000, RES_PWM);
  ledcSetup(2, 1000, RES_PWM);
  ledcSetup(3, 1000, RES_PWM);
  ledcSetup(4, 1000, RES_PWM);
  ledcSetup(5, 1000, RES_PWM);
  ledcSetup(6, 1000, RES_PWM);
  ledcSetup(7, 1000, RES_PWM);
  ledcSetup(8, 1000, RES_PWM);
  ledcSetup(9, 1000, RES_PWM);

  ledcAttachPin(RGB_RIGHT_RED, 0);
  ledcAttachPin(RGB_RIGHT_GREEN, 1);
  ledcAttachPin(RGB_RIGHT_BLUE, 2);
  ledcAttachPin(RGB_LEFT_RED, 3);
  ledcAttachPin(RGB_LEFT_GREEN, 4);
  ledcAttachPin(RGB_LEFT_BLUE, 5);
  ledcAttachPin(WARNING_LED, 6);
  ledcAttachPin(STATUS_LED,7);
  ledcAttachPin(OIL_LED, 8);
  ledcAttachPin(DPF_LED, 9);

  int i = 0;
   for(i=0;i<10;i++)
    {
      ledcWrite(i, 1024);
    }

  for(i=0;i<256;i++)
  {
    setRGBLEDColor(RIGHT_RGB,HeatScale[i].R,HeatScale[i].G,HeatScale[i].B,1);
    setRGBLEDColor(LEFT_RGB,HeatScale[i].R,HeatScale[i].G,HeatScale[i].B,1);
    delay(5);
  }

  //-------------------SETUP LEDs END---------------------
  
  setSingleLEDValue(WARNING_LED,1023,1);
  
  // at start remove all bonded devices.  
  removeAllBonded();
  //removed all bonded devices
  
  //startup light measurement system
  //Wire.begin();
  //lightMeter.begin(BH1750::ONE_TIME_LOW_RES_MODE); 
  
  //start connection to OBDII device
  ELM_PORT.begin("OBDII", true);
  //ELM_PORT.setPin("0000");  
  if (!ELM_PORT.connect("OBDII"))
  {
    DEBUG_PORT.println("Couldn't connect to OBD scanner - Phase 1");
    ESP.restart();  //reboot on error
  }

  if (!myELM327.begin(ELM_PORT, false, 2000))
  {
    Serial.println("Couldn't connect to OBD scanner - Phase 2");
    ESP.restart();  //reboot on error
  }

  Serial.println("Connected to ELM327");
  setRGBLEDColor(RIGHT_RGB,0,0,0,0);
  setRGBLEDColor(LEFT_RGB,0,0,0,0);
  setSingleLEDValue(WARNING_LED,0,0);
  setSingleLEDValue(STATUS_LED,1023,1);

}


void loop()
{
  /*-------------------------START OF MAIN LOOP--------------------------*/
  loopCount++;
  loopCount%=255;
  if(loopCount%200 == 0)
  {
   currentBrightnessTarget = getBackgroundLightLevel();
  }

  if(loopCount%100 == 0)
  {
    switch (queryFlag)
    {
    case 0: 
      MY_ENGINE_COOLANT_TEMP = myELM327.engineCoolantTemp();
      break;
    case 1:
      MY_ENGINE_RPM = myELM327.rpm();
      break;
    default:
      break;
    }

    if(myELM327.nb_rx_state == ELM_SUCCESS)
    {
      setSingleLEDValue(STATUS_LED,1024,1);
      setSingleLEDValue(WARNING_LED,0,0);
      queryFlag++;
      queryFlag%=2;
      errorCount = 0;
      if(MY_REGEN_STATE != 0)
      {
        setSingleLEDValue(DPF_LED,1020,1);
      }        
      else
      {
        setSingleLEDValue(DPF_LED,0,0);
      }

      setRGBLEDColor(LEFT_RGB,HeatScale[((MY_ENGINE_RPM)/40)+30].R,HeatScale[((MY_ENGINE_RPM)/40)+30].G,HeatScale[((MY_ENGINE_RPM)/40)+30].B,0.5);
      setRGBLEDColor(RIGHT_RGB,HeatScale[MY_ENGINE_COOLANT_TEMP+60].R,HeatScale[MY_ENGINE_COOLANT_TEMP+60].G,HeatScale[MY_ENGINE_COOLANT_TEMP+60].B,0.5);

      //Serial.print("Oil temp: "); 
      //Serial.println(MY_ENGINE_OIL_TEMP);
      //Serial.print("Coolant temp: "); 
      //Serial.println(MY_ENGINE_COOLANT_TEMP);
      //Serial.print("RPM: "); 
      //Serial.println(MY_ENGINE_RPM); 
      //Serial.print("Regeneration count: "); 
      //Serial.println(MY_REGEN_COUNT); 
      //Serial.print("Regeneration State: "); 
      //Serial.println(MY_REGEN_STATE); 
    }
    
    else if(myELM327.nb_rx_state != ELM_GETTING_MSG)
    {
      setSingleLEDValue(STATUS_LED,0,0);
      setSingleLEDValue(WARNING_LED,1024,1);
      Serial.println("ELM ERROR OCCURRED");
      myELM327.printError();
      errorCount++;
    }

  }

  /*If too much errors occur without a single good received data.*/
  if(errorCount>30)
  {
    ESP.restart();
  }
  /*-------------------------END OF MAIN LOOP--------------------------*/
}

void setRGBLEDColor(uint8_t ID, uint16_t R, uint16_t G, uint16_t B, float brightness)
{
  if(brightness>1)
    brightness = 1;
  if(brightness<0)
    brightness = 0;

  if(ID == RIGHT_RGB)
  {
    //RIGHT LED (ENGINE OIL SELECTED)
    ledcWrite(0, (LED_MAX-R*brightness));
    ledcWrite(1, (LED_MAX-G*brightness*GR_CORR));
    ledcWrite(2, (LED_MAX-B*brightness));
  }
  if(ID == LEFT_RGB)
  {
    //LEFT LED (COOLANT SELECTED)
    ledcWrite(3, (LED_MAX-R*brightness));
    ledcWrite(4, (LED_MAX-G*brightness*GR_CORR));
    ledcWrite(5, (LED_MAX-B*brightness));
  }
}

void setSingleLEDValue(uint8_t ID, uint16_t value, float brightness)
{
  value = value*brightness;
  if(value>=0)
    {
    switch (ID)
    {
    case WARNING_LED:
      ledcWrite(6,(LED_MAX-value)*(1-brightness));
      break;

    case STATUS_LED:
      ledcWrite(7,(LED_MAX-value)*(1-brightness));
      break;

    case OIL_LED:
      ledcWrite(8,(LED_MAX-value)*(1-brightness));
      break;

    case DPF_LED:
      ledcWrite(9,(LED_MAX-value)*(1-brightness));
      break;
    
    default:
      break;
    }
  }
}

float getBackgroundLightLevel()
{
  while (!lightMeter.measurementReady(true)) 
  {
    yield();
  }
  float lux = lightMeter.readLightLevel();
  lux = lux/54700;
  lightMeter.configure(BH1750::ONE_TIME_HIGH_RES_MODE);
  return lux;
}

void removeAllBonded()
{
  
  DEBUG_PORT.begin(115200);
  initBluetooth();
  DEBUG_PORT.print("ESP32 bluetooth address: ");
  DEBUG_PORT.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));
  // Get the numbers of bonded/paired devices in the BT module
  int count = esp_bt_gap_get_bond_device_num();
  if(!count) {
    DEBUG_PORT.print("No bonded device found.");
  } else {
    DEBUG_PORT.print("Bonded device count: "); DEBUG_PORT.println(count);
    if(PAIR_MAX_DEVICES < count) {
      count = PAIR_MAX_DEVICES; 
      DEBUG_PORT.print("Reset bonded device count: "); DEBUG_PORT.println(count);
    }
    esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    if(ESP_OK == tError) {
      for(int i = 0; i < count; i++) {
        DEBUG_PORT.print("Found bonded device # "); DEBUG_PORT.print(i); DEBUG_PORT.print(" -> ");
        DEBUG_PORT.print(bda2str(pairedDeviceBtAddr[i], bda_str, 18));     
        if(REMOVE_BONDED_DEVICES) {
          esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
          if(ESP_OK == tError) {
            DEBUG_PORT.print("Removed bonded device # "); 
          } else {
            DEBUG_PORT.print("Failed to remove bonded device # ");
          }
          DEBUG_PORT.println(i);
        }
      }        
    }
  }
}

char *bda2str(const uint8_t* bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}

bool initBluetooth()
{
  if(!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }
 
  if(esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }
 
  if(esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
  return true;
}