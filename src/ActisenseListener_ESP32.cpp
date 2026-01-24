// define to include or exclude the display code
#define USE_DISPLAY

#define ESP32_CAN_TX_PIN GPIO_NUM_5
#define ESP32_CAN_RX_PIN GPIO_NUM_4
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/pcnt.h"
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <NMEA2000_CAN.h>
#include <ActisenseReader.h>
#include <N2KDeviceList.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WebServer.h>

// Instantiate display, use HW I2C
#ifdef USE_DISPLAY
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 32, /* data=*/ 33); // for rev .9 of usb n2k board
#endif

// buffer used for displaying text
volatile char dispString[64] = {0};

// N2K device list and misc stuff
tN2kDeviceList *locN2KDeviceList;
uint8_t n2kConnected = 0; // # stations on bus
tActisenseReader ActisenseReader;
#define READ_STREAM Serial       
#define FORWARD_STREAM Serial    
Stream *ReadStream=&READ_STREAM;
Stream *ForwardStream=&FORWARD_STREAM;

// tweaked the NMEA2000_ESP32 driver to count tx/rx frames. Ugly but works.
// files from the "extras" directory need to be copied into the library once
// PlatformIO has downloaded the github lib
// modified file NMEA2000_ESP32.cpp and NMEA2000_ESP32.h to include 2 global counts of RX and TX Can frames
extern uint16_t canRxFrame;
extern uint16_t canTxFrame;

// forward declaration
void HandleStreamN2kMsg(const tN2kMsg &);
void displayDriverTask(void *parameter);

// externs from otaWeb
extern void otaSetup(void);
extern WebServer server;
extern bool startUpDelayDone;
extern bool fileUploadStarted;

// use this for N2K
u_int32_t chipId;

// main setup for Arduino like code
void setup() {

  uint8_t chipid [ 6 ];

  // derive a unique chip id from the burned in MAC address
  esp_efuse_mac_get_default ( chipid );
  for ( int i = 0 ; i < 6 ; i++ )
  chipId += ( chipid [ i ] << ( 7 * i ));

  // setup ota stuff
  otaSetup();

  // Define buffers big enough
  NMEA2000.SetN2kCANSendFrameBufSize(500);
  NMEA2000.SetN2kCANReceiveFrameBufSize(500);
  
  if (ReadStream!=ForwardStream) 
      READ_STREAM.begin(115200);

  FORWARD_STREAM.begin(115200);
  NMEA2000.SetForwardStream(ForwardStream); 
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndSend);
   
  // If streams are same, do not echo own messages.
  if (ReadStream==ForwardStream) 
      NMEA2000.SetForwardOwnMessages(false);
  
  // capture who is on N2K bus, should have at least 1
  locN2KDeviceList = new tN2kDeviceList(&NMEA2000); 
  NMEA2000.Open();

  // I originally had problem to use same Serial stream for reading and sending.
  // It worked for a while, but then stopped. Later it started to work.
  ActisenseReader.SetReadStream(ReadStream);
  ActisenseReader.SetDefaultSource(43);
  ActisenseReader.SetMsgHandler(HandleStreamN2kMsg); 

  // task for the I2C display
  xTaskCreatePinnedToCore(
    displayDriverTask, 
    "Display Task",
    4096, 
    NULL, 
    15, 
    NULL, 
    0);
} // end setup

void HandleStreamN2kMsg(const tN2kMsg &N2kMsg) {
  // N2kMsg.Print(&Serial);
  NMEA2000.SendMsg(N2kMsg,-1);
} 

// handle all N2K stuff in main Arduino loop
void loop() {
  
  // web ota stuff
  server.handleClient();

  NMEA2000.ParseMessages(); // from n2k bus?
  ActisenseReader.ParseMessages(); // from USB serial
  // record the number of devices on n2k bus
  n2kConnected = locN2KDeviceList->Count();
} // end loop

// hack to improve the esp32 adc linearity
double ReadVoltage(byte pin){
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if(reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
} // Added an improved polynomial, use either, comment out as required

// Task to handle the display, make sure priority is low so we dont starve
// N2K stuff
void displayDriverTask(void *parameter) {

  #ifdef USE_DISPLAY
  // try fast I2C speed
  u8g2.setBusClock(400000);
  // start the display, on I2C at address 3C
  u8g2.begin();
  double volts=0;
  uint16_t voltsRaw;
  // for better accuracy
  analogSetAttenuation(ADC_11db);

  // loop for handling updating the display
  while (true) {
    u8g2.clearBuffer();				// clear the internal memory
    // display update
    if (fileUploadStarted != true) {

      // read voltage and then adjust based on resister divider
      volts = (float)ReadVoltage(GPIO_NUM_35)*4.8;

      u8g2.setFont(u8g2_font_9x18B_tf);	// choose a suitable font
      // display devices
      sprintf((char *)&dispString, "St:%03d %05.2fV", n2kConnected, volts);
      //sprintf((char *)&dispString, "St:%03d %04d", n2kConnected, voltsRaw);
      u8g2.drawStr(3,20,(char *)&dispString);	// write something to the internal memory
      // display packet counts
      sprintf((char *)&dispString, "Can Rx: %05d", canRxFrame);
      u8g2.drawStr(5,40,(char *)&dispString);	// write something to the internal memory
      sprintf((char *)&dispString, "Can Tx: %05d", canTxFrame);
      u8g2.drawStr(5,60,(char *)&dispString);	// write something to the internal memory
    } else {
      // display devices
      sprintf((char *)&dispString, "Upload In");
      u8g2.drawStr(5,30,(char *)&dispString);	// write something to the internal memory
      // display packet counts
      sprintf((char *)&dispString, "Progress");
      u8g2.drawStr(5,50,(char *)&dispString);	// write something to the internal memory
    } // end if
    u8g2.sendBuffer();					// transfer internal memory to the display
    // add a small delay to yield
    vTaskDelay(pdMS_TO_TICKS(500));
  } // end loop
  #endif
  // should never get here but if we do cleanup
  vTaskDelete(NULL);
} // end dispaly driver task