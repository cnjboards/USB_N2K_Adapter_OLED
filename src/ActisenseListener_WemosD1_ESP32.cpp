// define to include or exclude the display code
#define USE_DISPLAY
#define SSD1306

//   Demo: NMEA2000 library. Bus listener and sender. 
//   Sends all bus data to serial in Actisense format.
//   Send all data received from serial in Actisense format to the N2kBus.
//   Use this e.g. with NMEA Simulator (see. http://www.kave.fi/Apps/index.html) to send simulated data to the bus.
//   I have plans to add receiving functionality and data forwarding to NMEA Simulator. Meanwhile you can define
//   other stream to different port so that you can send data with NMEA Simulator and listen it on other port with 
//   Actisense NMEA Reader.

//#define N2k_CAN_INT_PIN 21
#define ESP32_CAN_TX_PIN GPIO_NUM_5 // If you use ESP32 and do not have TX on default IO 16, uncomment this and and modify definition to match your CAN TX pin.
#define ESP32_CAN_RX_PIN GPIO_NUM_4 // If you use ESP32 and do not have RX on default IO 4, uncomment this and and modify definition to match your CAN TX pin.
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
#include "i2c_scanner.h"
#include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder.git 

// Instantiate display, use HW I2C
#ifdef SSD1306
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 32, /* data=*/ 33); // for rev .9 of usb n2k board
#endif

// buffer used for displaying text
volatile char dispString[64] = {0};

tN2kDeviceList *locN2KDeviceList;
uint8_t n2kConnected = 0; // # stations on bus

// tweaked the NMEA2000_ESP32 driver to count tx/rx frames. Ugly but works.
// files from the "extras" directory need to be copied into the library once
// PlatformIO has downloaded the github lib
// modified file NMEA2000_ESP32.cpp and NMEA2000_ESP32.h to include 2 global counts of RX and TX Can frames
extern uint16_t canRxFrame;
extern uint16_t canTxFrame;

// used for N2K
tActisenseReader ActisenseReader;

// Define READ_STREAM to port, where you write data from PC e.g. with NMEA Simulator.
#define READ_STREAM Serial       
// Define ForwardStream to port, what you listen on PC side. On Arduino Due you can use e.g. SerialUSB
#define FORWARD_STREAM Serial    

Stream *ReadStream=&READ_STREAM;
Stream *ForwardStream=&FORWARD_STREAM;

// forward declaration
void HandleStreamN2kMsg(const tN2kMsg &);
void displayDriverTask(void *parameter);

// 
void setup() {

  // Define buffers big enough
  NMEA2000.SetN2kCANSendFrameBufSize(500);
  NMEA2000.SetN2kCANReceiveFrameBufSize(500);
  
  if (ReadStream!=ForwardStream) READ_STREAM.begin(115200);
  FORWARD_STREAM.begin(115200);
  NMEA2000.SetForwardStream(ForwardStream); 
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndSend);
  // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show bus data in clear text
  if (ReadStream==ForwardStream) NMEA2000.SetForwardOwnMessages(false); // If streams are same, do not echo own messages.
  // NMEA2000.EnableForward(false);
  locN2KDeviceList = new tN2kDeviceList(&NMEA2000); // capture who is on N2K bus, should have at least 1
  NMEA2000.Open();

  // I originally had problem to use same Serial stream for reading and sending.
  // It worked for a while, but then stopped. Later it started to work.
  ActisenseReader.SetReadStream(ReadStream);
  ActisenseReader.SetDefaultSource(43);
  ActisenseReader.SetMsgHandler(HandleStreamN2kMsg); 

  // task for the display
  xTaskCreatePinnedToCore(
    displayDriverTask, 
    "Task for display",
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

// handle all N2K stuff in main loop
void loop() {
  NMEA2000.ParseMessages(); // from n2k bus?
  ActisenseReader.ParseMessages(); // from USB serial
  // record the number of devices on n2k bus
  n2kConnected = locN2KDeviceList->Count();
} // end loop

// Task to handle the display, make sure priority is low so we dont starve
// NMEA2000 stuff for cpu cycles
void displayDriverTask(void *parameter) {

  #ifdef ROTENC
  // setup discrete inputs
  pinMode(CON, INPUT_PULLUP);
  pinMode(BCK, INPUT_PULLUP);
  pinMode(ROTPB, INPUT_PULLUP);

  myEncoder.attachHalfQuad ( DT, CLK );
  myEncoder.setCount ( 0 );
  #endif
  // use to debug I2C
  #ifdef I2CSCAN
  I2CScanner myScan;
  while(true) {
    myScan.Init();
    myScan.Scan();
    vTaskDelay(pdMS_TO_TICKS(2000));    
  } // end while
  #endif

  #ifdef USE_DISPLAY
  // try fast I2C speed
  u8g2.setBusClock(400000);
  // start the display, on I2C at address 3C
  u8g2.begin();

  // loop for handling updating the display
  while (true) {
    u8g2.clearBuffer();					// clear the internal memory
    u8g2.setFont(u8g2_font_9x18B_tf);	// choose a suitable font
    #ifdef ROTENC
    sprintf((char *)&dispString, "POSN: %05d", (int)(myEncoder.getCount() / 2));
    u8g2.drawStr(5,20,(char *)&dispString);	// write something to the internal memory
    #else
    sprintf((char *)&dispString, "Devices:  %03d", n2kConnected);
    u8g2.drawStr(5,20,(char *)&dispString);	// write something to the internal memory
    #endif

    sprintf((char *)&dispString, "Can Rx: %05d", canRxFrame);
    u8g2.drawStr(5,40,(char *)&dispString);	// write something to the internal memory
    sprintf((char *)&dispString, "Can Tx: %05d", canTxFrame);
    u8g2.drawStr(5,60,(char *)&dispString);	// write something to the internal memory
    u8g2.sendBuffer();					// transfer internal memory to the display

    // add a small delay to yield
    vTaskDelay(pdMS_TO_TICKS(500));
  } // end loop
  #endif
  // should never get here but if we do cleanup
  vTaskDelete(NULL);
} // end dispaly driver task