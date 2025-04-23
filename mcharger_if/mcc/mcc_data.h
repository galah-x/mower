//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'mcc_data.h  Time-stamp: "2025-04-01 11:32:16


#include <Preferences.h>  // the NV memory interface
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "ESP32Time.h"   // must be V2.x or newer. V1.04 takes 5s to return a time string. breaks logging 
#include <stdio.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <hd44780.h>   // lcd library
#include <hd44780ioClass/hd44780_I2Cexp.h> // include i/o class header
#include <RotaryEncoder.h>
#include "FS.h"            // filesystem for sd logging
#include "SD.h"            // SD apparently uses ram more effectively than SDFat
#include "SPI.h"           // SPI access to sd card
#include <WiFiUdp.h>


// for preferences
#define RW_MODE false
#define RO_MODE true


// serial is used for debug and for programming NVM
const uint8_t longest_record = 24;  //   worst is display comand WD1=12345678901234567890
const uint8_t record_size = longest_record + 2;    //  char char = nnnnn \n
uint8_t serial_buf[record_size];
uint8_t serial_buf_pointer;

// this is used to buffer up the SD logging
const uint16_t sdblksize = 4096; // blksize is always 512, cluster size usually 4k. 
uint8_t sdbuf[sdblksize];
uint16_t sdbuf_ptr;

uint8_t baseMac[6];         // my own mac address
const  uint8_t msgbuflen= 128;  // for wifi transfers
uint8_t return_buf[msgbuflen]; // for responses

const char * version = "MCC 11 Apr 2025 Rev2S";

Preferences mccPrefs;  // NVM structure
// these will be initialized from the NV memory

int     fan_on_temp;
int     fan_off_temp;
bool    logging;
bool    writing_to_log;
uint8_t psu_mac[6];
uint8_t mco_mac[6];

// these are for WiFi access when moving the logfile around   
const uint8_t ssid_len = 40;
char   ssid[ssid_len];
char   password[ssid_len];

// this inhibits wifi updates writing to the display while the UI is being used.
// Doesn't stop logging of 'lost' screen data.
bool UI_owns_display;

typedef struct struct_message {
  uint8_t message[msgbuflen];
} struct_message;

// create the wifi message struct
struct_message message_data;

esp_now_peer_info_t peerInfo;


/* IOs  definitions */
const uint8_t enc_an_pin  = 34;    // rotary encoder a
const uint8_t enc_bn_pin  = 35;    // rotary encoder b
const uint8_t enc_pushn_pin = 33;    // rotary encoder pushbutton
const uint8_t spi_sclk_pin = 14;
const uint8_t spi_cs_pin   = 15;  // sdcard
const uint8_t spi_mosi_pin = 13;
const uint8_t spi_miso_pin = 12;  
const uint8_t power48V_en_pin = 25;    // turn on 48V to mower
const uint8_t fan_en_pin = 27;    // turn on fan 
const uint8_t sda_pin   = 21;    // fan3
const uint8_t scl_pin   = 22;    // fan3

// I2C addresses 
const uint8_t hs_temperature_addr = 0x76;
const uint8_t display_addr = 0x27;

Adafruit_BMP280 tsense;    // I2C connected bmp280 temperature and pressure sensor. Just used for temps.

// A pointer to the dynamic created rotary encoder instance.
// This will be used in setup()
RotaryEncoder *encoder = nullptr;
int     enc_pos;
uint8_t enc_pushn_state;
int     enc_change = 0;
uint8_t enc_press = 0;
int     enc_press_debounce = 50; //msecs
bool    show_menu;
int     menu;
int     menu_line;
const uint8_t cursor = '>';
int     hs_temperature;    // stored value of heatsink temperature for the mower switch.
const char * logfile = "/log.txt";

File file;

// this is the encoder ISR. Not at all sure I want to use the encoder interrupts. Just poll in loop() for now.
//IRAM_ATTR void checkPosition()
//{
//  encoder->tick(); // just call tick() to check the state.
//}

// Going to do all my basic event timing and logging in seconds, so will use ESP32Time for convenience.
// not going to bother setting the clock, just demark time in HMS from whenever the board last got
// turned on.
ESP32Time rtc;
unsigned long last_temp_update_time;         // time last temperature was polled at, in seconds
const unsigned long temp_update_period = 10; // seconds.

unsigned long last_psu_update_time;         // time last power supply voltage and current was polled at
const unsigned long psu_update_period = 2;  // seconds.

// logfile flushes when the buffer is full.
// Manually flush via UI on termination if last fragment of log is wanted.

// create lcd object
hd44780_I2Cexp lcd;

bool espnow = true;

const uint8_t server_len = 15;
char server[server_len];
uint16_t LogPort;

//The udp library class
WiFiUDP udp;
// Use WiFiClient class to create TCP connections
WiFiClient client;

bool connected = false;

const uint8_t default_mac[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
