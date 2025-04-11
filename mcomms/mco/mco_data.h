//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'data and variables for mco  Time-stamp: "2025-04-11 12:10:41 john"';


// test cmt MAC is 5c013b6c9938
// test mco MAC is 5c013b6c6a44   which replaces the cmt later in the dev process Mower COntroller
// test psu MAC is       wifi to serial adapter for the power supply. Originally a vichy mm adaptor
// test mcc MAC (ME) is 5c013b6cf4fc   MowerChargeController
// test vmon1 MAC (ME) is 5c013b6c7d14   vmon # 1
// test vmon2 MAC (ME) is 5c013b660c9c   vmon # 2
// test vmon3 MAC (ME) is 240ac4ee0360   vmon # 3 (smoked i2c with a spanner. replaced soc.)
// test vmon4 MAC (ME) is 5c013b6cea48   vmon # 4
// test imon           is 9c9c1fc6f7ac

#include <Preferences.h>  // the NV memory interface
#include <Wire.h>
#include "ESP32Time.h"   // must be V2.x or newer. V1.04 takes 5s to return a time string. breaks logging 
// #include <ADS1115_WE.h>   // adc converter isn't used and might not be fitted. So lets not depend on it
// fram
#include <stdio.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
// #include "FRAM.h"            // works, but seems incompatible with ADS1115_WE
#include "Adafruit_FRAM_I2C.h"  // only supports single byte access, but doesn't break ADC.

// #define DEBUG
// #define MEAS_PERF

#define DC_BEEPER

// for preferences
#define RW_MODE false
#define RO_MODE true

const uint8_t vmons = 4;  // how many vmons am I servicing 

struct vmon_str
{
  float     volt ;          // battery voltage.           filled by a polled read
  int16_t   battemp;        // battery temperature        filled by a polled read
  int16_t   restemp;        // resistor temperature       filled in by a polled read
  uint32_t  mostrecent;     // timestamp of most recent volt message from device
  uint32_t  received;       // number of messages I've received from this board   
  uint32_t  sent;           // number of messages I've sent to this board
  uint8_t   balance;        // should it be balancing     filled in by current battery voltage comparisons then
                            //                           . polled send to vmons.
  uint8_t   act_balance;    // is it balancing?            filled in by a polled read
  uint16_t  protocol_err;   // lets keep the struct aligned
  float     gain ;          // adc gain
  float     offset ;        // adc offset
  uint8_t   mac[6];         // vmon mac address

} vmon[vmons];   // state structure for all the vmons    

struct imon_str
{
  float     current ;       // battery current.           filled by an unpolled write from imon
  uint32_t  mostrecent;     // timestamp of most recent message from device
  uint32_t  received;       // number of messages I've received from this board   
  uint16_t  protocol_err;   
  uint8_t   mac[6];         // vmon mac address

} imon;   // state structure for the imon    

const char cmdlen = 20;
const char cmdqlen = 1;

struct vmon_pollq
{
  char     cmd[cmdlen] ;    // command string
  uint8_t  cmd_avail;       // is a command available
  uint8_t  qclaimed;        // has someone claimed a queue spot
  uint8_t  board;           // which board is the cmd for. for msg counting. 0..3
  uint8_t  spare;           // align it to 32b
} vq[cmdqlen];              // instantiate a q



// #define ADC_FITTED
// ADC is not required locally since the design adc doesn't support negative inputs.
// an External IMON board is now used. I could redesign the mco board with a instrumentation
// amplifier likely on split rails.
// (ie add a local -5V rail for the buffer just like I did on the haflinger battery fan controller BFC. )
// Maybe if I do a respin of mco. I'm clearly getting old and forgetful :(

#ifdef ADC_FITTED
const uint8_t adc_addr = 0x48;
ADS1115_WE adc = ADS1115_WE(adc_addr); // I2C connected ads1115 16b adcb  designed initially for battery current.
#endif

// serial is used for debug and for programming NVM
const uint8_t longest_record = 24;  //   worst is display comand WD1=12345678901234567890
const uint8_t record_size = longest_record + 2;    //  char char = nnnnn \n
char serial_buf[record_size];
uint8_t serial_buf_pointer;
int32_t soc;
int32_t soc_pps_freq=1000;
uint8_t soc_pc; // 0..100


// serial2 is used for communicating with vmons
//const uint8_t longest_record2 = 24;  //   worst is display comand WD1=12345678901234567890
//const uint8_t record_size2 = longest_record2 + 2;    //  char char = nnnnn \n
//char serial2_buf[record_size2];
//uint8_t serial2_buf_pointer;


uint8_t baseMac[6];         // my own mac address
const  uint16_t msgbuflen= 128;  // for wifi transfers
const char * version = "MCO 11 Apr 2025 Rev1";

Preferences mcoPrefs;  // NVM structure
// these will be initialized from the NV memory

uint8_t psu_mac[6];
uint8_t mcc_mac[6];
uint8_t cmt_mac[6];

float    initial_charge_current;
float    initial_charge_voltage;
float    topoff_charge_current;
float    topoff_charge_voltage;
float    transition_current;
float    cutoff_current;
float    max_battery_voltage;
float    min_battery_voltage; 
float    batt_balance_voltage;
float    batt_balance_tol_voltage;
int32_t  battery_capacity;  //mA Seconds
float    fbattery_capacity;  //mA Seconds
int32_t  beep_SOC;
float    adc_gain;
float    adc_offset;
float    psu_gain;
float    psu_offset;
uint32_t old_message_time;

struct charger_str
{
  float voltage;
  float current;
  float set_v;
  float set_i;
  uint32_t mostrecent;
  uint32_t sent;
  uint32_t received;
  bool  enable;
} charger ;


typedef struct struct_message {
  char message[msgbuflen];
} struct_message;

// create the wifi message struct
struct_message incoming_data;
struct_message outgoing_data;

esp_now_peer_info_t peerInfo;



/* IOs  definitions */
const uint8_t buz_en_pin    = 23;    // buzzer
const uint8_t alrt_pin      = 34;      // alert output from adc
const uint8_t charger_connected_pin = 35;  // inhibit beep on charger
const uint8_t fram_wp_pin   = 25;
const uint8_t soc_pps_pin   = 27;
const uint8_t sda_pin       = 21; 
const uint8_t scl_pin       = 22; 
const uint8_t txd2_pin      = 17;
const uint8_t rxd2_pin      = 16;

// I2C addresses 
const uint8_t fram_addr = 0x50;

// Going to do all my basic event timing and logging in seconds, so will use ESP32Time for convenience.
// not going to bother setting the clock, just demark time in HMS from whenever the board last got
// turned on.
ESP32Time rtc;
// uint16_t of seconds is 64k seconds is 18 hours. not quite enough for this.  
uint32_t       last_display_update_time;         // time last display update happened , in seconds
uint16_t display_update_period = 1; // seconds.

uint32_t last_current_update_time;
uint16_t current_update_period = 1; // seconds.

uint32_t       last_charger_state_update_time;     // time last charger state update happened , in seconds
uint16_t       charger_state_update_period = 2; // seconds.

enum CState{Mowing, Charger_not_init, Charger_init_CC, CC, Charger_init_CV, CV, Done} State; 


// FRAM fram;
Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();

// beeper
// setting PWM properties
const int freq_beep = 2000;
const int beep_res   = 7;
const int beep_50_pwm = 128;
const int freq_soc_50 = 168;
const int resolution = 8;
bool last_beep_state=0;


// vmon polling going to update vmon and psu more rapidly.
// psu polls take a few 10s of ms, vmon polls 5ms.
// but  thats not a delay, as the received data structure is handled asynchronously in a callback

// vmon polling structures

uint8_t vmon_ii =0;
const uint8_t vmon_ii_max =24;
uint32_t last_vmon_time=0;
//const uint32_t vmon_period_millis = 1000 / vmon_ii_max ; // roughly 40 ms 
uint32_t vmon_period_millis = 2000 / vmon_ii_max ; // roughly 1s for testing

bool suspend_vmon_polling = false;
bool vmon_struct_loaded = false;

struct vpoll_str
{
  uint8_t addr;
  uint8_t letter;
} vpoll[vmon_ii_max];


uint32_t last_beep_time ;
uint32_t beep_on_period = 300;


#ifdef MEAS_PERF
// vmon polling performance work
struct poll_perf
{
  uint32_t issue;
  uint32_t resp;
} vmon_polltime[vmons];
#endif

// psu polling
bool suspend_psu_polling = false;
uint8_t psu_ii =0;
uint32_t last_psu_time=0;
const uint8_t psu_ii_max = 5;
uint32_t psu_period_millis = 2000 / psu_ii_max ; // roughly 300 ms 
uint8_t psu_addr[] ={ 30, 31, 12, 10, 11 };

// These values are used to check the charger actually received new
// settings prior to a state change. I'm not using a reliable transport service.
// Checks only mean something with the charger output disabled.
// They are for checking settings, not what the load is causing. 
const float charger_voltage_tol = 0.2;
const float charger_current_tol = 0.05;



const uint8_t default_mac[]     = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
const uint8_t default_mcc_mac[] = { 0x5c, 0x01, 0x3b, 0x6c, 0xf4, 0xfc };
const uint8_t default_psu_mac[] = { 0x5c, 0x01, 0x3b, 0x6c, 0xe2, 0xd0 };
const uint8_t default_cmt_mac[] = { 0x5c, 0x01, 0x3b, 0x6c, 0x99, 0x38 };
const uint8_t default_v1_mac[]  = { 0x5c, 0x01, 0x3b, 0x6c, 0x7d, 0x14 };
const uint8_t default_v2_mac[]  = { 0x5c, 0x01, 0x3b, 0x66, 0x0c, 0x9c };
const uint8_t default_v3_mac[]  = { 0x24, 0x0a, 0xc4, 0xee, 0x03, 0x60 };
const uint8_t default_v4_mac[]  = { 0x5c, 0x01, 0x3b, 0x6c, 0xea, 0x48 };
const uint8_t default_imon_mac[]  = { 0x9c, 0x9c, 0x1f, 0xc6, 0xf7, 0xac };

