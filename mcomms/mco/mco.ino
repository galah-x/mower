//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'mco  Time-stamp: "2025-03-31 21:12:22 john"';

// this is the app to run the mower comms controller for the Ryobi mower.
// use tools -> board ->  ESP32 Dev module 

/* mco
   can can talk to the power supply to adjust current and voltage settings.

   mco is responsible for talking to the 4 vmons.
   And, directly or indirectly, psu.
   It has to report all relevant loggable info to mcc.
   It has to get the info from vmons. It runs this.

   Locally, mco has access to the current into/out of the batteries via current shunt and adc
   It intergates this to get SOC.
   It zeros SOC on empty, adjusting the battery capacity measure when that happens.
   It 100%s SOC on full, adjusting the battery capacity measure when that happens.
   It beeps on empty when battery empty && not_charging
   it records SOC in local fram. multiple copies most likely.

   beeper
   then the core logic.
*/

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
#include <ADS1115_WE.h>   // adc converter
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



// #define NOADC

#ifndef NOADC
const uint8_t adc_addr = 0x48;

ADS1115_WE adc = ADS1115_WE(adc_addr); // I2C connected ads1115 16b adcb  Used for battery voltage.
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
const char * version = "MCO 31 Mar 2025 Rev1";

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
  uint32_t mostrecent;
  bool  enable;
} charger ;


typedef struct struct_message {
  char message[msgbuflen];
} struct_message;

// create the wifi message struct
struct_message incoming_data;
struct_message outgoing_data;

esp_now_peer_info_t peerInfo;

// wifi callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming_data, incomingData, sizeof(incoming_data));
  // Serial.print("received ");
  // Serial.print(len);
  // Serial.print(" bytes from ");  
  // Serial.printf("%02x%02x%02x%02x%02x%02x\n", mac[12], mac[13], mac[14], mac[15], mac[16], mac[17]) ;
  /* I suspect the mac' field here is the 24 byte mac header structure espressif uses
     fields 12 to 17 seem to be the source MAC. rest isn't obvious */

  // from psu?  Record polled readings. more differences at end of mac string
  if ((mac[17] == psu_mac[5]) && (mac[16] == psu_mac[4]) && (mac[15] == psu_mac[3]) &&
      (mac[14] == psu_mac[2]) && (mac[13] == psu_mac[1]) && (mac[12] == psu_mac[0]))
    parse_psu_wifi_buf(incoming_data.message);
  // from imon?  Record unpolled readings
  else if ((mac[12] == imon.mac[0]) && (mac[13] == imon.mac[1]) && (mac[14] == imon.mac[2]) &&
      (mac[15] == imon.mac[3]) && (mac[16] == imon.mac[4]) && (mac[17] == imon.mac[5]))
    parse_imon_wifi_buf(incoming_data.message);
  // from a vmon?  Record polled readings
  else {
    uint8_t i;
    for (i=0; i< vmons; i++)
      {
	if ((mac[17] == vmon[i].mac[5]) && (mac[16] == vmon[i].mac[4]) && (mac[15] == vmon[i].mac[3]) &&
	    (mac[14] == vmon[i].mac[2]) && (mac[13] == vmon[i].mac[1]) && (mac[12] == vmon[i].mac[0]))
	  parse_vmon_wifi_buf(incoming_data.message, i);
      }
  }
}

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
const uint16_t display_update_period = 1; // seconds.

uint32_t last_current_update_time;
const uint16_t current_update_period = 1; // seconds.

uint32_t       last_charger_state_update_time;     // time last charger state update happened , in seconds
const uint16_t charger_state_update_period = 2; // seconds.

enum CState{Mowing, Charger_not_init, CC, CV, Done} State; 


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
const uint32_t vmon_period_millis = 2000 / vmon_ii_max ; // roughly 1s for testing

bool suspend_vmon_polling = false;

struct vpoll_str
{
  uint8_t addr;
  uint8_t letter;
} vpoll[vmon_ii_max];


uint32_t last_beep_time ;
const uint32_t beep_on_period = 300;


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
const uint8_t psu_ii_max = 3;
const uint32_t psu_period_millis = 2000 / psu_ii_max ; // roughly 300 ms 
uint8_t psu_addr[] ={ 30, 31, 12 };

void setup (void) {
  Wire.begin();   //   Including Wire.begin seems to break FRAM ???
  Serial.begin(115200);
  Serial.println(version);
  
  // interestingly, while rx=16 txd=17 is supposedly the default pin allocation,  Serial2 doesn't
  // work without explicitly filling in the pin numbers here
  //Serial2.begin(9600, SERIAL_8N1, 16, 17);  

  //init vmon structure
  int i;
  int j;
  
  for (i=0;i<vmons;i++)
    {
      vmon[i].volt=0.0;
      vmon[i].battemp=0;
      vmon[i].restemp=0;
      vmon[i].mostrecent=0;
      vmon[i].received=0;
      vmon[i].sent=0;
      vmon[i].balance=0;
      vmon[i].protocol_err=0;
      vmon[i].act_balance=255;
      vmon[i].gain = -1;
      vmon[i].offset = -1;
    }

  // init imon struct
      imon.current=0.0;
      imon.mostrecent=0;
      imon.received=0;
      imon.protocol_err=0;
  
  
  //init vmon command q
  for (i=0;i<cmdqlen;i++)
    {
      // vq[i].cmd=(char) 0;
      vq[i].cmd_avail=0;
      vq[i].qclaimed=0;
    }

  // init the vmon polling management struct
  for (i=0; i<vmon_ii_max; i++)    // set which vmon to poll     ie the vmon poll address
	vpoll[i].addr = i & 0x03;  // poll vmons in order 0,1,2,3,0,1,2,3 etc
  for (i=0;i<vmons;i++)              
    {                                 // set what to poll for each of the 4 vmons
      vpoll[i].letter         = 'v';  // first do a read voltage  for each vmon
      vpoll[i+  vmons].letter = 's';  // then do a read battery temp for each vmon
      vpoll[i+2*vmons].letter = 'r';  // then do a read resistor temp for each vmon
      vpoll[i+3*vmons].letter = 'v';  // then do another read voltage poll for each vmon
      vpoll[i+4*vmons].letter = 'B';  // then do a write balance  for each vmon
      vpoll[i+5*vmons].letter = 'b';  // then do a read actual balance state poll for each vmon, monitor for temp effects
    }

  // init charger structure
  charger.voltage = 0.0;
  charger.current = 0.0;
  charger.enable = 0;
  charger.mostrecent = 0;
  State = Charger_not_init;
    
  
  Serial.println("init NVM");
  // initialize NVM  
   mcoPrefs.begin("mcoPrefs", RO_MODE);     // Open our namespace (or create it if it doesn't exist)
   bool tpInit = mcoPrefs.isKey("nvsInit"); // Test for the existence of the "already initialized" key.
   mcoPrefs.end();                          // close the namespace in RO mode
   if (tpInit == false) 
     reinit_NVM();                          // reinitialize the nvm structure if magik key was missing 
   
   // load local variables from NVM
   load_operational_params();                // load all variables from NVM

   // init IO
   pinMode(fram_wp_pin, OUTPUT);
   pinMode(buz_en_pin, OUTPUT);
   pinMode(alrt_pin, INPUT);
   pinMode(charger_connected_pin, INPUT);
   pinMode(soc_pps_pin, OUTPUT);

   digitalWrite(fram_wp_pin,  HIGH);
 
   // buzzer allocate
#ifdef DC_BEEPER
   digitalWrite(buz_en_pin, 0);
#else
   ledcAttach(buz_en_pin, freq_beep, beep_res);
   ledcWriteTone(buz_en_pin, 0);
#endif
   // soc allocate
   ledcAttach(soc_pps_pin, freq_soc_50, resolution);

   //   ledcWrite(buz_en_pin, 0);     // beeper off
   ledcWrite(soc_pps_pin, 128);  // set soc gauge to 50%
   

   // init some general variables and IOs 
   serial_buf_pointer = 0;
   last_current_update_time = rtc.getEpoch();
   last_display_update_time = rtc.getEpoch();
   last_charger_state_update_time = rtc.getEpoch();

   // at startup, confirm all 4 vmons are present.
   // play a beep code if not so.
   Serial.println("init FRAM");
   //   if (!fram.begin(0x50, fram_wp_pin))
   if (!fram.begin(0x50))
     {
       Serial.println("FRAM not connected!");
     }



   
   // check soc. stored in fram 3 times. choose the majority. 
   if (get_SOC(0) == get_SOC(1))
     {
     if (get_SOC(2) != get_SOC(0))
       write_fram_SOC(2, get_SOC(0));    // is 2 is the odd one out, rewrite it.
     }
   else if (get_SOC(0) == get_SOC(2))
     {
       if (get_SOC(1) != get_SOC(0))
	 write_fram_SOC(1, get_SOC(0));    // is 1 is the odd one out, rewrite it.
     }
   else if (get_SOC(1) == get_SOC(2))
     {
       if (get_SOC(0) != get_SOC(1))
	 write_fram_SOC(0, get_SOC(1));    // is 1 is the odd one out, rewrite it.
     } 
   else
     { 
       write_SOC(0, 3600000 * 50);    // initilize to 50% as I have no idea
     }       
   soc = get_SOC(0);

  Serial.println("init ADS1115");
   // init adc
   if(!adc.init()){
     Serial.println("ADS1115 not connected!");
   }
   adc.setVoltageRange_mV(ADS1115_RANGE_4096);  // 4.096V input range.
   // input attenuator about 0.2  0.2 * 15V max = 3V
   adc.setCompareChannels(ADS1115_COMP_0_1);    // measure between 0 and 1.
   adc.setConvRate(ADS1115_8_SPS);              // 8 samples per sec, slowest.
   adc.setAlertPinMode(ADS1115_DISABLE_ALERT);  // not using voltage range checks
   adc.setMeasureMode(ADS1115_CONTINUOUS);          // initiate measurements

   // init esp_now wifi 
   WiFi.mode(WIFI_STA);

   esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
   Serial.printf("ESP32 MAC Address: %02x%02x%02x%02x%02x%02x\n",
		 baseMac[0],baseMac[1],baseMac[2],baseMac[3],baseMac[4],baseMac[5]);

   Serial.println("init espnow");
   if (esp_now_init() != ESP_OK) {
     Serial.println("Error initializing ESP-NOW");
     return;
   }
   // Once ESPNow is successfully Init, we will register for Send CB to
   // get the status of Trasnmitted packet
   esp_now_register_send_cb(OnDataSent);
    
   // Once ESPNow is successfully Init, we will register for recv CB to
   // get recv packer info
   esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

    Serial.println("register macs");
    // Register peer mcc
    memcpy(peerInfo.peer_addr, mcc_mac, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
    }
    // Register peer psu
    memcpy(peerInfo.peer_addr, psu_mac, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
    }
    // Register peer cmt   // hmmm, with this registered i could not talk to vmon4. 
    //memcpy(peerInfo.peer_addr, cmt_mac, 6);
    //if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //  Serial.println("Failed to add peer");
    //}
    for (i=0; i< vmons; i++)
      {
	// Register vmons
	memcpy(peerInfo.peer_addr, vmon[i].mac, 6);
	Serial.printf("registering vmon%d mac = %02x%02x%02x%02x%02x%02x\n", i, vmon[i].mac[0], vmon[i].mac[1], vmon[i].mac[2], vmon[i].mac[3], vmon[i].mac[4], vmon[i].mac[5]); 
	if (esp_now_add_peer(&peerInfo) != ESP_OK){
	  Serial.println("Failed to add peer");
	}
      }
    Serial.println("done setup");
    
}

   

void loop (void)
{
  // service serial character if any available.
  do_serial_if_ready();

  if (millis() > last_beep_time + beep_on_period)
    {
      if (last_beep_state)
	{
#ifdef DC_BEEPER
	  digitalWrite(buz_en_pin, 0);
#else  
	  ledcWriteTone(buz_en_pin, 0); 
#endif
	  // ledcWrite(buz_en_pin, 0);     // beeper off
	  last_beep_state = false;
	}
      else
	{
	  // Serial.printf("ccpin=%d, notccpin=%d, soc<beept=%d\n",
	  // 		digitalRead(charger_connected_pin), !digitalRead(charger_connected_pin),
	  // 		(soc < beep_SOC));
	  if (!digitalRead(charger_connected_pin) && (soc < beep_SOC))
	    {
#ifdef DC_BEEPER
	      Serial.printf("setting beeper on\n");
	      digitalWrite(buz_en_pin, 1);
#else
	      ledcWriteTone(buz_en_pin, freq_beep);
#endif
	    }
	  last_beep_state = true;
	}
      last_beep_time = millis();
    }


  
  // update current and SOC state if its due 
  if (rtc.getEpoch() > (last_current_update_time + current_update_period))
    {
      while(adc.isBusy()) {
	Serial.println("adc is busy?");
	delay(1000);
      }
      soc = get_SOC(0) + (int32_t) (imon.current * 2000.0 * current_update_period);
      if (soc < 0)
	soc = 0;
      if (soc > battery_capacity)
	soc = battery_capacity;
      write_SOC(0, soc);
      int soc_f =     336.0 * (float)soc /  fbattery_capacity;
      if (soc_f != soc_pps_freq)
	{ // only update soc freq generator when freq changes. meter was glitchy prior
	  ledcChangeFrequency(soc_pps_pin, soc_f, 8);
	  soc_pps_freq = soc_f;
	}
      last_current_update_time = rtc.getEpoch();
    }
  
  // perform vmon polling according to the management structure. update all about once per 2 seconds
  if (millis()  > (last_vmon_time + vmon_period_millis))
    {
#ifdef MEAS_PERF
      // record the issue time of voltage polls
      if (vmon.[vmon_ii].letter = 'v') 
	{
	  vmon_polltime[vpoll[vmon_ii].addr].issue = millis();
	}
#endif
      if (vq[0].cmd_avail)
	{
	  //  if there is a msg in the vmon queue, send it in the curent ringbuffer timeslot.
	  // pause the real poll timeslot, dont skip the timeslot.
	  // note as curently coded, its single entry queue with no shifting.
	  // anyone wanting to write to the queue has to check the q is empty first.
	  send_q_msg();
	}
      else   // only increment vmon_ii command ring buffer if I did a poll. not on queue interruption.
	{
	  if (suspend_vmon_polling == false)
	    {
	      if (vpoll[vmon_ii].letter == 'B') // the desired poll is a Balance write, all others are reads 
		sprintf(outgoing_data.message, "Wb=%1d\n",  vmon[vpoll[vmon_ii].addr].balance );
	      else 
		sprintf(outgoing_data.message, "R%c\n",  vpoll[vmon_ii].letter);
	      esp_now_send(vmon[vpoll[vmon_ii].addr].mac, (uint8_t *) &outgoing_data, strlen(outgoing_data.message)+1);
	      
	      // record that a message was sent on that channel
	      vmon[vpoll[vmon_ii].addr].sent++;
	      
	      // increment vmon_ii
	      vmon_ii++;
	      if (vmon_ii >= vmon_ii_max)
		vmon_ii = 0;
	    }
	}
      last_vmon_time = millis();
    }
  
  // poll psu
  // update voltage/current/enable from psu if its due . update all about once per second
  if (millis()  > (last_psu_time + psu_period_millis))
    {
      if (! suspend_psu_polling)
	{
	  sprintf(outgoing_data.message, ":01r%02d=0,\n", psu_addr[psu_ii]);
	  psu_ii++;
	  if (psu_ii >= psu_ii_max)
	    psu_ii = 0;
	  esp_now_send(psu_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
	}
      last_psu_time = millis();
    }
  
  //   update diaplay on mcc
  if (rtc.getEpoch() > (last_display_update_time + display_update_period))
    {
      uint8_t i;
      for (i=0; i< vmons; i++)    // battery voltages
	{ 
	  if ((rtc.getEpoch() - vmon[i].mostrecent) <= old_message_time)  
	    sprintf(outgoing_data.message, "WD%1d=%1d %2.3fV B%1d", i, i+1, vmon[i].volt, vmon[i].act_balance);
	  else
	    sprintf(outgoing_data.message, "WD%1d=%1d ??????????", i, i+1);
	  esp_now_send(mcc_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
	}
      // d4 charger voltage
	if ((rtc.getEpoch() - charger.mostrecent) <= old_message_time)  
	  sprintf(outgoing_data.message, "WD4=%2.3fV ", charger.voltage);
	else
	  sprintf(outgoing_data.message, "WD4=??????V ");
      esp_now_send(mcc_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
      
      //d5, charger current
      if ((rtc.getEpoch() - charger.mostrecent) <= old_message_time)  
	sprintf(outgoing_data.message, "WD5=%2.3fA ", charger.current);
      else 
	sprintf(outgoing_data.message, "WD5=??????A ", charger.current);
      esp_now_send(mcc_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
      
      //d6 SOC
      if (battery_capacity > 0) 
	{
	  sprintf(outgoing_data.message, "WD6=SOC=%2d%%", (int16_t) ((100.0 * soc)/battery_capacity));
	  esp_now_send(mcc_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
	}
      
      //D7 battery temp Vmon1
      if ((rtc.getEpoch() - vmon[0].mostrecent) <= old_message_time)  
	sprintf(outgoing_data.message, "WD7=BT=%2dC", vmon[0].battemp);
      else 
	sprintf(outgoing_data.message, "WD7=BT=??dC", vmon[0].battemp);
      esp_now_send(mcc_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
      last_display_update_time=rtc.getEpoch();
    }

  // update charger state if its due
  if (millis()  > (last_charger_state_update_time + charger_state_update_period))
    {
      if ( (State == Mowing) && (digitalRead(charger_connected_pin)==1))
	// hmmm, what about if I was mowing and now Im not.
	State = Charger_not_init;
      
      if (digitalRead(charger_connected_pin)==0)
	{
	  State = Mowing;
	  if ((vmon[0].volt < min_battery_voltage) ||(vmon[1].volt < min_battery_voltage) ||
	      (vmon[2].volt < min_battery_voltage) ||(vmon[3].volt < min_battery_voltage))
	    { // a battery voltage is too low, set soc to 0% so beeping should start
	      soc  =  0;
	      write_SOC(0, soc );
	    }
	}
      // items below here mean the charger is plugged in. It may not necessarily be either turned on on nor enabled.
      else if (State == Charger_not_init)
	{ // as I don't know if the charger is actually on and listening, repeat this till  
	  set_psu_v(initial_charge_voltage);
	  set_psu_i(initial_charge_current);
	  set_psu_e(1);
	  if (imon.current > (1.1*transition_current)) // charger is now actually charging...)
	    {
	      State = CC;
	    }
	}
      else if ((State == CC) && (charger.current < transition_current))
	{
	  set_psu_v(topoff_charge_voltage);
	  set_psu_i(topoff_charge_current);
	  State = CV;
	}
      else if ((State == CV) && (charger.current < cutoff_current))
	{
	  set_psu_e(0);
	  State = Done;
	}
      else if (State == CV)
	{ // manage balance settings.
	  // choose the smallest of the 4 battery voltages
	  // any battery less than  batt_balance_voltage should not be balancing
	  // any battery less than  smallest + batt_balance_tol_voltage should not be balancing
	  // any battery > smallest + batt_balance_tol_voltage should be balancing

	  // choose the smallest of the 4 battery voltages
	  float lowest_battery_voltage = vmon[0].volt;
	  int i;
	  for (i=1; i<vmons; i++)
	    if (vmon[i].volt < lowest_battery_voltage)
	      lowest_battery_voltage = vmon[i].volt;

	  // decide the voltage threshold above which balance should happen 
	  float min_balance_battery_voltage = lowest_battery_voltage + batt_balance_tol_voltage;
	  if  (min_balance_battery_voltage < batt_balance_voltage)
	    min_balance_battery_voltage = batt_balance_voltage;

	  // set balance condition for each battery according to its relationship to the balance threshold 
	  for (i=0; i<vmons; i++)
	    if (vmon[i].volt < min_balance_battery_voltage)
	      vmon[i].balance = 0;
	    else 
	      vmon[i].balance = 1;
	}

      // check this whenever actually charging.
      if ((State != Mowing) && ( (vmon[0].volt > max_battery_voltage) ||(vmon[1].volt > max_battery_voltage) ||
				    (vmon[2].volt > max_battery_voltage) ||(vmon[3].volt > max_battery_voltage)))
	{ // an individual battery voltage is too high
	  set_psu_e(0);              // turn off charger
	  State = Done;
	  soc  =  battery_capacity;  // set soc to max
	  write_SOC(0, soc );
	}
      
      last_charger_state_update_time = rtc.getEpoch();
    }

  // 
  // and maybe later... adjust stored battery capacity on SOC unusual transitions.

}

/***** OK here are the subroutines   *************/

void send_q_msg ( void)
{ 	  
  memcpy(outgoing_data.message, vq[0].cmd,  cmdlen);
  esp_now_send(vmon[vq[0].board].mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
  vmon[vq[0].board].sent++;     // note another message sent to this vmon 
  vq[0].cmd_avail = 0;            // mark cmd queue consumed. 
}

void wait_next_q_slot (uint8_t slots)
{
  uint8_t i;
  for (i=0; i<slots; i++)
    {
      while  (millis()  <= (last_vmon_time + vmon_period_millis))
	{
	  delay(5);
	}
      last_vmon_time = millis();
    }
}

const uint8_t default_mac[]     = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
const uint8_t default_mcc_mac[] = { 0x5c, 0x01, 0x3b, 0x6c, 0xf4, 0xfc };
const uint8_t default_psu_mac[] = { 0x5c, 0x01, 0x3b, 0x6c, 0xe2, 0xd0 };
const uint8_t default_cmt_mac[] = { 0x5c, 0x01, 0x3b, 0x6c, 0x99, 0x38 };
const uint8_t default_v1_mac[]  = { 0x5c, 0x01, 0x3b, 0x6c, 0x7d, 0x14 };
const uint8_t default_v2_mac[]  = { 0x5c, 0x01, 0x3b, 0x66, 0x0c, 0x9c };
const uint8_t default_v3_mac[]  = { 0x24, 0x0a, 0xc4, 0xee, 0x03, 0x60 };
const uint8_t default_v4_mac[]  = { 0x5c, 0x01, 0x3b, 0x6c, 0xea, 0x48 };
const uint8_t default_imon_mac[]  = { 0x9c, 0x9c, 0x1f, 0xc6, 0xf7, 0xac };

void reinit_NVM (void)
{
  Serial.println("Initializing NVM");
  
  // If tpInit is 'false', the key "nvsInit" does not yet exist therefore this
  //  must be our first-time run. We need to set up our Preferences namespace keys. So...
  mcoPrefs.begin("mcoPrefs", RW_MODE);       //  open it in RW mode.
  
  // The .begin() method created the "mcoPrefs" namespace and since this is our
  //  first-time run we will create
  //  our keys and store the initial "factory default" values.

  mcoPrefs.putBytes("cmtmac", default_cmt_mac, 6);  // mac address of tester
  mcoPrefs.putBytes("psumac", default_psu_mac, 6);  // mac address of power supply 
  mcoPrefs.putBytes("mccmac", default_mcc_mac, 6);  // mac address of power supply controller 
  mcoPrefs.putBytes("v1mac",  default_v1_mac, 6);  // mac address of vmon # 1
  mcoPrefs.putBytes("v2mac",  default_v2_mac, 6);  // mac address of vmon # 2
  mcoPrefs.putBytes("v3mac",  default_v3_mac, 6);  // mac address of vmon # 3
  mcoPrefs.putBytes("v4mac",  default_v4_mac, 6);  // mac address of vmon # 4
  mcoPrefs.putBytes("imonmac",default_imon_mac, 6);  // mac address of imon
  mcoPrefs.putFloat("adco", 0.0);            // current adc offset term
  mcoPrefs.putFloat("adcg", 1.0);            // current adc gain term
  mcoPrefs.putFloat("psug",  1.0 );          // cal term from local voltage to psu 
  mcoPrefs.putFloat("psuo",  0.0 );          // cal term from local voltage to psu 
  mcoPrefs.putFloat("icc", 10.0);            // initial charge current
  mcoPrefs.putFloat("icv", 55.2);            // initial charge voltage 13.8*4 = 55.2
  mcoPrefs.putFloat("tcc",  3.0);            // topoff charge current
  mcoPrefs.putFloat("tcv", 56.0);            // topoff charge voltage 14.0*4 = 56
  mcoPrefs.putFloat("tc",  3.0);             // transition current
  mcoPrefs.putFloat("fc",  0.3);             // final current
  mcoPrefs.putFloat("maxbv", 14.3);          // max single battery voltage
  mcoPrefs.putFloat("minbv", 11.5);          // min single battery voltage
  mcoPrefs.putFloat("bbv",  13.8);           // start balancing battery above this voltage 3.45 * 4 =13.8 
  mcoPrefs.putFloat("bbt",  0.005);          // battery balance tolerance
  mcoPrefs.putLong("bsoc",  36000000);       // beep SOC, 10%
  mcoPrefs.putLong("bcap", 360000000);       // battery capacity in mAS  100AH = 360e6 maS 
  mcoPrefs.putULong("omt", 10);              // what defines an OLD message. in seconds.
  mcoPrefs.putBool("nvsInit", true);            // Create the "already initialized"
  //  key and store a value.
  // The "factory defaults" are created and stored so...
  mcoPrefs.end();                               // Close the namespace in RW mode and...
}


// regarding offsets..
// read the adc, add adv_offset, multiply the sum by the gain to get the real reading.
//   same for a voltage reading from the psu.
//   the inverse for setting the psu voltage.
void load_operational_params(void)
{

  mcoPrefs.begin("mcoPrefs", RO_MODE);       // Open our namespace (or create it
                                               //  if it doesn't exist) in RO mode.
   // Retrieve the operational parameters from the namespace
   //  and save them into their run-time variables.

   mcoPrefs.getBytes("psumac", psu_mac, 6);             // load power supply mac
   mcoPrefs.getBytes("mccmac", mcc_mac, 6);             // load mco/cmt supply mac
   mcoPrefs.getBytes("cmtmac", cmt_mac, 6);             // load mco/cmt supply mac
   mcoPrefs.getBytes("v1mac", vmon[0].mac, 6);          // load vmon 1 mac
   mcoPrefs.getBytes("v2mac", vmon[1].mac, 6);          // load vmon  mac
   mcoPrefs.getBytes("v3mac", vmon[2].mac, 6);          // load vmon  mac
   mcoPrefs.getBytes("v4mac", vmon[3].mac, 6);          // load vmon  mac
   mcoPrefs.getBytes("imonmac", imon.mac, 6);           // load imon  mac
   adc_offset = mcoPrefs.getFloat("adco");              // current adc offset term
   adc_gain = mcoPrefs.getFloat("adcg");                // current adc gain term
   psu_offset = mcoPrefs.getFloat("psuo");              // cal term from local voltage to psu 
   psu_gain = mcoPrefs.getFloat("psug");                // cal term from local voltage to psu 
   initial_charge_current = mcoPrefs.getFloat("icc");   // initial charge current
   initial_charge_voltage = mcoPrefs.getFloat("icv");   // initial charge voltage 13.8*4 = 55.2
   topoff_charge_current = mcoPrefs.getFloat("tcc");    // topoff charge current
   topoff_charge_voltage = mcoPrefs.getFloat("tcv");    // topoff charge voltage 14.0*4 = 56
   transition_current = mcoPrefs.getFloat("tc");        // transition current
   cutoff_current = mcoPrefs.getFloat("fc");            // final current
   max_battery_voltage = mcoPrefs.getFloat("maxbv");    // max single battery voltage
   min_battery_voltage = mcoPrefs.getFloat("minbv");    // min single battery voltage
   batt_balance_voltage = mcoPrefs.getFloat("bbv");     // start balancing a battery above this 3.45 * 4 =13.8
   batt_balance_tol_voltage = mcoPrefs.getFloat("bbt"); // battery balance tolerance
   beep_SOC = mcoPrefs.getLong("bsoc");                 // beep SOC, 10%
   battery_capacity = mcoPrefs.getLong("bcap");         // battery capacity in mAS  100AH = 360e6 maS 
   fbattery_capacity = (float) battery_capacity;
   old_message_time  = mcoPrefs.getULong("omt");        // what defines an OLD message. in seconds.
   // All done. Last run state (or the factory default) is now restored.
   mcoPrefs.end();                                      // Close our preferences namespace.
}

// this is for S0, debug
void parse_buf (char * in_buf)
{
  // Z eraZe NV memory
  // Rf Read and print Field,
  //    where Field could be    lower case for current stuff, upper case for NVM
  //          i battery current 
  //          v[1234] vmon state
  //          t current time
  //          c scaled charger voltage, current, enable
  //          s SOC
  //          st state
  //          M Show macs
  //          V=version
  //          I=initial charge current, voltage
  //          B=balance above per battery voltage, tolerance, maxV
  //          E BEep below this 
  //          T=topoff voltage, current, final current
  //          G[VI] adc gain   current=local adc voltage=DPM8624
  //          P[VI] psu offset voltage =local adc voltage=DPM8624
  //          S     SOC total capacity in mA.S i guess
  // Wf Field,
  //    where Field could be (volatile stuff in lower case)
  //          Mx mac    WMP=<12 ascii hex digits>  mac for Psu or Mcc or Cmt
  //          bn=[01] switch balance on board 1..4 to given level. this is sent by a poll, and updated by loop() 
  //          cv=f    charger voltage   set psu V
  //          ci=f    charger current   set psu I
  //          ce=b   charger enable    set pse e
  //          p=b    polling  enable 1 or disable 0  vmon polling.
  //          s=n     set SOC  0..100
  //          d=display WD0=string   write display line 0..7 with given string
  //
  //          II=f   initial charge current
  //          IV=f   initial charge voltage
  //          BA=f   balance above per battery voltage
  //          BT=f   balance tolerance voltage
  //          BM=f   max single battery voltage
  //          E=d  BEep below this 
  //          TV=f   topoff voltage
  //          TI=f   topoff current
  //          TE=f   topoff final current to stop charging
  //          G[VI]=f adc gain   current=local adc voltage=DPM8624
  //          O[VI]=f adc offset current=local adc voltage=DPM8624
  //          S=d
  //          V[1234]E=%4d set cal write enable for specified vmon.  vmon we resets at start or on nth voltage read.
  //          V[1234]G=f write adc Gain for specified vmon. It must be write enabled
  //          V[1234]O=f write adc offset for specified vmon. It must be write enabled
  //                        
  int8_t  address;
  
  uint8_t  cmd;
  uint8_t  field;
  uint8_t  field2;
  uint8_t  field3;
  int      value;
  uint8_t  mvalue[6]; // mac address
  float    fvalue;
  const uint8_t out_buf_len = 20;
  char     out_buf[out_buf_len];
  int      match ;
  int      i;
  int32_t soc;
  uint32_t time1;
  uint32_t time2;
  
  cmd = in_buf[0];
  
  switch (cmd)
    {
    case 'Z':
      reinit_NVM();
      load_operational_params();
      break;
      
    case 'R':
      switch (in_buf[1])
	{
	case 'i':
	  Serial.printf("Imon current=%1.2fA updated %1d seconds ago msgs r=%1d errs=%1d\n",
			imon.current,          rtc.getEpoch() - imon.mostrecent,
			imon.received,         imon.protocol_err); 
	
	  break;
      
	case 'v':
	  address = ((in_buf[2] -1) & 0x03)  ;   // convert ascii 1..4 ie 0x31 to 0x34 to 0..3  31->2 34->1  
	  if ((in_buf[3] == 'G') || (in_buf[3] == 'O'))
	    {
	      // read gain or offset. this does not come from the polled read values, must add a read cmd.
	      // I'll add a read to the write queue, and ensure the callback parser knows about them
	      snprintf(out_buf, out_buf_len, "R%c\n", in_buf[3]);    // generate q cmd string for a write balance  
	      write_vmon_wq(address, out_buf, strlen(out_buf)+1);
	      // wait till next q slot. or next 4 if the target may be napping
	      wait_next_q_slot(4);
	  
	      // sendq
	      send_q_msg();
	  
	      // wait till next q slot
	      wait_next_q_slot(4);
	  
	      // read result.
	      if (in_buf[3] == 'G')
		Serial.printf("vmon %d G=%f\n", address+1, vmon[address].gain);
	      else
		Serial.printf("vmon %d O=%f\n", address+1, vmon[address].offset);
	      // NB, as loop is single threaded and loop runs the poll q, must read twice!
	    }
	  else
	    {
	      Serial.printf("Vmon %d voltage=%1.3f bt=%1dC rt=%1dC b=%d be=%d updated %1d seconds ago msgs s=%1d r=%1d errs=%1d\n",
			    address+1, vmon[address].volt,
			    vmon[address].battemp, vmon[address].restemp,
			    vmon[address].balance, vmon[address].act_balance, 
			    rtc.getEpoch() - vmon[address].mostrecent,
			    vmon[address].sent, vmon[address].received, vmon[address].protocol_err); 
	    }
	  break;
	case 't' :
	  Serial.printf("current time %d %s\n", rtc.getEpoch(), rtc.getTime()); 
	  break;
	case 'c' :
	  Serial.printf("charger voltage=%1.2f current=%1.3f enable=%d updated  %d seconds ago \n",
			charger.voltage, charger.current, charger.enable, rtc.getEpoch() - charger.mostrecent); 
	  break;
	case 's' : // FIXME enum CState{} State; 

	  if (inbuf[2] == 't')
	    Serial.printf("State=%d   (Mowing, Charger_not_init, CC, CV, Done) \n", State);
	  else 
	    Serial.printf("SOC=%dmAS %2.1f%%\n", get_SOC(0), 100.0 * (float) get_SOC(0) / (float)battery_capacity);
	  break;
	case 'M':
	  Serial.printf("MCO_MAC=%02x%02x%02x%02x%02x%02x PSU=%02x%02x%02x%02x%02x%02x MCC=%02x%02x%02x%02x%02x%02x CMT=%02x%02x%02x%02x%02x%02x\n",
			baseMac[0],baseMac[1],baseMac[2],baseMac[3],baseMac[4],baseMac[5],
			psu_mac[0],psu_mac[1],psu_mac[2],psu_mac[3],psu_mac[4],psu_mac[5],
			mcc_mac[0],mcc_mac[1],mcc_mac[2],mcc_mac[3],mcc_mac[4],mcc_mac[5],
			cmt_mac[0],cmt_mac[1],cmt_mac[2],cmt_mac[3],cmt_mac[4],cmt_mac[5]);
	  Serial.printf("VMON1_MAC=%02x%02x%02x%02x%02x%02x V2=%02x%02x%02x%02x%02x%02x V3=%02x%02x%02x%02x%02x%02x V4=%02x%02x%02x%02x%02x%02x\n",
			vmon[0].mac[0], vmon[0].mac[1], vmon[0].mac[2], vmon[0].mac[3], vmon[0].mac[4], vmon[0].mac[5], 
			vmon[1].mac[0], vmon[1].mac[1], vmon[1].mac[2], vmon[1].mac[3], vmon[1].mac[4], vmon[1].mac[5], 
			vmon[2].mac[0], vmon[2].mac[1], vmon[2].mac[2], vmon[2].mac[3], vmon[2].mac[4], vmon[2].mac[5], 
			vmon[3].mac[0], vmon[3].mac[1], vmon[3].mac[2], vmon[3].mac[3], vmon[3].mac[4], vmon[3].mac[5]);
	  Serial.printf("IMON_MAC=%02x%02x%02x%02x%02x%02x\n",
			imon.mac[0],imon.mac[1],imon.mac[2],imon.mac[3],imon.mac[4],imon.mac[5]);
      
      
	  break;
	case 'I' :
	  Serial.printf("Initial charge voltage=%2.2fV current=%2.3fA\n",
			initial_charge_voltage, initial_charge_current);
	  break;
	case 'T' :
	  Serial.printf("Topoff charge voltage=%2.2fV current=%2.3fA\n",
			topoff_charge_voltage, topoff_charge_current);
	  break;
	case 'G' :
	  Serial.printf("Current adc gain=%f adc_offset=%f\n",
			adc_gain, adc_offset);
	  break;
	case 'P' :
	  Serial.printf("Power supply voltage gain=%f offset=%f\n",
			psu_gain, psu_offset);
	  break;
	case 'B' :
	  Serial.printf("Balance battery above %2.3fV Tolerance=%1.3fV Max voltage=%2.3fV Min voltage=%2.3fV \n",
			batt_balance_voltage, batt_balance_tol_voltage, max_battery_voltage, min_battery_voltage);
	  break;
	case 'E' :
	  Serial.printf("Beep at or below SOC=%d mAS %d%%\n",
			beep_SOC, 100*beep_SOC/battery_capacity);
	  break;
	case 'S' :
	  Serial.printf("Battery capacity=%ld mAS %ldAH\n",
			battery_capacity, battery_capacity/3600000);
	  break;
	case 'V':
	  Serial.println(version);
	  break;
	}
      break;

    case 'W':
      switch (in_buf[1]) {
      case 'b' :
	match =  sscanf(in_buf, "%c%c%c=%d", &cmd, &field, &field2, &value);
	if (match == 4)
	  {
	    field2 = (field2 -1) & 0x03; // map 1..4 to 0..3
	    vmon[field2].balance = value;
	  }
	break;

      case 'd':
	memcpy(&incoming_data.message, in_buf, strlen(in_buf));
	esp_now_send(mcc_mac, (uint8_t *) &incoming_data, sizeof(incoming_data));
	break;
      
      case  'c' :
	match =  sscanf(in_buf, "%c%c%c=%f", &cmd, &field,&field2, &fvalue);
	switch (field2)
	  {
	  case 'v' :
	    set_psu_v(fvalue);
	    break;
	  case 'i' :
	    set_psu_i(fvalue);
	    break;
	  case 'e' :
	    match =  sscanf(in_buf, "%c%c%c=%d", &cmd, &field,&field2, &value);
	    set_psu_e(value);
	    break;
	  }
	break;
      
      case 'p' :
	if (in_buf[3] == '1')
	  {
#ifdef DEBUG
	    Serial.printf("enable vmon polling\n");
#endif
	    suspend_vmon_polling = false; // enable polling ie don't suspend polling
	  }
	else
	  {
#ifdef DEBUG
	    Serial.printf("disable vmon polling\n");
#endif
	    suspend_vmon_polling = true;
	  }
	break;
	
      case 's' :
	match =  sscanf(in_buf, "%c%c=%d", &cmd, &field, &value);
	soc  = (float) value * (float) battery_capacity / 100.0;
	write_SOC(0, soc );
	break;
      case  'I' :
	match =  sscanf(in_buf, "%c%c%c=%f", &cmd, &field,&field2, &fvalue);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'V') 
	  mcoPrefs.putFloat("icv", fvalue);
	else if (field2 == 'I') 
	  mcoPrefs.putFloat("icc", fvalue);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
      case 'M':  // WMP=MACADDRESS for the guy to respond to
	match =  sscanf(in_buf, "%c%c%c=%2x%2x%2x%2x%2x%2x", &cmd, &field,&field2,
			&mvalue[0],&mvalue[1],&mvalue[2],&mvalue[3],&mvalue[4],&mvalue[5]);
      
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'P') 
	  mcoPrefs.putBytes("psumac", mvalue, 6);
	else if (field2 == 'M') 
	  mcoPrefs.putBytes("mccmac", mvalue, 6);
	else if (field2 == 'C') 
	  mcoPrefs.putBytes("cmtmac", mvalue, 6);
	else if (field2 == 'I') 
	  mcoPrefs.putBytes("imonmac", mvalue, 6);
	match =  sscanf(in_buf, "%c%c%c%c=%2x%2x%2x%2x%2x%2x", &cmd, &field, &field2, &field3,
			&mvalue[0],&mvalue[1],&mvalue[2],&mvalue[3],&mvalue[4],&mvalue[5]);
	if (field2 == 'V')
	  {
	    if (field3 == '1')
	      mcoPrefs.putBytes("v1mac", mvalue, 6);
	    else if (field3 == '2') 
	      mcoPrefs.putBytes("v2mac", mvalue, 6);
	    else if (field3 == '3') 
	      mcoPrefs.putBytes("v3mac", mvalue, 6);
	    else if (field3 == '4') 
	      mcoPrefs.putBytes("v4mac", mvalue, 6);
	  }
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;

      case  'T' :
	match =  sscanf(in_buf, "%c%c%c=%f", &cmd, &field,&field2, &fvalue);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'V') 
	  mcoPrefs.putFloat("tcv", fvalue);
	else if (field2 == 'I') 
	  mcoPrefs.putFloat("tcc", fvalue);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
      case  'E' :
	match =  sscanf(in_buf, "%c%c=%d", &cmd, &field, &value);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	mcoPrefs.putFloat("bsoc", value);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
      case  'B' :
	match =  sscanf(in_buf, "%c%c%c=%f", &cmd, &field,&field2, &fvalue);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'A') 
	  mcoPrefs.putFloat("bbv", fvalue);
	else if (field2 == 'T') 
	  mcoPrefs.putFloat("bbt", fvalue);
	else if (field2 == 'X') 
	  mcoPrefs.putFloat("maxbv", fvalue);
	else if (field2 == 'N') 
	  mcoPrefs.putFloat("minbv", fvalue);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
      case  'G' :
	match =  sscanf(in_buf, "%c%c%c=%f", &cmd, &field,&field2, &fvalue);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'V') 
	  mcoPrefs.putFloat("psug", fvalue);
	else if (field2 == 'I') 
	  mcoPrefs.putFloat("adcg", fvalue);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;

      case  'O' :
	match =  sscanf(in_buf, "%c%c%c=%f", &cmd, &field,&field2, &fvalue);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'V') 
	  mcoPrefs.putFloat("psuo", fvalue);
	else if (field2 == 'I') 
	  mcoPrefs.putFloat("adco", fvalue);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;

      case  'S' :
	match =  sscanf(in_buf, "%c%c=%f", &cmd, &field, &fvalue);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	mcoPrefs.putFloat("bcap", fvalue);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
      
      case 'V' :
	// doing Write Vmon #vmon we || Write Vmon #vmon G=f || Write Vmon #vmon O=f  to write cal terms to vmons  
	match =  sscanf(in_buf, "WV%c%c=%f", &field2, &cmd, &fvalue);
	if ((match == 3) && ((cmd == 'G') || (cmd == 'O'))) 
	  { // either WV2O=1.234  or WV1G=5.678
#ifdef DEBUG
	    Serial.printf("cal gain/offset write on vmon %c %c=%f\n", field2, cmd, fvalue);
#endif
	    field2 = (field2 -1) & 0x03; // map 0x31..0x34 to 0..3
	    snprintf(out_buf, out_buf_len, "W%c=%f\n", cmd, fvalue);    // generate q cmd string for a write   
	    write_vmon_wq(field2, out_buf, strlen(out_buf)+1);
	  }
	else
	  {
	    match =  sscanf(in_buf, "WV%cE=%4d", &field2, &value);
	    if ((match == 2))
	      { //  WV3E=%4d
#ifdef DEBUG
		Serial.printf("enable cal write on vmon %c inhibit napping %d times.\n", field2, value);
#endif
		field2 = (field2 -1) & 0x03; // map 0x31..0x34 to 0..3
		snprintf(out_buf, out_buf_len, "WW=%04d\n", value);    // generate q cmd string for a cal write enable
		write_vmon_wq(field2, out_buf, strlen(out_buf)+1);
	      }	    
	  }
	break;
      }      
      break;
      // end of 'W'
    }    
}

// write a message to the vmon write queue.  As vmons are polled continuously, this queue mechanism interrupts
// polling, using the next poll slot to send this message to the specified vmon.  polling resumes without
// skipping a poll in the subsequent poll slot. So polling just gets a tad delayed. 
//
//  uint not ascii vmon address  0..3
void write_vmon_wq (      uint8_t vmon, char * message, uint8_t msglen)
{
  // check nobody else is writing a record
  while (vq[0].qclaimed == 1)
	delay(10);
  // check there is a queue slot available. its a single entry queue right now.
  while (vq[0].cmd_avail == 1)
	delay(10);
  // ok, queue position is available. write the command to the queue
  vq[0].qclaimed == 1;
  memcpy(vq[0].cmd,  message, msglen);
  vq[0].board = vmon;                            // write q board address
  vq[0].cmd_avail = 1;
  vq[0].qclaimed = 0;
}


void set_psu_v(float volt)
{
  suspend_psu_polling=true;
  delay(30);
  // implement inverse psu gain trim based on observed accuracy at 50V
  // ie set to something that should produce 50V when asked to set to 50V
  sprintf(outgoing_data.message, ":01w10=%04d,\n", (int32_t) (100.0 * ((volt/psu_gain) -psu_offset)));
  // Serial.printf(" volt=%f  made %s g=%f o=%f\n", volt, outgoing_data.message, psu_gain, psu_offset);
  esp_now_send(psu_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
  suspend_psu_polling=false;
  delay(30);
}

void set_psu_i(float current)
{
  suspend_psu_polling=true;
  delay(30);
  sprintf(outgoing_data.message, ":01w11=%05d,\n", (int32_t ) 1000.0 * current);
  esp_now_send(psu_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
  suspend_psu_polling=false;
  delay(30);
}

void set_psu_e(int value)
{
  char out_buf[20];
  suspend_psu_polling=true;
  delay(30);
  sprintf(outgoing_data.message, ":01w12=%1d,\n",  value);
  esp_now_send(psu_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
  suspend_psu_polling=false;
  delay(30);
}

uint32_t get_SOC (uint16_t address)
{
  uint32_t retval;
  //  retval  = fram.read32(address);
  retval  = (uint32_t) (fram.read(address*4) << 24)  +  (uint32_t) (fram.read(address*4+1) << 16)
    + (uint32_t) ( fram.read(address*4+2) << 8)      + (uint32_t) fram.read(address*4+3);     
  return retval;
}

void write_SOC (uint16_t address, int32_t data)
{ // write all 3 fram addresses
  write_fram_SOC (address , data);
  write_fram_SOC (address+1 , data);
  write_fram_SOC (address+2 , data);
}

void write_fram_SOC (uint16_t address, int32_t data)
{
  digitalWrite(fram_wp_pin, LOW);
  // fram.write32(address, data);
  fram.write(address*4,   (uint8_t) (data >>24));
  fram.write(address*4+1,  (uint8_t) (data >>16));
  fram.write(address*4+2, (uint8_t) (data >>8 ));
  fram.write(address*4+3, (uint8_t) (data ));
  digitalWrite(fram_wp_pin, HIGH);
}
    
void do_serial_if_ready (void)
{
  int serial_byte;
  if (Serial.available())
    {
      serial_byte = Serial.read();
      if ((serial_byte != '\n') && (serial_buf_pointer < longest_record))
	{
	  serial_buf[serial_buf_pointer] = serial_byte;  // write char to buffer if space is available
	  serial_buf_pointer++;
	}    
      if (serial_byte == '\n') {
	serial_buf[serial_buf_pointer] = (uint8_t) 0;  // write string terminator to buffer
	if (serial_buf_pointer >= 1) {                 // at least a command letter
	  parse_buf(serial_buf);      // parse the buffer if at least one char in it.
	  // Serial.print(return_buf);
	}
	serial_buf_pointer = 0;
      }
    }
}



void parse_psu_wifi_buf (char * buf)
{
  if (buf[0]==':')
    {
      if ((buf[3]=='o') && (buf[4]=='k'))
	{ // typical write response.
	  //	  Serial.println("psu: ok");
	  charger.mostrecent = rtc.getEpoch{};
	}
      else if (buf[3]=='r')
	{ // general read handler
	  int cmdaddr;
	  int value;
	  int matched;
	  matched = sscanf(buf, ":01r%2d=%d.", &cmdaddr, &value);
	  //	  Serial.printf("%s matched=%d addr=%d val=%d\n", buf, matched, cmdaddr, value);
	  if (matched==2)
	    {
	      if (cmdaddr == 30)
		//  Serial.printf("psu: meas_voltage=%d.%02dV\n", value/100, value % 100);  
		charger.voltage = psu_gain * (psu_offset+ (float) (value  / 100.0 ))  ;
	      else if (cmdaddr == 31)
		//	      Serial.printf("psu: meas_current=%d.%03dA\n", value/1000, value % 1000);  
		charger.current = (float) value / 1000.0;
	      else if (cmdaddr == 12)
		if (value == 0)
		  charger.enable = 0;
	      // Serial.print("psu: status=CV\n");
		else 
		  charger.enable = 1;
	      charger.mostrecent = rtc.getEpoch{};
	  }
	}
    }
}

void parse_vmon_wifi_buf (char * buf, uint8_t address)
{
  int       value;
  float     fvalue;
  int       matched;

  switch (buf[0])
    {
    case 'b' : // balance state (can wiggle when hot)
      matched = sscanf(buf, "b=%1d\n",  &value);
      if (matched==1)
	{
	  vmon[address].act_balance = value;
	  vmon[address].received++;
	}
      else if ((buf[1] == 'o') && (buf[2] == 'k'))
	  vmon[address].received++;
      else
	vmon[address].protocol_err++;
      break;

    case 'v' : // read voltage
      matched = sscanf(buf, "v=%f\n",  &fvalue);
      if (matched==1)
	{
	  vmon[address].volt = fvalue;
#ifdef MEAS_PERF
	  vmon_polltime[address].resp = millis();
	  Serial.printf("vmon %d serial voltage poll took %d milliseconds\n", address+1,
			vmon_polltime[address].resp - vmon_polltime[address].issue);
#endif
	  vmon[address].received++;
	  vmon[address].mostrecent = rtc.getEpoch();
	}
      else 
	vmon[address].protocol_err++;
      break;
      
    case 'r' : // read resistor temp
      matched = sscanf(buf, "r=%dC\n",  &value);
      if (matched==1)
	{
	  vmon[address].restemp = value;
	  vmon[address].received++;
	}
      else 
	vmon[address].protocol_err++;
      break;
      
    case 's' : // read battery temp
      matched = sscanf(buf, "s=%dC\n",  &value);
      if (matched==1)
	{
	  vmon[address].battemp = value;
	  vmon[address].received++;
	}
      else 
	vmon[address].protocol_err++;
      break;

    case 'G' : // read adc gain
      matched = sscanf(buf, "G=%f\n",  &fvalue);
      if (matched==1)
	{
#ifdef DEBUG
	  Serial.printf("read vmon %d adc_gain=%f\n", address, fvalue);
#endif
	  vmon[address].gain = fvalue;
	  // vmon[address].received++;
	}
      else 
	vmon[address].protocol_err++;
      break;

    case 'O' : // read adc gain
      matched = sscanf(buf, "O=%f\n",  &fvalue);
      if (matched==1)
	{
#ifdef DEBUG
	  Serial.printf("read vmon %d adc_offset=%f\n", address, fvalue);
#endif
	  vmon[address].offset = fvalue;
	  // vmon[address].received++;
	}
      else 
	vmon[address].protocol_err++;
      break;

    default:
      vmon[address].protocol_err++;
      break;
    }
}
void parse_imon_wifi_buf (char * buf)
{
  float     fvalue;
  int       matched;

  matched = sscanf(buf, "I=%f",  &fvalue);
  if (matched==1)
    {
      imon.current = fvalue;
      imon.mostrecent = rtc.getEpoch();
      imon.received++;
    }
  else
    {
      imon.protocol_err++;
    }      
}

//

