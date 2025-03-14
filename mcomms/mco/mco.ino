//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'mco  Time-stamp: "2025-03-15 08:58:18 john"';

// this is the app to run the mower comms controller for the Ryobi mower.
// use tools -> board ->  ESP32 Dev module 

/* mco
   (maybe) can can talk to the power supply to adjust current and voltage settings.
     or maybe mcc does.
   mco is responsible for talking to the 4 vmons.
   And, directly or indirectly, psu.
   It has to report all relevant loggable info to mcc.
   It has to get the info from vmons. It runs this.

   Locally, mco has access to the currne into/out of the batteries via current shunt and adc
   It intergates this to get SOC.
   It zeros SOC on empty, adjusting the battery capacity measure when that happens.
   It 100%s SOC on full, adjusting the battery capacity measure when that happens.
   It beeps on empty when battery empty && not_charging
   it records SOC in local fram. multiple copies most likely.

   so have to get up in some order
   comms to vmons
   fram driver
   adc
   beeper
   soc meter driver 
   NVM
   wifi 
   then the core logic.
*/

// test cmt MAC is 5c013b6c9938
// test mco MAC is      which replaces the cmt later in the dev process Mower COntroller
// test psu MAC is       wifi to serial adapter for the power supply. Originally a vichy mm adaptor
// test mcc MAC (ME) is 5c013b6cf4fc   MowerChargeController

#include <Preferences.h>  // the NV memory interface
#include <Wire.h>
#include "ESP32Time.h"   // must be V2.x or newer. V1.04 takes 5s to return a time string. breaks logging 
#include <ADS1115_WE.h>   // adc converter
// fram
#include <stdio.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "FRAM.h"
// #define DEBUG

// for preferences
#define RW_MODE false
#define RO_MODE true

const uint8_t vmons = 4;  // how many vmons am I servicing 
struct vsdata
{
  float     volt = 0.0;         // battery voltage
  int16_t   battemp = 0;        // battery temperature
  int16_t   restemp = 0;        // resistor temperature
  uint32_t  mostrecent = 0;     // timestamp of most recent message from device
  uint32_t  received = 0;       // number of messages I've received from this board   
  uint32_t  sent = 0;           // number of messages I've sent to this board
  uint8_t   balance = 0;        // should it be balancing?
  uint8_t   spare1 = 0;
  uint16_t  protocol_err = 0;   // lets keep the struct aligned
};
struct vsdata vdata[vmons];   // state structure for all the vmons    
int32_t s2_comms_errors = 0;

const uint8_t adc_addr = 0x48;
ADS1115_WE adc = ADS1115_WE(adc_addr); // I2C connected ads1115 16b adcb  Used for battery voltage.

// serial is used for debug and for programming NVM
const uint8_t longest_record = 24;  //   worst is display comand WD1=12345678901234567890
const uint8_t record_size = longest_record + 2;    //  char char = nnnnn \n
char serial_buf[record_size];
uint8_t serial_buf_pointer;

// serial2 is used for communicating with vmons
const uint8_t longest_record2 = 24;  //   worst is display comand WD1=12345678901234567890
const uint8_t record_size2 = longest_record2 + 2;    //  char char = nnnnn \n
char serial2_buf[record_size2];
uint8_t serial2_buf_pointer;


uint8_t baseMac[6];         // my own mac address
const  uint16_t msgbuflen= 128;  // for wifi transfers
char return_buf[msgbuflen]; // for responses

const char * version = "MCO 14 Mar 2025 Reva";

Preferences mcoPrefs;  // NVM structure
// these will be initialized from the NV memory

uint8_t psu_mac[6];
uint8_t mcc_mac[6];
uint8_t cmt_mac[6];
float   initial_charge_current;
float   initial_charge_voltage;
float   topoff_charge_current;
float   topoff_charge_voltage;
float   transition_current;
float   cutoff_current;
float   max_battery_voltage;
float   batt_balance_voltage;
float   batt_balance_tol_voltage;
int32_t  battery_capacity;  //mA Seconds
int32_t  beep_SOC;
float   adc_gain;
float   adc_offset;
float   psu_gain;
float   psu_offset;

// and runtime variables
float battery_current;
float charger_voltage;
float charger_current;
bool  charger_enable;


typedef struct struct_message {
  char message[msgbuflen];
} struct_message;

// create the wifi message struct
struct_message message_data;

esp_now_peer_info_t peerInfo;

// wifi callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}

struct_message response_data;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&response_data, incomingData, sizeof(response_data));
  //  Serial.print("received ");
  // Serial.print(len);
  //Serial.print(" bytes from ");  
  //Serial.printf("%02x%02x%02x%02x%02x%02x\n", mac[12], mac[13], mac[14], mac[15], mac[16], mac[17]) ;
  /* I suspect the mac' field here is the 24 byte mac header structure espressif uses
     fields 12 to 17 seem to be the source MAC. rest isn't obvious */

  parse_buf(response_data.message);
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
uint32_t       last_temp_update_time;         // time last temperature was polled at, in seconds
const uint16_t temp_update_period = 10; // seconds.

uint32_t last_current_update_time;
const uint16_t current_update_period = 1; // seconds.


//uint32_t last_vmon_update_time;         // time last vmon was polled at
//const uint16_t vmon_update_period = 1;  // seconds.
//uint8_t vmon_upto = 0;

bool adc_is_converting = false;
FRAM fram;

// beeper
// setting PWM properties
const int freq_beep = 2000;
const int freq_soc_50 = 168;
const int resolution = 8;
const uint8_t beep_pwm_channel = 0;
const uint8_t soc_pwm_channel = 1;
bool last_beep_state=0;

// vmon polling

uint8_t vmon_ii =0;
const uint8_t vmon_ii_max =24;
uint32_t last_vmon_time=0;
const uint32_t vmon_period_millis = 1000 / vmon_ii_max ; // roughly 40 ms 
uint8_t vmon_which[] ={ '1','2','3','4','1','2','3','4','1','2','3','4','1','2','3','4','1','2','3','4','1','2','3','4' };
uint8_t vmon_ltr[]   ={ 'v','v','v','v','r','r','r','r','v','v','v','v','s','s','s','s','v','v','v','v','e','e','e','e'};

// psu polling
bool suspend_psu_polling = false;
uint8_t psu_ii =0;
uint32_t last_psu_time=0;
const uint8_t psu_ii_max = 3;
const uint32_t psu_period_millis = 1000 / psu_ii_max ; // roughly 300 ms 
uint8_t psu_addr[] ={ 30, 31, 12 };

void setup (void) {
  Serial.begin(115200);
  Serial.println(version);

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
   ledcAttachChannel(buz_en_pin, freq_beep, resolution, beep_pwm_channel);
   // soc allocate
   ledcAttachChannel(soc_pps_pin, freq_soc_50, resolution, soc_pwm_channel);

   ledcWrite(buz_en_pin, 0);     // beeper off
   ledcWrite(soc_pps_pin, 128);  // set soc gauge to 50%
   

   // init some general variables and IOs 
   serial_buf_pointer = 0;
   serial2_buf_pointer = 0;
   last_current_update_time = rtc.getEpoch();


   // at startup, confirm all 4 vmons are present.
   // play a beep code if not so.

   // init adc
   if(!adc.init()){
     Serial.println("ADS1115 not connected!");
   }
   adc.setVoltageRange_mV(ADS1115_RANGE_4096);  // 4.096V input range.
   // input attenuator about 0.2  0.2 * 15V max = 3V
   adc.setCompareChannels(ADS1115_COMP_0_1);    // measure between 0 and 1.
   adc.setConvRate(ADS1115_8_SPS);              // 8 samples per sec, slowest.
   adc.setAlertPinMode(ADS1115_DISABLE_ALERT);  // not using voltage range checks
   adc.setMeasureMode(ADS1115_SINGLE);          // initiate measurements

   if (!fram.begin(0x50))
     {
       Serial.println("FRAM not connected!");
     }
   
   // check soc. stored in fram 3 times. choose the majority. 
   if (get_SOC(0) == get_SOC(1))
     {
     if (get_SOC(2) != get_SOC(0))
       write_SOC(2, get_SOC(0));    // is 2 is the odd one out, rewrite it.
     }
   else if (get_SOC(0) == get_SOC(2))
     {
       if (get_SOC(1) != get_SOC(0))
	 write_SOC(1, get_SOC(0));    // is 1 is the odd one out, rewrite it.
     }
   else if (get_SOC(1) == get_SOC(2))
     {
       if (get_SOC(0) != get_SOC(1))
	 write_SOC(0, get_SOC(1));    // is 1 is the odd one out, rewrite it.
     } 
   else
     { 
       write_SOC(0, 3600000 * 50);    // initilize to 50% as I have no idea
       write_SOC(1, 3600000 * 50);    // initilize to 50% as I have no idea
       write_SOC(2, 3600000 * 50);    // initilize to 50% as I have no idea
     }       
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
      return;
    }
    // Register peer psu
    memcpy(peerInfo.peer_addr, psu_mac, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    // Register peer cmt
    memcpy(peerInfo.peer_addr, cmt_mac, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
}

   

void loop (void)
{
  // service serial character if any available.
  do_serial_if_ready();
  do_serial2_if_ready();

  
  // update current and SOC state if its due 
  if (rtc.getEpoch() > (last_current_update_time + current_update_period))
    {
      if (adc_is_converting)
	{
	  while(adc.isBusy()) { Serial.println("adc is busy?");}
	  battery_current = adc_gain * (adc.getResult_V() + adc_offset);
	  adc_is_converting = false;
	  //        in mAS                  in A              to mA, period is 2x current_update_period  
	  int32_t soc = get_SOC(0) + (int32_t) (battery_current * 2000.0 * current_update_period);
	  write_SOC(0, soc);
	  write_SOC(1, soc);
	  write_SOC(2, soc);
	  int soc_f =     336 * ( (float) soc / (float) battery_capacity);     
	  ledcAttachChannel(soc_pps_pin, soc_f, resolution, soc_pwm_channel);

	  if (last_beep_state)
	    {
	      ledcWrite(buz_en_pin, 0);     // beeper off
	      last_beep_state = false;
	    }
	  else
	    {
	      if (digitalRead(charger_connected_pin) && (soc < beep_SOC))
		ledcWrite(buz_en_pin, 128);     // beeper on
	      last_beep_state = true;
	    }
		
	}
      else  // every current_update_period the adc gets started, then read, etc. 
	{
	  adc.startSingleMeasurement();
	  adc_is_converting = true;
	}
      last_current_update_time = rtc.getEpoch();
    }
  
  // update voltage from vmons if its due . update all about once per second
  if (millis()  > (last_vmon_time + vmon_period_millis))
    {
      Serial2.printf(":%cr%c\n", vmon_which[vmon_ii], vmon_ltr[vmon_ii]);
      vmon_ii++;
      if (vmon_ii >= vmon_ii_max)
	vmon_ii = 0;
      last_vmon_time = millis();
    }

  
      // poll psu
  // update voltage/current/enable from psu if its due . update all about once per second
  if (millis()  > (last_psu_time + psu_period_millis))
    {
      if (! suspend_psu_polling)
	{
	  Serial2.printf(":%cr%c\n", vmon_which[vmon_ii], vmon_ltr[vmon_ii]);
	  vmon_ii++;
	  if (vmon_ii >= vmon_ii_max)
	    vmon_ii = 0;
	}
      last_psu_time = millis();
    }

  


      // FIXME do logic
      

  
}

const uint8_t default_mac[]     = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
const uint8_t default_mcc_mac[] = { 0x5c, 0x01, 0x3b, 0x6c, 0xf4, 0xfc };
const uint8_t default_psu_mac[] = { 0x5c, 0x01, 0x3b, 0x6c, 0xe2, 0xd0 };
const uint8_t default_cmt_mac[] = { 0x5c, 0x01, 0x3b, 0x6c, 0x99, 0x38 };

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
  mcoPrefs.putFloat("mbv", 14.3);            // max single battery voltage
  mcoPrefs.putFloat("bbv",  13.8);           // start balancing battery above this voltage 3.45 * 4 =13.8 
  mcoPrefs.putFloat("bbt",  0.005);          // battery balance tolerance
  mcoPrefs.putLong("bsoc",  36000000);       // beep SOC, 10%
  mcoPrefs.putLong("bcap", 360000000);       // battery capacity in mAS  100AH = 360e6 maS 

  mcoPrefs.putBool("nvsInit", true);            // Create the "already initialized"
  //  key and store a value.
  // The "factory defaults" are created and stored so...
  mcoPrefs.end();                               // Close the namespace in RW mode and...
}


void load_operational_params(void)
{

  mcoPrefs.begin("mcoPrefs", RO_MODE);       // Open our namespace (or create it
                                               //  if it doesn't exist) in RO mode.
   // Retrieve the operational parameters from the namespace
   //  and save them into their run-time variables.

   mcoPrefs.getBytes("psumac", psu_mac, 6);             // load power supply mac
   mcoPrefs.getBytes("mccmac", mcc_mac, 6);             // load mco/cmt supply mac
   mcoPrefs.getBytes("cmtmac", cmt_mac, 6);             // load mco/cmt supply mac
   mcoPrefs.getFloat("adco",  adc_offset);              // current adc offset term
   mcoPrefs.getFloat("adcg", adc_gain);                 // current adc gain term
   mcoPrefs.getFloat("psug", psu_offset);               // cal term from local voltage to psu 
   mcoPrefs.getFloat("psuo", psu_gain);                 // cal term from local voltage to psu 
   mcoPrefs.getFloat("icc",  initial_charge_current);   // initial charge current
   mcoPrefs.getFloat("icv",  initial_charge_voltage);   // initial charge voltage 13.8*4 = 55.2
   mcoPrefs.getFloat("tcc",  topoff_charge_current);    // topoff charge current
   mcoPrefs.getFloat("tcv",  topoff_charge_voltage);    // topoff charge voltage 14.0*4 = 56
   mcoPrefs.getFloat("tc",   transition_current);       // transition current
   mcoPrefs.getFloat("fc",   cutoff_current);           // final current
   mcoPrefs.getFloat("mbv",  max_battery_voltage);      // max single battery voltage
   mcoPrefs.getFloat("bbv",  batt_balance_voltage); // start balancing a battery above this 3.45 * 4 =13.8
   mcoPrefs.getFloat("bbt",  batt_balance_tol_voltage); // battery balance tolerance
   mcoPrefs.getLong("bsoc",  beep_SOC);                 // beep SOC, 10%
   mcoPrefs.getLong("bcap",  battery_capacity);         // battery capacity in mAS  100AH = 360e6 maS 

   // All done. Last run state (or the factory default) is now restored.
   mcoPrefs.end();                                      // Close our preferences namespace.
}

// this is for S0, debug
void parse_buf (char * in_buf)
{
  // Z eraZe NV memory
  // Rf Read and print Field,
  //    where Field could be    lower case for current stuff, upper case for NVM
  //          e s2 comms errors
  //          i battery current 
  //          v[1234] vmon state
  //          t current time
  //          c charger voltage, current, enable
  //          s SOC
  //          M Show macs
  //          V=version
  //          I=initial charge current, voltage
  //          B=balance above per battery voltage, tolerance, maxV
  //          E BEep below this 
  //          T=topoff voltage, current, final current
  //          G[VI] adc gain   current=local adc voltage=DPM8624
  //          P[VI] adc offset current=local adc voltage=DPM8624
  //          S     SOC total capacity in mA.S i guess
  // Wf Field,
  //    where Field could be
  //          Mx mac    WMP=<12 ascii hex digits>  mac for Psu or Mcc or Cmt
  //          cv=f    charger voltage   set psu V
  //          ci=f    charger current   set psu I
  //          ce=[01] charger enable    set pse e
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
  //                        
  int8_t  address;
  
  uint8_t  cmd;
  uint8_t  field;
  uint8_t  field2;
  int      value;
  uint8_t  mvalue[6]; // mac address
  float    fvalue;
  const uint8_t out_buf_len = 20;
  char     out_buf[out_buf_len];
  int      match ;
  int      i;
  uint32_t soc;
  cmd = in_buf[0];
  
  switch (cmd) {
  case 'Z':
    reinit_NVM();
    load_operational_params();
    break;

  case 'R':
    switch (in_buf[1])
      {
      case 'e':
	Serial.printf("Serial2 comms errors= %d\n", s2_comms_errors); 
	break;
      case 'i':
	Serial.printf("Battery current=%f\n", battery_current); 
	break;
      case 'v':
	address = (in_buf[2] & 0x03) -1 ;   // convert ascii 1..4 ie 0x31 to 0x34 to 0..3  31->2 34->1  
	if (address < 0)
	  address=3;
	Serial.printf("Vmon %d voltage=%f temp=%dC resistor=%dC updated %s s ago msgs=%d errs=%d\n",
		      address+1, vdata[address].volt,
		      vdata[address].battemp, vdata[address].restemp,
		      rtc.getEpoch() - vdata[address].mostrecent,
		      vdata[address].protocol_err); 
	break;
      case 't' :
	Serial.printf("current time %d %s\n", rtc.getEpoch(), rtc.getTime()); 
	break;
      case 'c' :
	Serial.printf("charger voltage=%1.2f current=%1.3f enable=%d\n",
		      charger_voltage, charger_current, charger_enable); 
	break;
      case 's' :
	Serial.printf("SOC=%dmAS %2.1f%%\n", get_SOC(0), 100.0 * (float) get_SOC(0) / (float)battery_capacity);
	break;
      case 'M':
	Serial.printf("MCO_MAC=%02x%02x%02x%02x%02x%02x PSU=%02x%02x%02x%02x%02x%02x MCC=%02x%02x%02x%02x%02x%02x CMT=%02x%02x%02x%02x%02x%02x\n",
		      baseMac[0],baseMac[1],baseMac[2],baseMac[3],baseMac[4],baseMac[5],
		      psu_mac[0],psu_mac[1],psu_mac[2],psu_mac[3],psu_mac[4],psu_mac[5],
		      mcc_mac[0],mcc_mac[1],mcc_mac[2],mcc_mac[3],mcc_mac[4],mcc_mac[5],
		      cmt_mac[0],cmt_mac[1],cmt_mac[2],cmt_mac[3],cmt_mac[4],cmt_mac[5]);
	break;
      case 'I' :
	Serial.printf("Initial charge voltage=%2.2V current=%2.3A\n",
		      initial_charge_voltage, initial_charge_current);
	break;
      case 'T' :
	Serial.printf("Topoff charge voltage=%2.2V current=%2.3A\n",
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
	Serial.printf("Balance battery above %2.3fV Tolerance=%1.3fV Max voltage=%2.3fV\n",
		      batt_balance_voltage, batt_balance_tol_voltage, max_battery_voltage);
	break;
      case 'E' :
	Serial.printf("Beep at or below SOC=%d mAS %d%%\n",
		      beep_SOC, 100*beep_SOC/battery_capacity);
	break;
      case 'S' :
	Serial.printf("Battery capacity=%d mAS %dAH\n",
		      battery_capacity, battery_capacity/3600000);
	break;
      case 'V':
	snprintf(out_buf, out_buf_len, "%s\n", version);
	break;
      }
      break;

  case 'W':
    switch (in_buf[1]) {
    case 'd':
      memcpy(&response_data.message, in_buf, strlen(in_buf));
      esp_now_send(mcc_mac, (uint8_t *) &response_data, sizeof(response_data));
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
    case 's' :
      match =  sscanf(in_buf, "%c%c=%d", &cmd, &field, &value);
      soc  = (float) value * (float) battery_capacity / 100.0;
      write_SOC(0, soc );
      write_SOC(1, soc );
      write_SOC(2, soc );
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
      else if (field2 == 'M') 
	mcoPrefs.putFloat("mbv", fvalue);
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
    }      
    break;
    // end of 'W'
  }    
}

void set_psu_v(float volt)
{
  char out_buf[20];
  suspend_psu_polling=true;
  delay(100);
  esp_now_send(psu_mac, (uint8_t *) &response_data, sizeof(response_data));
  suspend_psu_polling=false;
  delay(100);
}

void set_psu_i(float current)
{
  char out_buf[20];
  suspend_psu_polling=true;
  delay(100);
  sprintf(response_data.message, ":01w11=%05d,\n", (int ) 1000.0 * current);
  esp_now_send(psu_mac, (uint8_t *) &response_data, sizeof(response_data));
  suspend_psu_polling=false;
  delay(100);
}

void set_psu_e(int value)
{
  char out_buf[20];
  suspend_psu_polling=true;
  delay(100);
  sprintf(response_data.message, ":01w12=%1d,\n",  value);
  esp_now_send(psu_mac, (uint8_t *) &response_data, sizeof(response_data));
  suspend_psu_polling=false;
  delay(100);
}

uint32_t get_SOC (uint16_t address)
{
  uint32_t retval;
  retval  = fram.read32(address);
  return retval;
}

void write_SOC (uint16_t address, int32_t data)
{
  digitalWrite(fram_wp_pin, LOW);
  fram.write32(address, data);
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
	  Serial.print(return_buf);
	}
	serial_buf_pointer = 0;
      }
    }
}


// this is a parser for S2.  It records messages back from a vmon.
// as this is a shared bus, I want a checksum.
// :a<string>cs\n
// WHERE : is start of message
// a is the address of the sender ascii(31) .. ascii(34)
// <string> is the response to some query from the tx side of this process
// cs is a 2 character hex integer such that the sum of the ascii characters from the :
//    to immediately before the cs / (last string char) inclusive is 0 base 256.
// messages with illegal checksums are ignored and an error count is incremented.
// it records the most timestamp of messages from each of the 4 vmons
// it records the number of responses from each vmon
// it stores voltage and temperatures from each vmon in a struct
// it records responses from illegal addresses as a sort of error counter

void parse2_buf (char * in_buf, uint8_t length)
{
  /* the <string> part of the field I know about is 
   * writes produce ok. I shall just ignore that here
   * reads that I care about produce
   *   rv=<float>V
   *   rr=intC
   *   rs=intC
   *   rb=int.
   */
  
  uint8_t   field;
  int       value;
  float     fvalue;
  uint8_t   match ;
  uint8_t   address;   // its a short integer here, not a char. 0 based!
  uint8_t   cs_carried;
  uint8_t   cs_calc;
  int i;
  // uint8_t length; input points to the terminating 0. Messages shorter than :aokcs0 dont get here.
  // do checksum validation
  // hmmm, scanf requires a const address string. so tail anchor with \n
  match = sscanf(in_buf, "%2x\n", &cs_carried);
  if (match != 1)
    {
      s2_comms_errors++;
      return;
    }
  cs_calc = cs_carried;
  for (i=0; i<(length-2); i++)
    cs_calc += in_buf[i];
  if (cs_calc != 0)
    {
      s2_comms_errors++;
      return;
    }
  if (in_buf[0] != ':')
    {
      s2_comms_errors++;
      return;
    }
  address = (in_buf[1] & 0x0f) -1;   // 0 based uint
  if (address >= vmons)
    {
      s2_comms_errors++;
      return;
    }

  // now basic checks are out of the way, lets record message and timestamp
  // anything not understood is a protocol error
  vdata[address].mostrecent = rtc.getEpoch();
  vdata[address].received++;
  
  switch (in_buf[2])
    {
    case 'r':
      switch (in_buf[3])
	{
	case 'v':
	  match = sscanf(in_buf + 4, "=%fV", &fvalue);
	  if (match == 1)
	    vdata[address].volt=fvalue;
	  else
	    vdata[address].protocol_err++;
	  break;

	case 'r':
	  match = sscanf(in_buf + 4, "=%dC", &value);
	  if (match == 1)
	    vdata[address].restemp=value;
	  else
	    vdata[address].protocol_err++;
	  break;

	case 's':
	  match = sscanf(in_buf + 4, "=%dC", &value);
	  if (match == 1)
	    vdata[address].battemp=value;
	  else
	    vdata[address].protocol_err++;
	  break;

	case 'b':
	  match = sscanf(in_buf + 4, "=%d.", &value);
	  if (match == 1)
	    vdata[address].balance=value;
	  else
	    vdata[address].protocol_err++;
	  break;

	default: 
	    vdata[address].protocol_err++;
	} // end of case 'r'

    case 'o' :
      if (in_buf[3] != 'k')
	    vdata[address].protocol_err++;
      break;

    default:
      vdata[address].protocol_err++;
    }
}

void do_serial2_if_ready (void)
{
  int serial_byte;
  if (Serial2.available())
    {
      serial_byte = Serial2.read();
      if ((serial_byte != '\n') && (serial2_buf_pointer < longest_record))
	{
	  serial2_buf[serial2_buf_pointer] = serial_byte;  // write char to buffer if space is available
	  serial2_buf_pointer++;
	}    
      if (serial_byte == '\n')
	{
	  serial2_buf[serial2_buf_pointer] = (uint8_t) 0;  // write string terminator to buffer
	  if (serial_buf_pointer >= 6) {                 // min permitted is :aokcs0
	    parse2_buf(serial2_buf, serial2_buf_pointer);      // parse the buffer if min message length.
	  }
	  else
	    s2_comms_errors++;
	serial2_buf_pointer = 0;
	}
    }
}


