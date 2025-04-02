//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'imon   Time-stamp: "2025-03-24 14:29:45 john"';

// this is the app to run an isolated voltage shunt resistor current probe
// use tools -> board ->  ESP32 Dev module 

/*
  A vmon is an esp32 with an adc sensing a current shunt and talkign to an mco.
  the current shunt inputs are biassed to midrange vcc , 2 510R resistors to 3v3 and gnd.
  
  Its powerd from an isolateed switcher from 12v, producing about 5V.
  It chats to mco  via wifi.  This is the esp_now() branch.

  Imon contains a calibrated 16 bit ADC to monitor the battery current.
  Imon relays current to the mco, unpolled. About 4x a second.
*/

// test mco MAC is 5c013b6c6a44  
// Im at     9c9c1fc6f7ac after replacing esp32



#include <Preferences.h>  // the NV memory interface
#include <Wire.h>
#include<ADS1115_WE.h>   // adc converter
#include <stdio.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
// #define DEBUG

// for preferences
#define RW_MODE false
#define RO_MODE true


// serial0 is used for debug and for programming NVM
const uint8_t longest_record = 24;  //   worst is display comand WD1=12345678901234567890
const uint8_t record_size = longest_record + 2;    //  char char = nnnnn \n
char serial_buf[record_size];
uint8_t serial_buf_pointer;

const  uint16_t msgbuflen= 128;  // for serial responses
char return_buf[msgbuflen]; 

const char * version = "IMON WIFI 2 Apr 2025 Rev1";

Preferences imonPrefs;  // NVM structure
// these will be initialized from the NV memory

uint8_t baseMac[6];         // my own mac address
uint8_t mco_mac[6];
uint8_t cmt_mac[6];

float   adc0;
float   adcgain;
bool nap;

/* there are no IOs  */

// I2C addresses 
const uint8_t adc_addr = 0x48;

ADS1115_WE adc = ADS1115_WE(adc_addr); // I2C connected ads1115 16b adcb  Used for battery current.

float   current = 0.0;           // current battery 
uint32_t board_id;

const uint32_t current_update_period = 250; // milliseconds.
uint32_t last_current_update_time;

uint32_t nap_us = current_update_period * 800; // nap for 80% of the time, and scale ms to us. Stay an integer.
// uint32_t nap_us = 200000; 

// create the wifi message struct

typedef struct struct_message {
  char message[msgbuflen];
} struct_message;

struct_message outgoing_data;
esp_now_peer_info_t peerInfo;

// wifi callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
      // nap here, as data has just been sent.
  if (nap)
    {
      esp_wifi_stop();
      esp_err_t result = esp_sleep_enable_timer_wakeup(nap_us);
      esp_light_sleep_start();     // Enter light sleep
      esp_wifi_start();
      // note, napping is bad for the serial processing. If debugging serially, either reduce the nap time to ~50%
      // of the sampe period, of just comment out these 4 lines above. Will just take the power dissipation
      // of imon up to about 70mA at 12V with no napping. About half of that is the radio.
      // probably about 1/4 of that with napping included.   Not a lot for imon to do while waiting to
      // do the next current sample.
    }
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
}


void setup (void) {
  Wire.begin();  
  Serial.begin(115200);
  Serial.println(version);

  nap = true;
  // init adc
  if(!adc.init()){
    Serial.println("ADS1115 not connected!");
  }
  adc.setVoltageRange_mV(ADS1115_RANGE_0256);  // 256mv input range. Its a 75mv shunt at 100A.
  adc.setCompareChannels(ADS1115_COMP_0_1);    // measure between 0 and 1.
  adc.setConvRate(ADS1115_8_SPS);              // 8 samples per sec, slowest.
  adc.setAlertPinMode(ADS1115_DISABLE_ALERT);  // not using voltage range checks
  adc.setMeasureMode(ADS1115_CONTINUOUS);      // initiate measurements
  
  // check NVM looks defined  
  imonPrefs.begin("imonPrefs", RO_MODE);     // Open our namespace (or create it if it doesn't exist)
  bool tpInit = imonPrefs.isKey("nvsInit"); // Test for the existence of the "already initialized" key.
  imonPrefs.end();                          // close the namespace in RO mode
   if (tpInit == false) 
     reinit_NVM();                          // reinitialize the nvm structure if magick key was missing 
   
   // load local variables from NVM
   load_operational_params();                // load all variables from NVM

   // init some general variables and IOs 
   serial_buf_pointer = 0;

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
   
   // Once ESPNow is successfully Inited, we will register for Send CB to
   // get the status of Trasnmitted packet
   esp_now_register_send_cb(OnDataSent);
   
   // Once ESPNow is successfully Inited, we will register for recv CB to
   // get recv packer info
   esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

   Serial.println("register macs");
   // Register peer mco
   memcpy(peerInfo.peer_addr, mco_mac, 6);
   peerInfo.channel = 0;  
   peerInfo.encrypt = false;
    
   if (esp_now_add_peer(&peerInfo) != ESP_OK){
     Serial.println("Failed to add peer");
   }
   Serial.println("done setup");
}

   

void loop (void)
{
  // service serial character if any available.
  do_serial_if_ready();

   // update temperature and monitor balance state if its due 
  if (millis() > (last_current_update_time + current_update_period))
    {
      current = get_current();
      snprintf(outgoing_data.message, sizeof(outgoing_data), "I=%1.3fA\n", current);
      esp_now_send(mco_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));

      last_current_update_time = millis();


    }
}

float get_current ()
{
  float I;
  I = adcgain * (adc.getResult_V() + adc0);
  return I;
}
  
const uint8_t default_mac[]     = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
const uint8_t default_mco_mac[] = { 0x5c, 0x01, 0x3b, 0x6c, 0x6a, 0x44 };
const uint8_t default_cmt_mac[] = { 0x5c, 0x01, 0x3b, 0x6c, 0x99, 0x38 };

void reinit_NVM (void)
{
  Serial.println("Initializing NVM");
  
  // If tpInit is 'false', the key "nvsInit" does not yet exist therefore this
  //  must be our first-time run. We need to set up our Preferences namespace keys. So...
  imonPrefs.begin("imonPrefs", RW_MODE);       //  open it in RW mode.
  
  // The .begin() method created the "vmnPrefs" namespace and since this is our
  //  first-time run we will create
  //  our keys and store the initial "factory default" values.

  imonPrefs.putUInt("boardid", 1);           // boardID
  imonPrefs.putFloat("adcgain", 1000.0/0.75);  // approx adc gain term 75mv/100A, 1A=0.75mv
  imonPrefs.putFloat("adc0", 0.0);             // adc 0 offset term
  imonPrefs.putBytes("cmtmac", default_cmt_mac, 6);  // mac address of tester
  imonPrefs.putBytes("mcomac", default_mco_mac, 6);  // mac address of mower comms controller 
  imonPrefs.putBool("nvsInit", true);            // Create the "already initialized"
  //  key and store a value.
  // The "factory defaults" are created and stored so...
  imonPrefs.end();                               // Close the namespace in RW mode and...
}


void load_operational_params(void)
{

  imonPrefs.begin("imonPrefs", RO_MODE);       // Open our namespace (or create it
                                             //  if it doesn't exist) in RO mode.
   // Retrieve the operational parameters from the namespace
   //  and save them into their run-time variables.
  adcgain          = imonPrefs.getFloat("adcgain");       // adc gain term
  adc0             = imonPrefs.getFloat("adc0");          // adc 0 offset term
  board_id    = imonPrefs.getUInt("boardid");         // board id
  imonPrefs.getBytes("mcomac", mco_mac, 6);             // load mco comms controller mac
  imonPrefs.getBytes("cmtmac", cmt_mac, 6);             // load cmt tester mac
  
   // All done. Last run state (or the factory default) is now restored.
   imonPrefs.end();                                      // Close our preferences namespace.
}

void parse_buf (char * in_buf, char * out_buf, int out_buf_len)
{
  // Z eraZe NV memory
  // Rf Read and print Field,
  //    where Field could be
  //         A boardID
  //         G adc gain
  //         M Macs
  //         O adc offset
  //         V version
  //         i current current
  //         n nap state
  // Wf Field,
  //    where Field could be B baud rate on S2
  //         A boardID
  //         G adc gain
  //         M[CO] Macs
  //         O adc offset
  //         n=b nap enable

  uint8_t read_pointer = 0;
  uint8_t  cmd;
  uint8_t  field;
  uint8_t  field2;
  int      value;
  float    fvalue;
  int match ;
  uint8_t  mvalue[6]; // mac address
  cmd = in_buf[read_pointer++];
  out_buf[0]=0;
  
  switch (cmd) {
  case 'Z':
    reinit_NVM();
    load_operational_params();
    break;

  case 'R':
    field = in_buf[read_pointer++];
    switch (field)
      {
      case 'A':
	snprintf(out_buf, out_buf_len, "board_id=%d\n", board_id);
	break;

      case 'G':
	snprintf(out_buf, out_buf_len, "adc_gain=%f\n", adcgain);
	break;

      case 'M':

	Serial.printf("IMON_MAC=%02x%02x%02x%02x%02x%02x  MCO=%02x%02x%02x%02x%02x%02x CMT=%02x%02x%02x%02x%02x%02x\n",
		      baseMac[0],baseMac[1],baseMac[2],baseMac[3],baseMac[4],baseMac[5],
		      mco_mac[0],mco_mac[1],mco_mac[2],mco_mac[3],mco_mac[4],mco_mac[5],
		      cmt_mac[0],cmt_mac[1],cmt_mac[2],cmt_mac[3],cmt_mac[4],cmt_mac[5]);
	break;

      case 'O':
	snprintf(out_buf, out_buf_len, "adc_offset=%f\n", adc0);
	break;

      case 'V':
	snprintf(out_buf, out_buf_len, "%s\n", version);
	break;

      case 'i':
	  snprintf(out_buf, out_buf_len, "current=%fA\n", get_current());
	break;
      case 'n' :
	  snprintf(out_buf, out_buf_len, "nap=%d\n", nap);
	  break ;	
      }
      break;
    

  case 'W':
    {
      // first case, WA=3     decimal integer up to ~5 sig figures
      match =  sscanf(in_buf, "%c%c=%d", &cmd, &field, &value);
      
      switch (field) {
      case 'A':
	imonPrefs.begin("imonPrefs", RW_MODE);         // Open our namespace for write
	imonPrefs.putUInt("boardid", value);               
	imonPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
	
      case 'G':
	match =  sscanf(in_buf, "%c%c=%f", &cmd, &field, &fvalue);
	imonPrefs.begin("imonPrefs", RW_MODE);         // Open our namespace for write
	imonPrefs.putFloat("adcgain", fvalue);               
	imonPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
	
      case 'M':  // WMP=MACADDRESS for the guy to respond to
	match =  sscanf(in_buf, "%c%c%c=%2x%2x%2x%2x%2x%2x", &cmd, &field,&field2,
			&mvalue[0],&mvalue[1],&mvalue[2],&mvalue[3],&mvalue[4],&mvalue[5]);
	
	imonPrefs.begin("imonPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'O') 
	  imonPrefs.putBytes("mcomac", mvalue, 6);
	else if (field2 == 'C') 
	  imonPrefs.putBytes("cmtmac", mvalue, 6);
	imonPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
	
      case 'O':
	match =  sscanf(in_buf, "%c%c=%f", &cmd, &field, &fvalue);
	imonPrefs.begin("imonPrefs", RW_MODE);         // Open our namespace for write
	imonPrefs.putFloat("adc0", fvalue);               
	imonPrefs.end();                              // Close the namespace
	load_operational_params();
	break;

      case 'n' :
	if (value == 0)
	  nap = false;
	else
	  nap = true;
	break;
      }      
      break;
      // end of 'W'
    }
  }
}

// S1 msg  gets parsed
 
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
      if (serial_byte == '\n')
	{
	  serial_buf[serial_buf_pointer] = (uint8_t) 0;  // write string terminator to buffer
	  if (serial_buf_pointer >= 1)                  // at least a command letter
	    {
	      parse_buf(serial_buf, return_buf, 127);  // parse the buffer if at least one char in it.
	      Serial.print(return_buf);
	    }
	  serial_buf_pointer = 0;
	}
    }
}
