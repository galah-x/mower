//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'mco  Time-stamp: "2025-03-13 20:41:37 john"';

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
#include<ADS1115_WE.h>   // adc converter
// fram
#include <stdio.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define DEBUG

// for preferences
#define RW_MODE false
#define RO_MODE true


// serial is used for debug and for programming NVM
const uint8_t longest_record = 24;  //   worst is display comand WD1=12345678901234567890
const uint8_t record_size = longest_record + 2;    //  char char = nnnnn \n
char serial_buf[record_size];
uint8_t serial_buf_pointer;

// serial2 is used for communicating with vmons
const uint8_t longest_record2 = 24;  //   worst is display comand WD1=12345678901234567890
const uint8_t record_size2 = longest_record2 + 2;    //  char char = nnnnn \n
char serial2_buf[record_size];
uint8_t serial2_buf_pointer;


uint8_t baseMac[6];         // my own mac address
const  uint16_t msgbuflen= 128;  // for wifi transfers
char return_buf[msgbuflen]; // for responses

const char * version = "MCO 13 Mar 2025 Reva";

Preferences mcoPrefs;  // NVM structure
// these will be initialized from the NV memory

int     fan_on_temp;
int     fan_off_temp;
bool    logging;
uint8_t psu_mac[6];
uint8_t mcc_mac[6];
uint8_t cmt_mac[6];

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

  parse_buf(response_data.message, return_buf, msgbuflen);
  Serial.print("parsed responded: ");
  // im initiating things. don't respond!
  Serial.printf("%s", return_buf);
  Serial.print(response_data.message);
  //** return return_buf to requestor here **
}

/* IOs  definitions */
const uint8_t buz_en_pin    = 23;    // buzzer
const uint8_t alrt_pin      = 34;      // alert output from adc
const uint8_t fram_wp_pin   = 25;
const uint8_t soc_pps_pin   = 27;
const uint8_t sda_pin       = 21; 
const uint8_t scl_pin       = 22; 
const uint8_t txd2_pin      = 17;
const uint8_t rxd2_pin      = 16;

// I2C addresses 
const uint8_t fram_addr = 0x50;
const uint8_t adc_addr =  0x48;

// Going to do all my basic event timing and logging in seconds, so will use ESP32Time for convenience.
// not going to bother setting the clock, just demark time in HMS from whenever the board last got
// turned on.
ESP32Time rtc;
unsigned long last_temp_update_time;         // time last temperature was polled at, in seconds
const unsigned long temp_update_period = 10; // seconds.

unsigned long last_psu_update_time;         // time last power supply voltage and current was polled at
const unsigned long psu_update_period = 2;  // seconds.

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

   // start adc current temperature sensor

   // init IO
   pinMode(power48V_en_pin, OUTPUT);
   pinMode(fan_en_pin, OUTPUT);
   pinMode(enc_an_pin, INPUT);
   pinMode(enc_bn_pin, INPUT);
   pinMode(enc_pushn_pin, INPUT);

   digitalWrite(fan_en_pin,  LOW);
   digitalWrite(power48V_en_pin, LOW);

    buz_en_pin    = 23;    // buzzer
 alrt_pin      = 34;      // alert output from adc
 uint8_t fram_wp_pin   = 25;
 uint8_t soc_pps_pin   = 27;

   
   // init some general variables and IOs 
   serial_buf_pointer = 0;
   serial2_buf_pointer = 0;
   last_current_update_time = rtc.getEpoch();


   // at startup, confirm all 4 vmons are present.
   // play a beep code if not so.
   
   
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
    // Register peer mco
    memcpy(peerInfo.peer_addr, mco_mac, 6);
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

}

   

void loop (void)
{
  char     logmsg[30]; // for building logging messages
  unsigned long t1;
  // service serial character if any available.
  do_serial_if_ready();
  do_serial2_if_ready();

  
  // update current and SOC state if its due 
  if (rtc.getEpoch() > (last_current_update_time + current_update_period))
    {
      hs_temperature=(int) tsense.readTemperature();
      snprintf(logmsg, 30, "Temp=%d", hs_temperature);
      
      t1 = millis();
      logger(logmsg);
      
      if (hs_temperature > fan_on_temp)
	{
	  if (digitalRead(fan_en_pin) == 0)
	    {
	      logger("Turning fan on");
	    }
	  digitalWrite(fan_en_pin, HIGH);
	}
      else
	if (hs_temperature < fan_off_temp)
	  {
	    if (digitalRead(fan_en_pin) != 0)
	      logger("Turning fan off");
	    digitalWrite(fan_en_pin, LOW);
	  }
      last_temp_update_time = rtc.getEpoch();
    }
}



  
const uint8_t default_mac[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

void reinit_NVM (void)
{
  Serial.println("Initializing NVM");
  
  // If tpInit is 'false', the key "nvsInit" does not yet exist therefore this
  //  must be our first-time run. We need to set up our Preferences namespace keys. So...
  mcoPrefs.begin("mcoPrefs", RW_MODE);       //  open it in RW mode.
  
  // The .begin() method created the "mcoPrefs" namespace and since this is our
  //  first-time run we will create
  //  our keys and store the initial "factory default" values.

  mcoPrefs.putFloat("psu_gain_cal", psu_gain_cal ); // cal term from local voltage to psu 

  mcoPrefs.putBytes("cmt_mac", default_mac, 6); // mac address of power supply 
  mcoPrefs.putBytes("psu_mac", default_mac, 6); // mac address of power supply 
  mcoPrefs.putBytes("mcc_mac", default_mac, 6); // mac address of power supply 

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

   fan_on_temp = mcoPrefs.getInt("fan_on_temp");
   fan_off_temp = mcoPrefs.getInt("fan_off_temp");
   logging = mcoPrefs.getBool("log"); 
   mcoPrefs.getBytes("psu_mac", psu_mac, 6);           // load power supply mac
   mcoPrefs.getBytes("mco_mac", mco_mac, 6);           // load mco/cmt supply mac

   // All done. Last run state (or the factory default) is now restored.
   mcoPrefs.end();                                      // Close our preferences namespace.
}

void parse_buf (char * in_buf, char * out_buf, int out_buf_len)
{
  // Z eraZe NV memory
  // Rf Read and print Field,
  //    where Field could be v voltage xx.xxx V fixed format
  //                         i current xx.xxx A fixed format
  //                         E power state enabled
  //                         H=templimit_fanon in integer degrees
  //                         L=fan temp off again low
  //                         l=log state
  //                         M print mac addresses
  //                         T=temperature
  //                         V=version
  // Wf Field,
  //    where Field could be v voltage Wv=xx.xxx    fixed format set power supply voltage
  //                         i current Wi=xx.xxx    fixed format set psu current
  //                         E enable  WE=1         turn on/off mower charge power
  //                         H=fan on  WH=45        fan on temperature in decomal degrees
  //                         L=fan off WL=23        fan off temperature in decomal degrees
  //                         l                      set log state
  //                         D=display WD0=string   write display line 0..3 with given string
  //                         Mx mac    WMP=<12 ascii hex digits>  mac for Psu or Mco
  //                        
  uint8_t read_pointer = 0;
  uint8_t  cmd;
  uint8_t  field;
  uint8_t  field2;
  int      value;
  uint8_t  mvalue[6]; // mac address
  uint8_t  msg[21];   // 20 chars and 0 terminator
  char     logmsg[30]; // for building logging messages
  int match ;
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
	// FIXME support v,i read voltage and current settings from PSU and report.
      case 'E':
	snprintf(out_buf, out_buf_len, "Mower_power=%d\n", digitalRead(power48V_en_pin));
	break;

      case 'H':
	snprintf(out_buf, out_buf_len, "Fan_on_at=%d degrees \n", fan_on_temp);
	break;
	
      case 'L':
	snprintf(out_buf, out_buf_len, "Fan_off_at=%d degrees\n", fan_off_temp);
	break;

      case 'l':
	snprintf(out_buf, out_buf_len, "log=%d\n", logging);
	break;
	
      case 'M':
	snprintf(out_buf, out_buf_len, "MCO_MAC=%02x%02x%02x%02x%02x%02x PSU=%02x%02x%02x%02x%02x%02x MCO=%02x%02x%02x%02x%02x%02x\n",
		 baseMac[0],baseMac[1],baseMac[2],baseMac[3],baseMac[4],baseMac[5],
		 psu_mac[0],psu_mac[1],psu_mac[2],psu_mac[3],psu_mac[4],psu_mac[5],
		 mco_mac[0],mco_mac[1],mco_mac[2],mco_mac[3],mco_mac[4],mco_mac[5]
		 );
	break;

      case 'T':
	snprintf(out_buf, out_buf_len, "Temp=%d degrees\n", hs_temperature);
	break;

      case 'V':
	snprintf(out_buf, out_buf_len, "%s\n", version);
	break;
      }
      break;
    

  case 'W':
    // first case, WA=3     decimal integer up to ~5 sig figures
    match =  sscanf(in_buf, "%c%c=%d", &cmd, &field, &value);

    switch (field) {
      // FIXME add v, i
    case 'D':
      // sscanf(in_buf, "%1c%10s", &cmd, &out_buf);  // whatever followed the 'm'
      // nope, I cannot make sscanf work for a char then a 0 terminated string. So do a manual scan
      field2 = (in_buf[2] - (uint8_t) 1) & 0x3; // ascii display row to write to [31..34] +=> 0123 after anding with 3
      //   jump over WD0=12340            
      for (read_pointer=4; read_pointer < 24; read_pointer++)
	{
	  out_buf[read_pointer-4] = in_buf[read_pointer];
	  if (in_buf[read_pointer] == 0)
	    { out_buf[read_pointer-4]= ' ';
	      break;
	    }
	}
      snprintf(logmsg, 30, "B%c %s", in_buf[2], out_buf);
      logger(logmsg);
      if (! UI_owns_display)
	{
	  lcd.setCursor(0,field2);
	  lcd.print(out_buf);
	}
      out_buf[0]=0;
      break;
      
    case 'E':
      digitalWrite(power48V_en_pin, value);
      break;

    case 'H':
      mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
      mcoPrefs.putInt("fan_on_temp", value);               
      mcoPrefs.end();                              // Close the namespace
      load_operational_params();
      break;
      
    case 'L':
      mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
      mcoPrefs.putInt("fan_off_temp", value);               
      mcoPrefs.end();                              // Close the namespace
      load_operational_params();
      break;
      
    case 'l':
      mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
      mcoPrefs.putBool("log", value);               
      mcoPrefs.end();                              // Close the namespace
      load_operational_params();
      break;
      
    case 'M':  // WMP=MACADDRESS for the guy to respond to
      match =  sscanf(in_buf, "%c%c%c=%2x%2x%2x%2x%2x%2x", &cmd, &field,&field2,
		      &mvalue[0],&mvalue[1],&mvalue[2],&mvalue[3],&mvalue[4],&mvalue[5]);
      
      mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
      if (field2 == 'P') 
	mcoPrefs.putBytes("psu_mac", mvalue, 6);
      if (field2 == 'M') 
	mcoPrefs.putBytes("mco_mac", mvalue, 6);
      mcoPrefs.end();                              // Close the namespace
      load_operational_params();
      break;
    }      
    break;
    // end of 'W'
  }    
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
	  parse_buf(serial_buf, return_buf, 127);      // parse the buffer if at least one char in it.
	  Serial.print(return_buf);
	}
	serial_buf_pointer = 0;
      }
    }
}


