//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'mcc  Time-stamp: "2025-03-09 13:49:24 john"';

// this is the app to run the mower charger interface for the Ryobi mower.
// use tools -> board ->  ESP32 Dev module 

/* mcc can talk to the power supply to adjust current and voltage settings.
   It can disconnect the power supply if it needs to.
   It has a display, 4 x 20 char LCD I2C connected . Scanned at address 0x27
   It has a rotary encoder with push knob for UI control
   It has a temp sensor fot the switch heatsink, and can turn on a 12V fan if needed.
   Its got a SD card to log to
   It gets instructed by the MCO in the mower as to what to do.
   The model where I can control it either serially from the esp32 debug terminal, or using
   the same commands across esp_now from the MCO seems to work ok for test + development. */


// test cmt MAC is 5c013b6c9938
// test mco MAC is      which replaces the cmt later in the dev process Mower COntroller
// test psu MAC is       wifi to serial adapter for the power supply. Originally a vichy mm adaptor
// test mcc MAC (ME) is 5c013b6cf4fc   MowerChargeController

#include <Preferences.h>  // the NV memory interface
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "ESP32Time.h"
#include <stdio.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define DEBUG

// for preferences
#define RW_MODE false
#define RO_MODE true


// serial is used for debug and for programming NVM
const uint8_t longest_record = 20;  //  char char = nnnnn 0, worst is write MAC
const uint8_t record_size = longest_record + 2;    //  char char = nnnnn \n
char serial_buf[record_size];
uint8_t serial_buf_pointer;
int serial_byte;

uint8_t baseMac[6];
const  uint16_t msgbuflen= 128;
char return_buf[msgbuflen]; // for responses

/* IOs */


const char * version = "MCC test app 9 Mar 2025 Revb";

Preferences mccPrefs;  // NVM structure
// these will be initialized from the NV memory

int     fan_on_temp;
int     fan_off_temp;
uint8_t psu_mac[6];
uint8_t mco_mac[6];


typedef struct struct_message {
  char message[msgbuflen];
} struct_message;

// create the message struct
struct_message message_data;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  //  Serial.print("\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

struct_message response_data;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&response_data, incomingData, sizeof(response_data));
  Serial.print("received ");
  Serial.print(len);
  Serial.print(" bytes from ");  
  Serial.printf("%02x%02x%02x%02x%02x%02x\n", mac[12], mac[13], mac[14], mac[15], mac[16], mac[17]) ;
  /* I suspect the mac' field here is the 24 byte mac header structure espressif uses
     fields 12 to 17 seem to be the source MAC. rest isn't obvious */

  parse_buf(response_data.message, return_buf, msgbuflen);
  Serial.print("parsed responded: ");
  Serial.printf("%s", return_buf);
  Serial.print(response_data.message);
  //** return return_buf to requestor here **

  strncpy(response_data.message, return_buf, msgbuflen);
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(mco_mac, (uint8_t *) &response_data, sizeof(response_data));
  
  if (result == ESP_OK) {
    // Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

/* IOs */
const uint8_t enc_an_in_pin  = 34;    // rotary encoder a
const uint8_t enc_bn_in_pin  = 35;    // rotary encoder b
const uint8_t enc_push_n_pin = 33;    // rotary encoder pushbutton
const uint8_t spi_sclk_pin = 14;
const uint8_t spi_cs_pin   = 15;  // sdcard
const uint8_t spi_mosi_pin = 13;
const uint8_t spi_miso_pin = 12;  
const uint8_t power48V_en_pin = 25;    // turn on 48V to mower
const uint8_t fan_en_pin = 27;    // turn on fan 
const uint8_t sda_pin   = 21;    // fan3
const uint8_t scl_pin   = 22;    // fan3

const uint8_t hs_temperature_addr = 0x76;
const uint8_t display_addr = 0x27;

Adafruit_BMP280 tsense;    // I2C


unsigned long millisecs_temp;
const unsigned long next_temp_delay = 1000;  // milliseconds.

int hs_temperature;

void setup (void) {
  Serial.begin(115200);
  Serial.println(version);
  // initialize NVM  
   mccPrefs.begin("mccPrefs", RO_MODE);         // Open our namespace (or create it
                                                //  if it doesn't exist) in RO mode.
   bool tpInit = mccPrefs.isKey("nvsInit");     // Test for the existence
                                                // of the "already initialized" key.
   mccPrefs.end();                              // close the namespace in RO mode
   if (tpInit == false) {
     reinit_NVM();
   }
   // load local variables from NVM
   load_operational_params();                     // load all systemwide constants from NVM

   // start temperature sensor
   if (!tsense.begin(hs_temperature_addr))
     {  
       Serial.print("Could not find a valid BMP280 sensor at address 0x");  
       Serial.println(hs_temperature_addr, HEX);  
     }
   hs_temperature = (int)  tsense.readTemperature();
   Serial.print("Temp=");
   Serial.println(hs_temperature);

   // Carry on with the rest of your setup code...
   serial_buf_pointer = 0;
   millisecs_temp = millis();

   pinMode(power48V_en_pin, OUTPUT);
   pinMode(fan_en_pin, OUTPUT);
   pinMode(enc_an_in_pin, INPUT);
   pinMode(enc_bn_in_pin, INPUT);
   pinMode(enc_push_n_pin, INPUT);

   digitalWrite(fan_en_pin,  LOW);
   digitalWrite(power48V_en_pin, LOW);

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

    Serial.println("register mco mac");
    // Register peer
    memcpy(peerInfo.peer_addr, mco_mac, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    Serial.println("add peer");
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    Serial.println("register psu mac");
    // Register peer
    memcpy(peerInfo.peer_addr, psu_mac, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    Serial.println("done setup");
}

   

void loop (void) {
  // service serial character if available.
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
	if (serial_buf_pointer >= 1) {  // at least a command letter
	  parse_buf(serial_buf, return_buf, 127);           // parse the buffer if at least one char in it.
	  Serial.print(return_buf);
	}
	serial_buf_pointer = 0;
      }
    }

  // update each temperature 
  if (millis() > (millisecs_temp + next_temp_delay))
    {
      hs_temperature=(int) tsense.readTemperature();
      if (hs_temperature > fan_on_temp)
	digitalWrite(fan_en_pin, HIGH);
      else
	if (hs_temperature < fan_off_temp)
	  digitalWrite(fan_en_pin, HIGH);
      millisecs_temp= millis();
    }
}



const uint8_t default_mac[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

void reinit_NVM (void)
{
  Serial.println("Initializing NVM");
  
  // If tpInit is 'false', the key "nvsInit" does not yet exist therefore this
  //  must be our first-time run. We need to set up our Preferences namespace keys. So...
  mccPrefs.begin("mccPrefs", RW_MODE);       //  open it in RW mode.
  
  // The .begin() method created the "mccPrefs" namespace and since this is our
  //  first-time run we will create
  //  our keys and store the initial "factory default" values.

  mccPrefs.putInt("fan_on_temp",  45);        // start fan at this temp if auto_fan
  mccPrefs.putInt("fan_off_temp", 35);       // stop fan at this temp if auto_fan
  mccPrefs.putBytes("psu_mac", default_mac, 6);  // mac address of power supply 
  mccPrefs.putBytes("psu_mac", default_mac, 6);  // mac address of power supply 

  mccPrefs.putBool("nvsInit", true);            // Create the "already initialized"
  //  key and store a value.
  // The "factory defaults" are created and stored so...
  mccPrefs.end();                              // Close the namespace in RW mode and...
}


void load_operational_params(void)
{

  mccPrefs.begin("mccPrefs", RO_MODE);       // Open our namespace (or create it
                                               //  if it doesn't exist) in RO mode.
   // Retrieve the operational parameters from the namespace
   //  and save them into their run-time variables.

   fan_on_temp = mccPrefs.getInt("fan_on_temp");
   fan_off_temp = mccPrefs.getInt("fan_off_temp");
   mccPrefs.getBytes("psu_mac", psu_mac, 6);           // load power supply mac
   mccPrefs.getBytes("mco_mac", mco_mac, 6);           // load mco/cmt supply mac

   // All done. Last run state (or the factory default) is now restored.
   mccPrefs.end();                                      // Close our preferences namespace.
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
  //                         M print mac addresses
  //                         T=temperature
  //                         V=version
  // Wf Field,
  //    where Field could be v voltage Wv=xx.xxx    fixed format set power supply voltage
  //                         i current Wi=xx.xxx    fixed format set psu current
  //                         E enable  WE=1         turn on/off mower charge power
  //                         H=fan on  WH=45        fan on temperature in decomal degrees
  //                         L=fan off WL=23        fan off temperature in decomal degrees
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
	
      case 'M':
	snprintf(out_buf, out_buf_len, "MCC_MAC=%02x%02x%02x%02x%02x%02x PSU=%02x%02x%02x%02x%02x%02x MCO=%02x%02x%02x%02x%02x%02x\n",
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
    int match =  sscanf(in_buf, "%c%c=%d", &cmd, &field, &value);
    
    switch (field) {
      // FIXME add v, i, D
    case 'E':
      digitalWrite(power48V_en_pin, value);
      break;

    case 'H':
      mccPrefs.begin("mccPrefs", RW_MODE);         // Open our namespace for write
      mccPrefs.putInt("fan_on_temp", value);               
      mccPrefs.end();                              // Close the namespace
      load_operational_params();
      break;
      
    case 'L':
      mccPrefs.begin("mccPrefs", RW_MODE);         // Open our namespace for write
      mccPrefs.putInt("fan_off_temp", value);               
      mccPrefs.end();                              // Close the namespace
      load_operational_params();
      break;
      
    case 'M':  // WMP=MACADDRESS for the guy to respond to
      match =  sscanf(in_buf, "%c%c%c=%2x%2x%2x%2x%2x%2x", &cmd, &field,&field2,
		      &mvalue[0],&mvalue[1],&mvalue[2],&mvalue[3],&mvalue[4],&mvalue[5]);
      
      mccPrefs.begin("mccPrefs", RW_MODE);         // Open our namespace for write
      if (field2 == 'P') 
	mccPrefs.putBytes("psu_mac", mvalue, 6);
      if (field2 == 'M') 
	mccPrefs.putBytes("mco_mac", mvalue, 6);
      mccPrefs.end();                              // Close the namespace
      load_operational_params();
      break;
    }      
    break;
    // end of 'W'
  }    
}


