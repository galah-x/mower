//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'psu  Time-stamp: "2025-03-14 18:24:27 john"';

// this is the app to run the espnow2serial (vichyctl) on the DPM8624 power supply the mcc talks to.
// use tools -> board ->  ESP32 Dev module 

/* mcc can talk to the power supply to adjust current and voltage via this board. This board just
   does espnow to serial with no protocol addons. */

// test cmt MAC is 5c013b6c9938
// test mcc MAC is 5c013b6cf4fc      wifi to serial adapter for the power supply. Originally a vichy mm adaptor
// test psu MAC (ME) is 5c013b6ce2d0

#include <Preferences.h>  // the NV memory interface
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

// serial2 is used for talkng to the DPM8624
const uint8_t longest_record2 = 24;  //   worst is display comand WD1=12345678901234567890
const uint8_t record_size2 = longest_record + 2;    //  char char = nnnnn \n
char serial2_buf[record_size];
uint8_t serial2_buf_pointer;




uint8_t baseMac[6];         // my own mac address
const  uint16_t msgbuflen= 128;  // for wifi transfers
char return_buf[msgbuflen]; // for responses

const char * version = "PSU 12 Mar 2025 Revd";

Preferences psuPrefs;  // NVM structure
// these will be initialized from the NV memory

// variables filled from NVM
uint8_t cmt_mac[6];
uint8_t mcc_mac[6];
uint8_t in_mac[6];  // incoming message source MAC
int s2baud;

typedef struct struct_message {
  char message[msgbuflen];
} struct_message;

// create the wifi message struct
struct_message outgoing_msg;

esp_now_peer_info_t peerInfo;

// function prototype
void parse_buf(char * in_buf, char * out_buf, int out_buf_len);

// wifi callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}


struct_message incoming_msg;

int msglen;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming_msg, incomingData, sizeof(incoming_msg));
  //  Serial.print("received ");
  //Serial.print(len);
  //Serial.print(" bytes from ");  
  //  Serial.printf("%02x%02x%02x%02x%02x%02x\n", mac[12], mac[13], mac[14], mac[15], mac[16], mac[17]) ;
  /* I suspect the mac' field here is the 24 byte mac header structure espressif uses
     fields 12 to 17 seem to be the source MAC. rest isn't obvious */
  
  //remember the mac address whom the most recent wifi message is from 
  int i;
  esp_err_t result;
  for (i=0; i<6; i=i+1)
    in_mac[i] = mac[12+i];
  
  Serial.printf("%s\n", incoming_msg.message);   // incoming wifi msg goes to S1 debug terminal
  if (incoming_msg.message[0] == ':')           // incoming wifi message starting with : goes to S2
    {
      msglen = strlen(incoming_msg.message);
      Serial2.printf("%s\r\n", incoming_msg.message);   // incoming wifi msg starting with : goes to S2
    }
  else
    {
      parse_buf(incoming_msg.message, return_buf, msgbuflen);
      Serial.printf("%s\n", return_buf);
      //** return return_buf to requestor here **
      strncpy(outgoing_msg.message, return_buf, msgbuflen);
      result = ESP_OK;
      // Send message via ESP-NOW
      if ((in_mac[0] == mcc_mac[0]) && (in_mac[1] == mcc_mac[1]) && (in_mac[2] == mcc_mac[2]) &&
	  (in_mac[3] == mcc_mac[3]) && (in_mac[4] == mcc_mac[4]) && (in_mac[5] == mcc_mac[5]))
	result = esp_now_send(mcc_mac, (uint8_t *) &outgoing_msg, msgbuflen);
      else if ((in_mac[0] == cmt_mac[0]) && (in_mac[1] == cmt_mac[1]) && (in_mac[2] == cmt_mac[2]) &&
	       (in_mac[3] == cmt_mac[3]) && (in_mac[4] == cmt_mac[4]) && (in_mac[5] == cmt_mac[5]))
	result = esp_now_send(cmt_mac, (uint8_t *) &outgoing_msg, msgbuflen);
      
      if (result == ESP_OK) {
	// Serial.println("Sent with success");
      }
      else {
	Serial.println("Error sending the data");
      }
    }
}

void setup (void) {
  Serial.begin(115200);
  Serial.println(version);

  // check NVM looks defined  
   psuPrefs.begin("psuPrefs", RO_MODE);     // Open our namespace (or create it if it doesn't exist)
   bool tpInit = psuPrefs.isKey("nvsInit"); // Test for the existence of the "already initialized" key.
   psuPrefs.end();                          // close the namespace in RO mode
   if (tpInit == false) 
     reinit_NVM();                          // reinitialize the nvm structure if magick key was missing 
   
   // load local variables from NVM
   load_operational_params();                // load all variables from NVM

   // init some general variables and IOs 
   serial_buf_pointer = 0;
   serial2_buf_pointer = 0;

   // interestingly, while rx=16 txd=17 is supposedly the default pin allocation,  Serial2 doesn't
   // work without explicitly filling in the pin numbers here
   Serial2.begin(s2baud, SERIAL_8N1, 16, 17);  

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
   // get the status of Transmitted packets
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
    // Register peer cmt
    memcpy(peerInfo.peer_addr, cmt_mac, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    Serial.println("done setup");
}

   

void loop (void)
{
  // service serial character if any available.
  do_serial_if_ready();
  do_serial2_if_ready();

  // a message in from s1 starting with : goes to S2     - erasing sender mac
  // a message in from s1 NOT starting with : is parsed
  // a message in from wifi starting with : goes to S2   - noting sender MAC , and to S1
  // a message in from wifi NOT starting with : is parsed , and to S1. parse results go back to sender mac
  // a message in from s2 goes to S1, and wifi if last received mac is valid
}



  
const uint8_t default_mac[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

void reinit_NVM (void)
{
  Serial.println("Initializing NVM");
  
  // If tpInit is 'false', the key "nvsInit" does not yet exist therefore this
  //  must be our first-time run. We need to set up our Preferences namespace keys. So...
  psuPrefs.begin("psuPrefs", RW_MODE);       //  open it in RW mode.
  
  // The .begin() method created the "mccPrefs" namespace and since this is our
  //  first-time run we will create
  //  our keys and store the initial "factory default" values.

  psuPrefs.putInt("s2baud", 9600);          // baud rate serial 2 DPM8624
  psuPrefs.putBytes("mcc_mac", default_mac, 6); 
  psuPrefs.putBytes("cmt_mac", default_mac, 6); 

  psuPrefs.putBool("nvsInit", true);            // Create the "already initialized"
  //  key and store a value.
  // The "factory defaults" are created and stored so...
  psuPrefs.end();                               // Close the namespace in RW mode and...
}


void load_operational_params(void)
{

  psuPrefs.begin("psuPrefs", RO_MODE);       // Open our namespace (or create it
                                             //  if it doesn't exist) in RO mode.
   // Retrieve the operational parameters from the namespace
   //  and save them into their run-time variables.

   s2baud = psuPrefs.getInt("s2baud");
   psuPrefs.getBytes("mcc_mac", mcc_mac, 6);           
   psuPrefs.getBytes("cmt_mac", cmt_mac, 6);           

   // All done. Last run state (or the factory default) is now restored.
   psuPrefs.end();                                      // Close our preferences namespace.
}

void parse_buf (char * in_buf, char * out_buf, int out_buf_len)
{
  // Z eraZe NV memory
  // Rf Read and print Field,
  //    where Field could be B baud rate on S2
  //                         M print mac addresses
  //                         V=version
  // Wf Field,
  //    where Field could be B baud rate on S2
  //                         Mx mac    WMP=<12 ascii hex digits>  mac for mcc or cmt
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
      case 'B':
	snprintf(out_buf, out_buf_len, "s2baud=%d\n", s2baud);
	break;

      case 'M':
	snprintf(out_buf, out_buf_len, "PSU_MAC=%02x%02x%02x%02x%02x%02x MCC=%02x%02x%02x%02x%02x%02x CMT=%02x%02x%02x%02x%02x%02x\n",
		 baseMac[0],baseMac[1],baseMac[2],baseMac[3],baseMac[4],baseMac[5],
		 mcc_mac[0],mcc_mac[1],mcc_mac[2],mcc_mac[3],mcc_mac[4],mcc_mac[5],
		 cmt_mac[0],cmt_mac[1],cmt_mac[2],cmt_mac[3],cmt_mac[4],cmt_mac[5]
		 );
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

    case 'B':
      psuPrefs.begin("psuPrefs", RW_MODE);         // Open our namespace for write
      psuPrefs.putInt("s2baud", value);               
      psuPrefs.end();                              // Close the namespace
      load_operational_params();
      break;
      
    case 'M':  // WMP=MACADDRESS for the guy to respond to
      match =  sscanf(in_buf, "%c%c%c=%2x%2x%2x%2x%2x%2x", &cmd, &field,&field2,
		      &mvalue[0],&mvalue[1],&mvalue[2],&mvalue[3],&mvalue[4],&mvalue[5]);
      
      psuPrefs.begin("psuPrefs", RW_MODE);         // Open our namespace for write
      if (field2 == 'M') 
	psuPrefs.putBytes("mcc_mac", mvalue, 6);
      if (field2 == 'C') 
	psuPrefs.putBytes("cmt_mac", mvalue, 6);
      psuPrefs.end();                              // Close the namespace
      load_operational_params();
      break;
    }      
    break;
    // end of 'W'
  }    
}

// S1 msg starting with : goes to S2 and in_mac gets erased
// S1 msg NOT starting with : gets parsed
 
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
	  if (serial_buf_pointer >= 1) {                 // at least a command letter
	    if (serial_buf[0] == ':')                    // starts with : so send to s1. 
	      {
		int i;
		for (i = 0; i<6; i++) // erase the from-mac address, so I don't send the S2 response 
		  in_mac[i]=0;        // to the prior wifi client for this console command
		Serial2.printf("%s\r\n", serial_buf);
	      }
	    else                                         // doesn't start with : so parse.
	      {
		parse_buf(serial_buf, return_buf, 127);  // parse the buffer if at least one char in it.
		Serial.print(return_buf);
	      }
	  }
	serial_buf_pointer = 0;
      }
    }
}
// S2 msg goes to both S1 and wifi. 
 
void do_serial2_if_ready (void)
{
  int serial2_byte;
  if (Serial2.available())
    {
      serial2_byte = Serial2.read();
      // note: neither \r nor \n is written to buffer. a zero is added to the buffer on \n
      if ((serial2_byte != '\n') && (serial2_byte != '\r') && (serial2_buf_pointer < longest_record2))
	{
	  serial2_buf[serial2_buf_pointer] = serial2_byte;  // write char to buffer if space is available
	  serial2_buf_pointer++;
	}    
      if (serial2_byte == '\n') {
	serial2_buf[serial2_buf_pointer] = (uint8_t) 0;     // write string terminator to buffer
	if (serial2_buf_pointer >= 1) {                      // at least a command letter
	  {
	    Serial.printf("%s\n", serial2_buf); // always send s2 output to s0 debug console
	    strncpy(outgoing_msg.message, serial2_buf, msgbuflen);
	      esp_err_t result = ESP_OK;
	    // return the message via ESP-NOW to whomever sent the prev command
	    if ((in_mac[0] == mcc_mac[0]) && (in_mac[1] == mcc_mac[1]) && (in_mac[2] == mcc_mac[2]) &&
		(in_mac[3] == mcc_mac[3]) && (in_mac[4] == mcc_mac[4]) && (in_mac[5] == mcc_mac[5]))
	      result = esp_now_send(mcc_mac, (uint8_t *) &outgoing_msg, msgbuflen);
	    else if ((in_mac[0] == cmt_mac[0]) && (in_mac[1] == cmt_mac[1]) && (in_mac[2] == cmt_mac[2]) &&
		     (in_mac[3] == cmt_mac[3]) && (in_mac[4] == cmt_mac[4]) && (in_mac[5] == cmt_mac[5]))
	      result = esp_now_send(cmt_mac, (uint8_t *) &outgoing_msg, msgbuflen);
	    
	    if (result == ESP_OK) {
	      // Serial.println("Sent with success");
	    }
	    else {
	      Serial.println("Error sending the data");
	    }
	  }
	}
	serial2_buf_pointer = 0;
      }
    }
}

