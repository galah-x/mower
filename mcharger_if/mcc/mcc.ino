//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'mcc  Time-stamp: "2025-03-10 20:55:53 john"';

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
// NB, logging now writes in 4K chunks to minimize RmodifyW delays as part of locating logging
//     crazy 10s delays. Which were predominately a broken ESP32Time when Time wasn't set to > year 1980.   
//     Should help CDcard wear levelling no end also. Downside is you need to use the UI to turn
//     off logging (which flushes then closes the file) prior to power down if you want
//     the last ~~10 minutes of logs.
//     The 4k write seems to take 40ms. hopefully thats not generally noticeable.

// test cmt MAC is 5c013b6c9938
// test mco MAC is      which replaces the cmt later in the dev process Mower COntroller
// test psu MAC is       wifi to serial adapter for the power supply. Originally a vichy mm adaptor
// test mcc MAC (ME) is 5c013b6cf4fc   MowerChargeController

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
//#include "SdFat.h"             // Include library for SD card. required for cow list.
// SdFile file;                   // Create SdFile object used for accessing files on SD card

#define DEBUG

// for preferences
#define RW_MODE false
#define RO_MODE true


// serial is used for debug and for programming NVM
const uint8_t longest_record = 24;  //   worst is display comand WD1=12345678901234567890
const uint8_t record_size = longest_record + 2;    //  char char = nnnnn \n
char serial_buf[record_size];
uint8_t serial_buf_pointer;

const uint16_t sdblksize = 4096; // blksize is always 512, cluster size usually 4k. 
uint8_t sdbuf0[sdblksize];
uint8_t sdbuf1[sdblksize];
uint8_t which_sdbuf;
uint16_t sdbuf_ptr;

uint8_t baseMac[6];         // my own mac address
const  uint16_t msgbuflen= 128;
char return_buf[msgbuflen]; // for responses

const char * version = "MCC 9 Mar 2025 Revk";

Preferences mccPrefs;  // NVM structure
// these will be initialized from the NV memory

int     fan_on_temp;
int     fan_off_temp;
bool    logging;
uint8_t psu_mac[6];
uint8_t mco_mac[6];


bool UI_owns_display;

typedef struct struct_message {
  char message[msgbuflen];
} struct_message;

// create the wifi message struct
struct_message message_data;

esp_now_peer_info_t peerInfo;

// wifi callback when data is sent
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
// This will be done in setup()
RotaryEncoder *encoder = nullptr;
int     enc_pos;
uint8_t enc_pushn_state;
int     enc_change = 0;
uint8_t enc_press = 0;
int     enc_press_debounce = 50; //msecs
bool    log_file_open=0;
bool    show_menu;
int     menu;
int     menu_line;
const uint8_t cursor = '>';
int     hs_temperature;
const char * logfile = "/log.txt";

File file;

// this is the encoder ISR. Not certain I actually want to use encoder interrupts.
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

unsigned long last_log_update_time;         // time logfile was last flushed, in seconds
const unsigned long log_update_period = 60; // seconds.

// create lcd object
hd44780_I2Cexp lcd;

void setup (void) {
  Serial.begin(115200);
  Serial.println(version);

  // initialize NVM  
   mccPrefs.begin("mccPrefs", RO_MODE);     // Open our namespace (or create it if it doesn't exist)
   bool tpInit = mccPrefs.isKey("nvsInit"); // Test for the existence of the "already initialized" key.
   mccPrefs.end();                          // close the namespace in RO mode
   if (tpInit == false) 
     reinit_NVM();                          // reinitialize the nvm structure if magik key was missing 
   
   // load local variables from NVM
   load_operational_params();                // load all variables from NVM

   // start temperature sensor
   if (!tsense.begin(hs_temperature_addr))
     {  
       Serial.print("Could not find a valid BMP280 sensor at address 0x");  
       Serial.println(hs_temperature_addr, HEX);  
     }
   hs_temperature = (int)  tsense.readTemperature();
   Serial.print("Temp=");
   Serial.println(hs_temperature);

   // init some general variables and IOs 
   serial_buf_pointer = 0;
   last_temp_update_time = rtc.getEpoch();

   pinMode(power48V_en_pin, OUTPUT);
   pinMode(fan_en_pin, OUTPUT);
   pinMode(enc_an_pin, INPUT);
   pinMode(enc_bn_pin, INPUT);
   pinMode(enc_pushn_pin, INPUT);

   digitalWrite(fan_en_pin,  LOW);
   digitalWrite(power48V_en_pin, LOW);

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

    // initialize the display
    lcd.begin(20,4);
    lcd.setCursor(0,0);
    lcd.print(version);

    // initialize the rotary encoder
    encoder = new RotaryEncoder(enc_an_pin, enc_bn_pin, RotaryEncoder::LatchMode::FOUR3);
    /* register interrupt routine... hmmm, do I _really_ need interrupts?
     * Interrupts can have tricky side effects if not done properly. I don't know properly. 
     * Happy to just poll this in loop() until it becomes obvious that I miss too many transitions.
     * The way the sample code uses both ints AND loop() polling makes me suspect the relevant 
     * library author(s) might not know interrupts properly either. */
    // attachInterrupt(digitalPinToInterrupt(enc_an_pin), checkPosition, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(enc_bn_pin), checkPosition, CHANGE); 

    enc_press = 0;
    enc_change = 0;
    enc_pos = encoder->getPosition();
    UI_owns_display = false;

    // sdcard
    SPI.begin(spi_sclk_pin, spi_miso_pin, spi_mosi_pin, spi_cs_pin);
    if (!SD.begin(spi_cs_pin)) {
       Serial.println("Card Mount Failed");
    }
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE) {
      Serial.println("No SD card attached");
    }
    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) {
      Serial.println("MMC");
    } else if (cardType == CARD_SD) {
      Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
      Serial.println("SDHC");
    } else {
      Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    
    listDir(SD, "/", 0);
    Serial.println("done setup");
    sdbuf_ptr=0;
    which_sdbuf = 0;
    
    if (logging)
      open_logfile();
}

   

void loop (void)
{
  char     logmsg[30]; // for building logging messages
  unsigned long t1;
  // service serial character if available.
  do_serial_if_ready();

  // service encoder and process the UI if required
  do_encoder_and_UI();
  
  // update temperature and fan state if its due 
  if (rtc.getEpoch() > (last_temp_update_time + temp_update_period))
    {
      hs_temperature=(int) tsense.readTemperature();
      snprintf(logmsg, 30, "Temp=%d", hs_temperature);
      
      t1 = millis();
      logger(logmsg);
      Serial.print("temp log took ");
      Serial.print(millis() - t1);
      Serial.print(" which_sdbuf=");
      Serial.print(which_sdbuf);
      Serial.print(" sdbuf_ptr=");
      Serial.println(sdbuf_ptr);
      
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



void do_encoder_and_UI (void)
{
  // look for encoder tick
  encoder->tick(); // just call tick() to check the state.
  
  int newPos = encoder->getPosition();
  
  if (enc_pos != newPos) {
    if  ( (int) encoder->getDirection() < 0)
      {
	enc_change = -1;
	//	Serial.println("enc changed -1");
      }
    else
      {
	enc_change = 1;
	// Serial.println("enc changed 1");
      }    
    enc_pos = newPos;
  } else {
    enc_change = 0;
  }
  
  int newenc_pushn_state = digitalRead(enc_pushn_pin);
  
  if (enc_pushn_state  != newenc_pushn_state )
    delay(enc_press_debounce);
  if ((enc_pushn_state == 1) && (newenc_pushn_state == 0))
    {
      // Serial.println("enc pressed");
      enc_press  = 1;
    }
  else
    enc_press = 0;
  enc_pushn_state = newenc_pushn_state;
  
  if (enc_press && !UI_owns_display)
    {
      UI_owns_display = true;
      menu=0;
      menu_line=0;
      show_menu=true;
      enc_press = 0;
    }
  if (UI_owns_display)
    {
      //Serial.print("menu ");
      //Serial.print(menu);
      //Serial.print(" line");
      //Serial.println(menu_line);
      do_menu();
    }
}


void do_menu (void )
{
  if (menu == 0)
    do_menu_0();
  else if (menu == 1)
    do_menu_1();
}

void do_menu_0 (void)
{
  if (show_menu)
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(" go back");
      lcd.setCursor(0,1);
      lcd.print(" log menu");
      lcd.setCursor(0,menu_line);
      lcd.write(cursor);
      show_menu=false;
    }
  if (enc_press == true)
    {
      if (menu_line == 0)
	{
	  lcd.clear();
	  UI_owns_display=false;
	}
      if (menu_line == 1)
	{
	  menu=1;
	  menu_line = 0;
	  show_menu=1;
	}
    }
  if (enc_change == 1)
    if (menu_line <= 2)
      {
	lcd.setCursor(0,menu_line);
	lcd.print(' ');
	menu_line++;
	lcd.setCursor(0,menu_line);
	lcd.write(cursor);
      }
  if (enc_change == -1)
    if (menu_line >= 1)
      {
	lcd.setCursor(0,menu_line);
	lcd.print(' ');
	menu_line--;
	lcd.setCursor(0,menu_line);
	lcd.write(cursor);
      }
}
void do_menu_1 (void)  // log menu
{
  if (show_menu)
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(" go back");
      lcd.setCursor(0,1);
      if (logging==1)
	lcd.print(" turn off logging");
      else 
	lcd.print(" turn on logging");
      lcd.setCursor(0,menu_line);
      lcd.write(cursor);
      show_menu=false;
    }
  if (enc_press == true)
    {
      if (menu_line == 0)
	{
	  menu = 0;
	  menu_line = 0;
	  show_menu=1;
	}
      if (menu_line == 1)
	{
	  if (logging==1)
	    {
	      // turn off logging
	      logging=0;
	      show_menu=1;
	      close_logfile();
	    }
	  else
	    {
	      // turn on logging
	      logging=1;
	      show_menu=1;
	      open_logfile();
	    }
	}
    }
  if (enc_change == 1)
    if (menu_line <= 2)
      {
	lcd.setCursor(0,menu_line);
	lcd.print(' ');
	menu_line++;
	lcd.setCursor(0,menu_line);
	lcd.write(cursor);
      }
  if (enc_change == -1)
    if (menu_line >= 1)
      {
	lcd.setCursor(0,menu_line);
	lcd.print(' ');
	menu_line--;
	lcd.setCursor(0,menu_line);
	lcd.write(cursor);
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

  mccPrefs.putBool("log", true);                // default is to log
  mccPrefs.putInt("fan_on_temp",  45);          // start fan at this temp if auto_fan
  mccPrefs.putInt("fan_off_temp", 35);          // stop fan at this temp if auto_fan
  mccPrefs.putBytes("psu_mac", default_mac, 6); // mac address of power supply 
  mccPrefs.putBytes("psu_mac", default_mac, 6); // mac address of power supply 

  mccPrefs.putBool("nvsInit", true);            // Create the "already initialized"
  //  key and store a value.
  // The "factory defaults" are created and stored so...
  mccPrefs.end();                               // Close the namespace in RW mode and...
}


void load_operational_params(void)
{

  mccPrefs.begin("mccPrefs", RO_MODE);       // Open our namespace (or create it
                                               //  if it doesn't exist) in RO mode.
   // Retrieve the operational parameters from the namespace
   //  and save them into their run-time variables.

   fan_on_temp = mccPrefs.getInt("fan_on_temp");
   fan_off_temp = mccPrefs.getInt("fan_off_temp");
   logging = mccPrefs.getBool("log"); 
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
    match =  sscanf(in_buf, "%c%c=%d", &cmd, &field, &value);

    switch (field) {
      // FIXME add v, i

    case 'D':
      // sscanf(in_buf, "%1c%10s", &cmd, &out_buf);  // whatever followed the 'm'
      // nope, cannot make sscanf work for a char then a 0 terminated string. ????
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
      
    case 'l':
      mccPrefs.begin("mccPrefs", RW_MODE);         // Open our namespace for write
      mccPrefs.putBool("log", value);               
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
	if (serial_buf_pointer >= 1) {  // at least a command letter
	  parse_buf(serial_buf, return_buf, 127);           // parse the buffer if at least one char in it.
	  Serial.print(return_buf);
	}
	serial_buf_pointer = 0;
      }
    }
}


void open_logfile (void) 
{
  openFile(SD, logfile);
  file.seek(0);             // always truncate a file that exists as I'm doing cluster writes
  // file.open("log.txt", O_WRONLY | O_APPEND);
}

void close_logfile (void)
{
  flush();
  file.close();
}

void logger (char*message)
{ if (logging)
    appendtoFile(message);
}

void openFile(fs::FS &fs, const char *path) {
  Serial.printf("Opening to file: %s\n", path);
  file = fs.open(path, FILE_WRITE); // open at end of current file
  if (!file) {
    Serial.println("Failed to open file for write");
  }
}

// for  short messages, <100 each after timestamp added
void appendtoFile(char *message) {
  uint16_t buf_remaining;
  uint8_t *buf;    //  pointer to buf of uint8's
  uint8_t *obuf;
  uint32_t msize;
  unsigned long day;
  String gtime;
  //unsigned long epoch_s;
  
  
  const uint8_t max_msg_size = 100;
  char msg[max_msg_size];
  // add timestamp
  //Serial.print("append millis=");
  //Serial.println(millis());
  day = rtc.getDay()-1;
   //Serial.print("day millis=");
   //Serial.println(millis());
  gtime = rtc.getTime();
  //Serial.print("time millis=");
  //Serial.println(millis());
  //epoch_s = rtc.getEpoch();
  //Serial.print("epoch millis=");
  //Serial.println(millis());
  
  //  snprintf(msg, max_msg_size, "%ld %s\n", epoch_s, message); 
  snprintf(msg, max_msg_size, "%2d:%s %s\n", day, gtime, message); 
  //Serial.print("snprintf millis=");
  //Serial.println(millis());
  buf_remaining = sdblksize - sdbuf_ptr;
  if (which_sdbuf == 0)
    {
       buf  = sdbuf0;
       obuf = sdbuf1;
    }
  else
    {
      buf  = sdbuf1;
      obuf = sdbuf0;
    }
  //Serial.print("choose buf millis=");
  //Serial.println(millis());
  msize = strlen(msg);
  //Serial.print("strlen millis=");
  //Serial.println(millis());
  if (buf_remaining > msize)
    {
      buf += sdbuf_ptr;
      memcpy(buf, msg, msize);
      sdbuf_ptr+= msize;
    }
  else if (buf_remaining == msize)
    {
      memcpy(buf + sdbuf_ptr, msg, msize);
      sdbuf_ptr= 0;
      file.write(buf, sdblksize);   // write seems to take about 40ms.
      which_sdbuf++;
      if (which_sdbuf == 2)
	which_sdbuf = 0;
    }
  else // too small to fit msg in buf
    {
      memcpy(buf + sdbuf_ptr, msg, buf_remaining);
      file.write(buf, sdblksize);
      sdbuf_ptr= msize-buf_remaining;
      memcpy(obuf, msg+buf_remaining, sdbuf_ptr);
      which_sdbuf++;
      if (which_sdbuf == 2)
	which_sdbuf = 0;
     }
  //Serial.print("end append millis=");
  //Serial.println(millis());

}

void flush (void)
{
  uint8_t *buf;  // buf is a pointer to array of chars
  if (which_sdbuf == 0)
    buf  = sdbuf0;
  else
    buf  = sdbuf1;
  file.write(buf, sdbuf_ptr);
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

