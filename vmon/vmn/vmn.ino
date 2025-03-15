//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'vmn   Time-stamp: "2025-03-15 12:33:00 john"';

// this is the app to run per battery vmon for the Ryobi mower.
// called vmn as vmon was taken for the pcb
// use tools -> board ->  ESP32 Dev module 

/*
  A vmon is per 12V LFP battery, and  runs off that battery.
  It physically bolts to the battery terminals.
  Its 12v to 5v switcher is enabled (by mco) (optically isolated) to minimize off state power consumption.
  It chats to mco serially, responding to mcc polls with addresses.   (wifi is possible)

  Vmon contains a calibrated 16 bit ADC to monitor the battery voltage.
  Vmon contains a passive balance circuit (aka a load resistor) to allow charge to be removed
  from the battery under test.   It's 25 1K resistors in parallel,
  It is expected balancing will happen at about 3.45V/cell, or 13.8V for the 4 cell 12v battery. So thats 345mA
  Its up to mco to turn that balance resistor on and off, noting vmon will turn the resistor off if it gets too hot.
  mco has vision across all batteries, its vmon's job to balance the batteries. Probably only on charge, at cv operation near top-off.

  Vmon can monitor battery and resistor temperature. Battery temperatures ghet reported. resistor temperature is for self protection.

*/


#include <Preferences.h>  // the NV memory interface
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "ESP32Time.h"   // must be V2.x or newer. V1.04 takes 5s to return a time string. breaks logging 
#include<ADS1115_WE.h>   // adc converter
#include <stdio.h>

#define DEBUG

// for preferences
#define RW_MODE false
#define RO_MODE true


// serial0 is used for debug and for programming NVM
const uint8_t longest_record = 24;  //   worst is display comand WD1=12345678901234567890
const uint8_t record_size = longest_record + 2;    //  char char = nnnnn \n
char serial_buf[record_size];
uint8_t serial_buf_pointer;

// serial2 is used for comms to mco
const uint8_t longest_record2 = 24;  //   worst is display comand WD1=12345678901234567890
const uint8_t record_size2 = longest_record2 + 2;    //  char char = nnnnn \n
char serial2_buf[record_size2];
uint8_t serial2_buf_pointer;

const  uint16_t msgbuflen= 128;  // for serial responses
char return_buf[msgbuflen]; 


const char * version = "VMON 15 Mar 2025 Reva";

Preferences vmonPrefs;  // NVM structure
// these will be initialized from the NV memory

int     balance_on_temp;
int     balance_off_temp;
float   adc0;
float   adcgain;
uint8_t board_address;
int     s2baud;       // serial 2 baud rate

/* IOs  definitions */
const uint8_t balance_en_pin  = 18;    // enable balance current dumper
const uint8_t alrt_pin  = 34;          // alert from adc
const uint8_t sda_pin   = 21;          // 
const uint8_t scl_pin   = 22;          //
const uint8_t rxd2_pin  = 16;          //
const uint8_t txd2_pin  = 17;          //

// I2C addresses 
const uint8_t battery_temperature_addr = 0x77;
const uint8_t resistor_temperature_addr = 0x76;
const uint8_t adc_addr = 0x48;

Adafruit_BMP280 tsenseB;    // I2C connected bmp280 temperature and pressure sensor. Used for Battery temp.
Adafruit_BMP280 tsenseR;    // I2C connected bmp280 temperature and pressure sensor. Used for Resistor temp.
ADS1115_WE adc = ADS1115_WE(adc_addr); // I2C connected ads1115 16b adcb  Used for battery voltage.

int     battery_temperature;     // current value of heatsink temperature for the mower switch.
int     resistor_temperature;    // current value of heatsink temperature for the mower switch.
bool    balance=false;           // true if mco has told me to balance.
float   voltage = 0.0;           // current battery voltage

ESP32Time rtc;
unsigned long last_temp_update_time;   // time last temperature was polled in seconds. 2 sensors alternate
const unsigned long temp_update_period = 5; // seconds.
bool doing_resistor_temp = true;

unsigned long last_voltage_time;         // time last power supply voltage was touched
const unsigned long voltage_update_period = 1;  // seconds.
bool adc_is_converting = false;

unsigned long last_vmon_time;         // time last power supply voltage and current was polled at

  

void setup (void) {
  Serial.begin(115200);
  Serial.println(version);

  /* init IOs */
  pinMode(balance_en_pin, OUTPUT);
  digitalWrite(balance_en_pin, 0);
  pinMode(alrt_pin, INPUT_PULLUP);

   // start temperature sensors
   if (!tsenseB.begin(battery_temperature_addr))
     {  
       Serial.print("Could not find a valid battery BMP280 sensor at address 0x");  
       Serial.println(battery_temperature_addr, HEX);  
     }
   battery_temperature = (int)  tsenseB.readTemperature();

   if (!tsenseR.begin(resistor_temperature_addr))
     {  
       Serial.print("Could not find a valid resistor BMP280 sensor at address 0x");  
       Serial.println(resistor_temperature_addr, HEX);  
     }
   resistor_temperature = (int)  tsenseR.readTemperature();
   
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
   
   // check NVM looks defined  
   vmonPrefs.begin("vmonPrefs", RO_MODE);     // Open our namespace (or create it if it doesn't exist)
   bool tpInit = vmonPrefs.isKey("nvsInit"); // Test for the existence of the "already initialized" key.
   vmonPrefs.end();                          // close the namespace in RO mode
   if (tpInit == false) 
     reinit_NVM();                          // reinitialize the nvm structure if magick key was missing 
   
   // load local variables from NVM
   load_operational_params();                // load all variables from NVM

   // init some general variables and IOs 
   serial_buf_pointer = 0;
   serial2_buf_pointer = 0;

   // interestingly, while rx=16 txd=17 is supposedly the default pin allocation,  Serial2 doesn't
   // work without explicitly filling in the pin numbers here
   Serial2.begin(s2baud, SERIAL_8N1, rxd2_pin, txd2_pin);  
   
   Serial.println("done setup");
}

   

void loop (void)
{
  // service serial character if any available.
  do_serial_if_ready();
  do_serial2_if_ready();
  
  // update temperature and monitor balance state if its due 
  if (rtc.getEpoch() > (last_temp_update_time + temp_update_period))
    {
      if (doing_resistor_temp)
	{
	  resistor_temperature = (int)  tsenseR.readTemperature();
	  doing_resistor_temp = false;
	  if (balance)
	    {
	      if (digitalRead(balance_en_pin) && (resistor_temperature >= balance_off_temp))
		digitalWrite(balance_en_pin, 0);
	      else if (!digitalRead(balance_en_pin) && (resistor_temperature <= balance_on_temp))
		digitalWrite(balance_en_pin, 1);
	    }
	}
      else
	{
	  battery_temperature = (int)  tsenseB.readTemperature();
	  doing_resistor_temp = true;
	}
      last_temp_update_time = rtc.getEpoch();
    }
  
   // update voltage if its due 
  if (rtc.getEpoch() > (last_voltage_time + voltage_update_period))
    {
      if (adc_is_converting)
	{
	  while(adc.isBusy()) { Serial.println("adc is busy?");}
	  voltage = adcgain * (adc.getResult_V() + adc0);
	  adc_is_converting = false;
	}
      else
	{
	  adc.startSingleMeasurement();
	  adc_is_converting = true;
	}
      last_voltage_time = rtc.getEpoch();
    }
}


  
void reinit_NVM (void)
{
  Serial.println("Initializing NVM");
  
  // If tpInit is 'false', the key "nvsInit" does not yet exist therefore this
  //  must be our first-time run. We need to set up our Preferences namespace keys. So...
  vmonPrefs.begin("vmonPrefs", RW_MODE);       //  open it in RW mode.
  
  // The .begin() method created the "mccPrefs" namespace and since this is our
  //  first-time run we will create
  //  our keys and store the initial "factory default" values.

  vmonPrefs.putInt("s2baud", 9600);          // baud rate serial 2 DPM8624
  vmonPrefs.putInt("bal_on_temp", 50);       // turn balance on again when temp gets this low
  vmonPrefs.putInt("bal_off_temp", 70);      // turn balance off if resistors getthis hot
  vmonPrefs.putFloat("adcgain", 5.3);        // approx adc gain term
  vmonPrefs.putFloat("adc0", 0.0);           // adc 0 offset term
  vmonPrefs.putUChar("address", '0');        // board address

  vmonPrefs.putBool("nvsInit", true);            // Create the "already initialized"
  //  key and store a value.
  // The "factory defaults" are created and stored so...
  vmonPrefs.end();                               // Close the namespace in RW mode and...
}


void load_operational_params(void)
{

  vmonPrefs.begin("vmonPrefs", RO_MODE);       // Open our namespace (or create it
                                             //  if it doesn't exist) in RO mode.
   // Retrieve the operational parameters from the namespace
   //  and save them into their run-time variables.
  s2baud           = vmonPrefs.getInt("s2baud");          // baud rate serial 2 DPM8624
  balance_on_temp  = vmonPrefs.getInt("bal_on_temp");     // turn balance on again when temp gets this low
  balance_off_temp = vmonPrefs.getInt("bal_off_temp");    // turn balance off if resistors getthis hot
  adcgain          = vmonPrefs.getFloat("adcgain");       // adc gain term
  adc0             = vmonPrefs.getFloat("adc0");          // adc 0 offset term
  board_address    = vmonPrefs.getUChar("address");         // board address
  

   // All done. Last run state (or the factory default) is now restored.
   vmonPrefs.end();                                      // Close our preferences namespace.
}

void parse_buf (char * in_buf, char * out_buf, int out_buf_len, uint8_t channel)
{
  // Z eraZe NV memory
  // Rf Read and print Field,
  //    where Field could be
  //         A board address 1..4
  //         B baud rate on S2
  //         G adc gain
  //         H highbalance temperature 
  //         L low balance temperature  
  //         O adc offset
  //         V version
  //         b current balance actual setting
  //         c current balance requested setting
  //         v current voltage
  //         r current resistor temperature
  //         s current battery temperature
  // Wf Field,
  //    where Field could be B baud rate on S2
  //         A board address 1..4
  //         B baud rate on S2
  //         G adc gain
  //         H highbalance temperature 
  //         L low balance temperature  
  //         O adc offset
  //         b balance               

  uint8_t read_pointer = 0;
  uint8_t  cmd;
  uint8_t  field;
  int      value;
  float    fvalue;
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
      case 'A':
	snprintf(out_buf, out_buf_len, "address=%c\n", board_address);
	break;

      case 'B':
	snprintf(out_buf, out_buf_len, "s2baud=%d\n", s2baud);
	break;

      case 'G':
	snprintf(out_buf, out_buf_len, "adc_gain=%f\n", adcgain);
	break;

      case 'H':
	snprintf(out_buf, out_buf_len, "balance_off=%dC\n",balance_off_temp );
	break;

      case 'L':
	snprintf(out_buf, out_buf_len, "balance_on=%dC\n", balance_on_temp);
	break;

      case 'O':
	snprintf(out_buf, out_buf_len, "adc_offset=%f\n", adc0);
	break;

      case 'V':
	snprintf(out_buf, out_buf_len, "%s\n", version);
	break;

      case 'b':
	if (channel != 2)
	snprintf(out_buf, out_buf_len, "balance_req=%d\n", balance);
	else
	  {
	    snprintf(out_buf, out_buf_len, ":%crb=%dC%c%c\n", board_address, balance, '0', '0');
	    insert_cs(out_buf);
	  }
	break;

      case 'c':
	snprintf(out_buf, out_buf_len, "balance_pin=%d\n", digitalRead(balance_en_pin));
	break;

      case 'v':
	if (channel != 2)
	  snprintf(out_buf, out_buf_len, "voltage=%fV\n", voltage);
	else // add checksum, : address
	  {
	    snprintf(out_buf, out_buf_len, ":%crv=%fV%c%c\n", board_address, voltage, '0', '0');
	    insert_cs(out_buf);
	  }
	break;

      case 'r': // had to go UTF8 to get the degree symbol in serial monitor
	if (channel != 2)
	  snprintf(out_buf, out_buf_len, "resistor_temp=%d%c%cC\n", resistor_temperature, char(0xC2), char(0xB0));
	else
	  {
	    snprintf(out_buf, out_buf_len, ":%crr=%dC%c%c\n", board_address, resistor_temperature, '0', '0');
	    insert_cs(out_buf);
	  }
	  
	  
	break;

      case 's':
	if (channel != 2)
	snprintf(out_buf, out_buf_len, "battery_temp=%d%c%cC\n", battery_temperature, char(0xc2),char(0xb0));
	else
	  {
	    snprintf(out_buf, out_buf_len, ":%crs=%dC%c%c\n", board_address, battery_temperature, '0', '0');
	    insert_cs(out_buf);
	  }
	break;
      }
      break;
    

  case 'W':
    if (channel != 2)
      {
    // first case, WA=3     decimal integer up to ~5 sig figures
    match =  sscanf(in_buf, "%c%c=%d", &cmd, &field, &value);

    switch (field) {
    case 'b' :   // balance
      if (value == 0)
	{
	  balance = false;
	  digitalWrite(balance_en_pin, LOW);
	}
      else
	{
	  balance = true;
	  digitalWrite(balance_en_pin, HIGH);
	}
      snprintf(out_buf, out_buf_len, "ok\n");
      break;
      
    case 'A':
      vmonPrefs.begin("vmonPrefs", RW_MODE);         // Open our namespace for write
      vmonPrefs.putUChar("address", in_buf[3]);               
      vmonPrefs.end();                              // Close the namespace
      load_operational_params();
      break;
      
    case 'B':
      vmonPrefs.begin("vmonPrefs", RW_MODE);         // Open our namespace for write
      vmonPrefs.putInt("s2baud", value);               
      vmonPrefs.end();                              // Close the namespace
      load_operational_params();
      break;
      
    case 'H':
      vmonPrefs.begin("vmonPrefs", RW_MODE);         // Open our namespace for write
      vmonPrefs.putInt("bal_off_temp", value);               
      vmonPrefs.end();                              // Close the namespace
      load_operational_params();
      break;
      
    case 'L':
      vmonPrefs.begin("vmonPrefs", RW_MODE);         // Open our namespace for write
      vmonPrefs.putInt("bal_on_temp", value);               
      vmonPrefs.end();                              // Close the namespace
      load_operational_params();
      break;
      
    case 'G':
      match =  sscanf(in_buf, "%c%c=%f", &cmd, &field, &fvalue);
      vmonPrefs.begin("vmonPrefs", RW_MODE);         // Open our namespace for write
      vmonPrefs.putFloat("adcgain", fvalue);               
      vmonPrefs.end();                              // Close the namespace
      load_operational_params();
      break;

    case 'O':
      match =  sscanf(in_buf, "%c%c=%f", &cmd, &field, &fvalue);
      vmonPrefs.begin("vmonPrefs", RW_MODE);         // Open our namespace for write
      vmonPrefs.putFloat("adc0", fvalue);               
      vmonPrefs.end();                              // Close the namespace
      load_operational_params();
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
	      parse_buf(serial_buf, return_buf, 127,0);  // parse the buffer if at least one char in it.
	      Serial.print(return_buf);
	    }
	  serial_buf_pointer = 0;
	}
    }
}

void do_serial2_if_ready (void)
{
  int serial2_byte;
  if (Serial2.available())
    {
      serial2_byte = Serial2.read();
      // note: neither \r nor \n is written to buffer. a zero is added to the buffer on \n
      if ((serial2_byte != '\n') && (serial2_buf_pointer < longest_record2))
	{
	  serial2_buf[serial2_buf_pointer] = serial2_byte;  // write char to buffer if space is available
	  serial2_buf_pointer++;
	}    
      if (serial2_byte == '\n') {
	serial2_buf[serial2_buf_pointer] = (uint8_t) 0;     // write string terminator to buffer
	if (serial2_buf_pointer >= 2)                       // at least : and address
	  {
	    Serial.printf("%s\n", serial2_buf); // always send s2 input to s0 debug console
	    if ((serial2_buf[0] == ':') && (serial2_buf[1] == board_address))
	      {
		parse_buf(serial2_buf+2, return_buf, 127,2);  // parse the buffer, same parser,
		                                             // skip first 2 chars :<my_addr>
		Serial2.print(return_buf);
	      }
	  }
	serial2_buf_pointer = 0;
      }
    }
}


// given a string that ends in : stuff '0' '0' \n 0, replace the two '0' chars with a ascii hex checksum such
// that the checksum of the string from [0] to the last character in <stuff> plus the checksum is 0. 
void insert_cs (char * buffer)
{
  uint8_t length = strlen(buffer);
  int cs_pos = length - 3;
  if (cs_pos < 0)
    return;
  uint8_t cs = 0;
  int i;
  for (i=0; i < cs_pos; i++)
    cs += buffer[i];
  cs = ~cs + 1;
  sprintf(buffer + cs_pos, "%02x\n", cs);
}
  
  
