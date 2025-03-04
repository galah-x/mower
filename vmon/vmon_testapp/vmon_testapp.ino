//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'vmon_testapp  Time-stamp: "2025-03-05 10:53:09 john"';

// this is a little app to test, calibrate and set NV memory on a vmon board.

#include <Preferences.h>  // the NV memory interface
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "ESP32Time.h"
#include <stdio.h>

#define RW_MODE false
#define RO_MODE true

// #define HASTEMPSENSOR
// #define HASADC
#define DEBUG
Preferences vmonPrefs;


const uint8_t longest_record = 13;  //  char char = nnnnn 0, worst WG=1.2345e-3
const uint8_t record_size = longest_record + 2;    //  char char = nnnnn \n
char serial_buf[record_size];
uint8_t serial_buf_pointer;
int serial_byte;
bool serial_buf_aligned;


// these will be initialized from the NV memory
uint8_t board_address;
int     adc_offset;
float   adc_gain;

/* IOs */
const uint8_t alrt_pin = 34;     // is a dedicated input
const uint8_t balance_pin = 18;  // 350mA 5W balance dump resistors
const uint8_t battery_temp_addr = 0x76;
const uint8_t resistor_temp_addr = 0x77;


Adafruit_BMP280 battery_temp;    // I2C
Adafruit_BMP280 resistor_temp;    // I2C

unsigned long millisecs_temp;
const unsigned long next_temp_delay = 500;  // milliseconds.
// going to alternate battry temp then resistor temp every 500 msecs. So each updates once per second.

unsigned long millisecs_volt;
const unsigned long next_volt_delay = 10;    // milliseconds.
ESP32Time rtc;

unsigned long temperature_resistor;
unsigned long temperature_battery;
bool take_temp_battery;

bool should_balance_be_on;
long battery_temperature_limit;
long resistor_temperature_limit;

long batt_volt_adc = 0;  // this one is uncalibrated, the raw adc reading.
float batt_volt = 0.0;   // this one includes calibration

void setup (void) {

  Serial.begin(115200);
  Serial.println("Vmon test app 5 Mar 2025");
  
   vmonPrefs.begin("VmonPrefs", RO_MODE);         // Open our namespace (or create it
                                                  //  if it doesn't exist) in RO mode.

   bool tpInit = vmonPrefs.isKey("nvsInit");      // Test for the existence
                                                  // of the "already initialized" key.
   vmonPrefs.end();                               // close the namespace in RO mode and...

   if (tpInit == false) {
     reinit_NVM();
   }

   load_operational_params();                     // load all systemwide constants from NVM
   
   // Carry on with the rest of your setup code...
   serial_buf_pointer = 0;
   millisecs_temp = rtc.getEpoch();

   pinMode(balance_pin, OUTPUT);
   digitalWrite(balance_pin,  LOW);
   should_balance_be_on = false;

    take_temp_battery = true;
   // dont do this until I have hardware with temp sensors present
   //   temperature_battery=battery_temp.readTemperature();
   // temperature_resistor=resistor_temp.readTemperature();
   // initialize adc into some useful mode.
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
	  parse_serial_buf();           // parse the buffer if at least one char in it.
	}
	serial_buf_pointer = 0;
      }
    }

  // update each temperature each 1/2 second, alternating between resistor and battery temps.
#ifdef HASTEMPSENSOR
  if (rtc.getEpoch() > (millisecs_temp + next_temp_delay))
    {
      if (take_temp_battery)
	{
	  temperature_battery=battery_temp.readTemperature();
	  take_temp_battery = false;
	}
      else
	{ // update resistor temperature, and protect it if the local temp is too high.
	  temperature_resistor=resistor_temp.readTemperature();
	  take_temp_battery = true;
	  if ( (should_balance_be_on)  && (temperature_resistor > resistor_temperature_limit))
	    {
	      digitalWrite(balance_pin, 1);
	    }
	  else
	    {
	      digitalWrite(balance_pin, 0);
	    }
	}
      millisecs_temp= rtc.getEpoch();
    }
#endif
  // update voltage every ~~50ms
  // don't do this till I have hardware with an ADC present
#ifdef HASADC
  if (rtc.getEpoch() > (millisecs_volt + next_volt_delay))
    {
      batt_volt_adc = read_adc();
      batt_volt = (batt_volt_adc + adc_offset) * adc_gain;
      millisecs_volt = rtc.getEpoch(); 
    }
  // ??whichever the slowest update is updates the stored timestamp...
  // or is there a timestamp per reading?  THAT sounds more appropriate.
#endif
}


void reinit_NVM (void)
{
  Serial.println("Initializing NVM");
  
  // If tpInit is 'false', the key "nvsInit" does not yet exist therefore this
  //  must be our first-time run. We need to set up our Preferences namespace keys. So...
  vmonPrefs.begin("VmonPrefs", RW_MODE);       //  open it in RW mode.
  
  // The .begin() method created the "VMONPrefs" namespace and since this is our
  //  first-time run we will create
  //  our keys and store the initial "factory default" values.
  vmonPrefs.putUChar("Addr", 1);                // unsigned char address byte
  vmonPrefs.putInt("ADC0", 0);                  // signed  0 offset        
  vmonPrefs.putFloat("ADCg", 0.001);            // ADC gain term     
  
  vmonPrefs.putBool("nvsInit", true);           // Create the "already initialized"
  //  key and store a value.
  // The "factory defaults" are created and stored so...
  vmonPrefs.end();                              // Close the namespace in RW mode and...
}

void load_operational_params(void)
{
  vmonPrefs.begin("VmonPrefs", RO_MODE);       // Open our namespace (or create it
                                               //  if it doesn't exist) in RO mode.
   // Retrieve the operational parameters from the namespace
   //  and save them into their run-time variables.

   board_address = vmonPrefs.getUChar("Addr");
   adc_offset    = vmonPrefs.getInt("ADC0");
   adc_gain      = vmonPrefs.getFloat("ADCg");
   
   // All done. Last run state (or the factory default) is now restored.
   vmonPrefs.end();                                      // Close our preferences namespace.
}

void parse_serial_buf (void)
{
  // Z eraZe NV memory
  // Rf Read and print Field, where Field could be Address, O adc offset term, G adc gain term.
  // Wf=nnnnn Write Field (as above) with integer term nnnnn. G takes a float.
  //
  // A print calibrated ADC reading
  // a print raw adc reading.
  // B=n turn Balance resistor on[1]/off[0]
  // T[rb] print temperature of the Resistor or Battery
  uint8_t read_pointer = 0;
  uint8_t  cmd;
  uint8_t  field;
  int      value;
  float    fvalue;
  
  cmd = serial_buf[read_pointer++];
  switch (cmd) {
  case 'Z':
    { reinit_NVM();
      load_operational_params();
      break;
    }

  case 'R':
    {
      field = serial_buf[read_pointer++];
      switch (field) {
      case 'A':
	Serial.printf("Address = %d\n", board_address);
	break;
	
      case 'O':
	  Serial.printf("ADC Offset = %d\n", adc_offset);
	  break;
	
      case 'G':
	Serial.printf("ADC Gain = %f\n", adc_gain);
	break;

      case 'B':
	Serial.printf("current balance pin=%d, desired=%d\n", digitalRead(balance_pin), should_balance_be_on);
	break;
      }
      break;
    }

  case 'W':
    {
      int match =  sscanf(serial_buf, "%c%c=%d", &cmd, &field, &value);
#ifdef DEBUG
      Serial.printf("match=%d cmd=%c field=%c value=%d\n", match, cmd, field, value);
#endif
      if (field == 'G') {
	match =  sscanf(serial_buf, "%c%c=%f", &cmd, &field, &fvalue);
#ifdef DEBUG
	Serial.printf("match=%d cmd=%c field=%c value=%f\n", match, cmd, field, fvalue);
#endif
      }

      switch (field) {
      case 'A':
	vmonPrefs.begin("VmonPrefs", RW_MODE);         // Open our namespace for write
	vmonPrefs.putUChar("Addr", value);               
	vmonPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
	
      case 'O':
	vmonPrefs.begin("VmonPrefs", RW_MODE);         // Open our namespace for write
	vmonPrefs.putInt("ADC0", value);               
	vmonPrefs.end();                              // Close the namespace
	load_operational_params();
	break;

      case 'G':
	vmonPrefs.begin("VmonPrefs", RW_MODE);         // Open our namespace for write
	vmonPrefs.putFloat("ADCg", fvalue);               
	vmonPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
      }
      break;
    }
  
  case 'B':
    {
      sscanf(serial_buf, "%c=%d", &cmd, &value);
      pinMode(balance_pin, OUTPUT);
      if (value != 0)
	{
	  digitalWrite(balance_pin,  HIGH);
	  should_balance_be_on = 1;
	}
      else
	{
	  digitalWrite(balance_pin,  LOW);
	  should_balance_be_on = 0;
	}
      break;
    }
  }
}
