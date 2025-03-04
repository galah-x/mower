//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'vmon_testapp  Time-stamp: "2025-03-04 20:43:04 john"';

// this is a little app to test, calibrate and set NV memory on a vmon board.

#include <Preferences.h>  // the NV memory interface
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "ESP32Time.h"


#define RW_MODE false
#define RO_MODE true

Preferences vmonPrefs;


const uint8_t record_size = 12;  //  char string , %02x \n
uint8_t serial_buf[record_size];
uint8_t serial_buf_pointer;
int serial_byte;
bool serial_buf_aligned;

// these will be initialized from the NV memory
uint8_t board_address;
int_t   adc_offset;
float   adc_gain;

/* IOs */
const uint8_t alrt_pin = 34;     // a dedicated input
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

long batt_volt_adc;  // this one is uncalibrated
float batt_volt;     //  incl calibration

void setup () {

  Serial.begin(9600);
  Serial.println("Vmon test app 4 Mar 2025");

  
   vmonPrefs.begin("VmonPrefs", RO_MODE);         // Open our namespace (or create it
                                                  //  if it doesn't exist) in RO mode.

   bool tpInit = vmonPrefs.isKey("nvsInit");      // Test for the existence
                                                  // of the "already initialized" key.

   if (tpInit == false) {
     Serial.println("Initializing NVM");

     // If tpInit is 'false', the key "nvsInit" does not yet exist therefore this
     //  must be our first-time run. We need to set up our Preferences namespace keys. So...
     vmonPrefs.end();                             // close the namespace in RO mode and...
     vmonPrefs.begin("VmonPrefs", RW_MODE);       //  reopen it in RW mode.

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
      vmonPrefs.begin("VmonPrefs", RO_MODE);        //  reopen it in RO mode so the setup code
                                                    //  outside this first-time run 'if' block
                                                    //  can retrieve the run-time values
                                                    //  from the "VMONPrefs" namespace.
   }
   // Retrieve the operational parameters from the namespace
   //  and save them into their run-time variables.

   board_address = vmonPrefs.getUchar("Addr");
   adc_offset    = vmonPrefs.getInt("ADC0");
   adc_gain      = vmonPrefs.getFloat("ADCg");
   
   // All done. Last run state (or the factory default) is now restored.
   vmonPrefs.end();                                      // Close our preferences namespace.

   // Carry on with the rest of your setup code...
   serial_buf_pointer = 0;
   millisecs_temp = rtc.getEpoch();

   should_balance_be_on = false;
   take_temp_battery = true;
   temperature_battery=battery_temp.readTemperature();
   temperature_resistor=resistor_temp.readTemperature();
   // initialize adc into some useful mode.
}

loop () {
  // service serial character if available.
  if (Serial.available())
    {
      serial_byte = Serial.read();
      if ((serial_byte != '\n') && (serial_buf_pointer < sizeof(serial_buf)))
	{
	  serial_buf[serial_buf_pointer] = serial_byte;  // write char to buffer if space is available
	  serial_buf_pointer++;
	}    
      if (serial_byte == '\n') {
	if (serial_buf_pointer >= 2) {  // at least a command letter
	  parse_serial_buf();           // parse the buffer if at least one char in it.
	}
	serial_buf_pointer = 0;
      }
    }

  // update each temperature each 1/2 second, alternating between resistor and battery temps.
  if (rtc.getEpoch() > (millisecs_temp + next_temp_delay))
    {
      if (take_temp_battery)
	{
	  temperature_battery=battery_temp.readTemperature();
	  take_temp_battery = false;
	  millisecs_temp= rtc.getEpoch() + next_temp_delay;
	}
      else
	{ // update resistor temperature, and protect it if the local temp is too high.
	  temperature_resistor=resistor_temp.readTemperature();
	  take_temp_battery = true;
	  millisecs_temp= rtc.getEpoch() + next_temp_delay;
	  if ( (should_balance_be_on)  && (temperature_resistor > resistor_temperature_limit))
	    {
	      set_pin(balance_pin, 1);
	    }
	  else
	    {
	      set_pin(balance_pin, 0);
	    }
	  
	}
    }

  // update voltage every ~~50ms
  if (rtc.getEpoch() > (millisecs_volt + next_volt_delay))
    {
      batt_volt_adc = read_adc();
      batt_volt = (batt_volt_adc + adc_offset) * adc_gain;
    }
  // ??whichever the slowest update is updates the stored timestamp...
  // or is there a timestamp per reading?  THAT sounds more appropriate.
}
