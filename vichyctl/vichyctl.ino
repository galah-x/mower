//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'vichyctl add wifi+utp to vichy vc8145 multimeter  Time-stamp: "2025-02-15 19:53:03 john"';

#define CONNECT_TO_NETWORK

#ifdef CONNECT_TO_NETWORK
#include "WiFi.h"
#include "AsyncUDP.h"
#include "credentials.h"   // password and ssid live here
#endif

const uint8_t record_size = 12;  //  char string , %02x \n
uint8_t serial_buf[record_size];
uint8_t serial_buf_pointer;
int serial_byte;
bool serial_buf_aligned;
const uint8_t serial_buf_poll_char = 0x89;

char valuestring[8];   // construction string for adc value
char unitstring[4];    // construction string for units
char valuestring_o[8];  // semaphore protected value string
char unitstring_o[4];   // semaphore protected units string
bool  locked;           // a semaphore to ensure I don't udp read a string while its updating

//The udp library class
AsyncUDP udp;

unsigned long millisecs;
const unsigned long nextpoll = 200;  // milliseconds.

// ############### SETUP ##########################


void setup()
{
  Serial.begin(115200);          // Serial0 is used for debug and programming
  Serial.print("Hi. io seup\n");
  delay(2000);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Serial2 is used for the Vichy connection   

  Serial.print("About to wifi connect\n");
  delay(1000);
  
  //Connect to the WiFi network
#ifdef CONNECT_TO_NETWORK
  WiFi.mode(WIFI_STA);
  
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname);
  WiFi.begin(networkName, networkPswd);

  
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }

  if (udp.listen(8235)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udp.onPacket([](AsyncUDPPacket packet) {
      // if (packet.length() == 0) {
      {
	while (locked) { }
	packet.printf("%s %s\n", valuestring_o, unitstring_o);
      }
    }
      );
  }
#endif
  millisecs=millis();
  serial_buf_pointer = 0;
  serial_buf_aligned = 0;
  locked = 0;
}

// ############### LOOP ##########################


void loop()
{
  // issue a new poll when its due
  if (millis() > (millisecs + nextpoll))
    {
      //    Serial.println("poll");
      Serial2.write(serial_buf_poll_char);
      millisecs = millis(); 
    }

  // check serial, load any new character into the buffer. Parse the buffer when full.
  // if (0) {
  if (Serial2.available())
    {
      serial_byte = Serial2.read();
      if (!serial_buf_aligned)
	{
	  if (serial_byte == serial_buf_poll_char) {
	    serial_buf_pointer = 0;
	    serial_buf_aligned = 1;
	    serial_buf[serial_buf_pointer] = serial_byte;
	    serial_buf_pointer++;
	  }
	}
      else // serial buffer is aligned
	{
	  if ((serial_buf_pointer == 0) && (serial_byte != serial_buf_poll_char))
	    { serial_buf_aligned = 0 ;
	    }
	  else
	    {
	      serial_buf[serial_buf_pointer] = serial_byte;
	      serial_buf_pointer++;
	      if (serial_buf_pointer == sizeof(serial_buf))
		{
		  serial_buf_pointer = 0 ;
		  parse_serial_buf();
		}
	    }
	}
    }
  //  }
}


// serial_buf[0] should be the poll char 0x89. checked as part of buffer alignment
// serial_buf[1] mode bits[7:3] 0xa0 generator
//                              0xd0 frequency
//                              0xc8 capacitance
//                              0xc0 temperature
//                              0xd8 diode
//                              0xe0 resistance
//                              0xa8 current A
//                              0xb0 current mA
//                              0xe8 volts mV
//                              0xf8 volta AC
//                              0xf0 volts DC
// serial_buf[2] range bits[5:3] 0x00 dpp = 1
//                               0x08 dpp = 2 ie xx.xxx
//                               0X10 DPP=3
//                              0x18 dpp=4   ie xxxx.x  
//                               0x20 dpp=5  
//                               0x28 dpp=6
//               range bits [6]  autorange
// serial_buf[3] unknown
//  serial_buf[4] sign  bits[6:4]  0x40 positive
//                                0x50 negative
//  serial_buf[5:9] ascii reading characters

// serial_buf[10]  might be a checksum
// serial_buf[11]  might be a newline


int before_dp;
int char_ptr;
int ii;
int units;

  void parse_serial_buf(void)
  {
    if (0)
      {
    // for now, just print it.
	   Serial.println("0  1  2  3  4  5  6  7  8  9  a  b");
	 Serial.printf("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n",
		   serial_buf[0], serial_buf[1],
		   serial_buf[2], serial_buf[3],
		   serial_buf[4], serial_buf[5],
		   serial_buf[6], serial_buf[7],
		   serial_buf[8], serial_buf[9],
		   serial_buf[10], serial_buf[11]
		   );
      }
    if (1)
      {
	units = serial_buf[1] & 0xf8;
	// sign
	char_ptr=0;
	if ((serial_buf[4] & 0x70) == 0x50)
	  {
	    valuestring[char_ptr++] = '-';
	  }
	else
	  {
	    valuestring[char_ptr++] = ' ';
	  }
	// ascii string
	before_dp = 1+((serial_buf[2] & 0x38) >> 3);
	if (before_dp > 5) {
	  before_dp = 5;
	}
	if (before_dp < 1) {
	  before_dp = 1;
	}
	if (units == 0xE8) { before_dp++;}

	for (ii=0; ii<before_dp; ii++) {
	  valuestring[char_ptr++] = serial_buf[ii+5];
	}
	valuestring[char_ptr++]='.';
	for (ii=before_dp; ii<5; ii++) {
	  valuestring[char_ptr++] = serial_buf[ii+5];
	}
	valuestring[char_ptr]=0;
      }
    if (1)
      { // units
	strcpy(unitstring, "??");
	if (units == 0xd0) { strcpy(unitstring,"Hz");} // freq gen
	if (units == 0xC8) { strcpy(unitstring, "F");}  // capacitance
	if (units == 0xC0) { strcpy(unitstring, "C");}  // temperature
	if (units == 0xD8) { strcpy(unitstring, "V");}  // diode 
	if (units == 0xE0) { strcpy(unitstring, "ohm");}  // resistance 
	if (units == 0xA8) { strcpy(unitstring, "A");}  // current
	if (units == 0xB0) { strcpy(unitstring, "mA");}  // current
	if (units == 0xE8) { strcpy(unitstring, "mV");}  // voltage
	if (units == 0xf8) { strcpy(unitstring, "Vac");}  // voltage
	if (units == 0xF0) { strcpy(unitstring, "Vdc");}  // voltage
      }
    locked = 1;
    strncpy(valuestring_o, valuestring, 8);
    strncpy(unitstring_o, unitstring, 4);
    locked = 0;

    if (0) {
        Serial.printf("%s %s %s %s before=%d s2=%0x\n", valuestring_o, unitstring_o, valuestring, unitstring, before_dp, serial_buf[2]);
    }
  }
