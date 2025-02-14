//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'relaybox gadget  Time-stamp: "2025-02-14 13:29:05 john"';

// #include "cowconsts.h"   // the herd controls
// #include "mbuttons.h"    // main buttons

#define CONNECT_TO_NETWORK

#ifdef CONNECT_TO_NETWORK
#include "WiFi.h"
#include "AsyncUDP.h"
#include "credentials.h"   // password and ssid live here
#endif

#include "io.h"    // io map
#include "esp_mac.h"
#include "ESP32Time.h"

// server for both time and files
// const char *   Address = "203.1.77.121";
// const uint16_t udpLogPort  = 5678;
// const uint16_t tcpLogPort  = 5678;
// const uint16_t tcpTimePort = 5679;
// const uint16_t tcpCowPort  = 5680;

const uint8_t record_size = 11;  // 7 char string , %02x \n

//The udp library class
AsyncUDP udp;

ESP32Time rtc;
// not bothering to set this, as only interested in timeout delays.
// time is just measured from last reset.

unsigned long tsecs;
const unsigned long timeout = 75;  // seconds.
char msgbuf[150];

// ############### SETUP ##########################


void setup()
{
  Serial.begin(115200);
    Serial.print("Hi. io seup\n");
    delay(2000);
  // set spares to digital output low to minimize power and other funnies
  pinMode(B1pin, OUTPUT);
  digitalWrite(B1pin,  LOW);
  pinMode(B2pin, OUTPUT);
  digitalWrite(B2pin,  LOW);
  pinMode(B3pin, OUTPUT);
  digitalWrite(B3pin,  LOW);
  pinMode(B4pin, OUTPUT);
  digitalWrite(B4pin,  LOW);
  pinMode(B5pin, OUTPUT);
  digitalWrite(B5pin,  LOW);
  pinMode(I1pin, OUTPUT);
  digitalWrite(I1pin,  LOW);

    Serial.print("About to wifi connect\n");
    delay(2000);
  
    //Connect to the WiFi network
#ifdef CONNECT_TO_NETWORK
  WiFi.mode(WIFI_STA);

  // Variable to store the MAC address
  uint8_t baseMac[6];
  
  // Get MAC address of the WiFi station interface
  esp_efuse_mac_get_default(baseMac);
  Serial.print("Station MAC: ");
  for (int i = 0; i < 5; i++) {
    Serial.printf("%02X:", baseMac[i]);
  }
  Serial.printf("%02X\n", baseMac[5]);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname);
  WiFi.begin(networkName, networkPswd);

  
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    //    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }

  if (udp.listen(8234)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udp.onPacket([](AsyncUDPPacket packet) {
	if (0)
	  {
	    Serial.print("UDP Packet Type: ");
	    Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
	    Serial.print(", From: ");
	    Serial.print(packet.remoteIP());
	    Serial.print(":");
	    Serial.print(packet.remotePort());
	    Serial.print(", To: ");
	    Serial.print(packet.localIP());
	    Serial.print(":");
	    Serial.print(packet.localPort());
	    Serial.print(", Length: ");
	    Serial.print(packet.length());
	    Serial.print(", Data: ");
	    Serial.write(packet.data(), packet.length());
	    Serial.println();
	  }
	if (packet.length() == 3) {
	  if (packet.data()[0] == 'B') {
	    if (packet.data()[1] == '0') {
	      digitalWrite(B1pin,  LOW);
	      digitalWrite(B2pin,  LOW);
	      digitalWrite(B3pin,  LOW);
	      digitalWrite(B4pin,  LOW);
	      digitalWrite(B5pin,  LOW);
	    }	
	    else if (packet.data()[1] == '1') {
	      digitalWrite(B2pin,  LOW);
	      digitalWrite(B3pin,  LOW);
	      digitalWrite(B4pin,  LOW);
	      digitalWrite(B5pin,  LOW);
	      digitalWrite(B1pin,  HIGH);
	    }
	    else if (packet.data()[1] == '2') {
	      digitalWrite(B1pin,  LOW);
	      digitalWrite(B3pin,  LOW);
	      digitalWrite(B4pin,  LOW);
	      digitalWrite(B5pin,  LOW);
	      digitalWrite(B2pin,  HIGH);
	    }
	    else if (packet.data()[1] == '3') {
	      digitalWrite(B1pin,  LOW);
	      digitalWrite(B2pin,  LOW);
	      digitalWrite(B4pin,  LOW);
	      digitalWrite(B5pin,  LOW);
	      digitalWrite(B3pin,  HIGH);
	    }
	    else if (packet.data()[1] == '4') {
	      digitalWrite(B1pin,  LOW);
	      digitalWrite(B2pin,  LOW);
	      digitalWrite(B3pin,  LOW);
	      digitalWrite(B5pin,  LOW);
	      digitalWrite(B4pin,  HIGH);
	    }
	    else if (packet.data()[1] == '5') {
	      digitalWrite(B1pin,  LOW);
	      digitalWrite(B2pin,  LOW);
	      digitalWrite(B3pin,  LOW);
	      digitalWrite(B4pin,  LOW);
	      digitalWrite(B5pin,  HIGH);
	    }
	    packet.printf("OK %s", packet.data());
	  }    
	  if (packet.data()[0] == 'I') {
	    if (packet.data()[1] == '0') {
	      digitalWrite(I1pin,  LOW);
	    }
	    else if (packet.data()[1] == '1') {
	      digitalWrite(I1pin,  HIGH);
	      // record time for timeout
	      tsecs =  rtc.getEpoch();  
	    }
	    packet.printf("OK %s", packet.data());
	  }
	}
	if (packet.length() == 2) {
	  if (packet.data()[0] == 'V') {
	    packet.printf("relaybox V1.3 250214\n");
	  }
	  if (packet.data()[0] == 'S') {
	    snprintf(msgbuf, sizeof(msgbuf), "B%d%d%d%d%d\nI", 
		     digitalRead(B1pin),
		     digitalRead(B2pin),
		     digitalRead(B3pin),
		     digitalRead(B4pin),
		     digitalRead(B5pin));
	    if (digitalRead(I1pin))
	      {
		packet.printf("%s1 tremaining=%d\n", msgbuf, (tsecs + timeout - rtc.getEpoch()));
	      }
	    else
	      {
		packet.printf("%s0\n", msgbuf);
	      }
	  }
	}
      }
      );
  }
#endif
  
}

// ############### LOOP ##########################


void loop()
{
  if (digitalRead(I1pin)) {
    if (rtc.getEpoch() > (tsecs + timeout))
      {
	// reset the current sink on timeout
          digitalWrite(I1pin,  LOW);
      }
  }
  delay(1);
}
