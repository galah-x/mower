//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'mcc  Time-stamp: "2025-04-01 11:32:16

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
//     Should help SDcard wear levelling also. Downside is you need to use the UI to turn
//     off logging (which flushes then closes the file) prior to power down if you want
//     the last few lines of logs.
//     The 4k write seems to take 40ms. hopefully thats not generally noticeable.

// test cmt MAC is 5c013b6c9938
// test mco MAC is      which replaces the cmt later in the dev process Mower COntroller
// test psu MAC is       wifi to serial adapter for the power supply. Originally a vichy mm adaptor
// test mcc MAC (ME) is 5c013b6cf4fc   MowerChargeController

#include "mcc_data.h"

// #define DEBUG
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
#ifdef DEBUG
  Serial.print("received ");
  Serial.print(len);
  Serial.print(" bytes from ");  
  Serial.printf("%02x%02x%02x%02x%02x%02x\n", mac[12], mac[13], mac[14], mac[15], mac[16], mac[17]) ;
  /* I suspect the mac' field here is the 24 byte mac header structure espressif uses
     fields 12 to 17 seem to be the source MAC. rest isn't obvious */
#endif
  parse_buf(response_data.message, return_buf, msgbuflen);
#ifdef DEBUG
  Serial.print("parsed responded: ");
  Serial.printf("%s", return_buf);
  Serial.printf((char *) response_data.message);
#endif
  //** return return_buf to requestor here **

  strncpy((char *)response_data.message, (char *) return_buf, msgbuflen);
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(mco_mac, (uint8_t *) &response_data, sizeof(response_data));
  
  if (result == ESP_OK) {
    // Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

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
   writing_to_log=false;
   
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
      logging = 0;
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
    
    if (logging)
      {
	open_logfile(false, true); // truncate, justify
	close_logfile();
      }
}
   

void loop (void)
{
  // char     logmsg[30]; // for building logging messages
  // unsigned long t1;
  // service serial character if any available.
  do_serial_if_ready();

  // service encoder and process the UI if required
  do_encoder_and_UI();
  
  // update temperature and fan state if its due 
  if (rtc.getEpoch() > (last_temp_update_time + temp_update_period))
    {
      hs_temperature=(int) tsense.readTemperature();
      //  snprintf(logmsg, 30, "Temp=%d", hs_temperature);
      
      // t1 = millis();
      // logger(logmsg);
      
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
