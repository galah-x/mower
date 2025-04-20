//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'mco  Time-stamp: "2025-04-20 14:04:17 john"';

// this is the app to run the mower comms controller for the Ryobi mower.
// use tools -> board ->  ESP32 Dev module 

/* mco is responsible for talking to the 4 vmons.
   And, directly,  psu.  to adjust current and voltage settings.
   It has to report all relevant loggable info to mcc.
   It has to get the info from vmons. It runs all this.

   Locally, mco WAS SUPPOSED to have access to the current into/out of the batteries via current shunt and adc
   but my design was broken, the adc doesn't do bipolar inputs which the design needed.
   Now there is an imon board, with the same type adc on an isolated supply rail board, the shunt is biassed
   to mid rail. Data gets to mco via wifi, just like everything else.
   
   MCO integrates current to get SOC.
   It zeros SOC on empty, one day adjusting the battery capacity measure when that happens.
   It 100%s SOC on full, one day adjusting the battery capacity measure when that happens.
   It beeps  when battery SOC at ~10% && not_charging
   it records SOC in local fram.  3 copies currently.

   It controls power supply settings on charge to implement a main charge CC phase transitioning to CV as
   current drops below the preset max current. When current falls enough, switches state to a higher
   voltage, lower current setting for a topoff phase, where the balancers can kick in and make a significant
   effect.
   When current is low enough, or individual battery voltage high enough, it all stops.
*/

#include "mco_data.h"


// wifi callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming_data, incomingData, sizeof(incoming_data));
  // Serial.print("received ");
  // Serial.print(len);
  // Serial.print(" bytes from ");  
  // Serial.printf("%02x%02x%02x%02x%02x%02x\n", mac[12], mac[13], mac[14], mac[15], mac[16], mac[17]) ;
  /* I suspect the mac' field here is the 24 byte mac header structure espressif uses
     fields 12 to 17 seem to be the source MAC. rest isn't obvious */

  // from psu?  Record polled readings. more differences at end of mac string
  if ((mac[17] == psu_mac[5]) && (mac[16] == psu_mac[4]) && (mac[15] == psu_mac[3]) &&
      (mac[14] == psu_mac[2]) && (mac[13] == psu_mac[1]) && (mac[12] == psu_mac[0]))
    parse_psu_wifi_buf(incoming_data.message);
  // from imon?  Record unpolled readings
  else if ((mac[12] == imon.mac[0]) && (mac[13] == imon.mac[1]) && (mac[14] == imon.mac[2]) &&
      (mac[15] == imon.mac[3]) && (mac[16] == imon.mac[4]) && (mac[17] == imon.mac[5]))
    parse_imon_wifi_buf(incoming_data.message);
  // from a vmon?  Record polled readings
  else {
    uint8_t i;
    for (i=0; i< vmons; i++)
      {
	if ((mac[17] == vmon[i].mac[5]) && (mac[16] == vmon[i].mac[4]) && (mac[15] == vmon[i].mac[3]) &&
	    (mac[14] == vmon[i].mac[2]) && (mac[13] == vmon[i].mac[1]) && (mac[12] == vmon[i].mac[0]))
	  parse_vmon_wifi_buf(incoming_data.message, i);
      }
  }
}
void setup (void) {
  Wire.begin();   //   Including Wire.begin seems to break FRAM ???
  Serial.begin(115200);
  Serial.println(version);
  
  // interestingly, while rx=16 txd=17 is supposedly the default pin allocation,  Serial2 doesn't
  // work without explicitly filling in the pin numbers here
  //Serial2.begin(9600, SERIAL_8N1, 16, 17);  

  //init vmon structure
  int i;
  int j;
  
  for (i=0;i<vmons;i++)
    {
      vmon[i].volt=0.0;
      vmon[i].battemp=0;
      vmon[i].restemp=0;
      vmon[i].mostrecent=0;
      vmon[i].received=0;
      vmon[i].sent=0;
      vmon[i].balance=0;
      vmon[i].protocol_err=0;
      vmon[i].act_balance=255;
      vmon[i].gain = -1;
      vmon[i].offset = -1;
    }
  vmon_struct_loaded = false;

  // init imon struct
      imon.current=0.0;
      imon.mostrecent=0;
      imon.received=0;
      imon.protocol_err=0;
  
  
  //init vmon command q
  for (i=0;i<cmdqlen;i++)
    {
      // vq[i].cmd=(char) 0;
      vq[i].cmd_avail=0;
      vq[i].qclaimed=0;
    }

  // init the vmon polling management struct
  for (i=0; i<vmon_ii_max; i++)    // set which vmon to poll     ie the vmon poll address
	vpoll[i].addr = i & 0x03;  // poll vmons in order 0,1,2,3,0,1,2,3 etc
  for (i=0;i<vmons;i++)              
    {                                 // set what to poll for each of the 4 vmons
      vpoll[i].letter         = 'v';  // first do a read voltage  for each vmon
      vpoll[i+  vmons].letter = 's';  // then do a read battery temp for each vmon
      vpoll[i+2*vmons].letter = 'r';  // then do a read resistor temp for each vmon
      vpoll[i+3*vmons].letter = 'v';  // then do another read voltage poll for each vmon
      vpoll[i+4*vmons].letter = 'B';  // then do a write balance  for each vmon
      vpoll[i+5*vmons].letter = 'b';  // then do a read actual balance state poll for each vmon, monitor for temp effects
    }

  // init charger structure
  charger.voltage = 0.0;
  charger.current = 0.0;
  charger.enable = 0;
  charger.mostrecent = 0;
  charger.sent = 0;
  charger.received = 0;
  State = Charger_not_init;
    
  
  Serial.println("init NVM");
  // initialize NVM  
   mcoPrefs.begin("mcoPrefs", RO_MODE);     // Open our namespace (or create it if it doesn't exist)
   bool tpInit = mcoPrefs.isKey("nvsInit"); // Test for the existence of the "already initialized" key.
   mcoPrefs.end();                          // close the namespace in RO mode
   if (tpInit == false) 
     reinit_NVM();                          // reinitialize the nvm structure if magik key was missing 
   
   // load local variables from NVM
   load_operational_params();                // load all variables from NVM

   // init IO
   pinMode(fram_wp_pin, OUTPUT);
   pinMode(buz_en_pin, OUTPUT);
   pinMode(alrt_pin, INPUT);
   pinMode(charger_connected_pin, INPUT);
   pinMode(soc_pps_pin, OUTPUT);

   digitalWrite(fram_wp_pin,  HIGH);
 
   // buzzer allocate
#ifdef DC_BEEPER
   digitalWrite(buz_en_pin, 0);
#else
   ledcAttach(buz_en_pin, freq_beep, beep_res);
   ledcWriteTone(buz_en_pin, 0);
#endif
   // soc allocate
   ledcAttach(soc_pps_pin, freq_soc_50, resolution);

   //   ledcWrite(buz_en_pin, 0);     // beeper off
   ledcWrite(soc_pps_pin, 128);  // set soc gauge to 50%
   

   // init some general variables and IOs 
   serial_buf_pointer = 0;
   last_current_update_time = rtc.getEpoch();
   last_display_update_time = rtc.getEpoch();
   last_charger_state_update_time = rtc.getEpoch();

   // at startup, confirm all 4 vmons are present.
   // play a beep code if not so.
   Serial.println("init FRAM");
   //   if (!fram.begin(0x50, fram_wp_pin))
   if (!fram.begin(0x50))
     {
       Serial.println("FRAM not connected!");
     }



   
   // check soc. stored in fram 3 times. choose the majority. 
   if (get_SOC(0) == get_SOC(1))
     {
     if (get_SOC(2) != get_SOC(0))
       {
	 write_fram_SOC(2, get_SOC(0));    // is 2 is the odd one out, rewrite it.
	 Serial.printf("3rd copy fram differs, set to %d\n", get_SOC(0));
       }
     else 
       Serial.printf("3 fram copies match %d\n", get_SOC(0));
     }
   else if (get_SOC(0) == get_SOC(2))
     {
       if (get_SOC(1) != get_SOC(0))
	 {
	   write_fram_SOC(1, get_SOC(0));    // is 1 is the odd one out, rewrite it.
	   Serial.printf("2nd copy fram differs, set to %d\n", get_SOC(0));
	 }
     }
   else if (get_SOC(1) == get_SOC(2))
     {
       if (get_SOC(0) != get_SOC(1))
	 {
	   write_fram_SOC(0, get_SOC(1));    // is 1 is the odd one out, rewrite it.
	   Serial.printf("1st copy fram differs, set to %d\n", get_SOC(1));
	 }
     }
   else
     { 
       write_SOC(0, 3600000 * 50);    // initilize to 50% as I have no idea
       Serial.printf("all fram copies differ, set to %d\n", get_SOC(1));
     }       
   soc = get_SOC(0);

#ifdef ADC_FITTED
  Serial.println("init ADS1115");
   // init adc
   if(!adc.init()){
     Serial.println("ADS1115 not connected!");
   }
   adc.setVoltageRange_mV(ADS1115_RANGE_4096);  // 4.096V input range.
   // input attenuator about 0.2  0.2 * 15V max = 3V
   adc.setCompareChannels(ADS1115_COMP_0_1);    // measure between 0 and 1.
   adc.setConvRate(ADS1115_8_SPS);              // 8 samples per sec, slowest.
   adc.setAlertPinMode(ADS1115_DISABLE_ALERT);  // not using voltage range checks
   adc.setMeasureMode(ADS1115_CONTINUOUS);          // initiate measurements
#endif
   
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
    // Register peer mcc
    memcpy(peerInfo.peer_addr, mcc_mac, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
    }
    // Register peer psu
    memcpy(peerInfo.peer_addr, psu_mac, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
    }
    // Register peer cmt   // hmmm, with this registered i could not talk to vmon4. 
    //memcpy(peerInfo.peer_addr, cmt_mac, 6);
    //if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //  Serial.println("Failed to add peer");
    //}
    for (i=0; i< vmons; i++)
      {
	// Register vmons
	memcpy(peerInfo.peer_addr, vmon[i].mac, 6);
	Serial.printf("registering vmon%d mac = %02x%02x%02x%02x%02x%02x\n", i, vmon[i].mac[0], vmon[i].mac[1], vmon[i].mac[2], vmon[i].mac[3], vmon[i].mac[4], vmon[i].mac[5]); 
	if (esp_now_add_peer(&peerInfo) != ESP_OK){
	  Serial.println("Failed to add peer");
	}
      }
    Serial.println("done setup");
    
}

   

void loop (void)
{
  // service serial character if any available.
  do_serial_if_ready();
  
  
  if (millis() > last_beep_time + beep_on_period)
    {
      last_beep_time = millis();
      update_beeper_state();
    }
    
  
  // update current and SOC state if its due 
  if (rtc.getEpoch() > (last_current_update_time + current_update_period))
    {
      last_current_update_time = rtc.getEpoch();
      update_SOC_state();
    }
  
  // perform vmon polling according to the management structure. update all about once per 2 seconds
  if (millis()  > (last_vmon_time + vmon_period_millis))
    {
      last_vmon_time = millis();
      update_vmon_state();
    }
  
  // poll psu
  // update voltage/current/enable from psu if its due . update all about once per second
  if (millis()  > (last_psu_time + psu_period_millis))
    {
      last_psu_time = millis();
      update_psu_state();
    }
  
  
  //   update display on mcc
  if (rtc.getEpoch() > (last_display_update_time + display_update_period))
    {
      update_mcc_display();
      last_display_update_time=rtc.getEpoch();
    }      

  // update charger state machine  if its due
  if (rtc.getEpoch()  > (last_charger_state_update_time + charger_state_update_period))
    {
      last_charger_state_update_time = rtc.getEpoch();
      update_charge_state_machine();
    }
  // and maybe later... adjust stored battery capacity on SOC unusual transitions.
}

    
