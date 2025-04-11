//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'mco state machine  Time-stamp: "2025-04-11 12:12:34 john"';

// main functions for loop() in mco,  the state machine implementing the main runtime functionality


void update_charge_state_machine (void)
{
       
  // don't do much of this if I don't have a few readings from every vmon. Like when I'm bench testing a component.
  if (vmon_struct_loaded == false)
    if ((vmon[0].received > 20) && (vmon[1].received > 20) && (vmon[2].received > 20) && (vmon[3].received > 20))
      vmon_struct_loaded = true;


  // initialize the state structure when mowing
  if ( (State == Mowing) && (digitalRead(charger_connected_pin)==1))
    // hmmm, what about if I was mowing and now Im not.
    State = Charger_not_init;

  // process the state machine
  if (digitalRead(charger_connected_pin)==0)
    {
      State = Mowing;
      if (vmon_struct_loaded && 
	  ((vmon[0].volt < min_battery_voltage) ||(vmon[1].volt < min_battery_voltage) ||
	   (vmon[2].volt < min_battery_voltage) ||(vmon[3].volt < min_battery_voltage)))
	{ // a battery voltage is too low, set soc to 0% so beeping should start
	  soc  =  0;
	  write_SOC(0, soc );
	}
    }

  // items below here mean the charger is plugged in. It may not necessarily be either turned on on nor enabled.
  else if (State == Charger_not_init)
    { // as I don't know if the charger is actually on and listening, repeat this till  
      set_psu_e(0);  // should already be disabled.
      set_psu_v(initial_charge_voltage);
      set_psu_i(initial_charge_current);
      // don't leave this state till I'm sure the prior settings were correctly received by the charger
      if ((charger.voltage >= (initial_charge_voltage - charger_voltage_tol)) 
	  && (charger.voltage <= (initial_charge_voltage + charger_voltage_tol)) 
	  && (charger.current >= (initial_charge_current - charger_current_tol)) 
	  && (charger.current <= (initial_charge_current + charger_current_tol))) 
	State = Charger_init_CC;
    }
  
  else if (State = Charger_init_CC)
    {
      set_psu_e(1);
      if (imon.current > (1.1*transition_current)) // charger is now actually charging...)
	{
	  // checking the imon current inherantly checks the enable happened.
	  State = CC;
	}
    }

  else if ((State == CC) && (charger.current < transition_current))
    {
      set_psu_e(0);
      set_psu_v(topoff_charge_voltage);
      set_psu_i(topoff_charge_current);
      // don't leave this state till I'm sure the prior settings were correctly received by the charger
      if ((charger.voltage >= (topoff_charge_voltage - charger_voltage_tol)) 
	  && (charger.voltage <= (topoff_charge_voltage + charger_voltage_tol)) 
	  && (charger.current >= (topoff_charge_current - charger_current_tol)) 
	  && (charger.current <= (topoff_charge_current + charger_current_tol))) 
	State = Charger_init_CV;
    }


  else if ((State == Charger_init_CV))
    {
      set_psu_e(1);
      if (imon.current > (topoff_charge_current/2)) // charger is now actually charging...)
	{
	  // checking the imon current inherantly checks that the enable happened.
	  State = CV;
	}
    }

  else if ((State == CV) && (charger.current < cutoff_current))
    {
      soc  =  battery_capacity;  // set soc to 100%
      State = Done;
    }
  else if (State == CV)
    { // manage balance settings.
      // choose the smallest of the 4 battery voltages
      // any battery less than  batt_balance_voltage should not be balancing
      // any battery less than  smallest + batt_balance_tol_voltage should not be balancing
      // any battery > smallest + batt_balance_tol_voltage should be balancing
      
      // choose the smallest of the 4 battery voltages
      float lowest_battery_voltage = vmon[0].volt;
      int i;
      for (i=1; i<vmons; i++)
	if (vmon[i].volt < lowest_battery_voltage)
	  lowest_battery_voltage = vmon[i].volt;
      
      // decide the voltage threshold above which balance should happen 
      float min_balance_battery_voltage = lowest_battery_voltage + batt_balance_tol_voltage;
      if  (min_balance_battery_voltage < batt_balance_voltage)
	min_balance_battery_voltage = batt_balance_voltage;
      
      // set balance condition for each battery according to its relationship to the balance threshold 
      for (i=0; i<vmons; i++)
	if (vmon[i].volt < min_balance_battery_voltage)
	  vmon[i].balance = 0;
	else 
	  vmon[i].balance = 1;
    }

  else if (State == Done)
    // handle a potential loss of disable packet.
    // only way out of here is via a power down. Or unplug the charger.
    set_psu_e(0);

  

  
  // check this whenever actually charging.
  if ((State != Mowing) && ( (vmon[0].volt > max_battery_voltage) ||(vmon[1].volt > max_battery_voltage) ||
			     (vmon[2].volt > max_battery_voltage) ||(vmon[3].volt > max_battery_voltage)))
    { // an individual battery voltage is too high
      set_psu_e(0);              // turn off charger
      State = Done;
      soc  =  battery_capacity;  // set soc to max
      write_SOC(0, soc );
    }
}
 



/* check the serial interface to see if there are any characters, and process them if so */  

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
	if (serial_buf_pointer >= 1) {                 // at least a command letter
	  parse_buf(serial_buf);      // parse the buffer if at least one char in it.
	  // Serial.print(return_buf);
	}
	serial_buf_pointer = 0;
      }
    }
}



void update_beeper_state (void)
{
  if (last_beep_state)
    {
#ifdef DC_BEEPER
      digitalWrite(buz_en_pin, 0);
#else  
      ledcWriteTone(buz_en_pin, 0); 
#endif
      // ledcWrite(buz_en_pin, 0);     // beeper off
      last_beep_state = false;
    }
  else
    {
      if (!digitalRead(charger_connected_pin) && (soc < beep_SOC))
	{
#ifdef DC_BEEPER
	  // Serial.printf("setting beeper on\n");
	  digitalWrite(buz_en_pin, 1);
#else
	  ledcWriteTone(buz_en_pin, freq_beep);
#endif
	}
      last_beep_state = true;
    }
}




void update_SOC_state (void)
{

#ifdef ADC_FITTED
  while(adc.isBusy()) {
    Serial.println("adc is busy?");
    delay(1000);
  }
#endif
  
  soc = get_SOC(0) + (int32_t) (imon.current * 2000.0 * current_update_period);
  if (soc < 0)
    soc = 0;
  if (soc > battery_capacity)
    soc = battery_capacity;

  // update SOC stored in non volatile FRAM
  write_SOC(0, soc);

  // update the SOC gauge
  int soc_f =     336.0 * (float)soc /  fbattery_capacity;
  if (soc_f != soc_pps_freq)
    {
      // only update soc freq generator when freq changes. meter was glitchy prior
      ledcChangeFrequency(soc_pps_pin, soc_f, 8);
      soc_pps_freq = soc_f;
    }
}


void update_vmon_state (void)
{
#ifdef MEAS_PERF
  // record the issue time of voltage polls
  if (vmon.[vmon_ii].letter = 'v') 
    {
      vmon_polltime[vpoll[vmon_ii].addr].issue = millis();
    }
#endif

  if (vq[0].cmd_avail)
    {
      //  if there is a msg in the vmon queue, send it in the curent ringbuffer timeslot.
      // pause the real poll timeslot, dont skip the timeslot.
      // note as curently coded, its single entry queue with no shifting.
      // anyone wanting to write to the queue has to check the q is empty first.
      send_q_msg();
    }
  else   // only increment vmon_ii command ring buffer if I did a poll. not on queue interruption.
    {
      if (suspend_vmon_polling == false)
	{
	  if (vpoll[vmon_ii].letter == 'B') // the desired poll is a Balance write, all others are reads 
	    sprintf(outgoing_data.message, "Wb=%1d\n",  vmon[vpoll[vmon_ii].addr].balance );
	  else 
	    sprintf(outgoing_data.message, "R%c\n",  vpoll[vmon_ii].letter);
	  esp_now_send(vmon[vpoll[vmon_ii].addr].mac, (uint8_t *) &outgoing_data, strlen(outgoing_data.message)+1);
	  
	  // record that a message was sent on that channel
	  vmon[vpoll[vmon_ii].addr].sent++;
	  
	  // increment vmon_ii
	  vmon_ii++;
	  if (vmon_ii >= vmon_ii_max)
	    vmon_ii = 0;
	}
    }
}

void update_psu_state (void)
{
  if (! suspend_psu_polling)
    {
      sprintf(outgoing_data.message, ":01r%02d=0,\n", psu_addr[psu_ii]);
      psu_ii++;
      if (psu_ii >= psu_ii_max)
	psu_ii = 0;
      esp_now_send(psu_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
    }
}




void update_mcc_display (void)
{
  uint8_t i;
  for (i=0; i< vmons; i++)    // battery voltages
    { 
      if ((rtc.getEpoch() - vmon[i].mostrecent) <= old_message_time)  
	sprintf(outgoing_data.message, "WD%1d=%1d %2.3fV B%1d", i, i+1, vmon[i].volt, vmon[i].act_balance);
      else
	sprintf(outgoing_data.message, "WD%1d=%1d ??????????", i, i+1);
      esp_now_send(mcc_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
    }
  // d4 charger voltage
  if ((rtc.getEpoch() - charger.mostrecent) <= old_message_time)  
    sprintf(outgoing_data.message, "WD4=%2.3fV ", charger.voltage);
  else
    sprintf(outgoing_data.message, "WD4=??????V ");
  esp_now_send(mcc_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
  
  //d5, battery current
  if ((rtc.getEpoch() - imon.mostrecent) <= old_message_time)  
    sprintf(outgoing_data.message, "WD5=I=%2.3fA ", imon.current);
  else 
    sprintf(outgoing_data.message, "WD5=C=%2.3fA ", charger.current);
  esp_now_send(mcc_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
  
  //d6 SOC
  if (battery_capacity > 0) 
    {
      sprintf(outgoing_data.message, "WD6=SOC=%2d%%", (int16_t) ((100.0 * (float) soc)/(float) battery_capacity));
      esp_now_send(mcc_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
    }
  
  //D7 battery temp Vmon1
  if ((rtc.getEpoch() - vmon[0].mostrecent) <= old_message_time)  
    sprintf(outgoing_data.message, "WD7=S%1d %2dC", State, vmon[0].battemp);
  else 
    sprintf(outgoing_data.message, "WD7=S%1d?%2dC", State, vmon[0].battemp);
  esp_now_send(mcc_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
}


  
