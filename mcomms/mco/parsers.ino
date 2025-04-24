//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'parsers for mco  Time-stamp: "2025-04-23 18:23:05 john"';

// this is the parsers for app to run the mower comms controller
// for the Ryobi RM480e mower.
// use tools -> board ->  ESP32 Dev module 


// this is for S0, debug
void parse_buf (char * in_buf)
{
  // Z eraZe NV memory
  // Rf Read and print Field,
  //    where Field could be    lower case for current stuff, upper case for NVM
  //          i battery current 
  //          v[1234] vmon state
  //          t current time
  //          c scaled charger voltage, current, enable
  //          s SOC
  //          st state
  //          p show polling state
  //          M Show macs
  //          V=version
  //          I=initial charge current, voltage
  //          B=balance above per battery voltage, tolerance, maxV
  //          E BEep below this 
  //          T=topoff voltage, current, final current
  //          G[VI] adc gain   current=local adc voltage=DPM8624
  //          P[VI] psu offset voltage =local adc voltage=DPM8624
  //          S     SOC total capacity in mA.S i guess
  //          u     see update periods
  // Wf Field,
  //    where Field could be (volatile stuff in lower case)
  //          Mx mac    WMP=<12 ascii hex digits>  mac for Psu or Mcc or Cmt
  //          bn=[01] switch balance on board 1..4 to given level. this is sent by a poll, and updated by loop() 
  //          cv=f    charger voltage   set psu V
  //          ci=f    charger current   set psu I
  //          ce=b   charger enable    set pse e
  //          p[vp]=b    polling  enable 1 or disable 0  vmon polling.
  //          s=n     set SOC  0..100
  //          d=display WD0=string   write display line 0..7 with given string
  //          u[scd]=%d   set update period of state machine/currentSOC/display/psu/beep/vmon to d seconds (ms last 3).
  //
  //          II=f   initial charge current
  //          IV=f   initial charge voltage
  //          BA=f   balance above per battery voltage
  //          BT=f   balance tolerance voltage
  //          BM=f   max single battery voltage
  //          WE=f  BEep below this where f = %f%%  
  //          TV=f   topoff voltage
  //          TI=f   topoff current
  //          TE=f   topoff final current to stop charging
  //          G[VI]=f adc gain   current=local adc voltage=DPM8624
  //          O[VI]=f adc offset current=local adc voltage=DPM8624
  //          S=d
  //          V[1234]E=%4d set cal write enable for specified vmon.  vmon we resets at start or on nth voltage read.
  //          V[1234]G=f write adc Gain for specified vmon. It must be write enabled
  //          V[1234]O=f write adc offset for specified vmon. It must be write enabled
  //                        
  int8_t  address;
  
  uint8_t  cmd;
  uint8_t  field;
  uint8_t  field2;
  uint8_t  field3;
  int      value;
  uint8_t  mvalue[6]; // mac address
  float    fvalue;
  const uint8_t out_buf_len = 20;
  char     out_buf[out_buf_len];
  int      match ;
  int      i;
  int32_t soc;
  uint32_t time1;
  uint32_t time2;
  
  cmd = in_buf[0];
  
  switch (cmd)
    {
    case 'Z':
      reinit_NVM();
      load_operational_params();
      break;
      
    case 'R':
      switch (in_buf[1])
	{
	case 'i':
	  Serial.printf("Imon current=%1.2fA updated %1d seconds ago msgs r=%1d errs=%1d\n",
			imon.current,          rtc.getEpoch() - imon.mostrecent,
			imon.received,         imon.protocol_err); 
	
	  break;
      
	case 'v':
	  address = ((in_buf[2] -1) & 0x03)  ;   // convert ascii 1..4 ie 0x31 to 0x34 to 0..3  31->2 34->1  
	  if ((in_buf[3] == 'G') || (in_buf[3] == 'O'))
	    {
	      // read gain or offset. this does not come from the polled read values, must add a read cmd.
	      // I'll add a read to the write queue, and ensure the callback parser knows about them
	      snprintf(out_buf, out_buf_len, "R%c\n", in_buf[3]);    // generate q cmd string for a write balance  
	      write_vmon_wq(address, out_buf, strlen(out_buf)+1);
	      // wait till next q slot. or next 4 if the target may be napping
	      wait_next_q_slot(4);
	  
	      // sendq
	      send_q_msg();
	  
	      // wait till next q slot
	      wait_next_q_slot(4);
	  
	      // read result.
	      if (in_buf[3] == 'G')
		Serial.printf("vmon %d G=%f\n", address+1, vmon[address].gain);
	      else
		Serial.printf("vmon %d O=%f\n", address+1, vmon[address].offset);
	      // NB, as loop is single threaded and loop runs the poll q, must read twice!
	    }
	  else
	    {
	      Serial.printf("Vmon %d voltage=%1.3f bt=%1dC rt=%1dC b=%d be=%d updated %1d seconds ago msgs s=%1d r=%1d errs=%1d\n",
			    address+1, vmon[address].volt,
			    vmon[address].battemp, vmon[address].restemp,
			    vmon[address].balance, vmon[address].act_balance, 
			    rtc.getEpoch() - vmon[address].mostrecent,
			    vmon[address].sent, vmon[address].received, vmon[address].protocol_err); 
	    }
	  break;
	case 't' :
	  Serial.printf("current time %d %s\n", rtc.getEpoch(), rtc.getTime()); 
	  break;
	case 'c' :
	  Serial.printf("charger act_voltage=%1.2f act_current=%1.3f enable=%d updated  %d seconds ago \n",
			charger.voltage, charger.current, charger.enable, rtc.getEpoch() - charger.mostrecent); 
	  Serial.printf("charger set_voltage=%1.2f set_current=%1.3f\n",
			charger.set_v, charger.set_i); 
	  break;
	case 's' : // FIXME enum CState{} State; 

	  if (in_buf[2] == 't')
	    Serial.printf("State=%d   (Mowing, Charger_not_init, CC, CV, Done) \n", State);
	  else 
	    Serial.printf("SOC=%dmAS %2.1f%%\n", get_SOC(0), 100.0 * (float) get_SOC(0) / (float)battery_capacity);
	  break;
	case 'M':
	  Serial.printf("MCO_MAC=%02x%02x%02x%02x%02x%02x PSU=%02x%02x%02x%02x%02x%02x MCC=%02x%02x%02x%02x%02x%02x CMT=%02x%02x%02x%02x%02x%02x\n",
			baseMac[0],baseMac[1],baseMac[2],baseMac[3],baseMac[4],baseMac[5],
			psu_mac[0],psu_mac[1],psu_mac[2],psu_mac[3],psu_mac[4],psu_mac[5],
			mcc_mac[0],mcc_mac[1],mcc_mac[2],mcc_mac[3],mcc_mac[4],mcc_mac[5],
			cmt_mac[0],cmt_mac[1],cmt_mac[2],cmt_mac[3],cmt_mac[4],cmt_mac[5]);
	  Serial.printf("VMON1_MAC=%02x%02x%02x%02x%02x%02x V2=%02x%02x%02x%02x%02x%02x V3=%02x%02x%02x%02x%02x%02x V4=%02x%02x%02x%02x%02x%02x\n",
			vmon[0].mac[0], vmon[0].mac[1], vmon[0].mac[2], vmon[0].mac[3], vmon[0].mac[4], vmon[0].mac[5], 
			vmon[1].mac[0], vmon[1].mac[1], vmon[1].mac[2], vmon[1].mac[3], vmon[1].mac[4], vmon[1].mac[5], 
			vmon[2].mac[0], vmon[2].mac[1], vmon[2].mac[2], vmon[2].mac[3], vmon[2].mac[4], vmon[2].mac[5], 
			vmon[3].mac[0], vmon[3].mac[1], vmon[3].mac[2], vmon[3].mac[3], vmon[3].mac[4], vmon[3].mac[5]);
	  Serial.printf("IMON_MAC=%02x%02x%02x%02x%02x%02x\n",
			imon.mac[0],imon.mac[1],imon.mac[2],imon.mac[3],imon.mac[4],imon.mac[5]);
      
      
	  break;
	case 'I' :
	  Serial.printf("Initial charge voltage=%2.2fV current=%2.3fA\n",
			initial_charge_voltage, initial_charge_current);
	  break;
	case 'T' :
	  Serial.printf("Topoff charge voltage=%2.2fV current=%2.3fA\n",
			topoff_charge_voltage, topoff_charge_current);
	  break;
	case 'G' :
	  Serial.printf("Current adc gain=%f adc_offset=%f\n",
			adc_gain, adc_offset);
	  break;
	case 'P' :
	  Serial.printf("Power supply voltage gain=%f offset=%f\n",
			psu_gain, psu_offset);
	  break;
	case 'B' :
	  Serial.printf("Balance battery above %2.3fV Tolerance=%1.3fV Max voltage=%2.3fV Min voltage=%2.3fV \n",
			batt_balance_voltage, batt_balance_tol_voltage, max_battery_voltage, min_battery_voltage);
	  break;
	case 'E' :
	  Serial.printf("Beep at or below SOC=%d mAS %1.1f%%\n",
			beep_SOC, 100.0*beep_SOC/fbattery_capacity);
	  break;
	case 'S' :
	  Serial.printf("Battery capacity=%ld mAS %ldAH\n",
			battery_capacity, battery_capacity/3600000);
	  break;
	case 'V':
	  Serial.println(version);
	  break;
	case 'u':
	  Serial.printf("update periods vmon_poll=%dms beep_on=%dms psu_read=%dms\n", vmon_period_millis, beep_on_period, psu_period_millis);
	  Serial.printf("display_update=%ds SOC_update=%ds state_mc=%ds\n", display_update_period, current_update_period, charger_state_update_period);
	  break;
	case 'p':
	  Serial.printf("polling suspended for vmon=%d psu=%d\n", suspend_vmon_polling, suspend_psu_polling);
	  Serial.printf("last poll time vmon=%d psu=%d\n", last_vmon_time, last_psu_time);
	  Serial.printf("psu sent=%d psu received=%d\n", charger.sent, charger.received);
	  Serial.printf("vmon[0] sent=%d vmon[0] received=%d\n", vmon[0].sent, vmon[0].received);
	  break;
	}
      break;

    case 'W':
      switch (in_buf[1]) {
      case 'b' :
	match =  sscanf(in_buf, "%c%c%c=%d", &cmd, &field, &field2, &value);
	if (match == 4)
	  {
	    field2 = (field2 -1) & 0x03; // map 1..4 to 0..3
	    vmon[field2].balance = value;
	  }
	break;

      case 'd':
	memcpy(&incoming_data.message, in_buf, strlen(in_buf));
	esp_now_send(mcc_mac, (uint8_t *) &incoming_data, sizeof(incoming_data));
	break;
      
      case  'c' :
	match =  sscanf(in_buf, "%c%c%c=%f", &cmd, &field,&field2, &fvalue);
	switch (field2)
	  {
	  case 'v' :
	    set_psu_v(fvalue);
	    break;
	  case 'i' :
	    set_psu_i(fvalue);
	    break;
	  case 'e' :
	    match =  sscanf(in_buf, "%c%c%c=%d", &cmd, &field,&field2, &value);
	    set_psu_e(value);
	    break;
	  }
	break;
      
      case 'p' :
	if (in_buf[2] == 'v')
	  {
	    if (in_buf[4] == '1')
	      suspend_vmon_polling = false; // enable polling ie don't suspend polling
	    else
	      suspend_vmon_polling = true;
	  }
	else if (in_buf[2] == 'p')
	  {
	    if (in_buf[4] == '1')
	      suspend_psu_polling = false; // enable polling ie don't suspend polling
	    else
	      suspend_psu_polling = true;
	    break;
	  }
	
      case 's' :
	match =  sscanf(in_buf, "%c%c=%d", &cmd, &field, &value);
	soc  = (float) value * (float) battery_capacity / 100.0;
	write_SOC(0, soc );
	break;
      case  'I' :
	match =  sscanf(in_buf, "%c%c%c=%f", &cmd, &field,&field2, &fvalue);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'V') 
	  mcoPrefs.putFloat("icv", fvalue);
	else if (field2 == 'I') 
	  mcoPrefs.putFloat("icc", fvalue);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
      case 'M':  // WMP=MACADDRESS for the guy to respond to
	match =  sscanf(in_buf, "%c%c%c=%2x%2x%2x%2x%2x%2x", &cmd, &field,&field2,
			&mvalue[0],&mvalue[1],&mvalue[2],&mvalue[3],&mvalue[4],&mvalue[5]);
      
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'P') 
	  mcoPrefs.putBytes("psumac", mvalue, 6);
	else if (field2 == 'M') 
	  mcoPrefs.putBytes("mccmac", mvalue, 6);
	else if (field2 == 'C') 
	  mcoPrefs.putBytes("cmtmac", mvalue, 6);
	else if (field2 == 'I') 
	  mcoPrefs.putBytes("imonmac", mvalue, 6);
	match =  sscanf(in_buf, "%c%c%c%c=%2x%2x%2x%2x%2x%2x", &cmd, &field, &field2, &field3,
			&mvalue[0],&mvalue[1],&mvalue[2],&mvalue[3],&mvalue[4],&mvalue[5]);
	if (field2 == 'V')
	  {
	    if (field3 == '1')
	      mcoPrefs.putBytes("v1mac", mvalue, 6);
	    else if (field3 == '2') 
	      mcoPrefs.putBytes("v2mac", mvalue, 6);
	    else if (field3 == '3') 
	      mcoPrefs.putBytes("v3mac", mvalue, 6);
	    else if (field3 == '4') 
	      mcoPrefs.putBytes("v4mac", mvalue, 6);
	  }
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;

      case  'T' :
	match =  sscanf(in_buf, "%c%c%c=%f", &cmd, &field,&field2, &fvalue);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'V') 
	  mcoPrefs.putFloat("tcv", fvalue);
	else if (field2 == 'I') 
	  mcoPrefs.putFloat("tcc", fvalue);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
      case  'E' :
	match =  sscanf(in_buf, "%c%c=%f", &cmd, &field, &fvalue);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	mcoPrefs.putLong("bsoc", (int32_t) fvalue);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
      case  'B' :
	match =  sscanf(in_buf, "%c%c%c=%f", &cmd, &field,&field2, &fvalue);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'A') 
	  mcoPrefs.putFloat("bbv", fvalue);
	else if (field2 == 'T') 
	  mcoPrefs.putFloat("bbt", fvalue);
	else if (field2 == 'X') 
	  mcoPrefs.putFloat("maxbv", fvalue);
	else if (field2 == 'N') 
	  mcoPrefs.putFloat("minbv", fvalue);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
      case  'G' :
	match =  sscanf(in_buf, "%c%c%c=%f", &cmd, &field,&field2, &fvalue);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'V') 
	  mcoPrefs.putFloat("psug", fvalue);
	else if (field2 == 'I') 
	  mcoPrefs.putFloat("adcg", fvalue);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;

      case  'O' :
	match =  sscanf(in_buf, "%c%c%c=%f", &cmd, &field,&field2, &fvalue);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'V') 
	  mcoPrefs.putFloat("psuo", fvalue);
	else if (field2 == 'I') 
	  mcoPrefs.putFloat("adco", fvalue);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;

      case  'S' :
	match =  sscanf(in_buf, "%c%c=%f", &cmd, &field, &fvalue);
	mcoPrefs.begin("mcoPrefs", RW_MODE);         // Open our namespace for write
	mcoPrefs.putLong("bcap", fvalue);
	mcoPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
      
      case 'V' :
	// doing Write Vmon #vmon we || Write Vmon #vmon G=f || Write Vmon #vmon O=f  to write cal terms to vmons  
	match =  sscanf(in_buf, "WV%c%c=%f", &field2, &cmd, &fvalue);
	if ((match == 3) && ((cmd == 'G') || (cmd == 'O'))) 
	  { // either WV2O=1.234  or WV1G=5.678
#ifdef DEBUG
	    Serial.printf("cal gain/offset write on vmon %c %c=%f\n", field2, cmd, fvalue);
#endif
	    field2 = (field2 -1) & 0x03; // map 0x31..0x34 to 0..3
	    snprintf(out_buf, out_buf_len, "W%c=%f\n", cmd, fvalue);    // generate q cmd string for a write   
	    write_vmon_wq(field2, out_buf, strlen(out_buf)+1);
	  }
	else
	  {
	    match =  sscanf(in_buf, "WV%cE=%4d", &field2, &value);
	    if ((match == 2))
	      { //  WV3E=%4d
#ifdef DEBUG
		Serial.printf("enable cal write on vmon %c inhibit napping %d times.\n", field2, value);
#endif
		field2 = (field2 -1) & 0x03; // map 0x31..0x34 to 0..3
		snprintf(out_buf, out_buf_len, "WW=%04d\n", value);    // generate q cmd string for a cal write enable
		write_vmon_wq(field2, out_buf, strlen(out_buf)+1);
	      }	    
	  }
	break;

      case  'u' :
	match =  sscanf(in_buf, "Wu%c=%d", &field, &value);
	if ((match == 2) && (field == 's'))
	  charger_state_update_period = value;
	else if ((match == 2) && (field == 'c'))
	  current_update_period = value;
	else if ((match == 2) && (field == 'd'))
	  display_update_period = value;
	else if ((match == 2) && (field == 'p'))
	  psu_period_millis = value;
	else if ((match == 2) && (field == 'b'))
	  beep_on_period = value;
	else if ((match == 2) && (field == 'v'))
	  vmon_period_millis = value;
	break;


      }      
      break;
      // end of 'W'
    }    
}


void parse_psu_wifi_buf (char * buf)
{
  charger.received++;
  if (buf[0]==':')
    {
      if ((buf[3]=='o') && (buf[4]=='k'))
	{ // typical write response.
	  //	  Serial.println("psu: ok");
	  charger.mostrecent = rtc.getEpoch();
	}
      else if (buf[3]=='r')
	{ // general read handler
	  int cmdaddr;
	  int value;
	  int matched;
	  matched = sscanf(buf, ":01r%2d=%d.", &cmdaddr, &value);
	  //	  Serial.printf("%s matched=%d addr=%d val=%d\n", buf, matched, cmdaddr, value);
	  if (matched==2)
	    {
	      if (cmdaddr == 30)
		//  Serial.printf("psu: meas_voltage=%d.%02dV\n", value/100, value % 100);  
		charger.voltage = psu_gain * (psu_offset+ (float) (value  / 100.0 ))  ;
	      else if (cmdaddr == 31)
		//	      Serial.printf("psu: meas_current=%d.%03dA\n", value/1000, value % 1000);  
		charger.current = (float) value / 1000.0;
	      else if (cmdaddr == 10)
		//	      Serial.printf("psu: meas_current=%d.%03dA\n", value/1000, value % 1000);  
		charger.set_v = (float) value / 100.0;
	      else if (cmdaddr == 11)
		//	      Serial.printf("psu: meas_current=%d.%03dA\n", value/1000, value % 1000);  
		charger.set_i = (float) value / 1000.0;
	      else if (cmdaddr == 12)
		if (value == 0)
		  charger.enable = 0;
	      // Serial.print("psu: status=CV\n");
		else 
		  charger.enable = 1;
	      charger.mostrecent = rtc.getEpoch();
	  }
	}
    }
}

void parse_vmon_wifi_buf (char * buf, uint8_t address)
{
  int       value;
  float     fvalue;
  int       matched;

  switch (buf[0])
    {
    case 'b' : // balance state (can wiggle when hot)
      matched = sscanf(buf, "b=%1d\n",  &value);
      if (matched==1)
	{
	  vmon[address].act_balance = value;
	  vmon[address].received++;
	}
      else if ((buf[1] == 'o') && (buf[2] == 'k'))
	  vmon[address].received++;
      else
	vmon[address].protocol_err++;
      break;

    case 'v' : // read voltage
      matched = sscanf(buf, "v=%f\n",  &fvalue);
      if (matched==1)
	{
	  vmon[address].volt = fvalue;
#ifdef MEAS_PERF
	  vmon_polltime[address].resp = millis();
	  Serial.printf("vmon %d serial voltage poll took %d milliseconds\n", address+1,
			vmon_polltime[address].resp - vmon_polltime[address].issue);
#endif
	  vmon[address].received++;
	  vmon[address].mostrecent = rtc.getEpoch();
	}
      else 
	vmon[address].protocol_err++;
      break;
      
    case 'r' : // read resistor temp
      matched = sscanf(buf, "r=%dC\n",  &value);
      if (matched==1)
	{
	  vmon[address].restemp = value;
	  vmon[address].received++;
	}
      else 
	vmon[address].protocol_err++;
      break;
      
    case 's' : // read battery temp
      matched = sscanf(buf, "s=%dC\n",  &value);
      if (matched==1)
	{
	  vmon[address].battemp = value;
	  vmon[address].received++;
	}
      else 
	vmon[address].protocol_err++;
      break;

    case 'G' : // read adc gain
      matched = sscanf(buf, "G=%f\n",  &fvalue);
      if (matched==1)
	{
#ifdef DEBUG
	  Serial.printf("read vmon %d adc_gain=%f\n", address, fvalue);
#endif
	  vmon[address].gain = fvalue;
	  // vmon[address].received++;
	}
      else 
	vmon[address].protocol_err++;
      break;

    case 'O' : // read adc gain
      matched = sscanf(buf, "O=%f\n",  &fvalue);
      if (matched==1)
	{
#ifdef DEBUG
	  Serial.printf("read vmon %d adc_offset=%f\n", address, fvalue);
#endif
	  vmon[address].offset = fvalue;
	  // vmon[address].received++;
	}
      else 
	vmon[address].protocol_err++;
      break;

    default:
      vmon[address].protocol_err++;
      break;
    }
}
void parse_imon_wifi_buf (char * buf)
{
  float     fvalue;
  int       matched;

  matched = sscanf(buf, "I=%f",  &fvalue);
  if (matched==1)
    {
      imon.current = fvalue;
      imon.mostrecent = rtc.getEpoch();
      imon.received++;
    }
  else
    {
      imon.protocol_err++;
    }      
}

