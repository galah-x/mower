void reinit_NVM (void)
{
  Serial.println("Initializing NVM");
  
  // If tpInit is 'false', the key "nvsInit" does not yet exist therefore this
  //  must be our first-time run. We need to set up our Preferences namespace keys. So...
  mcoPrefs.begin("mcoPrefs", RW_MODE);       //  open it in RW mode.
  
  // The .begin() method created the "mcoPrefs" namespace and since this is our
  //  first-time run we will create
  //  our keys and store the initial "factory default" values.

  mcoPrefs.putBytes("cmtmac", default_cmt_mac, 6);  // mac address of tester
  mcoPrefs.putBytes("psumac", default_psu_mac, 6);  // mac address of power supply 
  mcoPrefs.putBytes("mccmac", default_mcc_mac, 6);  // mac address of power supply controller 
  mcoPrefs.putBytes("v1mac",  default_v1_mac, 6);  // mac address of vmon # 1
  mcoPrefs.putBytes("v2mac",  default_v2_mac, 6);  // mac address of vmon # 2
  mcoPrefs.putBytes("v3mac",  default_v3_mac, 6);  // mac address of vmon # 3
  mcoPrefs.putBytes("v4mac",  default_v4_mac, 6);  // mac address of vmon # 4
  mcoPrefs.putBytes("imonmac",default_imon_mac, 6);  // mac address of imon
  mcoPrefs.putFloat("adco", 0.0);            // current adc offset term
  mcoPrefs.putFloat("adcg", 1.0);            // current adc gain term
  mcoPrefs.putFloat("psug",  1.0 );          // cal term from local voltage to psu 
  mcoPrefs.putFloat("psuo",  0.0 );          // cal term from local voltage to psu 
  mcoPrefs.putFloat("icc", 10.0);            // initial charge current
  mcoPrefs.putFloat("icv", 55.2);            // initial charge voltage 13.8*4 = 55.2
  mcoPrefs.putFloat("tcc",  3.0);            // topoff charge current
  mcoPrefs.putFloat("tcv", 57.0);            // topoff charge voltage 14.0*4 = 56 57V=14.25/Battery or 3.56 per cell 
                /* Hmmm. Kings data on 
                 * https://www.4wdsupacentre.com.au/batteries/lithium/100ah-lithium/100ah-lithium-battery.html
                 * now says  "The BMS will also perform cell balancing at 3.5V per cell with 35mA of current."
                 * 3.5V per cell is 14V for the battery or 56V for the 4. So I need to be a bit above 14V per battery 
                 * The battery is rated for 14.4 (14.6?) max charge voltage, so 14.25 may be a better choice.
		 * also kings says 80% depth of discharge, so I may bump up to 20% */
                               
  mcoPrefs.putFloat("tc",  3.0);             // transition current
  mcoPrefs.putFloat("fc",  0.3);             // final current
  mcoPrefs.putFloat("maxbv", 14.3);          // max single battery voltage
  mcoPrefs.putFloat("minbv", 11.5);          // min single battery voltage
  mcoPrefs.putFloat("bbv",  13.8);           // start balancing battery above this voltage 3.45 * 4 =13.8 
  mcoPrefs.putFloat("bbt",  0.005);          // battery balance tolerance
  mcoPrefs.putLong("bsoc",  72000000);       // beep SOC, 20%
  mcoPrefs.putLong("bcap", 360000000);       // battery capacity in mAS  100AH = 360e6 maS 
  mcoPrefs.putULong("omt", 10);              // what defines an OLD message. in seconds.
  mcoPrefs.putBool("nvsInit", true);            // Create the "already initialized"
  //  key and store a value.
  // The "factory defaults" are created and stored so...
  mcoPrefs.end();                               // Close the namespace in RW mode and...
}


// regarding offsets..
// read the adc, add adv_offset, multiply the sum by the gain to get the real reading.
//   same for a voltage reading from the psu.
//   the inverse for setting the psu voltage.



void load_operational_params(void)
{

  mcoPrefs.begin("mcoPrefs", RO_MODE);       // Open our namespace (or create it
                                               //  if it doesn't exist) in RO mode.
   // Retrieve the operational parameters from the namespace
   //  and save them into their run-time variables.

   mcoPrefs.getBytes("psumac", psu_mac, 6);             // load power supply mac
   mcoPrefs.getBytes("mccmac", mcc_mac, 6);             // load mco/cmt supply mac
   mcoPrefs.getBytes("cmtmac", cmt_mac, 6);             // load mco/cmt supply mac
   mcoPrefs.getBytes("v1mac", vmon[0].mac, 6);          // load vmon 1 mac
   mcoPrefs.getBytes("v2mac", vmon[1].mac, 6);          // load vmon  mac
   mcoPrefs.getBytes("v3mac", vmon[2].mac, 6);          // load vmon  mac
   mcoPrefs.getBytes("v4mac", vmon[3].mac, 6);          // load vmon  mac
   mcoPrefs.getBytes("imonmac", imon.mac, 6);           // load imon  mac
   adc_offset = mcoPrefs.getFloat("adco");              // current adc offset term
   adc_gain = mcoPrefs.getFloat("adcg");                // current adc gain term
   psu_offset = mcoPrefs.getFloat("psuo");              // cal term from local voltage to psu 
   psu_gain = mcoPrefs.getFloat("psug");                // cal term from local voltage to psu 
   initial_charge_current = mcoPrefs.getFloat("icc");   // initial charge current
   initial_charge_voltage = mcoPrefs.getFloat("icv");   // initial charge voltage 13.8*4 = 55.2
   topoff_charge_current = mcoPrefs.getFloat("tcc");    // topoff charge current
   topoff_charge_voltage = mcoPrefs.getFloat("tcv");    // topoff charge voltage 14.0*4 = 56
   transition_current = mcoPrefs.getFloat("tc");        // transition current
   cutoff_current = mcoPrefs.getFloat("fc");            // final current
   max_battery_voltage = mcoPrefs.getFloat("maxbv");    // max single battery voltage
   min_battery_voltage = mcoPrefs.getFloat("minbv");    // min single battery voltage
   batt_balance_voltage = mcoPrefs.getFloat("bbv");     // start balancing a battery above this 3.45 * 4 =13.8
   batt_balance_tol_voltage = mcoPrefs.getFloat("bbt"); // battery balance tolerance
   beep_SOC = mcoPrefs.getLong("bsoc");                 // beep SOC, 20%
   battery_capacity = mcoPrefs.getLong("bcap");         // battery capacity in mAS  100AH = 360e6 maS 
   fbattery_capacity = (float) battery_capacity;
   old_message_time  = mcoPrefs.getULong("omt");        // what defines an OLD message. in seconds.
   // All done. Last run state (or the factory default) is now restored.
   mcoPrefs.end();                                      // Close our preferences namespace.
}
