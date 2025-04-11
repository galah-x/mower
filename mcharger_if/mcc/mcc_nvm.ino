//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'mcc_nvm  Time-stamp: "2025-04-01 11:32:16


void reinit_NVM (void)
{
  Serial.println("Initializing NVM");
  
  // If tpInit is 'false', the key "nvsInit" does not yet exist therefore this
  //  must be our first-time run. We need to set up our Preferences namespace keys. So...
  mccPrefs.begin("mccPrefs", RW_MODE);       //  open it in RW mode.
  
  // The .begin() method created the "mccPrefs" namespace and since this is our
  //  first-time run we will create
  //  our keys and store the initial "factory default" values.

  mccPrefs.putBool("log", true);                // default is to log
  mccPrefs.putInt("fan_on_temp",  45);          // start fan at this temp if auto_fan
  mccPrefs.putInt("fan_off_temp", 35);          // stop fan at this temp if auto_fan
  mccPrefs.putBytes("psu_mac", default_mac, 6); // mac address of power supply 
  mccPrefs.putBytes("psu_mac", default_mac, 6); // mac address of power supply 
  mccPrefs.putString("ssid",   "unset");        // network for conventional wifi access 
  mccPrefs.putString("password", "unset");      // network password for conventional wifi access 
  mccPrefs.putString("password", "unset");      // network password for conventional wifi access 
  mccPrefs.putString("server", "unset");        // network server  for conventional wifi access 
  mccPrefs.putUInt("logport", 5681);             // network server tcp port for conventional wifi access 

  mccPrefs.putBool("nvsInit", true);            // Create the "already initialized"
  //  key and store a value.
  // The "factory defaults" are created and stored so...
  mccPrefs.end();                               // Close the namespace in RW mode and...
}


void load_operational_params(void)
{

  mccPrefs.begin("mccPrefs", RO_MODE);       // Open our namespace (or create it
                                               //  if it doesn't exist) in RO mode.
   // Retrieve the operational parameters from the namespace
   //  and save them into their run-time variables.

   fan_on_temp = mccPrefs.getInt("fan_on_temp");
   fan_off_temp = mccPrefs.getInt("fan_off_temp");
   logging = mccPrefs.getBool("log"); 
   mccPrefs.getBytes("psu_mac", psu_mac, 6);           // load power supply mac
   mccPrefs.getBytes("mco_mac", mco_mac, 6);           // load mco/cmt supply mac
   mccPrefs.getString("password", password, ssid_len);
   mccPrefs.getString("ssid", ssid, ssid_len);
   mccPrefs.getString("server", server, ssid_len);
   LogPort = mccPrefs.getUInt("logport");
   // All done. Last run state (or the factory default) is now restored.
   mccPrefs.end();                                      // Close our preferences namespace.
}

