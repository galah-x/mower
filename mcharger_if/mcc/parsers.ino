//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'mcc_parsers  Time-stamp: "2025-04-01 11:32:16


// command parser and serial processing
// file access
// 

void parse_buf (uint8_t * in_buf, uint8_t * out_buf, uint8_t out_buf_len)
{
  // Z eraZe NV memory
  // Rf Read and print Field,
  //    where Field could be Rv voltage xx.xxx V fixed format
  //                         Ri current xx.xxx A fixed format
  //                         RB=%x\n read block of msgbuflen from logfile at current seek pointer 
  //                         RE power state enabled
  //                         RH=templimit_fanon in integer degrees
  //                         RL=fan temp off again low
  //                         Rl=log state
  //                         RM print mac addresses
  //                         RS read logfile seek pointer aka length
  //                         RT=temperature
  //                         RV=version
  //                         RN read network password and ssid
  // Wf Field,
  //    where Field could be Wv voltage Wv=xx.xxx    fixed format set power supply voltage
  //                         Wi current Wi=xx.xxx    fixed format set psu current
  //                         WE enable  WE=1         turn on/off mower charge power
  //                         WH=fan on  WH=45        fan on temperature in decomal degrees
  //                         WL=fan off WL=23        fan off temperature in decomal degrees
  //                         Wl                      set log state
  //                         WD=display WD0=string   write display line 0..3 with given string
  //                         WMx mac    WMP=<12 ascii hex digits>  mac for Psu or Mco
  //                         WT=0\n                   truncate logfile 
  //                         WN[PS]=string\n              write Wifi password used for logfile upload
  //                         WS[AP]=string\n              write Wifi server and tcpip port for logfile upload

  uint8_t  cmd;
  uint8_t  field;
  uint8_t  field2;
  uint8_t  field3;
  uint32_t value;
  uint8_t  mvalue[6]; // mac address
  char     msg[ssid_len];   
  char     logmsg[30]; // for building logging messages
  int match ;
  cmd = in_buf[0];
  out_buf[0]=0;
  
  switch (cmd) {
  case 'Z':
    reinit_NVM();
    load_operational_params();
    break;
    
  case 'R':
    field = in_buf[1];
    switch (field)
      {
      case 'B': // logfile block, start address in hex
	match =  sscanf((char *) in_buf, "RB=%x", &value);
	// close_logfile();
	file = SD.open(logfile, FILE_READ);
	file.seek(value);
	file.read(out_buf, out_buf_len);
	close_logfile();
	break;

      case 'E':
	snprintf((char *)out_buf, out_buf_len, "Mower_power=%d\n", digitalRead(power48V_en_pin));
	break;

      case 'H':
	snprintf((char *)out_buf, out_buf_len, "Fan_on_at=%d degrees \n", fan_on_temp);
	break;
	
      case 'L':
	snprintf((char *)out_buf, out_buf_len, "Fan_off_at=%d degrees\n", fan_off_temp);
	break;

      case 'l':
	snprintf((char *)out_buf, out_buf_len, "log=%d\n", logging);
	break;
	
      case 'M':
	snprintf((char *)out_buf, out_buf_len, "MCC_MAC=%02x%02x%02x%02x%02x%02x PSU=%02x%02x%02x%02x%02x%02x MCO=%02x%02x%02x%02x%02x%02x\n",
		 baseMac[0],baseMac[1],baseMac[2],baseMac[3],baseMac[4],baseMac[5],
		 psu_mac[0],psu_mac[1],psu_mac[2],psu_mac[3],psu_mac[4],psu_mac[5],
		 mco_mac[0],mco_mac[1],mco_mac[2],mco_mac[3],mco_mac[4],mco_mac[5]
		 );
	break;

      case 'N' : //network
	snprintf((char *)out_buf, out_buf_len, "SSID=%s Password=%s Server=%s Port=%d\n", ssid, password, server, LogPort);
	break;
	  
      case 'S'	: // logfile length
	open_logfile(false,false);
	value=file.size();
	close_logfile();
	snprintf((char *)out_buf, out_buf_len, "Logfile=%d\n", value);
	break;
	
      case 'T':
	snprintf((char *)out_buf, out_buf_len, "Temp=%d degrees\n", hs_temperature);
	break;

      case 'V':
	snprintf((char *)out_buf, out_buf_len, "%s\n", version);
	break;
      }
      break;
    

  case 'W':
    // first case, WA=3     decimal integer up to ~5 sig figures
    match =  sscanf((char *)in_buf, "%c%c=%d", &cmd, &field, &value);
    
    switch (field)
      {
	// FIXME add v, i
      case 'D':
	// sscanf(in_buf, "%1c%10s", &cmd, &out_buf);  // whatever followed the 'm'
	// nope, I cannot make sscanf work for a char then a 0 terminated string. So do a manual scan
	// field2 is y,  for 0..3 for in_buf2 = 0..3
	// field3 is x   0 for inbuf2 == 0..3   12 for inbuf[2] == 4..7
	field2 = in_buf[2]  & 0x3; // ascii display row to write to [31..34] +=> 0123 after anding with 3
	field3 = (in_buf[2]) & 0x4; // ascii display column to write to [31..34] +=> 0123 after anding with
	if (field3 != 0 )
	  field3 = 13;    // support writing to second column.
	
	//   jump over WD0=
	int i;
	int cmax;
	if (field3 ==0)
	  cmax = 13;
	else
	  cmax = 7;
	int l;
	l=strlen((char *)in_buf);
	//	Serial.printf("strlen=%0d\n",l);
	// this adds trailing blanks to clear the field. dont want that in the log.
	snprintf(logmsg, 30, "%c %s", in_buf[2], in_buf+4);
	logger(logmsg);

	for (i=4; i < 4+cmax; i++)
	  {
	    out_buf[i-4] = in_buf[i];
	    if ((in_buf[i] == 0) || (i>=l))
	      {
		out_buf[i-4]= ' ';
	      }
	  }
	if (field3==0) 
	  out_buf[cmax-1]=' ';
	out_buf[cmax]=0;
	  
	if (! UI_owns_display)
	  {
	    lcd.setCursor(field3,field2);
	    lcd.print((char *)out_buf);
	  }
	out_buf[0]=0;
	break;
	
      case 'E':
	digitalWrite(power48V_en_pin, value);
	break;
	
      case 'H':
	mccPrefs.begin("mccPrefs", RW_MODE);         // Open our namespace for write
	mccPrefs.putInt("fan_on_temp", value);               
	mccPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
	
      case 'L':
	mccPrefs.begin("mccPrefs", RW_MODE);         // Open our namespace for write
	mccPrefs.putInt("fan_off_temp", value);               
	mccPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
	
      case 'l':
	mccPrefs.begin("mccPrefs", RW_MODE);         // Open our namespace for write
	mccPrefs.putBool("log", value);               
	mccPrefs.end();                              // Close the namespace
	load_operational_params();
	break;
	
      case 'M':  // WMP=MACADDRESS for the guy to respond to
	match =  sscanf((char *)in_buf, "%c%c%c=%2x%2x%2x%2x%2x%2x", &cmd, &field,&field2,
			&mvalue[0],&mvalue[1],&mvalue[2],&mvalue[3],&mvalue[4],&mvalue[5]);
	
	mccPrefs.begin("mccPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'P') 
	  mccPrefs.putBytes("psu_mac", mvalue, 6);
	if (field2 == 'M') 
	  mccPrefs.putBytes("mco_mac", mvalue, 6);
	mccPrefs.end();                              // Close the namespace
	load_operational_params();
	break;

      case 'N':  // WNP=passwordstring WNS=string 
	match =  sscanf((char *)in_buf, "WN%c=%s", &field2, &msg);
	mccPrefs.begin("mccPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'P') 
	  mccPrefs.putString("password", msg);
	if (field2 == 'S') 
	  mccPrefs.putString("ssid", msg);
	mccPrefs.end();                              // Close the namespace
	load_operational_params();
	break;

      case 'S':  // WSP=port WSA=serverstring 
	match =  sscanf((char *)in_buf, "WS%c", &field2);
	
	mccPrefs.begin("mccPrefs", RW_MODE);         // Open our namespace for write
	if (field2 == 'A') 
	  {
	    match =  sscanf((char *)in_buf, "WS%c=%s", &field2, &msg);
	    mccPrefs.putString("server", msg);
	  }
	if (field2 == 'P') 
	  {
	    match =  sscanf((char *)in_buf, "WS%c=%d", &field2, &value);
	    mccPrefs.putUInt("logport", value);
	  }
	mccPrefs.end();                              // Close the namespace
	load_operational_params();
	break;

      case 'T': // truncate logfile
#ifdef DEBUG
	Serial.println("doing log truncate");
#endif
	open_logfile(true, false);  // truncate logfile, dont justify.
	close_logfile();            
	break;
      }
    break;
    
    // end of 'W'
  }    
}

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
	  parse_buf(serial_buf, return_buf, 127);      // parse the buffer if at least one char in it.
	  Serial.printf((char *)return_buf);
	}
	serial_buf_pointer = 0;
      }
    }
}


void open_logfile (bool truncate, bool justify) 
{
  int i;
  if (truncate)
    {

      file = SD.open(logfile, FILE_WRITE);      // open at end of current file
      i= file.seek(0);
      file.close();
#ifdef DEBUG
      Serial.printf("file.seek returned %d\n", i);
#endif
    }
  openFile(SD, logfile);
  if (justify)
    justify_logfile();
}

void close_logfile (void)
{
  file.close();
}

// logger is called by the rx callback. I suspect that can interrupt itself.
// so discard any messages that happen while appending. mco sends 8 display line updates
// sequentially, so I suspect a message could well arrive while the sdcard is flushing.
// not sure if the callback is reentrant.
void logger (char*message)
{ if (logging)
    if (!writing_to_log)
      {
	writing_to_log=true;
	appendtoFile(message);
	writing_to_log=false;
      }
}

// open file for write. 
void openFile(fs::FS &fs, const char *path) {
  //  Serial.printf("Opening file: %s\n", path);
  file = fs.open(path, FILE_APPEND);      // open at end of current file
  if (!file) {
    Serial.println("Failed to open file for append");
  }
}

// works for  short messages, <100 each after timestamp added. Or at least shorter than a buf
void appendtoFile(char *message) {
  uint16_t buf_remaining;
  //  uint8_t *buf;    //  pointer to buf of uint8's
  // uint8_t *obuf;
  uint32_t msize;
  const uint8_t max_msg_size = 100;
  char     msg[max_msg_size];

  // prepend a timestamp
  snprintf(msg, max_msg_size, "%02d:%s %s\n", rtc.getDay()-1, rtc.getTime(), message); 
  buf_remaining = sdblksize - sdbuf_ptr;
  msize = strlen(msg);
#ifdef DEBUG
  Serial.printf("msg in=%s logmsg=%s", message, msg);
#endif
  if (buf_remaining > msize)   // if smaller than remaining buffer space
    { // copy msg into buffer and adjust pointer accordingly
      memcpy(sdbuf + sdbuf_ptr, msg, msize);
      sdbuf_ptr+= msize;
#ifdef DEBUG
      Serial.printf("appended %d to buffer, now %d\n", msize, sdbuf_ptr);
#endif
    }
  else if (buf_remaining == msize)       // if exact fit into remaining buffer space
    { 
      memcpy(sdbuf + sdbuf_ptr, msg, msize);
      sdbuf_ptr= 0;
      open_logfile(false,false);
      file.write(sdbuf, sdblksize);                   // write seems to take about 40ms.
      close_logfile();
#ifdef DEBUG
      Serial.printf("appended %d to buffer, now %d, wrote buffer\n", msize, sdbuf_ptr);
#endif
      
    }
  else // msg would overfill the buffer
    {
      memcpy(sdbuf + sdbuf_ptr, msg, buf_remaining);  // put first part of message that fits into buffer end
      open_logfile(false,false);
      file.write(sdbuf, sdblksize);                   // write out the complete buffer to SD
      close_logfile();
      sdbuf_ptr = msize-buf_remaining;                // set pointer to after the remaining fragment
#ifdef DEBUG
      Serial.printf("appended %d to buffer, wrote buffer, now %d \n", msize, sdbuf_ptr);
#endif
      memcpy(sdbuf, msg+buf_remaining, sdbuf_ptr);    // copy the remaining message fragment
    }                                                 // to the start of the buffer for next time
}

// between runs, I probably want to preserve existing logfile content. Which is (probably) the default.
// I want to keep subseqent runs sector+cluster aligned, so no RMW's.
// So I'm going to fill any remaining space in the existing file last partial cluster with blank lines.
// if you don't want that, you can empty the file.

void justify_logfile(void) {
  uint16_t buf_remaining;
  uint32_t msize;
  uint32_t next_cluster_boundary;
  int i;
  int j;
  const int linelength = 64; // length of blank line. MUST divide neatly into sdblksize!
  // first, how long is the current file.
  msize = file.size();
  Serial.printf("existing logfile is %d long\n", msize);
  next_cluster_boundary = 1 + (msize | (sdblksize - 1));
  Serial.printf("next cluster bdry is %d\n", next_cluster_boundary);
  buf_remaining = next_cluster_boundary - msize;
  if (buf_remaining == sdblksize)
    {
      Serial.println("don't need to justify, already on a boundary");
      return;
    }
  // so I have to do a justify.
  // fill the buffer with lines of spaces.
  for (i=0; i<sdblksize ; i+= linelength)
    {
      for (j=0; j<linelength-1 ; j++)
	sdbuf[i+j] = ' ';
      sdbuf[i+linelength-1]='\n';
    }
	  
  // now write as much of the end part of sdbuf as required to bring the file size to a cluster boundary 
  file.write(sdbuf + sdblksize - buf_remaining, buf_remaining);
  Serial.printf("writing blank buf from %d length %d to logfile\n", sdblksize - buf_remaining, buf_remaining);
}

void flush (void)
{
  file.write(sdbuf, sdbuf_ptr);
  sdbuf_ptr = 0;
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}
