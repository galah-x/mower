//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'mcc menu_and_UI  Time-stamp: "2025-04-01 11:32:16




void do_encoder_and_UI (void)
{
  // look for encoder tick
  encoder->tick(); // just call tick() to check the state.
  int newPos = encoder->getPosition();
  
  if (enc_pos != newPos) {
    if  ( (int) encoder->getDirection() < 0)
      {
	enc_change = -1;
      }
    else
      {
	enc_change = 1;
      }    
    enc_pos = newPos;
  } else {
    enc_change = 0;
  }
  
  int newenc_pushn_state = digitalRead(enc_pushn_pin);
  
  if (enc_pushn_state  != newenc_pushn_state )
    delay(enc_press_debounce);
  if ((enc_pushn_state == 1) && (newenc_pushn_state == 0))
    {
      enc_press  = 1;
    }
  else
    enc_press = 0;
  enc_pushn_state = newenc_pushn_state;
  
  if (enc_press && !UI_owns_display)
    {
      UI_owns_display = true;
      menu=0;
      menu_line=0;
      show_menu=true;
      enc_press = 0;
    }
  if (UI_owns_display)
    {
      do_menu();
    }
}


void do_menu (void )
{
  if (menu == 0)
    do_menu_0();
  else if (menu == 1)
    do_menu_1();
}

void do_menu_0 (void)
{
  if (show_menu)
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(" go back");
      lcd.setCursor(0,1);
      lcd.print(" log menu");
      lcd.setCursor(0,menu_line);
      lcd.write(cursor);
      show_menu=false;
    }
  if (enc_press == true)
    {
      if (menu_line == 0)
	{
	  lcd.clear();
	  UI_owns_display=false;
	}
      if (menu_line == 1)
	{
	  menu=1;
	  menu_line = 0;
	  show_menu=1;
	}
    }
  if (enc_change == 1)
    if (menu_line <= 2)
      {
	lcd.setCursor(0,menu_line);
	lcd.print(' ');
	menu_line++;
	lcd.setCursor(0,menu_line);
	lcd.write(cursor);
      }
  if (enc_change == -1)
    if (menu_line >= 1)
      {
	lcd.setCursor(0,menu_line);
	lcd.print(' ');
	menu_line--;
	lcd.setCursor(0,menu_line);
	lcd.write(cursor);
      }
}
void do_menu_1 (void)  // log menu
{
  if (show_menu)
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(" go back");
      lcd.setCursor(0,1);
      if (logging==1)
	lcd.print(" turn off logging"); // change the menu item appropriately
      else 
	lcd.print(" turn on logging");
      lcd.setCursor(0,2);
      lcd.print(" empty logfile");
      lcd.setCursor(0,3);
      lcd.print(" send logfile");
      lcd.setCursor(0,menu_line);
      lcd.write(cursor);
      show_menu=false;
    }
  if (enc_press == true)
    {
      if (menu_line == 0)
	{
	  menu = 0;
	  menu_line = 0;
	  show_menu=1;
	}
      if (menu_line == 1) // switch logging state
	{
	  if (logging==1)
	    {
	      // turn off logging, and flush logfile
	      logging=0;
	      show_menu=1;
	      open_logfile(false, false);
	      flush();   // whatever is in the buffer
	      close_logfile();
	    }
	  else
	    {
	      // turn on logging
	      logging=1;
	      show_menu=1;
	      //open_logfile(false, true); // truncate, justify
	    }
	}
      else if (menu_line == 2) // truncate logfile
	{
	  open_logfile(true, false);
	  close_logfile();
	  sdbuf_ptr=0;
	}
      else if (menu_line == 3) // send logfile
	{
	send_logfile();
	}
    }
  if (enc_change == 1)
    if (menu_line <= 2)
      {
	lcd.setCursor(0,menu_line);
	lcd.print(' ');
	menu_line++;
	lcd.setCursor(0,menu_line);
	lcd.write(cursor);
      }
  if (enc_change == -1)
   if (menu_line >= 1)
      {
	lcd.setCursor(0,menu_line);
	lcd.print(' ');
	menu_line--;
	lcd.setCursor(0,menu_line);
	lcd.write(cursor);
      }
}
  

void send_logfile (void)
{
  if (espnow)
    {
      // stop espnow networking
      esp_wifi_stop();

      connectToWiFi(ssid, password);
    }
      
  // start up tcp
  while (!connected)
    {
      delay(200);
      Serial.print('.');
    }

  
  if (!client.connect(server, LogPort)) {
    Serial.println("Log Connection failed.");
  }
  
  Serial.println("Log Connection ok.");
  delay(100); // crashing somewhere around here
  
  // file = SD.open(logfile, O_RDONLY);  // this crashes the cpu :-(
  file = SD.open(logfile, FILE_READ);  
  delay(100);  
  int len = file.size(); // Read how big is file that we are opening
  Serial.printf("opened a logfile of len %d file=%d\n", len, file);
  int upto = 0;              // where am I up to
  int fragment;              // this transfer
  int wrote;
  int written = 0;
  const int chunk = msgbuflen/2;     // MUST be smaller then sizeof(msgbuf)
  
  Serial.print("sending logfile length ");
  Serial.println(len);
  delay(1000);  
  while (upto < len)
    {
      if ((len - upto) >= chunk)
	{
	  fragment = chunk;
	}
      else
	{
	  fragment = len - upto;
	}
      file.read(message_data.message, fragment);
      
      wrote = client.write((uint8_t *)message_data.message, fragment);
      upto = upto + fragment;
      written += wrote;
      //    Serial.printf("written %d bytes\n", upto);
    }
  file.close();
  
  Serial.print("sent length ");
  Serial.println(written);
  client.stop();
}


void connectToWiFi(const char * ssid, const char * pwd){

  Serial.println("Connecting to WiFi network: " + String(ssid));
  // delete old config
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);

  //register event handler
  WiFi.onEvent(WiFiEvent);

  WiFi.setHostname("MCC");
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
  while (!connected)
    {
      delay(100);
      Serial.println("waiting to connect");
    }
  Serial.println("connected");
}



//wifi event handler
void WiFiEvent(WiFiEvent_t event)
{
  switch(event)
    {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      //When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      connected = true;
      break;
      
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
    default: break;
    }
}



