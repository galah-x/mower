//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'mco low level functions  Time-stamp: "2025-04-11 08:19:09 john"';

/***** OK here are the subroutines   *************/

void send_q_msg ( void)
{ 	  
  memcpy(outgoing_data.message, vq[0].cmd,  cmdlen);
  esp_now_send(vmon[vq[0].board].mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
  vmon[vq[0].board].sent++;     // note another message sent to this vmon 
  vq[0].cmd_avail = 0;            // mark cmd queue consumed. 
}

void wait_next_q_slot (uint8_t slots)
{
  uint8_t i;
  for (i=0; i<slots; i++)
    {
      while  (millis()  <= (last_vmon_time + vmon_period_millis))
	{
	  delay(5);
	}
      last_vmon_time = millis();
    }
}


// write a message to the vmon write queue.  As vmons are polled continuously, this queue mechanism interrupts
// polling, using the next poll slot to send this message to the specified vmon.  polling resumes without
// skipping a poll in the subsequent poll slot. So polling just gets a tad delayed. 
//
//  uint not ascii vmon address  0..3
void write_vmon_wq (      uint8_t vmon, char * message, uint8_t msglen)
{
  // check nobody else is writing a record
  while (vq[0].qclaimed == 1)
	delay(10);
  // check there is a queue slot available. its a single entry queue right now.
  while (vq[0].cmd_avail == 1)
	delay(10);
  // ok, queue position is available. write the command to the queue
  vq[0].qclaimed == 1;
  memcpy(vq[0].cmd,  message, msglen);
  vq[0].board = vmon;                            // write q board address
  vq[0].cmd_avail = 1;
  vq[0].qclaimed = 0;
}


void set_psu_v(float volt)
{
  suspend_psu_polling=true;
  delay(30);
  // implement inverse psu gain trim based on observed accuracy at 50V
  // ie set to something that should produce 50V when asked to set to 50V
  sprintf(outgoing_data.message, ":01w10=%04d,\n", (int32_t) (100.0 * ((volt/psu_gain) -psu_offset)));
  // Serial.printf(" volt=%f  made %s g=%f o=%f\n", volt, outgoing_data.message, psu_gain, psu_offset);
  esp_now_send(psu_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
  charger.sent++;
  delay(30);
  suspend_psu_polling=false;
}

void set_psu_i(float current)
{
  suspend_psu_polling=true;
  delay(30);

  sprintf(outgoing_data.message, ":01w11=%05d,\n", (int32_t ) (1000.0 * current));
  //  Serial.printf("set_psu_i got %f and created message %s", current, outgoing_data.message);

  esp_now_send(psu_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
  charger.sent++;
  delay(30);
  suspend_psu_polling=false;
}

void set_psu_e(int value)
{
  char out_buf[20];
  suspend_psu_polling=true;
  delay(30);
  sprintf(outgoing_data.message, ":01w12=%1d,\n",  value);
  esp_now_send(psu_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
  charger.sent++;
  delay(30);
  suspend_psu_polling=false;
}

uint32_t get_SOC (uint16_t address)
{
  uint32_t retval;
  //  retval  = fram.read32(address);
  retval  = (uint32_t) (fram.read(address*4) << 24)  +  (uint32_t) (fram.read(address*4+1) << 16)
    + (uint32_t) ( fram.read(address*4+2) << 8)      + (uint32_t) fram.read(address*4+3);     
  return retval;
}

void write_SOC (uint16_t address, int32_t data)
{ // write all 3 fram addresses
  write_fram_SOC (address , data);
  write_fram_SOC (address+1 , data);
  write_fram_SOC (address+2 , data);
}

void write_fram_SOC (uint16_t address, int32_t data)
{
  digitalWrite(fram_wp_pin, LOW);
  // fram.write32(address, data);
  fram.write(address*4,   (uint8_t) (data >>24));
  fram.write(address*4+1,  (uint8_t) (data >>16));
  fram.write(address*4+2, (uint8_t) (data >>8 ));
  fram.write(address*4+3, (uint8_t) (data ));
  digitalWrite(fram_wp_pin, HIGH);
}
