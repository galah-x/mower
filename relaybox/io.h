
#ifndef __KEYPAD_H__
#define __KEYPAD_H__



/* IOs to control the relay voltage select IOs, and the current drawing IO */
const uint8_t I1pin = 13;
const uint8_t B1pin = 18;  
const uint8_t B2pin = 4;
const uint8_t B3pin = 16;
const uint8_t B4pin = 17; 
const uint8_t B5pin = 19;

// dont use 0 - a pwm out at boot
// dont use 1 - txout
// dont use 2 - led
// dont use 3 - rx
// dont use 5 - pwm at boot
// dont use 6 .. 11 connected to internal SPI flash
// dont use 12 - strapping pin
// dont use 14 - pwm at boot
// dont use 15 - pwm at boot

// 4, 13, 16..19  21..23 25..27  32..33 ok 
#endif
