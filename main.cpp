
/*
   eeprom test using a 24LC0128 eeprom
   page write time specified as 5 ms max
   
*/

#include <quan/stm32/millis.hpp>
#include "serial_port.hpp"
#include "led.hpp"
#include "i2c.hpp"

extern "C" void setup();

bool eeprom_busy_wait_test();
bool eeprom_tx_irq_test();
bool eeprom_rx_irq_test();

int main()
{
   setup();

// choose function to use

   //eeprom_tx_irq_test();
   // write delay
//   auto now = quan::stm32::millis();
//   typedef decltype(now) ms;
//   while( (quan::stm32::millis() - now) < ms{6U}){;}

    eeprom_rx_irq_test();

  // i2c::init();
  // eeprom_busy_wait_test();

   // blink forever to check we havent crashed...
   auto elapsed = quan::stm32::millis();
   while ( (quan::stm32::millis() - elapsed) <  quan::time_<uint32_t>::ms{5000U}){;}
   serial_port::write("into blink loop\n");
   for (;;) {
      auto const now = quan::stm32::millis();
      if ( (now - elapsed) >= quan::time_<uint32_t>::ms{1000U}){
         elapsed = now;
         led::complement();
      }
   }

   return 0;
}
