
/*
   eeprom test using a 24LC0128 eeprom
   page write time specified as 5 ms max
   
*/

#include <quan/stm32/millis.hpp>
#include "led.hpp"

extern "C" void setup();

bool eeprom_busy_wait_test();
bool eeprom_irq_test();

int main()
{
   setup();

// choose function to use

   eeprom_irq_test();
   eeprom_busy_wait_test();

   // blink forever to check we havent crashed...
   auto elapsed = quan::stm32::millis();
   for (;;) {
      auto const now = quan::stm32::millis();
      if ( (now - elapsed) >= quan::time_<uint32_t>::ms{1000U}){
         elapsed = now;
         led::complement();
      }
   }

   return 0;
}
