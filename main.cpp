
#include <quan/stm32/i2c_port.hpp>
#include <quan/stm32/millis.hpp>
#include <quan/conversion/itoa.hpp>

#include "serial_port.hpp"
#include "led.hpp"
#include "millis.hpp"
extern "C" void setup();



int main()
{
   setup();

   auto elapsed = millis();
   char buffer[50];
   quan::itoasc(SystemCoreClock,buffer, 10);
   serial_port::write("core clock == ");
   serial_port::write(buffer);   
   serial_port::write("\n");

   for (;;)
   {
      auto const now = millis();
      if ( (now - elapsed) >= 1000U){
         elapsed = now;
         led::complement();
         quan::itoasc(elapsed,buffer, 10);
         serial_port::write(buffer);
         serial_port::write(" Hello World\n");
      }
   }
}



