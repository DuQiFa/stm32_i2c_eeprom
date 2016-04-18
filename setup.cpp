

#include "serial_port.hpp"
#include "interrupt_priority.hpp"
//#include <quan/stm32/millis.hpp>
#include "led.hpp"

namespace {

   void setup_systick()
   {
      SysTick_Config(SystemCoreClock / 1000);
      NVIC_SetPriority(SysTick_IRQn,15);
      
   }

   void setup_serial_port()
   {
    serial_port::init();
    
    serial_port::set_irq_priority(interrupt_priority::serial_port);
   }
}

extern "C" void setup()
{

    setup_systick();
    setup_serial_port();
    led::setup();

}

namespace {
uint32_t systick_value  = 0;
}

extern "C" void Systick_Handler() __attribute__ ((interrupt ("IRQ")));
extern "C" void SysTick_Handler()
{ 
   ++systick_value;
}

uint32_t millis()
{
   return systick_value;
}