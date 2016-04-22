

#include <quan/stm32/millis.hpp>
#include "serial_port.hpp"
#include "interrupt_priority.hpp"
#include "i2c.hpp"
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
    serial_port::set_baudrate<115200,true>();
    serial_port::set_irq_priority(interrupt_priority::serial_port);
   }
}

extern "C" void setup()
{
    setup_systick();
    setup_serial_port();
    led::setup();
    i2c::init();
}

volatile uint32_t quan::stm32::detail::systick_tick::current = 0U;

extern "C" void Systick_Handler() __attribute__ ((interrupt ("IRQ")));
extern "C" void SysTick_Handler()
{ 
   ++quan::stm32::detail::systick_tick::current;
}

