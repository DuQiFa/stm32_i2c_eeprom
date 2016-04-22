#ifndef SERIAL_PORT_HPP_INCLUDED
#define SERIAL_PORT_HPP_INCLUDED

#include <quan/stm32/serial_port.hpp>
#include <quan/stm32/gpio.hpp>

struct sp_info{
   typedef quan::stm32::usart1 usart;
   typedef quan::mcu::pin<quan::stm32::gpioa,9>    txo_pin;
   typedef quan::mcu::pin<quan::stm32::gpioa,10>   rxi_pin;
};

typedef quan::stm32::serial_port<
   sp_info::usart,1000,1000,sp_info::txo_pin,sp_info::rxi_pin
> serial_port;

void panic(const char* str);


#endif // SERIAL_PORT_HPP_INCLUDED
