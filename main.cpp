
#include <quan/stm32/i2c_port.hpp>
#include <quan/stm32/millis.hpp>
#include <quan/conversion/itoa.hpp>

#include "serial_port.hpp"
#include "i2c.hpp"
#include "led.hpp"
#include <cstring>


extern "C" void setup();

bool i2c_write( uint16_t address, uint8_t * data, uint32_t len);

int main()
{
   setup();
   auto elapsed = quan::stm32::millis();

   char data[] = {"1234567"};
   i2c_write(5U,(uint8_t*)data,8);

   for (;;) {
      auto const now = quan::stm32::millis();
      if ( (now - elapsed) >= quan::time_<uint32_t>::ms{1000U}){
         elapsed = now;
         led::complement();
        // quan::itoasc(elapsed,buffer, 10);
        // serial_port::write(buffer);
        // serial_port::write(" Hello World\n");
      }
   }

   return 0;
}

namespace {

   static constexpr uint8_t eeprom_addr = 0b10100000;


   struct timer_t{
      timer_t (quan::time_<uint32_t>::ms const t) 
      : m_start{quan::stm32::millis()},m_timeout{t}{}

      void reset() { m_start = quan::stm32::millis();}
      bool operator()() const
      {
         return (quan::stm32::millis() - m_start) < m_timeout;
      }
      private:
      quan::time_<uint32_t>::ms m_start;  
      quan::time_<uint32_t>::ms const m_timeout;
   };
}

void prf(const char* text, bool val)
{
   serial_port::write(text);
   serial_port::write(" = ");
   serial_port::write(val?"true":"false");
   serial_port::write("\n");
}

bool i2c_write( uint16_t address, uint8_t * data, uint32_t len)
{
   timer_t timer = quan::time_<uint32_t>::ms{200U};
   
   while (i2c::is_busy()){
      if ( ! timer()){
         serial_port::write("i2c busy forever\n");
         return false;
      }
   }
   timer.reset();

   i2c::set_start(true);
   while (!i2c::get_sr1_sb() ) {
      if ( ! timer()){
         serial_port::write("couldnt get sb\n");
         return false;
      }
   }
   timer.reset();
    
   i2c::send_address(eeprom_addr);
   while (!i2c::get_sr1_addr() ){
      if ( ! timer()){
         serial_port::write("couldnt sel master mode tx\n");
         return false;
      }
   }
   (void) i2c::get_sr2_msl();
   timer.reset();

   while (! i2c::get_sr1_txe() ){
      if ( ! timer()){
         serial_port::write("tx reg never emptied 1\n");
         prf("tra",i2c::get_sr2_tra());
         prf("busy",i2c::is_busy());
         prf("msl",i2c::get_sr2_msl());
         prf("txe", i2c::get_sr1_txe());
         return false;
      }
   }
   timer.reset();
   i2c::send_data( static_cast<uint8_t>((address && 0xFF00) >> 8));
   while (! i2c::get_sr1_txe() ){
      if ( ! timer()){
         serial_port::write("tx reg never emptied 2\n");
         prf("tra",i2c::get_sr2_tra());
         prf("busy",i2c::is_busy());
         prf("msl",i2c::get_sr2_msl());
         prf("txe", i2c::get_sr1_txe());
         return false;
      }
   }
   timer.reset();
   i2c::send_data( static_cast<uint8_t>(address && 0xFF));
   timer.reset();
   for ( uint32_t i = 0; i < len; ++i){
      while (! i2c::get_sr1_txe() ){
         if ( ! timer()){
            serial_port::write("tx reg never emptied 3\n");
            return false;
         }
      }
      timer.reset();
      i2c::send_data(data[i]);
   }

   while (!i2c::get_sr1_btf()){
      if ( ! timer()){
         serial_port::write("couldnt end transmission\n");
         return false;
      }
   }
   timer.reset();

   i2c::set_stop(true);

   while (i2c::get_stop() ) {
      if ( ! timer()){
         serial_port::write("couldnt set stop\n");
         return false;
      }
   }
   serial_port::write("data written ok\n");
   return true;
}



