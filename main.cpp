

/*
   eeprom test using a 24LC0128 eeprom
*/

#include <quan/stm32/i2c_port.hpp>
#include <quan/stm32/millis.hpp>
#include <quan/conversion/itoa.hpp>

#include "serial_port.hpp"
#include "i2c.hpp"
#include "led.hpp"
#include <cstring>

extern "C" void setup();

bool i2c_write( uint16_t address, uint8_t const * data, uint32_t len);
bool i2c_write_byte( uint16_t address, uint8_t data);
bool i2c_read( uint16_t address, uint8_t * data, uint32_t len);

void write_delay()
{
   auto now = quan::stm32::millis();
   while ( (quan::stm32::millis() - now) < quan::time_<uint32_t>::ms{100U}){;}
}

int main()
{
   setup();
   auto elapsed = quan::stm32::millis();

   char data_out[] = {"bo....ob"};
   i2c_write(5U,(uint8_t const*)data_out,8);

   write_delay();

   char data_in[8] = {"-------"};

   bool result = i2c_read(5U,(uint8_t*)data_in,8);

   serial_port::write("read ");
   serial_port::write( result ? "succeeded\n" : "failed\n");
   serial_port::write("got ");
   
   serial_port::write(data_in,8);
   serial_port::write("\n");

   for ( uint8_t i = 0; i < 8; ++i){
      char buf[20];
      quan::itoasc(static_cast<uint32_t>(data_in[i]),buf,10);
      serial_port::write(buf);
      serial_port::write("\n");
   }
// blink forever to check we havent crashed
   for (;;) {
      auto const now = quan::stm32::millis();
      if ( (now - elapsed) >= quan::time_<uint32_t>::ms{1000U}){
         elapsed = now;
         led::complement();
      }
   }

   return 0;
}

namespace {

   // 24LC128 eeprom address (low 3 bits dependent on pins)
   static constexpr uint8_t eeprom_addr = 0b10100000;
/*
   bool event_test(bool (*pf)(void), bool wanted_result, quan::time_<uint32_t>::ms timeout, const char* text)
   {
      quan::time_<uint32_t>::ms start = quan::stm32::millis();
      while (pf() != wanted_result){
         if ((quan::stm32::millis() - start) >= timeout){
            serial_port::write(text);
            serial_port::write("\n");
            return false;
         }
      }
      return true;
   }
*/
   bool event(bool (*pf)(void), bool wanted_result, quan::time_<uint32_t>::ms timeout)
   {
      quan::time_<uint32_t>::ms start = quan::stm32::millis();
      while (pf() != wanted_result){
         if ((quan::stm32::millis() - start) >= timeout){
            return false;
         }
      }
      return true;
   }
   bool error(const char* text)
   {
      serial_port::write(text);
      serial_port::write("\n");
      return false;
   }

   void prf(const char* text, bool val)
   {
      serial_port::write(text);
      serial_port::write(" = ");
      serial_port::write(val?"true":"false");
      serial_port::write("\n");
   }

   bool i2c_send_data(uint8_t const * data, uint32_t len)
   {
      typedef quan::time_<uint32_t>::ms ms;
      for ( uint8_t i = 0; i < len; ++i){
         if (event(i2c::get_sr1_txe,true,ms{200U})){
            i2c::send_data( data[i]);
         }else {
            return error("no txe");
         }
      }
      return true;
   }

   bool i2c_common(uint32_t address)
   {
      typedef quan::time_<uint32_t>::ms ms;

      if (event(i2c::is_busy,false,ms{200U})){
         i2c::set_start(true);
      }else {
         return error("i2c busy forever");
      }
      
      if(event(i2c::get_sr1_sb,true,ms{200U})){ 
         i2c::send_address(eeprom_addr );
      }else{
         return error("couldnt get sb 1");
      }

      if (event(i2c::get_sr1_addr,true,ms{200U})){
         (void) i2c::get_sr2_msl();  
      }else{
         return error("no addr set 1");
      }

      uint8_t buf[2] = {
         static_cast<uint8_t>((address && 0xFF00) >> 8),
         static_cast<uint8_t>(address && 0xFF)
      };
      return i2c_send_data(buf,2);
   }
}

// address
bool i2c_read(uint16_t address, uint8_t* data, uint32_t len)
{
   typedef quan::time_<uint32_t>::ms ms;

   if (!i2c_common(address)){
      return false;
   }

   if ( event(i2c::get_sr1_btf,true,ms{200U})){
      i2c::set_start(true);
   }else{
      return error("no btf");
   }
   
   if (event(i2c::get_sr1_sb,true,ms{200U})){
      i2c::send_address(eeprom_addr | 1);
   }else{
      return error("couldnt get sb 2");
   }

   if (event(i2c::get_sr1_addr,true,ms{200U})){
      (void) i2c::get_sr2_msl();
   }else{
      return error("no addr set 2");
   }

   uint32_t bytes_left = len;
   i2c::enable_ack(true);

   for ( uint32_t i = 0; i < len; ++i){
      if ( event(i2c::get_sr1_rxne,true,ms{200U})){
         if ( bytes_left == 1){
            i2c::enable_ack(false);
            i2c::set_stop(true);
         }
         data[i] = i2c::receive_data();
         --bytes_left;
      }else{
         return error("no rxne");
      }
   }
   if(event(i2c::get_stop,true,ms{200U})){
      serial_port::write("data read ok\n");
      return true;
   }else{
      error("couldnt set stop");
      return false;
   }
}


bool i2c_write( uint16_t address, uint8_t const * data, uint32_t len)
{
   typedef quan::time_<uint32_t>::ms ms;

   if (!i2c_common(address)){return false;}

   if (!i2c_send_data(data,len)){return false;}

   if (event(i2c::get_sr1_btf,true,ms{200U})){
      i2c::set_stop(true);
   }else{
      return error("no btf");
   }
   if(event(i2c::get_stop,true,ms{200U})){
      serial_port::write("data written ok\n");
      return true;
   }else{
      return error("data write failed");
   }
}




