

/*
   eeprom test using a 24LC0128 eeprom
   page write time specified as 5 ms max
*/

//#include <quan/stm32/i2c_port.hpp>
#include <quan/stm32/millis.hpp>
#include <quan/conversion/itoa.hpp>

#include "serial_port.hpp"
#include "i2c.hpp"


/*
   eeprom read write impl using busy waiting
*/

namespace {

   bool eeprom_write( uint16_t address, uint8_t const * data, uint32_t len);
   bool eeprom_read( uint16_t address, uint8_t * data, uint32_t len);

   void write_delay()
   {
      auto now = quan::stm32::millis();
      while ( (quan::stm32::millis() - now) < quan::time_<uint32_t>::ms{6U}){;}
   }

   void setup_busy_wait()
   {
      //i2c::init();
   }
}

bool eeprom_busy_wait_test()
{
   setup_busy_wait();

  // char data_out[] = {"qwertyu"}; // the data to write

  // if ( eeprom_write(5U,(uint8_t const*)data_out,8) ){

  //    write_delay();  // wait for eeprom write to complete

      // put some data here to check it is being overwritten
      constexpr uint8_t num = 12;
      char data_in[num] = {"-------"};

      bool result = eeprom_read(5U,(uint8_t*)data_in,num);

      serial_port::write("read ");
      serial_port::write( result ? "succeeded\n" : "failed\n");

      // look anyway to see if anything was read...

      serial_port::write("got ");
      
      serial_port::write(data_in,num);
      serial_port::write("\n");

      // may not be ascii ...
      for ( uint8_t i = 0; i < num; ++i){
         char buf[20];
         quan::itoasc(static_cast<uint32_t>(data_in[i]),buf,10);
         serial_port::write(buf);
         serial_port::write("\n");
      }
      return result;

//   }else{
//      serial_port::write("eeprom write failed\n");
//      return false;
//   }

   return true;
}

namespace {

   // 24LC128 eeprom address (low 3 bits dependent on pins)
   static constexpr uint8_t eeprom_addr = 0b10100000;

   // wait for a flag function for a specified time
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
   // simple error format for legibilty
   bool error(const char* text)
   {
      serial_port::write(text);
      serial_port::write("\n");
      return false;
   }

   // print some info about a bool value
   void prf(const char* text, bool val)
   {
      serial_port::write(text);
      serial_port::write(" = ");
      serial_port::write(val?"true":"false");
      serial_port::write("\n");
   }

   // Once everything is setup send the data to the eeprom
   bool eeprom_send_data(uint8_t const * data, uint32_t len)
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

   // Read and write have a common preample 
   bool eeprom_common(uint32_t address)
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

      // send the address in the eeprom to write to
      uint8_t buf[2] = {
         static_cast<uint8_t>((address && 0xFF00) >> 8),
         static_cast<uint8_t>(address && 0xFF)
      };
      return eeprom_send_data(buf,2);
   }


/*
read consists of sending "dummy" write command with just address, 
followed by read command
*/
   bool eeprom_read(uint16_t address, uint8_t* data, uint32_t len)
   {
      typedef quan::time_<uint32_t>::ms ms;

      if (!eeprom_common(address)){
         return false;
      }
      // after common part wait for btf which signifies 
      // can send "restart"
      if ( event(i2c::get_sr1_btf,true,ms{200U})){
         i2c::set_start(true);
      }else{
         return error("no btf");
      }
      // now send the read address
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
      i2c::set_ack(true);

      for ( uint32_t i = 0; i < len; ++i){
         if ( event(i2c::get_sr1_rxne,true,ms{200U})){
            if ( bytes_left == 1){
               i2c::set_ack(false);
               i2c::set_stop(true);
            }
            data[i] = i2c::receive_data();
            --bytes_left;
         }else{
            return error("no rxne");
         }
      }
      serial_port::write("data read ok\n");
      return true;
   }

   bool eeprom_write( uint16_t address, uint8_t const * data, uint32_t len)
   {
      typedef quan::time_<uint32_t>::ms ms;

      if (!eeprom_common(address)){return false;}

      if (!eeprom_send_data(data,len)){return false;}

      if (event(i2c::get_sr1_btf,true,ms{200U})){
         i2c::set_stop(true);
         serial_port::write("data written ok\n");
         return true;
      }else{
         return error("no btf");
      }
   }
}

