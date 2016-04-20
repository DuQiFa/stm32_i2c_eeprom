
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

 //  char data[] = {"G234567"};
 //  i2c_write(5U,(uint8_t*)data,8);

 //  write_delay();

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

   bool test(bool (*pf)(void), bool wanted_result, quan::time_<uint32_t>::ms timeout, const char* text)
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
}

void prf(const char* text, bool val)
{
   serial_port::write(text);
   serial_port::write(" = ");
   serial_port::write(val?"true":"false");
   serial_port::write("\n");
}

// address
bool i2c_read(uint16_t address, uint8_t* data, uint32_t len)
{
#if 1
   typedef quan::time_<uint32_t>::ms ms;

   if (!test(i2c::is_busy,false,ms{200U},"i2c busy forever")){return false;}
   i2c::set_start(true);

   if (!test(i2c::get_sr1_sb,true,ms{200U},"couldnt get sb 1")){ return false;}
   i2c::send_address(eeprom_addr );

   if (!test(i2c::get_sr1_addr,true,ms{200U},"no addr set 1")){return false;}
   (void) i2c::get_sr2_msl();

   if (!test(i2c::get_sr1_txe,true,ms{200U},"no txe 1")){return false;}
   i2c::send_data( static_cast<uint8_t>((address && 0xFF00) >> 8));

   if ( !test(i2c::get_sr1_txe,true,ms{200U},"no txe 2")){return false;}
   i2c::send_data( static_cast<uint8_t>(address && 0xFF));

   if ( !test(i2c::get_sr1_txe,true,ms{200U},"no txe 3")){ return false;}
   if ( !test(i2c::get_sr1_btf,true,ms{200U},"no btf")){return false;}
   i2c::set_start(true);
   
   if ( !test(i2c::get_sr1_sb,true,ms{200U},"couldnt get sb 2")){return false;}
   i2c::send_address(eeprom_addr | 1 );

   if ( !test(i2c::get_sr1_addr,true,ms{200U},"no addr set 2")){return false;}
   (void) i2c::get_sr2_msl();

   uint32_t bytes_left = len;
   i2c::enable_ack(true);

   for ( uint32_t i = 0; i < len; ++i){
      if ( !test(i2c::get_sr1_rxne,true,ms{200U},"no rxne")){return false;}
      if ( bytes_left == 1){
         i2c::enable_ack(false);
         i2c::set_stop(true);
      }
      data[i] = i2c::receive_data();
      --bytes_left;
   }
   if(! test(i2c::get_stop,true,ms{200U},"couldnt set stop")){return false;}
   serial_port::write("data read ok\n");
   return true;
   
#else

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
   
   i2c::send_address(eeprom_addr );
   while (!i2c::get_sr1_addr() ){
      if ( ! timer()){
         serial_port::write("couldnt sel master mode tx\n");
         return false;
      }
   }
   (void) i2c::get_sr2_msl();
// eeprom address to read
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
   i2c::send_data( static_cast<uint8_t>(address && 0xFF));
   timer.reset();
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
   while (! i2c::get_sr1_btf() ){
      if ( ! timer()){
         serial_port::write("tx reg no btf \n");
         return false;
      }
   }
   i2c::set_start(true);
// wait for start
   while (!i2c::get_sr1_sb() ) {
      if ( ! timer()){
         serial_port::write("couldnt get sb\n");
         return false;
      }
   }
   timer.reset();
   

   i2c::send_address(eeprom_addr | 1 );
   while (!i2c::get_sr1_addr() ){
      if ( ! timer()){
         serial_port::write("couldnt sel master mode tx\n");
         return false;
      }
   }
   (void) i2c::get_sr2_msl();

   serial_port::write("got to rx data\n");

   uint32_t bytes_left = len;
   i2c::enable_ack(true);
   for ( uint32_t i = 0; i < len; ++i){
      while (! i2c::get_sr1_rxne() ){
         if ( ! timer()){
            serial_port::write("rx reg never empty\n");
            return false;
         }
      }
      timer.reset();
      if ( bytes_left == 1){
         i2c::enable_ack(false);
         i2c::set_stop(true);
      }
      
      data[i] =i2c::receive_data();
      --bytes_left;
     
      //serial_port::write("read byte\n");
   }
   
   while (i2c::get_stop() ) {
      if ( ! timer()){
         serial_port::write("couldnt set stop\n");
         return false;
      }
   }
   serial_port::write("data read ok\n");
   return true;

  #endif
}


bool i2c_write( uint16_t address, uint8_t const * data, uint32_t len)
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
   while (!i2c::get_sr1_sb() ) {  // read sb
      if ( ! timer()){
         serial_port::write("couldnt get sb\n");
         return false;
      }
   }
   timer.reset();
    
   i2c::send_address(eeprom_addr);  // write dr
   while (!i2c::get_sr1_addr() ){  // read sr1
      if ( ! timer()){
         serial_port::write("couldnt sel master mode tx\n");
         return false;
      }
   }
   (void) i2c::get_sr2_msl();  // read sr2
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




