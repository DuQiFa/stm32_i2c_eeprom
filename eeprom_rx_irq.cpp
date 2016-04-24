
#include <stm32f4xx.h>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
//#include <quan/stm32/i2c/typedefs.hpp>
//#include <quan/stm32/i2c/detail/get_irq_number.hpp>
#include <quan/stm32/millis.hpp>
#include "serial_port.hpp"
#include "i2c.hpp"
#include "led.hpp"
#include <quan/conversion/itoa.hpp>

namespace{

   struct i2c_eeprom_reader{

      

      // return true if apply ok
      // else leave in good state and return false
      // (call i2c::is_busy() first)
      static bool apply(uint8_t device_address,uint16_t data_address, uint8_t* data, uint16_t len)
      {
         // todo
         // check eeprom address < max
         // check data ptr not null
         // check len not 0
         // etc
         if (i2c::get_bus()){
            m_p_data = data;
            m_data_length = len;
            m_data_address[0] = static_cast<uint8_t>((data_address & 0xFF00) >> 8U);
            m_data_address[1] = static_cast<uint8_t>(data_address & 0xFF);
            m_device_address = device_address ; // write address
           
            i2c::set_error_handler(on_error);
            i2c::set_event_handler(on_start_sent);

            i2c::enable_error_interrupts(true);
            i2c::enable_event_interrupts(true);
            i2c::enable_buffer_interrupts(false);
            i2c::enable_ack_bit(true);
            i2c::enable_dma_bit(true);
            i2c::enable_dma_last_bit(true);
//
            i2c::set_dma_rx_buffer(m_p_data,m_data_length);
            i2c::clear_dma_rx_stream_flags();

            i2c::set_dma_rx_handler(on_dma_transfer_complete);
            i2c::enable_dma_rx_stream(true);

            i2c::request_start_condition();

            return true;
         }else{
            panic("couldnt get bus");
            return false;
         }    
      }
      
  private:

      // Start condition generated event . (EV5)
      // Clear by reading sr1 and
      // then writing dr with address of i2c device
      static void on_start_sent()
      {  // flags sr1.sb
         recorded_flags[0] = i2c::get_sr1();
         i2c::send_address(m_device_address); 
         i2c::set_event_handler(on_device_address_sent);
      }

      // device address sent event. EV6
      // Clear by reading SR1 followed by reading SR2.
      // then send first byte of data address
      static void on_device_address_sent()
      {  // flags sr2.[tra:busy:msl] sr1.[addr:txe]
         recorded_flags[1] = i2c::get_sr1();
         recorded_flags[1] |= (i2c::get_sr2() << 16U);
         i2c::send_data(m_data_address[0]);
         i2c::set_event_handler(on_data_address_hi_sent);
      }

      static void on_data_address_hi_sent()
      { 
          // flags sr2.[tra:busy:msl] sr1.[btf:txe]
          recorded_flags[2] = i2c::get_sr1();
          recorded_flags[2] |= (i2c::get_sr2() << 16U);
         
          i2c::send_data(m_data_address[1]);
          i2c::request_start_condition();
          i2c::set_event_handler(on_start2);
      }

      // btf on 2nd data address
      // restar
      static void on_start2()
      {
         // flags sr2.[tra:busy:msl] sr1.[btf:txe]
         recorded_flags[3] = i2c::get_sr1();
         recorded_flags[3] |= (i2c::get_sr2() << 16U);

         i2c::receive_data(); //clear the txe and btf flags
         i2c::set_event_handler(on_start3);
      }

      static void on_start3()
      {
         // flags sr1.sb
         recorded_flags[4] = i2c::get_sr1();
         
         i2c::send_address(m_device_address | 1);
         i2c::set_event_handler(on_device_read_address_sent);
      }
  
      // addr sent
      static void on_device_read_address_sent()
      {
          // flags sr2.[busy:msl] sr1.addr
          recorded_flags[5] = i2c::get_sr1();
          recorded_flags[5] |= (i2c::get_sr2() << 16U);
          i2c::enable_event_interrupts(false);
          i2c::set_event_handler(i2c::default_event_handler);
      }

//      static void on_rx()
//      {
//          
//          recorded_flags[6] = i2c::get_sr1();
//          recorded_flags[6] |= (i2c::get_sr2() << 16U);
//
//          i2c::enable_event_interrupts(false);
//
//      }
//
//      static void on_rx1()
//      {
//
//          recorded_flags[7] = i2c::get_sr1();
//          recorded_flags[7] |= (i2c::get_sr2() << 16U);
//      }

      // dma handler called when last byte of dma data recieved
      // disable dma and enable i2c event irq's to get btf
      // update the event handler
      static void on_dma_transfer_complete()
      {
        
         led::on();
         i2c::enable_dma_rx_stream(false);
         i2c::enable_dma_bit(false);
         i2c::enable_dma_last_bit(false);
         i2c::clear_dma_rx_stream_tcif(); 
         i2c::request_stop_condition();
         i2c::set_default_handlers();
         i2c::release_bus();
      }

      static void on_error()
      {
         panic ("i2c error");
      }

      static uint8_t* m_p_data;
      static uint16_t m_data_length;
      static uint8_t  m_data_address[2]; // could do for dma in dma avail memmory
      static uint8_t  m_device_address;
public:
      static volatile uint32_t flag_ptr;
      static volatile uint32_t recorded_flags[100];
   };

   volatile uint32_t i2c_eeprom_reader::recorded_flags[] = {0U};  
   volatile uint32_t i2c_eeprom_reader::flag_ptr  = 0U;
   uint8_t*          i2c_eeprom_reader::m_p_data = nullptr;
   uint16_t          i2c_eeprom_reader::m_data_length = 0U;
   uint8_t           i2c_eeprom_reader::m_data_address[] = {0U,0U};
   uint8_t           i2c_eeprom_reader::m_device_address = 0U;
} // ~namespace

/*
   test function
*/
constexpr uint16_t numbytes = 8U;
char data_in[numbytes] = {"-------"};  // the data to write n.b in dma available memory

bool eeprom_rx_irq_test()
{
    static constexpr uint8_t eeprom_addr = 0b10100000;

    bool result = i2c_eeprom_reader::apply( eeprom_addr ,5U,(uint8_t*)data_in,8);
   
    auto now = quan::stm32::millis();
    typedef decltype(now) ms;

    while(i2c::is_busy() && ((quan::stm32::millis() - now) < ms{500U})){;}
 
    now = quan::stm32::millis();
    while ((quan::stm32::millis() - now) < ms{500U}) {;}
    if (i2c::is_busy()){
         panic("looks like irq read hung");

         uint32_t const dma_flags = DMA1->LISR & 0x3D0000;

         char buffer [20];
         quan::itoasc(dma_flags,buffer,16);
         serial_port::write("dma flags = 0x");
         serial_port::write(buffer);
         serial_port::write("\n");

         uint32_t ndata = DMA1_Stream2->NDTR;
         quan::itoasc(ndata,buffer,10);
         serial_port::write("ndtr = ");
         serial_port::write(buffer);
         serial_port::write("\n");
         
        // return false;
    }

    serial_port::write("flags\n");
    for ( uint32_t i = 0; i < 8; ++i){
         char buf[100];
         sprintf(buf,"flags[%u] = 0x%x\n",
            static_cast<unsigned>(i), 
            static_cast<unsigned>(i2c_eeprom_reader::recorded_flags[i])
         );
        serial_port::write(buf);
    }
    serial_port::write("read got ");
    serial_port::write(data_in,numbytes);
    serial_port::write("\n");

      // may not be ascii ...
      for ( uint8_t i = 0; i < numbytes; ++i){
         char buf[20];
         quan::itoasc(static_cast<uint32_t>(data_in[i]),buf,10);
         serial_port::write(buf);
         serial_port::write("\n");
      }
    
    return result;;
}
