
#include <stm32f4xx.h>
#include <quan/stm32/i2c/typedefs.hpp>
#include <quan/stm32/i2c/detail/get_irq_number.hpp>
#include <quan/stm32/millis.hpp>
#include "serial_port.hpp"
#include "i2c.hpp"
#include "led.hpp"
#include <quan/conversion/itoa.hpp>

namespace{

   struct i2c_eeprom_writer{

      // return true if apply ok
      // else leave in good state and return false
      // (call i2c::is_busy() first)
      static bool apply(uint8_t device_address,uint16_t data_address, uint8_t const* data, uint16_t len)
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
            m_device_address = device_address;
           
            i2c::set_error_handler(on_error);
            i2c::set_event_handler(on_start_sent);

            i2c::enable_event_interrupts(true);
            i2c::enable_dma_bit(true);

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
      {
         i2c::get_sr1();
         i2c::send_address(m_device_address); 
         i2c::set_event_handler(on_device_address_sent);
      }

      // device address sent event. EV6
      // Clear by reading SR1 followed by reading SR2.
      // then send first byte of data address
      // want txe and point to next handler
      static void on_device_address_sent()
      {
         i2c::get_sr1();
         i2c::get_sr2();
         i2c::send_data(m_data_address[0]);
         i2c::enable_buffer_interrupts(true);
         i2c::set_event_handler(on_data_address_hi_sent);
      }

      // txe on first data address
      // send second byte of data address
      // update to next handler
      static void on_data_address_hi_sent()
      {
          i2c::send_data(m_data_address[1]);
          i2c::set_event_handler(on_data_address_lo_sent);
      }

      // txe on 2nd data address
      // disble i2c events
      // do final dma setup
      // and point dma handler to end of dma handler
      // start sending the data using dma
      static void on_data_address_lo_sent()
      {
          i2c::enable_event_interrupts(false);
          i2c::enable_buffer_interrupts(false);
          i2c::enable_dma_stream(false);
          i2c::set_dma_tx_handler(on_dma_transfer_complete);
          i2c::set_dma_tx_buffer(m_p_data,m_data_length);
          i2c::clear_dma_tx_stream_flags();
          i2c::enable_dma_stream(true);
      }

      // dma handler called when last byte of dma data sent
      // disable dma and enable i2c event irq's to get btf
      // update the event handler
      static void on_dma_transfer_complete()
      {
         i2c::enable_dma_stream(false);
         i2c::enable_dma_bit(false);
         i2c::clear_dma_tx_stream_tcif(); 
         i2c::enable_event_interrupts(true);
         i2c::set_event_handler(on_last_byte_transfer_complete);  
      }

      // btf at end of last byte transfer
      // request stop condition and clean up
      static void on_last_byte_transfer_complete()
      {
         i2c::enable_event_interrupts(false);
         i2c::request_stop_condition();
         i2c::set_default_handlers();
         i2c::release_bus();
      }

      static void on_error()
      {
         panic ("i2c error");
      }

      static uint8_t const * m_p_data;
      static uint16_t m_data_length;
      static uint8_t  m_data_address[2]; // could do for dma in dma avail memmory
      static uint8_t  m_device_address;
   };

   uint8_t const *   i2c_eeprom_writer::m_p_data = nullptr;
   uint16_t          i2c_eeprom_writer::m_data_length = 0U;
   uint8_t           i2c_eeprom_writer::m_data_address[] = {0U,0U};
   uint8_t           i2c_eeprom_writer::m_device_address = 0U;
} // ~namespace

/*
   test function
*/
char data_out[] = {"healthy"};  // the data to write n.b in dma available memory

bool eeprom_tx_irq_test()
{
    static constexpr uint8_t eeprom_addr = 0b10100000;
    i2c_eeprom_writer::apply( eeprom_addr ,5U,(uint8_t const*)data_out,8);
    
    // at this point still busy
    // 
    auto now = quan::stm32::millis();
    typedef decltype(now) ms;
    while(i2c::is_busy() && ((quan::stm32::millis() - now) < ms{500U})){;}
 
    if (i2c::is_busy()){
         panic("looks like irq write hung");

         uint32_t const dma_flags = DMA1->HISR & 0x3F;

         char buffer [20];
         quan::itoasc(dma_flags,buffer,16);
         serial_port::write("dma flags = 0x");
         serial_port::write(buffer);
         serial_port::write("\n");

         uint32_t ndata = DMA1_Stream4->NDTR;
         quan::itoasc(ndata,buffer,10);
         serial_port::write("ndtr = ");
         serial_port::write(buffer);
         serial_port::write("\n");
         
         return false;
    }
   // i2c::enable_dma_stream(false);
    now = quan::stm32::millis();
    while( (quan::stm32::millis() - now) < ms{6U}){;}
    return true;
}
