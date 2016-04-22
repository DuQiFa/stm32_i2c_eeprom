#ifndef QUAN_STM32_EEPROM_TEST_I2C_HPP_INCLUDED
#define QUAN_STM32_EEPROM_TEST_I2C_HPP_INCLUDED

extern "C" void DMA1_Stream4_IRQHandler() __attribute__ ( (interrupt ("IRQ")));
extern "C" void I2C3_EV_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void I2C3_ER_IRQHandler() __attribute__ ((interrupt ("IRQ")));

struct i2c{

   static void init();
   static bool is_busy() ;

   static bool get_bus();
   static bool release_bus();

   static void default_event_handler();
   static void default_error_handler();
   static void default_dma_handler();

   static void set_default_handlers();
   static void set_event_handler( void(*pfn_event)());
   static void set_error_handler( void(*pfn_event)());
   static void set_dma_handler( void(*pfn_event)());

   static void set_ack(bool b);
   static void set_start(bool b);
   static void enable_event_interrupts(bool b);
   static void enable_buffer_interrupts(bool b);
   static void enable_dma_bit(bool b);
   static void enable_dma_stream(bool b);
   static void set_dma_tx_buffer(uint8_t const* data, uint16_t numbytes);
   static void clear_dma_stream_flags();
   static void clear_dma_stream_tcif();
   static void clear_addr();
   static void set_stop(bool b);

   static uint16_t get_sr1();
   static uint16_t get_sr2();

   static bool get_sr2_msl();
   static bool get_sr1_btf() ;
   static bool get_sr1_txe() ;
   static bool get_sr2_tra() ;
   static bool get_sr1_addr() ;
   static bool get_sr1_sb();
   static bool get_sr1_rxne();
   static bool get_sr1_stop();

   static void send_address(uint8_t address);
   static void send_data(uint8_t data);
   static uint8_t receive_data();
   static const char* get_error_string();

private:
   friend void ::DMA1_Stream4_IRQHandler() ;
   friend void ::I2C3_EV_IRQHandler() ;
   friend void ::I2C3_ER_IRQHandler();
   static void setup_tx_dma();
   static volatile bool m_bus_taken_token;
   i2c() = delete;
   i2c(i2c const & ) = delete;
   i2c& operator = (i2c&) = delete;

   static void (* volatile pfn_event_handler)();// = default_event_irq_handler;
   static void (* volatile pfn_error_handler)() ;// = default_error_irq_handler;
   static void (* volatile pfn_dma_handler)();// = default_dma_handler;

};

#endif // QUAN_STM32_EEPROM_TEST_I2C_HPP_INCLUDED
