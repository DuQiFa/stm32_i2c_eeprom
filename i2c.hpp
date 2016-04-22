#ifndef QUAN_STM32_EEPROM_TEST_I2C_HPP_INCLUDED
#define QUAN_STM32_EEPROM_TEST_I2C_HPP_INCLUDED

struct i2c{

   static void init();
   static bool is_busy() ;

   static bool get_bus();
   static bool release_bus();

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
      static void setup_tx_dma();
      static volatile bool m_bus_taken_token;
      i2c() = delete;
      i2c(i2c const & ) = delete;
      i2c& operator = (i2c&) = delete;

};

#endif // QUAN_STM32_EEPROM_TEST_I2C_HPP_INCLUDED
