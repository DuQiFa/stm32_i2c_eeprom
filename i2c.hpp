#ifndef QUAN_STM32_EEPROM_TEST_I2C_HPP_INCLUDED
#define QUAN_STM32_EEPROM_TEST_I2C_HPP_INCLUDED

struct i2c{

   static void init();
   static bool is_busy() ;
//   static bool master_mode_selected() ;
//   static bool master_transmitter_selected() ;
//   static bool master_byte_transmitting() ;
//   static bool master_byte_transmitted() ;
   // the 2 byte receive nack mullarkey
   static void set_nack2(bool b);
   static void enable_ack(bool b);
   static void set_start(bool b);
   static void set_stop(bool b);

   static bool get_sr2_msl();
   static bool get_sr1_btf() ;
   static bool get_sr1_txe() ;
   static bool get_sr2_tra() ;
   static bool get_sr1_addr() ;
   static bool get_sr1_sb();

   
   static bool get_stop();
   static void send_address(uint8_t address);
   static void send_data(uint8_t data);
   static uint8_t receive_data();
   static const char* get_error_string();

};

#endif // QUAN_STM32_EEPROM_TEST_I2C_HPP_INCLUDED
