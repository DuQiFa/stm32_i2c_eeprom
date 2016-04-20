
#include <quan/time.hpp>
#include <quan/frequency.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/i2c/typedefs.hpp>
#include <quan/stm32/f4/i2c/module_enable_disable.hpp>
#include <quan/stm32/sys_freq.hpp>
#include <quan/stm32/i2c/detail/get_irq_number.hpp>
#include <quan/stm32/millis.hpp>
#include "i2c.hpp"

#include "interrupt_priority.hpp"

namespace {
   // PA8  I2C3_SCL
   // PC9  I2C3_SDA

   typedef quan::mcu::pin<quan::stm32::gpioa,8> scl_pin;
   typedef quan::mcu::pin<quan::stm32::gpioc,9> sda_pin;

   typedef quan::stm32::i2c3  i2c_type;
}


void i2c::init()
{
   // set mode

      quan::stm32::module_enable<i2c_type>();
      quan::stm32::module_enable<scl_pin::port_type>();
      quan::stm32::module_enable<sda_pin::port_type>();
   // TODO add check they are valid pins
      quan::stm32::apply<
         scl_pin
         ,quan::stm32::gpio::mode::af4 // all i2c pins are this af mode on F4
         ,quan::stm32::gpio::otype::open_drain
         ,quan::stm32::gpio::pupd::none         //  Use external pullup 5V tolerant pins
         ,quan::stm32::gpio::ospeed::slow 
      >();

      quan::stm32::apply<
         sda_pin
         ,quan::stm32::gpio::mode::af4 // all i2c pins are this af mode on F4
         ,quan::stm32::gpio::otype::open_drain
         ,quan::stm32::gpio::pupd::none          //  Use external pullup 5V tolerant pins
         ,quan::stm32::gpio::ospeed::slow 
      >();

     // set the bus  speed
      // all i2c on apb1?
      uint32_t constexpr apb1_freq = quan::stm32::get_bus_frequency<quan::stm32::detail::apb1>();
      static_assert(apb1_freq == 42000000,"unexpected freq");
      // set clock speed for 42 MHz APB1 bus
      uint32_t constexpr freq = apb1_freq / 1000000;
      static_assert(apb1_freq % 1000000 == 0, "invalid freq");
      uint32_t const temp_cr2 = i2c_type::get()->cr2.get() & ~0b111111;
      i2c_type::get()->cr2.set( temp_cr2 | freq);
//set clock for slow freq
      quan::frequency_<int32_t>::Hz constexpr i2c_freq_slow{100000}; // 100 kHz
      uint32_t ccr_reg_val = apb1_freq / (i2c_freq_slow.numeric_value() * 2);

      uint32_t const temp_ccr = i2c_type::get()->ccr.get() & ~0xFFF;
      i2c_type::get()->ccr.set(temp_ccr | ccr_reg_val);

      constexpr quan::time::ns max_scl_risetime{1000};
      uint32_t constexpr trise_reg_val 
         = static_cast<uint32_t>(max_scl_risetime * quan::frequency::Hz{apb1_freq} + 1.f);
      uint32_t const temp_trise = i2c_type::get()->trise.get() & ~0b111111;
      i2c_type::get()->trise.set(temp_trise | trise_reg_val);

    //  NVIC_EnableIRQ(quan::stm32::i2c::detail::get_event_irq_number<i2c_type>::value);
     // NVIC_EnableIRQ(quan::stm32::i2c::detail::get_error_irq_number<i2c_type>::value);
   /*
    Program the I2C_CR1 register to enable the peripheral
   */
      uint8_t constexpr i2c_cr1_pe_bit = 0;
      i2c_type::get()->cr1.bb_setbit<i2c_cr1_pe_bit>();

}

   bool i2c::get_sr2_msl(){constexpr uint8_t sr2_msl = 0; return i2c_type::get()->sr2.bb_getbit<sr2_msl>();}
   bool i2c::get_sr1_btf() {constexpr uint8_t sr1_btf = 2;return i2c_type::get()->sr1.bb_getbit<sr1_btf>();}
   bool i2c::get_sr1_txe() {constexpr uint8_t sr1_txe = 7;return  i2c_type::get()->sr1.bb_getbit<sr1_txe>(); }
   bool i2c::get_sr2_tra() {constexpr uint8_t sr2_tra = 2;return i2c_type::get()->sr2.bb_getbit<sr2_tra>();}
   bool i2c::get_sr1_addr() {constexpr uint8_t sr1_addr = 1;return i2c_type::get()->sr1.bb_getbit<sr1_addr>();}
   bool i2c::get_sr1_sb() {constexpr uint8_t sr1_sb = 0;return i2c_type::get()->sr1.bb_getbit<sr1_sb>();}
   bool i2c::get_sr1_rxne() {constexpr uint8_t sr1_rxne = 6;return i2c_type::get()->sr1.bb_getbit<sr1_rxne>();}
// 
//bool i2c::master_mode_selected()
//{
//   // busy  and master selected and in start bit
//   return is_busy() && 
//      get_sr2_msl() && 
//      get_sr1_sb() ; 
//}
//
//bool i2c::master_transmitter_selected() 
//{
//   // /* BUSY, MSL, ADDR, TXE and TRA flags */
//
//   return is_busy() && 
//      get_sr2_msl() && 
//      get_sr2_tra()  &&
//      get_sr1_addr()  &&
//      get_sr1_txe() ;
//        
//}
//
/////* TRA, BUSY, MSL, TXE flags */
//bool i2c::master_byte_transmitting()
//{
//   return 
//      get_sr2_tra()  &&
//      is_busy() && 
//      get_sr2_msl() && 
//      get_sr1_txe() ;
//}
//
// /* TRA, BUSY, MSL, TXE and BTF flags */
//bool i2c::master_byte_transmitted() 
//{
//   return 
//      get_sr2_tra()  &&
//      is_busy() && 
//      get_sr2_msl() && 
//      get_sr1_txe() ;
//      get_sr1_btf() ;
//}

void i2c::send_address(uint8_t data)
{
   i2c_type::get()->dr = data;
}

void i2c::send_data(uint8_t data)
{
   i2c_type::get()->dr = data;
}

uint8_t i2c::receive_data()
{
   return static_cast<uint8_t>(i2c_type::get()->dr);
}

bool i2c::is_busy()
{
   constexpr uint8_t i2c_sr2_busy_bit = 1;
   return i2c_type::get()->sr2.bb_getbit<i2c_sr2_busy_bit>();
}

void i2c::set_start(bool b)
{
   constexpr uint8_t cr1_start_bit = 8;
   i2c_type::get()->cr1.bb_putbit<cr1_start_bit>(b);
}
void i2c::set_stop(bool b)
{
  constexpr uint8_t cr1_stop_bit =9;
  i2c_type::get()->cr1.bb_putbit<cr1_stop_bit>(b);
}

bool i2c::get_stop()
{
  constexpr uint8_t cr1_stop_bit =9;
  return i2c_type::get()->cr1.bb_getbit<cr1_stop_bit>();
}

#if 0
// see the f4 i2c ref man
void i2c::set_nack2(bool b)
{
   constexpr uint8_t cr1_pos_bit = 11;
   i2c_type::get()->cr1.bb_putbit<cr1_pos_bit>(b);
}
#endif

void i2c::enable_ack(bool b)
{
   uint8_t constexpr  i2c_cr1_ack_bit = 10;
   i2c_type::get()->cr1.bb_putbit<i2c_cr1_ack_bit>(b);
}

const char* i2c::get_error_string()
{
   return "error todo";
}

//extern "C" void I2C3_EV_IRQHandler() __attribute__ ((interrupt ("IRQ")));
//extern "C" void I2C3_EV_IRQHandler()
//{     
//   static_assert(std::is_same<i2c_type, quan::stm32::i2c3>::value,"incorrect port irq");
//  // i2c_port::handle_irq();
//}
//
//extern "C" void I2C3_ER_IRQHandler() __attribute__ ((interrupt ("IRQ")));
//extern "C" void I2C3_ER_IRQHandler()
//{
//   static_assert(std::is_same<i2c_type, quan::stm32::i2c3>::value,"incorrect port irq");
//   uint32_t const sr1 = i2c_type::get()->sr1.get();
//   i2c_type::get()->sr1.set(sr1 & 0xFF); 
//   //i2c_port::i2c_errno = i2c_port::errno_t::i2c_err_handler;
//}