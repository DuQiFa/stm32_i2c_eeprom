
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
#include "serial_port.hpp"

namespace {
   // PA8  I2C3_SCL
   // PC9  I2C3_SDA

   typedef quan::mcu::pin<quan::stm32::gpioa,8> scl_pin;
   typedef quan::mcu::pin<quan::stm32::gpioc,9> sda_pin;

   typedef quan::stm32::i2c3  i2c_type;
}

volatile bool i2c::m_bus_taken_token = false;
void (* volatile i2c::pfn_event_handler)()  = i2c::default_event_handler;
void (* volatile i2c::pfn_error_handler)()  = i2c::default_error_handler;
void (* volatile i2c::pfn_dma_handler)()    = i2c::default_dma_tx_handler;

bool i2c::get_bus()
{
   if (m_bus_taken_token || is_busy()){
      return false;
   }
   return m_bus_taken_token = true;
}

bool i2c::release_bus()
{
   m_bus_taken_token = false;
   return true;
}

void i2c::init()
{
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

   NVIC_EnableIRQ(quan::stm32::i2c::detail::get_event_irq_number<i2c_type>::value);
   NVIC_EnableIRQ(quan::stm32::i2c::detail::get_error_irq_number<i2c_type>::value);
   setup_tx_dma();
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
void i2c::enable_dma_bit(bool b){constexpr uint8_t cr2_dmaen = 11; i2c_type::get()->cr2.bb_putbit<cr2_dmaen>(b);}
//void i2c::clera_addr_bit() { constexpr uint8_t sr1_addr = 1; i2c_type::get()->sr1.bb_clearbit<sr1_addr>();}

void i2c::enable_dma_stream(bool b)
{
   if (b){
      DMA1_Stream4->CR |= (1U << 0U); // (EN)
   } else {
      DMA1_Stream4->CR &= ~(1U << 0U); // (EN)
   }
}
void i2c::clear_dma_tx_stream_tcif()
{
    DMA1->HIFCR |= (1 << 5) ; // (TCIF)
}

uint16_t i2c::get_sr1()
{
  return i2c_type::get()->sr1.get();
}
uint16_t i2c::get_sr2()
{
  return i2c_type::get()->sr2.get();
}

void i2c::set_dma_tx_buffer(uint8_t const* data, uint16_t numbytes)
{
   DMA1_Stream4->M0AR = (uint32_t)data; // buffer address
   DMA1_Stream4->NDTR = numbytes;           // num data
}

void i2c::clear_dma_tx_stream_flags()
{
    DMA1->HIFCR |= (0b111101 << 0U) ; // clear flags for Dma1 Stream 4
  //  DMA1->HIFCR &= ~(0b111101 << 0U) ; // clear flags for Dma1 Stream 4
}

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

void i2c::request_start_condition()
{
   constexpr uint8_t cr1_start_bit = 8;
   i2c_type::get()->cr1.bb_setbit<cr1_start_bit>();
}

void i2c::enable_event_interrupts(bool b)
{
   constexpr uint8_t cr2_itevten_bit = 9;
   i2c_type::get()->cr2.bb_putbit<cr2_itevten_bit>(b);
}

void i2c::enable_buffer_interrupts(bool b)
{
   constexpr uint8_t cr2_itbufen_bit = 9;
   i2c_type::get()->cr2.bb_putbit<cr2_itbufen_bit>(b);
}

void i2c::request_stop_condition()
{
  constexpr uint8_t cr1_stop_bit =9;
  i2c_type::get()->cr1.bb_setbit<cr1_stop_bit>();
}

bool i2c::get_sr1_stop()
{
  constexpr uint8_t cr1_stop_bit =9;
  return i2c_type::get()->cr1.bb_getbit<cr1_stop_bit>();
}

void i2c::setup_tx_dma()
{
   // i2c3 tx on DMA1_Stream4.CH3
   quan::stm32::rcc::get()->ahb1enr |= (1 << 21); // DMA stream 1
   for ( uint8_t i = 0; i < 20; ++i){
      asm volatile ("nop" : : :);
   }
   DMA_Stream_TypeDef* dma_stream = DMA1_Stream4;
   constexpr uint32_t  dma_channel = 3;
   constexpr uint32_t  dma_priority = 0b00; // low
   constexpr uint32_t  msize = 0b00; // 8 bit mem loc
   constexpr uint32_t  psize = 0b00; // 8 bit periph loc 
   dma_stream->CR = (dma_stream->CR & ~(0b111 << 25U)) | ( dma_channel << 25U); //(CHSEL) select channel
   dma_stream->CR = (dma_stream->CR & ~(0b11 << 16U)) | (dma_priority << 16U); // (PL) priority
   dma_stream->CR = (dma_stream->CR & ~(0b11 << 13U)) | (msize << 13U); // (MSIZE) 8 bit memory transfer
   dma_stream->CR = (dma_stream->CR & ~(0b11 << 11U)) | (psize << 11U); // (PSIZE) 16 bit transfer
   dma_stream->CR |= (1 << 10);// (MINC)
   dma_stream->CR &= ~(1 << 9);// (PINC)
   dma_stream->CR = (dma_stream->CR & ~(0b11 << 6U)) | (0b01 << 6U) ; // (DIR ) memory to peripheral
   dma_stream->CR |= ( 1 << 4) ; // (TCIE)
   dma_stream->PAR = (uint32_t)&I2C3->DR;  // periph addr
   NVIC_SetPriority(DMA1_Stream4_IRQn,15);  // low prio
   NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}


void i2c::enable_ack_bit(bool b)
{
   uint8_t constexpr  i2c_cr1_ack_bit = 10;
   i2c_type::get()->cr1.bb_putbit<i2c_cr1_ack_bit>(b);
}

const char* i2c::get_error_string()
{
   return "error todo";
}

void i2c::default_event_handler()
{
    panic("i2c event def called");
}

void i2c::default_error_handler()
{
   panic("i2c error def called");
}

void i2c::default_dma_tx_handler()
{
    panic("i2c dma def called");
}

void i2c::set_dma_tx_handler( void(*pfn_event)())
{
   pfn_dma_handler = pfn_event;
}

void i2c::set_event_handler( void(*pfn_event)())
{
   pfn_event_handler = pfn_event;
}

void i2c::set_error_handler( void(*pfn_event)())
{
   pfn_error_handler = pfn_event;
}

extern "C" void DMA1_Stream4_IRQHandler() __attribute__ ( (interrupt ("IRQ")));
extern "C" void DMA1_Stream4_IRQHandler()
{
   i2c::pfn_dma_handler();
}

extern "C" void I2C3_EV_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void I2C3_EV_IRQHandler()
{
   i2c::pfn_event_handler();
}

extern "C" void I2C3_ER_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void I2C3_ER_IRQHandler()
{
   i2c::pfn_error_handler();
}

void i2c::set_default_handlers()
{
   pfn_event_handler = default_event_handler;
   pfn_error_handler = default_error_handler;
   pfn_dma_handler   = default_dma_tx_handler;
}
