#ifndef BLOCK_USART_H__
#define BLOCK_USART_H__

#include "../block.h"

namespace block {


/// USART peripheral
class USART: public Block
{
  static const ioptr_t IO_SIZE = 8;
  enum {
    IV_RXC = 0,
    IV_DRE,
    IV_TXC,
    IV_COUNT
  };

 public:
  USART(Device* dev, const Instance<USART>& instance);
  virtual ~USART();

  virtual ioptr_t io_size() const { return IO_SIZE; }
  virtual ivnum_t iv_count() const { return IV_COUNT; }

  virtual uint8_t getIo(ioptr_t addr);
  virtual void setIo(ioptr_t addr, uint8_t v);
  virtual void executeIv(ivnum_t iv);
  virtual void reset();

 private:
  union STATUS {
    uint8_t data;
    BitField<0> rxb8;
    BitField<2> perr;
    BitField<3> bufofv;
    BitField<4> ferr;
    BitField<5> dreif;
    BitField<6> txcif;
    BitField<7> rxcif;
  } status_;

  IntLvl rxc_intlvl_;
  IntLvl dre_intlvl_;
  IntLvl txc_intlvl_;

  union CTRLB {
    uint8_t data;
    BitField<0> txb8;
    BitField<1> mpcm;
    BitField<2> clk2x;
    BitField<3> txen;
    BitField<4> rxen;
  } ctrlb_;

  union CTRLC {
    uint8_t data;
    BitField<0,3> chsize;
    BitField<3,1> sbmode;
    BitField<4,2> pmode;
    BitField<6,2> cmode;
    // master SPI mode
    BitField<1> ucpha;
    BitField<2> udord;
  } ctrlc_;

  uint16_t baudrate_;
  int8_t baudscale_;

  uint8_t rxb_;
  uint8_t txb_;
};


namespace instances {
  const Instance<USART> USARTC0 = { "USARTC0", 0x08A0,  25 };
  const Instance<USART> USARTC1 = { "USARTC1", 0x08B0,  28 };
  const Instance<USART> USARTD0 = { "USARTD0", 0x09A0,  88 };
  const Instance<USART> USARTD1 = { "USARTD1", 0x09B0,  91 };
  const Instance<USART> USARTE0 = { "USARTE0", 0x0AA0,  58 };
  const Instance<USART> USARTE1 = { "USARTE1", 0x0AB0,  61 };
  const Instance<USART> USARTF0 = { "USARTF0", 0x0BA0, 119 };
  const Instance<USART> USARTF1 = { "USARTF1", 0x0BB0, 122 };
}

}

#endif
