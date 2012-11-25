#ifndef BLOCK_OSC_H__
#define BLOCK_OSC_H__

#include "../block.h"

namespace block {


/// OSC peripheral
class OSC: public Block
{
  static const ioptr_t IO_SIZE = 8;
  enum {
    IV_OSCF = 0,
    IV_COUNT
  };

 public:
  OSC(Device* dev);
  virtual ~OSC();

  virtual ioptr_t io_size() const { return IO_SIZE; }
  virtual ivnum_t iv_count() const { return IV_COUNT; }

  virtual uint8_t getIo(ioptr_t addr);
  virtual void setIo(ioptr_t addr, uint8_t v);
  virtual void reset();

  enum PLLSRC {
    PLLSRC_RC2M = 0,
    PLLSRC_RC32M = 2,
    PLLSRC_XOSC = 3,
  };

 private:
  union CTRL {
    uint8_t data;
    BitField<0> rc2men;
    BitField<2> rc32men;
    BitField<3> rc32ken;
    BitField<4> xoscen;
    BitField<5> pllen;
  } ctrl_;

  union STATUS {
    uint8_t data;
    BitField<0> rc2mrdy;
    BitField<2> rc32mrdy;
    BitField<3> rc32krdy;
    BitField<4> xoscrdy;
    BitField<5> pllrdy;
  } status_;

  union XOSCCTRL {
    uint8_t data;
    BitField<0,4> xoscsel_;
    BitField<5,1> x32kplm_;
    BitField<6,2> frqrange_;
  } xoscctrl_;

  union XOSCFAIL {
    uint8_t data;
    BitField<0> xoscfden;
    BitField<1> xoscfdif;
  } xoscfail_;

  uint8_t rc32kcal_;

  PLLSRC pllsrc_;
  uint8_t pllfac_;

  union DFLLCTRL {
    uint8_t data;
    BitField<0> rc2mcref;
    BitField<1> rc32mcref;
  } dfllctrl_;

};


}

#endif
