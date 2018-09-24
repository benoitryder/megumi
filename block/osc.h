#ifndef BLOCK_OSC_H
#define BLOCK_OSC_H

#include "../block.h"

namespace block {


/// OSC peripheral
class OSC: public Block
{
  static constexpr ioptr_t IO_SIZE = 8;
  enum {
    IV_OSCF = 0,
    IV_COUNT
  };

 public:
  OSC(Device& dev);
  virtual ~OSC() = default;

  ioptr_t io_size() const override { return IO_SIZE; }
  ivnum_t iv_count() const override { return IV_COUNT; }

  uint8_t getIo(ioptr_t addr) override;
  void setIo(ioptr_t addr, uint8_t v) override;
  void reset() override;

  /// Return resulting PLL frequency
  unsigned int getPllFrequency() const;

  enum PLLSRC {
    PLLSRC_RC2M = 0,
    PLLSRC_RC32M = 2,
    PLLSRC_XOSC = 3,
  };

 private:
  union CTRL {
    uint8_t data;
    BitField<0> rc2men;
    BitField<1> rc32men;
    BitField<2> rc32ken;
    BitField<3> xoscen;
    BitField<4> pllen;
  } ctrl_;

  union STATUS {
    uint8_t data;
    BitField<0> rc2mrdy;
    BitField<1> rc32mrdy;
    BitField<2> rc32krdy;
    BitField<3> xoscrdy;
    BitField<4> pllrdy;
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
