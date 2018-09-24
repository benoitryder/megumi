#ifndef BLOCK_CLK_H
#define BLOCK_CLK_H

#include "../block.h"

namespace block {

class OSC;


/// CLK peripheral
class CLK: public Block
{
  friend class ::Device;
  static constexpr ioptr_t IO_SIZE = 8;

 public:
  CLK(Device& dev, const OSC& osc);
  virtual ~CLK() = default;

  ioptr_t io_size() const override { return IO_SIZE; }

  uint8_t getIo(ioptr_t addr) override;
  void setIo(ioptr_t addr, uint8_t v) override;
  void reset() override;

  enum SCLKSEL {
    SCLKSEL_RC2M = 0,
    SCLKSEL_RC32M = 1,
    SCLKSEL_RC32K = 2,
    SCLKSEL_XOSC = 3,
    SCLKSEL_PLL = 4,
  };

  enum RTCSRC {
    RTCSRC_ULP = 0,
    RTCSRC_TOSC = 1,
    RTCSRC_RCOSC = 2,
    RTCSRC_TOSC32 = 5,
  };

 private:
  /// Update frequencies after configuration change
  void updateFrequencies();

  const OSC& osc_;
  SCLKSEL sclk_;

  union PSCTRL {
    uint8_t data;
    BitField<0,2> psbcdiv;
    BitField<2,5> psadiv;
  } psctrl_;

  bool locked_;
  RTCSRC rtcsrc_;
  bool rtcen_;

  // computed prescaler values and clock frequencies
  // these values must be kept in sync with register values

  unsigned int prescaler_a_;
  unsigned int prescaler_b_;
  unsigned int prescaler_c_;
  unsigned int f_sys_;
};


}

#endif
