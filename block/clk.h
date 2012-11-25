#ifndef BLOCK_CLK_H__
#define BLOCK_CLK_H__

#include "../block.h"

namespace block {


/// CLK peripheral
class CLK: public Block
{
  friend class ::Device;
  static const ioptr_t IO_SIZE = 8;

 public:
  CLK(Device* dev);
  virtual ~CLK();

  virtual ioptr_t io_size() const { return IO_SIZE; }

  virtual uint8_t getIo(ioptr_t addr);
  virtual void setIo(ioptr_t addr, uint8_t v);
  virtual void reset();

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
  unsigned int f_cpu_;
  unsigned int f_per_; // equal to f_cpu_
  unsigned int f_per2_;
  unsigned int f_per4_;
};


}

#endif
