#include "clk.h"
#include "../device.h"
#include "../log.h"

namespace block {


CLK::CLK(Device& dev, const OSC& osc):
    Block(dev, "CLK", 0x0040),
    osc_(osc)
{
}

CLK::~CLK()
{
}


uint8_t CLK::getIo(ioptr_t addr)
{
  assert(addr < IO_SIZE);
  if(addr == 0x00) { // CTRL
    return sclk_;
  } else if(addr == 0x01) { // PSCTRL
    return psctrl_.data;
  } else if(addr == 0x02) { // LOCK
    return locked_;
  } else if(addr == 0x03) { // RTCCTRL
    return (rtcen_ & 1) | (rtcsrc_ << 1);
  } else {
    DLOGF(WARNING, "I/O read {} + 0x{:02X}: reserved address", name(), addr);
    return 0;
  }
}

void CLK::setIo(ioptr_t addr, uint8_t v)
{
  assert(addr < IO_SIZE);
  if(addr == 0x00 && !locked_) { // CTRL
    SCLKSEL vsclk = static_cast<SCLKSEL>(v & 0x7);
    if(vsclk > SCLKSEL_PLL) {
      LOGF(ERROR, "invalid SCLKSEL value");
    } else if(device_.ccpState() & Device::CCP_IOREG) {
      if(vsclk == SCLKSEL_XOSC) {
        LOGF(WARNING, "XOSC clock source not supported");
      } else {
        sclk_ = vsclk;
        updateFrequencies();
      }
      //TODO takes 2 old clock cycles and 2 new clock cycles
      //TODO don't assume clock sources are always stable
    } else {
      LOGF(ERROR, "cannot set CLK.CTR: protected by CCP");
    }
  } else if(addr == 0x01 && !locked_) { // PSCTRL
    PSCTRL vreg;
    vreg.data = v & 0x7F;
    if(vreg.psadiv > 9 || (vreg.psadiv != 0 && !(vreg.psadiv&1))) {
      LOGF(ERROR, "invalid PSADIV value");
    } else {
      psctrl_.data = vreg.data;
      updateFrequencies();
    }
  } else if(addr == 0x02) { // LOCK
    if(!locked_ && v) {
      if(device_.ccpState() & Device::CCP_IOREG) {
        LOGF(NOTICE, "locked CLK.CTRL and CLK.PSCTRL");
        locked_ = true;
      } else {
        LOGF(ERROR, "cannot set CLK.LOCK: protected by CCP");
      }
    } else if(locked_ && !v) {
      LOGF(ERROR, "CLK.LOCK cannot be cleared");
    }
  } else if(addr == 0x03) { // RTCCTRL
    rtcen_ = v & 1;
    uint8_t vsrc = (v >> 1) & 7;
    if(vsrc <= RTCSRC_RCOSC || vsrc == RTCSRC_TOSC32) {
      rtcsrc_ = static_cast<RTCSRC>(vsrc);
    } else {
      LOGF(ERROR, "invalid RTCSRC value");
    }
  } else {
    LOGF(ERROR, "I/O write {} + 0x{:02X}: not writable", name(), addr);
  }
}


void CLK::reset()
{
  sclk_ = SCLKSEL_RC2M;
  psctrl_.data = 0;
  locked_ = false;
  rtcsrc_ = RTCSRC_ULP;
  rtcen_ = false;
  updateFrequencies();
}


void CLK::updateFrequencies()
{
  switch(sclk_) {
    case SCLKSEL_RC2M:
      f_sys_ = 2000000;
      break;
    case SCLKSEL_RC32M:
      f_sys_ = 32000000;
      break;
    case SCLKSEL_RC32K:
      f_sys_ = 32768;
      break;
    case SCLKSEL_PLL:
      //TODO PLL configuration cannot be modified while PLL is enabled
      f_sys_ = osc_.getPllFrequency();
      if(f_sys_ > 200000000) {
        LOGF(ERROR, "PLL frequency is too high");
      } else if(f_sys_ < 10000000) {
        LOGF(ERROR, "PLL frequency is too low");
      }
      break;
    case SCLKSEL_XOSC:
      // not reachable, check is made earlier
      throw BlockError(*this, "unsupported SCLKSEL value");
  }

  prescaler_a_ = psctrl_.psadiv == 0 ? 1 : 1 << ((psctrl_.psadiv>>1)+1);
  prescaler_b_ = (psctrl_.psbcdiv & 2) ? (1 << (4-psctrl_.psbcdiv)) : 1;
  prescaler_c_ = (1 << (psctrl_.psbcdiv & 1));

  device_.onClockConfigChange();
}


}


