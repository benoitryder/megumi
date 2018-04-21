#include "osc.h"
#include "../device.h"
#include "../log.h"

namespace block {


OSC::OSC(Device& dev):
    Block(dev, "OSC", 0x0050, 1)
{
}

OSC::~OSC()
{
}


uint8_t OSC::getIo(ioptr_t addr)
{
  assert(addr < IO_SIZE);
  if(addr == 0x00) { // CTRL
    return ctrl_.data;
  } else if(addr == 0x01) { // STATUS
    return status_.data;
  } else if(addr == 0x02) { // XOSCCTRL
    return xoscctrl_.data;
  } else if(addr == 0x03) { // XOSCFAIL
    return xoscfail_.data;
  } else if(addr == 0x04) { // RC32KCAL
    return rc32kcal_;
  } else if(addr == 0x05) { // PLLCTRL
    return (pllsrc_ << 6) | pllfac_;
  } else if(addr == 0x06) { // DFLLCTRL
    return dfllctrl_.data;
  } else {
    DLOGF(WARNING, "I/O read %s + 0x%02X: reserved address") % name() % addr;
    return 0;
  }
}

void OSC::setIo(ioptr_t addr, uint8_t v)
{
  assert(addr < IO_SIZE);
  if(addr == 0x00) { // CTRL
    ctrl_.data = v & 0x1F;
    status_.data = ctrl_.data; //TODO clock sources are immediately ready
    //TODO PLL source cannot be disabled
    //TODO since XOSC is not supported, never report them ready
    status_.xoscrdy = 0;
  } else if(addr == 0x02) { // XOSCCTRL
    xoscctrl_.data = v & 0xEF;
    //TODO value is not handled at all for now
  } else if(addr == 0x03) { // XOSCFAIL
    XOSCFAIL vreg;
    vreg.data = v & 0x03;
    // XOSCFDEN
    if(!xoscfail_.xoscfden && vreg.xoscfden) {
      if(device_.ccpState() & Device::CCP_IOREG) {
        xoscfail_.xoscfden = 1;
      } else {
        LOGF(ERROR, "cannot set XOSCFAIL.XOSCFDEN: protected by CCP");
      }
    } else if(xoscfail_.xoscfden && !vreg.xoscfden) {
      LOGF(ERROR, "XOSCFAIL.XOSCFDEN cannot be cleared");
    }
    // XOSCFDIF
    if(vreg.xoscfdif) {
      xoscfail_.xoscfdif = 0;
    }
  } else if(addr == 0x04) { // RC32KCAL
    rc32kcal_ = v;
  } else if(addr == 0x05) { // PLLCTRL
    if((v >> 6) == 1) {
      LOGF(ERROR, "invalid PLLSRC value");
    } else {
      pllsrc_ = static_cast<PLLSRC>(v >> 6);
    }
    pllfac_ = v & 0x1F;
  } else if(addr == 0x06) { // DFLLCTRL
    //TODO no check nor handling is made here
    dfllctrl_.data = v & 0x03;
  } else {
    LOGF(ERROR, "I/O write %s + 0x%02X: not writable") % name() % addr;
  }
}


void OSC::reset()
{
  ctrl_.data = 0x01;
  status_.data = ctrl_.data; //TODO clock sources are immediately ready
  xoscctrl_.data = 0;
  xoscfail_.data = 0;
  rc32kcal_ = 0x55; // random initial value
}


unsigned int OSC::getPllFrequency() const
{
  unsigned int f_base;
  switch(pllsrc_) {
    case PLLSRC_RC2M:
      f_base = 2000000;
      break;
    case PLLSRC_RC32M:
      f_base = 32000000/4;
      break;
    case PLLSRC_XOSC:
      throw BlockError(*this, "unsupported PLLSRC value");
  }

  return f_base * pllfac_;
}


}


