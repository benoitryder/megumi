#include <cassert>
#include "pmic.h"
#include "../device.h"
#include "../log.h"

namespace block {


PMIC::PMIC(Device* dev):
    Block(dev, "PMIC", 0x00A0)
{
}

PMIC::~PMIC()
{
}


uint8_t PMIC::getIo(ioptr_t addr)
{
  assert(addr < IO_SIZE);
  if(addr == 0x00) { // STATUS
    return status_.data;
  } else if(addr == 0x01) { // INTPRI
    LOGF(WARNING, "I/O read %s + 0x%02X: INTPRI not implemented") % name() % addr;
    return 0;
  } else if(addr == 0x02) { // CTRL
    return ctrl_.data;
  } else {
    DLOGF(WARNING, "I/O read %s + 0x%02X: reserved address") % name() % addr;
    return 0;
  }
}

void PMIC::setIo(ioptr_t addr, uint8_t v)
{
  assert(addr < IO_SIZE);
  if(addr == 0x01) { // INTPRI
    LOGF(WARNING, "I/O write %s + 0x%02X: INTPRI not implemented") % name() % addr;
  } else if(addr == 0x02) { // CTRL
    if(!(device_->ccpState() & Device::CCP_IOREG)) {
      v &= ~(1<<6);
    }
    ctrl_.data = v;
  } else {
    DLOGF(WARNING, "I/O write %s + 0x%02X: not writable") % name() % addr;
  }
}


void PMIC::reset()
{
  status_.data = 0;
  ctrl_.data = 0;
}

void PMIC::step()
{
  //TODO update status bits on RETI
}


}


