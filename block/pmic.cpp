#include <cassert>
#include "pmic.h"
#include "../device.h"
#include "../log.h"

namespace block {


PMIC::PMIC(Device& dev):
    Block(dev, "PMIC", 0x00A0)
{
}


uint8_t PMIC::getIo(ioptr_t addr)
{
  assert(addr < IO_SIZE);
  if(addr == 0x00) { // STATUS
    return status_.data;
  } else if(addr == 0x01) { // INTPRI
    logger_->warn("I/O read 0x{:02X}: INTPRI not implemented", addr);
    return 0;
  } else if(addr == 0x02) { // CTRL
    return ctrl_.data;
  } else {
    logger_->warn("I/O read 0x{:02X}: reserved address", addr);
    return 0;
  }
}

void PMIC::setIo(ioptr_t addr, uint8_t v)
{
  assert(addr < IO_SIZE);
  if(addr == 0x01) { // INTPRI
    logger_->warn("I/O write 0x{:02X}: INTPRI not implemented", addr);
  } else if(addr == 0x02) { // CTRL
    if(!(device_.ccpState() & Device::CCP_IOREG)) {
      v &= ~(1<<6);
    }
    ctrl_.data = v;
  } else {
    logger_->warn("I/O write 0x{:02X}: not writable", addr);
  }
}


void PMIC::reset()
{
  status_.data = 0;
  ctrl_.data = 0;
  //TODO device_.schedule(ClockType::PER, [&]() { return step(); });
}

unsigned int PMIC::step()
{
  return 0; //TODO update status bits on RETI
}


}


