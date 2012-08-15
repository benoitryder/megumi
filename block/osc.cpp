#include "osc.h"
#include "../device.h"
#include "../log.h"

namespace block {


OSC::OSC(Device* dev):
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
  } else {
    DLOGF(WARNING, "I/O read %s + 0x%02X: reserved address") % name() % addr;
    return 0;
  }
}

void OSC::setIo(ioptr_t addr, uint8_t v)
{
  assert(addr < IO_SIZE);
  if(addr == 0x00) { //CTRL
    ctrl_.data = v & 0x1F;
    status_.data = ctrl_.data; //TODO clock sources are immediately ready
    //TODO PLL source cannot be disabled
  } else {
    LOGF(ERROR, "I/O write %s + 0x%02X: not writable") % name() % addr;
  }
}


void OSC::reset()
{
  ctrl_.data = 0x01;
  status_.data = ctrl_.data; //TODO clock sources are immediately ready
}

void OSC::step()
{
}


}


