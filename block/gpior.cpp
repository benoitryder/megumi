#include <cassert>
#include "gpior.h"

namespace block {


GPIOR::GPIOR(Device& dev):
    Block(dev, "GPIOR", 0x0000)
{
}


uint8_t GPIOR::getIo(ioptr_t addr)
{
  assert(addr < IO_SIZE);
  return data_[addr];
}

void GPIOR::setIo(ioptr_t addr, uint8_t v)
{
  assert(addr < IO_SIZE);
  data_[addr] = v;
}

void GPIOR::reset()
{
  data_.fill(0);
}


}


