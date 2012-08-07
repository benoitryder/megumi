#include <cassert>
#include "port.h"
#include "../log.h"

namespace block {


Port::Port(Device* dev, const Instance<Port>& instance):
    Block(dev, instance.name, instance.io_addr, instance.iv_base)
{
}

Port::~Port()
{
}


uint8_t Port::getIo(ioptr_t addr)
{
  assert(addr < IO_SIZE);
  switch(addr) {
    case 0x00: // DIR
    case 0x01: // DIRSET
    case 0x02: // DIRCLR
    case 0x03: // DIRTGL
      return dir_;
    case 0x04: // OUT
    case 0x05: // OUTSET
    case 0x06: // OUTCLR
    case 0x07: // OUTTGL
      return out_;
    case 0x08: // IN
      //TODO bit value for pin "out"
      LOGF(WARNING, "I/O read %s + 0x%02X: not implemented") % name() % addr;
      return (out_ & dir_) | (/*TODO*/0 & ~dir_);
    case 0x09: // INTCTRL
    case 0x0A: // INT0MASK
    case 0x0B: // INT1MASK
    case 0x0C: // INTFLAGS
    case 0x10: // PINnCTRL
    case 0x11:
    case 0x12:
    case 0x13:
    case 0x14:
    case 0x15:
    case 0x16:
    case 0x17:
      //TODO not implemented yet
      LOGF(WARNING, "I/O read %s + 0x%02X: not implemented") % name() % addr;
      return 0;
    default:
      DLOGF(WARNING, "I/O read %s + 0x%02X: reserved address") % name() % addr;
      return 0; //TODO warning
  }
}

void Port::setIo(ioptr_t addr, uint8_t v)
{
  assert(addr < IO_SIZE);
  switch(addr) {
    case 0x00: // DIR
      dir_ = v;
      return;
    case 0x01: // DIRSET
      dir_ &= v;
      return;
    case 0x02: // DIRCLR
      dir_ &= ~v;
      return;
    case 0x03: // DIRTGL
      dir_ ^= v;
      return;
    case 0x04: // OUT
      out_ = v;
      return;
    case 0x05: // OUTSET
      out_ &= v;
      return;
    case 0x06: // OUTCLR
      out_ &= ~v;
      return;
    case 0x07: // OUTTGL
      out_ ^= v;
      return;
    case 0x08: // IN
      //TODO ??
      LOGF(WARNING, "I/O write %s + 0x%02X: not implemented") % name() % addr;
      return;
    case 0x09: // INTCTRL
    case 0x0A: // INT0MASK
    case 0x0B: // INT1MASK
    case 0x0C: // INTFLAGS
    case 0x10: // PINnCTRL
    case 0x11:
    case 0x12:
    case 0x13:
    case 0x14:
    case 0x15:
    case 0x16:
    case 0x17:
      //TODO not implemented yet
      LOGF(WARNING, "I/O write %s + 0x%02X: not implemented") % name() % addr;
      return;
    default:
      DLOGF(WARNING, "I/O write %s + 0x%02X: not writable") % name() % addr;
      return;
  }
}


void Port::reset()
{
  dir_ = 0;
  out_ = 0;
}


}

