#include <cassert>
#include "cpu.h"
#include "../device.h"
#include "../log.h"

namespace block {


CPU::CPU(Device& dev):
    Block(dev, "CPU", 0x0030),
    ramp_mask_((dev.mem_exsram_start_ + dev.mem_exsram_size_) >> 8),
    eind_mask_(dev.flash_size_ >> 9)
{
}


uint8_t CPU::getIo(ioptr_t addr)
{
  assert(addr < IO_SIZE);
  if(addr == 0x04) { // CCP
    return ccpState();
  } else if(addr == 0x08) { // RAMPD
    return rampd_;
  } else if(addr == 0x09) { // RAMPX
    return rampx_;
  } else if(addr == 0x0A) { // RAMPY
    return rampy_;
  } else if(addr == 0x0B) { // RAMPZ
    return rampz_;
  } else if(addr == 0x0C) { // EIND
    return eind_;
  } else if(addr == 0x0D) { // SPL
    return sp_ & 0xff;
  } else if(addr == 0x0E) { // SPH
    return (sp_ >> 8) & 0xff;
  } else if(addr == 0x0F) { // SREG
    return sreg_.data;
  } else {
    logger_->warn("I/O read 0x{:02X}: reserved address", addr);
    return 0;
  }
}

void CPU::setIo(ioptr_t addr, uint8_t v)
{
  assert(addr < IO_SIZE);
  if(addr == 0x04) { // CCP
    ccp_buffer_ = v;
  } else if(addr == 0x08) { // RAMPD
    rampd_ = v & ramp_mask_;
  } else if(addr == 0x09) { // RAMPX
    rampx_ = v & ramp_mask_;
  } else if(addr == 0x0A) { // RAMPY
    rampy_ = v & ramp_mask_;
  } else if(addr == 0x0B) { // RAMPZ
    rampz_ = v & ramp_mask_;
  } else if(addr == 0x0C) { // EIND
    eind_ = v & eind_mask_;
  } else if(addr == 0x0D) { // SPL
    sp_ = (sp_ & 0xFF00) | v;
  } else if(addr == 0x0E) { // SPH
    sp_ = (sp_ & 0x00FF) | (v << 8);
  } else if(addr == 0x0F) { // SREG
    sreg_.data = v;
  } else {
    logger_->error("I/O write 0x{:02X}: not writable", addr);
  }
}


void CPU::reset()
{
  ccp_buffer_ = 0;
  ccp_ioreg_cycles_ = 0;
  ccp_spm_cycles_ = 0;
  rampd_ = 0;
  rampx_ = 0;
  rampy_ = 0;
  rampz_ = 0;
  eind_ = 0;
  sp_ = device_.mem_exsram_start_-1;
  sreg_.data = 0;
  pc_ = 0; //TODO may start on bootloader
  device_.schedule(ClockType::PER, [&]() { return step(); });
}

unsigned int CPU::step()
{
  //TODO check execution order: decrement before/after instructions?
  if(ccp_ioreg_cycles_ > 0) {
    ccp_ioreg_cycles_--;
  }
  if(ccp_spm_cycles_ > 0) {
    ccp_spm_cycles_--;
  }

  if(ccp_buffer_ == 0xD8) {
    ccp_ioreg_cycles_ = 4;
    ccp_buffer_ = 0;
  } else if(ccp_buffer_ == 0x9D) {
    ccp_spm_cycles_ = 4;
    ccp_buffer_ = 0;
  }
  return 1;
}


}



