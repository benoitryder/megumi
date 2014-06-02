#include <cassert>
#include "tc.h"
#include "../device.h"
#include "../log.h"

namespace block {


TC::TC(Device* dev, const Instance<TC>& instance):
    Block(dev, instance.name, instance.io_addr, instance.iv_base)
{
  // set type from block name
  switch(*name().rbegin()) {
    case '0': type_ = 0; break;
    case '1': type_ = 1; break;
    default: throw std::runtime_error(name() + ": cannot retrieve type: name must end with 0 or 1");
  }
}

TC::~TC()
{
}


uint8_t TC::getIo(ioptr_t addr)
{
  assert(addr < IO_SIZE);
  if(addr == 0x00) { // CTRLA
    unsigned int clksel = 0;
    switch(prescaler_) {
      case 0: prescaler_ = 0; break;
      case 1: prescaler_ = 1; break;
      case 2: prescaler_ = 2; break;
      case 4: prescaler_ = 3; break;
      case 8: prescaler_ = 4; break;
      case 64: prescaler_ = 5; break;
      case 256: prescaler_ = 6; break;
      case 1024: prescaler_ = 7; break;
      default: throw std::runtime_error(name() + ": invalid internal prescaler value");
    }
    return clksel;
  } else if(addr == 0x01) { // CTRLB
    return ctrlb_.data;
  } else if(addr == 0x02) { // CTRLC
    return ctrlc_.data;
  } else if(addr == 0x03) { // CTRLD
    return 0;
  } else if(addr == 0x04) { // CTRLE
    return 0;
  } else if(addr == 0x06) { // INTCTRLA
    return (err_intlvl_ << 2) | ovf_intlvl_;
  } else if(addr == 0x07) { // INTCTRLB
    return (ccd_intlvl_ << 6) | (ccc_intlvl_ << 4) | (ccb_intlvl_ << 2) | cca_intlvl_;
  } else if(addr == 0x08) { // CTRLFCLR
    return ctrlf_.data;
  } else if(addr == 0x09) { // CTRLFSET
    return ctrlf_.data;
  } else if(addr == 0x0A) { // CTRLGCLR
    return ctrlg_.data;
  } else if(addr == 0x0B) { // CTRLGSET
    return ctrlg_.data;
  } else if(addr == 0x0C) { // INTFLAGS
    return intflags_.data;
  } else if(addr == 0x0F) { // TEMP
    return temp_;
  } else if(addr == 0x20) { // CNTL
    temp_ = (cnt_ >> 8) & 0xFF;
    return cnt_ & 0xFF;
  } else if(addr == 0x21) { // CNTH
    return temp_;
  } else if(addr == 0x26) { // PERL
    temp_ = (per_ >> 8) & 0xFF;
    return per_ & 0xFF;
  } else if(addr == 0x27) { // PERH
    return temp_;
  } else if(addr == 0x28) { // CCAL
    temp_ = (cca_ >> 8) & 0xFF;
    return cca_ & 0xFF;
  } else if(addr == 0x29) { // CCAH
    return temp_;
  } else if(addr == 0x2A) { // CCBL
    temp_ = (ccb_ >> 8) & 0xFF;
    return ccb_ & 0xFF;
  } else if(addr == 0x2B) { // CCBH
    return temp_;
  } else if(addr == 0x2C) { // CCCL
    temp_ = (ccc_ >> 8) & 0xFF;
    return ccc_ & 0xFF;
  } else if(addr == 0x2D) { // CCCH
    return temp_;
  } else if(addr == 0x2E) { // CCDL
    temp_ = (ccd_ >> 8) & 0xFF;
    return ccd_ & 0xFF;
  } else if(addr == 0x2F) { // CCDH
    return temp_;
  } else if(addr == 0x36) { // PERBUFL
    temp_ = (perbuf_ >> 8) & 0xFF;
    return perbuf_ & 0xFF;
  } else if(addr == 0x37) { // PERBUFH
    return temp_;
  } else if(addr == 0x38) { // CCABUFL
    temp_ = (ccabuf_ >> 8) & 0xFF;
    return ccabuf_ & 0xFF;
  } else if(addr == 0x39) { // CCABUFH
    return temp_;
  } else if(addr == 0x3A) { // CCBBUFL
    temp_ = (ccbbuf_ >> 8) & 0xFF;
    return ccbbuf_ & 0xFF;
  } else if(addr == 0x3B) { // CCBBUFH
    return temp_;
  } else if(addr == 0x3C) { // CCCBUFL
    temp_ = (cccbuf_ >> 8) & 0xFF;
    return cccbuf_ & 0xFF;
  } else if(addr == 0x3D) { // CCCBUFH
    return temp_;
  } else if(addr == 0x3E) { // CCDBUFL
    temp_ = (ccdbuf_ >> 8) & 0xFF;
    return ccdbuf_ & 0xFF;
  } else if(addr == 0x3F) { // CCDBUFH
    return temp_;
  } else {
    DLOGF(WARNING, "I/O read %s + 0x%02X: reserved address") % name() % addr;
    return 0;
  }
}

void TC::setIo(ioptr_t addr, uint8_t v)
{
  assert(addr < IO_SIZE);
  if(addr == 0x00) { // CTRLA
    if(v & 0x08) {
      //TODO
      LOGF(WARNING, "event channel source not supported");
    } else {
      const unsigned int prescalers[8] = { 0, 1, 2, 4, 8, 64, 256, 1024 };
      prescaler_ = prescalers[v];
    }
  } else if(addr == 0x01) { // CTRLB
    if(v & 0xF0) {
      //TODO
      LOGF(WARNING, "CCxEN bits not supported");
      v &= 0x0F;
    }
    //TODO
    ctrlb_.data = v & 0xF7;
    if(ctrlb_.wgmode == 2 || ctrlb_.wgmode == 4) {
      LOGF(ERROR, "invalid WGMODE value");
      ctrlb_.wgmode = 0;
    }
  } else if(addr == 0x02) { // CTRLC
    //TODO
    ctrlc_.data = v & 0x0F;
  } else if(addr == 0x03) { // CTRLD
    //TODO
    if(v != 0) {
      LOGF(WARNING, "event actions not supported");
    }
  } else if(addr == 0x04) { // CTRLE
    //TODO
    uint8_t vbytem = v & 0x03;
    if(vbytem == 0) {
      // normal mode
    } else if(vbytem == 1) {
      //TODO
      LOGF(WARNING, "BYTEMODE not supported");
    } else if(vbytem == 2) {
      //TODO
      LOGF(WARNING, "SPLITMODE not supported");
    } else {
      LOGF(ERROR, "invalid BYTEM value");
    }
  } else if(addr == 0x06) { // INTCTRLA
    ovf_intlvl_ = static_cast<IntLvl>(v & 0x3);
    err_intlvl_ = static_cast<IntLvl>((v >> 2) & 0x3);
    // update pending IVs
    if(intflags_.ovfif) setIvLvl(IV_OVF, ovf_intlvl_);
    if(intflags_.errif) setIvLvl(IV_ERR, err_intlvl_);
  } else if(addr == 0x07) { // INTCTRLB
    cca_intlvl_ = static_cast<IntLvl>(v & 0x3);
    ccb_intlvl_ = static_cast<IntLvl>((v >> 2) & 0x3);
    if(type_ != 0) {
      ccc_intlvl_ = static_cast<IntLvl>((v >> 4) & 0x3);
      ccd_intlvl_ = static_cast<IntLvl>((v >> 6) & 0x3);
    }
    // update pending IVs
    if(intflags_.ccaif) setIvLvl(IV_ERR, cca_intlvl_);
    if(intflags_.ccbif) setIvLvl(IV_ERR, ccb_intlvl_);
    if(type_ != 0) {
      if(intflags_.cccif) setIvLvl(IV_ERR, ccc_intlvl_);
      if(intflags_.ccdif) setIvLvl(IV_ERR, ccd_intlvl_);
    }
  } else if(addr == 0x08) { // CTRLFCLR
    //TODO
    ctrlf_.data &= ~(v & 0x03);
  } else if(addr == 0x09) { // CTRLFSET
    //TODO
    ctrlf_.data |= (v & 0x0F);
  } else if(addr == 0x0A) { // CTRLGCLR
    //TODO
    ctrlg_.data &= ~(v & 0x1F);
  } else if(addr == 0x0B) { // CTRLGSET
    //TODO
    ctrlg_.data |= (v & 0x1F);
  } else if(addr == 0x0C) { // INTFLAGS
    //TODO
    intflags_.data &= ~v;
  } else if(addr == 0x0F) { // TEMP
    temp_ = v;
  } else if(addr == 0x20) { // CNTL
    temp_ = v;
  } else if(addr == 0x21) { // CNTH
    //TODO
    cnt_ = temp_ | (v << 8);
  } else if(addr == 0x26) { // PERL
    temp_ = v;
  } else if(addr == 0x27) { // PERH
    //TODO
    per_ = temp_ | (v << 8);
  } else if(addr == 0x28) { // CCAL
    temp_ = v;
  } else if(addr == 0x29) { // CCAH
    //TODO
    cca_ = temp_ | (v << 8);
  } else if(addr == 0x2A) { // CCBL
    temp_ = v;
  } else if(addr == 0x2B) { // CCBH
    //TODO
    ccb_ = temp_ | (v << 8);
  } else if(addr == 0x2C) { // CCCL
    temp_ = v;
  } else if(addr == 0x2D) { // CCCH
    //TODO
    ccc_ = temp_ | (v << 8);
  } else if(addr == 0x2E) { // CCDL
    temp_ = v;
  } else if(addr == 0x2F) { // CCDH
    //TODO
    ccd_ = temp_ | (v << 8);
  } else if(addr == 0x36) { // PERBUFL
    temp_ = v;
  } else if(addr == 0x37) { // PERBUFH
    //TODO
    perbuf_ = temp_ | (v << 8);
  } else if(addr == 0x38) { // CCBBUFL
    temp_ = v;
  } else if(addr == 0x39) { // CCABUFH
    //TODO
    ccabuf_ = temp_ | (v << 8);
  } else if(addr == 0x3A) { // CCABUFL
    temp_ = v;
  } else if(addr == 0x3B) { // CCBBUFH
    //TODO
    ccbbuf_ = temp_ | (v << 8);
  } else if(addr == 0x3C) { // CCCBUFL
    temp_ = v;
  } else if(addr == 0x3D) { // CCCBUFH
    //TODO
    cccbuf_ = temp_ | (v << 8);
  } else if(addr == 0x3E) { // CCDBUFL
    temp_ = v;
  } else if(addr == 0x3F) { // CCDBUFH
    //TODO
    ccdbuf_ = temp_ | (v << 8);
  } else {
    LOGF(ERROR, "I/O write %s + 0x%02X: not writable") % name() % addr;
  }
}


void TC::executeIv(ivnum_t iv)
{
  //TODO
  assert(iv < IV_COUNT);
}


void TC::reset()
{
  prescaler_ = 0;
  ctrlb_.data = 0;
  ctrlc_.data = 0;
  ovf_intlvl_ = INTLVL_NONE;
  err_intlvl_ = INTLVL_NONE;
  cca_intlvl_ = INTLVL_NONE;
  ccb_intlvl_ = INTLVL_NONE;
  ccc_intlvl_ = INTLVL_NONE;
  ccd_intlvl_ = INTLVL_NONE;
  ctrlf_.data = 0;
  ctrlg_.data = 0;
  intflags_.data = 0;
  temp_ = 0;
  cnt_ = 0;
  per_ = 0;
  cca_ = 0;
  ccb_ = 0;
  ccc_ = 0;
  ccd_ = 0;
  perbuf_ = 0;
  ccabuf_ = 0;
  ccbbuf_ = 0;
  cccbuf_ = 0;
  ccdbuf_ = 0;

  step_event_ = 0;
}


unsigned int TC::step()
{
  return 0; //TODO
}


}


