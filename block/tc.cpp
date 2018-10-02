#include <cassert>
#include "tc.h"
#include "../device.h"
#include "../log.h"

namespace block {


TC::TC(Device& dev, const Instance<TC>& instance):
    Block(dev, instance.name, instance.io_addr, instance.iv_base),
    event_(ClockType::PER, [this]() { onEvent(); })
{
  // set type from block name
  switch(*name().rbegin()) {
    case '0': type_ = 0; break;
    case '1': type_ = 1; break;
    default: throw std::runtime_error(name() + ": cannot retrieve type: name must end with 0 or 1");
  }
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
    logger_->warn("I/O read 0x{:02X}: reserved address", addr);
    return 0;
  }
}

void TC::setIo(ioptr_t addr, uint8_t v)
{
  assert(addr < IO_SIZE);
  if(addr == 0x00) { // CTRLA
    if(v & 0x08) {
      //TODO
      logger_->warn("event channel source not supported");
    } else {
      const unsigned int prescalers[8] = { 0, 1, 2, 4, 8, 64, 256, 1024 };
      prescaler_ = prescalers[v];
      //TODO how is handle a prescaler change when TC is running?
      // reschedule step event
      if(!prescaler_) {
        if(event_scheduled_) {
          device_.unschedule(event_);
          event_scheduled_ = false;
        }
      } else if(prescaler_) {
        if(!event_scheduled_) {
          device_.schedule(event_, prescaler_);
          event_scheduled_ = true;
        } else {
          event_.period = prescaler_ * event_.scale;
        }
      }
    }
  } else if(addr == 0x01) { // CTRLB
    if(v & 0xF0) {
      //TODO
      logger_->warn("CCxEN bits not supported");
      v &= 0x0F;
    }
    //TODO
    ctrlb_.data = v & 0xF7;
    if(ctrlb_.wgmode == 2 || ctrlb_.wgmode == 4) {
      logger_->error("invalid WGMODE value");
      ctrlb_.wgmode = 0;
    }
  } else if(addr == 0x02) { // CTRLC
    //TODO
    ctrlc_.data = v & 0x0F;
  } else if(addr == 0x03) { // CTRLD
    //TODO
    if(v != 0) {
      logger_->warn("event actions not supported");
    }
  } else if(addr == 0x04) { // CTRLE
    //TODO
    uint8_t vbytem = v & 0x03;
    if(vbytem == 0) {
      // normal mode
    } else if(vbytem == 1) {
      //TODO
      logger_->warn("BYTEMODE not supported");
    } else if(vbytem == 2) {
      //TODO
      logger_->warn("SPLITMODE not supported");
    } else {
      logger_->error("invalid BYTEM value");
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
    ctrlf_.data &= ~(v & 0x03);
  } else if(addr == 0x09) { // CTRLFSET
    ctrlf_.data |= (v & 0x03);
    uint8_t cmd = (v >> 2) & 3;
    if(cmd == 1) {
      updateCommand();
    } else if(cmd == 2) {
      restartCommand();
    } else if(cmd == 3) {
      resetCommand();
    }
  } else if(addr == 0x0A) { // CTRLGCLR
    ctrlg_.data &= ~(v & 0x1F);
  } else if(addr == 0x0B) { // CTRLGSET
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
    perbuf_ = temp_ | (v << 8);
    ctrlg_.perbv = 1;
  } else if(addr == 0x38) { // CCBBUFL
    temp_ = v;
  } else if(addr == 0x39) { // CCABUFH
    ccabuf_ = temp_ | (v << 8);
    ctrlg_.ccabv = 1;
  } else if(addr == 0x3A) { // CCABUFL
    temp_ = v;
  } else if(addr == 0x3B) { // CCBBUFH
    ccbbuf_ = temp_ | (v << 8);
    ctrlg_.ccbbv = 1;
  } else if(addr == 0x3C) { // CCCBUFL
    temp_ = v;
  } else if(addr == 0x3D) { // CCCBUFH
    cccbuf_ = temp_ | (v << 8);
    ctrlg_.cccbv = 1;
  } else if(addr == 0x3E) { // CCDBUFL
    temp_ = v;
  } else if(addr == 0x3F) { // CCDBUFH
    ccdbuf_ = temp_ | (v << 8);
    ctrlg_.ccdbv = 1;
  } else {
    logger_->error("I/O write 0x{:02X}: not writable", addr);
  }
}


void TC::executeIv(ivnum_t iv)
{
  assert(iv < IV_COUNT);
  switch(iv) {
    case IV_OVF: intflags_.ovfif = 0; break;
    case IV_ERR: intflags_.errif = 0; break;
    case IV_CCA: intflags_.ccaif = 0; break;
    case IV_CCB: intflags_.ccbif = 0; break;
    case IV_CCC: intflags_.cccif = 0; break;
    case IV_CCD: intflags_.ccdif = 0; break;
  }
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
}


void TC::onEvent()
{
  uint8_t wgmode = ctrlb_.wgmode;
  uint8_t top = ctrlb_.wgmode == WGMODE_FRQ ? cca_ : per_;
  bool trigger_ovf = false;
  SPDLOG_DEBUG(logger_, "[{}] CNT = {}, DIR = {}, WGMODE = {}, PER = {}, CCA = {}",
               device_.clk_sys_tick(), cnt_, ctrlf_.dir, wgmode, per_, cca_);
  //TODO handle minimum resolution (PER=3 for slopes)

  if(ctrlf_.dir) {
    // down-counting
    if(cnt_ == 0) {
      // reset to TOP
      cnt_ = top;
    }

    cnt_--;
    if(cnt_ == 0) {
      // BOTTOM reached
      trigger_ovf = wgmode != WGMODE_DSTOP;
      processUpdate();
      if(wgmode > WGMODE_DSTOP) {
        // dual-slop: reverse direction
        ctrlf_.dir = 0;
      }
    }

  } else {
    // up-counting
    if(cnt_ == top) {
      // reset to BOTTOM
      cnt_ = 0;
    }

    cnt_++;
    if(cnt_ == top) {
      // TOP reached
      trigger_ovf = wgmode != WGMODE_DSBOTTOM;
      processUpdate();
      if(wgmode > WGMODE_DSTOP) {
        // dual-slop: reverse direction
        ctrlf_.dir = 0;
      }
    }
  }

  //TODO compare to CCx to update output pins on UPDATE

  if(trigger_ovf) {
    intflags_.ovfif = 1;
    setIvLvl(IV_OVF, ovf_intlvl_);
  }

  // process CCxIF interrupts
  if(cnt_ == cca_) {
    intflags_.ccaif = 1;
    setIvLvl(IV_CCA, cca_intlvl_);
  }
  if(cnt_ == ccb_) {
    intflags_.ccbif = 1;
    setIvLvl(IV_CCB, ccb_intlvl_);
  }
  if(type_ != 0) {
    if(cnt_ == ccc_) {
      intflags_.cccif = 1;
      setIvLvl(IV_CCC, ccc_intlvl_);
    }
    if(cnt_ == ccd_) {
      intflags_.ccdif = 1;
      setIvLvl(IV_CCD, ccd_intlvl_);
    }
  }
}


void TC::processUpdate()
{
  // flush double-buffered values
  if(ctrlg_.perbv) {
    per_ = perbuf_;
  }
  if(ctrlg_.ccabv) {
    cca_ = ccabuf_;
  }
  if(ctrlg_.ccbbv) {
    ccb_ = ccbbuf_;
  }
  if(ctrlg_.cccbv) {
    ccc_ = cccbuf_;
  }
  if(ctrlg_.ccdbv) {
    ccd_ = ccdbuf_;
  }
  // clear all BV flags
  ctrlg_.data = 0;
  //TODO are CCx copied before or after comparison for CCxIF?
}


void TC::updateCommand()
{
  if(ctrlf_.lupd) {
    // ignore UPDATE command if the lock update bit is set
    return;
  }

  processUpdate();
}


void TC::restartCommand()
{
  // reset counter direction
  ctrlf_.dir = 0;
  // reset all compare outputs
  ctrlc_.data = 0;
}


void TC::resetCommand()
{
  if(!off()) {
    logger_->warn("RESET command triggered but TC is not OFF");
    return;
  }

  reset();
}


}


