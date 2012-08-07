#include <cassert>
#include "usart.h"
#include "../device.h"
#include "../log.h"

namespace block {


USART::USART(Device* dev, const Instance<USART>& instance):
    Block(dev, instance.name, instance.io_addr, instance.iv_base)
{
}

USART::~USART()
{
}


uint8_t USART::getIo(ioptr_t addr)
{
  assert(addr < IO_SIZE);
  if(addr == 0x00) { // RXB (DATA)
    status_.bufofv = 0;
    status_.rxcif = 0;
    return rxb_;
  } else if(addr == 0x01) { // STATUS
    return status_.data;
  } else if(addr == 0x03) { // CTRLA
    return (rxc_intlvl_ << 4) | (txc_intlvl_ << 2) | dre_intlvl_;
  } else if(addr == 0x04) { // CTRLB
    return ctrlb_.data;
  } else if(addr == 0x05) { // CTRLC
    return ctrlc_.data;
  } else if(addr == 0x06) { // BAUDCTRLA
    return baudrate_ & 0xFF;
  } else if(addr == 0x07) { // BAUDCTRLB
    return ((baudrate_ >> 4) & 0xF0) | (baudscale_ & 0xF);
  } else {
    DLOGF(WARNING, "I/O read %s + 0x%02X: reserved address") % name() % addr;
    return 0;
  }
}

void USART::setIo(ioptr_t addr, uint8_t v)
{
  assert(addr < IO_SIZE);
  if(addr == 0x00) { // TXB (DATA)
    if(status_.dreif) {
      txb_ = v;
      status_.dreif = 0;
    }
  } else if(addr == 0x01) { // STATUS
    STATUS vreg = { .data=v };
    status_.rxb8 = vreg.rxb8;
    if(vreg.txcif) {
      status_.txcif = 0;
    }
    if(vreg.rxcif) {
      status_.rxcif = 0;
    }
  } else if(addr == 0x03) { // CTRLA
    rxc_intlvl_ = static_cast<IntLvl>((v >> 4) & 0x3);
    txc_intlvl_ = static_cast<IntLvl>((v >> 2) & 0x3);
    dre_intlvl_ = static_cast<IntLvl>(v & 0x3);
    // update pending IVs
    if(status_.rxcif) setIvLvl(IV_RXC, rxc_intlvl_);
    if(status_.txcif) setIvLvl(IV_TXC, txc_intlvl_);
    if(status_.dreif) setIvLvl(IV_DRE, dre_intlvl_);
  } else if(addr == 0x04) { // CTRLB
    ctrlb_.data = v & 0x1F;
  } else if(addr == 0x05) { // CTRLC
    CTRLC vreg = { .data=v };
    if(vreg.cmode != 0) {
      LOGF(WARNING, "only asynchronous mode is supported");
      vreg.cmode = 0;
    }
    if(vreg.chsize > 3 && vreg.chsize != 7) {
      LOGF(ERROR, "invalid CTRLC.CHSIZE value: 1");
      vreg.chsize = 3; // 8-bit
    }
    if(vreg.pmode == 1) {
      LOGF(ERROR, "invalid CTRLC.PMODE value: 1");
      vreg.pmode = 0;
    }
    ctrlc_.data = vreg.data;
  } else if(addr == 0x06) { // BAUDCTRLA
    baudrate_ = (baudrate_ & 0xF00) | v;
  } else if(addr == 0x07) { // BAUDCTRLB
    int8_t scale = u8_to_s8<4>(v & 0xF);
    if(scale == -8) {
      LOGF(ERROR, "invalid baudrate scale: -8");
      scale = 0;
    }
    baudrate_ = (baudrate_ & 0x0FF) | ((v & 0xF0) << 4);
    baudscale_ = scale;
  } else {
    LOGF(ERROR, "I/O write %s + 0x%02X: not writable") % name() % addr;
  }
}


void USART::executeIv(ivnum_t iv)
{
  assert(iv < IV_COUNT);
  if(iv == IV_TXC) {
    status_.txcif = 0;
  }
}


void USART::reset()
{
  status_.data = 0x20;
  ctrlb_.data = 0;
  ctrlc_.data = 0; //TODO check initial value
  baudrate_ = 0;
  baudscale_ = 0;
  rxb_ = 0;
  txb_ = 0;
}

void USART::step()
{
}


}



