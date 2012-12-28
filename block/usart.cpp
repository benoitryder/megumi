#ifdef __WIN32
#include <windows.h>
#undef ERROR
#endif
#include <cassert>
#include <fcntl.h>
#include <errno.h>
#include <boost/property_tree/ptree.hpp>
#include "usart.h"
#include "../device.h"
#include "../log.h"

namespace block {

// USART links

USARTLink::~USARTLink() {}


/// Dummy USART link
class USARTLinkDummy: public USARTLink
{
 public:
  //TODO use baudrate to limit send() speed
  USARTLinkDummy() {}
  virtual ~USARTLinkDummy() {}
  virtual void configure(const USART*) {}
  virtual int recv() { return -1; }
  virtual bool send(uint8_t) { return true; }
};


/// USART link to plain file
class USARTLinkFile: public USARTLink
{
 public:
  USARTLinkFile(const std::string& path);
  virtual ~USARTLinkFile();
  virtual void configure(const USART* usart);
  virtual int recv();
  virtual bool send(uint8_t v);
 private:
#ifdef __WIN32
  HANDLE h_;
  struct {
    OVERLAPPED o;
    uint8_t data;
    bool waiting;
  } state_read_, state_write_;
#else
  int fd_;
#endif
};

void USARTLinkFile::configure(const USART*)
{
  //TODO use baudrate to limit transfer speed
}

#ifdef __WIN32

USARTLinkFile::USARTLinkFile(const std::string& path)
{
  h_ = CreateFile(path.c_str(), GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
  if(h_ == INVALID_HANDLE_VALUE) {
    throw std::runtime_error("failed to open USART link file: "+path);
  }

  state_read_.o.hEvent = CreateEvent(NULL, true, false, NULL);
  state_write_.o.hEvent = CreateEvent(NULL, true, false, NULL);
  if(!state_read_.o.hEvent || !state_write_.o.hEvent) {
    throw std::runtime_error("failed to initialize USART link file");
  }
}

USARTLinkFile::~USARTLinkFile()
{
  CancelIo(h_);
  CloseHandle(state_read_.o.hEvent);
  CloseHandle(state_write_.o.hEvent);
  CloseHandle(h_);
}

int USARTLinkFile::recv()
{
  if(!state_read_.waiting) {
    ResetEvent(state_read_.o.hEvent);
    if(!ReadFile(h_, &state_read_.data, 1, NULL, &state_read_.o)) {
      if(GetLastError() != ERROR_IO_PENDING) {
        throw std::runtime_error("read error on USART link file");
      }
      state_read_.waiting = true;
      return -1;
    } else {
      return state_read_.data;
    }
  } else {
    switch(WaitForSingleObject(state_read_.o.hEvent, 0)) {
      case WAIT_OBJECT_0: {
        DWORD dummy;
        if(!GetOverlappedResult(h_, &state_read_.o, &dummy, false)) {
          throw std::runtime_error("read error on USART link file");
        }
        state_read_.waiting = false;
        return state_read_.data;
      }
      case WAIT_TIMEOUT:
        return -1;
      default:
        throw std::runtime_error("read error on USART link file");
    }
  }
}

bool USARTLinkFile::send(uint8_t v)
{
  if(!state_write_.waiting) {
    ResetEvent(state_write_.o.hEvent);
    state_write_.data = v;
    if(!WriteFile(h_, &state_write_.data, 1, NULL, &state_write_.o)) {
      if(GetLastError() != ERROR_IO_PENDING) {
        throw std::runtime_error("failed to write on USART link file");
      }
      state_write_.waiting = true;
      return false;
    } else {
      return true;
    }
  } else {
    switch(WaitForSingleObject(state_write_.o.hEvent, 0)) {
      case WAIT_OBJECT_0: {
        DWORD dummy;
        if(!GetOverlappedResult(h_, &state_write_.o, &dummy, FALSE)) {
          throw std::runtime_error("write error on USART link file");
        }
        state_write_.waiting = false;
        return true;
      }
      case WAIT_TIMEOUT:
        return false;
      default:
        throw std::runtime_error("write error on USART link file");
    }
  }
}

#else

USARTLinkFile::USARTLinkFile(const std::string& path)
{
  fd_ = ::open(path.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(fd_ < 0) {
    throw std::runtime_error("failed to open USART link file: "+path);
  }
}

USARTLinkFile::~USARTLinkFile()
{
  ::close(fd_);
}

int USARTLinkFile::recv()
{
  uint8_t c;
  ssize_t n = ::read(fd_, &c, 1);
  if(n == 1) {
    return c;
  } else if(n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
    return -1;
  } else {
    throw std::runtime_error("failed to read on USART link file");
  }
}

bool USARTLinkFile::send(uint8_t v)
{
  ssize_t n = ::write(fd_, &v, 1);
  if(n == 1) {
    return true;
  } else if(n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
    return false;
  } else {
    throw std::runtime_error("failed to write on USART link file");
  }
}

#endif



// USART block

USART::USART(Device* dev, const Instance<USART>& instance):
    Block(dev, instance.name, instance.io_addr, instance.iv_base)
{
  const ConfTree& conf = this->conf();
  std::string link_path = conf.get<std::string>("link_path", "");
  if(!link_path.empty()) {
    std::string link_type = conf.get<std::string>("link_type", "");
    if(link_type.empty()) {
#define STARTSWITH(s,prefix)  (!(s).compare(0, sizeof(prefix)-1, prefix))
#if __WIN32
      if(STARTSWITH(link_path, "COM") || STARTSWITH(link_path, R"(\\.\COM)"))
#else
      if(link_path == "/dev/ptmx" || STARTSWITH(link_path, "/dev/tty"))
#endif
#undef STARTSWITH
      {
        link_type = "serial";
      } else {
        link_type = "file";
      }
    }
    if(link_type == "serial") {
      throw std::runtime_error(name() + ": 'link_type = serial' not supported yet");
    } else if(link_type == "file") {
      link_ = std::unique_ptr<USARTLink>(new USARTLinkFile(link_path));
    } else {
      throw std::runtime_error(name() + ": invalid link_type value");
    }
  } else {
    link_ = std::unique_ptr<USARTLink>(new USARTLinkDummy());
  }
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
    link_->configure(this);
  } else if(addr == 0x06) { // BAUDCTRLA
    baudrate_ = (baudrate_ & 0xF00) | v;
    link_->configure(this);
  } else if(addr == 0x07) { // BAUDCTRLB
    int8_t scale = u8_to_s8<4>(v & 0xF);
    if(scale == -8) {
      LOGF(ERROR, "invalid baudrate scale: -8");
      scale = 0;
    }
    baudrate_ = (baudrate_ & 0x0FF) | ((v & 0xF0) << 4);
    baudscale_ = scale;
    link_->configure(this);
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
  //TODO schedule only the USART is enable, unschedule when disabled
  device_->schedule(Device::ClockType::PER, std::bind(&USART::step, this), 1);
}


void USART::step()
{
  //TODO baudrate is completely ignored here
  if(ctrlb_.rxen) {
    int v = link_->recv();
    if(v >= 0) {
      if(status_.rxcif) {
        status_.bufofv = 1;
      } else {
        DLOGF(NOTICE, "%s received %02X") % name() % v;
        rxb_ = v;
        status_.rxcif = 1;
        setIvLvl(IV_RXC, rxc_intlvl_);
      }
    }
  }
  if(ctrlb_.txen) {
    if(!status_.dreif) {
      if(link_->send(txb_)) {
        DLOGF(NOTICE, "%s send %02X") % name() % (int)txb_;
        //TODO TXC and DRE should not be triggered simultaneously
        status_.dreif = 1;
        status_.txcif = 1;
        setIvLvl(IV_DRE, dre_intlvl_);
        setIvLvl(IV_TXC, txc_intlvl_);
      }
    }
  }
}


}



