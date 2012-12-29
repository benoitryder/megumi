#ifdef __WIN32
#include <windows.h>
#undef ERROR
#else
#include <termios.h>
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

USARTLink::USARTLink(const USART& usart):
    usart_(usart)
{}
USARTLink::~USARTLink() {}


/// Dummy USART link
class USARTLinkDummy: public USARTLink
{
 public:
  USARTLinkDummy(const USART& usart): USARTLink(usart) {}
  virtual ~USARTLinkDummy() {}
  virtual void configure() {}
  virtual int recv() { return -1; }
  virtual bool send(uint16_t) { return true; }
};


#ifdef __WIN32

/// USART link to plain file
class USARTLinkFile: public USARTLink
{
 public:
  USARTLinkFile(const USART& usart, const std::string& path);
  virtual ~USARTLinkFile();
  virtual void configure() {}
  virtual int recv();
  virtual bool send(uint16_t v);
 protected:
  HANDLE h_;
  struct {
    OVERLAPPED o;
    uint8_t data;
    bool waiting;
  } state_read_, state_write_;
};

USARTLinkFile::USARTLinkFile(const USART& usart, const std::string& path):
    USARTLink(usart)
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

bool USARTLinkFile::send(uint16_t v)
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


/// USART link to serial port
class USARTLinkSerial: public USARTLinkFile
{
 public:
  USARTLinkSerial(const USART& usart, const std::string& path):
      USARTLinkFile(usart, path) {}
  virtual void configure();
};

void USARTLinkSerial::configure()
{
  DCB dcb = {0};
  if(!GetCommState(h_, &dcb)) {
    throw std::runtime_error("GetCommState() failed");
  }

  dcb.BaudRate = usart_.baudrate();
  dcb.ByteSize = usart_.databits() ;
  switch(usart_.parity()) {
    case USART::Parity::NO:
      dcb.fParity = false;
      dcb.Parity = NOPARITY;
      break;
    case USART::Parity::ODD:
      dcb.fParity = true;
      dcb.Parity = ODDPARITY;
      break;
    case USART::Parity::EVEN:
      dcb.fParity = true;
      dcb.Parity = EVENPARITY;
      break;
  }
  dcb.StopBits = usart_.stopbits() == 1 ? ONESTOPBIT : TWOSTOPBITS;
  if(!SetCommState(h_, &dcb)) {
    throw std::runtime_error("SetCommState() failed");
  }
}


#else

/// USART link to plain file
class USARTLinkFile: public USARTLink
{
 public:
  USARTLinkFile(const USART& usart, const std::string& path);
  virtual ~USARTLinkFile();
  virtual void configure() {}
  virtual int recv();
  virtual bool send(uint16_t v);
 protected:
  int fd_;
};

USARTLinkFile::USARTLinkFile(const USART& usart, const std::string& path):
    USARTLink(usart)
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

bool USARTLinkFile::send(uint16_t v)
{
  uint8_t v8 = v; // keep only 8 bits
  ssize_t n = ::write(fd_, &v8, 1);
  if(n == 1) {
    return true;
  } else if(n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
    return false;
  } else {
    throw std::runtime_error("failed to write on USART link file");
  }
}


/// USART link to serial port
class USARTLinkSerial: public USARTLinkFile
{
 public:
  USARTLinkSerial(const USART& usart, const std::string& path);
  virtual void configure();
};

USARTLinkSerial::USARTLinkSerial(const USART& usart, const std::string& path):
  USARTLinkFile(usart, path)
{
  if(path == "/dev/ptmx") {
    LOGF(INFO, "%s ptsname: %s") % usart_.name() % ::ptsname(fd_);
    if(::grantpt(fd_)) {
      throw std::runtime_error("granpt() failed");
    }
    if(::unlockpt(fd_)) {
      throw std::runtime_error("unlockpt() failed");
    }
  }
}

void USARTLinkSerial::configure()
{
  struct termios tos;
  if(::tcgetattr(fd_, &tos)) {
    throw std::runtime_error("tcgetattr() failed");
  }
  tos.c_iflag &= ~(INPCK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
  tos.c_oflag &= ~OPOST;
  tos.c_cflag = CREAD|CLOCAL;
  tos.c_lflag = 0;

  // get closest known baudrate value
  {
    struct BaudrateConstant { ::speed_t c; unsigned int v; };
    const BaudrateConstant baudrates[] = {
      { B0, 0 },
      { B50, 50 },
      { B75, 75 },
      { B110, 110 },
      { B134, 134 },
      { B150, 150 },
      { B200, 200 },
      { B300, 300 },
      { B600, 600 },
      { B1200, 1200 },
      { B1800, 1800 },
      { B2400, 2400 },
      { B4800, 4800 },
      { B9600, 9600 },
      { B19200, 19200 },
      { B38400, 38400 },
      // non-POSIX baudrate values
#ifdef B57600
      { B57600, 57600 },
#endif
#ifdef B115200
      { B115200, 115200 },
#endif
#ifdef B230400
      { B230400, 230400 },
#endif
#ifdef B460800
      { B460800, 460800 },
#endif
    };

    unsigned int baudrate = usart_.baudrate();
    const size_t baudrates_n = sizeof(baudrates)/sizeof(*baudrates);
    size_t i;
    for(i=0; i<baudrates_n && baudrates[i].v < baudrate; ++i) ;
    if(i == baudrates_n || (i > 0 && baudrate-baudrates[i-1].v < baudrates[i].v-baudrate)) {
      --i;
    }
    if(::cfsetspeed(&tos, baudrates[i].c)) {
      throw std::runtime_error("cfsetspeed() failed");
    }
  }

  switch(usart_.databits()) {
    case 5: tos.c_cflag |= CS5; break;
    case 6: tos.c_cflag |= CS6; break;
    case 7: tos.c_cflag |= CS7; break;
    case 8: tos.c_cflag |= CS8; break;
    case 9: throw std::runtime_error("CTRLC.CHSIZE value not supported");
  }

  switch(usart_.parity()) {
    case USART::Parity::ODD: tos.c_cflag |= PARENB|PARODD; break;
    case USART::Parity::EVEN: tos.c_cflag |= PARENB; break;
    default: break;
  }
  switch(usart_.parity()) {
    case USART::Parity::NO:
      tos.c_cflag &= ~(PARENB|PARODD);
      break;
    case USART::Parity::ODD:
      tos.c_cflag |= PARENB|PARODD;
      break;
    case USART::Parity::EVEN:
      tos.c_cflag |= PARENB;
      tos.c_cflag &= ~PARODD;
      break;
  }
  if(usart_.stopbits() == 1) {
    tos.c_cflag &= ~CSTOPB;
  } else {
    tos.c_cflag |= CSTOPB;
  }

  tos.c_cc[VMIN] = 1;
  tos.c_cc[VTIME] = 0;

  if(::tcsetattr(fd_, TCSANOW, &tos)) {
    throw std::runtime_error("tcsetattr() failed");
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
      link_ = std::unique_ptr<USARTLink>(new USARTLinkSerial(*this, link_path));
    } else if(link_type == "file") {
      link_ = std::unique_ptr<USARTLink>(new USARTLinkFile(*this, link_path));
    } else {
      throw std::runtime_error(name() + ": invalid link_type value");
    }
  } else {
    link_ = std::unique_ptr<USARTLink>(new USARTLinkDummy(*this));
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
      LOGF(ERROR, "invalid CTRLC.CHSIZE value");
      vreg.chsize = 3; // 8-bit
    }
    if(vreg.pmode == 1) {
      LOGF(ERROR, "invalid CTRLC.PMODE value: 1");
      vreg.pmode = 0;
    }
    ctrlc_.data = vreg.data;
    configure();
  } else if(addr == 0x06) { // BAUDCTRLA
    baudrate_ = (baudrate_ & 0xF00) | v;
    if(baudrate_ == 0 && baudscale_ != 0) {
      LOGF(ERROR, "if BSEL is 0, BSCALE must be 0 too");
      baudscale_ = 0;
    }
    configure();
  } else if(addr == 0x07) { // BAUDCTRLB
    int8_t scale = u8_to_s8<4>(v & 0xF);
    if(scale == -8) {
      LOGF(ERROR, "invalid baudrate scale: -8");
      scale = 0;
    }
    baudrate_ = (baudrate_ & 0x0FF) | ((v & 0xF0) << 4);
    if(baudrate_ == 0 && scale != 0) {
      LOGF(ERROR, "if BSEL is 0, BSCALE must be 0 too");
      scale = 0;
    }
    baudscale_ = scale;
    configure();
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
  configure();
  next_recv_tick_ = 0;
  next_send_tick_ = 0;
}


void USART::step()
{
  unsigned int sys_tick = device_->clk_sys_tick();
  if(ctrlb_.rxen && sys_tick >= next_recv_tick_) {
    int v = link_->recv();
    if(v >= 0) {
      v &= (1 << databits())-1;
      next_recv_tick_ = sys_tick + frame_sys_ticks_ * device_->getClockScale(Device::ClockType::PER);
      if(status_.rxcif) {
        status_.bufofv = 1;
      } else {
        DLOGF(NOTICE, "%s received %02X") % name() % v;
        rxb_ = v;
        status_.rxb8 = v & 0x100;
        status_.rxcif = 1;
        setIvLvl(IV_RXC, rxc_intlvl_);
      }
    }
  }
  if(ctrlb_.txen) {
    if(!status_.dreif && sys_tick >= next_send_tick_) {
      uint16_t v = (txb_ | (ctrlb_.txb8 << 8)) & ((1 << databits())-1);
      if(link_->send(v)) {
        next_send_tick_ = sys_tick + frame_sys_ticks_ * device_->getClockScale(Device::ClockType::PER);
        DLOGF(NOTICE, "%s send %02X") % name() % v;
        //TODO TXC and DRE should not be triggered simultaneously
        status_.dreif = 1;
        status_.txcif = 1;
        setIvLvl(IV_DRE, dre_intlvl_);
        setIvLvl(IV_TXC, txc_intlvl_);
      }
    }
  }
}


unsigned int USART::baudrate() const
{
  // note: same formula in configure()
  unsigned int bit_ticks = (ctrlb_.clk2x ? 8 : 16) *
      (baudscale_ >= 0 ? ((baudrate_ + 1) << baudscale_) : (baudrate_ >> -baudscale_) + 1);
  return device_->getClockFrequency(Device::ClockType::PER) / bit_ticks;
}


void USART::configure()
{
  //TODO only supports "ctrlc_.cmod == 0" (asynchronous mode)
  // for SPI, computed values are different
  if(databits() > 8) {
    LOGF(WARNING, "%s: 9-bit character mode is not fully supported") % name();
  }

  // number of bits per frame
  unsigned int frame_bits = 1 + databits() + (parity() != Parity::NO) + stopbits();
  // PER ticks per bit
  // note: same formula in baudrate()
  unsigned int bit_ticks = (ctrlb_.clk2x ? 8 : 16) *
      (baudscale_ >= 0 ? ((baudrate_ + 1) << baudscale_) : (baudrate_ >> -baudscale_) + 1);
  frame_sys_ticks_ = frame_bits * bit_ticks;
  link_->configure();

  DLOGF(NOTICE, "%s reconfigured: %u bauds") % name() % baudrate();
}


}


