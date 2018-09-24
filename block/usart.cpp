#ifdef __WIN32
#include "../win32.h"
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
  USARTLinkFile(const USART& usart, const std::string& path_in, const std::string& path_out);
  virtual ~USARTLinkFile();
  virtual void configure() {}
  virtual int recv();
  virtual bool send(uint16_t v);
 protected:
  void initHandles();

  HANDLE h_in_;
  HANDLE h_out_;
  struct {
    OVERLAPPED o;
    uint8_t data;
    bool waiting;
  } state_read_, state_write_;
};

USARTLinkFile::USARTLinkFile(const USART& usart, const std::string& path):
    USARTLink(usart),
    h_in_(INVALID_HANDLE_VALUE),
    h_out_(INVALID_HANDLE_VALUE),
    state_read_{}, state_write_{}
{
  HANDLE h = CreateFile(path.c_str(), GENERIC_READ|GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
  if(h == INVALID_HANDLE_VALUE) {
    DWORD error = GetLastError();
    throw win32_error(error, usart_.name()+": failed to open link file "+path);
  }
  h_in_ = h_out_ = h;
  initHandles();
}

USARTLinkFile::USARTLinkFile(const USART& usart, const std::string& path_in, const std::string& path_out):
    USARTLink(usart),
    h_in_(INVALID_HANDLE_VALUE),
    h_out_(INVALID_HANDLE_VALUE),
    state_read_{}, state_write_{}
{
  if(!path_in.empty()) {
    h_in_ = CreateFile(path_in.c_str(), GENERIC_READ, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
    if(h_in_ == INVALID_HANDLE_VALUE) {
      DWORD error = GetLastError();
      throw win32_error(error, usart_.name()+": failed to open link input file "+path_in);
    }
  }

  if(!path_out.empty()) {
    h_out_ = CreateFile(path_out.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_FLAG_OVERLAPPED, nullptr);
    if(h_out_ == INVALID_HANDLE_VALUE) {
      DWORD error = GetLastError();
      throw win32_error(error, usart_.name()+": failed to open link output file "+path_out);
    }
  }

  initHandles();
}

void USARTLinkFile::initHandles()
{
  if(h_in_ != INVALID_HANDLE_VALUE) {
    state_read_.o.Offset = 0;
    state_read_.o.OffsetHigh = 0;
    state_read_.o.hEvent = CreateEvent(nullptr, true, false, nullptr);
    if(!state_read_.o.hEvent) {
      DWORD error = GetLastError();
      throw win32_error(error, usart_.name()+": CreateEvent() failed");
    }
  }
  if(h_out_ != INVALID_HANDLE_VALUE) {
    state_write_.o.Offset = 0xFFFFFFFF;
    state_write_.o.OffsetHigh = 0xFFFFFFFF;
    state_write_.o.hEvent = CreateEvent(nullptr, true, false, nullptr);
    if(!state_write_.o.hEvent) {
      DWORD error = GetLastError();
      throw win32_error(error, usart_.name()+": CreateEvent() failed");
    }
  }
}

USARTLinkFile::~USARTLinkFile()
{
  if(h_in_ != INVALID_HANDLE_VALUE) {
    CancelIo(h_in_);
    CloseHandle(h_in_);
    CloseHandle(state_read_.o.hEvent);
  }
  if(h_out_ != INVALID_HANDLE_VALUE) {
    if(h_in_ != h_out_) {
      CancelIo(h_out_);
      CloseHandle(h_out_);
    }
    CloseHandle(state_write_.o.hEvent);
  }
}

int USARTLinkFile::recv()
{
  if(h_in_ == INVALID_HANDLE_VALUE) {
    return -1;
  }
  if(!state_read_.waiting) {
    ResetEvent(state_read_.o.hEvent);
    if(!ReadFile(h_in_, &state_read_.data, 1, nullptr, &state_read_.o)) {
      DWORD error = GetLastError();
      if(error != ERROR_IO_PENDING) {
        throw win32_error(error, usart_.name()+": read error");
      }
      state_read_.waiting = true;
      return -1;
    } else {
      return state_read_.data;
    }
  } else {
    DWORD count;
    if(!GetOverlappedResult(h_in_, &state_read_.o, &count, false)) {
      DWORD error = GetLastError();
      if(error != ERROR_IO_PENDING && error != ERROR_IO_INCOMPLETE) {
        throw win32_error(error, usart_.name()+": read error");
      }
      return -1;
    } else {
      state_read_.o.Offset += count;
      state_read_.waiting = false;
      return state_read_.data;
    }
  }
}

bool USARTLinkFile::send(uint16_t v)
{
  if(h_out_ == INVALID_HANDLE_VALUE) {
    return true;
  }
  if(!state_write_.waiting) {
    ResetEvent(state_write_.o.hEvent);
    state_write_.data = v;
    if(!WriteFile(h_out_, &state_write_.data, 1, nullptr, &state_write_.o)) {
      DWORD error = GetLastError();
      if(error != ERROR_IO_PENDING) {
        throw win32_error(error, usart_.name()+": write error");
      }
      state_write_.waiting = true;
      return false;
    } else {
      return true;
    }
  } else {
    DWORD dummy = 0;
    if(!GetOverlappedResult(h_out_, &state_write_.o, &dummy, false)) {
      DWORD error = GetLastError();
      if(error != ERROR_IO_PENDING && error != ERROR_IO_INCOMPLETE) {
        throw win32_error(error, usart_.name()+": write error");
      }
      return false;
    } else {
      state_write_.waiting = false;
      return true;
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
  if(!GetCommState(h_in_, &dcb)) {
    DWORD error = GetLastError();
    throw win32_error(error, usart_.name()+": GetCommState() failed");
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
  if(!SetCommState(h_in_, &dcb)) {
    DWORD error = GetLastError();
    throw win32_error(error, usart_.name()+": SetCommState() failed");
  }
}


#else

/// USART link to plain file
class USARTLinkFile: public USARTLink
{
 public:
  USARTLinkFile(const USART& usart, const std::string& path);
  USARTLinkFile(const USART& usart, const std::string& path_in, const std::string& path_out);
  virtual ~USARTLinkFile();
  virtual void configure() {}
  virtual int recv();
  virtual bool send(uint16_t v);
 protected:
  int fd_in_;
  int fd_out_;
};

USARTLinkFile::USARTLinkFile(const USART& usart, const std::string& path):
    USARTLink(usart), fd_in_(0), fd_out_(0)
{
  int fd = ::open(path.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(fd < 0) {
    throw std::runtime_error("failed to open USART link file: "+path);
  }
  fd_in_ = fd_out_ = fd;
}

USARTLinkFile::USARTLinkFile(const USART& usart, const std::string& path_in, const std::string& path_out):
    USARTLink(usart), fd_in_(0), fd_out_(0)
{
  if(!path_in.empty()) {
    fd_in_ = ::open(path_in.c_str(), O_RDONLY|O_NOCTTY|O_NONBLOCK);
    if(fd_in_ < 0) {
      throw std::runtime_error("failed to open USART link input file: "+path_in);
    }
  }
  if(!path_out.empty()) {
    fd_out_ = ::open(path_out.c_str(), O_WRONLY|O_TRUNC|O_CREAT|O_NOCTTY|O_NONBLOCK, 0644);
    if(fd_out_ < 0) {
      throw std::runtime_error("failed to open USART link output file: "+path_out);
    }
  }
}

USARTLinkFile::~USARTLinkFile()
{
  if(fd_in_) {
    ::close(fd_in_);
  }
  if(fd_out_ && fd_out_ != fd_in_) {
    ::close(fd_out_);
  }
}

int USARTLinkFile::recv()
{
  if(!fd_in_) {
    return -1;
  }
  uint8_t c;
  ssize_t n = ::read(fd_in_, &c, 1);
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
  if(!fd_out_) {
    return true;
  }
  uint8_t v8 = v; // keep only 8 bits
  ssize_t n = ::write(fd_out_, &v8, 1);
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
    logger->info("{} ptsname: {}", usart_.name(), ::ptsname(fd_in_));
    if(::grantpt(fd_in_)) {
      throw std::runtime_error("granpt() failed");
    }
    if(::unlockpt(fd_in_)) {
      throw std::runtime_error("unlockpt() failed");
    }
  }
}

void USARTLinkSerial::configure()
{
  struct termios tos;
  if(::tcgetattr(fd_in_, &tos)) {
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

  if(::tcsetattr(fd_in_, TCSANOW, &tos)) {
    throw std::runtime_error("tcsetattr() failed");
  }
}


#endif



// USART block

USART::USART(Device& dev, const Instance<USART>& instance):
    Block(dev, instance.name, instance.io_addr, instance.iv_base)
{
  const ConfTree& conf = this->conf();
  std::string link_type = conf.get<std::string>("link_type", "");
  if(link_type.empty()) {
    link_ = std::make_unique<USARTLinkDummy>(*this);
  } else if(link_type == "serial") {
    std::string link_path = conf.get<std::string>("link_path", "");
#ifdef __WIN32
    if(link_path.find_first_of("\\/") == std::string::npos) {
      link_path = "\\\\.\\" + link_path;
    }
#endif
    link_ = std::make_unique<USARTLinkSerial>(*this, link_path);
  } else if(link_type == "file") {
    std::string link_path = conf.get<std::string>("link_path", "");
    std::string link_in = conf.get<std::string>("link_in", "");
    std::string link_out = conf.get<std::string>("link_out", "");
    if(link_path.empty() && link_in.empty() && link_out.empty()) {
      throw std::runtime_error(name() + ": missing link_path or link_in or link_out");
    } else if(!link_path.empty() && !(link_in.empty() && link_out.empty())) {
      throw std::runtime_error(name() + ": link_path and link_in/link_out are mutually exclusive");
    } else if(!link_path.empty()) {
      link_ = std::make_unique<USARTLinkFile>(*this, link_path);
    } else {
      link_ = std::make_unique<USARTLinkFile>(*this, link_in, link_out);
    }
  } else {
    throw std::runtime_error(name() + ": invalid link_type value");
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
    logger_->warn("I/O read 0x{:02X}: reserved address", addr);
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
    if(step_event_) {
      if(!ctrlb_.txen && !ctrlb_.rxen) {
        device_.unschedule(step_event_);
        step_event_ = nullptr;
      }
    } else {
      if(ctrlb_.txen || ctrlb_.rxen) {
        step_event_ = device_.schedule(ClockType::PER, [&]() { return step(); });
      }
    }
  } else if(addr == 0x05) { // CTRLC
    CTRLC vreg = { .data=v };
    if(vreg.cmode != 0) {
      logger_->warn("only asynchronous mode is supported");
      vreg.cmode = 0;
    }
    if(vreg.chsize > 3 && vreg.chsize != 7) {
      logger_->error("invalid CTRLC.CHSIZE value");
      vreg.chsize = 3; // 8-bit
    }
    if(vreg.pmode == 1) {
      logger_->error("invalid CTRLC.PMODE value: 1");
      vreg.pmode = 0;
    }
    ctrlc_.data = vreg.data;
    configure();
  } else if(addr == 0x06) { // BAUDCTRLA
    baudrate_ = (baudrate_ & 0xF00) | v;
    if(baudrate_ == 0 && baudscale_ != 0) {
      logger_->error("if BSEL is 0, BSCALE must be 0 too");
      baudscale_ = 0;
    }
    configure();
  } else if(addr == 0x07) { // BAUDCTRLB
    int8_t scale = u8_to_s8<4>(v & 0xF);
    if(scale == -8) {
      logger_->error("invalid baudrate scale: -8");
      scale = 0;
    }
    baudrate_ = (baudrate_ & 0x0FF) | ((v & 0xF0) << 4);
    if(baudrate_ == 0 && scale != 0) {
      logger_->error("if BSEL is 0, BSCALE must be 0 too");
      scale = 0;
    }
    baudscale_ = scale;
    configure();
  } else {
    logger_->error("I/O write 0x{:02X}: not writable", addr);
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
  rxc_intlvl_ = INTLVL_NONE;
  dre_intlvl_ = INTLVL_NONE;
  txc_intlvl_ = INTLVL_NONE;
  baudrate_ = 0;
  baudscale_ = 0;
  rxb_ = 0;
  txb_ = 0;
  step_event_ = nullptr;
  configure();
  next_recv_tick_ = 0;
  next_send_tick_ = 0;
}


unsigned int USART::step()
{
  unsigned int sys_tick = device_.clk_sys_tick();
  if(ctrlb_.rxen && sys_tick >= next_recv_tick_) {
    int v = link_->recv();
    if(v >= 0) {
      v &= (1 << databits())-1;
      next_recv_tick_ = sys_tick + frame_sys_ticks_ * device_.getClockScale(ClockType::PER);
      if(status_.rxcif) {
        status_.bufofv = 1;
      } else {
        logger_->debug("received {:02X}", v);
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
        next_send_tick_ = sys_tick + frame_sys_ticks_ * device_.getClockScale(ClockType::PER);
        logger_->debug("send {:02X}", v);
        //TODO TXC and DRE should not be triggered simultaneously
        status_.dreif = 1;
        status_.txcif = 1;
        setIvLvl(IV_DRE, dre_intlvl_);
        setIvLvl(IV_TXC, txc_intlvl_);
      }
    }
  }
  return 1;
}


unsigned int USART::baudrate() const
{
  // note: same formula in configure()
  unsigned int bit_ticks = (ctrlb_.clk2x ? 8 : 16) *
      (baudscale_ >= 0 ? ((baudrate_ + 1) << baudscale_) : (baudrate_ >> -baudscale_) + 1);
  return device_.getClockFrequency(ClockType::PER) / bit_ticks;
}


void USART::configure()
{
  //TODO only supports "ctrlc_.cmod == 0" (asynchronous mode)
  // for SPI, computed values are different
  if(databits() > 8) {
    logger_->warn("9-bit character mode is not fully supported");
  }

  // number of bits per frame
  unsigned int frame_bits = 1 + databits() + (parity() != Parity::NO) + stopbits();
  // PER ticks per bit
  // note: same formula in baudrate()
  unsigned int bit_ticks = (ctrlb_.clk2x ? 8 : 16) *
      (baudscale_ >= 0 ? ((baudrate_ + 1) << baudscale_) : (baudrate_ >> -baudscale_) + 1);
  frame_sys_ticks_ = frame_bits * bit_ticks;
  link_->configure();

  logger_->info("reconfigured: {} bauds", baudrate());
}


}


