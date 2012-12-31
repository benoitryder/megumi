#ifdef __WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#undef ERROR
#define close closesocket
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#endif
#include <errno.h>
#include <string.h>
#include "gdbserver.h"
#include "device.h"
#include "common.h"
#include "log.h"


// parse/build helper methods

static char hex_digits[] = "0123456789abcdef";


static int parse_hex_digit(char c)
{
  if(c >= '0' && c <= '9') return c-'0';
  c |= ' ';
  if(c >= 'a' && c <= 'f') return (c-'a')+0xa;
  return -1;
}

/** @brief Parse a little endian hexadecimal value
 *
 * @note \e buf is assumed to be large enough
 */
template <unsigned N>
typename nbits_to_utype<N>::type parse_hex_le(const char* buf)
{
  typename nbits_to_utype<N>::type ret = 0;
  for(size_t i=0; i<N/4; i+=2) {
    int d0 = parse_hex_digit(buf[i]);
    int d1 = parse_hex_digit(buf[i]);
    if(d0 == -1 || d1 == -1) {
      throw GdbServerError("invalid hex value");
    }
    ret += (d0 << (i+1)*4) | (d1 << i*4);
  }
  return ret;
}

/** @brief Parse a big-endian hexadecimal value until a given delimiter
 *
 * \e buf should be NUL terminated.
 *
 * @return A pointer pointing after the found delimiter.
 * @note Only supports values up to INT_MAX.
 */
const char* parse_hex_be_until(unsigned int* v, const char* buf, char delim)
{
  *v = 0;
  while(*buf != delim) {
    if(*buf == '\0') {
      throw GdbServerError("delimiter not found");
    }
    int d = parse_hex_digit(*buf++);
    if(d == -1) {
      throw GdbServerError("invalid hex value");
    }
    *v = (*v << 4) + d;
  }
  return buf+1;
}


/// Push a little-endian hexadecimal value to a string
template <unsigned N>
void build_hex_le(std::string* data, typename nbits_to_utype<N>::type v)
{
  char digits[N/4];
  for(size_t i=0; i<sizeof(digits); i+=2) {
    digits[i] = hex_digits[(v >> (i+1)*4) & 0xf];
    digits[i+1] = hex_digits[(v >> i*4) & 0xf];
  }
  data->append(digits, sizeof(digits));
}

/** @brief Build a big-endian hexadecimal value
 *
 * \e buf should be NUL terminated.
 *
 * @return A pointer pointing after the found delimiter.
 * @note Only supports values up to INT_MAX.
 */
void build_hex_be(std::string* data, unsigned int v)
{
  unsigned int b = 4*(sizeof(v)-1);
  // skip leading 0
  for(; b > 4 && (v & (0xf << b)) == 0; b-=4) ;
  char digits[b/4];
  unsigned int i = 0;
  for(;;) {
    digits[i] = hex_digits[(v >> b) & 0xf];
    if(b == 0) {
      break;
    }
    b -= 4;
    i++;
  }
  data->append(digits, sizeof(digits));
}



GdbServer::GdbServer(Device* dev):
    device_(dev),
    sock_client_(-1)
{
}

GdbServer::~GdbServer()
{
#ifdef __WIN32
  WSACleanup();
#endif
}


void GdbServer::run(int port)
{
#ifdef __WIN32
  WSADATA wsa_data;
  WSAStartup(MAKEWORD(2,2), &wsa_data);
#endif

  // initialize server socket
  int sock_server = ::socket(PF_INET, SOCK_STREAM, 0);
  if(sock_server < 0) {
    throw GdbServerError(errno, "failed to create server socket");
  }
  int optval = 1;
#ifdef __WIN32
  const char* optval_ptr = (char*)&optval;
#else
  const int* optval_ptr = &optval;
#endif
  ::setsockopt(sock_server, SOL_SOCKET, SO_REUSEADDR, optval_ptr, sizeof(optval));
  // enabling "nodelay" provide a huge boost of communication speed
  ::setsockopt(sock_server, IPPROTO_TCP, TCP_NODELAY, optval_ptr, sizeof(optval));

  struct ::sockaddr_in addr = {
    .sin_family = AF_INET,
    .sin_port = htons(port),
  };

  if(::bind(sock_server, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    ::close(sock_server);
    throw GdbServerError(errno, "failed to bind server socket");
  }

  // listen on server socket, waiting for client
  if(::listen(sock_server, 1) < 0) {
    ::close(sock_server);
    throw GdbServerError(errno, "failed to listen on server socket");
  }
  LOGF(NOTICE, "GDB server: listening on port %d") % port;

  sock_client_ = ::accept(sock_server, NULL, NULL);
  if(sock_client_ < 0) {
    ::close(sock_server);
    throw GdbServerError(errno, "accept() failed");
  }
  LOGF(NOTICE, "GDB server: client connected");

  try {
    // handle client 
    for(;;) {
      processPacket(recvPacket());
    }
  } catch(const std::exception&) {
    ::close(sock_client_);
    ::close(sock_server);
    sock_client_ = -1;
    throw;
  }
}


void GdbServer::execContinue()
{
  // always step once
  execStep();
  for(;;) {
    // handle BREAK and breakpoints
    device_->step();
    if(breakpoints_.find(device_->getPC()) != breakpoints_.end()) {
      return;
    } else if(device_->breaked()) {
      return;
    }
  }
}


void GdbServer::execStep()
{
  // skip steps which does not change PC (e.g. multi-cycles instructions)
  flashptr_t pc = device_->getPC();
  do {
    device_->step();
  } while(pc == device_->getPC());
}


void GdbServer::processPacket(const std::string& data)
{
  if(data.size() < 1) {
    DLOGF(WARNING, "GDB server: empty packet data");
    return;
  }

  //TODO some commands silently fails (eg. SP overflow)
  DLOGF(INFO, "GDB server: received command: %s") % data;

  std::string reply;
  try {
    switch(data[0]) {
      // general query
      case 'q': {
        std::string subcmd = data.substr(1, data.find_first_of(",:;")-1);
        if(subcmd == "Supported") {
          reply = "PacketSize=1024";
        } else if(subcmd == "C") {
          build_hex_be(&reply, 1); // only use thread 1
        } else {
          DLOGF(INFO, "GDB server: unhandled query: %s") % subcmd;
        }
      } break;

      // set thread
      case 'H': {
        if(data.size() < 1+1) {
          throw GdbServerError("not enought data");
        }
        if(data[1] != 'g' && data[1] != 'c') {
          // should not happen
        } else if(data.substr(2) == "-1") {
          break; // -1: all threads, not supported
        } else {
          // only accept thread 1, and 0 (any)
          unsigned int n;
          parse_hex_be_until(&n, data.c_str()+2, '\0');
          if(n == 0 || n == 1) {
            reply = "OK";
          }
        }
      } break;

      // read general registers
      // GDB excepts 35 values for a total of 39 bytes
      //    0-32  regfile content (1-byte each)
      //      33  SREG (1-byte)
      //   34-35  SP (2-byte)
      //   36-39  PC (4-byte)
      case 'g': {
        reply.reserve(2*39);
        for(uint8_t v : device_->regfile()) {
          build_hex_le<8>(&reply, v);
        }
        build_hex_le<8>(&reply, device_->getSREG().data);
        build_hex_le<16>(&reply, device_->getSP());
        build_hex_le<32>(&reply, 2*device_->getPC());
      } break;

      // write general registers
      // (see register list above)
      case 'G': {
        if(data.size() != 1+39) {
          throw GdbServerError("unexpected data size");
        }
        const char* p = data.data()+1;
        for(uint8_t& v : device_->regfile()) {
          v = parse_hex_le<8>(p++);
        }
        device_->setSREG(parse_hex_le<8>(p++));
        device_->setSP(parse_hex_le<16>(p)); p += 2;
        device_->setPC(parse_hex_le<32>(p)/2); p += 4;
        reply = "OK";
      } break;

      // read a single general register
      case 'p': {
        if(data.size() < 2) {
          throw GdbServerError("data is too short");
        }
        unsigned int n;
        parse_hex_be_until(&n, data.c_str()+1, '\0');
        if(n < 32) {
          build_hex_le<8>(&reply, device_->regfile()[n]);
        } else if(n == 32) {
          build_hex_le<8>(&reply, device_->getSREG().data);
        } else if(n == 33) {
          build_hex_le<16>(&reply, device_->getSP());
        } else if(n == 34) {
          build_hex_le<32>(&reply, 2*device_->getPC());
        } else {
          throw GdbServerError("invalid register number");
        }
      } break;

      // write a single general register
      case 'P': {
        if(data.size() < 2) {
          throw GdbServerError("data is too short");
        }
        unsigned int n;
        const char *p = data.c_str()+1;
        p = parse_hex_be_until(&n, p, '=');
        if(n < 32) {
          device_->regfile()[n] = parse_hex_le<8>(p);
        } else if(n == 32) {
          device_->setSREG(parse_hex_le<8>(p));
        } else if(n == 33) {
          device_->setSP(parse_hex_le<16>(p));
        } else if(n == 34) {
          device_->setPC(parse_hex_le<32>(p)/2);
        } else {
          throw GdbServerError("invalid register number");
        }
        reply = "OK";
      } break;

      // read memory
      case 'm': {
        const char* p = data.c_str()+1;
        unsigned int addr;
        p = parse_hex_be_until(&addr, p, ',');
        unsigned int len;
        p = parse_hex_be_until(&len, p, '\0');

        reply.reserve(2*len);
        for(unsigned int i=0; i<len; i++) {
          build_hex_le<8>(&reply, getGdbMem(addr+i));
        }
      } break;

      // write memory
      case 'M': {
        const char* p0 = data.c_str();
        const char* p = p0+1;
        unsigned int addr;
        p = parse_hex_be_until(&addr, p, ',');
        unsigned int len;
        p = parse_hex_be_until(&len, p, ':');

        if(data.size() != (size_t)(p-p0) + 2*len) {
          throw GdbServerError("unexpected data size");
        }
        for(unsigned int i=0; i<len; i++) {
          setGdbMem(addr+i, parse_hex_le<8>(p++));
        }
        reply = "OK";
      } break;

      // continue
      case 'c': {
        const char* p = data.c_str()+1;
        if(*p != '\0') {
          unsigned int addr;
          parse_hex_be_until(&addr, p, '\0');
          device_->setPC(addr/2);
        }
        execContinue();
        reply = buildStopReply();
      } break;

      // step
      case 's': {
        const char* p = data.c_str()+1;
        if(*p != '\0') {
          unsigned int addr;
          parse_hex_be_until(&addr, p, '\0');
          device_->setPC(addr/2);
        }
        execStep();
        reply = buildStopReply();
      } break;

      // insert breakpoint
      case 'Z': {
        const char* p = data.c_str()+1;
        unsigned int type;
        p = parse_hex_be_until(&type, p, ',');
        unsigned int addr;
        p = parse_hex_be_until(&addr, p, ',');
        unsigned int len;
        p = parse_hex_be_until(&len, p, '\0');
        if(type == 0) {
          // note: len is ignored
          breakpoints_.insert(addr/2);
          reply = "OK";
        } else {
          // not supported
        }
      } break;

      // remove breakpoint
      case 'z': {
        const char* p = data.c_str()+1;
        unsigned int type;
        p = parse_hex_be_until(&type, p, ',');
        unsigned int addr;
        p = parse_hex_be_until(&addr, p, ',');
        unsigned int len;
        p = parse_hex_be_until(&len, p, '\0');
        if(type == 0) {
          // note: len is ignored
          auto it = breakpoints_.find(addr/2);
          if(it == breakpoints_.end()) {
            throw GdbServerError("no such breakpoint");
          }
          breakpoints_.erase(it);
          reply = "OK";
        } else {
          // not supported
        }
      } break;

      // reason the target halted
      case '?': {
        reply = "S05"; // signal TRAP
      } break;

      default:
        DLOGF(INFO, "GDB server: unhandled packet: %c") % data[0];
        // not handled, empty reply
    }
  } catch(const GdbServerError& e) {
    DLOGF(ERROR, "GDB server: error for command '%c': %s") % data[0] % e.what();
    reply = "E00";
  }

  sendPacket(reply);
}


std::string GdbServer::recvPacket()
{
  bool start_seen = false;
  char buf[512];
  for(;;) {
    int ret = ::recv(sock_client_, buf, sizeof(buf), 0);
    if(ret < 0 && errno == EINTR) {
      continue;
    } else if(ret < 0) {
      throw GdbServerError(errno, "recv() failed");
    } else if(ret == 0) {
      throw GdbServerError("connection closed");
    }

    rbuf_.append(buf, ret);
    // detect packet start
    // note: +/- acknowledgment is ignored
    if(!start_seen) {
      size_t p = rbuf_.find('$');
      if(p != std::string::npos) {
        rbuf_.erase(0, p+1);
        start_seen = true;
      } else {
        continue;
      }
    }
    // detect packet end, wait for checksum
    size_t p = rbuf_.find('#');
    if(p == std::string::npos || rbuf_.size() <= p+2) {
      continue;
    }
    // checksum
    // don't use parse_hex_le as we don't want to throw execptions
    int chk0 = parse_hex_digit(rbuf_[p+1]);
    int chk1 = parse_hex_digit(rbuf_[p+2]);
    int checksum_field;
    if(chk0 == -1 || chk1 == -1) {
      checksum_field = -1;
    } else {
      checksum_field = (chk0 << 4) + chk1;
    }
    int checksum_data = 0;
    for(unsigned int i=0; i<p; i++) {
      checksum_data = (checksum_data + rbuf_[i]) & 0xff;
    }
    if(checksum_field < 0 || checksum_data != checksum_field) {
      sendAck(false); // mismatch: request retransmission
      rbuf_.erase(0, p+2);
    } else {
      sendAck(true);
      std::string data = rbuf_.substr(0, p);
      rbuf_.erase(0, p+2);
      return data;
    }
  }
}


void GdbServer::sendPacket(const std::string& data)
{
  DLOGF(INFO, "GDB server: send packet: %s") % data;
  const size_t data_size = data.size();

  // compute checksum
  unsigned int checksum = 0;
  for(unsigned int i=0; i<data_size; i++) {
    checksum = (checksum + data[i]) & 0xff;
  }

  // fill buffer to send
  const size_t buf_size = data_size+4;
  char buf[buf_size];
  buf[0] = '$';
  data.copy(buf+1, data.size());
  buf[data_size+1] = '#';
  buf[data_size+2] = hex_digits[checksum >> 4];
  buf[data_size+3] = hex_digits[checksum & 0xf];

  // send the buffer
  size_t pos = 0;
  while(pos < buf_size) {
    int ret = ::send(sock_client_, buf+pos, buf_size-pos, 0);
    if(ret < 0 && errno == EINTR) {
      continue;
    } else if(ret < 0) {
      throw GdbServerError(errno, "send() failed");
    } else {
      pos += ret;
    }
  }
}

void GdbServer::sendAck(bool ack)
{
  char c = ack ? '+' : '-';
  for(;;) {
    int ret = ::send(sock_client_, &c, 1, 0);
    if(ret < 0 && errno == EINTR) {
      continue;
    } else if(ret < 0) {
      throw GdbServerError(errno, "send() failed");
    } else if(ret > 0) {
      return;
    }
  }
}


uint8_t GdbServer::getGdbMem(unsigned int addr)
{
  // GDB uses a special addressing to allow addressing of flash, SRAM, etc.
  //  - flash starts at 0x00000000
  //  - SRAM starts at 0x00800000
  //  - mask for memory space is 0x00f00000
  // Constants used below are retrieved from GDB sources, in avr-tdep.c.

  if(addr & 0x00f00000) {
    return device_->getDataMem(addr & 0xfffff);
  } else {
    //TODO check offset for overflow?
    uint16_t word = device_->flash_data()[addr/2];
    return addr % 2 ? (word >> 8) : (word & 0xFF);
  }
}

void GdbServer::setGdbMem(unsigned int addr, uint8_t v)
{
  // see comments in getGdbMem() for addressing values
  if(addr & 0x00f00000) {
    device_->setDataMem(addr & 0xfffff, v);
  } else {
    //TODO check offset for overflow?
    uint16_t word = device_->flash_data()[addr/2];
    if(addr % 2) {
      word = (word & 0x00FF) | (v << 8);
    } else {
      word = (word & 0xFF00) | v;
    }
    device_->flash_data()[addr/2] = word;
  }
}


std::string GdbServer::buildStopReply() const
{
  std::string reply = "T0520:";
  build_hex_le<8>(&reply, device_->getSREG().data);
  reply += ";21:";
  build_hex_le<16>(&reply, device_->getSP());
  reply += ";22:";
  build_hex_le<32>(&reply, 2*device_->getPC());
  reply += ";";
  return reply;
}



GdbServerError::GdbServerError(int errnum, const std::string& msg):
    std::runtime_error(msg + ": "+::strerror(errnum)) {}

