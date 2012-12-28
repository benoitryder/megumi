#ifndef BLOCK_USART_H__
#define BLOCK_USART_H__

#include <memory>
#include "../block.h"

namespace block {

class USART;


/// USART link
class USARTLink
{
 public:
  USARTLink(const USART& usart);
  virtual ~USARTLink();
  /// Reconfigure the link
  virtual void configure() = 0;
  /// Receive a single byte, return -1 if there is nothing to read
  virtual int recv() = 0;
  /// Send a single byte, return \e false if it would block
  virtual bool send(uint8_t v) = 0;
 protected:
  const USART& usart_;
};


/** @brief USART peripheral
 *
 * Baudrate is limited by the peripheral implementation to ensure data is not
 * sent/received too fast wrt the device clock.
 * It may be limited by the link too.
 *
 * The following configuration values are defined:
 *  - \c link_path: file or device the USART is linked to
 *  - \c link_type: address type
 *
 * \e link_path is a path to a plain or special file which will be handled
 * depending on \e link_type
 *  - \c file: normal file access using usual open/read/write
 *  - \c serial: serial port, the port is configured to reflect block state
 *
 * Default type depends on the provided path:
 *  - \c /dev/tty*: \c serial (Linux)
 *  - \c COM* or \c \\.\COM*: \c serial (Windows)
 *  - \c /dev/ptmx: \c serial, create a pseudoterminal master
 *  - other: \c file
 */
class USART: public Block
{
  static const ioptr_t IO_SIZE = 8;
  enum {
    IV_RXC = 0,
    IV_DRE,
    IV_TXC,
    IV_COUNT
  };

 public:
  USART(Device* dev, const Instance<USART>& instance);
  virtual ~USART();

  virtual ioptr_t io_size() const { return IO_SIZE; }
  virtual ivnum_t iv_count() const { return IV_COUNT; }

  virtual uint8_t getIo(ioptr_t addr);
  virtual void setIo(ioptr_t addr, uint8_t v);
  virtual void executeIv(ivnum_t iv);
  virtual void reset();
  void step();

 private:
  /// Update internals after configuration change
  void configure();

  union STATUS {
    uint8_t data;
    BitField<0> rxb8;
    BitField<2> perr;
    BitField<3> bufofv;
    BitField<4> ferr;
    BitField<5> dreif;
    BitField<6> txcif;
    BitField<7> rxcif;
  } status_;

  IntLvl rxc_intlvl_;
  IntLvl dre_intlvl_;
  IntLvl txc_intlvl_;

  union CTRLB {
    uint8_t data;
    BitField<0> txb8;
    BitField<1> mpcm;
    BitField<2> clk2x;
    BitField<3> txen;
    BitField<4> rxen;
  } ctrlb_;

  union CTRLC {
    uint8_t data;
    BitField<0,3> chsize;
    BitField<3,1> sbmode;
    BitField<4,2> pmode;
    BitField<6,2> cmode;
    // master SPI mode
    BitField<1> ucpha;
    BitField<2> udord;
  } ctrlc_;

  uint16_t baudrate_;
  int8_t baudscale_;

  uint8_t rxb_;
  uint8_t txb_;

  std::unique_ptr<USARTLink> link_;
  unsigned int frame_sys_ticks_; ///< SYS ticks per frame
  unsigned int next_recv_tick_; ///< SYS tick before next received frame
  unsigned int next_send_tick_; ///< SYS tick before next sent frame
};


namespace instances {
  const Instance<USART> USARTC0 = { "USARTC0", 0x08A0,  25 };
  const Instance<USART> USARTC1 = { "USARTC1", 0x08B0,  28 };
  const Instance<USART> USARTD0 = { "USARTD0", 0x09A0,  88 };
  const Instance<USART> USARTD1 = { "USARTD1", 0x09B0,  91 };
  const Instance<USART> USARTE0 = { "USARTE0", 0x0AA0,  58 };
  const Instance<USART> USARTE1 = { "USARTE1", 0x0AB0,  61 };
  const Instance<USART> USARTF0 = { "USARTF0", 0x0BA0, 119 };
  const Instance<USART> USARTF1 = { "USARTF1", 0x0BB0, 122 };
}

}

#endif
