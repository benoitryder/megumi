#ifndef BLOCK_PORT_H__
#define BLOCK_PORT_H__

#include "../block.h"

namespace block {


/** @brief I/O port block
 */
class Port: public Block
{
  static const ioptr_t IO_SIZE = 32;
  static const ivnum_t IV_COUNT = 2;

 public:
  Port(Device* dev, const Instance<Port>& instance);
  virtual ~Port();

  virtual ioptr_t io_size() const { return IO_SIZE; }
  virtual ivnum_t iv_count() const { return IV_COUNT; }

  virtual uint8_t getIo(ioptr_t addr);
  virtual void setIo(ioptr_t addr, uint8_t v);
  virtual void reset();
  //TODO interrupts
  //TODO input pin values, received from the outside

 private:
  uint8_t dir_;  ///< Data direction bitmask (1 is OUT, 0 is IN)
  uint8_t out_;  ///< Output value
};


namespace instances {
  const Instance<Port> PORTA = { "PORTA", 0x0600,  66 };
  const Instance<Port> PORTB = { "PORTB", 0x0620,  34 };
  const Instance<Port> PORTC = { "PORTC", 0x0640,   2 };
  const Instance<Port> PORTD = { "PORTD", 0x0660,  64 };
  const Instance<Port> PORTE = { "PORTE", 0x0680,  43 };
  const Instance<Port> PORTF = { "PORTF", 0x06A0, 104 };
  const Instance<Port> PORTH = { "PORTH", 0x06E0,  96 };
  const Instance<Port> PORTJ = { "PORTJ", 0x0700,  98 };
  const Instance<Port> PORTK = { "PORTK", 0x0720, 100 };
  const Instance<Port> PORTQ = { "PORTQ", 0x07C0,  94 };
  const Instance<Port> PORTR = { "PORTR", 0x07E0,   4 };
}

}

#endif
