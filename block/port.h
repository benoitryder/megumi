#ifndef BLOCK_PORT_H
#define BLOCK_PORT_H

#include "../block.h"

namespace block {


/** @brief I/O port block
 */
class Port: public Block
{
  static constexpr ioptr_t IO_SIZE = 32;
  static constexpr ivnum_t IV_COUNT = 2;

 public:
  Port(Device& dev, const Instance<Port>& instance);
  virtual ~Port() = default;

  ioptr_t io_size() const override { return IO_SIZE; }
  ivnum_t iv_count() const override { return IV_COUNT; }

  uint8_t getIo(ioptr_t addr) override;
  void setIo(ioptr_t addr, uint8_t v) override;
  void reset() override;
  //TODO interrupts
  //TODO input pin values, received from the outside

 private:
  uint8_t dir_;  ///< Data direction bitmask (1 is OUT, 0 is IN)
  uint8_t out_;  ///< Output value
};


namespace instances {
  constexpr Instance<Port> PORTA = { "PORTA", 0x0600,  66 };
  constexpr Instance<Port> PORTB = { "PORTB", 0x0620,  34 };
  constexpr Instance<Port> PORTC = { "PORTC", 0x0640,   2 };
  constexpr Instance<Port> PORTD = { "PORTD", 0x0660,  64 };
  constexpr Instance<Port> PORTE = { "PORTE", 0x0680,  43 };
  constexpr Instance<Port> PORTF = { "PORTF", 0x06A0, 104 };
  constexpr Instance<Port> PORTH = { "PORTH", 0x06E0,  96 };
  constexpr Instance<Port> PORTJ = { "PORTJ", 0x0700,  98 };
  constexpr Instance<Port> PORTK = { "PORTK", 0x0720, 100 };
  constexpr Instance<Port> PORTQ = { "PORTQ", 0x07C0,  94 };
  constexpr Instance<Port> PORTR = { "PORTR", 0x07E0,   4 };
}

}

#endif
