#ifndef BLOCK_GPIOR_H
#define BLOCK_GPIOR_H

#include <array>
#include "../block.h"

namespace block {


/** @brief General Purpose I/O Register block
 */
class GPIOR: public Block
{
  static constexpr ioptr_t IO_SIZE = 16;

 public:
  GPIOR(Device& dev);
  virtual ~GPIOR() = default;

  ioptr_t io_size() const override { return IO_SIZE; }

  uint8_t getIo(ioptr_t addr) override;
  void setIo(ioptr_t addr, uint8_t v) override;
  void reset() override;

 private:
  std::array<uint8_t, IO_SIZE> data_;
};



}

#endif
