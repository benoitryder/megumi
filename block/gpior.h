#ifndef BLOCK_GPIOR_H__
#define BLOCK_GPIOR_H__

#include <array>
#include "../block.h"

namespace block {


/** @brief General Purpose I/O Register block
 */
class GPIOR: public Block
{
  static const ioptr_t IO_SIZE = 16;

 public:
  GPIOR(Device* dev);
  virtual ~GPIOR();

  virtual ioptr_t io_size() const { return IO_SIZE; }

  virtual uint8_t getIo(ioptr_t addr);
  virtual void setIo(ioptr_t addr, uint8_t v);
  virtual void reset();

 private:
  std::array<uint8_t, IO_SIZE> data_;
};



}

#endif
