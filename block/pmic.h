#ifndef BLOCK_PMIC_H__
#define BLOCK_PMIC_H__

#include "../block.h"

namespace block {


/** @brief Programmable Multi-level Interrupt Controller
 */
class PMIC: public Block
{
  friend class ::Device;
  static const ioptr_t IO_SIZE = 3;

 public:
  PMIC(Device* dev);
  virtual ~PMIC();

  virtual ioptr_t io_size() const { return IO_SIZE; }

  virtual uint8_t getIo(ioptr_t addr);
  virtual void setIo(ioptr_t addr, uint8_t v);
  virtual void reset();
  virtual void step();

 private:
  union {
    uint8_t data;
    BitField<0> lolvlex;
    BitField<1> medlvlex;
    BitField<2> hilvlex;
    BitField<7> nmiex;
  } status_;
  union {
    uint8_t data;
    BitField<0> lolvlen;
    BitField<1> medlvlen;
    BitField<2> hilvlen;
    BitField<6> ivsel;
    BitField<7> rren;
  } ctrl_;
};



}

#endif
