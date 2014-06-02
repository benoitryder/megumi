#ifndef BLOCK_TC_H__
#define BLOCK_TC_H__

#include <memory>
#include "../block.h"

namespace block {

/** @brief TC 16-bit Timer/Counter peripheral
 */
class TC: public Block
{
  static const ioptr_t IO_SIZE = 0x40;
  enum {
    IV_OVF = 0,
    IV_ERR,
    IV_CCA,
    IV_CCB,
    IV_CCC,
    IV_CCD,
    IV_COUNT
  };

 public:
  TC(Device* dev, const Instance<TC>& instance);
  virtual ~TC();

  virtual ioptr_t io_size() const { return IO_SIZE; }
  virtual ivnum_t iv_count() const { return type_ == 0 ? IV_COUNT : IV_CCC; }

  virtual uint8_t getIo(ioptr_t addr);
  virtual void setIo(ioptr_t addr, uint8_t v);
  virtual void executeIv(ivnum_t iv);
  virtual void reset();
  unsigned int step();

  /// Return TC type (0 or 1)
  unsigned int type() const { return type_; }

  /// Return true if TC is in OFF state
  bool off() const { return prescaler_ == 0; }

 private:
  unsigned int type_; // 0 (TCx0) or 1 (TCx1)

  /// Clock source prescaler value
  unsigned int prescaler_;

  union CTRLB {
    uint8_t data;
    BitField<0,2> wgmode;
    BitField<4> ccaen;
    BitField<5> ccben;
    BitField<6> cccen;
    BitField<7> ccden;
  } ctrlb_;

  union CTRLC {
    uint8_t data;
    BitField<0> cmpa;
    BitField<1> cmpb;
    BitField<2> cmpc;
    BitField<3> cmpd;
  } ctrlc_;

  IntLvl ovf_intlvl_;
  IntLvl err_intlvl_;
  IntLvl cca_intlvl_;
  IntLvl ccb_intlvl_;
  IntLvl ccc_intlvl_;
  IntLvl ccd_intlvl_;

  union CTRLF {
    uint8_t data;
    BitField<0> dir;
    BitField<1> lupd;
    BitField<2,2> cmd;
  } ctrlf_;

  union CTRLG {
    uint8_t data;
    BitField<0> perbv;
    BitField<1> ccabv;
    BitField<2> ccbbv;
    BitField<3> cccbv;
    BitField<4> ccdbv;
  } ctrlg_;

  union INTFLAGS {
    uint8_t data;
    BitField<0> ovfif;
    BitField<1> errif;
    BitField<2> ccaif;
    BitField<3> ccbif;
    BitField<4> cccif;
    BitField<5> ccdif;
  } intflags_;

  uint8_t temp_;
  uint16_t cnt_;

  uint16_t per_;
  uint16_t cca_;
  uint16_t ccb_;
  uint16_t ccc_;
  uint16_t ccd_;

  uint16_t perbuf_;
  uint16_t ccabuf_;
  uint16_t ccbbuf_;
  uint16_t cccbuf_;
  uint16_t ccdbuf_;

  // Scheduled step() event
  const ClockEvent* step_event_;
};


namespace instances {
  const Instance<TC> TCC0 = { "TCC0", 0x0800,  14 };
  const Instance<TC> TCC1 = { "TCC1", 0x0840,  20 };
  const Instance<TC> TCD0 = { "TCD0", 0x0900,  77 };
  const Instance<TC> TCD1 = { "TCD1", 0x0940,  83 };
  const Instance<TC> TCE0 = { "TCE0", 0x0A00,  47 };
  const Instance<TC> TCE1 = { "TCE1", 0x0A40,  53 };
  const Instance<TC> TCF0 = { "TCF0", 0x0B00, 108 };
  const Instance<TC> TCF1 = { "TCF1", 0x0B40, 114 };
}

}

#endif
