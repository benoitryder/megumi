#ifndef BLOCK_CPU_H
#define BLOCK_CPU_H

#include "../block.h"

namespace block {


/** @brief CPU core registers
 *
 * This block defines CPU core registers and the device strongly depends on it.
 */
class CPU: public Block
{
  friend class ::Device;
  static const ioptr_t IO_SIZE = 16;

 public:
  CPU(Device& dev);
  virtual ~CPU();

  ioptr_t io_size() const override { return IO_SIZE; }

  uint8_t getIo(ioptr_t addr) override;
  void setIo(ioptr_t addr, uint8_t v) override;
  void reset() override;
  unsigned int step();

  /// Return CCP state as read in the I/O register
  uint8_t ccpState() const {
    return (ccp_ioreg_cycles_ ? 0x01 : 0) | (ccp_spm_cycles_ ? 0x02 : 0);
  }

  union SREG {
    uint8_t data;
    BitField<0> C;
    BitField<1> Z;
    BitField<2> N;
    BitField<3> V;
    BitField<4> S;
    BitField<5> H;
    BitField<6> T;
    BitField<7> I;
  };

 private:
  uint8_t ccp_ioreg_cycles_;  ///< Timeout in cycles for I/O register protection
  uint8_t ccp_spm_cycles_;  ///< Timeout in cycles for SPM protection
  uint8_t rampd_;
  uint8_t rampx_;
  uint8_t rampy_;
  uint8_t rampz_;
  uint8_t eind_;
  uint16_t sp_;
  SREG sreg_;

  const uint8_t ramp_mask_;  ///< Mask for RAMP writes
  const uint8_t eind_mask_;  ///< Mask for EIND writes

  // CPU values not accessed through I/O registers
  flashptr_t pc_; ///< Program counter (PC), in 16-bit words
  /** @brief Pending CCP change
   *
   * The new value is buffered for more accurate cycle timeout.
   */
  uint8_t ccp_buffer_;
};



}

#endif
