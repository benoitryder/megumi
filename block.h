#ifndef BLOCK_H
#define BLOCK_H

#include <stdexcept>
#include <string>
#include <boost/property_tree/ptree_fwd.hpp>
#include "common.h"
#include "log.h"

class Device;


/** @brief XMEGA block
 *
 * A block (or module) provides a given feature and/or handle a peripheral.
 * It handles an range of I/O memory addresses, possibly interruptions, etc.
 */
class Block
{
 public:
  using ConfTree = boost::property_tree::ptree;

  Block(Device& device_, const std::string& name, ioptr_t io_addr, ivnum_t iv_base=0);
  virtual ~Block() = default;

  const std::string& name() const { return name_; }
  ioptr_t io_addr() const { return io_addr_; }
  ivnum_t iv_base() const { return iv_base_; }

  /// Get I/O memory size
  virtual ioptr_t io_size() const = 0;
  /// Get number of interrupt vectors
  virtual ivnum_t iv_count() const { return 0; }

  /// Read an I/O register
  virtual uint8_t getIo(ioptr_t addr) = 0;
  /// Write an I/O register
  virtual void setIo(ioptr_t addr, uint8_t v) = 0;
  /// Called when an IV is executed
  virtual void executeIv(ivnum_t) {};
  /// Reset the block
  virtual void reset() = 0;

  /// Return the block configuration subtree
  const ConfTree& conf() const;

 protected:
  /// Device the block is connected to
  Device& device_;
  /// Logger for this block
  std::shared_ptr<spdlog::logger> logger_;
  /// Set level of a block's IV, relative to IV base
  void setIvLvl(ivnum_t iv, IntLvl lvl) const;

 private:
  /// Full block name
  const std::string name_;
  /// I/O memory base address
  const ioptr_t io_addr_;
  /// Number of the first interrupt vector
  const ivnum_t iv_base_;

};


class BlockError: public std::runtime_error
{
 public:
  BlockError(const Block& block, const std::string& msg):
      std::runtime_error(block.name()+": "+msg) {}
};


namespace block {
  /// Structure to define block instance in subclasses
  template <class T>
  struct Instance {
    const char* name;
    ioptr_t io_addr;
    ivnum_t iv_base;
  };
}


#endif
