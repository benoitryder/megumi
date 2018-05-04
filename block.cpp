#include <boost/property_tree/ptree.hpp>
#include "device.h"
#include "block.h"


Block::Block(Device& dev, const std::string& name, ioptr_t io_addr, ivnum_t iv_base):
    device_(dev), logger_(Log::getLogger(name)), name_(name), io_addr_(io_addr), iv_base_(iv_base)
{
  if(iv_base_ == 0 && iv_count() != 0) {
    throw BlockError(*this, "invalid interrupt vector number: 0");
  }
}

Block::~Block()
{
}


void Block::setIvLvl(ivnum_t iv, IntLvl lvl) const
{
  device_.setIvLvl(iv+iv_base(), lvl);
}


const Block::ConfTree& Block::conf() const
{
  return device_.conf(name_);
}

