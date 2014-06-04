#ifndef MODEL_XMEGA128A1_H__
#define MODEL_XMEGA128A1_H__

#include "../device.h"
#include "../block/port.h"
#include "../block/usart.h"
#include "../block/tc.h"

namespace model {

class ATxmega128A1: public Device
{
  static const ModelConf MODEL_CONF;

 public:
  ATxmega128A1(ConfTree& conf);
  virtual ~ATxmega128A1();

  virtual const char* model_name() const { return "ATxmega128A1"; }

  // blocks
  block::Port portA_;
  block::Port portB_;
  block::Port portC_;
  block::Port portD_;
  block::Port portE_;
  block::Port portF_;
  block::Port portH_;
  block::Port portJ_;
  block::Port portK_;
  block::Port portQ_;
  block::Port portR_;
  block::USART usartC0_;
  block::USART usartC1_;
  block::USART usartD0_;
  block::USART usartD1_;
  block::USART usartE0_;
  block::USART usartE1_;
  block::USART usartF0_;
  block::USART usartF1_;
  block::TC tcC0_;
  block::TC tcC1_;
  block::TC tcD0_;
  block::TC tcD1_;
  block::TC tcE0_;
  block::TC tcE1_;
  block::TC tcF0_;
  block::TC tcF1_;
};

}

#endif
