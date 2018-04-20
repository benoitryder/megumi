#ifndef GDBSERVER_H
#define GDBSERVER_H

#include <stdexcept>
#include <set>
#include "common.h"

class Device;


/// Server implementing GDB remote protocol
class GdbServer
{
 public:
  GdbServer(Device& dev);
  ~GdbServer();

  /** @brief Run the GDB server
   *
   * @note The device should be initialized (reset at least once) before
   * running the server.
   */
  void run(int port);

  /// Continue execution
  void execContinue();
  /// Execute a single step
  void execStep();

 private:
  /// Process a client packet data
  void processPacket(const std::string& data);

  /// Receive a packet, return its data
  std::string recvPacket();
  /// Send a packet (with ACK)
  void sendPacket(const std::string& data);
  /// Send a ACK or a NACK
  void sendAck(bool ack);

  /// Get memory data using GDB addressing
  uint8_t getGdbMem(unsigned int addr);
  /// Get memory data using GDB addressing
  void setGdbMem(unsigned int addr, uint8_t v);

  /// Build a 'T' stop reply
  std::string buildStopReply() const;

  Device& device_;
  int sock_client_;
  std::string rbuf_; ///< Reception buffer

  /// Breakpoints, as a set of PC values
  std::multiset<flashptr_t> breakpoints_;
};


class GdbServerError: public std::runtime_error
{
 public:
  GdbServerError(const std::string& msg):
      std::runtime_error(msg) {}
  GdbServerError(int errnum, const std::string& msg);
};


#endif
