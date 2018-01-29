#include <stdexcept>

#include "can_talon_srx/cansocket_impl.h"

namespace can_talon_srx
{

  CanSocketInterface::CanSocketInterface(const char* interface_name)
  {
  }

  CanSocketInterface::~CanSocketInterface()
  {
  }

  void CanSocketInterface::sendMessage(uint32_t messageID, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status)
  {
    throw std::runtime_error("not implemented");
  }

  void CanSocketInterface::receiveMessage(uint32_t *messageID, uint32_t messageIDMask, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp, int32_t *status)
  {
    throw std::runtime_error("not implemented");
  }

  void CanSocketInterface::openStreamSession(uint32_t *sessionHandle, uint32_t messageID, uint32_t messageIDMask, uint32_t maxMessages, int32_t *status)
  {
    throw std::runtime_error("not implemented");
  }

  void CanSocketInterface::closeStreamSession(uint32_t sessionHandle)
  {
    throw std::runtime_error("not implemented");
  }

  void CanSocketInterface::readStreamSession(uint32_t sessionHandle, struct tCANStreamMessage *messages, uint32_t messagesToRead, uint32_t *messagesRead, int32_t *status)
  {
    throw std::runtime_error("not implemented");
  }

  void CanSocketInterface::getCANStatus(float *percentBusUtilization, uint32_t *busOffCount, uint32_t *txFullCount, uint32_t *receiveErrorCount, uint32_t *transmitErrorCount, int32_t *status)
  {
    throw std::runtime_error("not implemented");
  }

}
