#include <cstring>
#include <stdexcept>

#include <linux/can.h>
#include <linux/can/bcm.h>

#include <net/if.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <unistd.h>

#include "can_talon_srx/cansocket_impl.h"

namespace can_talon_srx
{

  CanSocketInterface::CanSocketInterface(const char* interface_name)
  {
    struct sockaddr_can addr;
    struct ifreq ifr;

    socket_ = socket(PF_CAN, SOCK_RAW, CAN_BCM);
    if (socket < 0)
    {
      // TODO: error message
    }

    strcpy(ifr.ifr_name, interface_name);
    ioctl(socket_, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (connect(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      // TODO: error message
    }

    // TODO: start up a thread to read messages
  }

  CanSocketInterface::~CanSocketInterface()
  {
    close(socket_);
  }

  void CanSocketInterface::sendMessage(uint32_t arbID, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status)
  {
    struct single_can_msg {
      struct bcm_msg_head msg_head;
      struct can_frame frames[1];
    } can_msg;

    if (periodMs == 0)
    {
      struct no_can_msgs {
        struct bcm_msg_head msg_head;
        struct can_frame frames[0];
      } rm_msg;

      // see if a message with this arbID has already been sent;
      // if it has, cancel that message
      if (sendingIds.find(arbID) != sendingIds.end())
      {
        rm_msg.msg_head.opcode = TX_DELETE;
        // rm_msg.msg_head.flags = 0;
        // rm_msg.msg_head.count = 0;
        // rm_msg.msg_head.ival1 = 0;
        // rm_msg.msg_head.ival2 = 0;
        rm_msg.msg_head.can_id = arbID;
        // rm_msg.msg_head.nframes = 0;

        int nbytes = write(socket_, &rm_msg, sizeof(rm_msg));
        if (nbytes < 0)
        {
          // TODO: write warning message
        }
        else if (nbytes < sizeof(rm_msg))
        {
        }
        else
        {
          sendingIds.erase(arbID);
        }
      }
      // then send this message as a single shot
      can_msg.msg_head.opcode = TX_SEND;
      can_msg.msg_head.flags = 0;
      // can_msg.msg_head.count = 0;
      // can_msg.msg_head.ival1 = 0;
      // can_msg.msg_head.ival2 = 0;
      can_msg.msg_head.can_id = arbID;
      can_msg.msg_head.nframes = 1;

      can_msg.frames[0].can_id = arbID;
      can_msg.frames[0].can_dlc = dataSize;
      if (data != nullptr)
      {
        memcpy(can_msg.frames[0].data, data, dataSize);
      }
      else
      {
        memset(can_msg.frames[0].data, 0, dataSize);
      }

      int nbytes = write(socket_, &can_msg, sizeof(can_msg));
      if (nbytes < 0)
      {
        // TODO: write warning message
      }
      else if (nbytes < sizeof(can_msg))
      {
        // TODO: write error message
      }
    }
    else
    {
      // write the request to the kernel and add the arbitration ID to the sending set

      can_msg.msg_head.opcode = TX_SETUP;
      can_msg.msg_head.flags = SETTIMER | STARTTIMER;
      can_msg.msg_head.count = 0;
      // can_msg.msg_head.ival1 = 0;
      can_msg.msg_head.ival2.tv_sec = periodMs / 1000;
      can_msg.msg_head.ival2.tv_usec = 1000 * (periodMs % 1000);
      can_msg.msg_head.can_id = arbID;
      can_msg.msg_head.nframes = 1;

      can_msg.frames[0].can_id = arbID;
      can_msg.frames[0].can_dlc = dataSize;
      if (data != nullptr)
      {
        memcpy(can_msg.frames[0].data, data, dataSize);
      }
      else
      {
        memset(can_msg.frames[0].data, 0, dataSize);
      }

      int nbytes = write(socket_, &can_msg, sizeof(can_msg));
      if (nbytes < 0)
      {
        // TODO: write warning message
      }
      else if (nbytes < sizeof(can_msg))
      {
        // TODO: write error message
      }

      sendingIds.insert(arbID);
    }
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
