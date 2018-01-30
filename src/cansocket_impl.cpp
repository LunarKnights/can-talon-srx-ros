#include <cstring>
#include <stdexcept>

#include <linux/can.h>
#include <linux/can/bcm.h>

#include <net/if.h>

#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <unistd.h>

#include "ros/ros.h"

#include "can_talon_srx/cansocket_impl.h"

namespace can_talon_srx
{

  CanSocketInterface::CanSocketInterface(const char* interface_name): running(true)
  {
    struct sockaddr_can addr;
    struct ifreq ifr;

    socket_ = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
    if (socket < 0)
    {
      ROS_FATAL("unable to create socket!");
    }

    strncpy(ifr.ifr_name, interface_name, IFNAMSIZ);
    ioctl(socket_, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (connect(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      ROS_FATAL("unable to connect socket!");
    }

    auto messageBox = std::make_shared<MessageBox>();
    receivedMessages_ = messageBox;

    // start up a thread to read messages
    readThread = std::thread([&]()
    {
        // copy the socket file descriptor so we can make it nonblocking
        // this prevents the read() call from ever blocking, making sure
        // this thread doesn't ever get blocked
        int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket < 0)
        {
          ROS_FATAL("unable to create socket!");
        }

        strncpy(ifr.ifr_name, interface_name, IFNAMSIZ);
        ioctl(s, SIOCGIFINDEX, &ifr);

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (connect(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
          ROS_FATAL("unable to connect socket!");
        }

        int flags = fcntl(s, F_GETFL, 0);
        if (flags == -1)
        {
          ROS_FATAL("unable to get socket flags!");
        }
        int retval = fcntl(s, F_SETFL, flags | O_NONBLOCK);
        if (retval == -1)
        {
          ROS_FATAL("unable to set socket flags!");
        }

        auto messages = messageBox;

        fd_set readfds;
        struct timeval timeout;
        while(running)
        {
          // set up a fd set
          FD_ZERO(&readfds);
          FD_SET(s, &readfds);

          // 100 ms timeout
          timeout.tv_sec = 0;
          timeout.tv_usec = 100000;
          int retval = select(1, &readfds, 0, 0, &timeout);
          if (retval == -1)
          {
            // TODO: report error
          }
          else
          {
            // read from s to get can frames and work with them accordingly
            struct can_frame frame;
            int nbytes = read(s, &frame, sizeof(frame));
            if (nbytes < 0)
            {
              ROS_WARN("read fail %d", errno);
            }
            else if (nbytes < sizeof(frame))
            {
              ROS_WARN("incomplete read %d of %ld", nbytes, sizeof(frame));
            }
            else
            {
              std::lock_guard<std::mutex> guard(messages->lock);
              // flip ID length bit
              uint32_t arbID = frame.can_id ^ (1 << 31);
              auto &ptr = messages->messages[arbID];
              auto new_frame = Message
              {
                .arbID=arbID,
                .data={0},
                .len=frame.can_dlc,
              };
              memcpy(&new_frame.data, frame.data, 8);
              ptr = std::make_shared<Message>(new_frame);
            }
          }
        }

        close(s);
    });
  }

  CanSocketInterface::~CanSocketInterface()
  {
    running = false;
    readThread.join();
    close(socket_);
  }

  void CanSocketInterface::sendMessage(uint32_t arbID, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status)
  {
    struct single_can_msg {
      struct bcm_msg_head msg_head;
      struct can_frame frames[1];
    } can_msg;

    // flip the MSB because it means the opposite in Linux CAN IDs than in 
    // CTRE CAN IDs
    arbID ^= 1 << 31;

    if (periodMs == 0)
    {
      struct no_can_msgs {
        struct bcm_msg_head msg_head;
        struct can_frame frames[0];
      } rm_msg;

      // see if a message with this arbID has already been sent;
      // if it has, cancel that message
      if (sendingIds_.find(arbID) != sendingIds_.end())
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
          ROS_ERROR("unable to send CAN message: %d", errno);
        }
        else if (nbytes < sizeof(rm_msg))
        {
          ROS_ERROR("unable to send complete CAN message: sent %d of %ld", nbytes, sizeof(rm_msg));
        }
        else
        {
          sendingIds_.erase(arbID);
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
        ROS_ERROR("unable to send CAN message: %d", errno);
      }
      else if (nbytes < sizeof(can_msg))
      {
        ROS_ERROR("unable to send complete CAN message: sent %d of %ld", nbytes, sizeof(can_msg));
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
        ROS_ERROR("unable to send CAN message: %d", errno);
      }
      else if (nbytes < sizeof(can_msg))
      {
        ROS_ERROR("unable to send complete CAN message: sent %d of %ld", nbytes, sizeof(can_msg));
      }

      sendingIds_.insert(arbID);
    }
  }

  void CanSocketInterface::receiveMessage(uint32_t *messageID, uint32_t messageIDMask, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp, int32_t *status)
  {
    // NOTE: not entirely sure what messageIDMask does; it's usually set to something
    // like 0x1FFFFFFF or 0xFFFFFFFF so I'm guessing it doesn't matter that much
    // Also not sure what timeStamp is for; it's not used so I'm gonna ignore it for now

    // Also need to flip the MSB because the meaning's inverted on Linux instead of CTRE
    // Having bit 31 set means the ID is 11-bit in CTRE, but having it clear means
    // the identifier is 11-bit in Linux SocketCAN
    // So we flip it
    const uint32_t arbID = (*messageID ^ (1 << 31)) & messageIDMask;
    // check the message box to see if a message has been received
    std::lock_guard<std::mutex> guard(receivedMessages_->lock);
    auto& entry = receivedMessages_->messages[arbID];
    if (entry)
    {
      *status = 0;
      // swap the entry out
      std::shared_ptr<Message> msg;
      entry.swap(msg);

      // copy the data out
      memcpy(data, msg->data, msg->len);
      *dataSize = msg->len;
    }
    else
    {
      // no entry; set status accordingly
      *status = 1;
    }
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
