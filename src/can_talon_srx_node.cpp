#include <atomic>
#include <chrono>
#include <thread>

#include <ros/ros.h>

#include "wpilib/CanTalonSRX.h"

#include "can_talon_srx/can_base.h"
#include "can_talon_srx/cansocket_impl.h"

boost::shared_ptr<can_talon_srx::CanInterface> can_interface;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "can_talon_srx_node");
  auto nh = ros::NodeHandle();

  ROS_INFO("setting up CAN interface...");
  can_interface = boost::shared_ptr<can_talon_srx::CanInterface>(
      new can_talon_srx::CanSocketInterface("can0"));
  ROS_INFO("CAN interface setup succesful!");

  CanTalonSRX testTalon1(1);
  CanTalonSRX testTalon2(2);
  CanTalonSRX testTalon3(3);
  CanTalonSRX testTalon4(4);
  CanTalonSRX testTalon5(5);
  CanTalonSRX testTalon6(6);
  CanTalonSRX testTalon7(7);
  CanTalonSRX testTalon8(8);

  std::atomic<bool> running(true);
  auto thr = std::thread([&]() {
    testTalon1.SetModeSelect(CanTalonSRX::kMode_DutyCycle, 100);
    testTalon2.SetModeSelect(CanTalonSRX::kMode_DutyCycle, 100);
    testTalon3.SetModeSelect(CanTalonSRX::kMode_DutyCycle, 100);
    testTalon4.SetModeSelect(CanTalonSRX::kMode_DutyCycle, 100);
    testTalon5.SetModeSelect(CanTalonSRX::kMode_DutyCycle, 100);
    testTalon6.SetModeSelect(CanTalonSRX::kMode_DutyCycle, 100);
    testTalon7.SetModeSelect(CanTalonSRX::kMode_DutyCycle, 100);
    testTalon8.SetModeSelect(CanTalonSRX::kMode_DutyCycle, 100);
    int count = 0;
    while (running)
    {
      if ((count % 200) == 100)
      {
        testTalon1.SetDemand(100);
        testTalon2.SetDemand(100);
        testTalon3.SetDemand(100);
        testTalon4.SetDemand(100);
        testTalon5.SetDemand(100);
        testTalon6.SetDemand(100);
        testTalon7.SetDemand(100);
        testTalon8.SetDemand(100);
      }
      else if ((count % 200) == 199)
      {
        testTalon1.SetDemand(-100);
        testTalon2.SetDemand(-100);
        testTalon3.SetDemand(-100);
        testTalon4.SetDemand(-100);
        testTalon5.SetDemand(-100);
        testTalon6.SetDemand(-100);
        testTalon7.SetDemand(-100);
        testTalon8.SetDemand(-100);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      ++count;
    }
  });

  ros::Rate update_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    update_rate.sleep();
  }
  running = false;
  thr.join();

  return 0;
}

extern "C"
{

void FRC_NetworkCommunication_CANSessionMux_sendMessage(uint32_t messageID, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status)
{
  if (can_interface)
  {
    can_interface.get()->sendMessage(messageID, data, dataSize, periodMs, status);
  }
  else
  {
    ROS_ERROR("CAN interface was unitialized!");
  }
}

void FRC_NetworkCommunication_CANSessionMux_receiveMessage(uint32_t *messageID, uint32_t messageIDMask, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp, int32_t *status)
{
  if (can_interface)
  {
    can_interface.get()->receiveMessage(messageID, messageIDMask, data, dataSize, timeStamp, status);
  }
  else
  {
    ROS_ERROR("CAN interface was unitialized!");
  }
}

void FRC_NetworkCommunication_CANSessionMux_openStreamSession(uint32_t *sessionHandle, uint32_t messageID, uint32_t messageIDMask, uint32_t maxMessages, int32_t *status)
{
  if (can_interface)
  {
    can_interface.get()->openStreamSession(sessionHandle, messageID, messageIDMask, maxMessages, status);
  }
  else
  {
    ROS_ERROR("CAN interface was unitialized!");
  }
}

void FRC_NetworkCommunication_CANSessionMux_closeStreamSession(uint32_t sessionHandle)
{
  if (can_interface)
  {
    can_interface.get()->closeStreamSession(sessionHandle);
  } 
  else
  {
    ROS_ERROR("CAN interface was unitialized!");
  }
}

void FRC_NetworkCommunication_CANSessionMux_readStreamSession(uint32_t sessionHandle, struct tCANStreamMessage *messages, uint32_t messagesToRead, uint32_t *messagesRead, int32_t *status)
{
  if (can_interface)
  {
    can_interface.get()->readStreamSession(sessionHandle, messages, messagesToRead, messagesRead, status);
  }
  else
  {
    ROS_ERROR("CAN interface was unitialized!");
  }
}

void FRC_NetworkCommunication_CANSessionMux_getCANStatus(float *percentBusUtilization, uint32_t *busOffCount, uint32_t *txFullCount, uint32_t *receiveErrorCount, uint32_t *transmitErrorCount, int32_t *status)
{
  if (can_interface)
  {
    can_interface.get()->getCANStatus(percentBusUtilization, busOffCount, txFullCount, receiveErrorCount, transmitErrorCount, status);
  }
  else
  {
    ROS_ERROR("CAN interface was unitialized!");
  }
}

}
