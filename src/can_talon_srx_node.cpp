#include <atomic>
#include <chrono>
#include <thread>

#include <ros/ros.h>

#include "wpilib/CanTalonSRX.h"

#include "can_talon_srx/can_base.h"
#include "can_talon_srx/cansocket.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "can_talon_srx_node");
  auto nh = ros::NodeHandle();

  ROS_INFO("setting up CAN interface...");
  can_talon_srx::CanSocketInterface::Init("can0");
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


