# can-talon-srx-ros
ROS C++ library for communicating with Talon SRX over CAN

This repository contains code originating from First Robotics; the code there is within the `wpilib` folders in both `include/` and `src/`.
The rest of the repository is under an MIT license.
More appropriate licensing will be done some time eventually probably

Basically lots of code copied from the `wpilibc` from before the Talon SRX drivers were removed ( https://github.com/wpilibsuite/allwpilib/tree/3fe0f49ac082e7228c948270becb15623d27abf9 ) with a bit of reworking to let the CAN layer be abstracted away somewhat.
This project contains some example code in `src/can_talon_srx_node.cpp` to demonstrate how to use this library.

TODOS:
- [x] copy protocol stuff from wpilib
- [ ] figure out and document CAN interaction layer
