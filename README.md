# can-talon-srx-ros
ROS node for communicating with Talon SRX over CAN

Basically lots of code copied from the wpilib from before the Talon SRX drivers were removed ( https://github.com/wpilibsuite/allwpilib/tree/3fe0f49ac082e7228c948270becb15623d27abf9 ) with a bit of reworking to let the CAN layer be abstracted away using ROS plugins.

TODOS:
- [x] copy protocol stuff from wpilib
- [ ] figure out and document CAN interaction layer
- [ ] specify CAN interface in a ROS plugin
- [ ] write message formats for talking with the ROS CAN node
- [ ] write code for main node
