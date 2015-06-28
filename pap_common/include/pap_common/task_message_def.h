#ifndef TASK_MESSAGE_DEF_H_
#define TASK_MESSAGE_DEF_H_

namespace pap_common {

enum TASK {
  HOMING = 1,
  CURRENT = 2,
  // COORD : set x,y,z coordinates of target point as data(1,2,3)
  COORD =3,
  // MANUAL : Set DIRECTION in data 1 and MOTOR in data 2
  MANUAL = 4,
  CONNECT = 5,
  STOP = 6
};

enum DESTINATION {
  CONTROLLER = 1,
  VISION = 2
};

enum DIRECTION {
  FORWARD = 1,
  BACKWARD = 2
};

enum MOTOR {
  XMOTOR = 1,
  YMOTOR = 2,
  ZMOTOR = 3
};

} // end namespace 'common'

#endif

