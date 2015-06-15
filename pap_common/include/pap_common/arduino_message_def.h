#ifndef ARDUINO_MESSAGE_DEF_H_
#define ARDUINO_MESSAGE_DEF_H_

namespace pap_common {

enum ARDUINO_TASK {
  SETRELAIS = 1,
  RESETRELAIS = 2,
  RUNSTEPPER1 = 3,
  RUNSTEPPER2 = 4,
  RESETSTEPPERS = 5
};

enum RELAIS {
  RELAIS1 = 1,
  RELAIS2 = 2,
  RELAIS3 = 3,
  RELAIS4 = 4,
  RELAIS5 = 5,
  RELAIS6 = 6,
  RELAIS7 = 7,
  RELAIS8 = 8,
  RELAIS9 = 9,
  RELAIS = 10
};

} // end namespace 'common'

#endif

