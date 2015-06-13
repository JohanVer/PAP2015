#ifndef STATUS_MESSAGE_DEF_H_
#define STATUS_MESSAGE_DEF_H_

namespace pap_common {

enum STATUS {
  STATUSOK = 1,
  ERROR = 2,
  NOERROR = 3,
  ENERGIZED = 4,
  NOENERGY = 5,
  POSITIONREACHED = 6,
  POSITIONNOTREACHED = 7,
  DISCONNECTED = 8,
  CONNECTED = 9,
  LAST_CMD_FAILED = 10
};


} // end namespace 'common'

#endif

