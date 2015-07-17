#ifndef VISION_MESSAGE_DEF_H_
#define VISION_MESSAGE_DEF_H_

namespace pap_vision {

enum VISION {
  START_VISION	 	= 1,
  STOP_VISION	 	= 2,
  START_CHIP_FINDER 	= 3,
  START_SMALL_FINDER    = 4,
  START_TAPE_FINDER     = 5,
  START_PAD_FINDER      = 6,
  SELECT_PAD			= 7,
  SEARCH_CIRCLE			= 8,
  CHIP_BOTTOM			= 9,
  START__QRCODE_FINDER	= 10
};


} // end namespace 'common'

#endif

