#ifndef PLACER_MESSAGE_DEF_H_
#define PLACER_MESSAGE_DEF_H_

namespace pap_common {

enum PROCESS {
  IDLE_STATE = 1,
  CALIBRATION_STATE = 2,
  //GOTOPCBORIGIN_STATE = 3,
  //FINDPADS_STATE = 4,
  GOTOBOX_STATE = 3,
  //FINDCOMPONENT_STATE = 6,
  //GOTOPICKUPCOOR_STATE = 7,
  STARTPICKUP_STATE = 4,
  //GOTOBOTTOMCAM_STATE = 9,
  //CHECKCOMPONENTPICKUP_STATE = 10,
  GOTOPCBCOMP_STATE = 5,
  //CHECKCOMPPOSITION_STATE = 12,
  //GOTOPLACECOORD_STATE = 13,
  //CHECKCOMPONENTPOSITION_STATE = 14,
  STARTPLACEMET_STATE = 6,
  //CHECKCOMPONENTPLACEMENT_STATE = 16,
  HOMING_STATE = 7,
  INFO = 8
  //ERROR_STATE = 18
};

enum PLACER_STATUS {
  PLACER_IDLE = 1,
  PLACER_ACTIVE = 2,
  PLACER_FINISHED = 3,
  PLACER_ERROR = 4
};


} // end namespace

#endif
