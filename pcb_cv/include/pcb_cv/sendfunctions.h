#ifndef VISION_SENDFUNCTIONS_H
#define VISION_SENDFUNCTIONS_H


#include <actionlib/client/simple_action_client.h>
#include <pap_common/VisionAction.h>
#include <pap_common/vision_message_def.h>

//!
//! These functions can be used to send action requests to the pcb_cv node
//!
namespace vision_send_functions{

//!
//! \brief Ros Action client
//!
typedef actionlib::SimpleActionClient<pap_common::VisionAction> Client;

bool sendVisionTask(Client &action_client, enum pap_vision::VISION task);
bool sendVisionTask(Client &action_client, enum pap_vision::VISION task, pap_vision::CAMERA_SELECT camera_select, const double &x, const double &y, const double &z, pap_common::VisionResult &res, size_t num_averages = 200);
bool sendVisionTask(Client &action_client, enum pap_vision::VISION task,enum pap_vision::VISION_QR_CALIBRATION  type, enum pap_vision::CAMERA_SELECT camera, pap_common::VisionResult &res, size_t num_averages = 200);
}

#endif // VISION_SENDFUNCTIONS_H
