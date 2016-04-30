#ifndef VISION_SENDFUNCTIONS_H
#define VISION_SENDFUNCTIONS_H


#include <actionlib/client/simple_action_client.h>
#include <pap_common/VisionAction.h>
#include <pap_common/vision_message_def.h>

namespace vision_send_functions{

typedef actionlib::SimpleActionClient<pap_common::VisionAction> Client;

bool sendVisionTask(Client &action_client, enum pap_vision::VISION task);
bool sendVisionTask(Client &action_client, enum pap_vision::VISION task, pap_vision::CAMERA_SELECT camera_select, const double &x, const double &y, const double &z, pap_common::VisionResult &res, size_t num_averages = 1);
bool sendVisionTask(Client &action_client, enum pap_vision::VISION task,enum pap_vision::VISION_QR_CALIBRATION  type, enum pap_vision::CAMERA_SELECT camera, pap_common::VisionResult &res, size_t num_averages = 1);
}

#endif // VISION_SENDFUNCTIONS_H
