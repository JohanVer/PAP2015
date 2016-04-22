#ifndef VISION_SENDFUNCTIONS_H
#define VISION_SENDFUNCTIONS_H


#include <actionlib/client/simple_action_client.h>
#include <pap_common/VisionAction.h>
#include <pap_common/vision_message_def.h>

namespace vision_send_functions{

typedef actionlib::SimpleActionClient<pap_common::VisionAction> Client;

bool sendVisionTask(Client &action_client, enum pap_vision::VISION task);
bool sendVisionTask(Client &action_client, enum pap_vision::VISION task, const double &x, const double &y, const double &z, pap_common::VisionResult &res);
bool sendVisionTask(Client &action_client, enum pap_vision::VISION task, int type, int camera, pap_common::VisionResult &res);

}

#endif // VISION_SENDFUNCTIONS_H
