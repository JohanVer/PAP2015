#ifndef SENDFUNCTIONS_H
#define SENDFUNCTIONS_H

#include <actionlib/client/simple_action_client.h>
#include <pap_common/MotorControllerActionAction.h>
#include <pap_common/Task.h>
#include <pap_common/task_message_def.h>


namespace motor_send_functions{

typedef actionlib::SimpleActionClient<pap_common::MotorControllerActionAction> Client;

bool sendMotorControllerAction(Client& action_client, pap_common::TASK task, float x, float y, float z);
bool sendMotorControllerAction(Client& action_client, pap_common::TASK task,  float x, float y, float z, float velx, float vely);
bool sendMotorControllerAction(Client& action_client, pap_common::TASK task);

}
#endif // SENDFUNCTIONS_H
