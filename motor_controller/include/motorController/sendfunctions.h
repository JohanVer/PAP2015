#ifndef MOTOR_SENDFUNCTIONS_H
#define MOTOR_SENDFUNCTIONS_H

#include <actionlib/client/simple_action_client.h>
#include <pap_common/MotorControllerActionAction.h>
#include <pap_common/Task.h>
#include <pap_common/task_message_def.h>

//!
//! These functions can be used to interact with the controller node via ros actions
//!
namespace motor_send_functions{

//!
//! \brief Client defines the ros action client
//!
typedef actionlib::SimpleActionClient<pap_common::MotorControllerActionAction> Client;

//!
//! \brief sendMotorControllerAction sends a task to the motor controller node
//! \param action_client ros action client
//! \param task task to send
//! \param x x-coordinate
//! \param y y-coordinate
//! \param z z-coordinate
//! \return true if successfull
//!
bool sendMotorControllerAction(Client& action_client, pap_common::TASK task, float x, float y, float z);
//!
//! \brief sendMotorControllerAction sends a task to the motor controller node
//! \param action_client ros action client
//! \param task task to send
//! \param x x-coordinate
//! \param y y-coordinate
//! \param z z-coordinate
//! \param velx velocity of x axis
//! \param vely velocity of y axis
//! \return true if successfull
//!
bool sendMotorControllerAction(Client& action_client, pap_common::TASK task,  float x, float y, float z, float velx, float vely);

//!
//! \brief sendMotorControllerAction sends a task to the motor controller node
//! \param action_client ros action client
//! \param task task to send
//! \return true if successfull
//!
bool sendMotorControllerAction(Client& action_client, pap_common::TASK task);

}
#endif // MOTOR_SENDFUNCTIONS_H
