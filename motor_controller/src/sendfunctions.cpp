#include <motorController/sendfunctions.h>

namespace motor_send_functions{

bool sendMotorControllerAction(Client& action_client, pap_common::TASK task, float x, float y, float z){
    action_client.waitForServer();
    pap_common::MotorControllerActionGoal motor_goal;
    motor_goal.task = task;
    motor_goal.data1 = x;
    motor_goal.data2 = y;
    motor_goal.data3 = z;
    // Fill in goal here
    action_client.sendGoal(motor_goal);
    action_client.waitForResult(ros::Duration(12.0));
    if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        return true;
    }else{
        return false;
    }
}

bool sendMotorControllerAction(Client& action_client, pap_common::TASK task,  float x, float y, float z, float velx, float vely){
    action_client.waitForServer();
    pap_common::MotorControllerActionGoal motor_goal;
    motor_goal.task = task;
    motor_goal.data1 = x;
    motor_goal.data2 = y;
    motor_goal.data3 = z;
    motor_goal.velX = velx;
    motor_goal.velY = vely;

    // Fill in goal here
    action_client.sendGoal(motor_goal);
    action_client.waitForResult(ros::Duration(12.0));
    if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        return true;
    }else{
        return false;
    }
}

bool sendMotorControllerAction(Client& action_client, pap_common::TASK task,  float x, float y, float z, float velx, float vely, float velz, pap_common::MotorControllerActionResult &res){
    action_client.waitForServer();
    pap_common::MotorControllerActionGoal motor_goal;
    motor_goal.task = task;
    motor_goal.data1 = x;
    motor_goal.data2 = y;
    motor_goal.data3 = z;
    motor_goal.velX = velx;
    motor_goal.velY = vely;
    motor_goal.velZ = velz;

    // Fill in goal here
    action_client.sendGoal(motor_goal);
    action_client.waitForResult(ros::Duration(12.0));
    if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        pap_common::MotorControllerActionResultConstPtr res_ptr = action_client.getResult();
        res = *res_ptr;
        return true;
    }else{
        return false;
    }
}

bool sendMotorControllerAction(Client& action_client, pap_common::TASK task){
    action_client.waitForServer();
    pap_common::MotorControllerActionGoal motor_goal;
    motor_goal.task = task;
    // Fill in goal here
    action_client.sendGoal(motor_goal);
    action_client.waitForResult(ros::Duration(12.0));
    if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        return true;
    }else{
        std::cerr << "Action was not successfull, error or timeout...\n";
        return false;
    }
}

}
