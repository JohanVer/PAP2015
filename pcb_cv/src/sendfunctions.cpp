#include <pcb_cv/sendfunctions.h>

namespace vision_send_functions{

// For : Start, Stop,
bool sendVisionTask(Client &action_client, enum pap_vision::VISION task) {
    action_client.waitForServer();
    pap_common::VisionGoal vis_goal;
    vis_goal.task = task;
    // Fill in goal here
    action_client.sendGoal(vis_goal);
    action_client.waitForResult(ros::Duration(12.0));
    if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        return true;
    }else{
        return false;
    }
}

bool sendVisionTask(Client &action_client, enum pap_vision::VISION task, const double &x, const double &y, const double &z, pap_common::VisionResult &res) {
    action_client.waitForServer();
    pap_common::VisionGoal vis_goal;
    vis_goal.task = task;
    vis_goal.data1 = x;
    vis_goal.data2 = y;
    vis_goal.data3 = z;

    // Fill in goal here
    action_client.sendGoal(vis_goal);
    action_client.waitForResult(ros::Duration(12.0));
    if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        pap_common::VisionResultConstPtr res_ptr = action_client.getResult();
        res = *res_ptr;
        return true;
    }else{
        return false;
    }
}

bool sendVisionTask(Client &action_client, enum pap_vision::VISION task, int type, int camera, pap_common::VisionResult &res) {
    action_client.waitForServer();
    pap_common::VisionGoal vis_goal;
    vis_goal.task = task;
    vis_goal.data1 = type;
    vis_goal.data2 = camera;
    // Fill in goal here
    action_client.sendGoal(vis_goal);
    action_client.waitForResult(ros::Duration(12.0));
    if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        pap_common::VisionResultConstPtr res_ptr = action_client.getResult();
        res = *res_ptr;
        return true;
    }else{
        return false;
    }
}

}
