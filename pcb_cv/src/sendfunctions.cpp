#include <pcb_cv/sendfunctions.h>

namespace vision_send_functions{

// For : START_VISION, STOP_VISION,
bool sendVisionTask(Client &action_client, enum pap_vision::VISION task) {
    action_client.waitForServer();
    pap_common::VisionGoal vis_goal;
    vis_goal.task = task;
    // Fill in goal here
    action_client.sendGoal(vis_goal);
    action_client.waitForResult(ros::Duration(20.0));
    if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        return true;
    }else{
        return false;
    }
}

// FOR: START_CHIP_FINDER , START_TAPE_FINDER, SEARCH_CIRCLE
bool sendVisionTask(Client &action_client, enum pap_vision::VISION task, enum pap_vision::CAMERA_SELECT camera_select, const double &x, const double &y, const double &z, pap_common::VisionResult &res, size_t num_averages) {
    action_client.waitForServer();
    pap_common::VisionGoal vis_goal;
    vis_goal.task = task;
    vis_goal.data1 = x;
    vis_goal.data2 = y;
    vis_goal.data3 = z;
    vis_goal.numAverages = num_averages;
    vis_goal.cameraSelect = (int) camera_select;

    // Fill in goal here
    action_client.sendGoal(vis_goal);
    action_client.waitForResult(ros::Duration(20.0));
    if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        pap_common::VisionResultConstPtr res_ptr = action_client.getResult();
        res = *res_ptr;
        return true;
    }else{
        return false;
    }
}

// FOR: START__QRCODE_FINDER
bool sendVisionTask(Client &action_client, enum pap_vision::VISION task,enum pap_vision::VISION_QR_CALIBRATION  type, enum pap_vision::CAMERA_SELECT camera, pap_common::VisionResult &res, size_t num_averages) {
    action_client.waitForServer();
    pap_common::VisionGoal vis_goal;
    vis_goal.task = task;
    vis_goal.data1 = type;
    vis_goal.cameraSelect = camera;
    vis_goal.numAverages = num_averages;
    // Fill in goal here
    action_client.sendGoal(vis_goal);
    action_client.waitForResult(ros::Duration(20.0));
    if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        pap_common::VisionResultConstPtr res_ptr = action_client.getResult();
        res = *res_ptr;
        return true;
    }else{
        return false;
    }
}

}
