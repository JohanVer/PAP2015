#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "../../motor_controller/include/motorController/controllerClass.hpp"

bool energized = false;
controllerStatus controllerState1, controllerState2, controllerState3;
static tf::TransformBroadcaster br;
ros::Publisher statusPublisher;

void checkStatusController(int numberOfController,
		controllerStatus* controllerStatusAct) {

	pap_common::Status stateMessage;
	stateMessage.data1 = numberOfController;

	if (controllerStatusAct->energized) {
		stateMessage.status = pap_common::ENERGIZED;
	} else {
		stateMessage.status = pap_common::NOENERGY;
	}
	statusPublisher.publish(stateMessage);

	if (controllerStatusAct->error) {
		stateMessage.status = pap_common::ERROR;
	} else {
		stateMessage.status = pap_common::NOERROR;
	}
	statusPublisher.publish(stateMessage);

	if (controllerStatusAct->positionReached) {
		stateMessage.status = pap_common::POSITIONREACHED;
	} else {
		stateMessage.status = pap_common::POSITIONNOTREACHED;
	}
	statusPublisher.publish(stateMessage);
}

void parseTask(const pap_common::TaskConstPtr& taskMsg) {
	switch (taskMsg->destination) {
	case pap_common::CONTROLLER:
		switch (taskMsg->task) {
		case pap_common::HOMING:
			//if (!controller.sendHoming()) {
			//	ROS_ERROR("Error while sending homing command");
			//}
			break;
		case pap_common::CURRENT:
			if (!energized) {
				controllerState1.energized = true;
				controllerState2.energized = true;
				controllerState3.energized = true;
			} else {
				controllerState1.energized = false;
				controllerState2.energized = false;
				controllerState3.energized = false;
			}
			break;
		case pap_common::COORD:
			//coordError = controller.gotoCoord(taskMsg->data1, taskMsg->data2,
			//		taskMsg->data3);
			break;

		case pap_common::MANUAL:
			if (taskMsg->data1 == (float) (pap_common::XMOTOR)) {
				if (taskMsg->data2 == (float) (pap_common::FORWARD)) {
					//controller.manual(1, 1);
				} else {
					//controller.manual(1, 0);
				}
			} else if (taskMsg->data1 == (float) (pap_common::YMOTOR)) {
				if (taskMsg->data2 == (float) (pap_common::FORWARD)) {
					//controller.manual(2, 1);
				} else {
					//controller.manual(2, 0);
				}
			} else if (taskMsg->data1 == (float) (pap_common::ZMOTOR)) {
				if (taskMsg->data2 == (float) (pap_common::FORWARD)) {
					//controller.manual(3, 1);
				} else {
					//controller.manual(3, 0);
				}
			}
			break;

		case pap_common::CONNECT:
			ROS_INFO("Searching for devices...");
			ROS_INFO("Controller 1 connected...");
			ROS_INFO("Controller 2 connected...");
			ROS_INFO("Controller 3 connected...");
			;
			controllerState1.positionReached = false;
			controllerState2.positionReached = false;
			controllerState3.positionReached = false;
			controllerState1.error = false;
			controllerState2.error = false;
			controllerState3.error = false;
			break;

		case pap_common::STOP:
			if (taskMsg->data1 == (float) (pap_common::XMOTOR)) {
				//controller.stop(1);
			} else if (taskMsg->data1 == (float) (pap_common::YMOTOR)) {
				//controller.stop(2);
			} else if (taskMsg->data1 == (float) (pap_common::ZMOTOR)) {
				//controller.stop(3);
			}
			break;
		}
		break;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "tf_sender");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	ros::Subscriber taskSubscriber_ = n.subscribe("task", 1, &parseTask);

	while (ros::ok()) {

		// Put in simulation here! (Current update rate 100Hz)
		// The commands are processed in the function "parseTask".There the simulator must be sensitive to.

		// When a simulation iteration is calculated you can send the positions of each link
		// with the function "sendTransforms" to the simulated robot.

		// Keep in mind that everytime the status of one link changes you have to make sure that it is changed
		// in the controllerState1,controllerState2,controllerState3 too.
		// To send the status the function "checkStatusController" must be called in the loop with the right number of joint
		// and status-struct.

		// When a new coordinate should be applied the status of the corresponding link should change to reachedPosition=false,
		// when the goal is reached it's the other way around.

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void sendTransforms(double x_des, double y_des, double z_des, double nozzle_1,
		double nozzle_2) {
	tf::Transform transform;
	tf::Transform transformReference;
	tf::Transform transformX;
	tf::Transform transformY;
	float x = 0.0;
	float y = 0.0;

	// Reference frame
	transformReference.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	tf::Quaternion qRef;
	qRef.setRPY(0, 0, 0);
	transformReference.setRotation(qRef);

	// Base_link frame
	transform.setOrigin(tf::Vector3(0, 0, 0));
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);

	// X-Link
	x = x + 0.001;
	if (x >= 0.3)
		x = 0.0;

	y = y + 0.001;
	if (y >= 0.09)
		y = 0.0;

	transformX.setOrigin(tf::Vector3(0.0059907 + x, 0.052959, 0.11591));
	tf::Quaternion qX;
	qX.setX(0.707108);
	qX.setY(0);
	qX.setZ(0);
	qX.setW(0.707105);
	transformX.setRotation(qX);

	// Y-Link
	transformY.setOrigin(tf::Vector3(0.059991 + x, 0.22188 + y, 0.15122));
	tf::Quaternion qY;
	qY.setX(0.707107);
	qY.setY(0.707107);
	qY.setZ(1.29867e-06);
	qY.setW(-1.29867e-06);
	transformY.setRotation(qY);

	br.sendTransform(
			tf::StampedTransform(transformReference, ros::Time::now(),
					"/base_link", "/world"));
	//br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/world","/base_link"));

	// Axes
	br.sendTransform(
			tf::StampedTransform(transformX, ros::Time::now(), "/base_link",
					"/XLink"));
	br.sendTransform(
			tf::StampedTransform(transformY, ros::Time::now(), "/base_link",
					"/YLink"));
}
