#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "math.h"
#include "../../motor_controller/include/motorController/controllerClass.hpp"

/*****************************************************************************
 * Parameter
 *****************************************************************************/
double maxVelocity = 120;		// mm/s
double accX = 300;				// mm/s²
double accXDelay = 300;			// mm/s²
double epsilonDistance = 0.5;	// mm
double ts = 0.01;				// s

double xHome = 0.0;
double yHome = 0.0;
double zHome = 0.0;

bool energized = false;
bool controller1StatusSet = false;

double distXTotal, distYTotal, distZTotal = 0;
double distX, distY, distZ = 0;
double desX, desY, desZ = 0;

void sendTransforms(double x_des, double y_des, double z_des, double nozzle_1,
		double nozzle_2);

struct state {
	double x, y, z;
	double vx, vy, vz;
} currentState;

double matrix[6][6] = { { ts, 0.0, 0.0, 0.5 * pow(ts, 2.0), 0.0, 0.0 }, { 0.0,
		ts, 0.0, 0.0, 0.5 * pow(ts, 2.0), 0.0 }, { 0.0, 0.0, ts, 0.0, 0.0, 0.5
		* pow(ts, 2.0) }, { 1.0, 0.0, 0.0, ts, 0.0, 0.0 }, { 0.0, 1.0, 0.0, 0.0,
		ts, 0.0 }, { 0.0, 0.0, 1.0, 0.0, 0.0, ts } };

controllerStatus controllerState1, controllerState2, controllerState3;

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

			desX = taskMsg->data1;					// Desired position
			distXTotal = desX - currentState.x;	// Distance we still have to go

			desY = taskMsg->data2;					// Desired position
			distYTotal = desY - currentState.y;	// Distance we still have to go

			desZ = taskMsg->data3;					// Desired position
			distZTotal = desZ - currentState.z;	// Distance we still have to go
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

double vectors_dot_prod(const double *x, const double *y, int n) {
	double res = 0.0;
	int i;
	for (i = 0; i < n; i++) {
		res += x[i] * y[i];
	}
	return res;
}

void matrix_vector_mult(const double **mat, const double *vec, double *result,
		int rows, int cols) { // in matrix form: result = mat * vec;
	int i;
	for (i = 0; i < rows; i++) {
		result[i] = vectors_dot_prod(mat[i], vec, cols);
	}
}

void simulate_next_step(double accX, double accY, double accZ, double ts) {

	const double result[6] = { 0, 0, 0, 0, 0, 0 };
	const double input[6] = { currentState.vx, currentState.vy, currentState.vz,
			accX, accY, accZ };
	//matrix_vector_mult(matrix[][], input, result, 6, 6);

	// Update current state
	currentState.x = result[0] + currentState.x;
	currentState.y = result[1] + currentState.y;
	currentState.z = result[2] + currentState.z;
	currentState.vx = result[3];
	currentState.vy = result[4];
	currentState.vz = result[5];
}

void simulate_next_step_x(double accX, double ts) {

	// Update current state
	currentState.x = ts * currentState.vx + 0.5 * pow(ts, 2) * accX
			+ currentState.x;
	currentState.vx = currentState.vx + accX * ts;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "tf_sender");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	ros::Subscriber taskSubscriber_ = n.subscribe("task", 1, &parseTask);
	statusPublisher = n.advertise<pap_common::Status>("status", 1000);
	// Initialize positions
	currentState.x = xHome;
	currentState.y = yHome;
	currentState.z = zHome;

	// Send transformBroadcast to initial position?

	while (ros::ok()) {

		// If there is a distance to go
		if (distXTotal != 0) {

			std::cout << "Simulation running" << std::endl;

			// Update controller status
			if (!controller1StatusSet) {
				//controllerState1.positionReached = false;
				//checkStatusController(1, &controllerState1);
				controller1StatusSet = true;
			}

			// update distance x, brake_path @ current velocity
			distX = desX - currentState.x;
			double brakeTime = currentState.vx / accX;
			double brakePath = 0.5 * accX * pow(brakeTime, 2.0);

			// More than half of total distance left -> accelerate or keep max velocity
			if (distX > (0.5 * distXTotal)) {
				if (currentState.vx < maxVelocity) {
					simulate_next_step_x(accX, ts);		// Accelerate until Vmax
				} else {
					simulate_next_step_x(0, ts);				// stay at Vmax
				}

				// Half distance done, keep v or slow down depending on distance left
			} else if (distX > epsilonDistance) {

				if (distX > brakePath) {
					simulate_next_step(0, 0, 0, ts);			// stay at Vmax
				} else {
					simulate_next_step_x(-accXDelay, ts);		// Slow down
				}

				// Distance left is now smaller than epsilonDistance
			} else {
				currentState.x = desX;
				currentState.vx = 0;
				distXTotal = 0;
				distX = 0;
				controller1StatusSet = false;
				//controllerState1.positionReached = true;
				//checkStatusController(1, &controllerState1);
			}

			std::cout << "currrent x: " << currentState.x << "current vx: " << currentState.vx << std::endl;
		}

		sendTransforms((currentState.x/1000), (currentState.y/1000), (currentState.z/1000), 0.0, 0.0);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void sendTransforms(double x, double y, double z, double nozzle_1,
		double nozzle_2) {
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Transform transformReference;
	tf::Transform transformX;
	tf::Transform transformY, transformZ, transformS1,transformS2;
//	static float x = 0.0;
//	static float y = 0.0;
//	static float z = 0.0;

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
	//x = x + 0.0002;
//	if (x >= 0.400)
//		x = 0.0;

//	y = y + 0.0002;
//	if (y >= 0.250)
//		y = 0.0;

//	z = z + 0.0002;
//		if (z >= 0.07)
//			z = 0.0;

	transformX.setOrigin(tf::Vector3(0.023026, 0.11628+x, 0.11244));
	tf::Quaternion qX;
	qX.setX(-0.699941);
	qX.setY(0.000131839);
	qX.setZ(0.000619792);
	qX.setW(0.7142);
	transformX.setRotation(qX);

	// Y-Link
	transformY.setOrigin(tf::Vector3(-0.11097 + y, 0.14813 + x, 0.16924));
	tf::Quaternion qY;
	qY.setX(-0.504923);
	qY.setY(0.494495);
	qY.setZ(0.505109);
	qY.setW(-0.495371);
	transformY.setRotation(qY);

	// Z-Link
	transformZ.setOrigin(tf::Vector3(-0.032259 + y, 0.16839 + x, 0.15228 + z));
	tf::Quaternion qZ;
	qZ.setX(0.000131839);
	qZ.setY(0.699941);
	qZ.setZ(0.7142);
	qZ.setW(-0.000619792);
	transformZ.setRotation(qZ);

	// Stepper1-Link
	transformS1.setOrigin(tf::Vector3(-0.044085 + y, 0.2214 + x, 0.10935+z));
	tf::Quaternion qS1;
	qS1.setX(0.00737794);
	qS1.setY(-0.706695);
	qS1.setZ(-0.00688299);
	qS1.setW(0.707446);
	transformS1.setRotation(qS1);

	// Stepper2-Link
	transformS2.setOrigin(tf::Vector3(-0.11908 + y, 0.22134 + x, 0.10935 +z));
	tf::Quaternion qS2;
	qS2.setX(0.00737794);
	qS2.setY(-0.706695);
	qS2.setZ(-0.00688299);
	qS2.setW(0.707446);
	transformS2.setRotation(qS2);

	br.sendTransform(
			tf::StampedTransform(transformReference, ros::Time::now(),
					"/base_link", "/world"));
	//br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/world","/base_link"));

	// Axes
	br.sendTransform(
			tf::StampedTransform(transformX, ros::Time::now(), "/base_link",
					"/x-axis"));
	br.sendTransform(
			tf::StampedTransform(transformY, ros::Time::now(), "/base_link",
					"/y-axis"));

	br.sendTransform(
				tf::StampedTransform(transformZ, ros::Time::now(), "/base_link",
						"/z-axis"));

	br.sendTransform(
					tf::StampedTransform(transformS1, ros::Time::now(), "/base_link",
							"/nozzle_1"));

	br.sendTransform(
					tf::StampedTransform(transformS2, ros::Time::now(), "/base_link",
							"/nozzle_2"));
}
