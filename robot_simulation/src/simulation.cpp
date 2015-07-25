#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "math.h"
#include "../../motor_controller/include/motorController/controllerClass.hpp"
#include "../../pap_common/include/pap_common/arduino_message_def.h"
#include <pap_common/ArduinoMsg.h>
#include <pap_common/Status.h>

/*****************************************************************************
 * Parameter
 *****************************************************************************/
double maxVelocity = 150;		// mm/s
double epsilonDistance = 1;		// mm
double ts = 0.01;				// s

double accX = 300;				// mm/s²
double accY = 300;
double accZ = 300;

double accXDelay = 300;			// mm/s²
double accYDelay = 300;
double accZDelay = 300;

double xHome, yHome, zHome = 0.0;	// mm
double tip1Home, tip2Home = 0.0;

bool energized = false;
bool controllerEnergized = false;
bool controllerConnected = false;

bool controller1StatusSet = false;
bool controller2StatusSet = false;
bool controller3StatusSet = false;

double distXTotal, distYTotal, distZTotal = 0;
double distX, distY, distZ = 0;
double desX, desY, desZ = 0;

controllerStatus controllerState1, controllerState2, controllerState3;
ros::Publisher statusPublisher;

void sendTransforms(double x_des, double y_des, double z_des, double nozzle_1,
		double nozzle_2);
void simulateXAxisMovement(void);
void simulateYAxisMovement(void);
void simulateZAxisMovement(void);

struct state {
	double x, y, z;
	double vx, vy, vz;
	double tip1, tip2;
} currentState;

double matrix[6][6] = { { ts, 0.0, 0.0, 0.5 * pow(ts, 2.0), 0.0, 0.0 }, { 0.0,
		ts, 0.0, 0.0, 0.5 * pow(ts, 2.0), 0.0 }, { 0.0, 0.0, ts, 0.0, 0.0, 0.5
		* pow(ts, 2.0) }, { 1.0, 0.0, 0.0, ts, 0.0, 0.0 }, { 0.0, 1.0, 0.0, 0.0,
		ts, 0.0 }, { 0.0, 0.0, 1.0, 0.0, 0.0, ts } };

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

void parseArduinoTask(const pap_common::ArduinoMsg& taskMsg) {
	if ((taskMsg.command == 2 && taskMsg.data == 3)
			|| (taskMsg.command == 1 && taskMsg.data == 6)) {
		currentState.tip1 = -20.00;
	} else if ((taskMsg.command == 1 && taskMsg.data == 3)
			|| (taskMsg.command == 2 && taskMsg.data == 6)) {
		currentState.tip1 = 0.00;
	}
	if (taskMsg.data == 7) {
		if (taskMsg.command == 1) {
			currentState.tip2 = -20.00;
		} else {
			currentState.tip2 = 0.00;
		}
	}
}

void parseTask(const pap_common::TaskConstPtr& taskMsg) {
	switch (taskMsg->destination) {
	case pap_common::CONTROLLER:
		switch (taskMsg->task) {
		case pap_common::HOMING:
			if (controllerConnected && controllerEnergized) {
				desX = xHome;							// Desired position
				distXTotal = desX - currentState.x;	// Distance we have to go

				desY = yHome;							// Desired position
				distYTotal = desY - currentState.y;	// Distance we have to go

				desZ = zHome;							// Desired position
				distZTotal = desZ - currentState.z;	// Distance we have to go
			}

			break;
		case pap_common::CURRENT:
			if (!energized) {
				controllerState1.energized = true;
				controllerState2.energized = true;
				controllerState3.energized = true;
				controllerEnergized = true;
			} else {
				controllerState1.energized = false;
				controllerState2.energized = false;
				controllerState3.energized = false;
				controllerEnergized = false;
			}
			break;
		case pap_common::COORD:
			if (controllerConnected && controllerEnergized) {
				desX = taskMsg->data1;					// Desired position
				distXTotal = desX - currentState.x;	// Distance we have to go

				desY = taskMsg->data2;					// Desired position
				distYTotal = desY - currentState.y;	// Distance we have to go

				desZ = taskMsg->data3;					// Desired position
				distZTotal = desZ - currentState.z;	// Distance we have to go
			}
			break;

		case pap_common::COORD_VEL:
			if (controllerConnected && controllerEnergized) {
				desX = taskMsg->data1;					// Desired position
				distXTotal = desX - currentState.x;	// Distance we have to go

				desY = taskMsg->data2;					// Desired position
				distYTotal = desY - currentState.y;	// Distance we have to go

				desZ = taskMsg->data3;					// Desired position
				distZTotal = desZ - currentState.z;	// Distance we have to go
			}
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

			if (!controllerConnected) {
				controllerConnected = true;
			} else {
				controllerConnected = false;
			}

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

void simulate_next_step_y(double accY, double ts) {

	// Update current state
	currentState.y = ts * currentState.vy + 0.5 * pow(ts, 2) * accY
			+ currentState.y;
	currentState.vy = currentState.vy + accY * ts;
}

void simulate_next_step_z(double accZ, double ts) {

	// Update current state
	currentState.z = ts * currentState.vz + 0.5 * pow(ts, 2) * accZ
			+ currentState.z;
	currentState.vz = currentState.vz + accZ * ts;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "tf_sender");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	ros::Subscriber taskSubscriber_ = n.subscribe("task", 1, &parseTask);
	ros::Subscriber ardunioTaskSubscriber_ = n.subscribe("arduinoTx", 1,
			&parseArduinoTask);
	statusPublisher = n.advertise<pap_common::Status>("status", 1000);

	// Initialize positions
	currentState.x = xHome;
	currentState.y = yHome;
	currentState.z = zHome;
	currentState.tip1 = tip1Home;
	currentState.tip2 = tip2Home;
	unsigned int counter = 0;
	while (ros::ok()) {

		counter++;
		if (counter == 10) {
			pap_common::Status stateMessage;
			stateMessage.data1 = pap_common::XMOTOR;
			stateMessage.posX = currentState.x;
			statusPublisher.publish(stateMessage);
			stateMessage.data1 = pap_common::YMOTOR;
			stateMessage.posY = currentState.y;
			statusPublisher.publish(stateMessage);
			stateMessage.data1 = pap_common::ZMOTOR;
			stateMessage.posZ = currentState.z;
			statusPublisher.publish(stateMessage);
			counter = 0;
		}

		if (controllerConnected && controllerEnergized) {
			if (distXTotal != 0)
				simulateXAxisMovement();
			if (distYTotal != 0)
				simulateYAxisMovement();
			if (distZTotal != 0)
				simulateZAxisMovement();
		}

		// Init and set tips!
		sendTransforms((currentState.x / 1000), (currentState.y / 1000),
				(currentState.z / 1000), (currentState.tip1 / 1000),
				(currentState.tip2 / 1000));

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void simulateXAxisMovement() {

	// If there is a positive distance to go
	if (distXTotal > 0) {

		// Update controller status
		if (!controller1StatusSet) {
			controllerState1.positionReached = false;
			checkStatusController(1, &controllerState1);
			controller1StatusSet = true;
		}

		// update distance x, brake_path @ current velocity
		distX = desX - currentState.x;
		double brakeTime = currentState.vx / accXDelay;
		double brakePath = 0.5 * accXDelay * pow(brakeTime, 2.0);

		// More than half of total distance left -> accelerate or keep max velocity
		if (distX > (0.5 * distXTotal)) {
			if (currentState.vx < maxVelocity) {
				simulate_next_step_x(accX, ts);			// Accelerate until Vmax
			} else {
				currentState.vx = maxVelocity;
				simulate_next_step_x(0, ts);				// stay at Vmax
			}

			// Half distance done, keep v or slow down depending on distance left
		} else if (distX > epsilonDistance && currentState.vx > 0) {

			if (distX > brakePath) {
				simulate_next_step_x(0, ts);				// stay at Vmax
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
			controllerState1.positionReached = true;
			checkStatusController(1, &controllerState1);
		}

		//std::cout << "currrent x: " << currentState.x << "  current vx: " << currentState.vx << std::endl;
	}

	// If there is a negative distance to go
	if (distXTotal < 0) {

		// Update controller status
		if (!controller1StatusSet) {
			controllerState1.positionReached = false;
			checkStatusController(1, &controllerState1);
			controller1StatusSet = true;
		}

		// update distance x, brake_path @ current velocity
		distX = abs(desX - currentState.x);
		double brakeTime = abs(currentState.vx) / accXDelay;
		double brakePath = 0.5 * accXDelay * pow(brakeTime, 2.0);

		// More than half of total distance left -> accelerate or keep max velocity
		if (distX > (0.5 * abs(distXTotal))) {
			if (abs(currentState.vx) < maxVelocity) {
				simulate_next_step_x(-accX, ts);		// Accelerate until Vmax
			} else {
				currentState.vx = -maxVelocity;
				simulate_next_step_x(0, ts);				// stay at Vmax
			}

			// Half distance done, keep v or slow down depending on distance left
		} else if (distX > epsilonDistance && abs(currentState.vx) > 0) {

			if (distX > brakePath) {
				simulate_next_step_x(0, ts);				// stay at Vmax
			} else {
				simulate_next_step_x(+accXDelay, ts);		// Slow down
			}

			// Distance left is now smaller than epsilonDistance
		} else {
			currentState.x = desX;
			currentState.vx = 0;
			distXTotal = 0;
			distX = 0;
			controller1StatusSet = false;
			controllerState1.positionReached = true;
			checkStatusController(1, &controllerState1);
		}

		//std::cout << "currrent x: " << currentState.x << "  current vx: " << currentState.vx << std::endl;
	}
}

void simulateYAxisMovement() {

	// If there is a positive distance to go
	if (distYTotal > 0) {

		// Update controller status
		if (!controller2StatusSet) {
			controllerState2.positionReached = false;
			checkStatusController(2, &controllerState2);
			controller2StatusSet = true;
		}

		// update distance x, brake_path @ current velocity
		distY = desY - currentState.y;
		double brakeTime = currentState.vy / accYDelay;
		double brakePath = 0.5 * accYDelay * pow(brakeTime, 2.0);

		// More than half of total distance left -> accelerate or keep max velocity
		if (distY > (0.5 * distYTotal)) {
			if (currentState.vy < maxVelocity) {
				simulate_next_step_y(accY, ts);			// Accelerate until Vmax
			} else {
				currentState.vy = maxVelocity;
				simulate_next_step_y(0, ts);				// stay at Vmax
			}

			// Half distance done, keep v or slow down depending on distance left
		} else if (distY > epsilonDistance && currentState.vy > 0) {

			if (distY > brakePath) {
				simulate_next_step_y(0, ts);				// stay at Vmax
			} else {
				simulate_next_step_y(-accYDelay, ts);		// Slow down
			}

			// Distance left is now smaller than epsilonDistance
		} else {
			currentState.y = desY;
			currentState.vy = 0;
			distYTotal = 0;
			distY = 0;
			controller2StatusSet = false;
			controllerState2.positionReached = true;
			checkStatusController(2, &controllerState2);
		}

		//std::cout << "currrent x: " << currentState.x << "  current vx: " << currentState.vx << std::endl;
	}

	// If there is a negative distance to go
	if (distYTotal < 0) {

		// Update controller status
		if (!controller2StatusSet) {
			controllerState2.positionReached = false;
			checkStatusController(2, &controllerState2);
			controller2StatusSet = true;
		}

		// update distance x, brake_path @ current velocity
		distY = abs(desY - currentState.y);
		double brakeTime = abs(currentState.vy) / accYDelay;
		double brakePath = 0.5 * accYDelay * pow(brakeTime, 2.0);

		// More than half of total distance left -> accelerate or keep max velocity
		if (distY > (0.5 * abs(distYTotal))) {
			if (abs(currentState.vy) < maxVelocity) {
				simulate_next_step_y(-accY, ts);		// Accelerate until Vmax
			} else {
				currentState.vy = -maxVelocity;
				simulate_next_step_y(0, ts);				// stay at Vmax
			}

			// Half distance done, keep v or slow down depending on distance left
		} else if (distY > epsilonDistance && abs(currentState.vy) > 0) {

			if (distY > brakePath) {
				simulate_next_step_y(0, ts);				// stay at Vmax
			} else {
				simulate_next_step_y(+accYDelay, ts);		// Slow down
			}

			// Distance left is now smaller than epsilonDistance
		} else {
			currentState.y = desY;
			currentState.vy = 0;
			distYTotal = 0;
			distY = 0;
			controller2StatusSet = false;
			controllerState2.positionReached = true;
			checkStatusController(2, &controllerState2);
		}

		//std::cout << "currrent x: " << currentState.x << "  current vx: " << currentState.vx << std::endl;
	}
}

void simulateZAxisMovement() {

	// If there is a positive distance to go
	if (distZTotal > 0) {

		// Update controller status
		if (!controller3StatusSet) {
			controllerState3.positionReached = false;
			checkStatusController(3, &controllerState3);
			controller3StatusSet = true;
		}

		// update distance x, brake_path @ current velocity
		distZ = desZ - currentState.z;
		double brakeTime = currentState.vz / accZDelay;
		double brakePath = 0.5 * accZDelay * pow(brakeTime, 2.0);

		// More than half of total distance left -> accelerate or keep max velocity
		if (distZ > (0.5 * distZTotal)) {
			if (currentState.vz < maxVelocity) {
				simulate_next_step_z(accZ, ts);			// Accelerate until Vmax
			} else {
				currentState.vz = maxVelocity;
				simulate_next_step_z(0, ts);				// stay at Vmax
			}

			// Half distance done, keep v or slow down depending on distance left
		} else if (distZ > epsilonDistance && currentState.vz > 0) {

			if (distZ > brakePath) {
				simulate_next_step_z(0, ts);				// stay at Vmax
			} else {
				simulate_next_step_z(-accZDelay, ts);		// Slow down
			}

			// Distance left is now smaller than epsilonDistance
		} else {
			currentState.z = desZ;
			currentState.vz = 0;
			distZTotal = 0;
			distZ = 0;
			controller3StatusSet = false;
			controllerState3.positionReached = true;
			checkStatusController(3, &controllerState3);
		}

		//std::cout << "currrent x: " << currentState.x << "  current vx: " << currentState.vx << std::endl;
	}

	// If there is a negative distance to go
	if (distZTotal < 0) {

		// Update controller status
		if (!controller3StatusSet) {
			controllerState3.positionReached = false;
			checkStatusController(3, &controllerState3);
			controller3StatusSet = true;
		}

		// update distance x, brake_path @ current velocity
		distZ = abs(desZ - currentState.z);
		double brakeTime = abs(currentState.vz) / accZDelay;
		double brakePath = 0.5 * accZDelay * pow(brakeTime, 2.0);

		// More than half of total distance left -> accelerate or keep max velocity
		if (distZ > (0.5 * abs(distZTotal))) {
			if (abs(currentState.vz) < maxVelocity) {
				simulate_next_step_z(-accZ, ts);		// Accelerate until Vmax
			} else {
				currentState.vz = -maxVelocity;
				simulate_next_step_z(0, ts);				// stay at Vmax
			}

			// Half distance done, keep v or slow down depending on distance left
		} else if (distZ > epsilonDistance && abs(currentState.vz) > 0) {

			if (distZ > brakePath) {
				simulate_next_step_z(0, ts);				// stay at Vmax
			} else {
				simulate_next_step_z(+accZDelay, ts);		// Slow down
			}

			// Distance left is now smaller than epsilonDistance
		} else {
			currentState.z = desZ;
			currentState.vz = 0;
			distZTotal = 0;
			distZ = 0;
			controller3StatusSet = false;
			controllerState3.positionReached = true;
			checkStatusController(3, &controllerState3);
		}

		//std::cout << "currrent x: " << currentState.x << "  current vx: " << currentState.vx << std::endl;
	}
}

void sendTransforms(double x, double y, double z, double nozzle_1,
		double nozzle_2) {
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Transform transformReference;
	tf::Transform transformX;
	tf::Transform transformY, transformZ, transformS1, transformS2;
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

	transformX.setOrigin(tf::Vector3(0.023026, 0.11628 + x, 0.11244));
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
	//transformS1.setOrigin(tf::Vector3(-0.044085 + y, 0.2214 + x, 0.10935+z));
	transformS1.setOrigin(
			tf::Vector3(-0.044085 + y, 0.2214 + x, 0.10935 + z + nozzle_1));
	tf::Quaternion qS1;
	qS1.setX(0.00737794);
	qS1.setY(-0.706695);
	qS1.setZ(-0.00688299);
	qS1.setW(0.707446);
	transformS1.setRotation(qS1);

	// Stepper2-Link
	//transformS2.setOrigin(tf::Vector3(-0.11908 + y, 0.22134 + x, 0.10935 +z));
	transformS2.setOrigin(
			tf::Vector3(-0.11908 + y, 0.22134 + x, 0.10935 + z + nozzle_2));// Change Niko
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
