#include <robot_simulation/controller_simulator.h>

namespace controller_simulator{

const double maxVelocity = 150;		// mm/s
const double epsilonDistance = 1;		// mm
const double ts = 0.01;				// s

const double accX = 300;				// mm/s²
const double accY = 300;
const double accZ = 300;

const double accXDelay = 300;			// mm/s²
const double accYDelay = 300;
const double accZDelay = 300;

const double xHome = 0.0;
const double yHome = 0.0;
const double zHome = 0.0;	// mm

const double tip1Home = 0.0;
const double tip2Home = 0.0; //mm

const double matrix[6][6] = { { ts, 0.0, 0.0, 0.5 * pow(ts, 2.0), 0.0, 0.0 }, { 0.0,
                                                                                ts, 0.0, 0.0, 0.5 * pow(ts, 2.0), 0.0 }, { 0.0, 0.0, ts, 0.0, 0.0, 0.5
                                                                                                                           * pow(ts, 2.0) }, { 1.0, 0.0, 0.0, ts, 0.0, 0.0 }, { 0.0, 1.0, 0.0, 0.0,
                                                                                                                                                                                ts, 0.0 }, { 0.0, 0.0, 1.0, 0.0, 0.0, ts } };

ControllerSimulator::ControllerSimulator()
{

    energized = false;
    controllerEnergized = false;
    controllerConnected = false;

    controller1StatusSet = false;
    controller2StatusSet = false;
    controller3StatusSet = false;

    distXTotal = 0.0;
    distYTotal = 0.0;
    distZTotal = 0.0;
    distX = 0.0;
    distY = 0.0;
    distZ = 0.0;
    desX = 0.0;
    desY = 0.0;
    desZ = 0.0;

    currentState.x = xHome;
    currentState.y = yHome;
    currentState.z = zHome;
    currentState.tip1 = tip1Home;
    currentState.tip2 = tip2Home;

}

void ControllerSimulator::simulate_next_step(double accX, double accY, double accZ, double ts) {

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

void ControllerSimulator::simulate_next_step_x(double accX, double ts) {

    // Update current state
    currentState.x = ts * currentState.vx + 0.5 * pow(ts, 2) * accX
            + currentState.x;
    currentState.vx = currentState.vx + accX * ts;
}

void ControllerSimulator::simulate_next_step_y(double accY, double ts) {

    // Update current state
    currentState.y = ts * currentState.vy + 0.5 * pow(ts, 2) * accY
            + currentState.y;
    currentState.vy = currentState.vy + accY * ts;
}

void ControllerSimulator::simulate_next_step_z(double accZ, double ts) {

    // Update current state
    currentState.z = ts * currentState.vz + 0.5 * pow(ts, 2) * accZ
            + currentState.z;
    currentState.vz = currentState.vz + accZ * ts;
}

void ControllerSimulator::simulateXAxisMovement() {

    // If there is a positive distance to go
    if (distXTotal > 0) {

        // Update controller status
        if (!controller1StatusSet) {
            controllerState1.positionReached = false;
            //checkStatusController(1, &controllerState1, &oldControllerState1);
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
            //checkStatusController(1, &controllerState1, &oldControllerState1);
        }

        //std::cout << "currrent x: " << currentState.x << "  current vx: " << currentState.vx << std::endl;
    }

    // If there is a negative distance to go
    if (distXTotal < 0) {

        // Update controller status
        if (!controller1StatusSet) {
            controllerState1.positionReached = false;
            //checkStatusController(1, &controllerState1, &oldControllerState1);
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
            //checkStatusController(1, &controllerState1, &oldControllerState1);
        }

        //std::cout << "currrent x: " << currentState.x << "  current vx: " << currentState.vx << std::endl;
    }
}

void ControllerSimulator::simulateYAxisMovement() {

    // If there is a positive distance to go
    if (distYTotal > 0) {

        // Update controller status
        if (!controller2StatusSet) {
            controllerState2.positionReached = false;
            //checkStatusController(2, &controllerState2, &oldControllerState2);
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
            //checkStatusController(2, &controllerState2, &oldControllerState2);
        }

        //std::cout << "currrent x: " << currentState.x << "  current vx: " << currentState.vx << std::endl;
    }

    // If there is a negative distance to go
    if (distYTotal < 0) {

        // Update controller status
        if (!controller2StatusSet) {
            controllerState2.positionReached = false;
            //checkStatusController(2, &controllerState2, &oldControllerState2);
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
            //checkStatusController(2, &controllerState2, &oldControllerState2);
        }

        //std::cout << "currrent x: " << currentState.x << "  current vx: " << currentState.vx << std::endl;
    }
}

void ControllerSimulator::simulateZAxisMovement() {

    // If there is a positive distance to go
    if (distZTotal > 0) {

        // Update controller status
        if (!controller3StatusSet) {
            controllerState3.positionReached = false;
            //checkStatusController(3, &controllerState3, &oldControllerState3);
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
            //checkStatusController(3, &controllerState3, &oldControllerState3);
        }

        //std::cout << "currrent x: " << currentState.x << "  current vx: " << currentState.vx << std::endl;
    }

    // If there is a negative distance to go
    if (distZTotal < 0) {

        // Update controller status
        if (!controller3StatusSet) {
            controllerState3.positionReached = false;
            //checkStatusController(3, &controllerState3, &oldControllerState3);
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
            //checkStatusController(3, &controllerState3, &oldControllerState3);
        }

        //std::cout << "currrent x: " << currentState.x << "  current vx: " << currentState.vx << std::endl;
    }
}

int ControllerSimulator::gotoCoord(float x, float y, float z, float velX, float velY, float velZ ){
    if (controllerConnected && controllerEnergized) {
        desX = x;							// Desired position
        distXTotal = desX - currentState.x;		// Distance we have to go
        desY = y;							// Desired position
        distYTotal = desY - currentState.y;		// Distance we have to go
        desZ = z;							// Desired position
        distZTotal = desZ - currentState.z;		// Distance we have to go
    }
    return 0;
}


void ControllerSimulator::energizeAllAxis(bool activate){
    if(activate){
        energizeAxis(pap_common::XMOTOR, true);
        energizeAxis(pap_common::YMOTOR, true);
        energizeAxis(pap_common::ZMOTOR, true);
    }else{
        energizeAxis(pap_common::XMOTOR, false);
        energizeAxis(pap_common::YMOTOR, false);
        energizeAxis(pap_common::ZMOTOR, false);
    }
}

bool ControllerSimulator::energizeAxis(enum pap_common::MOTOR adressDevice, bool trigger){
    switch (adressDevice){
    case pap_common::MOTOR::XMOTOR:
        if(trigger)
        {
            controllerEnergized = true;
            controllerState1.energized = true;
            controllerState1.positionReached = true;
            controllerState1.error = false;
        }
        else
        {
            controllerEnergized = false;
            controllerState1.energized = false;
            controllerState1.positionReached = false;
            controllerState1.error = false;
        }
        break;

    case pap_common::MOTOR::YMOTOR:
        if(trigger)
        {
            controllerEnergized = true;
            controllerState2.energized = true;
            controllerState2.positionReached = true;
            controllerState2.error = false;
        }
        else
        {
            controllerEnergized = false;
            controllerState2.energized = false;
            controllerState2.positionReached = false;
            controllerState2.error = false;
        }

        break;

    case pap_common::MOTOR::ZMOTOR:
        if(trigger)
        {
            controllerEnergized = true;
            controllerState3.energized = true;
            controllerState3.positionReached = true;
            controllerState3.error = false;
        }
        else
        {
            controllerEnergized = false;
            controllerState3.energized = false;
            controllerState3.positionReached = false;
            controllerState3.error = false;
        }
        break;
    }
    return true;
}

void ControllerSimulator::searchForDevices(){
    connectController(true);
}

void ControllerSimulator::connectController(bool connect){
    ROS_INFO("Searching for devices...");
    ROS_INFO("Controller 1 connected...");
    ROS_INFO("Controller 2 connected...");
    ROS_INFO("Controller 3 connected...");
    controllerState1.positionReached = false;
    controllerState2.positionReached = false;
    controllerState3.positionReached = false;
    controllerState1.error = false;
    controllerState2.error = false;
    controllerState3.error = false;

    if (connect) {
        controllerConnected = true;
    } else {
        controllerConnected = false;
    }
}

motor_controller::controllerStatus ControllerSimulator::getFullStatusController(enum pap_common::MOTOR addressDevice){
    switch(addressDevice){
    case pap_common::MOTOR::XMOTOR:
        controllerState1.position = currentState.x;
        controllerState1.failed= false;
        return controllerState1;
        break;

    case pap_common::MOTOR::YMOTOR:
        controllerState2.position = currentState.y;
        controllerState2.failed= false;
        return controllerState2;
        break;

    case pap_common::MOTOR::ZMOTOR:
        controllerState3.position = currentState.z;
        controllerState3.failed= false;
        return controllerState3;
        break;
    }

}

void ControllerSimulator::simulationStep(void){
    if (controllerConnected && controllerEnergized) {
        if (distXTotal != 0)
            simulateXAxisMovement();
        if (distYTotal != 0)
            simulateYAxisMovement();
        if (distZTotal != 0)
            simulateZAxisMovement();
    }
}

bool ControllerSimulator::sendHoming(){
    gotoCoord(xHome, yHome, zHome);
    return true;
}

bool ControllerSimulator::isConnected(enum pap_common::MOTOR device){
    return controllerConnected;
}

void ControllerSimulator::connectToBus(){
    this->start();
}

void ControllerSimulator::run(){
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        simulationStep();
        loop_rate.sleep();
    }
}

}
