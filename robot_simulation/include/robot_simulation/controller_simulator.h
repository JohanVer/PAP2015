#ifndef CONTROLLER_SIMULATOR_H
#define CONTROLLER_SIMULATOR_H

#include <motorController/controllerClass.hpp>
#include <pap_common/task_message_def.h>

namespace controller_simulator{

struct state {
    double x, y, z;
    double vx, vy, vz;
    double tip1, tip2;
};

class ControllerSimulator
{
public:
    ControllerSimulator();

    void gotoCoord(const double x,  const double y, const double z);
    void energizeAllAxis(bool activate);
    void energizeAxis(enum pap_common::MOTOR device, bool activate);
    void connectController(bool connect);
    controllerStatus getStatusController(enum pap_common::MOTOR device);
    void simulationStep(void);
    void homing();
    bool isConnected();

private:
    void simulate_next_step(double accX, double accY, double accZ, double ts);
    void simulate_next_step_x(double accX, double ts);
    void simulate_next_step_y(double accY, double ts);
    void simulate_next_step_z(double accZ, double ts);
    void simulateXAxisMovement(void);
    void simulateYAxisMovement(void);
    void simulateZAxisMovement(void);


    state currentState;

    bool energized;
    bool controllerEnergized;
    bool controllerConnected;

    bool controller1StatusSet;
    bool controller2StatusSet;
    bool controller3StatusSet;

    double distXTotal, distYTotal, distZTotal;
    double distX, distY, distZ;
    double desX, desY, desZ;

    controllerStatus controllerState1, controllerState2, controllerState3;

};


}
#endif // CONTROLLER_SIMULATOR_H
