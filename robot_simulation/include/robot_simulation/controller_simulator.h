#ifndef CONTROLLER_SIMULATOR_H
#define CONTROLLER_SIMULATOR_H

#include <ros/ros.h>
#include <motorController/controller_interface.h>
#include <pap_common/task_message_def.h>
#include <QThread>

namespace controller_simulator{

struct state {
    double x, y, z;
    double vx, vy, vz;
    double tip1, tip2;
};

class ControllerSimulator : public QThread
{
public:
    ControllerSimulator();

    virtual void gotoCoord(float x, float y, float z, float velX = 50.0, float velY = 300.0, float velZ = 100.0 );
    virtual void energizeAxis(enum pap_common::MOTOR adressDevice, bool trigger);
    virtual motor_controller::controllerStatus getFullStatusController(enum pap_common::MOTOR addressDevice);
    virtual bool isConnected(enum pap_common::MOTOR device);
    virtual void sendHoming();
    virtual void connectToBus();
    virtual void searchForDevices();

    void energizeAllAxis(bool activate);
    void connectController(bool connect);
    void run();
    void simulationStep(void);

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

    motor_controller::controllerStatus controllerState1, controllerState2, controllerState3;

};


}
#endif // CONTROLLER_SIMULATOR_H
