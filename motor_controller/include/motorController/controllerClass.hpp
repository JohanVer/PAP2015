#ifndef CONTROLLERCLASS_H
#define CONTROLLERCLASS_H

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <vector>
#include <cmath>
#include "pap_common/Task.h"
#include "pap_common/Status.h"
#include <pap_common/task_message_def.h>
#include <pap_common/status_message_def.h>
#include <motorController/controller_interface.h>
#include <stdio.h>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"

namespace motor_controller{

class motorController: public motor_controller::ControllerInterface {
public:
	motorController();
	~motorController();

	void ReqParker(unsigned char rw, unsigned char* data, unsigned char length,
			unsigned char adress);
	unsigned int readInt16AddressParker(unsigned char addressDevice,
			unsigned int adressReg, unsigned char subadress);
	float readFloat32AddressParker(unsigned char addressDevice,
				unsigned int adressReg, unsigned char subadress);
	bool writeInt16AddressParker(unsigned char addressDevice,
			unsigned int adressReg, unsigned char subadress, unsigned int data);
	bool writeInt32AddressParker(unsigned char addressDevice,
			unsigned int adressReg, unsigned char subadress, unsigned int data);
	bool writeFloat32AddressParker(unsigned char addressDevice,
			unsigned int adressReg, unsigned char subadress, float data);
	bool checkForACK();

	bool sendHoming();
	void searchForDevices();
	void connectToBus();
	bool checkError(unsigned char adressDevice);
    bool isConnected(enum pap_common::MOTOR device);
    bool disconnect(enum pap_common::MOTOR device);
	controllerStatus getStatusController(unsigned char adressDevice);
    controllerStatus getFullStatusController(pap_common::MOTOR addressDevice);
	bool positionReached(unsigned char adressDevice);
	float getPosition(unsigned char adressDevice);
	bool axisEnergized(unsigned char adressDevice);
    bool energizeAxis(pap_common::MOTOR adressDevice, bool trigger);
	bool setSetting(unsigned char addressDevice, unsigned int line, float goal,
			float vel, unsigned int acc, unsigned int dec);
	bool startSetting(unsigned char adressDevice, unsigned int settingAddress);
    bool manual(pap_common::MOTOR deviceAddress, unsigned char direction);
	int  gotoCoord(float x, float y, float z, float velX = 50.0, float velY = 300.0, float velZ = 100.0 );
    bool stop(pap_common::MOTOR deviceAddress);


	bool controllerConnected_1_,controllerConnected_2_,controllerConnected_3_;
private:
	unsigned int UpdateCRC16(unsigned int crc, unsigned char wert);
	//serial::Serial serialPort;
	unsigned char buffer[100];
	unsigned int controlWord1;
    std::unique_ptr<serial::Serial> serialPort_;

};

}
#endif // CONTROLLERCLASS_H
