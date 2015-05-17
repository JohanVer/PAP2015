#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <vector>
#include <cmath>
#include "pap_common/Task.h"
#include "pap_common/Status.h"
#include "../../../pap_common/include/pap_common/task_message_def.h"
#include "../../../pap_common/include/pap_common/status_message_def.h"
#include <stdio.h>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"

namespace error_codes {
enum ERROR_CODES {
	NO_ERROR, X_ERROR, Y_ERROR, Z_ERROR
};
}

class controllerStatus {
public:
	controllerStatus() {
		error = false;
		energized = false;
		positionReached = false;
	}

	bool error;
	bool energized;
	bool positionReached;
private:
};

class motorController {
public:
	motorController();
	~motorController();

	void ReqParker(unsigned char rw, unsigned char* data, unsigned char length,
			unsigned char adress);
	unsigned int readInt16AddressParker(unsigned char addressDevice,
			unsigned int adressReg, unsigned char subadress);
	bool writeInt16AddressParker(unsigned char addressDevice,
			unsigned int adressReg, unsigned char subadress, unsigned int data);
	bool writeInt32AddressParker(unsigned char addressDevice,
			unsigned int adressReg, unsigned char subadress, unsigned int data);
	bool writeFloat32AddressParker(unsigned char addressDevice,
			unsigned int adressReg, unsigned char subadress, float data);
	bool checkForACK();

	bool sendHoming();
	void connectToBus();
	bool checkError(unsigned char adressDevice);
	controllerStatus getStatusController(unsigned char adressDevice);
	bool positionReached(unsigned char adressDevice);
	bool axisEnergized(unsigned char adressDevice);
	bool energizeAxis(unsigned char adressDevice, bool trigger);
	bool setSetting(unsigned char addressDevice, unsigned int line, float goal,
			float vel, unsigned int acc, unsigned int dec);
	bool startSetting(unsigned char adressDevice, unsigned int settingAddress);
	bool manual(unsigned char deviceAddress, unsigned char direction);
	int gotoCoord(float x, float y, float z);
private:
	unsigned int UpdateCRC16(unsigned int crc, unsigned char wert);
	//serial::Serial serialPort;
	unsigned char buffer[100];
	unsigned int controlWord1;
	bool controllerConnected_1_,controllerConnected_2_,controllerConnected_3_;
};
