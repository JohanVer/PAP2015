#include "../include/motorController/controllerClass.hpp"

using namespace std;
using namespace error_codes;

serial::Serial serialPort("/dev/ttyUSB0", 38400,
		serial::Timeout::simpleTimeout(100));

const unsigned int CRC16_table[256] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
		0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
		0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
		0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
		0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
		0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
		0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
		0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
		0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
		0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
		0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
		0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
		0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
		0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
		0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
		0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
		0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
		0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
		0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
		0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
		0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
		0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
		0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
		0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
		0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };

unsigned int motorController::UpdateCRC16(unsigned int crc,
		unsigned char wert) {
	unsigned int crc16;
	crc16 = (CRC16_table[(crc >> 8) & 0x00FF] ^ (crc << 8)
			^ (unsigned int) (wert));
	return crc16;
}

motorController::motorController() {
	controlWord1 = 0;
	controllerConnected_1_ = false;
	controllerConnected_2_ = false;
	controllerConnected_3_ = false;
}

motorController::~motorController() {
	serialPort.close();
}

// rw = 0xAD for read, rw = 0xCF for write
void motorController::ReqParker(unsigned char rw, unsigned char* data,
		unsigned char length, unsigned char adress) {

	int totalLength = 5 + length;


	unsigned char dataComplete[totalLength];
	dataComplete[0] = rw;
	dataComplete[1] = adress;
	dataComplete[2] = length - 1;

	// Put in data
	for (int i = 0; i < length; i++) {
		dataComplete[i + 3] = data[i];
	}

	// Calc CRC
	unsigned int crc = 0;
	for (int j = 0; j < totalLength - 2; j++) {
		crc = UpdateCRC16(crc, dataComplete[j]);
	}

	dataComplete[3 + length] = (crc >> 8) & 0xff;
	dataComplete[4 + length] = crc & 0xff;

	size_t bytes_wrote = serialPort.write((uint8_t*) dataComplete,
			(size_t) totalLength);
	if (bytes_wrote != totalLength) {
		ROS_ERROR("Error while writing to serial port");
	}
}
// Reads a 16-Bit-Integer from parker device
unsigned int motorController::readInt16AddressParker(
		unsigned char addressDevice, unsigned int adressReg,
		unsigned char subadress) {
	buffer[0] = (adressReg >> 8) & 0xff;
	buffer[1] = (adressReg & 0xff);
	buffer[2] = subadress;

	ReqParker(0xAD, buffer, 3, addressDevice);

	unsigned char readBuffer[10];

	// Read data
	unsigned char numToRead = 6;
	int readBytes = serialPort.read(readBuffer, numToRead);

	if (readBytes != numToRead) {
		ROS_ERROR("Unexpected end of serial data %d",readBytes);
		return 0;
	}
	// Calc CRC
	unsigned int crc = 0;
	for (int j = 0; j < numToRead - 2; j++) {
		crc = UpdateCRC16(crc, readBuffer[j]);
	}

	unsigned char crc_h = (crc >> 8) & 0xff;
	unsigned char crc_l = crc & 0xff;

	// Check crc and start sequence
	if ((crc_h == readBuffer[numToRead - 2])
			&& (crc_l == readBuffer[numToRead - 1]) && (readBuffer[0] = 0x05)) {
		return (readBuffer[2] << 8) | (readBuffer[3]);
	} else {
		ROS_ERROR("CRC or start-sequence failed");
		return 0;
	}
}

// Write a 16-Bit-Integer to parker device
bool motorController::writeInt16AddressParker(unsigned char addressDevice,
		unsigned int adressReg, unsigned char subadress, unsigned int data) {
	buffer[0] = (adressReg >> 8) & 0xff;
	buffer[1] = (adressReg & 0xff);
	buffer[2] = subadress;

	buffer[3] = (data >> 8) & 0xff;
	buffer[4] = (data) & 0xff;
	ReqParker(0xCD, buffer, 5, addressDevice);
	if (checkForACK()) {
		return true;
	} else
		return false;
}

// Write a 32-Bit-Integer to parker device
bool motorController::writeInt32AddressParker(unsigned char addressDevice,
		unsigned int adressReg, unsigned char subadress, unsigned int data) {
	buffer[0] = (adressReg >> 8) & 0xff;
	buffer[1] = (adressReg & 0xff);
	buffer[2] = subadress;

	buffer[3] = (data >> 24) & 0xff;
	buffer[4] = (data >> 16) & 0xff;
	buffer[5] = (data >> 8) & 0xff;
	buffer[6] = (data) & 0xff;
	ReqParker(0xCD, buffer, 7, addressDevice);
	if (checkForACK()) {
		return true;
	} else
		return false;
}

// Write a 32-Bit-Float to parker device
bool motorController::writeFloat32AddressParker(unsigned char addressDevice,
		unsigned int adressReg, unsigned char subadress, float data) {
	buffer[0] = (adressReg >> 8) & 0xff;
	buffer[1] = (adressReg & 0xff);
	buffer[2] = subadress;

	// Divide the float into 4 bytes
	float f = data;
	unsigned char c[sizeof f];
	if (sizeof f != 4) {
		ROS_ERROR("Float size is other than 4");
	}
	memcpy(c, &f, sizeof f);

	buffer[3] = c[3];
	buffer[4] = c[2];
	buffer[5] = c[1];
	buffer[6] = c[0];

	ReqParker(0xCD, buffer, 7, addressDevice);
	if (checkForACK()) {
		return true;
	} else
		return false;
}

// Try's to read the acknowledge sequence
bool motorController::checkForACK() {
	unsigned int totalLength = 6;
	unsigned char readBuffer[100];
	int readBytes = serialPort.read(readBuffer, totalLength);

	if (readBytes != totalLength) {
		ROS_ERROR("Unexpected end of serial data %d",readBytes);
		return false;
	}

	unsigned int crc = 0;
	for (int j = 0; j < totalLength - 2; j++) {
		crc = UpdateCRC16(crc, readBuffer[j]);
	}

	unsigned char crc_h = (crc >> 8) & 0xff;
	unsigned char crc_l = crc & 0xff;

	if ((crc_h == readBuffer[totalLength - 2])
			&& (crc_l == readBuffer[totalLength - 1])) {
		if (readBuffer[0] == 0x06) {
			return true;
		} else if (readBuffer[0] == 0x07) {
			ROS_ERROR("Got NACK");
			return false;
		} else {
			ROS_ERROR("Unexpected syntax");
			return false;
		}
	} else {
		ROS_ERROR("CRC failed");
		return false;
	}
}

void motorController::searchForDevices() {
	controllerStatus statusTemp;
	statusTemp = getStatusController(1);
	if (statusTemp.failed == false) {
		controllerConnected_1_ = true;
		ROS_INFO("Controller 1 connected...");
	} else {
		controllerConnected_1_ = false;
	}

	statusTemp = getStatusController(2);
	if (statusTemp.failed == false) {
		controllerConnected_2_ = true;
		ROS_INFO("Controller 2 connected...");
	} else {
		controllerConnected_2_ = false;
	}

	statusTemp = getStatusController(3);
	if (statusTemp.failed == false) {
		controllerConnected_3_ = true;
		ROS_INFO("Controller 3 connected...");
	} else {
		controllerConnected_3_ = false;
	}
}

void motorController::connectToBus() {
	if (serialPort.isOpen())
		ROS_INFO("Serialport opened...");
	else
		ROS_ERROR("Serialport could not be opened");
}

bool motorController::checkError(unsigned char adressDevice) {
	unsigned int statusByte = readInt16AddressParker(adressDevice, 1000, 3);
	ROS_INFO("StatusByte: %d", statusByte);
	if (statusByte & (0x01 << 8)) {
		return false;
	} else {
		return true;
	}
}

bool motorController::positionReached(unsigned char adressDevice) {
	unsigned int statusByte = readInt16AddressParker(adressDevice, 1000, 3);
	ROS_INFO("StatusByte: %d", statusByte);
	if (statusByte & (0x01 << 9)) {
		return true;
	} else {
		return false;
	}
}

bool motorController::axisEnergized(unsigned char adressDevice) {
	unsigned int statusByte = readInt16AddressParker(adressDevice, 1000, 3);
	ROS_INFO("StatusByte: %d", statusByte);
	if (statusByte & (0x01 << 10)) {
		return false;
	} else {
		return true;
	}
}

controllerStatus motorController::getStatusController(
		unsigned char adressDevice) {
	controllerStatus status;
	unsigned int statusByte = readInt16AddressParker(adressDevice, 1000, 3);
	if (statusByte == 0) {
		ROS_INFO("Error while requesting status on device: %d", adressDevice);
		status.failed = true;
		return status;
	} else {
		status.failed = false;
	}

	if (statusByte & (0x01 << 8)) {
		status.error = false;
	} else {
		status.error = true;
	}

	if (statusByte & (0x01 << 10)) {
		status.energized = false;
	} else {
		status.energized = true;
	}

	if (statusByte & (0x01 << 11)) {
		status.positionReached = true;
	} else {
		status.positionReached = false;
	}
	return status;
}

bool motorController::energizeAxis(unsigned char adressDevice, bool trigger) {
	if(adressDevice == 1){
		if (!controllerConnected_1_) {
		ROS_ERROR("Controller 1 not connected");
		return false;
		}
	}
	if(adressDevice == 2){
		if (!controllerConnected_2_) {
		ROS_ERROR("Controller 2 not connected");
		return false;
		}
	}
	if(adressDevice == 3){
		if (!controllerConnected_3_) {
		ROS_ERROR("Controller 3 not connected");
		return false;
		}
	}
	unsigned int control = 0x00;
	if (trigger) {
		controlWord1 = controlWord1 | 0x01;
	} else {
		controlWord1 = controlWord1 & (~0x01);
	}
	control = controlWord1;
	if (writeInt16AddressParker(adressDevice, 1100, 3, control)) {
		return true;
	} else {
		return false;
	}
}

bool motorController::startSetting(unsigned char adressDevice,
		unsigned int settingAddress) {

	// First set start to zero (rising edge necessary)
	unsigned int control = 0x00;
	control = control | (settingAddress << 8); 	// Set address
	control = control | (0x4002); 				// Set no-stop flags
	control = control | controlWord1;
	if (!writeInt16AddressParker(adressDevice, 1100, 3, control)) {
		return false;
	}
	// Set start flag
	control = control | (0x2000);				// Set start-flag
	// Set start flag
	if (writeInt16AddressParker(adressDevice, 1100, 3, control)) {
		return true;
	} else {
		return false;
	}
}

//bool motorController::initialize(unsigned char addressDevice, unsigned int line,
//		float goal, float vel, unsigned int acc, unsigned int dec)

bool motorController::setSetting(unsigned char addressDevice, unsigned int line,
		float goal, float vel, unsigned int acc, unsigned int dec) {

	if (!writeFloat32AddressParker(addressDevice, 1901, line, goal)) {
		return false;
	}

	if (!writeFloat32AddressParker(addressDevice, 1902, line, vel)) {
		return false;
	}

	// PSB's
	if (!writeInt16AddressParker(addressDevice, 1904, line, 0x0032)) {
		return false;
	}

	// Mode. 1 = absolute position
	if (!writeInt16AddressParker(addressDevice, 1905, line, 0x0001)) {
		return false;
	}

	if (!writeInt32AddressParker(addressDevice, 1906, line, acc)) {
		return false;
	}

	if (!writeInt32AddressParker(addressDevice, 1907, line, dec)) {
		return false;
	}

	if (!writeInt32AddressParker(addressDevice, 1908, line, 10000)) {
		return false;
	}

	if (!writeInt16AddressParker(addressDevice, 1904, line, 0x0032)) {
		return false;
	}else
	return true;
}

bool motorController::manual(unsigned char deviceAddress,
		unsigned char direction) {
	unsigned int control = 0x00;
	if (direction == 1) {
		control = 0x4006 | controlWord1;
	} else {
		control = 0x400A | controlWord1;
	}
	if (writeInt16AddressParker(deviceAddress, 1100, 3, control)) {
		return true;
	} else {
		return false;
	}
}

bool motorController::stop(unsigned char deviceAddress) {
	unsigned int control = controlWord1;

	if (writeInt16AddressParker(deviceAddress, 1100, 3, control)) {
		return true;
	} else {
		return false;
	}
}

bool motorController::sendHoming() {
	bool error = true;;
	// Start setting with address zero
	if (controllerConnected_1_) {
		if (!startSetting(1, 0)) {
			ROS_ERROR("Error while sending homing command for x-axis");
			error = false;
		}
	} else {
		ROS_INFO("Homing: Controller 1 not connected");
	}

	if (controllerConnected_2_) {
		if (!startSetting(2, 0)) {
			ROS_ERROR("Error while sending homing command for x-axis");
			error = false;
		}
	} else {
		ROS_INFO("Homing: Controller 2 not connected");
	}

	if (controllerConnected_3_) {
		if (!startSetting(3, 0)) {
			ROS_ERROR("Error while sending homing command for x-axis");
			error = false;
		}
	} else {
		ROS_INFO("Homing: Controller 3 not connected");
	}
	return error;
}

int motorController::gotoCoord(float x, float y, float z) {
	ROS_INFO("Sending coordinate command...");
	int error = NO_ERROR;

	if (controllerConnected_1_) {
		// Setting lines and collumns in the postion-set
		if (!setSetting(1, 1, x, 100.0, 600, 800)) {
			error = X_ERROR;
		}
	} else {
		ROS_INFO("Move: Controller 1 not connected");
	}

	if (controllerConnected_2_) {
		if (!setSetting(2, 1, y, 200, 600, 800)) {
			error = Y_ERROR;
		}
	} else {
		ROS_INFO("Move: Controller 2 not connected");
	}

	if (controllerConnected_3_) {
		if (!setSetting(3, 1, z, 100, 300, 500)) {
			error = Z_ERROR;
		}
	} else {
		ROS_INFO("Move: Controller 3 not connected");
	}

	// Starting the positon-set
	if (controllerConnected_1_ && error != X_ERROR) {
		if (!startSetting(1, 1)) {
			error = X_ERROR;
		}
	} else {
		ROS_INFO("Move start: Controller 1 not connected");
	}

	if (controllerConnected_2_ && error != Y_ERROR) {
		if (!startSetting(2, 1)) {
			error = Y_ERROR;
		}
	} else {
		ROS_INFO("Move start: Controller 2 not connected");
	}

	if (controllerConnected_3_ && error != Z_ERROR) {
		if (!startSetting(3, 1)) {
			error = Z_ERROR;
		}
	} else {
		ROS_INFO("Move start: Controller 3 not connected");
	}

	return error;
}
