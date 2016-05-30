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

//!
//! \brief The motorController class holds all low level functions to communicate with parker devices
//! This class implements the virtual functions of the controller_interface
//!
class motorController: public motor_controller::ControllerInterface {
public:
	motorController();
	~motorController();

    //!
    //! \brief ReqParker sends a read or write request to the parker device
    //! \param rw  0xAD for read, rw = 0xCF for write
    //! \param data data storage
    //! \param length length of data
    //! \param adress address of device
    //!
	void ReqParker(unsigned char rw, unsigned char* data, unsigned char length,
			unsigned char adress);

    //!
    //! \brief readInt16AddressParker reads an 16 bit integer from parker device
    //! \param addressDevice address of device
    //! \param adressReg register address
    //! \param subadress sub-register address
    //! \return integer which was read
    //!
	unsigned int readInt16AddressParker(unsigned char addressDevice,
			unsigned int adressReg, unsigned char subadress);

    //!
    //! \brief readFloat32AddressParker reads an 32 bit float from parker device
    //! \param addressDevice address of device
    //! \param adressReg register address
    //! \param subadress sub-register address
    //! \return read float
    //!
	float readFloat32AddressParker(unsigned char addressDevice,
				unsigned int adressReg, unsigned char subadress);

    //!
    //! \brief writeInt16AddressParker writes a 16 bit integer to the parker device
    //! \param addressDevice address of the device
    //! \param adressReg register address
    //! \param subadress sub-register address
    //! \param data data to transmit
    //! \return true if successfull
    //!
	bool writeInt16AddressParker(unsigned char addressDevice,
			unsigned int adressReg, unsigned char subadress, unsigned int data);

    //!
    //! \brief writeInt32AddressParker writes a 32 bit integer to the parker device
    //! \param addressDevice address of the device
    //! \param adressReg register address
    //! \param subadress sub-register address
    //! \param data data to transmit
    //! \return true if successfull
    //!
	bool writeInt32AddressParker(unsigned char addressDevice,
			unsigned int adressReg, unsigned char subadress, unsigned int data);

    //!
    //! \brief writeFloat32AddressParker writes a 32 bit float to the parker device
    //! \param addressDevice address of the device
    //! \param adressReg register address
    //! \param subadress sub-register address
    //! \param data data to transmit
    //! \return true if successfull
    //!
	bool writeFloat32AddressParker(unsigned char addressDevice,
			unsigned int adressReg, unsigned char subadress, float data);

    //!
    //! \brief checkForACK check if last transmition was acknowledged
    //! \return true if ACK was received
    //!
	bool checkForACK();

    // Interface functions //////////////////////////////////////////////

    //!
    //! \brief sendHoming sends homing command to parker device
    //! \return true if successfull
    //!
	bool sendHoming();

    //!
    //! \brief searchForDevices searches available devices and connects to them
    //!
	void searchForDevices();

    //!
    //! \brief connectToBus connects to serial bus
    //!
	void connectToBus();

    //!
    //! \brief checkError checks wether an error exists on specified parker device
    //! \param adressDevice address of parker device
    //! \return true if an error exists
    //!
	bool checkError(unsigned char adressDevice);

    //!
    //! \brief isConnected checks if specified device is connected
    //! \param device device name
    //! \return true if connected
    //!
    bool isConnected(enum pap_common::MOTOR device);

    //!
    //! \brief disconnect disconnects from specified device
    //! \param device device name
    //! \return true if disconnected successfully
    //!
    bool disconnect(enum pap_common::MOTOR device);

    //!
    //! \brief getStatusController gets status message from parker device
    //! \param adressDevice device address
    //! \return device status message
    //!
	controllerStatus getStatusController(unsigned char adressDevice);

    //!
    //! \brief getFullStatusController gets full status of controller (with position data)
    //! \param addressDevice device address
    //! \return full status message
    //!
    controllerStatus getFullStatusController(pap_common::MOTOR addressDevice);

    //!
    //! \brief positionReached checks if position of axis controller is reached
    //! \param adressDevice device address
    //! \return true if reached
    //!
	bool positionReached(unsigned char adressDevice);

    //!
    //! \brief getPosition gets position of controller in mm
    //! \param adressDevice device address
    //! \return position
    //!
	float getPosition(unsigned char adressDevice);

    //!
    //! \brief axisEnergized checks if axis controller is energized
    //! \param adressDevice device address
    //! \return true if energized
    //!
	bool axisEnergized(unsigned char adressDevice);

    //!
    //! \brief energizeAxis energize or deenergize axis controller
    //! \param adressDevice device address
    //! \param trigger if true axis is energized
    //! \return true if successfull
    //!
    bool energizeAxis(pap_common::MOTOR adressDevice, bool trigger);

    //!
    //! \brief setSetting specifies and sends a movement profile to parker device
    //! \param addressDevice device address
    //! \param line slot of profile
    //! \param goal goal position
    //! \param vel velocity mm/s
    //! \param acc acceleration mm/s²
    //! \param dec decceleration mm/s²
    //! \return true if successfull
    //!
	bool setSetting(unsigned char addressDevice, unsigned int line, float goal,
			float vel, unsigned int acc, unsigned int dec);

    //!
    //! \brief startSetting starts movement profile (setting)
    //! \param adressDevice device address
    //! \param settingAddresss slot of setting
    //! \return true if successfull
    //!
	bool startSetting(unsigned char adressDevice, unsigned int settingAddress);

    //!
    //! \brief manual manual control
    //! \param deviceAddress device address
    //! \param direction forward = 1 backward = 0
    //! \return true if successfull
    //!
    bool manual(pap_common::MOTOR deviceAddress, unsigned char direction);

    //!
    //! \brief gotoCoord drives with all three axis controller to specified position
    //! \param x x-position
    //! \param y y-position
    //! \param z z-position
    //! \param velX x-velocity
    //! \param velY y-velocity
    //! \param velZ z-velocity
    //! \return error code (ERROR_CODES)
    //!
	int  gotoCoord(float x, float y, float z, float velX = 50.0, float velY = 300.0, float velZ = 100.0 );

    //!
    //! \brief stop stops motion of specified device
    //! \param deviceAddress device address
    //! \return true if successfull
    //!
    bool stop(pap_common::MOTOR deviceAddress);


	bool controllerConnected_1_,controllerConnected_2_,controllerConnected_3_;
private:
	unsigned int UpdateCRC16(unsigned int crc, unsigned char wert);
	unsigned char buffer[100];
	unsigned int controlWord1;
    std::unique_ptr<serial::Serial> serialPort_;

};

}
#endif // CONTROLLERCLASS_H
