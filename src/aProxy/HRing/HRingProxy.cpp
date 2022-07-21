// Project Header files
#include "HRingProxy.hpp"
#include "Timer.hpp"

// System Header files
#include <stdio.h>
#include <windows.h>


/** Opens a handle to a serial port in Windows using CreateFile.
 * portName: The name of the port.
 *   Examples: "COM4", "USB#VID_1FFB&PID_0089&MI_04#6&3ad40bf600004#".
 * baudRate: The baud rate in bits per second.
 * Returns INVALID_HANDLE_VALUE if it fails.  Otherwise returns a handle to the port.
 */
HANDLE openPort(const char* portName, unsigned int baudRate)
{
	HANDLE port_;
	DCB commState;
	BOOL success;
	COMMTIMEOUTS timeouts;

	/* Open the serial port. */
	port_ = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (port_ == INVALID_HANDLE_VALUE)
	{
		switch (GetLastError())
		{
		case ERROR_ACCESS_DENIED:
			fprintf(stderr, "Error: Access denied.  Try closing all other programs that are using the device.\n");
			break;
		case ERROR_FILE_NOT_FOUND:
			fprintf(stderr, "Error: Serial port not found.  "
				"Make sure that \"%s\" is the right port name.  "
				"Try closing all programs using the device and unplugging the "
				"device, or try rebooting.\n", portName);
			break;
		default:
			fprintf(stderr, "Error: Unable to open serial port.  Error code 0x%lx.\n", GetLastError());
			break;
		}
		return INVALID_HANDLE_VALUE;
	}

	/* Set the timeouts. */
	success = GetCommTimeouts(port_, &timeouts);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to get comm timeouts.  Error code 0x%lx.\n", GetLastError());
		CloseHandle(port_);
		return INVALID_HANDLE_VALUE;
	}
	timeouts.ReadIntervalTimeout = 1000;
	timeouts.ReadTotalTimeoutConstant = 1000;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 1000;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	success = SetCommTimeouts(port_, &timeouts);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to set comm timeouts.  Error code 0x%lx.\n", GetLastError());
		CloseHandle(port_);
		return INVALID_HANDLE_VALUE;
	}

	/* Set the baud rate. */
	success = GetCommState(port_, &commState);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to get comm state.  Error code 0x%lx.\n", GetLastError());
		CloseHandle(port_);
		return INVALID_HANDLE_VALUE;
	}
	commState.BaudRate = baudRate;
	success = SetCommState(port_, &commState);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to set comm state.  Error code 0x%lx.\n", GetLastError());
		CloseHandle(port_);
		return INVALID_HANDLE_VALUE;
	}

	/* Flush out any bytes received from the device earlier. */
	success = FlushFileBuffers(port_);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to flush port buffers.  Error code 0x%lx.\n", GetLastError());
		CloseHandle(port_);
		return INVALID_HANDLE_VALUE;
	}

	return port_;
}

/** Implements the Maestro's Get Position serial command.
 * channel: Channel number from 0 to 23
 * position: A pointer to the returned position value (for a servo channel, the units are quarter-milliseconds)
 * Returns 1 on success, 0 on failure.
 * For more information on this command, see the "Serial Servo Commands"
 * section of the Maestro User's Guide: http://www.pololu.com/docs/0J40 */
BOOL maestroGetPosition(HANDLE port, unsigned char channel, unsigned short* position)
{
	unsigned char command[2];
	unsigned char response[2];
	BOOL success;
	DWORD bytesTransferred;

	// Compose the command.
	command[0] = 0x90;
	command[1] = channel;

	// Send the command to the device.
	success = WriteFile(port, command, sizeof(command), &bytesTransferred, NULL);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to write Get Position command to serial port.  Error code 0x%lx.\n", GetLastError());
		return 0;
	}
	if (sizeof(command) != bytesTransferred)
	{
		fprintf(stderr, "Error: Expected to write %d bytes but only wrote %ld.\n", (int)sizeof(command), bytesTransferred);
		return 0;
	}

	// Read the response from the device.
	success = ReadFile(port, response, sizeof(response), &bytesTransferred, NULL);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to read Get Position response from serial port.  Error code 0x%lx.\n", GetLastError());
		return 0;
	}
	if (sizeof(response) != bytesTransferred)
	{
		fprintf(stderr, "Error: Expected to read %d bytes but only read %ld (timeout). "
			"Make sure the Maestro's serial mode is USB Dual Port or USB Chained.\n", (int)sizeof(command), bytesTransferred);
		return 0;
	}

	// Convert the bytes received in to a position.
	*position = response[0] + 256 * response[1];

	return 1;
}



/** Implements the Maestro's Set Target serial command.
 * channel: Channel number from 0 to 23
 * target: The target value (for a servo channel, the units are quarter-milliseconds)
 * Returns 1 on success, 0 on failure.
 * Fore more information on this command, see the "Serial Servo Commands"
 * section of the Maestro User's Guide: http://www.pololu.com/docs/0J40 */
BOOL maestroSetTarget(HANDLE port_, unsigned char channel, unsigned short target)
{
	unsigned char command[4];
	DWORD bytesTransferred;
	BOOL success;

	// Compose the command.
	command[0] = 0x84;
	command[1] = channel;
	command[2] = target & 0x7F;
	command[3] = (target >> 7) & 0x7F;

	// Send the command to the device.
	success = WriteFile(port_, command, sizeof(command), &bytesTransferred, NULL);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to write Set Target command to serial port.  Error code 0x%lx.\n", GetLastError());
		return 0;
	}
	if (sizeof(command) != bytesTransferred)
	{
		fprintf(stderr, "Error: Expected to write %d bytes but only wrote %ld.\n", (int)sizeof(command), bytesTransferred);
		return 0;
	}

	return 1;
}

/**
* @brief Default contructor of HRingProxy class
*
*/
HRingProxy::HRingProxy() : HapticProxy() {


}


/* @brief Default destroyer of HRingProxy class
*
*/
HRingProxy::~HRingProxy(){}

/**
* @brief Copy constructor of the HRingProxy class
* @param hp the HRingProxy
*/
HRingProxy::HRingProxy(HRingProxy& hp) {}

/**
* @brief Default init function
*/
void HRingProxy::init() {

	this->hapticDOF = 2;
	this->hapticState.force.setZero(this->hapticDOF);
	this->availability(true);
	this->setRunning(true);

	// Set Maestro motors channels (hard-coded here but can be generalized and set from file later)
	this->leftChannelNum = 2;
	this->rightChannelNum = 5;

}

/**
* @brief Default run function
*/
void HRingProxy::run() {

	// Time variables
	double tictoc, dt, rate, tic, toc, tac, Ts;
	Timer clock;
	clock.setRate(20.0);
	rate = clock.getRate();
	Ts = 1.0 / rate;

	// Init variables
	char* portName;
	int baudRate;
	BOOL success;
	unsigned short target;

	// Wait for some seconds to show the task menu on screen correctly (TODO: find a smart way and remove this horror)
	clock.timeSleep(1.0);

	/* Choose the baud rate (bits per second).
	* If the Maestro's serial mode is USB Dual Port, this number does not matter. */
	portName = "COM4";  // Each double slash in this source code represents one slash in the actual name.
	baudRate = 9600;

	/* Open the Maestro's serial port. */
	HANDLE port_h = openPort(portName, baudRate);
	this->port = (void*)(&port_h);
	if (*((HANDLE*)this->port) == INVALID_HANDLE_VALUE) {
		std::cout << "Invalid Maestro port " << *((HANDLE*)this->port) << ". Unable to open the communication with HRing." << std::endl;
	}
	else {
		std::cout << "Connected through HANDLE variable port_h = " << *((HANDLE*)this->port) << std::endl;
	}

	// Send the haptic force on the device
	//this->sendForce(this->hapticState.force);
	//this->running = false;

	while (this->isRunning()) {

		// Measure starting time
		tic = clock.getCurTime();

		//----------------------------------------------------------------//
		// Do stuff here... 

		// Send the haptic force on the device
		float fx = this->hapticState.force(HRING_DIRS::X_DIR);
		float fz = this->hapticState.force(HRING_DIRS::Z_DIR);

		fx = (fx < MAX_FLOAT_FORCE_VAL) ? ((fx > -MAX_FLOAT_FORCE_VAL) ? fx : (-MAX_FLOAT_FORCE_VAL)) : MAX_FLOAT_FORCE_VAL;
		fz = (fz < MAX_FLOAT_FORCE_VAL) ? ((fz > -MAX_FLOAT_FORCE_VAL) ? fz : (-MAX_FLOAT_FORCE_VAL)) : MAX_FLOAT_FORCE_VAL;


		//std::cout << "[fx, fz] = [" << fx << ", " << fz << "] " << std::endl;

		if (std::fabs(fx) > std::fabs(fz)) {
			this->stretch(fx);
		}
		else if (std::fabs(fz) > std::fabs(fx)) {
			this->press(fz);
		}
		else {
			this->release();
		}
		//----------------------------------------------------------------//

		// Measure the ending time and the elapsed time
		toc = clock.getCurTime();
		tictoc = clock.elapsedTime(tic, toc);

		// Wait until Ts
		if (tictoc < Ts) {
			clock.timeSleep(Ts - tictoc);
		}

		// Measure the final time after sleep to check the actual rate of the thread
		tac = clock.getCurTime();
		dt = clock.elapsedTime(tic, tac);
		//std::cout << "[ArmandProxy] Running rate:" << (1.0 / dt) << std::endl;

	}

	// Reset haptic force
	this->hapticState.force.setZero(this->getHapticDOF());
	this->release();

	/* Close the serial port so other programs can use it.
	 * Alternatively, you can just terminate the process (return from main). */
	CloseHandle(*((HANDLE*)this->port));


}

/**
* @brief Default clear function
*/
void HRingProxy::clear() {

	// Set running on false
	this->setRunning(false);
}


/**
* @brief Press function
* Actuate the motors to achieve pressure on the finger (vertical force)
* @param val: the intensity of the pressure to be applied
*/
void HRingProxy::press(const float& val) {

	unsigned short target = (unsigned short)((val + MAX_FLOAT_FORCE_VAL) / (2.0 * MAX_FLOAT_FORCE_VAL) * (MAX_MOTOR_POSITION - MIN_MOTOR_POSITION) + MIN_MOTOR_POSITION);
	target *= 4;
	BOOL successL = maestroSetTarget(*((HANDLE*)this->port), this->leftChannelNum, target);
	BOOL successR = maestroSetTarget(*((HANDLE*)this->port), this->rightChannelNum, target);
}

/**
* @brief Stretch function
* Actuate the motors to achieve lateral stretching on the finger (lateral force)
* @param val: the intensity of the stretching to be applied. Right direction positive
*/
void HRingProxy::stretch(const float& val) {

	unsigned short targetL, targetR;
	if (val > 0) {
		targetL = (unsigned short)((val + MAX_FLOAT_FORCE_VAL) / (2.0 * MAX_FLOAT_FORCE_VAL) * (MAX_MOTOR_POSITION - MIN_MOTOR_POSITION) + MIN_MOTOR_POSITION);
		targetR = (unsigned short)((-val + MAX_FLOAT_FORCE_VAL) / (2.0 * MAX_FLOAT_FORCE_VAL) * (MAX_MOTOR_POSITION - MIN_MOTOR_POSITION) + MIN_MOTOR_POSITION);
	}
	BOOL successL = maestroSetTarget(*((HANDLE*)this->port), this->leftChannelNum, targetL);
	BOOL successR = maestroSetTarget(*((HANDLE*)this->port), this->rightChannelNum, targetR);

}

/**
* @brief Release function
* Zeroes the motors commands
*/
void HRingProxy::release() {

	/* Set the target of channel 0. */
	BOOL successL = maestroSetTarget(*((HANDLE*)this->port), this->leftChannelNum, MIN_MOTOR_POSITION);
	BOOL successR = maestroSetTarget(*((HANDLE*)this->port), this->rightChannelNum, MIN_MOTOR_POSITION);

}
