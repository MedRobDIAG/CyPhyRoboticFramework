#ifndef HRINGPROXY_HPP_
#define HRINGPROXY_HPP_

// Project Header files
#include "HapticProxy.hpp"
#define MIN_MOTOR_POSITION 992.
#define MAX_MOTOR_POSITION 1496.
//#define MAX_MOTOR_POSITION 2000.
#define MAX_FLOAT_FORCE_VAL 1.0

enum HRING_DIRS { X_DIR, Z_DIR };


class HRingProxy : public HapticProxy {//HLInterface, 

public:

	/**
	* @brief Default contructor of HRingProxy class
	*
	*/
	HRingProxy();


	/* @brief Default destroyer of HRingProxy class
	*
	*/
	~HRingProxy();

	/**
	* @brief Copy constructor of the HRingProxy class
	* @param hp the HRingProxy
	*/
	HRingProxy(HRingProxy& hp);

	/**
	* @brief Default init function
	*/
	void init();

	/**
	* @brief Default run function
	*/
	void run();

	/**
	* @brief Default clear function
	*/
	void clear();

private:

	void* port;
	int leftChannelNum;
	int rightChannelNum;

	/**
	* @brief Press function
	* Actuate the motors to achieve pressure on the finger (vertical force)
	* @param val: the intensity of the pressure to be applied
	*/
	void press(const float& val);

	/**
	* @brief Stretch function
	* Actuate the motors to achieve lateral stretching on the finger (lateral force)
	* @param val: the intensity of the stretching to be applied. Right direction positive
	*/
	void stretch(const float& val);

	/**
	* @brief Release function
	* Zeroes the motors commands
	*/
	void release();
};

#endif // HRINGPROXY_HPP_