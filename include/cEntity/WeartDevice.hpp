#ifndef WEART_DEVICE_HPP_
#define WEART_DEVICE_HPP_

// Project Header files
#include "HapticInterface.hpp"

class WeartDevice : public HapticInterface {

public:

	/**
	* @brief Default constructor of the WeartDevice class
	*/
	WeartDevice();

	/**
	* @brief Default constructor of the WeartDevice class
	*/
	WeartDevice(const std::string& name_);

	/**
	* @brief Default destroyer of the WeartDevice class
	*/
	~WeartDevice();

	void loadParamsFromConfigFile(const std::string& comment, const std::string& value);

	/**
	* @brief Set function
	* Set the intensity of the vibration feedback
	* @param v: the intensity of the vibration feedback
	*/
	inline void setVibrationVolume(const float& v) { this->vibrationVolume = v; }

	/**
	* @brief Get function
	* Get the intensity of the vibration feedback
	* @return the intensity of the vibration feedback
	*/
	inline float getVibrationVolume() { return this->vibrationVolume; }

private:

	float vibrationVolume;		//!< Intensity of the vibration feedback

};




#endif // WEART_DEVICE_HPP_