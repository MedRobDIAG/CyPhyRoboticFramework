#ifndef ARMBAND_HPP_
#define ARMBAND_HPP_

// Project Header files
#include "HapticInterface.hpp"

class Armband : public HapticInterface {

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	* @brief Default constructor of the Armband class
	*/
	Armband();

	/**
	* @brief Default constructor of the Armband class
	*/
	Armband(const std::string& name_);

	/**
	* @brief Default destroyer of the Armband class
	*/
	~Armband() {}

	/**
	* @brief Set function
	* Set the 6D measurement wrench vector
	* @param meas: 6D measurement wrench vector
	*/
	inline void setErrorData(const Eigen::Vector3f& meas) { this->EF_Error = meas; }

	/**
	* @brief Get function
	* Get the Error data
	* @return the Error data
	*/
	inline Eigen::Vector3f getErrorData() { return this->EF_Error; }

	void loadParamsFromConfigFile(const std::string& comment, const std::string& value);


private:

	Eigen::Vector3f EF_Error;

};

#endif // ARMBAND_HPP_
