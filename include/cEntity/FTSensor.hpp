#ifndef FTSENSOR_HPP_
#define FTSENSOR_HPP_

// Project Header files
#include "EndEffector.hpp"

// Eigen Header files
#include <Eigen\Dense>

// Twist dimension
#ifndef TWIST_DIM
#define TWIST_DIM 6
#endif // TWIST_DIM


class FTSensor : public EndEffector {

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	* @brief Default constructor of the FTSensor class
	*/
	FTSensor() : EndEffector() {}

	/**
	* @brief Default constructor of the FTSensor class
	*/
	FTSensor(const std::string& name_) : EndEffector(name_) {}

	/**
	* @brief Default destroyer of the FTSensor class
	*/
	~FTSensor() {}

	/**
	* @brief Init function
	* Check the input pair of comment+value to assign the corresponding parameter
	* The function assumes that configFile has been previously set
	* @param comment: the string specifying the title comment of parameter
	* @param value: the string specifying the value of the parameter
	*/
	//void loadParamsFromConfigFile(const std::string& comment, const std::string& value);

	/**
	* @brief Set function
	* Set the 6D measurement wrench vector
	* @param meas: 6D measurement wrench vector
	*/
	inline void setWrench(const Eigen::Vector6d& meas) { this->wrench = meas; }

	/**
	* @brief Get function
	* Get the 6D measurement wrench vector
	* @return 6D measurement wrench vector
	*/
	inline Eigen::Vector6d getWrench() { return this->wrench ; }

private:

	Eigen::Vector6d wrench;				//!< 6D Vector containing the current force/torque measurement of the F/T sensor

};


#endif // FTSENSOR_HPP_
