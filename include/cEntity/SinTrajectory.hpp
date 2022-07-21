#ifndef SINTRAJECTORY_HPP_
#define SINTRAJECTORY_HPP_

// Project Header files
#include "Trajectory.hpp"

class SinTrajectory : public Trajectory {

public:

	/**
	* @brief Constructor of the SinTrajectory object
	* The constructor takes as argument the list of boolean values specifying on which component the sinusoidal trajectory should be defined
	* @param x_axis: boolean value specifying if the sinusoidal is to be generated on the x axis
	* @param y_axis: boolean value specifying if the sinusoidal is to be generated on the y axis
	* @param z_axis: boolean value specifying if the sinusoidal is to be generated on the z axis
	*
	*/
	//SinTrajectory(const bool& x_axis = false, const bool& y_axis = false, const bool& z_axis = true);

	/**
	* @brief Constructor of the Trajectory object with filename string parameter
	* @param fname: the filename string parameter of the configuration file
	*
	*/
	SinTrajectory(const std::string& fname);

	/**
	* @brief Destroyer of the Trajectory object
	*
	*/
	~SinTrajectory();

	/**
	* @brief Set function
	* Set the magnitude of the sinusoidal function
	* @param mag: the input magnitude
	*/
	inline void setMagnitude(const float& mag) { this->A = mag; }

	/**
	* @brief Set function
	* Set the magnitude of the sinusoidal function
	* @param mag: the input magnitude
	*/
	inline float getMagnitude() { return this->A; ; }

	/**
	* @brief Set function
	* Set the frequency of the sinusoidal function
	* @param freq: the input frequency
	*/
	inline void setFrequency(const float& freq) { this->f = freq; }

	/**
	* @brief Get function
	* Get the frequency of the sinusoidal function
	* @return the input frequency
	*/
	inline float getFrequency() { return this->f; }

	/**
	* @brief Set function
	* Set the shift of the sinusoidal function
	* @param s: the input shift
	*/
	inline void setShift(const float& s) { this->shift = s; }

	/**
	* @brief Get function
	* Get the length of the trajectory
	* @return the length of the trajectory
	*/
	inline float getLength() { return this->A * 2.0; }

	/**
	* @brief Set function 
	* Set the length of the trajectory
	* @param the length of the trajectory
	*/
	inline void setLength(const float& l) { this->A = l * 0.5; }

	/**
	* @brief Evaluation function
	* Evaluate the function
	*/
	Eigen::Vector6f eval();

	/**
	* @brief Load function
	* Load the parameters of the class from the given input file
	*/
	void loadDataFromConfigFile();

private:


	bool xyzMasks[SPACE_DIM];			//!< boolean mask specifying on which components the sinusoidal should be generated

	float A;							//!< Magnitude of the sinusoid
	float f;							//!< Frequency of the sinusoid
	float shift;						//!< Shift of the sinusoid

};


#endif // SINTRAJECTORY_HPP_