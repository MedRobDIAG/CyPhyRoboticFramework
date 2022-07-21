#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

// Standard Header files
#include <vector>
#include <string>

// Eigen Header files
#include <Eigen\Dense>
#include "utils.hpp"

namespace Eigen {
	typedef Eigen::Matrix<float, TWIST_DIM, 1> Vector6f;
	typedef Eigen::Matrix<double, TWIST_DIM, 1> Vector6d;
}

enum TRAJ_TYPE{ SINUSOIDAL, TRAJ_NUM };

class Trajectory {

public:

	/**
	* @brief Constructor of the Trajectory object
	*
	*/
	Trajectory() { this->t = 0.0; }

	/**
	* @brief Constructor of the Trajectory object with filename string parameter
	* @param fname: the filename string parameter of the configuration file
	*
	*/
	Trajectory(const std::string& fname) : filename(fname) { 
		this->t = 0.0; 
		this->accomplished = false;
	}//*/

	/**
	* @brief Destroyer of the Trajectory object
	*
	*/
	~Trajectory() {}

	/**
	* @param Set function
	* Set the sample time of the Trajectory object
	* @param dt: the sample time
	*/
	inline void setSampleTime(const float& dt_) { this->dt = dt_; }


	/**
	* @param Get function
	* Get the sample time of the Trajectory object
	* @return the sample time
	*/
	inline float getSampleTime() { return this->dt ; }

	/**
	* @brief Update function
	* Update the current evaluating time
	*/
	inline void updateTime() { this->t += this->dt; }

	/**
	* @brief Evaluation function
	* Evaluate the function 
	*/
	virtual Eigen::Vector6f eval() = 0;

	/**
	* @brief Get function
	* Get the trajectory time
	* @return the trajectory time
	*/
	inline float getTrajTime() { return this->t; }

	/**
	* @brief Check function
	* Check if the trajectory has been completed
	* @return true if the trajectory has been completed, false otherwise
	*/
	inline bool isTrajectoryDone() { return this->accomplished; }

	/**
	* @brief Set function
	* Set the trajectory as completed
	* 
	*/
	inline void stop() { this->accomplished = true; }

	/**
	* @brief Set function
	* Reset the trajectory as non completed
	*/
	inline void reset() { this->accomplished = false; this->t = 0.0; }

	/**
	* @brief Get function (virtual)
	* Get the length of the trajectory
	* @return the length of the trajectory
	*/
	virtual float getLength() = 0;

	/**
	* @brief Set function (virtual)
	* Set the length of the trajectory
	* @param the length of the trajectory
	*/
	virtual void setLength(const float& l) = 0;
	
	/**
	* @brief Get function (virtual)
	* Get the frequency of the trajectory
	* @return the frequency of the trajectory
	*/
	virtual float getFrequency() = 0;

	/**
	* @brief Set function (virtual)
	* Set the frequency of the trajectory
	* @param the frequency of the trajectory
	*/
	virtual void setFrequency(const float& fin) = 0;

	/**
	* @brief Load function
	* Load the parameters of the class from the given input file
	*/
	virtual void loadDataFromConfigFile() = 0;

protected:

	float t;							//!< Time variable
	float dt;							//!< Sample time
	bool accomplished;					//!< If the trajectory has been performed successfully
	std::string filename;				//!< Configuration filename
};


#endif // TRAJECTORY_HPP_
