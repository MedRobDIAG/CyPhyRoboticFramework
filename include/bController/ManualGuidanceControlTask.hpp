#ifndef MANUAL_GUIDANCE_CONTROL_TASK_HPP_
#define MANUAL_GUIDANCE_CONTROL_TASK_HPP_

// Project Header files
#include "ControlTask.hpp"

class ManualGuidanceControlTask : public ControlTask {

public:

	/**
	* @brief Constructor of ManualGuidanceControlTask object with RobotInterface pointer argument
	* @param tc: the task class
	* @param td: the task dimension
	* @param ri: the RobotInterface pointer argument
	*
	*/
	ManualGuidanceControlTask(const int& tc, const int& td, RobotInterface* ri = nullptr);

	/**
	* @brief Default destroyer of AutonomousControlTask object
	*
	*/
	~ManualGuidanceControlTask();

	/**
	* @brief Update function
	* Update the feed forward input vector
	*/
	void updateFeedForwardInput();

	/**
	* @brief Update function
	* Update the task vector
	*/
	void updateTaskVector();

	/**
	* @brief Update function
	* Update the task reference vector
	*/
	void updateTaskReferenceVector();

	/**
	* @brief Update function
	* Update the task Jacobian matrix
	*/
	void updateTaskJacobianMatrix();

	/**
	* @brief Set function
	* Set the Residual threshold for the manual guidance control task
	* @param rt: the residual threshold
	*/
	inline void setResThreshold(const float& rt) { this->resThresh = rt; }

	/**
	* @brief Set function
	* Set the flag establishing if the low-pass filter must be applied to the residual signal or not
	* @param the filter flag
	*/
	inline void applyLPFilter(const bool& filt) { this->withLPFilter = filt; }

	/**
	* @brief Set function
	* Set the sample time for the low-pass filter
	* @param dt = the sample time
	*/
	inline void setTc(const float& dt) { this->sampleTime = dt; }

	/**
	* @brief Set function
	* Set the cutoff frequency for the low-pass filter
	* @param f = the sample time
	*/
	inline void setCutF(const float& f) { this->fcut = f; }

private:

	Eigen::VectorXf residual;		//!< Residual signal
	float resThresh;				//!< Residual threshold 

	bool withLPFilter;				//!< If residual signal has to be low-pass filtered
	float sampleTime;				//!< Sample time for the low-pass filter
	float fcut;						//!< Cut-off frequency of the low-pass filter
};


#endif // MANUAL_GUIDANCE_CONTROL_TASK_HPP_
