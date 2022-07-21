#ifndef TELEOP_CONTROL_STRATEGY_HPP_
#define TELEOP_CONTROL_STRATEGY_HPP_

// Project Header files
#include "ControlStrategy.hpp"
#include "Trajectory.hpp"
#include "Geomagic.hpp"
#include "FTSensor.hpp"

class TeleopControlStrategy : public ControlStrategy {

public:

	/*
	* @brief Constructor of ControlStrategy
	* @param r: the pointer to RobotInterface
	*/
	TeleopControlStrategy(RobotInterface* r = nullptr, Geomagic* g = nullptr, FTSensor* ft = nullptr) : ControlStrategy(r), hapticDev(g), ftsensor(ft) {}

	/*
	* @brief Destroyer of ControlStrategy
	*/
	~TeleopControlStrategy() {}

	/**
	* @brief Init function
	* Initialize the control strategy
	*/
	void init();

	/**
	* @brief Main function
	* Implements the control law associated to the ControlStrategy
	*/
	Eigen::VectorXf controlLaw();

	/**
	* @brief Set function
	* Set the FTSensor pointer 
	* @param ft: the FTSensor pointer
	*/
	inline void setFTSensor(FTSensor* ft) {
		this->ftsensor = ft;
	}

private:

	Geomagic* hapticDev;
	FTSensor* ftsensor;
};


#endif // TELEOP_CONTROL_STRATEGY_HPP_
