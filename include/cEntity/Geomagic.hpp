#ifndef GEOMAGIC_HPP_
#define GEOMAGIC_HPP_

// Project Header files
#include "HapticInterface.hpp"

class Geomagic : public HapticInterface {

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	* @brief Default constructor of the Geomagic class
	*/
	Geomagic();

	/**
	* @brief Default constructor of the Geomagic class
	*/
	Geomagic(const std::string& name_);

	/**
	* @brief Default destroyer of the Geomagic class
	*/
	~Geomagic() {}

	/**
	* @brief Set function
	* Set the scale factor for the master commanded linear velocity on the slave system
	* @param vs the scale factor to be set
	*/
	inline void setLinVelScale(const float& vs) { this->lvelScale = vs; }

	/**
	* @brief Get function
	* Get the scale factor for the master commanded linear velocity on the slave system
	* @return the scale factor to be retrieved
	*/
	inline float getLinVelScale() { return this->lvelScale; }

	/**
	* @brief Set function
	* Set the scale factor for the master commanded angular velocity on the slave system
	* @param vs the scale factor to be set
	*/
	inline void setAngVelScale(const float& vs) { this->avelScale = vs; }

	/**
	* @brief Get function
	* Get the scale factor for the master commanded angular velocity on the slave system
	* @return the scale factor to be retrieved
	*/
	inline float getAngVelScale() { return this->avelScale; }

	/**
	* @brief Set function
	* Set the HIP velocity of the Geomagic
	* @param vel: the HIP velocity of the Geomagic
	*/
	inline void setHIPVel(const Eigen::Vector6f& vel) { this->hipVel = vel; }

	/**
	* @brief Get function
	* Get the HIP velocity of the Geomagic
	* @return the HIP velocity of the Geomagic
	*/
	inline Eigen::Vector6f getHIPVel() { return this->hipVel; }

	/**
	* @brief Init function
	* Check the input pair of comment+value to assign the corresponding parameter
	* The function assumes that configFile has been previously set
	* @param comment: the string specifying the title comment of parameter
	* @param value: the string specifying the value of the parameter
	*/
	void loadParamsFromConfigFile(const std::string& comment, const std::string& value);

	/**
	* @brief Get function
	* Get the state of the clutch button of the Geomagic device
	* @return true if the clucth is active, false otherwise
	*/
	inline bool isClutchActive() { return this->clutchState; }

	/**
	* @brief Set function
	* Set the state of the clutch button of the Geomagic device
	* @param state: true if the clucth is active, false otherwise
	*/
	inline void setClutchState(const bool& state) { this->clutchState = state; }

private:

	float lvelScale;				//!< Scale factor for the master commanded linear velocity on the slave system
	float avelScale;				//!< Scale factor for the master commanded angular velocity on the slave system
	Eigen::Vector6f hipVel;			//!< 6D velocity vector of the Haptic Interaction Point (HIP) of the Geomagic
	bool clutchState;				//!< Boolean variable storing the state of the clutch button
};





#endif // GEOMAGIC_HPP_
