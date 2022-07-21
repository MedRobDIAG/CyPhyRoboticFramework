#ifndef HAPTIC_PROXY_HPP_
#define HAPTIC_PROXY_HPP_

// Project Header files
#include "ExtSystemProxy.hpp"
#include "utils.hpp"

struct HapticState {

	Eigen::VectorXf force;		// force vector of the kinestetic feedback
	float vibroIntensity;		// intensity of the vibrotactile feedback -- Please note that, differently from what can be expected, the armband still uses the force variable (kinestetic) feedback rendering, rather than this variable, as this separation has been planned later. TODO: Consider to re-organize and associate variables to proper devices accordingly.

};

class HapticProxy : public ExtSystemProxy {

public:

	/**
	* @brief Default contructor of HapticProxy class
	*
	*/
	HapticProxy() {}

	/**
	* @brief Default destroyer of HapticProxy class
	*
	*/
	~HapticProxy() {}

	/**
	* @brief Set function
	* Set the number of degrees of freedom of the haptic feedback
	* @param num: the number of degrees of freedom of the haptic feedback
	*/
	inline void setHapticDOF(const int& num) { this->hapticDOF = num; }

	/**
	* @brief Get function
	* Get the number of degrees of freedom of the haptic feedback
	* @return the number of degrees of freedom of the haptic feedback
	*/
	inline int getHapticDOF() { return this->hapticDOF ; }

	/**
	* @brief Set function
	* Set the feedback haptic force on the Geomagic device
	* @param f: the feedback force to be set
	*/
	inline void setHapticForce(const Eigen::VectorXf& f) { this->hapticState.force = f; }

	/**
	* @brief Get function
	* Get the feedback haptic force on the Geomagic device
	* @return the feedback force
	*/
	inline Eigen::VectorXf getHapticForce() { return this->hapticState.force; }

	/**
	* @brief Set function
	* Set the vibrotactile feedback
	* @param f: the vibrotactile feedback
	*/
	inline void setVibrotactileIntensity(const float& v) { this->hapticState.vibroIntensity = v; }

	/**
	* @brief Get function
	* Get the vibrotactile feedback
	* @return the vibrotactile feedback
	*/
	inline float getVibrotactileIntensity() { return this->hapticState.vibroIntensity; }

	/**
	* @brief Get function
	* Get the state of the haptic device
	* @return the state of the haptic device
	*/
	inline HapticState getHapticState() { return this->hapticState; }
	
	/**
	* @brief Set function
	* Set the state of the haptic device
	* @param hs: the state of the haptic device
	*/
	inline void setHapticState(const HapticState& hs) { this->hapticState = hs; }


protected:

	int hapticDOF;						//!< Number of degrees of freedom of the haptic feedback

	HapticState hapticState;			//!< State of the haptic device (force vector + rendering flag)
};




#endif // HAPTIC_PROXY_HPP_
