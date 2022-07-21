#ifndef TELEOPPROXY_HPP_
#define TELEOPPROXY_HPP_

// Project Header files
#include "ExtSystemProxy.hpp"
#include "utils.hpp"

class TeleopState {

public:

	TeleopState() {}
	~TeleopState() {}


	/**
	* @brief Set function
	* Set the number of buttons available on the device
	* @param num: the number of buttons available on the device
	*/
	inline void setButtonsNum(const int& num) { this->buttonsNum = num; }

	/**
	* @brief Get function
	* Get the number of buttons available on the device
	* @return the number of buttons available on the device
	*/
	inline int getButtonsNum() { return this->buttonsNum; }

	/**
	* @brief Set function
	* Set the 3D vector of the hip position of the device
	* @param p: the 3D vector of the hip position of the device
	*/
	inline void setHIPPosition(const Eigen::Vector3f& p) { this->hipPosition = p; }

	/**
	* @brief Set function
	* Set the 3D vector of the hip orientation of the device
	* @param p: the 3D vector of the hip orientation of the device
	*/
	inline void setHIPGimbalAngles(const Eigen::Vector3f& ga) { this->hipGimbalAngles = ga; }

	/**
	* @brief Set function
	* Set the 6D vector of the hip velocity of the device
	* @param v: the 6D vector of the hip velocity of the device
	*/
	inline void setHIPVelocity(const Eigen::Vector6f& v) { this->hipVelocity = v; }

	/**
	* @brief Set function
	* Set the dynamic array with the state (on/off) of the buttons placed on the device
	* @param state: the dynamic array with the state (on/off) of the buttons placed on the device
	*/
	inline void setButtonState(const bool* state) { for (int i = 0; i < buttonsNum; i++) this->buttonState[i] = state[i]; }

	/**
	* @brief Get function
	* Get the 3D vector of the hip position of the device
	* @return the 3D vector of the hip position of the device
	*/
	inline Eigen::Vector3f getHIPPosition() { return this->hipPosition; }

	/**
	* @brief Get function
	* Get the 3D vector of the hip orientation of the device
	* @return the 3D vector of the hip orientation of the device
	*/
	inline Eigen::Vector3f getHIPGimbalAngles() { return this->hipGimbalAngles; }

	/**
	* @brief Get function
	* Get the 6D velocity of the HIP
	* @return the 6D velocity of the HIP
	*/
	inline Eigen::Vector6f getHIPVelocity() { return this->hipVelocity; }

	/**
	* @brief Get function
	* Get the dynamic array with the state (on/off) of the buttons placed on the device
	* @return the dynamic array with the state (on/off) of the buttons placed on the device
	*/
	inline bool* getButtonState() { return this->buttonState; }


	// move to protected later
	int asyncTriggerState;				//!< State corresponding to asyncronous triggered events (e.g., stylus buttons)
	int buttonsNum;						//!< Number of buttons available on the device
	bool* buttonState;					//!< Array with the state of the buttons placed on the device
	Eigen::Vector3f hipPosition;		//!< Vector of the 3D position of the HIP
	Eigen::Vector3f hipGimbalAngles;	//!< Vector of the 3D orientation of the HIP
	Eigen::Vector6f hipVelocity;		//!< Vector of the 6D velocity of the HIP

protected:

};



class TeleopProxy : public ExtSystemProxy {

public:

	/**
	* @brief Default contructor of TeleopProxy class
	*
	*/
	TeleopProxy() {}

	/**
	* @brief Default destroyer of TeleopProxy class
	*
	*/
	~TeleopProxy() {}

	/**
	* @brief Set function
	* Set the number of degrees of freedom of the kinematic structure of the device
	* @param num: the number of degrees of freedom of the kinematic structure of the device
	*/
	inline void setJointDOF(const int& num) { this->jointDOF = num; }

	/**
	* @brief Get function
	* Get the TeleopStatus structure from the current class
	* @return the TeleopStatus structure
	*/
	//virtual void* getTeleopState() = 0;
	//virtual TeleopState* getTeleopState() = 0;
	inline TeleopState* getTeleopState() { return this->state; }

	/**
	* @brief Get function
	* Get the TeleopStatus structure from the current class
	* @param ts: the TeleopStatus structure
	*/
	inline void setTeleopStatus(TeleopState* ts) { this->state = ts; }

	/**
	* @brief Set function
	* Set the number of buttons available on the device
	* @param num: the number of buttons available on the device
	*/
	inline void setStateButtonsNum(const int& num) { this->state->setButtonsNum(num); }

	/**
	* @brief Get function
	* Get the number of buttons available on the device
	* @return the number of buttons available on the device
	*/
	inline int getStateButtonsNum() { return this->state->getButtonsNum(); }

	/**
	* @brief Set function
	* Set the 3D vector of the hip position of the device
	* @param p: the 3D vector of the hip position of the device
	*/
	inline void setStateHIPPosition(const Eigen::Vector3f& p) { this->state->setHIPPosition(p); }

	/**
	* @brief Set function
	* Set the 3D vector of the hip orientation of the device
	* @param p: the 3D vector of the hip orientation of the device
	*/
	inline void setStateHIPGimbalAngles(const Eigen::Vector3f& ga) { this->state->setHIPGimbalAngles(ga); }

	/**
	* @brief Set function
	* Set the 6D vector of the hip velocity of the device
	* @param v: the 6D vector of the hip velocity of the device
	*/
	inline void setStateHIPVelocity(const Eigen::Vector6f& v) { this->state->setHIPVelocity(v); }

	/**
	* @brief Get function
	* Get the 3D vector of the hip position of the device
	* @return the 3D vector of the hip position of the device
	*/
	inline Eigen::Vector3f getStateHIPPosition() { return this->state->getHIPPosition(); }

	/**
	* @brief Get function
	* Get the 3D vector of the hip orientation of the device
	* @return the 3D vector of the hip orientation of the device
	*/
	inline Eigen::Vector3f getStateHIPGimbalAngles() { return this->state->getHIPGimbalAngles(); }

	/**
	* @brief Get function
	* Get the 6D velocity of the HIP
	* @return the 6D velocity of the HIP
	*/
	inline Eigen::Vector6f getStateHIPVelocity() { return this->state->getHIPVelocity(); }

	/**
	* @brief Get function
	* Get the dynamic array with the state (on/off) of the buttons placed on the device
	* @return the dynamic array with the state (on/off) of the buttons placed on the device
	*/
	//inline bool* getButtonState() { return this->buttonState; }

protected:

	int jointDOF;						//!< Number of degrees of freedom of the kinematic structure of the device
	TeleopState* state;					//!< Structure with the full state of the teleoperation device

};

#endif //TELEOPPROXY_HPP_
