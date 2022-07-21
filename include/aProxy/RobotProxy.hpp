#ifndef ROBOT_PROXY_HPP_
#define ROBOT_PROXY_HPP_

// Project Header files
#include "ExtSystemProxy.hpp"
#include "utils.hpp"


struct RobotState {

	double stamp;
	Eigen::VectorXf msrJointPosition;
	Eigen::VectorXf msrJointTorque;
	Eigen::VectorXf extEstJointTorque;
	Eigen::Vector6f extEstCartForce;

};

class RobotProxy : public ExtSystemProxy {

public:

	/**
	* @brief Default contructor of RobotProxy class
	*
	*/
	RobotProxy() {}

	/**
	* @brief Default destroyer of RobotProxy class
	*
	*/
	~RobotProxy() {}

	/**
	* @brief Set function
	* Set the Degrees-of-Freedom of the robot system
	* @param num: the Degrees-of-Freedom of the robot system
	*/
	inline void setJointDOFs(const int& num) { this->jointDOFs = num; }

	/**
	* @brief Set function (virtual)
	* Set the current vector of measured joint position
	* @param q: the current vector of measured joint position
	*/
	virtual void setMsrJointPosition(const Eigen::VectorXf& q) = 0;

	/**
	* @brief Get function (virtual)
	* Get the current vector of measured joint position
	* @return the current vector of measured joint position
	*/
	virtual Eigen::VectorXf getMsrJointPosition() = 0;

	/**
	* @brief Set function (virtual)
	* Set the current vector of measured joint torque
	* @param tau: the current vector of measured joint torque
	*/
	virtual void setMsrJointTorque(const Eigen::VectorXf& tau) = 0;

	/**
	* @brief Get function (virtual)
	* Get the current vector of measured joint torque
	* @return the current vector of measured joint torque
	*/
	virtual Eigen::VectorXf getMsrJointTorque() = 0;

	/**
	* @brief Set function (virtual)
	* Set the current vector of estimated external joint torque
	* @param tau: the current vector of estimated external joint torque
	*/
	virtual void setEstExtJointTorque(const Eigen::VectorXf& tau) = 0;

	/**
	* @brief Get function (virtual)
	* Get the current vector of estimated external joint torque
	* @return the current vector of estimated external joint torque
	*/
	virtual Eigen::VectorXf getEstExtJointTorque() = 0;

	/**
	* @brief Set function (virtual)
	* Set the current vector of estimated external Cartesian force
	* @param q: the current vector of estimated external Cartesian force
	*/
	virtual void setEstExtCartForce(const Eigen::Vector6f& f) = 0;

	/**
	* @brief Get function (virtual)
	* Get the current vector of estimated external Cartesian force
	* @return the current vector of estimated external Cartesian force
	*/
	virtual Eigen::Vector6f getEstExtCartForce() = 0;

	/**
	*@brief Get function
	* Get the full state of the robot from the proxy
	* @return the full state of the robot from the proxy
	*/
	/*inline RobotState getRobotState() {
		RobotState rs;
		rs.msrJointPosition = this->getMsrJointPosition();
		rs.msrJointTorque = this->getMsrJointTorque();
		rs.extEstJointTorque = this->getEstExtJointTorque();
		rs.extEstCartForce = this->getEstExtCartForce();
		return rs;
	}//*/

	/**
	* @brief Set Robot commands (virtual)
	* Set the input command joint vector
	* @param qcmd: the input vector to be set
	*/
	virtual void setCommands(const Eigen::VectorXf& qcmd) = 0;

	/**
	* @brief Set Joint reference vector (virtual)
	* Set the joint reference vector for autonomous pose regolation 
	* @param qcmd: the joint reference vector
	*/
	virtual void setJointRefs(const Eigen::VectorXf& jref) = 0;
	
	/*
	* @brief Get function
	* Get the full robot state from the proxy
	* @return the RobotState structure
	*/
	virtual RobotState getRobotState() = 0;

protected:

	int jointDOFs;							//!< Degrees-of-Freedom of the robot system

};




#endif // ROBOT_PROXY_HPP_
