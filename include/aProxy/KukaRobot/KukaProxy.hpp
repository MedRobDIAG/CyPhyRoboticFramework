#ifndef KUKA_PROXY_HPP_
#define KUKA_PROXY_HPP_

#define HAVE_STRUCT_TIMESPEC

// FRI Header files
#include <FastResearchInterface.h>
#include <TypeIRML.h>
#include <pthread.h>
#include <mutex>

// Windows Header files
#include <WindowsAbstraction.h>

// Standard Header files
#include <algorithm>

// Eigen Header files
#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen\Dense>

// Project Header files
#include "RobotProxy.hpp"

// Number of joints
#ifndef JOINT_NUM
#define JOINT_NUM 7
#endif // JOINT_NUM

// Space dimension is 3...
#ifndef SPACE_DIM
#define SPACE_DIM 3
#endif // SPACE_DIM

// Twist dimension
#ifndef TWIST_DIM
#define TWIST_DIM 6
#endif // TWIST_DIM

// Typedef Eigen
#ifndef EIGEN_TYPEDEF_7DIM
#define EIGEN_TYPEDEF_7DIM 1

#define FRI_INIT_DUMMY_VALUE 1e3

#define CYCLE_TIME 0.005


namespace Eigen {

	typedef Eigen::Matrix<double, JOINT_NUM, 1> Vector7d;
	typedef Eigen::Matrix<float, JOINT_NUM, 1> Vector7f;

	typedef Eigen::Matrix<double, TWIST_DIM, 1> Vector6d;
	typedef Eigen::Matrix<float, TWIST_DIM, 1> Vector6f;

	typedef Eigen::Matrix<double, TWIST_DIM, JOINT_NUM> Matrix6x7d;
	typedef Eigen::Matrix<float, TWIST_DIM, JOINT_NUM> Matrix6x7f;

	typedef Eigen::Matrix<float, JOINT_NUM, JOINT_NUM> Matrix7f;
	typedef Eigen::Matrix<float, JOINT_NUM, JOINT_NUM> Matrix7f;

	typedef Eigen::Matrix<double, SPACE_DIM, JOINT_NUM> Matrix3x7d;
	typedef Eigen::Matrix<float, SPACE_DIM, JOINT_NUM> Matrix3x7f;

}
#endif // EIGEN_TYPEDEF_7DIM


struct FRIState {

	Eigen::Vector7f jointPositions;
	Eigen::Vector7f jointTorques;
	Eigen::Vector7f extEstJointTorques;
	Eigen::Vector6f extEstCartForces;

	Eigen::Vector7f friGravityVector;
	Eigen::Matrix7f friInertiaMatrix;

	double stamp;

};

typedef Eigen::Vector7f FRICommands;
typedef Eigen::Vector7f FRIJointRefs;


class KukaProxy : public RobotProxy {

public: 

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	* @brief Default contructor of KukaProxy class
	*
	*/
	KukaProxy();

	/**
	* @brief Default destroyer of KukaProxy class
	*
	*/
	~KukaProxy();

	/**
	* @brief Default init function
	*/
	void init();

	/**
	* @brief Default run function
	*/
	void run();

	/**
	* @brief Default clear function
	*/
	void clear();

	/**
	* @brief Set function
	* Set the state of the robot aquired from FRI
	* @param state_ the state of the robot (i.e., joint position, joint velocity, joint torque)
	*/
	void setFRIState(const FRIState& state_);
	
	/**
	* @brief Set function
	* Set the input of the robot as a vector of commanded joint positions
	* @param the commanded joint position vector
	*/
	void setFRICommands(const FRICommands& input_);

	/**
	* @brief Get function
	* Retrieves the input of the robot as a vector of commanded joint positions
	* @return the commanded joint position vector
	*/
	FRICommands getFRICommands();

	/**
	* @brief Get function
	* Get the state of the robot aquired from FRI
	* @return the state of the robot (i.e., joint position, joint velocity, joint torque)
	*/
	FRIState getFRIState();

	/**
	*@brief Get function
	* Get the full state of the robot from the proxy
	* @return the full state of the robot from the proxy
	*/
	//RobotState getRobotState();

	/**
	* @brief Set function
	* Set the Vector with reference joint position values
	* @param the Vector with reference joint position values
	*/
	void setFRIJointRefs(const FRIJointRefs& ref_);

	/**
	* @brief Get function
	* Retrieves the Vector with reference joint position values
	* @return the Vector with reference joint position values
	*/
	FRIJointRefs getFRIJointRefs();

	/**
	* @brief Regulation routine
	* Regulate the robot joint configuration to the input desired vector, using the Reflexxes motion library
	* Require jointRefs variable to be set
	*/
	void autonomousJointPositionRegulation();

	/**
	* @brief Wake function
	* Asynchrounous external request of notify on all the condition variables
	*/
	inline void wakeFromCondVar() { this->stateReady = true; cvState.notify_all(); cvInput.notify_all(); }

	/**
	* @brief Set function (virtual)
	* Set the current vector of measured joint position
	* @param q: the current vector of measured joint position
	*/
	void setMsrJointPosition(const Eigen::VectorXf& q);

	/**
	* @brief Get function (virtual)
	* Get the current vector of measured joint position
	* @return the current vector of measured joint position
	*/
	Eigen::VectorXf getMsrJointPosition();

	/**
	* @brief Set function (virtual)
	* Set the current vector of measured joint torque
	* @param tau: the current vector of measured joint torque
	*/
	void setMsrJointTorque(const Eigen::VectorXf& tau);

	/**
	* @brief Get function (virtual)
	* Get the current vector of measured joint torque
	* @return the current vector of measured joint torque
	*/
	Eigen::VectorXf getMsrJointTorque();

	/**
	* @brief Set function (virtual)
	* Set the current vector of estimated external joint torque
	* @param tau: the current vector of estimated external joint torque
	*/
	void setEstExtJointTorque(const Eigen::VectorXf& tau);

	/**
	* @brief Get function (virtual)
	* Get the current vector of estimated external joint torque
	* @return the current vector of estimated external joint torque
	*/
	Eigen::VectorXf getEstExtJointTorque();

	/**
	* @brief Set function (virtual)
	* Set the current vector of estimated external Cartesian force
	* @param q: the current vector of estimated external Cartesian force
	*/
	void setEstExtCartForce(const Eigen::Vector6f& f);

	/**
	* @brief Get function (virtual)
	* Get the current vector of estimated external Cartesian force
	* @return the current vector of estimated external Cartesian force
	*/
	Eigen::Vector6f getEstExtCartForce();

	inline void setCommands(const Eigen::VectorXf& qcmd) {
		Eigen::Vector7f qcmd_ = qcmd;
		this->setFRICommands(qcmd_);
	}


	/**
	* @brief Set Joint reference vector (virtual)
	* Set the joint reference vector for autonomous pose regolation
	* @param qcmd: the joint reference vector
	*/
	inline void setJointRefs(const Eigen::VectorXf& jref) {
		Eigen::Vector7f jref_ = jref;
		this->setFRIJointRefs(jref_);
	}


	/*
	* @brief Get function
	* Get the full robot state from the proxy
	* @return the RobotState structure
	*/
	inline RobotState getRobotState();



	double t_;							//!< tiime stamp

	
private: 

	FastResearchInterface *FRI;			//!< The FastResearchInterface object to communicate with the real KUKA robot
	FRIState state;						//!< State of the robot acquired from FRI (i.e., joint positions, velocities and torques)
	FRICommands input;					//!< Input of the robot as vector of commanded joint positions
	FRIJointRefs jointRefs;				//!< Vector with reference joint position values

	std::mutex cmdMtx;
	std::mutex stateMtx;
	std::condition_variable cvState;
	std::condition_variable cvInput;
	bool stateReady;
	bool inputReady;

	bool stateConsumed;
	bool inputConsumed;
	std::condition_variable cvStateConsumed;
	std::condition_variable cvInputConsumed;
};


#endif // KUKA_PROXY_HPP_
