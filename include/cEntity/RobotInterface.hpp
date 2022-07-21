#ifndef ROBOT_INTERFACE_HPP_
#define ROBOT_INTERFACE_HPP_

// System Header files
#include <vector>
#include <mutex>

// Eigen Header files
#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen\Dense>

// Project Header files
#include "Instrument.hpp"
#include "EndEffector.hpp"
#include "utils.hpp"


struct RobotDynParams {

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::MatrixXf B;		//!< Inertia matrix
	Eigen::MatrixXf C;		//!< Coriolis and centrifula terms
	Eigen::VectorXf g;		//!< Gravity vector
	Eigen::VectorXf f;		//!< Friction vector
	Eigen::VectorXf tau;	//!< Torque vector

};

class RobotInterface : public Instrument {

public:

	/** PLEASE READ THIS CAREFULLY **/
	/* When we define a class containing fixed-size Eigen structures,
	 * and we later instantiate such class dynamically, it occurs an issue 
	 * that may result in undesired and unexpected program crash if not 
	 * properly handled, due to ACCESS VIOLATIONS.
	 * To prevent this, every class containing fixed-size Eigen variables
	 * must add the line below in the public part of the class definition, 
	 * in order to overload the new operator and explicitly handle alignment
	 * problems in the variable definitions. If you have a polymorphic structure
	 * among your class, place this macro in the Base class, like here. 
	 */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
		
	/**
	* @brief Default constructor of the Instrument class
	*/
	RobotInterface();

	/**
	* @brief Constructor of the RobotInterface class with name argument
	* @param name_ the name of the Instrument
	*/
	RobotInterface(const std::string& name_);

	/**
	* @brief Constructor of the RobotInterface class with name argument
	* @param qnum the dofs of the robot
	*/
	RobotInterface(const int& qnum);

	/**
	* @brief Default destroyer of the RobotInterface class
	*/
	~RobotInterface() {}

	/**
	* @brief Init function (virtual)
	* Check the input pair of comment+value to assign the corresponding parameter
	* The function assumes that configFile has been previously set
	*/
	virtual void loadParamsFromConfigFile(const std::string& comment, const std::string& value) = 0;

	/**
	* @brief Set function
	* Set the number of joints of the robot
	* @param num: the number of joints of the robot
	*/
	inline void setJointNum(const int& num) { this->jointNum = num; }

	/**
	* @brief Get function
	* Get the number of joints of the robot
	* @return the number of joints of the robot
	*/
	inline int getJointNum() { return this->jointNum ; }

	/**
	* @brief Init function
	* Init all the dynamic data of the class with the known value of dofs of the 
	*/
	void initData();

	/**
	* @brief Forward kinematics function (virtual)
	* Compute the forward kinematic of the robot, based on the current set value of this->jointPosition
	* Internally set the array of homogeneous transformation matrices, i.e., this->Tbli
	*/
	virtual void computeKinematics() = 0;

	/**
	* @brief Forward differential kinematics function
	* Compute the forward differential kinematic of the robot, by evaluating the Jacobian matrix on the current set value of this->jointPosition
	* Internally set the array of homogeneous transformation matrices, i.e., this->Tbli
	*/
	void computeDiffKinematics();

	/**
	* @brief Jacobian function (virtual)
	* Compute the linear Jacobian matrix for the given chosen link (end-effector by default).
	* @param link the link of which the Jacobian matrix has to be computed
	* @return the requested Jacobian matrix
	*/
	virtual Eigen::MatrixXf computeLinearJacobian(const int& link) = 0;

	/**
	* @brief Jacobian function (virtual)
	* Compute the angular Jacobian matrix for the given chosen link (end-effector by default).
	* @param link the link of which the Jacobian matrix has to be computed
	* @return the requested Jacobian matrix
	*/
	virtual Eigen::MatrixXf computeAngularJacobian(const int& link) = 0;

	/**
	* @brief Jacobian function
	* Compute the full Jacobian matrix for the given chosen link (end-effector by default).
	* @param link the link of which the Jacobian matrix has to be computed
	* @return the requested Jacobian matrix
	*/
	Eigen::MatrixXf computeFullJacobian(const int& link);

	/**
	* @brief Forward dynamics function
	* Compute the dynamic model of the robot, based on the current set value of joint position, velocity and torques,
	* along with the known dynamic parameters of the model
	*/
	virtual void computeDynamics() = 0;

	/**
	* @brief Update function
	* Update the full state of the robot, given the input joint positions, velocities and torques
	* Internally compute forward kinematics, differential kinematics, dynamics and residual vectors
	*/
	void updateRobotState(const Eigen::VectorXf& q, const Eigen::VectorXf& qdot, const Eigen::VectorXf& tau, const bool& online);

	/**
	* @brief Force reconstruction function
	* Compute the Cartesian force vector from the given joint torque vector, through the static relationship
	* @param tau: the joint torque vector
	* @return the corresponding Cartesian force vector
	*/
	Eigen::Vector6f staticForceReconstruction(const Eigen::VectorXf& tau);

	/**
	* @brief Residual calculation function
	* COmpute the full residual vector from the known joint velocity qdot and the applied torques trq
	* Set internally the residual vector
	* @param q the joint position vector
	* @param qdot the joint velocity vector
	* @param teq the joint applied torques
	* @param gain the gain for the computation of the residual (default = 5.0)
	*/
	void computeResidualFull(const Eigen::VectorXf& q, const Eigen::VectorXf& q_dot, const Eigen::VectorXf& trq);

	/**
	* @brief Init function
	* Initialize the generalized momentum vector
	* Internally fills the variable p0
	*/
	inline void initializeGeneralizedMomentum() { this->p0 = this->robDynPars.B * this->jointMsrVelocity__; }

	/**
	* @brief Get function
	* Get the struct containing the model dynamic parameters
	* @return the dynamic parameters
	*/
	inline RobotDynParams getModelDynParams() { return this->robDynPars; }

	/**
	* @brief Add an EndEffector to the robot
	* @param ee the EndEffector object to be added
	*/
	inline void addEndEffector(const EndEffector& ee) { this->tools.push_back(ee); }


	inline EndEffector getEndEffectorFromName(const std::string& eeName) { return *(std::find(this->tools.begin(), this->tools.end(), EndEffector(eeName))); }

	/**
	* @brief Set function
	* Set the sample time of the robot (for KUKA it has to be always = 0.005)
	* @param dt: the sample time to be set
	*/
	inline void setSampleTime(const float& dt) { this->dt__ = dt; }

	/**
	* @brief Get function
	* Get the sample time of the robot (for KUKA it has to be always = 0.005)
	* @return the sample time
	*/
	inline float getSampleTime() { return this->dt__; }

	/**
	* @brief Set function
	* Set the timstamp associated to the robot data
	* @param t: the timestamp
	*/
	inline void setDataTimestamp(const float& t) { this->timestamp__ = t; }
	
	/**
	* @brief Get function
	* Get the timstamp associated to the robot data
	* @return the timestamp
	*/
	inline float getDataTimestamp() { return this->timestamp__; }

	/**
	* @brief Reset data
	* Reset the robot data
	*/
	virtual void resetDynParams() = 0;

	/**
	* @brief Set function
	* Set the measured joint position vector of the KUKA robot
	* @param q the measured joint position vector to be set
	*/
	inline void setMsrJointPosition(const Eigen::VectorXf& q) { this->jointMsrPosition__ = q; }

	/**
	* @brief Set function
	* Set the commanded joint position vector of the robot
	* @param q the commanded joint position vector to be set
	*/
	inline void setCmdJointPosition(const Eigen::VectorXf& q) { this->jointCmdPosition__ = q; };

	/**
	* @brief Set function
	* Set the measured joint velocity vector of the robot
	* @param qd the measured joint velocity vector to be set
	*/
	inline void setMsrJointVelocity(const Eigen::VectorXf& qdot) { this->jointMsrVelocity__ = qdot; }

	/**
	* @brief Set function
	* Set the commandede joint velocity vector of the robot
	* @param qd the commanded joint velocity vector to be set
	*/
	inline void setCmdJointVelocity(const Eigen::VectorXf& qdot) { this->jointCmdVelocity__ = qdot; }


	/**
	* @brief Set function
	* Set the measured joint acceleration vector of the robot
	* @param qd the measured joint acceleration vector to be set
	*/
	inline void setMsrJointAcceleration(const Eigen::VectorXf& qdotdot) { this->jointAcceleration__ = qdotdot; }

	/**
	* @brief Set function
	* Set the measured joint torques vector of the robot
	* @param qd the measured joint torques vector to be set
	*/
	inline void setMsrJointTorques(const Eigen::VectorXf& tau) { this->jointMsrTorques__ = tau; }

	/**
	* @brief Get function
	* Get the measured joint torques vector of the robot
	* @return the measured joint torques vector to be set
	*/
	inline Eigen::VectorXf getMsrJointTorques() { return this->jointMsrTorques__; }

	/**
	* @brief Set function
	* Set the link length of the robot
	* @param ll: the link length of the robot
	*/
	inline void setLinkLengths(const Eigen::VectorXf& l) { this->linkLengths__ = l; }

	/**
	* @brief Get function
	* Get the measured joint position vector of the robot
	* @return the measured joint position vector to be set
	*/
	inline Eigen::VectorXf getMsrJointPosition() { return this->jointMsrPosition__; }

	/**
	* @brief Get function
	* Get the commanded joint position vector of the robot
	* @return  the commanded joint position vector to be set
	*/
	inline Eigen::VectorXf getCmdJointPosition() { return this->jointCmdPosition__; }

	/**
	* @brief Get function
	* Get the measured joint velocity vector of the robot
	* @return the measured joint velocity vector to be set
	*/
	inline Eigen::VectorXf getMsrJointVelocity() { return this->jointMsrVelocity__; }

	/**
	* @brief Get function
	* Get the commandede joint velocity vector of the robot
	* @return the commanded joint velocity vector to be set
	*/
	inline Eigen::VectorXf getCmdJointVelocity() { return this->jointCmdVelocity__; }

	/**
	* @brief Set function 
	* Set the 4x4 homogeneous matrix of the EE pose wrt the base frame
	* @param T: the 4x4 homogeneous matrix
	*/
	inline void setTbee(const Eigen::Matrix4f& T) { this->Tbee__ = T; }
	
	/**
	* @brief Get function 
	* Get the 4x4 homogeneous matrix of the EE pose wrt the base frame
	* @return the 4x4 homogeneous matrix
	*/
	inline Eigen::Matrix4f getTbee() { return this->Tbee__; }

	/**
	* @brief Set function 
	* Set the Jacobian matrix related to the velocity of the EE wrt the base frame
	* @param T: the Jacobian matrix
	*/
	inline void setJacobian(const Eigen::MatrixXf& J) { this->Jbee__ = J; }

	/**
	* @brief Get function
	* Get the Jacobian matrix related to the velocity of the EE wrt the base frame
	* @return the Jacobian matrix
	*/
	inline Eigen::MatrixXf getJacobian() { return this->Jbee__; }

	/**
	* @brief Pseudo-inverse compuation function
	* Compute and return the nx6 pseudo-inverse matrix of the current Jacobian
	* @return the nx6 pseudo-inverse matrix of the current Jacobian
	*/
	Eigen::MatrixXf getJPinv();

	/**
	* @brief Set function 
	* Set the Cartesian velocity vector of the EE velocity wrt the base frame
	* @param v: the Cartesian velocity vector
	*/
	inline void setEEVelocity(const Eigen::Vector6f& v) { this->vbee__ = v; }

	/**
	* @brief Get function
	* Compute and return the 6D velocity of the end-effector
	* @return the 6D velocity vector of the end-effector
	*/
	Eigen::Vector6f getEEVelocity() { return this->vbee__; }

	/**
	* @brief Set function
	* Set the residual vector computed by this class, as estimation of the external torques applied on the model
	* @param r: the residual vecotr computed by this class
	*/
	inline void setResidualVector(const Eigen::VectorXf& r) {

		{
			std::lock_guard<std::mutex> lock(resMtx);
			this->res__ = r;
		}
	}
	
	/**
	* @brief Get function
	* Get the Residual vector of the manipulator
	* @return the residual vector
	*/
	inline Eigen::VectorXf getResidualVector() {

		{
			std::lock_guard<std::mutex> lock(resMtx);
			return this->res__;
		}
	}

	/**
	* @brief Check function
	* Verifies if the offset has to be taken into account for the computation of the residual vectors
	* @return true if the offset is accounted, false otherwise
	*/
	inline bool applyResidualOffset() { return this->withResOffset; }

	/**
	* @brief Get function
	* Get the residual gain vector
	* @return the residual gain vector
	*/
	inline Eigen::MatrixXf getResidualGain() { return this->resGain; }

	/**
	* @brief Set function 
	* Set the Residual offset vector of the manipulator
	* @param r: the residual vector offset
	*/
	void setResidualVectorOffset(const Eigen::VectorXf& r) { this->resOffset__ = r; }

	/**
	* @brief Get function (virtual)
	* Get the Residual offset vector of the manipulator
	* @return the residual vector offset
	*/
	Eigen::VectorXf getResidualVectorOffset() { return this->resOffset__; }

	/**
	* @brief Get function
	* Get the Cartesian Force vector at the end-effector wrt base frame, reconstructed from residual
	* @return the Cartesian force vector
	*/
	inline Eigen::Vector6f getExtContactForce() { return this->extFbee__; }

	/**
	* @brief Get function
	* Retrieves the position of the i-th link
	* @return the i-th link position vector
	*/
	inline Eigen::Vector3f getLinkPosition(const int& link) { return this->Tbli__[link].block<3, 1>(0, 3); }

	/**
	* @brief Get function
	* Retrieves the rotation matrix of the i-th link
	* @return the i-th link orientation matirx
	*/
	inline Eigen::Matrix3f getLinkRotMat(const int& link) { return this->Tbli__[link].topLeftCorner(3, 3); }

	/**
	* @brief Get function
	* Retrieves the position of the End-Effector tip (accounting also the optional EndEffector objects attached)
	* @return the tip position vector
	*/
	inline Eigen::Vector3f getEEPosition() { return getLinkPosition(jointNum-1); }

	/**
	* @brief Get function
	* Retrieves the rotation matrix of the End-Effector tip (accounting also the optional EndEffector objects attached)
	* @return the tip position vector
	*/
	inline Eigen::Matrix3f getEERotMat() { return getLinkRotMat(jointNum - 1); }

	/**
	* @brief Set function
	* Set if the dynamic model to be computed has to account the dynamic parameters of the tools
	* @param true if the dynamic model taks into account the tool, false otherwise
	*/
	inline void dynModelWithTool(const bool& withTool) { this->includeToolDynParams = withTool; }

	/**
	* @brief Check function
	* Check if the dynamic model to be computed has to account the dynamic parameters of the tools
	* @return true if the dynamic model taks into account the tool, false otherwise
	*/
	inline bool isDynModelWithTool() { return this->includeToolDynParams; }

	/**
	* @brief Set function
	* Set if the dynamic model has to account the friction
	* @param true if the friction is taken into account, false otherwise
	*/
	inline void includeFrictionInModel(const bool& fric) { this->withFriction = fric; }

	/**
	* @brief Check function
	* Check if the dynamic model takes into account the friction
	* @return true if the friction is taken into account, false otherwise
	*/
	inline bool isDynModelWithFriction() { return this->withFriction; }

	/**
	* @brief Set function
	* Set the startOnPlace flag to state if the robot will start from the currently set joint configuration
	* @param onplace: the flag to be set
	*/
	inline void setOnPlace(const bool& onplace) { this->startOnPlace = onplace; }

	/**
	* @brief Get function
	* Get the startOnPlace flag to state if the robot will start from the currently set joint configuration
	* @return the flag
	*/
	inline bool robotStartsOnPlace() { return this->startOnPlace; }

	/**
	* @brief Get function
	* State if use the model-based residual vector or the measurement acquired from FRI
	* @return true if using the model-based residual vector, false otherwise
	*/
	inline bool useModelBasedResidual() { return this->withModelBasedResidual; }


protected:

	int jointNum;										//!< Number of joints of the robot manipulator
	Eigen::VectorXf jointMsrPosition__;					//!< Vector of measured joint position
	Eigen::VectorXf jointCmdPosition__;					//!< Vector of commanded joint position
	Eigen::VectorXf jointMsrVelocity__;					//!< Vector of measured joint velocity
	Eigen::VectorXf jointCmdVelocity__;					//!< Vector of commanded joint velocity
	Eigen::VectorXf jointAcceleration__;				//!< Vector of measured joint acceleration
	Eigen::VectorXf jointMsrTorques__;					//!< Vector of measured joint torques
	Eigen::VectorXf linkLengths__;						//!< Vector of link lengths of the manipulator
	std::vector <Eigen::MatrixXf> Tbli__;				//!< Array of 4x4 homogeneous transformation matrices of the i-th link pose wrt the link i-1
	std::vector<EndEffector> tools;						//!< List of endeffectors attached to the robot (standard setup is force senor + needle)
	Eigen::Matrix4f Tbee__;								//!< 4x4 homogeneous matrix of the pose of the EE wrt the base
	Eigen::MatrixXf Jbee__;								//!< 6xn Jacobian matrix
	Eigen::Vector6f vbee__;								//!< 6D vector of velocity of the EE in base frame
	Eigen::VectorXf res__;								//!< Vector of estimated residual
	Eigen::VectorXf resOffset__;						//!< Vector of estimated residual offset
	Eigen::Vector6f extFbee__;							//!< External Cartesian force vector generated from residual vector through static relationship
	RobotDynParams robDynPars;							//!< Struct containing the dynamic parameters and contributions of the robot


	Eigen::MatrixXf resGain;							//!< Matrix of residual gains
	Eigen::MatrixXf invResGainDt;						//!< Constant gain matrix occuring in the formulation of the residual expression as inverse matrix
	Eigen::VectorXf dynModelSum;						//!< Vector built by summing up all the components of the dynamic model
	Eigen::VectorXf residualSum;						//!< Cumulative residual variable
	Eigen::VectorXf p0;									//!< Generalized momentum at time 0

	bool startOnPlace;									//!< State if the robot has to start from the currently set joint configuration, or if it has to move to an initial specified one
	bool includeToolDynParams;							//!< State if the computation of the dynamic model has to account also the tool parameters
	bool withFriction;									//!< State if the computation of the dynamic model has to account also the friction
	bool withResOffset;									//!< State if the residual vector has to be take into account the initial offset to be removed
	bool withModelBasedResidual;						//!< State if the residual vector has to be read from FRI or computed from model-based formulation

	float timestamp__;									//!< Timestamp associated to the stored data
	float dt__;											//!< Sample time (e.g., for Kuka it has to be always = 0.005)
	std::mutex resMtx;									//! mutex for the residual vector


	/**
	* @brief Utility function
	* Compute the offset to apply in the last DH matrix of the forward kinematics to get the coordinates of the final tip
	* @return the offset to apply on the z-axis of the end-effector frame
	*/
	float computeEEOffset();

	/**
	* @brief CoM computation
	* Retrieves the total CoM of all the attached end-effectors
	* @return the total CoM
	*/
	void computeOverallEECoM(float out[]);

	/**
	* @brief Weight computation
	* Retrieves the total weight of all the attached end-effectors
	* @return the total weight
	*/
	float computeTotalEEWeight();

};


#endif // ROBOT_INTERFACE_HPP_
