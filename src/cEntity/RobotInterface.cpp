// Project Header files
#include "RobotInterface.hpp"

// Standard header files
#include <iostream>

/**
* @brief Default constructor of the Instrument class
*/
RobotInterface::RobotInterface() : Instrument() {

	this->jointNum = -1;
	this->timestamp__ = 0.0;
}

/**
* @brief Constructor of the RobotInterface class with name argument
* @param name_ the name of the Instrument
*/
RobotInterface::RobotInterface(const std::string& name_) : Instrument(name_) {

	this->jointNum = -1;
	this->timestamp__ = 0.0;

}


/**
* @brief Constructor of the RobotInterface class with name argument
* @param qnum the dofs of the robot
*/
RobotInterface::RobotInterface(const int& qnum) : Instrument(){

	this->setJointNum(qnum);
	this->initData();

}


/**
* @brief Init function
* Init all the dynamic data of the class with the known value of dofs of the
*/
void RobotInterface::initData() {

	if (this->jointNum != -1) {
		this->jointMsrPosition__.setZero(this->jointNum);
		this->jointCmdPosition__.setZero(this->jointNum);
		this->jointMsrVelocity__.setZero(this->jointNum);
		this->jointCmdVelocity__.setZero(this->jointNum);
		this->jointAcceleration__.setZero(this->jointNum);
		this->jointMsrTorques__.setZero(this->jointNum);
		this->linkLengths__.setZero(this->jointNum);
		this->Tbli__.resize(this->jointNum);
		for (int i = 0; i < this->Tbli__.size(); i++) {
			this->Tbli__[i].setIdentity();
		}

		this->Tbee__.setIdentity();
		this->Jbee__.setZero(SPACE_DIM * 2, this->jointNum);
		this->vbee__.setZero();
		this->res__.setZero(this->jointNum);
		this->resOffset__.setZero(this->jointNum);

		this->robDynPars.B.setZero(this->jointNum, this->jointNum);
		this->robDynPars.C.setZero(this->jointNum, this->jointNum);
		this->robDynPars.f.setZero(this->jointNum);
		this->robDynPars.g.setZero(this->jointNum);
		this->robDynPars.tau.setZero(this->jointNum);

		this->resGain.setZero(this->jointNum, this->jointNum);
		this->invResGainDt.setZero(this->jointNum, this->jointNum);
		this->dynModelSum.setZero(this->jointNum);
		this->residualSum.setZero(this->jointNum);
		this->p0.setZero(this->jointNum);

		this->includeToolDynParams = false;
		this->withFriction = false;			
		this->timestamp__ = 0.0;

	}
	else {
		std::cout << "[RobotInterface] WARNING: jointNum not set!!!" << std::endl;
	}

}

/**
* @brief Forward differential kinematics function
* Compute the forward differential kinematic of the robot, by evaluating the Jacobian matrix on the current set value of this->jointPosition
* Internally set the array of homogeneous transformation matrices, i.e., this->Tbli
*/
void RobotInterface::computeDiffKinematics() {

	Eigen::MatrixXf J0n(SPACE_DIM * 2, this->jointNum);
	Eigen::Matrix6f Wne;	// Twist matrix
	Eigen::Matrix3f Sren;	// Skew-symmetricc matrix
	Eigen::Vector3f ren;
	Eigen::MatrixXf JeeT(this->jointNum,SPACE_DIM*2);

	// Build the twist matrix
	ren << 0.0, 0.0, -this->computeEEOffset();
	Sren = skew(ren);
	Wne.setIdentity();
	//Wne.topRightCorner(SPACE_DIM, SPACE_DIM) = Sren;
	//Wne.setIdentity(); //<-- Check this, maybe the tools offset is already taken into account in J0n, thus it's already a Jee matrix

	// Compute the Jacobian matrix
	J0n = this->computeFullJacobian(this->jointNum-1);

	// Compute the Jacobian matrix related to the end-effector
	this->Jbee__ = Wne * J0n;

	// Compute the end-effector Cartesian velocity
	this->vbee__ = this->Jbee__ * this->jointMsrVelocity__;

}


/**
* @brief Jacobian function
* Compute the full Jacobian matrix for the given chosen link (end-effector by default).
* @param link the link of which the Jacobian matrix has to be computed
* @return the requested Jacobian matrix
*/
Eigen::MatrixXf RobotInterface::computeFullJacobian(const int& link) {

	Eigen::MatrixXf J(SPACE_DIM*2 , this->jointNum);
	Eigen::MatrixXf Jl(SPACE_DIM , this->jointNum);
	Eigen::MatrixXf Ja(SPACE_DIM , this->jointNum);

	// Compute Linear part
	Jl = this->computeLinearJacobian(link);

	// Compute Linear part
	Ja = this->computeAngularJacobian(link);

	// Build the full Jacobian
	J.topRows(SPACE_DIM) = Jl;
	J.bottomRows(SPACE_DIM) = Ja;

	// Set the Jacobian matrix in the class
	this->Jbee__ = J;

	return J;

}

/**
* @brief Pseudo-inverse compuation function
* Compute and return the nx6 pseudo-inverse matrix of the current Jacobian
* @return the nx6 pseudo-inverse matrix of the current Jacobian
*/
Eigen::MatrixXf RobotInterface::getJPinv() {

	Eigen::MatrixXf JT(this->jointNum, SPACE_DIM*2), Jpinv(this->jointNum, SPACE_DIM * 2);
	Eigen::MatrixXf J(SPACE_DIM * 2, this->jointNum);

	J = this->Jbee__;
	JT = J.transpose();

	// Compute the Jacobian pseudo-inverse
	Jpinv = JT * (J * JT).inverse();

	// Return the pseudo-inverse
	return Jpinv;

}

/**
* @brief Update function
* Update the full state of the robot, given the input joint positions, velocities and torques
* Internally compute forward kinematics, differential kinematics, dynamics and residual vectors
*
*/
void RobotInterface::updateRobotState(const Eigen::VectorXf& q, const Eigen::VectorXf& qdot, const Eigen::VectorXf& tau, const bool& online) {

	// Set joint positions, joint velocities and joint measured torques on the robot
	this->setMsrJointPosition(q);
	this->setMsrJointVelocity(qdot);
	this->setMsrJointTorques(tau);

	// Compute forward kinematics
	this->computeKinematics();

	// Compute differential kinematics
	this->computeDiffKinematics();

	// Compute dynamics
	this->computeDynamics();

	// Compute residual vector
	this->computeResidualFull(q, qdot, tau);
	/*if (online) {
		this->computeResidualFull(q, qdot, tau);
	}//*/

	// Compute external contact force vector from residual
	this->extFbee__ = this->staticForceReconstruction(this->res__);
}


/**
* @brief Residual calculation function
* COmpute the full residual vector from the known joint velocity qdot and the applied torques trq
* Set internally the residual vector
* @param q the joint position vector
* @param qdot the joint velocity vector
* @param teq the joint applied torques
* @param gain the gain for the computation of the residual (default = 5.0)
*/
void RobotInterface::computeResidualFull(const Eigen::VectorXf& q, const Eigen::VectorXf& q_dot, const Eigen::VectorXf& trq) {

	// Define required variables
	Eigen::VectorXf rk(this->jointNum), qdot_k(this->jointNum), tau_k(this->jointNum), gk(this->jointNum), fk(this->jointNum), tau_friction(this->jointNum);
	Eigen::MatrixXf KDtIinv(this->jointNum, this->jointNum), Bk(this->jointNum, this->jointNum), Ck(this->jointNum, this->jointNum), CkT(this->jointNum, this->jointNum), K(this->jointNum, this->jointNum);
	Eigen::VectorXd q_(this->jointNum), qdot_(this->jointNum), trq_(this->jointNum), params_(this->jointNum);
	float dt = this->dt__;

	// Initialize variables
	KDtIinv = this->invResGainDt;
	K = this->resGain;
	q_ = q.cast<double>();
	qdot_ = q_dot.cast<double>();
	trq_ = trq.cast<double>();
	//params_ = this->frictionGravityParams.cast<double>();

	// Get variables of the current iteration
	Bk = this->robDynPars.B;
	Ck = this->robDynPars.C;
	gk = this->robDynPars.g;
	fk = this->robDynPars.f;
	CkT = Ck.transpose();
	qdot_k = q_dot;

	// Set the measured torques
	tau_k = trq;
	if (this->isDynModelWithFriction()) {
		//tau_k -= (this->computeDeltaFriction(q_, qdot_, params_)).cast<float>();
	}

	// Update dynModelSum
	this->dynModelSum += (tau_k + CkT * qdot_k - gk);

	// Compute the current value of the residual vector
	rk = KDtIinv * K * ((Bk * qdot_k - this->p0) - this->dynModelSum * dt - this->residualSum * dt);


	// Update the residual cumulative term
	this->residualSum += rk;

	// Update the residual vector with the quantity computed above
	if (this->withResOffset) {
		// If offset reset is requested, subtract it to the residual measurement
		this->setResidualVector(rk - this->resOffset__);
	}
	else {
		this->setResidualVector(rk);
	}

	//this->residual = rk;

}


/**
* @brief Force reconstruction function
* Compute the Cartesian force vector from the given joint torque vector, through the static relationship
* @param tau: the joint torque vector
* @return the corresponding Cartesian force vector
*/
Eigen::Vector6f RobotInterface::staticForceReconstruction(const Eigen::VectorXf& tau) {

	Eigen::Vector6f f;
	Eigen::MatrixXf JT(this->jointNum,SPACE_DIM*2);
	Eigen::MatrixXf J(SPACE_DIM*2, this->jointNum), JThash(SPACE_DIM*2, this->jointNum);
	Eigen::MatrixXf Jl(SPACE_DIM, this->jointNum), JlThash(SPACE_DIM, this->jointNum);
	Eigen::MatrixXf JlT(this->jointNum, SPACE_DIM);
	f.setZero();

	// Build the pseudo-inverse of the transpose jacobian
	J = this->Jbee__;
	JT = J.transpose();
	Jl = J.topRows(SPACE_DIM);
	JlT = Jl.transpose();
	JThash = (J * JT).inverse() * J; //<-- This is the correct one
	JlThash = (Jl * JlT).inverse() * Jl; //<--- This is more robust if you assume moments equal to 0

	// Compute the force vector
	//f = JThash * tau; // // <--- If you want to use full Jacobian
	f.topRows(SPACE_DIM) = JlThash * tau; // <--- If you want to use only linear Jacobian

	// Return f
	return f;

}


/**
* @brief Utility function
* Compute the offset to apply in the last DH matrix of the forward kinematics to get the coordinates of the final tip
* @return the offset to apply on the z-axis of the end-effector frame
*/
float RobotInterface::computeEEOffset() {

	float offset = 0.0;
	float tool_i_size[SPACE_DIM];

	for (int i = 0; i < this->tools.size(); i++) {
		(this->tools[i].getSize(tool_i_size));
		offset += tool_i_size[Z_AXIS];
	}
	return offset;

}


/**
* @brief Weight computation
* Retrieves the total weight of all the attached end-effectors
* @return the total weight
*/
float RobotInterface::computeTotalEEWeight() {

	float weight = 0.0;

	for (int i = 0; i < this->tools.size(); i++) {
		float wi = this->tools[i].getWeight();
		weight += wi;
	}

	return weight;

}

/**
* @brief CoM computation
* Retrieves the total CoM of all the attached end-effectors
* @return the total CoM
*/
void RobotInterface::computeOverallEECoM(float out[]) {

	float com[SPACE_DIM];
	float tool_i_com[SPACE_DIM];

	std::memset(com, 0.0, SPACE_DIM * sizeof(float));
	std::memset(tool_i_com, 0.0, SPACE_DIM * sizeof(float));

	for (int i = 0; i < this->tools.size(); i++) {

		this->tools[i].getCoM(tool_i_com);

		com[0] += tool_i_com[0];
		com[1] += tool_i_com[1];
		com[2] += tool_i_com[2];
	}
	// TODO: Check this hard-coded value for force sensor + needle
	com[2] = 0.1069;
	//this->tools[0].getCoM(tool_i_com);
	//std::memcpy(com, tool_i_com, SPACE_DIM * sizeof(float));

	std::memcpy(out, com, SPACE_DIM * sizeof(float));

}

