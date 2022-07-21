// Standard Header files
#define _USE_MATH_DEFINES
#include <cmath>

// Project Header files
#include "KUKARobot.hpp"
#include "utils.hpp"
#include "DBWrapper.hpp"

int KUKARobot::frictionParamsNum;						//!< Number of parameters considered for the friction model
bool KUKARobot::withDeltaGravity;						//!< State if the computation of the dynamic model has to account also the locally estimated delta-gravity 

/**
* @brief Default constructor of the KUKARobot class
*/
KUKARobot::KUKARobot() : RobotInterface() {

	this->jointNum = KUKA_JOINT_NUM;
	this->initData();

}


/**
* @brief Default constructor of the KUKARobot class
* @param name_: the identifying name of the instrument in use
*/
KUKARobot::KUKARobot(const std::string& name_) : RobotInterface(name_) {

	this->jointNum = KUKA_JOINT_NUM;
	this->initData();

}


/**
* @brief Constructor of the RobotInterface class with name argument
* @param qnum the dofs of the robot
*/
KUKARobot::KUKARobot(const int& qnum) : RobotInterface(qnum) {

	this->initData();

}


/**
* @brief Init function
* Init all the dynamic data of the class with the known value of dofs of the
*/
void KUKARobot::initData() {

	RobotInterface::initData();

	this->resFriFbe.setZero();							//!< Cartesian Force vector at the end-effector wrt base frame, reconstructed from FRI residual
	this->detJJT = 0.0;
	this->detJTJ = 0.0;
	this->friResidual_off.setZero();

}


/**
* @brief Init function
* Check the input pair of comment+value to assign the corresponding parameter
* The function assumes that configFile has been previously set
* @param comment: the string specifying the title comment of parameter
* @param value: the string specifying the value of the parameter
*/
void KUKARobot::loadParamsFromConfigFile(const std::string& comment, const std::string& value) {

	/* Please note that this function is called several times*/
	if (comment.find("Links length") != std::string::npos) {
		std::vector < double > vec = parseCSVLine(value);
		for (int j = 0; j < vec.size(); j++) {
			this->linkLengths__(j) = vec[j];
		}
	}
	else if (comment.find("Dynamic model with tool") != std::string::npos) {
		this->includeToolDynParams = (std::stod(value)) ? true : false;
	}
	else if (comment.find("With Friction") != std::string::npos) {
		this->withFriction = (std::stod(value)) ? true : false;
	}
	else if (comment.find("Use model-based residual") != std::string::npos) {
		this->withModelBasedResidual = (std::stod(value)) ? true : false;
	}
	else if (comment.find("Residual gain") != std::string::npos) {
		float gain = std::stod(value);
		this->resGain = Eigen::MatrixXf::Identity(KUKA_JOINT_NUM, KUKA_JOINT_NUM) * gain;
		this->invResGainDt = (Eigen::MatrixXf::Identity(KUKA_JOINT_NUM, KUKA_JOINT_NUM) + this->resGain * this->dt__).inverse();
	}
	else if (comment.find("Start on place") != std::string::npos) {
		this->startOnPlace = (std::stod(value)) ? true : false;
	}
	else if (comment.find("Residual offset") != std::string::npos) {
		this->withResOffset = (std::stod(value)) ? true : false;
	}
	else if (comment.find("Initial configuration") != std::string::npos) {
		std::vector < double > vec = parseCSVLine(value);
		for (int j = 0; j < vec.size(); j++) {
			this->jointMsrPosition__(j) = (vec[j]) * M_PI / 180.0;
			this->jointCmdPosition__(j) = (vec[j]) * M_PI / 180.0;
		}
	}


}

/**
* @brief Forward kinematics function
* Compute the forward kinematic of the robot, based on the current set value of this->jointPosition
* Internally set the array of homogeneous transformation matrices, i.e., this->Tbli
*/
void KUKARobot::computeKinematics() {

	// Define required variables
	float C1, S1, C2, S2, C3, S3, C4, S4, C5, S5, C6, S6, C7, S7;
	Eigen::Matrix4f T0;
	Eigen::Matrix4f Ti[KUKA_JOINT_NUM];
	T0.setIdentity();


	// Pre-compute sin and cos
	C1 = cos(this->jointMsrPosition__(0));
	S1 = sin(this->jointMsrPosition__(0));
	C2 = cos(this->jointMsrPosition__(1));
	S2 = sin(this->jointMsrPosition__(1));
	C3 = cos(this->jointMsrPosition__(2));
	S3 = sin(this->jointMsrPosition__(2));
	C4 = cos(this->jointMsrPosition__(3));
	S4 = sin(this->jointMsrPosition__(3));
	C5 = cos(this->jointMsrPosition__(4));
	S5 = sin(this->jointMsrPosition__(4));
	C6 = cos(this->jointMsrPosition__(5));
	S6 = sin(this->jointMsrPosition__(5));
	C7 = cos(this->jointMsrPosition__(6));
	S7 = sin(this->jointMsrPosition__(6));

	//debugPrint<Eigen::Vector7f>("\n[In KUKA] jointPosition:", this->jointPosition.transpose());


	// Compute local transformation matrices from link i-1 to link i
	Ti[LINK1] << C1, 0, S1, 0,
		S1, 0, -C1, 0,
		0, 1, 0, (float)(this->linkLengths__(LINK1)),
		0, 0, 0, 1;
	Ti[LINK2] << C2, 0, -S2, 0,
		S2, 0, C2, 0,
		0, -1, 0, 0,
		0, 0, 0, 1;
	Ti[LINK3] << C3, 0, -S3, 0,
		S3, 0, C3, 0,
		0, -1, 0, (float)(this->linkLengths__(LINK2) + this->linkLengths__(LINK3)),
		0, 0, 0, 1;
	Ti[LINK4] << C4, 0, S4, 0,
		S4, 0, -C4, 0,
		0, 1, 0, 0,
		0, 0, 0, 1;
	Ti[LINK5] << C5, 0, S5, 0,
		S5, 0, -C5, 0,
		0, 1, 0, (float)(this->linkLengths__(LINK4) + this->linkLengths__(LINK5)),
		0, 0, 0, 1;
	Ti[LINK6] << C6, 0, -S6, 0,
		S6, 0, C6, 0,
		0, -1, 0, 0,
		0, 0, 0, 1;
	Ti[END_EFFECTOR] << C7, -S7, 0, 0,
		S7, C7, 0, 0,
		0, 0, 1, (float)(this->linkLengths__(LINK6) + this->computeEEOffset()),
		0, 0, 0, 1;

	// Compute the corresponding homogeneous transformation matrices of the i-th link pose wrt the base frame
	this->Tbli__[LINK1] = T0*Ti[LINK1];
	for (int i = LINK2; i <= END_EFFECTOR; i++) {

		this->Tbli__[i] = this->Tbli__[i-1] * Ti[i];

	}
	this->Tbee__ = this->Tbli__[END_EFFECTOR];
	
}

/**
* @brief Forward dynamics function
* Compute the dynamic model of the robot, based on the current set value of joint position, velocity and torques,
* along with the known dynamic parameters of the model
*/
#ifdef BUILD_KUKA_DYN_MODEL
void KUKARobot::computeDynamics() {

	// Required variables
	float tipDynParams[DYN_PARAMS_NUM];
	float jointFloatPosition[KUKA_JOINT_NUM], jointFloatVelocity[KUKA_JOINT_NUM], jointFloatAcceleration[KUKA_JOINT_NUM];
	float** B;									//!< Inertia matrix
	float** C;									//!< Coriolis and centrifula terms
	float g[KUKA_JOINT_NUM];							//!< Gravity vector
	float f[KUKA_JOINT_NUM];							//!< Friction vector
	float tau[KUKA_JOINT_NUM];						//!< Torque vector
	float CoM[SPACE_DIM];
	std::memset(tipDynParams, 0x0, DYN_PARAMS_NUM * sizeof(float));

	// Initialize dynamic matrices of B and C
	B = new float*[KUKA_JOINT_NUM];
	C = new float*[KUKA_JOINT_NUM];
	for (int i = 0; i < KUKA_JOINT_NUM; i++) B[i] = new float[KUKA_JOINT_NUM];
	for (int i = 0; i < KUKA_JOINT_NUM; i++) C[i] = new float[KUKA_JOINT_NUM];

	// Assign the vector of dynamic parametrs of the tool with the known data
	// Weight
	tipDynParams[0] = this->computeTotalEEWeight();	// mass [Kg]

	// CoM
	this->computeOverallEECoM(CoM); // CoM [m]	
	std::memcpy(&(*(tipDynParams + 1)), CoM, SPACE_DIM * sizeof(float));

	// Convert Eigen structures to float* arrays
	std::memcpy(jointFloatPosition, this->jointMsrPosition__.data() , KUKA_JOINT_NUM * sizeof(float));
	std::memcpy(jointFloatVelocity, this->jointMsrVelocity__.data(), KUKA_JOINT_NUM * sizeof(float));
	std::memcpy(jointFloatAcceleration, this->jointAcceleration__.data(), KUKA_JOINT_NUM * sizeof(float));

	
	// Get the dynamic parameters of the robot (mass and coriolis matrices, gravity vector, torques and frictions)
	dyn.get_g(g, jointFloatPosition, (this->isDynModelWithTool() ? (tipDynParams) : NULL));
	dyn.get_B(B, jointFloatPosition, (this->isDynModelWithTool() ? (tipDynParams) : NULL));
	dyn.get_S(C, jointFloatPosition, jointFloatVelocity, (this->isDynModelWithTool() ? (tipDynParams) : NULL));
	dyn.get_friction(f, jointFloatVelocity);

	// Conver to Eigen structures
	std::memcpy(robDynPars.g.data(), g, KUKA_JOINT_NUM * sizeof(float));
	std::memcpy(robDynPars.f.data(), f, KUKA_JOINT_NUM * sizeof(float));
	//std::memcpy(robDynPars.tau.data(), tau, KUKA_JOINT_NUM * sizeof(float));
	
	for (int i = 0; i < KUKA_JOINT_NUM; i++) {
		for (int j = 0; j < KUKA_JOINT_NUM; j++) {
			robDynPars.B(i, j) = B[i][j];
			robDynPars.C(i, j) = C[i][j];
		}
	}

	// Delete dynamic matrices
	for (int i = 0; i < KUKA_JOINT_NUM; i++) delete B[i];
	for (int i = 0; i < KUKA_JOINT_NUM; i++) delete C[i];
	delete B;
	delete C;


	// Why? vvv
	robDynPars.C(END_EFFECTOR, END_EFFECTOR) = 0.0;


}
#endif // BUILD_KUKA_DYN_MODEL

/**
* @brief Delta-friction computation function
* Compute the delta-friction contribution with the known joint positions and parameters of estimated the delta-dynamics
* @param qdot_k: the joint position vector
* @param qdot_k: the joint velocity vector
* @param fg: the estimated friction/gravity parameters
* @return the resulting delta friction torques
*/
Eigen::Vector7d KUKARobot::computeDeltaFriction(const Eigen::Vector7d& q_k, const Eigen::Vector7d& qdot_k, const Eigen::VectorXd& fg) {

	Eigen::Vector7d dtau_f;
	dtau_f.setZero();

	// The vector of the estimating parameters of the delta-dynamics should be arranged as follows
	// fg = [Fs1, Fv1, sigK1, ..., Fs7, Fv7, sigK7, mx1, my1, mz1, ... , mx7, my7, mz7, m1, ..., m7]^T

	for (int i = 0; i < KUKA_JOINT_NUM; i++) {

		double Fs = fg(i*frictionParamsNum + 0);
		double Fv = fg(i*frictionParamsNum + 1);
		double ki = fg(i*frictionParamsNum + 2);
		double expDen_k = (1.0 + exp(-ki * qdot_k(i)));
		dtau_f(i) = (Fs + Fv / expDen_k);

		if (frictionParamsNum == 4) {
			double Fq = fg(i*frictionParamsNum + 3);
			dtau_f(i) += Fq * q_k(i);
		}


	}


	return dtau_f;
}


/**
* @brief Delta-dynamics callback function
* The function computes the delta-dynamics of the robot model, from the current joint positions
* velocities and estimated dynamics parameters. The firm of the function needs to match the
* callback firm to be called from the Gauss-Newton estimation algorithm, in the related estimation task
* @param q: joint position data
* @param qdot: joint velocity data
* @param params: joint estimated friction parameters data
* @param fricParamsNum: the number of coefficients used for friction for each joint
* @param if params accounts gravity term or not
*/
Eigen::VectorXd KUKARobot::computeDeltaDynamics(
	const Eigen::VectorXd& q, 
	const Eigen::VectorXd& qdot, 
	const Eigen::VectorXd& params){

	Eigen::VectorXd deltaDyn;
	Eigen::VectorXd delta_friction;
	Eigen::VectorXd delta_gravity;
	int N, S, M;

	// Get the data sizes
	N = frictionParamsNum;
	S = (N * KUKA_JOINT_NUM);
	M = (KUKA_JOINT_NUM);
	//S = (withDeltaGravity) ? (N * JOINT_NUM + GRAVITY_DYN_PARAMS_NUM) : (N * JOINT_NUM);
	//M = (withDeltaGravity) ? (2 * JOINT_NUM) : (JOINT_NUM);
	deltaDyn.setZero(M);

	// Compute the delta-friction
	delta_friction = KUKARobot::computeDeltaFriction(q, qdot, params);

	// Compute the delta-gravity
	//delta_gravity = KUKARobot::computeDeltaGravity(q, params);

	//Fill the delta-dynamics vector
	//deltaDyn.topRows(JOINT_NUM) = delta_friction;
	deltaDyn = delta_friction;
	
	/*if (withDeltaGravity) {
		deltaDyn.bottomRows(JOINT_NUM) = delta_gravity;
	}//*/

	// Return the vector
	return deltaDyn;

}

/**
* @brief Delta-dynamics callback function
* The function computes the Jacobian matrix of the delta-dynamics of the robot model, from the current joint positions
* velocities and estimated dynamics parameters. The firm of the function needs to match the
* callback firm to be called from the Gauss-Newton estimation algorithm, in the related estimation task
* @param q: joint position data
* @param q: joint velocity data
* @param q: joint estimated friction parameters data
* @return the jacobian matrix
*/
Eigen::MatrixXd KUKARobot::computeDeltaDynJacobian(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& params) {

	Eigen::MatrixXd Jacobian, Jf, Jg;
	int N, S, M;

	// The vector of the estimating parameters of the delta-dynamics should be arranged as follows
	// fg = [Fs1, Fv1, sigK1, ..., Fs7, Fv7, sigK7, mx1, my1, mz1, ... , mx7, my7, mz7, m1, ..., m7]^T (size = 49)
	int fricOffset = frictionParamsNum * KUKA_JOINT_NUM; //<-- TODO: from file
	//int mxyzNum = SPACE_DIM * JOINT_NUM; //<-- TODO: from file
	//float g0 = -9.81; // [m/s^2]

	N = frictionParamsNum;
	S = (N * KUKA_JOINT_NUM);
	M = (KUKA_JOINT_NUM);

	//S = (withDeltaGravity) ? (N * JOINT_NUM + GRAVITY_DYN_PARAMS_NUM) : (N * JOINT_NUM);
	//M = (withDeltaGravity) ? (2 * JOINT_NUM) : (JOINT_NUM);

	Jacobian.setZero(M, S);
	Jf.setZero(KUKA_JOINT_NUM, N * KUKA_JOINT_NUM);
	//Jg.setZero(JOINT_NUM, GRAVITY_DYN_PARAMS_NUM);

	// Fill Jacobian for friction part	
	for (int i = 0; i < KUKA_JOINT_NUM; i++) {
		double Fv_i = params(N*i + 1);
		double Ki_i = params(N*i + 2);
		double expDen_i = (1.0 + exp(-Ki_i * qdot(i)));

		Jf(i, N*i + 0) = 1.0;
		Jf(i, N*i + 1) = 1.0 / expDen_i;
		Jf(i, N*i + 2) = Fv_i * qdot(i) * exp(-Ki_i*qdot(i)) / pow(expDen_i, 2);

		if (N == 4) {
			Jf(i, N*i + 3) = q(i);
		}
	}

	// Fill Jacobian for gravity part
	//Jg = KUKARobot::computeDeltaGravityJacobian(q, params);

	// Fill the Jacobian matrix
	//Jacobian.block(0,0,JOINT_NUM, N * JOINT_NUM) = Jf;
	//Jacobian.block(JOINT_NUM, N * JOINT_NUM, JOINT_NUM, GRAVITY_DYN_PARAMS_NUM) = Jg;
	Jacobian = Jf;

	// Return the jacobian matrix
	return Jacobian;

}


/**
* @brief Delta-gravity Jacobian computation function
* Compute the delta-gravity jacobian contribution with the known joint positions, velocities and parameters of estimated the delta-dynamics
* @param q_k: the joint position vector
* @param fg: the estimated friction/gravity parameters
* @return the resulting delta gravity Jacobian matrix
*/
/*Eigen::MatrixXd KUKARobot::computeDeltaGravityJacobian(const Eigen::Vector7d& q_k, const Eigen::VectorXd& fg) {

	Eigen::MatrixXd Jg;
	Jg.setZero(JOINT_NUM, GRAVITY_DYN_PARAMS_NUM);
	float g0 = -9.81; // [m/s^2]

	float q2 = q_k(1);
	float q3 = q_k(2);
	float q4 = q_k(3);
	float q5 = q_k(4);
	float q6 = q_k(5);
	float q7 = q_k(6);

	float t2 = cos(q2);
	float t3 = sin(q2);
	float t4 = cos(q3);
	float t5 = sin(q3);
	float t6 = cos(q4);
	float t7 = sin(q4);
	float t8 = cos(q5);
	float t9 = g0*t2*t4*t7;
	float t10 = t9 - g0*t3*t6;
	float t11 = sin(q5);
	float t12 = sin(q6);
	float t13 = cos(q6);
	float t14 = g0*t2*t5*t8;
	float t15 = g0*t3*t7*t11;
	float t16 = g0*t2*t4*t6*t11;
	float t17 = sin(q7);
	float t18 = cos(q7);
	float t19 = g0*t2*t4*t7*t13;
	float t20 = g0*t2*t5*t11*t12;
	float t21 = t19 + t20 - g0*t3*t6*t13 - g0*t3*t7*t8*t12 - g0*t2*t4*t6*t8*t12;
	float t22 = g0*t2*t4*t7*(3.9E1 / 1.0E2);
	float t24 = g0*t3*(2.0 / 5.0);
	float t25 = g0*t3*t6*(3.9E1 / 1.0E2);
	float t23 = t22 - t24 - t25;
	float t26 = g0*t3*t4*t8;
	float t27 = g0*t3*t4*t11*t12;
	float t28 = g0*t3*t5*t6*t8*t12;
	float t29 = t27 + t28 - g0*t3*t5*t7*t13;
	float t30 = g0*t3*t4*t6;
	float t31 = t30 - g0*t2*t7;
	float t32 = g0*t3*t4*t6*t13;
	float t33 = g0*t2*t6*t8*t12;
	float t34 = g0*t3*t4*t7*t8*t12;
	float t35 = t32 + t33 + t34 - g0*t2*t7*t13;
	float t36 = g0*t3*t4*t6*(3.9E1 / 1.0E2);
	float t38 = g0*t2*t7*(3.9E1 / 1.0E2);
	float t37 = t36 - t38;
	float t39 = g0*t3*t4*t6*t8;
	float t40 = g0*t3*t5*t8*t12;
	float t41 = g0*t3*t4*t6*t11*t12;
	float t42 = t40 + t41 - g0*t2*t7*t11*t12;
	float t43 = g0*t2*t7*t8*t13;
	float t44 = g0*t3*t5*t11*t13;
	float t45 = t43 + t44 - g0*t2*t6*t12 - g0*t3*t4*t7*t12 - g0*t3*t4*t6*t8*t13;

	Jg(1,3) = g0*t2;
	Jg(1,5) = -g0*t3;
	Jg(1,6) = g0*t2*t4;
	Jg(1,7) = g0*t3;
	Jg(1,8) = -g0*t2*t5;
	Jg(1,9) = g0*t3*t7 + g0*t2*t4*t6;
	Jg(1,10) = -g0*t2*t5;
	Jg(1,11) = t10;
	Jg(1,12) = -g0*t2*t5*t11 + g0*t3*t7*t8 + g0*t2*t4*t6*t8;
	Jg(1,13) = t10;
	Jg(1,14) = t14 + t15 + t16;
	Jg(1,15) = -g0*t3*t6*t12 + g0*t2*t4*t7*t12 - g0*t2*t5*t11*t13 + g0*t3*t7*t8*t13 + g0*t2*t4*t6*t8*t13;
	Jg(1,16) = -t14 - t15 - t16;
	Jg(1,17) = t21;
	Jg(1,18) = -g0*t2*t5*t8*t17 - g0*t3*t7*t11*t17 - g0*t3*t6*t12*t18 - g0*t2*t4*t6*t11*t17 + g0*t2*t4*t7*t12*t18 - g0*t2*t5*t11*t13*t18 + g0*t3*t7*t8*t13*t18 + g0*t2*t4*t6*t8*t13*t18;
	Jg(1,19) = -g0*t2*t5*t8*t18 + g0*t3*t6*t12*t17 - g0*t3*t7*t11*t18 - g0*t2*t4*t6*t11*t18 - g0*t2*t4*t7*t12*t17 + g0*t2*t5*t11*t13*t17 - g0*t3*t7*t8*t13*t17 - g0*t2*t4*t6*t8*t13*t17;
	Jg(1,20) = t21;
	Jg(1,23) = g0*t3*(-2.0 / 5.0);
	Jg(1,24) = g0*t3*(-2.0 / 5.0);
	Jg(1,25) = t23;
	Jg(1,26) = t23;
	Jg(1,27) = t23;
	Jg(2,6) = -g0*t3*t5;
	Jg(2,8) = -g0*t3*t4;
	Jg(2,9) = -g0*t3*t5*t6;
	Jg(2,10) = -g0*t3*t4;
	Jg(2,11) = -g0*t3*t5*t7;
	Jg(2,12) = -g0*t3*t4*t11 - g0*t3*t5*t6*t8;
	Jg(2,13) = -g0*t3*t5*t7;
	Jg(2,14) = t26 - g0*t3*t5*t6*t11;
	Jg(2,15) = -g0*t3*t5*t7*t12 - g0*t3*t4*t11*t13 - g0*t3*t5*t6*t8*t13;
	Jg(2,16) = -t26 + g0*t3*t5*t6*t11;
	Jg(2,17) = t29;
	Jg(2,18) = -g0*t3*t4*t8*t17 + g0*t3*t5*t6*t11*t17 - g0*t3*t5*t7*t12*t18 - g0*t3*t4*t11*t13*t18 - g0*t3*t5*t6*t8*t13*t18;
	Jg(2,19) = -g0*t3*t4*t8*t18 + g0*t3*t5*t6*t11*t18 + g0*t3*t5*t7*t12*t17 + g0*t3*t4*t11*t13*t17 + g0*t3*t5*t6*t8*t13*t17;
	Jg(2,20) = t29;
	Jg(2,25) = g0*t3*t5*t7*(-3.9E1 / 1.0E2);
	Jg(2,26) = g0*t3*t5*t7*(-3.9E1 / 1.0E2);
	Jg(2,27) = g0*t3*t5*t7*(-3.9E1 / 1.0E2);
	Jg(3,9) = -g0*t2*t6 - g0*t3*t4*t7;
	Jg(3,11) = t31;
	Jg(3,12) = -g0*t2*t6*t8 - g0*t3*t4*t7*t8;
	Jg(3,13) = t31;
	Jg(3,14) = -g0*t2*t6*t11 - g0*t3*t4*t7*t11;
	Jg(3,15) = -g0*t2*t7*t12 + g0*t3*t4*t6*t12 - g0*t2*t6*t8*t13 - g0*t3*t4*t7*t8*t13;
	Jg(3,16) = g0*t2*t6*t11 + g0*t3*t4*t7*t11;
	Jg(3,17) = t35;
	Jg(3,18) = g0*t2*t6*t11*t17 - g0*t2*t7*t12*t18 + g0*t3*t4*t7*t11*t17 + g0*t3*t4*t6*t12*t18 - g0*t2*t6*t8*t13*t18 - g0*t3*t4*t7*t8*t13*t18;
	Jg(3,19) = g0*t2*t6*t11*t18 + g0*t2*t7*t12*t17 - g0*t3*t4*t6*t12*t17 + g0*t3*t4*t7*t11*t18 + g0*t2*t6*t8*t13*t17 + g0*t3*t4*t7*t8*t13*t17;
	Jg(3,20) = t35;
	Jg(3,25) = t37;
	Jg(3,26) = t37;
	Jg(3,27) = t37;
	Jg(4,12) = -g0*t3*t5*t8 + g0*t2*t7*t11 - g0*t3*t4*t6*t11;
	Jg(4,14) = t39 - g0*t2*t7*t8 - g0*t3*t5*t11;
	Jg(4,15) = -g0*t3*t5*t8*t13 + g0*t2*t7*t11*t13 - g0*t3*t4*t6*t11*t13;
	Jg(4,16) = -t39 + g0*t2*t7*t8 + g0*t3*t5*t11;
	Jg(4,17) = t42;
	Jg(4,18) = g0*t2*t7*t8*t17 + g0*t3*t5*t11*t17 - g0*t3*t4*t6*t8*t17 - g0*t3*t5*t8*t13*t18 + g0*t2*t7*t11*t13*t18 - g0*t3*t4*t6*t11*t13*t18;
	Jg(4,19) = g0*t2*t7*t8*t18 + g0*t3*t5*t11*t18 - g0*t3*t4*t6*t8*t18 + g0*t3*t5*t8*t13*t17 - g0*t2*t7*t11*t13*t17 + g0*t3*t4*t6*t11*t13*t17;
	Jg(4,20) = t42;
	Jg(5,15) = g0*t2*t6*t13 + g0*t3*t4*t7*t13 + g0*t2*t7*t8*t12 + g0*t3*t5*t11*t12 - g0*t3*t4*t6*t8*t12;
	Jg(5,17) = t45;
	Jg(5,18) = g0*t2*t6*t13*t18 + g0*t3*t4*t7*t13*t18 + g0*t2*t7*t8*t12*t18 + g0*t3*t5*t11*t12*t18 - g0*t3*t4*t6*t8*t12*t18;
	Jg(5,19) = -g0*t2*t6*t13*t17 - g0*t3*t4*t7*t13*t17 - g0*t2*t7*t8*t12*t17 - g0*t3*t5*t11*t12*t17 + g0*t3*t4*t6*t8*t12*t17;
	Jg(5,20) = t45;
	Jg(6,18) = -g0*t3*t5*t8*t18 - g0*t2*t6*t12*t17 + g0*t2*t7*t11*t18 - g0*t3*t4*t6*t11*t18 - g0*t3*t4*t7*t12*t17 + g0*t2*t7*t8*t13*t17 + g0*t3*t5*t11*t13*t17 - g0*t3*t4*t6*t8*t13*t17;
	Jg(6,19) = g0*t3*t5*t8*t17 - g0*t2*t7*t11*t17 - g0*t2*t6*t12*t18 + g0*t3*t4*t6*t11*t17 - g0*t3*t4*t7*t12*t18 + g0*t2*t7*t8*t13*t18 + g0*t3*t5*t11*t13*t18 - g0*t3*t4*t6*t8*t13*t18;

	return Jg;
}//*/

/**
* @brief Delta-gravity computation function
* Compute the delta-gravity contribution with the known joint positions, velocities and parameters of estimated the delta-dynamics
* @param q_k: the joint position vector
* @param fg: the estimated friction/gravity parameters
* @return the resulting delta gravity torques
*/
/*Eigen::VectorXd KUKARobot::computeDeltaGravity(const Eigen::Vector7d& q_k, const Eigen::VectorXd& fg){

	Eigen::VectorXd dg;
	dg.setZero(JOINT_NUM);

	// The vector of the estimating parameters of the delta-dynamics should be arranged as follows
	// fg = [Fs1, Fv1, sigK1, ..., Fs7, Fv7, sigK7, mx1, my1, mz1, ... , mx7, my7, mz7, m1, ..., m7]^T (size = 49)
	int fricOffset = frictionParamsNum * JOINT_NUM; //<-- TODO: from file
	int mxyzNum = SPACE_DIM * JOINT_NUM; //<-- TODO: from file
	float g0 = -9.81; // [m/s^2]

	float MX2 = fg(fricOffset + 3);
	float MZ2 = fg(fricOffset + 5);
	float MX3 = fg(fricOffset + 6);
	float MY3 = fg(fricOffset + 7);
	float MZ3 = fg(fricOffset + 8);

	float MX4 = fg(fricOffset + 9);
	float MY4 = fg(fricOffset + 10);
	float MZ4 = fg(fricOffset + 11);

	float MX5 = fg(fricOffset + 12);
	float MY5 = fg(fricOffset + 13);
	float MZ5 = fg(fricOffset + 14);

	float MX6 = fg(fricOffset + 15);
	float MY6 = fg(fricOffset + 16);
	float MZ6 = fg(fricOffset + 17);

	float MX7 = fg(fricOffset + 18);
	float MY7 = fg(fricOffset + 19);
	float MZ7 = fg(fricOffset + 20);

	float m3 = fg(fricOffset + mxyzNum + 2);
	float m4 = fg(fricOffset + mxyzNum + 3);
	float m5 = fg(fricOffset + mxyzNum + 4);
	float m6 = fg(fricOffset + mxyzNum + 5);
	float m7 = fg(fricOffset + mxyzNum + 6);

	// Build the delta gravity
	float q2 = q_k(1);
	float q3 = q_k(2);
	float q4 = q_k(3);
	float q5 = q_k(4);
	float q6 = q_k(5);
	float q7 = q_k(6);
	float t2 = sin(q2);
	float t3 = cos(q2);
	float t4 = sin(q3);
	float t5 = cos(q4);
	float t6 = cos(q3);
	float t7 = sin(q4);
	float t8 = cos(q5);
	float t9 = cos(q6);
	float t10 = sin(q5);
	float t11 = sin(q6);
	float t12 = cos(q7);
	float t13 = sin(q7);


	dg(1) = MX2*g0*t3 + MY3*g0*t2 - MZ2*g0*t2 - g0*m3*t2*(2.0 / 5.0) - g0*m4*t2*(2.0 / 5.0) - g0*m5*t2*(2.0 / 5.0) - g0*m6*t2*(2.0 / 5.0) - g0*m7*t2*(2.0 / 5.0) - g0*m5*t2*t5*(3.9E1 / 1.0E2) - g0*m6*t2*t5*(3.9E1 / 1.0E2) - g0*m7*t2*t5*(3.9E1 / 1.0E2) + MX3*g0*t3*t6 + MX4*g0*t2*t7 - MY4*g0*t3*t4 - MY5*g0*t2*t5 - MZ3*g0*t3*t4 - MZ4*g0*t2*t5 + MX4*g0*t3*t5*t6 + MX5*g0*t2*t7*t8 - MX5*g0*t3*t4*t10 - MX6*g0*t2*t5*t11 + MY5*g0*t3*t6*t7 - MY6*g0*t3*t4*t8 - MY6*g0*t2*t7*t10 + MZ4*g0*t3*t6*t7 + MZ5*g0*t3*t4*t8 - MZ6*g0*t2*t5*t9 - MZ7*g0*t2*t5*t9 + MZ5*g0*t2*t7*t10 + g0*m5*t3*t6*t7*(3.9E1 / 1.0E2) + g0*m6*t3*t6*t7*(3.9E1 / 1.0E2) + g0*m7*t3*t6*t7*(3.9E1 / 1.0E2) + MX5*g0*t3*t5*t6*t8 + MX6*g0*t2*t7*t8*t9 - MX6*g0*t3*t4*t9*t10 + MX6*g0*t3*t6*t7*t11 - MX7*g0*t3*t4*t8*t13 - MX7*g0*t2*t5*t11*t12 - MX7*g0*t2*t7*t10*t13 - MY6*g0*t3*t5*t6*t10 - MY7*g0*t3*t4*t8*t12 + MY7*g0*t2*t5*t11*t13 - MY7*g0*t2*t7*t10*t12 + MZ5*g0*t3*t5*t6*t10 + MZ6*g0*t3*t6*t7*t9 + MZ7*g0*t3*t6*t7*t9 - MZ6*g0*t2*t7*t8*t11 + MZ6*g0*t3*t4*t10*t11 - MZ7*g0*t2*t7*t8*t11 + MZ7*g0*t3*t4*t10*t11 + MX6*g0*t3*t5*t6*t8*t9 - MX7*g0*t3*t5*t6*t10*t13 + MX7*g0*t2*t7*t8*t9*t12 - MX7*g0*t3*t4*t9*t10*t12 + MX7*g0*t3*t6*t7*t11*t12 - MY7*g0*t3*t5*t6*t10*t12 - MY7*g0*t2*t7*t8*t9*t13 + MY7*g0*t3*t4*t9*t10*t13 - MY7*g0*t3*t6*t7*t11*t13 - MZ6*g0*t3*t5*t6*t8*t11 - MZ7*g0*t3*t5*t6*t8*t11 + MX7*g0*t3*t5*t6*t8*t9*t12 - MY7*g0*t3*t5*t6*t8*t9*t13;
	dg(2) = -MX3*g0*t2*t4 - MY4*g0*t2*t6 - MZ3*g0*t2*t6 - MX4*g0*t2*t4*t5 - MX5*g0*t2*t6*t10 - MY5*g0*t2*t4*t7 - MY6*g0*t2*t6*t8 - MZ4*g0*t2*t4*t7 + MZ5*g0*t2*t6*t8 - g0*m5*t2*t4*t7*(3.9E1 / 1.0E2) - g0*m6*t2*t4*t7*(3.9E1 / 1.0E2) - g0*m7*t2*t4*t7*(3.9E1 / 1.0E2) - MX5*g0*t2*t4*t5*t8 - MX6*g0*t2*t4*t7*t11 - MX6*g0*t2*t6*t9*t10 - MX7*g0*t2*t6*t8*t13 + MY6*g0*t2*t4*t5*t10 - MY7*g0*t2*t6*t8*t12 - MZ5*g0*t2*t4*t5*t10 - MZ6*g0*t2*t4*t7*t9 - MZ7*g0*t2*t4*t7*t9 + MZ6*g0*t2*t6*t10*t11 + MZ7*g0*t2*t6*t10*t11 - MX6*g0*t2*t4*t5*t8*t9 + MX7*g0*t2*t4*t5*t10*t13 - MX7*g0*t2*t4*t7*t11*t12 - MX7*g0*t2*t6*t9*t10*t12 + MY7*g0*t2*t4*t5*t10*t12 + MY7*g0*t2*t4*t7*t11*t13 + MY7*g0*t2*t6*t9*t10*t13 + MZ6*g0*t2*t4*t5*t8*t11 + MZ7*g0*t2*t4*t5*t8*t11 - MX7*g0*t2*t4*t5*t8*t9*t12 + MY7*g0*t2*t4*t5*t8*t9*t13;
	dg(3) = g0*m5*t3*t7*(-3.9E1 / 1.0E2) - g0*m6*t3*t7*(3.9E1 / 1.0E2) - g0*m7*t3*t7*(3.9E1 / 1.0E2) - MX4*g0*t3*t5 - MY5*g0*t3*t7 - MZ4*g0*t3*t7 - MX4*g0*t2*t6*t7 - MX5*g0*t3*t5*t8 - MX6*g0*t3*t7*t11 + MY5*g0*t2*t5*t6 + MY6*g0*t3*t5*t10 + MZ4*g0*t2*t5*t6 - MZ5*g0*t3*t5*t10 - MZ6*g0*t3*t7*t9 - MZ7*g0*t3*t7*t9 + g0*m5*t2*t5*t6*(3.9E1 / 1.0E2) + g0*m6*t2*t5*t6*(3.9E1 / 1.0E2) + g0*m7*t2*t5*t6*(3.9E1 / 1.0E2) - MX5*g0*t2*t6*t7*t8 + MX6*g0*t2*t5*t6*t11 - MX6*g0*t3*t5*t8*t9 + MX7*g0*t3*t5*t10*t13 - MX7*g0*t3*t7*t11*t12 + MY6*g0*t2*t6*t7*t10 + MY7*g0*t3*t5*t10*t12 + MY7*g0*t3*t7*t11*t13 + MZ6*g0*t2*t5*t6*t9 + MZ7*g0*t2*t5*t6*t9 - MZ5*g0*t2*t6*t7*t10 + MZ6*g0*t3*t5*t8*t11 + MZ7*g0*t3*t5*t8*t11 - MX6*g0*t2*t6*t7*t8*t9 + MX7*g0*t2*t5*t6*t11*t12 - MX7*g0*t3*t5*t8*t9*t12 + MX7*g0*t2*t6*t7*t10*t13 - MY7*g0*t2*t5*t6*t11*t13 + MY7*g0*t2*t6*t7*t10*t12 + MY7*g0*t3*t5*t8*t9*t13 + MZ6*g0*t2*t6*t7*t8*t11 + MZ7*g0*t2*t6*t7*t8*t11 - MX7*g0*t2*t6*t7*t8*t9*t12 + MY7*g0*t2*t6*t7*t8*t9*t13;
	dg(4) = -MX5*g0*t2*t4*t8 + MX5*g0*t3*t7*t10 + MY6*g0*t2*t4*t10 + MY6*g0*t3*t7*t8 - MZ5*g0*t2*t4*t10 - MZ5*g0*t3*t7*t8 - MX5*g0*t2*t5*t6*t10 - MX6*g0*t2*t4*t8*t9 + MX6*g0*t3*t7*t9*t10 + MX7*g0*t2*t4*t10*t13 + MX7*g0*t3*t7*t8*t13 - MY6*g0*t2*t5*t6*t8 + MY7*g0*t2*t4*t10*t12 + MY7*g0*t3*t7*t8*t12 + MZ5*g0*t2*t5*t6*t8 + MZ6*g0*t2*t4*t8*t11 + MZ7*g0*t2*t4*t8*t11 - MZ6*g0*t3*t7*t10*t11 - MZ7*g0*t3*t7*t10*t11 - MX6*g0*t2*t5*t6*t9*t10 - MX7*g0*t2*t5*t6*t8*t13 - MX7*g0*t2*t4*t8*t9*t12 + MX7*g0*t3*t7*t9*t10*t12 - MY7*g0*t2*t5*t6*t8*t12 + MY7*g0*t2*t4*t8*t9*t13 - MY7*g0*t3*t7*t9*t10*t13 + MZ6*g0*t2*t5*t6*t10*t11 + MZ7*g0*t2*t5*t6*t10*t11 - MX7*g0*t2*t5*t6*t9*t10*t12 + MY7*g0*t2*t5*t6*t9*t10*t13;
	dg(5) = MX6*g0*t3*t5*t9 - MZ6*g0*t3*t5*t11 - MZ7*g0*t3*t5*t11 + MX6*g0*t2*t6*t7*t9 + MX6*g0*t2*t4*t10*t11 + MX6*g0*t3*t7*t8*t11 + MX7*g0*t3*t5*t9*t12 - MY7*g0*t3*t5*t9*t13 + MZ6*g0*t2*t4*t9*t10 - MZ6*g0*t2*t6*t7*t11 + MZ7*g0*t2*t4*t9*t10 + MZ6*g0*t3*t7*t8*t9 - MZ7*g0*t2*t6*t7*t11 + MZ7*g0*t3*t7*t8*t9 - MX6*g0*t2*t5*t6*t8*t11 + MX7*g0*t2*t6*t7*t9*t12 + MX7*g0*t2*t4*t10*t11*t12 + MX7*g0*t3*t7*t8*t11*t12 - MY7*g0*t2*t6*t7*t9*t13 - MY7*g0*t2*t4*t10*t11*t13 - MY7*g0*t3*t7*t8*t11*t13 - MZ6*g0*t2*t5*t6*t8*t9 - MZ7*g0*t2*t5*t6*t8*t9 - MX7*g0*t2*t5*t6*t8*t11*t12 + MY7*g0*t2*t5*t6*t8*t11*t13;
	dg(6) = -MX7*g0*t2*t4*t8*t12 - MX7*g0*t3*t5*t11*t13 + MX7*g0*t3*t7*t10*t12 + MY7*g0*t2*t4*t8*t13 - MY7*g0*t3*t5*t11*t12 - MY7*g0*t3*t7*t10*t13 - MX7*g0*t2*t5*t6*t10*t12 + MX7*g0*t2*t4*t9*t10*t13 - MX7*g0*t2*t6*t7*t11*t13 + MX7*g0*t3*t7*t8*t9*t13 + MY7*g0*t2*t5*t6*t10*t13 + MY7*g0*t2*t4*t9*t10*t12 - MY7*g0*t2*t6*t7*t11*t12 + MY7*g0*t3*t7*t8*t9*t12 - MX7*g0*t2*t5*t6*t8*t9*t13 - MY7*g0*t2*t5*t6*t8*t9*t12;

	return dg;
}//*/




/**
* @brief Jacobian function
* Compute the linear Jacobian matrix for the given chosen link (end-effector by default).
* @param link the link of which the Jacobian matrix has to be computed
* @return the requested Jacobian matrix
*/
Eigen::MatrixXf KUKARobot::computeLinearJacobian(const int& link) {

	Eigen::MatrixXf Jl(SPACE_DIM, KUKA_JOINT_NUM);
	
	float tool0_s[SPACE_DIM];
	this->tools[0].getSize(tool0_s);

	// Define utility variables
	float C1, S1, C2, S2, C3, S3, C4, S4, C5, S5, C6, S6;
	float l2 = this->linkLengths__(LINK2);
	float l3 = this->linkLengths__(LINK3);
	float l4 = this->linkLengths__(LINK4);
	float l5 = this->linkLengths__(LINK5);
	float l6 = this->linkLengths__(LINK6) +(float)this->computeEEOffset();
	//float l6 = this->linkLengths__(LINK6) + tool0_s[Z_AXIS];
	
	// Pre-compute sin and cos
	C1 = cos(jointMsrPosition__(0));
	S1 = sin(jointMsrPosition__(0));
	C2 = cos(jointMsrPosition__(1));
	S2 = sin(jointMsrPosition__(1));
	C3 = cos(jointMsrPosition__(2));
	S3 = sin(jointMsrPosition__(2));
	C4 = cos(jointMsrPosition__(3));
	S4 = sin(jointMsrPosition__(3));
	C5 = cos(jointMsrPosition__(4));
	S5 = sin(jointMsrPosition__(4));
	C6 = cos(jointMsrPosition__(5));
	S6 = sin(jointMsrPosition__(5));

	if (link == LINK2)
	{
		Jl(0, 0) = 0;
		Jl(1, 0) = 0;
		Jl(2, 0) = 0;

		Jl(0, 1) = 0;
		Jl(1, 1) = 0;
		Jl(2, 1) = 0;
	}

	else if (link == LINK3)
	{
		Jl(0, 0) = (l2 + l3)*S1*S2;
		Jl(1, 0) = -(l2 + l3)*C1*S2;
		Jl(2, 0) = 0;

		Jl(0, 1) = -(l2 + l3)*C1*C2;
		Jl(1, 1) = -(l2 + l3)*S1*C2;
		Jl(2, 1) = -(l2 + l3)*S2;

		Jl(0, 2) = 0;
		Jl(1, 2) = 0;
		Jl(2, 2) = 0;
	}

	else if (link == LINK4)
	{
		Jl(0, 0) = (l2 + l3)*S1*S2;
		Jl(1, 0) = -(l2 + l3)*C1*S2;
		Jl(2, 0) = 0;

		Jl(0, 1) = -(l2 + l3)*C1*C2;
		Jl(1, 1) = -(l2 + l3)*S1*C2;
		Jl(2, 1) = -(l2 + l3)*S2;

		Jl(0, 2) = 0;
		Jl(1, 2) = 0;
		Jl(2, 2) = 0;

		Jl(0, 3) = 0;
		Jl(1, 3) = 0;
		Jl(2, 3) = 0;
	}

	else if (link == LINK5)
	{
		Jl(0, 0) = -(l4 + l5)*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) + (l2 + l3)*S1*S2;
		Jl(1, 0) = -(l4 + l5)*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) - (l2 + l3)*C1*S2;
		Jl(2, 0) = 0;

		Jl(0, 1) = -C1*((l4 + l5)*(C2*C4 + C3*S2*S4) + (l2 + l3)*C2);
		Jl(1, 1) = -S1*((l4 + l5)*(C2*C4 + C3*S2*S4) + (l2 + l3)*C2);
		Jl(2, 1) = -(l2 + l3)*S2 - (l4 + l5)*C4*S2 + (l4 + l5)*C2*C3*S4;

		Jl(0, 2) = -(l4 + l5)*S4*(C3*S1 + C1*C2*S3);
		Jl(1, 2) = (l4 + l5)*S4*(C1*C3 - C2*S1*S3);
		Jl(2, 2) = -(l4 + l5)*S2*S3*S4;

		Jl(0, 3) = (l4 + l5)*C1*S2*S4 - (l4 + l5)*C4*S1*S3 + (l4 + l5)*C1*C2*C3*C4;
		Jl(1, 3) = (l4 + l5)*S1*S2*S4 + (l4 + l5)*C1*C4*S3 + (l4 + l5)*C2*C3*C4*S1;
		Jl(2, 3) = -(l4 + l5)*C2*S4 + (l4 + l5)*C3*C4*S2;

		Jl(0, 4) = 0;
		Jl(1, 4) = 0;
		Jl(2, 4) = 0;
	}

	else if (link == LINK6)
	{
		Jl(0, 0) = -(l4 + l5)*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) + (l2 + l3)*S1*S2;
		Jl(1, 0) = -(l4 + l5)*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) - (l2 + l3)*C1*S2;
		Jl(2, 0) = 0;

		Jl(0, 1) = -C1*((l4 + l5)*(C2*C4 + C3*S2*S4) + (l2 + l3)*C2);
		Jl(1, 1) = -S1*((l4 + l5)*(C2*C4 + C3*S2*S4) + (l2 + l3)*C2);
		Jl(2, 1) = -(l2 + l3)*S2 - (l4 + l5)*C4*S2 + (l4 + l5)*C2*C3*S4;

		Jl(0, 2) = -(l4 + l5)*S4*(C3*S1 + C1*C2*S3);
		Jl(1, 2) = (l4 + l5)*S4*(C1*C3 - C2*S1*S3);
		Jl(2, 2) = -(l4 + l5)*S2*S3*S4;

		Jl(0, 3) = (l4 + l5)*C1*S2*S4 - (l4 + l5)*C4*S1*S3 + (l4 + l5)*C1*C2*C3*C4;
		Jl(1, 3) = (l4 + l5)*S1*S2*S4 + (l4 + l5)*C1*C4*S3 + (l4 + l5)*C2*C3*C4*S1;
		Jl(2, 3) = -(l4 + l5)*C2*S4 + (l4 + l5)*C3*C4*S2;

		Jl(0, 4) = 0;
		Jl(1, 4) = 0;
		Jl(2, 4) = 0;

		Jl(0, 5) = 0;
		Jl(1, 5) = 0;
		Jl(2, 5) = 0;
	}

	else if (link == END_EFFECTOR)
	{
		Jl(0, 0) = -(l4 + l5)*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) - l6*(C6*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) - S6*(C5*(C4*(C1*S3 + C2*C3*S1) + S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) + (l2 + l3)*S1*S2;
		Jl(1, 0) = -(l4 + l5)*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) - l6*(C6*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) - S6*(C5*(C4*(S1*S3 - C1*C2*C3) - C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - (l2 + l3)*C1*S2;
		Jl(2, 0) = 0;

		Jl(0, 1) = -l6*C1*(S6*(C5*(C2*S4 - C3*C4*S2) + S2*S3*S5) + C6*(C2*C4 + C3*S2*S4)) - (l2 + l3)*C1*C2 - (l4 + l5)*C1*(C2*C4 + C3*S2*S4);
		Jl(1, 1) = -l6*S1*(S6*(C5*(C2*S4 - C3*C4*S2) + S2*S3*S5) + C6*(C2*C4 + C3*S2*S4)) - (l2 + l3)*C2*S1 - (l4 + l5)*S1*(C2*C4 + C3*S2*S4);
		Jl(2, 1) = l6*C2*S3*S5*S6 - (l4 + l5)*C4*S2 + (l4 + l5)*C2*C3*S4 - l6*C4*C6*S2 + l6*C2*C3*C6*S4 - (l2 + l3)*S2 - l6*C5*S2*S4*S6 - l6*C2*C3*C4*C5*S6;

		Jl(0, 2) = -(l4 + l5)*C3*S1*S4 - (l4 + l5)*C1*C2*S3*S4 + l6*C3*C6*S1*S4 - l6*S1*S3*S5*S6 - l6*C1*C2*C6*S3*S4 + l6*C1*C2*C3*S5*S6 + l6*C3*C4*C5*S1*S6 + l6*C1*C2*C4*C5*S3*S6;
		Jl(1, 2) = -(l4 + l5)*C2*S1*S3*S4 + l6*C1*C3*C6*S4 + (l4 + l5)*C1*C3*S4 + l6*C1*S3*S5*S6 - l6*C1*C3*C4*C5*S6 - l6*C2*C6*S1*S3*S4 + l6*C2*C3*S1*S5*S6 + l6*C2*C4*C5*S1*S3*S6;
		Jl(2, 2) = -S2*((l4 + l5)*S3*S4 + l6*C6*S3*S4 - l6*C3*S5*S6 - l6*C4*C5*S3*S6);

		Jl(0, 3) = (l4 + l5)*C1*S2*S4 - (l4 + l5)*C4*S1*S3 + (l4 + l5)*C1*C2*C3*C4 + l6*C1*C6*S2*S4 - l6*C4*C6*S1*S3 - l6*C1*C4*C5*S2*S6 - l6*C5*S1*S3*S4*S6 + l6*C1*C2*C3*C4*C6 + l6*C1*C2*C3*C5*S4*S6;
		Jl(1, 3) = (l4 + l5)*S1*S2*S4 + (l4 + l5)*C1*C4*S3 + (l4 + l5)*C2*C3*C4*S1 + l6*C1*C4*C6*S3 + l6*C6*S1*S2*S4 + l6*C2*C3*C4*C6*S1 - l6*C4*C5*S1*S2*S6 + l6*C1*C5*S3*S4*S6 + l6*C2*C3*C5*S1*S4*S6;
		Jl(2, 3) = l6*C3*C5*S2*S4*S6 + (l4 + l5)*C3*C4*S2 - l6*C2*C6*S4 + l6*C3*C4*C6*S2 + l6*C2*C4*C5*S6 - (l4 + l5)*C2*S4;

		Jl(0, 4) = l6*S6*(C3*C5*S1 + C1*C2*C5*S3 + C1*S2*S4*S5 - C4*S1*S3*S5 + C1*C2*C3*C4*S5);
		Jl(1, 4) = l6*S6*(C2*C5*S1*S3 - C1*C3*C5 + C1*C4*S3*S5 + S1*S2*S4*S5 + C2*C3*C4*S1*S5);
		Jl(2, 4) = l6*S6*(C5*S2*S3 - C2*S4*S5 + C3*C4*S2*S5);

		Jl(0, 5) = l6*C1*C4*S2*S6 + l6*C3*C6*S1*S5 + l6*S1*S3*S4*S6 - l6*C1*C2*C3*S4*S6 + l6*C1*C2*C6*S3*S5 - l6*C1*C5*C6*S2*S4 + l6*C4*C5*C6*S1*S3 - l6*C1*C2*C3*C4*C5*C6;
		Jl(1, 5) = l6*C4*S1*S2*S6 - l6*C1*C3*C6*S5 - l6*C1*S3*S4*S6 - l6*C1*C4*C5*C6*S3 - l6*C2*C3*S1*S4*S6 + l6*C2*C6*S1*S3*S5 - l6*C5*C6*S1*S2*S4 - l6*C2*C3*C4*C5*C6*S1;
		Jl(2, 5) = -l6*C3*S2*S4*S6 + l6*C2*C5*C6*S4 - l6*C2*C4*S6 + l6*C6*S2*S3*S5 - l6*C3*C4*C5*C6*S2;

		Jl(0, 6) = 0;
		Jl(1, 6) = 0;
		Jl(2, 6) = 0;
	}

	return Jl;

}

/**
* @brief Jacobian function
* Compute the angular Jacobian matrix for the given chosen link (end-effector by default).
* @param link the link of which the Jacobian matrix has to be computed
* @return the requested Jacobian matrix
*/
Eigen::MatrixXf KUKARobot::computeAngularJacobian(const int& link) {

	Eigen::MatrixXf Ja(SPACE_DIM,KUKA_JOINT_NUM);

	float tool0_s[SPACE_DIM];
	this->tools[0].getSize(tool0_s);

	// Define utility variables
	float C1, S1, C2, S2, C3, S3, C4, S4, C5, S5, C6, S6;
	float l2 = this->linkLengths__(LINK2);
	float l3 = this->linkLengths__(LINK3);
	float l4 = this->linkLengths__(LINK4);
	float l5 = this->linkLengths__(LINK5);
	float l6 = this->linkLengths__(LINK6); + this->computeEEOffset();
	//float l6 = this->linkLengths__(LINK6) + tool0_s[Z_AXIS];

	// Pre-compute sin and cos
	C1 = cos(jointMsrPosition__(0));
	S1 = sin(jointMsrPosition__(0));
	C2 = cos(jointMsrPosition__(1));
	S2 = sin(jointMsrPosition__(1));
	C3 = cos(jointMsrPosition__(2));
	S3 = sin(jointMsrPosition__(2));
	C4 = cos(jointMsrPosition__(3));
	S4 = sin(jointMsrPosition__(3));
	C5 = cos(jointMsrPosition__(4));
	S5 = sin(jointMsrPosition__(4));
	C6 = cos(jointMsrPosition__(5));
	S6 = sin(jointMsrPosition__(5));

	if (link == LINK2)
	{
		Ja(0, 0) = 0;
		Ja(1, 0) = 0;
		Ja(2, 0) = 1;

		Ja(0, 1) = S1;
		Ja(1, 1) = -C1;
		Ja(2, 1) = 0;
	}

	else if (link == LINK3)
	{
		Ja(0, 0) = 0;
		Ja(1, 0) = 0;
		Ja(2, 0) = 1;

		Ja(0, 1) = S1;
		Ja(1, 1) = -C1;
		Ja(2, 1) = 0;

		Ja(0, 2) = -C1*S2;
		Ja(1, 2) = -S1*S2;
		Ja(2, 2) = C2;
	}

	else if (link == LINK4)
	{
		Ja(0, 0) = 0;
		Ja(1, 0) = 0;
		Ja(2, 0) = 1;

		Ja(0, 1) = S1;
		Ja(1, 1) = -C1;
		Ja(2, 1) = 0;

		Ja(0, 2) = -C1*S2;
		Ja(1, 2) = -S1*S2;
		Ja(2, 2) = C2;

		Ja(0, 3) = -C3*S1 - C1*C2*S3;
		Ja(1, 3) = -C2*S1*S3 + C1*C3;
		Ja(2, 3) = -S2*S3;
	}

	else if (link == LINK5)
	{
		Ja(0, 0) = 0;
		Ja(1, 0) = 0;
		Ja(2, 0) = 1;

		Ja(0, 1) = S1;
		Ja(1, 1) = -C1;
		Ja(2, 1) = 0;

		Ja(0, 2) = -C1*S2;
		Ja(1, 2) = -S1*S2;
		Ja(2, 2) = C2;

		Ja(0, 3) = -C3*S1 - C1*C2*S3;
		Ja(1, 3) = -C2*S1*S3 + C1*C3;
		Ja(2, 3) = -S2*S3;

		Ja(0, 4) = -S4*(S1*S3 - C1*C2*C3) - C1*C4*S2;
		Ja(1, 4) = S4*(C1*S3 + C2*C3*S1) - C4*S1*S2;
		Ja(2, 4) = C2*C4 + C3*S2*S4;
	}

	else if (link == LINK6)
	{
		Ja(0, 0) = 0;
		Ja(1, 0) = 0;
		Ja(2, 0) = 1;

		Ja(0, 1) = S1;
		Ja(1, 1) = -C1;
		Ja(2, 1) = 0;

		Ja(0, 2) = -C1*S2;
		Ja(1, 2) = -S1*S2;
		Ja(2, 2) = C2;

		Ja(0, 3) = -C3*S1 - C1*C2*S3;
		Ja(1, 3) = -C2*S1*S3 + C1*C3;
		Ja(2, 3) = -S2*S3;

		Ja(0, 4) = -S4*(S1*S3 - C1*C2*C3) - C1*C4*S2;
		Ja(1, 4) = S4*(C1*S3 + C2*C3*S1) - C4*S1*S2;
		Ja(2, 4) = C2*C4 + C3*S2*S4;

		Ja(0, 5) = C5*(C3*S1 + C1*C2*S3) - S5*(C4*(S1*S3 - C1*C2*C3) - C1*S2*S4);
		Ja(1, 5) = S5*(C4*(C1*S3 + C2*C3*S1) + S1*S2*S4) - C5*(C1*C3 - C2*S1*S3);
		Ja(2, 5) = -S5*(C2*S4 - C3*C4*S2) + C5*S2*S3;
	}

	else if (link == END_EFFECTOR)
	{
		Ja(0, 0) = 0;
		Ja(1, 0) = 0;
		Ja(2, 0) = 1;

		Ja(0, 1) = S1;
		Ja(1, 1) = -C1;
		Ja(2, 1) = 0;

		Ja(0, 2) = -C1*S2;
		Ja(1, 2) = -S1*S2;
		Ja(2, 2) = C2;

		Ja(0, 3) = -C3*S1 - C1*C2*S3;
		Ja(1, 3) = -C2*S1*S3 + C1*C3;
		Ja(2, 3) = -S2*S3;

		Ja(0, 4) = -S4*(S1*S3 - C1*C2*C3) - C1*C4*S2;
		Ja(1, 4) = S4*(C1*S3 + C2*C3*S1) - C4*S1*S2;
		Ja(2, 4) = C2*C4 + C3*S2*S4;

		Ja(0, 5) = C5*(C3*S1 + C1*C2*S3) - S5*(C4*(S1*S3 - C1*C2*C3) - C1*S2*S4);
		Ja(1, 5) = S5*(C4*(C1*S3 + C2*C3*S1) + S1*S2*S4) - C5*(C1*C3 - C2*S1*S3);
		Ja(2, 5) = -S5*(C2*S4 - C3*C4*S2) + C5*S2*S3;

		Ja(0, 6) = -C6*(S4*(S1*S3 - C1*C2*C3) + C1*C4*S2) + S6*(C5*(C4*(S1*S3 - C1*C2*C3) - C1*S2*S4) + S5*(C3*S1 + C1*C2*S3));
		Ja(1, 6) = C6*(S4*(C1*S3 + C2*C3*S1) - C4*S1*S2) - S6*(C5*(C4*(C1*S3 + C2*C3*S1) + S1*S2*S4) + S5*(C1*C3 - C2*S1*S3));
		Ja(2, 6) = C6*(C2*C4 + C3*S2*S4) + S6*(C5*(C2*S4 - C3*C4*S2) + S2*S3*S5);
	}


	return Ja;
}


/**
* @brief Reset data
* Reset the robot data
*/
void KUKARobot::resetDynParams() {

	// Reset the cumulative contributions of the residual
	this->dynModelSum.setZero(this->jointNum);
	this->residualSum.setZero(this->jointNum);

	// Reset the residual
	this->res__.setZero(this->jointNum);

	// Compute the new offset to be taken into account in the computation of the residual
	Eigen::VectorXf g = this->getModelDynParams().g;
	Eigen::VectorXf tau = this->getMsrJointTorques();
	Eigen::VectorXf resOff = g - tau;
	this->setResidualVectorOffset(resOff);

	// Reset FRI residual vector
	Eigen::VectorXf friResidual = this->getFRIResidual();
	this->setFRIResidualOffset(friResidual);

	//std::cout << "Robot dynamic paramters reset. " << std::endl;
}

/**
* @brief Set function
* Save the friction parameters on a file
* @param filename: the name of the file where the friction parameters have to be saved
*/
void KUKARobot::saveEstimatedFrictionParameters(const char* filename) {

	std::stringstream fpSS;
	DBWrapper db(filename);
	Eigen::VectorXf fp;

	fp = this->frictionGravityParams;
	fpSS << "# Estimated friction parameters" << std::endl;
	fpSS << "### Friction Parameters Size" << std::endl;
	fpSS << frictionParamsNum << std::endl << std::endl;
	fpSS << "### Friction Parameters Vector" << std::endl;
	for (int i = 0; i < fp.size(); i++) {
		fpSS << fp(i) << (i == fp.size() - 1 ? ";" : ",");
	}

	db.write(fpSS);

}

/**
* @brief Load function
* Load the friction parameters from a file
* @param filename: the name of the file where the friction parameters are stored
*/
void KUKARobot::loadEstimatedFrictionParameters(const char* filename) {

	DBWrapper db(filename);
	std::vector < std::pair < std::string, std::string > > content;

	// Read the file
	content = db.readLabeledFile();

	// Parse the content file
	for (int i = 0; i < content.size(); i++) {
		std::string comment = content[i].first;
		std::string value = content[i].second;

		if (comment.find("Friction Parameters Size") != std::string::npos) {
			frictionParamsNum = std::stod(value);
		}
		else if (comment.find("Friction Parameters Vector") != std::string::npos) {
			std::vector < double > vec = parseCSVLine(value);
			this->frictionGravityParams.setZero(vec.size());
			for (int j = 0; j < vec.size(); j++) {
				this->frictionGravityParams(j) = vec[j];
			}
		}
	}

	std::cout << "Friction paramters loaded = \n " << this->frictionGravityParams.transpose() << std::endl;

}


