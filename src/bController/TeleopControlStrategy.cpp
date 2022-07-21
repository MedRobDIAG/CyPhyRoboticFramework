// Project Header files
#include "TeleopControlStrategy.hpp"
#include "Configuration.hpp"
#include "KUKARobot.hpp"

/**
* @brief Init function
* Initialize the control strategy
*/
void TeleopControlStrategy::init() {

}


/**
* @brief Main function
* Implements the control law associated to the ControlStrategy
*/
Eigen::VectorXf TeleopControlStrategy::controlLaw() {

	Eigen::VectorXf qdcmd; // commanded joint velocity vector
	Eigen::Matrix3f Rhr; // Rotation matrix expressing robot wrt haptic device
	Eigen::Matrix3f Rrbee; // Rotation matrix expressing robot ee wrt robot base
	Eigen::Vector6f fbForce; // force vector measured from the chosen source
	Eigen::Vector6f vHaptic; // 6D velocity vector in the haptic frame
	Eigen::Vector6f vRobot; // 6D velocity vector in the robot frame
	Eigen::Vector6f vRobot_constr; // 6D constrained velocity vector in the robot frame
	Eigen::Vector6f vRobotEE_constr; // 6D constrained velocity vector in the robot frame
	Eigen::MatrixXf J, Jee, JeeT, pinvJ, pinvJee; // Jacobian matrices and related transpose and pseudo-inverse
	Eigen::Vector3f masterForce; // The 3D feedback force vector to be set on the haptic device
	float lvelScale, avelScale, masterForceScale; // Velocity and force scale factors
	int jointNum = robot->getJointNum();

	// Get Configuration object
	Configuration* config = Configuration::GetInstance("");

	// Initialize the output joint velocity command vector
	qdcmd.setZero(jointNum);
	J.setZero(SPACE_DIM * 2, jointNum);
	Jee.setZero(SPACE_DIM * 2, jointNum);
	JeeT.setZero(jointNum, SPACE_DIM * 2);
	pinvJ.setZero(jointNum, SPACE_DIM * 2);
	pinvJee.setZero(jointNum, SPACE_DIM * 2);

	// Get the Master-slave rotation matrix
	Rhr = this->hapticDev->getRms();
	/* If you are not using the real robot in the typical configuration of the lab (user master console
	* *behind* the robot and you are directly commanding the simulated robot, rotate the master-slave
	* orientation of 180° wrt z axis (i.e., assume you are seing the robot from a front view).*/
	/*if (!config.isRealRobotRequested()) {
	Rhr.col(X_AXIS) *= -1;
	Rhr.col(Y_AXIS) *= -1; 
	}//*/

	// Get the rotation matrix of the robot end-effector wrt the robot base frame
	Rrbee = this->robot->getEERotMat();

	// Get the master velocity scale factor
	lvelScale = this->hapticDev->getLinVelScale();
	avelScale = this->hapticDev->getAngVelScale();

	// Get the master force scale factor
	masterForceScale = this->hapticDev->getForceScale();

	// Get EE force from the robot model if force feedback is enabled
	if (this->hapticDev->isTeleoperationWithForceFB()) {
		if (config->getCurrentFFBSrcType() == FT_SENSOR_SOURCE && this->ftsensor != nullptr) {
			fbForce = -(this->ftsensor->getWrench()).cast<float>();
		}
		else if (config->getCurrentFFBSrcType() == MODEL_RESIDUAL_SOURCE) {
			fbForce = this->robot->getExtContactForce();
		}
		else if (config->getCurrentFFBSrcType() == FRI_RESIDUAL_SOURCE && config->getRobotManipulatorType() == "KUKA") {
			fbForce = dynamic_cast<KUKARobot*>(this->robot)->getFriResFbee();
		}
	}

	// Get the velocity of the HIP in the Haptic device frame
	vHaptic = this->hapticDev->getHIPVel();

	// Rotate the linear velocity (angular optionally) in the robot frame
	vRobot.topRows(SPACE_DIM) = (Rhr.transpose()) * vHaptic.topRows(SPACE_DIM);
	vRobot.bottomRows(SPACE_DIM) = vHaptic.bottomRows(SPACE_DIM); // pre-multiply by Rhr^T if you want to rotate angular velocity as well

	// Apply the eventual kinematic constraint on the input velocity
	vRobot_constr = this->constraintMat * vRobot;

	// Scale the velocity
	vRobot_constr.topRows(SPACE_DIM) *= lvelScale;
	vRobot_constr.bottomRows(SPACE_DIM) *= -avelScale;

	// Rotate the reference velocity from base frame to end-effector frame
	vRobotEE_constr.topRows(SPACE_DIM) = Rrbee.transpose() * vRobot_constr.topRows(SPACE_DIM);
	vRobotEE_constr.bottomRows(SPACE_DIM) = Rrbee.transpose() * vRobot_constr.bottomRows(SPACE_DIM);

	// Compute the Jacobian pseudo-inverse
	//pinvJ = this->kuka->getJPinv(); // this takes directly the pseudo-inverse of the robot Jacobian in the base frame
	J = this->robot->getJacobian();
	Jee.topRows(SPACE_DIM) = Rrbee.transpose() * J.topRows(SPACE_DIM);
	Jee.bottomRows(SPACE_DIM) = Rrbee.transpose() * J.bottomRows(SPACE_DIM);
	JeeT = Jee.transpose();
	pinvJee = JeeT * (Jee * JeeT).inverse(); // this is the pseudo-inverse of the robot Jacobian in the ee frame

	if (this->hapticDev->isClutchActive()) {
		// Compute the joint velocity commands from pseudo-inversion of the Jacobian
		//qdcmd = pinvJ * vRobot_constr;
		qdcmd = pinvJee * vRobotEE_constr;
	}
	else {
		qdcmd.setZero();
	}

	// Singularity check
	bool singular = false;
	int i = 0;
	while (i < jointNum && !singular) {
		if (isnan(qdcmd(i))) {
			qdcmd.setZero();
			singular = true;
		}
		i++;
	}

	// Compute the force feedback to set on the Geomagic Proxy
	if (this->hapticDev->isTeleoperationWithForceFB() && this->hapticDev->isClutchActive()) {
	masterForce = Rhr * fbForce.topRows(SPACE_DIM) * masterForceScale;
	}
	else {
	masterForce.setZero();
	}

	// Set the master force on the device
	this->hapticDev->setForceFeedback(masterForce);

	return qdcmd;

}
