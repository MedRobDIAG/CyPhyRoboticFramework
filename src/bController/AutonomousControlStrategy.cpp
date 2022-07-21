// Project Header files
#include "AutonomousControlStrategy.hpp"
#include "Configuration.hpp"

/**
* @brief Init function
* Initialize the control strategy
*/
void AutonomousControlStrategy::init() {

	// Initialize the initial pose and the sample time of the trajectory
	// Sample time
	float dt = this->robot->getSampleTime();
	this->traj->setSampleTime(dt);

	// Initial pose
	this->Tbee_0  = this->robot->getTbee();

}



/**
* @brief Main function
* Implements the control law associated to the ControlStrategy
*/
Eigen::VectorXf AutonomousControlStrategy::controlLaw() {

	Eigen::VectorXf qdcmd;
	Eigen::Matrix6f K;
	Eigen::Matrix3f Rbee_0;
	Eigen::Vector3f pbee_0, pb, p, pd, vd, pbee_t, vbee_t;
	Eigen::Vector6f ref, vff, vffconstr, e;
	Eigen::MatrixXf Jpinv, J, JT;
	int jointNum = robot->getJointNum();

	// Get Configuration object
	Configuration* config = Configuration::GetInstance("");
	K = Eigen::Matrix6f::Identity() * config->getAutonomousCtrlGain();

	// Initialization
	qdcmd.setZero(jointNum);
	J.setZero(SPACE_DIM * 2, jointNum);
	JT.setZero(jointNum, SPACE_DIM * 2);
	Jpinv.setZero(jointNum, SPACE_DIM * 2);
	
	// Get the initial position and rotation matrix of the ee wrt the base frame
	Rbee_0 = this->Tbee_0.topLeftCorner(SPACE_DIM, SPACE_DIM);
	pbee_0 = this->Tbee_0.block<3, 1>(0, 3);

	if (!this->traj->isTrajectoryDone()) {
		// Get the current trajectory sample
		this->traj->updateTime();
		ref = this->traj->eval();
		pd = ref.topRows(SPACE_DIM);
		vd = ref.bottomRows(SPACE_DIM);

		// Convert data in the base frame
		pbee_t = Rbee_0 * pd + pbee_0;
		vbee_t = Rbee_0 * vd;

		// Get the current end-effector position
		pb = this->robot->getEEPosition();
		p = Rbee_0 * pb + pbee_0;

		// Build the feed-foward velocity
		vff.topRows(SPACE_DIM) = vbee_t;
		vff.bottomRows(SPACE_DIM).setZero(); //<--- To be filled

		/* Warning: this admittance control here should be relocated
		* somewhere else. I temporarily put it here for the US-probe
		* demo.
		*/
		/*Eigen::Vector6f vforce;
		float f = -(this->ftsensor->getWrench().cast<float>())(Z_AXIS);
		float fd = 1.0;
		vforce.setZero();
		if (std::fabs(f) > 1e-2) {
		this->Kforce = 4e-3;
		}
		vforce(Z_AXIS) = this->Kforce * (f - fd);
		vff += vforce;//*/

		// Apply constraints
		//vffconstr = this->constraint.applyConstraint(vff);
		vffconstr = vff; // TODO: fix this

		// Build the error vector
		e.topRows(SPACE_DIM) = p - pbee_t;
		e.bottomRows(SPACE_DIM).setZero(); //<-- To be filled

		// Compute the Jacobian pseudo-inverse
		Jpinv = this->robot->getJPinv();

		// Compute the joint velocity commands from pseudo-inversion of the Jacobian
		qdcmd = Jpinv * (vffconstr - K * e);//*/

		/*Eigen::Matrix<float, 5, 7> J1;
		Eigen::Matrix<float, 7, 5> J1T, J1pinv;
		Eigen::Matrix<float, 7, 7> P1;
		Eigen::Matrix6x7f J2, J2P1;
		Eigen::Matrix7x6f J2T, J2pinv, J2P1T, J2P1pinv;
		Eigen::Vector6f e2;
		Eigen::Matrix<float, 5, 1> e1;


		e2 = e;
		J1T = J1.transpose();
		J1pinv = J1T * (J1 * J1T).inverse();
		P1 = (Eigen::Matrix7f::Identity() - J1pinv*J1);
		J2P1 = J2 * P1;
		J2P1T = J2P1.transpose();
		J2P1pinv = J2P1T * (J2P1 * J2P1T).inverse();
		qdcmd = J1pinv * (-K*e1) + P1*J2P1pinv*(-K*e2 - J2*J1pinv*(-K*e1));//*/


		/*if (this->vrepProxy->isAvailable()) {
			this->vrepProxy->setFloatSignal("vz", vff(Z_AXIS), simx_opmode_oneshot, this->simPort);
			this->vrepProxy->setFloatSignal("etrack_x", e(X_AXIS), simx_opmode_oneshot, this->simPort);
			this->vrepProxy->setFloatSignal("etrack_y", e(Y_AXIS), simx_opmode_oneshot, this->simPort);
			this->vrepProxy->setFloatSignal("etrack_z", e(Z_AXIS), simx_opmode_oneshot, this->simPort);
			//this->vrepProxy->setFloatSignal("f", f, simx_opmode_oneshot, this->simPort);
		}//*/

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
	}
	else {
		qdcmd.setZero();
	}

	return qdcmd;

}
