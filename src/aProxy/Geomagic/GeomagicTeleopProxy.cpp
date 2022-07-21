// Project Header files
#include "GeomagicTeleopProxy.hpp"

hduVector3Dd eigen2hdu3D(const Eigen::Vector3d& eig_in) {

	hduVector3Dd out;
	for (int i = 0; i < 3; i++) {
		out[i] = eig_in(i);
	}

	return out;
}


Eigen::Vector3d hdu2Eigen3D(const hduVector3Dd& hdu_in) {

	Eigen::Vector3d out;
	for (int i = 0; i < 3; i++) {
		out(i) = hdu_in[i];
	}

	return out;

}


/**
* @brief Default contructor of GeomagicTeleopProxy class
*
*/
GeomagicTeleopProxy::GeomagicTeleopProxy() {

	//this->geoTeleopState = new GeoTeleopState();
	this->state = new GeoTeleopState();
	GeoTeleopState* geoTeleopState = (GeoTeleopState*)(state);

	geoTeleopState->buttonState = new bool[GEOMAGIC_BUTTONS_NUM];

}

/**
* @brief Default destroyer of GeomagicTeleopProxy class
*
*/
GeomagicTeleopProxy::~GeomagicTeleopProxy() {
	
	GeoTeleopState* geoTeleopState = (GeoTeleopState*)(state);
	delete[] geoTeleopState->buttonState;
	delete geoTeleopState;

}


/**
* @brief Default init function
*/
void GeomagicTeleopProxy::init() {

	GeoTeleopState* geoTeleopState = (GeoTeleopState*)(state);

	// Initialize all the structures to zero
	geoTeleopState->hipPosition.setZero();
	geoTeleopState->hipGimbalAngles.setZero();
	geoTeleopState->hipVelocity.setZero();

	// Initialize variables related to the buttons state
	geoTeleopState->stylusButtons = 0;
	geoTeleopState->action[GEOMAGIC_LOW_BUTTON] = false;
	geoTeleopState->action[GEOMAGIC_HIGH_BUTTON] = false;
	geoTeleopState->evHoldButton[GEOMAGIC_LOW_BUTTON] = false;
	geoTeleopState->evHoldButton[GEOMAGIC_HIGH_BUTTON] = false;
	geoTeleopState->evRaiseEdge[GEOMAGIC_LOW_BUTTON] = false;
	geoTeleopState->evRaiseEdge[GEOMAGIC_HIGH_BUTTON] = false;
	geoTeleopState->evTrailEdge[GEOMAGIC_LOW_BUTTON] = false;
	geoTeleopState->evTrailEdge[GEOMAGIC_HIGH_BUTTON] = false;

	// Utility data
	this->hdUtils.prvPos.set(0.0, 0.0, 0.0);
	this->hdUtils.lstPos.set(0.0, 0.0, 0.0);
	this->hdUtils.prvAng.set(0.0, 0.0, 0.0);
	this->hdUtils.lstAng.set(0.0, 0.0, 0.0);
	this->hdUtils.prvInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lstInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstInputVel.set(0.0, 0.0, 0.0);
	this->hdUtils.prvOutVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lstOutVel.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstOutVel.set(0.0, 0.0, 0.0);
	this->hdUtils.prvInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lstInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstInputAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.prvOutAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lstOutAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.vrLstOutAngVel.set(0.0, 0.0, 0.0);
	this->hdUtils.lvelocity.set(0.0, 0.0, 0.0);
	this->hdUtils.lvelocityTemp.set(0.0, 0.0, 0.0);
	this->hdUtils.avelocity.set(0.0, 0.0, 0.0);
	this->hdUtils.avelocityTemp.set(0.0, 0.0, 0.0);
	std::memset(this->hdUtils.jointPosPrev, 0.0, sizeof(double)*GEOMAGIC_JOINTS);

	this->availability(true);
	std::cout << "GeomagicTeleopProxy initialized. " << std::endl;

}

/**
* @brief Default run function
*/
void GeomagicTeleopProxy::run() {}

/**
* @brief Default clear function
*/
void GeomagicTeleopProxy::clear() {}

/**
* @brief Update function
* Update the linear and angular velocities of the Geomagic stylus
* Set internally stylusLinearVelocity and stylusAngularVelocity
*/
void GeomagicTeleopProxy::updateVelocities() {

	GeoTeleopState* geoTeleopState = (GeoTeleopState*)(state);

	hduVector3Dd stylusPosition = eigen2hdu3D(geoTeleopState->hipPosition.cast<double>());
	hduVector3Dd stylusGimbalAngles = eigen2hdu3D(geoTeleopState->hipGimbalAngles.cast<double>());
	hduVector3Dd stylusLinearVelocity, stylusAngularVelocity;
	
	// Compute linear velocities
	hduVector3Dd vel_buff(0.0, 0.0, 0.0);
	vel_buff = (stylusPosition * 3 - 4 * hdUtils.prvPos + hdUtils.lstPos) / 0.002; //mm/s, 2nd order backward dif
	hdUtils.lvelocity = (.2196 * (vel_buff + hdUtils.vrLstInputVel) + .6588 * (hdUtils.prvInputVel + hdUtils.lstInputVel)) / 1000.0 - (-2.7488 * hdUtils.prvOutVel + 2.5282 * hdUtils.lstOutVel - 0.7776 * hdUtils.vrLstOutVel); //cutoff freq of 20 Hz
	hdUtils.lstPos = hdUtils.prvPos;
	hdUtils.prvPos = stylusPosition;
	hdUtils.vrLstInputVel = hdUtils.lstInputVel;
	hdUtils.lstInputVel = hdUtils.prvInputVel;
	hdUtils.prvInputVel = vel_buff;
	hdUtils.vrLstOutVel = hdUtils.lstOutVel;
	hdUtils.lstOutVel = hdUtils.prvOutVel;
	hdUtils.prvOutVel = hdUtils.lvelocity;
	hdUtils.lvelocityTemp = hdUtils.lvelocityTemp + ((hdUtils.lvelocity - hdUtils.lvelocityTemp) * (0.001 / (0.001 + 0.07)));

	// Set linear velocities
	stylusLinearVelocity = hdUtils.lvelocityTemp;

	// Compute angular velocities
	vel_buff.set(0.0, 0.0, 0.0);
	vel_buff = (stylusGimbalAngles * 3 - 4 * hdUtils.prvAng + hdUtils.lstAng) / 0.002; //mm/s, 2nd order backward dif
	hdUtils.avelocity = (.2196 * (vel_buff + hdUtils.vrLstInputAngVel) + .6588 * (hdUtils.prvInputAngVel + hdUtils.lstInputAngVel)) / 1000.0 - (-2.7488 * hdUtils.prvOutAngVel + 2.5282 * hdUtils.lstOutAngVel - 0.7776 * hdUtils.vrLstOutAngVel); //cutoff freq of 20 Hz
	hdUtils.lstAng = hdUtils.prvAng;
	hdUtils.prvAng = stylusGimbalAngles;
	hdUtils.vrLstInputAngVel = hdUtils.lstInputAngVel;
	hdUtils.lstInputAngVel = hdUtils.prvInputAngVel;
	hdUtils.prvInputAngVel = vel_buff;
	hdUtils.vrLstOutAngVel = hdUtils.lstOutAngVel;
	hdUtils.lstOutAngVel = hdUtils.prvOutAngVel;
	hdUtils.prvOutAngVel = hdUtils.avelocity;
	hdUtils.avelocityTemp = hdUtils.avelocityTemp + ((hdUtils.avelocity - hdUtils.avelocityTemp) * (0.001 / (0.001 + 0.07)));

	// Set angular velocities
	stylusAngularVelocity = hdUtils.avelocityTemp;

	// Update the Eigen variables in the member structure TeleopStatus
	geoTeleopState->hipVelocity.topRows(SPACE_DIM) = hdu2Eigen3D(stylusLinearVelocity).cast<float>();
	geoTeleopState->hipVelocity.bottomRows(SPACE_DIM) = hdu2Eigen3D(stylusAngularVelocity).cast<float>();
}


/**
* @brief Event catch function
* Catch the event when the input button has been pressed, computed on the consecutive raising and trailing edges of the button pressing state
* @param button: the current state of the pressed button
* @param button_prev: the previous state of the pressed button
* @param raise: the raising edge of the event
* @param trail: the trailing edge of the event
* @param trigger: the boolean value to be returned
*/
void GeomagicTeleopProxy::catchButtonPressEvent(const bool& button, bool& button_prev, bool& raise, bool& trail, bool& trigger) {


	if (button && !button_prev) {
		raise = true;
	}
	if (!button && button_prev) {
		trail = true;
	}
	if (raise && trail) {

		trigger = !trigger;
		raise = false;
		trail = false;
	}

	button_prev = button;

}