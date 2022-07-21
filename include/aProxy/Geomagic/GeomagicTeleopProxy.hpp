#ifndef GEOMAGIC_TELEOP_PROXY_HPP_
#define GEOMAGIC_TELEOP_PROXY_HPP_

// Project Header files
#include "TeleopProxy.hpp"

// Geomagic Header files
#include <HD\hd.h>
#include <HDU\hduError.h>
#include <HDU\hduMatrix.h>
#include <HDU\hduVector.h>

#define GEOMAGIC_JOINTS 6

enum BUTTONS { GEOMAGIC_LOW_BUTTON, GEOMAGIC_HIGH_BUTTON, GEOMAGIC_BUTTONS_NUM };
enum PRESSED_BUTTONS { NO_PRESSED, PRESSED_LOW, PRESSED_HIGH, PRESSED_BOTH };

class GeoTeleopState : public TeleopState {

public:

	GeoTeleopState() : TeleopState() {}
	~GeoTeleopState() {}

	// make this private later
	int stylusButtons;												//!< Status of the buttons on the Geomagic stylus
	bool action[GEOMAGIC_BUTTONS_NUM];								//!< Static array of boolean values stating if each button has been pressed (true on the raising edge of the event)
	bool evHoldButton[GEOMAGIC_BUTTONS_NUM];						//!< Static array of boolean values stating if each button has been pressed (true on the raising edge of the event)
	bool evRaiseEdge[GEOMAGIC_BUTTONS_NUM];							//!< Static array of boolean values stating if each button has been pressed (true on the raising edge of the event)
	bool evTrailEdge[GEOMAGIC_BUTTONS_NUM];							//!< Static array of boolean values stating if each button has been pressed (true on the raising edge of the event)

private:


};


hduVector3Dd eigen2hdu3D(const Eigen::Vector3d& eig_in);
Eigen::Vector3d hdu2Eigen3D(const hduVector3Dd& hdu_in);


struct HDUtilityData {

	hduVector3Dd prvPos;
	hduVector3Dd lstPos;

	hduVector3Dd prvAng;
	hduVector3Dd lstAng;

	hduVector3Dd prvInputVel;
	hduVector3Dd lstInputVel;
	hduVector3Dd vrLstInputVel;
	hduVector3Dd prvOutVel;
	hduVector3Dd lstOutVel;
	hduVector3Dd vrLstOutVel;

	hduVector3Dd prvInputAngVel;
	hduVector3Dd lstInputAngVel;
	hduVector3Dd vrLstInputAngVel;
	hduVector3Dd prvOutAngVel;
	hduVector3Dd lstOutAngVel;
	hduVector3Dd vrLstOutAngVel;

	hduVector3Dd lvelocity;
	hduVector3Dd lvelocityTemp;

	hduVector3Dd avelocity;
	hduVector3Dd avelocityTemp;

	HDdouble jointPosPrev[GEOMAGIC_JOINTS];

};


/**
* @brief Callback function
* Update the state of the Geomagic device
* @param data the data containing the updated status
*/
HDCallbackCode HDCALLBACK updateGeoStateCallback(void* data);

class GeomagicTeleopProxy : public TeleopProxy {

	/**
	* @brief Callback function
	* Update the state of the Geomagic device
	* @param data the data containing the updated status
	*/
	friend HDCallbackCode HDCALLBACK updateGeoStateCallback(void* data);

public:

	/**
	* @brief Default contructor of GeomagicTeleopProxy class
	*
	*/
	GeomagicTeleopProxy();

	/**
	* @brief Default destroyer of GeomagicTeleopProxy class
	*
	*/
	~GeomagicTeleopProxy();


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
	* @brief Event catch function
	* Catch the event when the input button has been pressed, computed on the consecutive raising and trailing edges of the button pressing state
	* @param button: the current state of the pressed button
	* @param button_prev: the previous state of the pressed button
	* @param raise: the raising edge of the event
	* @param trail: the trailing edge of the event
	* @param trigger: the boolean value to be returned
	*/
	void catchButtonPressEvent(const bool& button, bool& button_prev, bool& raise, bool& trail, bool& trigger);

	/**
	* @brief Update function
	* Update the linear and angular velocities of the Geomagic stylus
	* Set internally stylusLinearVelocity and stylusAngularVelocity
	*/
	void updateVelocities();

private:

	HDUtilityData hdUtils;						//!< Structure containing some utility variables necessary to process linear and angular velocities

};



#endif // GEOMAGIC_TELEOP_PROXY_HPP_
