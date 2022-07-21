#ifndef GEOMAGICPROXY_HPP_
#define GEOMAGICPROXY_HPP_

// Project Header files
#include "GeomagicHapticProxy.hpp"
#include "GeomagicTeleopProxy.hpp"

/*struct GeomagicStatus {

	hduVector3Dd stylusPosition;									//!< Position vector of the styus HIP (Haptic Interface Point) of the Geomagic device
	hduVector3Dd stylusGimbalAngles;								//!< Orientation vector containing the gimbal angles of the stylus of the Geomagic device

	hduVector3Dd stylusLinearVelocity;								//!< Linear velocity vector of the styus HIP (Haptic Interface Point) of the Geomagic device
	hduVector3Dd stylusAngularVelocity;								//!< Angular velocity vector of the styus HIP (Haptic Interface Point) of the Geomagic device
	hduVector3Dd force;												//!< Force vector of the Geomagic device

	double jointPosition[GEOMAGIC_JOINTS];							//!< Array of Geomagic joint positions
	double jointVelocity[GEOMAGIC_JOINTS];							//!< Array of Geomagic joint velocity
	double jacobian[SPACE_DIM * 2 * GEOMAGIC_JOINTS];				//!< Vectorized Jacobian matrix (in which order it is stored?)

	int stylusButtons;												//!< Status of the buttons on the Geomagic stylus
	bool action[GEOMAGIC_BUTTONS_NUM];								//!< Static array of boolean values stating if each button has been pressed (true on the raising edge of the event)
	bool evHoldButton[GEOMAGIC_BUTTONS_NUM];						//!< Static array of boolean values stating if each button has been pressed (true on the raising edge of the event)
	bool evRaiseEdge[GEOMAGIC_BUTTONS_NUM];							//!< Static array of boolean values stating if each button has been pressed (true on the raising edge of the event)
	bool evTrailEdge[GEOMAGIC_BUTTONS_NUM];							//!< Static array of boolean values stating if each button has been pressed (true on the raising edge of the event)
};//*/


/**
* @brief Callback function
* Force feedback callback of the Geomagic device
* @param data the data containing the force feedback data
*/



class GeomagicProxy : public ExtSystemProxy {


public:

	/**
	* @brief Default contructor of GeomagicProxy class
	*
	*/
	GeomagicProxy();

	/**
	* @brief Contructor of GeomagicProxy class with pointers
	* @param hp: the pointer to the GeomagicHapticProxy component
	* @param tp: the pointer to the GeomagicTeleopProxy component
	*
	*/
	GeomagicProxy(GeomagicHapticProxy* hp, GeomagicTeleopProxy* tp);

	/**
	* @brief Default destroyer of GeomagicProxy class
	*
	*/
	~GeomagicProxy();


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
	* @brief Dynamic instance function
	* Create a dynamic instance of the GeomagicHapticProxy class
	* @return the pointer to a dynamic instance of GeomagicHapticProxy
	*/
	GeomagicHapticProxy* instantiateHaptic();

	/**
	* @brief Get function
	* Get the dynamic instance of the GeomagicHapticProxy class
	* @return the pointer to a dynamic instance of GeomagicHapticProxy
	*/
	inline GeomagicHapticProxy* getHapticHandler() { return this->geoHapticProxy; }

	/**
	* @brief Dynamic instance function
	* Create a dynamic instance of the GeomagicTeleopProxy class
	* @return the pointer to a dynamic instance of GeomagicTeleopProxy
	*/
	GeomagicTeleopProxy* instantiateTeleop();

	/**
	* @brief Get function
	* Get the dynamic instance of the GeomagicTeleopProxy class
	* @return the pointer to a dynamic instance of GeomagicTeleopProxy
	*/
	inline GeomagicTeleopProxy* getTeleopHandler() { return this->geoTeleopProxy; }

	/**
	* @brief Set function
	* Set the haptics flag of the Geomagic 
	* @param haptics: flag of the Geomagic to be set
	*/
	inline void enableHaptics(const bool& haptics) { this->useHaptics = haptics; }

	/**
	* @brief Set function
	* Set the teleop flag of the Geomagic
	* @param teleop: flag of the Geomagic to be set
	*/
	inline void enableTeleop(const bool& teleop) { this->useTeleop = teleop; }

	/**
	* @brief Check function
	* Check if the the haptics module of the Geomagic is requested (through the flag useHaptics)
	* @return true if the haptics module of the Geomagic is requested 
	*/
	inline bool isHapticsRequested() { return this->useHaptics; }

	/**
	* @brief Check function
	* Check if the the teleoperation module of the Geomagic is requested (through the flag useTeleop)
	* @return true if the teleoperation module of the Geomagic is requested
	*/
	inline bool isTeleopRequested() { return this->useTeleop; }

	/**
	* @brief Calibrate function
	* Calibrate the Geomagic device
	* @return true if the device has been successfully calibrated
	*/
	bool calibrate();

	HHD dvcHandle;								//!< OpenHaptics device handler

protected:

	HDSchedulerHandle schHandle;				//!< OpenHaptics scheduler handler
	GeomagicHapticProxy* geoHapticProxy;
	GeomagicTeleopProxy* geoTeleopProxy;
	bool useHaptics;
	bool useTeleop;

};



#endif // GEOMAGICPROXY_HPP_