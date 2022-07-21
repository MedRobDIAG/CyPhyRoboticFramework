#ifndef GEOMAGICHAPTICPROXY_HPP_
#define GEOMAGICHAPTICPROXY_HPP_

//Project Header files
#include "HapticProxy.hpp"
#include "utils.hpp"

#define GEOMAGIC_HAPTIC_DOF 3

#include <conio.h>
#include <HD\hd.h>
#include <HDU\hduError.h>
#include <HDU\hduMatrix.h>
#include <HDU\hduVector.h>

HDCallbackCode HDCALLBACK forceFeedbackCallback(void* data);

class GeomagicHapticProxy : public HapticProxy {


	/**
	* @brief Callback function
	* Force feedback callback of the Geomagic device
	* @param data the data containing the force feedback data
	*/
	friend HDCallbackCode HDCALLBACK forceFeedbackCallback(void* data);


public:

	/**
	* @brief Default contructor of GeomagicHapticProxy class
	*
	*/
	GeomagicHapticProxy();

	/**
	* @brief Default destroyer of GeomagicProxy class
	*
	*/
	~GeomagicHapticProxy();

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
	* @brief Get function
	* Retrieves the 6D velocity vector from the Geomagic Proxy
	* @param v [out] the 6D velocity vector
	*/
	//void getHIPVelocity(float v[SPACE_DIM*2]);

	/**
	* @brief Get function
	* Retrieve the HapticStatus structure containing the current status of the haptic device
	* @return the HapticStatus structure 
	*/
	//inline HapticStatus getHapticStatus() { return this->geoStatus; }

	/**
	* @brief Set function
	* Set the feedback haptic force on the Geomagic device
	* @param f: the feedback force to be set
	*/
	//void setHapticForce(const Eigen::Vector3f& f);


private:


};


#endif // GEOMAGICHAPTICPROXY_HPP_
