#ifndef WEARTPROXY_HPP_
#define WEARTPROXY_HPP_

using namespace std;

//Project Header files
#include "HapticProxy.hpp"

#include <WEART_SDK/WeArtClient.h>
#include <WEART_SDK/WeArtHapticObject.h>
#include <WEART_SDK/WeArtThimbleTrackingObject.h>
#include "WEART_SDK/TouchEffect.h"

#define MAX_FLOAT_FORCE_VAL 1.0
#define MAX_FLOAT_FRICTION_VAL 2.0

class WeartProxy : public HapticProxy {

public:

	/**
	* @brief Default contructor of WeartProxy class
	*
	*/
	WeartProxy();


	/* @brief Default destroyer of WeartProxy class
	*
	*/
	~WeartProxy();

	/**
	* @brief Copy constructor of the WeartProxy class
	* @param hp the WeartProxy
	*/
	WeartProxy(WeartProxy& wp);

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


private:

	WeArtClient* client;
	WeArtHapticObject* haptic;
	TouchEffect* touchEffect;

};


#endif // WEARTPROXY_HPP_
