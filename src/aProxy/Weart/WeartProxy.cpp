// Project Header files
#include "WeartProxy.hpp"
#include "Timer.hpp"

/**
* @brief Default contructor of WeartProxy class
*
*/
WeartProxy::WeartProxy() : HapticProxy() {


}


/* @brief Default destroyer of WeartProxy class
*
*/
WeartProxy::~WeartProxy() {}

/**
* @brief Copy constructor of the WeartProxy class
* @param hp the WeartProxy
*/
WeartProxy::WeartProxy(WeartProxy& wp) {}

/**
* @brief Default init function
*/
void WeartProxy::init() {

	this->hapticDOF = 1;
	this->hapticState.force.setZero(this->hapticDOF);
	this->hapticState.vibroIntensity = 0.0;
	this->availability(true);
	this->setRunning(true);

}

/**
* @brief Default run function
*/
void WeartProxy::run() {

	// Initialize the client
	//this->client = new WeArtClient("192.168.1.167", WeArtConstants::DEFAULT_TCP_PORT); //IP ADDRESS and PORT of Middleware PC
	//this->client = new WeArtClient("10.10.92.207", WeArtConstants::DEFAULT_TCP_PORT); //IP ADDRESS and PORT of Middleware PC
	//this->client = new WeArtClient("169.254.89.218", WeArtConstants::DEFAULT_TCP_PORT); //IP ADDRESS and PORT of Middleware PC
	//this->client = new WeArtClient("10.90.64.229", WeArtConstants::DEFAULT_TCP_PORT); //IP ADDRESS and PORT of Middleware PC
	//this->client = new WeArtClient("10.195.132.173", WeArtConstants::DEFAULT_TCP_PORT); //IP ADDRESS and PORT of Middleware PC
	this->client = new WeArtClient("10.90.64.229", WeArtConstants::DEFAULT_TCP_PORT); //IP ADDRESS and PORT of Middleware PC

	
	// create haptic object to manage actuation on Righ hand and Index Thimble
	this->haptic = new WeArtHapticObject(this->client);
	this->haptic->handSideFlag = HandSide::Right;
	this->haptic->actuationPointFlag = ActuationPoint::Middle;

	//define feeling properties to create an effect
	WeArtTemperature temperature = WeArtTemperature();
	WeArtForce force = WeArtForce();
	WeArtTexture texture = WeArtTexture();

	// Create default texture behavior
	texture.textureVelocity(0.0, 0.0, 1.0);
	texture.textureType(TextureType::VelcroHooks); // Check which could be better

	// instance a new effect with feeling properties and add effect to thimble
	touchEffect = new TouchEffect(temperature, force, texture);

	this->haptic->AddEffect(touchEffect);

	// run socket communication 
	this->client->Run();
	std::cout << "WeArt client successfully connected and run. " << std::endl;

	// Start client
	this->client->Start();
	
	// Time variables
	double tictoc, dt, rate, tic, toc, tac, Ts;
	Timer clock;
	clock.setRate(250.0);
	rate = clock.getRate();
	Ts = 1.0 / rate;

	while (this->isRunning()) {

		// Measure starting time
		tic = clock.getCurTime();

		//----------------------------------------------------------------//
		// Do stuff here... 

		// Update the force value
		float f = this->hapticState.force(0);
		f = (f < MAX_FLOAT_FORCE_VAL) ? ((f > -MAX_FLOAT_FORCE_VAL) ? f : (-MAX_FLOAT_FORCE_VAL)) : MAX_FLOAT_FORCE_VAL;
		force.active = true;
		force.value(f);

		// Update the vibrotactile value
		float vibIntensity = this->hapticState.vibroIntensity;
		vibIntensity = vibIntensity / MAX_FLOAT_FRICTION_VAL * 100.0f;
		texture.active = true;
		texture.volume(vibIntensity);
		//std::cout << "[WP] vibIntensity = " << vibIntensity << std::endl;

		// Set the final touch effect on the device
		this->touchEffect->Set(temperature, force, texture);

		// Update touch effects
		this->haptic->UpdateEffects();

		//----------------------------------------------------------------//

		// Measure the ending time and the elapsed time
		toc = clock.getCurTime();
		tictoc = clock.elapsedTime(tic, toc);

		// Wait until Ts
		if (tictoc < Ts) {
			clock.timeSleep(Ts - tictoc);
		}

		// Measure the final time after sleep to check the actual rate of the thread
		tac = clock.getCurTime();
		dt = clock.elapsedTime(tic, tac);
		//std::cout << "[WeartProxy] Running rate:" << (1.0 / dt) << std::endl;
	}


	// Reset feedback data
	force.active = false;
	texture.active = false;
	this->touchEffect->Set(temperature, force, texture);
	this->haptic->UpdateEffects();

	// Start client
	this->client->Stop();


}

/**
* @brief Default clear function
*/
void WeartProxy::clear() {

	// Set running on false
	this->setRunning(false);

}
