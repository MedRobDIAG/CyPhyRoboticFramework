#ifndef HRING
#define HRING

// Project Header files
#include "HapticInterface.hpp"

class HRing : public HapticInterface {

public:

	/**
	* @brief Default constructor of the HRing class
	*/
	HRing();

	/**
	* @brief Default constructor of the HRing class
	*/
	HRing(const std::string& name_);

	/**
	* @brief Default destroyer of the HRing class
	*/
	~HRing();

	void loadParamsFromConfigFile(const std::string& comment, const std::string& value);


private:

};


#endif // HRING
