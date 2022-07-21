#ifndef ENDEFFECTOR_HPP_
#define ENDEFFECTOR_HPP_

// Project Header files
#include "Instrument.hpp"
#include "utils.hpp"

// Standard Header files
#include <cstring>

// Space dimension is 3...
#ifndef SPACE_DIM
#define SPACE_DIM 3
#endif // SPACE_DIM

// Index of the 3D axes
enum AXES{ X_AXIS = 0, Y_AXIS, Z_AXIS };


class EndEffector : public Instrument{

public:

	/**
	* @brief Default constructor of the EndEffector class
	*/
	EndEffector() : Instrument() {}

	/**
	* @brief Default constructor of the EndEffector class
	*/
	EndEffector(const std::string& name_) : Instrument(name_) {}

	/**
	* @brief Init function 
	* Check the input pair of comment+value to assign the corresponding parameter
	* The function assumes that configFile has been previously set
	* @param comment: the string specifying the title comment of parameter
	* @param value: the string specifying the value of the parameter
	*/
	void loadParamsFromConfigFile(const std::string& comment, const std::string& value);

	/**
	* @brief Default destroyer of the EndEffector class
	*/
	~EndEffector() {}

	/**
	* @brief Set function
	* Set the size of the End-Effector
	* @param s the size to be set
	*/
	inline void setSize(float s[]) { std::memcpy(this->size, s, SPACE_DIM * sizeof(float)); }

	/**
	* @brief Get function
	* Retrieves the size of the End-Effector
	* @return the requested size
	*/
	inline void getSize(float s[]) { std::memcpy(s, this->size, SPACE_DIM * sizeof(float)); }

	/**
	* @brief Set function
	* Set the CoM of the End-Effector
	* @param com the CoM to be set
	*/
	inline void setCoM(float com[]) { std::memcpy(this->CoM, com, SPACE_DIM * sizeof(float)); }

	/**
	* @brief Get function
	* Retrieves the CoM of the End-Effector
	* @return the requested CoM
	*/
	inline void getCoM(float com[]) { std::memcpy(com, this->CoM, SPACE_DIM * sizeof(float));	}

	/**
	* @brief Set function
	* Set the weight of the End-Effector
	* @param w the weight to be set
	*/
	inline void setWeight(const float& w) { this->weight = w; }

	/**
	* @brief Get function
	* Retrieves the weight of the End-Effector
	* @return the requested weight
	*/
	inline float getWeight() { return this->weight; }

protected:

	float size[SPACE_DIM];
	float CoM[SPACE_DIM];
	float weight;


};

#endif //ENDEFFECTOR_HPP_
