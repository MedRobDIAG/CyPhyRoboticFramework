// Project Header files
#include "MultiLayerGelPhantom.hpp"
#include "utils.hpp"

// System Header files
#include <memory>
#include <iostream>

/**
* @brief Default constructor of MultiLayerGelPhantom class
*
*/
MultiLayerGelPhantom::MultiLayerGelPhantom() : Phantom() {

	this->numLayers = -1;
	this->z0 = 0.0;
}

/**
* @brief constructor of MultiLayerGelPhantom class with name argument
* @param name_ the name of the instrument
*/
MultiLayerGelPhantom::MultiLayerGelPhantom(const std::string& name_) : Phantom(name_) {}

/**
* @brief Default destroyer of MultiLayerGelPhantom class
*
*/
MultiLayerGelPhantom::~MultiLayerGelPhantom() {}

/**
* @brief Init function
* Initialize the target from the opened simulator scene
*/
void MultiLayerGelPhantom::initTargetFromSimulator() {}

/**
* @brief Init function
* Check the input pair of comment+value to assign the corresponding parameter
* The function assumes that configFile has been previously set
* @param comment: the string specifying the title comment of parameter
* @param value: the string specifying the value of the parameter
*/
void MultiLayerGelPhantom::loadParamsFromConfigFile(const std::string& comment, const std::string& value) {

	if (comment.find("Number of layers") != std::string::npos) {
		this->numLayers = std::stod(value);
		this->target.resize(this->numLayers);
		for (int i = 0; i < this->numLayers; i++) {
			this->target[i] = new TissueLayer;
		}
	}

	if (this->numLayers != -1) {

		for (int i = 1; i <= this->numLayers; i++) {
			std::string layerStr("Layer n. " + std::to_string(i));
			if (comment.find("Layer n. " + std::to_string(i)) != std::string::npos) {
				std::vector < std::string > vec = parseStringLine(value);
				this->target[i-1]->setDepth(std::stod(vec[0]));
				this->target[i-1]->setKcoeff(std::stod(vec[1]));
				this->target[i-1]->setDcoeff(std::stod(vec[2]));
			}
		}

	}

}


/**
* @brief Set function
* Set the input depth to the desired layer specified by the input index
* @param idx: the index of the desired layer
* @param d: the depth to be set
*/
void MultiLayerGelPhantom::setLayerDepth(const int& idx, const float& d) {

	this->target[idx]->setDepth(d);

}


/**
* @brief Set function
* Set the input elastic coefficient to the desired layer specified by the input index
* @param idx: the index of the desired layer
* @param K: the elastic coefficient to be set
*/
void MultiLayerGelPhantom::setLayerKcoeff(const int& idx, const float& K) {

	this->target[idx]->setKcoeff(K);

}

/**
* @brief Set function
* Set the input viscous coefficient to the desired layer specified by the input index
* @param idx: the index of the desired layer
* @param D: the viscous coefficient to be set
*/
void MultiLayerGelPhantom::setLayerDcoeff(const int& idx, const float& D) {

	this->target[idx]->setDcoeff(D);

}


/**
* @brief Set function
* Set the input simulated rupture threshold to the desired layer specified by the input index
* @param idx: the index of the desired layer
* @param D: the simulated rupture threshold to be set
*/
void MultiLayerGelPhantom::setLayerRuptureThreshold(const int& idx, const float& r) {

	this->target[idx]->setRuptThresh(r);
}


/**
* @brief Set function
* Set the flag punctured for the layer specified by the input index
* @param idx: the index of the desired layer
* @param D: the flag punctured
*/
void MultiLayerGelPhantom::setLayerPunctured(const int& idx, const bool& p) {
	
	this->target[idx]->setPunctured(p);

}


/**
* @brief Utility function
* Compute and store in a vector the cumulative depths of each layer (i.e., v = [0.0, d1, d1+d2, d1+d2+d3, ...])
* @return the cumulative vector of depths
*/
std::vector <float> MultiLayerGelPhantom::getCumulativeDepthsVector() {

	std::vector <float> ret;
	float cumdepth;

	cumdepth = 0.0;
	ret.push_back(cumdepth);
	for (int i = 0; i < this->numLayers; i++) {
		cumdepth += this->target[i]->getDepth();
		ret.push_back(cumdepth);
	}

	return ret;

}


/**
* @brief Reset function
* Reset the dynamic structure of the Phantom
*/
void MultiLayerGelPhantom::reset() {

	for (int i = 0; i < this->target.size(); i++) {
		delete this->target[i];
	}

	this->target.clear();

}


/**
* @brief Add function
* Add new layer
* @param tl: the new TissueLayer object
*/
void MultiLayerGelPhantom::addNewLayer(TissueLayer* tl) {

	this->target.push_back(tl);
	this->numLayers = this->target.size();
	std::cout << "MLGP: Now numLayers is = " << this->numLayers << std::endl;

}

