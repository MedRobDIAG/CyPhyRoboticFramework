#ifndef MULTILAYER_GEL_PHANTOM_HPP_
#define MULTILAYER_GEL_PHANTOM_HPP_

// Project Header files
#include "Phantom.hpp"


class TissueLayer {

public:

	/**
	* @brief Default constructor of TissueLayer  class
	*
	*/
	TissueLayer() {}

	/**
	* @brief Constructor of TissueLayer class with depth, elasticity and viscousity parameters
	* @param d: the depth parameter
	* @param K: the elastic coefficient
	* @param D: the viscous coefficient
	*/
	TissueLayer(const float& d, const float& K = 0.0, const float& D = 0.0) : depth(d), Kcoeff(K), Dcoeff(D) {}

	/**
	* @brief Default destroyer of Layer class
	*
	*/
	~TissueLayer() {}

	/**
	* @brief Set function
	* Set the depth parameter of the tissue layer
	* @param d: the depth parameter
	*/
	inline void setDepth(const float& d) { this->depth = d; }

	/**
	* @brief Get function
	* Get the depth parameter of the tissue layer
	* @return the depth parameter
	*/
	inline float getDepth() { return this->depth; }

	/**
	* @brief Set function
	* Set the elastic coefficient of the tissue layer
	* @param k: the elastic coefficient
	*/
	inline void setKcoeff(const float& k) { this->Kcoeff = k; }

	/**
	* @brief Get function
	* Get the elastic coefficient of the tissue layer
	* @return the elastic coefficient
	*/
	inline float getKcoeff() { return this->Kcoeff; }

	/**
	* @brief Set function
	* Set the viscous  coefficient of the tissue layer
	* @param k: the viscous coefficient
	*/
	inline void setDcoeff(const float& d) { this->Dcoeff = d; }

	/**
	* @brief Get function
	* Get the viscous coefficient of the tissue layer
	* @return the viscous coefficient
	*/
	inline float getDcoeff() { return this->Dcoeff; }

	/**
	* @brief Set function
	* Set the simulated rupture threshold of the tissue layer
	* @param k: the viscous coefficient
	*/
	inline void setRuptThresh(const float& r) { this->ruptureThresh = r; }

	/**
	* @brief Get function
	* Get the simulated rupture threshold of the tissue layer
	* @return the viscous coefficient
	*/
	inline float getRuptThresh() { return this->ruptureThresh; }

	/**
	* @brief Get function
	* Check if the tissue has been punctured
	* @return the flag punctured
	*/
	inline bool isPunctured() { return this->punctured; }

	/**
	* @brief Set function
	* Set the flag punctured
	* @param the flag to be set
	*/
	inline void setPunctured(const bool& p) { this->punctured = p; }


private:

	float depth;				//!< Depth of the layer
	float Kcoeff;				//!< Elastic coefficient of the tissue layer
	float Dcoeff;				//!< Viscous coefficient of the tissue layer
	float ruptureThresh;		//!< Force threshold for rupture simulation
	bool punctured;				//!< If the tissue has been punctured or not

};


class MultiLayerGelPhantom : public Phantom {

public:

	/**
	* @brief Default constructor of MultiLayerGelPhantom class
	*
	*/
	MultiLayerGelPhantom();

	/**
	* @brief constructor of MultiLayerGelPhantom class with name argument
	* @param name_ the name of the instrument
	*/
	MultiLayerGelPhantom(const std::string& name_);

	/**
	* @brief Default destroyer of MultiLayerGelPhantom class
	*
	*/
	~MultiLayerGelPhantom();

	/**
	* @brief Init function
	* Check the input pair of comment+value to assign the corresponding parameter
	* The function assumes that configFile has been previously set
	* @param comment: the string specifying the title comment of parameter
	* @param value: the string specifying the value of the parameter
	*/
	void loadParamsFromConfigFile(const std::string& comment, const std::string& value);

	/**
	* @brief Init function
	* Initialize the target from the opened simulator scene
	*/
	void initTargetFromSimulator();

	/**
	* @brief Set function
	* Set the z0 of the phantom
	* @param z: the z0 of the phantom
	*/
	inline void setZ0(const float& z) { this->z0 = z; }

	/**
	* @brief Get function
	* Get the z0 of the phantom
	* @return the z0 of the phantom
	*/
	inline float getZ0() { return this->z0 ; }

	/**
	* @param Set function
	* Set the number of layers of the phantom
	* @param nl: the number of layers to be set
	*/
	inline void setNumLayers(const int& nl) { this->numLayers = nl; }

	/**
	* @param Get function
	* Get the number of layers of the phantom
	* @return the number of layers to be retrieved 
	*/
	inline int getNumLayers() { return this->numLayers ; }

	/**
	* @brief Get function
	* Get the TissueLayer corresponding to the input index
	* @param idx: the input tissue index
	* @return the TissueLayer corresponding to the input index
	*/
	inline TissueLayer getLayerInfo(const int& idx) { if (idx < numLayers) { return *target[idx]; } else { return TissueLayer(); } }

	/**
	* @brief Set function
	* Set the input depth to the desired layer specified by the input index
	* @param idx: the index of the desired layer
	* @param d: the depth to be set
	*/
	void setLayerDepth(const int& idx, const float& d);

	/**
	* @brief Set function
	* Set the input elastic coefficient to the desired layer specified by the input index
	* @param idx: the index of the desired layer
	* @param K: the elastic coefficient to be set
	*/
	void setLayerKcoeff(const int& idx, const float& K);

	/**
	* @brief Set function
	* Set the input viscous coefficient to the desired layer specified by the input index
	* @param idx: the index of the desired layer
	* @param D: the viscous coefficient to be set
	*/
	void setLayerDcoeff(const int& idx, const float& D);

	/**
	* @brief Set function
	* Set the input simulated rupture threshold to the desired layer specified by the input index
	* @param idx: the index of the desired layer
	* @param D: the simulated rupture threshold to be set
	*/
	void setLayerRuptureThreshold(const int& idx, const float& r);

	/**
	* @brief Set function
	* Set the flag punctured for the layer specified by the input index
	* @param idx: the index of the desired layer
	* @param D: the flag punctured
	*/
	void setLayerPunctured(const int& idx, const bool& p);

	/**
	* @brief Utility function
	* Compute and store in a vector the cumulative depths of each layer (i.e., v = [0.0, d1, d1+d2, d1+d2+d3, ...])
	* @return the cumulative vector of depths
	*/
	std::vector <float> getCumulativeDepthsVector();

	/**
	* @brief Reset function
	* Reset the dynamic structure of the Phantom
	*/
	void reset();

	/**
	* @brief Add function
	* Add new layer
	* @param tl: the new TissueLayer object
	*/
	void addNewLayer(TissueLayer* tl);

private:

	float z0;									//!< Z coordinate of the origin where the target is placed wrt the robot frame
	int numLayers;								//!< Number of layers
	std::vector < TissueLayer* > target;		//!< Vector of TissueLayer objects defining the multi layer gel phantom
};


#endif // MULTILAYER_GEL_PHANTOM_HPP_
