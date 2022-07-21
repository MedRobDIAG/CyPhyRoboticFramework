#ifndef ABDOMEN_PHANTOM_HPP_
#define ABDOMEN_PHANTOM_HPP_

// Project Header files
#include "Phantom.hpp"


class AbdomenPhantom : public Phantom {

public:

	/**
	* @brief Default constructor of AbdomenPhantom class
	*
	*/
	AbdomenPhantom();

	/**
	* @brief constructor of AbdomenPhantom class with name argument
	* @param name_ the name of the instrument
	*/
	AbdomenPhantom(const std::string& name_);

	/**
	* @brief Default destroyer of AbdomenPhantom class
	*
	*/
	~AbdomenPhantom();

	/**
	* @brief Init function
	* Check the input pair of comment+value to assign the corresponding parameter
	* The function assumes that configFile has been previously set
	* @param comment: the string specifying the title comment of parameter
	* @param value: the string specifying the value of the parameter
	*/
	void loadParamsFromConfigFile(const std::string& comment, const std::string& value);

	/**
	* @brief Set function
	* Set the set of external points of the phantom
	* @param extPoints: external points of the phantom
	*/
	void setExtPoints(const Eigen::MatrixXd& extPoints) { this->extPts = extPoints; }

	/**
	* @brief Get function
	* Get the radiopaque points placed on the surface of the Phantom
	* @return the radiopaque points
	*/
	inline Eigen::MatrixXd getRadioPts() { return this->radioPts; }

	/**
	* @brief Save function
	* Save the acquired points on the requested file
	* @brief filename the name of the file on which save the acquired points
	*/
	void saveAcquiredPoints(const char* filename);

	/**
	* @brief Get function
	* Retrieve the number of radioopaque landmark points placed on the phantom
	* @return the number of points
	*/
	inline int getPointsNum() { return this->radioPts.rows(); }

	/**
	* @brief Load function
	* Load the radiopaque points from the file with the given name
	* @param filename the name of the file to be loaded
	* @return the radiopaque points
	*/
	Eigen::MatrixXd loadRadioPts(const char* filename);

	/**
	* @brief Extract function
	* Extract the point coordinates of the phantom from the input file content
	* @param the content of the file organized as set of string lines
	* @return the set of point coordinates of the phantom radiopaque points
	*/
	Eigen::MatrixXd extractRadioPts(std::vector < std::string >& content);

	/**
	* @brief Load function
	* Load the acquired points on the requested file
	* @return the eset of external point coordinates
	*/
	Eigen::MatrixXd loadAcquiredPoints(const char* filename);

private:

	Eigen::MatrixXd radioPts;		//!< Set of radio-opaque point coordinates of the phantom, expressed in the CT frame
	Eigen::MatrixXd extPts;			//!< Set of radio-opaque point coordinates of the phantom, expressed in the external reference frame to be registered


};

#endif // ABDOMEN_PHANTOM_HPP_