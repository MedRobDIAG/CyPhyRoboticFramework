#ifndef ICP_HPP_
#define ICP_HPP_


// Eigen Header files
#include <Eigen/Dense>

#ifdef DEBUG
#include <iostream>
#endif // DEBUG

#define POSE12D 12
#define COORDINATE_SIZE 3
#define POSE6D 6

class ICP {


public:


	/**
	* @brief Default Construcor
	* @param iters: number of iterations of the ICP algorithm (default is 10)
	*/
	ICP(const int& iters = 30);

	/**
	* @brief Default Destroyer
	*/
	~ICP();

	/**
	* @brief Solve function 
	* Implement the ICP algorithm
	*/
	void run();

	/**
	* @brief Set function
	* Fill the set of source points
	* @param a vector containing the set of source points
	*/
	void populateSrcSet(const Eigen::MatrixXd& src);

	/**
	* @brief Set function
	* Fill the set of destination points
	* @param a vector containing the set of destination points
	*/
	void populateDstSet(const Eigen::MatrixXd& dst);

	/**
	* @brief Set function
	* Set the solution vector of the ICP algorithm
	* @param the solution vector of the ICP algorithm
	*/
	inline void setSolution(const Eigen::VectorXd& sol) { this->solution = sol; }

	/**
	* @brief Get function
	* Get the solution vector of the ICP algorithm
	* @return the solution vector of the ICP algorithm
	*/
	inline Eigen::VectorXd getSolution() { return this->solution; }


	/**
	* @brief Get function
	* Get the solution 4x4 Matrix of the ICP algorithm
	* @return the solution 4x4 Matrix of the ICP algorithm
	*/
	inline Eigen::Matrix4d getTsol() { return this->Tctp; }

	/**
	* @brief Set function
	* Set the number of iterations of the ICP algorithm
	* @param the number of iterations of the ICP algorithm
	*/
	inline void setIterations(const int& iters) { this->iterations = iters; }

	/**
	* @brief Get function
	* Get the number of iterations of the ICP algorithm
	* @return the number of iterations of the ICP algorithm
	*/
	inline int getIterations() { return this->iterations; }


#ifdef DEBUG
	/**
	* @brief Debug function
	* Print the two sets of points on which the ICP is performed
	*/
	void showPointSets();
#endif // DEBUG

	/**
	* @brief Get function
	* Retrieves the calibration stats vector
	* @return the calibration stats vector
	*/
	inline Eigen::VectorXd getChiStats() { return this->chi_stats; }

	/**
	* Save function
	* Save the ICP stats on an output file
	*/
	void saveStats();


	/**
	* @brief Get function
	* Retrieves the source set of points processed by ICP (required to let know the re-order of the points at application level)
	* @return the reodered set of source points
	*/
	inline Eigen::MatrixXd getSrcSet() { return this->srcSet; }

	/**
	* @brief Set function
	* Set the flag stating if the data association is known
	* @param the flag
	*/
	inline void setDataAssociationKnown(const bool& flag) { this->knownDataAssociations = flag; }

	/**
	* @brief Get function
	* Get the flag stating if the data association is known
	* @return true if data association is known
	*/
	inline bool isDataAssociationKnown() { return this->knownDataAssociations; }

private:

	Eigen::MatrixXd dstSet;			//!< Destination set of points (convention: each point on a row)
	Eigen::MatrixXd srcSet;			//!< Source set of points (convention: each point on a row)
	Eigen::VectorXd initialGuess;	//!< Initial guess to initialize the ICP algorithm
	Eigen::VectorXd solution;		//!< Final solution obtained as output of the ICP algorithm
	Eigen::Matrix4d Tctp;			//!< Final solution in form of a 4x4 homogeneous matrix
	Eigen::VectorXd chi_stats;		//!< Time evolution of the chi square error
	int iterations;					//!< Number of iterations of the ICP algorithm
	bool knownDataAssociations;		//!< Flag stating if the data association is known
	/**
	* @brief Initial guess computation
	* Compute the initial guess based on the known point correspondences
	* Sets internally the initialGuess member variable
	*/
	void computeInitialGuess();

	/**
	* @brief Solve function
	* Compute the main algorithm of ICP
	* @param ...
	* @return ...
	*/
	void solve();

	/**
	* @brief Jacobian function
	* Compute the error vector and the Jacobian matrix for ICP, based on the current state and
	* measurements
	* @param x: the current state vector
	* @param p: the known point vector
	* @param z: the current measurement
	* @return a pair containing the current error vector and Jacobian matrix
	*/
	std::pair < Eigen::VectorXd, Eigen::MatrixXd> errorAndJacobian(const Eigen::VectorXd& x, const Eigen::VectorXd& p, const Eigen::VectorXd& z);

	/**
	* @brief Convert function
	* Convert the vectorized rotation matrix (12D vector) to the 3D vector of alpha, beta and gamma angles
	* @param rot_: the vectorized rotation matrix (12D vector)
	* @return abg: the 3D vector of alpha, beta and gamma angles
	*/
	Eigen::Vector3d rot2abg(const Eigen::VectorXd& rot_);

	/**
	* @brief Convert function
	* Convert the vectorized solution (6D vector) to the coresponding 4x4 homogeneous matrix
	* Use internally the member variable 'solution' and sets internally the member variable Tsol
	*/
	void V2TsolConversion();


	/**
	* @brief Data association function
	* Find the correspondences between the input clouds of points for the ICP algorithm
	* @param current guess
	* @return candidate correspondences
	*/
	Eigen::MatrixXd findCorrespondences(const Eigen::VectorXd& guess);

	/**
	* @brief Data association function (v2)
	* Find the correspondences between the input clouds of points for the ICP algorithm
	* @param current guess
	* @return candidate correspondences
	*/
	Eigen::MatrixXd findCorrespondencesV2(const Eigen::VectorXd& guess);

	/**
	* @brief Convert function
	* Convert the vectorized solution (6D vector) to the coresponding 4x4 homogeneous matrix
	*/
	Eigen::Matrix4d v2t(const Eigen::VectorXd& v);

};



#endif // ICP_HPP_