#ifndef GAUSSNEWTONALGORITHM_HPP
#define GAUSSNEWTONALGORITHM_HPP

// Standard Header files
#include <vector>

// Project Header files
#include "utils.hpp"

// Typedef for pointer to member functions, to be used by external classes without the object (requires static definition)
typedef Eigen::VectorXd(*zhatCallback)(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& params);// , const int& fricParamsNum, const bool& wGravity);
typedef Eigen::MatrixXd(*jacobianCallback)(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& params);// , const int& fricParamsNum, const bool& wGravity);

class GaussNewtonAlgorithm {


public:

	/**
	* @brief Default constructor of the GaussNewtonAlgorithm class
	*/
	GaussNewtonAlgorithm();

	/**
	* @brief Default destroyer of the GaussNewtonAlgorithm class
	*/
	~GaussNewtonAlgorithm() {}


	/**
	* @brief Set function
	* Set the size of the state (and input) vector
	* @param the state size
	*/
	inline void setStateSize(const int& s) { this->stateSize = s; }

	/**
	* @brief Set function
	* Set the size of the measurement vector
	* @param the measurement vector size
	*/
	inline void setMeasSize(const int& s) { this->measSize = s; }

	/**
	* @brief Get function
	* Retrieve the estimation state vector
	* @return the estimation state vector
	*/
	inline Eigen::VectorXd getX() { return this->X.back(); }

	/**
	* @brief Get function
	* Retrieve the history of the estimation state vector
	* @return the history of the estimation state vector
	*/
	inline std::vector < Eigen::VectorXd > getXFull() { return this->X; }

	/**
	* @brief Get function
	* Retrieve the history of the predicted measurement
	* @return the history of the predicted measurement
	*/
	inline std::vector < Eigen::VectorXd > getZhatFull() { return this->Zhat; }

	/**
	* @brief Run function
	* Run a single iteration of the Gauss-Newton algorithm
	* TODO
	* IT'S STILL NOT CLEAR HOW TO IMPLEMENT ONLINE VERSION
	*/
	void runOneRound(const Eigen::VectorXd& phi_k, const Eigen::VectorXd& z);

	/**
	* @brief Run function
	* Run the batch version of the Gauss-Newton algorithm
	* @return the estimated friction parameters
	*/
	Eigen::VectorXd runBatch();

	/**
	* @brief Set function
	* Add the k-th entry of the input array
	* @param the k-th entry to be added
	*/
	void addInputEntry(const Eigen::VectorXd& q_k, const Eigen::VectorXd& qd_k);

	/**
	* @brief Set function
	* Add the k-th entry of the Z matrix
	* @param the k-th entry to be added
	*/
	inline void addZkEntry(const Eigen::VectorXd& z_k) { this->Z.push_back(z_k); }

	/**
	* @brief Set function
	* Set the number of iterations for the batch implementation of the GN algorithm
	* @param the number of iterations
	*/
	inline void setIterationsNum(const int& iterNum) { this->iterationsNum = iterNum; }

	/**
	* @brief Save function
	* Save the statistics of the algorithm on the given file
	* @param xFilename: the name of the file with the state x
	* @param zhatFilename: the name of the file with the predicted measurement zhat
	* @param zFilename: the name of the file with the measurement x
	*/
	void saveStats(const char* xFileName, const char* zhatFilename, const char* zFilename);

	/**
	* @brief Run function
	* Run the batch version of the algorithm, accepting as input arguments the function pointers to the measurement model and the related Jacobian
	* @param zhat: the function pointer accepting joint position and velocities and the state as inputs, and returning the vector of the predicted measurement
	* @param jacobian: the function pointer accepting joint position and velocities and the state as inputs, and returning the matrix of the jacobian measurement
	*/
	Eigen::VectorXd runBatchV2(zhatCallback zhat, jacobianCallback jacobian);

	/**
	* @brief Create folder function
	* Create the folder in which the stats data have to be stored
	* @param folderName: the name of the folder where data have to be stored
	* @return the name of the folder (? mayb useless)
	*/
	std::string createStatsFolder(const char* folderName);

	/**
	* @brief Load function
	* Load the datasets from previous sessions to run the algorithm offline
	* @param logNum: the number identifying the previous log session
	* @param [out] q: the dataset of joint position
	* @param [out] qd: the dataset of joint velocity
	* @param [out] r: the dataset of residuals
	*/
	void loadData(const int& logNum, std::vector<Eigen::VectorXd>& q, std::vector<Eigen::VectorXd>& qd, std::vector<Eigen::VectorXd>& r);

private:

	std::vector < std::pair < Eigen::VectorXd , Eigen::VectorXd > > input;	//!< Array of pair of input data
	std::vector < Eigen::VectorXd > X;										//!< Array of estimating state vectors
	std::vector < Eigen::VectorXd > Z;										//!< Array of reference measurements
	std::vector < Eigen::VectorXd > Zhat;									//!< Array of predicted measurements

	int iterationsNum;			//!< Number of iterations for batch implementation
	int stateSize;				//!< Size of the state 
	int measSize;				//!< Size of the measurement

	std::string statsFolder;	//!< Name of the folder where the statistics are stored

};


#endif // GAUSSNEWTONALGORITHM_HPP