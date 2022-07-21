#ifndef REGISTRATION_DATA_HPP_
#define REGISTRATION_DATA_HPP_

// TODO: LANDMARKS_NUM could be chosen at run-time
#define LANDMARKS_NUM 4
#define MINIMUM_LANDMARKS_NUM 3
#define COORDINATE_SIZE 3

// Eigen Header files
#include <Eigen/Dense>

// System header files
#include <vector>

class RegistrationData{


public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
		/**
	* @brief Constructor of the RegistrationData Class
	*/
	RegistrationData();

	/**
	* @brief Default destroyer of the RegistrationData Class
	*/
	~RegistrationData();

	/**
	* @brief Set function
	* St the homogeneous transformation matrix
	* @param T the transformation matrix
	*/
	inline void setTransform(const Eigen::Matrix4d& T_) { this->T = T_; }

	/**
	* @brief Get function
	* Get the homogeneous transformation matrix
	* @return T the transformation matrix
	*/
	inline Eigen::Matrix4d getTransform() { return this->T; }

	/**
	* @brief Set function
	* St the ground truth point coordinates
	* @param the matrix containting ground truth point coordinates
	*/
	inline void setGTPoints(const Eigen::MatrixXd& gtp_) { this->GTPoints = gtp_; }

	/**
	* @brief Get function
	* Get the ground truth point coordinates
	* @return the matrix containting ground truth point coordinates
	*/
	inline Eigen::MatrixXd getGTpoints() { return this->GTPoints; }

	/**
	* @brief Set function
	* Set the landmark coordinates in the Polaris frame
	* @param the matrix containting the landmark coordinates in the Polaris frame
	*/
	inline void setPolarisPoints(const Eigen::MatrixXd& Pp_) { this->polarisPoints = Pp_; }

	/**
	* @brief Get function
	* Get the landmark coordinates in the Polaris frame
	* @return the matrix containting the landmark coordinates in the Polaris frame
	*/
	inline Eigen::MatrixXd getPolarisPoints() { return this->polarisPoints; }

	/**
	* @brief Set function
	* Set the coordinates of the probe tip in the probe frame
	* @param the coordinates of the probe tip in the probe frame
	*/
	inline void setProbeTip(const Eigen::Vector3d& tip_) { this->probeTip = tip_; }

	/**
	* @brief Get function
	* Get the coordinates of the probe tip in the probe frame
	* @return the coordinates of the probe tip in the probe frame
	*/
	inline Eigen::Vector3d getProbeTip() { return this->probeTip; }

	/**
	* @brief Load the registered transformation from a file
	* @param filename: the name of the file containing the transformation of the sensor wrt the gt frame
	* @return true if the transformation has been loaded. False otherwise
	*/
	bool loadTransformation(const char* filename);

	/**
	* @brief Load the ground truth point coordinates from a file
	* @param filename: the name of the file containing the 3D point coordinates in the ground truth frame
	* @return true if the data have been loaded. False otherwise
	*/
	bool loadGTPoints(const char* filename);

	/**
	* @brief Load the Polaris point coordinates from a file
	* @param filename: the name of the file containing the 3D point coordinates in the Polaris frame
	* @return true if the data have been loaded. False otherwise
	*/
	bool loadPolarisPoints(const char* filename);

	/**
	* @brief Load the coordinates of the probe tip from a file
	* @param filename: the name of the file containing the coordinates of the probe tip
	* @return true if the data have been loaded. False otherwise
	*/
	bool loadProbeTip(const char* filename);

	/**
	* @brief Extract data function
	* Extract the expected data from the content of the registered transformation file to save data properly (set internally T)
	* @param content: the vector of strings with the content of the read file
	*/
	bool extractTransformData(const std::vector < std::string >& content);

	/**
	* @brief Extract data function
	* Extract the expected data from the content of the ground truth point file to save data properly (set internally GTpoints)
	* @param content: the vector of strings with the content of the read file
	*/
	bool extractGTPointData(const std::vector < std::string >& content);

	/**
	* @brief Extract data function
	* Extract the expected data from the content of the Polaris point file to save data properly (set internally PolarisPoints)
	* @param content: the vector of strings with the content of the read file
	*/
	bool extractPolarisPointData(const std::vector < std::string >& content);

	/**
	* @brief Extract data function
	* Extract the expected data from the content of the probe tip coordinates file to save data properly (set internally ProbeTip)
	* @param content: the vector of strings with the content of the read file
	*/
	bool extractProbeTipData(const std::vector < std::string >& content);

	/**
	* @brief Conversion function
	* Convert the vector of point coordinates provided in input to the corresponding Eigen matrix
	* @param the landmark point coordinates in vector structure
	*/
	void populatePolarisPoints(const std::vector < Eigen::Vector3d >& points);

	/**
	* Save function
	* Save the homogeneous transformation matrix on a file
	* params the name of the file on which the transformation has to be saved
	* params the name of the file on which the Polaris points coordinates have to be saved
	* @return true if the Transformation has been saved
	*/
	bool saveTransformation(const char* Tfilename, const char* Pfilename);

	inline void setChiStats(const Eigen::VectorXd& stats_) { this->stats = stats_; }

	/**
	* Save function
	* Save the calibration stats on a file
	* params the name of the file on which the calibration stats have to be saved
	*/
	void saveStats(const char* filename);

private:

	Eigen::Matrix4d T;				//!< Homogeneous transformation matrix expressing the pose of the Polaris frame wrt the CT scan frame
	Eigen::MatrixXd GTPoints;		//!< Set of 3D landmark point coordinates in the CT scan frame
	Eigen::MatrixXd polarisPoints;	//!< Set of 3D landmark point coordinates in the CT scan frame
	Eigen::Vector3d probeTip;		//!< Vector of 3D coordinates of the probe tip wrt the probe frame
	Eigen::VectorXd stats;			//!< Time evolution of the chi square error

};

#endif //REGISTRATION_DATA_HPP_