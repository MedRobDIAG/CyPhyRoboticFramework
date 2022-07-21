// Project Header files
#include "utils.hpp"

// Standard Header files
#include <cctype>
#include <algorithm>



/**
* @brief Utility function
* parse the input string line into an array of doubled-precision floating values
* @param the input string line
* @return the corresponding vector of double-precision floating values
*/
std::vector < double > parseCSVLine(const std::string& line) {

	std::vector < double > vec;
	std::string comma(",");
	std::string semicolumn(";");

	std::string temp_line = line;
	bool endloop = false;
	while (!endloop && temp_line.length() > 0) {

		// Find the comma
		int delimiter = temp_line.find(comma);

		// If not found, look for the semicolumn
		if (delimiter == std::string::npos) {
			delimiter = temp_line.find(semicolumn);
			endloop = true;
		}

		// Assign the j-th value
		vec.push_back(std::stod(temp_line.substr(0, delimiter)));

		// Update the line
		temp_line = temp_line.substr(delimiter + 1, temp_line.length());
	}
	return vec;
}

/**
* @brief Utility function
* parse the input string line into an array of string values
* @param the input string line
* @return the corresponding vector of string values
*/
std::vector < std::string > parseStringLine(const std::string& line) {

	std::vector < std::string > vec;
	std::string comma(",");
	std::string semicolumn(";");

	std::string temp_line = line;
	bool endloop = false;
	while (!endloop) {

		// Find the comma
		int delimiter = temp_line.find(comma);

		// If not found, look for the semicolumn
		if (delimiter == std::string::npos) {
			delimiter = temp_line.find(semicolumn);
			endloop = true;
		}

		// Assign the j-th value
		std::string val = std::string(temp_line.substr(0, delimiter));
		val.erase(std::remove_if(val.begin(), val.end(), std::isspace), val.end());
		vec.push_back(val);

		// Update the line
		temp_line = temp_line.substr(delimiter + 1, temp_line.length());

	}

	return vec;
}

/**
* @brief Low-pass filter function
* Apply a low-pass filter to the input signal with given cycle time and frequency cut
* @param actual: the input signal current sample
* @param previous: the input signal previous sample
* @param CycleTime: the cycle time
* @param fcut: the frequency cut
* @return the filtered signal
*/
double low_pass_filter(double in, double out_old, double Tc, double tau) {

	double out, alpha;

	alpha = 1 / (1 + (tau / Tc));

	out = alpha * in + (1 - alpha) * out_old;

	return out;

}

/**
* @brief Skew-symmetric matrix function
* Compute the skew-symmetric matrix of the input vector
* @param v: the input vector
* @param the skew-symmetric matrix
*/
Eigen::Matrix3f skew(const Eigen::Vector3f& v) {

	Eigen::Matrix3f S;

	S <<    0, -v(2),  v(1),
		 v(2),     0, -v(0),
		-v(1),  v(0),     0;

	return S;
}


/**
* @brief Index search function
* Search, within the input vector of timestamps, the index whose time value is closest to the input tcurr
* @param timestamps: the vector of timestamps
* @param the current timestamp to evaluate
* @param (in/out): the index in which the timestamp is found
*/
void findClosestIndexFromTimeStamp(const std::vector < double >& timestamps, double& tcurr, int& idx) {

	int new_idx = 0;
	for (int i = idx; i < timestamps.size(); i++) {
		
		if (tcurr > timestamps[i - 1] && tcurr < timestamps[i]) {
			idx = i;
		}
	
	}

}

/**
* @brief Conversion function
* Convert the input Rotation matrix to the roll-pitch-yaw angles triple
* @param R: the input rotation matrix
* @param rpy_prev: the previous 3D vector of roll-pitch-yaw,  to evaluate the closest solution in the generated pair
* @return the output roll-pitch-yaw angle triple vector
*/
//Eigen::Vector3f rotmat2rpy(const Eigen::Matrix3f& R, const Eigen::Vector3f& rpy_prev) {
Eigen::Vector3f rotmat2rpy(const Eigen::Matrix3f & R) {

	Eigen::Vector3f rpy_p, rpy_m, rpy;
	float pitch_p = atan2(-R(2, 0), sqrt(R(2, 1)*R(2, 1) + R(2, 2)*R(2, 2)));
	float pitch_m = atan2(-R(2, 0), -sqrt(R(2, 1)*R(2, 1) + R(2, 2)*R(2, 2)));

	float roll_p = atan2(R(2, 1) / cos(pitch_p), R(2, 2) / cos(pitch_p));
	float roll_m = atan2(R(2, 1) / cos(pitch_m), R(2, 2) / cos(pitch_m));

	float yaw_p = atan2(R(1, 0) / cos(pitch_p), R(0, 0) / cos(pitch_p));
	float yaw_m = atan2(R(1, 0) / cos(pitch_m), R(0, 0) / cos(pitch_m));

	rpy_p << roll_p, pitch_p, yaw_p;
	rpy_m << roll_m, pitch_m, yaw_m;//*/

	//rpy = ((rpy_p - rpy_prev).norm() < (rpy_m - rpy_prev).norm()) ? (rpy_p) : (rpy_m);

	return rpy_p;

}