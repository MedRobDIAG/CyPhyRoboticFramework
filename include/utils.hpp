#ifndef UTILS_HPP_
#define UTILS_HPP_

// System Header files
#ifdef DEBUG
#include <iostream>
#endif // DEBUG
#include <vector>
#include <string>

// Eigen Header files
#include <Eigen\Dense>

#define SPACE_DIM 3

// Twist dimension
#ifndef TWIST_DIM
#define TWIST_DIM 6
#endif // TWIST_DIM

#define _USE_MATH_DEFINES
#include <math.h>

namespace Eigen {

	typedef Eigen::Matrix<float, 6, 1> Vector6f;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;
	typedef Eigen::Matrix<float, 6, 6> Matrix6f;
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<float, 7, 1> Vector7f;
	typedef Eigen::Matrix<double, 7, 1> Vector7d;
	typedef Eigen::Matrix<int, 6, 6> Matrix6i;
	typedef Eigen::Matrix<int, 6, 1> Vector6i;

}

/**
* @brief Print function
* Print if DEBUG preprocessor condition active
*/
template <typename T>
void debugPrint(const char* text, const T& data) {

#ifdef DEBUG
	std::cout << text << "\n" << data;
#endif // DEBUG

}


/**
* @brief Utility function
* parse the input string line into an array of doubled-precision floating values
* @param the input string line
* @return the corresponding vector of double-precision floating values
*/
std::vector < double > parseCSVLine(const std::string& line);

/**
* @brief Utility function
* parse the input string line into an array of string values
* @param the input string line
* @return the corresponding vector of string values
*/
std::vector < std::string > parseStringLine(const std::string& line);


/**
* @ Template radiant 2 degrees conversion function
* Convert input degree data in radiant
* @param in: input degree data
* @return out: output radiant data
*/
template<class T>
T rad2deg(const T& in) {

	T out;

	out = in * 180.0 / M_PI;

	return out;

}

/**
* @ Template degree 2 radiant conversion function
* Convert input radiant data in radiant
* @param in: input radiant data
* @return out: output degree data
*/
template<class T>
T deg2rad(const T& in) {

	T out;

	out = in * M_PI / 180.0 ;

	return out;

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
double low_pass_filter(double in, double out_old, double Tc, double tau);


/**
* @brief Skew-symmetric matrix function
* Compute the skew-symmetric matrix of the input vector
* @param v: the input vector
* @param the skew-symmetric matrix
*/
Eigen::Matrix3f skew(const Eigen::Vector3f& v);

/**
* @brief Index search function
* Search, within the input vector of timestamps, the index whose time value is closest to the input tcurr
* @param timestamps: the vector of timestamps
* @param the current timestamp to evaluate
* @param (in/out): the index in which the timestamp is found
*/
void findClosestIndexFromTimeStamp(const std::vector < double >& timestamps, double& tcurr, int& idx);

/**
* @brief Conversion function
* Convert the input Rotation matrix to the roll-pitch-yaw angles triple
* @param R: the input rotation matrix
* @param rpy_prev: the previous 3D vector of roll-pitch-yaw,  to evaluate the closest solution in the generated pair
* @return the output roll-pitch-yaw angle triple vector
*/
//Eigen::Vector3f rotmat2rpy(const Eigen::Matrix3f& R, const Eigen::Vector3f& rpy_prev);
Eigen::Vector3f rotmat2rpy(const Eigen::Matrix3f& R);


#endif// UTILS_HPP_

