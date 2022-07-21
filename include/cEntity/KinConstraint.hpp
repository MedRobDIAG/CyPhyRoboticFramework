#ifndef KINCONSTRAINT_HPP_
#define KINCONSTRAINT_HPP_

// Eigen Header files
#include <Eigen/Dense>

// Twist dimension
#ifndef TWIST_DIM
#define TWIST_DIM 6
#endif // TWIST_DIM

namespace Eigen {

	typedef Eigen::Matrix<float, TWIST_DIM, TWIST_DIM> Matrix6f;
	typedef Eigen::Matrix<double, TWIST_DIM, TWIST_DIM> Matrix6d;

	typedef Eigen::Matrix<float, TWIST_DIM, 1> Vector6f;
	typedef Eigen::Matrix<double, TWIST_DIM, 1> Vector6d;

}

class KinConstraint {


public:
	/**
	* @brief Default constructor of the KinConstraint class
	*/
	KinConstraint() { this->clearConstraint(); }

	/**
	* @brief Default destroyer of the KinConstraint class
	*/
	~KinConstraint() {}

	/**
	* @brief Set function
	* Set the 6x6 matrix constraint for the current object
	* @param constrMat: the constraint matrix
	*/
	inline void set(const Eigen::Matrix6f& constrMat) { this->M = constrMat; }

	/**
	* @brief Get function
	* Get the 6x6 matrix constraint for the current object
	* @return the constraint matrix
	*/
	inline Eigen::Matrix6f getConstraint() { return this->M;  }

	/**
	* @brief Clear function
	* Clear the currently set constraint by assigning M to the identity matrix (i.e. no constraints applied)
	*/
	inline void clearConstraint() { this->M.setIdentity(); }

	/**
	* @brief Constraint function
	* Apply the currently set constraint on the input 6D (velocity) vector, returning the corresponding constrained vector
	* @param in: the 6D input (velocity) vector
	* @return the corresponding 6D constrained vector
	*/
	inline Eigen::Vector6f applyConstraint(const Eigen::Vector6f& in) { return (this->M * in); }

private:

	Eigen::Matrix6f M;		//!< 6x6 matrix mapping the un-constrained 6D input velocity to the velocity subject to the current constraint


};


#endif // KINCONSTRAINT_HPP_
