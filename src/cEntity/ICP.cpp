// System header files
#include <vector>

// Project header files
#include "ICP.hpp"

/**
* @brief Default Constructor
*/
ICP::ICP(const int& iters) {

	this->iterations = iters;

	this->knownDataAssociations = false;
}

/**
* @brief Default Destroyer
*/
ICP::~ICP() {}

/**
* @brief Set function
* Fill the set of source points
* @param a vector containing the set of source points
*/
void ICP::populateSrcSet(const Eigen::MatrixXd& src) {

	// Extract the size of the input matrix
	int r = src.rows();
	int c = src.cols();

	this->srcSet.setZero(r, c);
	this->srcSet = src;

}

/**
* @brief Set function
* Fill the set of destination points
* @param a vector containing the set of destination points
*/
void ICP::populateDstSet(const Eigen::MatrixXd& dst) {

	// Extract the size of the input matrix
	int r = dst.rows();
	int c = dst.cols();

	this->dstSet.setZero(r, c);
	this->dstSet = dst;


}

#ifdef DEBUG
/**
* @brief Debug function
* Print the two sets of points on which the ICP is performed
*/
void ICP::showPointSets() {

	std::cout << "Source Set: \n" << this->srcSet << std::endl;
	std::cout << "Destination Set: \n" << this->dstSet << std::endl;

}

/**
* Save function
* Save the ICP stats on an output file
*/
void ICP::saveStats()
{
}
#endif // DEBUG


/**
* @brief Initial guess computation
* Compute the initial guess based on the known point correspondences
* @return the initial guess to use in the next ICP iterations
*/
void ICP::computeInitialGuess() {

	// Get the measurement size
	int M = this->srcSet.rows() * COORDINATE_SIZE; // Each point requires three points in the A matrix

	// Define the required matrices
	Eigen::MatrixXd A(M, POSE12D);
	Eigen::MatrixXd AT(POSE12D, M);
	Eigen::VectorXd b(M);
	Eigen::VectorXd x(POSE12D), xguess(POSE6D);
	Eigen::MatrixXd pinvA(POSE12D,M);

	// b Vector
	for (int i = 0; i < M; i++) {
		
		int r = i / COORDINATE_SIZE;
		int c = i % COORDINATE_SIZE;
		b(i) = this->dstSet(r, c);
	
	}

	// A matrix
	for (int i = 0; i < this->srcSet.rows(); i++) {
	
		// Get source point coordinates
		double xs = this->srcSet(i, 0);
		double ys = this->srcSet(i, 1);
		double zs = this->srcSet(i, 2);

		A.block<3, POSE12D>(i * COORDINATE_SIZE, 0) << xs, ys, zs,  0,  0,  0,  0,  0,  0, 1, 0, 0,
														0,  0,  0, xs, ys, zs,  0,  0,  0, 0, 1, 0,
														0,  0,  0,  0,  0,  0, xs, ys, zs, 0, 0, 1;
	
	}

	/*b(0) = x1P;
	b(1) = y1P;
	b(2) = z1P;
	b(3) = x2P;
	b(4) = y2P;
	b(5) = z2P;
	b(6) = x3P;
	b(7) = y3P;
	b(8) = z3P;
	b(9) = x4P;
	b(10) = y4P;
	b(11) = z4P;

	//A matrix
	
	A << x1T, y1T, z1T, 0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, x1T, y1T, z1T, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, x1T, y1T, z1T, 0, 0, 1,
		x2T, y2T, z2T, 0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, x2T, y2T, z2T, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, x2T, y2T, z2T, 0, 0, 1,
		x3T, y3T, z3T, 0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, x3T, y3T, z3T, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, x3T, y3T, z3T, 0, 0, 1,
		x4T, y4T, z4T, 0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, x4T, y4T, z4T, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, x4T, y4T, z4T, 0, 0, 1;//*/


	// A matrix Transpose
	AT = A.transpose();

	// Compute the pseudo-inverse of the A matrix
	pinvA = AT * (A*AT).inverse(); //pseudoinverse formula

	// Solve the linear system to compute the initial guess ( x = A \ b )
	x = pinvA * b;	//compute x inverting the formula
	//*/

	// Build the initial guess from the x vector
	xguess.topRows(COORDINATE_SIZE) = x.topRows(COORDINATE_SIZE); // position
	xguess.bottomRows(COORDINATE_SIZE) = this->rot2abg(x.bottomRows(POSE12D - COORDINATE_SIZE));

	// Set the initial guess member variable
	this->initialGuess.setZero(POSE6D);
	this->initialGuess = xguess;

/*#ifdef DEBUG
	std::cout << "dstSet: \n" << this->dstSet << std::endl;
	std::cout << "b vector: \n" << b << std::endl;
	std::cout << "srcSet: \n" << this->srcSet << std::endl;
	std::cout << "A matrix: \n" << A << std::endl;
	std::cout << "x vector: " << x.transpose() << std::endl;
	std::cout << "xguess vector: " << xguess.transpose() << std::endl;
#endif // DEBUG//*/

}


/**
* @brief Jacobian function
* Compute the error vector and the Jacobian matrix for ICP, based on the current state and
* measurements
* @param x: the current state vector
* @param p: the known point vector
* @param z: the current measurement
* @return a pair containing the current error vector and Jacobian matrix
*/
std::pair < Eigen::VectorXd, Eigen::MatrixXd> ICP::errorAndJacobian(const Eigen::VectorXd& x, const Eigen::VectorXd& p, const Eigen::VectorXd& z) {

	std::pair < Eigen::VectorXd, Eigen::MatrixXd > eJ;
	Eigen::VectorXd e, zhat;
	Eigen::MatrixXd J;
	Eigen::Matrix3d rx, ry, rz;
	Eigen::Matrix3d drx, dry, drz;
	Eigen::Vector3d t, v0, v1, v2;

	// Initialize dynamic structure
	e.setZero(COORDINATE_SIZE);
	zhat.setZero(COORDINATE_SIZE);
	J.setZero(COORDINATE_SIZE, POSE6D);

	// Compute the rotation matrices
	rx << 1,         0,          0,
		  0, cos(x(3)), -sin(x(3)),
		  0, sin(x(3)),  cos(x(3));

	ry <<  cos(x(4)), 0, sin(x(4)),
		           0, 1,         0,
		  -sin(x(4)), 0, cos(x(4));

	rz << cos(x(5)), -sin(x(5)), 0,
		  sin(x(5)),  cos(x(5)), 0,
		          0,          0, 1;

	// Compute the derivatives of the rotation matrices
	drx << 0,          0,          0,
		   0, -sin(x(3)), -cos(x(3)),
		   0,  cos(x(3)), -sin(x(3));

	dry << -sin(x(4)), 0,  cos(x(4)),
		            0, 0,          0,
		   -cos(x(4)), 0, -sin(x(4));

	drz << -sin(x(5)), -cos(x(5)), 0,
		    cos(x(5)), -sin(x(5)), 0,
		            0,          0, 0;

	// Initialize translation vector
	t = x.topRows(COORDINATE_SIZE);

	zhat = rx * ry*rz*p + t;	//prediction
	e = zhat - z;				//error

	//Vectors for ICP
	v0 = drx *  ry * rz  * p;
	v1 =  rx * dry * rz  * p;
	v2 =  rx *  ry * drz * p;

	J << 1, 0, 0, v0(0), v1(0), v2(0), //de/dax
		 0, 1, 0, v0(1), v1(1), v2(1), //de/day
		 0, 0, 1, v0(2), v1(2), v2(2); //de/daz

	//return the pair of error vector and Jacobian matrix
	eJ.first = e;
	eJ.second = J;

	return eJ;

}


/**
* @brief Solve function
* Compute the main algorithm of ICP
* @param ...
* @return ...
*/
void ICP::solve() {

	// Initial guess
	Eigen::VectorXd x = this->initialGuess;
	Eigen::VectorXd sol = x;

	// Use readable matrix for source and destination point set
	Eigen::MatrixXd P, Z;
	P = this->dstSet;
	Z = this->srcSet;

#ifdef DEBUG
	std::cout << "P matrix: \n " << P << std::endl;
	std::cout << "Z matrix: \n " << Z << std::endl;
#endif // DEBUG

	// Stat vector containing the mean square error at each iteration
	//Eigen::VectorXd chi_stats;

	//sol.setZero(POSE6D);
	this->chi_stats.setZero(this->iterations);
	//chi_stats = chi_stats.transpose();

#ifdef DEBUG
	std::cout << "iterations: " << this->iterations << std::endl;
#endif // DEBUG

	this->setDataAssociationKnown(true);
	if (!this->knownDataAssociations) {
		// Find correspondences
		Z = findCorrespondencesV2(sol);
		this->srcSet = Z;
	}

	//Code given here: https://gitlab.com/grisetti/probabilistic_robotics_2018_19/blob/master/slides/probabilistic_robotics_20_3D_point_registration.pdf
	double chi = 0;
	for (int j = 0; j < this->iterations; j++)
	{
		Eigen::MatrixXd H(POSE6D, POSE6D);
		Eigen::VectorXd b(POSE6D);
		H.setZero();
		b.setZero();
		chi = 0;

		Eigen::MatrixXd W(6, 6);
		W(0, 0) = 1.0;
		W(1, 1) = 1.0;
		W(2, 2) = 1.0;
		W(3, 3) = 1.0;
		W(4, 4) = 1.0;
		W(5, 5) = 1.0;

		for (int i = 0; i < P.rows(); i++)
		{
			// Compute the current error vector and the corresponding Jacobian matrix
			std::pair < Eigen::VectorXd , Eigen::MatrixXd > eJ = errorAndJacobian(sol, P.row(i), Z.row(i));

			Eigen::VectorXd e = eJ.first;
			Eigen::MatrixXd J = eJ.second;

			H += J.transpose()*W(i,i)*J;
			b += J.transpose()*W(i, i)*e;
			chi += e.transpose()*W(i, i)*e;
		}

		this->chi_stats(j) = chi;
		Eigen::VectorXd dx(POSE6D);
		dx = -(H.inverse()*b);
		sol += dx;

		// Set the obtained solution of the ICP algorithm
		this->solution.setZero(POSE6D);
		this->solution = sol;

#ifdef DEBUG
/*		if (j > 0)
		{
			//print the behaviour of the error through iterations
			std::cout << "MSE at iteration n° " << j << " is: " << chi << ", derivative: " << chi - this->chi_stats(j - 1) << std::endl;
			//if (abs(chi - chi_stats(iteration - 1)) < 0.0001) //if the error converges, break the loop
		}
		else
		{
			//first iteration
			std::cout << "MSE at iteration n° " << j << " is: " << chi << std::endl;
		}

		std::cout << "ICP solution: " << this->solution.transpose() << std::endl;//*/
#endif // DEBUG

	}

#ifdef DEBUG
	std::cout << "Final chi2 error: " << chi << std::endl;
#endif //DEBUG


	// Convert the found solution to homogeneous transformation 
	this->V2TsolConversion();

}


/**
* @brief Data association function (v2)
* Find the correspondences between the input clouds of points for the ICP algorithm
* @param current guess
* @return candidate correspondences
*/
Eigen::MatrixXd ICP::findCorrespondencesV2(const Eigen::VectorXd& guess) {

	Eigen::MatrixXd Zordered, P, Z;
	Eigen::MatrixXd gtDistances;
	Eigen::MatrixXd measDistances;
	Eigen::VectorXd minIndexs;
	Eigen::MatrixXd distErr;
	Eigen::VectorXd measDist_i;
	int n;

	// Get the GT points
	P = this->dstSet;
	n = P.rows();

	// Get the unordered measurement set
	Z = this->srcSet;
	Zordered.setZero(n, 3);
	gtDistances.setZero(n, n);
	measDistances.setZero(n, n);
	distErr.setZero(n, n);
	minIndexs.setConstant(n,-1);
	measDist_i.setZero(n);

	// Compute the ground truth distances
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			gtDistances(i, j) = (P.row(j) - P.row(i)).norm();
			measDistances(i, j) = (Z.row(j) - Z.row(i)).norm();
		}
	}

	// Compute the correspondences by evaluating the minimum distance errors
	/*for (int i = 0; i < n; i++) {
		measDist_i = measDistances.row(i);
		for (int j = 0; j < n; j++) {
			distErr(i,j) = std::fabs(measDist_i.norm() - gtDistances.row(j).norm());
			std::cout << "measDistances.row(i) = " << measDistances.row(i) << std::endl;
			std::cout << "gtDistances.row(j) = " << gtDistances.row(j) << std::endl;
			//distErr(i, j) = (measDistances.row(i) - gtDistances.row(j)).norm();
		}
		distErr.row(i).minCoeff(&minIndexs(i));
		Zordered.row(minIndexs(i)) = Z.row(i);
	}//*/

	int candidateIdx = -1;
	for (int i = 0; i < n; i++) {
		measDist_i = measDistances.row(i);
		for (int j = 0; j < n; j++) {
			distErr(i, j) = std::fabs(measDist_i.norm() - gtDistances.row(j).norm());
			std::cout << "measDistances.row(i) = " << measDistances.row(i) << std::endl;
			std::cout << "gtDistances.row(j) = " << gtDistances.row(j) << std::endl;
			//distErr(i, j) = (measDistances.row(i) - gtDistances.row(j)).norm();
		}


		bool isCandidateTaken = false;
		do {
			isCandidateTaken = false;
			distErr.row(i).minCoeff(&candidateIdx);
			std::cout << "distErr.row(i) = " << distErr.row(i) << std::endl;
			std::cout << "candidateIdx = " << candidateIdx << std::endl;
			std::cout << "minIndexs = " << minIndexs.transpose() << std::endl;
			std::cout << "i = " << i << std::endl;
			for (int k = 0; k < n; k++) {
				if (candidateIdx == minIndexs(k)) {
					isCandidateTaken = true;
					std::cout << "Setting isCandidateTaken to true" << std::endl;
				}
			}

			if (!isCandidateTaken) {
				minIndexs(i) = candidateIdx;
				std::cout << "Setting minIndexs(" << i << ") to " << candidateIdx << std::endl;
			}
			else {
				distErr.row(i)(candidateIdx) = 1000;
				//isCandidateTaken = false;
			}

			std::cout << "isCandidateTaken = " << isCandidateTaken << std::endl;
		} while (isCandidateTaken == true);
 
		if (minIndexs(i) >= 0) {
			std::cout << "Setting Zordered in minINdexs(i) (" << minIndexs(i) << std::endl;
			Zordered.row(minIndexs(i)) = Z.row(i);
		}
	}

#ifdef DEBUG
	std::cout << "gtDistances: \n " << gtDistances << std::endl;
	std::cout << "measDistances: \n " << measDistances << std::endl;
	std::cout << "dist errors: \n " << distErr << std::endl;// gtDistances - measDistances << std::endl;
	std::cout << "minIndexs: \n " << minIndexs.transpose() << std::endl;
	std::cout << "Zordered: \n" << Zordered << std::endl;//*/
#endif

	return Zordered;

}



/**
* @brief Data association function
* Find the correspondences between the input clouds of points for the ICP algorithm
* @param current guess
* @return candidate correspondences
*/
Eigen::MatrixXd ICP::findCorrespondences(const Eigen::VectorXd& guess) {

	Eigen::MatrixXd Zguess, Zcurr, Znormd, PTransf, PTnormd;
	Eigen::Vector3d Zavg, PTavg;
	Eigen::Matrix4d T, Tinv;
	Eigen::Matrix3d R;
	Eigen::Vector3d o;
	Eigen::Vector4d argmins_;
	Eigen::MatrixXd minCandidates;
	Eigen::MatrixXd distances;
	Zcurr = this->srcSet;
	Zguess = this->srcSet;
	int n = Zcurr.rows();

	minCandidates.setZero(n + 1,n);
	distances.setZero(n, n);

	// Convert the current guess estimation vector to the corresponding
	// 4x4 homogeneous transformation
	T = v2t(guess);
	R = T.topLeftCorner(3, 3);
	o = T.block<3, 1>(0, 3);

	// Compute the centroids of the sets
	Zavg.setZero();
	PTavg.setZero();
	PTransf.setZero(n, 3);
	for (int i = 0; i < Zguess.rows(); i++) {
		
		Eigen::VectorXd cur_ref = this->dstSet.row(i);
		Eigen::VectorXd transfPnt = o + R * cur_ref;

		Zavg += Zcurr.row(i) * 1.0/Zcurr.rows();
		PTavg += transfPnt * 1.0 / Zcurr.rows();
		PTransf.row(i) = transfPnt;
	}


	// Implement here data association
	for (int i = 0; i < Zguess.rows(); i++) {
	
		Eigen::VectorXd cur_meas_n = Zcurr.row(i).transpose() - Zavg;
		
		double min_rms_err_i = 1000;
		double argmin = -1;
		for (int j = 0; j < this->dstSet.rows(); j++) {
		
			Eigen::VectorXd transfPnt_n = PTransf.row(j).transpose() - PTavg;
			double rms_err_j = (transfPnt_n - cur_meas_n).squaredNorm();
			distances(i, j) = rms_err_j;
#ifdef DEBUG
			std::cout << "cur_meas: " << cur_meas_n.transpose() << std::endl;
			std::cout << "transfPnt: " << transfPnt_n.transpose() << std::endl;
			std::cout << "rms_err_j: " << rms_err_j << std::endl;
			std::cout << "min_rms_err_i: " << min_rms_err_i << std::endl;//*/
#endif //DEBUG

			//errMatrix(i, j) = rms_err_j;

			if (rms_err_j < min_rms_err_i) {
				argmin = j;
				min_rms_err_i = rms_err_j;
			}
		}

		argmins_(i) = argmin;
		minCandidates(i + 1, argmin) = min_rms_err_i;
		minCandidates(0, argmin)++;
		Zguess.row(argmin) = Zcurr.row(i);

		
	}

#ifdef DEBUG
	std::cout << "Zguess: \n" << Zguess << std::endl;
	std::cout << "argmins: " << argmins_.transpose() << std::endl << std::endl;
	std::cout << "minCandidates: \n" << minCandidates << std::endl << std::endl;
	std::cout << "distances: \n" << distances << std::endl << std::endl;
#endif //DEBUG

	for (int i = 0; i < Zguess.rows(); i++) {
		if (minCandidates(0, i) > 1) {
			
		}
	}


	return Zguess;


}




/**
* @brief Solve function
* Implement the ICP algorithm
*/
void ICP::run() {

	// Compute the initial guess
	this->computeInitialGuess();

#ifdef DEBUG
	//std::cout << "before solve() function ... " << std::endl;
#endif // DEBUG

	// Solve the iterative ICP algorithm
	this->solve();


#ifdef DEBUG
	//std::cout << "after solve() function ... " << std::endl;
#endif // DEBUG

}

/**
* @brief Convert function
* Convert the vectorized rotation matrix (12D vector) to the 3D vector of alpha, beta and gamma angles
* @param rot_: the vectorized rotation matrix (12D vector)
* @return abg: the 3D vector of alpha, beta and gamma angles
*/
Eigen::Vector3d ICP::rot2abg(const Eigen::VectorXd& rot_) {

	Eigen::Vector3d abg;
	Eigen::Matrix3d R;

	// Fill the rotation matrix
	R << rot_(0), rot_(1), rot_(2),
	  	 rot_(3), rot_(4), rot_(5),
		 rot_(6), rot_(7), rot_(8);

	// THIS IS THE CONVERSION FUNCTIONS USED BY THE STUDENTS. THEY SEEM TO BE THE FUNCTIONS CONVERTING
	// THE ROTATION MATRIX IN ROLL-PITCH-YAW ANGLES, BUT THEY DO NOT DIVIDE OVER COS(PITCH).
	// MOREOVER, THE REVERSE FUNCTION (ABG->ROTMAT) IS NOT BUILD BY COMPOSING THE ELEMENTARY MATRICES
	// IN THE ORDER RZ * RY * RZ (AS ROLL-PITCH-YAW ANGLES WOULD REQUIRE), BUT IN THE ORDER 
	// RX * RY * RZ.
	//abg(1) = atan2(rot_(2, 1), rot_(2, 2));
	//abg(2) = atan2(-rot_(2, 1), (sqrt(rot_(2, 1)*rot_(2, 1) + rot_(2, 2)*rot_(2, 2))));
	//abg(3) = atan2(rot_(1, 0), rot_(0, 0));

	// This should be the correct conversion for the order RX *RY *RZ
	abg(1) = atan2(R(0, 2), sqrt(R(1, 2) * R(1, 2) + R(2, 2) * R(2, 2)));
	abg(0) = atan2(-R(1, 2) / cos(abg(1)), R(2, 2) / cos(abg(1)));
	abg(2) = atan2(-R(0, 1) / cos(abg(1)), R(0, 0) / cos(abg(1)));

	return abg;

}

/**
* @brief Convert function
* Convert the vectorized solution (6D vector) to the coresponding 4x4 homogeneous matrix
*/
Eigen::Matrix4d ICP::v2t(const Eigen::VectorXd& x) {

	Eigen::Matrix4d T;
	Eigen::Matrix3d Rx, Ry, Rz, R;
	Eigen::Vector3d abg;

	// Initialize the output data
	R.setIdentity();
	T.setIdentity();

	// Build the rotation matrix
	Rx << 1, 0, 0,
		0, cos(x(3)), -sin(x(3)),
		0, sin(x(3)), cos(x(3));

	Ry << cos(x(4)), 0, sin(x(4)),
		0, 1, 0,
		-sin(x(4)), 0, cos(x(4));

	Rz << cos(x(5)), -sin(x(5)), 0,
		sin(x(5)), cos(x(5)), 0,
		0, 0, 1;
	R = Rx * Ry * Rz;

	// Set the output matrix
	T.topLeftCorner(COORDINATE_SIZE, COORDINATE_SIZE) = R;
	T.block<3, 1>(0, 3) = x.topRows(COORDINATE_SIZE);


	return T;


}




/**
* @brief Convert function
* Convert the vectorized solution (6D vector) to the coresponding 4x4 homogeneous matrix
* Use internally the member variable 'solution' and sets internally the member variable Tsol
*/
void ICP::V2TsolConversion()
{
	Eigen::Matrix3d Rx, Ry, Rz, R;
	Eigen::Matrix4d T;
	Eigen::Vector3d abg;

	// Save the solution vector in a local variable
	Eigen::VectorXd x(POSE6D);
	x = this->solution;

	T = v2t(x);

	// Initialize the output data
	/*R.setIdentity();
	T.setIdentity();

	// Build the rotation matrix
	Rx << 1,         0,          0,
		  0, cos(x(3)), -sin(x(3)),
		  0, sin(x(3)),  cos(x(3));

	Ry <<  cos(x(4)), 0, sin(x(4)),
		           0, 1,         0,
		  -sin(x(4)), 0, cos(x(4));

	Rz << cos(x(5)), -sin(x(5)), 0,
		  sin(x(5)),  cos(x(5)), 0,
		          0,          0, 1;
	R = Rx * Ry * Rz;

	// Set the output matrix
	T.topLeftCorner(COORDINATE_SIZE, COORDINATE_SIZE) = R;
	T.block<3, 1>(0, 3) = x.topRows(COORDINATE_SIZE);//*/

	this->Tctp = T.inverse();

}

