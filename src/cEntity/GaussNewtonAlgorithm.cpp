// Project Header files
#include "GaussNewtonAlgorithm.hpp"
#include "DBWrapper.hpp"

// Windows Header files
#include <Windows.h>


// Standard Header files
#include <iostream>


/**
* @brief Default constructor of the GaussNewtonAlgorithm class
*/
GaussNewtonAlgorithm::GaussNewtonAlgorithm() {

	std::string folderName("GN-Results");
	this->statsFolder = folderName;
	char buf[256];

	// Get the current program directory
	GetCurrentDirectoryA(256, buf);

	// Iterate to create a folder with a not pre-existing name (e.g., from previous logs)
	CreateDirectory(std::string(std::string(buf) + "\\" + folderName).c_str(), NULL);


}

/**
* @brief Run function
* Run a single iteration of the Gauss-Newton algorithm
* TODO
* IT'S STILL NOT CLEAR HOW TO IMPLEMENT ONLINE VERSION
*/
void GaussNewtonAlgorithm::runOneRound(const Eigen::VectorXd& phi_k, const Eigen::VectorXd& z) {}


/**
* @brief Run function
* Run the batch version of the algorithm, accepting as input arguments the function pointers to the measurement model and the related Jacobian
* @param zhat: the function pointer accepting joint position and velocities and the state as inputs, and returning the vector of the predicted measurement
* @param jacobian: the function pointer accepting joint position and velocities and the state as inputs, and returning the matrix of the jacobian measurement
*/
Eigen::VectorXd GaussNewtonAlgorithm::runBatchV2(zhatCallback zhat, jacobianCallback jacobian){
//Eigen::VectorXd GaussNewtonAlgorithm::runBatchV2(Eigen::VectorXd(KUKARobot::*zhatCallback)(const void* q, const void* qdot, const void* xhat),
//												 Eigen::MatrixXf(KUKARobot::*jacobianCallback)(const void* q, const void* qdot, const void* xhat)){

	Eigen::MatrixXd H, H_k;
	Eigen::VectorXd b, b_k;
	Eigen::VectorXd x_kp, x_k;
	Eigen::VectorXd q_k, qdot_k;
	Eigen::VectorXd z_k, zhat_k, e_k;
	Eigen::MatrixXd J_k, J_kT;
	float sigma, Omega, lambda;

	// Set the dataset structure
	int N        = this->input.size();
	int inputNum = 2; // TODO: generalize this
	int Nin      = this->input[0].first.size();
	int S        = this->stateSize;
	int M        = this->measSize;

	// Initialize dimensions
	this->Zhat.resize(N);
	for (int i = 0; i < N; i++) {
		this->Zhat[i].setZero(M);
	}

	this->X.resize(this->iterationsNum);
	for (int i = 0; i < this->iterationsNum; i++) {
		this->X[i].setZero(S);
	}

	H.setZero(S, S);
	H_k.setZero(S, S);
	b.setZero(S);
	b_k.setZero(S);
	x_kp.setZero(S);
	x_k.setZero(S);
	q_k.setZero(Nin);
	qdot_k.setZero(Nin);
	z_k.setZero(M);
	zhat_k.setZero(M);
	e_k.setZero(M);
	J_k.setZero(M, S);

	// Measurement variance
	Eigen::VectorXd sigmas(M);
	Eigen::MatrixXd Omegas(M, M);
	//sigmas << 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7;
	Omegas = sigmas.array().cwiseInverse().matrix().asDiagonal();

	std::cout << "Omega: \n" << Omegas << std::endl;

	sigma = 1e-4;
	Omega = 1.0 / sigma;

	// Damping factor
	//lambda = 1e-5;
	lambda = 1e-6;

	for (int i = 1; i < this->iterationsNum; i++) {

		// Reset H and b structures
		H.setZero(S, S);
		b.setZero(S);

		// Get the latest estimation vector
		x_kp = this->X[i - 1];

		// Iterate over the samples of the dataset
		for (int k = 0; k < N; k++) {

			// Get the k-th sample of the input vector
			// The generic k-th input is stored as [q_k , qdot_k], each as a column
			q_k = this->input[k].first;
			qdot_k = this->input[k].second;

			// Get the k-th sample of the z measurement vector
			z_k = this->Z[k];

			// Call the callback for the computation of zhat_k
			zhat_k = zhat(q_k, qdot_k, x_kp);
			zhat_k *= -1.0;
			this->Zhat[k] = zhat_k;

			// Compute the k-th error
			e_k = zhat_k - z_k;
			
			// Call the callback for the computation of J_k
			J_k = jacobian(q_k, qdot_k, x_kp);
			J_kT = J_k.transpose();
			
			// Compute the k-th Hessian matrix contribution
			H_k = J_kT * Omega * J_k;

			// Compute the k-th b term contribution
			b_k = J_kT * Omega * e_k;

			// Update H
			H += H_k;

			// Update b
			b += b_k;//*/

		}

		// Solve the resulting linear system
		H += lambda * Eigen::MatrixXd::Identity(S, S);
		Eigen::VectorXd delta_x = H.inverse() * b;

		// Update the state
		x_k = x_kp + delta_x;//*/

		this->X[i] = x_k;

	}


	// Return the latest value of the estimated coefficients
	return this->X.back();

}

/**
* @brief Run function
* Run the batch version of the Gauss-Newton algorithm
* @return the estimated friction parameters
*/
Eigen::VectorXd GaussNewtonAlgorithm::runBatch() {

	Eigen::MatrixXd H, H_k;
	Eigen::VectorXd b, b_k;
	Eigen::VectorXd x_kp;
	Eigen::VectorXd x_k;
	Eigen::VectorXd qdot_k;
	Eigen::VectorXd z_k;
	Eigen::VectorXd zhat_k;
	Eigen::VectorXd e_k;
	Eigen::VectorXd J_k, J_kT;
	Eigen::VectorXd expDen_k;
	float sigma, Omega, lambda;
	float Fs, Fv, ki, Fqd;

	// Set the dataset structure
	int N = this->input.size();
	int S = this->stateSize;

	// Initialize dimensions
	this->Zhat.resize(N);
	for (int i = 0; i < N; i++) {
		this->Zhat[i].setZero(this->measSize);
	}

	this->X.resize(this->iterationsNum);
	for (int i = 0; i < this->iterationsNum; i++) {
		this->X[i].setZero(S);
	}

	// Copy input data
	//this->Phi = Phi;
	//this->Z = Z;

	// Run the algorithm
	H.setZero(S, S);
	H_k.setZero(S, S);
	b.setZero(S);
	b_k.setZero(S);
	x_kp.setZero(S);
	x_k.setZero(S);
	z_k.setZero(this->measSize);
	zhat_k.setZero(this->measSize);
	e_k.setZero(this->measSize);
	qdot_k.setZero(this->measSize);
	J_k.setZero(S);
	J_kT.setZero(S);

	// Measurement variance
	sigma = 1e-5;
	Omega = 1.0 / sigma;

	// Damping factor
	lambda = 1e-3;

	for (int i = 1; i < this->iterationsNum; i++) {

		// Reset H and b structures
		H.setZero(S, S);
		b.setZero(S);
	
		// Get the latest estimation vector
		x_kp = this->X[i - 1];
		Fs  = x_kp(0);
		Fv  = x_kp(1);
		ki  = x_kp(2);
		if (S == 4) {
			Fqd = x_kp(3);
		}
		// Iterate over the samples of the dataset
		for (int k = 0; k < N; k++) {
			
			// Get the k-th sample of the input vector
			qdot_k = this->input[k].second;

			expDen_k = (1.0 + (-ki * qdot_k.array()).exp());

			// Get the k-th sample of the z measurement vector
			z_k = this->Z[k];

			// Compute the k-th predicted measurement
			if (S == 3) {
				zhat_k = Fs + Fv / expDen_k.array();
			}
			else if(S == 4){
				zhat_k = Fs + Fv / expDen_k.array() + Fqd * qdot_k.array();
			}
			zhat_k *= -1.0;
			this->Zhat[k] = zhat_k;

			// Compute the k-th error
			e_k = zhat_k - z_k;

			// Compute the k-th value of the Jacobian (assuming the measurement model as phi^T * x)
			if (S == 3) {
				J_k << 1.0, 1.0/expDen_k.array() , Fv * qdot_k.array() * (-ki*qdot_k.array()).exp() / (expDen_k.array()).pow(2.0);
			}
			else if (S == 4){
				J_k << 1.0, 1.0 / expDen_k.array(), Fv * qdot_k.array() * (-ki*qdot_k.array()).exp() / (expDen_k.array()).pow(2.0), qdot_k.array();			
			}

			// J_kT is used to call .transpose() method on J_k
			J_kT = J_k;

			// Compute the k-th Hessian matrix contribution
			H_k = J_k * Omega * J_kT.transpose();

			// Compute the k-th b term contribution
			b_k = J_k * Omega * e_k.transpose();

			// Update H
			H += H_k;

			// Update b
			b += b_k;

		}

		// Solve the resulting linear system
		H += lambda * Eigen::MatrixXd::Identity(S, S);
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);

		// Update the state
		x_k = x_kp + svd.solve(b);

		this->X[i] = x_k;
	}

	// Return the latest value of the estimated coefficients
	return this->X.back();


}//*/

/**
* @brief Create folder function
* Create the folder in which the stats data have to be stored
* @param folderName: the name of the folder where data have to be stored
* @return the name of the folder (? mayb useless)
*/
std::string GaussNewtonAlgorithm::createStatsFolder(const char* folderName) {


	if (!CreateDirectory(folderName, NULL)) {
		return std::string("error");
		std::cout << "error in creating the folder " << std::endl;
	}
	else {
		std::cout << "Created folder at path = " << folderName << std::endl;
	}

	// Assign the path to the class memeber variable
	this->statsFolder = folderName;
	std::cout << "this->statsFolder = " << this->statsFolder << std::endl;

	return this->statsFolder;

}



/**
* @brief Save function
* Save the statistics of the algorithm on the given file
* @param xFilename: the name of the file with the state x
* @param zhatFilename: the name of the file with the predicted measurement zhat
* @param zFilename: the name of the file with the measurement x
*/
void GaussNewtonAlgorithm::saveStats(const char* xFileName, const char* zhatFilename, const char* zFilename) {

	DBWrapper db;
	std::string fullpath;
	std::stringstream xSS, zhatSS, zSS;

	/// X dataset
	for (int i = 0; i < this->X.size(); i++) {
		for (int j = 0; j < this->stateSize; j++) {
			Eigen::VectorXd Xi = this->X[i];
			xSS << Xi(j) << (j == this->stateSize - 1 ? ";\n" : ",");
		}
	}

	fullpath = std::string(xFileName);
	std::cout << "X dataset fullpath : " << fullpath << std::endl;
	db.setDBFile(fullpath.c_str());
	db.write(xSS);

	/// Zhat dataset
	for (int i = 0; i < this->Zhat.size(); i++) {
		for (int j = 0; j < this->measSize; j++) {
			Eigen::VectorXd Zhati = this->Zhat[i];
			zhatSS << Zhati(j) << (j == this->measSize - 1 ? ";\n" : ",");
		}
	}

	fullpath = std::string(zhatFilename);
	std::cout << "Zhat dataset fullpath : " << fullpath << std::endl;
	db.setDBFile(fullpath.c_str());
	db.write(zhatSS);

	/// Z dataset
	for (int i = 0; i < this->Z.size(); i++) {
		for (int j = 0; j < this->measSize; j++) {
			Eigen::VectorXd Zi = this->Z[i];
			zSS << Zi(j) << (j == this->measSize - 1 ? ";\n" : ",");
		}
	}

	fullpath = std::string(zFilename);
	std::cout << "Z dataset fullpath : " << fullpath << std::endl;	db.setDBFile(fullpath.c_str());
	db.write(zSS);
}

/**
* @brief Load function
* Load the datasets from previous sessions to run the algorithm offline
* @param logNum: the number identifying the previous log session
* @param [out] q: the dataset of joint position
* @param [out] qd: the dataset of joint velocity
* @param [out] r: the dataset of residuals
*/
void GaussNewtonAlgorithm::loadData(const int& logNum, std::vector<Eigen::VectorXd>& q, std::vector<Eigen::VectorXd>& qd, std::vector<Eigen::VectorXd>& r) {

	std::string logPath, filePath;
	std::vector < std::string > qcontent, qdcontent, rcontent;
	std::string qFile("qmsr.txt");
	std::string qdFile("qdotcmd.txt");
	std::string rFile("modelResidual.txt");
	DBWrapper db;
	char buf[256];

	// Get the current program directory
	GetCurrentDirectoryA(256, buf);
	logPath = std::string(buf) + "\\LogSession_" + std::string(std::to_string(logNum)) + "\\KUKARobot\\";
	filePath = logPath + qFile;

	// Joint positionss
	db.setDBFile(filePath.c_str());
	qcontent = db.readCSVFile();
	std::cout << "qcontent.size() = " << qcontent.size() << std::endl;
	for (int i = 0; i < qcontent.size(); i++) {
		std::vector < double > vec = parseCSVLine(qcontent[i]);
		Eigen::VectorXd q_j(vec.size());
		for (int j = 0; j < vec.size(); j++) {
			q_j(j) = vec[j];
		}
		q.push_back(q_j);
	}

	// Joint velocities
	filePath = logPath + qdFile;
	db.setDBFile(filePath.c_str());
	qdcontent = db.readCSVFile();
	for (int i = 0; i < qdcontent.size(); i++) {
		std::vector < double > vec = parseCSVLine(qdcontent[i]);
		Eigen::VectorXd qd_j(vec.size());
		for (int j = 0; j < vec.size(); j++) {
			qd_j(j) = vec[j];
		}
		qd.push_back(qd_j);
	}

	// Joint positionss
	filePath = logPath + rFile;
	db.setDBFile(filePath.c_str());
	rcontent = db.readCSVFile();
	for (int i = 0; i < rcontent.size(); i++) {
		std::vector < double > vec = parseCSVLine(rcontent[i]);
		Eigen::VectorXd r_j(vec.size());
		for (int j = 0; j < vec.size(); j++) {
			r_j(j) = vec[j];
		}
		r.push_back(r_j);
	}
}

/**
* @brief Set function
* Add the k-th entry of the input array
* @param the k-th entry to be added
*/
void GaussNewtonAlgorithm::addInputEntry(const Eigen::VectorXd& q_k, const Eigen::VectorXd& qd_k) {

	std::pair < Eigen::VectorXd, Eigen::VectorXd > pp;
	pp.first = q_k;
	pp.second = qd_k;
	this->input.push_back(pp);

}
