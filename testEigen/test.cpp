
#if 0
#include<iostream>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <chrono>

using namespace Eigen;
using namespace std;
using namespace std::chrono;

int main()
{
	Vector3d a;

	MatrixXd J = MatrixXd::Random(4000,6);
	MatrixXd Jt = J.transpose();
	MatrixXd A = Jt * J;
	MatrixXd b = MatrixXd::Random(6,1);

	// Cholesky 
	LDLT<MatrixXd> ldlt(A);
	bool pos = ldlt.isPositive();
	auto start = high_resolution_clock::now();
	MatrixXd x1 = ldlt.solve(b);
	if (ldlt.info() != Success)
	{
		cout << "Error in LDLT"<<endl;
	}
	auto end = high_resolution_clock::now();
	float time = duration_cast<milliseconds>(end - start).count();
	cout << "LDLT time cost " << time << "ms"<< endl;
	LLT<MatrixXd> llt(A);
	MatrixXd x11 = llt.solve(b);
	// QR
	HouseholderQR<MatrixXd> hqr(A);
	MatrixXd x2 = hqr.solve(b);

	ColPivHouseholderQR<MatrixXd> cqr(A);
	MatrixXd x21 = cqr.solve(b);

	FullPivHouseholderQR<MatrixXd> fqr(A);
	MatrixXd x22 = fqr.solve(b);

	// SVD
	JacobiSVD<MatrixXd> svd(A,ComputeThinU | ComputeThinV);
	MatrixXd x3 = svd.solve(b);

	// LU
	FullPivLU<MatrixXd> lu(A);
	MatrixXd x4 = lu.solve(b);
	
	/*cout << "J: " << endl << J << endl;

	cout << "Jt: " << endl<<  Jt << endl;

	cout << "A: " << endl << A << endl;*/

	cout << "Cholesky solution LDLT " << endl << x1 <<endl;

	cout << "Cholesky solution LLT" << endl << x11 <<endl;

	cout << "QR " << endl << x2 <<endl;

	cout << "QR2 " << endl << x21 <<endl;

	cout << "QR3 " << endl << x22 <<endl;

	cout << "SVD " << endl << x3 << endl;

	cout << "LU " << endl << x4 << endl;

	return 0;
}
#endif