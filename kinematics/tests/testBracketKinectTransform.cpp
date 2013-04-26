/**
 * @file testBracketKinectTransform.cpp
 * @author Can Erdogan
 * @date Apr 26, 2013
 * @brief Tests the transformation computation between the shoulder bracket and Kinect. We want to
 % compute P^K = T^K_B * P^B given a P^B.
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

/* ******************************************************************************************** */
/// The main test
TEST(Main, main) {
	
	/// Set the constants
	const double r = 2, s = 3, a = 6, b = 4, t = 0.5, th = 150.0 / 180.0 * M_PI;

	// Get the transformation T^b_h, representation of hinge frame in bracket frame
	MatrixXd Tbh (4,4);
	Tbh << cos(th), -sin(th), 0, a,
         sin(th), cos(th), 0, b,
               0,       0, 1, 0,
               0,       0, 0, 1;

	// Get the transformation T^h_k, representation of kinect in the hinge frame
	MatrixXd Thk (4,4);
	Thk << 0, 0, 1, r,
         0, 1, 0, -s,
        -1, 0, 0, -t,
         0, 0, 0, 1;
	cout << "Thk: " << Thk << endl;

	// Compute the inverses
	MatrixXd Thb = Tbh.inverse();
	MatrixXd Tkh = Thk.inverse();

	// Try different P^b values to make sure it makes sense
	Vector4d Pb1 (0.0, 0.0, 0.0, 1.0);
	Vector4d Ph1 = Thb * Pb1;
//	cout << "Thb: " << Thb << endl;
//	cout << "Ph1: " << Ph1.transpose() << endl;
	Vector4d Pk1 = Tkh * Ph1;
//	cout << "Tkh: " << Tkh << endl;
//	cout << "Pk1: " << Pk1.transpose() << endl;

	// Try different P^b values to make sure it makes sense
	Vector4d Pb2 (-3.0, 0.0, 0.0, 1.0);
	Vector4d Ph2 = Thb * Pb2;
	cout << "Thb: " << Thb << endl;
	cout << "Ph2: " << Ph2.transpose() << endl;
	Vector4d Pk2 = Tkh * Ph2;
	cout << "Tkh: " << Tkh << endl;
	cout << "Pk2: " << Pk2.transpose() << endl;


}

/* ******************************************************************************************** */
int main(int argc, char* argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();	
}
/* ******************************************************************************************** */
